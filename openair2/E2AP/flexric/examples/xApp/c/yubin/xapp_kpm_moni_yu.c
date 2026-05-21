/*
 * KPM Monitoring-only xApp with Redis SDL
 * - KPM collection period: 1 second
 * - Subscribe KPM by S-NSSAI
 * - Calculate current 1s KPM from cumulative KPM counters
 * - Accumulate 1s KPM values for 10 seconds
 * - Store 10s accumulated KPM to Redis
 * - Remove hardcoded UE IMSI/db description
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/util/time_now_us.h"
#include "../../../../src/util/alg_ds/ds/lock_guard/lock_guard.h"
#include "../../../../src/util/e.h"

#include <hiredis/hiredis.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <assert.h>

static uint64_t const period_ms = 1000;
static pthread_mutex_t mtx;

#define REDIS_HOST "127.0.0.1"
#define REDIS_PORT 6379
#define SDL_STORE_PERIOD_US 10000000

static redisContext* g_redis = NULL;
static int64_t g_last_sdl_store_us = 0;

/* --------------------------------------------------------------- */
/* Slice profile                                                   */
/* --------------------------------------------------------------- */

typedef struct {
  char const* profile_name;
  uint8_t sst;
  uint8_t sd[3];
} slice_profile_t;

static slice_profile_t const g_profile_sst1 = {
  .profile_name = "slice-sst1",
  .sst = 1,
  .sd = {0x00, 0x00, 0x03},
};

static slice_profile_t const g_profile_sst2 = {
  .profile_name = "slice-sst2",
  .sst = 2,
  .sd = {0x00, 0x00, 0x01},
};

static slice_profile_t const g_profile_sst3 = {
  .profile_name = "slice-sst3",
  .sst = 3,
  .sd = {0x00, 0x00, 0x02},
};

/* --------------------------------------------------------------- */
/* Observed UE table                                               */
/* --------------------------------------------------------------- */

#define MAX_OBSERVED_UE 64

typedef struct {
  bool valid;
  ue_id_e2sm_t ue_id;
  uint64_t key;
  slice_profile_t profile;

  bool has_prb_dl;
  bool has_prb_ul;
  bool has_pdcp_dl;
  bool has_pdcp_ul;

  int64_t prev_prb_dl;
  int64_t prev_prb_ul;
  int64_t prev_pdcp_dl;
  int64_t prev_pdcp_ul;

  int64_t cur_prb_dl;
  int64_t cur_prb_ul;
  int64_t cur_pdcp_dl;
  int64_t cur_pdcp_ul;

  double cur_rlc_delay_dl;
  double cur_ue_thp_dl;
  double cur_ue_thp_ul;

  int64_t acc_prb_dl;
  int64_t acc_prb_ul;
  int64_t acc_pdcp_dl;
  int64_t acc_pdcp_ul;

  double acc_rlc_delay_dl;
  double acc_ue_thp_dl;
  double acc_ue_thp_ul;
  uint64_t acc_real_cnt;
  uint64_t acc_sample_cnt;
} observed_ue_t;

static observed_ue_t g_observed_ues[MAX_OBSERVED_UE] = {0};

/* --------------------------------------------------------------- */
/* UE key helpers                                                  */
/* --------------------------------------------------------------- */

static uint64_t make_ue_key(ue_id_e2sm_t const* ue)
{
  assert(ue != NULL);

  switch (ue->type) {
    case GNB_UE_ID_E2SM:
      if (ue->gnb.ran_ue_id != NULL)
        return *ue->gnb.ran_ue_id;
      return ue->gnb.amf_ue_ngap_id;

    case GNB_DU_UE_ID_E2SM:
      if (ue->gnb_du.ran_ue_id != NULL)
        return *ue->gnb_du.ran_ue_id;
      return ue->gnb_du.gnb_cu_ue_f1ap;

    case GNB_CU_UP_UE_ID_E2SM:
      if (ue->gnb_cu_up.ran_ue_id != NULL)
        return *ue->gnb_cu_up.ran_ue_id;
      return ue->gnb_cu_up.gnb_cu_cp_ue_e1ap;

    default:
      return 0;
  }
}

static observed_ue_t* find_or_add_observed_ue(ue_id_e2sm_t const* ue,
                                              slice_profile_t const* profile)
{
  assert(ue != NULL);
  assert(profile != NULL);

  uint64_t key = make_ue_key(ue);
  assert(key != 0);

  for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
    if (g_observed_ues[i].valid &&
        g_observed_ues[i].key == key &&
        g_observed_ues[i].profile.sst == profile->sst &&
        memcmp(g_observed_ues[i].profile.sd, profile->sd, 3) == 0) {
      return &g_observed_ues[i];
    }
  }

  for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
    if (!g_observed_ues[i].valid) {
      g_observed_ues[i].valid = true;
      g_observed_ues[i].key = key;
      g_observed_ues[i].profile = *profile;
      g_observed_ues[i].ue_id = cp_ue_id_e2sm(ue);
      return &g_observed_ues[i];
    }
  }

  assert(false && "Observed UE table full");
  return NULL;
}

static int64_t safe_delta(bool has_prev, int64_t cur, int64_t prev)
{
  if (!has_prev)
    return cur;

  int64_t delta = cur - prev;

  if (delta < 0)
    return cur;

  return delta;
}

/* --------------------------------------------------------------- */
/* KPM logging and accumulation                                    */
/* --------------------------------------------------------------- */

static void log_current_int_kpm(observed_ue_t* ue,
                                byte_array_t name,
                                meas_record_lst_t meas_record)
{
  assert(ue != NULL);

  int64_t cur = meas_record.int_val;

  if (cmp_str_ba("RRU.PrbTotDl", name) == 0) {
    int64_t delta = safe_delta(ue->has_prb_dl, cur, ue->prev_prb_dl);
    ue->prev_prb_dl = cur;
    ue->has_prb_dl = true;

    ue->cur_prb_dl = delta;
    ue->acc_prb_dl += delta;

    printf("RRU.PrbTotDl.current = %ld [PRBs/1s]\n", delta);

  } else if (cmp_str_ba("RRU.PrbTotUl", name) == 0) {
    int64_t delta = safe_delta(ue->has_prb_ul, cur, ue->prev_prb_ul);
    ue->prev_prb_ul = cur;
    ue->has_prb_ul = true;

    ue->cur_prb_ul = delta;
    ue->acc_prb_ul += delta;

    printf("RRU.PrbTotUl.current = %ld [PRBs/1s]\n", delta);

  } else if (cmp_str_ba("DRB.PdcpSduVolumeDL", name) == 0) {
    int64_t delta = safe_delta(ue->has_pdcp_dl, cur, ue->prev_pdcp_dl);
    ue->prev_pdcp_dl = cur;
    ue->has_pdcp_dl = true;

    ue->cur_pdcp_dl = delta;
    ue->acc_pdcp_dl += delta;

    printf("DRB.PdcpSduVolumeDL.current = %ld [kb/1s]\n", delta);

  } else if (cmp_str_ba("DRB.PdcpSduVolumeUL", name) == 0) {
    int64_t delta = safe_delta(ue->has_pdcp_ul, cur, ue->prev_pdcp_ul);
    ue->prev_pdcp_ul = cur;
    ue->has_pdcp_ul = true;

    ue->cur_pdcp_ul = delta;
    ue->acc_pdcp_ul += delta;

    printf("DRB.PdcpSduVolumeUL.current = %ld [kb/1s]\n", delta);

  } else {
    printf("Unsupported INT measurement\n");
  }
}

static void log_real_kpm(observed_ue_t* ue,
                         byte_array_t name,
                         meas_record_lst_t meas_record)
{
  assert(ue != NULL);

  if (cmp_str_ba("DRB.RlcSduDelayDl", name) == 0) {
    ue->cur_rlc_delay_dl = meas_record.real_val;
    ue->acc_rlc_delay_dl += meas_record.real_val;
    printf("DRB.RlcSduDelayDl = %.2f [us]\n", meas_record.real_val);

  } else if (cmp_str_ba("DRB.UEThpDl", name) == 0) {
    ue->cur_ue_thp_dl = meas_record.real_val;
    ue->acc_ue_thp_dl += meas_record.real_val;
    printf("DRB.UEThpDl = %.2f [kbps]\n", meas_record.real_val);

  } else if (cmp_str_ba("DRB.UEThpUl", name) == 0) {
    ue->cur_ue_thp_ul = meas_record.real_val;
    ue->acc_ue_thp_ul += meas_record.real_val;
    printf("DRB.UEThpUl = %.2f [kbps]\n", meas_record.real_val);

  } else {
    printf("Unsupported REAL measurement\n");
    return;
  }

  ue->acc_real_cnt++;
}

static void log_kpm_measurements_current(observed_ue_t* ue,
                                         kpm_ind_msg_format_1_t const* msg_frm_1)
{
  assert(ue != NULL);
  assert(msg_frm_1 != NULL);
  assert(msg_frm_1->meas_info_lst_len > 0);

  ue->acc_sample_cnt++;

  for (size_t j = 0; j < msg_frm_1->meas_data_lst_len; j++) {
    meas_data_lst_t const data_item = msg_frm_1->meas_data_lst[j];

    for (size_t z = 0; z < data_item.meas_record_len; z++) {
      meas_type_t const meas_type = msg_frm_1->meas_info_lst[z].meas_type;
      meas_record_lst_t const record_item = data_item.meas_record_lst[z];

      if (meas_type.type != NAME_MEAS_TYPE)
        continue;

      if (record_item.value == INTEGER_MEAS_VALUE) {
        log_current_int_kpm(ue, meas_type.name, record_item);
      } else if (record_item.value == REAL_MEAS_VALUE) {
        log_real_kpm(ue, meas_type.name, record_item);
      }

      if (data_item.incomplete_flag && *data_item.incomplete_flag == TRUE_ENUM_VALUE)
        printf("Measurement Record not reliable\n");
    }
  }
}

/* --------------------------------------------------------------- */
/* UE ID logging                                                   */
/* --------------------------------------------------------------- */

static void log_ue_id(ue_id_e2sm_t const* id)
{
  assert(id != NULL);

  if (id->type == GNB_UE_ID_E2SM) {
    if (id->gnb.gnb_cu_ue_f1ap_lst != NULL) {
      for (size_t i = 0; i < id->gnb.gnb_cu_ue_f1ap_lst_len; i++)
        printf("UE ID type = gNB-CU, gnb_cu_ue_f1ap = %u\n",
               id->gnb.gnb_cu_ue_f1ap_lst[i]);
    } else {
      printf("UE ID type = gNB, amf_ue_ngap_id = %lu\n",
             id->gnb.amf_ue_ngap_id);
    }

    if (id->gnb.ran_ue_id != NULL)
      printf("ran_ue_id = %lx\n", *id->gnb.ran_ue_id);

  } else if (id->type == GNB_DU_UE_ID_E2SM) {
    printf("UE ID type = gNB-DU, gnb_cu_ue_f1ap = %u\n",
           id->gnb_du.gnb_cu_ue_f1ap);

    if (id->gnb_du.ran_ue_id != NULL)
      printf("ran_ue_id = %lx\n", *id->gnb_du.ran_ue_id);

  } else if (id->type == GNB_CU_UP_UE_ID_E2SM) {
    printf("UE ID type = gNB-CU-UP, gnb_cu_cp_ue_e1ap = %u\n",
           id->gnb_cu_up.gnb_cu_cp_ue_e1ap);

    if (id->gnb_cu_up.ran_ue_id != NULL)
      printf("ran_ue_id = %lx\n", *id->gnb_cu_up.ran_ue_id);
  }
}

/* --------------------------------------------------------------- */
/* Redis SDL                                                       */
/* --------------------------------------------------------------- */

static void reset_ue_accumulator(observed_ue_t* ue)
{
  assert(ue != NULL);

  ue->acc_prb_dl = 0;
  ue->acc_prb_ul = 0;
  ue->acc_pdcp_dl = 0;
  ue->acc_pdcp_ul = 0;

  ue->acc_rlc_delay_dl = 0.0;
  ue->acc_ue_thp_dl = 0.0;
  ue->acc_ue_thp_ul = 0.0;

  ue->acc_real_cnt = 0;
  ue->acc_sample_cnt = 0;
}

static void store_ue_kpm_to_sdl(observed_ue_t* ue, int64_t now_us)
{
  assert(ue != NULL);

  if (g_redis == NULL || g_redis->err) {
    printf("[SDL] Redis not connected\n");
    return;
  }

  char key[256];

  snprintf(key, sizeof(key),
           "slice:%u:%02x%02x%02x:ue:%lu:kpm10s",
           ue->profile.sst,
           ue->profile.sd[0],
           ue->profile.sd[1],
           ue->profile.sd[2],
           ue->key);

  double avg_rlc_delay_dl = 0.0;
  double avg_ue_thp_dl = 0.0;
  double avg_ue_thp_ul = 0.0;

  if (ue->acc_real_cnt > 0) {
    avg_rlc_delay_dl = ue->acc_rlc_delay_dl / ue->acc_real_cnt;
    avg_ue_thp_dl = ue->acc_ue_thp_dl / ue->acc_real_cnt;
    avg_ue_thp_ul = ue->acc_ue_thp_ul / ue->acc_real_cnt;
  }

  redisReply* reply = redisCommand(
      g_redis,
      "HSET %s "
      "timestamp_us %lld "
      "window_us %lld "
      "profile %s "
      "sst %u "
      "sd %02x%02x%02x "
      "ue_key %lu "
      "sample_count %llu "
      "prb_dl_sum %lld "
      "prb_ul_sum %lld "
      "pdcp_dl_kb_sum %lld "
      "pdcp_ul_kb_sum %lld "
      "rlc_delay_dl_us_avg %.2f "
      "ue_thp_dl_kbps_avg %.2f "
      "ue_thp_ul_kbps_avg %.2f "
      "last_prb_dl %lld "
      "last_prb_ul %lld "
      "last_pdcp_dl_kb %lld "
      "last_pdcp_ul_kb %lld",
      key,
      (long long)now_us,
      (long long)SDL_STORE_PERIOD_US,
      ue->profile.profile_name,
      ue->profile.sst,
      ue->profile.sd[0],
      ue->profile.sd[1],
      ue->profile.sd[2],
      ue->key,
      (unsigned long long)ue->acc_sample_cnt,
      (long long)ue->acc_prb_dl,
      (long long)ue->acc_prb_ul,
      (long long)ue->acc_pdcp_dl,
      (long long)ue->acc_pdcp_ul,
      avg_rlc_delay_dl,
      avg_ue_thp_dl,
      avg_ue_thp_ul,
      (long long)ue->cur_prb_dl,
      (long long)ue->cur_prb_ul,
      (long long)ue->cur_pdcp_dl,
      (long long)ue->cur_pdcp_ul
  );

  if (reply == NULL) {
    printf("[SDL] Redis HSET failed: %s\n", g_redis->errstr);
    return;
  }

  freeReplyObject(reply);

  printf("[SDL] stored 10s accumulated KPM key=%s\n", key);

  reset_ue_accumulator(ue);
}

static void store_all_observed_ues_to_sdl_if_needed(void)
{
  int64_t now_us = time_now_us();

  if (g_last_sdl_store_us != 0 &&
      now_us - g_last_sdl_store_us < SDL_STORE_PERIOD_US) {
    return;
  }

  g_last_sdl_store_us = now_us;

  printf("\n[SDL] storing 10s accumulated KPM to Redis\n");

  for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
    if (!g_observed_ues[i].valid)
      continue;

    store_ue_kpm_to_sdl(&g_observed_ues[i], now_us);
  }
}

/* --------------------------------------------------------------- */
/* KPM callbacks per slice                                         */
/* --------------------------------------------------------------- */

static void handle_kpm_with_profile(sm_ag_if_rd_t const* rd,
                                    slice_profile_t const* profile)
{
  assert(rd != NULL);
  assert(profile != NULL);
  assert(rd->type == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == KPM_STATS_V3_0);

  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  kpm_ric_ind_hdr_format_1_t const* hdr_frm_1 =
      &ind->hdr.kpm_ric_ind_hdr_format_1;
  kpm_ind_msg_format_3_t const* msg_frm_3 = &ind->msg.frm_3;

  static int counter = 1;
  int64_t const now = time_now_us();

  lock_guard(&mtx);

  printf("\n[KPM-MON][%s] latency = %ld [us], counter=%d\n",
         profile->profile_name, now - hdr_frm_1->collectStartTime, counter);

  printf("[KPM-MON][%s] subscribed slice=(sst=%u, sd=%02x%02x%02x)\n",
         profile->profile_name,
         profile->sst,
         profile->sd[0], profile->sd[1], profile->sd[2]);

  for (size_t i = 0; i < msg_frm_3->ue_meas_report_lst_len; i++) {
    ue_id_e2sm_t const* ue_id =
        &msg_frm_3->meas_report_per_ue[i].ue_meas_report_lst;

    observed_ue_t* ue = find_or_add_observed_ue(ue_id, profile);

    printf("[KPM-MON][%s] UE key = %lu, slice=(sst=%u, sd=%02x%02x%02x)\n",
           profile->profile_name,
           ue->key,
           profile->sst,
           profile->sd[0],
           profile->sd[1],
           profile->sd[2]);

    log_ue_id(&ue->ue_id);

    log_kpm_measurements_current(
        ue,
        &msg_frm_3->meas_report_per_ue[i].ind_msg_format_1
    );
  }

  store_all_observed_ues_to_sdl_if_needed();

  counter++;
}

static void sm_cb_kpm_sst1(sm_ag_if_rd_t const* rd)
{
  handle_kpm_with_profile(rd, &g_profile_sst1);
}

static void sm_cb_kpm_sst2(sm_ag_if_rd_t const* rd)
{
  handle_kpm_with_profile(rd, &g_profile_sst2);
}

static void sm_cb_kpm_sst3(sm_ag_if_rd_t const* rd)
{
  handle_kpm_with_profile(rd, &g_profile_sst3);
}

/* --------------------------------------------------------------- */
/* KPM subscription helpers                                        */
/* --------------------------------------------------------------- */

static test_info_lst_t filter_predicate_snssai(slice_profile_t const* profile)
{
  assert(profile != NULL);

  test_info_lst_t dst = {0};

  dst.test_cond_type = S_NSSAI_TEST_COND_TYPE;
  dst.S_NSSAI = TRUE_TEST_COND_TYPE;

  dst.test_cond = calloc(1, sizeof(test_cond_e));
  assert(dst.test_cond != NULL);
  *dst.test_cond = EQUAL_TEST_COND;

  dst.test_cond_value = calloc(1, sizeof(test_cond_value_t));
  assert(dst.test_cond_value != NULL);
  dst.test_cond_value->type = OCTET_STRING_TEST_COND_VALUE;

  dst.test_cond_value->octet_string_value = calloc(1, sizeof(byte_array_t));
  assert(dst.test_cond_value->octet_string_value != NULL);

  dst.test_cond_value->octet_string_value->len = 4;
  dst.test_cond_value->octet_string_value->buf = calloc(4, sizeof(uint8_t));
  assert(dst.test_cond_value->octet_string_value->buf != NULL);

  dst.test_cond_value->octet_string_value->buf[0] = profile->sst;
  dst.test_cond_value->octet_string_value->buf[1] = profile->sd[0];
  dst.test_cond_value->octet_string_value->buf[2] = profile->sd[1];
  dst.test_cond_value->octet_string_value->buf[3] = profile->sd[2];

  printf("[KPM-MON] subscribe profile=%s, S-NSSAI=(sst=%u, sd=%02x%02x%02x)\n",
         profile->profile_name,
         profile->sst,
         profile->sd[0], profile->sd[1], profile->sd[2]);

  return dst;
}

static label_info_lst_t fill_kpm_label(void)
{
  label_info_lst_t label_item = {0};
  label_item.noLabel = ecalloc(1, sizeof(enum_value_e));
  *label_item.noLabel = TRUE_ENUM_VALUE;
  return label_item;
}

static kpm_act_def_format_1_t fill_act_def_frm_1(
    ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);

  kpm_act_def_format_1_t ad_frm_1 = {0};
  size_t const sz = report_item->meas_info_for_action_lst_len;

  ad_frm_1.meas_info_lst_len = sz;
  ad_frm_1.meas_info_lst = calloc(sz, sizeof(meas_info_format_1_lst_t));
  assert(ad_frm_1.meas_info_lst != NULL);

  for (size_t i = 0; i < sz; i++) {
    meas_info_format_1_lst_t* meas_item = &ad_frm_1.meas_info_lst[i];

    meas_item->meas_type.type = NAME_MEAS_TYPE;
    meas_item->meas_type.name =
        copy_byte_array(report_item->meas_info_for_action_lst[i].name);

    meas_item->label_info_lst_len = 1;
    meas_item->label_info_lst = ecalloc(1, sizeof(label_info_lst_t));
    meas_item->label_info_lst[0] = fill_kpm_label();
  }

  ad_frm_1.gran_period_ms = period_ms;
  ad_frm_1.cell_global_id = NULL;

#if defined KPM_V2_03 || defined KPM_V3_00
  ad_frm_1.meas_bin_range_info_lst_len = 0;
  ad_frm_1.meas_bin_info_lst = NULL;
#endif

  return ad_frm_1;
}

static kpm_act_def_t fill_report_style_4_with_profile(
    ric_report_style_item_t const* report_item,
    slice_profile_t const* profile)
{
  assert(report_item != NULL);
  assert(profile != NULL);
  assert(report_item->act_def_format_type == FORMAT_4_ACTION_DEFINITION);

  kpm_act_def_t act_def = {.type = FORMAT_4_ACTION_DEFINITION};

  act_def.frm_4.matching_cond_lst_len = 1;
  act_def.frm_4.matching_cond_lst =
      calloc(1, sizeof(matching_condition_format_4_lst_t));
  assert(act_def.frm_4.matching_cond_lst != NULL);

  act_def.frm_4.matching_cond_lst[0].test_info_lst =
      filter_predicate_snssai(profile);

  act_def.frm_4.action_def_format_1 =
      fill_act_def_frm_1(report_item);

  return act_def;
}

static kpm_sub_data_t gen_kpm_subs_for_profile(
    kpm_ran_function_def_t const* ran_func,
    slice_profile_t const* profile)
{
  assert(ran_func != NULL);
  assert(profile != NULL);
  assert(ran_func->ric_event_trigger_style_list != NULL);

  kpm_sub_data_t kpm_sub = {0};

  assert(ran_func->ric_event_trigger_style_list[0].format_type ==
         FORMAT_1_RIC_EVENT_TRIGGER);

  kpm_sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  kpm_sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms =
      period_ms;

  kpm_sub.sz_ad = 1;
  kpm_sub.ad = calloc(kpm_sub.sz_ad, sizeof(kpm_act_def_t));
  assert(kpm_sub.ad != NULL);

  ric_report_style_item_t* const report_item =
      &ran_func->ric_report_style_list[0];

  *kpm_sub.ad = fill_report_style_4_with_profile(report_item, profile);

  return kpm_sub;
}

/* --------------------------------------------------------------- */
/* Generic RAN function lookup                                     */
/* --------------------------------------------------------------- */

static bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  return elem->id == id;
}

static size_t find_sm_idx(sm_ran_function_t* rf,
                          size_t sz,
                          bool (*f)(sm_ran_function_t const*, int const),
                          int const id)
{
  for (size_t i = 0; i < sz; i++) {
    if (f(&rf[i], id))
      return i;
  }

  assert(false && "SM ID could not be found in RAN Function List");
  return 0;
}

/* --------------------------------------------------------------- */
/* Main                                                            */
/* --------------------------------------------------------------- */

int main(int argc, char* argv[])
{
  fr_args_t args = init_fr_args(argc, argv);

  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  assert(nodes.len > 0);

  printf("[KPM-MON-XAPP] Connected E2 nodes = %d\n", nodes.len);

  pthread_mutexattr_t attr = {0};
  int rc = pthread_mutex_init(&mtx, &attr);
  assert(rc == 0);

  g_redis = redisConnect(REDIS_HOST, REDIS_PORT);

  if (g_redis == NULL || g_redis->err) {
    if (g_redis != NULL)
      printf("[SDL] Redis connection error: %s\n", g_redis->errstr);
    else
      printf("[SDL] Redis connection error: cannot allocate redis context\n");

    assert(false && "Redis connection failed");
  }

  printf("[SDL] Connected to Redis %s:%d\n", REDIS_HOST, REDIS_PORT);

  sm_ans_xapp_t* hndl_sst1 = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  sm_ans_xapp_t* hndl_sst2 = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  sm_ans_xapp_t* hndl_sst3 = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(hndl_sst1 != NULL && hndl_sst2 != NULL && hndl_sst3 != NULL);

  int const KPM_ran_function = 2;

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];
    size_t const idx = find_sm_idx(n->rf, n->len_rf, eq_sm, KPM_ran_function);

    assert(n->rf[idx].defn.type == KPM_RAN_FUNC_DEF_E);

    if (n->rf[idx].defn.kpm.ric_report_style_list == NULL)
      continue;

    kpm_sub_data_t sub_sst1 =
        gen_kpm_subs_for_profile(&n->rf[idx].defn.kpm, &g_profile_sst1);
    kpm_sub_data_t sub_sst2 =
        gen_kpm_subs_for_profile(&n->rf[idx].defn.kpm, &g_profile_sst2);
    kpm_sub_data_t sub_sst3 =
        gen_kpm_subs_for_profile(&n->rf[idx].defn.kpm, &g_profile_sst3);

    hndl_sst1[i] =
        report_sm_xapp_api(&n->id, KPM_ran_function, &sub_sst1, sm_cb_kpm_sst1);
    hndl_sst2[i] =
        report_sm_xapp_api(&n->id, KPM_ran_function, &sub_sst2, sm_cb_kpm_sst2);
    hndl_sst3[i] =
        report_sm_xapp_api(&n->id, KPM_ran_function, &sub_sst3, sm_cb_kpm_sst3);

    assert(hndl_sst1[i].success == true);
    assert(hndl_sst2[i].success == true);
    assert(hndl_sst3[i].success == true);

    free_kpm_sub_data(&sub_sst1);
    free_kpm_sub_data(&sub_sst2);
    free_kpm_sub_data(&sub_sst3);
  }

  xapp_wait_end_api();

  for (size_t i = 0; i < nodes.len; ++i) {
    if (hndl_sst1[i].success == true)
      rm_report_sm_xapp_api(hndl_sst1[i].u.handle);
    if (hndl_sst2[i].success == true)
      rm_report_sm_xapp_api(hndl_sst2[i].u.handle);
    if (hndl_sst3[i].success == true)
      rm_report_sm_xapp_api(hndl_sst3[i].u.handle);
  }

  free(hndl_sst1);
  free(hndl_sst2);
  free(hndl_sst3);

  for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
    if (g_observed_ues[i].valid) {
      free_ue_id_e2sm(&g_observed_ues[i].ue_id);
      g_observed_ues[i].valid = false;
    }
  }

  while (try_stop_xapp_api() == false)
    usleep(1000);

  if (g_redis != NULL) {
    redisFree(g_redis);
    g_redis = NULL;
  }

  free_e2_node_arr_xapp(&nodes);

  rc = pthread_mutex_destroy(&mtx);
  assert(rc == 0);

  printf("[KPM-MON-XAPP] run SUCCESSFULLY\n");
  return 0;
}
