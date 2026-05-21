/*
 * Burst-aware UE priority xApp (dry-run)
 *
 * 목적:
 *  - OAI KPM에서 현재 받을 수 있는 metric만 사용
 *  - UE별 내부 상태(state) 계산
 *  - burst / starvation / fairness 기반 priority 생성
 *  - 현재는 dry-run 로그 출력
 *
 * 전제:
 *  - OAI + FlexRIC build
 *  - KPM report style 4 사용
 *
 * 참고:
 *  - 실제 scheduler 반영은 기본 OAI에서 바로 되지 않으므로
 *    send_priority_control()는 현재 placeholder
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/util/time_now_us.h"
#include "../../../../src/util/alg_ds/ds/lock_guard/lock_guard.h"

#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define KPM_RAN_FUNCTION_ID 2
#define REPORT_PERIOD_MS 100

#define MAX_UE 128
#define SHORT_WIN 10   // 10 * 100ms = 1s
#define LONG_WIN  50   // 50 * 100ms = 5s

#define BURST_TH 1.5
#define FAIRNESS_CONGESTION_TH 0.85
#define PRIORITY_MIN 0.5
#define PRIORITY_MAX 2.0

typedef struct {
  bool valid;

  // UE identity
  uint64_t amf_ue_ngap_id;
  uint64_t ran_ue_id;
  uint32_t gnb_cu_ue_f1ap;

  // raw instantaneous metrics
  double thp_dl_inst;
  double thp_ul_inst;
  double pdcp_vol_dl_inst;
  double pdcp_vol_ul_inst;
  double rlc_delay_dl_inst;

  // history
  double thp_dl_hist[LONG_WIN];
  double thp_ul_hist[LONG_WIN];
  double delay_hist[LONG_WIN];
  double vol_dl_hist[LONG_WIN];
  double vol_ul_hist[LONG_WIN];
  int hist_idx;
  int hist_count;

  // derived state
  double thp_dl_avg_short;
  double thp_dl_avg_long;
  double thp_ul_avg_short;
  double thp_ul_avg_long;
  double pdcp_vol_dl_delta;
  double pdcp_vol_ul_delta;
  double thp_dl_trend;
  double delay_trend;
  double burst_score;
  double starvation_score;
  double normalized_share;

  // policy output
  double priority_dl;
  double priority_ul;
} ue_state_t;

typedef struct {
  ue_state_t ue[MAX_UE];
  int ue_count;

  // cell/global state
  double total_dl_thp;
  double total_ul_thp;
  double total_prb_dl;
  double total_prb_ul;
  int active_ue_num;
  double fairness_index;
  int burst_ue_count;
  bool congestion_flag;
} xapp_state_t;

static pthread_mutex_t g_mtx;
static xapp_state_t g_state = {0};

/* ---------- helpers ---------- */

static double clamp(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static double avg_last(const double* arr, int hist_count, int hist_idx, int win)
{
  if (hist_count == 0) return 0.0;
  int n = hist_count < win ? hist_count : win;
  double s = 0.0;
  for (int i = 0; i < n; ++i) {
    int idx = (hist_idx - 1 - i + LONG_WIN) % LONG_WIN;
    s += arr[idx];
  }
  return s / n;
}

static double last_minus_prev(const double* arr, int hist_count, int hist_idx)
{
  if (hist_count < 2) return 0.0;
  int last = (hist_idx - 1 + LONG_WIN) % LONG_WIN;
  int prev = (hist_idx - 2 + LONG_WIN) % LONG_WIN;
  return arr[last] - arr[prev];
}

static double jains_fairness(const double* x, int n)
{
  if (n <= 0) return 1.0;
  double sum = 0.0, sq = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += x[i];
    sq += x[i] * x[i];
  }
  if (sq <= 1e-12) return 1.0;
  return (sum * sum) / (n * sq);
}

static int find_or_create_ue_slot(uint64_t amf_ue_ngap_id, uint64_t ran_ue_id, uint32_t gnb_cu_ue_f1ap)
{
  for (int i = 0; i < MAX_UE; ++i) {
    if (!g_state.ue[i].valid) continue;
    if (g_state.ue[i].amf_ue_ngap_id == amf_ue_ngap_id &&
        g_state.ue[i].ran_ue_id == ran_ue_id &&
        g_state.ue[i].gnb_cu_ue_f1ap == gnb_cu_ue_f1ap) {
      return i;
    }
  }

  for (int i = 0; i < MAX_UE; ++i) {
    if (!g_state.ue[i].valid) {
      g_state.ue[i].valid = true;
      g_state.ue[i].amf_ue_ngap_id = amf_ue_ngap_id;
      g_state.ue[i].ran_ue_id = ran_ue_id;
      g_state.ue[i].gnb_cu_ue_f1ap = gnb_cu_ue_f1ap;
      g_state.ue_count++;
      return i;
    }
  }

  assert(false && "No free UE slot");
  return -1;
}

static void push_hist(ue_state_t* u)
{
  int idx = u->hist_idx;
  u->thp_dl_hist[idx] = u->thp_dl_inst;
  u->thp_ul_hist[idx] = u->thp_ul_inst;
  u->delay_hist[idx]  = u->rlc_delay_dl_inst;
  u->vol_dl_hist[idx] = u->pdcp_vol_dl_inst;
  u->vol_ul_hist[idx] = u->pdcp_vol_ul_inst;

  u->hist_idx = (u->hist_idx + 1) % LONG_WIN;
  if (u->hist_count < LONG_WIN) u->hist_count++;
}

static void derive_ue_state(ue_state_t* u, double total_dl_thp, int active_ue_num)
{
  u->thp_dl_avg_short = avg_last(u->thp_dl_hist, u->hist_count, u->hist_idx, SHORT_WIN);
  u->thp_dl_avg_long  = avg_last(u->thp_dl_hist, u->hist_count, u->hist_idx, LONG_WIN);

  u->thp_ul_avg_short = avg_last(u->thp_ul_hist, u->hist_count, u->hist_idx, SHORT_WIN);
  u->thp_ul_avg_long  = avg_last(u->thp_ul_hist, u->hist_count, u->hist_idx, LONG_WIN);

  u->pdcp_vol_dl_delta = last_minus_prev(u->vol_dl_hist, u->hist_count, u->hist_idx);
  u->pdcp_vol_ul_delta = last_minus_prev(u->vol_ul_hist, u->hist_count, u->hist_idx);

  u->thp_dl_trend = last_minus_prev(u->thp_dl_hist, u->hist_count, u->hist_idx);
  u->delay_trend  = last_minus_prev(u->delay_hist,  u->hist_count, u->hist_idx);

  if (u->thp_dl_avg_long > 1e-9)
    u->burst_score = u->thp_dl_avg_short / u->thp_dl_avg_long;
  else
    u->burst_score = 1.0;

  double expected_share = (active_ue_num > 0) ? (total_dl_thp / active_ue_num) : 0.0;
  if (expected_share > 1e-9)
    u->starvation_score = fmax(0.0, (expected_share - u->thp_dl_inst) / expected_share);
  else
    u->starvation_score = 0.0;

  if (total_dl_thp > 1e-9)
    u->normalized_share = u->thp_dl_inst / total_dl_thp;
  else
    u->normalized_share = 0.0;
}

static void compute_global_state(void)
{
  g_state.total_dl_thp = 0.0;
  g_state.total_ul_thp = 0.0;
  g_state.active_ue_num = 0;
  g_state.burst_ue_count = 0;

  double arr[MAX_UE] = {0};
  int n = 0;

  for (int i = 0; i < MAX_UE; ++i) {
    ue_state_t* u = &g_state.ue[i];
    if (!u->valid) continue;

    g_state.total_dl_thp += u->thp_dl_inst;
    g_state.total_ul_thp += u->thp_ul_inst;

    if (u->thp_dl_inst > 1e-6 || u->thp_ul_inst > 1e-6)
      g_state.active_ue_num++;

    if (u->burst_score > BURST_TH)
      g_state.burst_ue_count++;

    arr[n++] = u->thp_dl_inst;
  }

  g_state.fairness_index = jains_fairness(arr, n);
  g_state.congestion_flag = (g_state.fairness_index < FAIRNESS_CONGESTION_TH);
}

static void generate_priority_policy(void)
{
  for (int i = 0; i < MAX_UE; ++i) {
    ue_state_t* u = &g_state.ue[i];
    if (!u->valid) continue;

    double p = 1.0;

    // burst 억제
    if (u->burst_score > BURST_TH) {
      double excess = u->burst_score - BURST_TH;
      p -= 0.25 * excess;
    }

    // starvation 보호
    p += 0.6 * u->starvation_score;

    // fairness가 나쁘고 특정 UE share가 큰 경우 추가 억제
    if (g_state.congestion_flag && u->normalized_share > 0.5) {
      p -= 0.2;
    }

    // delay가 너무 늘어나면 약간 우대
    if (u->delay_trend > 0.0) {
      p += 0.05;
    }

    u->priority_dl = clamp(p, PRIORITY_MIN, PRIORITY_MAX);
    u->priority_ul = clamp(1.0, PRIORITY_MIN, PRIORITY_MAX); // 현재는 UL 단순 유지
  }
}

static void send_priority_control(const ue_state_t* u)
{
  // 현재는 dry-run
  // 기본 OAI에는 UE priority를 scheduler에 바로 반영하는 표준 control이 없으므로
  // 여기서는 로그만 출력. 추후 scheduler patch 후 실제 control로 교체.
  printf("[DRY-RUN CTRL] UE(amf=%lu ran=%lu f1ap=%u) -> DL_priority=%.2f UL_priority=%.2f\n",
         u->amf_ue_ngap_id, u->ran_ue_id, u->gnb_cu_ue_f1ap, u->priority_dl, u->priority_ul);
}

static void print_state_summary(void)
{
  printf("\n=== GLOBAL STATE ===\n");
  printf("active_ue=%d total_dl_thp=%.2f total_ul_thp=%.2f fairness=%.3f burst_ue=%d congestion=%d\n",
         g_state.active_ue_num, g_state.total_dl_thp, g_state.total_ul_thp,
         g_state.fairness_index, g_state.burst_ue_count, g_state.congestion_flag);

  for (int i = 0; i < MAX_UE; ++i) {
    const ue_state_t* u = &g_state.ue[i];
    if (!u->valid) continue;

    printf("UE(amf=%lu ran=%lu f1ap=%u) "
           "thp_dl=%.2f short=%.2f long=%.2f burst=%.2f starv=%.2f share=%.2f prio=%.2f\n",
           u->amf_ue_ngap_id, u->ran_ue_id, u->gnb_cu_ue_f1ap,
           u->thp_dl_inst, u->thp_dl_avg_short, u->thp_dl_avg_long,
           u->burst_score, u->starvation_score, u->normalized_share, u->priority_dl);
  }
}

/* ---------- KPM parsing ---------- */

static uint64_t extract_amf_ue_ngap_id(ue_id_e2sm_t const* id)
{
  if (id->type == GNB_UE_ID_E2SM)
    return id->gnb.amf_ue_ngap_id;
  return 0;
}

static uint64_t extract_ran_ue_id(ue_id_e2sm_t const* id)
{
  if (id->type == GNB_UE_ID_E2SM && id->gnb.ran_ue_id != NULL)
    return *id->gnb.ran_ue_id;
  if (id->type == GNB_DU_UE_ID_E2SM && id->gnb_du.ran_ue_id != NULL)
    return *id->gnb_du.ran_ue_id;
  if (id->type == GNB_CU_UP_UE_ID_E2SM && id->gnb_cu_up.ran_ue_id != NULL)
    return *id->gnb_cu_up.ran_ue_id;
  return 0;
}

static uint32_t extract_f1ap_id(ue_id_e2sm_t const* id)
{
  if (id->type == GNB_UE_ID_E2SM && id->gnb.gnb_cu_ue_f1ap_lst != NULL && id->gnb.gnb_cu_ue_f1ap_lst_len > 0)
    return id->gnb.gnb_cu_ue_f1ap_lst[0];
  if (id->type == GNB_DU_UE_ID_E2SM)
    return id->gnb_du.gnb_cu_ue_f1ap;
  return 0;
}

static void update_meas_by_name(ue_state_t* u, byte_array_t name, meas_record_lst_t rec)
{
  if (cmp_str_ba("DRB.UEThpDl", name) == 0 && rec.value == REAL_MEAS_VALUE) {
    u->thp_dl_inst = rec.real_val;
  } else if (cmp_str_ba("DRB.UEThpUl", name) == 0 && rec.value == REAL_MEAS_VALUE) {
    u->thp_ul_inst = rec.real_val;
  } else if (cmp_str_ba("DRB.PdcpSduVolumeDL", name) == 0 && rec.value == INTEGER_MEAS_VALUE) {
    u->pdcp_vol_dl_inst = rec.int_val;
  } else if (cmp_str_ba("DRB.PdcpSduVolumeUL", name) == 0 && rec.value == INTEGER_MEAS_VALUE) {
    u->pdcp_vol_ul_inst = rec.int_val;
  } else if (cmp_str_ba("DRB.RlcSduDelayDl", name) == 0 && rec.value == REAL_MEAS_VALUE) {
    u->rlc_delay_dl_inst = rec.real_val;
  } else if (cmp_str_ba("RRU.PrbTotDl", name) == 0 && rec.value == INTEGER_MEAS_VALUE) {
    g_state.total_prb_dl = rec.int_val;
  } else if (cmp_str_ba("RRU.PrbTotUl", name) == 0 && rec.value == INTEGER_MEAS_VALUE) {
    g_state.total_prb_ul = rec.int_val;
  }
}

static void parse_ue_meas_report(kpm_ind_msg_format_3_t const* msg_frm_3)
{
  for (size_t i = 0; i < msg_frm_3->ue_meas_report_lst_len; ++i) {
    ue_id_e2sm_t const* id = &msg_frm_3->meas_report_per_ue[i].ue_meas_report_lst;

    uint64_t amf  = extract_amf_ue_ngap_id(id);
    uint64_t ran  = extract_ran_ue_id(id);
    uint32_t f1ap = extract_f1ap_id(id);

    int slot = find_or_create_ue_slot(amf, ran, f1ap);
    ue_state_t* u = &g_state.ue[slot];

    kpm_ind_msg_format_1_t const* frm1 = &msg_frm_3->meas_report_per_ue[i].ind_msg_format_1;

    for (size_t j = 0; j < frm1->meas_data_lst_len; ++j) {
      meas_data_lst_t const* data_item = &frm1->meas_data_lst[j];

      for (size_t z = 0; z < data_item->meas_record_len; ++z) {
        meas_type_t const meas_type = frm1->meas_info_lst[z].meas_type;
        meas_record_lst_t const rec = data_item->meas_record_lst[z];

        if (meas_type.type == NAME_MEAS_TYPE) {
          update_meas_by_name(u, meas_type.name, rec);
        }
      }
    }

    push_hist(u);
  }
}

static void sm_cb_kpm(sm_ag_if_rd_t const* rd)
{
  assert(rd != NULL);
  assert(rd->type == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == KPM_STATS_V3_0);

  lock_guard(&g_mtx);

  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  kpm_ind_msg_format_3_t const* msg_frm_3 = &ind->msg.frm_3;
  parse_ue_meas_report(msg_frm_3);

  compute_global_state();

  for (int i = 0; i < MAX_UE; ++i) {
    ue_state_t* u = &g_state.ue[i];
    if (!u->valid) continue;
    derive_ue_state(u, g_state.total_dl_thp, g_state.active_ue_num);
  }

  generate_priority_policy();
  print_state_summary();

  for (int i = 0; i < MAX_UE; ++i) {
    ue_state_t* u = &g_state.ue[i];
    if (!u->valid) continue;
    send_priority_control(ue_id, dl_weight, ul_weight, validity_ms);
  }
}

/* ---------- KPM subscription ---------- */

static label_info_lst_t fill_kpm_label(void)
{
  label_info_lst_t label_item = {0};
  label_item.noLabel = calloc(1, sizeof(enum_value_e));
  assert(label_item.noLabel != NULL);
  *label_item.noLabel = TRUE_ENUM_VALUE;
  return label_item;
}

static kpm_act_def_format_1_t fill_act_def_frm_1(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);

  kpm_act_def_format_1_t ad = {0};
  ad.meas_info_lst_len = report_item->meas_info_for_action_lst_len;
  ad.meas_info_lst = calloc(ad.meas_info_lst_len, sizeof(meas_info_format_1_lst_t));
  assert(ad.meas_info_lst != NULL);

  for (size_t i = 0; i < ad.meas_info_lst_len; ++i) {
    ad.meas_info_lst[i].meas_type.type = NAME_MEAS_TYPE;
    ad.meas_info_lst[i].meas_type.name = copy_byte_array(report_item->meas_info_for_action_lst[i].name);
    ad.meas_info_lst[i].label_info_lst_len = 1;
    ad.meas_info_lst[i].label_info_lst = calloc(1, sizeof(label_info_lst_t));
    assert(ad.meas_info_lst[i].label_info_lst != NULL);
    ad.meas_info_lst[i].label_info_lst[0] = fill_kpm_label();
  }

  ad.gran_period_ms = REPORT_PERIOD_MS;
  ad.cell_global_id = NULL;
#if defined KPM_V2_03 || defined KPM_V3_00
  ad.meas_bin_range_info_lst_len = 0;
  ad.meas_bin_info_lst = NULL;
#endif
  return ad;
}

static test_info_lst_t filter_predicate_s_nssai(int value)
{
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
  dst.test_cond_value->octet_string_value->len = 1;
  dst.test_cond_value->octet_string_value->buf = calloc(1, sizeof(uint8_t));
  assert(dst.test_cond_value->octet_string_value->buf != NULL);
  dst.test_cond_value->octet_string_value->buf[0] = value;
  return dst;
}

static kpm_act_def_t fill_report_style_4(ric_report_style_item_t const* report_item)
{
  kpm_act_def_t act_def = {.type = FORMAT_4_ACTION_DEFINITION};

  act_def.frm_4.matching_cond_lst_len = 1;
  act_def.frm_4.matching_cond_lst = calloc(1, sizeof(matching_condition_format_4_lst_t));
  assert(act_def.frm_4.matching_cond_lst != NULL);
  act_def.frm_4.matching_cond_lst[0].test_info_lst = filter_predicate_s_nssai(1);

  act_def.frm_4.action_def_format_1 = fill_act_def_frm_1(report_item);
  return act_def;
}

static kpm_sub_data_t gen_kpm_subs(kpm_ran_function_def_t const* ran_func)
{
  assert(ran_func != NULL);
  assert(ran_func->ric_event_trigger_style_list != NULL);

  kpm_sub_data_t sub = {0};
  sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms = REPORT_PERIOD_MS;

  sub.sz_ad = 1;
  sub.ad = calloc(1, sizeof(kpm_act_def_t));
  assert(sub.ad != NULL);

  ric_report_style_item_t* report_item = &ran_func->ric_report_style_list[0];
  *sub.ad = fill_report_style_4(report_item);

  return sub;
}

static bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  return elem->id == id;
}

static size_t find_sm_idx(sm_ran_function_t* rf, size_t sz,
                          bool (*f)(sm_ran_function_t const*, int const), int const id)
{
  for (size_t i = 0; i < sz; ++i) {
    if (f(&rf[i], id))
      return i;
  }
  assert(false && "SM ID not found");
  return 0;
}

/* ---------- main ---------- */

int main(int argc, char* argv[])
{
  fr_args_t args = init_fr_args(argc, argv);
  init_xapp_api(&args);
  sleep(1);

  int rc = pthread_mutex_init(&g_mtx, NULL);
  assert(rc == 0);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  assert(nodes.len > 0);

  printf("[BURST-PRIO-XAPP] connected E2 nodes = %d\n", nodes.len);

  sm_ans_xapp_t* hndl = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(hndl != NULL);

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];
    size_t idx = find_sm_idx(n->rf, n->len_rf, eq_sm, KPM_RAN_FUNCTION_ID);
    assert(n->rf[idx].defn.type == KPM_RAN_FUNC_DEF_E);

    if (n->rf[idx].defn.kpm.ric_report_style_list != NULL) {
      kpm_sub_data_t sub = gen_kpm_subs(&n->rf[idx].defn.kpm);
      hndl[i] = report_sm_xapp_api(&n->id, KPM_RAN_FUNCTION_ID, &sub, sm_cb_kpm);
      assert(hndl[i].success == true);
      free_kpm_sub_data(&sub);
    }
  }

  xapp_wait_end_api();

  for (int i = 0; i < nodes.len; ++i) {
    if (hndl[i].success == true)
      rm_report_sm_xapp_api(hndl[i].u.handle);
  }

  free(hndl);
  free_e2_node_arr_xapp(&nodes);

  while (try_stop_xapp_api() == false)
    usleep(1000);

  rc = pthread_mutex_destroy(&g_mtx);
  assert(rc == 0);

  printf("[BURST-PRIO-XAPP] stopped\n");
  return 0;
}
