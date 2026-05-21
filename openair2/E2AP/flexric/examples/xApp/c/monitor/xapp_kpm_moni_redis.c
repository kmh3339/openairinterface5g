/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BAS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/util/alg_ds/alg/defer.h"
#include "../../../../src/util/time_now_us.h"
#include "../../../../src/util/alg_ds/ds/lock_guard/lock_guard.h"
#include "../../../../src/util/e.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>

#include <hiredis/hiredis.h>

static
uint64_t const period_ms = 1000;

static
pthread_mutex_t mtx;

/* =========================
 * Redis(SDL) settings
 * ========================= */
static const char* REDIS_HOST = "127.0.0.1";
static const int   REDIS_PORT = 6379;

/* UE별 상태를 “최신만” 유지하려면 TTL 권장 (초 단위)
 * 0이면 TTL 미사용(영구 저장) */
static const int SDL_TTL_SEC = 0; // 예: 30으로 바꾸면 30초 후 자동 만료

static redisContext* g_redis = NULL;

/* Redis 연결/재연결 */
static redisContext* connect_redis(void)
{
  redisContext* c = redisConnect(REDIS_HOST, REDIS_PORT);
  if (c == NULL) {
    fprintf(stderr, "[Redis] redisConnect returned NULL\n");
    return NULL;
  }
  if (c->err) {
    fprintf(stderr, "[Redis] connect error: %s\n", c->errstr);
    redisFree(c);
    return NULL;
  }
  return c;
}

/* Redis 명령 reply 에러 체크 */
static int redis_check_reply(redisContext* c, redisReply* r, const char* what)
{
  if (r == NULL) {
    if (c && c->err) fprintf(stderr, "[Redis] %s: %s\n", what, c->errstr);
    else fprintf(stderr, "[Redis] %s: NULL reply\n", what);
    return -1;
  }
  if (r->type == REDIS_REPLY_ERROR) {
    fprintf(stderr, "[Redis] %s: ERROR reply: %s\n", what, r->str ? r->str : "(null)");
    return -1;
  }
  return 0;
}

/* Redis가 죽었거나 끊긴 경우를 대비한 “필요 시 재연결” */
static void ensure_redis_connected(void)
{
  if (g_redis == NULL) {
    g_redis = connect_redis();
    return;
  }
  if (g_redis->err) {
    fprintf(stderr, "[Redis] context error detected: %s (reconnecting)\n", g_redis->errstr);
    redisFree(g_redis);
    g_redis = connect_redis();
  }
}

/* byte_array_t -> null-terminated C string */
static char* ba_to_cstr(byte_array_t ba)
{
  if (ba.len == 0 || ba.buf == NULL) {
    char* s = (char*)calloc(1, 1);
    return s;
  }
  char* s = (char*)calloc(ba.len + 1, 1);
  if (!s) return NULL;
  memcpy(s, ba.buf, ba.len);
  s[ba.len] = '\0';
  return s;
}

/* UE ID를 키 문자열로 만들기 */
static void build_ue_key(char* out, size_t out_sz, ue_id_e2sm_t ue_id)
{
  if (out == NULL || out_sz == 0) return;

  // type에 따라 식별자 조합이 다름
  switch (ue_id.type) {
    case GNB_UE_ID_E2SM: {
      // gNB-mono / CU / CU-CP 공통
      unsigned long amf = (unsigned long)ue_id.gnb.amf_ue_ngap_id;
      unsigned long ran = 0;
      if (ue_id.gnb.ran_ue_id != NULL) ran = (unsigned long)(*ue_id.gnb.ran_ue_id);
      // gnb_cu_ue_f1ap 리스트가 있으면 첫 번째만 써도 되고, 전체를 키에 넣으면 너무 길어질 수 있음
      if (ue_id.gnb.gnb_cu_ue_f1ap_lst != NULL && ue_id.gnb.gnb_cu_ue_f1ap_lst_len > 0) {
        unsigned int f1 = ue_id.gnb.gnb_cu_ue_f1ap_lst[0];
        snprintf(out, out_sz, "gnb_cu_f1ap=%u|amf=%lu|ran=%lu", f1, amf, ran);
      } else {
        snprintf(out, out_sz, "gnb|amf=%lu|ran=%lu", amf, ran);
      }
    } break;

    case GNB_DU_UE_ID_E2SM: {
      unsigned int f1 = ue_id.gnb_du.gnb_cu_ue_f1ap;
      unsigned long ran = 0;
      if (ue_id.gnb_du.ran_ue_id != NULL) ran = (unsigned long)(*ue_id.gnb_du.ran_ue_id);
      snprintf(out, out_sz, "gnb_du|f1ap=%u|ran=%lu", f1, ran);
    } break;

    case GNB_CU_UP_UE_ID_E2SM: {
      unsigned int e1 = ue_id.gnb_cu_up.gnb_cu_cp_ue_e1ap;
      unsigned long ran = 0;
      if (ue_id.gnb_cu_up.ran_ue_id != NULL) ran = (unsigned long)(*ue_id.gnb_cu_up.ran_ue_id);
      snprintf(out, out_sz, "gnb_cuup|e1ap=%u|ran=%lu", e1, ran);
    } break;

    default:
      snprintf(out, out_sz, "unknown");
      break;
  }
}

/* Redis(SDL)에 UE별 KPM 상태를 저장 */
static void sdl_store_kpm(
  long e2node_id,
  ue_id_e2sm_t ue_id,
  int64_t latency_us,
  byte_array_t meas_name,
  meas_record_lst_t meas_record
)
{
  ensure_redis_connected();
  if (g_redis == NULL) return;

  char ue_key[256];
  memset(ue_key, 0, sizeof(ue_key));
  build_ue_key(ue_key, sizeof(ue_key), ue_id);

  // Key: sdl:kpm:<node_id>:<ue_key>
  char redis_key[512];
  snprintf(redis_key, sizeof(redis_key), "sdl:kpm:%ld:%s", e2node_id, ue_key);

  // Measurement name -> cstr
  char* name = ba_to_cstr(meas_name);
  if (name == NULL) return;

  // Measurement value -> string
  char val_buf[128];
  memset(val_buf, 0, sizeof(val_buf));

  if (meas_record.value == INTEGER_MEAS_VALUE) {
    snprintf(val_buf, sizeof(val_buf), "%d", meas_record.int_val);
  } else if (meas_record.value == REAL_MEAS_VALUE) {
    // 소수점 자리수는 필요에 따라 조절
    snprintf(val_buf, sizeof(val_buf), "%.6f", meas_record.real_val);
  } else {
    // 미지원 타입은 저장 생략
    free(name);
    return;
  }

  // 1) 기본 메타 업데이트 (ts_us, latency_us)
  int64_t now = time_now_us();
  redisReply* r1 = (redisReply*)redisCommand(
    g_redis,
    "HSET %s ts_us %lld latency_us %lld",
    redis_key,
    (long long)now,
    (long long)latency_us
  );
  if (redis_check_reply(g_redis, r1, "HSET meta") != 0) {
    if (r1) freeReplyObject(r1);
    free(name);
    return;
  }
  freeReplyObject(r1);

  // 2) 측정치 업데이트 (필드명 = Measurement name)
  redisReply* r2 = (redisReply*)redisCommand(
    g_redis,
    "HSET %s %s %s",
    redis_key,
    name,
    val_buf
  );
  if (redis_check_reply(g_redis, r2, "HSET meas") != 0) {
    if (r2) freeReplyObject(r2);
    free(name);
    return;
  }
  freeReplyObject(r2);

  // 3) 버전 증가 (업데이트 카운터)
  char ver_key[560];
  snprintf(ver_key, sizeof(ver_key), "%s:ver", redis_key);
  redisReply* r3 = (redisReply*)redisCommand(g_redis, "INCR %s", ver_key);
  if (redis_check_reply(g_redis, r3, "INCR ver") == 0) {
    // 필요하면 출력
    // printf("[SDL] %s ver=%lld\n", ver_key, (long long)r3->integer);
  }
  if (r3) freeReplyObject(r3);

  // 4) TTL 옵션 (최신 상태만 유지하려면 TTL 권장)
  if (SDL_TTL_SEC > 0) {
    redisReply* r4 = (redisReply*)redisCommand(g_redis, "EXPIRE %s %d", redis_key, SDL_TTL_SEC);
    if (r4) freeReplyObject(r4);
    redisReply* r5 = (redisReply*)redisCommand(g_redis, "EXPIRE %s %d", ver_key, SDL_TTL_SEC);
    if (r5) freeReplyObject(r5);
  }

  free(name);
}

static
void log_gnb_ue_id(ue_id_e2sm_t ue_id)
{
  if (ue_id.gnb.gnb_cu_ue_f1ap_lst != NULL) {
    for (size_t i = 0; i < ue_id.gnb.gnb_cu_ue_f1ap_lst_len; i++) {
      printf("UE ID type = gNB-CU, gnb_cu_ue_f1ap = %u\n", ue_id.gnb.gnb_cu_ue_f1ap_lst[i]);
    }
  } else {
    printf("UE ID type = gNB, amf_ue_ngap_id = %lu\n", ue_id.gnb.amf_ue_ngap_id);
  }
  if (ue_id.gnb.ran_ue_id != NULL) {
    printf("ran_ue_id = %lx\n", *ue_id.gnb.ran_ue_id); // RAN UE NGAP ID
  }
}

static
void log_du_ue_id(ue_id_e2sm_t ue_id)
{
  printf("UE ID type = gNB-DU, gnb_cu_ue_f1ap = %u\n", ue_id.gnb_du.gnb_cu_ue_f1ap);
  if (ue_id.gnb_du.ran_ue_id != NULL) {
    printf("ran_ue_id = %lx\n", *ue_id.gnb_du.ran_ue_id); // RAN UE NGAP ID
  }
}

static
void log_cuup_ue_id(ue_id_e2sm_t ue_id)
{
  printf("UE ID type = gNB-CU-UP, gnb_cu_cp_ue_e1ap = %u\n", ue_id.gnb_cu_up.gnb_cu_cp_ue_e1ap);
  if (ue_id.gnb_cu_up.ran_ue_id != NULL) {
    printf("ran_ue_id = %lx\n", *ue_id.gnb_cu_up.ran_ue_id); // RAN UE NGAP ID
  }
}

typedef void (*log_ue_id)(ue_id_e2sm_t ue_id);

static
log_ue_id log_ue_id_e2sm[END_UE_ID_E2SM] = {
    log_gnb_ue_id, // common for gNB-mono, CU and CU-CP
    log_du_ue_id,
    log_cuup_ue_id,
    NULL,
    NULL,
    NULL,
    NULL,
};

static
void log_int_value(byte_array_t name, meas_record_lst_t meas_record)
{
  if (cmp_str_ba("RRU.PrbTotDl", name) == 0) {
    printf("RRU.PrbTotDl = %d [PRBs]\n", meas_record.int_val);
  } else if (cmp_str_ba("RRU.PrbTotUl", name) == 0) {
    printf("RRU.PrbTotUl = %d [PRBs]\n", meas_record.int_val);
  } else if (cmp_str_ba("DRB.PdcpSduVolumeDL", name) == 0) {
    printf("DRB.PdcpSduVolumeDL = %d [kb]\n", meas_record.int_val);
  } else if (cmp_str_ba("DRB.PdcpSduVolumeUL", name) == 0) {
    printf("DRB.PdcpSduVolumeUL = %d [kb]\n", meas_record.int_val);
  } else {
    printf("Measurement Name not yet supported\n");
  }
}

static
void log_real_value(byte_array_t name, meas_record_lst_t meas_record)
{
  if (cmp_str_ba("DRB.RlcSduDelayDl", name) == 0) {
    printf("DRB.RlcSduDelayDl = %.2f [μs]\n", meas_record.real_val);
  } else if (cmp_str_ba("DRB.UEThpDl", name) == 0) {
    printf("DRB.UEThpDl = %.2f [kbps]\n", meas_record.real_val);
  } else if (cmp_str_ba("DRB.UEThpUl", name) == 0) {
    printf("DRB.UEThpUl = %.2f [kbps]\n", meas_record.real_val);
  } else {
    printf("Measurement Name not yet supported\n");
  }
}

typedef void (*log_meas_value)(byte_array_t name, meas_record_lst_t meas_record);

static
log_meas_value get_meas_value[END_MEAS_VALUE] = {
    log_int_value,
    log_real_value,
    NULL,
};

static
void match_meas_name_type(meas_type_t meas_type, meas_record_lst_t meas_record)
{
  // Get the value of the Measurement
  get_meas_value[meas_record.value](meas_type.name, meas_record);
}

static
void match_id_meas_type(meas_type_t meas_type, meas_record_lst_t meas_record)
{
  (void)meas_type;
  (void)meas_record;
  assert(false && "ID Measurement Type not yet supported");
}

typedef void (*check_meas_type)(meas_type_t meas_type, meas_record_lst_t meas_record);

static
check_meas_type match_meas_type[END_MEAS_TYPE] = {
    match_meas_name_type,
    match_id_meas_type,
};

static
void log_kpm_measurements(kpm_ind_msg_format_1_t const* msg_frm_1)
{
  assert(msg_frm_1->meas_info_lst_len > 0 && "Cannot correctly print measurements");

  // UE Measurements per granularity period
  for (size_t j = 0; j < msg_frm_1->meas_data_lst_len; j++) {
    meas_data_lst_t const data_item = msg_frm_1->meas_data_lst[j];

    for (size_t z = 0; z < data_item.meas_record_len; z++) {
      meas_type_t const meas_type = msg_frm_1->meas_info_lst[z].meas_type;
      meas_record_lst_t const record_item = data_item.meas_record_lst[z];

      match_meas_type[meas_type.type](meas_type, record_item);

      if (data_item.incomplete_flag && *data_item.incomplete_flag == TRUE_ENUM_VALUE)
        printf("Measurement Record not reliable");
    }
  }

}

static
void sm_cb_kpm(sm_ag_if_rd_t const* rd)
{
  assert(rd != NULL);
  assert(rd->type == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == KPM_STATS_V3_0);

  // Reading Indication Message Format 3
  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  kpm_ric_ind_hdr_format_1_t const* hdr_frm_1 = &ind->hdr.kpm_ric_ind_hdr_format_1;
  kpm_ind_msg_format_3_t const* msg_frm_3 = &ind->msg.frm_3;

  int64_t const now = time_now_us();
  static int counter = 1;
  {
    lock_guard(&mtx);

    int64_t latency_us = now - hdr_frm_1->collectStartTime;

    printf("\n%7d KPM ind_msg latency = %ld [μs]\n", counter, latency_us); // xApp <-> E2 Node

    // Reported list of measurements per UE
    for (size_t i = 0; i < msg_frm_3->ue_meas_report_lst_len; i++) {
      // log UE ID
      ue_id_e2sm_t const ue_id_e2sm = msg_frm_3->meas_report_per_ue[i].ue_meas_report_lst;
      ue_id_e2sm_e const type = ue_id_e2sm.type;
      log_ue_id_e2sm[type](ue_id_e2sm);

      // log measurements (stdout)
      log_kpm_measurements(&msg_frm_3->meas_report_per_ue[i].ind_msg_format_1);

      // ===== Redis(SDL) 저장 =====
      // msg format 1 내부: meas_info_lst(z) <-> meas_record_lst(z)로 매칭
      kpm_ind_msg_format_1_t const* frm1 = &msg_frm_3->meas_report_per_ue[i].ind_msg_format_1;
      assert(frm1->meas_info_lst_len > 0);

      // UE Measurements per granularity period
      for (size_t j = 0; j < frm1->meas_data_lst_len; j++) {
        meas_data_lst_t const data_item = frm1->meas_data_lst[j];

        for (size_t z = 0; z < data_item.meas_record_len; z++) {
          meas_type_t const meas_type = frm1->meas_info_lst[z].meas_type;
          meas_record_lst_t const record_item = data_item.meas_record_lst[z];

          // 이름 기반 meas만 저장 (ID meas type은 미지원)
          if (meas_type.type == NAME_MEAS_TYPE) {
            // e2 node id는 rd->ind..에서 직접 얻기 어렵고, 여기서는 "xApp이 연결한 node id"를 추적하지 않음.
            // 대신, 이 샘플은 e2node_id를 0으로 저장. 실제로는 subscription 등록할 때 node id를 콜백 컨텍스트로 넘기는 방식 추천.
            // 우선 "0"으로 저장하되, 아래 TODO 참고.
            long e2node_id = 0;

            sdl_store_kpm(
              e2node_id,
              ue_id_e2sm,
              latency_us,
              meas_type.name,
              record_item
            );
          }
        }
      }
      // ===== Redis(SDL) 저장 끝 =====
    }
    counter++;
  }
}

static
test_info_lst_t filter_predicate(test_cond_type_e type, test_cond_e cond, int value)
{
  test_info_lst_t dst = {0};

  dst.test_cond_type = type;
  // It can only be TRUE_TEST_COND_TYPE so it does not matter the type
  // but ugly ugly...
  dst.S_NSSAI = TRUE_TEST_COND_TYPE;

  dst.test_cond = calloc(1, sizeof(test_cond_e));
  assert(dst.test_cond != NULL && "Memory exhausted");
  *dst.test_cond = cond;

  dst.test_cond_value = calloc(1, sizeof(test_cond_value_t));
  assert(dst.test_cond_value != NULL && "Memory exhausted");
  dst.test_cond_value->type = OCTET_STRING_TEST_COND_VALUE;

  dst.test_cond_value->octet_string_value = calloc(1, sizeof(byte_array_t));
  assert(dst.test_cond_value->octet_string_value != NULL && "Memory exhausted");
  const size_t len_nssai = 1;
  dst.test_cond_value->octet_string_value->len = len_nssai;
  dst.test_cond_value->octet_string_value->buf = calloc(len_nssai, sizeof(uint8_t));
  assert(dst.test_cond_value->octet_string_value->buf != NULL && "Memory exhausted");
  dst.test_cond_value->octet_string_value->buf[0] = value;

  return dst;
}

static
label_info_lst_t fill_kpm_label(void)
{
  label_info_lst_t label_item = {0};

  label_item.noLabel = ecalloc(1, sizeof(enum_value_e));
  *label_item.noLabel = TRUE_ENUM_VALUE;

  return label_item;
}

static
kpm_act_def_format_1_t fill_act_def_frm_1(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);

  kpm_act_def_format_1_t ad_frm_1 = {0};

  size_t const sz = report_item->meas_info_for_action_lst_len;

  // [1, 65535]
  ad_frm_1.meas_info_lst_len = sz;
  ad_frm_1.meas_info_lst = calloc(sz, sizeof(meas_info_format_1_lst_t));
  assert(ad_frm_1.meas_info_lst != NULL && "Memory exhausted");

  for (size_t i = 0; i < sz; i++) {
    meas_info_format_1_lst_t* meas_item = &ad_frm_1.meas_info_lst[i];
    // 8.3.9
    // Measurement Name
    meas_item->meas_type.type = NAME_MEAS_TYPE;
    meas_item->meas_type.name = copy_byte_array(report_item->meas_info_for_action_lst[i].name);

    // [1, 2147483647]
    // 8.3.11
    meas_item->label_info_lst_len = 1;
    meas_item->label_info_lst = ecalloc(1, sizeof(label_info_lst_t));
    meas_item->label_info_lst[0] = fill_kpm_label();
  }

  // 8.3.8 [0, 4294967295]
  ad_frm_1.gran_period_ms = period_ms;

  // 8.3.20 - OPTIONAL
  ad_frm_1.cell_global_id = NULL;

#if defined KPM_V2_03 || defined KPM_V3_00
  // [0, 65535]
  ad_frm_1.meas_bin_range_info_lst_len = 0;
  ad_frm_1.meas_bin_info_lst = NULL;
#endif

  return ad_frm_1;
}

static
kpm_act_def_t fill_report_style_4(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);
  assert(report_item->act_def_format_type == FORMAT_4_ACTION_DEFINITION);

  kpm_act_def_t act_def = {.type = FORMAT_4_ACTION_DEFINITION};

  // Fill matching condition
  // [1, 32768]
  act_def.frm_4.matching_cond_lst_len = 1;
  act_def.frm_4.matching_cond_lst = calloc(act_def.frm_4.matching_cond_lst_len, sizeof(matching_condition_format_4_lst_t));
  assert(act_def.frm_4.matching_cond_lst != NULL && "Memory exhausted");
  // Filter connected UEs by S-NSSAI criteria
  test_cond_type_e const type = S_NSSAI_TEST_COND_TYPE; // CQI_TEST_COND_TYPE
  test_cond_e const condition = EQUAL_TEST_COND; // GREATERTHAN_TEST_COND
  int const value = 1;
  act_def.frm_4.matching_cond_lst[0].test_info_lst = filter_predicate(type, condition, value);

  // Fill Action Definition Format 1
  // 8.2.1.2.1
  act_def.frm_4.action_def_format_1 = fill_act_def_frm_1(report_item);

  return act_def;
}

typedef kpm_act_def_t (*fill_kpm_act_def)(ric_report_style_item_t const* report_item);

static
fill_kpm_act_def get_kpm_act_def[END_RIC_SERVICE_REPORT] = {
    NULL,
    NULL,
    NULL,
    fill_report_style_4,
    NULL,
};

static
kpm_sub_data_t gen_kpm_subs(kpm_ran_function_def_t const* ran_func)
{
  assert(ran_func != NULL);
  assert(ran_func->ric_event_trigger_style_list != NULL);

  kpm_sub_data_t kpm_sub = {0};

  // Generate Event Trigger
  assert(ran_func->ric_event_trigger_style_list[0].format_type == FORMAT_1_RIC_EVENT_TRIGGER);
  kpm_sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  kpm_sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms = period_ms;

  // Generate Action Definition
  kpm_sub.sz_ad = 1;
  kpm_sub.ad = calloc(kpm_sub.sz_ad, sizeof(kpm_act_def_t));
  assert(kpm_sub.ad != NULL && "Memory exhausted");

  // Multiple Action Definitions in one SUBSCRIPTION message is not supported in this project
  // Multiple REPORT Styles = Multiple Action Definition = Multiple SUBSCRIPTION messages
  ric_report_style_item_t* const report_item = &ran_func->ric_report_style_list[0];
  ric_service_report_e const report_style_type = report_item->report_style_type;
  *kpm_sub.ad = get_kpm_act_def[report_style_type](report_item);

  return kpm_sub;
}

static
bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  if (elem->id == id)
    return true;

  return false;
}

static
size_t find_sm_idx(sm_ran_function_t* rf, size_t sz, bool (*f)(sm_ran_function_t const*, int const), int const id)
{
  for (size_t i = 0; i < sz; i++) {
    if (f(&rf[i], id))
      return i;
  }

  assert(0 != 0 && "SM ID could not be found in the RAN Function List");
}

int main(int argc, char* argv[])
{
  fr_args_t args = init_fr_args(argc, argv);

  // Init the xApp
  init_xapp_api(&args);
  sleep(1);

  // Connect Redis once at startup (reconnect is handled in ensure_redis_connected)
  g_redis = connect_redis();
  if (g_redis != NULL) {
    printf("[Redis] Connected to %s:%d\n", REDIS_HOST, REDIS_PORT);
  } else {
    printf("[Redis] Not connected (will retry on first store)\n");
  }

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  defer({ free_e2_node_arr_xapp(&nodes); });

  assert(nodes.len > 0);

  printf("Connected E2 nodes = %d\n", nodes.len);

  pthread_mutexattr_t attr = {0};
  int rc = pthread_mutex_init(&mtx, &attr);
  assert(rc == 0);

  sm_ans_xapp_t* hndl = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(hndl != NULL);

  ////////////
  // START KPM
  ////////////
  int const KPM_ran_function = 2;

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];

    size_t const idx = find_sm_idx(n->rf, n->len_rf, eq_sm, KPM_ran_function);
    assert(n->rf[idx].defn.type == KPM_RAN_FUNC_DEF_E && "KPM is not the received RAN Function");

    // if REPORT Service is supported by E2 node, send SUBSCRIPTION
    // e.g. OAI CU-CP
    if (n->rf[idx].defn.kpm.ric_report_style_list != NULL) {
      // Generate KPM SUBSCRIPTION message
      kpm_sub_data_t kpm_sub = gen_kpm_subs(&n->rf[idx].defn.kpm);

      hndl[i] = report_sm_xapp_api(&n->id, KPM_ran_function, &kpm_sub, sm_cb_kpm);
      assert(hndl[i].success == true);

      free_kpm_sub_data(&kpm_sub);
    }
  }
  ////////////
  // END KPM
  ////////////

  xapp_wait_end_api();

  for (int i = 0; i < (int)nodes.len; ++i) {
    // Remove the handle previously returned
    if (hndl[i].success == true)
      rm_report_sm_xapp_api(hndl[i].u.handle);
  }
  free(hndl);

  // Stop the xApp
  while (try_stop_xapp_api() == false)
    usleep(1000);

  if (g_redis != NULL) {
    redisFree(g_redis);
    g_redis = NULL;
  }

  printf("Test xApp run SUCCESSFULLY\n");
}

