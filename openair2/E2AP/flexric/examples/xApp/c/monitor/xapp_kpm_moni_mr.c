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
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <stdbool.h>

// ---- migration framework + kpm state hooks ----
#include "migration_function.h"
#include "kpm_state.h"

static uint64_t const period_ms = 1000;

// xApp의 논리 상태 ctx (migration 대상)
static kpm_state_ctx_t kpm_ctx;

// (출력용) 버퍼 append helper
static void appendf(char** buf, size_t* cap, size_t* len, const char* fmt, ...)
{
  if (!buf || !cap || !len) return;

  if (*buf == NULL) {
    *cap = 4096;
    *len = 0;
    *buf = (char*)malloc(*cap);
    if (!*buf) return;
    (*buf)[0] = '\0';
  }

  va_list ap;
  va_start(ap, fmt);
  int needed = vsnprintf(NULL, 0, fmt, ap);
  va_end(ap);
  if (needed < 0) return;

  size_t req = *len + (size_t)needed + 1;
  if (req > *cap) {
    while (*cap < req) *cap *= 2;
    char* nb = (char*)realloc(*buf, *cap);
    if (!nb) return;
    *buf = nb;
  }

  va_start(ap, fmt);
  vsnprintf(*buf + *len, *cap - *len, fmt, ap);
  va_end(ap);

  *len += (size_t)needed;
}

static void append_ue_id(char** buf, size_t* cap, size_t* len, ue_id_e2sm_t ue_id)
{
  if (ue_id.type == GNB_UE_ID_E2SM) {
    if (ue_id.gnb.gnb_cu_ue_f1ap_lst != NULL) {
      for (size_t i = 0; i < ue_id.gnb.gnb_cu_ue_f1ap_lst_len; i++) {
        appendf(buf, cap, len, "UE ID type = gNB-CU, gnb_cu_ue_f1ap = %u\n", ue_id.gnb.gnb_cu_ue_f1ap_lst[i]);
      }
    } else {
      appendf(buf, cap, len, "UE ID type = gNB, amf_ue_ngap_id = %lu\n", ue_id.gnb.amf_ue_ngap_id);
    }
    if (ue_id.gnb.ran_ue_id != NULL) {
      appendf(buf, cap, len, "ran_ue_id = %lx\n", *ue_id.gnb.ran_ue_id);
    }
  } else if (ue_id.type == GNB_DU_UE_ID_E2SM) {
    appendf(buf, cap, len, "UE ID type = gNB-DU, gnb_cu_ue_f1ap = %u\n", ue_id.gnb_du.gnb_cu_ue_f1ap);
    if (ue_id.gnb_du.ran_ue_id != NULL) {
      appendf(buf, cap, len, "ran_ue_id = %lx\n", *ue_id.gnb_du.ran_ue_id);
    }
  } else if (ue_id.type == GNB_CU_UP_UE_ID_E2SM) {
    appendf(buf, cap, len, "UE ID type = gNB-CU-UP, gnb_cu_cp_ue_e1ap = %u\n", ue_id.gnb_cu_up.gnb_cu_cp_ue_e1ap);
    if (ue_id.gnb_cu_up.ran_ue_id != NULL) {
      appendf(buf, cap, len, "ran_ue_id = %lx\n", *ue_id.gnb_cu_up.ran_ue_id);
    }
  } else {
    appendf(buf, cap, len, "UE ID type = %d (not formatted)\n", (int)ue_id.type);
  }
}

static void append_kpm_measurements(char** buf, size_t* cap, size_t* len, kpm_ind_msg_format_1_t const* msg_frm_1)
{
  assert(msg_frm_1->meas_info_lst_len > 0);

  for (size_t j = 0; j < msg_frm_1->meas_data_lst_len; j++) {
    meas_data_lst_t const data_item = msg_frm_1->meas_data_lst[j];

    for (size_t z = 0; z < data_item.meas_record_len; z++) {
      meas_type_t const meas_type = msg_frm_1->meas_info_lst[z].meas_type;
      meas_record_lst_t const record_item = data_item.meas_record_lst[z];

      if (meas_type.type == NAME_MEAS_TYPE) {
        const size_t nlen = meas_type.name.len;
        char name_str[256];
        size_t copy = (nlen < sizeof(name_str) - 1) ? nlen : (sizeof(name_str) - 1);
        memcpy(name_str, meas_type.name.buf, copy);
        name_str[copy] = '\0';

        if (record_item.value == INTEGER_MEAS_VALUE) {
          appendf(buf, cap, len, "%s = %d\n", name_str, record_item.int_val);
        } else if (record_item.value == REAL_MEAS_VALUE) {
          appendf(buf, cap, len, "%s = %.2f\n", name_str, record_item.real_val);
        } else {
          appendf(buf, cap, len, "%s = (unsupported value type %d)\n", name_str, (int)record_item.value);
        }
      } else {
        appendf(buf, cap, len, "Measurement Type not NAME (type=%d)\n", (int)meas_type.type);
      }

      if (data_item.incomplete_flag && *data_item.incomplete_flag == TRUE_ENUM_VALUE)
        appendf(buf, cap, len, "Measurement Record not reliable\n");
    }
  }
}

/* =========================================================================
 *  KPM CALLBACK (🔥 수정)
 * ========================= */

static void sm_cb_kpm(sm_ag_if_rd_t const* rd)
{
  assert(rd && rd->ind.type == KPM_STATS_V3_0);

  if (kpm_ctx.migrating) {
    if (kpm_cb_enter(&kpm_ctx))
      return;
  }

  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  kpm_ind_msg_format_3_t const* msg = &ind->msg.frm_3;
  int64_t now = time_now_us();

  char* report = NULL;
  size_t cap = 0, len = 0;

  appendf(&report, &cap, &len,
          "\n%7lu KPM latency = %ld [us]\n",
          (unsigned long)kpm_ctx.counter,
          now - ind->hdr.kpm_ric_ind_hdr_format_1.collectStartTime);

  for (size_t i = 0; i < msg->ue_meas_report_lst_len; ++i) {
    append_ue_id(&report, &cap, &len, msg->meas_report_per_ue[i].ue_meas_report_lst);
    append_kpm_measurements(&report, &cap, &len, &msg->meas_report_per_ue[i].ind_msg_format_1);
  }

  // 출력은 굳이 mutex 필요 없음(정말 깔끔하게 하려면 stdout 전용 mutex를 따로)
  fputs(report, stdout);
  fflush(stdout);

  // last_report는 내부에서 mutex 잡음
  kpm_update_last_report(&kpm_ctx, report, len + 1);

  // counter는 mutex로 보호(내부 mutex 재사용)
  pthread_mutex_lock(&kpm_ctx.mtx);
  kpm_ctx.counter++;
  pthread_mutex_unlock(&kpm_ctx.mtx);

  free(report);

  if (kpm_ctx.migrating) {
    kpm_cb_exit(&kpm_ctx);
  }
}


//========================================================================================================


// --- 이하 원본 그대로 ---

static test_info_lst_t filter_predicate(test_cond_type_e type, test_cond_e cond, int value)
{
  test_info_lst_t dst = {0};

  dst.test_cond_type = type;
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

static label_info_lst_t fill_kpm_label(void)
{
  label_info_lst_t label_item = {0};

  label_item.noLabel = ecalloc(1, sizeof(enum_value_e));
  *label_item.noLabel = TRUE_ENUM_VALUE;

  return label_item;
}

static kpm_act_def_format_1_t fill_act_def_frm_1(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);

  kpm_act_def_format_1_t ad_frm_1 = {0};

  size_t const sz = report_item->meas_info_for_action_lst_len;

  ad_frm_1.meas_info_lst_len = sz;
  ad_frm_1.meas_info_lst = calloc(sz, sizeof(meas_info_format_1_lst_t));
  assert(ad_frm_1.meas_info_lst != NULL && "Memory exhausted");

  for (size_t i = 0; i < sz; i++) {
    meas_info_format_1_lst_t* meas_item = &ad_frm_1.meas_info_lst[i];
    meas_item->meas_type.type = NAME_MEAS_TYPE;
    meas_item->meas_type.name = copy_byte_array(report_item->meas_info_for_action_lst[i].name);

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

static kpm_act_def_t fill_report_style_4(ric_report_style_item_t const* report_item)
{
  assert(report_item != NULL);
  assert(report_item->act_def_format_type == FORMAT_4_ACTION_DEFINITION);

  kpm_act_def_t act_def = {.type = FORMAT_4_ACTION_DEFINITION};

  act_def.frm_4.matching_cond_lst_len = 1;
  act_def.frm_4.matching_cond_lst = calloc(act_def.frm_4.matching_cond_lst_len, sizeof(matching_condition_format_4_lst_t));
  assert(act_def.frm_4.matching_cond_lst != NULL && "Memory exhausted");

  test_cond_type_e const type = S_NSSAI_TEST_COND_TYPE;
  test_cond_e const condition = EQUAL_TEST_COND;
  int const value = 1;
  act_def.frm_4.matching_cond_lst[0].test_info_lst = filter_predicate(type, condition, value);

  act_def.frm_4.action_def_format_1 = fill_act_def_frm_1(report_item);

  return act_def;
}

typedef kpm_act_def_t (*fill_kpm_act_def)(ric_report_style_item_t const* report_item);

static fill_kpm_act_def get_kpm_act_def[END_RIC_SERVICE_REPORT] = {
    NULL,
    NULL,
    NULL,
    fill_report_style_4,
    NULL,
};

static kpm_sub_data_t gen_kpm_subs(kpm_ran_function_def_t const* ran_func)
{
  assert(ran_func != NULL);
  assert(ran_func->ric_event_trigger_style_list != NULL);

  kpm_sub_data_t kpm_sub = {0};

  assert(ran_func->ric_event_trigger_style_list[0].format_type == FORMAT_1_RIC_EVENT_TRIGGER);
  kpm_sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  kpm_sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms = period_ms;

  kpm_sub.sz_ad = 1;
  kpm_sub.ad = calloc(kpm_sub.sz_ad, sizeof(kpm_act_def_t));
  assert(kpm_sub.ad != NULL && "Memory exhausted");

  ric_report_style_item_t* const report_item = &ran_func->ric_report_style_list[0];
  ric_service_report_e const report_style_type = report_item->report_style_type;
  *kpm_sub.ad = get_kpm_act_def[report_style_type](report_item);

  return kpm_sub;
}

static bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  if (elem->id == id)
    return true;

  return false;
}

static size_t find_sm_idx(sm_ran_function_t* rf, size_t sz, bool (*f)(sm_ran_function_t const*, int const), int const id)
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

  assert(kpm_state_init(&kpm_ctx) == 0);
  kpm_ctx.migrating = false;

  // ====== HERE: NODE_ID 기반으로 UDS 경로 결정 ======
  const char* node_id = getenv("NODE_ID");
  if (node_id == NULL || node_id[0] == '\0') {
    node_id = "A"; // 기본값
  }

  char sock_path[PATH_MAX];
  snprintf(sock_path, sizeof(sock_path), "/tmp/xapp_%s.sock", node_id);

  mig_ops_t ops = kpm_get_mig_ops();
  assert(mig_server_start_uds(sock_path, ops, &kpm_ctx) != NULL);

  printf("[MR] NODE_ID=%s, mig UDS=%s\n", node_id, sock_path);

  // Init the xApp
  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  defer({ free_e2_node_arr_xapp(&nodes); });

  assert(nodes.len > 0);

  printf("Connected E2 nodes = %d\n", nodes.len);

  sm_ans_xapp_t* hndl = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(hndl != NULL);

  int const KPM_ran_function = 2;

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];

    size_t const idx = find_sm_idx(n->rf, n->len_rf, eq_sm, KPM_ran_function);
    assert(n->rf[idx].defn.type == KPM_RAN_FUNC_DEF_E && "KPM is not the received RAN Function");

    if (n->rf[idx].defn.kpm.ric_report_style_list != NULL) {
      kpm_sub_data_t kpm_sub = gen_kpm_subs(&n->rf[idx].defn.kpm);

      hndl[i] = report_sm_xapp_api(&n->id, KPM_ran_function, &kpm_sub, sm_cb_kpm);
      assert(hndl[i].success == true);

      free_kpm_sub_data(&kpm_sub);
    }
  }

  xapp_wait_end_api();

  for (int i = 0; i < nodes.len; ++i) {
    if (hndl[i].success == true)
      rm_report_sm_xapp_api(hndl[i].u.handle);
  }
  free(hndl);

  while (try_stop_xapp_api() == false)
    usleep(1000);

  kpm_state_destroy(&kpm_ctx);

  printf("Test xApp run SUCCESSFULLY\n");
}

