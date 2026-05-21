/*
 * OAI/FlexRIC xApp example (RLC-driven periodic TC control) - richer policy
 *
 * - RLC indication에서 다양한 state 구성: max/avg/rb_len + EWMA + trend + PI error
 * - CTRL_PERIOD_US마다 PCR(BDP) MOD 제어(drbsz) 내려줌
 * - Queue/Classifier는 1회만 설치 (ADD)
 * - Ctrl+C(SIGINT) clean stop 지원 (init_xapp_api() 이후 sigaction으로 재설치)
 *
 * [MOD] Multi-port classifier:
 *  - 5201, 5202 모두 동일 dst_queue=1로 분류되도록 classifier 2개 설치
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/sm/slice_sm/slice_sm_id.h"
#include "../../../../src/util/alg_ds/alg/defer.h"
#include "../../../../src/util/time_now_us.h"

#include <assert.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <signal.h>
#include <poll.h>
#include <arpa/inet.h>
#include <math.h>

// =================== Tuning knobs ===================

#ifndef RLC_REPORT_INTERVAL_STR
#define RLC_REPORT_INTERVAL_STR "5_ms"
#endif

// 5ms * 100 = 0.5s -> 1초에 2번 출력
#ifndef PRINT_EVERY_N_IND
#define PRINT_EVERY_N_IND 100
#endif

// 제어 주기: 0.5s -> 1초에 2번 제어
#ifndef CTRL_PERIOD_US
#define CTRL_PERIOD_US 500000ULL
#endif

#ifndef IND_STALE_US
#define IND_STALE_US 1000000ULL
#endif

#ifndef RLC_SM_ID
#define RLC_SM_ID 143
#endif

#ifndef TC_SM_ID
#define TC_SM_ID 146
#endif

// --------- Classifier ports (multi UE iperf) ---------
// UE1: iperf dst port 5201
// UE2: iperf dst port 5202
#ifndef CLS_DST_PORT1
#define CLS_DST_PORT1 5201
#endif
#ifndef CLS_DST_PORT2
#define CLS_DST_PORT2 5202
#endif

#ifndef CODEL_INTERVAL_MS
#define CODEL_INTERVAL_MS 400
#endif

#ifndef CODEL_TARGET_MS
#define CODEL_TARGET_MS 20
#endif

// ---------- Policy parameters (tune here) ----------

// 목표 max 지연(ms). (실험 편의상 10ms 근처로 맞춤)
#ifndef TARGET_MAX_MS
#define TARGET_MAX_MS 10.0
#endif

// EWMA 계수 (0~1). 클수록 현재값 반영 큼(=노이즈 증가)
#ifndef EWMA_ALPHA
#define EWMA_ALPHA 0.25
#endif

// PI gains (경험적으로 시작하는 값: 필요시 튜닝)
#ifndef KP
#define KP 0.9
#endif
#ifndef KI
#define KI 0.15
#endif

// 적분 windup 제한
#ifndef I_MIN
#define I_MIN (-50.0)
#endif
#ifndef I_MAX
#define I_MAX (50.0)
#endif

// 추세(trend) 가중치 (max 지연이 증가 중이면 선제 tightening)
#ifndef K_TREND
#define K_TREND 0.8
#endif

// RB 개수 가중치 (혼잡 징후로 소폭 tightening)
#ifndef K_RBLEN
#define K_RBLEN 0.15
#endif

// drb_sz 변경 최소 간격(레이트 리밋) (us)
#ifndef MIN_ACT_GAP_US
#define MIN_ACT_GAP_US 200000ULL   // 200ms
#endif

// drb_sz 히스테리시스 (같은 레벨 유지하려는 완충)
#ifndef SCORE_HYST
#define SCORE_HYST 0.6
#endif

// =================== Global state ===================

static _Atomic int g_stop = 0;
static _Atomic int g_sig_cnt = 0;

// RLC raw stats
static _Atomic uint32_t g_max_wt_ms   = 0;
static _Atomic uint32_t g_avg_wt_ms   = 0;
static _Atomic uint32_t g_rb_len      = 0;
static _Atomic uint64_t g_last_ind_us = 0;
static _Atomic uint64_t g_ind_cnt     = 0;

// Policy states (migration 관점에서 “내부 상태”)
static _Atomic double g_ewma_max = 0.0;
static _Atomic double g_ewma_avg = 0.0;
static _Atomic double g_trend_max = 0.0;
static _Atomic double g_ierr = 0.0;

// TC install flag
static _Atomic int g_tc_installed = 0;

// last applied action/state
static _Atomic uint32_t g_last_drb_sz = 0;
static _Atomic double   g_last_score  = 0.0;
static _Atomic uint64_t g_last_act_us  = 0;

// =================== Signal handler ===================
// signal handler에서는 printf 금지(비동기-시그널 안전 아님). write 사용.
static void on_sigint(int sig)
{
  (void)sig;
  int c = atomic_fetch_add(&g_sig_cnt, 1) + 1;

  if (c == 1) {
    const char msg[] = "\n[SIG] SIGINT/SIGTERM received (soft stop)\n";
    (void)write(1, msg, sizeof(msg) - 1);
    atomic_store(&g_stop, 1);
  } else {
    const char msg[] = "\n[SIG] received again (hard exit)\n";
    (void)write(1, msg, sizeof(msg) - 1);
    _exit(1);
  }
}

static void install_signal_handlers_after_init(void)
{
  struct sigaction sa;
  sa.sa_handler = on_sigint;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;

  sigaction(SIGINT,  &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
}

// =================== TC control message generators ===================

static tc_ctrl_msg_t gen_mod_bdp_pcr(uint32_t drb_sz)
{
  tc_ctrl_msg_t ans = {
    .type = TC_CTRL_SM_V0_PCR,
    .pcr.act = TC_CTRL_ACTION_SM_V0_MOD
  };

  ans.pcr.mod.type = TC_PCR_5G_BDP;
  ans.pcr.mod.bdp.tstamp = time_now_us();
  ans.pcr.mod.bdp.drb_sz = drb_sz;
  return ans;
}

static tc_ctrl_msg_t gen_add_codel_queue(uint32_t interval_ms, uint32_t target_ms)
{
  tc_ctrl_msg_t ans = {
    .type = TC_CTRL_SM_V0_QUEUE,
    .q.act = TC_CTRL_ACTION_SM_V0_ADD
  };

  tc_add_ctrl_queue_t* q = &ans.q.add;
  q->type = TC_QUEUE_CODEL;
  q->codel.interval_ms = interval_ms;
  q->codel.target_ms = target_ms;
  return ans;
}

static tc_ctrl_msg_t gen_add_osi_cls(uint32_t dst_port)
{
  tc_ctrl_msg_t ans = {
    .type = TC_CTRL_SM_V0_CLS,
    .cls.act = TC_CTRL_ACTION_SM_V0_ADD
  };

  tc_add_ctrl_cls_t* add = &ans.cls.add;
  add->type = TC_CLS_OSI;
  add->osi.dst_queue = 1;          // <<< 두 포트 모두 같은 큐(1)로 보냄
  add->osi.l3.src_addr = -1;
  add->osi.l3.dst_addr = -1;
  add->osi.l4.src_port = -1;
  add->osi.l4.dst_port = dst_port;
  add->osi.l4.protocol = -1;
  return ans;
}

// =================== RLC callback: raw + EWMA + trend update ===================

static void sm_cb_rlc(sm_ag_if_rd_t const* rd)
{
  assert(rd != NULL);
  assert(rd->type == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == RLC_STATS_V0);

  rlc_ind_msg_t const* msg = &rd->ind.rlc.msg;

  uint32_t max_wt = 0;
  uint64_t sum_wt = 0;

  for (size_t i = 0; i < msg->len; ++i) {
    rlc_radio_bearer_stats_t const* rb = &msg->rb[i];
    uint32_t wt = (uint32_t)rb->txpdu_wt_ms;
    if (wt > max_wt) max_wt = wt;
    sum_wt += wt;
  }

  uint32_t avg_wt = 0;
  if (msg->len > 0)
    avg_wt = (uint32_t)(sum_wt / msg->len);

  uint64_t now_us = (uint64_t)time_now_us();

  atomic_store(&g_max_wt_ms, max_wt);
  atomic_store(&g_avg_wt_ms, avg_wt);
  atomic_store(&g_rb_len, (uint32_t)msg->len);
  atomic_store(&g_last_ind_us, now_us);

  // EWMA / trend update (atomic double store)
  double prev_ewma_max = atomic_load(&g_ewma_max);
  double prev_ewma_avg = atomic_load(&g_ewma_avg);

  double cur_ewma_max = (prev_ewma_max == 0.0)
                        ? (double)max_wt
                        : (EWMA_ALPHA * (double)max_wt + (1.0 - EWMA_ALPHA) * prev_ewma_max);

  double cur_ewma_avg = (prev_ewma_avg == 0.0)
                        ? (double)avg_wt
                        : (EWMA_ALPHA * (double)avg_wt + (1.0 - EWMA_ALPHA) * prev_ewma_avg);

  // trend: ewma_max의 변화량(간단히 diff로 근사)
  double trend = cur_ewma_max - prev_ewma_max;

  atomic_store(&g_ewma_max, cur_ewma_max);
  atomic_store(&g_ewma_avg, cur_ewma_avg);
  atomic_store(&g_trend_max, trend);

  uint64_t c = atomic_fetch_add(&g_ind_cnt, 1) + 1;

  if (PRINT_EVERY_N_IND > 0 && (c % PRINT_EVERY_N_IND) == 0) {
    printf("[IND] #%lu t=%lu us rb_len=%u max=%u ms avg=%u ms  ewma_max=%.2f ewma_avg=%.2f trend=%.2f\n",
           (unsigned long)c,
           (unsigned long)now_us,
           (unsigned)msg->len,
           (unsigned)max_wt,
           (unsigned)avg_wt,
           cur_ewma_max, cur_ewma_avg, trend);
    fflush(stdout);
  }
}

// =================== Policy: PI + trend + rb_len => score => drb_sz ===================

// score가 커질수록 "더 타이트"(drb_sz 감소)하게 가는 구조
// score = KP*err + KI*I(err) + K_TREND*trend + K_RBLEN*rb_len_term
static double compute_score_and_update_ierr(double ewma_max, double trend_max, uint32_t rb_len)
{
  // err > 0 이면 목표(TARGET_MAX_MS)보다 지연이 큰 상태
  double err = ewma_max - TARGET_MAX_MS;

  double ierr = atomic_load(&g_ierr);
  ierr += err; // CTRL_PERIOD마다 1 step 적분(시간스케일은 gain으로 조절)

  if (ierr < I_MIN) ierr = I_MIN;
  if (ierr > I_MAX) ierr = I_MAX;
  atomic_store(&g_ierr, ierr);

  // rb_len 항은 너무 크면 과대해지니 log로 완화
  double rb_term = (rb_len > 0) ? log((double)rb_len + 1.0) : 0.0;

  double score = KP * err + KI * ierr + K_TREND * trend_max + K_RBLEN * rb_term;
  return score;
}

// score -> drb_sz mapping (다단계)
// score가 클수록(나쁠수록) drb_sz를 낮춰 pacing 강화(=타이트)
static uint32_t score_to_drb_sz(double score, uint32_t last_drb_sz)
{
  // 히스테리시스: 기존 레벨 주변에서는 쉽게 바뀌지 않게 완충
  double prev_score = atomic_load(&g_last_score);

  // 현재 score가 이전과 큰 차이 없으면 안정 유지 유도
  if (fabs(score - prev_score) < SCORE_HYST) {
    return last_drb_sz;
  }

  if (score <= -6.0) return 80000;
  if (score <= -2.5) return 65000;
  if (score <=  1.0) return 50000;
  if (score <=  5.0) return 35000;
  if (score <=  9.0) return 25000;
  return 18000;
}

// =================== Install TC once ===================
static void install_tc_once(global_e2_node_id_t* node_id, uint32_t tc_sm_id)
{
  int expected = 0;
  if (!atomic_compare_exchange_strong(&g_tc_installed, &expected, 1))
    return;

  // Queue ADD (1회)
  {
    sm_ag_if_wr_t wr_q = {.type = CONTROL_SM_AG_IF_WR};
    wr_q.ctrl.type = TC_CTRL_REQ_V0;
    wr_q.ctrl.tc_req_ctrl.msg = gen_add_codel_queue(CODEL_INTERVAL_MS, CODEL_TARGET_MS);

    printf("[CTRL-INIT] ADD QUEUE: CoDel interval_ms=%u target_ms=%u\n",
           (unsigned)CODEL_INTERVAL_MS, (unsigned)CODEL_TARGET_MS);

    sm_ans_xapp_t a = control_sm_xapp_api(node_id, tc_sm_id, &wr_q);
    if (!a.success) printf("[CTRL-INIT] WARN: queue add failed\n");

    free_tc_ctrl_msg(&wr_q.ctrl.tc_req_ctrl.msg);
  }

  // Classifier ADD (5201 -> queue 1)
  {
    sm_ag_if_wr_t wr_cls = {.type = CONTROL_SM_AG_IF_WR};
    wr_cls.ctrl.type = TC_CTRL_REQ_V0;
    wr_cls.ctrl.tc_req_ctrl.msg = gen_add_osi_cls(CLS_DST_PORT1);

    printf("[CTRL-INIT] ADD CLS: OSI dst_port=%u -> dst_queue=1\n", (unsigned)CLS_DST_PORT1);

    sm_ans_xapp_t a = control_sm_xapp_api(node_id, tc_sm_id, &wr_cls);
    if (!a.success) printf("[CTRL-INIT] WARN: classifier add failed (port %u)\n", (unsigned)CLS_DST_PORT1);

    free_tc_ctrl_msg(&wr_cls.ctrl.tc_req_ctrl.msg);
  }

  // Classifier ADD (5202 -> queue 1)  <<< [MOD] 추가된 부분
  {
    sm_ag_if_wr_t wr_cls = {.type = CONTROL_SM_AG_IF_WR};
    wr_cls.ctrl.type = TC_CTRL_REQ_V0;
    wr_cls.ctrl.tc_req_ctrl.msg = gen_add_osi_cls(CLS_DST_PORT2);

    printf("[CTRL-INIT] ADD CLS: OSI dst_port=%u -> dst_queue=1\n", (unsigned)CLS_DST_PORT2);

    sm_ans_xapp_t a = control_sm_xapp_api(node_id, tc_sm_id, &wr_cls);
    if (!a.success) printf("[CTRL-INIT] WARN: classifier add failed (port %u)\n", (unsigned)CLS_DST_PORT2);

    free_tc_ctrl_msg(&wr_cls.ctrl.tc_req_ctrl.msg);
  }

  printf("[CTRL-INIT] TC initial rules installed (ports %u,%u -> queue 1).\n",
         (unsigned)CLS_DST_PORT1, (unsigned)CLS_DST_PORT2);
  fflush(stdout);
}

// =================== main ===================

int main(int argc, char *argv[])
{
  fr_args_t args = init_fr_args(argc, argv);

  init_xapp_api(&args);
  sleep(1);

  // init 이후에 우리가 다시 signal handler 덮어쓰기
  install_signal_handlers_after_init();

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  defer({ free_e2_node_arr_xapp(&nodes); });
  assert(nodes.len > 0);

  printf("Connected E2 nodes = %d\n", nodes.len);

  e2_node_connected_xapp_t* n = &nodes.n[0];
  for (size_t k = 0; k < n->len_rf; ++k)
    printf("Registered ran func id = %d\n", n->rf[k].id);

  // ---------------- RLC subscribe ----------------
  const char* inter = RLC_REPORT_INTERVAL_STR;
  sm_ans_xapp_t rlc_h = report_sm_xapp_api(&nodes.n[0].id, RLC_SM_ID, (void*)inter, sm_cb_rlc);
  assert(rlc_h.success == true);
  printf("Registered to RLC SM (id=%d interval=%s)\n", RLC_SM_ID, inter);

  // ---------------- TC install once ----------------
  install_tc_once(&nodes.n[0].id, TC_SM_ID);

  // ---------------- Periodic control loop ----------------
  uint64_t last_ctrl_us = (uint64_t)time_now_us();

  // 초기 drb_sz (안전한 중간값)
  atomic_store(&g_last_drb_sz, 50000);

  while (!atomic_load(&g_stop)) {

    uint64_t now_us = (uint64_t)time_now_us();

    if (now_us - last_ctrl_us >= CTRL_PERIOD_US) {

      uint64_t last_ind = atomic_load(&g_last_ind_us);

      if (last_ind != 0 && (now_us - last_ind) < IND_STALE_US) {

        // diverse inputs
        uint32_t max_wt = atomic_load(&g_max_wt_ms);
        uint32_t avg_wt = atomic_load(&g_avg_wt_ms);
        uint32_t rb_len = atomic_load(&g_rb_len);

        double ewma_max = atomic_load(&g_ewma_max);
        double ewma_avg = atomic_load(&g_ewma_avg);
        double trend    = atomic_load(&g_trend_max);

        // compute score (PI + trend + rb_len)
        double score = compute_score_and_update_ierr(ewma_max, trend, rb_len);

        uint32_t last_drb = atomic_load(&g_last_drb_sz);
        uint32_t next_drb = score_to_drb_sz(score, last_drb);

        // rate limit: 너무 자주 바뀌면 안정성이 떨어짐
        uint64_t last_act = atomic_load(&g_last_act_us);
        int can_change = (now_us - last_act >= MIN_ACT_GAP_US);

        if (next_drb != last_drb && can_change) {

          sm_ag_if_wr_t wr_pcr = {.type = CONTROL_SM_AG_IF_WR};
          wr_pcr.ctrl.type = TC_CTRL_REQ_V0;
          wr_pcr.ctrl.tc_req_ctrl.msg = gen_mod_bdp_pcr(next_drb);

          printf("[CTRL] PCR MOD: drb_sz %u -> %u | score=%.2f (max=%u avg=%u rb=%u ewma_max=%.2f ewma_avg=%.2f trend=%.2f ierr=%.2f) t=%lu us\n",
                 (unsigned)last_drb, (unsigned)next_drb,
                 score,
                 (unsigned)max_wt, (unsigned)avg_wt, (unsigned)rb_len,
                 ewma_max, ewma_avg, trend,
                 atomic_load(&g_ierr),
                 (unsigned long)now_us);

          sm_ans_xapp_t a = control_sm_xapp_api(&nodes.n[0].id, TC_SM_ID, &wr_pcr);
          if (!a.success) printf("[CTRL] WARN: PCR mod failed\n");

          free_tc_ctrl_msg(&wr_pcr.ctrl.tc_req_ctrl.msg);

          atomic_store(&g_last_drb_sz, next_drb);
          atomic_store(&g_last_score, score);
          atomic_store(&g_last_act_us, now_us);

          fflush(stdout);

        } else {
          printf("[CTRL] HOLD: drb_sz=%u | score=%.2f (ewma_max=%.2f trend=%.2f rb=%u) %s t=%lu us\n",
                 (unsigned)last_drb, score, ewma_max, trend, (unsigned)rb_len,
                 (next_drb != last_drb && !can_change) ? "[rate-limited]" : "",
                 (unsigned long)now_us);
          atomic_store(&g_last_score, score);
          fflush(stdout);
        }

      } else {
        printf("[CTRL] Skip (no recent RLC indication) t=%lu us\n", (unsigned long)now_us);
        fflush(stdout);
      }

      last_ctrl_us = now_us;
    }

    poll(NULL, 0, 10);
  }

  // ---------------- graceful shutdown ----------------
  printf("[EXIT] signal received. Unsubscribing RLC...\n");
  fflush(stdout);

  if (rlc_h.success) {
    rm_report_sm_xapp_api(rlc_h.u.handle);
    printf("[EXIT] Unsubscribed RLC report handle.\n");
    fflush(stdout);
  }

  poll(NULL, 0, 300);

  while (try_stop_xapp_api() == false)
    usleep(1000);

  printf("Test xApp run SUCCESSFULLY\n");
  return 0;
}
