// kpm_state.h
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <stdbool.h>

#include "migration_function.h"  // mig_ops_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // paused / inflight 보호용 (quiesce 시 in-flight 정리)
  pthread_mutex_t mtx;
  pthread_cond_t  cv;

  int paused;              // 1이면 콜백에서 drop
  unsigned inflight;       // 실행 중 콜백 개수

  // ---- 논리 상태 ----
  uint64_t counter;        // 기존 static counter 대체
  bool migrating;           // migration 중 여부 (세미콜론 필수)

  // ---- "현재 수집된 KPM" 스냅샷 ----
  // 가장 최근 KPM 리포트를 텍스트로 저장
  char*   last_report;
  size_t  last_report_len; // bytes (NUL 제외/포함은 아래 구현에서 포함)
} kpm_state_ctx_t;

int  kpm_state_init(kpm_state_ctx_t* ctx);
void kpm_state_destroy(kpm_state_ctx_t* ctx);

// 콜백 진입/종료 헬퍼: paused/inflight 관리
// enter가 1이면 paused 상태이므로 콜백에서 즉시 return 하세요.
int  kpm_cb_enter(kpm_state_ctx_t* ctx);
void kpm_cb_exit(kpm_state_ctx_t* ctx);

// last_report 업데이트(내부 복사)
int  kpm_update_last_report(kpm_state_ctx_t* ctx, const char* buf, size_t len);

// migration_function 프레임워크에 넘길 ops
mig_ops_t kpm_get_mig_ops(void);

#ifdef __cplusplus
}
#endif

