// migration_action.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const char* src_node;          // "A"
  const char* dst_node;          // "B"
  const char* xapp_path;         // 실행할 xApp 바이너리 경로 (예: ./build/.../xapp_kpm_moni_mr)
  int terminate_src;             // 1이면 마지막에 source 종료(TERMINATE)
  int wait_retries;              // dst UDS ping 재시도 횟수
  int wait_us;                   // 재시도 간격 us

  // ✅ 추가: dst xApp 실행 시 넘길 XAPP_DURATION(ms) 문자열
  // 예: "0"(무한), "5000"(5초), "10000"(10초)
  const char* xapp_duration_ms;
} mig_action_cfg_t;

typedef struct {
  long long quiesce_us;
  long long export_us;
  long long spawn_wait_us;
  long long import_us;
  long long resume_us;
  long long total_us;
  unsigned long long state_bytes;
} mig_action_stats_t;

int run_stopcopy_migration_nodeid(const mig_action_cfg_t* cfg, mig_action_stats_t* out_stats);

#ifdef __cplusplus
}
#endif

