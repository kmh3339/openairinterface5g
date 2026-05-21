// migration_function.h
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================
// Protocol (UDS 메시지)
// =========================

typedef enum {
  MIG_CMD_PING      = 1,
  MIG_CMD_QUIESCE   = 2,
  MIG_CMD_EXPORT    = 3,
  MIG_CMD_IMPORT    = 4,
  MIG_CMD_RESUME    = 5,
  MIG_CMD_TERMINATE = 6
} mig_cmd_e;

typedef enum {
  MIG_OK = 0,
  MIG_ERR = 1,
  MIG_ERR_BAD_CMD = 2,
  MIG_ERR_BAD_STATE = 3,
  MIG_ERR_IO = 4,
  MIG_ERR_NO_MEM = 5,
  MIG_ERR_TOO_LARGE = 6
} mig_rc_e;

// fixed-size header (request/response 공통)
typedef struct __attribute__((packed)) {
  uint32_t magic;       // "MIG1" = 0x4D494731
  uint16_t cmd;         // mig_cmd_e
  uint16_t rc;          // mig_rc_e (request: 0, response: return code)
  uint32_t payload_len; // bytes (0 allowed)
  uint32_t reserved;    // future use (e.g., migration_id low bits)
} mig_msg_hdr_t;

#define MIG_MAGIC 0x4D494731u

// 안전한 최대 payload 크기 (MVP)
// 필요하면 키워서 쓰세요.
#ifndef MIG_MAX_PAYLOAD
#define MIG_MAX_PAYLOAD (128u * 1024u * 1024u) // 128MB
#endif

// =========================
// xApp hook interface (xApp별 구현)
// =========================
// - migration_function은 상태 의미를 모르기 때문에,
//   아래 훅을 "호출만" 합니다.
typedef struct {
  // xApp 동작 정지: 콜백/루프에서 상태 업데이트/제어 차단
  // return 0 on success
  int (*quiesce)(void* ctx);

  // xApp 동작 재개
  int (*resume)(void* ctx);

  // 논리 상태 전체 직렬화
  // out_buf는 malloc 할당(호출자가 free). out_len은 byte 길이.
  int (*export_state)(void* ctx, uint8_t** out_buf, size_t* out_len);

  // 논리 상태 복원
  int (*import_state)(void* ctx, const uint8_t* buf, size_t len);

  // (선택) 종료 처리. 없으면 terminate 명령은 ACK만 하고 끝냄.
  int (*terminate)(void* ctx);
} mig_ops_t;

// =========================
// xApp-side: embedded UDS control server
// =========================
typedef struct mig_server mig_server_t;

// UDS 서버 시작 (내부적으로 스레드 1개 생성)
// sock_path: 예) "/tmp/xapp_A.sock"
mig_server_t* mig_server_start_uds(const char* sock_path, mig_ops_t ops, void* ctx);

// 서버 종료 및 자원 정리
void mig_server_stop(mig_server_t* srv);

// =========================
// controller-side: UDS client helpers
// =========================
int mig_client_ping_uds(const char* sock_path);
int mig_client_quiesce_uds(const char* sock_path);
int mig_client_resume_uds(const char* sock_path);
int mig_client_terminate_uds(const char* sock_path);

// EXPORT: 서버가 payload로 blob을 반환 (malloc). 호출자가 free 해야 함.
int mig_client_export_uds(const char* sock_path, uint8_t** out_buf, size_t* out_len);

// IMPORT: controller가 blob을 payload로 전송하고 ACK 받음.
int mig_client_import_uds(const char* sock_path, const uint8_t* buf, size_t len);

#ifdef __cplusplus
}
#endif

