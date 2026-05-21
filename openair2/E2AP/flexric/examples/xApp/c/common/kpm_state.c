// kpm_state.c
#include "kpm_state.h"

#include <stdlib.h>
#include <string.h>
#include <signal.h>

#define KPM_STATE_MAGIC 0x4B504D31u  // 'KPM1'
#define KPM_STATE_VER   1u

typedef struct __attribute__((packed)) {
  uint32_t magic;       // KPM_STATE_MAGIC
  uint16_t version;     // KPM_STATE_VER
  uint16_t reserved;
  uint64_t counter;
  uint32_t report_len;  // last_report bytes (NUL 포함해서 저장)
  uint32_t reserved2;
  // followed by report bytes[report_len]
} kpm_state_blob_hdr_v1_t;

int kpm_state_init(kpm_state_ctx_t* ctx)
{
  if (!ctx) return -1;
  memset(ctx, 0, sizeof(*ctx));

  if (pthread_mutex_init(&ctx->mtx, NULL) != 0) return -1;
  if (pthread_cond_init(&ctx->cv, NULL) != 0) {
    pthread_mutex_destroy(&ctx->mtx);
    return -1;
  }

  ctx->paused   = 0;
  ctx->inflight = 0;

  ctx->counter   = 1;
  ctx->migrating = false;   // 🔥 필수: 초기화 안 하면 쓰레기값으로 gate 걸림

  ctx->last_report = NULL;
  ctx->last_report_len = 0;
  return 0;
}

void kpm_state_destroy(kpm_state_ctx_t* ctx)
{
  if (!ctx) return;
  free(ctx->last_report);
  ctx->last_report = NULL;
  ctx->last_report_len = 0;

  pthread_cond_destroy(&ctx->cv);
  pthread_mutex_destroy(&ctx->mtx);
}

int kpm_cb_enter(kpm_state_ctx_t* ctx)
{
  if (!ctx) return 1;

  pthread_mutex_lock(&ctx->mtx);
  if (ctx->paused) {
    pthread_mutex_unlock(&ctx->mtx);
    return 1; // drop
  }
  ctx->inflight++;
  pthread_mutex_unlock(&ctx->mtx);
  return 0;
}

void kpm_cb_exit(kpm_state_ctx_t* ctx)
{
  if (!ctx) return;

  pthread_mutex_lock(&ctx->mtx);
  if (ctx->inflight > 0) ctx->inflight--;
  if (ctx->paused && ctx->inflight == 0) {
    pthread_cond_broadcast(&ctx->cv);
  }
  pthread_mutex_unlock(&ctx->mtx);
}

int kpm_update_last_report(kpm_state_ctx_t* ctx, const char* buf, size_t len)
{
  if (!ctx) return -1;

  pthread_mutex_lock(&ctx->mtx);

  free(ctx->last_report);
  ctx->last_report = NULL;
  ctx->last_report_len = 0;

  if (buf && len > 0) {
    char* p = (char*)malloc(len);
    if (!p) {
      pthread_mutex_unlock(&ctx->mtx);
      return -1;
    }
    memcpy(p, buf, len);
    ctx->last_report = p;
    ctx->last_report_len = len;
  }

  pthread_mutex_unlock(&ctx->mtx);
  return 0;
}

// =========================
// migration hooks (ops)
// =========================

static int kpm_quiesce(void* vctx)
{
  kpm_state_ctx_t* ctx = (kpm_state_ctx_t*)vctx;
  if (!ctx) return -1;

  pthread_mutex_lock(&ctx->mtx);

  // 🔥 migration 시작: xApp callback이 "migration gate"를 타도록
  ctx->migrating = true;

  // callback drop + inflight drain
  ctx->paused = 1;
  while (ctx->inflight != 0) {
    pthread_cond_wait(&ctx->cv, &ctx->mtx);
  }

  pthread_mutex_unlock(&ctx->mtx);
  return 0;
}

static int kpm_resume(void* vctx)
{
  kpm_state_ctx_t* ctx = (kpm_state_ctx_t*)vctx;
  if (!ctx) return -1;

  pthread_mutex_lock(&ctx->mtx);

  // 정상 수집 재개
  ctx->paused = 0;

  // 🔥 migration 종료: 다시 일반 동작 모드로
  ctx->migrating = false;

  pthread_mutex_unlock(&ctx->mtx);
  return 0;
}

static int kpm_export_state(void* vctx, uint8_t** out_buf, size_t* out_len)
{
  kpm_state_ctx_t* ctx = (kpm_state_ctx_t*)vctx;
  if (!ctx || !out_buf || !out_len) return -1;

  *out_buf = NULL;
  *out_len = 0;

  pthread_mutex_lock(&ctx->mtx);

  const uint32_t rlen = (ctx->last_report_len > 0 && ctx->last_report)
                        ? (uint32_t)ctx->last_report_len
                        : 0u;

  const size_t total = sizeof(kpm_state_blob_hdr_v1_t) + (size_t)rlen;

  kpm_state_blob_hdr_v1_t hdr;
  hdr.magic = KPM_STATE_MAGIC;
  hdr.version = KPM_STATE_VER;
  hdr.reserved = 0;
  hdr.counter = ctx->counter;
  hdr.report_len = rlen;
  hdr.reserved2 = 0;

  uint8_t* buf = (uint8_t*)malloc(total);
  if (!buf) {
    pthread_mutex_unlock(&ctx->mtx);
    return -1;
  }

  memcpy(buf, &hdr, sizeof(hdr));
  if (rlen > 0) {
    memcpy(buf + sizeof(hdr), ctx->last_report, rlen);
  }

  pthread_mutex_unlock(&ctx->mtx);

  *out_buf = buf;
  *out_len = total;
  return 0;
}

static int kpm_import_state(void* vctx, const uint8_t* in, size_t in_len)
{
  kpm_state_ctx_t* ctx = (kpm_state_ctx_t*)vctx;
  if (!ctx || !in) return -1;
  if (in_len < sizeof(kpm_state_blob_hdr_v1_t)) return -1;

  kpm_state_blob_hdr_v1_t hdr;
  memcpy(&hdr, in, sizeof(hdr));

  if (hdr.magic != KPM_STATE_MAGIC) return -1;
  if (hdr.version != KPM_STATE_VER) return -1;

  const size_t expected = sizeof(kpm_state_blob_hdr_v1_t) + (size_t)hdr.report_len;
  if (expected != in_len) return -1;

  const char* report_ptr = (const char*)(in + sizeof(kpm_state_blob_hdr_v1_t));
  const size_t report_len = (size_t)hdr.report_len;

  pthread_mutex_lock(&ctx->mtx);

  ctx->counter = hdr.counter;

  free(ctx->last_report);
  ctx->last_report = NULL;
  ctx->last_report_len = 0;

  if (report_len > 0) {
    char* p = (char*)malloc(report_len);
    if (!p) {
      pthread_mutex_unlock(&ctx->mtx);
      return -1;
    }
    memcpy(p, report_ptr, report_len);
    ctx->last_report = p;
    ctx->last_report_len = report_len;
  }

  pthread_mutex_unlock(&ctx->mtx);
  return 0;
}

static int kpm_terminate(void* vctx)
{
  (void)vctx;
  raise(SIGTERM);
  return 0;
}

mig_ops_t kpm_get_mig_ops(void)
{
  mig_ops_t ops;
  memset(&ops, 0, sizeof(ops));
  ops.quiesce      = kpm_quiesce;
  ops.resume       = kpm_resume;
  ops.export_state = kpm_export_state;
  ops.import_state = kpm_import_state;
  ops.terminate    = kpm_terminate; // 원치 않으면 NULL
  return ops;
}

