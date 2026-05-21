// migration_action.c
#define _GNU_SOURCE
#include "migration_action.h"
#include "migration_function.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>

static long long now_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000LL + (long long)(ts.tv_nsec / 1000);
}

static void make_sock_path(char* out, size_t out_sz, const char* node_id) {
  // 예: /tmp/xapp_A.sock
  snprintf(out, out_sz, "/tmp/xapp_%s.sock", node_id);
}

// dst xApp 실행: NODE_ID를 dst로 주입한 뒤 execv
static pid_t spawn_xapp_with_node(const char* xapp_path, const char* node_id, const char* xapp_duration_ms)
{
  pid_t pid = fork();
  if (pid < 0) return -1;

  if (pid == 0) {
    setenv("NODE_ID", node_id, 1);

    // 🔥 dst가 바로 죽지 않게 명시
    if (xapp_duration_ms && xapp_duration_ms[0] != '\0')
      setenv("XAPP_DURATION", xapp_duration_ms, 1);
    else
      setenv("XAPP_DURATION", "600000", 1); // 기본 10분

    char* const argv[] = { (char*)xapp_path, NULL };
    execv(xapp_path, argv);
    _exit(127);
  }
  return pid;
}

static int wait_dst_ready(const char* dst_sock, int retries, int wait_us_each) {
  for (int i = 0; i < retries; ++i) {
    if (mig_client_ping_uds(dst_sock) == 0) return 0;
    usleep(wait_us_each);
  }
  return -1;
}

int run_stopcopy_migration_nodeid(const mig_action_cfg_t* cfg, mig_action_stats_t* out_stats) {
  if (!cfg || !cfg->src_node || !cfg->dst_node || !cfg->xapp_path) return -1;

  mig_action_stats_t st;
  memset(&st, 0, sizeof(st));

  const int retries = (cfg->wait_retries > 0) ? cfg->wait_retries : 300; // 3초(10ms*300)
  const int wait_us_each = (cfg->wait_us > 0) ? cfg->wait_us : 10 * 1000;

  char src_sock[256], dst_sock[256];
  make_sock_path(src_sock, sizeof(src_sock), cfg->src_node);
  make_sock_path(dst_sock, sizeof(dst_sock), cfg->dst_node);

  long long t0 = now_us();

  // 1) QUIESCE source(A)
  long long t1 = now_us();
  if (mig_client_quiesce_uds(src_sock) != 0) {
    fprintf(stderr, "ERR: QUIESCE failed (src_sock=%s)\n", src_sock);
    return -2;
  }
  long long t2 = now_us();
  st.quiesce_us = (t2 - t1);

  // 2) EXPORT from source(A)
  uint8_t* blob = NULL;
  size_t blob_len = 0;

  long long t3 = now_us();
  if (mig_client_export_uds(src_sock, &blob, &blob_len) != 0) {
    fprintf(stderr, "ERR: EXPORT failed (src_sock=%s)\n", src_sock);
    return -3;
  }
  long long t4 = now_us();
  st.export_us = (t4 - t3);
  st.state_bytes = (unsigned long long)blob_len;

  // 3) Spawn destination xApp with NODE_ID=B
  long long t5 = now_us();
  
  // spawn 전에 stale sock 제거 권장
  unlink(dst_sock);

  pid_t dst_pid = spawn_xapp_with_node(cfg->xapp_path, cfg->dst_node, cfg->xapp_duration_ms);
  if (dst_pid < 0) {
    fprintf(stderr, "ERR: spawn destination failed (xapp=%s, NODE_ID=%s)\n", cfg->xapp_path, cfg->dst_node);
    free(blob);
    return -4;
  }

  // 4) Wait destination UDS ready (/tmp/xapp_B.sock)
  if (wait_dst_ready(dst_sock, retries, wait_us_each) != 0) {
    fprintf(stderr, "ERR: destination not ready (dst_sock=%s)\n", dst_sock);
    free(blob);
    // 🔥 src를 다시 살려야 함
    (void)mig_client_resume_uds(src_sock);
  
    return -5;
  }
  long long t6 = now_us();
  st.spawn_wait_us = (t6 - t5);

  // 5) IMPORT into destination(B)
  long long t7 = now_us();
  if (mig_client_import_uds(dst_sock, blob, blob_len) != 0) {
    fprintf(stderr, "ERR: IMPORT failed (dst_sock=%s)\n", dst_sock);
    free(blob);
    return -6;
  }
  long long t8 = now_us();
  st.import_us = (t8 - t7);

  free(blob);

  // 6) RESUME destination(B)
  long long t9 = now_us();
  if (mig_client_resume_uds(dst_sock) != 0) {
    fprintf(stderr, "ERR: RESUME failed (dst_sock=%s)\n", dst_sock);
    return -7;
  }
  long long t10 = now_us();
  st.resume_us = (t10 - t9);

  // 7) (optional) TERMINATE source(A)
  if (cfg->terminate_src) {
    (void)mig_client_terminate_uds(src_sock);
  }

  st.total_us = (now_us() - t0);

  if (out_stats) *out_stats = st;
  (void)dst_pid;
  return 0;
}

// ------------------------------
// CLI wrapper (A->B 기본)
// ------------------------------
static void usage(const char* prog) {
  fprintf(stderr,
    "Usage:\n"
    "  %s --xapp <path_to_xapp_kpm_moni_mr> [--src A] [--dst B] [--terminate-src] [--retries N] [--wait-us U]\n\n"
    "Example:\n"
    "  export NODE_ID=A\n"
    "  XAPP_DURATION=0 ./build/examples/xApp/c/monitor/xapp_kpm_moni_mr   # source(A) 먼저 실행\n"
    "  ./build/examples/xApp/c/monitor/migration_action --xapp ./build/examples/xApp/c/monitor/xapp_kpm_moni_mr --src A --dst B --terminate-src\n\n",
    prog);
}

int main(int argc, char** argv) {
  mig_action_cfg_t cfg;
  memset(&cfg, 0, sizeof(cfg));

  cfg.src_node = "A";
  cfg.dst_node = "B";
  cfg.terminate_src = 0;
  cfg.wait_retries = 300;
  cfg.wait_us = 10 * 1000;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--xapp") == 0 && i + 1 < argc) {
      cfg.xapp_path = argv[++i];
    } else if (strcmp(argv[i], "--src") == 0 && i + 1 < argc) {
      cfg.src_node = argv[++i];
    } else if (strcmp(argv[i], "--dst") == 0 && i + 1 < argc) {
      cfg.dst_node = argv[++i];
    } else if (strcmp(argv[i], "--terminate-src") == 0) {
      cfg.terminate_src = 1;
    } else if (strcmp(argv[i], "--retries") == 0 && i + 1 < argc) {
      cfg.wait_retries = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--wait-us") == 0 && i + 1 < argc) {
      cfg.wait_us = atoi(argv[++i]);
    } else {
      usage(argv[0]);
      return 1;
    }
  }

  if (!cfg.xapp_path) {
    usage(argv[0]);
    return 1;
  }

  mig_action_stats_t st;
  int rc = run_stopcopy_migration_nodeid(&cfg, &st);
  if (rc != 0) {
    fprintf(stderr, "Migration failed rc=%d\n", rc);
    return 2;
  }

  printf("Migration OK (%s -> %s)\n", cfg.src_node, cfg.dst_node);
  printf("  state_bytes      = %llu\n", st.state_bytes);
  printf("  quiesce_us       = %lld\n", st.quiesce_us);
  printf("  export_us        = %lld\n", st.export_us);
  printf("  spawn_wait_us    = %lld\n", st.spawn_wait_us);
  printf("  import_us        = %lld\n", st.import_us);
  printf("  resume_us        = %lld\n", st.resume_us);
  printf("  total_us         = %lld\n", st.total_us);

  return 0;
}

