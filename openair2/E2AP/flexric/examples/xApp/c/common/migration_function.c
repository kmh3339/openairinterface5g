// migration_function.c
#define _GNU_SOURCE
#include "migration_function.h"

#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/un.h>

// =========================
// Low-level I/O helpers
// =========================

static int write_full(int fd, const void* buf, size_t len) {
  const uint8_t* p = (const uint8_t*)buf;
  size_t off = 0;
  while (off < len) {
    ssize_t n = write(fd, p + off, len - off);
    if (n < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)n;
  }
  return 0;
}

static int read_full(int fd, void* buf, size_t len) {
  uint8_t* p = (uint8_t*)buf;
  size_t off = 0;
  while (off < len) {
    ssize_t n = read(fd, p + off, len - off);
    if (n == 0) return -1; // EOF
    if (n < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)n;
  }
  return 0;
}

// =========================
// UDS utilities
// =========================

static int uds_connect(const char* sock_path) {
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return -1;

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

  if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

static int uds_listen(const char* sock_path) {
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return -1;

  // 기존 소켓 파일 제거
  unlink(sock_path);

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }
  if (listen(fd, 16) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

// =========================
// Protocol helpers
// =========================

static int send_req(int fd, mig_cmd_e cmd, const uint8_t* payload, uint32_t payload_len) {
  mig_msg_hdr_t h;
  memset(&h, 0, sizeof(h));
  h.magic = MIG_MAGIC;
  h.cmd = (uint16_t)cmd;
  h.rc = 0; // request
  h.payload_len = payload_len;

  if (write_full(fd, &h, sizeof(h)) < 0) return -1;
  if (payload_len > 0 && payload) {
    if (write_full(fd, payload, payload_len) < 0) return -1;
  }
  return 0;
}

static int recv_reply(int fd,
                      mig_cmd_e expect_cmd,
                      uint8_t** out_payload,
                      uint32_t* out_len,
                      mig_rc_e* out_rc) {
  mig_msg_hdr_t h;
  if (read_full(fd, &h, sizeof(h)) < 0) return -1;
  if (h.magic != MIG_MAGIC) return -1;
  if ((mig_cmd_e)h.cmd != expect_cmd) return -1;

  uint8_t* payload = NULL;
  if (h.payload_len > 0) {
    payload = (uint8_t*)malloc(h.payload_len);
    if (!payload) return -1;
    if (read_full(fd, payload, h.payload_len) < 0) {
      free(payload);
      return -1;
    }
  }

  if (out_payload) *out_payload = payload;
  else free(payload);

  if (out_len) *out_len = h.payload_len;
  if (out_rc) *out_rc = (mig_rc_e)h.rc;

  return 0;
}

static void send_reply(int cfd, mig_cmd_e cmd, mig_rc_e rc, const uint8_t* payload, uint32_t len) {
  mig_msg_hdr_t h;
  memset(&h, 0, sizeof(h));
  h.magic = MIG_MAGIC;
  h.cmd = (uint16_t)cmd;
  h.rc = (uint16_t)rc;
  h.payload_len = len;

  (void)write_full(cfd, &h, sizeof(h));
  if (len > 0 && payload) {
    (void)write_full(cfd, payload, len);
  }
}

// =========================
// Server implementation
// =========================

struct mig_server {
  int listen_fd;
  char* sock_path;
  mig_ops_t ops;
  void* ctx;
  pthread_t th;
  int stop_flag;
};

static void handle_one_client(struct mig_server* s, int cfd) {
  mig_msg_hdr_t req;
  if (read_full(cfd, &req, sizeof(req)) < 0) return;
  if (req.magic != MIG_MAGIC) return;

  if (req.payload_len > MIG_MAX_PAYLOAD) {
    // drain? (단순화: 바로 에러 응답)
    send_reply(cfd, (mig_cmd_e)req.cmd, MIG_ERR_TOO_LARGE, NULL, 0);
    return;
  }

  uint8_t* payload = NULL;
  if (req.payload_len > 0) {
    payload = (uint8_t*)malloc(req.payload_len);
    if (!payload) {
      send_reply(cfd, (mig_cmd_e)req.cmd, MIG_ERR_NO_MEM, NULL, 0);
      return;
    }
    if (read_full(cfd, payload, req.payload_len) < 0) {
      free(payload);
      return;
    }
  }

  mig_cmd_e cmd = (mig_cmd_e)req.cmd;
  mig_rc_e rc = MIG_OK;

  switch (cmd) {
    case MIG_CMD_PING:
      send_reply(cfd, cmd, MIG_OK, NULL, 0);
      break;

    case MIG_CMD_QUIESCE:
      if (s->ops.quiesce) {
        rc = (s->ops.quiesce(s->ctx) == 0) ? MIG_OK : MIG_ERR;
      } else {
        rc = MIG_ERR_BAD_STATE;
      }
      send_reply(cfd, cmd, rc, NULL, 0);
      break;

    case MIG_CMD_EXPORT: {
      if (!s->ops.export_state) {
        send_reply(cfd, cmd, MIG_ERR_BAD_STATE, NULL, 0);
        break;
      }
      uint8_t* out = NULL;
      size_t out_len = 0;
      int r = s->ops.export_state(s->ctx, &out, &out_len);
      if (r != 0 || (out_len > 0 && out == NULL)) {
        send_reply(cfd, cmd, MIG_ERR, NULL, 0);
      } else if (out_len > MIG_MAX_PAYLOAD) {
        send_reply(cfd, cmd, MIG_ERR_TOO_LARGE, NULL, 0);
      } else {
        send_reply(cfd, cmd, MIG_OK, out, (uint32_t)out_len);
      }
      free(out);
      break;
    }

    case MIG_CMD_IMPORT:
      if (s->ops.import_state) {
        rc = (s->ops.import_state(s->ctx, payload, (size_t)req.payload_len) == 0) ? MIG_OK : MIG_ERR;
      } else {
        rc = MIG_ERR_BAD_STATE;
      }
      send_reply(cfd, cmd, rc, NULL, 0);
      break;

    case MIG_CMD_RESUME:
      if (s->ops.resume) {
        rc = (s->ops.resume(s->ctx) == 0) ? MIG_OK : MIG_ERR;
      } else {
        rc = MIG_ERR_BAD_STATE;
      }
      send_reply(cfd, cmd, rc, NULL, 0);
      break;

    case MIG_CMD_TERMINATE:
      if (s->ops.terminate) {
        (void)s->ops.terminate(s->ctx);
      }
      // terminate 훅이 exit() 하지 않는다면 ACK만 보냄
      send_reply(cfd, cmd, MIG_OK, NULL, 0);
      break;

    default:
      send_reply(cfd, cmd, MIG_ERR_BAD_CMD, NULL, 0);
      break;
  }

  free(payload);
}

static void* server_thread_main(void* arg) {
  struct mig_server* s = (struct mig_server*)arg;

  while (!s->stop_flag) {
    int cfd = accept(s->listen_fd, NULL, NULL);
    if (cfd < 0) {
      if (errno == EINTR) continue;
      if (s->stop_flag) break;
      continue;
    }
    handle_one_client(s, cfd);
    close(cfd);
  }
  return NULL;
}

mig_server_t* mig_server_start_uds(const char* sock_path, mig_ops_t ops, void* ctx) {
  if (!sock_path) return NULL;

  int fd = uds_listen(sock_path);
  if (fd < 0) return NULL;

  struct mig_server* s = (struct mig_server*)calloc(1, sizeof(*s));
  if (!s) {
    close(fd);
    return NULL;
  }

  s->listen_fd = fd;
  s->sock_path = strdup(sock_path);
  s->ops = ops;
  s->ctx = ctx;
  s->stop_flag = 0;

  if (!s->sock_path) {
    close(fd);
    free(s);
    return NULL;
  }

  if (pthread_create(&s->th, NULL, server_thread_main, s) != 0) {
    close(fd);
    unlink(sock_path);
    free(s->sock_path);
    free(s);
    return NULL;
  }

  return (mig_server_t*)s;
}

void mig_server_stop(mig_server_t* srv) {
  if (!srv) return;
  struct mig_server* s = (struct mig_server*)srv;

  s->stop_flag = 1;

  // accept() unblock을 위해 shutdown/close
  shutdown(s->listen_fd, SHUT_RDWR);
  close(s->listen_fd);

  pthread_join(s->th, NULL);

  if (s->sock_path) {
    unlink(s->sock_path);
    free(s->sock_path);
  }
  free(s);
}

// =========================
// Client helpers
// =========================

static int client_simple_cmd(const char* sock_path, mig_cmd_e cmd) {
  int fd = uds_connect(sock_path);
  if (fd < 0) return -1;

  if (send_req(fd, cmd, NULL, 0) < 0) {
    close(fd);
    return -1;
  }

  mig_rc_e rc;
  if (recv_reply(fd, cmd, NULL, NULL, &rc) < 0) {
    close(fd);
    return -1;
  }

  close(fd);
  return (rc == MIG_OK) ? 0 : -1;
}

int mig_client_ping_uds(const char* sock_path)      { return client_simple_cmd(sock_path, MIG_CMD_PING); }
int mig_client_quiesce_uds(const char* sock_path)   { return client_simple_cmd(sock_path, MIG_CMD_QUIESCE); }
int mig_client_resume_uds(const char* sock_path)    { return client_simple_cmd(sock_path, MIG_CMD_RESUME); }
int mig_client_terminate_uds(const char* sock_path) { return client_simple_cmd(sock_path, MIG_CMD_TERMINATE); }

int mig_client_export_uds(const char* sock_path, uint8_t** out_buf, size_t* out_len) {
  if (!out_buf || !out_len) return -1;
  *out_buf = NULL;
  *out_len = 0;

  int fd = uds_connect(sock_path);
  if (fd < 0) return -1;

  if (send_req(fd, MIG_CMD_EXPORT, NULL, 0) < 0) {
    close(fd);
    return -1;
  }

  uint8_t* payload = NULL;
  uint32_t plen = 0;
  mig_rc_e rc;

  if (recv_reply(fd, MIG_CMD_EXPORT, &payload, &plen, &rc) < 0) {
    close(fd);
    return -1;
  }
  close(fd);

  if (rc != MIG_OK) {
    free(payload);
    return -1;
  }

  *out_buf = payload;
  *out_len = (size_t)plen;
  return 0;
}

int mig_client_import_uds(const char* sock_path, const uint8_t* buf, size_t len) {
  if (len > MIG_MAX_PAYLOAD) return -1;
  if (len > UINT32_MAX) return -1; // 헤더 payload_len이 uint32_t

  int fd = uds_connect(sock_path);
  if (fd < 0) return -1;

  if (send_req(fd, MIG_CMD_IMPORT, buf, (uint32_t)len) < 0) {
    close(fd);
    return -1;
  }

  mig_rc_e rc;
  if (recv_reply(fd, MIG_CMD_IMPORT, NULL, NULL, &rc) < 0) {
    close(fd);
    return -1;
  }
  close(fd);

  return (rc == MIG_OK) ? 0 : -1;
}

