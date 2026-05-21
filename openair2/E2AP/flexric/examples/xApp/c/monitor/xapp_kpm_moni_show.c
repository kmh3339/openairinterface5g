/*
 * xapp_kpm_moni_show.c
 *
 * 목적:
 *  - Redis에 저장된 KPM 상태(현재는 UE 단위 키: sdl:kpm:... ) 중
 *    ts_us가 가장 큰(가장 최근) 상태를 찾아 HGETALL로 출력하고 종료.
 *
 * 빌드:
 *  - flexRIC xApp 빌드 시스템(CMake)에서 hiredis 링크 추가 필요
 *  - target_link_libraries(... hiredis)
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/util/e.h"

#include <hiredis/hiredis.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char* REDIS_HOST = "127.0.0.1";
static const int   REDIS_PORT = 6379;

static void die_msg(const char* msg)
{
  fprintf(stderr, "%s\n", msg);
  exit(EXIT_FAILURE);
}

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

static int ends_with(const char* s, const char* suffix)
{
  if (!s || !suffix) return 0;
  size_t sl = strlen(s);
  size_t su = strlen(suffix);
  if (su > sl) return 0;
  return (strcmp(s + (sl - su), suffix) == 0);
}

static long long parse_ll_safe(const char* s, int* ok)
{
  if (ok) *ok = 0;
  if (s == NULL) return 0;

  char* end = NULL;
  long long v = strtoll(s, &end, 10);
  if (end == s || *end != '\0') return 0;
  if (ok) *ok = 1;
  return v;
}

/*
 * Redis에서 pattern에 매칭되는 키 중,
 *  - ":ver" 등 제외 (여기서는 :ver만 제외)
 *  - hash 필드 ts_us를 읽어서 가장 큰 키를 선택
 *
 * return: malloc된 best_key (없으면 NULL). caller가 free
 */
static char* find_latest_kpm_key(redisContext* c, const char* pattern)
{
  if (c == NULL) return NULL;

  const int COUNT = 200; // 한번에 스캔할 키 개수 힌트
  const char* cursor = "0";

  char* best_key = NULL;
  long long best_ts = -1;

  while (1) {
    redisReply* r = (redisReply*)redisCommand(c, "SCAN %s MATCH %s COUNT %d", cursor, pattern, COUNT);
    if (r == NULL) {
      fprintf(stderr, "[Redis] SCAN returned NULL reply\n");
      break;
    }
    if (r->type != REDIS_REPLY_ARRAY || r->elements != 2) {
      fprintf(stderr, "[Redis] Unexpected SCAN reply type\n");
      freeReplyObject(r);
      break;
    }

    // r->element[0] = new cursor (string)
    // r->element[1] = array of keys
    redisReply* new_cursor = r->element[0];
    redisReply* keys = r->element[1];

    if (new_cursor->type != REDIS_REPLY_STRING || keys->type != REDIS_REPLY_ARRAY) {
      fprintf(stderr, "[Redis] Unexpected SCAN inner reply types\n");
      freeReplyObject(r);
      break;
    }

    cursor = new_cursor->str;

    for (size_t i = 0; i < keys->elements; i++) {
      redisReply* k = keys->element[i];
      if (k->type != REDIS_REPLY_STRING || k->str == NULL) continue;

      const char* key = k->str;

      // 버전 키 제외 (writer가 :ver를 따로 둠)
      if (ends_with(key, ":ver")) continue;

      // key 타입이 hash인지 체크(안전)
      redisReply* t = (redisReply*)redisCommand(c, "TYPE %s", key);
      if (t == NULL) continue;
      int is_hash = (t->type == REDIS_REPLY_STATUS && t->str && strcmp(t->str, "hash") == 0);
      freeReplyObject(t);
      if (!is_hash) continue;

      // ts_us 읽기
      redisReply* tsr = (redisReply*)redisCommand(c, "HGET %s ts_us", key);
      if (tsr == NULL) continue;

      if (tsr->type == REDIS_REPLY_STRING && tsr->str != NULL) {
        int ok = 0;
        long long ts = parse_ll_safe(tsr->str, &ok);
        if (ok && ts > best_ts) {
          best_ts = ts;

          free(best_key);
          best_key = strdup(key);
          if (best_key == NULL) {
            freeReplyObject(tsr);
            freeReplyObject(r);
            die_msg("Memory exhausted while strdup(best_key)");
          }
        }
      }
      freeReplyObject(tsr);
    }

    freeReplyObject(r);

    // cursor가 "0"이면 종료
    if (cursor != NULL && strcmp(cursor, "0") == 0) break;
  }

  return best_key;
}

static void print_hgetall(redisContext* c, const char* key)
{
  redisReply* r = (redisReply*)redisCommand(c, "HGETALL %s", key);
  if (r == NULL) {
    fprintf(stderr, "[Redis] HGETALL NULL reply\n");
    return;
  }
  if (r->type != REDIS_REPLY_ARRAY) {
    fprintf(stderr, "[Redis] HGETALL unexpected type=%d\n", r->type);
    freeReplyObject(r);
    return;
  }

  printf("==== HGETALL %s ====\n", key);
  for (size_t i = 0; i + 1 < r->elements; i += 2) {
    redisReply* f = r->element[i];
    redisReply* v = r->element[i + 1];
    const char* fs = (f && f->type == REDIS_REPLY_STRING) ? f->str : "(non-string)";
    const char* vs = (v && v->type == REDIS_REPLY_STRING) ? v->str : "(non-string)";
    printf("%s = %s\n", fs ? fs : "(null)", vs ? vs : "(null)");
  }
  printf("====================\n");

  freeReplyObject(r);
}

static void print_ver(redisContext* c, const char* key)
{
  char ver_key[1024];
  snprintf(ver_key, sizeof(ver_key), "%s:ver", key);

  redisReply* r = (redisReply*)redisCommand(c, "GET %s", ver_key);
  if (r == NULL) {
    fprintf(stderr, "[Redis] GET(ver) NULL reply\n");
    return;
  }

  if (r->type == REDIS_REPLY_STRING && r->str) {
    printf("VER (%s) = %s\n", ver_key, r->str);
  } else if (r->type == REDIS_REPLY_NIL) {
    printf("VER (%s) = (nil)\n", ver_key);
  } else if (r->type == REDIS_REPLY_INTEGER) {
    printf("VER (%s) = %lld\n", ver_key, (long long)r->integer);
  } else {
    printf("VER (%s) = (unexpected type=%d)\n", ver_key, r->type);
  }

  freeReplyObject(r);
}

int main(int argc, char* argv[])
{
  // flexRIC xApp 초기화(형식 유지). 실제로 E2 구독은 하지 않음.
  fr_args_t args = init_fr_args(argc, argv);
  init_xapp_api(&args);

  // Redis 연결
  redisContext* c = connect_redis();
  if (c == NULL) {
    die_msg("[Redis] Failed to connect. Is redis-server running?");
  }
  printf("[Redis] Connected to %s:%d\n", REDIS_HOST, REDIS_PORT);

  // 가장 최근 KPM 키 찾기
  // 현재 writer가 sdl:kpm:* 형태로 저장 중이므로 그 패턴 사용
  const char* pattern = "sdl:kpm:*";
  char* latest_key = find_latest_kpm_key(c, pattern);

  if (latest_key == NULL) {
    printf("[SHOW] No KPM key found with pattern '%s'\n", pattern);
    redisFree(c);

    // xApp 종료
    while (try_stop_xapp_api() == false) usleep(1000);
    return 0;
  }

  printf("[SHOW] Latest KPM key = %s\n", latest_key);

  // 내용 출력
  print_hgetall(c, latest_key);
  print_ver(c, latest_key);

  free(latest_key);
  redisFree(c);

  // xApp 종료
  while (try_stop_xapp_api() == false) usleep(1000);

  printf("[SHOW] Done. Exiting.\n");
  return 0;
}

