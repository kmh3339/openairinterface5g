/*
 * RC xApp for STYLE 2 / ACTION 6
 * - Collect UE IDs from KPM
 * - Assign S-NSSAI by slice-specific KPM subscription
 * - Send RC CONTROL: Slice-level PRB quota
 * - Closed-loop target is applied to MAX PRB, not dedicated PRB
 *
 * Assumptions:
 * 1) RC RAN Function ID = 3
 * 2) KPM RAN Function ID = 2
 * 3) RC agent side already supports:
 *    - CONTROL Style 2: Radio Resource Allocation Control
 *    - Action 6: Slice-level PRB quota
 * 4) KPM REPORT style 4 is available
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_struct.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_list.h"
#include "../../../../src/util/time_now_us.h"
#include "../../../../src/util/alg_ds/ds/lock_guard/lock_guard.h"
#include "../../../../src/util/e.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <assert.h>

static uint64_t const period_ms = 100;
static pthread_mutex_t mtx;

#define NUM_SLICE_PROFILES 3
#define LOG_PREFIX "[baseline-slice-xapp]"
#define CONTROL_PERIOD_MS 1000
#define CONTROL_LOOP_PERIOD_MS CONTROL_PERIOD_MS
#define CONTROL_COOLDOWN_MS 2000
#define HYSTERESIS_PRB_DELTA 3
#define CONTROL_HYSTERESIS_PRB HYSTERESIS_PRB_DELTA
#define KPI_STALE_TIMEOUT_MS 3000
#define MS_TO_US(ms) ((int64_t)(ms) * 1000)
#define CONTROL_LOOP_PERIOD_US MS_TO_US(CONTROL_LOOP_PERIOD_MS)
#define CONTROL_COOLDOWN_US MS_TO_US(CONTROL_COOLDOWN_MS)
#define KPI_STALE_TIMEOUT_US MS_TO_US(KPI_STALE_TIMEOUT_MS)
#define SMOOTHING_ALPHA 0.3
#define KPI_EMA_ALPHA SMOOTHING_ALPHA
#define TOTAL_PRB_BUDGET_RATIO 100
#define DEFAULT_MIN_PRB_RATIO 10
#define DEFAULT_MAX_PRB_RATIO 80
#define DEMAND_EPSILON 0.001
#define DL_THP_NORMALIZER_KBPS 50000.0
#define PRB_USAGE_NORMALIZER_PCT 100.0
/* Verification mode: force RC max/dedicated PRB ratio to make MAC enforcement visible. Set 0 to disable. */
#define VERIFY_FORCE_PRB_MAX_RATIO 0

/* --------------------------------------------------------------- */
/* Slice profile                                                   */
/* --------------------------------------------------------------- */

typedef struct {
  char const* profile_name;
  char const* db_ue_desc;      // for log only
  uint8_t sst;
  uint8_t sd[3];               // 24-bit SD
  uint16_t min_prb;
  uint16_t max_prb;
  uint16_t dedicated_prb;
  uint8_t plmn[3];             // 3-byte PLMN identity (00101 -> 00 F1 10)
} slice_profile_t;

/*
 * Align these with your actual DB / RAN configuration.
 *
 * Example:
 * - UE1 -> SST=1, SD=000003
 * - UE3 -> SST=2, SD=000001
 * - UE4 -> SST=3, SD=000002
 */
static slice_profile_t const g_profile_sst1 = {
  .profile_name = "slice-sst1",
  .db_ue_desc = "001010000000001",
  .sst = 1,
  .sd = {0x00, 0x00, 0x03},
  .min_prb = DEFAULT_MIN_PRB_RATIO,
  .max_prb = DEFAULT_MAX_PRB_RATIO,
  .dedicated_prb = 34,
  .plmn = {0x00, 0xF1, 0x10},   // PLMN 00101
};

static slice_profile_t const g_profile_sst2 = {
  .profile_name = "slice-sst2",
  .db_ue_desc = "001010000000003",
  .sst = 2,
  .sd = {0x00, 0x00, 0x01},
  .min_prb = DEFAULT_MIN_PRB_RATIO,
  .max_prb = DEFAULT_MAX_PRB_RATIO,
  .dedicated_prb = 33,
  .plmn = {0x00, 0xF1, 0x10},   // PLMN 00101
};

static slice_profile_t const g_profile_sst3 = {
  .profile_name = "slice-sst3",
  .db_ue_desc = "001010000000004",
  .sst = 3,
  .sd = {0x00, 0x00, 0x02},
  .min_prb = DEFAULT_MIN_PRB_RATIO,
  .max_prb = DEFAULT_MAX_PRB_RATIO,
  .dedicated_prb = 33,
  .plmn = {0x00, 0xF1, 0x10},   // PLMN 00101
};

/* --------------------------------------------------------------- */
/* Closed-loop KPI / PRB state                                     */
/* --------------------------------------------------------------- */

typedef struct {
  bool has_pdcp_dl_kb;
  bool has_pdcp_ul_kb;
  bool has_thp_dl_kbps;
  bool has_thp_ul_kbps;
  bool has_prb_dl_pct;
  bool has_prb_ul_pct;
  uint32_t pdcp_dl_kb;
  uint32_t pdcp_ul_kb;
  double thp_dl_kbps;
  double thp_ul_kbps;
  double prb_dl_pct;
  double prb_ul_pct;
} kpi_sample_t;

typedef struct {
  bool valid;
  bool has_ctrl;
  bool demand_initialized;
  bool has_pdcp_dl_kb;
  bool has_pdcp_ul_kb;
  bool has_thp_dl_kbps;
  bool has_thp_ul_kbps;
  bool has_prb_dl_pct;
  bool has_prb_ul_pct;
  slice_profile_t const* profile;
  uint64_t last_ue_key;
  int64_t last_kpm_us;
  uint64_t sample_count;
  uint32_t pdcp_dl_kb;
  uint32_t pdcp_ul_kb;
  double thp_dl_kbps;
  double thp_ul_kbps;
  double prb_dl_pct;
  double prb_ul_pct;
  double ema_thp_dl_kbps;
  double ema_thp_ul_kbps;
  double ema_prb_dl_pct;
  double ema_prb_ul_pct;
  double last_raw_demand;
  double smoothed_demand;
  uint16_t last_target_prb;
  uint16_t last_sent_max_prb;
  int64_t last_ctrl_us;
} slice_state_t;

static slice_state_t g_slice_states[NUM_SLICE_PROFILES] = {
  {.profile = &g_profile_sst1},
  {.profile = &g_profile_sst2},
  {.profile = &g_profile_sst3},
};

static atomic_bool g_control_stop = false;

/* --------------------------------------------------------------- */
/* Observed UE table                                               */
/* --------------------------------------------------------------- */

#define MAX_OBSERVED_UE 64

typedef struct {
  bool valid;
  ue_id_e2sm_t ue_id;
  uint64_t key;
  slice_profile_t profile;
} observed_ue_t;

static observed_ue_t g_observed_ues[MAX_OBSERVED_UE] = {0};

/* --------------------------------------------------------------- */
/* Closed-loop helpers                                             */
/* --------------------------------------------------------------- */

static
bool same_snssai(slice_profile_t const* a, slice_profile_t const* b)
{
  assert(a != NULL);
  assert(b != NULL);

  return a->sst == b->sst &&
         memcmp(a->sd, b->sd, sizeof(a->sd)) == 0;
}

static
int find_slice_state_idx(slice_profile_t const* profile)
{
  assert(profile != NULL);

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (same_snssai(g_slice_states[i].profile, profile))
      return i;
  }

  return -1;
}

static
double clamp_double(double v, double lo, double hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static
uint16_t clamp_prb_ratio(uint16_t v, slice_profile_t const* profile)
{
  assert(profile != NULL);

  uint16_t const min_prb = profile->min_prb <= profile->max_prb ?
      profile->min_prb : profile->max_prb;
  uint16_t const max_prb = profile->min_prb <= profile->max_prb ?
      profile->max_prb : profile->min_prb;

  if (v < min_prb)
    return min_prb;
  if (v > max_prb)
    return max_prb;
  return v;
}

static
uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
  return a > b ? a - b : b - a;
}

static
uint16_t profile_effective_min(slice_profile_t const* profile)
{
  assert(profile != NULL);
  return profile->min_prb <= profile->max_prb ? profile->min_prb : profile->max_prb;
}

static
uint16_t profile_effective_max(slice_profile_t const* profile)
{
  assert(profile != NULL);
  return profile->min_prb <= profile->max_prb ? profile->max_prb : profile->min_prb;
}

static
uint32_t slice_sd_u32(slice_profile_t const* profile)
{
  assert(profile != NULL);
  return ((uint32_t)profile->sd[0] << 16) |
         ((uint32_t)profile->sd[1] << 8) |
         (uint32_t)profile->sd[2];
}

static
void add_sample_to_aggregate(kpi_sample_t* agg, kpi_sample_t const* sample)
{
  assert(agg != NULL);
  assert(sample != NULL);

  if (sample->has_pdcp_dl_kb) {
    agg->has_pdcp_dl_kb = true;
    agg->pdcp_dl_kb += sample->pdcp_dl_kb;
  }

  if (sample->has_pdcp_ul_kb) {
    agg->has_pdcp_ul_kb = true;
    agg->pdcp_ul_kb += sample->pdcp_ul_kb;
  }

  if (sample->has_thp_dl_kbps) {
    agg->has_thp_dl_kbps = true;
    agg->thp_dl_kbps += sample->thp_dl_kbps;
  }

  if (sample->has_thp_ul_kbps) {
    agg->has_thp_ul_kbps = true;
    agg->thp_ul_kbps += sample->thp_ul_kbps;
  }

  if (sample->has_prb_dl_pct) {
    agg->has_prb_dl_pct = true;
    agg->prb_dl_pct += sample->prb_dl_pct;
    agg->prb_dl_pct = clamp_double(agg->prb_dl_pct, 0.0, 100.0);
  }

  if (sample->has_prb_ul_pct) {
    agg->has_prb_ul_pct = true;
    agg->prb_ul_pct += sample->prb_ul_pct;
    agg->prb_ul_pct = clamp_double(agg->prb_ul_pct, 0.0, 100.0);
  }
}

static
void update_sample_from_record(kpi_sample_t* sample,
                               meas_type_t meas_type,
                               meas_record_lst_t meas_record)
{
  assert(sample != NULL);

  if (meas_type.type != NAME_MEAS_TYPE)
    return;

  if (meas_record.value == INTEGER_MEAS_VALUE) {
    if (cmp_str_ba("DRB.PdcpSduVolumeDL", meas_type.name) == 0) {
      sample->has_pdcp_dl_kb = true;
      sample->pdcp_dl_kb = meas_record.int_val;
    } else if (cmp_str_ba("DRB.PdcpSduVolumeUL", meas_type.name) == 0) {
      sample->has_pdcp_ul_kb = true;
      sample->pdcp_ul_kb = meas_record.int_val;
    } else if (cmp_str_ba("RRU.PrbTotDl", meas_type.name) == 0) {
      sample->has_prb_dl_pct = true;
      sample->prb_dl_pct = clamp_double((double)meas_record.int_val, 0.0, 100.0);
    } else if (cmp_str_ba("RRU.PrbTotUl", meas_type.name) == 0) {
      sample->has_prb_ul_pct = true;
      sample->prb_ul_pct = clamp_double((double)meas_record.int_val, 0.0, 100.0);
    }
  } else if (meas_record.value == REAL_MEAS_VALUE) {
    if (cmp_str_ba("DRB.UEThpDl", meas_type.name) == 0) {
      sample->has_thp_dl_kbps = true;
      sample->thp_dl_kbps = meas_record.real_val;
    } else if (cmp_str_ba("DRB.UEThpUl", meas_type.name) == 0) {
      sample->has_thp_ul_kbps = true;
      sample->thp_ul_kbps = meas_record.real_val;
    }
  }
}

static
void update_slice_kpi_state(slice_profile_t const* profile,
                            uint64_t last_ue_key,
                            int64_t now_us,
                            kpi_sample_t const* sample)
{
  assert(profile != NULL);
  assert(sample != NULL);

  int const idx = find_slice_state_idx(profile);
  assert(idx >= 0);

  slice_state_t* st = &g_slice_states[idx];
  st->valid = true;
  st->last_ue_key = last_ue_key;
  st->last_kpm_us = now_us;
  st->sample_count++;

  if (sample->has_pdcp_dl_kb) {
    st->has_pdcp_dl_kb = true;
    st->pdcp_dl_kb = sample->pdcp_dl_kb;
  }
  if (sample->has_pdcp_ul_kb) {
    st->has_pdcp_ul_kb = true;
    st->pdcp_ul_kb = sample->pdcp_ul_kb;
  }
  if (sample->has_thp_dl_kbps) {
    st->has_thp_dl_kbps = true;
    st->thp_dl_kbps = sample->thp_dl_kbps;
  }
  if (sample->has_thp_ul_kbps) {
    st->has_thp_ul_kbps = true;
    st->thp_ul_kbps = sample->thp_ul_kbps;
  }
  if (sample->has_prb_dl_pct) {
    st->has_prb_dl_pct = true;
    st->prb_dl_pct = sample->prb_dl_pct;
  }
  if (sample->has_prb_ul_pct) {
    st->has_prb_ul_pct = true;
    st->prb_ul_pct = sample->prb_ul_pct;
  }

  if (st->sample_count == 1) {
    st->ema_thp_dl_kbps = st->thp_dl_kbps;
    st->ema_thp_ul_kbps = st->thp_ul_kbps;
    st->ema_prb_dl_pct = st->prb_dl_pct;
    st->ema_prb_ul_pct = st->prb_ul_pct;
  } else {
    st->ema_thp_dl_kbps =
        KPI_EMA_ALPHA * st->thp_dl_kbps + (1.0 - KPI_EMA_ALPHA) * st->ema_thp_dl_kbps;
    st->ema_thp_ul_kbps =
        KPI_EMA_ALPHA * st->thp_ul_kbps + (1.0 - KPI_EMA_ALPHA) * st->ema_thp_ul_kbps;
    st->ema_prb_dl_pct =
        KPI_EMA_ALPHA * st->prb_dl_pct + (1.0 - KPI_EMA_ALPHA) * st->ema_prb_dl_pct;
    st->ema_prb_ul_pct =
        KPI_EMA_ALPHA * st->prb_ul_pct + (1.0 - KPI_EMA_ALPHA) * st->ema_prb_ul_pct;
  }

  printf("[RIC-XAPP][KPI] update profile=%s ue_key=%" PRIu64
         " samples=%" PRIu64 " pdcp_dl=%u[kb] pdcp_ul=%u[kb]"
         " thp_dl=%.2f[kbps] thp_ul=%.2f[kbps]"
         " prb_dl=%.2f[%%] prb_ul=%.2f[%%]"
         " ema_thp_dl=%.2f ema_prb_dl=%.2f\n",
         profile->profile_name,
         last_ue_key,
         st->sample_count,
         st->pdcp_dl_kb,
         st->pdcp_ul_kb,
         st->thp_dl_kbps,
         st->thp_ul_kbps,
         st->prb_dl_pct,
         st->prb_ul_pct,
         st->ema_thp_dl_kbps,
         st->ema_prb_dl_pct);

  printf(LOG_PREFIX " parsed KPI: slice=%s sst=%u sd=%02x%02x%02x"
         " subscription_context=S-NSSAI"
         " raw_tput_dl=%.2f raw_tput_ul=%.2f raw_prb_dl=%.2f raw_prb_ul=%.2f"
         " active_ue_key=%" PRIu64 " sample_count=%" PRIu64 "\n",
         profile->profile_name,
         profile->sst,
         profile->sd[0],
         profile->sd[1],
         profile->sd[2],
         st->thp_dl_kbps,
         st->thp_ul_kbps,
         st->prb_dl_pct,
         st->prb_ul_pct,
         last_ue_key,
         st->sample_count);
}

/* --------------------------------------------------------------- */
/* Logging helpers                                                 */
/* --------------------------------------------------------------- */

static
void log_rc_ran_param_value(seq_ran_param_t const* p, char const* prefix)
{
  assert(p != NULL);
  assert(prefix != NULL);

  switch (p->ran_param_val.type) {
    case ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.flag_true != NULL) {
        printf("%s value.type = FLAG_TRUE\n", prefix);
        if (p->ran_param_val.flag_true->type == INTEGER_RAN_PARAMETER_VALUE) {
          printf("%s value.int = %" PRId64 "\n", prefix, p->ran_param_val.flag_true->int_ran);
        } else if (p->ran_param_val.flag_true->type == OCTET_STRING_RAN_PARAMETER_VALUE) {
          printf("%s value.octet.len = %zu\n", prefix, p->ran_param_val.flag_true->octet_str_ran.len);
        } else {
          printf("%s value.non-int type = %d\n", prefix, p->ran_param_val.flag_true->type);
        }
      }
      break;

    case ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.flag_false != NULL) {
        printf("%s value.type = FLAG_FALSE\n", prefix);
        if (p->ran_param_val.flag_false->type == INTEGER_RAN_PARAMETER_VALUE) {
          printf("%s value.int = %" PRId64 "\n", prefix, p->ran_param_val.flag_false->int_ran);
        } else if (p->ran_param_val.flag_false->type == OCTET_STRING_RAN_PARAMETER_VALUE) {
          printf("%s value.octet.len = %zu\n", prefix, p->ran_param_val.flag_false->octet_str_ran.len);
        } else {
          printf("%s value.non-int type = %d\n", prefix, p->ran_param_val.flag_false->type);
        }
      }
      break;

    case LIST_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.lst != NULL) {
        printf("%s value.type = LIST, list_size = %zu\n",
               prefix, p->ran_param_val.lst->sz_lst_ran_param);

        for (size_t i = 0; i < p->ran_param_val.lst->sz_lst_ran_param; i++) {
          lst_ran_param_t const* item = &p->ran_param_val.lst->lst_ran_param[i];
          printf("%s list[%zu].struct_size = %zu\n",
                 prefix, i, item->ran_param_struct.sz_ran_param_struct);

          for (size_t j = 0; j < item->ran_param_struct.sz_ran_param_struct; j++) {
            seq_ran_param_t const* child = &item->ran_param_struct.ran_param_struct[j];
            printf("%s list[%zu].struct[%zu].ran_param_id = %u\n",
                   prefix, i, j, child->ran_param_id);
            log_rc_ran_param_value(child, prefix);
          }
        }
      }
      break;

    case STRUCTURE_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.strct != NULL) {
        printf("%s value.type = STRUCTURE, struct_size = %zu\n",
               prefix, p->ran_param_val.strct->sz_ran_param_struct);

        for (size_t i = 0; i < p->ran_param_val.strct->sz_ran_param_struct; i++) {
          seq_ran_param_t const* child = &p->ran_param_val.strct->ran_param_struct[i];
          printf("%s struct[%zu].ran_param_id = %u\n", prefix, i, child->ran_param_id);
          log_rc_ran_param_value(child, prefix);
        }
      } else {
        printf("%s value.type = STRUCTURE (NULL)\n", prefix);
      }
      break;

    default:
      printf("%s value.type = UNKNOWN(%d)\n", prefix, p->ran_param_val.type);
      break;
  }
}

static
void log_gnb_ue_id(ue_id_e2sm_t ue_id)
{
  if (ue_id.gnb.gnb_cu_ue_f1ap_lst != NULL) {
    for (size_t i = 0; i < ue_id.gnb.gnb_cu_ue_f1ap_lst_len; i++) {
      printf("UE ID type = gNB-CU, gnb_cu_ue_f1ap = %u\n",
             ue_id.gnb.gnb_cu_ue_f1ap_lst[i]);
    }
  } else {
    printf("UE ID type = gNB, amf_ue_ngap_id = %" PRIu64 "\n",
           (uint64_t)ue_id.gnb.amf_ue_ngap_id);
  }

  if (ue_id.gnb.ran_ue_id != NULL) {
    printf("ran_ue_id = %" PRIx64 "\n", (uint64_t)*ue_id.gnb.ran_ue_id);
  }
}

static
void log_du_ue_id(ue_id_e2sm_t ue_id)
{
  printf("UE ID type = gNB-DU, gnb_cu_ue_f1ap = %u\n", ue_id.gnb_du.gnb_cu_ue_f1ap);
  if (ue_id.gnb_du.ran_ue_id != NULL) {
    printf("ran_ue_id = %" PRIx64 "\n", (uint64_t)*ue_id.gnb_du.ran_ue_id);
  }
}

static
void log_cuup_ue_id(ue_id_e2sm_t ue_id)
{
  printf("UE ID type = gNB-CU-UP, gnb_cu_cp_ue_e1ap = %u\n", ue_id.gnb_cu_up.gnb_cu_cp_ue_e1ap);
  if (ue_id.gnb_cu_up.ran_ue_id != NULL) {
    printf("ran_ue_id = %" PRIx64 "\n", (uint64_t)*ue_id.gnb_cu_up.ran_ue_id);
  }
}

typedef void (*log_ue_id_fn)(ue_id_e2sm_t ue_id);

static
log_ue_id_fn log_ue_id_e2sm[END_UE_ID_E2SM] = {
    log_gnb_ue_id,
    log_du_ue_id,
    log_cuup_ue_id,
    NULL,
    NULL,
    NULL,
    NULL,
};

static
void log_saved_ue_id(char const* prefix, ue_id_e2sm_t const* id)
{
  assert(prefix != NULL);
  assert(id != NULL);

  printf("%s UE-ID dump start\n", prefix);

  if (id->type < END_UE_ID_E2SM && log_ue_id_e2sm[id->type] != NULL) {
    log_ue_id_e2sm[id->type](*id);
  } else {
    printf("%s UE ID type not supported for logging: %d\n", prefix, id->type);
  }

  printf("%s UE-ID dump end\n", prefix);
}

static
void log_rc_ctrl_msg(char const* prefix, rc_ctrl_req_data_t const* rc_ctrl)
{
  assert(prefix != NULL);
  assert(rc_ctrl != NULL);

  printf("%s ===== RC CONTROL MESSAGE DUMP START =====\n", prefix);
  printf("%s hdr.format = %d\n", prefix, rc_ctrl->hdr.format);
  printf("%s msg.format = %d\n", prefix, rc_ctrl->msg.format);

  if (rc_ctrl->hdr.format == FORMAT_1_E2SM_RC_CTRL_HDR) {
    printf("%s ric_style_type = %d\n", prefix, rc_ctrl->hdr.frmt_1.ric_style_type);
    printf("%s ctrl_act_id = %d\n", prefix, rc_ctrl->hdr.frmt_1.ctrl_act_id);
    log_saved_ue_id(prefix, &rc_ctrl->hdr.frmt_1.ue_id);
  }

  if (rc_ctrl->msg.format == FORMAT_1_E2SM_RC_CTRL_MSG) {
    printf("%s sz_ran_param = %zu\n", prefix, rc_ctrl->msg.frmt_1.sz_ran_param);

    for (size_t i = 0; i < rc_ctrl->msg.frmt_1.sz_ran_param; i++) {
      seq_ran_param_t const* p = &rc_ctrl->msg.frmt_1.ran_param[i];
      printf("%s ran_param[%zu].id = %u\n", prefix, i, p->ran_param_id);
      log_rc_ran_param_value(p, prefix);
    }
  }

  printf("%s ===== RC CONTROL MESSAGE DUMP END =====\n", prefix);
}

/* --------------------------------------------------------------- */
/* UE key helpers                                                  */
/* --------------------------------------------------------------- */

static
uint64_t make_ue_key(ue_id_e2sm_t const* ue)
{
  assert(ue != NULL);

  switch (ue->type) {
    case GNB_UE_ID_E2SM:
      if (ue->gnb.ran_ue_id != NULL)
        return *ue->gnb.ran_ue_id;
      return ue->gnb.amf_ue_ngap_id;

    case GNB_DU_UE_ID_E2SM:
      if (ue->gnb_du.ran_ue_id != NULL)
        return *ue->gnb_du.ran_ue_id;
      return ue->gnb_du.gnb_cu_ue_f1ap;

    case GNB_CU_UP_UE_ID_E2SM:
      if (ue->gnb_cu_up.ran_ue_id != NULL)
        return *ue->gnb_cu_up.ran_ue_id;
      return ue->gnb_cu_up.gnb_cu_cp_ue_e1ap;

    default:
      return 0;
  }
}

static
observed_ue_t* find_or_add_observed_ue(ue_id_e2sm_t const* ue, slice_profile_t const* profile)
{
  assert(ue != NULL);
  assert(profile != NULL);

  uint64_t key = make_ue_key(ue);
  assert(key != 0 && "UE key could not be derived");

  for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
    if (g_observed_ues[i].valid && g_observed_ues[i].key == key) {
      return &g_observed_ues[i];
    }
  }

  for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
    if (!g_observed_ues[i].valid) {
      g_observed_ues[i].valid = true;
      g_observed_ues[i].key = key;
      g_observed_ues[i].profile = *profile;
      g_observed_ues[i].ue_id = cp_ue_id_e2sm(ue);
      return &g_observed_ues[i];
    }
  }

  assert(false && "Observed UE table full");
  return NULL;
}

/* --------------------------------------------------------------- */
/* KPM measurement logging                                         */
/* --------------------------------------------------------------- */

static
void log_int_value(byte_array_t name, meas_record_lst_t meas_record)
{
  if (cmp_str_ba("RRU.PrbTotDl", name) == 0) {
    printf("RRU.PrbTotDl = %u [%%]\n", meas_record.int_val);
  } else if (cmp_str_ba("RRU.PrbTotUl", name) == 0) {
    printf("RRU.PrbTotUl = %u [%%]\n", meas_record.int_val);
  } else if (cmp_str_ba("DRB.PdcpSduVolumeDL", name) == 0) {
    printf("DRB.PdcpSduVolumeDL = %u [kb]\n", meas_record.int_val);
  } else if (cmp_str_ba("DRB.PdcpSduVolumeUL", name) == 0) {
    printf("DRB.PdcpSduVolumeUL = %u [kb]\n", meas_record.int_val);
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

typedef void (*log_meas_value_fn)(byte_array_t name, meas_record_lst_t meas_record);

static
log_meas_value_fn get_meas_value[END_MEAS_VALUE] = {
    log_int_value,
    log_real_value,
    NULL,
};

static
void match_meas_name_type(meas_type_t meas_type, meas_record_lst_t meas_record)
{
  if (meas_record.value < END_MEAS_VALUE && get_meas_value[meas_record.value] != NULL)
    get_meas_value[meas_record.value](meas_type.name, meas_record);
}

static
void match_id_meas_type(meas_type_t meas_type, meas_record_lst_t meas_record)
{
  (void)meas_type;
  (void)meas_record;
  printf(LOG_PREFIX " warning: ID Measurement Type not yet supported; skipping record\n");
}

typedef void (*check_meas_type_fn)(meas_type_t meas_type, meas_record_lst_t meas_record);

static
check_meas_type_fn match_meas_type[END_MEAS_TYPE] = {
    match_meas_name_type,
    match_id_meas_type,
};

static
char const* meas_value_type_name(meas_value_e value)
{
  switch (value) {
    case INTEGER_MEAS_VALUE:
      return "int";
    case REAL_MEAS_VALUE:
      return "real";
    case NO_VALUE_MEAS_VALUE:
      return "none";
    default:
      return "unknown";
  }
}

static
void log_kpm_csv_line(slice_profile_t const* profile,
                      uint64_t ue_key,
                      int64_t collect_start_us,
                      int64_t recv_time_us,
                      int counter,
                      size_t report_idx,
                      size_t record_idx,
                      meas_type_t meas_type,
                      meas_record_lst_t meas_record,
                      bool incomplete)
{
  assert(profile != NULL);
  assert(meas_type.type == NAME_MEAS_TYPE);

  int const name_len = (int)meas_type.name.len;
  char const* name = (char const*)meas_type.name.buf;
  int64_t const latency_us = recv_time_us - collect_start_us;

  printf("[KPMCSV] t_us=%" PRId64 " collect_start_us=%" PRId64
         " latency_us=%" PRId64 " counter=%d profile=%s sst=%u"
         " sd=%02x%02x%02x db_desc=%s ue_key=%" PRIu64
         " report_idx=%zu record_idx=%zu meas=%.*s type=%s",
         recv_time_us,
         collect_start_us,
         latency_us,
         counter,
         profile->profile_name,
         profile->sst,
         profile->sd[0],
         profile->sd[1],
         profile->sd[2],
         profile->db_ue_desc,
         ue_key,
         report_idx,
         record_idx,
         name_len,
         name,
         meas_value_type_name(meas_record.value));

  switch (meas_record.value) {
    case INTEGER_MEAS_VALUE:
      printf(" value=%u", meas_record.int_val);
      break;
    case REAL_MEAS_VALUE:
      printf(" value=%.6f", meas_record.real_val);
      break;
    case NO_VALUE_MEAS_VALUE:
      printf(" value=nan");
      break;
    default:
      printf(" value=nan");
      break;
  }

  printf(" incomplete=%d\n", incomplete ? 1 : 0);
}

static
void log_kpm_measurements(kpm_ind_msg_format_1_t const* msg_frm_1,
                          slice_profile_t const* profile,
                          uint64_t ue_key,
                          int64_t collect_start_us,
                          int64_t recv_time_us,
                          int counter,
                          size_t report_idx,
                          kpi_sample_t* sample)
{
  assert(msg_frm_1->meas_info_lst_len > 0 && "Cannot correctly print measurements");
  assert(sample != NULL);

  for (size_t j = 0; j < msg_frm_1->meas_data_lst_len; j++) {
    meas_data_lst_t const data_item = msg_frm_1->meas_data_lst[j];
    bool const incomplete = data_item.incomplete_flag != NULL &&
                            *data_item.incomplete_flag == TRUE_ENUM_VALUE;
    size_t const record_len = data_item.meas_record_len < msg_frm_1->meas_info_lst_len ?
        data_item.meas_record_len : msg_frm_1->meas_info_lst_len;

    if (data_item.meas_record_len != msg_frm_1->meas_info_lst_len) {
      printf(LOG_PREFIX " warning: KPM meas_record_len=%zu meas_info_len=%zu;"
             " parsing %zu records\n",
             data_item.meas_record_len,
             msg_frm_1->meas_info_lst_len,
             record_len);
    }

    for (size_t z = 0; z < record_len; z++) {
      meas_type_t const meas_type = msg_frm_1->meas_info_lst[z].meas_type;
      meas_record_lst_t const record_item = data_item.meas_record_lst[z];

      update_sample_from_record(sample, meas_type, record_item);

      if (meas_type.type == NAME_MEAS_TYPE) {
        log_kpm_csv_line(profile,
                         ue_key,
                         collect_start_us,
                         recv_time_us,
                         counter,
                         report_idx,
                         z,
                         meas_type,
                         record_item,
                         incomplete);
      }

      if (meas_type.type < END_MEAS_TYPE && match_meas_type[meas_type.type] != NULL) {
        match_meas_type[meas_type.type](meas_type, record_item);
      } else {
        printf(LOG_PREFIX " warning: unsupported KPM measurement type=%d\n",
               meas_type.type);
      }

      if (incomplete)
        printf("Measurement Record not reliable\n");
    }
  }
}

/* --------------------------------------------------------------- */
/* KPM callbacks per slice                                         */
/* --------------------------------------------------------------- */

static
void handle_kpm_with_profile(sm_ag_if_rd_t const* rd, slice_profile_t const* profile)
{
  assert(rd != NULL);
  assert(profile != NULL);
  assert(rd->type == INDICATION_MSG_AGENT_IF_ANS_V0);
  assert(rd->ind.type == KPM_STATS_V3_0);

  kpm_ind_data_t const* ind = &rd->ind.kpm.ind;
  kpm_ric_ind_hdr_format_1_t const* hdr_frm_1 = &ind->hdr.kpm_ric_ind_hdr_format_1;
  kpm_ind_msg_format_3_t const* msg_frm_3 = &ind->msg.frm_3;

  static int counter = 1;
  int64_t const now = time_now_us();

  lock_guard(&mtx);

  printf("\n[KPM][%s] latency = %" PRId64 " [us], counter=%d\n",
         profile->profile_name, now - hdr_frm_1->collectStartTime, counter);

  printf("[KPM][%s] expected snssai=(sst=%u, sd=%02x%02x%02x)\n",
         profile->profile_name,
         profile->sst,
         profile->sd[0], profile->sd[1], profile->sd[2]);

  printf(LOG_PREFIX " kpi_mapping: subscription_context=S-NSSAI profile=%s"
         " sst=%u sd=%02x%02x%02x sd_u32=%06" PRIx32
         " report_ue_count=%zu\n",
         profile->profile_name,
         profile->sst,
         profile->sd[0],
         profile->sd[1],
         profile->sd[2],
         slice_sd_u32(profile),
         msg_frm_3->ue_meas_report_lst_len);

  if (msg_frm_3->ue_meas_report_lst_len == 0) {
    printf("[KPM][%s] No UE matches this S-NSSAI in current report\n",
           profile->profile_name);
  } else if (msg_frm_3->ue_meas_report_lst_len > 1) {
    printf(LOG_PREFIX " warning: subscription profile=%s returned %zu UE reports;"
           " aggregating UE-level KPI into slice state by S-NSSAI filter\n",
           profile->profile_name,
           msg_frm_3->ue_meas_report_lst_len);
  }

  kpi_sample_t agg = {0};
  uint64_t last_ue_key = 0;

  for (size_t i = 0; i < msg_frm_3->ue_meas_report_lst_len; i++) {
    ue_id_e2sm_t const* ue = &msg_frm_3->meas_report_per_ue[i].ue_meas_report_lst;
    observed_ue_t* slot = find_or_add_observed_ue(ue, profile);
    kpi_sample_t sample = {0};

    printf("[KPM][%s] saved UE key = %" PRIu64 ", db_desc = %s\n",
           profile->profile_name, slot->key, profile->db_ue_desc);

    log_saved_ue_id("[KPM]", &slot->ue_id);
    log_kpm_measurements(&msg_frm_3->meas_report_per_ue[i].ind_msg_format_1,
                         profile,
                         slot->key,
                         hdr_frm_1->collectStartTime,
                         now,
                         counter,
                         i,
                         &sample);

    add_sample_to_aggregate(&agg, &sample);
    last_ue_key = slot->key;
  }

  if (msg_frm_3->ue_meas_report_lst_len > 0)
    update_slice_kpi_state(profile, last_ue_key, now, &agg);

  counter++;
}

static
void sm_cb_kpm_sst1(sm_ag_if_rd_t const* rd)
{
  handle_kpm_with_profile(rd, &g_profile_sst1);
}

static
void sm_cb_kpm_sst2(sm_ag_if_rd_t const* rd)
{
  handle_kpm_with_profile(rd, &g_profile_sst2);
}

static
void sm_cb_kpm_sst3(sm_ag_if_rd_t const* rd)
{
  handle_kpm_with_profile(rd, &g_profile_sst3);
}

/* --------------------------------------------------------------- */
/* RC STYLE 2 / ACTION 6 enums                                     */
/* --------------------------------------------------------------- */

typedef enum {
  DRX_parameter_configuration_7_6_3_1 = 1,
  SR_periodicity_configuration_7_6_3_1 = 2,
  SPS_parameters_configuration_7_6_3_1 = 3,
  Configured_grant_control_7_6_3_1 = 4,
  CQI_table_configuration_7_6_3_1 = 5,
  Slice_level_PRB_quota_7_6_3_1 = 6,
} rc_ctrl_service_style_2_e;

typedef enum {
  RRM_POLICY_RATIO_LIST_8_4_3_6       = 1,
  RRM_POLICY_RATIO_GROUP_8_4_3_6      = 2,
  RRM_POLICY_8_4_3_6                  = 3,
  RRM_POLICY_MEMBER_LIST_8_4_3_6      = 5,
  PLMN_IDENTITY_8_4_3_6               = 7,
  S_NSSAI_8_4_3_6                     = 8,
  SST_8_4_3_6                         = 9,
  SD_8_4_3_6                          = 10,
  MIN_PRB_POLICY_RATIO_8_4_3_6        = 11,
  MAX_PRB_POLICY_RATIO_8_4_3_6        = 12,
  DEDICATED_PRB_POLICY_RATIO_8_4_3_6  = 13,
} slice_prb_quota_conf_e;

/* --------------------------------------------------------------- */
/* RC parameter builders                                           */
/* --------------------------------------------------------------- */

static
seq_ran_param_t fill_int_flag_true_param(uint32_t ran_param_id, int64_t value)
{
  seq_ran_param_t p = {0};
  p.ran_param_id = ran_param_id;
  p.ran_param_val.type = ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.flag_true = calloc(1, sizeof(ran_parameter_value_t));
  assert(p.ran_param_val.flag_true != NULL && "Memory exhausted");

  p.ran_param_val.flag_true->type = INTEGER_RAN_PARAMETER_VALUE;
  p.ran_param_val.flag_true->int_ran = value;
  return p;
}

static
seq_ran_param_t fill_int_flag_false_param(uint32_t ran_param_id, int64_t value)
{
  seq_ran_param_t p = {0};
  p.ran_param_id = ran_param_id;
  p.ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.flag_false = calloc(1, sizeof(ran_parameter_value_t));
  assert(p.ran_param_val.flag_false != NULL && "Memory exhausted");

  p.ran_param_val.flag_false->type = INTEGER_RAN_PARAMETER_VALUE;
  p.ran_param_val.flag_false->int_ran = value;
  return p;
}

static
seq_ran_param_t fill_octet_flag_false_param(uint32_t ran_param_id,
                                            uint8_t const* buf,
                                            size_t len)
{
  seq_ran_param_t p = {0};
  p.ran_param_id = ran_param_id;
  p.ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.flag_false = calloc(1, sizeof(ran_parameter_value_t));
  assert(p.ran_param_val.flag_false != NULL && "Memory exhausted");

  p.ran_param_val.flag_false->type = OCTET_STRING_RAN_PARAMETER_VALUE;
  p.ran_param_val.flag_false->octet_str_ran.buf = calloc(len, sizeof(uint8_t));
  assert(p.ran_param_val.flag_false->octet_str_ran.buf != NULL && "Memory exhausted");
  memcpy(p.ran_param_val.flag_false->octet_str_ran.buf, buf, len);
  p.ran_param_val.flag_false->octet_str_ran.len = len;

  return p;
}

static
seq_ran_param_t fill_snssai_param(slice_profile_t const* profile)
{
  assert(profile != NULL);

  seq_ran_param_t p = {0};
  p.ran_param_id = S_NSSAI_8_4_3_6;
  p.ran_param_val.type = STRUCTURE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.strct = calloc(1, sizeof(ran_param_struct_t));
  assert(p.ran_param_val.strct != NULL && "Memory exhausted");

  p.ran_param_val.strct->sz_ran_param_struct = 2;
  p.ran_param_val.strct->ran_param_struct = calloc(2, sizeof(seq_ran_param_t));
  assert(p.ran_param_val.strct->ran_param_struct != NULL && "Memory exhausted");

  p.ran_param_val.strct->ran_param_struct[0] =
      fill_int_flag_true_param(SST_8_4_3_6, profile->sst);

  p.ran_param_val.strct->ran_param_struct[1] =
      fill_octet_flag_false_param(SD_8_4_3_6, profile->sd, sizeof(profile->sd));

  return p;
}

static
seq_ran_param_t fill_rrm_policy_member_list_param(slice_profile_t const* profile)
{
  assert(profile != NULL);

  seq_ran_param_t p = {0};
  p.ran_param_id = RRM_POLICY_MEMBER_LIST_8_4_3_6;
  p.ran_param_val.type = LIST_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.lst = calloc(1, sizeof(ran_param_list_t));
  assert(p.ran_param_val.lst != NULL && "Memory exhausted");

  ran_param_list_t* list = p.ran_param_val.lst;
  list->sz_lst_ran_param = 1;
  list->lst_ran_param = calloc(1, sizeof(lst_ran_param_t));
  assert(list->lst_ran_param != NULL && "Memory exhausted");

  lst_ran_param_t* item = &list->lst_ran_param[0];
  item->ran_param_struct.sz_ran_param_struct = 2;
  item->ran_param_struct.ran_param_struct = calloc(2, sizeof(seq_ran_param_t));
  assert(item->ran_param_struct.ran_param_struct != NULL && "Memory exhausted");

  item->ran_param_struct.ran_param_struct[0] =
      fill_octet_flag_false_param(PLMN_IDENTITY_8_4_3_6,
                                  profile->plmn,
                                  sizeof(profile->plmn));

  item->ran_param_struct.ran_param_struct[1] = fill_snssai_param(profile);

  return p;
}

static
seq_ran_param_t fill_rrm_policy_param(slice_profile_t const* profile)
{
  assert(profile != NULL);

  seq_ran_param_t p = {0};
  p.ran_param_id = RRM_POLICY_8_4_3_6;
  p.ran_param_val.type = STRUCTURE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.strct = calloc(1, sizeof(ran_param_struct_t));
  assert(p.ran_param_val.strct != NULL && "Memory exhausted");

  p.ran_param_val.strct->sz_ran_param_struct = 1;
  p.ran_param_val.strct->ran_param_struct = calloc(1, sizeof(seq_ran_param_t));
  assert(p.ran_param_val.strct->ran_param_struct != NULL && "Memory exhausted");

  p.ran_param_val.strct->ran_param_struct[0] =
      fill_rrm_policy_member_list_param(profile);

  return p;
}

static
seq_ran_param_t fill_slice_level_prb_quota_param(slice_profile_t const* profile)
{
  assert(profile != NULL);

  seq_ran_param_t top = {0};
  top.ran_param_id = RRM_POLICY_RATIO_LIST_8_4_3_6;
  top.ran_param_val.type = LIST_RAN_PARAMETER_VAL_TYPE;
  top.ran_param_val.lst = calloc(1, sizeof(ran_param_list_t));
  assert(top.ran_param_val.lst != NULL && "Memory exhausted");

  ran_param_list_t* ratio_list = top.ran_param_val.lst;
  ratio_list->sz_lst_ran_param = 1;
  ratio_list->lst_ran_param = calloc(1, sizeof(lst_ran_param_t));
  assert(ratio_list->lst_ran_param != NULL && "Memory exhausted");

  lst_ran_param_t* item = &ratio_list->lst_ran_param[0];
  item->ran_param_struct.sz_ran_param_struct = 4;
  item->ran_param_struct.ran_param_struct = calloc(4, sizeof(seq_ran_param_t));
  assert(item->ran_param_struct.ran_param_struct != NULL && "Memory exhausted");

  item->ran_param_struct.ran_param_struct[0] = fill_rrm_policy_param(profile);
  item->ran_param_struct.ran_param_struct[1] =
      fill_int_flag_false_param(MIN_PRB_POLICY_RATIO_8_4_3_6, profile->min_prb);
  item->ran_param_struct.ran_param_struct[2] =
      fill_int_flag_false_param(MAX_PRB_POLICY_RATIO_8_4_3_6, profile->max_prb);
  item->ran_param_struct.ran_param_struct[3] =
      fill_int_flag_false_param(DEDICATED_PRB_POLICY_RATIO_8_4_3_6,
                                profile->dedicated_prb);

  return top;
}

/* --------------------------------------------------------------- */
/* RC CONTROL message generation                                   */
/* --------------------------------------------------------------- */

static
void fill_rc_ctrl_act_style_2(seq_ctrl_act_2_t const* ctrl_act,
                              size_t const sz,
                              slice_profile_t const* profile,
                              e2sm_rc_ctrl_hdr_frmt_1_t* hdr,
                              e2sm_rc_ctrl_msg_frmt_1_t* msg)
{
  assert(ctrl_act != NULL);
  assert(profile != NULL);
  assert(hdr != NULL);
  assert(msg != NULL);

  for (size_t i = 0; i < sz; i++) {
    if (cmp_str_ba("Slice-level PRB quota", ctrl_act[i].name) != 0)
      continue;

    printf("[RIC-XAPP][CTRL] matched action name = Slice-level PRB quota\n");

    hdr->ctrl_act_id = Slice_level_PRB_quota_7_6_3_1;

    msg->sz_ran_param = 1;
    msg->ran_param = calloc(msg->sz_ran_param, sizeof(seq_ran_param_t));
    assert(msg->ran_param != NULL && "Memory exhausted");

    msg->ran_param[0] = fill_slice_level_prb_quota_param(profile);
    return;
  }

  assert(false && "Slice-level PRB quota action not found in RC style 2");
}

static
rc_ctrl_req_data_t gen_rc_ctrl_msg_style_2(ran_func_def_ctrl_t const* ran_func,
                                           observed_ue_t const* obs)
{
  assert(ran_func != NULL);
  assert(obs != NULL);
  assert(obs->valid == true);

  rc_ctrl_req_data_t rc_ctrl = {0};

  printf("[RIC-XAPP][CTRL] gen_rc_ctrl_msg_style_2: start\n");
  printf("[RIC-XAPP][CTRL] target profile = %s, db_desc = %s, snssai=(%u,%02x%02x%02x)\n",
         obs->profile.profile_name, obs->profile.db_ue_desc,
         obs->profile.sst,
         obs->profile.sd[0], obs->profile.sd[1], obs->profile.sd[2]);

  for (size_t i = 0; i < ran_func->sz_seq_ctrl_style; i++) {
    if (cmp_str_ba("Radio Resource Allocation Control", ran_func->seq_ctrl_style[i].name) != 0)
      continue;

    printf("[RIC-XAPP][CTRL] matched control style[%zu] = Radio Resource Allocation Control\n", i);

    rc_ctrl.hdr.format = ran_func->seq_ctrl_style[i].hdr;
    assert(rc_ctrl.hdr.format == FORMAT_1_E2SM_RC_CTRL_HDR);

    rc_ctrl.hdr.frmt_1.ric_style_type = 2;
    rc_ctrl.hdr.frmt_1.ue_id = cp_ue_id_e2sm(&obs->ue_id);

    rc_ctrl.msg.format = ran_func->seq_ctrl_style[i].msg;
    assert(rc_ctrl.msg.format == FORMAT_1_E2SM_RC_CTRL_MSG);

    fill_rc_ctrl_act_style_2(ran_func->seq_ctrl_style[i].seq_ctrl_act,
                             ran_func->seq_ctrl_style[i].sz_seq_ctrl_act,
                             &obs->profile,
                             &rc_ctrl.hdr.frmt_1,
                             &rc_ctrl.msg.frmt_1);

    log_rc_ctrl_msg("[RIC-XAPP][CTRL]", &rc_ctrl);
    printf("[RIC-XAPP][CTRL] gen_rc_ctrl_msg_style_2: end\n");
    return rc_ctrl;
  }

  assert(false && "Radio Resource Allocation Control style not found");
  return rc_ctrl;
}

/* --------------------------------------------------------------- */
/* KPM subscription helpers                                        */
/* --------------------------------------------------------------- */

static
test_info_lst_t filter_predicate_snssai(slice_profile_t const* profile)
{
  assert(profile != NULL);

  test_info_lst_t dst = {0};

  dst.test_cond_type = S_NSSAI_TEST_COND_TYPE;
  dst.S_NSSAI = TRUE_TEST_COND_TYPE;

  dst.test_cond = calloc(1, sizeof(test_cond_e));
  assert(dst.test_cond != NULL && "Memory exhausted");
  *dst.test_cond = EQUAL_TEST_COND;

  dst.test_cond_value = calloc(1, sizeof(test_cond_value_t));
  assert(dst.test_cond_value != NULL && "Memory exhausted");
  dst.test_cond_value->type = OCTET_STRING_TEST_COND_VALUE;

  dst.test_cond_value->octet_string_value = calloc(1, sizeof(byte_array_t));
  assert(dst.test_cond_value->octet_string_value != NULL && "Memory exhausted");

  /* Encode as [SST][SD0][SD1][SD2] */
  dst.test_cond_value->octet_string_value->len = 4;
  dst.test_cond_value->octet_string_value->buf = calloc(4, sizeof(uint8_t));
  assert(dst.test_cond_value->octet_string_value->buf != NULL && "Memory exhausted");

  dst.test_cond_value->octet_string_value->buf[0] = profile->sst;
  dst.test_cond_value->octet_string_value->buf[1] = profile->sd[0];
  dst.test_cond_value->octet_string_value->buf[2] = profile->sd[1];
  dst.test_cond_value->octet_string_value->buf[3] = profile->sd[2];

  printf("[RIC-XAPP][KPM] subscribe profile=%s, S-NSSAI=(sst=%u, sd=%02x%02x%02x)\n",
         profile->profile_name,
         profile->sst,
         profile->sd[0], profile->sd[1], profile->sd[2]);

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

static
kpm_act_def_t fill_report_style_4_with_profile(ric_report_style_item_t const* report_item,
                                               slice_profile_t const* profile)
{
  assert(report_item != NULL);
  assert(profile != NULL);
  assert(report_item->act_def_format_type == FORMAT_4_ACTION_DEFINITION);

  kpm_act_def_t act_def = {.type = FORMAT_4_ACTION_DEFINITION};

  act_def.frm_4.matching_cond_lst_len = 1;
  act_def.frm_4.matching_cond_lst = calloc(1, sizeof(matching_condition_format_4_lst_t));
  assert(act_def.frm_4.matching_cond_lst != NULL && "Memory exhausted");

  act_def.frm_4.matching_cond_lst[0].test_info_lst = filter_predicate_snssai(profile);
  act_def.frm_4.action_def_format_1 = fill_act_def_frm_1(report_item);

  return act_def;
}

static
kpm_sub_data_t gen_kpm_subs_for_profile(kpm_ran_function_def_t const* ran_func,
                                        slice_profile_t const* profile)
{
  assert(ran_func != NULL);
  assert(profile != NULL);
  assert(ran_func->ric_event_trigger_style_list != NULL);

  kpm_sub_data_t kpm_sub = {0};

  assert(ran_func->ric_event_trigger_style_list[0].format_type == FORMAT_1_RIC_EVENT_TRIGGER);
  kpm_sub.ev_trg_def.type = FORMAT_1_RIC_EVENT_TRIGGER;
  kpm_sub.ev_trg_def.kpm_ric_event_trigger_format_1.report_period_ms = period_ms;

  kpm_sub.sz_ad = 1;
  kpm_sub.ad = calloc(kpm_sub.sz_ad, sizeof(kpm_act_def_t));
  assert(kpm_sub.ad != NULL && "Memory exhausted");

  ric_report_style_item_t* const report_item = &ran_func->ric_report_style_list[0];
  *kpm_sub.ad = fill_report_style_4_with_profile(report_item, profile);

  return kpm_sub;
}

/* --------------------------------------------------------------- */
/* Generic RAN function lookup                                     */
/* --------------------------------------------------------------- */

static
bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  return elem->id == id;
}

static
size_t find_sm_idx(sm_ran_function_t* rf,
                   size_t sz,
                   bool (*f)(sm_ran_function_t const*, int const),
                   int const id)
{
  for (size_t i = 0; i < sz; i++) {
    if (f(&rf[i], id))
      return i;
  }

  assert(false && "SM ID could not be found in the RAN Function List");
  return 0;
}

static
bool find_sm_idx_optional(sm_ran_function_t* rf,
                          size_t sz,
                          bool (*f)(sm_ran_function_t const*, int const),
                          int const id,
                          size_t* out_idx)
{
  assert(out_idx != NULL);

  for (size_t i = 0; i < sz; i++) {
    if (f(&rf[i], id)) {
      *out_idx = i;
      return true;
    }
  }

  return false;
}

static
bool rc_supports_slice_prb_quota(ran_func_def_ctrl_t const* ran_func)
{
  if (ran_func == NULL)
    return false;

  for (size_t i = 0; i < ran_func->sz_seq_ctrl_style; i++) {
    if (cmp_str_ba("Radio Resource Allocation Control",
                   ran_func->seq_ctrl_style[i].name) != 0)
      continue;

    for (size_t j = 0; j < ran_func->seq_ctrl_style[i].sz_seq_ctrl_act; j++) {
      if (cmp_str_ba("Slice-level PRB quota",
                     ran_func->seq_ctrl_style[i].seq_ctrl_act[j].name) == 0)
        return true;
    }
  }

  return false;
}

/* --------------------------------------------------------------- */
/* Closed-loop PRB control                                         */
/* --------------------------------------------------------------- */

typedef struct {
  bool valid;
  int slice_idx;
  uint64_t ue_key;
  uint16_t desired_prb;
  uint16_t previous_prb;
  observed_ue_t obs;
} control_candidate_t;

typedef struct {
  e2_node_arr_xapp_t* nodes;
} control_loop_args_t;

static
uint16_t round_prb_ratio(double v)
{
  if (v <= 0.0)
    return 0;
  if (v >= 100.0)
    return 100;
  return (uint16_t)(v + 0.5);
}

static
double bounded_metric_ratio(double value, double normalizer)
{
  if (normalizer <= 0.0 || value <= 0.0)
    return 0.0;
  return clamp_double(value / normalizer, 0.0, 1.0);
}

static
double compute_demand_from_available_kpi(slice_state_t const* st, bool* has_metric)
{
  assert(st != NULL);
  assert(has_metric != NULL);

  /*
   * The current OAI RC slice PRB policy is enforced by the DL scheduler
   * only, so closed-loop demand must be based on DL-side KPI only.
   * UL KPI is still collected and logged for visibility, but it must not
   * increase a slice's DL PRB quota.
   */
  double demand = 0.0;
  double weight = 0.0;

  if (st->has_thp_dl_kbps) {
    demand += bounded_metric_ratio(st->ema_thp_dl_kbps, DL_THP_NORMALIZER_KBPS);
    weight += 1.0;
  }

  if (st->has_prb_dl_pct) {
    demand += 0.75 * bounded_metric_ratio(st->ema_prb_dl_pct, PRB_USAGE_NORMALIZER_PCT);
    weight += 0.75;
  }

  if (weight <= 0.0) {
    *has_metric = false;
    return 0.0;
  }

  *has_metric = true;
  return demand / weight;
}

typedef struct {
  bool valid;
  bool stale;
  bool has_metric;
  int64_t age_ms;
  double raw_demand;
  double old_smoothed_demand;
  double smoothed_demand;
  uint16_t before_bound_prb;
  uint16_t target_prb;
  uint16_t previous_prb;
} prb_calc_entry_t;

static
double demand_weight_for_allocation(double demand, bool equal_fallback)
{
  if (equal_fallback)
    return 1.0;
  return demand > DEMAND_EPSILON ? demand : DEMAND_EPSILON;
}

static
size_t count_eligible_slices(bool const eligible[NUM_SLICE_PROFILES])
{
  size_t n = 0;

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (eligible[i])
      n++;
  }

  return n;
}

static
void calculate_proportional_prb_targets(bool const eligible[NUM_SLICE_PROFILES],
                                        double const demand[NUM_SLICE_PROFILES],
                                        uint16_t before_bound[NUM_SLICE_PROFILES],
                                        uint16_t target[NUM_SLICE_PROFILES])
{
  size_t const n_eligible = count_eligible_slices(eligible);
  double total_demand = 0.0;

  memset(before_bound, 0, NUM_SLICE_PROFILES * sizeof(before_bound[0]));
  memset(target, 0, NUM_SLICE_PROFILES * sizeof(target[0]));

  if (n_eligible == 0)
    return;

  int sum_min = 0;
  int sum_max = 0;

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (!eligible[i])
      continue;

    slice_profile_t const* profile = g_slice_states[i].profile;
    uint16_t const min_prb = profile_effective_min(profile);
    uint16_t const max_prb = profile_effective_max(profile);

    if (profile->min_prb > profile->max_prb) {
      printf(LOG_PREFIX " warning: invalid PRB bounds for slice=%s min=%u max=%u;"
             " using effective_min=%u effective_max=%u\n",
             profile->profile_name,
             (unsigned)profile->min_prb,
             (unsigned)profile->max_prb,
             (unsigned)min_prb,
             (unsigned)max_prb);
    }

    sum_min += min_prb;
    sum_max += max_prb;
  }

  if (sum_min > TOTAL_PRB_BUDGET_RATIO) {
    printf(LOG_PREFIX " warning: sum(min_prb)=%d exceeds total_budget=%u;"
           " keeping per-slice minimums, target sum may exceed budget\n",
           sum_min,
           (unsigned)TOTAL_PRB_BUDGET_RATIO);
  }

  if (sum_max < TOTAL_PRB_BUDGET_RATIO) {
    printf(LOG_PREFIX " warning: sum(max_prb)=%d below total_budget=%u;"
           " target sum cannot fully consume budget\n",
           sum_max,
           (unsigned)TOTAL_PRB_BUDGET_RATIO);
  }

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (eligible[i])
      total_demand += demand[i];
  }

  bool const equal_fallback = total_demand <= DEMAND_EPSILON;
  double total_weight = 0.0;

  if (!equal_fallback) {
    for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
      if (eligible[i])
        total_weight += demand_weight_for_allocation(demand[i], false);
    }
  }

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (!eligible[i])
      continue;

    double const share = equal_fallback ?
        1.0 / (double)n_eligible : demand_weight_for_allocation(demand[i], false) / total_weight;
    before_bound[i] = round_prb_ratio((double)TOTAL_PRB_BUDGET_RATIO * share);
  }

  bool fixed[NUM_SLICE_PROFILES] = {0};
  int remaining_budget = TOTAL_PRB_BUDGET_RATIO;

  while (true) {
    bool changed = false;
    size_t remaining_count = 0;
    double remaining_demand = 0.0;

    for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
      if (eligible[i] && !fixed[i]) {
        remaining_count++;
        remaining_demand += demand_weight_for_allocation(demand[i], equal_fallback);
      }
    }

    if (remaining_count == 0)
      break;

    for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
      if (!eligible[i] || fixed[i])
        continue;

      slice_profile_t const* profile = g_slice_states[i].profile;
      uint16_t const min_prb = profile_effective_min(profile);
      uint16_t const max_prb = profile_effective_max(profile);
      double const weight = demand_weight_for_allocation(demand[i], equal_fallback);
      double const raw = remaining_demand <= DEMAND_EPSILON ?
          (double)remaining_budget / (double)remaining_count :
          (double)remaining_budget * weight / remaining_demand;

      if (raw < (double)min_prb) {
        target[i] = min_prb;
        fixed[i] = true;
        remaining_budget -= target[i];
        changed = true;
      } else if (raw > (double)max_prb) {
        target[i] = max_prb;
        fixed[i] = true;
        remaining_budget -= target[i];
        changed = true;
      }
    }

    if (!changed)
      break;
  }

  size_t remaining_count = 0;
  double remaining_demand = 0.0;

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (eligible[i] && !fixed[i]) {
      remaining_count++;
      remaining_demand += demand_weight_for_allocation(demand[i], equal_fallback);
    }
  }

  int assigned = 0;
  double fractional[NUM_SLICE_PROFILES] = {0.0};

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (!eligible[i] || fixed[i])
      continue;

    slice_profile_t const* profile = g_slice_states[i].profile;
    uint16_t const min_prb = profile_effective_min(profile);
    uint16_t const max_prb = profile_effective_max(profile);
    double const weight = demand_weight_for_allocation(demand[i], equal_fallback);
    double raw = remaining_demand <= DEMAND_EPSILON ?
        (double)remaining_budget / (double)remaining_count :
        (double)remaining_budget * weight / remaining_demand;

    raw = clamp_double(raw, min_prb, max_prb);
    target[i] = (uint16_t)raw;
    fractional[i] = raw - (double)target[i];
    assigned += target[i];
  }

  int residual = remaining_budget - assigned;

  while (residual > 0) {
    int best = -1;
    double best_fraction = -1.0;

    for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
      if (!eligible[i] || target[i] >= profile_effective_max(g_slice_states[i].profile))
        continue;
      if (fractional[i] > best_fraction) {
        best_fraction = fractional[i];
        best = i;
      }
    }

    if (best < 0)
      break;

    target[best]++;
    residual--;
  }

  while (residual < 0) {
    int best = -1;
    double best_demand = 0.0;

    for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
      if (!eligible[i] || target[i] <= profile_effective_min(g_slice_states[i].profile))
        continue;
      double const w = demand_weight_for_allocation(demand[i], equal_fallback);
      if (best < 0 || w < best_demand) {
        best = i;
        best_demand = w;
      }
    }

    if (best < 0)
      break;

    target[best]--;
    residual++;
  }
}

static
size_t build_control_candidates(control_candidate_t* candidates,
                                size_t max_candidates,
                                int64_t now_us,
                                uint64_t cycle)
{
  assert(candidates != NULL);

  bool eligible[NUM_SLICE_PROFILES] = {0};
  double demand[NUM_SLICE_PROFILES] = {0.0};
  uint16_t before_bound[NUM_SLICE_PROFILES] = {0};
  uint16_t target[NUM_SLICE_PROFILES] = {0};
  prb_calc_entry_t calc[NUM_SLICE_PROFILES] = {0};
  bool candidate_for_slice[NUM_SLICE_PROFILES] = {0};
  bool observed_for_slice[NUM_SLICE_PROFILES] = {0};
  uint16_t previous_prb[NUM_SLICE_PROFILES] = {0};
  size_t n = 0;
  size_t num_valid_slices = 0;
  size_t num_eligible_slices = 0;
  size_t num_stale_slices = 0;

  pthread_mutex_lock(&mtx);

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (g_slice_states[i].valid)
      num_valid_slices++;
  }

  printf(LOG_PREFIX " KPI snapshot: num_slices=%zu\n", num_valid_slices);

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    slice_state_t* st = &g_slice_states[i];
    int64_t const age_us = st->valid ? now_us - st->last_kpm_us : INT64_MAX;
    bool const stale = !st->valid || age_us < 0 || age_us > KPI_STALE_TIMEOUT_US;
    uint16_t const previous = st->has_ctrl ?
        st->last_sent_max_prb : st->profile->max_prb;
    bool has_metric = false;
    double raw_demand = 0.0;

    previous_prb[i] = previous;
    calc[i].valid = st->valid;
    calc[i].stale = stale;
    calc[i].age_ms = st->valid && age_us >= 0 ? age_us / 1000 : -1;
    calc[i].previous_prb = previous;

    printf(LOG_PREFIX " parsed KPI: slice=%s sst=%u sd=%02x%02x%02x"
           " subscription_context=S-NSSAI"
           " raw_tput_dl=%.2f raw_tput_ul=%.2f raw_prb_dl=%.2f raw_prb_ul=%.2f"
           " active_ue=%" PRIu64 " last_kpi_age_ms=%" PRId64 " stale=%d valid=%d\n",
           st->profile->profile_name,
           st->profile->sst,
           st->profile->sd[0],
           st->profile->sd[1],
           st->profile->sd[2],
           st->thp_dl_kbps,
           st->thp_ul_kbps,
           st->prb_dl_pct,
           st->prb_ul_pct,
           st->last_ue_key,
           calc[i].age_ms,
           stale ? 1 : 0,
           st->valid ? 1 : 0);

    if (!st->valid) {
      printf(LOG_PREFIX " skip control send: cycle=%" PRIu64
             " slice=%s reason=no valid KPI age_ms=%" PRId64 "\n",
             cycle,
             st->profile->profile_name,
             calc[i].age_ms);
      continue;
    }

    eligible[i] = true;
    num_eligible_slices++;

    if (stale)
      num_stale_slices++;

    if (stale) {
      raw_demand = st->has_ctrl ?
          (double)st->last_sent_max_prb / (double)TOTAL_PRB_BUDGET_RATIO : 1.0;
      printf(LOG_PREFIX " stale KPI slice=%s age_ms=%" PRId64
             ": using previous allocation fallback demand=%.3f previous=%u\n",
             st->profile->profile_name,
             calc[i].age_ms,
             raw_demand,
             (unsigned)previous);
    } else {
      raw_demand = compute_demand_from_available_kpi(st, &has_metric);
      if (!has_metric) {
        raw_demand = 1.0;
        printf(LOG_PREFIX " warning: slice=%s has no supported DL slice KPI metric; using equal-share fallback demand\n",
               st->profile->profile_name);
      }
    }

    double const old_smoothed = st->demand_initialized ? st->smoothed_demand : raw_demand;
    double const new_smoothed = st->demand_initialized ?
        SMOOTHING_ALPHA * raw_demand + (1.0 - SMOOTHING_ALPHA) * st->smoothed_demand :
        raw_demand;

    st->demand_initialized = true;
    st->last_raw_demand = raw_demand;
    st->smoothed_demand = new_smoothed;

    demand[i] = new_smoothed;
    calc[i].has_metric = has_metric;
    calc[i].raw_demand = raw_demand;
    calc[i].old_smoothed_demand = old_smoothed;
    calc[i].smoothed_demand = new_smoothed;

    printf(LOG_PREFIX " smoothing: slice=%s raw=%.3f old=%.3f new=%.3f\n",
           st->profile->profile_name,
           raw_demand,
           old_smoothed,
           new_smoothed);
  }

  if (num_eligible_slices > 0 && num_stale_slices == num_eligible_slices) {
    printf(LOG_PREFIX " warning: all eligible KPI stale; using previous allocation fallback"
           " for this cycle to avoid oscillation\n");

    for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
      if (!eligible[i])
        continue;

      slice_state_t* st = &g_slice_states[i];
      double const fallback_demand = st->has_ctrl ?
          (double)previous_prb[i] / (double)TOTAL_PRB_BUDGET_RATIO : 1.0;

      demand[i] = fallback_demand;
      st->last_raw_demand = fallback_demand;
      st->smoothed_demand = fallback_demand;
      calc[i].raw_demand = fallback_demand;
      calc[i].old_smoothed_demand = fallback_demand;
      calc[i].smoothed_demand = fallback_demand;

      printf(LOG_PREFIX " smoothing: slice=%s all_stale_fallback raw=%.3f new=%.3f"
             " previous=%u age_ms=%" PRId64 "\n",
             st->profile->profile_name,
             fallback_demand,
             fallback_demand,
             (unsigned)previous_prb[i],
             calc[i].age_ms);
    }
  }

  calculate_proportional_prb_targets(eligible, demand, before_bound, target);

  int target_sum = 0;
  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (eligible[i])
      target_sum += target[i];
  }

  if (target_sum != TOTAL_PRB_BUDGET_RATIO) {
    printf(LOG_PREFIX " warning: target PRB sum=%d differs from total_budget=%u"
           " after bounds/residual handling\n",
           target_sum,
           (unsigned)TOTAL_PRB_BUDGET_RATIO);
  }

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (!eligible[i])
      continue;

    slice_state_t* st = &g_slice_states[i];
    calc[i].before_bound_prb = before_bound[i];
    calc[i].target_prb = target[i];
    st->last_target_prb = target[i];

    printf(LOG_PREFIX " prb_calc_before_bounds: cycle=%" PRIu64
           " slice=%s demand=%.3f target=%u\n",
           cycle,
           st->profile->profile_name,
           calc[i].smoothed_demand,
           (unsigned)before_bound[i]);

    printf(LOG_PREFIX " prb_calc_after_bounds: cycle=%" PRIu64
           " slice=%s target=%u min=%u max=%u total_target_sum=%d total_budget=%u\n",
           cycle,
           st->profile->profile_name,
           (unsigned)target[i],
           (unsigned)st->profile->min_prb,
           (unsigned)st->profile->max_prb,
           target_sum,
           (unsigned)TOTAL_PRB_BUDGET_RATIO);
  }

  for (size_t u = 0; u < MAX_OBSERVED_UE && n < max_candidates; u++) {
    if (!g_observed_ues[u].valid)
      continue;

    int const idx = find_slice_state_idx(&g_observed_ues[u].profile);
    if (idx < 0 || !eligible[idx] || candidate_for_slice[idx])
      continue;

    observed_for_slice[idx] = true;

    slice_state_t const* st = &g_slice_states[idx];
    uint16_t const previous = st->has_ctrl ?
        st->last_sent_max_prb : st->profile->max_prb;
    uint16_t const calculated_desired = clamp_prb_ratio(target[idx], st->profile);
    uint16_t desired = calculated_desired;
    if (VERIFY_FORCE_PRB_MAX_RATIO > 0)
      desired = clamp_prb_ratio(VERIFY_FORCE_PRB_MAX_RATIO, st->profile);
    uint16_t const diff = abs_diff_u16(previous, desired);
    int64_t const ctrl_age_us = st->has_ctrl ? now_us - st->last_ctrl_us : 0;
    bool const cooldown_ready = !st->has_ctrl || ctrl_age_us >= CONTROL_COOLDOWN_US;
    bool const hysteresis_ready = diff >= CONTROL_HYSTERESIS_PRB;
    bool const should_send = !st->has_ctrl || (cooldown_ready && hysteresis_ready);
    char const* skip_reason = "none";

    if (!should_send) {
      skip_reason = !hysteresis_ready ? "no significant delta" : "cooldown active";
    }

    if (VERIFY_FORCE_PRB_MAX_RATIO > 0) {
      printf(LOG_PREFIX " verify_force_prb_max: cycle=%" PRIu64
             " slice=%s calculated_max=%u forced_max=%u force_ratio=%u\n",
             cycle,
             st->profile->profile_name,
             (unsigned)calculated_desired,
             (unsigned)desired,
             (unsigned)VERIFY_FORCE_PRB_MAX_RATIO);
    }

    uint16_t msg_min = profile_effective_min(st->profile);
    if (msg_min > desired) {
      printf(LOG_PREFIX " warning: adjusted control msg min for slice=%s"
             " min=%u target_max=%u to keep min<=max\n",
             st->profile->profile_name,
             (unsigned)msg_min,
             (unsigned)desired);
      msg_min = desired;
    }

    uint16_t msg_dedicated = st->profile->dedicated_prb;
    if (msg_dedicated > desired) {
      printf(LOG_PREFIX " warning: clipped fixed dedicated for slice=%s"
             " dedicated=%u target_max=%u to keep dedicated<=max\n",
             st->profile->profile_name,
             (unsigned)msg_dedicated,
             (unsigned)desired);
      msg_dedicated = desired;
    }

    printf(LOG_PREFIX " decision: cycle=%" PRIu64
           " slice=%s current_max=%u target_max=%u delta=%u hysteresis_ok=%d"
           " cooldown_ok=%d cooldown_age_ms=%" PRId64 " initial=%d reason=%s\n",
           cycle,
           st->profile->profile_name,
           (unsigned)previous,
           (unsigned)desired,
           (unsigned)diff,
           hysteresis_ready ? 1 : 0,
           cooldown_ready ? 1 : 0,
           st->has_ctrl && ctrl_age_us >= 0 ? ctrl_age_us / 1000 : -1,
           st->has_ctrl ? 0 : 1,
           should_send ? "send" : skip_reason);

    if (!st->has_ctrl) {
      printf(LOG_PREFIX " initial allocation: cycle=%" PRIu64
             " slice=%s target_max=%u current_default_max=%u\n",
             cycle,
             st->profile->profile_name,
             (unsigned)desired,
             (unsigned)previous);
    }

    printf(LOG_PREFIX " allocation_before: cycle=%" PRIu64
           " slice=%s ue_key=%" PRIu64 " current_max=%u target_max=%u diff=%u"
           " msg_min=%u msg_max=%u msg_dedicated=%u\n",
           cycle,
           st->profile->profile_name,
           g_observed_ues[u].key,
           (unsigned)previous,
           (unsigned)desired,
           (unsigned)diff,
           (unsigned)msg_min,
           (unsigned)desired,
           (unsigned)msg_dedicated);

    if (!should_send) {
      printf(LOG_PREFIX " skip control send: cycle=%" PRIu64
             " slice=%s ue_key=%" PRIu64
             " reason=%s target_max=%u current_max=%u delta=%u"
             " hysteresis_ok=%d cooldown_ok=%d cooldown_age_ms=%" PRId64 "\n",
             cycle,
             st->profile->profile_name,
             g_observed_ues[u].key,
             skip_reason,
             (unsigned)desired,
             (unsigned)previous,
             (unsigned)diff,
             hysteresis_ready ? 1 : 0,
             cooldown_ready ? 1 : 0,
             st->has_ctrl && ctrl_age_us >= 0 ? ctrl_age_us / 1000 : -1);
      printf(LOG_PREFIX " allocation_after: cycle=%" PRIu64
             " slice=%s current_max=%u target_max=%u sent=0 reason=%s\n",
             cycle,
             st->profile->profile_name,
             (unsigned)previous,
             (unsigned)desired,
             skip_reason);
      candidate_for_slice[idx] = true;
      continue;
    }

    control_candidate_t* c = &candidates[n++];
    memset(c, 0, sizeof(*c));
    c->valid = true;
    c->slice_idx = idx;
    c->ue_key = g_observed_ues[u].key;
    c->desired_prb = desired;
    c->previous_prb = previous;
    c->obs.valid = true;
    c->obs.key = g_observed_ues[u].key;
    c->obs.profile = *st->profile;
    /*
     * The local OAI scheduler currently consumes min/max from the stored RC
     * policy. Therefore the closed-loop target is applied to max_prb, not
     * dedicated_prb. dedicated_prb is kept fixed from the slice profile and
     * only clipped when needed to keep a valid min/dedicated/max relation.
     */
    c->obs.profile.min_prb = msg_min;
    c->obs.profile.max_prb = desired;
    c->obs.profile.dedicated_prb = msg_dedicated;
    c->obs.ue_id = cp_ue_id_e2sm(&g_observed_ues[u].ue_id);

    candidate_for_slice[idx] = true;
  }

  for (int i = 0; i < NUM_SLICE_PROFILES; i++) {
    if (!eligible[i] || observed_for_slice[i])
      continue;

    slice_state_t const* st = &g_slice_states[i];
    printf(LOG_PREFIX " skip control send: cycle=%" PRIu64
           " slice=%s reason=no observed UE target_max=%u current_max=%u\n",
           cycle,
           st->profile->profile_name,
           (unsigned)target[i],
           (unsigned)previous_prb[i]);
    printf(LOG_PREFIX " allocation_after: cycle=%" PRIu64
           " slice=%s current_max=%u target_max=%u sent=0 reason=no observed UE\n",
           cycle,
           st->profile->profile_name,
           (unsigned)previous_prb[i],
           (unsigned)target[i]);
  }

  pthread_mutex_unlock(&mtx);

  return n;
}

static
void mark_control_sent(control_candidate_t const* c, int64_t now_us)
{
  assert(c != NULL);
  assert(c->slice_idx >= 0 && c->slice_idx < NUM_SLICE_PROFILES);

  pthread_mutex_lock(&mtx);
  g_slice_states[c->slice_idx].has_ctrl = true;
  g_slice_states[c->slice_idx].last_sent_max_prb = c->desired_prb;
  g_slice_states[c->slice_idx].last_ctrl_us = now_us;
  pthread_mutex_unlock(&mtx);
}

static
void free_control_candidate(control_candidate_t* c)
{
  assert(c != NULL);

  if (c->valid) {
    free_ue_id_e2sm(&c->obs.ue_id);
    c->valid = false;
  }
}

static
void* run_closed_loop_prb_control(void* data)
{
  control_loop_args_t* args = data;
  assert(args != NULL);
  assert(args->nodes != NULL);

  int const RC_ran_function = 3;
  uint64_t cycle = 0;

  printf(LOG_PREFIX " closed-loop PRB control started period_ms=%u"
         " cooldown_ms=%u hysteresis_prb=%u stale_timeout_ms=%u total_budget=%u"
         " demand_mode=dl_only verify_force_prb_max=%u\n",
         (unsigned)CONTROL_LOOP_PERIOD_MS,
         (unsigned)CONTROL_COOLDOWN_MS,
         (unsigned)CONTROL_HYSTERESIS_PRB,
         (unsigned)KPI_STALE_TIMEOUT_MS,
         (unsigned)TOTAL_PRB_BUDGET_RATIO,
         (unsigned)VERIFY_FORCE_PRB_MAX_RATIO);

  while (!atomic_load(&g_control_stop)) {
    control_candidate_t candidates[NUM_SLICE_PROFILES] = {0};
    int64_t const now_us = time_now_us();

    printf(LOG_PREFIX " cycle=%" PRIu64 " start\n", cycle);

    size_t const n_candidates =
        build_control_candidates(candidates, NUM_SLICE_PROFILES, now_us, cycle);

    if (n_candidates == 0) {
      printf(LOG_PREFIX " control_send: cycle=%" PRIu64
             " skipped slices=0 reason=no significant change, cooldown active, or no observed UE\n",
             cycle);
    }

    if (args->nodes->len == 0 && n_candidates > 0) {
      printf(LOG_PREFIX " control_send: cycle=%" PRIu64
             " skipped slices=%zu reason=no E2 node/control handle\n",
             cycle,
             n_candidates);
      for (size_t c_idx = 0; c_idx < n_candidates; c_idx++) {
        control_candidate_t* c = &candidates[c_idx];
        printf(LOG_PREFIX " control_send_result: cycle=%" PRIu64
               " slice=%s sent=0 success=0 reason=no E2 node/control handle\n",
               cycle,
               c->obs.profile.profile_name);
        printf(LOG_PREFIX " allocation_after: cycle=%" PRIu64
               " slice=%s current_max=%u target_max=%u sent=0 reason=no E2 node/control handle\n",
               cycle,
               c->obs.profile.profile_name,
               (unsigned)c->previous_prb,
               (unsigned)c->desired_prb);
      }
    }

    for (size_t i = 0; i < args->nodes->len; ++i) {
      e2_node_connected_xapp_t* n = &args->nodes->n[i];
      size_t idx = 0;

      if (!find_sm_idx_optional(n->rf, n->len_rf, eq_sm, RC_ran_function, &idx) ||
          n->rf[idx].defn.type != RC_RAN_FUNC_DEF_E ||
          n->rf[idx].defn.rc.ctrl == NULL) {
        printf(LOG_PREFIX " control_send: cycle=%" PRIu64
               " node=%zu skipped slices=%zu reason=no E2 node/control handle\n",
               cycle,
               i,
               n_candidates);
        for (size_t c_idx = 0; c_idx < n_candidates; c_idx++) {
          control_candidate_t* c = &candidates[c_idx];
          printf(LOG_PREFIX " control_send_result: cycle=%" PRIu64
                 " slice=%s sent=0 success=0 reason=no E2 node/control handle\n",
                 cycle,
                 c->obs.profile.profile_name);
          printf(LOG_PREFIX " allocation_after: cycle=%" PRIu64
                 " slice=%s current_max=%u target_max=%u sent=0 reason=no E2 node/control handle\n",
                 cycle,
                 c->obs.profile.profile_name,
                 (unsigned)c->previous_prb,
                 (unsigned)c->desired_prb);
        }
        continue;
      }

      if (!rc_supports_slice_prb_quota(n->rf[idx].defn.rc.ctrl)) {
        printf(LOG_PREFIX " control_send: cycle=%" PRIu64
               " node=%zu skipped slices=%zu reason=unsupported control message\n",
               cycle,
               i,
               n_candidates);
        for (size_t c_idx = 0; c_idx < n_candidates; c_idx++) {
          control_candidate_t* c = &candidates[c_idx];
          printf(LOG_PREFIX " control_send_result: cycle=%" PRIu64
                 " slice=%s sent=0 success=0 reason=unsupported control message\n",
                 cycle,
                 c->obs.profile.profile_name);
          printf(LOG_PREFIX " allocation_after: cycle=%" PRIu64
                 " slice=%s current_max=%u target_max=%u sent=0 reason=unsupported control message\n",
                 cycle,
                 c->obs.profile.profile_name,
                 (unsigned)c->previous_prb,
                 (unsigned)c->desired_prb);
        }
        continue;
      }

      for (size_t c_idx = 0; c_idx < n_candidates; c_idx++) {
        control_candidate_t* c = &candidates[c_idx];

        printf(LOG_PREFIX " control_send: cycle=%" PRIu64
               " node=%zu slices=1 slice=%s ue_key=%" PRIu64
               " min=%u max=%u dedicated=%u previous=%u\n",
               cycle,
               i,
               c->obs.profile.profile_name,
               c->ue_key,
               (unsigned)c->obs.profile.min_prb,
               (unsigned)c->obs.profile.max_prb,
               (unsigned)c->obs.profile.dedicated_prb,
               (unsigned)c->previous_prb);

        printf("[RIC-XAPP][CTRL] send cycle=%" PRIu64
               " node=%zu profile=%s ue_key=%" PRIu64
               " min=%u max=%u dedicated=%u previous=%u\n",
               cycle,
               i,
               c->obs.profile.profile_name,
               c->ue_key,
               (unsigned)c->obs.profile.min_prb,
               (unsigned)c->obs.profile.max_prb,
               (unsigned)c->obs.profile.dedicated_prb,
               (unsigned)c->previous_prb);

        rc_ctrl_req_data_t rc_ctrl =
            gen_rc_ctrl_msg_style_2(n->rf[idx].defn.rc.ctrl, &c->obs);

        printf("[RIC-XAPP][CTRL] calling control_sm_xapp_api()\n");
        sm_ans_xapp_t ans = control_sm_xapp_api(&n->id, RC_ran_function, &rc_ctrl);
        printf("[RIC-XAPP][CTRL] control_sm_xapp_api() returned success=%d\n",
               ans.success ? 1 : 0);

        free_rc_ctrl_req_data(&rc_ctrl);
        if (ans.success)
          mark_control_sent(c, time_now_us());

        printf(LOG_PREFIX " control_send_result: cycle=%" PRIu64
               " slice=%s sent=%d success=%d reason=%s\n",
               cycle,
               c->obs.profile.profile_name,
               ans.success ? 1 : 0,
               ans.success ? 1 : 0,
               ans.success ? "ok" :
                   (ans.u.reason != NULL ? ans.u.reason : "control_sm_xapp_api failed"));

        printf(LOG_PREFIX " allocation_after: cycle=%" PRIu64
               " slice=%s current_max=%u target_max=%u sent=%d\n",
               cycle,
               c->obs.profile.profile_name,
               (unsigned)(ans.success ? c->desired_prb : c->previous_prb),
               (unsigned)c->desired_prb,
               ans.success ? 1 : 0);

        printf("[RIC-XAPP][CTRL] sent profile=%s max=%u dedicated=%u success=%d and freed RC control message\n",
               c->obs.profile.profile_name,
               (unsigned)c->obs.profile.max_prb,
               (unsigned)c->obs.profile.dedicated_prb,
               ans.success ? 1 : 0);
      }
    }

    for (size_t c_idx = 0; c_idx < n_candidates; c_idx++)
      free_control_candidate(&candidates[c_idx]);

    cycle++;
    printf(LOG_PREFIX " cycle=%" PRIu64 " sleep_ms=%u\n",
           cycle - 1,
           (unsigned)CONTROL_LOOP_PERIOD_MS);
    usleep(CONTROL_LOOP_PERIOD_US);
  }

  printf(LOG_PREFIX " closed-loop PRB control stopped\n");
  return NULL;
}

/* --------------------------------------------------------------- */
/* Main                                                            */
/* --------------------------------------------------------------- */

int main(int argc, char* argv[])
{
  fr_args_t args = init_fr_args(argc, argv);

  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  assert(nodes.len > 0);

  printf("[RC-ACTION6-XAPP] Connected E2 nodes = %zu\n", (size_t)nodes.len);

  pthread_mutexattr_t attr = {0};
  int rc = pthread_mutex_init(&mtx, &attr);
  assert(rc == 0);

  sm_ans_xapp_t* hndl_sst1 = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  sm_ans_xapp_t* hndl_sst2 = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  sm_ans_xapp_t* hndl_sst3 = calloc(nodes.len, sizeof(sm_ans_xapp_t));
  assert(hndl_sst1 != NULL && hndl_sst2 != NULL && hndl_sst3 != NULL);

  /* ------------------------------------------------------------ */
  /* START KPM                                                    */
  /* ------------------------------------------------------------ */
  int const KPM_ran_function = 2;

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];
    size_t const idx = find_sm_idx(n->rf, n->len_rf, eq_sm, KPM_ran_function);

    assert(n->rf[idx].defn.type == KPM_RAN_FUNC_DEF_E);

    if (n->rf[idx].defn.kpm.ric_report_style_list != NULL) {
      printf("[RIC-XAPP][KPM] node=%zu subscribe slice1 snssai=(%u,%02x%02x%02x)\n",
             i,
             g_profile_sst1.sst,
             g_profile_sst1.sd[0], g_profile_sst1.sd[1], g_profile_sst1.sd[2]);

      printf("[RIC-XAPP][KPM] node=%zu subscribe slice2 snssai=(%u,%02x%02x%02x)\n",
             i,
             g_profile_sst2.sst,
             g_profile_sst2.sd[0], g_profile_sst2.sd[1], g_profile_sst2.sd[2]);

      printf("[RIC-XAPP][KPM] node=%zu subscribe slice3 snssai=(%u,%02x%02x%02x)\n",
             i,
             g_profile_sst3.sst,
             g_profile_sst3.sd[0], g_profile_sst3.sd[1], g_profile_sst3.sd[2]);

      kpm_sub_data_t sub_sst1 = gen_kpm_subs_for_profile(&n->rf[idx].defn.kpm, &g_profile_sst1);
      kpm_sub_data_t sub_sst2 = gen_kpm_subs_for_profile(&n->rf[idx].defn.kpm, &g_profile_sst2);
      kpm_sub_data_t sub_sst3 = gen_kpm_subs_for_profile(&n->rf[idx].defn.kpm, &g_profile_sst3);

      hndl_sst1[i] = report_sm_xapp_api(&n->id, KPM_ran_function, &sub_sst1, sm_cb_kpm_sst1);
      hndl_sst2[i] = report_sm_xapp_api(&n->id, KPM_ran_function, &sub_sst2, sm_cb_kpm_sst2);
      hndl_sst3[i] = report_sm_xapp_api(&n->id, KPM_ran_function, &sub_sst3, sm_cb_kpm_sst3);

      assert(hndl_sst1[i].success == true);
      assert(hndl_sst2[i].success == true);
      assert(hndl_sst3[i].success == true);

      free_kpm_sub_data(&sub_sst1);
      free_kpm_sub_data(&sub_sst2);
      free_kpm_sub_data(&sub_sst3);
    }
  }

  pthread_t ctrl_thread = {0};
  control_loop_args_t ctrl_args = {.nodes = &nodes};
  atomic_store(&g_control_stop, false);

  rc = pthread_create(&ctrl_thread, NULL, run_closed_loop_prb_control, &ctrl_args);
  assert(rc == 0);

  xapp_wait_end_api();

  atomic_store(&g_control_stop, true);
  rc = pthread_join(ctrl_thread, NULL);
  assert(rc == 0);

  for (size_t i = 0; i < nodes.len; ++i) {
    if (hndl_sst1[i].success == true)
      rm_report_sm_xapp_api(hndl_sst1[i].u.handle);
    if (hndl_sst2[i].success == true)
      rm_report_sm_xapp_api(hndl_sst2[i].u.handle);
    if (hndl_sst3[i].success == true)
      rm_report_sm_xapp_api(hndl_sst3[i].u.handle);
  }

  free(hndl_sst1);
  free(hndl_sst2);
  free(hndl_sst3);

  pthread_mutex_lock(&mtx);
  {
    for (size_t i = 0; i < MAX_OBSERVED_UE; i++) {
      if (g_observed_ues[i].valid) {
        free_ue_id_e2sm(&g_observed_ues[i].ue_id);
        g_observed_ues[i].valid = false;
      }
    }
  }
  pthread_mutex_unlock(&mtx);

  while (try_stop_xapp_api() == false)
    usleep(1000);

  free_e2_node_arr_xapp(&nodes);

  rc = pthread_mutex_destroy(&mtx);
  assert(rc == 0);

  printf("[RC-ACTION6-XAPP] run SUCCESSFULLY\n");
  return 0;
}

