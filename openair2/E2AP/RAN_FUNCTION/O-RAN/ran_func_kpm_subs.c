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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#include "ran_func_kpm_subs.h"

#include <search.h>

typedef struct kpm_counter_state {
  bool valid;
  uint64_t ue_key;

  bool pdcp_dl_valid;
  bool pdcp_ul_valid;
  uint32_t pdcp_dl_bytes;
  uint32_t pdcp_ul_bytes;

  bool rlc_dl_valid;
  bool rlc_ul_valid;
  uint32_t rlc_dl_bytes;
  uint32_t rlc_ul_bytes;

  bool prb_dl_valid;
  bool prb_ul_valid;
  uint32_t prb_dl;
  uint32_t prb_ul;
} kpm_counter_state_t;

static kpm_counter_state_t kpm_counter_states[MAX_MOBILES_PER_GNB] = {0};

static uint64_t get_ue_counter_key(cudu_ue_info_pair_t ue_info)
{
  if (ue_info.ue != NULL)
    return (1ULL << 63) | ue_info.ue->rnti;

  return ue_info.rrc_ue_id;
}

static kpm_counter_state_t *get_counter_state(cudu_ue_info_pair_t ue_info)
{
  const uint64_t key = get_ue_counter_key(ue_info);

  for (size_t i = 0; i < MAX_MOBILES_PER_GNB; i++) {
    if (kpm_counter_states[i].valid && kpm_counter_states[i].ue_key == key)
      return &kpm_counter_states[i];
  }

  for (size_t i = 0; i < MAX_MOBILES_PER_GNB; i++) {
    if (!kpm_counter_states[i].valid) {
      kpm_counter_states[i].valid = true;
      kpm_counter_states[i].ue_key = key;
      return &kpm_counter_states[i];
    }
  }

  assert(false && "KPM counter table full");
  return NULL;
}

static uint32_t counter_delta_u32_or_zero(uint32_t cur, uint32_t *last, bool *last_valid)
{
  assert(last != NULL);
  assert(last_valid != NULL);

  if (!*last_valid) {
    *last = cur;
    *last_valid = true;
    return 0;
  }

  const uint32_t delta = cur - *last;
  *last = cur;
  return delta;
}

static uint32_t clamp_u64_to_u32(uint64_t value)
{
  return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

static uint32_t bytes_to_kbits_u32(uint32_t bytes)
{
  return clamp_u64_to_u32(((uint64_t)bytes * 8U) / 1000U);
}

#if defined (NGRAN_GNB_DU)
static uint32_t prb_usage_percent(uint32_t used_prbs,
                                  uint16_t bwp_size,
                                  uint8_t scs,
                                  uint32_t gran_period_ms)
{
  if (bwp_size == 0 || gran_period_ms == 0 || scs >= 5)
    return 0;

  const uint64_t slots_in_period = (uint64_t)gran_period_ms << scs;
  const uint64_t available_prbs = (uint64_t)bwp_size * slots_in_period;
  if (available_prbs == 0)
    return 0;

  uint64_t usage = ((uint64_t)used_prbs * 100U + available_prbs / 2U) / available_prbs;
  if (usage > 100U)
    usage = 100U;

  return (uint32_t)usage;
}
#endif

static nr_pdcp_statistics_t get_pdcp_stats_per_drb(const uint32_t rrc_ue_id, const int rb_id)
{
  nr_pdcp_statistics_t pdcp = {0};
  const int srb_flag = 0;

  // Get PDCP stats for specific DRB
  const bool rc = nr_pdcp_get_statistics(rrc_ue_id, srb_flag, rb_id, &pdcp);
  assert(rc == true && "Cannot get PDCP stats\n");

  return pdcp;
}

/* 3GPP TS 28.552 - section 5.1.2.1.1.1
  note: this measurement is calculated as per spec */
static meas_record_lst_t fill_DRB_PdcpSduVolumeDL(__attribute__((unused))uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  (void)ue_idx;

  // Get PDCP stats per DRB
  const int rb_id = 1;  // at the moment, only 1 DRB is supported
  nr_pdcp_statistics_t pdcp = get_pdcp_stats_per_drb(ue_info.rrc_ue_id, rb_id);

  meas_record.value = INTEGER_MEAS_VALUE;

  kpm_counter_state_t *state = get_counter_state(ue_info);
  const uint32_t delta = counter_delta_u32_or_zero(pdcp.rxsdu_bytes,
                                                   &state->pdcp_dl_bytes,
                                                   &state->pdcp_dl_valid);

  // Get DL data volume delivered to PDCP layer during this reporting period.
  meas_record.int_val = bytes_to_kbits_u32(delta);   // [kb]

  return meas_record;
}

/* 3GPP TS 28.552 - section 5.1.2.1.2.1
  note: this measurement is calculated as per spec */
static meas_record_lst_t fill_DRB_PdcpSduVolumeUL(__attribute__((unused))uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  (void)ue_idx;

  // Get PDCP stats per DRB
  const int rb_id = 1;  // at the moment, only 1 DRB is supported
  nr_pdcp_statistics_t pdcp = get_pdcp_stats_per_drb(ue_info.rrc_ue_id, rb_id);

  meas_record.value = INTEGER_MEAS_VALUE;

  kpm_counter_state_t *state = get_counter_state(ue_info);
  const uint32_t delta = counter_delta_u32_or_zero(pdcp.txsdu_bytes,
                                                   &state->pdcp_ul_bytes,
                                                   &state->pdcp_ul_valid);

  // Get UL data volume delivered from PDCP layer during this reporting period.
  meas_record.int_val = bytes_to_kbits_u32(delta);   // [kb]

  return meas_record;
}

#if defined (NGRAN_GNB_DU)
static nr_rlc_statistics_t get_rlc_stats_per_drb(const rnti_t rnti, const int rb_id)
{
  nr_rlc_statistics_t rlc = {0};
  const int srb_flag = 0;

  // Get RLC stats for specific DRB
  const bool rc = nr_rlc_get_statistics(rnti, srb_flag, rb_id, &rlc);
  assert(rc == true && "Cannot get RLC stats\n");

  // Activate average sojourn time at the RLC buffer for specific DRB
  nr_rlc_activate_avg_time_to_tx(rnti, rb_id+3, 1);
  
  return rlc;  
}

/* 3GPP TS 28.552 - section 5.1.3.3.3
  note: by default this measurement is calculated for previous 100ms (openair2/LAYER2/nr_rlc/nr_rlc_entity.c:118, 173, 213); please, update according to your needs */
static meas_record_lst_t fill_DRB_RlcSduDelayDl(__attribute__((unused))uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, __attribute__((unused))const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  
  // Get RLC stats per DRB
  const int rb_id = 1;  // at the moment, only 1 DRB is supported
  nr_rlc_statistics_t rlc = get_rlc_stats_per_drb(ue_info.ue->rnti, rb_id);

  meas_record.value = REAL_MEAS_VALUE;

  // Get the value of sojourn time at the RLC buffer
  meas_record.real_val = rlc.txsdu_avg_time_to_tx;  // [μs]

  return meas_record;
}

/* 3GPP TS 28.552 - section 5.1.1.3.1
  note: per spec, average UE throughput in DL (taken into consideration values from all UEs, and averaged)
        here calculated as: UE specific throughput in DL */
static meas_record_lst_t fill_DRB_UEThpDl(uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  (void)ue_idx;
  
  // Get RLC stats per DRB
  const int rb_id = 1;  // at the moment, only 1 DRB is supported
  nr_rlc_statistics_t rlc = get_rlc_stats_per_drb(ue_info.ue->rnti, rb_id);
  meas_record.value = REAL_MEAS_VALUE;

  kpm_counter_state_t *state = get_counter_state(ue_info);
  const uint32_t delta = counter_delta_u32_or_zero(rlc.txpdu_bytes,
                                                   &state->rlc_dl_bytes,
                                                   &state->rlc_dl_valid);

  // Calculate DL throughput during this reporting period.
  meas_record.real_val = gran_period_ms == 0 ? 0.0 : (double)delta * 8.0 / gran_period_ms;  // [kbps]

  return meas_record;
}

/* 3GPP TS 28.552 - section 5.1.1.3.3
  note: per spec, average UE throughput in UL (taken into consideration values from all UEs, and averaged)
        here calculated as: UE specific throughput in UL */
static meas_record_lst_t fill_DRB_UEThpUl(uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  (void)ue_idx;
  
  // Get RLC stats per DRB
  const int rb_id = 1;  // at the moment, only 1 DRB is supported
  nr_rlc_statistics_t rlc = get_rlc_stats_per_drb(ue_info.ue->rnti, rb_id);

  meas_record.value = REAL_MEAS_VALUE;

  kpm_counter_state_t *state = get_counter_state(ue_info);
  const uint32_t delta = counter_delta_u32_or_zero(rlc.rxpdu_bytes,
                                                   &state->rlc_ul_bytes,
                                                   &state->rlc_ul_valid);

  // Calculate UL throughput during this reporting period.
  meas_record.real_val = gran_period_ms == 0 ? 0.0 : (double)delta * 8.0 / gran_period_ms;  // [kbps]
  
  return meas_record;
}

/* 3GPP TS 28.552 - section 5.1.1.2.1
  note: per spec, DL PRB usage [%] = (total used PRBs for DL traffic / total available PRBs for DL traffic) * 100 */
static meas_record_lst_t fill_RRU_PrbTotDl(__attribute__((unused))uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  (void)ue_idx;
  
  meas_record.value = INTEGER_MEAS_VALUE;

  kpm_counter_state_t *state = get_counter_state(ue_info);
  const uint32_t delta = counter_delta_u32_or_zero(ue_info.ue->mac_stats.dl.total_rbs,
                                                   &state->prb_dl,
                                                   &state->prb_dl_valid);

  // Get DL PRB usage for this reporting period.
  meas_record.int_val = prb_usage_percent(delta,
                                          ue_info.ue->current_DL_BWP.BWPSize,
                                          ue_info.ue->current_DL_BWP.scs,
                                          gran_period_ms);   // [%]

  return meas_record;
}

/* 3GPP TS 28.552 - section 5.1.1.2.2
  note: per spec, UL PRB usage [%] = (total used PRBs for UL traffic / total available PRBs for UL traffic) * 100 */
static meas_record_lst_t fill_RRU_PrbTotUl(__attribute__((unused))uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  meas_record_lst_t meas_record = {0};
  (void)ue_idx;

  meas_record.value = INTEGER_MEAS_VALUE;

  kpm_counter_state_t *state = get_counter_state(ue_info);
  const uint32_t delta = counter_delta_u32_or_zero(ue_info.ue->mac_stats.ul.total_rbs,
                                                   &state->prb_ul,
                                                   &state->prb_ul_valid);

  // Get UL PRB usage for this reporting period.
  meas_record.int_val = prb_usage_percent(delta,
                                          ue_info.ue->current_UL_BWP.BWPSize,
                                          ue_info.ue->current_UL_BWP.scs,
                                          gran_period_ms);   // [%]

  return meas_record;
}
#endif

static kv_measure_t lst_measure[] = {
  {.key = "DRB.PdcpSduVolumeDL", .value = fill_DRB_PdcpSduVolumeDL }, 
  {.key = "DRB.PdcpSduVolumeUL", .value = fill_DRB_PdcpSduVolumeUL },
#if defined (NGRAN_GNB_DU)
  {.key = "DRB.RlcSduDelayDl", .value =  fill_DRB_RlcSduDelayDl }, 
  {.key = "DRB.UEThpDl", .value =  fill_DRB_UEThpDl }, 
  {.key = "DRB.UEThpUl", .value =  fill_DRB_UEThpUl }, 
  {.key = "RRU.PrbTotDl", .value =  fill_RRU_PrbTotDl }, 
  {.key = "RRU.PrbTotUl", .value =  fill_RRU_PrbTotUl }, 
#endif
}; 

void init_kpm_subs_data(void)
{
  const size_t ht_len = sizeof(lst_measure) / sizeof(lst_measure[0]);
  hcreate(ht_len);

  ENTRY kv_pair;

  for (size_t i = 0; i < ht_len; i++) {
    kv_pair.key = lst_measure[i].key;
    kv_pair.data = &lst_measure[i];
    hsearch(kv_pair, ENTER);
  }
}

meas_record_lst_t get_kpm_meas_value(char* kpm_meas_name, uint32_t gran_period_ms, cudu_ue_info_pair_t ue_info, const size_t ue_idx)
{
  assert(kpm_meas_name != NULL);

  ENTRY search_entry = {.key = kpm_meas_name};
  ENTRY *found_entry = hsearch(search_entry, FIND);
  assert(found_entry != NULL && "Unsupported KPM measurement name");

  kv_measure_t *kv_found = (kv_measure_t *)found_entry->data;
  meas_record_lst_t meas_record = kv_found->value(gran_period_ms, ue_info, ue_idx);

  return meas_record;
}

