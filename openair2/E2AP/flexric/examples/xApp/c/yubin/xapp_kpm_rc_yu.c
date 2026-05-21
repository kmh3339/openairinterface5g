/*
 * RC Control-only xApp
 * - No KPM monitoring
 * - No Redis read
 * - Receive ue_key and PRB policy from environment variables
 * - Send RC Style 2 / Action 6 slice-level PRB quota once
 * - Then exit
 *
 * Run example:
 * S1_UE=1 S1_MIN=10 S1_MAX=90 S1_DED=75 \
 * S2_UE=3 S2_MIN=5  S2_MAX=10 S2_DED=7  \
 * S3_UE=4 S3_MIN=5  S3_MAX=20 S3_DED=10 \
 * ./xapp_rc_prb_ctrl_yu
 */

#include "../../../../src/xApp/e42_xapp_api.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_struct.h"
#include "../../../../src/sm/rc_sm/ie/ir/ran_param_list.h"
#include "../../../../src/util/e.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

typedef struct {
  char const* profile_name;
  uint8_t sst;
  uint8_t sd[3];
  uint16_t min_prb;
  uint16_t max_prb;
  uint16_t dedicated_prb;
  uint64_t ue_key;
  uint8_t plmn[3];
} slice_profile_t;

static slice_profile_t g_profile_sst1 = {
  .profile_name = "slice-sst1",
  .sst = 1,
  .sd = {0x00, 0x00, 0x03},
  .plmn = {0x00, 0xF1, 0x10},
};

static slice_profile_t g_profile_sst2 = {
  .profile_name = "slice-sst2",
  .sst = 2,
  .sd = {0x00, 0x00, 0x01},
  .plmn = {0x00, 0xF1, 0x10},
};

static slice_profile_t g_profile_sst3 = {
  .profile_name = "slice-sst3",
  .sst = 3,
  .sd = {0x00, 0x00, 0x02},
  .plmn = {0x00, 0xF1, 0x10},
};

typedef enum {
  Slice_level_PRB_quota_7_6_3_1 = 6,
} rc_ctrl_service_style_2_e;

typedef enum {
  RRM_POLICY_RATIO_LIST_8_4_3_6       = 1,
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

static uint64_t getenv_u64(char const* name, uint64_t def)
{
  char const* v = getenv(name);
  if (v == NULL)
    return def;

  return strtoull(v, NULL, 10);
}

static uint16_t getenv_u16(char const* name, uint16_t def)
{
  char const* v = getenv(name);
  if (v == NULL)
    return def;

  return (uint16_t)atoi(v);
}

static void load_policy_from_env(void)
{
  g_profile_sst1.ue_key = getenv_u64("S1_UE", 1);
  g_profile_sst1.min_prb = getenv_u16("S1_MIN", 10);
  g_profile_sst1.max_prb = getenv_u16("S1_MAX", 90);
  g_profile_sst1.dedicated_prb = getenv_u16("S1_DED", 75);

  g_profile_sst2.ue_key = getenv_u64("S2_UE", 3);
  g_profile_sst2.min_prb = getenv_u16("S2_MIN", 5);
  g_profile_sst2.max_prb = getenv_u16("S2_MAX", 10);
  g_profile_sst2.dedicated_prb = getenv_u16("S2_DED", 7);

  g_profile_sst3.ue_key = getenv_u64("S3_UE", 4);
  g_profile_sst3.min_prb = getenv_u16("S3_MIN", 5);
  g_profile_sst3.max_prb = getenv_u16("S3_MAX", 20);
  g_profile_sst3.dedicated_prb = getenv_u16("S3_DED", 10);

  printf("[RC-PRB-CTRL-XAPP] loaded policy from env\n");
  printf("  slice1: ue=%lu min=%u max=%u ded=%u\n",
         g_profile_sst1.ue_key,
         g_profile_sst1.min_prb,
         g_profile_sst1.max_prb,
         g_profile_sst1.dedicated_prb);

  printf("  slice2: ue=%lu min=%u max=%u ded=%u\n",
         g_profile_sst2.ue_key,
         g_profile_sst2.min_prb,
         g_profile_sst2.max_prb,
         g_profile_sst2.dedicated_prb);

  printf("  slice3: ue=%lu min=%u max=%u ded=%u\n",
         g_profile_sst3.ue_key,
         g_profile_sst3.min_prb,
         g_profile_sst3.max_prb,
         g_profile_sst3.dedicated_prb);
}

static ue_id_e2sm_t make_ue_id_from_key(uint64_t ue_key)
{
  ue_id_e2sm_t ue = {0};

  ue.type = GNB_UE_ID_E2SM;
  ue.gnb.amf_ue_ngap_id = ue_key;
  ue.gnb.gnb_cu_ue_f1ap_lst = NULL;
  ue.gnb.gnb_cu_ue_f1ap_lst_len = 0;

  ue.gnb.ran_ue_id = calloc(1, sizeof(uint64_t));
  assert(ue.gnb.ran_ue_id != NULL);
  *ue.gnb.ran_ue_id = ue_key;

  return ue;
}

static void free_local_ue_id(ue_id_e2sm_t* ue)
{
  if (ue != NULL && ue->type == GNB_UE_ID_E2SM && ue->gnb.ran_ue_id != NULL) {
    free(ue->gnb.ran_ue_id);
    ue->gnb.ran_ue_id = NULL;
  }
}

static seq_ran_param_t fill_int_flag_true_param(uint32_t ran_param_id, int64_t value)
{
  seq_ran_param_t p = {0};

  p.ran_param_id = ran_param_id;
  p.ran_param_val.type = ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.flag_true = calloc(1, sizeof(ran_parameter_value_t));
  assert(p.ran_param_val.flag_true != NULL);

  p.ran_param_val.flag_true->type = INTEGER_RAN_PARAMETER_VALUE;
  p.ran_param_val.flag_true->int_ran = value;

  return p;
}

static seq_ran_param_t fill_int_flag_false_param(uint32_t ran_param_id, int64_t value)
{
  seq_ran_param_t p = {0};

  p.ran_param_id = ran_param_id;
  p.ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.flag_false = calloc(1, sizeof(ran_parameter_value_t));
  assert(p.ran_param_val.flag_false != NULL);

  p.ran_param_val.flag_false->type = INTEGER_RAN_PARAMETER_VALUE;
  p.ran_param_val.flag_false->int_ran = value;

  return p;
}

static seq_ran_param_t fill_octet_flag_false_param(uint32_t ran_param_id,
                                                   uint8_t const* buf,
                                                   size_t len)
{
  seq_ran_param_t p = {0};

  p.ran_param_id = ran_param_id;
  p.ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.flag_false = calloc(1, sizeof(ran_parameter_value_t));
  assert(p.ran_param_val.flag_false != NULL);

  p.ran_param_val.flag_false->type = OCTET_STRING_RAN_PARAMETER_VALUE;
  p.ran_param_val.flag_false->octet_str_ran.buf = calloc(len, sizeof(uint8_t));
  assert(p.ran_param_val.flag_false->octet_str_ran.buf != NULL);

  memcpy(p.ran_param_val.flag_false->octet_str_ran.buf, buf, len);
  p.ran_param_val.flag_false->octet_str_ran.len = len;

  return p;
}

static seq_ran_param_t fill_snssai_param(slice_profile_t const* profile)
{
  seq_ran_param_t p = {0};

  p.ran_param_id = S_NSSAI_8_4_3_6;
  p.ran_param_val.type = STRUCTURE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.strct = calloc(1, sizeof(ran_param_struct_t));
  assert(p.ran_param_val.strct != NULL);

  p.ran_param_val.strct->sz_ran_param_struct = 2;
  p.ran_param_val.strct->ran_param_struct = calloc(2, sizeof(seq_ran_param_t));
  assert(p.ran_param_val.strct->ran_param_struct != NULL);

  p.ran_param_val.strct->ran_param_struct[0] =
      fill_int_flag_true_param(SST_8_4_3_6, profile->sst);

  p.ran_param_val.strct->ran_param_struct[1] =
      fill_octet_flag_false_param(SD_8_4_3_6, profile->sd, sizeof(profile->sd));

  return p;
}

static seq_ran_param_t fill_rrm_policy_member_list_param(slice_profile_t const* profile)
{
  seq_ran_param_t p = {0};

  p.ran_param_id = RRM_POLICY_MEMBER_LIST_8_4_3_6;
  p.ran_param_val.type = LIST_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.lst = calloc(1, sizeof(ran_param_list_t));
  assert(p.ran_param_val.lst != NULL);

  ran_param_list_t* list = p.ran_param_val.lst;
  list->sz_lst_ran_param = 1;
  list->lst_ran_param = calloc(1, sizeof(lst_ran_param_t));
  assert(list->lst_ran_param != NULL);

  lst_ran_param_t* item = &list->lst_ran_param[0];

  item->ran_param_struct.sz_ran_param_struct = 2;
  item->ran_param_struct.ran_param_struct = calloc(2, sizeof(seq_ran_param_t));
  assert(item->ran_param_struct.ran_param_struct != NULL);

  item->ran_param_struct.ran_param_struct[0] =
      fill_octet_flag_false_param(PLMN_IDENTITY_8_4_3_6,
                                  profile->plmn,
                                  sizeof(profile->plmn));

  item->ran_param_struct.ran_param_struct[1] =
      fill_snssai_param(profile);

  return p;
}

static seq_ran_param_t fill_rrm_policy_param(slice_profile_t const* profile)
{
  seq_ran_param_t p = {0};

  p.ran_param_id = RRM_POLICY_8_4_3_6;
  p.ran_param_val.type = STRUCTURE_RAN_PARAMETER_VAL_TYPE;
  p.ran_param_val.strct = calloc(1, sizeof(ran_param_struct_t));
  assert(p.ran_param_val.strct != NULL);

  p.ran_param_val.strct->sz_ran_param_struct = 1;
  p.ran_param_val.strct->ran_param_struct = calloc(1, sizeof(seq_ran_param_t));
  assert(p.ran_param_val.strct->ran_param_struct != NULL);

  p.ran_param_val.strct->ran_param_struct[0] =
      fill_rrm_policy_member_list_param(profile);

  return p;
}

static seq_ran_param_t fill_slice_level_prb_quota_param(slice_profile_t const* profile)
{
  seq_ran_param_t top = {0};

  top.ran_param_id = RRM_POLICY_RATIO_LIST_8_4_3_6;
  top.ran_param_val.type = LIST_RAN_PARAMETER_VAL_TYPE;
  top.ran_param_val.lst = calloc(1, sizeof(ran_param_list_t));
  assert(top.ran_param_val.lst != NULL);

  ran_param_list_t* ratio_list = top.ran_param_val.lst;
  ratio_list->sz_lst_ran_param = 1;
  ratio_list->lst_ran_param = calloc(1, sizeof(lst_ran_param_t));
  assert(ratio_list->lst_ran_param != NULL);

  lst_ran_param_t* item = &ratio_list->lst_ran_param[0];

  item->ran_param_struct.sz_ran_param_struct = 4;
  item->ran_param_struct.ran_param_struct = calloc(4, sizeof(seq_ran_param_t));
  assert(item->ran_param_struct.ran_param_struct != NULL);

  item->ran_param_struct.ran_param_struct[0] =
      fill_rrm_policy_param(profile);

  item->ran_param_struct.ran_param_struct[1] =
      fill_int_flag_false_param(MIN_PRB_POLICY_RATIO_8_4_3_6,
                                profile->min_prb);

  item->ran_param_struct.ran_param_struct[2] =
      fill_int_flag_false_param(MAX_PRB_POLICY_RATIO_8_4_3_6,
                                profile->max_prb);

  item->ran_param_struct.ran_param_struct[3] =
      fill_int_flag_false_param(DEDICATED_PRB_POLICY_RATIO_8_4_3_6,
                                profile->dedicated_prb);

  return top;
}

static void fill_rc_ctrl_act_style_2(seq_ctrl_act_2_t const* ctrl_act,
                                     size_t const sz,
                                     slice_profile_t const* profile,
                                     e2sm_rc_ctrl_hdr_frmt_1_t* hdr,
                                     e2sm_rc_ctrl_msg_frmt_1_t* msg)
{
  for (size_t i = 0; i < sz; i++) {
    if (cmp_str_ba("Slice-level PRB quota", ctrl_act[i].name) != 0)
      continue;

    hdr->ctrl_act_id = Slice_level_PRB_quota_7_6_3_1;

    msg->sz_ran_param = 1;
    msg->ran_param = calloc(1, sizeof(seq_ran_param_t));
    assert(msg->ran_param != NULL);

    msg->ran_param[0] = fill_slice_level_prb_quota_param(profile);
    return;
  }

  assert(false && "Slice-level PRB quota action not found");
}

static rc_ctrl_req_data_t gen_rc_ctrl_msg_style_2(ran_func_def_ctrl_t const* ran_func,
                                                  slice_profile_t const* profile,
                                                  ue_id_e2sm_t const* ue_id)
{
  rc_ctrl_req_data_t rc_ctrl = {0};

  for (size_t i = 0; i < ran_func->sz_seq_ctrl_style; i++) {
    if (cmp_str_ba("Radio Resource Allocation Control",
                   ran_func->seq_ctrl_style[i].name) != 0)
      continue;

    rc_ctrl.hdr.format = ran_func->seq_ctrl_style[i].hdr;
    assert(rc_ctrl.hdr.format == FORMAT_1_E2SM_RC_CTRL_HDR);

    rc_ctrl.hdr.frmt_1.ric_style_type = 2;
    rc_ctrl.hdr.frmt_1.ue_id = cp_ue_id_e2sm(ue_id);

    rc_ctrl.msg.format = ran_func->seq_ctrl_style[i].msg;
    assert(rc_ctrl.msg.format == FORMAT_1_E2SM_RC_CTRL_MSG);

    fill_rc_ctrl_act_style_2(ran_func->seq_ctrl_style[i].seq_ctrl_act,
                             ran_func->seq_ctrl_style[i].sz_seq_ctrl_act,
                             profile,
                             &rc_ctrl.hdr.frmt_1,
                             &rc_ctrl.msg.frmt_1);

    return rc_ctrl;
  }

  assert(false && "Radio Resource Allocation Control style not found");
  return rc_ctrl;
}

static bool eq_sm(sm_ran_function_t const* elem, int const id)
{
  return elem->id == id;
}

static size_t find_sm_idx(sm_ran_function_t* rf,
                          size_t sz,
                          bool (*f)(sm_ran_function_t const*, int const),
                          int const id)
{
  for (size_t i = 0; i < sz; i++) {
    if (f(&rf[i], id))
      return i;
  }

  assert(false && "SM ID could not be found");
  return 0;
}

int main(int argc, char* argv[])
{
  load_policy_from_env();

  fr_args_t args = init_fr_args(argc, argv);

  init_xapp_api(&args);
  sleep(1);

  e2_node_arr_xapp_t nodes = e2_nodes_xapp_api();
  assert(nodes.len > 0);

  printf("[RC-PRB-CTRL-XAPP] Connected E2 nodes = %d\n", nodes.len);

  int const RC_ran_function = 3;

  slice_profile_t const* profiles[] = {
      &g_profile_sst1,
      &g_profile_sst2,
      &g_profile_sst3,
  };

  for (size_t i = 0; i < nodes.len; ++i) {
    e2_node_connected_xapp_t* n = &nodes.n[i];

    size_t const idx = find_sm_idx(n->rf, n->len_rf, eq_sm, RC_ran_function);

    assert(n->rf[idx].defn.type == RC_RAN_FUNC_DEF_E);

    if (n->rf[idx].defn.rc.ctrl == NULL) {
      printf("[RC-PRB-CTRL-XAPP] node has no RC control definition\n");
      continue;
    }

    for (size_t p = 0; p < 3; p++) {
      slice_profile_t const* profile = profiles[p];

      ue_id_e2sm_t ue_id = make_ue_id_from_key(profile->ue_key);

      printf("[RC-PRB-CTRL-XAPP] send control: profile=%s, ue_key=%lu, "
             "sst=%u, sd=%02x%02x%02x, min=%u, max=%u, dedicated=%u\n",
             profile->profile_name,
             profile->ue_key,
             profile->sst,
             profile->sd[0],
             profile->sd[1],
             profile->sd[2],
             profile->min_prb,
             profile->max_prb,
             profile->dedicated_prb);

      rc_ctrl_req_data_t rc_ctrl =
          gen_rc_ctrl_msg_style_2(n->rf[idx].defn.rc.ctrl, profile, &ue_id);

      control_sm_xapp_api(&n->id, RC_ran_function, &rc_ctrl);

      free_rc_ctrl_req_data(&rc_ctrl);
      free_local_ue_id(&ue_id);
    }
  }

  sleep(1);

  try_stop_xapp_api();

  free_e2_node_arr_xapp(&nodes);

  printf("[RC-PRB-CTRL-XAPP] run SUCCESSFULLY\n");

  return 0;
}
