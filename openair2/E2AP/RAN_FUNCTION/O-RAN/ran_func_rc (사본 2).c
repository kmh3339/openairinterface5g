#include "ran_func_rc.h"
#include "ran_func_rc_subs.h"
#include "ran_func_rc_extern.h"
#include "ran_e2sm_ue_id.h"
#include "../../flexric/src/sm/rc_sm/ie/ir/lst_ran_param.h"
#include "../../flexric/src/sm/rc_sm/ie/ir/ran_param_list.h"
#include "../../flexric/src/agent/e2_agent_api.h"
#include "openair2/E2AP/flexric/src/lib/sm/enc/enc_ue_id.h"
#include "openair2/E2AP/flexric/src/sm/rc_sm/rc_sm_id.h"

#include <stdio.h>
#include <unistd.h>
#include "common/ran_context.h"

#include "openair2/LAYER2/NR_MAC_gNB/nr_mac_gNB.h"

static
void log_rc_ue_id_ran(char const* prefix, ue_id_e2sm_t const* id)
{
  assert(prefix != NULL);
  assert(id != NULL);

  printf("%s UE-ID dump start\n", prefix);
  printf("%s ue_id.type = %d\n", prefix, id->type);

  switch(id->type) {
    case GNB_UE_ID_E2SM:
      if (id->gnb.gnb_cu_ue_f1ap_lst != NULL) {
        for (size_t i = 0; i < id->gnb.gnb_cu_ue_f1ap_lst_len; i++) {
          printf("%s gNB-CU gnb_cu_ue_f1ap_lst[%zu] = %u\n",
                 prefix, i, id->gnb.gnb_cu_ue_f1ap_lst[i]);
        }
      } else {
        printf("%s gNB amf_ue_ngap_id = %lu\n", prefix, id->gnb.amf_ue_ngap_id);
      }

      if (id->gnb.ran_ue_id != NULL) {
        printf("%s gNB ran_ue_id = %lx\n", prefix, *id->gnb.ran_ue_id);
      }
      break;

    case GNB_DU_UE_ID_E2SM:
      printf("%s gNB-DU gnb_cu_ue_f1ap = %u\n", prefix, id->gnb_du.gnb_cu_ue_f1ap);
      if (id->gnb_du.ran_ue_id != NULL) {
        printf("%s gNB-DU ran_ue_id = %lx\n", prefix, *id->gnb_du.ran_ue_id);
      }
      break;

    case GNB_CU_UP_UE_ID_E2SM:
      printf("%s gNB-CU-UP gnb_cu_cp_ue_e1ap = %u\n", prefix, id->gnb_cu_up.gnb_cu_cp_ue_e1ap);
      if (id->gnb_cu_up.ran_ue_id != NULL) {
        printf("%s gNB-CU-UP ran_ue_id = %lx\n", prefix, *id->gnb_cu_up.ran_ue_id);
      }
      break;

    default:
      printf("%s UE ID type not explicitly handled\n", prefix);
      break;
  }

  printf("%s UE-ID dump end\n", prefix);
}

static
void log_ran_param_value_rc(char const* prefix, seq_ran_param_t const* p)
{
  assert(prefix != NULL);
  assert(p != NULL);

  printf("%s ran_param_id = %u\n", prefix, p->ran_param_id);

  switch(p->ran_param_val.type) {

    case ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.flag_true != NULL) {
        printf("%s type = FLAG_TRUE\n", prefix);
        if (p->ran_param_val.flag_true->type == INTEGER_RAN_PARAMETER_VALUE) {
          printf("%s value = %ld\n", prefix, p->ran_param_val.flag_true->int_ran);
        } else if (p->ran_param_val.flag_true->type == OCTET_STRING_RAN_PARAMETER_VALUE) {
          printf("%s octet_string.len = %zu\n", prefix,
                 p->ran_param_val.flag_true->octet_str_ran.len);
        }
      }
      break;

    case ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.flag_false != NULL) {
        printf("%s type = FLAG_FALSE\n", prefix);
        if (p->ran_param_val.flag_false->type == INTEGER_RAN_PARAMETER_VALUE) {
          printf("%s value = %ld\n", prefix, p->ran_param_val.flag_false->int_ran);
        } else if (p->ran_param_val.flag_false->type == OCTET_STRING_RAN_PARAMETER_VALUE) {
          printf("%s octet_string.len = %zu\n", prefix,
                 p->ran_param_val.flag_false->octet_str_ran.len);
        }
      }
      break;

    case STRUCTURE_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.strct != NULL) {
        printf("%s type = STRUCTURE, size = %zu\n", prefix,
               p->ran_param_val.strct->sz_ran_param_struct);
        for (size_t i = 0; i < p->ran_param_val.strct->sz_ran_param_struct; i++) {
          log_ran_param_value_rc(prefix,
                                 &p->ran_param_val.strct->ran_param_struct[i]);
        }
      }
      break;

    case LIST_RAN_PARAMETER_VAL_TYPE:
      if (p->ran_param_val.lst != NULL) {
        printf("%s type = LIST, size = %zu\n", prefix,
               p->ran_param_val.lst->sz_lst_ran_param);

        for (size_t i = 0; i < p->ran_param_val.lst->sz_lst_ran_param; i++) {
          lst_ran_param_t const* item = &p->ran_param_val.lst->lst_ran_param[i];
          for (size_t j = 0; j < item->ran_param_struct.sz_ran_param_struct; j++) {
            log_ran_param_value_rc(prefix,
                                   &item->ran_param_struct.ran_param_struct[j]);
          }
        }
      }
      break;

    default:
      printf("%s unknown type = %d\n", prefix, p->ran_param_val.type);
      break;
  }
}

static
void store_slice_prb_policy(module_id_t mod_id,
                            uint8_t sst,
                            uint32_t sd,
                            uint8_t min_ratio,
                            uint8_t max_ratio,
                            uint8_t dedicated_ratio)
{
  gNB_MAC_INST *mac = RC.nrmac[mod_id];
  if (!mac)
    return;

  pthread_mutex_lock(&mac->rc_slice_policy.lock);

  int idx = -1;
  for (int i = 0; i < mac->rc_slice_policy.n_policy; i++) {
    nr_rc_slice_policy_t *p = &mac->rc_slice_policy.policy[i];
    if (p->valid && p->sst == sst && p->sd == sd) {
      idx = i;
      break;
    }
  }

  if (idx < 0 && mac->rc_slice_policy.n_policy < MAX_RC_SLICE_POLICY)
    idx = mac->rc_slice_policy.n_policy++;

  if (idx >= 0) {
    nr_rc_slice_policy_t *p = &mac->rc_slice_policy.policy[idx];
    p->valid = true;
    p->sst = sst;
    p->sd = sd;
    p->min_ratio = min_ratio;
    p->max_ratio = max_ratio;
    p->dedicated_ratio = dedicated_ratio;
    printf("[RIC->MAC] stored policy: sst=%u sd=%u min=%u max=%u dedicated=%u\n",
       sst, sd, min_ratio, max_ratio, dedicated_ratio);
  }

  pthread_mutex_unlock(&mac->rc_slice_policy.lock);
}

static pthread_once_t once_rc_mutex = PTHREAD_ONCE_INIT;
static rc_subs_data_t rc_subs_data = {0};
static pthread_mutex_t rc_mutex = PTHREAD_MUTEX_INITIALIZER;

static
ngran_node_t get_e2_node_type(void)
{
  ngran_node_t node_type = 0;

#if defined(NGRAN_GNB_DU) && defined(NGRAN_GNB_CUUP) && defined(NGRAN_GNB_CUCP)
  node_type = RC.nrrrc[0]->node_type;
#elif defined (NGRAN_GNB_CUUP)
  node_type = ngran_gNB_CUUP;
#endif

  return node_type;
}

static
void init_once_rc(void)
{
  init_rc_subs_data(&rc_subs_data);
}

static
seq_ev_trg_style_t fill_ev_tr_format_4(const char *ev_style_name)
{
  seq_ev_trg_style_t ev_trig_style = {0};
  ev_trig_style.style = 4;
  ev_trig_style.name = cp_str_to_ba(ev_style_name);
  ev_trig_style.format = FORMAT_4_E2SM_RC_EV_TRIGGER_FORMAT;
  return ev_trig_style;
}

static
seq_ev_trg_style_t fill_ev_tr_format_1(const char *ev_style_name)
{
  seq_ev_trg_style_t ev_trig_style = {0};
  ev_trig_style.style = 1;
  ev_trig_style.name = cp_str_to_ba(ev_style_name);
  ev_trig_style.format = FORMAT_1_E2SM_RC_EV_TRIGGER_FORMAT;
  return ev_trig_style;
}

static
void fill_rc_ev_trig(ran_func_def_ev_trig_t* ev_trig)
{
  ev_trig->sz_seq_ev_trg_style = 2;
  ev_trig->seq_ev_trg_style = calloc(ev_trig->sz_seq_ev_trg_style, sizeof(seq_ev_trg_style_t));
  assert(ev_trig->seq_ev_trg_style != NULL && "Memory exhausted");

  ev_trig->seq_ev_trg_style[0] = fill_ev_tr_format_1("Message Event");
  ev_trig->seq_ev_trg_style[1] = fill_ev_tr_format_4("UE Information Change");

  ev_trig->sz_seq_ran_param_l2_var = 0;
  ev_trig->seq_ran_param_l2_var = NULL;
  ev_trig->sz_seq_call_proc_type = 0;
  ev_trig->seq_call_proc_type = NULL;
  ev_trig->sz_seq_ran_param_id_ue = 0;
  ev_trig->seq_ran_param_id_ue = NULL;
  ev_trig->sz_seq_ran_param_id_cell = 0;
  ev_trig->seq_ran_param_id_cell = NULL;
}

static
seq_report_sty_t fill_report_style_4(const char *report_name)
{
  seq_report_sty_t report_style = {0};

  report_style.report_type = 4;
  report_style.name = cp_str_to_ba(report_name);
  report_style.ev_trig_type = FORMAT_4_E2SM_RC_EV_TRIGGER_FORMAT;
  report_style.act_frmt_type = FORMAT_1_E2SM_RC_ACT_DEF;
  report_style.ind_hdr_type = FORMAT_1_E2SM_RC_IND_HDR;
  report_style.ind_msg_type = FORMAT_2_E2SM_RC_IND_MSG;

  report_style.sz_seq_ran_param = 1;
  report_style.ran_param = calloc(report_style.sz_seq_ran_param, sizeof(seq_ran_param_3_t));
  assert(report_style.ran_param != NULL && "Memory exhausted");

  report_style.ran_param[0].id = E2SM_RC_RS4_RRC_STATE_CHANGED_TO;
  report_style.ran_param[0].name = cp_str_to_ba("RRC State Changed To");
  report_style.ran_param[0].def = NULL;

  return report_style;
}

static
seq_report_sty_t fill_report_style_1(const char *report_name)
{
  seq_report_sty_t report_style = {0};

  report_style.report_type = 1;
  report_style.name = cp_str_to_ba(report_name);
  report_style.ev_trig_type = FORMAT_1_E2SM_RC_EV_TRIGGER_FORMAT;
  report_style.act_frmt_type = FORMAT_1_E2SM_RC_ACT_DEF;
  report_style.ind_hdr_type = FORMAT_1_E2SM_RC_IND_HDR;
  report_style.ind_msg_type = FORMAT_1_E2SM_RC_IND_MSG;

  report_style.sz_seq_ran_param = 2;
  report_style.ran_param = calloc_or_fail(report_style.sz_seq_ran_param, sizeof(seq_ran_param_3_t));

  report_style.ran_param[0].id = E2SM_RC_RS1_RRC_MESSAGE;
  report_style.ran_param[1].id = E2SM_RC_RS1_UE_ID;

  report_style.ran_param[0].name = cp_str_to_ba("RRC Message");
  report_style.ran_param[1].name = cp_str_to_ba("UE ID");

  report_style.ran_param[0].def = NULL;
  report_style.ran_param[1].def = NULL;

  return report_style;
}

static
void fill_rc_report(ran_func_def_report_t* report)
{
  report->sz_seq_report_sty = 2;
  report->seq_report_sty = calloc(report->sz_seq_report_sty, sizeof(seq_report_sty_t));
  assert(report->seq_report_sty != NULL && "Memory exhausted");

  report->seq_report_sty[0] = fill_report_style_1("Message Copy");
  report->seq_report_sty[1] = fill_report_style_4("UE Information");
}

/*
 * IMPORTANT:
 * - Style 1 / Action 2: keep existing behavior
 * - Style 2 / Action 6: advertise ONLY top-level associated parameter
 *   with def = NULL
 *
 * Reason:
 * Current RC ASN definition encoder/decoder cannot handle nested
 * ranParameter_Definition inside LIST/STRUCTURE for RAN Function Definition.
 * But actual CONTROL message LIST/STRUCTURE values are supported.
 */
static
void fill_rc_control(ran_func_def_ctrl_t* ctrl)
{
  ctrl->sz_seq_ctrl_style = 2;
  ctrl->seq_ctrl_style = calloc(ctrl->sz_seq_ctrl_style, sizeof(seq_ctrl_style_t));
  assert(ctrl->seq_ctrl_style != NULL && "Memory exhausted");

  /* --------------------------------------------------------------- */
  /* STYLE 1: Radio Bearer Control                                   */
  /* --------------------------------------------------------------- */
  seq_ctrl_style_t* ctrl_style = &ctrl->seq_ctrl_style[0];

  ctrl_style->style_type = 1;
  ctrl_style->name = cp_str_to_ba("Radio Bearer Control");
  ctrl_style->hdr = FORMAT_1_E2SM_RC_CTRL_HDR;
  ctrl_style->msg = FORMAT_1_E2SM_RC_CTRL_MSG;
  ctrl_style->call_proc_id_type = NULL;
  ctrl_style->out_frmt = FORMAT_1_E2SM_RC_CTRL_OUT;

  ctrl_style->sz_seq_ctrl_act = 1;
  ctrl_style->seq_ctrl_act = calloc(ctrl_style->sz_seq_ctrl_act, sizeof(seq_ctrl_act_2_t));
  assert(ctrl_style->seq_ctrl_act != NULL && "Memory exhausted");

  seq_ctrl_act_2_t* ctrl_act = &ctrl_style->seq_ctrl_act[0];
  ctrl_act->id = 2;
  ctrl_act->name = cp_str_to_ba("QoS flow mapping configuration");

  ctrl_act->sz_seq_assoc_ran_param = 2;
  ctrl_act->assoc_ran_param = calloc(ctrl_act->sz_seq_assoc_ran_param, sizeof(seq_ran_param_3_t));
  assert(ctrl_act->assoc_ran_param != NULL && "Memory exhausted");

  seq_ran_param_3_t* assoc_ran_param = ctrl_act->assoc_ran_param;

  assoc_ran_param[0].id = 1;
  assoc_ran_param[0].name = cp_str_to_ba("DRB ID");
  assoc_ran_param[0].def = NULL;

  assoc_ran_param[1].id = 2;
  assoc_ran_param[1].name = cp_str_to_ba("List of QoS Flows to be modified in DRB");
  assoc_ran_param[1].def = calloc(1, sizeof(ran_param_def_t));
  assert(assoc_ran_param[1].def != NULL && "Memory exhausted");

  assoc_ran_param[1].def->type = LIST_RAN_PARAMETER_DEF_TYPE;
  assoc_ran_param[1].def->lst = calloc(1, sizeof(ran_param_type_t));
  assert(assoc_ran_param[1].def->lst != NULL && "Memory exhausted");

  ran_param_type_t* lst = assoc_ran_param[1].def->lst;
  lst->sz_ran_param = 2;
  lst->ran_param = calloc(lst->sz_ran_param, sizeof(ran_param_lst_struct_t));
  assert(lst->ran_param != NULL && "Memory exhausted");

  lst->ran_param[0].ran_param_id = 4;
  lst->ran_param[0].ran_param_name = cp_str_to_ba("QoS Flow Identifier");
  lst->ran_param[0].ran_param_def = NULL;

  lst->ran_param[1].ran_param_id = 5;
  lst->ran_param[1].ran_param_name = cp_str_to_ba("QoS Flow Mapping Indication");
  lst->ran_param[1].ran_param_def = NULL;

  ctrl_style->sz_ran_param_ctrl_out = 0;
  ctrl_style->ran_param_ctrl_out = NULL;

  /* --------------------------------------------------------------- */
  /* STYLE 2: Radio Resource Allocation Control                      */
  /* --------------------------------------------------------------- */
  seq_ctrl_style_t* ctrl_style2 = &ctrl->seq_ctrl_style[1];

  ctrl_style2->style_type = 2;
  ctrl_style2->name = cp_str_to_ba("Radio Resource Allocation Control");
  ctrl_style2->hdr = FORMAT_1_E2SM_RC_CTRL_HDR;
  ctrl_style2->msg = FORMAT_1_E2SM_RC_CTRL_MSG;
  ctrl_style2->call_proc_id_type = NULL;
  ctrl_style2->out_frmt = FORMAT_1_E2SM_RC_CTRL_OUT;

  ctrl_style2->sz_seq_ctrl_act = 1;
  ctrl_style2->seq_ctrl_act = calloc(ctrl_style2->sz_seq_ctrl_act, sizeof(seq_ctrl_act_2_t));
  assert(ctrl_style2->seq_ctrl_act != NULL && "Memory exhausted");

  seq_ctrl_act_2_t* ctrl_act2 = &ctrl_style2->seq_ctrl_act[0];
  ctrl_act2->id = 6;
  ctrl_act2->name = cp_str_to_ba("Slice-level PRB quota");

  /*
   * Keep associated parameter advertisement minimal.
   * Do NOT attach nested ran_param_def here.
   */
  ctrl_act2->sz_seq_assoc_ran_param = 1;
  ctrl_act2->assoc_ran_param = calloc(ctrl_act2->sz_seq_assoc_ran_param, sizeof(seq_ran_param_3_t));
  assert(ctrl_act2->assoc_ran_param != NULL && "Memory exhausted");

  ctrl_act2->assoc_ran_param[0].id = 1;
  ctrl_act2->assoc_ran_param[0].name = cp_str_to_ba("RRM Policy Ratio List");
  ctrl_act2->assoc_ran_param[0].def = NULL;

  ctrl_style2->sz_ran_param_ctrl_out = 0;
  ctrl_style2->ran_param_ctrl_out = NULL;
}

static
ran_function_name_t fill_rc_ran_func_name(void)
{
  ran_function_name_t dst = {
    .name = cp_str_to_ba(SM_RAN_CTRL_SHORT_NAME),
    .oid = cp_str_to_ba(SM_RAN_CTRL_OID),
    .description = cp_str_to_ba(SM_RAN_CTRL_DESCRIPTION),
    .instance = NULL
  };
  return dst;
}

e2sm_rc_func_def_t fill_rc_ran_def_gnb(void)
{
  e2sm_rc_func_def_t def = {0};

  def.name = fill_rc_ran_func_name();

  def.ev_trig = calloc(1, sizeof(ran_func_def_ev_trig_t));
  assert(def.ev_trig != NULL && "Memory exhausted");
  fill_rc_ev_trig(def.ev_trig);

  def.report = calloc(1, sizeof(ran_func_def_report_t));
  assert(def.report != NULL && "Memory exhausted");
  fill_rc_report(def.report);

  def.insert = NULL;

  def.ctrl = calloc(1, sizeof(ran_func_def_ctrl_t));
  assert(def.ctrl != NULL && "Memory exhausted");
  fill_rc_control(def.ctrl);

  def.policy = NULL;

  return def;
}

static
e2sm_rc_func_def_t fill_rc_ran_def_cu(void)
{
  e2sm_rc_func_def_t def = {0};

  def.name = fill_rc_ran_func_name();

  def.ev_trig = calloc(1, sizeof(ran_func_def_ev_trig_t));
  assert(def.ev_trig != NULL && "Memory exhausted");
  fill_rc_ev_trig(def.ev_trig);

  def.report = calloc(1, sizeof(ran_func_def_report_t));
  assert(def.report != NULL && "Memory exhausted");
  fill_rc_report(def.report);

  def.insert = NULL;

  def.ctrl = calloc(1, sizeof(ran_func_def_ctrl_t));
  assert(def.ctrl != NULL && "Memory exhausted");
  fill_rc_control(def.ctrl);

  def.policy = NULL;

  return def;
}

static
e2sm_rc_func_def_t fill_rc_ran_def_null(void)
{
  e2sm_rc_func_def_t def = {0};

  def.name = fill_rc_ran_func_name();
  def.ev_trig = NULL;
  def.report = NULL;
  def.insert = NULL;
  def.ctrl = NULL;
  def.policy = NULL;

  return def;
}

static
e2sm_rc_func_def_t fill_rc_ran_def_cucp(void)
{
  e2sm_rc_func_def_t def = {0};

  def.name = fill_rc_ran_func_name();

  def.ev_trig = calloc(1, sizeof(ran_func_def_ev_trig_t));
  assert(def.ev_trig != NULL && "Memory exhausted");
  fill_rc_ev_trig(def.ev_trig);

  def.report = calloc(1, sizeof(ran_func_def_report_t));
  assert(def.report != NULL && "Memory exhausted");
  fill_rc_report(def.report);

  def.insert = NULL;
  def.ctrl = NULL;
  def.policy = NULL;

  return def;
}

typedef e2sm_rc_func_def_t (*fp_rc_func_def)(void);

static const fp_rc_func_def ran_def_rc[END_NGRAN_NODE_TYPE] =
{
  NULL,
  NULL,
  fill_rc_ran_def_gnb,
  NULL,
  NULL,
  fill_rc_ran_def_cu,
  NULL,
  fill_rc_ran_def_null,
  NULL,
  fill_rc_ran_def_cucp,
  fill_rc_ran_def_null,
};

void read_rc_setup_sm(void* data)
{
  assert(data != NULL);
  rc_e2_setup_t* rc = (rc_e2_setup_t*)data;

  const ngran_node_t node_type = get_e2_node_type();
  rc->ran_func_def = ran_def_rc[node_type]();

  const int ret = pthread_once(&once_rc_mutex, init_once_rc);
  DevAssert(ret == 0);
}

static
seq_ran_param_t fill_rrc_state_change_seq_ran(const rrc_state_e2sm_rc_e rrc_state)
{
  seq_ran_param_t seq_ran_param = {0};

  seq_ran_param.ran_param_id = E2SM_RC_RS4_RRC_STATE_CHANGED_TO;
  seq_ran_param.ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  seq_ran_param.ran_param_val.flag_false = calloc(1, sizeof(ran_parameter_value_t));
  assert(seq_ran_param.ran_param_val.flag_false != NULL && "Memory exhausted");
  seq_ran_param.ran_param_val.flag_false->type = INTEGER_RAN_PARAMETER_VALUE;
  seq_ran_param.ran_param_val.flag_false->int_ran = rrc_state;

  return seq_ran_param;
}

static
rc_ind_data_t* fill_ue_rrc_state_change(const gNB_RRC_UE_t *rrc_ue_context,
                                        const rrc_state_e2sm_rc_e rrc_state,
                                        const uint16_t cond_id)
{
  rc_ind_data_t* rc_ind = calloc(1, sizeof(rc_ind_data_t));
  assert(rc_ind != NULL && "Memory exhausted");

  rc_ind->hdr.format = FORMAT_1_E2SM_RC_IND_HDR;
  rc_ind->hdr.frmt_1.ev_trigger_id = malloc_or_fail(sizeof(uint32_t));
  *rc_ind->hdr.frmt_1.ev_trigger_id = cond_id;

  rc_ind->msg.format = FORMAT_2_E2SM_RC_IND_MSG;
  rc_ind->msg.frmt_2.sz_seq_ue_id = 1;
  rc_ind->msg.frmt_2.seq_ue_id = calloc(rc_ind->msg.frmt_2.sz_seq_ue_id, sizeof(seq_ue_id_t));
  assert(rc_ind->msg.frmt_2.seq_ue_id != NULL && "Memory exhausted");

  const ngran_node_t node_type = get_e2_node_type();
  rc_ind->msg.frmt_2.seq_ue_id[0].ue_id = fill_ue_id_data[node_type](rrc_ue_context, 0, 0);

  rc_ind->msg.frmt_2.seq_ue_id[0].sz_seq_ran_param = 1;
  rc_ind->msg.frmt_2.seq_ue_id[0].seq_ran_param =
      calloc(rc_ind->msg.frmt_2.seq_ue_id[0].sz_seq_ran_param, sizeof(seq_ran_param_t));
  assert(rc_ind->msg.frmt_2.seq_ue_id[0].seq_ran_param != NULL && "Memory exhausted");
  rc_ind->msg.frmt_2.seq_ue_id[0].seq_ran_param[0] = fill_rrc_state_change_seq_ran(rrc_state);

  return rc_ind;
}

static
void send_aper_ric_ind(const uint32_t ric_req_id, rc_ind_data_t* rc_ind_data)
{
  async_event_agent_api(ric_req_id, rc_ind_data);
  printf("[E2 AGENT] Event for RIC request ID %d generated\n", ric_req_id);
}

static
rc_ind_data_t* fill_ue_id(const gNB_RRC_UE_t *rrc_ue_context, const uint16_t cond_id)
{
  rc_ind_data_t* rc_ind = malloc_or_fail(sizeof(rc_ind_data_t));

  rc_ind->hdr.format = FORMAT_1_E2SM_RC_IND_HDR;
  rc_ind->hdr.frmt_1.ev_trigger_id = malloc_or_fail(sizeof(uint32_t));
  *rc_ind->hdr.frmt_1.ev_trigger_id = cond_id;

  rc_ind->msg.format = FORMAT_1_E2SM_RC_IND_MSG;
  rc_ind->msg.frmt_1.sz_seq_ran_param = 1;
  rc_ind->msg.frmt_1.seq_ran_param = calloc_or_fail(rc_ind->msg.frmt_1.sz_seq_ran_param, sizeof(seq_ran_param_t));

  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_id = E2SM_RC_RS1_UE_ID;
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.flag_false = malloc_or_fail(sizeof(ran_parameter_value_t));
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.flag_false->type = OCTET_STRING_RAN_PARAMETER_VALUE;

  const ngran_node_t node_type = get_e2_node_type();
  ue_id_e2sm_t ue_id_data = fill_ue_id_data[node_type](rrc_ue_context, 0, 0);

  UEID_t enc_ue_id_data = enc_ue_id_asn(&ue_id_data);
  const enum asn_transfer_syntax syntax = ATS_ALIGNED_BASIC_PER;
  asn_encode_to_new_buffer_result_t res =
      asn_encode_to_new_buffer(NULL, syntax, &asn_DEF_UEID, &enc_ue_id_data);
  assert(res.buffer != NULL && res.result.encoded > 0 && "[E2 agent] Failed to encode UE ID.");
  byte_array_t ba = {.buf = res.buffer, .len = res.result.encoded};

  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.flag_false->octet_str_ran = copy_byte_array(ba);
  rc_ind->proc_id = NULL;

  free_ue_id_e2sm(&ue_id_data);
  ASN_STRUCT_RESET(asn_DEF_UEID, &enc_ue_id_data);
  free(res.buffer);

  return rc_ind;
}

/* ---- subscription / indication helper code unchanged ---- */
/* ---- keep your existing check_ue_id_cond, signal_ue_id,   */
/* ---- check_rrc_state, signal_rrc_state_changed_to,         */
/* ---- fill_rrc_msg_copy, check_rrc_msg_copy, signal_rrc_msg */
/* ---- free_aperiodic_subscription, get_sa,                 */
/* ---- get_list_for_report_style, write_subs_rc_sm          */
/* ---- exactly as in your current file                      */

static
void check_ue_id_cond(const gNB_RRC_UE_t *rrc_ue_context, const uint16_t class,
                      const uint32_t msg_id, const uint32_t ric_req_id,
                      const e2sm_rc_ev_trg_frmt_1_t *frmt_1)
{
  for (size_t i = 0; i < frmt_1->sz_msg_ev_trg; i++) {
    msg_ev_trg_t *ev_item = &frmt_1->msg_ev_trg[i];
    if ((ev_item->msg_type == RRC_MSG_MSG_TYPE_EV_TRG &&
         ev_item->rrc_msg.type == NR_RRC_MESSAGE_ID &&
         ev_item->rrc_msg.nr == class &&
         ev_item->rrc_msg.rrc_msg_id == msg_id)
        || (ev_item->msg_type == NETWORK_INTERFACE_MSG_TYPE_EV_TRG &&
            ev_item->net.ni_type == class)) {
      rc_ind_data_t* rc_ind_data = fill_ue_id(rrc_ue_context, ev_item->ev_trigger_cond_id);
      send_aper_ric_ind(ric_req_id, rc_ind_data);
    }
  }
}

void signal_ue_id(const gNB_RRC_UE_t *rrc_ue_context, const uint16_t class, const uint32_t msg_id)
{
  pthread_mutex_lock(&rc_mutex);
  if (rc_subs_data.rs1_param4.data == NULL) {
    pthread_mutex_unlock(&rc_mutex);
    return;
  }

  const size_t num_subs = seq_arr_size(&rc_subs_data.rs1_param4);
  for (size_t sub_idx = 0; sub_idx < num_subs; sub_idx++) {
    const ran_param_data_t data = *(const ran_param_data_t *)seq_arr_at(&rc_subs_data.rs1_param4, sub_idx);
    check_ue_id_cond(rrc_ue_context, class, msg_id, data.ric_req_id, &data.ev_tr.frmt_1);
  }

  pthread_mutex_unlock(&rc_mutex);
}

static
void check_rrc_state(const gNB_RRC_UE_t *rrc_ue_context, const rrc_state_e2sm_rc_e rrc_state,
                     const uint32_t ric_req_id, const e2sm_rc_ev_trg_frmt_4_t *frmt_4)
{
  for (size_t i = 0; i < frmt_4->sz_ue_info_chng; i++) {
    const uint16_t cond_id = frmt_4->ue_info_chng[i].ev_trig_cond_id;
    const rrc_state_lst_t *rrc_elem = &frmt_4->ue_info_chng[i].rrc_state;
    for (size_t j = 0; j < rrc_elem->sz_rrc_state; j++) {
      const rrc_state_e2sm_rc_e ev_tr_rrc_state = rrc_elem->state_chng_to[j].state_chngd_to;
      if (ev_tr_rrc_state == rrc_state || ev_tr_rrc_state == ANY_RRC_STATE_E2SM_RC) {
        rc_ind_data_t* rc_ind_data = fill_ue_rrc_state_change(rrc_ue_context, rrc_state, cond_id);
        send_aper_ric_ind(ric_req_id, rc_ind_data);
      }
    }
  }
}

void signal_rrc_state_changed_to(const gNB_RRC_UE_t *rrc_ue_context, const rrc_state_e2sm_rc_e rrc_state)
{
  pthread_mutex_lock(&rc_mutex);
  if (rc_subs_data.rs4_param202.data == NULL) {
    pthread_mutex_unlock(&rc_mutex);
    return;
  }

  const size_t num_subs = seq_arr_size(&rc_subs_data.rs4_param202);
  for (size_t sub_idx = 0; sub_idx < num_subs; sub_idx++) {
    const ran_param_data_t data = *(const ran_param_data_t *)seq_arr_at(&rc_subs_data.rs4_param202, sub_idx);
    check_rrc_state(rrc_ue_context, rrc_state, data.ric_req_id, &data.ev_tr.frmt_4);
  }

  pthread_mutex_unlock(&rc_mutex);
}

static
rc_ind_data_t* fill_rrc_msg_copy(const byte_array_t rrc_ba, const uint16_t cond_id)
{
  rc_ind_data_t* rc_ind = malloc_or_fail(sizeof(rc_ind_data_t));

  rc_ind->hdr.format = FORMAT_1_E2SM_RC_IND_HDR;
  rc_ind->hdr.frmt_1.ev_trigger_id = malloc_or_fail(sizeof(uint32_t));
  *rc_ind->hdr.frmt_1.ev_trigger_id = cond_id;

  rc_ind->msg.format = FORMAT_1_E2SM_RC_IND_MSG;
  rc_ind->msg.frmt_1.sz_seq_ran_param = 1;
  rc_ind->msg.frmt_1.seq_ran_param = calloc_or_fail(rc_ind->msg.frmt_1.sz_seq_ran_param, sizeof(seq_ran_param_t));

  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_id = E2SM_RC_RS1_RRC_MESSAGE;
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.type = ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE;
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.flag_false = malloc_or_fail(sizeof(ran_parameter_value_t));
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.flag_false->type = OCTET_STRING_RAN_PARAMETER_VALUE;
  rc_ind->msg.frmt_1.seq_ran_param[0].ran_param_val.flag_false->octet_str_ran = copy_byte_array(rrc_ba);

  rc_ind->proc_id = NULL;
  return rc_ind;
}

static
void check_rrc_msg_copy(const nr_rrc_class_e nr_channel, const uint32_t rrc_msg_id,
                        const byte_array_t rrc_ba, const uint32_t ric_req_id,
                        const e2sm_rc_ev_trg_frmt_1_t *frmt_1)
{
  for (size_t i = 0; i < frmt_1->sz_msg_ev_trg; i++) {
    if (frmt_1->msg_ev_trg[i].msg_type != RRC_MSG_MSG_TYPE_EV_TRG)
      continue;
    if (frmt_1->msg_ev_trg[i].rrc_msg.type != NR_RRC_MESSAGE_ID)
      continue;
    if (frmt_1->msg_ev_trg[i].rrc_msg.nr == nr_channel &&
        frmt_1->msg_ev_trg[i].rrc_msg.rrc_msg_id == rrc_msg_id) {
      rc_ind_data_t* rc_ind_data = fill_rrc_msg_copy(rrc_ba, frmt_1->msg_ev_trg[i].ev_trigger_cond_id);
      send_aper_ric_ind(ric_req_id, rc_ind_data);
    }
  }
}

void signal_rrc_msg(const nr_rrc_class_e nr_channel, const uint32_t rrc_msg_id, const byte_array_t rrc_ba)
{
  pthread_mutex_lock(&rc_mutex);
  if (rc_subs_data.rs1_param3.data == NULL) {
    pthread_mutex_unlock(&rc_mutex);
    return;
  }

  const size_t num_subs = seq_arr_size(&rc_subs_data.rs1_param3);
  for (size_t sub_idx = 0; sub_idx < num_subs; sub_idx++) {
    const ran_param_data_t data = *(const ran_param_data_t *)seq_arr_at(&rc_subs_data.rs1_param3, sub_idx);
    check_rrc_msg_copy(nr_channel, rrc_msg_id, rrc_ba, data.ric_req_id, &data.ev_tr.frmt_1);
  }

  pthread_mutex_unlock(&rc_mutex);
}

static
void free_aperiodic_subscription(uint32_t ric_req_id)
{
  remove_rc_subs_data(&rc_subs_data, ric_req_id);
}

static
seq_arr_t *get_sa(const e2sm_rc_event_trigger_t *ev_tr, const uint32_t ran_param_id)
{
  seq_arr_t *sa = NULL;

  switch (ev_tr->format) {
    case FORMAT_1_E2SM_RC_EV_TRIGGER_FORMAT:
      if (ran_param_id == E2SM_RC_RS1_RRC_MESSAGE)
        sa = &rc_subs_data.rs1_param3;
      else if (ran_param_id == E2SM_RC_RS1_UE_ID)
        sa = &rc_subs_data.rs1_param4;
      break;

    case FORMAT_4_E2SM_RC_EV_TRIGGER_FORMAT:
      if (ran_param_id == E2SM_RC_RS4_RRC_STATE_CHANGED_TO)
        sa = &rc_subs_data.rs4_param202;
      break;

    default:
      printf("[E2 AGENT] RC REPORT Style %d not yet implemented.\n", ev_tr->format + 1);
      break;
  }

  return sa;
}

static
void get_list_for_report_style(const uint32_t ric_req_id, const e2sm_rc_event_trigger_t *ev_tr,
                               const size_t sz, const param_report_def_t *param_def)
{
  for (size_t i = 0; i < sz; i++) {
    seq_arr_t *sa = get_sa(ev_tr, param_def[i].ran_param_id);
    if (!sa) {
      printf("[E2 AGENT] Requested RAN Parameter ID %d not yet implemented",
             param_def[i].ran_param_id);
    } else {
      struct ran_param_data data = {
        .ric_req_id = ric_req_id,
        .ev_tr = cp_e2sm_rc_event_trigger(ev_tr)
      };
      insert_rc_subs_data(sa, &data);
    }
  }
}

sm_ag_if_ans_t write_subs_rc_sm(void const* src)
{
  assert(src != NULL);
  wr_rc_sub_data_t* wr_rc = (wr_rc_sub_data_t*)src;
  assert(wr_rc->rc.ad != NULL && "Cannot be NULL");

  sm_ag_if_ans_t ans = {0};

  const uint32_t ric_req_id = wr_rc->ric_req_id;
  const uint32_t report_style = wr_rc->rc.ad->ric_style_type;

  switch (wr_rc->rc.ad->format) {
    case FORMAT_1_E2SM_RC_ACT_DEF: {
      if (wr_rc->rc.et.format + 1 != report_style) {
        AssertError(false, return ans,
                    "[E2 AGENT] Event Trigger Definition Format %d doesn't correspond to REPORT style %d.\n",
                    wr_rc->rc.et.format + 1, report_style);
      }
      get_list_for_report_style(ric_req_id, &wr_rc->rc.et,
                                wr_rc->rc.ad->frmt_1.sz_param_report_def,
                                wr_rc->rc.ad->frmt_1.param_report_def);
      break;
    }

    default:
      AssertError(wr_rc->rc.ad->format == FORMAT_1_E2SM_RC_ACT_DEF, return ans,
                  "[E2 AGENT] Action Definition Format %d not yet implemented",
                  wr_rc->rc.ad->format + 1);
  }

  ans.type = SUBS_OUTCOME_SM_AG_IF_ANS_V0;
  ans.subs_out.type = APERIODIC_SUBSCRIPTION_FLRC;
  ans.subs_out.aper.free_aper_subs = free_aperiodic_subscription;

  return ans;
}

sm_ag_if_ans_t write_ctrl_rc_sm(void const* data)
{
  assert(data != NULL);

  rc_ctrl_req_data_t const* ctrl = (rc_ctrl_req_data_t const*)data;

  printf("[RAN-RC][CTRL] ==================================================\n");
  printf("[RAN-RC][CTRL] write_ctrl_rc_sm: start\n");

  assert(ctrl->hdr.format == FORMAT_1_E2SM_RC_CTRL_HDR &&
         "Indication Header Format received not valid");
  assert(ctrl->msg.format == FORMAT_1_E2SM_RC_CTRL_MSG &&
         "Indication Message Format received not valid");

  printf("[RAN-RC][CTRL] hdr.format = %d\n", ctrl->hdr.format);
  printf("[RAN-RC][CTRL] msg.format = %d\n", ctrl->msg.format);
  printf("[RAN-RC][CTRL] ric_style_type = %d\n", ctrl->hdr.frmt_1.ric_style_type);
  printf("[RAN-RC][CTRL] ctrl_act_id = %d\n", ctrl->hdr.frmt_1.ctrl_act_id);

  log_rc_ue_id_ran("[RAN-RC][CTRL]", &ctrl->hdr.frmt_1.ue_id);

  printf("[RAN-RC][CTRL] msg.frmt_1.sz_ran_param = %zu\n", ctrl->msg.frmt_1.sz_ran_param);

  const seq_ran_param_t* ran_param = ctrl->msg.frmt_1.ran_param;
  assert(ran_param != NULL);

  for (size_t i = 0; i < ctrl->msg.frmt_1.sz_ran_param; i++) {
    printf("[RAN-RC][CTRL] ran_param[%zu].ran_param_id = %u\n", i, ran_param[i].ran_param_id);
    log_ran_param_value_rc("[RAN-RC][CTRL]", &ran_param[i]);
  }

  if (ctrl->hdr.frmt_1.ric_style_type == 1 && ctrl->hdr.frmt_1.ctrl_act_id == 2) {

    printf("[RAN-RC][CTRL] matched action: QoS flow mapping configuration\n");

    assert(ctrl->msg.frmt_1.sz_ran_param >= 2);
    assert(ran_param[0].ran_param_id == 1);
    assert(ran_param[0].ran_param_val.type == ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE);
    assert(ran_param[0].ran_param_val.flag_true != NULL);
    assert(ran_param[0].ran_param_val.flag_true->type == INTEGER_RAN_PARAMETER_VALUE);

    int64_t drb_id = ran_param[0].ran_param_val.flag_true->int_ran;
    printf("[RAN-RC][CTRL] parsed DRB ID = %ld\n", drb_id);

    assert(ran_param[1].ran_param_id == 2);
    assert(ran_param[1].ran_param_val.type == LIST_RAN_PARAMETER_VAL_TYPE);
    assert(ran_param[1].ran_param_val.lst != NULL);

    printf("[RAN-RC][CTRL] parsing List of QoS Flows to be modified in DRB\n");
    printf("[RAN-RC][CTRL] number of QoS flow list items = %zu\n",
           ran_param[1].ran_param_val.lst->sz_lst_ran_param);

    assert(ran_param[1].ran_param_val.lst->sz_lst_ran_param > 0);

    for (size_t list_idx = 0; list_idx < ran_param[1].ran_param_val.lst->sz_lst_ran_param; list_idx++) {
      const lst_ran_param_t* lrp = &ran_param[1].ran_param_val.lst->lst_ran_param[list_idx];
      assert(lrp->ran_param_struct.sz_ran_param_struct >= 2);

      assert(lrp->ran_param_struct.ran_param_struct[0].ran_param_id == 4);
      assert(lrp->ran_param_struct.ran_param_struct[0].ran_param_val.type ==
             ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE);
      assert(lrp->ran_param_struct.ran_param_struct[0].ran_param_val.flag_true != NULL);
      assert(lrp->ran_param_struct.ran_param_struct[0].ran_param_val.flag_true->type ==
             INTEGER_RAN_PARAMETER_VALUE);

      int64_t qfi =
          lrp->ran_param_struct.ran_param_struct[0].ran_param_val.flag_true->int_ran;
      assert(qfi > -1 && qfi < 65);

      assert(lrp->ran_param_struct.ran_param_struct[1].ran_param_id == 5);
      assert(lrp->ran_param_struct.ran_param_struct[1].ran_param_val.type ==
             ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE);
      assert(lrp->ran_param_struct.ran_param_struct[1].ran_param_val.flag_false != NULL);
      assert(lrp->ran_param_struct.ran_param_struct[1].ran_param_val.flag_false->type ==
             INTEGER_RAN_PARAMETER_VALUE);

      int64_t dir =
          lrp->ran_param_struct.ran_param_struct[1].ran_param_val.flag_false->int_ran;
      assert(dir == 0 || dir == 1);

      printf("[RAN-RC][CTRL] parsed list[%zu]: qfi = %ld, dir = %ld\n",
             list_idx, qfi, dir);
    }

    printf("[RAN-RC][CTRL] NOTE: current implementation parses and logs control values only.\n");
    printf("[RAN-RC][CTRL] NOTE: no actual OAI scheduler / bearer reconfiguration is applied here yet.\n");
  }

  else if (ctrl->hdr.frmt_1.ric_style_type == 2 && ctrl->hdr.frmt_1.ctrl_act_id == 6) {

 printf("[RAN-RC][CTRL] matched action: Slice-level PRB quota\n");

  assert(ctrl->msg.frmt_1.sz_ran_param >= 1);

  assert(ran_param[0].ran_param_id == 1);
  assert(ran_param[0].ran_param_val.type == LIST_RAN_PARAMETER_VAL_TYPE);
  assert(ran_param[0].ran_param_val.lst != NULL);
  assert(ran_param[0].ran_param_val.lst->sz_lst_ran_param > 0);

  printf("[RAN-RC][CTRL] number of RRM Policy Ratio Group items = %zu\n",
         ran_param[0].ran_param_val.lst->sz_lst_ran_param);

  for (size_t list_idx = 0; list_idx < ran_param[0].ran_param_val.lst->sz_lst_ran_param; list_idx++) {
    int64_t sst = -1;
    uint32_t sd = 0; /* TODO: actual SD parsing if needed */

    const lst_ran_param_t* ratio_group_item =
        &ran_param[0].ran_param_val.lst->lst_ran_param[list_idx];

    assert(ratio_group_item->ran_param_struct.sz_ran_param_struct >= 4);
    const seq_ran_param_t* group = ratio_group_item->ran_param_struct.ran_param_struct;

    assert(group[0].ran_param_id == 3);
    assert(group[0].ran_param_val.type == STRUCTURE_RAN_PARAMETER_VAL_TYPE);
    assert(group[0].ran_param_val.strct != NULL);
    assert(group[0].ran_param_val.strct->sz_ran_param_struct >= 1);

    const seq_ran_param_t* rrm_policy_struct =
        group[0].ran_param_val.strct->ran_param_struct;

    assert(rrm_policy_struct[0].ran_param_id == 5);
    assert(rrm_policy_struct[0].ran_param_val.type == LIST_RAN_PARAMETER_VAL_TYPE);
    assert(rrm_policy_struct[0].ran_param_val.lst != NULL);
    assert(rrm_policy_struct[0].ran_param_val.lst->sz_lst_ran_param > 0);

    for (size_t member_idx = 0;
         member_idx < rrm_policy_struct[0].ran_param_val.lst->sz_lst_ran_param;
         member_idx++) {

      const lst_ran_param_t* member_item =
          &rrm_policy_struct[0].ran_param_val.lst->lst_ran_param[member_idx];

      assert(member_item->ran_param_struct.sz_ran_param_struct >= 2);
      const seq_ran_param_t* member = member_item->ran_param_struct.ran_param_struct;

      assert(member[0].ran_param_id == 7);
      printf("[RAN-RC][CTRL] member[%zu]: PLMN Identity present\n", member_idx);

      assert(member[1].ran_param_id == 8);
      assert(member[1].ran_param_val.type == STRUCTURE_RAN_PARAMETER_VAL_TYPE);
      assert(member[1].ran_param_val.strct != NULL);
      assert(member[1].ran_param_val.strct->sz_ran_param_struct >= 2);

      const seq_ran_param_t* snssai = member[1].ran_param_val.strct->ran_param_struct;

      assert(snssai[0].ran_param_id == 9);
      assert(snssai[0].ran_param_val.type == ELEMENT_KEY_FLAG_TRUE_RAN_PARAMETER_VAL_TYPE);
      assert(snssai[0].ran_param_val.flag_true != NULL);
      assert(snssai[0].ran_param_val.flag_true->type == INTEGER_RAN_PARAMETER_VALUE);
      sst = snssai[0].ran_param_val.flag_true->int_ran;

      assert(snssai[1].ran_param_id == 10);
      printf("[RAN-RC][CTRL] member[%zu]: parsed SST=%ld, SD field present, value_type=%d\n",
             member_idx, sst, snssai[1].ran_param_val.type);
    }

    assert(group[1].ran_param_id == 11);
    assert(group[1].ran_param_val.type == ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE);
    assert(group[1].ran_param_val.flag_false != NULL);
    assert(group[1].ran_param_val.flag_false->type == INTEGER_RAN_PARAMETER_VALUE);
    int64_t min_ratio = group[1].ran_param_val.flag_false->int_ran;

    assert(group[2].ran_param_id == 12);
    assert(group[2].ran_param_val.type == ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE);
    assert(group[2].ran_param_val.flag_false != NULL);
    assert(group[2].ran_param_val.flag_false->type == INTEGER_RAN_PARAMETER_VALUE);
    int64_t max_ratio = group[2].ran_param_val.flag_false->int_ran;

    assert(group[3].ran_param_id == 13);
    assert(group[3].ran_param_val.type == ELEMENT_KEY_FLAG_FALSE_RAN_PARAMETER_VAL_TYPE);
    assert(group[3].ran_param_val.flag_false != NULL);
    assert(group[3].ran_param_val.flag_false->type == INTEGER_RAN_PARAMETER_VALUE);
    int64_t dedicated_ratio = group[3].ran_param_val.flag_false->int_ran;

    printf("[RAN-RC][CTRL] parsed ratio_group[%zu]: min=%ld max=%ld dedicated=%ld\n",
           list_idx, min_ratio, max_ratio, dedicated_ratio);

    assert(min_ratio >= 0 && min_ratio <= 100);
    assert(max_ratio >= 0 && max_ratio <= 100);
    assert(dedicated_ratio >= 0 && dedicated_ratio <= 100);

    printf("[RAN-RC][CTRL] calling store_slice_prb_policy: sst=%ld sd=%u min=%ld max=%ld dedicated=%ld\n",
           sst, sd, min_ratio, max_ratio, dedicated_ratio);

    store_slice_prb_policy(0,
                           (uint8_t)sst,
                           sd,
                           (uint8_t)min_ratio,
                           (uint8_t)max_ratio,
                           (uint8_t)dedicated_ratio);
  }

  printf("[RAN-RC][CTRL] NOTE: current implementation parses and stores slice PRB quota.\n");
  printf("[RAN-RC][CTRL] NOTE: no actual OAI slice scheduler reconfiguration is applied here yet.\n");
}

  else {
    assert(false && "Unsupported RC control style/action");
  }

  sm_ag_if_ans_t ans = {.type = CTRL_OUTCOME_SM_AG_IF_ANS_V0};
  ans.ctrl_out.type = RAN_CTRL_V1_3_AGENT_IF_CTRL_ANS_V0;

  printf("[RAN-RC][CTRL] write_ctrl_rc_sm: end\n");
  return ans;
}

bool read_rc_sm(void* data)
{
  assert(data != NULL);
  assert(0 != 0 && "Not implemented");
  return true;
}
