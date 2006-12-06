/* slow_dl.c: slow downlink packet specification
 *
 * This software is copyright (C) 2004-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "channels.h"
#include "slow_dl.h"
#include "blast.h"

/* SLOWDL_FORCE_INT force an integer on numbits between min and max */
/* SLOWDL_U_MASK    masks off all but the lowest numbits and sends those */
/* SLOWDL_TAKE_BIT  takes bit numbits, counting from 1 */
/* "src", type, numbits, min, max */
struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {
  /* PV */
  {"p_pv",          SLOWDL_FORCE_INT, 8,  160, 0},
  {"t_pv",          SLOWDL_FORCE_INT, 8, 65, -10},
  {"i_pv",          SLOWDL_FORCE_INT, 8, 10, -10}, /* 78 mA */
  {"lst",           SLOWDL_FORCE_INT, 8, 0, 24},
  {"sam_i_am",      SLOWDL_FORCE_INT, 8, 0, 255},
  {"lat",           SLOWDL_FORCE_INT, 8, 0, 90}, /* 0.35 deg */
  {"lon",           SLOWDL_FORCE_INT, 8, -180, 360},
  {"sip_alt",       SLOWDL_FORCE_INT, 8, 0, 40000}, /* 157 m */
  {"cpu_temp1",     SLOWDL_FORCE_INT, 8, 0, 75}, /* 0.29 deg */
  {"disk_free",     SLOWDL_FORCE_INT, 8, 0, 200},
  {"timeout",       SLOWDL_FORCE_INT, 8, 0, 15000},
  {"plover",        SLOWDL_U_MASK, 8},
  /* POINTING */
  {"sensor_veto",   SLOWDL_U_MASK,8},
  {"p_h",           SLOWDL_FORCE_INT, 8, 0, 10},
  {"p_mode",        SLOWDL_U_MASK,8},
  {"p_x_deg",       SLOWDL_FORCE_INT, 8, 0, 360},
  {"p_vaz",         SLOWDL_FORCE_INT, 8, 0, 10},
  {"p_del",         SLOWDL_FORCE_INT, 8, 0, 10},
  {"p_w",           SLOWDL_FORCE_INT, 8, 0, 10},
  {"ra",            SLOWDL_FORCE_INT, 8, 0, 24},
  {"dec",           SLOWDL_FORCE_INT, 8, -90, 90},
  {"az",            SLOWDL_FORCE_INT, 8, -180, 180},
  {"el",            SLOWDL_FORCE_INT, 8, 10, 65},
  /* MOTORS */
  {"t_el_mc",       SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_el_mot",      SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_roll",        SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_reac",        SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_reac_mc",     SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_piv_mc",      SLOWDL_FORCE_INT, 8, 55, -55},
  {"i_roll",        SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"i_piv",         SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"i_reac",        SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"i_el",          SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"g_p_el",        SLOWDL_FORCE_INT, 8, 0, 30000},  /* 118 */
  {"g_i_el",        SLOWDL_FORCE_INT, 8, 0, 30000},  /* 118 */
  {"g_p_roll",      SLOWDL_FORCE_INT, 8, 0, 30000},  /* 118 */
  {"g_p_az",        SLOWDL_FORCE_INT, 8, 0, 30000},  /* 118 */
  {"g_i_az",        SLOWDL_FORCE_INT, 8, 0, 30000},  /* 118 */
  {"g_p_pivot",     SLOWDL_FORCE_INT, 8, 0, 30000},  /* 118 */
  {"set_reac",      SLOWDL_FORCE_INT, 8, 0, 3},
  {"rps_reac",      SLOWDL_FORCE_INT, 8, -3, 3},
  /* ACS */
  {"i_apm_3v",      SLOWDL_FORCE_INT, 8, 10, -10}, /* 78 mA */
  {"i_apm_5v",      SLOWDL_FORCE_INT, 8, 10, -10}, /* 78 mA */
  {"i_apm_10v",     SLOWDL_FORCE_INT, 8, 10, -10}, /* 78 mA */
  {"t_apm_3v",      SLOWDL_FORCE_INT, 8, 70, -55},
  {"t_apm_5v",      SLOWDL_FORCE_INT, 8, 70, -55},
  {"t_apm_10v",     SLOWDL_FORCE_INT, 8, 70, -55},
  {"sensor_reset",  SLOWDL_U_MASK,8},
  {"lock_pot",      SLOWDL_U_MASK,8},
  /* BALANCE */
  {"balpump_lev",   SLOWDL_FORCE_INT, 8,  100, 0}, /* 0.39 */
  {"inpump_lev",    SLOWDL_FORCE_INT, 8,  100, 0}, /* 0.39 */
  {"outpump_lev",   SLOWDL_FORCE_INT, 8,  100, 0}, /* 0.39 */
  {"bal_veto",      SLOWDL_FORCE_INT, 8,  0, 1000},
  /* MAGNETOMETER */
  {"mag_model",     SLOWDL_FORCE_INT, 8,  0, 360},
  {"mag_az",        SLOWDL_FORCE_INT, 8,  0, 360},
  /* DGPS */
  {"dgps_pitch",    SLOWDL_FORCE_INT, 8, -10, 10},
  {"dgps_roll",     SLOWDL_FORCE_INT, 8, -10, 10},
  {"dgps_alt",      SLOWDL_FORCE_INT, 8,   0, 40000}, /* 157 m */
  {"dgps_att_ok",   SLOWDL_U_MASK,8},
  {"dgps_n_sat",    SLOWDL_U_MASK,8},
  {"dgps_az",       SLOWDL_FORCE_INT, 8, 0, 360},
  {"dgps_climb",    SLOWDL_FORCE_INT, 8, -20, 20},
  {"dgps_dir",      SLOWDL_FORCE_INT, 8, 0, 360},
  {"dgps_speed",    SLOWDL_FORCE_INT, 8, 0, 100},
  /* ELEVATION ENCODER */
  {"enc_el",        SLOWDL_FORCE_INT, 8, 0, 90},
  /* PUMPS */
  {"outcool_state", SLOWDL_U_MASK,8},
  {"incool_state",  SLOWDL_U_MASK,8},
  /* ISC */
  {"i_starcam",     SLOWDL_FORCE_INT, 8,   5, 0}, /* 20 mA */
  {"isc_fpulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"isc_spulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"isc_framenum",  SLOWDL_U_MASK,8},
  {"isc_mapmean",   SLOWDL_FORCE_INT, 8,   0, 16383},
  {"isc_state",     SLOWDL_U_MASK,8},
  {"isc_nblobs",    SLOWDL_U_MASK,8},
  {"isc_az",        SLOWDL_FORCE_INT, 8,   0, 360},
  {"isc_el",        SLOWDL_FORCE_INT, 8,   0, 90},
  {"t_isc_flange",  SLOWDL_FORCE_INT, 8, -55, 55},
  {"isc_pressure1", SLOWDL_FORCE_INT, 8, 0, 20},
  {"isc_sigma",     SLOWDL_FORCE_INT, 8, 0, 50},
  /* OSC */
  {"osc_fpulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"osc_spulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"osc_framenum",  SLOWDL_U_MASK,8},
  {"osc_mapmean",   SLOWDL_FORCE_INT, 8,   0, 4095},
  {"osc_state",     SLOWDL_U_MASK,8},
  {"osc_nblobs",    SLOWDL_U_MASK,8},
  {"osc_az",        SLOWDL_FORCE_INT, 8,   0, 360},
  {"osc_el",        SLOWDL_FORCE_INT, 8,   0, 90},
  {"t_osc_flange",  SLOWDL_FORCE_INT, 8, -55, 55},
  {"osc_pressure1", SLOWDL_FORCE_INT, 8, 0, 20},
  {"osc_sigma",     SLOWDL_FORCE_INT, 8, 0, 50},
  /* SUN SENSOR */
  {"i_sun",         SLOWDL_FORCE_INT, 8,   5,  0}, /* 20 mA */
  {"sun_az",        SLOWDL_FORCE_INT, 8,   0, 360},
  {"ss_az",         SLOWDL_FORCE_INT, 8,   0, 360},
  {"t_pv",     SLOWDL_FORCE_INT, 8,   0, 150},
  {"t_pv",    SLOWDL_FORCE_INT, 8, -55, 100},
  {"t_pv",   SLOWDL_FORCE_INT, 8, -55, 100},
  /* CINOMETERS */
  {"roll_clin_piv", SLOWDL_FORCE_INT, 8, -10, 10},
  {"pch_clin_piv",  SLOWDL_FORCE_INT, 8, -10, 10},
  {"roll_clin_pyr", SLOWDL_FORCE_INT, 8, 10, -10},
  {"pch_clin_pyr",  SLOWDL_FORCE_INT, 8, -10, 10},
  {"t_clin_piv",    SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_clin_pyr",    SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_clin_sip",    SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_clin_if",     SLOWDL_FORCE_INT, 8, 55, -55},
  /* POWER */
  {"apcu_reg",      SLOWDL_FORCE_INT, 8, 24,  34},
  {"dpcu_reg",      SLOWDL_FORCE_INT, 8, 24,  34},
  {"v_batt_acs",    SLOWDL_FORCE_INT, 8, 37,  17}, /* 0.078 deg */
  {"v_batt_das",    SLOWDL_FORCE_INT, 8, 37,  17}, /* 0.078 deg */
  {"t_apcu",        SLOWDL_FORCE_INT, 8, 65, -55},
  {"t_dpcu",        SLOWDL_FORCE_INT, 8, 65, -55},
  {"t_sol_port",    SLOWDL_FORCE_INT, 8, 150, 0}, /* 0.59 deg */
  {"t_sol_stbd",    SLOWDL_FORCE_INT, 8, 150, 0}, /* 0.59 deg */
  {"t_batt_acs",    SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_batt_das",    SLOWDL_FORCE_INT, 8, 55, -55},
  {"i_gond_acs",    SLOWDL_FORCE_INT, 8,   30, 0}, /* 0.12 mA */
  {"i_gond_das",    SLOWDL_FORCE_INT, 8,   30, 0}, /* 0.12 mA */
  {"i_batt_acs",    SLOWDL_FORCE_INT, 8, -30, 30}, /* 0.24 mA */
  {"i_batt_das",    SLOWDL_FORCE_INT, 8, -30, 30}, /* 0.24 mA */
  {"i_sol_acs",     SLOWDL_FORCE_INT, 8,   30, 0}, /* 0.12 mA */
  {"i_sol_das",     SLOWDL_FORCE_INT, 8,   30, 0},
  {"v_sol_acs1",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_acs2",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_acs3",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_acs4",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_acs5",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_acs6",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_das1",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_das2",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_das3",    SLOWDL_FORCE_INT, 8,  60, 20},
  {"v_sol_das4",    SLOWDL_FORCE_INT, 8,  60, 20},
  /* OF GONDOLA THERMOMETRY */
  {"t_apm_3v",      SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_apm_10v",     SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_apm_5v",      SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_pv_ext",      SLOWDL_FORCE_INT, 8, 55, -55},
  {"ss_case_temp",  SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_apcu",        SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_lock_motor",  SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_dpcu",        SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_acs",         SLOWDL_FORCE_INT, 8, 55, -55},
  /* IF GONDOLA THERMOMETRY */
  {"t_if_top_frnt", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_if_top_back", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_if_bot_frnt", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_if_bot_back", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_primary_1",   SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_primary_2",   SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_secondary_1", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_secondary_2", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_strut_bot",   SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_strut_side",  SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_push_plate",  SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_act_motor",   SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_ss_back_mid", SLOWDL_FORCE_INT, 8, 55, -55},
  {"t_chin_mid",    SLOWDL_FORCE_INT, 8, 55, -55},
  /* DAS */
  {"i_dpm_28v",     SLOWDL_FORCE_INT, 8, -2, 10}, /* 78 mA */
  {"i_dpm_3v",      SLOWDL_FORCE_INT, 8, 10, -2}, /* 78 mA */
  {"i_dpm_5v",      SLOWDL_FORCE_INT, 8, 10, -2}, /* 78 mA */
  {"i_dpm_10v",     SLOWDL_FORCE_INT, 8, 10, -2}, /* 78 mA */
  {"i_rec",         SLOWDL_FORCE_INT, 8, -2, 10}, /* 78 mA */
  {"t_dpm_7.5v",    SLOWDL_FORCE_INT, 8, 70, -55},
  {"t_dpm_10v",     SLOWDL_FORCE_INT, 8, 70, -55},
  {"t_dpm_5v",      SLOWDL_FORCE_INT, 8, 70, -55},
  {"t_das",         SLOWDL_FORCE_INT, 8, 70, -55},
  {"t_rec",         SLOWDL_FORCE_INT, 8, 70, -55},
  /* BIAS */
  {"biasin",        SLOWDL_U_MASK, 8},
  {"bias_lev1",     SLOWDL_U_MASK, 8},
  {"bias_lev2",     SLOWDL_U_MASK, 8},
  {"bias_lev3",     SLOWDL_U_MASK, 8},
  {"b_amp1",        SLOWDL_FORCE_INT, 8, 127, 0},
  {"b_amp2",        SLOWDL_FORCE_INT, 8, 0, 127},
  {"b_amp3",        SLOWDL_FORCE_INT, 8, 127, 0},
  /* GYROS */
  {"t_gyro1",       SLOWDL_FORCE_INT, 8, 60, -30},
  {"t_gyro2",       SLOWDL_FORCE_INT, 8, 60, -30},
  {"t_gyro3",       SLOWDL_FORCE_INT, 8, 60, -30},
  {"t_gybox1",      SLOWDL_FORCE_INT, 8, 60, -30},
  {"t_gybox2",      SLOWDL_FORCE_INT, 8, 60, -30},
  {"use_analogue",  SLOWDL_U_MASK,8},
  {"i_gybox",       SLOWDL_FORCE_INT, 8, 10, -10}, /* 78 mA */
  /* CRYO SENSORS */
  {"he4_lev",       SLOWDL_FORCE_INT, 8,   10, 0}, /* 0.039 */
  {"cryoin",        SLOWDL_U_MASK,8},
  {"cryostate",     SLOWDL_U_MASK,    16},
  /* CRYO DIODES */
  {"t_lhe",         SLOWDL_FORCE_INT, 8, 10, 8.98}, /* 0-6 deg / 0.020 deg */
  {"t_lhe_filt",    SLOWDL_FORCE_INT, 8, 10, 8.98}, /* 0-6 deg / 0.020 deg */
  {"t_he4pot_d",    SLOWDL_FORCE_INT, 8, 10, 8.98}, /* 0-6 deg / 0.020 deg */
  {"t_vcs_filt",    SLOWDL_FORCE_INT, 8, 7.5, 6}, /* 0.024 deg */
  {"t_ln2",         SLOWDL_FORCE_INT, 8, 7, 5.5}, /* 0.024 deg */
  {"t_ln2_filt",    SLOWDL_FORCE_INT, 8, 7, 5.5}, /* 0.024 deg */
  {"t_charcoal",    SLOWDL_FORCE_INT, 8, 10,  6}, /* 0-6 deg / 0.024 deg */
  {"t_heatswitch",  SLOWDL_FORCE_INT, 8, 10,  6}, /* 0-6 deg / 0.024 deg */
  {"t_jfet",        SLOWDL_FORCE_INT, 8, 6.5, 4.5}, /* 0-6 deg / 0.024 deg */
  {"t_vcs_fet",     SLOWDL_FORCE_INT, 8, 7.5,  6}, /* 0-6 deg / 0.024 deg */
  {"t_opt_box_ext", SLOWDL_FORCE_INT, 8, 10,  5}, /* 0-6 deg / 0.024 deg */
  /* CRYO ROXES */
  {"t_he3fridge",   SLOWDL_FORCE_INT, 8, 1.25, 5.5},
  {"t_m3",          SLOWDL_FORCE_INT, 8,    1, 2.5},
  {"t_m4",          SLOWDL_FORCE_INT, 8,    1, 2.5},
  {"t_m5",          SLOWDL_FORCE_INT, 8,    1, 2.5},
  {"t_horn_250",    SLOWDL_FORCE_INT, 8, 1.25, 5.5},
  {"t_horn_350",    SLOWDL_FORCE_INT, 8, 1.25, 5.5},
  {"t_horn_500",    SLOWDL_FORCE_INT, 8, 1.25, 5.5},
  {"t_300mk_strap", SLOWDL_FORCE_INT, 8, 1.25, 5.5},
  {"t_he4pot",      SLOWDL_FORCE_INT, 8,    1, 2.5},
  {"t_optbox_filt", SLOWDL_FORCE_INT, 8,    1, 2.5},
  /* LOCK MOTOR */
  {"lokmot_pin",    SLOWDL_FORCE_INT, 8, 0, 255}
};

void InitSlowDL(void) {
  int i;
  struct NiosStruct *address;

  for (i = 0; i < SLOWDL_NUM_DATA; i++) {
    address = GetNiosAddr(SlowDLInfo[i].src);
    SlowDLInfo[i].wide = address->wide;
    SlowDLInfo[i].mindex = ExtractBiPhaseAddr(address)->index;
    SlowDLInfo[i].chnum = ExtractBiPhaseAddr(address)->channel;
    SlowDLInfo[i].max = (SlowDLInfo[i].calib_max - address->b) / address->m;
    SlowDLInfo[i].min = (SlowDLInfo[i].calib_min - address->b) / address->m;
  }
}
