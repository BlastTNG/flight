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

/* SLOWDL_FORCE_INT force an integer on numbits between min and max */
/* SLOWDL_U_MASK    masks off all but the lowest numbits and sends those */
/* SLOWDL_TAKE_BIT  takes bit numbits, counting from 1 */
/* "src", type, numbits, min, max */
struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {
  /* PV */
  {"p_pv",          SLOWDL_FORCE_INT, 8,  0, 255},
  {"t_pv",          SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"i_pv",          SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"time",          SLOWDL_FORCE_INT, 16, 0, 65536}, 
  {"sam_i_am",      SLOWDL_U_MASK,    1},
  {"lat",           SLOWDL_FORCE_INT, 8, 0, 90}, /* 0.35 deg */
  {"lon",           SLOWDL_FORCE_INT, 8, 0, 360}, /* 1.41 deg */
  {"sip_alt",       SLOWDL_FORCE_INT, 8, 0, 40000}, /* 157 m */
  {"cpu_temp1",     SLOWDL_FORCE_INT, 8, 0, 75}, /* 0.29 deg */
  {"disk_free",     SLOWDL_FORCE_INT, 8, 0, 200},
  /* POINTING */
  {"sensor_veto",   SLOWDL_U_MASK, 8},
  {"p_h",           SLOWDL_FORCE_INT, 8, 0, 24},
  {"p_mode",        SLOWDL_U_MASK, 4},
  {"p_x_deg",       SLOWDL_FORCE_INT, 8, 0, 360},
  {"p_vaz",         SLOWDL_FORCE_INT, 8, 0, 10},
  {"p_del",         SLOWDL_FORCE_INT, 8, 0, 10},
  {"p_w",           SLOWDL_FORCE_INT, 8, 0, 360},
  {"ra",            SLOWDL_FORCE_INT, 8, 0, 24},
  {"dec",           SLOWDL_FORCE_INT, 8, 0, 360},
  /* MOTORS */
  {"t_el_mc",       SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_el_mot",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_roll",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_reac",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_reac_mc",     SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_piv_mc",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
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
  {"set_reac",      SLOWDL_FORCE_INT, 8, 0, 10},     /* 0.039 */ 
  /* ACS */
  {"i_apm_3v",      SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"i_apm_5v",      SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"i_apm_10v",     SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  {"t_apm_3v",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_apm_5v",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_apm_10v",     SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"sensor_reset",  SLOWDL_U_MASK,    8},
  {"lock_bits",     SLOWDL_U_MASK,    8},
  /* BALANCE */
  {"balpump_lev",   SLOWDL_FORCE_INT, 8,  0, 100}, /* 0.39 */
  {"inpump_lev",    SLOWDL_FORCE_INT, 8,  0, 100}, /* 0.39 */
  {"outpump_lev",   SLOWDL_FORCE_INT, 8,  0, 100}, /* 0.39 */
  {"bal_veto",      SLOWDL_FORCE_INT, 8,  0, 1000},
  /* MAGNETOMETER */
  {"mag_model",     SLOWDL_FORCE_INT, 8,  0, 360},
  {"mag_az",        SLOWDL_FORCE_INT, 8,  0, 360},
  /* DGPS */
  {"dgps_pitch",    SLOWDL_FORCE_INT, 8, -10, 10},
  {"dgps_roll",     SLOWDL_FORCE_INT, 8, -10, 10},
  {"dgps_alt",      SLOWDL_FORCE_INT, 8,   0, 40000}, /* 157 m */
  {"dgps_att_ok",   SLOWDL_U_MASK, 1},
  {"dgps_n_sat",    SLOWDL_U_MASK, 4},
  {"dgps_az",       SLOWDL_FORCE_INT, 8, 0, 360},
  /* ELEVATION ENCODER */
  {"enc_el",        SLOWDL_FORCE_INT, 8, 0, 90},
  /* PUMPS */
  {"outcool_state", SLOWDL_U_MASK, 4},
  {"incool_state",  SLOWDL_U_MASK, 4},
  /* ISC */
  {"i_starcam",     SLOWDL_FORCE_INT, 8,   0,  5}, /* 20 mA */
  {"isc_fpulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"isc_spulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"isc_framenum",  SLOWDL_U_MASK,    8},
  {"isc_mapmean",   SLOWDL_FORCE_INT, 8,   0, 16383},
  {"isc_state",     SLOWDL_U_MASK,    8},
  {"isc_nblobs",    SLOWDL_U_MASK,    4},
  {"isc_az",        SLOWDL_FORCE_INT, 8,   0, 360},
  {"isc_el",        SLOWDL_FORCE_INT, 8,   0, 90},
  {"t_isc_flange",  SLOWDL_FORCE_INT, 8, -10, 55},
  {"isc_pressure1", SLOWDL_FORCE_INT, 8, 0, 20},
  /* OSC */
  {"osc_fpulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"osc_spulse",    SLOWDL_FORCE_INT, 8,   0, 1000},
  {"osc_framenum",  SLOWDL_U_MASK,    8},
  {"osc_mapmean",   SLOWDL_FORCE_INT, 8,   0, 4095},
  {"osc_state",     SLOWDL_U_MASK,    8},
  {"osc_nblobs",    SLOWDL_U_MASK,    4},
  {"osc_az",        SLOWDL_FORCE_INT, 8,   0, 360},
  {"osc_el",        SLOWDL_FORCE_INT, 8,   0, 90},
  {"t_osc_flange",  SLOWDL_FORCE_INT, 8, -10, 55},
  {"osc_pressure1", SLOWDL_FORCE_INT, 8, 0, 20},
  /* SUN SENSOR */
  {"i_sun",         SLOWDL_FORCE_INT, 8,   0,  5}, /* 20 mA */
  {"sun_az",        SLOWDL_FORCE_INT, 8,   0, 360},
  {"ss_az",         SLOWDL_FORCE_INT, 8,   0, 360},
  {"ss_pc_temp",    SLOWDL_FORCE_INT, 8,   0, 127},
  /* CINOMETERS */
  {"roll_clin_piv", SLOWDL_FORCE_INT, 8, -10, 10}, /* 4.68 arcmin */
  {"pch_clin_piv",  SLOWDL_FORCE_INT, 8, -10, 10}, /* 4.68 arcmin */
  {"roll_clin_pyr", SLOWDL_FORCE_INT, 8, -10, 10}, /* 4.68 arcmin */
  {"pch_clin_pyr",  SLOWDL_FORCE_INT, 8, -10, 10}, /* 4.68 arcmin */
  {"t_clin_piv",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_clin_pyr",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_clin_sip",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_clin_if",     SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  /* POWER */
  {"apcu_reg",      SLOWDL_FORCE_INT, 8, 24,  32}, /* 31 mV */
  {"dpcu_reg",      SLOWDL_FORCE_INT, 8, 24,  32}, /* 31 mV */
  {"v_batt_acs",    SLOWDL_FORCE_INT, 8, 20,  40}, /* 0.078 deg */
  {"v_batt_das",    SLOWDL_FORCE_INT, 8, 20,  40}, /* 0.078 deg */
  {"t_apcu",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_dpcu",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_sol_port",    SLOWDL_FORCE_INT, 8,  0, 150}, /* 0.59 deg */
  {"t_sol_stbd",    SLOWDL_FORCE_INT, 8,  0, 150}, /* 0.59 deg */
  {"t_batt_acs",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_batt_das",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"i_gond_acs",    SLOWDL_FORCE_INT, 8,   0, 30}, /* 0.12 mA */
  {"i_gond_das",    SLOWDL_FORCE_INT, 8,   0, 30}, /* 0.12 mA */
  {"i_batt_acs",    SLOWDL_FORCE_INT, 8, -30, 30}, /* 0.24 mA */
  {"i_batt_das",    SLOWDL_FORCE_INT, 8, -30, 30}, /* 0.24 mA */
  {"i_sol_acs",     SLOWDL_FORCE_INT, 8,   0, 30}, /* 0.12 mA */
  {"i_sol_das",     SLOWDL_FORCE_INT, 8,   0, 30}, /* 0.12 mA */
  {"v_sol_acs1",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_acs2",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_acs3",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_acs4",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_acs5",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_acs6",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_das1",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_das2",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_das3",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  {"v_sol_das4",    SLOWDL_FORCE_INT, 8,  20, 40}, /* 21 mV */
  /* OF GONDOLA THERMOMETRY */
  {"t_out_heatx",   SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_sun_sensor",  SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_scoop_tip",   SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_l_sshield",   SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_r_sshield",   SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_mb_sshield",  SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_lock_motor",  SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_el_bearing",  SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_acs",         SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_pv_ext",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  /* IF GONDOLA THERMOMETRY */
  {"t_sc_baf",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_scoop",       SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_prim",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_cryo",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_cryo_valve",  SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_bal_bot",     SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_if_tpb",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_if_tss",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_triangle",        SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_rec",         SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_in_heatx",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  /* DAS */
  {"i_dpm_28v",     SLOWDL_FORCE_INT, 8,  -2, 10}, /* 78 mA */
  {"i_dpm_3v",      SLOWDL_FORCE_INT, 8,  -2, 10}, /* 78 mA */
  {"i_dpm_5v",      SLOWDL_FORCE_INT, 8,  -2, 10}, /* 78 mA */
  {"i_dpm_10v",     SLOWDL_FORCE_INT, 8,  -2, 10}, /* 78 mA */
  {"i_rec",         SLOWDL_FORCE_INT, 8,  -2, 10}, /* 78 mA */
  {"t_dpm_7.5v",    SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_dpm_10v",     SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_dpm_5v",      SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  {"t_das",         SLOWDL_FORCE_INT, 8, -10, 55}, /* 0.25 deg */
  /* BIAS */
  {"biasin",        SLOWDL_U_MASK, 8},
  {"bias_lev1",     SLOWDL_U_MASK, 8},
  {"bias_lev2",     SLOWDL_U_MASK, 8},
  {"bias_lev3",     SLOWDL_U_MASK, 8},
  /* GYROS */
  {"t_gyro1",       SLOWDL_FORCE_INT, 8, 10,  50}, /* 0.16 deg */
  {"t_gyro2",       SLOWDL_FORCE_INT, 8, 10,  50}, /* 0.16 deg */
  {"t_gyro3",       SLOWDL_FORCE_INT, 8, 10,  50}, /* 0.16 deg */
  {"use_analogue",  SLOWDL_U_MASK,    1},
  {"i_gybox",       SLOWDL_FORCE_INT, 8, -10, 10}, /* 78 mA */
  /* CRYO SENSORS */
  {"he4_lev",       SLOWDL_FORCE_INT, 8,   0, 10}, /* 0.039 */
  {"cryoin",        SLOWDL_U_MASK,    6},
  {"cryostate",     SLOWDL_U_MASK,    10},
  /* CRYO DIODES */
  {"t_lhe",         SLOWDL_FORCE_INT, 8, 8.98, 10}, /* 0-6 deg / 0.020 deg */
  {"t_lhe_filt",    SLOWDL_FORCE_INT, 8, 8.98, 10}, /* 0-6 deg / 0.020 deg */
  {"t_he4pot_d",    SLOWDL_FORCE_INT, 8, 8.98, 10}, /* 0-6 deg / 0.020 deg */
  {"t_vcs_filt",    SLOWDL_FORCE_INT, 8,   6, 7.5}, /* 0.024 deg */
  {"t_ln2",         SLOWDL_FORCE_INT, 8, 5.5,  7}, /* 0.024 deg */
  {"t_ln2_filt",    SLOWDL_FORCE_INT, 8, 5.5,  7}, /* 0.024 deg */
  {"t_charcoal",    SLOWDL_FORCE_INT, 8,   6,  10}, /* 0-6 deg / 0.024 deg */
  {"t_heatswitch",  SLOWDL_FORCE_INT, 8,   6,  10}, /* 0-6 deg / 0.024 deg */
  {"t_jfet",        SLOWDL_FORCE_INT, 8, 4.5, 6.5}, /* 0-6 deg / 0.024 deg */
  {"t_vcs_fet",     SLOWDL_FORCE_INT, 8,   6, 7.5}, /* 0-6 deg / 0.024 deg */
  {"t_opt_box_ext", SLOWDL_FORCE_INT, 8,   5,  10}, /* 0-6 deg / 0.024 deg */
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
  {"lokmot_pin",    SLOWDL_U_MASK,    1},
};

void InitSlowDL(void) {
  int i;
  struct NiosStruct *address;

  for (i = 0; i < SLOWDL_NUM_DATA; i++) {
    address = GetNiosAddr(SlowDLInfo[i].src);
    SlowDLInfo[i].wide = address->wide;
    SlowDLInfo[i].mindex = ExtractBiPhaseAddr(address)->index;
    SlowDLInfo[i].chnum = ExtractBiPhaseAddr(address)->channel;
    SlowDLInfo[i].max = (SlowDLInfo[i].calib_max - address->b) * address->m;
    SlowDLInfo[i].min = (SlowDLInfo[i].calib_min - address->b) * address->m;
  }
}
