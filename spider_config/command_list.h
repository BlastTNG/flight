/* command_list.h: Spider command specification file definitions
 *
 * This software is copyright (C) 2002-20010 University of Toronto
 *
 * This file is part of the Spider flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#include "netcmd.h"  /* common parts of command defintions moved here */

/* N_SCOMMANDS and N_MCOMMANDS are now automatically calculated at compile time
 */

#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

//maximum insert number for hk commands.
#define HK_MAX  6

#define N_GROUPS 27

#define GR_POINT        0x00000001
#define GR_SCTAB        0x00000002
#define GR_HWPR         0x00000004
#define GR_TRIM         0x00000008
#define GR_SCGOOD       0x00000010
#define GR_SFTV         0x00000020
#define GR_VETO         0x00000040
#define GR_SCBAD        0x00000080
#define GR_BIAS         0x00000100
#define GR_GAIN         0x00000200
#define GR_SCUGLY       0x00000400
#define GR_CRYO_HEAT    0x00000800
#define GR_POWER        0x00001000
#define GR_MPC          0x00002000
#define GR_THEO_HEAT    0x00004000
#define GR_MOTPWR       0x00008000
#define GR_MPCPARAM     0x00010000
#define GR_TELEM        0x00020000
#define GR_GYPWR        0x00040000
#define GR_TUNE         0x00080000
#define GR_LOCK         0x00100000
#define GR_IFPOWER      0x00200000
#define GR_SQUID        0x00400000
#define GR_TIMING       0x00800000
#define GR_MCCPWR       0x01000000
#define GR_DET          0x02000000
#define GR_MISC         0x04000000

#define MCECMD          0x40000000 /* MCE command flag */
//reserved for CONFIRM  0x80000000

#define N_HEATERS 6
extern const char *const command_list_serial;
extern const int command_list_serial_as_int(void);

extern const char *const GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  az_auto_gyro,      az_disable,        az_enable,         hwp_panic,
  el_disable,        el_enable,         pin_in,            reset_trims,
  elenc1_allow,      elenc2_allow,      elenc1_veto,       elenc2_veto,
  gps_allow,         gps_veto,          mag_allow,         mag_veto,
  stop,              pss_veto,          pss_allow,         lock,
  unlock,            lock_off,          lock_on,
  force_el_enable,   gps_cycle,         rw_cycle,
  piv_cycle,         elmot_cycle,       hub232_cycle,      das_cycle,
  rsc_cycle,         bsc_cycle,         rsc_off,           rsc_on,
  gps_off,           gps_on,            rw_off,            rw_on,
  piv_off,           piv_on,            elmot_off,         elmot_on,
  vtx_off,           vtx_on,            bi0_off,           bi0_on,
  das_off,           das_on,            bsc_off,           bsc_on,
  table_off,         table_on,          table_cycle,
  of_charge_off,     of_charge_on,      of_charge_cycle,
  if_charge_off,     if_charge_on,       if_charge_cycle,
  ofroll_1_gy_allow, ofroll_1_gy_veto,  ofroll_2_gy_allow, ofroll_2_gy_veto,
  ofyaw_1_gy_allow,  ofyaw_1_gy_veto,   ofyaw_2_gy_allow,  ofyaw_2_gy_veto,
  ofpch_1_gy_allow,  ofpch_1_gy_veto,   ofpch_2_gy_allow,  ofpch_2_gy_veto,
  ofroll_1_gy_off,   ofroll_1_gy_on,    ofroll_2_gy_off,   ofroll_2_gy_on,
  ofyaw_1_gy_off,    ofyaw_1_gy_on,     ofyaw_2_gy_off,    ofyaw_2_gy_on,
  ofpch_1_gy_off,    ofpch_1_gy_on,     ofpch_2_gy_off,    ofpch_2_gy_on,
  ofroll_1_gy_cycle, ofroll_2_gy_cycle, ofyaw_1_gy_cycle,  ofyaw_2_gy_cycle,
  ofpch_1_gy_cycle,  ofpch_2_gy_cycle,  gybox_off,         gybox_on,
  hub232_off,        hub232_on,         gybox_cycle,       reap_itsy,
  reap_bitsy,        antisun,           blast_rocks,       blast_sucks,
  at_float,          not_at_float,      el_auto_gyro,
  halt_itsy,         halt_bitsy,        restore_piv,
  reset_rw,          reset_piv,         reset_elev,
  use_pyramid,       no_pyramid,
  thegood_expose,    thegood_autofocus, thegood_settrig_ext, thegood_pause,
  thebad_expose,     thebad_autofocus,  thebad_settrig_ext,  thebad_pause,
  theugly_expose,    theugly_autofocus, theugly_settrig_ext, theugly_pause,
  thegood_run,       thebad_run,        theugly_run,       table_track,
  hwp_step,          hwp_repoll,        sftv_on,           sftv_off,
  sftv_atm_open,     sftv_atm_close,    sftv_pump_open,    sftv_pump_close,
  sftv_atm_stop,     sftv_pump_stop,    sft_pump_on,       sft_pump_off,
  //theo heater commands
  hk_mt_bottom_heat_on,   hk_mt_bottom_heat_off,
  hk_sft_lines_heat_on,   hk_sft_lines_heat_off,
  hk_capillary_heat_on,   hk_capillary_heat_off,
  hk_vcs2_hx_heat_on,     hk_vcs2_hx_heat_off,
  hk_vcs1_hx_heat_on,     hk_vcs1_hx_heat_off,
  hk_mt_lines_heat_on,    hk_mt_lines_heat_off,
  hk_sft_bottom_heat_on,  hk_sft_bottom_heat_off,
  bbc_sync_ext,      bbc_sync_int,      bbc_sync_auto,     elmot_auto,
  //make better use of unused groups
  pull_cmb_pin, global_thermonuclear_war,
  mce23_on,          mce23_off,         mce23_cycle,
  mce46_on,          mce46_off,         mce46_cycle,
  mce15_on,          mce15_off,         mce15_cycle,
  sync_on,           sync_off,          sync_cycle,
  hwp_on,            hwp_off,           hwp_cycle,
  hk_preamp_on,      hk_preamp_off,     hk_preamp_cycle,
  mcc1_on,           mcc1_off,          mcc1_cycle,
  mcc2_on,           mcc2_off,          mcc2_cycle,
  mcc3_on,           mcc3_off,          mcc3_cycle,
  mcc4_on,           mcc4_off,          mcc4_cycle,
  mcc5_on,           mcc5_off,          mcc5_cycle,
  mcc6_on,           mcc6_off,          mcc6_cycle,
  get_superslow,     thermveto_enable,  thermveto_disable,
  mcc_wdog_enable,   mcc_wdog_disable,  restart_reset_on, restart_reset_off,
  pv_data2_145_on, pv_data2_145_off, pv_data2_145_cycle,
  pv_data2_236_on, pv_data2_236_off, pv_data2_236_cycle,
  pv_data3_245_on, pv_data3_245_off, pv_data3_245_cycle,
  pv_data3_136_on, pv_data3_136_off, pv_data3_136_cycle,

  /* DON'T PUT ANYTHING BELOW THIS */
  xyzzy, N_SCOMMANDS /* SENTINAL: this must be the last thing in this list */
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,           dac2_level,
  set_scan_params,
  az_el_trim,        drift,             el_gain,           el_twist,
  pivot_gain,        ra_dec_goto,       ra_dec_set,        oth_set,
  set_heaters,       tdrss_bw,          iridium_bw,        pilot_bw,
  timeout,           slot_sched,        az_gyro_offset,
  cov_gps,           reset_adc,         hk_cycle_params,   hk_burp_params_1,
  hk_auto_cycle_on,  hk_auto_cycle_off, hk_fridge_cycle,   hk_burp_params_2,
  el_gyro_offset,    slew_veto,         arraystat_oth,
  motors_verbose,    get_mce_param,
  thegood_any,        thegood_settrig_timed,
  thegood_exp_params, thegood_focus_res, thegood_focus_range,
  thebad_any,         thebad_settrig_timed,
  thebad_exp_params,  thebad_focus_res,  thebad_focus_range,
  theugly_any,        theugly_settrig_timed,
  theugly_exp_params, theugly_focus_res, theugly_focus_range,
  thegood_bad_pix,    thegood_platescale,
  thegood_maxblobs,   thegood_grid,      thegood_thresh,   thegood_mdist,
  thegood_lens_any,   thegood_lens_move, thegood_viewer,
  thebad_bad_pix,     thebad_platescale,
  thebad_maxblobs,    thebad_grid,      thebad_thresh,     thebad_mdist,
  thebad_lens_any,    thebad_lens_move,
  theugly_bad_pix,    theugly_platescale,
  theugly_maxblobs,   theugly_grid,      theugly_thresh,   theugly_mdist,
  theugly_lens_any,   theugly_lens_move,  rsc_trig_wait,
  thegood_lens_params,thebad_lens_params,   theugly_lens_params, bsc_trig_delay,
  table_gain,        table_goto,        table_drift,       ants_gps,
  hwp_vel,           hwp_i,             hwp_move,          hwp_general,
  hk_pump_heat_on,   hk_pump_heat_off,  hk_heat_switch_on, hk_heat_switch_off,
  hk_ssa_heat_on,    hk_ssa_heat_off,   hk_htr1_heat_on,   hk_htr1_heat_off,
  hk_htr2_heat_on,   hk_htr2_heat_off,  hk_htr3_heat_on,   hk_htr3_heat_off,
  hk_fphi_heat_on,   hk_fphi_heat_off,  hk_fplo_heat_set,  hk_ring_heat_set,
  hk_ampl_cernox,    hk_ampl_ntd,       hk_phase_cernox,   hk_phase_ntd,
  hk_phase_step_cernox, hk_phase_step_ntd, hk_bias_freq,      hk_pump_servo_on,
  hk_mt_bottom_pulse,  hk_sft_lines_pulse, hk_capillary_pulse,
  hk_vcs2_hx_pulse,    hk_vcs1_hx_pulse,   hk_mt_lines_pulse,
  hk_sft_bottom_pulse,hk_fphi_heat_pulse,
  spider_scan,        sine_scan,         bbc_rate_ext,      bbc_rate_int,
  el_pulse,           hwp_halt,          hwp_phase,         el_rel_move,
  hwp_bias_on,        hwp_bias_off,      mce_row_len,       mce_num_rows,
  mce_data_rate,      bset,              set_piv_mode,
  data_mode_bits,     mag_cal,           pss_off_cal,       pss_d_cal,
  beam_map, //TODO: get rid of beam_map before flight

  /* mce stuff */
  send_exptcfg, mce_wb, acq_iv_curve, send_iv_curve, send_tuning, use_tuning,
  bias_tes_all, bias_tes_rc1, tile_heater_on, tile_heater_off, bias_ramp,
  tile_heater_kick, servo_reset, force_config, tune_biases, reload_mce_config,
  flux_loop_init, lcloop, mce_veto, mce_unveto, bias_step, pick_biases,
  acquire, reconfig, pause_acq, data_mode, cycle_acq, mce_clock_int,
  drive_check, bias_tes_rc2, mce_clock_ext, bolo_stat_reset, bolo_stat_gains,
  bolo_stat_timing, bias_kick_params, partial_iv_curve, ref_tuning,
  tuning_tries, tuning_check_on, tuning_check_off, array_monitor, ref_biases,
  sa_ramp_check_crit, sq2_servo_check_crit, sq1_servo_check_crit,
  sq1_ramp_check_crit, sa_ramp_check_params, sq2_servo_check_params,
  sq1_servo_check_params, sq1_ramp_check_params, auto_iv_params, tune_array,

  /* MCE experiment.cfg commands */
  column_on, column_off, sa_offset_bias_ratio, sa_ramp_bias_on,
  sa_ramp_bias_off, sa_ramp_flux, sa_ramp_bias, sq2_tuning_row,
  sq2_servo_gain, sq1_servo_gain, sq1_servo_bias_on, sq1_servo_bias_off,
  sq1_servo_flux, sq1_servo_bias, sq2_servo_bias_on, sq2_servo_bias_off,
  sq2_servo_flux, sq2_servo_bias, sq1_ramp_flux, sq1_ramp_tes_bias_on,
  sq1_ramp_tes_bias_off, sq1_ramp_tes_bias, tes_bias_idle, tes_bias_normal,
  tes_bias_normal_time, tuning_check_bias_on, tuning_check_bias_off,
  tuning_therm_time, tuning_do_plots_on, tuning_do_plots_off,
  sq2servo_safb_init, sq1servo_sq2fb_init, sample_num, fb_dly,
  row_dly, flux_jumping_on, flux_jumping_off, servo_pid_col, servo_pid_pixel,
  servo_pid_frail, dead_detector, frail_detector, healthy_detector,
  sa_flux_quantum, sq2_flux_quantum, sq1_flux_quantum, sq1_bias, sq1_off_bias,
  sq2_bias, sq2_fb, sa_bias, sa_fb, sa_offset, adc_offset, bias_tes_col,
  integral_clamp,

  /* DON'T PUT ANYTHING BELOW THIS */
  plugh,   N_MCOMMANDS, /* SENTINAL: this must be the last thing in the list */
  sched_packet = 0xff   //not really a command, more of a placeholder
};

extern const struct scom scommands[N_SCOMMANDS];

/* parameter type:
 * i :  parameter is 16 bit unnormalised integer
 * f :  parameter is 16 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern const struct mcom mcommands[N_MCOMMANDS];

/* validator function for mcommands */
extern int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer);

#endif /* COMMAND_LIST_H */
