/* command_list.h: BLAST command specification file definitions
 *
 * This software is copyright (C) 2002-2012 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_COMMAND_LIST_H
#define INCLUDE_COMMAND_LIST_H

#include <limits.h>
#include <sys/types.h>

#include "netcmd.h"  /* common parts of command defintions moved here */


#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)    // deprecated. Probably want CMD_I_MAX instead
#define CMD_I_MAX USHRT_MAX
#define CMD_L_MAX UINT_MAX

#define N_GROUPS 31

#define GRPOS_POINT 0
#define GRPOS_BAL   1
#define GRPOS_HWPR  2
#define GRPOS_TRIM  3
#define GRPOS_ELECT 4
#define GRPOS_BIAS  5
#define GRPOS_VETO  6
#define GRPOS_ACT   7
#define GRPOS_XSC_HOUSE 8
#define GRPOS_XSC_MODE  9
#define GRPOS_XSC_PARAM 10
#define GRPOS_MOTOR  11
#define GRPOS_CRYO  12
#define GRPOS_POWER 13
#define GRPOS_LOCK  14
#define GRPOS_TELEM 15
#define GRPOS_MISC  16
#define GRPOS_FOCUS 17
#define GRPOS_ROACH 18

#define GR_POINT        (1 << GRPOS_POINT)
#define GR_BAL          (1 << GRPOS_BAL)
#define GR_HWPR         (1 << GRPOS_HWPR)
#define GR_TRIM         (1 << GRPOS_TRIM)
#define GR_ELECT        (1 << GRPOS_ELECT)
#define GR_BIAS         (1 << GRPOS_BIAS)
#define GR_VETO         (1 << GRPOS_VETO)
#define GR_ACT          (1 << GRPOS_ACT)
#define GR_XSC_HOUSE    (1 << GRPOS_XSC_HOUSE)
#define GR_XSC_MODE     (1 << GRPOS_XSC_MODE)
#define GR_XSC_PARAM    (1 << GRPOS_XSC_PARAM)
#define GR_MOTOR        (1 << GRPOS_MOTOR)
#define GR_CRYO         (1 << GRPOS_CRYO)
#define GR_POWER        (1 << GRPOS_POWER)
#define GR_LOCK         (1 << GRPOS_LOCK)
#define GR_TELEM        (1 << GRPOS_TELEM)
#define GR_MISC         (1 << GRPOS_MISC)
#define GR_FOCUS        (1 << GRPOS_FOCUS)
#define GR_ROACH        (1 << GRPOS_ROACH)
// reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  az_auto_gyro,     az_off,             az_on,
  balance_auto,     balance_off,        
  // cal_off,          cal_on,
  hwpr_panic,   el_off,             el_on,
  elclin_allow,     elclin_veto,        elenc_allow,      elenc_veto,
  fixed,
  l_valve_close,    he_valve_on,        he_valve_off,     l_valve_open,
  elmotenc_allow,   elmotenc_veto,
  xsc0_veto,        xsc0_allow,
  xsc1_veto,        xsc1_allow,
  mag_allow,        mag_veto,
  pin_in,  ramp,               reset_trims,
  stop,             pss_veto,		    trim_xsc0_to_xsc1,
  pss_allow,        trim_xsc1_to_xsc0,    autotrim_off,
  trim_to_xsc0,      unlock,             lock_off,
  force_el_on,
  actbus_cycle,
          hub232_cycle,

  xsc0_off,         xsc0_on,            xsc0_cycle,
  xsc1_off,         xsc1_on,            xsc1_cycle,
  rw_off,	        rw_on,              rw_cycle,
  piv_off,	        piv_on,             piv_cycle,
  elmot_off,	    elmot_on,           elmot_cycle,
  vtx_off,	        vtx_on,
  bi0_off,	        bi0_on,
  rx_off,		    rx_on,
  rx_hk_off,        rx_hk_on,
  rx_amps_off,	    rx_amps_on,
  charge_off,	    charge_on,		charge_cycle,

  ifroll_1_gy_allow, ifroll_1_gy_veto,   ifroll_2_gy_allow, ifroll_2_gy_veto,
  ifyaw_1_gy_allow, ifyaw_1_gy_veto,    ifyaw_2_gy_allow, ifyaw_2_gy_veto,
  ifel_1_gy_allow,  ifel_1_gy_veto,	ifel_2_gy_allow,  ifel_2_gy_veto,
  ifroll_1_gy_off,  ifroll_1_gy_on,	ifroll_2_gy_off,  ifroll_2_gy_on,
  ifyaw_1_gy_off,   ifyaw_1_gy_on,	ifyaw_2_gy_off,	  ifyaw_2_gy_on,
  ifel_1_gy_off,    ifel_1_gy_on,	ifel_2_gy_off,	  ifel_2_gy_on,
  ifroll_1_gy_cycle, ifroll_2_gy_cycle,  ifyaw_1_gy_cycle, ifyaw_2_gy_cycle,
  ifel_1_gy_cycle,  ifel_2_gy_cycle,    gybox_off,        gybox_on,
  hub232_off,      hub232_on,
  gybox_cycle,
            reap_north,       reap_south,
  xy_panic,
  trim_to_xsc1,      antisun,            blast_rocks,      blast_sucks,
  at_float,           not_at_float,     el_auto_gyro,
  repoll,           autofocus_allow,
  autofocus_veto,   north_halt,         south_halt,       actbus_on,
  actbus_off,       actuator_stop,      restore_piv,
  reset_rw,         reset_piv,
  reset_elev,         
  // hs_pot_on,        hs_pot_off,       bda_on,             bda_off, 
  hwpr_enc_on,
  hwpr_enc_off,     hwpr_enc_pulse,
  vtx1_xsc0,	    vtx1_xsc1,		vtx2_xsc0,
  vtx2_xsc1,
  hwpr_step,          hwpr_pot_is_dead, hwpr_pot_is_alive,
  hwpr_step_off,    hwpr_step_on,       shutter_init,     shutter_close,
  shutter_reset,    shutter_open,       shutter_off,      shutter_open_close,
  lock45,           shutter_close_slow, heater_300mk_on,  heater_300mk_off,
  charcoal_hs_on,   charcoal_hs_off,
  lna350_on, lna350_off, lna250_on, lna250_off, lna500_on, lna500_off,
  // level_sensor_on,  level_sensor_off,   
  charcoal_on,      charcoal_off,
  heater_1k_on, heater_1k_off, power_box_on, power_box_off, amp_supply_on,
  amp_supply_off, therm_readout_on, therm_readout_off, heater_supply_on,
  pump_valve_open, pump_valve_close, fill_valve_open, fill_valve_close,
  ln_valve_on, ln_valve_off, pot_valve_on, pot_valve_off, pot_valve_open, pot_valve_close,
  heater_supply_off, reboot_ljcryo1, bias_reset_rox,
    cycle_hd_pv, cycle_eth_switch, cycle_fc1, cycle_xsc1, cycle_fc2,
    cycle_xsc0, cycle_gyros, cycle_data_transmit, cycle_el_mot, cycle_pivot,
    cycle_magnetometer, cycle_rw_mot, cycle_steppers, cycle_clinometers, cycle_of_15,
    cycle_of_16, cycle_if_1, cycle_if_2, cycle_if_3, cycle_if_4,
    cycle_if_5, cycle_if_6, cycle_if_7, cycle_if_8, cycle_if_9,
    cycle_if_10,
    hd_pv_on, hd_pv_off, eth_switch_on, eth_switch_off,
    fc1_on, fc1_off, xsc1_acs_on, xsc1_acs_off,
    fc2_on, fc2_off, xsc0_acs_on, xsc0_acs_off,
    gyros_on, gyros_off, data_transmit_on, data_transmit_off,
    el_mot_on, el_mot_off, pivot_on, pivot_off,
    magnetometer_on, magnetometer_off, rw_mot_on, rw_mot_off,
    steppers_on, steppers_off, clinometers_on, clinometers_off,
    of_relay_15_on, of_relay_15_off, of_relay_16_on, of_relay_16_off,
    if_relay_1_on, if_relay_1_off, if_relay_2_on, if_relay_2_off,
    if_relay_3_on, if_relay_3_off, if_relay_4_on, if_relay_4_off,
    if_relay_5_on, if_relay_5_off, if_relay_6_on, if_relay_6_off,
    if_relay_7_on, if_relay_7_off, if_relay_8_on, if_relay_8_off,
    if_relay_9_on, if_relay_9_off, if_relay_10_on, if_relay_10_off,
    level_sensor_pulse, single_cal_pulse, heaters_off, load_curve,
    heater_sync,
  xyzzy
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,          balance_gain,
  balance_manual,    balance_vel,       balance_i,
  bias_level_500,    bias_level_350,    bias_level_250,   bias_level_rox,
  bias_level_x,      
  // fridge_cycle_params,  
  box,
  // cal_repeat,        
  cap,              cur_mode,
  az_el_trim,        drift,             el_gain,
  hwpr_jump,         hwpr_goto_i,
  autotrim_to_sc,
  lock,              phase,             act_offset,
  pivot_gain,        ra_dec_goto,      ra_dec_set,
  pos_set,
  az_scan_accel,
  // t_gyro_set,        
  highrate_bw,       pilot_bw,         biphase_bw,
  biphase_clk_speed, highrate_through_tdrss,              set_linklists,
  // t_gyro_gain,       
  timeout,           vcap,
  vbox,              slot_sched,        az_gyro_offset,
  hwpr_set_overshoot,
  // jfet_set,          
  hwpr_vel,          hwpr_i,
  gyro_off,	         quad,
  el_gyro_offset,    general,           slew_veto,        set_secondary,
  thermo_gain,       actuator_servo,    xy_goto,          actuator_vel,
  xy_jump,           xy_xscan,          xy_yscan,         xy_raster,
  actuator_i,        lock_vel,          lock_i,           actuator_delta,
  delta_secondary,   lvdt_limit,        thermo_param,     focus_offset,
  motors_verbose,    bias_step,         
  // phase_step,  
  hwpr_repeat,      hwpr_define_pos,
  hwpr_goto,	     hwpr_goto_pot,     act_enc_trim,     actuator_tol,
  el_scan,           el_box,            shutter_step,     shutter_step_slow,
  set_scan_params,   mag_cal,           pss_cal,          params_test,
  potvalve_set_vel, potvalve_set_current, valves_set_vel, valves_set_current,

  xsc_is_new_window_period,
  xsc_offset,
  xsc_heaters_off,
  xsc_heaters_on,
  xsc_heaters_auto,
  xsc_exposure_timing,
  xsc_multi_trigger,
  xsc_trigger_threshold,
  xsc_scan_force_trigger,
  xsc_quit,
  xsc_shutdown,
  xsc_network_reset,
  xsc_main_settings,
  xsc_display_zoom,
  xsc_image_client,
  xsc_init_focus,
  xsc_get_focus,
  xsc_set_focus,
  xsc_stop_focus,
  xsc_define_focus,
  xsc_set_focus_incremental,
  xsc_run_autofocus,
  xsc_set_autofocus_range,
  xsc_abort_autofocus,
  xsc_autofocus_display_mode,
  xsc_init_aperture,
  xsc_get_aperture,
  xsc_set_aperture,
  xsc_stop_aperture,
  xsc_define_aperture,
  xsc_get_gain,
  xsc_set_gain,
  xsc_fake_sky_brightness,
  xsc_solver_general,
  xsc_solver_abort,
  xsc_selective_mask,
  xsc_blob_finding,
  xsc_blob_cells,
  xsc_pattern_matching,
  xsc_filter_hor_location,
  xsc_filter_hor_roll,
  xsc_filter_el,
  xsc_filter_eq_location,
  xsc_filter_matching,
  vna_sweep,
  targ_sweep,
  reset_roach,
  df_calc,
  opt_tones,
  auto_retune,
  end_sweep,
  load_new_tone_amplitudes,
  cal_attens,
  set_attens,
  find_kids,
  set_rox_bias_amp,
  cal_length,
  level_length,
  send_dac,
  plugh,                // plugh should be at the end of the list
  sched_packet = 0xff   // not really a command, more of a placeholder
};

#define N_SCOMMANDS (xyzzy + 1)
#define N_MCOMMANDS (plugh + 2)

extern struct scom scommands[N_SCOMMANDS];

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern struct mcom mcommands[N_MCOMMANDS];

/* validator function for mcommands */
extern int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer);

#endif /* COMMAND_LIST_H */
