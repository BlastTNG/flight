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

#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#include <limits.h>
#include <sys/types.h>

#include "netcmd.h"  /* common parts of command defintions moved here */
#include "isc_protocol.h"  /* required for constants */


/* WARNING: if either N_xCOMMANDS exceeds 254, commanding will break */
#define N_SCOMMANDS 227        /* total number of single word cmds */
#define N_MCOMMANDS 124        /* total number of multiword commands */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)    //deprecated. Probably want CMD_I_MAX instead
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
//reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  az_auto_gyro,     az_off,             az_on,
  balance_auto,     balance_off,        cal_off,          cal_on,
  charcoal_off,     charcoal_on,        hs_charcoal_off,  hwpr_panic,
  hs_charcoal_on,   el_off,             el_on,
  elclin_allow,     elclin_veto,        elenc_allow,      elenc_veto,
  fixed,
  l_valve_close,    he_valve_on,        he_valve_off,     l_valve_open,
  elmotenc_allow,   elmotenc_veto,
  xsc0_veto,        xsc0_allow,
  xsc1_veto,        xsc1_allow,
  level_on,         level_off,
  mag_allow,        mag_veto,
  pin_in,           pot_valve_close,    pot_valve_off,    pot_valve_on,       
  pot_valve_open,   ramp,               reset_trims,
  stop,             pss_veto,		    trim_isc_to_osc,
  pss_allow,        trim_osc_to_isc,    autotrim_off,
  trim_to_isc,      unlock,             lock_off,         
  force_el_on,      auto_jfetheat,      auto_cycle,
  actbus_cycle,
          hub232_cycle,     das_cycle,

  xsc0_off,         xsc0_on,            xsc0_cycle,
  xsc1_off,         xsc1_on,            xsc1_cycle,
  rw_off,	        rw_on,              rw_cycle,
  piv_off,	        piv_on,             piv_cycle,
  elmot_off,	    elmot_on,           elmot_cycle,
  vtx_off,	        vtx_on,
  bi0_off,	        bi0_on,
  das_off,	        das_on,
  rx_off,		    rx_on,
  rx_hk_off,        rx_hk_on,
  rx_amps_off,	    rx_amps_on,
  charge_off,	    charge_on,		charge_cycle,

  ifroll_1_gy_allow,ifroll_1_gy_veto,   ifroll_2_gy_allow,ifroll_2_gy_veto,
  ifyaw_1_gy_allow, ifyaw_1_gy_veto,    ifyaw_2_gy_allow, ifyaw_2_gy_veto,
  ifel_1_gy_allow,  ifel_1_gy_veto,	ifel_2_gy_allow,  ifel_2_gy_veto,
  ifroll_1_gy_off,  ifroll_1_gy_on,	ifroll_2_gy_off,  ifroll_2_gy_on,
  ifyaw_1_gy_off,   ifyaw_1_gy_on,	ifyaw_2_gy_off,	  ifyaw_2_gy_on,
  ifel_1_gy_off,    ifel_1_gy_on,	ifel_2_gy_off,	  ifel_2_gy_on,
  ifroll_1_gy_cycle,ifroll_2_gy_cycle,  ifyaw_1_gy_cycle, ifyaw_2_gy_cycle,
  ifel_1_gy_cycle,  ifel_2_gy_cycle,    gybox_off,        gybox_on,
  hub232_off,      hub232_on,
  gybox_cycle,      ln_valve_on,      ln_valve_off,
            reap_north,       reap_south,
  xy_panic,
  trim_to_osc,      antisun,            blast_rocks,      blast_sucks,
  fridge_cycle,     at_float,           not_at_float,     el_auto_gyro,
  repoll,           autofocus_allow,
  autofocus_veto,   north_halt,         south_halt,       actbus_on,
  actbus_off,       actuator_stop,      level_pulse,      restore_piv,
  reset_rw,         reset_piv,
  reset_elev,       jfet_on,            jfet_off,         hs_pot_on,
  hs_pot_off,       bda_on,             bda_off,          hwpr_enc_on,
  hwpr_enc_off,     hwpr_enc_pulse,     balance_heat_on,  balance_heat_off,
  vtx1_isc,	    vtx1_osc,		vtx2_isc,
  vtx2_osc,	    cam_cycle,
  cam_expose,	    cam_autofocus,	cam_settrig_ext,  cam_force_lens, 
  cam_unforce_lens, hwpr_step,          hwpr_pot_is_dead, hwpr_pot_is_alive,
  hwpr_step_off,    hwpr_step_on,       shutter_init,     shutter_close,
  shutter_reset,    shutter_open,       shutter_off,      shutter_open_close,
  lock45,           shutter_close_slow,
  xyzzy
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,          balance_gain,
  balance_manual,
  bias_level_500,    bias_level_350,    bias_level_250,   bias_level_rox,
  bias_level_x,      fridge_cycle_params,  box,
  cal_pulse,         cal_repeat,        cap,
  az_el_trim,        drift,             el_gain,
  inner_level,       hwpr_jump,         hwpr_goto_i,
  autotrim_to_sc,
  lock,              phase,             act_offset,
  pivot_gain,        ra_dec_goto,      ra_dec_set,
  pos_set,
  roll_gain,         az_scan_accel,
  t_gyro_set,        tdrss_bw,         iridium_bw,
  t_gyro_gain,       timeout,           vcap,
  vbox,              slot_sched,        az_gyro_offset,
  hwpr_set_overshoot,
  jfet_set,          hwpr_vel,          hwpr_i,
  gyro_off,	         gyro_on,           quad,
  el_gyro_offset,    general,           slew_veto,        set_secondary,
  thermo_gain,       actuator_servo,    xy_goto,          actuator_vel,
  xy_jump,           xy_xscan,          xy_yscan,         xy_raster,
  actuator_i,        lock_vel,          lock_i,           actuator_delta,
  delta_secondary,   lvdt_limit,        thermo_param,     focus_offset,
  balance_tset,
  motors_verbose,    bias_step,         phase_step,       hwpr_set_backlash,
  hwpr_repeat,      hwpr_define_pos,
  hwpr_goto,	     hwpr_goto_pot,     act_enc_trim,     actuator_tol,
  el_scan,           el_box,            shutter_step,     shutter_step_slow,
  set_scan_params,   mag_cal,           pss_cal,          params_test,
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
  xsc_set_focus,
  xsc_set_focus_incremental,
  xsc_run_autofocus,
  xsc_set_autofocus_range,
  xsc_abort_autofocus,
  xsc_autofocus_display_mode,
  xsc_init_aperture,
  xsc_set_aperture,
  xsc_get_gain,
  xsc_set_gain,
  xsc_fake_sky_brightness,
  xsc_solver_general,
  xsc_solver_abort,
  xsc_selective_mask,
  xsc_blob_finding,
  xsc_blob_cells,
  xsc_motion_psf,
  xsc_pattern_matching,
  xsc_filter_hor_location,
  xsc_filter_hor_roll,
  xsc_filter_el,
  xsc_filter_eq_location,
  xsc_filter_matching,

  plugh,                //plugh should be at the end of the list
  sched_packet = 0xff   //not really a command, more of a placeholder
};

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
