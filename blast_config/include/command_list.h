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
#define N_SCOMMANDS 225        /* total number of single word cmds */
#define N_MCOMMANDS 125        /* total number of multiword commands */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)    //deprecated. Probably want CMD_I_MAX instead
#define CMD_I_MAX USHRT_MAX
#define CMD_L_MAX UINT_MAX

#define N_GROUPS 25

#define GR_POINT        0x00000001
#define GR_BAL          0x00000002
#define GR_HWPR         0x00000004
#define GR_TRIM         0x00000008
#define GR_ELECT        0x00000010
#define GR_BIAS         0x00000020
#define GR_VETO         0x00000040
#define GR_ACT          0x00000080
#define GR_SBSC		0x00000100
#define GR_GAIN         0x00000200
#define GR_FOCUS        0x00000400
#define GR_CRYO_HEAT    0x00000800
#define GR_POWER        0x00001000
#define GR_LOCK         0x00002000
#define GR_CRYO_CONTROL 0x00004000
#define GR_TELEM        0x00008000
#define GR_ISC_HOUSE    0x00010000
#define GR_OSC_HOUSE    0x00020000
#define GR_STAGE        0x00040000
#define GR_ISC_MODE     0x00080000
#define GR_OSC_MODE     0x00100000
#define GR_MISC         0x00200000
#define GR_ISC_PARAM    0x00400000
#define GR_OSC_PARAM    0x00800000
#define GR_SHUTTER      0x01000000
//reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  isc_auto_focus,   az_auto_gyro,       az_off,           az_on,
  balance_auto,     balance_off,        cal_off,          cal_on,
  charcoal_off,     charcoal_on,        hs_charcoal_off,  hwpr_panic,
  hs_charcoal_on,   isc_discard_images, el_off,           el_on,
  elclin_allow,     elclin_veto,        elenc_allow,      elenc_veto,
  fixed,            isc_full_screen,    gps_allow,        gps_veto,
  l_valve_close,    he_valve_on,        he_valve_off,     l_valve_open,
  isc_abort,        isc_allow,          isc_pause,        isc_reconnect,
  isc_run,          isc_shutdown,       isc_veto,         level_off,
  level_on,         mag_allow,          mag_veto,         osc_auto_focus,
  pin_in,           pot_valve_close,    pot_valve_off,    pot_valve_on,       
  pot_valve_open,   ramp,               reset_trims,      isc_save_images,
  stop,             pss_veto,		trim_isc_to_osc,
  pss_allow,        isc_eye_off,        trim_osc_to_isc,  autotrim_off,
  trim_to_isc,      unlock,             lock_off,         
  isc_reboot,       isc_cam_cycle,      osc_run,          osc_shutdown,
  osc_reboot,       osc_cam_cycle,      osc_pause,        osc_abort,
  osc_reconnect,    osc_save_images,    osc_discard_images, osc_full_screen,  
  force_el_on,      auto_jfetheat,      auto_cycle,       sbsc_cycle,
  isc_cycle,        osc_cycle,          actbus_cycle,     rw_cycle,
  piv_cycle,        elmot_cycle,        hub232_cycle,     das_cycle,
  sbsc_off,         sbsc_on,            isc_off,          isc_on,
  osc_off,          osc_on,             rw_off,	          rw_on,
  piv_off,	    piv_on,		elmot_off,	  elmot_on,
  vtx_off,	    vtx_on,		bi0_off,	  bi0_on,
  das_off,	    das_on,		rx_off,		  rx_on,
  rx_hk_off,        rx_hk_on,           rx_amps_off,	  rx_amps_on,
  charge_off,	    charge_on,		charge_cycle,
  ifroll_1_gy_allow,ifroll_1_gy_veto,   ifroll_2_gy_allow,ifroll_2_gy_veto,
  ifyaw_1_gy_allow, ifyaw_1_gy_veto,    ifyaw_2_gy_allow, ifyaw_2_gy_veto,
  ifel_1_gy_allow,  ifel_1_gy_veto,	ifel_2_gy_allow,  ifel_2_gy_veto,
  ifroll_1_gy_off,  ifroll_1_gy_on,	ifroll_2_gy_off,  ifroll_2_gy_on,
  ifyaw_1_gy_off,   ifyaw_1_gy_on,	ifyaw_2_gy_off,	  ifyaw_2_gy_on,
  ifel_1_gy_off,    ifel_1_gy_on,	ifel_2_gy_off,	  ifel_2_gy_on,
  ifroll_1_gy_cycle,ifroll_2_gy_cycle,  ifyaw_1_gy_cycle, ifyaw_2_gy_cycle,
  ifel_1_gy_cycle,  ifel_2_gy_cycle,    gybox_off,        gybox_on,
  hub232_off,	      hub232_on,
  gybox_cycle,      isc_trig_int,       isc_trig_ext,
  osc_trig_int,     osc_trig_ext,       ln_valve_on,      ln_valve_off,
  osc_veto,         osc_allow,          reap_north,       reap_south,
  isc_eye_on,       osc_eye_on,         osc_eye_off,      xy_panic,
  trim_to_osc,      antisun,            blast_rocks,      blast_sucks,
  fridge_cycle,     at_float,           not_at_float,     el_auto_gyro,
  repoll,           autofocus_allow,
  isc_use_pyramid,  isc_no_pyramid,     osc_use_pyramid,  osc_no_pyramid,
  autofocus_veto,   north_halt,         south_halt,       actbus_on,
  actbus_off,       actuator_stop,      level_pulse,      restore_piv,
  reset_rw,         reset_piv,
  reset_elev,       jfet_on,            jfet_off,         hs_pot_on,
  hs_pot_off,       bda_on,             bda_off,          hwpr_enc_on,
  hwpr_enc_off,     hwpr_enc_pulse,     balance_heat_on,  balance_heat_off,
  vtx1_isc,	    vtx1_osc,		vtx1_sbsc,	  vtx2_isc,
  vtx2_osc,	    vtx2_sbsc,		cam_cycle,
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
  balance_manual,    osc_set_focus,     osc_set_aperture, osc_save_period,
  bias_level_500,    bias_level_350,    bias_level_250,   bias_level_rox,
  bias_level_x,      isc_blob_centre,   dac2_level,       fridge_cycle_params,
  box,               osc_pixel_centre,  osc_blob_centre,  isc_gain,
  cal_pulse,         cal_repeat,        cap,              isc_catalogue,
  az_el_trim,        isc_det_set,       drift,            el_gain,
  isc_integrate,     osc_integrate,     osc_det_set,      osc_blobs,
  inner_level,       isc_offset,        hwpr_jump,        hwpr_goto_i,
  osc_catalogue,     osc_tolerances,    osc_hold_current, autotrim_to_sc,
  lock,              isc_blobs,         phase,            act_offset,
  pivot_gain,        isc_pixel_centre,  ra_dec_goto,      ra_dec_set,
  roll_gain,         isc_set_aperture,  isc_set_focus,    az_scan_accel,
  t_gyro_set,        osc_gain,          tdrss_bw,         iridium_bw,      oth_set,  
  t_gyro_gain,       timeout,           isc_tolerances,   vcap,
  vbox,              slot_sched,        az_gyro_offset,   isc_hold_current,
  isc_save_period,   osc_offset,        hwpr_set_overshoot,
  jfet_set,          isc_foc_off,       hwpr_vel,         hwpr_i,
  osc_foc_off,       gyro_off,	        gyro_on,          quad,
  el_gyro_offset,    general,           slew_veto,        set_secondary,
  thermo_gain,       actuator_servo,    xy_goto,          actuator_vel,
  xy_jump,           xy_xscan,          xy_yscan,         xy_raster,
  actuator_i,        lock_vel,          lock_i,           actuator_delta,
  delta_secondary,   lvdt_limit,        thermo_param,     focus_offset,
  isc_max_age,	     osc_max_age,	reset_adc,        balance_tset,
  motors_verbose,    bias_step,         phase_step,       hwpr_set_backlash,
  cam_any,	     cam_settrig_timed, cam_exp_params,	  cam_focus_params,
  cam_bad_pix,	     cam_blob_params,	cam_lens_any,	  cam_lens_move, 
  cam_lens_params,   cam_trig_delay,	hwpr_repeat,      hwpr_define_pos,
  hwpr_goto,	     hwpr_goto_pot,     act_enc_trim,     actuator_tol,
  el_scan,           el_box,            shutter_step,     shutter_step_slow,
  set_scan_params,   mag_cal,           pss_cal,          params_test,
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
