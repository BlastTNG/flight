/* command_list.h: BLAST command specification file definitions
  isc_use_pyramid,  isc_no_pyramid
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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

#include "isc_protocol.h"  /* required for constants */

#define N_SCOMMANDS 204        /* total number of single word cmds */
#define N_MCOMMANDS 104        /* total number of multiword commands */
#define MAX_N_PARAMS 10
#define CMD_STRING_LEN 32      /* maximum allowable lenght of command string */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 80
#define SIZE_ABOUT 80
#define SIZE_PARNAME 80

#define N_GROUPS 24

#define GR_POINT        0x00000001
#define GR_BAL          0x00000002
#define GR_HWPR         0x00000004
#define GR_TRIM         0x00000008
#define GR_ELECT        0x00000010
#define GR_BIAS         0x00000020
#define GR_VETO         0x00000040
#define GR_ACT          0x00000080
//#define GR_CALLAMP      0x00000100
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

#define CONFIRM         0x80000000

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
  stop,             sun_veto,           sun_allow,        isc_eye_off,
  trim_to_isc,      unlock,             lock_off,         xyzzy,
  isc_reboot,       isc_cam_cycle,      osc_run,          osc_shutdown,
  osc_reboot,       osc_cam_cycle,      osc_pause,        osc_abort,
  osc_reconnect,    osc_save_images,    osc_discard_images, osc_full_screen,  
  force_el_on,      auto_jfetheat,      auto_cycle,       gps_cycle,
  isc_cycle,        osc_cycle,          actbus_cycle,     rw_cycle,
  piv_cycle,        elmot_cycle,        hub232_cycle,     das_cycle,
  gps_off,          gps_on,             isc_off,          isc_on,
  osc_off,          osc_on,             rw_off,	          rw_on,
  piv_off,	    piv_on,		elmot_off,	  elmot_on,
  sc_tx_off,	    sc_tx_on,		bi0_off,	  bi0_on,
  das_off,	    das_on,		preamp_off,	  preamp_on,
  bias_off,	    bias_on,		hk_off,		  hk_on,
  um250_off,	    um250_on,		um350_off,	  um350_on,
  um500_off,	    um500_on,		heat_off,	  heat_on,
  ifroll_1_gy_allow,ifroll_1_gy_veto,   ifroll_2_gy_allow,ifroll_2_gy_veto,
  ifyaw_1_gy_allow, ifyaw_1_gy_veto,    ifyaw_2_gy_allow, ifyaw_2_gy_veto,
  ifel_1_gy_allow,  ifel_1_gy_veto,	ifel_2_gy_allow,  ifel_2_gy_veto,
  ifroll_1_gy_off,  ifroll_1_gy_on,	ifroll_2_gy_off,  ifroll_2_gy_on,
  ifyaw_1_gy_off,   ifyaw_1_gy_on,	ifyaw_2_gy_off,	  ifyaw_2_gy_on,
  ifel_1_gy_off,    ifel_1_gy_on,	ifel_2_gy_off,	  ifel_2_gy_on,
  ifroll_1_gy_cycle,ifroll_2_gy_cycle,  ifyaw_1_gy_cycle, ifyaw_2_gy_cycle,
  ifel_1_gy_cycle,  ifel_2_gy_cycle,    gybox_off,        gybox_on,
  ss_off,           ss_on,              hub232_off,	  hub232_on,
  ss_cycle,         gybox_cycle,        isc_trig_int,     isc_trig_ext,
  osc_trig_int,     osc_trig_ext,       ln_valve_on,      ln_valve_off,
  osc_veto,         osc_allow,          reap_north,       reap_south,
  isc_eye_on,       osc_eye_on,         osc_eye_off,      xy_panic,
  trim_to_osc,      antisun,            blast_rocks,      blast_sucks,
  fridge_cycle,     at_float,           not_at_float,     el_auto_gyro,
  repoll,           biascmd_inh,        biascmd_ena,      autofocus_allow,
  isc_use_pyramid,  isc_no_pyramid,     osc_use_pyramid,  osc_no_pyramid,
  autofocus_veto,   north_halt,         south_halt,       actbus_on,
  actbus_off,       actuator_stop,      level_pulse,      restore_piv,
  reset_dr,         actpos_trim,        reset_rw,         reset_piv,
  reset_elev,       jfet_on,            jfet_off,         hs_pot_on,
  hs_pot_off,       bda_on,             bda_off,          hwpr_enc_on,
  hwpr_enc_off,     hwpr_enc_pulse,     balance_heat_on,  balance_heat_off
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,          balance_gain,
  balance_manual,    osc_set_focus,     osc_set_aperture, osc_save_period,
  bias_level_500,    bias_level_350,    bias_level_250,   bias_level_rox,
  bias_level_x,      isc_blob_centre,
  /* dac level commands are temporary */
  dac1_level,        dac2_level,        dac3_level,       dac4_level,
  dac5_level,
  box,               osc_pixel_centre,  osc_blob_centre,  isc_gain,
  cal_pulse,         cal_repeat,        cap,              isc_catalogue,
  az_el_trim,        isc_det_set,       drift,            el_gain,
  isc_integrate,     osc_integrate,     osc_det_set,      osc_blobs,
  inner_level,       isc_offset,        hwpr_jump,        hwpr_goto,
  osc_catalogue,     osc_tolerances,    osc_hold_current,
  lock,              isc_blobs,         phase,            encoder_offset,
  pivot_gain,        isc_pixel_centre,  ra_dec_goto,      ra_dec_set,
  roll_gain,         isc_set_aperture,  isc_set_focus,  
  t_gyro_set,        osc_gain,
  t_gyro_gain,       timeout,           isc_tolerances,   vcap,
  vbox,              alice_file,        az_gyro_offset,   isc_hold_current,
  isc_save_period,   osc_offset,        plugh,		  
  jfet_set,          isc_foc_off,       hwpr_vel,         hwpr_i,
  osc_foc_off,       apcu_charge,	gyro_off,	  gyro_on,
  dpcu_charge,       auto_apcu,         auto_dpcu,        quad,
  el_gyro_offset,    general,           slew_veto,        set_secondary,
  thermo_gain,       actuator_servo,    xy_goto,          actuator_vel,
  xy_jump,           xy_xscan,          xy_yscan,         xy_raster,
  actuator_i,        lock_vel,          lock_i,           actuator_delta,
  delta_secondary,   lvdt_limit,        thermo_param,     focus_offset,
  isc_max_age,	     osc_max_age,	reset_adc,        balance_tset,
  balance_veto,      motors_verbose
};

//32-bit and 64-bit sytems disagree on packing
#pragma pack(4)

struct scom {
  enum singleCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
};

struct par {
  char name[SIZE_PARNAME];
  double min;
  double max;
  char type;
  char field[20];
};

struct mcom {
  enum multiCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
  char numparams;
  struct par params[MAX_N_PARAMS];
};

#pragma pack()   //return to default packing

extern struct scom scommands[N_SCOMMANDS];

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern struct mcom mcommands[N_MCOMMANDS];

#endif /* COMMAND_LIST_H */
