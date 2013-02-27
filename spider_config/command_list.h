/* command_list.h: BLAST command specification file definitions
 *
 * This software is copyright (C) 2002-20010 University of Toronto
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

#include "netcmd.h"  /* common parts of command defintions moved here */

/* WARNING: if either N_xCOMMANDS exceeds 254, commanding will break */
#define N_SCOMMANDS 212        /* total number of single word cmds */
#define N_MCOMMANDS 122        /* total number of multiword commands */

#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

//maximum insert number for hk commands. TODO-theo temporarily set to 4
#define	HK_MAX  6

#define N_GROUPS 24

#define GR_POINT        0x00000001
#define GR_CMB          0x00000002
#define GR_HWPR         0x00000004
#define GR_TRIM         0x00000008
#define GR_ELECT        0x00000010
#define GR_BIAS         0x00000020
#define GR_VETO         0x00000040
#define GR_ACT          0x00000080
//#define GR_BSC	  0x00000100	  //unused
#define GR_GAIN         0x00000200
#define GR_LOCK         0x00000400
#define GR_CRYO_HEAT    0x00000800
#define GR_POWER        0x00001000
#define GR_SCTAB        0x00002000
#define GR_THEO_HEAT	0x00004000
#define GR_TELEM        0x00008000
#define GR_SCGOOD       0x00010000
#define GR_IFPOWER      0x00020000	  
#define GR_STAGE        0x00040000
#define GR_SCBAD        0x00080000
#define GR_MCE          0x00100000
#define GR_MISC         0x00200000
#define GR_SCUGLY       0x00400000
//#define GR_OSC_PARAM    0x00800000	  //unused

#define MCECMD          0x40000000 /* MCE command flag */
//reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const int command_list_serial_as_int(void);

extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  az_auto_gyro,      az_off,            az_on,             hwp_panic,
  el_off,            el_on,             pin_in,            reset_trims,
  elclin_allow,      elclin_veto,       elenc1_allow,      elenc2_allow,
  elenc1_veto,       elenc2_veto,
  gps_allow,         gps_veto,          mag_allow,         mag_veto,
  stop,              pss_veto,          pss_allow,         lock,
  trim_to_isc,       unlock,            lock_off,          lock_on,
  force_el_on,       gps_cycle,         actbus_cycle,      rw_cycle,
  piv_cycle,         elmot_cycle,       hub232_cycle,      das_cycle,
  gps_off,           gps_on,            rw_off,	           rw_on,
  piv_off,           piv_on,            elmot_off,         elmot_on,
  vtx_off,           vtx_on,            bi0_off,           bi0_on,
  das_off,           das_on,             
  of_charge_off,     of_charge_on,      of_charge_cycle,
  if_charge_off,     if_charge_on,       if_charge_cycle,
  ifroll_1_gy_allow, ifroll_1_gy_veto,  ifroll_2_gy_allow, ifroll_2_gy_veto,
  ifyaw_1_gy_allow,  ifyaw_1_gy_veto,   ifyaw_2_gy_allow,  ifyaw_2_gy_veto,
  ifel_1_gy_allow,   ifel_1_gy_veto,    ifel_2_gy_allow,   ifel_2_gy_veto,
  ifroll_1_gy_off,   ifroll_1_gy_on,    ifroll_2_gy_off,   ifroll_2_gy_on,
  ifyaw_1_gy_off,    ifyaw_1_gy_on,     ifyaw_2_gy_off,	   ifyaw_2_gy_on,
  ifel_1_gy_off,     ifel_1_gy_on,      ifel_2_gy_off,	   ifel_2_gy_on,
  ifroll_1_gy_cycle, ifroll_2_gy_cycle, ifyaw_1_gy_cycle,  ifyaw_2_gy_cycle,
  ifel_1_gy_cycle,   ifel_2_gy_cycle,   gybox_off,         gybox_on,
  hub232_off,        hub232_on,         gybox_cycle,        
  reap_itsy,         reap_bitsy,        xy_panic,
  trim_to_osc,       antisun,           blast_rocks,       blast_sucks,
  at_float,          not_at_float,      el_auto_gyro,
  repoll,            halt_itsy,	        halt_bitsy,        actbus_on,
  actbus_off,        actuator_stop,     restore_piv,
  reset_rw,          reset_piv,         reset_elev,
  vtx1_isc,          vtx1_osc,          vtx1_bsc,          vtx2_isc,
  vtx2_osc,          vtx2_bsc,
  thegood_off,	     thegood_on,        thegood_cam_cycle, thegood_cpu_cycle,
  thebad_off,        thebad_on,         thebad_cam_cycle,  thebad_cpu_cycle,
  theugly_off,	     theugly_on,        theugly_cam_cycle, theugly_cpu_cycle,
  thegood_expose,    thegood_autofocus,	thegood_settrig_ext, thegood_pause, 
  thebad_expose,     thebad_autofocus,	thebad_settrig_ext,	thebad_pause, 
  theugly_expose,    theugly_autofocus,	theugly_settrig_ext, theugly_pause, 
  thegood_run,	     thebad_run,        theugly_run,       table_track,  
  hwp_step,          hwp_repoll,
  //theo heater commands. TODO-theo assign non-temporary names
  hk_mt_bottom_heat_on,    hk_mt_bottom_heat_off,
  hk_t1_heat_on,           hk_t1_heat_off,
  hk_vcs1_hx1_heat_on,     hk_vcs1_hx1_heat_off,
  hk_vcs2_hx1_heat_on,     hk_vcs2_hx1_heat_off,
  hk_vcs1_hx2_heat_on,     hk_vcs1_hx2_heat_off,
  hk_vcs2_hx2_heat_on,     hk_vcs2_hx2_heat_off,
  hk_sft_bottom_heat_on,   hk_sft_bottom_heat_off,
  hk_t7_heat_on,           hk_t7_heat_off,
  bbc_sync_ext,      bbc_sync_int,      bbc_sync_auto,     elmot_auto,
  //make better use of unused groups
  pull_cmb_pin, global_thermonuclear_war, lock_and_load, 
  mce1_on,           mce1_off,          mce1_cycle,
  mce2_on,           mce2_off,          mce2_cycle,
  mce3_on,           mce3_off,          mce3_cycle,
  mac_on,            mac_off,           mac_cycle,
  sync_on,           sync_off,          sync_cycle,
  eth_on,            eth_off,           eth_cycle,
  hwp_on,            hwp_off,           hwp_cycle,
  hk_preamp_on,      hk_preamp_off,     hk_preamp_cycle, 
  mcc1_on,           mcc1_off,          mcc1_cycle,
  mcc2_on,           mcc2_off,          mcc2_cycle,
  mcc3_on,           mcc3_off,          mcc3_cycle,
  mcc4_on,           mcc4_off,          mcc4_cycle,
  mcc5_on,           mcc5_off,          mcc5_cycle,
  mcc6_on,           mcc6_off,          mcc6_cycle,
  mpc_ping, /* MPC test command */
  xyzzy	    //xyzzy should be at the end of the list
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,           dac2_level,  
  set_scan_params,
  az_el_trim,        drift,             el_gain,           act_offset,
  actuator_tol,
  pivot_gain,        ra_dec_goto,       ra_dec_set,
  t_gyro_set,        tdrss_bw,          iridium_bw,
  t_gyro_gain,       timeout,           slot_sched,        az_gyro_offset,
  cov_gps,	         lvdt_limit,        reset_adc,        
  hk_auto_cycle_on,  hk_auto_cycle_off, hk_fridge_cycle,
  el_gyro_offset,    general,           slew_veto,        
  actuator_servo,    xy_goto,           actuator_vel,
  xy_jump,           xy_xscan,          xy_yscan,          xy_raster,
  actuator_i,        actuator_delta,
  motors_verbose,    
  thegood_any,	     thegood_settrig_timed,	thegood_exp_params,	thegood_focus_params,
  thebad_any,	     thebad_settrig_timed,	thebad_exp_params,	thebad_focus_params,
  theugly_any,	     theugly_settrig_timed,	theugly_exp_params,	theugly_focus_params,
  thegood_bad_pix,   thegood_blob_params,	thegood_lens_any, 	thegood_lens_move,
  thebad_bad_pix,    thebad_blob_params,	thebad_lens_any,   	thebad_lens_move,
  theugly_bad_pix,   theugly_blob_params,	theugly_lens_any,	theugly_lens_move,
  thegood_lens_params,thebad_lens_params,	theugly_lens_params, 
  table_gain,	       table_goto,        table_relmove,     table_speed, 
  t_bsc_set,         t_rsc_set,         ants_gps,          act_enc_trim,
  hwp_vel,           hwp_i,             hwp_move,          hwp_general,
  hk_pump_heat_on,   hk_pump_heat_off,  hk_heat_switch_on, hk_heat_switch_off,
  hk_ssa_heat_on,    hk_ssa_heat_off,   hk_htr1_heat_on,   hk_htr1_heat_off,
  hk_htr2_heat_on,   hk_htr2_heat_off,  hk_htr3_heat_on,   hk_htr3_heat_off,
  hk_fphi_heat_on,   hk_fphi_heat_off,  hk_fplo_heat_set,  hk_strap_heat_set, 
  hk_ampl_cernox,    hk_ampl_ntd,       hk_phase_cernox,   hk_phase_ntd,
  hk_phase_step_cernox, hk_phase_step_ntd, hk_bias_freq,      hk_pump_servo_on,
  hk_mt_bottom_pulse,  hk_t1_pulse,         hk_vcs1_hx1_pulse,  
  hk_vcs2_hx1_pulse,   hk_vcs1_hx2_pulse,   hk_vcs2_hx2_pulse,
  hk_sft_bottom_pulse, hk_t7_pulse,
  spider_scan,	     sine_scan,         bbc_rate_ext,      bbc_rate_int,
  el_pulse,           hwp_halt,          hwp_phase,        el_rel_move,
  hwp_bias_on,        hwp_bias_off,
  /* start MCE block */
  mpc_test,
  /* end MCE block */
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

#endif /* COMMAND_LIST_H */
