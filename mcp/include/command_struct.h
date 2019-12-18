/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_COMMAND_STRUCT_H
#define INCLUDE_COMMAND_STRUCT_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <xsc_protocol.h>
#include "command_list.h"
#include "channels_tng.h"
#include "mcp_sched.h"
#include "roach.h"
#include "pointing_struct.h"
#include "microscroll.h"

#define AXIS_VEL      0
#define AXIS_POSITION 1
#define AXIS_LOCK     2

#define P_AZEL_GOTO  1
#define P_AZ_SCAN    2
#define P_DRIFT      3
#define P_RADEC_GOTO 4
#define P_VCAP       5
#define P_CAP        6
#define P_BOX        7
#define P_LOCK       8
#define P_VBOX       9
#define P_QUAD      10
#define P_EL_SCAN   11
#define P_EL_BOX    12
#define P_CURRENT   13

#define MAX_ISC_SLOW_PULSE_SPEED 0.015

#define XYSTAGE_PANIC  0
#define XYSTAGE_GOTO   1
#define XYSTAGE_JUMP   2
#define XYSTAGE_SCAN   3
#define XYSTAGE_RASTER 4

// Defines the bits responsible for each of the heaters in
// CommandData.uei_command.uei_of_dio_432_out.
// TODO(laura): These are for BLASTPol (taken from das.c). Get updated list from Jeff.
// TODO(laura): move this to a separate cryo control program modeled on das.c
#define HEAT_HELIUM_LEVEL    0x0001
#define HEAT_CHARCOAL        0x0002
#define HEAT_POT_HS          0x0004
#define HEAT_CHARCOAL_HS     0x0008
#define HEAT_JFET            0x0010
#define HEAT_BDA             0x0020
#define HEAT_CALIBRATOR      0x0040
#define HEAT_HWPR_POS        0x0080

/* latching relay pulse length in 200ms slow frames */
#define LATCH_PULSE_LEN	 2
/* time (slow frames) to keep power off when power cycling devices */
#define PCYCLE_HOLD_LEN	 20

#define PREV_STATUS_FILE "/data/etc/blast/mcp.prev_status"

/* Need to undef I here in for source files that utilize complex.h */
#undef I
struct GainStruct {
  float P;
  float I;
  float D;
  float SP;
  float DB; // Deadband the integral term
  float PT; // position velocity gain
  float F; // Current offset to overcome static friction.
};

// used for pivot loop gains
struct PivGainStruct {
    float PV; // prop to RW velocity
    float IV; // prop to RW velocity
    float PE; // prop to velocity error
    float IE; // prop to velocity error
    double SP; // RW velocity Set Point
    double F; // Current offset to overcome static friction.
};

#define LS_OPEN        0x0001  // Set => Lock is OPEN
#define LS_CLOSED      0x0002  // Set => Lock is CLOSED
#define LS_DRIVE_OFF   0x0004  // Set => Drive is OFF or NOT MOVING
#define LS_POT_RAIL    0x0008  // now defunct
#define LS_DRIVE_EXT   0x0010  // Set => Drive is extending
#define LS_DRIVE_RET   0x0020  // Set => Drive is retracting
#define LS_DRIVE_STP   0x0040  // This is never set. No idea what this is.
#define LS_DRIVE_JIG   0x0080  // now defunct
#define LS_DRIVE_UNK   0x0100
#define LS_EL_OK       0x0200
#define LS_IGNORE_EL   0x0400
#define LS_DRIVE_FORCE 0x0800
#define LS_DRIVE_MASK  0x09F4

#define SHUTTER_OPEN    0x0001
#define SHUTTER_CLOSED  0x0002
#define SHUTTER_INIT    0x0004
#define SHUTTER_OFF     0x0008
#define SHUTTER_RESET   0x0010
#define SHUTTER_NOP     0x0020
#define SHUTTER_CLOSED2 0x0040
#define SHUTTER_CLOSED_SLOW 0x0080
#define SHUTTER_UNK     0x0100
#define SHUTTER_KEEPCLOSED 0x0200
#define SHUTTER_KEEPOPEN 0X0400

#define ACTBUS_FM_SLEEP  0
#define ACTBUS_FM_SERVO  1
#define ACTBUS_FM_FOCUS  2
#define ACTBUS_FM_OFFSET 3
#define ACTBUS_FM_THERMO 4
#define ACTBUS_FM_NOW    5  // unused
#define ACTBUS_FM_DELTA  6
#define ACTBUS_FM_PANIC  7
#define ACTBUS_FM_DELFOC 8
#define ACTBUS_FM_TRIM   9

#define TC_MODE_ENABLED  0
#define TC_MODE_AUTOVETO 1
#define TC_MODE_VETOED   2

#define XYSTAGE_PANIC  0
#define XYSTAGE_GOTO   1
#define XYSTAGE_JUMP   2
#define XYSTAGE_SCAN   3
#define XYSTAGE_RASTER 4
#define HWPR_PANIC		0
#define HWPR_SLEEP		1
#define HWPR_GOTO		2
#define HWPR_GOTO_REL	3
#define HWPR_STEP		4
#define HWPR_REPEAT		5
#define HWPR_GOTO_I		6
#define HWPR_GOTO_POT	7

#define ROACH_TLM_IQDF 0x1
#define ROACH_TLM_DELTA 0x2

// mode        X     Y    vaz   del    w    h
// LOCK              el
// AZEL_GOTO   az    el
// AZ_SCAN     az    el   vaz
// DRIFT                  vaz   vel
// RADEC_GOTO  ra    dec
// VCAP        ra    dec  vaz   vel    r
// CAP         ra    dec  vaz   elstep r
// BOX         ra    dec  vaz   elstep w    h
struct PointingModeStruct {
  int nw; /* used for gy-offset veto during slews */
  int mode;
  double X;
  double Y;
  double vaz;
  double del;
  double w;
  double h;
  time_t t;
  double ra[4]; // the RAs for radbox (ie, quad)
  double dec[4]; // the decs for radbox (ie, quad)
  uint32_t n_dith; // Elevation dither step
  int next_i_dith; // Dither starting index for next scan
  int next_i_hwpr; // HWPR pos for next scan
  double vel; // Elevation scan velocity
  double daz; // Azimuth step size (for el scans)
};

struct latch_pulse {
  int set_count;
  int rst_count;
};


enum calmode { on, off, pulse, repeat };

struct Step {
  uint16_t do_step;
  uint16_t start;
  uint16_t end;
  uint16_t nsteps;
  uint16_t arr_ind; // only used for bias
  uint16_t dt;
  uint16_t pulse_len;  // only used for bias
};

typedef enum
{
    xsc_heater_off, xsc_heater_on, xsc_heater_auto
} xsc_heater_modes_t;

typedef struct XSCHeaters
{
    xsc_heater_modes_t mode;
    double setpoint;
} XSCHeaters;

typedef struct XSCTriggerThresholds
{
    bool enabled;
    double blob_streaking_px;
}
XSCTriggerThreshold;

typedef struct XSCTrigger
{
    int exposure_time_cs;
    int grace_period_cs;
    int post_trigger_counter_mcp_share_delay_cs; // should probably be less than grace period

    int num_triggers;
    int multi_trigger_time_between_triggers_cs;

    XSCTriggerThreshold threshold;
    bool scan_force_trigger_enabled;
} XSCTrigger;

typedef struct XSCCommandStruct
{
    int is_new_window_period_cs;
    XSCHeaters heaters;
    XSCTrigger trigger;
    XSCClientData net;
    double cross_el_trim;
    double el_trim;
} XSCCommandStruct;

typedef struct
{
    uint32_t uei_of_dio_432_out; ///!< BITFIELD for UEI_OF digital output
} uei_commands_t;

typedef enum {intermed = 0, opened, closed, loose_closed} valve_state_t;

typedef struct {
  int16_t hwprPos;
  int hwpr_pos_old;

  uint16_t cal_length, calib_period;
  int calib_repeats;
  int calib_hwpr;
  int potvalve_on;
  valve_state_t potvalve_goal;
  uint32_t potvalve_vel;
  uint16_t potvalve_opencurrent, potvalve_closecurrent, potvalve_hold_i;
  uint16_t potvalve_open_threshold, potvalve_lclosed_threshold, potvalve_closed_threshold;
  uint16_t potvalve_min_tighten_move;
  uint16_t aalborg_valve_goal[N_AALBORG_VALVES];
  float aalborg_speed;
  valve_state_t valve_goals[2];
  int valve_stop[2];
  uint16_t valve_vel, valve_move_i, valve_hold_i, valve_acc;
  uint16_t lvalve_open, lhevalve_on, lvalve_close, lnvalve_on;
  int do_cal_pulse;
  int do_level_pulse;
  uint16_t level_length;
  uint16_t heater_300mk, charcoal_hs, charcoal, lna_250, lna_350, lna_500, heater_1k;
  uint16_t heater_update;
  uint16_t heater_status;
  uint16_t sync;
  uint16_t force_cycle, auto_cycling;
  uint16_t pot_filling;
  uint16_t forced;
  int labjack, send_dac, load_curve, cycle_allowed, watchdog_allowed, pot_forced;
  float dac_value;
  float tcrit_fpa;
  uint32_t counter, counter_max;
  uint16_t num_pulse, separation, length, periodic_pulse;
} cryo_cmds_t;

typedef struct {
	int new_cmd;
	int reg;
	float value;
} aalborg_test_t;

typedef struct {
    int go, just_received, no_pulse;
    uint16_t length;
} ir_cmds_t;

typedef struct {
    float supply_24va, supply_24vb;
    float relay_12v_on, relay_12v_off;
    float supply_12v;
} microscroll_control_t;

typedef struct {
  float of_1_on, of_2_on, of_3_on, of_4_on, of_5_on, of_6_on, of_7_on, of_8_on;
  float of_1_off, of_2_off, of_3_off, of_4_off, of_5_off, of_6_off, of_7_off, of_8_off;
  float of_9_on, of_10_on, of_11_on, of_12_on, of_13_on, of_14_on, of_15_on, of_16_on;
  float of_9_off, of_10_off, of_11_off, of_12_off, of_13_off, of_14_off, of_15_off, of_16_off;
  float if_1_on, if_1_off, if_2_on, if_2_off, if_3_on, if_3_off, if_4_on, if_4_off;
  float if_5_on, if_5_off, if_6_on, if_6_off, if_7_on, if_7_off, if_8_on, if_8_off;
  float if_9_on, if_9_off, if_10_on, if_10_off;
  float cycle_of_1, cycle_of_2, cycle_of_3, cycle_of_4, cycle_of_5, cycle_of_6;
  float cycle_of_7, cycle_of_8, cycle_of_9, cycle_of_10, cycle_of_11, cycle_of_12;
  float cycle_of_13, cycle_of_14, cycle_of_15, cycle_of_16;
  float cycle_if_1, cycle_if_2, cycle_if_3, cycle_if_4, cycle_if_5, cycle_if_6;
  float cycle_if_7, cycle_if_8, cycle_if_9, cycle_if_10;
  float cycled_of, cycled_if;
  float rec_on, rec_off, amp_supply_on, amp_supply_off;
  float therm_supply_on, therm_supply_off, heater_supply_on, heater_supply_off;
  float update_rec, update_of, update_if;
  uint16_t labjack[5];
  int of_relays[16], if_relays[10];
  int update_video, video_trans;
} relay_cmds_t;

typedef struct {
    uint16_t lj_q_on;
    uint16_t which_q[11];
    uint16_t set_q;
} labjack_queue_t;

typedef struct slinger_commanding
{
    unsigned int downlink_rate_bps;
    bool highrate_active;
    bool biphase_active;
} slinger_commanding_t;

typedef struct udp_roach
{
    bool store_udp;
    bool publish_udp;
} udp_roach_t;

typedef struct roach
{
    unsigned int roach_new_state;
    unsigned int roach_desired_state;
    unsigned int change_roach_state;
    unsigned int get_roach_state;
    unsigned int do_df_calc;
    unsigned int auto_retune;
    unsigned int opt_tones;
    unsigned int do_sweeps;
    unsigned int new_atten;
    unsigned int load_vna_amps;
    unsigned int load_targ_amps;
    unsigned int calibrate_adc;
    unsigned int set_attens;
    unsigned int read_attens;
    unsigned int find_kids;
    unsigned int adc_rms;
    unsigned int test_tone;
    unsigned int do_cal_sweeps;
    unsigned int get_phase_centers;
    unsigned int get_timestream;
    unsigned int chan;
    unsigned int tune_amps;
    unsigned int refit_res_freqs;
    unsigned int change_tone_amps;
    unsigned int do_master_chop;
    unsigned int load_new_freqs;
    unsigned int calc_ref_params;
    unsigned int do_check_retune;
    unsigned int do_retune;
    unsigned int set_lo;
    unsigned int read_lo;
    unsigned int find_kids_default;
    unsigned int change_targ_freq;
    unsigned int change_tone_phase;
    unsigned int change_tone_freq;
    unsigned int on_res;
    unsigned int auto_find;
    unsigned int recenter_df;
    unsigned int check_response;
    unsigned int reboot_pi_now;
    unsigned int do_df_targ;
    unsigned int auto_el_retune;
    // Set whether we want to check the roach tuning after every scan.
    unsigned int do_full_loop;
    unsigned int auto_correct_freqs;
    unsigned int do_noise_comp;
    unsigned int do_fk_loop;
    unsigned int kill;
    unsigned int do_turnaround_loop;
    unsigned int n_outofrange_thresh;
    unsigned int enable_chop_lo;
    unsigned int is_chopping_lo;
    unsigned int has_lamp_control;
    unsigned int ext_ref;
    unsigned int change_extref;
    unsigned int min_nkids;
    unsigned int max_nkids;
    unsigned int is_sweeping;
    unsigned int read_temp;
} roach_status_t;

typedef struct roach_params
{
//  Parameters input to find_kids script
    double smoothing_scale;
    double peak_threshold;
    double spacing_threshold;
//  Set attenuators
    double set_in_atten;
    double set_out_atten;
    double read_in_atten;
    double read_out_atten;
    double new_out_atten;
    double test_freq;
    double atten_step;
    double npoints;
    int ncycles;
    double num_sec;
    double lo_offset;
    double delta_amp;
    double delta_phase;
    double freq_offset;
    int resp_thresh;
    double dBm_per_tone;
    double lo_freq_MHz;
    double df_retune_threshold;
    double df_diff_retune_threshold;
} roach_params_t;

// Ethercat controller/device commands
typedef struct {
    bool reset;
    bool fix_rw;
    bool fix_el;
    bool fix_piv;
    bool fix_hwpr;
    bool have_commutated_rw;
    bool rw_commutate_next_ec_reset;
} ec_devices_struct_t;

typedef struct {
    enum {bal_rest = 0, bal_manual, bal_auto} mode;
    enum {neg = 0, no_bal, pos} bal_move_type;
    uint32_t pos;
    uint32_t vel;
    uint16_t hold_i;
    uint16_t move_i;
    uint16_t acc;

    // servo parameters
    double i_el_on_bal;
    double i_el_off_bal;
    double gain_bal;
} cmd_balance_t;

typedef struct {
    uint8_t amp;
    int8_t status;
    bool reset;
} cmd_rox_bias_t;

typedef struct {
    unsigned int kid;
    unsigned int roach;
    unsigned int rtype;
    unsigned int index;
    char name[64];
} roach_tlm_t;

struct CommandDataStruct {
  uint16_t command_count;
  uint16_t last_command;

  // TODO(seth): Insert these into "Scheduler struct"
  uint16_t timeout;
  uint16_t slot_sched; // what slot to use
  uint16_t upslot_sched; // slot being uplinked
  uint32_t parts_sched; // bitfield up pulinked parts
  uint16_t uplink_sched; // use uplink sched
  uint16_t sucks;
  uint16_t lat_range;
  uint16_t at_float;

  uint32_t highrate_bw;
  uint32_t pilot_bw;
  uint32_t biphase_bw;

  float highrate_allframe_fraction;
  float pilot_allframe_fraction;
  float biphase_allframe_fraction;

  uint32_t biphase_clk_speed;
  bool biphase_rnrz;
  bool highrate_through_tdrss;
  char pilot_linklist_name[32];
  char bi0_linklist_name[32];
  char highrate_linklist_name[32];
  char sbd_linklist_name[32];
  uint32_t pilot_oth;
  roach_tlm_t roach_tlm[NUM_ROACH_TLM];
  char roach_tlm_mode;
  unsigned int num_channels_all_roaches[NUM_ROACHES];

  enum {VTX_XSC0, VTX_XSC1} vtx_sel[2];

  roach_status_t roach[NUM_ROACHES];
  udp_roach_t udp_roach[NUM_ROACHES];
  roach_params_t roach_params[NUM_ROACHES];
  unsigned int tar_all_data;
  unsigned int roach_run_cycle_checker;
  // motors.c sets this flag when a scan is nearly complete
  // to (optionally) trigger a retune
  unsigned int trigger_roach_tuning_check;
  unsigned int trigger_lo_offset_check;
  unsigned int cal_lamp_roach_hold;
  unsigned int enable_roach_lamp;
  uei_commands_t uei_command;

  cmd_rox_bias_t rox_bias;

  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct PivGainStruct pivot_gain;

  struct {
    struct latch_pulse sc_tx;
    struct latch_pulse bi0;
    struct latch_pulse charge;
    int gyro_off_auto[6];
  } power;

  uint16_t disable_az;
  uint16_t disable_el;
  uint16_t force_el;

  uint16_t reset_rw;
  uint16_t reset_piv;
  uint16_t reset_elev;
  uint16_t restore_piv;

  uint16_t verbose_rw;
  uint16_t verbose_el;
  uint16_t verbose_piv;
  int az_autogyro;
  int el_autogyro;
  double offset_ifel_gy;
  double offset_ifroll_gy;
  double offset_ifyaw_gy;
  uint32_t gymask;

  unsigned char use_elmotenc;
  unsigned char use_elclin;
  unsigned char use_pss;
  unsigned char use_xsc0;
  unsigned char use_xsc1;
  unsigned char use_mag1;
  unsigned char use_mag2;
  unsigned char use_dgps;

  uint16_t fast_offset_gy;
  uint32_t slew_veto;

  double az_accel;

  double clin_el_trim;
  double enc_motor_el_trim;
  double null_az_trim;
  double null_el_trim;
  double mag_az_trim[2];
  double pss_az_trim;
  double dgps_az_trim;

  int autotrim_enable;
  double autotrim_thresh;    // in sc sigma
  double autotrim_rate;      // degrees/s
  time_t autotrim_time;      // in seconds
  time_t autotrim_xsc0_last_bad;
  time_t autotrim_xsc1_last_bad;

  double cal_xmax_mag[2];
  double cal_xmin_mag[2];
  double cal_ymax_mag[2];
  double cal_ymin_mag[2];
  double cal_mag_align[2];

  double cal_d_pss[NUM_PSS];
  double cal_az_pss[NUM_PSS];
  double cal_az_pss_array;
  double cal_el_pss[NUM_PSS];
  double cal_roll_pss[NUM_PSS];
  double pss_noise;


  double cal_imin_pss;

  struct {
    int biasRamp;
    uint16_t bias[5];
    unsigned char setLevel[5];
    struct Step biasStep;
  } Bias;

  cryo_cmds_t Cryo;
  aalborg_test_t Aalborg;
  ir_cmds_t IRsource;
  microscroll_control_t Microscroll;

  labjack_queue_t Labjack_Queue;

  relay_cmds_t Relays;

  cmd_balance_t balance;

  ec_devices_struct_t ec_devices;

  struct {
    int off;
    int force_repoll;

    /* arbitrary command */
    int cindex;
    int caddr[3];
    char command[3][CMD_STRING_LEN];

    /* thermal control */
    double g_primary;
    double g_secondary;
    int tc_step;
    int tc_wait;
    int tc_mode;
    int tc_prefp;
    int tc_prefs;
    double tc_spread;
    int sf_offset;
    int sf_time;

    /* actuator control */
    int act_vel;
    int act_acc;
    int act_hold_i;
    int act_move_i;
    uint16_t act_tol;

    /* low-level actuator servo */
    int focus_mode;
    int goal[3];
    int delta[3];
    int offset[3];
    int trim[3];
    int focus;
    int lvdt_delta;
    int lvdt_low;
    int lvdt_high;

    /* lock control */
    int lock_vel;
    int lock_acc;
    int lock_hold_i;
    int lock_move_i;

    uint32_t lock_goal;

    /* shutter control */
    int shutter_step;
    int shutter_step_slow;
    int shutter_out;
    int shutter_move_i;
    int shutter_hold_i;
    int shutter_vel;
    int shutter_acc;

    uint32_t  shutter_goal;
  } actbus;

  struct {
    int vel;
	int acc;
	int hold_i, move_i;
    int force_repoll;
    int mode, is_new;
	float target;
    int n_pos, repeats, step_wait, step_size;
	float overshoot;
	float backoff;
    double pos[2];
    int i_pos;
    int no_step;
    int use_pot;
    double pot_targ;
	float margin;
  } hwpr;

  int pin_is_in;
  int mag_reset;

  struct {
    int x1, y1, x2, y2, step, xvel, yvel, is_new, mode;
    int force_repoll;
  } xystage;

  /* sensors output: read in mcp:SensorReader() */
  uint16_t temp1, temp2, temp3;
  uint16_t df;

  uint16_t plover;

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)
  double lat;
  double lon;

  struct {
    int pulse_width;
    int fast_pulse_width;
    int reconnect;
    int autofocus;
    int save_period;
    int auto_save;
    int max_age;    // maximum allowed time between trigger and solution
    int age;	    // last measured time between trigger and solution
  } ISCControl[2];

  struct XSCCommandStruct XSC[2];

  slinger_commanding_t packet_slinger;

  uint32_t checksum;
};

void InitCommandData();
double LockPosition(double);
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);
void WritePrevStatus();

extern struct CommandDataStruct CommandData;

#endif   // COMMAND_STRUCT_H

