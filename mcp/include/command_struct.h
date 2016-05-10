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
#include "isc_protocol.h"
#include "command_list.h"
#include "channels_tng.h"
#include "mcp_sched.h"

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

#define LS_CLOSED      0x0002
#define LS_DRIVE_OFF   0x0004
#define LS_POT_RAIL    0x0008  // now defunct
#define LS_DRIVE_EXT   0x0010
#define LS_DRIVE_RET   0x0020
#define LS_DRIVE_STP   0x0040
#define LS_DRIVE_JIG   0x0080  // now defunct
#define LS_DRIVE_UNK   0x0100
#define LS_EL_OK       0x0200
#define LS_IGNORE_EL   0x0400
#define LS_DRIVE_FORCE 0x0800
#define LS_DRIVE_MASK  0x09F4

/* latching relay pulse length in 200ms slow frames */
#define LATCH_PULSE_LEN	 2
/* time (slow frames) to keep power off when power cycling devices */
#define PCYCLE_HOLD_LEN	 20

#define PREV_STATUS_FILE "/data/etc/blast/mcp.prev_status"

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
    double SP; // RW velocity Set Point
    double F; // Current offset to overcome static friction.
};

#define LS_OPEN        0x0001
#define LS_CLOSED      0x0002
#define LS_DRIVE_OFF   0x0004
#define LS_POT_RAIL    0x0008  // now defunct
#define LS_DRIVE_EXT   0x0010
#define LS_DRIVE_RET   0x0020
#define LS_DRIVE_STP   0x0040
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
#define HWPR_PANIC	0
#define HWPR_SLEEP	1
#define HWPR_GOTO	2
#define HWPR_JUMP	3
#define HWPR_STEP	4
#define HWPR_REPEAT	5
#define HWPR_GOTO_I	6
#define HWPR_GOTO_POT	7

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

typedef struct {
  uint16_t charcoalHeater;
  uint16_t hsCharcoal;
  uint16_t fridgeCycle;
  uint16_t force_cycle;

  double cycle_start_temp;
  double cycle_pot_max;
  double cycle_charcoal_max;
  double cycle_charcoal_settle;
  // timeouts in minutes (NB: time will probably be in seconds)
  double cycle_charcoal_timeout;
  double cycle_settle_timeout;

  uint16_t BDAHeat;
  uint16_t hsPot;
  int16_t heliumLevel;
  int he4_lev_old;
  int16_t hwprPos;
  int hwpr_pos_old;

  uint16_t JFETHeat;
  uint16_t autoJFETheat;
  double JFETSetOn, JFETSetOff;

  enum calmode calibrator;
  uint16_t calib_pulse, calib_period;
  int calib_repeats;
  int calib_hwpr;

  uint16_t potvalve_open, potvalve_on, potvalve_close;
  uint16_t lvalve_open, lhevalve_on, lvalve_close, lnvalve_on;
} cryo_cmds_t;

typedef struct slinger_commanding
{
    unsigned int downlink_rate_bps;
    bool highrate_active;
    bool biphase_active;
} slinger_commanding_t;

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
  uint32_t tdrss_bw;
  uint32_t iridium_bw;

  enum {vtx_isc, vtx_osc} vtx_sel[2];

  uei_commands_t uei_command;

  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct PivGainStruct pivot_gain;

  struct {
    struct latch_pulse sc_tx;
    struct latch_pulse das;
    struct latch_pulse xsc0;
    struct latch_pulse xsc1;
    struct latch_pulse rw;
    struct latch_pulse piv;
    struct latch_pulse elmot;
    struct latch_pulse bi0;
    struct latch_pulse rx_main;
    struct latch_pulse rx_hk;
    struct latch_pulse rx_amps;
    struct latch_pulse charge;
    int gybox_off;
    int gyro_off[6];
    int gyro_off_auto[6];
    int hub232_off;
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

  unsigned char use_elenc;
  unsigned char use_elmotenc;
  unsigned char use_elclin;
  unsigned char use_pss;
  unsigned char use_xsc0;
  unsigned char use_xsc1;
  unsigned char use_mag;

  uint16_t fast_offset_gy;
  uint32_t slew_veto;

  double az_accel;

  double clin_el_trim;
  double enc_el_trim;
  double enc_motor_el_trim;
  double null_az_trim;
  double mag_az_trim;
  double pss_az_trim;

  int autotrim_enable;
  double autotrim_thresh;    // in sc sigma
  double autotrim_rate;      // degrees/s
  time_t autotrim_time;      // in seconds
  time_t autotrim_xsc0_last_bad;
  time_t autotrim_osc_last_bad;

  double cal_xmax_mag;
  double cal_xmin_mag;
  double cal_ymax_mag;
  double cal_ymin_mag;

  double cal_off_pss1;
  double cal_off_pss2;
  double cal_off_pss3;
  double cal_off_pss4;

  double cal_d_pss1;
  double cal_d_pss2;
  double cal_d_pss3;
  double cal_d_pss4;

  double cal_imin_pss;
  struct {
    int biasRamp;
    uint16_t bias[5];
    unsigned char setLevel[5];
    struct Step biasStep;
  } Bias;

  cryo_cmds_t Cryo;

  struct {
    enum {bal_rest, bal_manual, bal_auto} mode;
    enum {pos, neg} bal_move_type;
    uint32_t pos;
    uint16_t vel;
    double i_hold;
    double i_move;

    // servo parameters
    double i_el_on_bal;
    double i_el_off_bal;
    double i_el_target_bal;
    double gain_bal;
  } balance;

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

    uint32_t  shutter_goal;
  } actbus;

  struct {
    int vel, acc, hold_i, move_i;
    int force_repoll;
    int mode, is_new, target;
    int n_pos, repeats, step_wait, step_size, overshoot;
    double pos[4];
    int i_pos;
    int no_step;
    int use_pot;
    double pot_targ;
  } hwpr;

  int pin_is_in;

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

  /* Integrating Star Camera Stuff */
  struct ISCStatusStruct ISCState[2];

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

extern struct CommandDataStruct CommandData;

#endif   // COMMAND_STRUCT_H

