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

#ifndef COMMAND_STRUCT_H
#define COMMAND_STRUCT_H

#include "isc_protocol.h"
#include "command_list.h"
#include "channels_tng.h"
#include "mcp_sched.h"
#include <time.h>

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

#define MAX_ISC_SLOW_PULSE_SPEED 0.015

/* latching relay pulse length in 200ms slow frames */
#define LATCH_PULSE_LEN	 2
/* time (slow frames) to keep power off when power cycling devices */
#define PCYCLE_HOLD_LEN	 20
/* time (in slow frames) to suppress ADC card watchdog, to induce reset */
#define	RESET_ADC_LEN	 80

#define PREV_STATUS_FILE "/data/etc/blast/mcp.prev_status"

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
  unsigned short int PT; // position velocity gain
};

// used for pivot loop gains
struct PivGainStruct { 
  unsigned short int PV; // prop to RW velocity
  unsigned short int PE; // prop to velocity error
  double SP; // RW velocity Set Point 
  double F; // Current offset to overcome static friction. 
};

#define LS_OPEN        0x0001
#define LS_CLOSED      0x0002
#define LS_DRIVE_OFF   0x0004
#define LS_POT_RAIL    0x0008  //now defunct
#define LS_DRIVE_EXT   0x0010
#define LS_DRIVE_RET   0x0020
#define LS_DRIVE_STP   0x0040
#define LS_DRIVE_JIG   0x0080  //now defunct
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
#define ACTBUS_FM_NOW    5  //unused
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
  unsigned int n_dith; // Elevation dither step 
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
  unsigned short do_step;
  unsigned short start;
  unsigned short end;
  unsigned short nsteps;
  unsigned short arr_ind; // only used for bias
  unsigned short dt;
  unsigned short pulse_len;  // only used for bias
};

#define SBSC_COMM_BUF_SIZE  10000
#define SBSC_CMD_Q_SIZE 6
struct SBSCCommandData {
  //camera and lens configuration
  short int forced;  //are lens moves forced?
  int expInt;        //exposure interval (ms) (0=triggered)
  int expTime;       //exposure duration (ms)
  int focusRes;      //steps to divide lens range into for focus
  int focusRng;      //inverse fraction of total focal range to go through for autofocus
  int moveTol;       //precision (ticks) for lens moves
  double delay;	     //number of seconds between sending exposure command and pulse_sbsc 

  //image processing configuration
  int maxBlobs;      //max number of blobs to find
  int grid;          //search grid cell size (pix)
  double threshold;  //# sigma threshold for star finding
  int minBlobDist;   //min dist (pix) between blobs
  
  //uplink commands
  char uplink_cmd[SBSC_CMD_Q_SIZE][SBSC_COMM_BUF_SIZE];
  int i_uplink_r;
  int i_uplink_w;
};

struct CommandDataStruct {
  unsigned short command_count;
  unsigned short last_command;

  struct {
    unsigned short dac_out[5];
    unsigned char setLevel[5];
  } Temporary;

  unsigned short int timeout;
  unsigned short int slot_sched; // what slot to use
  unsigned short int upslot_sched; // slot being uplinked
  unsigned int parts_sched; // bitfield up pulinked parts
  unsigned short int uplink_sched; // use uplink sched
  unsigned short int sucks;
  unsigned short int lat_range;
  unsigned short int at_float;
  unsigned int tdrss_bw;
  unsigned int iridium_bw;
  unsigned int pilot_bw;
  unsigned int channelset_oth;
  
  enum {vtx_isc, vtx_osc, vtx_sbsc} vtx_sel[2];

  /*
  double apcu_reg;
  double  apcu_trim;
  short int apcu_auto;
  double dpcu_reg;
  double dpcu_trim;
  short int dpcu_auto;
  */
  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct PivGainStruct pivot_gain;

  struct SBSCCommandData cam;
  
  struct {
    struct latch_pulse sc_tx;
    struct latch_pulse das;
    struct latch_pulse isc;
    struct latch_pulse osc;
    struct latch_pulse sbsc;
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
    unsigned char adc_reset[16];
  } power;

  unsigned short disable_az;
  unsigned short disable_el;
  unsigned short force_el;

  unsigned short reset_rw;
  unsigned short reset_piv;
  unsigned short reset_elev;
  unsigned short restore_piv;

  unsigned short verbose_rw;
  unsigned short verbose_el;
  unsigned short verbose_piv;
  int az_autogyro;
  int el_autogyro;
  double offset_ifel_gy;
  double offset_ifroll_gy;
  double offset_ifyaw_gy;
  unsigned int gymask;

  struct {
    double setpoint;
    int age;
    struct GainStruct gain;
  } gyheat;

  unsigned char use_elenc;
  unsigned char use_elclin;
  unsigned char use_pss;
  unsigned char use_isc;
  unsigned char use_osc;
  unsigned char use_mag;
  unsigned char use_gps;

  unsigned short fast_offset_gy;
  unsigned int slew_veto;

  double az_accel;

  double clin_el_trim;
  double enc_el_trim;
  double null_az_trim;
  double mag_az_trim;
  double dgps_az_trim;
  double pss_az_trim;

  int autotrim_enable;
  double autotrim_thresh;    //in sc sigma
  double autotrim_rate;      //degrees/s
  time_t autotrim_time;      //in seconds
  time_t autotrim_isc_last_bad;
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
    unsigned short bias[5];
    unsigned char setLevel[5];
    struct Step biasStep;
  } Bias;
  
  struct {
    unsigned short charcoalHeater;
    unsigned short hsCharcoal;
    unsigned short fridgeCycle;
    unsigned short force_cycle;

    double cycle_start_temp;
    double cycle_pot_max;
    double cycle_charcoal_max;
    double cycle_charcoal_settle;
    //timeouts in minutes (NB: time will probably be in seconds)
    double cycle_charcoal_timeout;
    double cycle_settle_timeout;

    unsigned short BDAHeat;
    unsigned short hsPot;
    short heliumLevel;
    int he4_lev_old;
    short hwprPos;
    int hwpr_pos_old;

    unsigned short JFETHeat;
    unsigned short autoJFETheat;
    double JFETSetOn, JFETSetOff;

    enum calmode calibrator;
    unsigned short calib_pulse, calib_period;
    int calib_repeats;
    int calib_hwpr; 

    unsigned short potvalve_open, potvalve_on, potvalve_close;
    unsigned short lvalve_open, lhevalve_on, lvalve_close, lnvalve_on;
  } Cryo;

  struct {
    enum {bal_rest, bal_manual, bal_auto} mode;
    double level;

    // servo parameters
    double level_on_bal;
    double level_off_bal;
    double level_target_bal;
    double gain_bal;

    // heating card parameters
    double heat_on;
    double heat_tset;

  } pumps;

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
    unsigned short act_tol;

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

    unsigned int lock_goal;

    /* shutter control */
    int shutter_step;
    int shutter_step_slow;
    int shutter_out;

    unsigned int  shutter_goal;

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

  struct {
    int x1, y1, x2, y2, step, xvel, yvel, is_new, mode;
    int force_repoll;
  } xystage;

  int pin_is_in;

  /* sensors output: read in mcp:SensorReader() */
  unsigned short temp1, temp2, temp3;
  unsigned short df;

  unsigned short plover;

  unsigned short bi0FifoSize;
  unsigned short bbcFifoSize;

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
    int max_age;    //maximum allowed time between trigger and solution
    int age;	    //last measured time between trigger and solution
  } ISCControl[2];
};

void InitCommandData();
double LockPosition(double);
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);

extern struct CommandDataStruct CommandData;

#endif   //COMMAND_STRUCT_H

