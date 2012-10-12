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

#include <stdbool.h>	  //bool type!
#include <time.h>
#include "command_list.h"
#include "channels.h"

#define AXIS_VEL      0
#define AXIS_POSITION 1
#define AXIS_LOCK     2

#define P_AZEL_GOTO  1
#define P_AZ_SCAN    2
#define P_DRIFT      3
#define P_RADEC_GOTO 4
#define P_LOCK       5
#define P_SPIDER     6
#define P_SINE       7

/* latching relay pulse length in 200ms slow frames */
#define LATCH_PULSE_LEN	 2
/* time (slow frames) to keep power off when power cycling devices */
#define PCYCLE_HOLD_LEN	 20
/* time (in slow frames) to suppress ADC card watchdog, to induce reset */
#define	RESET_ADC_LEN	 80

#define PREV_STATUS_FILE "/data/etc/spider/pcm.prev_status"

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
  unsigned short int PT; // position velocity gain
};

// used for pivot loop gains
struct PivGainStruct { 
  unsigned short int V_RW; // prop to RW velocity
  unsigned short int V_AZ; // prop to gondola az speed
  unsigned short int P_RW; // prop to RW position (integrated velocity)
  unsigned short int T_RW; // prop to gondola az accel
  double SP; // RW velocity Set Point 
};

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
// VCAP*       ra    dec  vaz   vel    r         
// CAP*        ra    dec  vaz   elstep r        
// BOX*        ra    dec  vaz   elstep w    h    
// SPIDER      ra    dec        elstep                 
// SINE        az    el                w

// *not used for Spider

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
  double dith; // Elevation dither step
  // TODO: Write the three below out to frame???
  int Nscans;  // number of half-scans per el step in SPIDER mode
  int Nsteps;  // total number of el steps in SPIDER mode
  int new_spider; // set to 1 if a new, distinct spider scan is commanded
};

struct latch_pulse {
  int set_count;
  int rst_count;
};

// for HK Theo heaters
// max power in Watts, TODO update command_list with these values
#define HK_MT_BOTTOM_PMAX   4.5 // TODO add H2B2 in parallel --> 9
#define HK_T1_PMAX          1.0 // TODO install capillary 4K heater
#define HK_VCS1_HX1_PMAX    4.5 // TODO put VCS1 HX2 in parallel --> 6.3
#define HK_VCS2_HX1_PMAX    1.8 // TODO put VCS2 HX2 in parallel --> 3.6
#define HK_VCS1_HX2_PMAX    1.8 // TODO replace with MT fill?
#define HK_VCS2_HX2_PMAX    1.8 // TODO replace with MT vent?
#define HK_SFT_BOTTOM_PMAX  9.0 // TODO change to LC --> 0.250
#define HK_T7_PMAX          1.0 // not wired!

struct PWMStruct {
  bool state;                 // heater state (0=off, 1=on)
  int duration;               // seconds requested (-1=infinity)
  time_t start_time;          // start time of pulse mode
  unsigned short duty_target; // duty cycle target (8-bit resolution)
  unsigned short duty_avg;    // running average duty cycle
};

// for HK RTD's
struct RTDStruct {
  double phase;
  double ampl;
  bool do_phase_step;
  double phase_start;
  double phase_end;
  double phase_step;
  int phase_dt;
  time_t phase_time;
};

struct SCCommandData {
  //camera and lens configuration
  short int paused;  //1=image capture paused, 0=image capture running
  int expInt;        //exposure interval (ms) (0=triggered)
  int expTime;       //exposure duration (ms)
  int focusRes;      //steps to divide lens range into for focus
  int focusRng;      //inverse fraction of total focal range to go through for autofocus
  int moveTol;       //precision (ticks) for lens moves

  //image processing configuration
  int maxBlobs;      //max number of blobs to find
  int grid;          //search grid cell size (pix)
  double threshold;  //# sigma threshold for star finding
  int minBlobDist;   //min dist (pix) between blobs
};

struct TableStruct {
  struct GainStruct tableGain;	//PID
  double vel;
  double pos;
  double move;
  double mode;		//0=track, 1=move to pos, 2=relative move
};

struct CommandDataStruct {
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
  
  enum {vtx_isc, vtx_osc, vtx_bsc} vtx_sel[2];

  //struct GainStruct ele_gain;

  struct {
    double com;
    double diff;
    double pulse_port;
    double pulse_starboard;
    int manual_pulses;
    double twist;
  } ele_gain; 
 
  struct GainStruct azi_gain;
  struct PivGainStruct pivot_gain;

  struct SCCommandData thegood;
  struct SCCommandData thebad;
  struct SCCommandData theugly;
  struct TableStruct table;

  struct {
    struct latch_pulse sc_tx;
    struct latch_pulse das;
    struct latch_pulse isc;
    struct latch_pulse osc;
    struct latch_pulse gps;
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
    int lock_off;
    int thegood_cpu_off;
    int thebad_cpu_off;
    int theugly_cpu_off;
    int thegood_cam_off;
    int thebad_cam_off;
    int theugly_cam_off;
    unsigned char adc_reset[16];
    int elmot_auto; // automatically power el motors on/off before/after a move
    int elmot_is_on; // 1 if el drive on, 0 if not
  } power;

  unsigned short disable_az;
  unsigned short disable_el;
  unsigned short force_el;

  unsigned short reset_rw;
  unsigned short reset_piv;
  unsigned short reset_elev;
  unsigned short restore_piv;
  unsigned short restore_rw;  // JAS--added this in, not sure if we need it yet

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

  double t_set_bsc;
  double t_set_rsc;

  unsigned char use_elenc1;
  unsigned char use_elenc2;
  unsigned char use_elclin;
  unsigned char use_pss;
  unsigned char use_mag;
  unsigned char use_gps;

  double dgps_cov_limit;
  double dgps_ants_limit;

  unsigned short fast_offset_gy;
  unsigned int slew_veto;

  double az_accel;
  double az_accel_max; // max gondola accel in az

  double clin_el_trim;
  double enc_el_trim;
  double null_az_trim;
  double mag_az_trim;
  double dgps_az_trim;
  double pss_az_trim;

  struct {
    bool pump_heat;
    bool heat_switch;
    bool fphi_heat;
    bool ssa_heat;
    bool htr1_heat;
    bool htr2_heat;
    bool htr3_heat;
    double fplo_heat;
    double strap_heat;

    bool auto_cycle_on;
    bool force_cycle;
    
    bool pump_servo_on;
    double pump_servo_low;
    double pump_servo_high;
    
    struct RTDStruct cernox;
    struct RTDStruct ntd;
  } hk[6];    //one per insert

  struct PWMStruct hk_theo_heat[8];

  short hk_last;
  double hk_vheat_last;
  unsigned short hk_bias_freq;

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
  } actbus;

  struct {
    enum {lock_do_nothing = 0, lock_insert, lock_el_wait_insert,
      lock_retract} goal;
    enum {lock_unknown = 0, lock_closed, lock_open, lock_closing,
      lock_opening} state_p, state_s;
    int pin_is_in;
  } lock;

  struct {
    /* move parameters */
    double vel;    //velocity in dps, shared by all motors
    double move_i; //hold current in A, shared by all motors

    /* command parameters */
    int who;
    double delta; //degrees
    enum {hwp_m_sleep, hwp_m_panic, hwp_m_halt, hwp_m_rel_move,
      hwp_m_step} mode;
    int force_repoll;
    double phase;

    /* arbitrary command */
    int cindex;
    int caddr[3];
    char command[3][CMD_STRING_LEN];
  } hwp;

  struct {
    int x1, y1, x2, y2, step, xvel, yvel, is_new, mode;
    int force_repoll;
  } xystage;

  /* sensors output: read in mcp:SensorReader() */
  unsigned short temp1, temp2, temp3;
  size_t df;

  unsigned short plover;

  unsigned short bi0FifoSize;
  unsigned short bbcFifoSize;

  unsigned int bbcIntFrameRate;         //internal frame rate (ADC samples)
  unsigned int bbcExtFrameRate;         //external frame rate (sync frames)
  unsigned int bbcExtFrameMeas;         //measured external rate (32MHz)
  unsigned char bbcIsExt;               //is the bbc in external mode
  unsigned char bbcAutoExt;             //should pcm auto-switch ext/int mode

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)
  double lat;
  double lon;

  unsigned short questionable_behaviour;

};

struct ScheduleEvent {
  int t;
  int is_multi;
  int command;
  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];
};

struct ScheduleType {
  int n_sched;
  time_t t0;
  struct ScheduleEvent* event;
};

void InitCommandData();
double LockPosition(double);
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);

extern struct CommandDataStruct CommandData;

#endif   //COMMAND_STRUCT_H

