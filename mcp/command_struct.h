/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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

#include "isc_protocol.h"
#include "command_list.h"
#include "channels.h"
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

#define MAX_ISC_SLOW_PULSE_SPEED 0.015

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
};

#define LS_OPEN        0x0001
#define LS_CLOSED      0x0002
#define LS_DRIVE_OFF   0x0004
#define LS_POT_RAIL    0x0008
#define LS_DRIVE_EXT   0x0010
#define LS_DRIVE_RET   0x0020
#define LS_DRIVE_STP   0x0040
#define LS_DRIVE_UNK   0x0080
#define LS_EL_OK       0x0100
#define LS_IGNORE_EL   0x0200
#define LS_DRIVE_FORCE 0x0400
#define LS_DRIVE_MASK  0x04F4

#define ACTBUS_FM_SLEEP  0
#define ACTBUS_FM_SERVO  1
#define ACTBUS_FM_FOCUS  2
#define ACTBUS_FM_OFFSET 3
#define ACTBUS_FM_THERMO 4
#define ACTBUS_FM_NOW    5
#define ACTBUS_FM_DELTA  6
#define ACTBUS_FM_PANIC  7
#define ACTBUS_FM_DELFOC 8

#define TC_MODE_ENABLED  0
#define TC_MODE_AUTOVETO 1
#define TC_MODE_VETOED   2

#define XYSTAGE_PANIC  0
#define XYSTAGE_GOTO   1
#define XYSTAGE_JUMP   2
#define XYSTAGE_SCAN   3
#define XYSTAGE_RASTER 4

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
};

enum calmode { on, off, pulse, repeat };

struct CommandDataStruct {
  unsigned short int timeout;
  unsigned short int alice_file;
  unsigned short int sucks;
  unsigned short int lat_range;
  unsigned short int at_float;

  double apcu_reg;
  double  apcu_trim;
  short int apcu_auto;
  double dpcu_reg;
  double dpcu_trim;
  short int dpcu_auto;

  struct GainStruct roll_gain;
  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct GainStruct pivot_gain;

  int emf_gain;     /* for reaction wheel  */
  int emf_offset;   /*   back-EMF tweaking */

  struct {
    int gps;       /* dgps is off */
    int gyro;      /* digital gybox is off */
    int isc;       /* isc is off */
    int osc;       /* osc is off */
    int ss;        /* ss is off */
  } sensors_off;

  unsigned short disable_az;
  unsigned short disable_el;
  unsigned short force_el;
  unsigned short use_analogue_gyros;

  int az_autogyro;
  int el_autogyro;
  double gy1_offset;
  double gy2_offset;
  double gy3_offset;

  struct {
    double setpoint;
    int age;
    struct GainStruct gain;
  } gyheat[2];

  unsigned char use_elenc;
  unsigned char use_elclin;
  unsigned char use_sun;
  unsigned char use_isc;
  unsigned char use_osc;
  unsigned char use_mag;
  unsigned char use_gps;

  unsigned short fast_gy_offset;
  unsigned int slew_veto;

  double clin_el_trim;
  double enc_el_trim;
  double null_az_trim;
  double mag_az_trim;
  double dgps_az_trim;
  double ss_az_trim;

  struct {
    int dont_do_anything;
    int clockInternal;
    int biasAC;
    int biasRamp;
    int bias1;
    int bias2;
    int bias3;
    int SetLevel1, SetLevel2, SetLevel3;
  } Bias;
  
  struct {
    short heliumLevel;
    unsigned short coldPlate;
    unsigned short heatSwitch;
    unsigned short CryoSparePWM;

    enum calmode calibrator;
    unsigned short calib_pulse, calib_period;

    unsigned short autoBDAHeat;
    unsigned short BDAHeat;
    struct GainStruct BDAGain;
    unsigned short BDAFiltLen;

    unsigned short autoJFETheat;
    unsigned short JFETHeat;
    double JFETSetOn, JFETSetOff;
    int he4_lev_old;

    unsigned short charcoalHeater;
    unsigned short fridgeCycle;
    unsigned short force_cycle;

    unsigned short potvalve_open, potvalve_on, potvalve_close;
    unsigned short lvalve_open, lhevalve_on, lvalve_close, lnvalve_on;
  } Cryo;

  int Phase[DAS_CARDS];

  struct {
    int bal_veto;
    int bal1_on;
    int bal1_reverse;
    int bal2_on;
    int bal2_reverse;
    double bal_on;
    double bal_off;
    double bal_target;
    double bal_gain;

    int inframe_auto;
    int inframe_cool_on;
    int inframe_cool_off;
    int pwm1, pwm2, pwm3, pwm4;
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
    double sf_t_primary;
    double sf_t_secondary;
    int sf_position;
    int sf_time;
    int sf_in_focus;

    /* actuator control */
    int act_vel;
    int act_acc;
    int act_hold_i;
    int act_move_i;

    /* low-level actuator servo */
    int focus_mode;
    int goal[3];
    int delta[3];
    int offset[3];
    int focus;
    int dead_reckon[3];
    int last_good[3];
    int pos_trim[3];
    int reset_dr;
    int lvdt_num;
    double lvdt_low;
    double lvdt_high;

    /* lock control */
    int lock_vel;
    int lock_acc;
    int lock_hold_i;
    int lock_move_i;

    unsigned int lock_goal;
  } actbus;

  struct {
    int x1, y1, x2, y2, xvel, yvel, is_new, mode;
  } xystage;

  int pin_is_in;

  /* sensors output: read in mcp:SensorReader() */
  unsigned short fan;
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
  } ISCControl[2];
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
