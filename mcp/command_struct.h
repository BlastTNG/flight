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
#include "tx_struct.h"
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

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
};

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
  int mode;
  double X;
  double Y;
  double vaz;
  double del;
  double w;
  double h;
  time_t t;
};

struct PumpStruct {
  int bal_veto;
  int bal1_on;
  int bal1_reverse;
  int bal2_on;
  int bal2_reverse;
  double bal_on;
  double bal_off;
  double bal_target;
  double bal_gain;
  int bal_max;
  int bal_min;

  int inframe_cool1_on;
  int inframe_cool1_off;
  int lock_out;
  int lock_off;
  int lock_in;
  int lock_point;
  int outframe_cool1_on;
  int outframe_cool1_off;
  int outframe_cool2_on;
  int outframe_cool2_off;
  int pwm1, pwm2, pwm3, pwm4;
};

struct BiasStruct {
  int clockInternal;
  int biasAC;
  int biasRamp;
  int bias1;
  int bias2;
  int bias3;
  int SetLevel1, SetLevel2, SetLevel3;
};

enum calmode { on, off, pulse, repeat };

struct CryoStruct {
  unsigned short heliumLevel;
  unsigned short charcoalHeater;
  unsigned short coldPlate;
  enum calmode calibrator;
  unsigned short calib_pulse, calib_period;
  unsigned short autoBDAHeat;
  unsigned short BDAHeat;
  struct GainStruct BDAGain;
  unsigned short BDAFiltLen;
  unsigned short JFETHeat;
  unsigned short heatSwitch;
  unsigned short CryoSparePWM;
  unsigned short potvalve_open, potvalve_on, potvalve_close;
  unsigned short lhevalve_open, lhevalve_on, lhevalve_close;
};

struct ISCControlStruct {
  int pulse_width;
  int fast_pulse_width;
  int old_focus;
  int reconnect;
  int autofocus;
  int save_period;
  int auto_save;
};

struct CommandDataStruct {
  unsigned short int timeout;
  unsigned short int alice_file;

  struct GainStruct roll_gain;
  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct GainStruct pivot_gain;
  struct GainStruct gy_heat_gain;

  int emf_gain;     /* for reaction wheel  */
  int emf_offset;   /*   back-EMF tweaking */

  unsigned short disable_az;
  unsigned short disable_el;
  unsigned short force_el;

  int autogyro;
  double gy2_offset;
  double gy3_offset;

  double t_gybox_setpoint;
  double t_isc_setpoint;

  unsigned char use_elenc;
  unsigned char use_elclin;
  unsigned char use_sun;
  unsigned char use_isc;
  unsigned char use_mag;
  unsigned char use_gps;

  struct BiasStruct Bias;

  struct CryoStruct Cryo;
  int Phase[DAS_CARDS];

  struct PumpStruct pumps;

  /* sensors output: read in mcp:SensorReader() */
  unsigned short fan;
  unsigned short temp1, temp2, temp3;
  unsigned short df;

  unsigned short plover;

  unsigned short bi0FifoSize;
  unsigned short bbcFifoSize;
  unsigned short tdrssVeto;
  unsigned short ADC_sync_timeout;

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)

  /* Integrating Star Camera Stuff */
  struct ISCStatusStruct ISCState[2];
  struct ISCControlStruct ISCControl[2];
};

struct ScheduleType {
  int n_sched;
  time_t t0;
  struct PointingModeStruct *p;
};

int bc_setserial(char *input_tty);
void InitCommandData();
double LockPosition(double);

extern struct CommandDataStruct CommandData;
