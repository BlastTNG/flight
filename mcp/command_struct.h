#include "tx_struct.h"

#define N_SLOWDL   7

#define AXIS_VEL 0
#define AXIS_POSITION 1
#define AXIS_LOCK 2

#define POINT_VEL 1
#define POINT_POINT 2
#define POINT_LOCK 4
#define POINT_SCAN 8
#define POINT_RASTER 16
#define POINT_RADEC_GOTO 32

struct SlowDLStruct {
  char src[20];
  char type;
  int numbits;
  double value;
  int ind1;
  int ind2;
  float m_c2e;
  float b_e2e;
  int ctype;
};

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
};

struct PointingModeStruct {
  // Used by:        VEL   POINT   LOCK   SCAN   RASTER
  int az_mode;
  double az1;     //         **            **      .
  double az2;     //                       **      .
  double az_vel;  //  **                   **      **
  int el_mode;
  double el1;     //         **     **     **      .
  double el2;     //                       **      .
  double el_vel;  //  **                   **      **
  double ra;      //                               **
  double dec;     //                               **
  double r;       //                               **
  time_t t_start_sched; // after this time, use sched file
};

struct PumpStruct {
  int bal_veto;
  int bal1_on;
  int bal1_reverse;
  int bal2_on;
  int bal2_reverse;
  float bal_on;
  float bal_off;
  float bal_target;
  int inframe_cool1_on;
  int inframe_cool1_off;
  int lock_out;
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
};

struct CryoStruct {
  int heliumLevel;
  int charcoalHeater;
  int coldPlate;
  int calibrator, calib_pulse, calib_repeat;
  int sparePwm;
  int JFETHeat;
  int heatSwitch;
  int heliumThree;
  int lnvalve_open, lnvalve_on, lnvalve_close;
  int lhevalve_open, lhevalve_on, lhevalve_close;
};

struct CommandDataStruct {
  unsigned short int timeout;

  struct GainStruct roll_gain;
  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct GainStruct pivot_gain;
  struct GainStruct gy_heat_gain;
  struct GainStruct isc_heat_gain;
  double t_gybox_setpoint;
  double t_isc_setpoint;

  unsigned char use_sun;
  unsigned char use_isc;
  unsigned char use_vsc;
  unsigned char use_mag;
  unsigned char use_gps;

  struct BiasStruct Bias;

  struct CryoStruct Cryo;
  int Phase[DAS_CARDS];

  struct PumpStruct pumps;

  // sensors output: read in mcp:SensorReader()
  short fan;
  short T;
  int df;

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)
};

struct EventType {
  time_t t;          /* event time in comoving local sideral seconds */
  double ra;
  double dec;
  double r;
  double el_vel;
  double az_vel;
};

struct ScheduleType {
  int n_sched;
  time_t t0;
  struct EventType *e;
};

int bc_setserial(char *input_tty);
void InitCommandData();
double LockPosition(double);
