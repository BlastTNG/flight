#include "isc_protocol.h"
#include "tx_struct.h"

#define N_SLOWDL   7

#define AXIS_VEL 0
#define AXIS_POSITION 1
#define AXIS_LOCK 2

#define P_AZEL_GOTO  1
#define P_AZ_SCAN    2
#define P_DRIFT      3
#define P_RADEC_GOTO 4
#define P_VCAP       5
#define P_CAP        6
#define P_BOX        7
#define P_LOCK       8

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
  time_t t_start_sched;
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

  int disable_az;
  int disable_el;

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
  short fan;
  short T;
  int df;

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)

  /* Integrating Star Camera Stuff */
  struct ISCStatusStruct ISCState;
  int ISC_pulse_width;
  int old_ISC_focus;
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

extern struct CommandDataStruct CommandData;
