#define N_SLOWDL   7

#define AXIS_VEL 0
#define AXIS_POSITION 1
#define AXIS_LOCK 2

#define POINT_VEL 0
#define POINT_POINT 1
#define POINT_LOCK 2
#define POINT_SCAN 3
#define POINT_RASTER 4

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

struct AxesModeStruct {
  int az_mode;
  int el_mode;
  double az_dest;
  double el_dest;
  double az_vel;
  double el_vel;
};

struct PointingModeStruct {
  int az_mode;
  int el_mode;
  double az1; // pointing, scan
  double el1; // poining, scan, lock
  double az2; // scan
  double el2; // scan
  double az_vel; // scan, map, vel
  double el_vel; // scan, map, vel
  double ra; // map
  double dec; // map
  double r; //map
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

  struct BiasStruct Bias;

  struct CryoStruct Cryo;
  int Phase[DAS_CARDS];

  struct PumpStruct pumps;

  // sensors output: read in mcp:SensorReader()
  short fan;
  short T;

  struct AxesModeStruct axes_mode; // low level velocity mode
  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)
};

int bc_setserial(char *input_tty);
void InitCommandData();
double LockPosition(double);
