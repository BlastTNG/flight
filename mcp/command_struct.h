#define N_SLOWDL   7

#define POINT_VEL 0
#define POINT_POSITION 1
#define POINT_LOCK 2

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

struct PointModeStruct {
  int az_mode;
  int el_mode;
  double az_dest;
  double el_dest;
  double az_vel;
  double el_vel;
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
  int calibrator;
  int sparePwm;
  int JFETHeat;
  int heatSwitch;
  int heliumThree;
  int lnvalve_open, lnvalve_on;
  int lnvalve_close, lnvalve_off;
  int lhevalve_open, lhevalve_on;
  int lhevalve_close, lhevalve_off;
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

  unsigned char current_mode;
  unsigned char default_mode;

  unsigned char default_sensor;
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
  struct PointModeStruct point_mode;
};

int bc_setserial(char *input_tty);
void InitCommandData();
double LockPosition(double);
