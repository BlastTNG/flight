#define INCLUDE_VARS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <pthread.h>

#include "tx_struct.h"
#include "command_struct.h"
#include "pointing_struct.h"

#include "commands.h"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

#define BAL_VETO_LENGTH 500

/* #define SUN             0 */
/* #define ISC             1 */
/* #define VSC             2 */
/* #define MAG             3 */

#define TAKEBIT    0
#define FORCEINT   1
#define U_MASK     2

#define LOCK_OFFSET (0.6)

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

void SetRaDec(double ra, double dec); // defined in pointing.c

struct SlowDLStruct SlowDLInfo[N_SLOWDL] = {
  {"t_dpm_3v", FORCEINT, 8, -1, -1, -1, -1, -1, -1},
  {"cpu_time", U_MASK,   8, -1, -1, -1, -1, -1, -1},
  {"t_gybox",  FORCEINT, 8, -1, -1, -1, -1, -1, -1},
  {"gyro1",    FORCEINT, 8, -1, -1, -1, -1, -1, -1},
  {"t_reac",   FORCEINT, 7, -1, -1, -1, -1, -1, -1},
  {"g_p_el",   FORCEINT, 8, -1, -1, -1, -1, -1, -1},
  {"i_el",     TAKEBIT,  3, -1, -1, -1, -1, -1, -1}
};

pthread_mutex_t mutex;

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus() {
  int fp, n;

  /** write the default file */
  fp = open("/tmp/mcp.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    perror("mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    perror("mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    perror("mcp.prev_status close()");
    return;
  }
}

void bc_close() {
}

int bc_setserial(char *input_tty) {
  int fd;
  struct termios term; 

  if ((fd = open(input_tty, O_RDWR)) < 0) {
    perror("Unable to open serial port");
    return -1;
  }

  if (tcgetattr(fd, &term)) {
    perror("Unable to get serial device attributes");
    return -1;
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if(cfsetospeed(&term, B1200)) {          /*  <======= SET THE SPEED HERE */
    perror("Error setting serial output speed");
    return -1;
  }
  if(cfsetispeed(&term, B1200)) {          /*  <======= SET THE SPEED HERE */
    perror("Error setting serial input speed");
    return -1;
  }

  if( tcsetattr(fd, TCSANOW, &term) ) {
    perror("Unable to set serial attributes");
    return -1;
  }

  return fd;
}

/* calculate the nearest lockable elevation */
double LockPosition (double elevation) {
  double position;

  position = floor(elevation / 10.0) * 10.0 + 5.0;
  if (position > 79.0) {
    position = 90.0;
  } else if (position < 10.0) {
    position = 5.0;
  }

  return position + LOCK_OFFSET;
}

float ParseGPS (unsigned char *data) {
  char exponent;
  char sign;
  long mantissa_bits;
  float mantissa;
  int i;

  mantissa = 0;
  mantissa_bits = 0;
  sign = 0;
  exponent = 0;

  /* N.B. that the data bytes are received in backwards order */

  if (((*(data + 3) >> 7) & 0x01) == 1)  /* First bit encodes the sign */
    sign = -1;
  else
    sign = 1;

  exponent = ((*(data + 3) << 1) & 0xFF) + ((*(data + 2) >> 7)  & 0x01) - 127;  /* Next eight bits = exponent + 127 */

  /* Mantissa contained in last 23 bits */
  mantissa_bits = ((*(data + 2) & 0x7F) << 16) + (*(data + 1) << 8) + *data;

  for (i = 23; i >= 0; i--) {           /* Construct mantissa = Sigma 2^n */
    if ((mantissa_bits >> i) & 0x01)
      mantissa += pow(2, i - 23);
  }

  return((mantissa + 1) * pow(2, exponent) * sign);
}

void SendRequest (int req, char tty_fd) {
  unsigned char buffer[3];

  buffer[0] = 0x10;
  buffer[1] = req;
  buffer[2] = 0x03;

  write(tty_fd, buffer, 3);
}


int SIndex(char *cmd) {
  int i;

  for (i = 0; i < N_NM_SCOMMANDS; i++) {
    if (strcmp(scommands[i].name, cmd) == 0)
      return i;
  }

  return -1;
}

/*** set fields unused in current mode to zero ***/
void ClearPointingModeExtraFields() {
  if ((CommandData.pointing_mode.az_mode != POINT_RASTER) &&
      (CommandData.pointing_mode.el_mode != POINT_RASTER) &&
      (CommandData.pointing_mode.az_mode != POINT_RADEC_GOTO) &&
      (CommandData.pointing_mode.el_mode != POINT_RADEC_GOTO)) {
    CommandData.pointing_mode.ra = CommandData.pointing_mode.dec =
      CommandData.pointing_mode.r = 0.0;
  }

  if (CommandData.pointing_mode.az_mode == POINT_VEL) {
    CommandData.pointing_mode.az1 = CommandData.pointing_mode.az2 = 0.0;
  }

  if (CommandData.pointing_mode.az_mode == POINT_POINT) {
    CommandData.pointing_mode.az2 = CommandData.pointing_mode.az_vel = 0.0;
  }

  if (CommandData.pointing_mode.el_mode == POINT_VEL) {
    CommandData.pointing_mode.el1 = CommandData.pointing_mode.el2 = 0.0;
  }

  if (CommandData.pointing_mode.el_mode == POINT_POINT) {
    CommandData.pointing_mode.el2 = CommandData.pointing_mode.el_vel = 0.0;
  }
}

void SingleCommand (int command) {
  int i_point;

  fprintf(stderr, "Single command %d: %s\n", command, scommands[command].name);

  /* Update CommandData structure with new info */

  if (command == SIndex("stop")) {      /* Pointing aborts */
    CommandData.pointing_mode.el_mode = POINT_VEL;
    CommandData.pointing_mode.az_mode = POINT_VEL;
    CommandData.pointing_mode.az_vel = 0.0;
    CommandData.pointing_mode.el_vel = 0.0;
    ClearPointingModeExtraFields();
  }

  else if (command == SIndex("az_off")) /* disable az motors */
    CommandData.disable_az = 1;
  else if (command == SIndex("az_on")) /* enable az motors */
    CommandData.disable_az = 0;
  else if (command == SIndex("el_off")) /* disable el motors */
    CommandData.disable_el = 1;
  else if (command == SIndex("el_on")) /* enable el motors */
    CommandData.disable_el = 0;

  else if (command == SIndex("sun_veto"))       /* Veto sensors */
    CommandData.use_sun = 0;
  else if (command == SIndex("isc_veto"))
    CommandData.use_isc = 0;
  else if (command == SIndex("mag_veto"))
    CommandData.use_mag = 0;
  else if (command == SIndex("gps_veto"))
    CommandData.use_gps = 0;
  else if (command == SIndex("elenc_veto"))
    CommandData.use_elenc = 0;
  else if (command == SIndex("elclin_veto"))
    CommandData.use_elclin = 0;


  else if (command == SIndex("sun_allow"))       /* Un-veto sensors */
    CommandData.use_sun = 1;
  else if (command == SIndex("isc_allow"))
    CommandData.use_isc = 1;
  else if (command == SIndex("mag_allow"))
    CommandData.use_mag = 1;
  else if (command == SIndex("gps_allow"))
    CommandData.use_gps = 1;
  else if (command == SIndex("elenc_allow"))
    CommandData.use_elenc = 1;
  else if (command == SIndex("elclin_allow"))
    CommandData.use_elclin = 1;

  else if (command == SIndex("clock_int"))    /* Bias settings */
    CommandData.Bias.clockInternal = 1;
  else if (command == SIndex("clock_ext"))
    CommandData.Bias.clockInternal = 0;
  else if (command == SIndex("bias_ac"))
    CommandData.Bias.biasAC = 1;
  else if (command == SIndex("bias_dc"))
    CommandData.Bias.biasAC = 0;
  else if (command == SIndex("ramp"))
    CommandData.Bias.biasRamp = 1;
  else if (command == SIndex("fixed"))
    CommandData.Bias.biasRamp = 0;

  else if (command == SIndex("level_on"))    /* Cryo commanding */
    CommandData.Cryo.heliumLevel = 1;
  else if (command == SIndex("level_off"))
    CommandData.Cryo.heliumLevel = 0;
  else if (command == SIndex("charcoal_on"))
    CommandData.Cryo.charcoalHeater = 1;
  else if (command == SIndex("charcoal_off"))
    CommandData.Cryo.charcoalHeater = 0;
  else if (command == SIndex("coldplate_on"))
    CommandData.Cryo.coldPlate = 1;
  else if (command == SIndex("coldplate_off"))
    CommandData.Cryo.coldPlate = 0;
  else if (command == SIndex("cal_on"))
    CommandData.Cryo.calibrator = 1;
  else if (command == SIndex("cal_off"))
    CommandData.Cryo.calibrator = 0;
  else if (command == SIndex("cal_stop"))
    CommandData.Cryo.calib_pulse = 0;
  else if (command == SIndex("ln_valve_open")) {
    CommandData.Cryo.lnvalve_open = 40;
    CommandData.Cryo.lnvalve_close = 0;
  } else if (command == SIndex("ln_valve_close")) {
    CommandData.Cryo.lnvalve_close = 40;
    CommandData.Cryo.lnvalve_open = 0;
  } else if (command == SIndex("ln_valve_on"))
    CommandData.Cryo.lnvalve_on = 1;
  else if (command == SIndex("ln_valve_off"))
    CommandData.Cryo.lnvalve_on = 0;
  else if (command == SIndex("he_valve_open")) {
    CommandData.Cryo.lhevalve_open = 40;
    CommandData.Cryo.lhevalve_close = 0;
  } else if (command == SIndex("he_valve_close")) {
    CommandData.Cryo.lhevalve_close = 40;
    CommandData.Cryo.lhevalve_open = 0;
  } else if (command == SIndex("he_valve_on"))
    CommandData.Cryo.lhevalve_on = 1;
  else if (command == SIndex("he_valve_off"))
    CommandData.Cryo.lhevalve_on = 0;

  else if (command == SIndex("balance_veto"))
    CommandData.pumps.bal_veto = -1;
  else if (command == SIndex("balance_allow"))
    CommandData.pumps.bal_veto = 0;

  else if (command == SIndex("pump1_on"))
    CommandData.pumps.bal1_on = 1;
  else if (command == SIndex("pump1_off"))
    CommandData.pumps.bal1_on = 0;
  else if (command == SIndex("pump1_fwd"))
    CommandData.pumps.bal1_reverse = 0;
  else if (command == SIndex("pump1_rev"))
    CommandData.pumps.bal1_reverse = 1;
  else if (command == SIndex("pump2_on"))
    CommandData.pumps.bal2_on = 1;
  else if (command == SIndex("pump2_off"))
    CommandData.pumps.bal2_on = 0;
  else if (command == SIndex("pump2_fwd"))
    CommandData.pumps.bal2_reverse = 0;
  else if (command == SIndex("pump2_rev"))
    CommandData.pumps.bal2_reverse = 1;

  else if (command == SIndex("inner_cool_on"))
    CommandData.pumps.inframe_cool1_on = 40;
  else if (command == SIndex("inner_cool_off"))
    CommandData.pumps.inframe_cool1_off = 40;

  else if (command == SIndex("outer_cool_on"))
    CommandData.pumps.outframe_cool1_on = 40;
  else if (command == SIndex("outer_cool_off"))
    CommandData.pumps.outframe_cool1_off = 40;
  else if (command == SIndex("outer_spare_on"))
    CommandData.pumps.outframe_cool2_on = 40;
  else if (command == SIndex("outer_spare_off"))
    CommandData.pumps.outframe_cool2_off = 40;
  else if (command == SIndex("pin_in"))
    CommandData.pumps.lock_in = 1;
  else if (command == SIndex("unlock")) {
    CommandData.pumps.lock_out = 1;
    if (CommandData.pointing_mode.el_mode == POINT_LOCK) {
      CommandData.pointing_mode.el_mode = POINT_VEL;
      CommandData.pointing_mode.el_vel = 0.0;
    }

    /***************************************/
    /********* ISC Commanding  *************/
  } else if (command == SIndex("isc_run")) {
    CommandData.ISCState.pause = 0;
  } else if (command == SIndex("pause")) {
    CommandData.ISCState.pause = 1;
  } else if (command == SIndex("isc_abort")) {
    CommandData.ISCState.abort = 1;
  } else if (command == SIndex("no_bright_star")) {
    CommandData.ISCState.brightStarMode = 0;
  } else if (command == SIndex("save_images")) {
    CommandData.ISCState.save = 1;
  } else if (command == SIndex("discard_images")) {
    CommandData.ISCState.save = 0;
  } else if (command == SIndex("full_screen")) {
    CommandData.ISCState.display_mode = full;
  } else if (command == SIndex("auto_focus")) {
    CommandData.ISCState.autofocus = 1;
    CommandData.old_ISC_focus = CommandData.ISCState.focus_pos;
    CommandData.ISCState.focus_pos = FOCUS_RANGE;
  } else {
    return; // invalid command - no write or update
  }

  i_point = GETREADINDEX(point_index);

  CommandData.pointing_mode.t_start_sched =
    PointingData[i_point].t + CommandData.timeout;

  WritePrevStatus();
}

int MIndex(char *cmd) {
  int i;

  for (i = 0; i < N_MCOMMANDS; i++) {
    if (strcmp(mcommands[i].name, cmd) == 0)
      return i;
  }

  return -1;
}

void MultiCommand (int command, unsigned short *dataq) {
  FILE *fp;

  int i, dataqind;
  double rvalues[MAX_N_PARAMS];
  unsigned short ivalues[MAX_N_PARAMS];
  char type;
  int i_point;


#ifndef BOLOTEST
  double min;

  /* compute renormalised values */
  for (i = dataqind = 0; i < mcommands[command].numparams; ++i) {
    min = mcommands[command].params[i].min;
    type = mcommands[command].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++];
      fprintf(stderr, "param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = (float)dataq[dataqind++] * (mcommands[command].params[i].max
          - min) / MAX_15BIT + min;
      fprintf(stderr, "param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = (float)((int)dataq[dataqind++] << 15); /* upper 15 bits */
      rvalues[i] += (float)dataq[dataqind++];             /* lower 15 bits */
      rvalues[i] = rvalues[i] * (mcommands[command].params[i].max - min) /
        MAX_30BIT + min;
      fprintf(stderr, "param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }
#else
  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  for (i = dataqind = 0; i < mcommands[command].numparams; ++i) {
    type = mcommands[command].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      fprintf(stderr, "param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      fprintf(stderr, "param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      fprintf(stderr, "param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }
#endif

  /* Update CommandData struct with new info
   * If the parameter is type 'i'     set CommandData using ivalues[i]
   * If the parameter is type 'f'/'l' set CommandData using rvalues[i]
   */
  if (0) { // allow easy re-arranging

    /***************************************/
    /********** Pointing Mode **************/
  } else if (command == MIndex("ra_dec_raster")) {  /* raster a circle */
    CommandData.pointing_mode.az_mode = POINT_RASTER;
    CommandData.pointing_mode.el_mode = POINT_RASTER;
    CommandData.pointing_mode.ra = rvalues[0];	
    CommandData.pointing_mode.dec = rvalues[1];
    CommandData.pointing_mode.r = rvalues[2];
    CommandData.pointing_mode.az_vel = rvalues[3];
    CommandData.pointing_mode.el_vel = rvalues[4];
    CommandData.pointing_mode.az1 = 0.0;
    CommandData.pointing_mode.az2 = 0.0;
    CommandData.pointing_mode.el1 = 0.0;
    CommandData.pointing_mode.el2 = 0.0;
  } else if (command == MIndex("ra_dec_goto")) {  /* raster a circle */
    CommandData.pointing_mode.az_mode = POINT_RADEC_GOTO;
    CommandData.pointing_mode.el_mode = POINT_RADEC_GOTO;
    CommandData.pointing_mode.ra = rvalues[0];	
    CommandData.pointing_mode.dec = rvalues[1];
    CommandData.pointing_mode.r = 0.0;
    CommandData.pointing_mode.az_vel = 0.0;
    CommandData.pointing_mode.el_vel = 0.0;
    CommandData.pointing_mode.az1 = 0.0;
    CommandData.pointing_mode.az2 = 0.0;
    CommandData.pointing_mode.el1 = 0.0;
    CommandData.pointing_mode.el2 = 0.0;    
  } else if (command == MIndex("ra_dec_set")) {
    CommandData.pointing_mode.el_mode = POINT_VEL;
    CommandData.pointing_mode.az_mode = POINT_VEL;
    CommandData.pointing_mode.az_vel = 0.0;
    CommandData.pointing_mode.el_vel = 0.0;
    ClearPointingModeExtraFields();
    SetRaDec(rvalues[0], rvalues[1]);
  } else if (command == MIndex("az_scan")) {  /* scan in azimuth */
    CommandData.pointing_mode.az_mode = POINT_SCAN;
    CommandData.pointing_mode.az1 = rvalues[0]-rvalues[1]/2.0;
    CommandData.pointing_mode.az2 = rvalues[0]+rvalues[1]/2.0;
    CommandData.pointing_mode.az_vel = rvalues[2];
    CommandData.pointing_mode.ra = 0.0;
    CommandData.pointing_mode.dec = 0.0;
    CommandData.pointing_mode.r = 0.0;
    if (CommandData.pointing_mode.el_mode & (POINT_RASTER|POINT_RADEC_GOTO)) {
      CommandData.pointing_mode.el_mode = POINT_VEL;
      CommandData.pointing_mode.el_vel = 0.0;
    }
  } else if (command == MIndex("az_goto")) {  /* point in azimuth */
    CommandData.pointing_mode.az_mode = POINT_POINT;
    CommandData.pointing_mode.az1 = rvalues[0];
    if (CommandData.pointing_mode.el_mode & (POINT_RASTER|POINT_RADEC_GOTO)) {
      CommandData.pointing_mode.el_mode = POINT_VEL;
      CommandData.pointing_mode.el_vel = 0.0;
    }
  } else if (command == MIndex("az_vel")) {  /* fixed azimuth velocity */
    CommandData.pointing_mode.az_mode = POINT_VEL;
    CommandData.pointing_mode.az_vel = rvalues[0];
    if (CommandData.pointing_mode.el_mode & (POINT_RASTER|POINT_RADEC_GOTO)) {
      CommandData.pointing_mode.el_mode = POINT_VEL;
      CommandData.pointing_mode.el_vel = 0.0;
    }
  } else if (command == MIndex("el_goto")) {  /* point in elevation */
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    CommandData.pointing_mode.el_mode = POINT_POINT;
    CommandData.pointing_mode.el1 = rvalues[0];
    if (CommandData.pointing_mode.el_mode & (POINT_RASTER|POINT_RADEC_GOTO)) {
      CommandData.pointing_mode.az_mode = POINT_VEL;
      CommandData.pointing_mode.az_vel = 0.0;
    }
  } else if (command == MIndex("el_vel")) {  /* fixed elevation velocity */
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    CommandData.pointing_mode.el_mode = POINT_VEL;
    CommandData.pointing_mode.el_vel = rvalues[0];
    if (CommandData.pointing_mode.el_mode & (POINT_RASTER|POINT_RADEC_GOTO)) {
      CommandData.pointing_mode.az_mode = POINT_VEL;
      CommandData.pointing_mode.az_vel = 0.0;
    }
    /***************************************/
    /********** Pointing Motor Gains *******/
  } else if (command == MIndex("roll_gain")) { /* roll Gains */
    CommandData.roll_gain.P = ivalues[0];
  } else if (command == MIndex("el_gain")) {  /* ele gains */
    CommandData.ele_gain.P = ivalues[0];
    CommandData.ele_gain.I = ivalues[1];
  } else if (command == MIndex("az_gain")) {  /* az gains */
    CommandData.azi_gain.P = ivalues[0];
    CommandData.azi_gain.I = ivalues[1];
  } else if (command == MIndex("pivot_gain")) {  /* pivot gains */
    CommandData.pivot_gain.SP = (rvalues[0] + 2.605) / 7.9498291016e-5;
    CommandData.pivot_gain.P = ivalues[1];

    /***************************************/
    /********** Inner Frame Lock  **********/
  } else if (command == MIndex("lock")) {  /* Lock Inner Frame */
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    CommandData.pumps.lock_point = 1;
    CommandData.pointing_mode.el_mode = POINT_LOCK;
    CommandData.pointing_mode.el1 = LockPosition(rvalues[0]);
    fprintf(stderr, "Lock Mode: %g\n", CommandData.pointing_mode.el1);

    /***************************************/
    /********** Balance System  ************/
  } else if (command == MIndex("setpoints")) {
    CommandData.pumps.bal_on = rvalues[0] * 1648.;
    CommandData.pumps.bal_off = rvalues[1] * 1648.;
    CommandData.pumps.bal_target = rvalues[2] * 1648.;
  } else if (command == MIndex("bal_level")) {
    CommandData.pumps.pwm1 = 2047 - rvalues[0] * 2047. / 100;
  } else if (command == MIndex("bal_gain")) {
    CommandData.pumps.bal_gain = rvalues[0];
    CommandData.pumps.bal_min = 2047 - rvalues[0] * 2047. / 100;
    CommandData.pumps.bal_max = 2047 - rvalues[0] * 2047. / 100;

    /***************************************/
    /********** Cooling System  ************/
  } else if (command == MIndex("spare_pwm")) {
    CommandData.pumps.pwm2 = ivalues[0];
  } else if (command == MIndex("inner_pwm")) {
    CommandData.pumps.pwm3 = ivalues[0];
  } else if (command == MIndex("outer_pwm")) {
    CommandData.pumps.pwm4 = ivalues[0];

    /***************************************/
    /******** Electronics Heaters  *********/
  } else if (command == MIndex("t_gyrobox")) {  /* gyro heater setpoint */
    CommandData.t_gybox_setpoint = rvalues[0];
  } else if (command == MIndex("t_gyro_gain")) {  /* gyro heater gains */
    CommandData.gy_heat_gain.P = ivalues[0];
    CommandData.gy_heat_gain.I = ivalues[1];
    CommandData.gy_heat_gain.D = ivalues[2];

    /***************************************/
    /*************** Misc  *****************/
  } else if (command == MIndex("timeout")) {        /* Set timeout */
    CommandData.timeout = ivalues[0];
  } else if (command == MIndex("xml_file")) {  /* change downlink XML file */
    if ((fp = fopen("./alice/index.al", "w")) != NULL) {
      fprintf(fp, "%d\n", ivalues[0]);
      fclose(fp);
    }

    /***************************************/
    /*************** Bias  *****************/
  } else if (command == MIndex("bias1_level")) {    /* Set bias 1 */
    CommandData.Bias.SetLevel1 = 1;
    CommandData.Bias.bias1 = ivalues[0];
  } else if (command == MIndex("bias2_level")) {   /* Set bias 2 */
    CommandData.Bias.SetLevel2 = 1;
    CommandData.Bias.bias2 = ivalues[0];
  } else if (command == MIndex("bias3_level")) {   /* Set bias 3 */
    CommandData.Bias.SetLevel3 = 1;
    CommandData.Bias.bias3 = ivalues[0];
  } else if (command == MIndex("phase")) {
    if (ivalues[0] >= 5 && ivalues[0] <= 16) 
      CommandData.Phase[ivalues[0] - 5] = ivalues[1];

    /***************************************/
    /*********** Cal Lamp  *****************/
  } else if (command == MIndex("cal_pulse")) {
    CommandData.Cryo.calib_pulse = ivalues[0];
    CommandData.Cryo.calib_repeat = 0;
  } else if (command == MIndex("cal_repeat")) {
    CommandData.Cryo.calib_pulse = ivalues[0];
    CommandData.Cryo.calib_repeat = rvalues[1];

    /***************************************/
    /********* Cryo heat   *****************/
  } else if (command == MIndex("jfet_heat")) {
    CommandData.Cryo.JFETHeat = rvalues[0] * 2047./100.;
  } else if (command == MIndex("heatsw_heat")) {
    CommandData.Cryo.heatSwitch = rvalues[0] * 2047./100.;
  } else if (command == MIndex("he3_heat")) {
    CommandData.Cryo.heliumThree = rvalues[0] * 2047./100.;
  } else if (command == MIndex("spare_heat")) {
    CommandData.Cryo.sparePwm = rvalues[0] * 2047./100.;


    /***************************************/
    /********* ISC Commanding  *************/
  } else if (command == MIndex("set_focus")) {
    CommandData.ISCState.focus_pos = ivalues[0];
  } else if (command == MIndex("set_aperture")) {
    CommandData.ISCState.ap_pos = ivalues[0];
  } else if (command == MIndex("pixel_centre")) {
    CommandData.ISCState.roi_x = ivalues[0];
    CommandData.ISCState.roi_y = ivalues[1];
    CommandData.ISCState.display_mode = roi;
  } else if (command == MIndex("blob_centre")) {
    CommandData.ISCState.blob_num = ivalues[0];
    CommandData.ISCState.display_mode = blob;
  } else if (command == MIndex("bda_offsets")) {
    CommandData.ISCState.azBDA = rvalues[0] * DEG2RAD;
    CommandData.ISCState.elBDA = rvalues[1] * DEG2RAD;
  } else if (command == MIndex("bright_star")) {
    CommandData.ISCState.brightRA = rvalues[0] * DEG2RAD;
    CommandData.ISCState.brightDEC = rvalues[1] * DEG2RAD;
    CommandData.ISCState.brightStarMode = 1;
  } else if (command == MIndex("integration")) {
    CommandData.ISC_pulse_width = (int)(rvalues[0] / 10.);
  } else if (command == MIndex("det_set")) {
    CommandData.ISCState.grid = ivalues[0];
    CommandData.ISCState.sn_threshold = rvalues[1];
    CommandData.ISCState.cenbox = ivalues[2];
    CommandData.ISCState.apbox = ivalues[3];
    CommandData.ISCState.mult_dist = ivalues[4];
  } else if (command == MIndex("max_blobs")) {
    CommandData.ISCState.maxBlobMatch = ivalues[0];
  } else if (command == MIndex("catalogue")) {
    CommandData.ISCState.mag_limit = rvalues[0];
    CommandData.ISCState.norm_radius = rvalues[1] * DEG2RAD;
    CommandData.ISCState.lost_radius = rvalues[2] * DEG2RAD;
  } else if (command == MIndex("tolerances")) {
    CommandData.ISCState.tolerance = rvalues[0] / 3600. * DEG2RAD;
    CommandData.ISCState.match_tol = rvalues[1] / 100;
    CommandData.ISCState.quit_tol = rvalues[2] / 100;
    CommandData.ISCState.rot_tol = rvalues[3] * DEG2RAD;

  } else {
    return; // invalid command - don't update
  }

  i_point = GETREADINDEX(point_index);

  CommandData.pointing_mode.t_start_sched =
    PointingData[i_point].t + CommandData.timeout;

  ClearPointingModeExtraFields();

  WritePrevStatus();
}

void GPSPosition (unsigned char *indata) {
  /* Send new information to CommandData */

  SIPData.GPSpos.lon = -ParseGPS(indata);
  SIPData.GPSpos.lat = ParseGPS(indata + 4); // sip sends east lon
  SIPData.GPSpos.alt = ParseGPS(indata + 8);
  SIPData.GPSstatus1 = *(indata + 12);
  SIPData.GPSstatus2 = *(indata + 13);

  WritePrevStatus();
}


void GPSTime (unsigned char *indata) {
  float GPStime, offset;
  int CPUtime, GPSweek;

  /* Send new information to CommandData */

  GPStime = ParseGPS(indata);
  GPSweek = (unsigned short)(*(indata + 4));
  offset = ParseGPS(indata + 6);
  CPUtime = ParseGPS(indata + 10);

  SIPData.GPStime.UTC = (int)(SEC_IN_WEEK * (GPSweek+1024) + GPStime - offset) +
    SUN_JAN_6_1980;
  SIPData.GPStime.CPU = CPUtime;

  WritePrevStatus();
}

void MKSAltitude (unsigned char *indata) {
  float hi, med, lo;
  float z_hi, z_med, z_lo;

  /* Update CommandData */

  /* The pressure data are two (backwards) bytes long */
  hi = (*(indata + 1) << 8) + *indata;
  med = (*(indata + 3) << 8) + *(indata + 2);
  lo = (*(indata + 5) << 8) + *(indata + 4);

  /* Calculate pressure */
  z_hi = log(SIPData.MKScal.m_hi * hi + SIPData.MKScal.b_hi);  
  z_med = log(SIPData.MKScal.m_med * med +
      SIPData.MKScal.b_med);
  z_lo = log(SIPData.MKScal.m_lo * lo + SIPData.MKScal.b_lo);

  /* Use the MKS algorithm to calculate altitude (ft) */
  SIPData.MKSalt.hi = 156776.89 - 25410.089 * z_hi
    + 462.44626 * pow(z_hi, 2)
    + 130.61746 * pow(z_hi, 3)
    - 20.0116288 * pow(z_hi, 4);

  SIPData.MKSalt.med = 156776.89 - 25410.089 * z_med
    + 462.44626 * pow(z_med, 2)
    + 130.61746 * pow(z_med, 3)
    - 20.0116288 * pow(z_med, 4);

  SIPData.MKSalt.lo = 156776.89 - 25410.089 * z_lo
    + 462.44626 * pow(z_lo, 2)
    + 130.61746 * pow(z_lo, 3)
    - 20.0116288 * pow(z_lo, 4);

  WritePrevStatus();
}


// Send TDRSS Low Rate Packet

void SendDownData(char tty_fd) {
  unsigned char buffer[50], data[37];
  int i, temp;
  int bitpos, bytepos, numbits;
  static char firsttime = 1;

  bitpos = 0;
  bytepos = 0;
  memset(data, 0, 37);

  for (i = 0; i < N_SLOWDL; i++) {
    switch (SlowDLInfo[i].type) {
      case FORCEINT:
        // Round value to an integer and try to fit it in numbits
        if ((int)SlowDLInfo[i].value > (1 << (SlowDLInfo[i].numbits - 1)) - 1)
          temp = 0;     // Indicates value was too big
        else if ((int)SlowDLInfo[i].value < -1 * ((1 << 
                (SlowDLInfo[i].numbits - 1)) - 2))
          temp = 1;     // Indicates value was too small
        else
          temp = (int)SlowDLInfo[i].value + (1 << (SlowDLInfo[i].numbits - 1));
        numbits = SlowDLInfo[i].numbits;
        break;

      case U_MASK:
        // Simply take the bottom numbits from the unsigned number
        temp = ((int)(SlowDLInfo[i].value)) & ((1 << SlowDLInfo[i].numbits) - 
            1);
        numbits = SlowDLInfo[i].numbits;
        break;

      case TAKEBIT:
        // Intended for bitfields:  get the value of bit number numbit
        temp = (((int)(SlowDLInfo[i].value)) >> (SlowDLInfo[i].numbits - 1))
          & 0x01;
        numbits = 1;
        break;

      default:
        temp = 0;
        numbits = 1;
        break;
    }


    if (numbits - 1 > 7 - bitpos) {         /* Do we need to wrap? */
      data[bytepos++] |= (temp & ((1 << (8 - bitpos)) - 1)) << bitpos;
      temp = temp << (8 - bitpos);
      numbits -= 8 - bitpos;
      bitpos = 0;
    }

    while (temp > 0xFF) {          /* Is temp still larger than one byte? */
      data[bytepos++] |= temp & 0xFF;
      temp = temp >> 8;
      numbits -= 8;
    }

    data[bytepos] |= temp << bitpos;
    bitpos += numbits;
    if (bitpos > 7) {
      bitpos = 0;
      bytepos++;
    }
  }

  if (firsttime) {
    fprintf(stderr, "Slow DL size = %d\n", bytepos);
    firsttime = 0;
  }

  buffer[0] = 0x10;
  buffer[1] = 0x53;
  buffer[2] = 37;
  memcpy(buffer + 3, data, 37);
  buffer[40] = 0x03;

  write(tty_fd, buffer, 41);
}

/* compute the size of the data queue for the given command */
int DataQSize(int command) {
  int i, size = mcommands[command].numparams;

  for (i = 0; i < mcommands[command].numparams; ++i)
    if (mcommands[command].params[i].type == 'l')
      size++;

  return size;
}

void WatchFIFO () {
  unsigned char buf[1];
  char command[100];
  char pbuf[30];
  int fifo;

  int mcommand = -1;
  int mcommand_count = 0;
  char *mcommand_data[DATA_Q_SIZE];

  int index, pindex = 0;

  int pid = getpid();
  fprintf(stderr, ">> WatchFIFO startup on pid %i\n", pid);

  if ((fifo = open("/tmp/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1) {
    perror("Unable to open FIFO");
    exit(1);
  }

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    fprintf(stderr, "Command received: %s\n", command);
    index = -1;
    while((command[++index] != ' ') && command[index]);
    command[index++] = 0;

    pindex = 0;
    mcommand_count = 0;
    do {
      if ((command[index] == ' ' || command[index] == 0) && pindex > 0) {
        pbuf[pindex] = 0;
        if (NULL == (mcommand_data[mcommand_count] =
              realloc(mcommand_data[mcommand_count], pindex + 2))) {
          perror("malloc failed in FIFO CommandData");
          exit(1);
        }
        strncpy(mcommand_data[mcommand_count++], pbuf, pindex + 1);
        pindex = 0;
      } else {
        pbuf[pindex++] = command[index];
      }
    } while (command[index++] != 0);
    fprintf(stderr, "%i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SIndex(command);
      SingleCommand(mcommand);
      mcommand = -1;
    } else {
      mcommand = MIndex(command);
      fprintf(stderr, " Multi word command received\n");
      MultiCommand(mcommand, (unsigned short*) mcommand_data);
      mcommand = -1;
    }

    // Relinquish control of memory
    pthread_mutex_unlock(&mutex);

  }
}

char *COMM[] = {"/dev/ttyS0", "/dev/ttyS4"};

void WatchPort (void* parameter) {
  unsigned char buf;
  unsigned short *indatadumper;
  unsigned char indata[20];
  char readstage = 0;
  char tty_fd;
  
  int port = (int)parameter;

  int mcommand = -1;
  int mcommand_count = 0;
  int dataqsize = 0;
  unsigned short mcommand_data[DATA_Q_SIZE];
  unsigned char mcommand_time = 0;

  int timer = 0;
  int bytecount = 0;

  int pid = getpid();
  fprintf(stderr, ">> WatchPort(%i) startup on pid %i\n", port, pid);

  if((tty_fd = bc_setserial(COMM[port])) < 0) {
    exit(1);
  }

  for(;;) {
    /* Loop until data come in */
    while (read(tty_fd, &buf, 1) <= 0) {
      timer++;
      /** Request updated info every 5 seconds */
      if (timer == 250) { 
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_POSITION, tty_fd);
        SendRequest (REQ_TIME, tty_fd);
        SendRequest (REQ_ALTITUDE, tty_fd);
        pthread_mutex_unlock(&mutex);
        timer = 0;
      }
      usleep(10000); /* sleep for 10ms */
    }
    
    // Take control of memory
    pthread_mutex_lock(&mutex);

    /* Process data */
    switch (readstage) {
      /* readstage: 0: waiting for packet beginning (0x10) */
      /*            1: waiting for packet type (e.g., 0x14 = command packet) */
      /*            2: waiting for command packet datum: case 0x14 */
      /*            3: waiting for request data packet end: case 0x13 */
      /*            4: waiting for GPS position datum: case 0x10 */
      /*            5: waiting for GPS time datum:  case 0x11 */
      /*            6: waiting for MKS pressure datum: case 0x12 */

      case 0: /* waiting for packet beginning */
        if (buf == 0x10)
          readstage = 1;
        break;
      case 1: /* wating for packet type */
        if (buf == 0x13) { /* Send data request */
          readstage = 3;
        } else if (buf == 0x14) { /* Command */
          readstage = 2;
        } else if (buf == 0x10) { /* GPS Position */
          readstage = 4;
        } else if (buf == 0x11) { /* GPS Position */
          readstage = 5;
        } else if (buf == 0x12) { /* MKS Pressure */
          readstage = 6;
        } else {
          fprintf(stderr,
              "COMM%i: Bad packet received: Unrecognised Packet Type: %02X\n",
              port + 1, buf);
          readstage = 0;
        }
        break;
      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf == 0x02)
            bytecount = 1;
          else {
            readstage = 0;
            fprintf(stderr,
                "COMM%i: Bad command packet: Improper Encoding: %02X\n",
                port + 1, buf);
          }
        } else if (bytecount >= 1 && bytecount <= 2) {
          /* Read the two data bytes of the command packet */
          indata[bytecount - 1] = buf;
          bytecount++;
        } else {
          bytecount = 0;
          if (buf == 0x03) {
            /* We should now be at the end of the command packet */
            readstage = 0;

            /* Check bits 6-8 from second data byte for type of command */
            /* Recall:    101 = 0x05 = single command */
            /*            100 = 0x04 = begin multi command */
            /*            110 = 0x06 = end multi command */
            /*            0?? = 0x00 = data packet in multi command */


            if (((indata[1] >> 5) & 0x07) == 0x05) {
              /*** Single command ***/
              fprintf(stderr, "COMM%i:  Single command received\n", port + 1);
              SingleCommand(indata[0]);
              mcommand = -1;
            } else if (((indata[1] >> 5) & 0x07) == 0x04) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(mcommand);
              fprintf(stderr, "COMM%i:  Multi word command %d (%s) started\n",
                  port + 1, mcommand, mcommands[mcommand].name);

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;  
            } else if ((((indata[1] >> 7) & 0x01) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              fprintf(stderr, "COMM%i:  Multi word command continues...\n",
                  port + 1);
              mcommand_count++;
            } else if ((((indata[1] >> 5) & 0x07) == 0x06) &&
                (mcommand == indata[0]) && 
                ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              fprintf(stderr, "COMM%i:  Multi word command ends \n", port + 1);
              MultiCommand(mcommand, (unsigned short *) mcommand_data);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              fprintf(stderr,
                  "COMM%i: Command packet discarded: Bad Encoding: %02X\n",
                  port + 1 , buf);
              mcommand_time = 0;
            }
          }
        }
        break;
      case 3: /* waiting for request data packet end */
        readstage = 0;
        if (buf == 0x03) {
          SendDownData(tty_fd);
        } else {
          fprintf(stderr, "COMM%i: Bad encoding: Bad packet terminator: %02X\n",
              port + 1, buf);
        }
        break;
      case 4: /* waiting for GPS position datum */
        if (bytecount < 14) {  /* There are 14 data bytes for GPS position */
          indata[bytecount] = buf;
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf == 0x03) {
            GPSPosition((unsigned char *) indata);
          } else {
            fprintf(stderr, "COMM%i: Bad encoding in GPS Position: "
                "Bad packet terminator: %02X\n", port + 1, buf);
          }
        }
        break;
      case 5: /* waiting for GPS time datum:  case 0x11 */
        if (bytecount < 14) {  /* There are fourteen data bytes for GPS time */
          indata[bytecount] = buf;
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf == 0x03) {
            GPSTime((unsigned char *) indata);
          } else {
            fprintf(stderr, "COMM%i: Bad encoding in GPS Time: "
                "Bad packet terminator: %02X\n", port + 1, buf);
          }
        }
        break;
      case 6: /* waiting for MKS pressure datum: case 0x12 */
        if (bytecount < 6) {
          indata[bytecount] = buf;
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf == 0x03) {
            MKSAltitude((unsigned char *) indata);
          } else {
            fprintf(stderr, "COMM%i: Bad encoding in MKS Altitude: "
                "Bad packet terminator: %02X\n", port + 1, buf);
          }
        }
    }

    // Relinquish control of memory
    pthread_mutex_unlock(&mutex);
  }
}

/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there is   */
/*   no previous state file, set to default                 */
/*                                                          */
/************************************************************/
void InitCommandData() {
  int fp, n_read = 0, junk, extra = 0;

  if ((fp = open("/tmp/mcp.prev_status", O_RDONLY)) < 0) {
    perror("Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      perror("prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      perror("extra prev_status read()");
    if (close(fp) < 0)
      perror("prev_status close()");
  }

  CommandData.pointing_mode.t_start_sched = time(NULL) + CommandData.timeout;

  /** initialize stuff that we don't want from prev_status here **/
  CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
  CommandData.pumps.bal1_on = 0;
  CommandData.pumps.bal1_reverse = 0;
  CommandData.pumps.bal2_on = 0;
  CommandData.pumps.bal2_reverse = 0;

  CommandData.pumps.inframe_cool1_on = 0;
  CommandData.pumps.inframe_cool1_off = 0;
  CommandData.pumps.lock_out = 0;
  CommandData.pumps.lock_in = 0;
  CommandData.pumps.lock_point = 0;
  CommandData.pumps.outframe_cool1_on = 0;
  CommandData.pumps.outframe_cool1_off = 0;
  CommandData.pumps.outframe_cool2_on = 0;
  CommandData.pumps.outframe_cool2_off = 0;
  CommandData.pumps.pwm1 = 0;
  CommandData.pumps.pwm2 = 0;
  CommandData.pumps.pwm3 = 0;
  CommandData.pumps.pwm4 = 0;

  CommandData.Bias.SetLevel1 = 1;
  CommandData.Bias.SetLevel2 = 1;
  CommandData.Bias.SetLevel3 = 1;

#ifndef BOLOTEST
  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    fprintf(stderr, "prev_status: Wanted %i bytes but got %i.\n",
        sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    fprintf(stderr, "prev_status: Extra bytes found.\n");
  else
    return;
#endif

  fprintf(stderr,"Warning: regenerating Command Data and prev_status\n");

  CommandData.pointing_mode.t_start_sched = time(NULL) + CommandData.timeout;

  /** put stuff that we want to keep from prev_status here **/
  CommandData.pointing_mode.az_mode = POINT_VEL;
  CommandData.pointing_mode.az_vel = 0.0;
  CommandData.pointing_mode.el_mode = POINT_VEL;
  CommandData.pointing_mode.el_vel = 0.0;

  CommandData.timeout = 60*60;

  CommandData.roll_gain.P = 30000;

  CommandData.ele_gain.I = 8000;
  CommandData.ele_gain.P = 1200;

  CommandData.azi_gain.P = 20000;
  CommandData.azi_gain.I = 5000; 

  CommandData.pivot_gain.SP = 2000;
  CommandData.pivot_gain.P = 200;
  CommandData.pivot_gain.D = 18;  //unused

  CommandData.t_gybox_setpoint = 30.0;
  CommandData.gy_heat_gain.P = 10;
  CommandData.gy_heat_gain.I = 60;
  CommandData.gy_heat_gain.D = 50;

  CommandData.disable_az = 0;
  CommandData.disable_el = 0;

  CommandData.use_elenc = 1;
  CommandData.use_elclin = 1;
  CommandData.use_sun = 1;
  CommandData.use_isc = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 1;

  SIPData.MKScal.m_hi = 0.01;
  SIPData.MKScal.m_med = 0.1;
  SIPData.MKScal.m_lo = 1;
  SIPData.MKScal.b_hi = 0;
  SIPData.MKScal.b_med = 0;
  SIPData.MKScal.b_lo = 0;

  SIPData.MKScal.m_hi = 0.01;
  SIPData.MKScal.m_med = 0.1;
  SIPData.MKScal.m_lo = 1;
  SIPData.MKScal.b_hi = 0;
  SIPData.MKScal.b_med = 0;
  SIPData.MKScal.b_lo = 0;

  CommandData.pointing_mode.az_mode = AXIS_VEL;
  CommandData.pointing_mode.el_mode = AXIS_VEL;
  CommandData.pointing_mode.az_vel = 0.0;
  CommandData.pointing_mode.el_vel = 0.0;

  CommandData.pumps.bal_on = 0.5 * 1648.;
  CommandData.pumps.bal_off = 0.2 * 1648.;
  CommandData.pumps.bal_target = 0.0 * 1648.;
  CommandData.pumps.bal_gain = 0.2;
  CommandData.pumps.bal_max = 600;  /* 70% */
  CommandData.pumps.bal_min = 1750; /* 15% */

  CommandData.Bias.clockInternal = 1;
  CommandData.Bias.biasAC = 1;
  CommandData.Bias.biasRamp = 0;

  CommandData.Bias.SetLevel1 = 1;
  CommandData.Bias.SetLevel2 = 1;
  CommandData.Bias.SetLevel3 = 1;
  CommandData.Bias.bias1 = 0x04;
  CommandData.Bias.bias2 = 0x08;
  CommandData.Bias.bias3 = 0x0f;

  CommandData.Cryo.heliumLevel = 0;
  CommandData.Cryo.charcoalHeater = 0;
  CommandData.Cryo.coldPlate = 0;
  CommandData.Cryo.JFETHeat = 0;
  CommandData.Cryo.heatSwitch = 0;
  CommandData.Cryo.heliumThree = 0;
  CommandData.Cryo.sparePwm = 0;
  CommandData.Cryo.calibrator = 0;
  CommandData.Cryo.lnvalve_on = 0;
  CommandData.Cryo.lnvalve_open = 0;
  CommandData.Cryo.lnvalve_close = 0;
  CommandData.Cryo.lhevalve_on = 0;
  CommandData.Cryo.lhevalve_open = 0;
  CommandData.Cryo.lhevalve_close = 0;

  CommandData.ISCState.abort = 0;
  CommandData.ISCState.pause = 0;
  CommandData.ISCState.save = 0;
  CommandData.ISCState.autofocus = 0;
  CommandData.ISCState.focus_pos = 2300;
  CommandData.ISCState.ap_pos = 495;
  CommandData.ISCState.display_mode = full;
  CommandData.ISCState.azBDA = 0;
  CommandData.ISCState.elBDA = 0;
  CommandData.ISCState.brightStarMode = 0;
  CommandData.ISCState.sn_threshold = 20;
  CommandData.ISCState.grid = 38;
  CommandData.ISCState.cenbox = 20;
  CommandData.ISCState.apbox = 5;
  CommandData.ISCState.maxBlobMatch = 5;
  CommandData.ISCState.mult_dist = 30;
  CommandData.ISCState.mag_limit = 8.5;
  CommandData.ISCState.norm_radius = 3. * DEG2RAD;
  CommandData.ISCState.lost_radius = 10. * DEG2RAD;
  CommandData.ISCState.tolerance = 35. / 3600. * DEG2RAD; /* 35 arcsec */
  CommandData.ISCState.match_tol = 0.5;
  CommandData.ISCState.quit_tol = 0.8;
  CommandData.ISCState.rot_tol = 5 * DEG2RAD;
  CommandData.ISC_pulse_width = 3;

  WritePrevStatus();
}
