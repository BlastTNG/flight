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
#define COMM1 "/dev/ttyS0"
#define COMM2 "/dev/ttyS4"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

#define BAL_VETO_LENGTH 500

#define ASCENT_MODE     0
#define RASTER_MODE     1
#define ALIGN_MODE      2
#define POINT_MODE      3

#define SUN             0
#define ISC             1
#define VSC             2
#define MAG             3

#define TAKEBIT    0
#define FORCEINT   1
#define U_MASK     2

#define LOCK_OFFSET (-0.22)

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

char *SatStatus[14] = {
  "3 sat, 2D",
  "4 sat, 3D",
  "doing position fixes",
  "do not have GPS time",
  "waiting for almanac collection",
  "PDOP is too high",
  "0 satellites",
  "1 satellite",
  "2 satellites",
  "3 satellites",
  "0 useable satellites",
  "1 useable satellite",
  "2 useable satellites",
  "3 useable satellites"  
};

char *ModeNames[4] = {
  "Ascent",
  "Raster",
  "Sensor Alignment",
  "Point"
};

char *SensorNames[4] = {
  "Sun",
  "Integrating Star Cam.",
  "Video Star Cam.",
  "Magnetometer"
};

char *SensorUse[2] = {
  "Vetoed",
  "Available"
};

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus() {
  int fp;
  
  /** write the default file */
  fp = open("/tmp/mcp.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp<0) {
    fprintf(stderr, "warning: could not open prev_status file\n");
    return;
  }

  write(fp, &CommandData, sizeof(struct CommandDataStruct));
  close(fp);
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
    perror("error setting serial output speed");
    return -1;
  }
  if(cfsetispeed(&term, B1200)) {          /*  <======= SET THE SPEED HERE */
    perror("error setting serial input speed");
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

void SingleCommand (int command) {

  printf("Single command %d: %s\n", command, scommands[command].name);
  
  /* Update CommandData structure with new info */
  if (command == SIndex("asc_mod"))                  /* Switch modes */
    CommandData.current_mode = ASCENT_MODE;
  else if (command == SIndex("ras_mod"))
    CommandData.current_mode = RASTER_MODE;
  else if (command == SIndex("sal_mod"))
    CommandData.current_mode = POINT_MODE;
  else if (command == SIndex("def_mod"))
    CommandData.default_mode = CommandData.current_mode;

  else if (command == SIndex("all_stp")) {      /* Pointing aborts */
    CommandData.point_mode.az_mode = POINT_VEL;
    CommandData.point_mode.el_mode = POINT_VEL;
    CommandData.point_mode.az_vel = 0.0;
    CommandData.point_mode.el_vel = 0.0;
  } else if (command == SIndex("sns_def"))     /* Set default sensor */
    CommandData.default_sensor = SUN;
  else if (command == SIndex("isc_def"))
    CommandData.default_sensor = ISC;
  else if (command == SIndex("vsc_def"))
    CommandData.default_sensor = VSC;
  else if (command == SIndex("mag_def"))
    CommandData.default_sensor = MAG;

  else if (command == SIndex("sns_vet"))       /* Veto sensors */
    CommandData.use_sun = 0;
  else if (command == SIndex("isc_vet"))
    CommandData.use_isc = 0;
  else if (command == SIndex("vsc_vet"))
    CommandData.use_vsc = 0;
  else if (command == SIndex("mag_vet"))
    CommandData.use_mag = 0;

  else if (command == SIndex("sns_uvt"))       /* Un-veto sensors */
    CommandData.use_sun = 1;
  else if (command == SIndex("isc_uvt"))
    CommandData.use_isc = 1;
  else if (command == SIndex("vsc_uvt"))
    CommandData.use_vsc = 1;
  else if (command == SIndex("mag_uvt"))
    CommandData.use_mag = 1;

  else if (command == SIndex("b_clint"))    /* Bias settings */
    CommandData.Bias.clockInternal = 1;
  else if (command == SIndex("b_clext"))
    CommandData.Bias.clockInternal = 0;
  else if (command == SIndex("bias_ac"))
    CommandData.Bias.biasAC = 1;
  else if (command == SIndex("bias_dc"))
    CommandData.Bias.biasAC = 0;
  else if (command == SIndex("b_extrn"))
    CommandData.Bias.biasRamp = 1;
  else if (command == SIndex("b_intrn"))
    CommandData.Bias.biasRamp = 0;

  else if (command == SIndex("he_lv_1"))    /* Cryo commanding */
    CommandData.Cryo.heliumLevel = 1;
  else if (command == SIndex("he_lv_0"))
    CommandData.Cryo.heliumLevel = 0;
  else if (command == SIndex("charc_1"))
    CommandData.Cryo.charcoalHeater = 1;
  else if (command == SIndex("charc_0"))
    CommandData.Cryo.charcoalHeater = 0;
  else if (command == SIndex("cplat_1"))
    CommandData.Cryo.coldPlate = 1;
  else if (command == SIndex("cplat_0"))
    CommandData.Cryo.coldPlate = 0;
  else if (command == SIndex("calib_1"))
    CommandData.Cryo.calibrator = 1;
  else if (command == SIndex("calib_0"))
    CommandData.Cryo.calibrator = 0;
  else if (command == SIndex("lnv_opn"))
    CommandData.Cryo.lndir = 1;
  else if (command == SIndex("lnv_cls"))
    CommandData.Cryo.lndir = 0;
  else if (command == SIndex("ln_val1"))
    CommandData.Cryo.lnvalve = 1;
  else if (command == SIndex("ln_val0"))
    CommandData.Cryo.lnvalve = 0;
  else if (command == SIndex("lhe_opn"))
    CommandData.Cryo.lhedir = 1;
  else if (command == SIndex("lhe_cls"))
    CommandData.Cryo.lhedir = 0;
  else if (command == SIndex("lhe_vl1"))
    CommandData.Cryo.lhevalve = 1;
  else if (command == SIndex("lhe_vl0"))
    CommandData.Cryo.lhevalve = 0;

  else if (command == SIndex("bal_vet"))
    CommandData.pumps.bal_veto = -1;
  else if (command == SIndex("bal_uvt"))
    CommandData.pumps.bal_veto = 0;

  else if (command == SIndex("bl_p1_1"))
    CommandData.pumps.bal1_on = 1;
  else if (command == SIndex("bl_p1_0"))
    CommandData.pumps.bal1_on = 0;
  else if (command == SIndex("bl_p1_f"))
    CommandData.pumps.bal1_reverse = 0;
  else if (command == SIndex("bl_p1_r"))
    CommandData.pumps.bal1_reverse = 1;
  else if (command == SIndex("bl_p2_1"))
    CommandData.pumps.bal2_on = 1;
  else if (command == SIndex("bl_p2_0"))
    CommandData.pumps.bal2_on = 0;
  else if (command == SIndex("bl_p2_f"))
    CommandData.pumps.bal2_reverse = 0;
  else if (command == SIndex("bl_p2_r"))
    CommandData.pumps.bal2_reverse = 1;

  else if (command == SIndex("if_p1_1"))
    CommandData.pumps.inframe_cool1_on = 10;
  else if (command == SIndex("if_p1_0"))
    CommandData.pumps.inframe_cool1_off = 10;

  else if (command == SIndex("of_p1_1"))
    CommandData.pumps.outframe_cool1_on = 10;
  else if (command == SIndex("of_p1_0"))
    CommandData.pumps.outframe_cool1_off = 10;
  else if (command == SIndex("of_p2_1"))
    CommandData.pumps.outframe_cool2_on = 10;
  else if (command == SIndex("of_p2_0"))
    CommandData.pumps.outframe_cool2_off = 10;
  else if (command == SIndex("lkmot_l"))
    CommandData.pumps.lock_in = 1;
  else if (command == SIndex("lkmot_u")) {
    CommandData.pumps.lock_out = 1;
    if (CommandData.point_mode.el_mode == POINT_LOCK) {
      CommandData.point_mode.el_mode = POINT_VEL;
      CommandData.point_mode.el_vel = 0.0;
    }
  }
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
  double min;
  double rvalues[MAX_N_PARAMS];
  unsigned short ivalues[MAX_N_PARAMS];
  char type;

#ifndef BOLOTEST
  /* compute renormalised values */
  for (i = dataqind = 0; i < mcommands[command].numparams; ++i) {
    min = mcommands[command].params[i].min;
    type = mcommands[command].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++];
      printf("param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = (float)dataq[dataqind++] * (mcommands[command].params[i].max
          - min) / MAX_15BIT + min;
      printf("param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = (float)((int)dataq[dataqind++] << 15); /* upper 15 bits */
      rvalues[i] += (float)dataq[dataqind++];             /* lower 15 bits */
      rvalues[i] = rvalues[i] * (mcommands[command].params[i].max - min) /
        MAX_30BIT + min;
      printf("param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }
#else
  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  for (i = dataqind = 0; i < mcommands[command].numparams; ++i) {
    type = mcommands[command].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      printf("param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      printf("param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      printf("param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }
#endif

  /* Update CommandData struct with new info
   * If the parameter is type 'i'     set CommandData using ivalues[i]
   * If the parameter is type 'f'/'l' set CommandData using rvalues[i]
   */
  if (command == MIndex("t_sched"))         /* Set timeout */
    CommandData.timeout = ivalues[0];
  else if (command == MIndex("gyrob_t"))  /* gyro heater setpoint */
    CommandData.t_gybox_setpoint = rvalues[0];
  else if (command == MIndex("iscbx_t"))  /* isc heater setpoint */
    CommandData.t_isc_setpoint = rvalues[0];
  else if (command == MIndex("xml_fil")) {  /* change downlink XML file */
    if ((fp = fopen("./alice/index.al", "w")) != NULL) {
      fprintf(fp, "%d\n", ivalues[0]);
      fclose(fp);
    }
  } else if (command == MIndex("gn_roll"))  /* roll Gains */
    CommandData.roll_gain.P = ivalues[0];
  else if (command == MIndex("gn_elev")) {  /* ele gains */
    CommandData.ele_gain.P = ivalues[0];
    CommandData.ele_gain.I = ivalues[1];
  } else if (command == MIndex("gn_azim")) {  /* az gains */
    CommandData.azi_gain.P = ivalues[0];
    CommandData.azi_gain.I = ivalues[1];
  } else if (command == MIndex("gn_pivo")) {  /* pivot gains */
    CommandData.pivot_gain.SP = (rvalues[0] + 2.605) / 7.9498291016e-5;
    CommandData.pivot_gain.P = ivalues[1];
  } else if (command == MIndex("gn_gyht")) {  /* gyro heater gains */
    CommandData.gy_heat_gain.P = ivalues[0];
    CommandData.gy_heat_gain.I = ivalues[1];
    CommandData.gy_heat_gain.D = ivalues[2];
  } else if (command == MIndex("gn_isc")) {  /* isc heater gains */
    CommandData.isc_heat_gain.P = ivalues[0];
    CommandData.isc_heat_gain.I = ivalues[1];
    CommandData.isc_heat_gain.D = ivalues[2];
  } else if (command == MIndex("lock_el")) {  /* Lock Inner Frame */
    CommandData.point_mode.el_mode = POINT_LOCK;
    CommandData.pumps.lock_point = 1;
    CommandData.point_mode.el_dest = LockPosition(rvalues[0]);
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    fprintf(stderr, "Lock Mode: %g\n", CommandData.point_mode.el_dest);
  } else if (command == MIndex("goto_el")) {  /* point in elevation */
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    CommandData.point_mode.el_mode = POINT_POSITION;
    CommandData.point_mode.el_dest = rvalues[0];
  } else if (command == MIndex("goto_az")) {  /* point in azimuth */
    CommandData.point_mode.az_mode = POINT_POSITION;
    CommandData.point_mode.az_dest = rvalues[0];
  } else if (command == MIndex("el_vel")) {  /* fixed elevation velocity */
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    CommandData.point_mode.el_mode = POINT_VEL;
    CommandData.point_mode.el_vel = rvalues[0];
  } else if (command == MIndex("az_vel")) {  /* fixed azimuth velocity */
    CommandData.point_mode.az_mode = POINT_VEL;
    CommandData.point_mode.az_vel = rvalues[0];
  } else if (command == MIndex("jfet_ht"))
    CommandData.Cryo.JFETHeat = 2047 - ivalues[0] * 20.47;
  else if (command == MIndex("hs_heat"))
    CommandData.Cryo.heatSwitch = 2047 - ivalues[0] * 20.47;
  else if (command == MIndex("he3_ht"))
    CommandData.Cryo.heliumThree = 2047 - ivalues[0] * 20.47;
  else if (command == MIndex("cryopwm"))
    CommandData.Cryo.sparePwm = 2047 - ivalues[0] * 20.47;
  else if (command == MIndex("b_levl1"))    /* Set bias 1 */
    CommandData.Bias.bias1 = rvalues[0];
  else if (command == MIndex("b_levl2"))    /* Set bias 1 */
    CommandData.Bias.bias2 = rvalues[0];
  else if (command == MIndex("b_levl3"))    /* Set bias 1 */
    CommandData.Bias.bias3 = rvalues[0];
  else if (command == MIndex("phase"))
    CommandData.Phase[ivalues[0]] = ivalues[1];
  else if (command == MIndex("balgoal")) {
    CommandData.pumps.bal_on = rvalues[0] * 1648.;
    CommandData.pumps.bal_off = rvalues[1] * 1648.;
    CommandData.pumps.bal_target = rvalues[2] * 1648.;
  } else if (command == MIndex("balpwm"))
    CommandData.pumps.pwm1 = ivalues[0];
  else if (command == MIndex("sprpwm"))
    CommandData.pumps.pwm2 = ivalues[0];
  else if (command == MIndex("inpwm"))
    CommandData.pumps.pwm3 = ivalues[0];
  else if (command == MIndex("outpwm"))
    CommandData.pumps.pwm4 = ivalues[0];

  WritePrevStatus();
}

void GPSPosition (unsigned char *indata, int commnum) {
  /* Send new information to CommandData */

  SIPData.GPSpos[commnum].lat = ParseGPS(indata);
  SIPData.GPSpos[commnum].lon = ParseGPS(indata + 4);
  SIPData.GPSpos[commnum].alt = ParseGPS(indata + 8);
  SIPData.GPSstatus1[commnum] = *(indata + 12);
  SIPData.GPSstatus2[commnum] = *(indata + 13);

  WritePrevStatus();
}


void GPSTime (unsigned char *indata, int commnum) {
  float GPStime, offset;
  int CPUtime, GPSweek;

  /* Send new information to CommandData */

  GPStime = ParseGPS(indata);
  GPSweek = (unsigned short)(*(indata + 4));
  offset = ParseGPS(indata + 6);
  CPUtime = ParseGPS(indata + 10);

  SIPData.GPStime[commnum].UTC = (int)(604800 * GPSweek + GPStime - offset);
  SIPData.GPStime[commnum].CPU = CPUtime;

  WritePrevStatus();
}

void MKSAltitude (unsigned char *indata, int commnum) {
  float hi, med, lo;
  float z_hi, z_med, z_lo;

  /* Update CommandData */

  /* The pressure data are two (backwards) bytes long */
  hi = (*(indata + 1) << 8) + *indata;
  med = (*(indata + 3) << 8) + *(indata + 2);
  lo = (*(indata + 5) << 8) + *(indata + 4);

  /* Calculate pressure */
  z_hi = log(SIPData.MKScal[commnum].m_hi * hi + SIPData.MKScal[commnum].b_hi);  
  z_med = log(SIPData.MKScal[commnum].m_med * med +
      SIPData.MKScal[commnum].b_med);
  z_lo = log(SIPData.MKScal[commnum].m_lo * lo + SIPData.MKScal[commnum].b_lo);

  /* Use the MKS algorithm to calculate altitude (ft) */
  SIPData.MKSalt[commnum].hi = 156776.89 - 25410.089 * z_hi
    + 462.44626 * pow(z_hi, 2)
    + 130.61746 * pow(z_hi, 3)
    - 20.0116288 * pow(z_hi, 4);

  SIPData.MKSalt[commnum].med = 156776.89 - 25410.089 * z_med
    + 462.44626 * pow(z_med, 2)
    + 130.61746 * pow(z_med, 3)
    - 20.0116288 * pow(z_med, 4);

  SIPData.MKSalt[commnum].lo = 156776.89 - 25410.089 * z_lo
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
    printf("Slow DL size = %d\n", bytepos);
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
        if (NULL == (mcommand_data[mcommand_count] = realloc(mcommand_data[mcommand_count], pindex + 2))) {
          perror("malloc failed in FIFO CommandData.\n");
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
      printf(" Multi word command received\n");
      MultiCommand(mcommand, (unsigned short*) mcommand_data);
      mcommand = -1;
    }

    // Relinquish control of memory
    pthread_mutex_unlock(&mutex);

  }
}

void WatchPortC1 () {
  unsigned char buf[1];
  unsigned short *indatadumper;
  unsigned char indata[20];
  char readstage = 0;
  char tty_fd;

  int mcommand = -1;
  int mcommand_count = 0;
  int dataqsize = 0;
  unsigned short mcommand_data[DATA_Q_SIZE];
  unsigned char mcommand_time = 0;

  int timer = 0;
  int bytecount = 0;


  if((tty_fd = bc_setserial(COMM1)) < 0) {
    perror("Unable to open serial port");
    exit(1);
  }

  printf("Watching Port C1\n");
  do {
    /* Loop until data come in */
    while (read(tty_fd, buf, 1) <= 0) {
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
        if (buf[0] == 0x10)
          readstage = 1;
        break;


      case 1: /* wating for packet type */
        if (buf[0] == 0x13) /* Send data request */
          readstage = 3;
        else if (buf[0] == 0x14) /* Command */
          readstage = 2;
        else if (buf[0] == 0x10) /* GPS Position */
          readstage = 4;
        else if (buf[0] == 0x11) /* GPS Position */
          readstage = 5;
        else if (buf[0] == 0x12) /* MKS Pressure */
          readstage = 6;
        break;


      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf[0] == 0x02)
            bytecount = 1;
          else {
            readstage = 0;
            fprintf(stderr, "\nBad packet terminated (improper encoding).\n");
          }
        } else if (bytecount >= 1 && bytecount <= 2) {
          /* Read the two data bytes of the command packet */
          indata[bytecount - 1] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          if (buf[0] == 0x03) {
            /* We should now be at the end of the command packet */
            readstage = 0;

            /* Check bits 6-8 from second data byte for type of command */
            /* Recall:    101 = 0x05 = single command */
            /*            100 = 0x04 = begin multi command */
            /*            110 = 0x06 = end multi command */
            /*            0?? = 0x00 = data packet in multi command */


            if (((indata[1] >> 5) & 0x07) == 0x05) {
              /*** Single command ***/
              printf(" Single command received\n");
              SingleCommand(indata[0]);
              mcommand = -1;
            } else if (((indata[1] >> 5) & 0x07) == 0x04) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(mcommand);
              printf(" Multi word command %d started\n", mcommand);

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;  
            } else if ((((indata[1] >> 7) & 0x01) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              printf(" Multi word command continues...\n");
              mcommand_count++;
            } else if ((((indata[1] >> 5) & 0x07) == 0x06) &&
                (mcommand == indata[0]) && 
                ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              printf(" Multi word command ends \n");
              MultiCommand(mcommand, (unsigned short *) mcommand_data);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              printf(" Command packet discarded (bad encoding).\n");
              mcommand_time = 0;
            }
          }
        }
        break;

      case 3: /* waiting for request data packet end */
        readstage = 0;
        if (buf[0] == 0x03) {
          //printf(" *1 \n");
          SendDownData(tty_fd);
        } else {
          fprintf(stderr, "\nBad packet terminated (improper closing byte).\n");
        }
        break;

      case 4: /* waiting for GPS position datum */
        if (bytecount < 14) {  /* There are 14 data bytes for GPS position */
          indata[bytecount] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf[0] == 0x03) {
            GPSPosition((unsigned char *) indata, 0);
            //printf(" @1 \n");
          } else {
            printf(" error in SIP GPS reading \n");
          }
        }
        break;


      case 5: /* waiting for GPS time datum:  case 0x11 */
        if (bytecount < 14) {  /* There are fourteen data bytes for GPS time */
          indata[bytecount] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf[0] == 0x03) {
            GPSTime((unsigned char *) indata, 0);
            //printf(" @ \n");
          } else {
            printf(" error in SIP TIME reading \n");
          }
        }
        break;

      case 6: /* waiting for MKS pressure datum: case 0x12 */
        if (bytecount < 6) {
          indata[bytecount] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf[0] == 0x03) {
            MKSAltitude((unsigned char *) indata, 0);
            //printf(" @ \n");
          } else {
            printf(" error in MKS altitude reading\n");
          }
        }
    }

    // Relinquish control of memory
    pthread_mutex_unlock(&mutex);

  } while (1 == 1);
}

void WatchPortC2 () {
  unsigned char buf[1];
  unsigned short *indatadumper;
  unsigned char indata[20];
  char readstage = 0;
  char tty_fd;

  int mcommand = -1;
  int mcommand_count = 0;
  int dataqsize = 0;
  unsigned short mcommand_data[DATA_Q_SIZE];
  unsigned char mcommand_time = 0;

  int timer = 0;
  int bytecount = 0;


  if((tty_fd = bc_setserial(COMM2)) < 0) {
    perror("Unable to open serial port");
    exit(1);
  }

  do {
    /* Loop until data come in */
    while (read(tty_fd, buf, 1) <= 0) {
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
        if (buf[0] == 0x10)
          readstage = 1;
        break;


      case 1: /* wating for packet type */
        if (buf[0] == 0x13) /* Send data request */
          readstage = 3;
        else if (buf[0] == 0x14) /* Command */
          readstage = 2;
        else if (buf[0] == 0x10) /* GPS Position */
          readstage = 4;
        else if (buf[0] == 0x11) /* GPS Position */
          readstage = 5;
        else if (buf[0] == 0x12) /* MKS Pressure */
          readstage = 6;
        else
          printf("Bad packet received (unrecognised header).\n");
        break;


      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf[0] == 0x02)
            bytecount = 1;
          else {
            readstage = 0;
            fprintf(stderr, "\nBad packet terminated (improper encoding).\n");
          }
        } else if (bytecount >= 1 && bytecount <= 2) {
          /* Read the two data bytes of the command packet */
          indata[bytecount - 1] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          if (buf[0] == 0x03) {
            /* We should now be at the end of the command packet */
            readstage = 0;

            /* Check bits 6-8 from second data byte for type of command */
            /* Recall:    101 = 0x05 = single command */
            /*            100 = 0x04 = begin multi command */
            /*            110 = 0x06 = end multi command */
            /*            0?? = 0x00 = data packet in multi command */


            if (((indata[1] >> 5) & 0x07) == 0x05) {
              /*** Single command ***/
              printf(" Single command received\n");
              SingleCommand(indata[0]);
              mcommand = -1;
            } else if (((indata[1] >> 5) & 0x07) == 0x04) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(mcommand);
              printf(" Multi word command %d started\n", mcommand);

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;  
            } else if ((((indata[1] >> 7) & 0x01) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              printf(" Multi word command continues...\n");
              mcommand_count++;
            } else if ((((indata[1] >> 5) & 0x07) == 0x06) &&
                (mcommand == indata[0]) && 
                ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              printf(" Multi word command ends \n");
              MultiCommand(mcommand, (unsigned short *) mcommand_data);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              printf(" Command packet discarded (bad encoding).\n");
              mcommand_time = 0;
            }
          }
        }
        break;

      case 3: /* waiting for request data packet end */
        readstage = 0;
        if (buf[0] == 0x03) {
          //printf(" *1 \n");
          SendDownData(tty_fd);
        } else {
          fprintf(stderr, "\nBad packet terminated (improper closing byte).\n");
        }
        break;

      case 4: /* waiting for GPS position datum */
        if (bytecount < 14) {  /* There are 14 data bytes for GPS position */
          indata[bytecount] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf[0] == 0x03) {
            GPSPosition((unsigned char *) indata, 0);
            //printf(" @1 \n");
          } else {
            printf(" error in SIP GPS reading \n");
          }
        }
        break;


      case 5: /* waiting for GPS time datum:  case 0x11 */
        if (bytecount < 14) {  /* There are fourteen data bytes for GPS time */
          indata[bytecount] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf[0] == 0x03) {
            GPSTime((unsigned char *) indata, 0);
            //printf(" @ \n");
          } else {
            printf(" error in SIP TIME reading \n");
          }
        }
        break;

      case 6: /* waiting for MKS pressure datum: case 0x12 */
        if (bytecount < 6) {
          indata[bytecount] = buf[0];
          bytecount++;
        } else {
          bytecount = 0;
          readstage = 0;
          if (buf[0] == 0x03) {
            MKSAltitude((unsigned char *) indata, 0);
            //printf(" @ \n");
          } else {
            printf(" error in MKS altitude reading\n");
          }
        }
    }

    // Relinquish control of memory
    pthread_mutex_unlock(&mutex);

  } while (1 == 1);
}


/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there is   */
/*   no previous state file, set to default                 */
/*                                                          */
/************************************************************/
void InitCommandData() {
  int fp, n_read=0;

  fp = open("/tmp/mcp.prev_status", O_RDONLY);

  if (fp >= 0) {
    n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct));
    close(fp);
  }

  /** initialize stuff that we don't want from prev_status here **/
  CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
  CommandData.pumps.bal1_on = 0;
  CommandData.pumps.bal1_reverse = 0;
  CommandData.pumps.bal2_on = 0;
  CommandData.pumps.bal2_reverse = 0;
  CommandData.pumps.bal_on = 0.5 * 1648.;
  CommandData.pumps.bal_off = 0.2 * 1648.;
  CommandData.pumps.bal_target = 0.0 * 1648.;

  CommandData.pumps.inframe_cool1_on = 0;
  CommandData.pumps.inframe_cool1_off = 0;
  CommandData.pumps.lock_out = 0;
  CommandData.pumps.lock_in = 0;
  CommandData.pumps.lock_point = 0;
  CommandData.pumps.outframe_cool1_on = 0;
  CommandData.pumps.outframe_cool1_off = 0;
  CommandData.pumps.outframe_cool2_on = 0;
  CommandData.pumps.outframe_cool2_off = 0;
  CommandData.pumps.pwm1 = 2047;
  CommandData.pumps.pwm2 = 2047;
  CommandData.pumps.pwm3 = 2047;
  CommandData.pumps.pwm4 = 2047;

  /** return if we succsesfully read the previous status **/
  if (n_read == sizeof(struct CommandDataStruct)) return;

  /** put stuff that we want to keep from prev_status here **/
  CommandData.timeout = 60*60;

  CommandData.roll_gain.P = 30000;

  CommandData.ele_gain.I = 15000;
  CommandData.ele_gain.P = 700;

  CommandData.azi_gain.P = 1200;
  CommandData.azi_gain.I = 10000; // unused

  CommandData.pivot_gain.SP = 2000;
  CommandData.pivot_gain.P = 60;
  CommandData.pivot_gain.D = 18;  //unused

  CommandData.t_gybox_setpoint = 30.0;
  CommandData.gy_heat_gain.P = 10;
  CommandData.gy_heat_gain.I = 60;
  CommandData.gy_heat_gain.D = 50;

  CommandData.t_isc_setpoint = 30.0;
  CommandData.isc_heat_gain.P = 10;
  CommandData.isc_heat_gain.I = 60;
  CommandData.isc_heat_gain.D = 50;

  CommandData.current_mode = RASTER_MODE;
  CommandData.default_mode = ASCENT_MODE;

  CommandData.default_sensor = ISC;
  CommandData.use_sun = 1;
  CommandData.use_isc = 1;
  CommandData.use_vsc = 0;
  CommandData.use_mag = 1;

  SIPData.MKScal[0].m_hi = 0.01;
  SIPData.MKScal[0].m_med = 0.1;
  SIPData.MKScal[0].m_lo = 1;
  SIPData.MKScal[0].b_hi = 0;
  SIPData.MKScal[0].b_med = 0;
  SIPData.MKScal[0].b_lo = 0;

  SIPData.MKScal[1].m_hi = 0.01;
  SIPData.MKScal[1].m_med = 0.1;
  SIPData.MKScal[1].m_lo = 1;
  SIPData.MKScal[1].b_hi = 0;
  SIPData.MKScal[1].b_med = 0;
  SIPData.MKScal[1].b_lo = 0;

  CommandData.point_mode.az_mode = POINT_VEL;
  CommandData.point_mode.el_mode = POINT_VEL;
  CommandData.point_mode.az_dest = 0;
  CommandData.point_mode.el_dest =45;
  CommandData.point_mode.az_vel = 0;
  CommandData.point_mode.el_vel = 0;

  CommandData.Bias.clockInternal = 1;
  CommandData.Bias.biasAC = 1;
  CommandData.Bias.biasRamp = 0;

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
  CommandData.Cryo.lndir = 0;
  CommandData.Cryo.lnvalve = 0;
  CommandData.Cryo.lhedir = 0;
  CommandData.Cryo.lhevalve = 0;

  WritePrevStatus();

}
