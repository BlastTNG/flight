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

#include "command_list.h"
#include "command_struct.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "slow_dl.h"
#include "tx_struct.h"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

#define BAL_VETO_LENGTH 500

/* #define SUN             0 */
/* #define ISC             1 */
/* #define VSC             2 */
/* #define MAG             3 */

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
#define LOCK_OFFSET (3.6)

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

#ifdef BOLOTEST
#  define USE_FIFO_CMD
#endif

void SetRaDec(double ra, double dec); /* defined in pointing.c */
void SetTrimToISC();
void ClearTrim();
void AzElTrim(double az, double el);

const char UnknownCommand[] = "Unknown Command";

extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];

pthread_mutex_t mutex;

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus() {
  int fp, n;

  /** write the default file */
  fp = open("/tmp/mcp.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    merror(MCP_ERROR, "mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    merror(MCP_ERROR, "mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    merror(MCP_ERROR, "mcp.prev_status close()");
    return;
  }
}

void bc_close() {
}

int bc_setserial(char *input_tty) {
  int fd;
  struct termios term; 

  if ((fd = open(input_tty, O_RDWR)) < 0)
    merror(MCP_TFATAL, "Unable to open serial port");

  if (tcgetattr(fd, &term))
    merror(MCP_TFATAL, "Unable to get serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if(cfsetospeed(&term, B1200))          /*  <======= SET THE SPEED HERE */
    merror(MCP_TFATAL, "Error setting serial output speed");

  if(cfsetispeed(&term, B1200))          /*  <======= SET THE SPEED HERE */
    merror(MCP_TFATAL, "Error setting serial input speed");

  if( tcsetattr(fd, TCSANOW, &term) )
    merror(MCP_TFATAL, "Unable to set serial attributes");

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

enum singleCommand SCommand(char *cmd) {
  int i;

  for (i = 0; i < N_SCOMMANDS; i++) {
    if (strcmp(scommands[i].name, cmd) == 0)
      return scommands[i].command;
  }

  return -1;
}

int SIndex(enum singleCommand command) {
  int i;

  for (i = 0; i < N_SCOMMANDS; i++)
    if (scommands[i].command == command)
      return i;

  return -1;
}

const char* SName(enum singleCommand command) {
  int i = SIndex(command);
  return (i == -1) ? UnknownCommand : scommands[i].name;
}

void SingleCommand (enum singleCommand command) {
  int i_point;

  mprintf(MCP_INFO, "Single command: %d (%s)\n", command, SName(command));

  /* Update CommandData structure with new info */

  if (command == stop) {      /* Pointing aborts */
    CommandData.pointing_mode.mode = P_DRIFT;
    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = 0;
    CommandData.pointing_mode.vaz = 0.0;
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;

  } else if (command == mcc_halt) {
    mputs(MCP_WARNING, "Halting the MCC\n");
    system("/sbin/halt");
  } else if (command == sync_adc)
    CommandData.ADC_sync_timeout = 0;
  else if (command == tdrss_veto)
    CommandData.tdrssVeto = 1;
  else if (command == tdrss_allow)
    CommandData.tdrssVeto = 0;

  else if (command == trim_to_isc)
    SetTrimToISC();
  else if (command == reset_trims)
    ClearTrim();
  else if (command == auto_gyro)
    CommandData.autogyro = 1;

  else if (command == az_off) /* disable az motors */
    CommandData.disable_az = 1;
  else if (command == az_on) /* enable az motors */
    CommandData.disable_az = 0;
  else if (command == el_off) { /* disable el motors */
    CommandData.disable_el = 1;
    CommandData.force_el = 0;
  } else if (command == el_on) { /* enable el motors */
    CommandData.disable_el = 0;
    CommandData.force_el = 0;
  } else if (command == force_el_on) { /* force enabling of el motors */
    CommandData.disable_el = 0;
    CommandData.force_el = 1;

  } else if (command == sun_veto)       /* Veto sensors */
    CommandData.use_sun = 0;
  else if (command == isc_veto)
    CommandData.use_isc = 0;
  else if (command == mag_veto)
    CommandData.use_mag = 0;
  else if (command == gps_veto)
    CommandData.use_gps = 0;
  else if (command == elenc_veto)
    CommandData.use_elenc = 0;
  else if (command == elclin_veto)
    CommandData.use_elclin = 0;

  else if (command == sun_allow)       /* Un-veto sensors */
    CommandData.use_sun = 1;
  else if (command == isc_allow)
    CommandData.use_isc = 1;
  else if (command == mag_allow)
    CommandData.use_mag = 1;
  else if (command == gps_allow)
    CommandData.use_gps = 1;
  else if (command == elenc_allow)
    CommandData.use_elenc = 1;
  else if (command == elclin_allow)
    CommandData.use_elclin = 1;

  else if (command == clock_int)    /* Bias settings */
    CommandData.Bias.clockInternal = 1;
  else if (command == clock_ext)
    CommandData.Bias.clockInternal = 0;
  else if (command == bias_ac)
    CommandData.Bias.biasAC = 1;
  else if (command == bias_dc)
    CommandData.Bias.biasAC = 0;
  else if (command == ramp)
    CommandData.Bias.biasRamp = 1;
  else if (command == fixed)
    CommandData.Bias.biasRamp = 0;

  else if (command == level_on)    /* Cryo commanding */
    CommandData.Cryo.heliumLevel = 1;
  else if (command == level_off)
    CommandData.Cryo.heliumLevel = 0;
  else if (command == charcoal_on)
    CommandData.Cryo.charcoalHeater = 1;
  else if (command == charcoal_off)
    CommandData.Cryo.charcoalHeater = 0;
  else if (command == coldplate_on)
    CommandData.Cryo.coldPlate = 1;
  else if (command == coldplate_off)
    CommandData.Cryo.coldPlate = 0;
  else if (command == cal_on)
    CommandData.Cryo.calibrator = on;
  else if (command == cal_off)
    CommandData.Cryo.calibrator = off;
  else if (command == pot_valve_open) {
    CommandData.Cryo.potvalve_open = 40;
    CommandData.Cryo.potvalve_close = 0;
  } else if (command == pot_valve_close) {
    CommandData.Cryo.potvalve_close = 40;
    CommandData.Cryo.potvalve_open = 0;
  } else if (command == pot_valve_on)
    CommandData.Cryo.potvalve_on = 1;
  else if (command == pot_valve_off)
    CommandData.Cryo.potvalve_on = 0;
  else if (command == he_valve_open) {
    CommandData.Cryo.lhevalve_open = 40;
    CommandData.Cryo.lhevalve_close = 0;
  } else if (command == he_valve_close) {
    CommandData.Cryo.lhevalve_close = 40;
    CommandData.Cryo.lhevalve_open = 0;
  } else if (command == he_valve_on)
    CommandData.Cryo.lhevalve_on = 1;
  else if (command == he_valve_off)
    CommandData.Cryo.lhevalve_on = 0;
  else if (command == auto_bdaheat)
    CommandData.Cryo.autoBDAHeat = 1;

  else if (command == balance_veto)
    CommandData.pumps.bal_veto = -1;
  else if (command == balance_allow)
    CommandData.pumps.bal_veto = 0;

  else if (command == balpump_on)
    CommandData.pumps.bal1_on = 1;
  else if (command == balpump_off)
    CommandData.pumps.bal1_on = 0;
  else if (command == balpump_up)
    CommandData.pumps.bal1_reverse = 0;
  else if (command == balpump_down)
    CommandData.pumps.bal1_reverse = 1;
  else if (command == sprpump_on)
    CommandData.pumps.bal2_on = 1;
  else if (command == sprpump_off)
    CommandData.pumps.bal2_on = 0;
  else if (command == sprpump_fwd)
    CommandData.pumps.bal2_reverse = 0;
  else if (command == sprpump_rev)
    CommandData.pumps.bal2_reverse = 1;

  else if (command == inner_cool_on)
    CommandData.pumps.inframe_cool1_on = 40;
  else if (command == inner_cool_off)
    CommandData.pumps.inframe_cool1_off = 40;

  else if (command == outer_cool_on)
    CommandData.pumps.outframe_cool1_on = 40;
  else if (command == outer_cool_off)
    CommandData.pumps.outframe_cool1_off = 40;
  else if (command == outer_spare_on)
    CommandData.pumps.outframe_cool2_on = 40;
  else if (command == outer_spare_off)
    CommandData.pumps.outframe_cool2_off = 40;
  else if (command == pin_in)
    CommandData.pumps.lock_in = 1;
  else if (command == lock_off)
    CommandData.pumps.lock_off = 1;
  else if (command == unlock) {
    CommandData.pumps.lock_out = 1;
    if (CommandData.pointing_mode.mode == P_LOCK) {
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
    }

    /***************************************/
    /********* ISC Commanding  *************/
  } else if (command == isc_run)
    CommandData.ISCState[0].pause = 0;
  else if (command == isc_shutdown)
    CommandData.ISCState[0].shutdown = 1;
  else if (command == isc_reboot)
    CommandData.ISCState[0].shutdown = 2;
  else if (command == isc_cam_cycle)
    CommandData.ISCState[0].shutdown = 3;
  else if (command == isc_pause)
    CommandData.ISCState[0].pause = 1;
  else if (command == isc_abort)
    CommandData.ISCState[0].abort = 1;
  else if (command == isc_reconnect)
    CommandData.ISCControl[0].reconnect = 1;
  else if (command == isc_save_images)
    CommandData.ISCState[0].save = 1;
  else if (command == isc_discard_images)
    CommandData.ISCState[0].save = 0;
  else if (command == isc_full_screen)
    CommandData.ISCState[0].display_mode = full;
  else if (command == isc_auto_focus) {
    CommandData.ISCState[0].abort = 1;
    CommandData.ISCControl[0].autofocus = 10;
    CommandData.ISCControl[0].old_focus = CommandData.ISCState[1].focus_pos;
    CommandData.ISCState[0].focus_pos = FOCUS_RANGE;

    /***************************************/
    /********* OSC Commanding  *************/
  } else if (command == osc_run)
    CommandData.ISCState[1].pause = 0;
  else if (command == osc_shutdown)
    CommandData.ISCState[1].shutdown = 1;
  else if (command == osc_reboot)
    CommandData.ISCState[1].shutdown = 2;
  else if (command == osc_cam_cycle)
    CommandData.ISCState[1].shutdown = 3;
  else if (command == osc_pause)
    CommandData.ISCState[1].pause = 1;
  else if (command == osc_abort)
    CommandData.ISCState[1].abort = 1;
  else if (command == osc_reconnect)
    CommandData.ISCControl[1].reconnect = 1;
  else if (command == osc_save_images)
    CommandData.ISCState[1].save = 1;
  else if (command == osc_discard_images)
    CommandData.ISCState[1].save = 0;
  else if (command == osc_full_screen)
    CommandData.ISCState[1].display_mode = full;
  else if (command == osc_auto_focus) {
    CommandData.ISCState[1].abort = 1;
    CommandData.ISCControl[1].autofocus = 10;
    CommandData.ISCControl[1].old_focus = CommandData.ISCState[1].focus_pos;
    CommandData.ISCState[1].focus_pos = FOCUS_RANGE;
  } else if (command == xyzzy)
    ;
  else {
    mputs(MCP_WARNING, "***Invalid Single Word Command***\n");
    return; /* invalid command - no write or update */
  }

  i_point = GETREADINDEX(point_index);

  CommandData.pointing_mode.t =
    PointingData[i_point].t + CommandData.timeout;

  WritePrevStatus();
}

enum multiCommand MCommand(char *cmd) {
  int i;

  for (i = 0; i < N_MCOMMANDS; i++) {
    if (strcmp(mcommands[i].name, cmd) == 0)
      return mcommands[i].command;
  }

  return -1;
}

int MIndex(enum multiCommand command) {
  int i;

  for (i = 0; i < N_MCOMMANDS; i++)
    if (mcommands[i].command == command)
      return i;

  return -1;
}

const char* MName(enum multiCommand command) {
  int i = MIndex(command);
  return (i == -1) ? UnknownCommand : mcommands[i].name;
}

void MultiCommand (enum multiCommand command, unsigned short *dataq) {
  int i, dataqind;
  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char type;
  int i_point;
  int index = MIndex(command);


#ifndef USE_FIFO_CMD
  double min;

  /* compute renormalised values */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    min = mcommands[index].params[i].min;
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++] + mcommands[index].params[i].min;
      mprintf(MCP_INFO, "param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = (float)dataq[dataqind++] * (mcommands[index].params[i].max
          - min) / MAX_15BIT + min;
      mprintf(MCP_INFO, "param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = (float)((int)dataq[dataqind++] << 15); /* upper 15 bits */
      rvalues[i] += (float)dataq[dataqind++];             /* lower 15 bits */
      rvalues[i] = rvalues[i] * (mcommands[index].params[i].max - min) /
        MAX_30BIT + min;
      mprintf(MCP_INFO, "param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }
#else
  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      mprintf(MCP_INFO, "param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      mprintf(MCP_INFO, "param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      mprintf(MCP_INFO, "param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }

  mprintf(MCP_INFO, "Multiword Command: %d (%s)\n", command, MName(command));
#endif

  /* Update CommandData struct with new info
   * If the parameter is type 'i'     set CommandData using ivalues[i]
   * If the parameter is type 'f'/'l' set CommandData using rvalues[i]
   */
  if (command == az_el_goto) {
    CommandData.pointing_mode.mode = P_AZEL_GOTO;
    CommandData.pointing_mode.X = rvalues[0];  /* az */
    CommandData.pointing_mode.Y = rvalues[1];  /* el */
    CommandData.pointing_mode.vaz = 0.0;
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;
  } else if (command == az_scan) {
    CommandData.pointing_mode.mode = P_AZ_SCAN;
    CommandData.pointing_mode.X = rvalues[0];  /* az */
    CommandData.pointing_mode.Y = rvalues[1];  /* el */
    CommandData.pointing_mode.w = rvalues[2];  /* width */
    CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.h = 0;
  } else if (command == drift) {
    CommandData.pointing_mode.mode = P_DRIFT;
    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = 0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.vaz = rvalues[0]; /* az speed */
    CommandData.pointing_mode.del = rvalues[1]; /* el speed */
    CommandData.pointing_mode.h = 0;
  } else if (command == ra_dec_goto) {
    CommandData.pointing_mode.mode = P_RADEC_GOTO;
    CommandData.pointing_mode.X = rvalues[0]; /* ra */
    CommandData.pointing_mode.Y = rvalues[1]; /* dec */
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.vaz = 0;
    CommandData.pointing_mode.del = 0;
    CommandData.pointing_mode.h = 0;
  } else if (command == vcap) {
    CommandData.pointing_mode.mode = P_VCAP;
    CommandData.pointing_mode.X = rvalues[0]; /* ra */
    CommandData.pointing_mode.Y = rvalues[1]; /* dec */
    CommandData.pointing_mode.w = rvalues[2]; /* radius */
    CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
    CommandData.pointing_mode.del = rvalues[4]; /* el drift speed */
    CommandData.pointing_mode.h = 0;
  } else if (command == cap) {
    CommandData.pointing_mode.mode = P_CAP;
    CommandData.pointing_mode.X = rvalues[0]; /* ra */
    CommandData.pointing_mode.Y = rvalues[1]; /* dec */
    CommandData.pointing_mode.w = rvalues[2]; /* radius */
    CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
    CommandData.pointing_mode.del = rvalues[4]; /* el step size */
    CommandData.pointing_mode.h = 0;
  } else if (command == box) {
    CommandData.pointing_mode.mode = P_BOX;
    CommandData.pointing_mode.X = rvalues[0]; /* ra */
    CommandData.pointing_mode.Y = rvalues[1]; /* dec */
    CommandData.pointing_mode.w = rvalues[2]; /* width */
    CommandData.pointing_mode.h = rvalues[3]; /* height */
    CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
    CommandData.pointing_mode.del = rvalues[5]; /* el step size */
  } else if (command == vbox) {
    CommandData.pointing_mode.mode = P_VBOX;
    CommandData.pointing_mode.X = rvalues[0]; /* ra */
    CommandData.pointing_mode.Y = rvalues[1]; /* dec */
    CommandData.pointing_mode.w = rvalues[2]; /* width */
    CommandData.pointing_mode.h = rvalues[3]; /* height */
    CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
    CommandData.pointing_mode.del = rvalues[5]; /* el drift speed */

    /***************************************/
    /********** Pointing Motor Trims *******/
  } else if (command == az_el_trim)
    AzElTrim(rvalues[0], rvalues[1]);
  else if (command == ra_dec_set)
    SetRaDec(rvalues[0], rvalues[1]);
  else if (command == gyro_override) {
    CommandData.gy2_offset = rvalues[0];
    CommandData.gy3_offset = rvalues[1];
    CommandData.autogyro = 0;

    /***************************************/
    /********** Pointing Motor Gains *******/
  } else if (command == roll_gain) /* roll Gains */
    CommandData.roll_gain.P = ivalues[0];
  else if (command == el_gain) {  /* ele gains */
    CommandData.ele_gain.P = ivalues[0];
    CommandData.ele_gain.I = ivalues[1];
  } else if (command == az_gain) {  /* az gains */
    CommandData.azi_gain.P = ivalues[0];
    CommandData.azi_gain.I = ivalues[1];
  } else if (command == pivot_gain) {  /* pivot gains */
    CommandData.pivot_gain.SP = (rvalues[0] / 60. + 2.605) / 7.9498291016e-5;
    CommandData.pivot_gain.P = ivalues[1];
  } else if (command == back_emf) {
    CommandData.emf_gain = rvalues[0] * 6500;
    CommandData.emf_offset = rvalues[1] * 500;

    /***************************************/
    /********** Inner Frame Lock  **********/
  } else if (command == lock) {  /* Lock Inner Frame */
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_VETO_LENGTH;
    CommandData.pumps.lock_point = 1;
    CommandData.pointing_mode.mode = P_LOCK;
    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = LockPosition(rvalues[0]);
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;
    CommandData.pointing_mode.vaz = 0;
    CommandData.pointing_mode.del = 0;
    mprintf(MCP_INFO, "Lock Mode: %g\n", CommandData.pointing_mode.Y);

    /***************************************/
    /********** Balance System  ************/
  } else if (command == setpoints) {
    CommandData.pumps.bal_on = rvalues[0] * 1648.;
    CommandData.pumps.bal_off = rvalues[1] * 1648.;
    CommandData.pumps.bal_target = rvalues[2] * 1648.;
  } else if (command == bal_level)
    CommandData.pumps.pwm1 = 2047 - rvalues[0] * 2047. / 100;
  else if (command == bal_gain) {
    CommandData.pumps.bal_gain = rvalues[0];
    CommandData.pumps.bal_max = 2047 - rvalues[1] * 2047. / 100;
    CommandData.pumps.bal_min = 2047 - rvalues[2] * 2047. / 100;

    /***************************************/
    /********** Cooling System  ************/
  } else if (command == spare_level)
    CommandData.pumps.pwm2 = 2047 - rvalues[0] * 2047. / 100;
  else if (command == inner_level)
    CommandData.pumps.pwm3 = 2047 - rvalues[0] * 2047. / 100;
  else if (command == outer_level)
    CommandData.pumps.pwm4 = 2047 - rvalues[0] * 2047. / 100;

    /***************************************/
    /******** Electronics Heaters  *********/
  else if (command == t_gyrobox)  /* gyro heater setpoint */
    CommandData.t_gybox_setpoint = rvalues[0];
  else if (command == t_gyro_gain) {  /* gyro heater gains */
    CommandData.gy_heat_gain.P = ivalues[0];
    CommandData.gy_heat_gain.I = ivalues[1];
    CommandData.gy_heat_gain.D = ivalues[2];

    /***************************************/
    /*************** Misc  *****************/
  } else if (command == timeout)        /* Set timeout */
    CommandData.timeout = ivalues[0];
  else if (command == alice_file)  /* change downlink XML file */
    CommandData.alice_file = ivalues[0];
  else if (command == plugh) /* A hollow voice says "Plugh". */
    CommandData.plover = ivalues[0];

    /***************************************/
    /*************** Bias  *****************/
  else if (command == bias1_level) {    /* Set bias 1 */
    CommandData.Bias.SetLevel1 = 1;
    CommandData.Bias.bias1 = ivalues[0];
  } else if (command == bias2_level) {   /* Set bias 2 */
    CommandData.Bias.SetLevel2 = 1;
    CommandData.Bias.bias2 = ivalues[0];
  } else if (command == bias3_level) {   /* Set bias 3 */
    CommandData.Bias.SetLevel3 = 1;
    CommandData.Bias.bias3 = ivalues[0];
  } else if (command == phase) {
    if (ivalues[0] >= 5 && ivalues[0] <= 16) 
      CommandData.Phase[ivalues[0] - 5] = ivalues[1];

    /***************************************/
    /*********** Cal Lamp  *****************/
  } else if (command == cal_pulse) {
    CommandData.Cryo.calibrator = pulse;
    CommandData.Cryo.calib_pulse = ivalues[0] / 10;
  } else if (command == cal_repeat) {
    CommandData.Cryo.calibrator = repeat;
    CommandData.Cryo.calib_pulse = ivalues[0] / 10;
    CommandData.Cryo.calib_period = ivalues[1];

    /***************************************/
    /********* Cryo heat   *****************/
  } else if (command == jfet_heat) {
    CommandData.Cryo.JFETHeat = rvalues[0] * 2047./100.;
  } else if (command == heatsw_heat) {
    CommandData.Cryo.heatSwitch = rvalues[0] * 2047./100.;
  } else if (command == he3_heat) {
    CommandData.Cryo.heliumThree = rvalues[0] * 2047./100.;
  } else if (command == bda_heat) {
    CommandData.Cryo.BDAHeat = rvalues[0] * 2047./100.;
  } else if (command == bda_gain) {
    CommandData.Cryo.BDAGain.P = ivalues[0];
    CommandData.Cryo.BDAGain.I = ivalues[1];
    CommandData.Cryo.BDAGain.D = ivalues[2];
    CommandData.Cryo.BDAFiltLen = ivalues[3];
  } else if (command == bda_heat) {
    CommandData.Cryo.BDAHeat = ivalues[0];
    CommandData.Cryo.autoBDAHeat = 0;


    /***************************************/
    /********* ISC Commanding  *************/
  } else if (command == isc_set_focus) {
    CommandData.ISCState[0].focus_pos = ivalues[0];
  } else if (command == isc_set_aperture) {
    CommandData.ISCState[0].ap_pos = ivalues[0];
  } else if (command == isc_pixel_centre) {
    CommandData.ISCState[0].roi_x = ivalues[0];
    CommandData.ISCState[0].roi_y = ivalues[1];
    CommandData.ISCState[0].display_mode = roi;
  } else if (command == isc_blob_centre) {
    CommandData.ISCState[0].blob_num = ivalues[0];
    CommandData.ISCState[0].display_mode = blob;
  } else if (command == isc_offset) {
    CommandData.ISCState[0].azBDA = rvalues[0] * DEG2RAD;
    CommandData.ISCState[0].elBDA = rvalues[1] * DEG2RAD;
  } else if (command == isc_integrate) {
    CommandData.ISCControl[0].fast_pulse_width = rvalues[0] * 10.4166666666667;
    CommandData.ISCControl[0].pulse_width = rvalues[1] * 10.4166666666667;
  } else if (command == isc_det_set) {
    CommandData.ISCState[0].grid = ivalues[0];
    CommandData.ISCState[0].sn_threshold = rvalues[1];
    CommandData.ISCState[0].cenbox = ivalues[2];
    CommandData.ISCState[0].apbox = ivalues[3];
    CommandData.ISCState[0].mult_dist = ivalues[4];
  } else if (command == isc_max_blobs) {
    CommandData.ISCState[0].maxBlobMatch = ivalues[0];
  } else if (command == isc_catalogue) {
    CommandData.ISCState[0].mag_limit = rvalues[0];
    CommandData.ISCState[0].norm_radius = rvalues[1] * DEG2RAD;
    CommandData.ISCState[0].lost_radius = rvalues[2] * DEG2RAD;
  } else if (command == isc_tolerances) {
    CommandData.ISCState[0].tolerance = rvalues[0] / 3600. * DEG2RAD;
    CommandData.ISCState[0].match_tol = rvalues[1] / 100;
    CommandData.ISCState[0].quit_tol = rvalues[2] / 100;
    CommandData.ISCState[0].rot_tol = rvalues[3] * DEG2RAD;
  } else if (command == isc_hold_current)
    CommandData.ISCState[0].hold_current = ivalues[0];
  else if (command == isc_save_period)
    CommandData.ISCControl[0].save_period = ivalues[0] * 100;
  else if (command == isc_gain) {
    CommandData.ISCState[0].gain = rvalues[0];
    CommandData.ISCState[0].offset = ivalues[1];


    /***************************************/
    /********* OSC Commanding  *************/
  } else if (command == osc_set_focus) {
    CommandData.ISCState[1].focus_pos = ivalues[0];
  } else if (command == osc_set_aperture) {
    CommandData.ISCState[1].ap_pos = ivalues[0];
  } else if (command == osc_pixel_centre) {
    CommandData.ISCState[1].roi_x = ivalues[0];
    CommandData.ISCState[1].roi_y = ivalues[1];
    CommandData.ISCState[1].display_mode = roi;
  } else if (command == osc_blob_centre) {
    CommandData.ISCState[1].blob_num = ivalues[0];
    CommandData.ISCState[1].display_mode = blob;
  } else if (command == osc_offset) {
    CommandData.ISCState[1].azBDA = rvalues[0] * DEG2RAD;
    CommandData.ISCState[1].elBDA = rvalues[1] * DEG2RAD;
  } else if (command == osc_integrate) {
    CommandData.ISCControl[1].fast_pulse_width = rvalues[0] * 10.4166666666667;
    CommandData.ISCControl[1].pulse_width = rvalues[1] * 10.4166666666667;
  } else if (command == osc_det_set) {
    CommandData.ISCState[1].grid = ivalues[0];
    CommandData.ISCState[1].sn_threshold = rvalues[1];
    CommandData.ISCState[1].cenbox = ivalues[2];
    CommandData.ISCState[1].apbox = ivalues[3];
    CommandData.ISCState[1].mult_dist = ivalues[4];
  } else if (command == osc_max_blobs) {
    CommandData.ISCState[1].maxBlobMatch = ivalues[0];
  } else if (command == osc_catalogue) {
    CommandData.ISCState[1].mag_limit = rvalues[0];
    CommandData.ISCState[1].norm_radius = rvalues[1] * DEG2RAD;
    CommandData.ISCState[1].lost_radius = rvalues[2] * DEG2RAD;
  } else if (command == osc_tolerances) {
    CommandData.ISCState[1].tolerance = rvalues[0] / 3600. * DEG2RAD;
    CommandData.ISCState[1].match_tol = rvalues[1] / 100;
    CommandData.ISCState[1].quit_tol = rvalues[2] / 100;
    CommandData.ISCState[1].rot_tol = rvalues[3] * DEG2RAD;
  } else if (command == osc_hold_current)
    CommandData.ISCState[1].hold_current = ivalues[0];
  else if (command == osc_save_period)
    CommandData.ISCControl[1].save_period = ivalues[0] * 100;
  else if (command == osc_gain) {
    CommandData.ISCState[1].gain = rvalues[0];
    CommandData.ISCState[1].offset = ivalues[1];

  } else {
    mputs(MCP_WARNING, "Invalid Multi Word Command***\n");
    return; /* invalid command - don't update */
  }

  i_point = GETREADINDEX(point_index);

  CommandData.pointing_mode.t =
    PointingData[i_point].t + CommandData.timeout;

  WritePrevStatus();
}

void GPSPosition (unsigned char *indata) {
  /* Send new information to CommandData */

  SIPData.GPSpos.lat = ParseGPS(indata);
  SIPData.GPSpos.lon = ParseGPS(indata + 4); /* sip sends east lon */
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
  z_med = log(SIPData.MKScal.m_med * med + SIPData.MKScal.b_med);
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


/* Send TDRSS Low Rate Packet */

void SendDownData(char tty_fd) {
  unsigned char buffer[SLOWDL_LEN], data[3 + SLOWDL_LEN + 1];
  int i, temp;
  int bitpos, bytepos, numbits;
  static char firsttime;

  bitpos = 0;
  bytepos = 0;
  memset(data, 0, SLOWDL_LEN);

  for (i = 0; i < SLOWDL_NUM_DATA; i++) {
    switch (SlowDLInfo[i].type) {
      case SLOWDL_FORCE_INT:
        /* Round value to an integer and try to fit it in numbits */
        if ((int)SlowDLInfo[i].value > (1 << (SlowDLInfo[i].numbits - 1)) - 1)
          temp = 0;     /* Indicates value was too big */
        else if ((int)SlowDLInfo[i].value < -1 * ((1 << 
                (SlowDLInfo[i].numbits - 1)) - 2))
          temp = 1;     /* Indicates value was too small */
        else
          temp = (int)SlowDLInfo[i].value + (1 << (SlowDLInfo[i].numbits - 1));
        numbits = SlowDLInfo[i].numbits;
        break;

      case SLOWDL_U_MASK:
        /* Simply take the bottom numbits from the unsigned number */
        temp = ((int)(SlowDLInfo[i].value)) & ((1 << SlowDLInfo[i].numbits) - 
            1);
        numbits = SlowDLInfo[i].numbits;
        break;

      case SLOWDL_TAKE_BIT:
        /* Intended for bitfields:  get the value of bit number numbit */
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

    if (bytepos >= SLOWDL_LEN) {
      mprintf(MCP_WARNING, "Slow DL size is larger than maximum size of %d.  "
              "Reduce length of SlowDLInfo structure.", SLOWDL_LEN);
      break;
    }
  }

  if (firsttime) {
    mprintf(MCP_INFO, "Slow DL size = %d\n", bytepos);
    firsttime = 0;
  }

  buffer[0] = SLOWDL_DLE;
  buffer[1] = SLOWDL_SYNC;
  buffer[2] = SLOWDL_LEN;
  memcpy(buffer + 3, data, SLOWDL_LEN);
  buffer[3 + SLOWDL_LEN] = SLOWDL_ETX;

  write(tty_fd, buffer, 3 + SLOWDL_LEN + 1);
}

/* compute the size of the data queue for the given command */
int DataQSize(int index) {
  int i, size = mcommands[index].numparams;

  for (i = 0; i < mcommands[index].numparams; ++i)
    if (mcommands[index].params[i].type == 'l')
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

  pthread_setspecific(identity, "fifo");
  mputs(MCP_STARTUP, "WatchFIFO startup\n");

  if ((fifo = open("/tmp/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
    merror(MCP_TFATAL, "Unable to open FIFO");

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    mprintf(MCP_INFO, "Command received: %s\n", command);
    index = -1;
    while((command[++index] != ' ') && command[index]);
    command[index++] = 0;

    pindex = 0;
    mcommand_count = 0;
    do {
      if ((command[index] == ' ' || command[index] == 0) && pindex > 0) {
        pbuf[pindex] = 0;
        if (NULL == (mcommand_data[mcommand_count] =
              realloc(mcommand_data[mcommand_count], pindex + 2)))
          merror(MCP_TFATAL, "malloc failed in FIFO CommandData");

        strncpy(mcommand_data[mcommand_count++], pbuf, pindex + 1);
        pindex = 0;
      } else {
        pbuf[pindex++] = command[index];
      }
    } while (command[index++] != 0);
    mprintf(MCP_INFO, "%i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SCommand(command);
      SingleCommand(mcommand);
      mcommand = -1;
    } else {
      mcommand = MCommand(command);
      mputs(MCP_INFO, " Multi word command received\n");
      MultiCommand(mcommand, (unsigned short*) mcommand_data);
      mcommand = -1;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);

  }
}

char *COMM[] = {"/dev/ttyS0", "/dev/ttyS4"};
char *ident[] = {"cmd0", "cmd1"};

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

  pthread_setspecific(identity, ident[port]);
  mprintf(MCP_STARTUP, "WatchPort(%i) startup\n", port);

  tty_fd = bc_setserial(COMM[port]);

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

    /* Take control of memory */
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
//                    mprintf(MCP_INFO, "COMM%i: Data request\n", port + 1);
        } else if (buf == 0x14) { /* Command */
          readstage = 2;
//                    mprintf(MCP_INFO, "COMM%i: Command\n", port + 1);
        } else if (buf == 0x10) { /* GPS Position */
          readstage = 4;
//                    mprintf(MCP_INFO, "COMM%i: GPS Position\n", port + 1);
        } else if (buf == 0x11) { /* GPS Time */
          readstage = 5;
//                    mprintf(MCP_INFO, "COMM%i: GPS Time\n", port + 1);
        } else if (buf == 0x12) { /* MKS Altitude */
          readstage = 6;
//                    mprintf(MCP_INFO, "COMM%i: MKS Altitude\n", port + 1);
        } else {
          mprintf(MCP_WARNING,
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
            mprintf(MCP_WARNING,
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
              mprintf(MCP_INFO, "COMM%i:  Single command received\n",
                  port + 1);
              SingleCommand(indata[0]);
              mcommand = -1;
            } else if (((indata[1] >> 5) & 0x07) == 0x04) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(MIndex(mcommand));
              mprintf(MCP_INFO,
                  "COMM%i:  Multi word command %d (%s) started\n",
                  port + 1, mcommand, MName(mcommand));

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;  
            } else if ((((indata[1] >> 7) & 0x01) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              mprintf(MCP_INFO, "COMM%i:  Multi word command continues...\n",
                  port + 1);
              mcommand_count++;
            } else if ((((indata[1] >> 5) & 0x07) == 0x06) &&
                (mcommand == indata[0]) && 
                ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              mprintf(MCP_INFO, "COMM%i:  Multi word command ends \n",
                  port + 1);
              MultiCommand(mcommand, (unsigned short *) mcommand_data);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              mprintf(MCP_WARNING,
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
          mprintf(MCP_WARNING,
              "COMM%i: Bad encoding: Bad packet terminator: %02X\n",
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
            mprintf(MCP_WARNING, "COMM%i: Bad encoding in GPS Position: "
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
            mprintf(MCP_WARNING, "COMM%i: Bad encoding in GPS Time: "
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
            mprintf(MCP_WARNING, "COMM%i: Bad encoding in MKS Altitude: "
                "Bad packet terminator: %02X\n", port + 1, buf);
          }
        }
    }

    /* Relinquish control of memory */
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
    merror(MCP_ERROR, "Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      merror(MCP_ERROR, "prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      merror(MCP_ERROR, "extra prev_status read()");
    if (close(fp) < 0)
      merror(MCP_ERROR, "prev_status close()");
  }

  CommandData.pointing_mode.t = time(NULL) + CommandData.timeout;

  /** initialize stuff that we don't want from prev_status here **/
  CommandData.force_el = 0;

  CommandData.pumps.bal_veto = -1; //BAL_VETO_LENGTH;
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

  CommandData.Bias.SetLevel1 = 1;
  CommandData.Bias.SetLevel2 = 1;
  CommandData.Bias.SetLevel3 = 1;

  CommandData.ISCState[0].shutdown = 0;
  CommandData.ISCState[1].shutdown = 0;

  CommandData.tdrssVeto = 0;

#ifndef USE_FIFO_CMD
  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    mprintf(MCP_WARNING, "prev_status: Wanted %i bytes but got %i.\n",
        sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    mputs(MCP_WARNING, "prev_status: Extra bytes found.\n");
  else
    return;
#endif

  mputs(MCP_WARNING, "Regenerating Command Data and prev_status\n");

  CommandData.pointing_mode.t = time(NULL) + CommandData.timeout;

  /** put stuff that we want to keep from prev_status here **/
  CommandData.pointing_mode.mode = P_DRIFT;
  CommandData.pointing_mode.X = 0;
  CommandData.pointing_mode.Y = 0;
  CommandData.pointing_mode.vaz = 0.0;
  CommandData.pointing_mode.del = 0.0;
  CommandData.pointing_mode.w = 0;
  CommandData.pointing_mode.h = 0;

  CommandData.timeout = 3600;

  CommandData.roll_gain.P = 30000;

  CommandData.ele_gain.I = 6000; /* was 8000 */
  CommandData.ele_gain.P = 700; /* was 1200 */

  CommandData.azi_gain.P = 20000;
  CommandData.azi_gain.I = 5000; 

  CommandData.pivot_gain.SP = 36960;
  CommandData.pivot_gain.P = 200;

  CommandData.emf_gain = 1;
  CommandData.emf_offset = 0;

  CommandData.t_gybox_setpoint = 30.0;
  CommandData.gy_heat_gain.P = 10;
  CommandData.gy_heat_gain.I = 60;
  CommandData.gy_heat_gain.D = 50;

  CommandData.disable_az = 0;
  CommandData.disable_el = 0;

  CommandData.use_elenc = 1;
  CommandData.use_elclin = 1;
  CommandData.use_sun = 0;
  CommandData.use_isc = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 1;

  SIPData.MKScal.m_hi = 0.01;
  SIPData.MKScal.m_med = 0.1;
  SIPData.MKScal.m_lo = 1;
  SIPData.MKScal.b_hi = 0;
  SIPData.MKScal.b_med = 0;
  SIPData.MKScal.b_lo = 0;

  CommandData.autogyro = 1;

  CommandData.pumps.bal_on = 0.5 * 1648.;
  CommandData.pumps.bal_off = 0.2 * 1648.;
  CommandData.pumps.bal_target = 0.0 * 1648.;
  CommandData.pumps.bal_gain = 0.2;
  CommandData.pumps.bal_max = 600;  /* 70% */
  CommandData.pumps.bal_min = 1750; /* 15% */

  CommandData.Bias.clockInternal = 0;
  CommandData.Bias.biasAC = 1;
  CommandData.Bias.biasRamp = 0;

  CommandData.Bias.bias1 = 0x02;
  CommandData.Bias.bias2 = 0x02;
  CommandData.Bias.bias3 = 0x02;

  CommandData.Cryo.heliumLevel = 0;
  CommandData.Cryo.charcoalHeater = 0;
  CommandData.Cryo.coldPlate = 0;
  CommandData.Cryo.JFETHeat = 0;
  CommandData.Cryo.heatSwitch = 0;
  CommandData.Cryo.heliumThree = 0;
  CommandData.Cryo.BDAHeat = 0;
  CommandData.Cryo.autoBDAHeat = 1;
  CommandData.Cryo.calibrator = off;
  CommandData.Cryo.potvalve_on = 0;
  CommandData.Cryo.potvalve_open = 0;
  CommandData.Cryo.potvalve_close = 0;
  CommandData.Cryo.lhevalve_on = 0;
  CommandData.Cryo.lhevalve_open = 0;
  CommandData.Cryo.lhevalve_close = 0;

  CommandData.ISCState[0].abort = 0;
  CommandData.ISCState[0].pause = 0;
  CommandData.ISCState[0].save = 0;
  CommandData.ISCState[0].autofocus = 0;
  CommandData.ISCState[0].focus_pos = 0;
  CommandData.ISCState[0].ap_pos = 495;
  CommandData.ISCState[0].display_mode = full;
  CommandData.ISCState[0].azBDA = 0;
  CommandData.ISCState[0].elBDA = 0;
  CommandData.ISCState[0].brightStarMode = 0;
  CommandData.ISCState[0].grid = 38;
  CommandData.ISCState[0].cenbox = 20;
  CommandData.ISCState[0].apbox = 5;
  CommandData.ISCState[0].maxBlobMatch = 5;
  CommandData.ISCState[0].sn_threshold = 3.5;
  CommandData.ISCState[0].mult_dist = 30;
  CommandData.ISCState[0].mag_limit = 9;
  CommandData.ISCState[0].norm_radius = 2. * DEG2RAD;
  CommandData.ISCState[0].lost_radius = 5. * DEG2RAD;
  CommandData.ISCState[0].tolerance = 20. / 3600. * DEG2RAD; /* 20 arcsec */
  CommandData.ISCState[0].match_tol = 0.8;
  CommandData.ISCState[0].quit_tol = 1;
  CommandData.ISCState[0].rot_tol = 5 * DEG2RAD;
  CommandData.ISCState[0].gain = 1;
  CommandData.ISCState[0].offset = 0;
  CommandData.ISCControl[0].save_period = 4000; /* 40 sec */
  CommandData.ISCControl[0].pulse_width = 3125; /* 300.00 msec */
  CommandData.ISCControl[0].fast_pulse_width = 625; /* 60.00 msec */

  CommandData.ISCState[1].abort = 0;
  CommandData.ISCState[1].pause = 0;
  CommandData.ISCState[1].save = 0;
  CommandData.ISCState[1].autofocus = 0;
  CommandData.ISCState[1].focus_pos = 0;
  CommandData.ISCState[1].ap_pos = 495;
  CommandData.ISCState[1].display_mode = full;
  CommandData.ISCState[1].azBDA = 0;
  CommandData.ISCState[1].elBDA = 0;
  CommandData.ISCState[1].brightStarMode = 0;
  CommandData.ISCState[1].grid = 38;
  CommandData.ISCState[1].cenbox = 20;
  CommandData.ISCState[1].apbox = 5;
  CommandData.ISCState[1].maxBlobMatch = 5;
  CommandData.ISCState[1].sn_threshold = 3.5;
  CommandData.ISCState[1].mult_dist = 30;
  CommandData.ISCState[1].mag_limit = 9;
  CommandData.ISCState[1].norm_radius = 2. * DEG2RAD;
  CommandData.ISCState[1].lost_radius = 5. * DEG2RAD;
  CommandData.ISCState[1].tolerance = 20. / 3600. * DEG2RAD; /* 20 arcsec */
  CommandData.ISCState[1].match_tol = 0.8;
  CommandData.ISCState[1].quit_tol = 1;
  CommandData.ISCState[1].rot_tol = 5 * DEG2RAD;
  CommandData.ISCState[1].gain = 1;
  CommandData.ISCState[1].offset = 0;
  CommandData.ISCControl[1].save_period = 4000; /* 40 sec */
  CommandData.ISCControl[0].pulse_width = 3125; /* 300.00 msec */
  CommandData.ISCControl[0].fast_pulse_width = 625; /* 60.00 msec */
  CommandData.ADC_sync_timeout = 0;

  WritePrevStatus();
}
