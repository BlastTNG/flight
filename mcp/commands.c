/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2005 University of Toronto
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

/* Define this symbol to have mcp log all SIP traffic */
#undef SIP_CHATTER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "command_list.h"
#include "command_struct.h"
#include "mcp.h"
#include "tx.h"
#include "pointing_struct.h"
#include "slow_dl.h"
#include "channels.h"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
#define LOCK_OFFSET (3.45)

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

/* based on isc_protocol.h */
#define ISC_SHUTDOWN_NONE     0
#define ISC_SHUTDOWN_HALT     1
#define ISC_SHUTDOWN_REBOOT   2
#define ISC_SHUTDOWN_CAMCYCLE 3

#define ISC_TRIGGER_INT  0
#define ISC_TRIGGER_EDGE 1
#define ISC_TRIGGER_POS  2
#define ISC_TRIGGER_NEG  3

void SetRaDec(double ra, double dec); /* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double az, double el);
void NormalizeAngle(double *A);

const char UnknownCommand[] = "Unknown Command";

extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];

extern int doing_schedule; /* sched.c */

extern pthread_t watchdog_id;  /* mcp.c */
pthread_mutex_t mutex;

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus() {
  int fp, n;

  /** write the default file */
  fp = open("/tmp/mcp.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "Commands: mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "Commands: mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "Commands: mcp.prev_status close()");
    return;
  }
}

void bc_close() {
}

int bc_setserial(char *input_tty) {
  int fd;
  struct termios term; 

  if ((fd = open(input_tty, O_RDWR)) < 0)
    berror(tfatal, "Commands: Unable to open serial port");

  if (tcgetattr(fd, &term))
    berror(tfatal, "Commands: Unable to get serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if(cfsetospeed(&term, B1200))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "Commands: Error setting serial output speed");

  if(cfsetispeed(&term, B1200))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "Commands: Error setting serial input speed");

  if( tcsetattr(fd, TCSANOW, &term) )
    berror(tfatal, "Commands: Unable to set serial attributes");

  return fd;
}

/* calculate the nearest lockable elevation */
double LockPosition (double elevation)
{
  double position;

  position = floor(elevation / 10.0) * 10.0 + 5.0;
  if (position > 79.0) {
    position = 90.0;
  } else if (position < 10.0) {
    position = 5.0;
  }

  return position + LOCK_OFFSET;
}

float ParseGPS (unsigned char *data)
{
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

  exponent = ((*(data + 3) << 1) & 0xFF) + ((*(data + 2) >> 7)  & 0x01) - 127;
  /* Next eight bits = exponent + 127 */

  /* Mantissa contained in last 23 bits */
  mantissa_bits = ((*(data + 2) & 0x7F) << 16) + (*(data + 1) << 8) + *data;

  for (i = 23; i >= 0; i--) {           /* Construct mantissa = Sigma 2^n */
    if ((mantissa_bits >> i) & 0x01)
      mantissa += pow(2, i - 23);
  }

  return((mantissa + 1) * pow(2, exponent) * sign);
}

void SendRequest (int req, char tty_fd)
{
  unsigned char buffer[3];

  buffer[0] = 0x10;
  buffer[1] = req;
  buffer[2] = 0x03;

  write(tty_fd, buffer, 3);
}

enum singleCommand SCommand(char *cmd)
{
  int i;

  for (i = 0; i < N_SCOMMANDS; i++) {
    if (strcmp(scommands[i].name, cmd) == 0)
      return scommands[i].command;
  }

  return -1;
}

int SIndex(enum singleCommand command)
{
  int i;

  for (i = 0; i < N_SCOMMANDS; i++)
    if (scommands[i].command == command)
      return i;

  return -1;
}

const char* SName(enum singleCommand command)
{
  int i = SIndex(command);
  return (i == -1) ? UnknownCommand : scommands[i].name;
}

void SingleCommand (enum singleCommand command, int scheduled)
{
#ifndef BOLOTEST
  int i_point = GETREADINDEX(point_index);
  double sun_az;
#endif

  if (!scheduled)
    bprintf(info, "Commands: Single command: %d (%s)\n", command,
        SName(command));

  /* Update CommandData structure with new info */

  switch (command) {
#ifndef BOLOTEST
    case stop: /* Pointing abort */
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case antisun: /* turn antisolar (az-only) */
      sun_az = PointingData[i_point].sun_az + 180;
      NormalizeAngle(&sun_az);

      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.X = sun_az;  /* az */
      CommandData.pointing_mode.Y = PointingData[i_point].el;  /* el */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;

    case trim_to_isc:
      SetTrimToSC(0);
      break;
    case trim_to_osc:
      SetTrimToSC(1);
      break;
    case reset_trims:
      ClearTrim();
      break;
    case az_auto_gyro:
      CommandData.az_autogyro = 1;
      break;
    case el_auto_gyro:
      CommandData.el_autogyro = 1;
      break;

    case az_off:/* disable az motors */
      CommandData.disable_az = 1;
      break;
    case az_on:/* enable az motors */
      CommandData.disable_az = 0;
      break;
    case el_off: /* disable el motors */
      CommandData.disable_el = 1;
      CommandData.force_el = 0;
      break;
    case el_on: /* enable el motors */
      CommandData.disable_el = 0;
      CommandData.force_el = 0;
      break;
    case force_el_on: /* force enabling of el motors */
      CommandData.disable_el = 0;
      CommandData.force_el = 1;
      break;

    case sun_veto:      /* Veto sensors */
      CommandData.use_sun = 0;
      break;
    case isc_veto:
      CommandData.use_isc = 0;
      break;
    case osc_veto:
      CommandData.use_osc = 0;
      break;
    case mag_veto:
      CommandData.use_mag = 0;
      break;
    case gps_veto:
      CommandData.use_gps = 0;
      break;
    case elenc_veto:
      CommandData.use_elenc = 0;
      break;
    case elclin_veto:
      CommandData.use_elclin = 0;
      break;

    case sun_allow:      /* Un-veto sensors */
      CommandData.use_sun = 1;
      break;
    case isc_allow:
      CommandData.use_isc = 1;
      break;
    case osc_allow:
      CommandData.use_osc = 1;
      break;
    case mag_allow:
      CommandData.use_mag = 1;
      break;
    case gps_allow:
      CommandData.use_gps = 1;
      break;
    case elenc_allow:
      CommandData.use_elenc = 1;
      break;
    case elclin_allow:
      CommandData.use_elclin = 1;
      break;

    case gps_off:          /* sensor power */
      CommandData.sensors_off.gps = 1;
      break;
    case gps_on:
      CommandData.sensors_off.gps = 0;
      break;
    case gyro_off:
      CommandData.sensors_off.gyro = 1;
      break;
    case gyro_on:
      CommandData.sensors_off.gyro = 0;
      break;
    case isc_off:
      CommandData.sensors_off.isc = 1;
      break;
    case isc_on:
      CommandData.sensors_off.isc = 0;
      break;
    case osc_off:
      CommandData.sensors_off.osc = 1;
      break;
    case osc_on:
      CommandData.sensors_off.osc = 0;
      break;
    case ss_off:
      CommandData.sensors_off.ss = 1;
      break;
    case ss_on:
      CommandData.sensors_off.ss = 0;
      break;

    case analogue_gyros:   /* gyro selection */
      CommandData.use_analogue_gyros = 1;
      CommandData.fast_gy_offset = 3000;
      break;
    case digital_gyros:
      CommandData.use_analogue_gyros = 0;
      CommandData.fast_gy_offset = 3000;
      break;
#endif

    case clock_int:   /* Bias settings */
      CommandData.Bias.clockInternal = 1;
      break;
    case clock_ext:
      CommandData.Bias.clockInternal = 0;
      break;
    case bias_ac:
      CommandData.Bias.biasAC = 1;
      break;
    case bias_dc:
      CommandData.Bias.biasAC = 0;
      break;
    case ramp:
      CommandData.Bias.biasRamp = 1;
      break;
    case fixed:
      CommandData.Bias.biasRamp = 0;
      break;

    case level_on:   /* Cryo commanding */
      CommandData.Cryo.heliumLevel = 1;
      break;
    case level_off:
      CommandData.Cryo.heliumLevel = 0;
      break;
    case charcoal_on:
      CommandData.Cryo.charcoalHeater = 1;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case charcoal_off:
      CommandData.Cryo.charcoalHeater = 0;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case auto_cycle:
      CommandData.Cryo.fridgeCycle = 1;
      CommandData.Cryo.force_cycle = 0;
      break;
    case fridge_cycle:
      CommandData.Cryo.fridgeCycle = 1;
      CommandData.Cryo.force_cycle = 1;
      break;
    case coldplate_on:
      CommandData.Cryo.coldPlate = 1;
      break;
    case coldplate_off:
      CommandData.Cryo.coldPlate = 0;
      break;
    case cal_on:
      CommandData.Cryo.calibrator = on;
      break;
    case cal_off:
      CommandData.Cryo.calibrator = off;
      break;
    case pot_valve_open:
      CommandData.Cryo.potvalve_open = 40;
      CommandData.Cryo.potvalve_close = 0;
      break;
    case pot_valve_close:
      CommandData.Cryo.potvalve_close = 40;
      CommandData.Cryo.potvalve_open = 0;
      break;
    case pot_valve_on:
      CommandData.Cryo.potvalve_on = 1;
      break;
    case pot_valve_off:
      CommandData.Cryo.potvalve_on = 0;
      break;
    case l_valve_open:
      CommandData.Cryo.lvalve_open = 40;
      CommandData.Cryo.lvalve_close = 0;
      break;
    case l_valve_close:
      CommandData.Cryo.lvalve_close = 40;
      CommandData.Cryo.lvalve_open = 0;
      break;
    case he_valve_on:
      CommandData.Cryo.lhevalve_on = 1;
      break;
    case he_valve_off:
      CommandData.Cryo.lhevalve_on = 0;
      break;
    case ln_valve_on:
      CommandData.Cryo.lnvalve_on = 1;
      break;
    case ln_valve_off:
      CommandData.Cryo.lnvalve_on = 0;
      break;
    case auto_bdaheat:
      CommandData.Cryo.autoBDAHeat = 1;
      break;
    case auto_jfetheat:
      CommandData.Cryo.autoJFETheat = 1;
      break;

#ifndef BOLOTEST
    case balance_veto:/* Balance/pump commanding */
      CommandData.pumps.bal_veto = -1;
      break;
    case balance_allow:
      CommandData.pumps.bal_veto = 1;
      break;

    case balpump_on:
      CommandData.pumps.bal1_on = 1;
      break;
    case balpump_off:
      CommandData.pumps.bal1_on = 0;
      break;
    case balpump_up:
      CommandData.pumps.bal1_reverse = 1;
      break;
    case balpump_down:
      CommandData.pumps.bal1_reverse = 0;
      break;
    case sprpump_on:
      CommandData.pumps.bal2_on = 1;
      break;
    case sprpump_off:
      CommandData.pumps.bal2_on = 0;
      break;
    case sprpump_fwd:
      CommandData.pumps.bal2_reverse = 0;
      break;
    case sprpump_rev:
      CommandData.pumps.bal2_reverse = 1;
      break;

    case inner_cool_on:
      CommandData.pumps.inframe_cool_on = 40;
      CommandData.pumps.inframe_auto = 0;
      break;
    case inner_cool_off:
      CommandData.pumps.inframe_cool_off = 40;
      CommandData.pumps.inframe_auto = 0;
      break;
    case inner_cool_auto:
      CommandData.pumps.inframe_auto = 1;
      break;

    case outer_cool_on:
      CommandData.pumps.outframe_cool1_on = 40;
      CommandData.pumps.outframe_auto = 0;
      break;
    case outer_cool_off:
      CommandData.pumps.outframe_cool1_off = 40;
      CommandData.pumps.outframe_auto = 0;
      break;
    case outer_cool_auto:
      CommandData.pumps.outframe_auto = 1;
      break;

    case outer_spare_on:
      CommandData.pumps.outframe_cool2_on = 40;
      break;
    case outer_spare_off:
      CommandData.pumps.outframe_cool2_off = 40;
      break;
    case pin_in:
      CommandData.pumps.lock_in = 1;
      break;
    case lock_off:
      CommandData.pumps.lock_off = 1;
      break;
    case unlock:
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
      break;
    case isc_run:
      CommandData.ISCState[0].pause = 0;
      break;
    case isc_shutdown:
      CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_HALT;
      break;
    case isc_reboot:
      CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_REBOOT;
      break;
    case isc_cam_cycle:
      CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_CAMCYCLE;
      break;
    case isc_pause:
      CommandData.ISCState[0].pause = 1;
      break;
    case isc_abort:
      CommandData.ISCState[0].abort = 1;
      break;
    case isc_reconnect:
      CommandData.ISCControl[0].reconnect = 1;
      break;
    case isc_save_images:
      CommandData.ISCState[0].save = 1;
      break;
    case isc_discard_images:
      CommandData.ISCState[0].save = 0;
      break;
    case isc_full_screen:
      CommandData.ISCState[0].display_mode = full;
      break;
    case isc_trig_int:
      CommandData.ISCState[0].triggertype = ISC_TRIGGER_INT;
      break;
    case isc_trig_ext:
      CommandData.ISCState[0].triggertype = ISC_TRIGGER_NEG;
      break;
    case isc_auto_focus:
      CommandData.ISCControl[0].autofocus = 10;
      break;
    case isc_eye_on:
      CommandData.ISCState[0].eyeOn = 1;
      break;
    case isc_eye_off:
      CommandData.ISCState[0].eyeOn = 0;

      /***************************************/
      /********* OSC Commanding  *************/
      break;
    case osc_run:
      CommandData.ISCState[1].pause = 0;
      break;
    case osc_shutdown:
      CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_HALT;
      break;
    case osc_reboot:
      CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_REBOOT;
      break;
    case osc_cam_cycle:
      CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_CAMCYCLE;
      break;
    case osc_pause:
      CommandData.ISCState[1].pause = 1;
      break;
    case osc_abort:
      CommandData.ISCState[1].abort = 1;
      break;
    case osc_reconnect:
      CommandData.ISCControl[1].reconnect = 1;
      break;
    case osc_save_images:
      CommandData.ISCState[1].save = 1;
      break;
    case osc_discard_images:
      CommandData.ISCState[1].save = 0;
      break;
    case osc_full_screen:
      CommandData.ISCState[1].display_mode = full;
      break;
    case osc_trig_int:
      CommandData.ISCState[1].triggertype = ISC_TRIGGER_INT;
      break;
    case osc_trig_ext:
      CommandData.ISCState[1].triggertype = ISC_TRIGGER_NEG;
      break;
    case osc_auto_focus:
      CommandData.ISCControl[1].autofocus = 10;
      break;
    case osc_eye_on:
      CommandData.ISCState[1].eyeOn = 1;
      break;
    case osc_eye_off:
      CommandData.ISCState[1].eyeOn = 0;
      break;

    case blast_rocks:
      CommandData.sucks = 0;
      break;
    case blast_sucks:
      CommandData.sucks = 1;
      break;

    case at_float:
      CommandData.at_float = 1;
      break;
    case not_at_float:
      CommandData.at_float = 0;
      break;

    case reap:  /* Miscellaneous commands */
      bprintf(err, "Commands: Reaping the watchdog tickle on command.");
      pthread_cancel(watchdog_id);
#endif
      break;
    case mcc_halt:
      bputs(warning, "Commands: Halting the MCC\n");
      system("/sbin/halt");
    case xyzzy:
      break;
    default:
      bputs(warning, "Commands: ***Invalid Single Word Command***\n");
      return; /* invalid command - no write or update */
  }

#ifndef BOLOTEST
  if (!scheduled) {
    if (doing_schedule)
      bprintf(info, "Scheduler: *** Out of schedule file mode ***");
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  } else
    CommandData.pointing_mode.t = PointingData[i_point].t;
#endif

  WritePrevStatus();
}

enum multiCommand MCommand(char *cmd)
{
  int i;

  for (i = 0; i < N_MCOMMANDS; i++) {
    if (strcmp(mcommands[i].name, cmd) == 0)
      return mcommands[i].command;
  }

  return -1;
}

int MIndex(enum multiCommand command)
{
  int i;

  for (i = 0; i < N_MCOMMANDS; i++)
    if (mcommands[i].command == command)
      return i;

  return -1;
}

const char* MName(enum multiCommand command)
{
  int i = MIndex(command);
  return (i == -1) ? UnknownCommand : mcommands[i].name;
}

void SetParameters(enum multiCommand command, unsigned short *dataq,
    double* rvalues, int* ivalues)
{
  int i, dataqind;
  char type;
  int index = MIndex(command);

#ifndef USE_FIFO_CMD
  double min;

  /* compute renormalised values */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    min = mcommands[index].params[i].min;
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++] + mcommands[index].params[i].min;
      bprintf(info, "Commands: param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = (float)dataq[dataqind++] * (mcommands[index].params[i].max
          - min) / MAX_15BIT + min;
      bprintf(info, "Commands: param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = (float)((int)dataq[dataqind++] << 15); /* upper 15 bits */
      rvalues[i] += (float)dataq[dataqind++];             /* lower 15 bits */
      rvalues[i] = rvalues[i] * (mcommands[index].params[i].max - min) /
        MAX_30BIT + min;
      bprintf(info, "Commands: param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }
#else
  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: 15 bits: %f\n", i, rvalues[i]);
    } else if (type == 'l') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: 30 bits: %f\n", i, rvalues[i]);
    }
  }

  bprintf(info, "Commands: Multiword Command: %d (%s)\n", command,
      MName(command));
#endif
}

void MultiCommand(enum multiCommand command, double *rvalues, int *ivalues,
    int scheduled)
{
#ifndef BOLOTEST
  int i;
#endif

  /* Update CommandData struct with new info
   * If the parameter is type 'i'     set CommandData using ivalues[i]
   * If the parameter is type 'f'/'l' set CommandData using rvalues[i]
   */

  /* Pointing Modes */
  switch(command) {
#ifndef BOLOTEST
    case az_el_goto:
      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case az_scan:
      CommandData.pointing_mode.mode = P_AZ_SCAN;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.w = rvalues[2];  /* width */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.h = 0;
      break;
    case drift:
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = rvalues[0]; /* az speed */
      CommandData.pointing_mode.del = rvalues[1]; /* el speed */
      CommandData.pointing_mode.h = 0;
      break;
    case ra_dec_goto:
      CommandData.pointing_mode.mode = P_RADEC_GOTO;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case vcap:
      CommandData.pointing_mode.mode = P_VCAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el drift speed */
      CommandData.pointing_mode.h = 0;
      break;
    case cap:
      CommandData.pointing_mode.mode = P_CAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el step size */
      CommandData.pointing_mode.h = 0;
      break;
    case box:
      CommandData.pointing_mode.mode = P_BOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el step size */
      break;
    case vbox:
      CommandData.pointing_mode.mode = P_VBOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el drift speed */
      break;
    case quad:
      CommandData.pointing_mode.mode = P_QUAD;
      CommandData.pointing_mode.ra[0] = rvalues[0];
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i * 2];
        CommandData.pointing_mode.dec[i] = rvalues[i * 2 + 1];
      }
      CommandData.pointing_mode.vaz = rvalues[8]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[9]; /* el step size */
      break;

      /***************************************/
      /********** Pointing Motor Trims *******/
    case az_el_trim:
      AzElTrim(rvalues[0], rvalues[1]);
      break;
    case ra_dec_set:
      SetRaDec(rvalues[0], rvalues[1]);
      break;
    case az_gyro_offset:
      CommandData.gy2_offset = rvalues[0];
      CommandData.gy3_offset = rvalues[1];
      CommandData.az_autogyro = 0;
      break;
    case el_gyro_offset:
      CommandData.gy1_offset = rvalues[0];
      CommandData.el_autogyro = 0;
      break;

      /***************************************/
      /********** Pointing Motor Gains *******/
    case roll_gain:/* roll Gains */
      CommandData.roll_gain.P = ivalues[0];
      break;
    case el_gain:  /* ele gains */
      CommandData.ele_gain.P = ivalues[0];
      CommandData.ele_gain.I = ivalues[1];
      break;
    case az_gain:  /* az gains */
      CommandData.azi_gain.P = ivalues[0];
      CommandData.azi_gain.I = ivalues[1];
      break;
    case pivot_gain:  /* pivot gains */
      CommandData.pivot_gain.SP = (rvalues[0] + 2.605) / 7.9498291016e-5;
      CommandData.pivot_gain.P = ivalues[1];
      break;
    case back_emf:
      CommandData.emf_gain = rvalues[0] * 6500;
      CommandData.emf_offset = rvalues[1] * 500;
      break;

      /***************************************/
      /********** Inner Frame Lock  **********/
    case lock:  /* Lock Inner Frame */
      if (CommandData.pumps.bal_veto >= 0)
        CommandData.pumps.bal_veto = BAL_VETO_MAX;
      CommandData.pumps.lock_point = 1;
      CommandData.pointing_mode.mode = P_LOCK;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = LockPosition(rvalues[0]);
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      bprintf(info, "Commands: Lock Mode: %g\n", CommandData.pointing_mode.Y);
      break;

      /***************************************/
      /********** Balance System  ************/
    case setpoints:
      CommandData.pumps.bal_on = rvalues[0] * 1648.;
      CommandData.pumps.bal_off = rvalues[1] * 1648.;
      CommandData.pumps.bal_target = rvalues[2] * 1648.;
      break;
    case bal_level:
      CommandData.pumps.pwm1 = 2047 - rvalues[0] * 2047. / 100;
      break;
    case bal_gain:
      CommandData.pumps.bal_gain = rvalues[0];
      break;

      /***************************************/
      /********** Cooling System  ************/
    case spare_level:
      CommandData.pumps.pwm2 = 2047 - rvalues[0] * 2047. / 100;
      break;
    case inner_level:
      CommandData.pumps.pwm3 = 2047 - rvalues[0] * 2047. / 100;
      break;
    case outer_level:
      CommandData.pumps.pwm4 = 2047 - rvalues[0] * 2047. / 100;
      break;

      /***************************************/
      /******** Electronics Heaters  *********/
    case t_gyro1_set:  /* gyro heater setpoint */
      CommandData.gyheat[0].setpoint = rvalues[0];
      CommandData.gyheat[0].age = 0;
      break;
    case t_gyro2_set:  /* gyro heater setpoint */
      CommandData.gyheat[1].setpoint = rvalues[0];
      CommandData.gyheat[1].age = 0;
      break;
    case t_gyro1_gain:  /* gyro heater gains */
      CommandData.gyheat[0].gain.P = ivalues[0];
      CommandData.gyheat[0].gain.I = ivalues[1];
      CommandData.gyheat[0].gain.D = ivalues[2];
      break;
    case t_gyro2_gain:  /* gyro heater gains */
      CommandData.gyheat[1].gain.P = ivalues[0];
      CommandData.gyheat[1].gain.I = ivalues[1];
      CommandData.gyheat[1].gain.D = ivalues[2];
      break;

      /***************************************/
      /*************** Misc  *****************/
    case timeout:       /* Set timeout */
      CommandData.timeout = ivalues[0];
      break;
    case alice_file: /* change downlink XML file */
      CommandData.alice_file = ivalues[0];
      break;
    case plugh:/* A hollow voice says "Plugh". */
      CommandData.plover = ivalues[0];
      break;
    case apcu_charge:
      CommandData.apcu_reg = rvalues[0]; // v_topoff, in V
      CommandData.apcu_auto = 0;
      break;
    case dpcu_charge:
      CommandData.dpcu_reg = rvalues[0]; // v_topoff, in V
      CommandData.dpcu_auto = 0;
      break;
    case auto_apcu:
      CommandData.apcu_trim = rvalues[0];
      CommandData.apcu_auto = 1;
      break;
    case auto_dpcu:
      CommandData.dpcu_trim = rvalues[0];
      CommandData.dpcu_auto = 1;
      break;
#endif

      /***************************************/
      /*************** Bias  *****************/
    case bias1_level:    /* Set bias 1 */
      CommandData.Bias.SetLevel1 = 1;
      CommandData.Bias.bias1 = ivalues[0];
      break;
    case bias2_level:   /* Set bias 2 */
      CommandData.Bias.SetLevel2 = 1;
      CommandData.Bias.bias2 = ivalues[0];
      break;
    case bias3_level:   /* Set bias 3 */
      CommandData.Bias.SetLevel3 = 1;
      CommandData.Bias.bias3 = ivalues[0];
      break;
    case phase:
      if (ivalues[0] >= 5 && ivalues[0] <= 16) 
        CommandData.Phase[ivalues[0] - 5] = ivalues[1];

      /***************************************/
      /*********** Cal Lamp  *****************/
      break;
    case cal_pulse:
      CommandData.Cryo.calibrator = pulse;
      CommandData.Cryo.calib_pulse = ivalues[0] / 10;
      break;
    case cal_repeat:
      CommandData.Cryo.calibrator = repeat;
      CommandData.Cryo.calib_pulse = ivalues[0] / 10;
      CommandData.Cryo.calib_period = ivalues[1] * 5;
      break;

      /***************************************/
      /********* Cryo heat   *****************/
    case jfet_heat:
      CommandData.Cryo.JFETHeat = rvalues[0] * 2047./100.;
      CommandData.Cryo.autoJFETheat = 0;
      break;
    case jfet_set:
      CommandData.Cryo.JFETSetOn = rvalues[0];
      CommandData.Cryo.JFETSetOff = rvalues[1];
      break;
    case heatsw_heat:
      CommandData.Cryo.heatSwitch = rvalues[0] * 2047./100.;
      break;
    case cryo_heat:
      CommandData.Cryo.CryoSparePWM = rvalues[0] * 2047./100.;
      break;
    case bda_heat:
      CommandData.Cryo.BDAHeat = rvalues[0] * 2047./100.;
      CommandData.Cryo.autoBDAHeat = 0;
      break;
    case bda_gain:
      CommandData.Cryo.BDAGain.P = ivalues[0];
      CommandData.Cryo.BDAGain.I = ivalues[1];
      CommandData.Cryo.BDAGain.D = ivalues[2];
      CommandData.Cryo.BDAFiltLen = ivalues[3];
      break;
    case bda_set:
      CommandData.Cryo.BDAGain.SP = ivalues[0];
      break;

#ifndef BOLOTEST
      /***************************************/
      /********* ISC Commanding  *************/
    case isc_set_focus:
      CommandData.ISCState[0].focus_pos = ivalues[0];
      break;
    case isc_foc_off:
      CommandData.ISCState[0].focusOffset = ivalues[0];
      break;
    case isc_set_aperture:
      CommandData.ISCState[0].ap_pos = ivalues[0];
      break;
    case isc_pixel_centre:
      CommandData.ISCState[0].roi_x = ivalues[0];
      CommandData.ISCState[0].roi_y = ivalues[1];
      CommandData.ISCState[0].display_mode = roi;
      break;
    case isc_blob_centre:
      CommandData.ISCState[0].blob_num = ivalues[0];
      CommandData.ISCState[0].display_mode = blob;
      break;
    case isc_offset:
      CommandData.ISCState[0].azBDA = rvalues[0] * DEG2RAD;
      CommandData.ISCState[0].elBDA = rvalues[1] * DEG2RAD;
      break;
    case isc_integrate:
      CommandData.ISCControl[0].fast_pulse_width = rvalues[0] / 10.;
      CommandData.ISCControl[0].pulse_width = rvalues[1] / 10.;
      break;
    case isc_det_set:
      CommandData.ISCState[0].grid = ivalues[0];
      CommandData.ISCState[0].sn_threshold = rvalues[1];
      CommandData.ISCState[0].mult_dist = ivalues[2];
      break;
    case isc_blobs:
      CommandData.ISCState[0].minBlobMatch = ivalues[0];
      CommandData.ISCState[0].maxBlobMatch = ivalues[1];
      break;
    case isc_catalogue:
      CommandData.ISCState[0].mag_limit = rvalues[0];
      CommandData.ISCState[0].norm_radius = rvalues[1] * DEG2RAD;
      CommandData.ISCState[0].lost_radius = rvalues[2] * DEG2RAD;
      break;
    case isc_tolerances:
      CommandData.ISCState[0].tolerance = rvalues[0] / 3600. * DEG2RAD;
      CommandData.ISCState[0].match_tol = rvalues[1] / 100;
      CommandData.ISCState[0].quit_tol = rvalues[2] / 100;
      CommandData.ISCState[0].rot_tol = rvalues[3] * DEG2RAD;
      break;
    case isc_hold_current:
      CommandData.ISCState[0].hold_current = ivalues[0];
      break;
    case isc_save_period:
      CommandData.ISCControl[0].save_period = ivalues[0] * 100;
      break;
    case isc_gain:
      CommandData.ISCState[0].gain = rvalues[0];
      CommandData.ISCState[0].offset = ivalues[1];
      break;

      /***************************************/
      /********* OSC Commanding  *************/
    case osc_set_focus:
      CommandData.ISCState[1].focus_pos = ivalues[0];
      break;
    case osc_foc_off:
      CommandData.ISCState[1].focusOffset = ivalues[0];
      break;
    case osc_set_aperture:
      CommandData.ISCState[1].ap_pos = ivalues[0];
      break;
    case osc_pixel_centre:
      CommandData.ISCState[1].roi_x = ivalues[0];
      CommandData.ISCState[1].roi_y = ivalues[1];
      CommandData.ISCState[1].display_mode = roi;
      break;
    case osc_blob_centre:
      CommandData.ISCState[1].blob_num = ivalues[0];
      CommandData.ISCState[1].display_mode = blob;
      break;
    case osc_offset:
      CommandData.ISCState[1].azBDA = rvalues[0] * DEG2RAD;
      CommandData.ISCState[1].elBDA = rvalues[1] * DEG2RAD;
      break;
    case osc_integrate:
      CommandData.ISCControl[1].fast_pulse_width = rvalues[0] / 10.;
      CommandData.ISCControl[1].pulse_width = rvalues[1] / 10.;
      break;
    case osc_det_set:
      CommandData.ISCState[1].grid = ivalues[0];
      CommandData.ISCState[1].sn_threshold = rvalues[1];
      CommandData.ISCState[1].mult_dist = ivalues[2];
      break;
    case osc_blobs:
      CommandData.ISCState[1].minBlobMatch = ivalues[0];
      CommandData.ISCState[1].maxBlobMatch = ivalues[1];
      break;
    case osc_catalogue:
      CommandData.ISCState[1].mag_limit = rvalues[0];
      CommandData.ISCState[1].norm_radius = rvalues[1] * DEG2RAD;
      CommandData.ISCState[1].lost_radius = rvalues[2] * DEG2RAD;
      break;
    case osc_tolerances:
      CommandData.ISCState[1].tolerance = rvalues[0] / 3600. * DEG2RAD;
      CommandData.ISCState[1].match_tol = rvalues[1] / 100;
      CommandData.ISCState[1].quit_tol = rvalues[2] / 100;
      CommandData.ISCState[1].rot_tol = rvalues[3] * DEG2RAD;
      break;
    case osc_hold_current:
      CommandData.ISCState[1].hold_current = ivalues[0];
      break;
    case osc_save_period:
      CommandData.ISCControl[1].save_period = ivalues[0] * 100;
      break;
    case osc_gain:
      CommandData.ISCState[1].gain = rvalues[0];
      CommandData.ISCState[1].offset = ivalues[1];
      break;
#endif
    default:
      bputs(warning, "Commands: Invalid Multi Word Command***\n");
      return; /* invalid command - don't update */
  }

#ifndef BOLOTEST
  int i_point = GETREADINDEX(point_index);

  if (!scheduled)
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  else
    CommandData.pointing_mode.t = PointingData[i_point].t;
#endif

  WritePrevStatus();
}

void GPSPosition (unsigned char *indata) {
  /* Send new information to CommandData */

  SIPData.GPSpos.lon = -ParseGPS(indata); /* sip sends east lon */
  SIPData.GPSpos.lat = ParseGPS(indata + 4);
  SIPData.GPSpos.alt = ParseGPS(indata + 8);
  SIPData.GPSstatus1 = *(indata + 12);
  SIPData.GPSstatus2 = *(indata + 13);

  WritePrevStatus();
}

const char* CommandName(int is_multi, int command)
{
  return (is_multi) ? MName(command) : SName(command);
}

void ScheduledCommand(struct ScheduleEvent *event)
{

  if (event->is_multi) {
    int i;
    int index = MIndex(event->command);

    bprintf(info, "Commands: Executing Scheduled Command: %i (%s)\n",
        event->command, MName(event->command));
    for (i = 0; i < mcommands[index].numparams; ++i) {
      int type = mcommands[index].params[i].type;
      if (type == 'i') /* 15 bit unsigned integer */
        bprintf(info, "Commands:   param%02i: integer: %i\n", i,
            event->ivalues[i]);
      else if (type == 'f') /* 15 bit floating point */
        bprintf(info, "Commands:   param%02i: 15 bits: %f\n", i,
            event->rvalues[i]);
      else if (type == 'l') /* 30 bit floating point */
        bprintf(info, "Commands:   param%02i: 30 bits: %f\n", i,
            event->rvalues[i]);
    }
    MultiCommand(event->command, event->rvalues, event->ivalues, 1);

  } else {
    bprintf(info, "Commands: Executing Scheduled Command: %i (%s)\n",
        event->command, SName(event->command));
    SingleCommand(event->command, 1);
  }
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

  SIPData.MKSalt.hi = ((unsigned short *)indata)[0];;
  SIPData.MKSalt.med = ((unsigned short *)indata)[1];;
  SIPData.MKSalt.lo = ((unsigned short *)indata)[2];;

  WritePrevStatus();
}

#ifndef BOLOTEST
/* Send TDRSS Low Rate Packet */

void SendDownData(char tty_fd) {
  unsigned char buffer[SLOWDL_LEN], data[3 + SLOWDL_LEN + 1];
  int i, temp;
  int bitpos, bytepos, numbits;
  double slowM, slowB;
  static char firsttime;

  bitpos = 0;
  bytepos = 0;
  memset(data, 0, SLOWDL_LEN);

  for (i = 0; i < SLOWDL_NUM_DATA; i++) {
    switch (SlowDLInfo[i].type) {
      case SLOWDL_FORCE_INT:
        /* Round value to an integer and try to fit it in numbits */
        numbits = SlowDLInfo[i].numbits; 
        slowM = (double)((1 << (numbits - 1)) - 1) / 
          (SlowDLInfo[i].max - SlowDLInfo[i].min); 
        slowB = - slowM * (double)SlowDLInfo[i].min;
        if ((int)SlowDLInfo[i].value > SlowDLInfo[i].max)
          temp = (int)(slowM * SlowDLInfo[i].max + slowB);
        else if ((int)SlowDLInfo[i].value < SlowDLInfo[i].min)
          temp = 0;
        else
          temp = (int)(slowM * SlowDLInfo[i].value + slowB);
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
    //bprintf(info, "%g %ld %ld %x %s\n", SlowDLInfo[i].value, SlowDLInfo[i].max,
    //	   SlowDLInfo[i].min, temp, SlowDLInfo[i].src);

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
      bprintf(warning, "Low Rate: Slow DL size is larger than maximum size "
          "of %d.  Reduce length of SlowDLInfo structure.", SLOWDL_LEN);
      break;
    }
  }

  if (firsttime) {
    bprintf(info, "Low Rate: Slow DL size = %d\n", bytepos);
    firsttime = 0;
  }

  buffer[0] = SLOWDL_DLE;
  buffer[1] = SLOWDL_SYNC;
  buffer[2] = SLOWDL_LEN;
  memcpy(buffer + 3, data, SLOWDL_LEN);
  buffer[3 + SLOWDL_LEN] = SLOWDL_ETX;

  write(tty_fd, buffer, 3 + SLOWDL_LEN + 1);
#if 0
  for (i=0; i<3 + SLOWDL_LEN + 1; i++) {
    bprintf(info, "%d %2x", i, buffer[i]);
  }
#endif
}
#endif

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

  int i;
  for (i = 0; i < DATA_Q_SIZE; ++i) {
    mcommand_data[i] = NULL;
  }

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];

  int index, pindex = 0;

  bputs(startup, "Commands: WatchFIFO startup\n");

  if ((fifo = open("/tmp/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
    berror(tfatal, "Commands: Unable to open FIFO");

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    bprintf(info, "Commands: Command received: %s\n", command);
    index = -1;
    while((command[++index] != ' ') && command[index]);
    command[index++] = 0;

    pindex = 0;
    mcommand_count = 0;
    do {
      if ((command[index] == ' ' || command[index] == 0) && pindex > 0) {
        pbuf[pindex] = 0;
        mcommand_data[mcommand_count] =
          reballoc(tfatal, mcommand_data[mcommand_count], pindex + 2);

        strncpy(mcommand_data[mcommand_count++], pbuf, pindex + 1);
        pindex = 0;
      } else {
        pbuf[pindex++] = command[index];
      }
    } while (command[index++] != 0);
    bprintf(info, "Commands: %i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SCommand(command);
      SingleCommand(mcommand, 0);
      mcommand = -1;
    } else {
      mcommand = MCommand(command);
      bputs(info, "Commands:  Multi word command received\n");
      SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues, ivalues);
      MultiCommand(mcommand, rvalues, ivalues, 0);
      mcommand = -1;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);

  }
}

char *COMM[] = {"/dev/ttyS0", "/dev/ttyS4"};

#ifndef BOLOTEST
void WatchPort (void* parameter) {
  unsigned char buf;
  unsigned short *indatadumper;
  unsigned char indata[20];
  int readstage = 0;
  int tty_fd;

  int port = (int)parameter;

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];

  int mcommand = -1;
  int mcommand_count = 0;
  int dataqsize = 0;
  unsigned short mcommand_data[DATA_Q_SIZE];
  unsigned char mcommand_time = 0;

  int timer = 0;
  int bytecount = 0;

  bprintf(startup, "Commands: WatchPort(%i) startup\n", port);

  tty_fd = bc_setserial(COMM[port]);

  for(;;) {
    /* Loop until data come in */
    while (read(tty_fd, &buf, 1) <= 0) {
      timer++;
      /** Request updated info every 50 seconds */
      if (timer == 800) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_POSITION, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Commands: COMM%i: Request SIP Position\n", port + 1);
#endif
        pthread_mutex_unlock(&mutex);
      } else if (timer == 1700) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_TIME, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Commands: COMM%i: Request SIP Time\n", port + 1);
#endif
        pthread_mutex_unlock(&mutex);	
      } else if (timer > 2500) { 
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_ALTITUDE, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Commands: COMM%i: Request SIP Altitude\n", port + 1);
#endif
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
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: Data request\n", port + 1);
#endif
        } else if (buf == 0x14) { /* Command */
          readstage = 2;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: Command\n", port + 1);
#endif
        } else if (buf == 0x10) { /* GPS Position */
          readstage = 4;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: GPS Position\n", port + 1);
#endif
        } else if (buf == 0x11) { /* GPS Time */
          readstage = 5;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: GPS Time\n", port + 1);
#endif
        } else if (buf == 0x12) { /* MKS Altitude */
          readstage = 6;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: MKS Altitude\n", port + 1);
#endif
        } else {
          bprintf(warning, "Commands: COMM%i: Bad packet received: "
              "Unrecognised Packet Type: %02X\n", port + 1, buf);
          readstage = 0;
        }
        break;
      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf == 0x02)
            bytecount = 1;
          else {
            readstage = 0;
            bprintf(warning, "Commands: COMM%i: Bad command packet: "
                "Unsupported Length: %02X\n", port + 1, buf);
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
              bprintf(info, "Commands: COMM%i:  Single command received\n",
                  port + 1);
              SingleCommand(indata[0], 0);
              mcommand = -1;
            } else if (((indata[1] >> 5) & 0x07) == 0x04) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(MIndex(mcommand));
              bprintf(info,
                  "Commands: COMM%i:  Multi word command %d (%s) started\n",
                  port + 1, mcommand, MName(mcommand));

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;  
            } else if ((((indata[1] >> 7) & 0x01) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              bprintf(info, "Commands: COMM%i:  Multi word command "
                  "continues...\n", port + 1);
              mcommand_count++;
            } else if ((((indata[1] >> 5) & 0x07) == 0x06) &&
                (mcommand == indata[0]) && 
                ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              bprintf(info, "Commands: COMM%i:  Multi word command ends \n",
                  port + 1);
              SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues,
                  ivalues);
              MultiCommand(mcommand, rvalues, ivalues, 0);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              bprintf(warning, "Commands: COMM%i: Command packet discarded: "
                  "Bad Encoding: %04X\n", port + 1, indata[1]);
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
          bprintf(warning,
              "Commands: COMM%i: Bad encoding: Bad packet terminator: %02X\n",
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
            bprintf(warning, "Commands: COMM%i: Bad encoding in GPS Position: "
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
            bprintf(warning, "Commands: COMM%i: Bad encoding in GPS Time: "
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
            bprintf(warning, "Commands: COMM%i: Bad encoding in MKS Altitude: "
                "Bad packet terminator: %02X\n", port + 1, buf);
          }
        }
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);
  }
}
#endif

/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there is   */
/*   no previous state file, set to default                 */
/*                                                          */
/************************************************************/
void InitCommandData() {
  int fp, n_read = 0, junk, extra = 0, i;

  if ((fp = open("/tmp/mcp.prev_status", O_RDONLY)) < 0) {
    berror(err, "Commands: Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      berror(err, "Commands: prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      berror(err, "Commands: extra prev_status read()");
    if (close(fp) < 0)
      berror(err, "Commands: prev_status close()");
  }

  /** this overrides prev_status **/
  CommandData.force_el = 0;

  if (CommandData.pumps.bal_veto != -1)
    CommandData.pumps.bal_veto = BAL_VETO_MAX;
  CommandData.pumps.bal1_on = 0;
  CommandData.pumps.bal1_reverse = 0;
  CommandData.pumps.bal2_on = 0;
  CommandData.pumps.bal2_reverse = 0;

  CommandData.pumps.inframe_cool_on = 0;
  CommandData.pumps.inframe_cool_off = 0;
  CommandData.pumps.lock_out = 0;
  CommandData.pumps.lock_in = 0;
  CommandData.pumps.lock_point = 0;
  CommandData.pumps.outframe_cool1_on = 0;
  CommandData.pumps.outframe_cool1_off = 0;
  CommandData.pumps.outframe_cool2_on = 0;
  CommandData.pumps.outframe_cool2_off = 0;

  CommandData.Bias.clockInternal = 0;
  CommandData.Bias.biasAC = 1;
  CommandData.Bias.biasRamp = 0;
  CommandData.Bias.SetLevel1 = 0;
  CommandData.Bias.SetLevel2 = 0;
  CommandData.Bias.SetLevel3 = 0;

  CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_NONE;
  CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_NONE;

  CommandData.sensors_off.gps = 0;
  CommandData.sensors_off.gyro = 0;
  CommandData.sensors_off.isc = 0;
  CommandData.sensors_off.osc = 0;
  CommandData.sensors_off.ss = 0;

  CommandData.gyheat[0].age = 0;
  CommandData.gyheat[1].age = 0;

  CommandData.Cryo.BDAHeat = 0;
  CommandData.Cryo.autoBDAHeat = 0;
  CommandData.Cryo.BDAGain.P = 600;
  CommandData.Cryo.BDAGain.I = 1;
  CommandData.Cryo.BDAGain.D = 20;
  CommandData.Cryo.BDAGain.SP = 21750;
  CommandData.Cryo.BDAFiltLen = 500;

  CommandData.Cryo.potvalve_on = 0;
  CommandData.Cryo.potvalve_open = 0;
  CommandData.Cryo.potvalve_close = 0;
  CommandData.Cryo.lhevalve_on = 0;
  CommandData.Cryo.lvalve_open = 0;
  CommandData.Cryo.lvalve_close = 0;
  CommandData.Cryo.lnvalve_on = 0;

  /* unused */
  CommandData.fast_gy_offset = 0;

  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    bprintf(warning, "Commands: prev_status: Wanted %i bytes but got %i.\n",
        sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    bputs(warning, "Commands: prev_status: Extra bytes found.\n");
  else
    return;

  bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

  /** prev_status overrides this stuff **/
  CommandData.at_float = 0;
  CommandData.timeout = 3600;

  CommandData.apcu_reg = 28.0;
  CommandData.apcu_trim = 0.0;
  CommandData.apcu_auto = 1;

  CommandData.dpcu_reg = 28.0;
  CommandData.dpcu_trim = 0.0;
  CommandData.dpcu_auto = 1;

  CommandData.pointing_mode.mode = P_DRIFT;
  CommandData.pointing_mode.X = 0;
  CommandData.pointing_mode.Y = 0;
  CommandData.pointing_mode.vaz = 0.0;
  CommandData.pointing_mode.del = 0.0;
  CommandData.pointing_mode.w = 0;
  CommandData.pointing_mode.h = 0;
  CommandData.pointing_mode.t = mcp_systime(NULL) + CommandData.timeout;

  CommandData.roll_gain.P = 30000;

  CommandData.ele_gain.I = 10000; /* was 8000 */
  CommandData.ele_gain.P = 10000; /* was 1200 */

  CommandData.azi_gain.P = 20000;
  CommandData.azi_gain.I = 5000; 

  CommandData.pivot_gain.SP = 36960;
  CommandData.pivot_gain.P = 200;

  CommandData.emf_gain = 1;
  CommandData.emf_offset = 0;

  CommandData.gyheat[0].setpoint = 30.0;
  CommandData.gyheat[0].gain.P = 30;
  CommandData.gyheat[0].gain.I = 10;
  CommandData.gyheat[0].gain.D = 3;

  CommandData.gyheat[1].setpoint = 35.0;
  CommandData.gyheat[1].gain.P = 40;
  CommandData.gyheat[1].gain.I = 5;
  CommandData.gyheat[1].gain.D = 0;

  CommandData.disable_az = 0;
  CommandData.disable_el = 0;

  CommandData.use_analogue_gyros = 0;

  CommandData.use_elenc = 1;
  CommandData.use_elclin = 1;
  CommandData.use_sun = 1;
  CommandData.use_isc = 1;
  CommandData.use_osc = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 1;
  CommandData.lat_range = 1;
  CommandData.sucks = 0;

  CommandData.clin_el_trim = 0;
  CommandData.enc_el_trim = 0;
  CommandData.null_az_trim = 0;
  CommandData.mag_az_trim = 0;
  CommandData.dgps_az_trim = 0;
  CommandData.ss_az_trim = 0;

  SIPData.MKScal.m_hi = 0.01;
  SIPData.MKScal.m_med = 0.1;
  SIPData.MKScal.m_lo = 1;
  SIPData.MKScal.b_hi = 0;
  SIPData.MKScal.b_med = 0;
  SIPData.MKScal.b_lo = 0;

  CommandData.az_autogyro = 1;
  CommandData.el_autogyro = 1;

  CommandData.pumps.pwm3 = 1638; /* inner frame cooling default --  20% */
  CommandData.pumps.pwm4 = 1638; /* outer frame cooling default --  20% */

  CommandData.pumps.bal_on = 0.5 * 1648.;
  CommandData.pumps.bal_off = 0.2 * 1648.;
  CommandData.pumps.bal_target = 0.0 * 1648.;
  CommandData.pumps.bal_gain = 0.2;
  CommandData.pumps.inframe_auto = 1;
  CommandData.pumps.outframe_auto = 0;

  CommandData.Bias.bias1 = 40;
  CommandData.Bias.bias2 = 25;
  CommandData.Bias.bias3 = 25;

  CommandData.pin_is_in = 1;

  CommandData.Cryo.heliumLevel = 0;
  CommandData.Cryo.charcoalHeater = 0;
  CommandData.Cryo.fridgeCycle = 1;
  CommandData.Cryo.coldPlate = 0;
  CommandData.Cryo.heatSwitch = 0;
  CommandData.Cryo.CryoSparePWM = 0;
  CommandData.Cryo.calibrator = off;

  CommandData.Cryo.JFETHeat = 0;
  CommandData.Cryo.autoJFETheat = 1;
  CommandData.Cryo.force_cycle = 0;
  CommandData.Cryo.JFETSetOn = 120;
  CommandData.Cryo.JFETSetOff = 135;

  CommandData.ISCState[0].abort = 0;
  CommandData.ISCState[0].pause = 0;
  CommandData.ISCState[0].save = 0;
  CommandData.ISCState[0].eyeOn = 1;
  CommandData.ISCState[0].autofocus = 0;
  CommandData.ISCState[0].focus_pos = 0;
  CommandData.ISCState[0].focusOffset = 0;
  CommandData.ISCState[0].ap_pos = 495;
  CommandData.ISCState[0].display_mode = full;
  /* ISC-BDA offsets per Ed Chapin 2005-05-17 */
  CommandData.ISCState[0].azBDA = 0.1894;
  CommandData.ISCState[0].elBDA = -0.2522;

  CommandData.ISCState[0].brightStarMode = 0;
  CommandData.ISCState[0].grid = 38;
  CommandData.ISCState[0].maxBlobMatch = 10;
  CommandData.ISCState[0].sn_threshold = 4.5;
  CommandData.ISCState[0].mult_dist = 30;
  CommandData.ISCState[0].mag_limit = 9;
  CommandData.ISCState[0].norm_radius = 3. * DEG2RAD;
  CommandData.ISCState[0].lost_radius = 6. * DEG2RAD;
  CommandData.ISCState[0].tolerance = 20. / 3600. * DEG2RAD; /* 20 arcsec */
  CommandData.ISCState[0].match_tol = 0.5;
  CommandData.ISCState[0].quit_tol = 1;
  CommandData.ISCState[0].rot_tol = 10 * DEG2RAD;
  CommandData.ISCState[0].triggertype = ISC_TRIGGER_NEG;
  CommandData.ISCState[0].gain = 1;
  CommandData.ISCState[0].offset = 0;
  CommandData.ISCControl[0].autofocus = 0;
  CommandData.ISCControl[0].save_period = 6000; /* 60 sec */
  CommandData.ISCControl[0].pulse_width = 50; /* 500.00 msec */
  CommandData.ISCControl[0].fast_pulse_width = 5; /* 50.00 msec */

  CommandData.ISCState[1].abort = 0;
  CommandData.ISCState[1].pause = 0;
  CommandData.ISCState[1].save = 0;
  CommandData.ISCState[1].eyeOn = 1;
  CommandData.ISCState[1].autofocus = 0;
  CommandData.ISCState[1].focus_pos = 0;
  CommandData.ISCState[1].focusOffset = 0;
  CommandData.ISCState[1].ap_pos = 495;
  CommandData.ISCState[1].display_mode = full;
  /* OSC-BDA offsets per Ed Chapin 2005-05-17 */
  CommandData.ISCState[1].azBDA = 0.1804;
  CommandData.ISCState[1].elBDA = 0.4828;

  CommandData.ISCState[1].brightStarMode = 0;
  CommandData.ISCState[1].grid = 38;
  CommandData.ISCState[1].maxBlobMatch = 10;
  CommandData.ISCState[1].sn_threshold = 4.5;
  CommandData.ISCState[1].mult_dist = 30;
  CommandData.ISCState[1].mag_limit = 9;
  CommandData.ISCState[1].norm_radius = 3. * DEG2RAD;
  CommandData.ISCState[1].lost_radius = 6. * DEG2RAD;
  CommandData.ISCState[1].tolerance = 20. / 3600. * DEG2RAD; /* 20 arcsec */
  CommandData.ISCState[1].match_tol = 0.5;
  CommandData.ISCState[1].quit_tol = 1;
  CommandData.ISCState[1].rot_tol = 10 * DEG2RAD;
  CommandData.ISCState[1].triggertype = ISC_TRIGGER_NEG;
  CommandData.ISCState[1].gain = 1;
  CommandData.ISCState[1].offset = 0;
  CommandData.ISCControl[1].autofocus = 0;
  CommandData.ISCControl[1].save_period = 6000; /* 60 sec */
  CommandData.ISCControl[1].pulse_width = 50; /* 500.00 msec */
  CommandData.ISCControl[1].fast_pulse_width = 5; /* 50.00 msec */

  for (i = 0; i < DAS_CARDS; ++i)
    CommandData.Phase[i] = 1970;

  WritePrevStatus();
}
