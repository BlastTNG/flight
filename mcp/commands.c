/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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
#undef VERBOSE_SIP_CHATTER

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
#define LOCK_OFFSET (-2.24)

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

#define VETO_MAX 60000

void ActPotTrim(void); /* actuators.c */
void RecalcOffset(double, double);

void SetRaDec(double, double); /* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

void nameThread(const char*);  /* mcp.c */

static const char *UnknownCommand = "Unknown Command";

extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];
extern short InCharge; /* tx.c */

extern int doing_schedule; /* sched.c */

extern pthread_t watchdog_id;  /* mcp.c */
extern short int SouthIAm;
pthread_mutex_t mutex;

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

/** Write the Previous Status: called whenever anything changes */
static void WritePrevStatus()
{
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

#ifndef USE_FIFO_CMD
static int bc_setserial(const char *input_tty)
{
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
#endif

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

#ifndef USE_FIFO_CMD
static float ParseGPS (unsigned char *data)
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

static void SendRequest (int req, char tty_fd)
{
  unsigned char buffer[3];

  buffer[0] = 0x10;
  buffer[1] = req;
  buffer[2] = 0x03;
  
#ifdef VERBOSE_SIP_CHATTER
  bprintf(info,"Commands: sending to SIP %02x %02x %02x\n",
      buffer[0],buffer[1],buffer[2]);
#endif

  if (write(tty_fd, buffer, 3) < 0)
    berror(warning, "Commands: error sending SIP request\n");
}
#endif

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

static const char* SName(enum singleCommand command)
{
  int i = SIndex(command);
  return (i == -1) ? UnknownCommand : scommands[i].name;
}

static void SingleCommand (enum singleCommand command, int scheduled)
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
      CommandData.pointing_mode.nw = CommandData.slew_veto;
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

      CommandData.pointing_mode.nw = CommandData.slew_veto;
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
    case reset_rw:
      CommandData.reset_rw=1;
      break;
    case reset_piv:
      CommandData.reset_piv=1;
      break;
    case reset_elev:
      CommandData.reset_elev=1;
      break;
    case restore_piv:
      CommandData.restore_piv=1;
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

    case gps_off:          /* power switching */
      CommandData.power.gps.set_count = 0;
      CommandData.power.gps.rst_count = LATCH_PULSE_LEN;
      break;
    case gps_on:
      CommandData.power.gps.rst_count = 0;
      CommandData.power.gps.set_count = LATCH_PULSE_LEN;
      break;
    case gps_cycle:
      CommandData.power.gps.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.gps.rst_count = LATCH_PULSE_LEN;
      break;
    case isc_off:
      CommandData.power.isc.set_count = 0;
      CommandData.power.isc.rst_count = LATCH_PULSE_LEN;
      break;
    case isc_on:
      CommandData.power.isc.rst_count = 0;
      CommandData.power.isc.set_count = LATCH_PULSE_LEN;
      break;
    case isc_cycle:
      CommandData.power.isc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.isc.rst_count = LATCH_PULSE_LEN;
      break;
    case osc_off:
      CommandData.power.osc.set_count = 0;
      CommandData.power.osc.rst_count = LATCH_PULSE_LEN;
      break;
    case osc_on:
      CommandData.power.osc.rst_count = 0;
      CommandData.power.osc.set_count = LATCH_PULSE_LEN;
      break;
    case osc_cycle:
      CommandData.power.osc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.osc.rst_count = LATCH_PULSE_LEN;
      break;
    case rw_off:
      CommandData.power.rw.set_count = 0;
      CommandData.power.rw.rst_count = LATCH_PULSE_LEN;
      break;
    case rw_on:
      CommandData.power.rw.rst_count = 0;
      CommandData.power.rw.set_count = LATCH_PULSE_LEN;
      break;
    case rw_cycle:
      CommandData.power.rw.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.rw.rst_count = LATCH_PULSE_LEN;
      break;
    case piv_off:
      CommandData.power.piv.set_count = 0;
      CommandData.power.piv.rst_count = LATCH_PULSE_LEN;
      break;
    case piv_on:
      CommandData.power.piv.rst_count = 0;
      CommandData.power.piv.set_count = LATCH_PULSE_LEN;
      break;
    case piv_cycle:
      // Pivot takes a long time to properly power cycle.
      CommandData.power.piv.set_count = 2*PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.piv.rst_count = LATCH_PULSE_LEN;
      break;
    case elmot_off:
      CommandData.power.elmot.set_count = 0;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
      break;
    case elmot_on:
      CommandData.power.elmot.rst_count = 0;
      CommandData.power.elmot.set_count = LATCH_PULSE_LEN;
      break;
    case elmot_cycle:
      CommandData.power.elmot.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
      break;
    case sc_tx_off:
      CommandData.power.sc_tx.set_count = 0;
      CommandData.power.sc_tx.rst_count = LATCH_PULSE_LEN;
      break;
    case sc_tx_on:
      CommandData.power.sc_tx.rst_count = 0;
      CommandData.power.sc_tx.set_count = LATCH_PULSE_LEN;
      break;
    case bi0_off:
      CommandData.power.bi0.set_count = 0;
      CommandData.power.bi0.rst_count = LATCH_PULSE_LEN;
      break;
    case bi0_on:
      CommandData.power.bi0.rst_count = 0;
      CommandData.power.bi0.set_count = LATCH_PULSE_LEN;
      break;
    case das_off:
      CommandData.power.das.set_count = 0;
      CommandData.power.das.rst_count = LATCH_PULSE_LEN;
      break;
    case das_on:
      CommandData.power.das.rst_count = 0;
      CommandData.power.das.set_count = LATCH_PULSE_LEN;
      break;
    case das_cycle:
      CommandData.power.das.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.das.rst_count = LATCH_PULSE_LEN;
      break;
    case preamp_off:
      CommandData.power.preamp.set_count = 0;
      CommandData.power.preamp.rst_count = LATCH_PULSE_LEN;
      break;
    case preamp_on:
      CommandData.power.preamp.rst_count = 0;
      CommandData.power.preamp.set_count = LATCH_PULSE_LEN;
      break;
    case bias_off:
      CommandData.power.bias.set_count = 0;
      CommandData.power.bias.rst_count = LATCH_PULSE_LEN;
      break;
    case bias_on:
      CommandData.power.bias.rst_count = 0;
      CommandData.power.bias.set_count = LATCH_PULSE_LEN;
      break;
    case heat_off:
      CommandData.power.heat.set_count = 0;
      CommandData.power.heat.rst_count = LATCH_PULSE_LEN;
      break;
    case heat_on:
      CommandData.power.heat.rst_count = 0;
      CommandData.power.heat.set_count = LATCH_PULSE_LEN;
      break;
    case hk_off:
      CommandData.power.hk.set_count = 0;
      CommandData.power.hk.rst_count = LATCH_PULSE_LEN;
      break;
    case hk_on:
      CommandData.power.hk.rst_count = 0;
      CommandData.power.hk.set_count = LATCH_PULSE_LEN;
      break;
    case um250_off:
      CommandData.power.um250.set_count = 0;
      CommandData.power.um250.rst_count = LATCH_PULSE_LEN;
      break;
    case um250_on:
      CommandData.power.um250.rst_count = 0;
      CommandData.power.um250.set_count = LATCH_PULSE_LEN;
      break;
    case um350_off:
      CommandData.power.um350.set_count = 0;
      CommandData.power.um350.rst_count = LATCH_PULSE_LEN;
      break;
    case um350_on:
      CommandData.power.um350.rst_count = 0;
      CommandData.power.um350.set_count = LATCH_PULSE_LEN;
      break;
    case um500_off:
      CommandData.power.um500.set_count = 0;
      CommandData.power.um500.rst_count = LATCH_PULSE_LEN;
      break;
    case um500_on:
      CommandData.power.um500.rst_count = 0;
      CommandData.power.um500.set_count = LATCH_PULSE_LEN;
      break;
    case ifroll_1_gy_allow:
      CommandData.gymask |= 0x01;
      break;
    case ifroll_1_gy_veto:
      CommandData.gymask &= ~0x01;
      break;
    case ifroll_2_gy_allow:
      CommandData.gymask |= 0x02;
      break;
    case ifroll_2_gy_veto:
      CommandData.gymask &= ~0x02;
      break;
    case ifyaw_1_gy_allow:
      CommandData.gymask |= 0x04;
      break;
    case ifyaw_1_gy_veto:
      CommandData.gymask &= ~0x04;
      break;
    case ifyaw_2_gy_allow:
      CommandData.gymask |= 0x08;
      break;
    case ifyaw_2_gy_veto:
      CommandData.gymask &= ~0x08;
      break;
    case ifel_1_gy_allow:
      CommandData.gymask |= 0x10;
      break;
    case ifel_1_gy_veto:
      CommandData.gymask &= ~0x10;
      break;
    case ifel_2_gy_allow:
      CommandData.gymask |= 0x20;
      break;
    case ifel_2_gy_veto:
      CommandData.gymask &= ~0x20;
      break;
    case ifroll_1_gy_off:
      CommandData.power.gyro_off[1] = -1;
      break;
    case ifroll_1_gy_on:
      CommandData.power.gyro_off[1] = 0;
      break;
    case ifroll_1_gy_cycle:
      CommandData.power.gyro_off[1] = PCYCLE_HOLD_LEN;
      break;
    case ifroll_2_gy_off:
      CommandData.power.gyro_off[5] = -1;
      break;
    case ifroll_2_gy_on:
      CommandData.power.gyro_off[5] = 0;
      break;
    case ifroll_2_gy_cycle:
      CommandData.power.gyro_off[5] = PCYCLE_HOLD_LEN;
      break;
    case ifyaw_1_gy_off:
      CommandData.power.gyro_off[0] = -1;
      break;
    case ifyaw_1_gy_on:
      CommandData.power.gyro_off[0] = 0;
      break;
    case ifyaw_1_gy_cycle:
      CommandData.power.gyro_off[0] = PCYCLE_HOLD_LEN;
      break;
    case ifyaw_2_gy_off:
      CommandData.power.gyro_off[2] = -1;
      break;
    case ifyaw_2_gy_on:
      CommandData.power.gyro_off[2] = 0;
      break;
    case ifyaw_2_gy_cycle:
      CommandData.power.gyro_off[2] = PCYCLE_HOLD_LEN;
      break;
    case ifel_1_gy_off:
      CommandData.power.gyro_off[3] = -1;
      break;
    case ifel_1_gy_on:
      CommandData.power.gyro_off[3] = 0;
      break;
    case ifel_1_gy_cycle:
      CommandData.power.gyro_off[3] = PCYCLE_HOLD_LEN;
      break;
    case ifel_2_gy_off:
      CommandData.power.gyro_off[4] = -1;
      break;
    case ifel_2_gy_on:
      CommandData.power.gyro_off[4] = 0;
      break;
    case ifel_2_gy_cycle:
      CommandData.power.gyro_off[4] = PCYCLE_HOLD_LEN;
      break;
    case gybox_off:
      CommandData.power.gybox_off = -1;
      break;
    case gybox_on:
      CommandData.power.gybox_off = 0;
      break;
    case gybox_cycle:
      CommandData.power.gybox_off = PCYCLE_HOLD_LEN;
      break;
    case hub232_off:
      CommandData.power.hub232_off = -1;
      break;
    case hub232_on:
      CommandData.power.hub232_off = 0;
      break;
    case hub232_cycle:
      CommandData.power.hub232_off = PCYCLE_HOLD_LEN;
      break;
    case ss_off:
      CommandData.power.ss_off = -1;
      break;
    case ss_on:
      CommandData.power.ss_off = 0;
      break;
    case ss_cycle:
      CommandData.power.ss_off = PCYCLE_HOLD_LEN;
      break;
    case actbus_off:
      CommandData.actbus.off = -1;
      break;
    case actbus_on:
      CommandData.actbus.off = 0;
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      break;
    case actbus_cycle:
      CommandData.actbus.off = PCYCLE_HOLD_LEN * FAST_PER_SLOW;
      //TODO check that repoll occurs after restart (not during)
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      break;

#endif

    case biascmd_inh:
      CommandData.Bias.dont_do_anything = 1;
      break;
    case biascmd_ena:
      CommandData.Bias.dont_do_anything = 0;
      break;
    case ramp:
      CommandData.Bias.biasRamp = 1;
      break;
    case fixed:
      CommandData.Bias.biasRamp = 0;
      break;

    case level_on:   /* Cryo commanding */
      CommandData.Cryo.heliumLevel = -1;
      break;
    case level_off:
      CommandData.Cryo.heliumLevel = 0;
      break;
    case level_pulse:
      CommandData.Cryo.heliumLevel = 350;  /* unit is 100Hz frames */
      break;
    case hwpr_enc_on:
      CommandData.Cryo.hwprPos = -1;
      break;
    case hwpr_enc_off:
      CommandData.Cryo.hwprPos = 0;
      break;
    case hwpr_enc_pulse:
      CommandData.Cryo.hwprPos = 18;
      break;
    case charcoal_on:
      CommandData.Cryo.charcoalHeater = 1;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case charcoal_off:
      CommandData.Cryo.charcoalHeater = 0;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case hs_charcoal_on:
      CommandData.Cryo.hsCharcoal= 1;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case hs_charcoal_off:
      CommandData.Cryo.hsCharcoal= 0;
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
    case cal_on:
      CommandData.Cryo.calibrator = on;
      break;
    case cal_off:
      CommandData.Cryo.calibrator = off;
      break;
    case jfet_on:
      CommandData.Cryo.JFETHeat = 1;
      CommandData.Cryo.autoJFETheat = 0;
      break;
    case jfet_off:
      CommandData.Cryo.JFETHeat = 0;
      CommandData.Cryo.autoJFETheat = 0;
      break;
    case hs_pot_on:
      CommandData.Cryo.hsPot = 1;
      break;
    case hs_pot_off:
      CommandData.Cryo.hsPot = 0;
      break;
    case bda_on:
      CommandData.Cryo.BDAHeat = 1;
      break;
    case bda_off:
      CommandData.Cryo.BDAHeat = 0;
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
    case auto_jfetheat:
      CommandData.Cryo.autoJFETheat = 1;
      break;

#ifndef BOLOTEST
    case balance_off:/* Balance pump commanding */
      CommandData.pumps.mode = bal_rest;
      CommandData.pumps.level = 0;
      break;
    case balance_auto:
      CommandData.pumps.mode= bal_auto;
      break;
    case balance_heat_on:/* Balance pump heating card commanding */
      CommandData.pumps.heat_on = 1;
      break;
    case balance_heat_off:
      CommandData.pumps.heat_on = 0;
      break;

#endif

    /* Lock */
    case pin_in:
      CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF | LS_IGNORE_EL;
      break;
    case lock_off:
      CommandData.actbus.lock_goal = LS_DRIVE_OFF | LS_DRIVE_FORCE;
      break;
    case unlock:
      CommandData.actbus.lock_goal = LS_OPEN | LS_DRIVE_OFF;
      if (CommandData.pointing_mode.mode == P_LOCK) {
        CommandData.pointing_mode.nw = CommandData.slew_veto;
	CommandData.pointing_mode.mode = P_DRIFT;
        CommandData.pointing_mode.X = 0;
        CommandData.pointing_mode.Y = 0;
        CommandData.pointing_mode.vaz = 0.0;
        CommandData.pointing_mode.del = 0.0;
        CommandData.pointing_mode.w = 0;
        CommandData.pointing_mode.h = 0;
      }
      break;
    case repoll:
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      CommandData.xystage.force_repoll = 1;
      break;

    /* Actuators */
    case actuator_stop:
      CommandData.actbus.focus_mode = ACTBUS_FM_PANIC;
      /* fallthrough */
    case autofocus_veto:
      CommandData.actbus.tc_mode = TC_MODE_VETOED;
      break;
    case autofocus_allow:
      CommandData.actbus.tc_mode = TC_MODE_ENABLED;
      break;
    case reset_dr:
      CommandData.actbus.reset_dr = 1;
      break;
    case actpos_trim:
      ActPotTrim();
      break;

    case hwpr_panic:
      CommandData.hwpr.mode = HWPR_PANIC;
      CommandData.hwpr.is_new = 1;
      break;

#ifndef BOLOTEST
      /***************************************/
      /********* ISC Commanding  *************/
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
      break;
    case isc_no_pyramid:
      CommandData.ISCState[0].useLost = 0;
      break;
    case isc_use_pyramid:
      CommandData.ISCState[0].useLost = 1;
      break;

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
    case osc_no_pyramid:
      CommandData.ISCState[1].useLost = 0;
      break;
    case osc_use_pyramid:
      CommandData.ISCState[1].useLost = 1;
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
#endif

    case reap_north:  /* Miscellaneous commands */
    case reap_south:
      if ((command == reap_north && !SouthIAm) || 
	  (command == reap_south && SouthIAm)) {
	bprintf(err, "Commands: Reaping the watchdog tickle on command.");
	pthread_cancel(watchdog_id);
      }
      break;
    case north_halt:
    case south_halt:
      if ((command == north_halt && !SouthIAm) || 
	  (command == south_halt && SouthIAm)) {
        bputs(warning, "Commands: Halting the MCC\n");
        if (system("/sbin/reboot") < 0)
	  berror(fatal, "Commands: failed to reboot, dying\n");
      }
      break;
    case xy_panic:
      CommandData.xystage.mode = XYSTAGE_PANIC;
      CommandData.xystage.is_new = 1;
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

static const char* MName(enum multiCommand command)
{
  int i = MIndex(command);
  return (i == -1) ? UnknownCommand : mcommands[i].name;
}

static void SetParameters(enum multiCommand command, unsigned short *dataq,
    double* rvalues, int* ivalues, char svalues[][CMD_STRING_LEN])
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
    } else if (type == 'l')  /* 30 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++] + mcommands[index].params[i].min;
      ivalues[i] += (dataq[dataqind++] << 15);
      bprintf(info, "Commands: param%02i: long   : %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = (float)dataq[dataqind++] * (mcommands[index].params[i].max
          - min) / MAX_15BIT + min;
      bprintf(info, "Commands: param%02i: float  : %f\n", i, rvalues[i]);
    } else if (type == 'd') { /* 30 bit floating point */
      rvalues[i] = (float)((int)dataq[dataqind++] << 15); /* upper 15 bits */
      rvalues[i] += (float)dataq[dataqind++];             /* lower 15 bits */
      rvalues[i] = rvalues[i] * (mcommands[index].params[i].max - min) /
        MAX_30BIT + min;
      bprintf(info, "Commands: param%02i: double : %f\n", i, rvalues[i]);
    } else if (type == 's') { /* string of 7-bit characters */
      int j;
      for (j = 0; j < mcommands[index].params[i].max; ++j)
        svalues[i][j] = ((j % 2) ? dataq[dataqind++] : dataq[dataqind] >> 8)
          & 0x7f;
      bprintf(info, "Commands: param%02i: string: %s\n", i, svalues[i]);
    } else
      bprintf(err,
          "Commands: Unknown parameter type ('%c') param%02i: ignored", type,
          i);
  }
#else
  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'l')  /* 30 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: long   : %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: float  : %f\n", i, rvalues[i]);
    } else if (type == 'd') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: double : %f\n", i, rvalues[i]);
    } else if (type == 's') { /* string */
      strncpy(svalues[i], dataqc[dataqind++], CMD_STRING_LEN - 1);
      svalues[i][CMD_STRING_LEN - 1] = 0;
      bprintf(info, "Commands: param%02i: string: %s\n", i, svalues[i]);
    } else
      bprintf(err,
          "Commands: Unknown parameter type ('%c') param%02i: ignored", type,
          i);
  }

  bprintf(info, "Commands: Multiword Command: %d (%s)\n", command,
      MName(command));
#endif
}

static inline void copysvalue(char* dest, const char* src)
{
  strncpy(dest, src, CMD_STRING_LEN - 1);
  dest[CMD_STRING_LEN - 1] = '\0';
}

static void MultiCommand(enum multiCommand command, double *rvalues,
    int *ivalues, char svalues[][CMD_STRING_LEN], int scheduled)
{
  int i;

  /* Update CommandData struct with new info
   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]
   */

  /* Pointing Modes */
  switch(command) {
#ifndef BOLOTEST
    case az_el_goto:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case az_scan:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZ_SCAN;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.w = rvalues[2];  /* width */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.h = 0;
      break;
    case drift:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = rvalues[0]; /* az speed */
      CommandData.pointing_mode.del = rvalues[1]; /* el speed */
      CommandData.pointing_mode.h = 0;
      break;
    case ra_dec_goto:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_RADEC_GOTO;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case vcap:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_VCAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el drift speed */
      CommandData.pointing_mode.h = 0;
      break;
    case cap:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_CAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el step size */
      CommandData.pointing_mode.h = 0;
      break;
    case box:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_BOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el step size */
      break;
    case vbox:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_VBOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el drift speed */
      break;
    case quad:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
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
      CommandData.offset_ifroll_gy = rvalues[0];
      CommandData.offset_ifyaw_gy = rvalues[1];
      CommandData.az_autogyro = 0;
      break;
    case el_gyro_offset:
      CommandData.offset_ifel_gy = rvalues[0];
      CommandData.el_autogyro = 0;
      break;
    case slew_veto:
      CommandData.slew_veto = rvalues[0] * SR;
      break;

      /***************************************/
      /********** Pointing Motor Gains *******/
    case el_gain:  /* ele gains */
      CommandData.ele_gain.P = ivalues[0];
      CommandData.ele_gain.I = ivalues[1];
      break;
    case az_gain:  /* az gains */
      CommandData.azi_gain.P = ivalues[0];
      CommandData.azi_gain.I = ivalues[1];
      break;
    case pivot_gain:  /* pivot gains */
      CommandData.pivot_gain.SP = rvalues[0];
      CommandData.pivot_gain.PE = ivalues[1];
      CommandData.pivot_gain.PV = ivalues[2];
      break;

      /***************************************/
      /***** Temporary test of motor DACs ****/
      /** TODO erase when done with them *****/
    case dac1_level:
      CommandData.Temporary.dac_out[0] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[0] = 1;
      break;
    case dac2_level:
      CommandData.Temporary.dac_out[1] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[1] = 1;
      break;
    case dac3_level:
      CommandData.Temporary.dac_out[2] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[2] = 1;
      break;
    case dac4_level:
      CommandData.Temporary.dac_out[3] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[3] = 1;
      break;
    case dac5_level:
      CommandData.Temporary.dac_out[4] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[4] = 1;
      break;
    case motors_verbose:
      CommandData.verbose_rw = ivalues[0];
      CommandData.verbose_el = ivalues[1];
      CommandData.verbose_piv = ivalues[2];
      break;
#endif

      /***************************************/
      /********** Lock / Actuators  **********/
    case lock:  /* Lock Inner Frame */
      if (CommandData.pumps.veto_bal >= 0)
        CommandData.pumps.veto_bal = VETO_MAX;
      CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_LOCK;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = LockPosition(rvalues[0]);
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      bprintf(info, "Commands: Lock Mode: %g\n", CommandData.pointing_mode.Y);
      break;
    case lock_vel:
      CommandData.actbus.lock_vel = ivalues[0];
      CommandData.actbus.lock_acc = ivalues[1];
      break;
    case lock_i:
      CommandData.actbus.lock_move_i = ivalues[0];
      CommandData.actbus.lock_hold_i = ivalues[1];
      break;
    case general: /* General actuator bus command */
      CommandData.actbus.caddr[CommandData.actbus.cindex] = ivalues[0] + 0x30;
      copysvalue(CommandData.actbus.command[CommandData.actbus.cindex],
          svalues[1]);
      CommandData.actbus.cindex = INC_INDEX(CommandData.actbus.cindex);
      break;
    case delta_secondary:
      CommandData.actbus.focus = ivalues[0];
      CommandData.actbus.focus_mode = ACTBUS_FM_DELFOC;
      break;
    case set_secondary:
      CommandData.actbus.focus = ivalues[0];
      CommandData.actbus.focus_mode = ACTBUS_FM_FOCUS;
      break;
    case thermo_gain:
      CommandData.actbus.tc_step = ivalues[2];
      CommandData.actbus.tc_wait = ivalues[3] * 300; /* convert min->5Hz */
      CommandData.actbus.tc_filter = ivalues[4] * 5; /* convert sec->5Hz */
      CommandData.actbus.sf_time = CommandData.actbus.tc_wait - 5;
      RecalcOffset(rvalues[0], rvalues[1]);
      CommandData.actbus.g_primary = rvalues[0];
      CommandData.actbus.g_secondary = rvalues[1];
      break;
    case focus_offset:
      CommandData.actbus.sf_offset = ivalues[0];
      CommandData.actbus.sf_time = CommandData.actbus.tc_wait - 5;
      break;
    case actuator_servo:
      CommandData.actbus.goal[0] = ivalues[0];
      CommandData.actbus.goal[1] = ivalues[1];
      CommandData.actbus.goal[2] = ivalues[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_SERVO;
      break;
    case actuator_delta:
      CommandData.actbus.delta[0] = ivalues[0];
      CommandData.actbus.delta[1] = ivalues[1];
      CommandData.actbus.delta[2] = ivalues[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_DELTA;
      break;
    case actuator_vel:
      CommandData.actbus.act_vel = ivalues[0];
      CommandData.actbus.act_acc = ivalues[1];
      break;
    case actuator_i:
      CommandData.actbus.act_move_i = ivalues[0];
      CommandData.actbus.act_hold_i = ivalues[1];
      break;
    case encoder_offset:
      CommandData.actbus.offset[0] = ivalues[0];
      CommandData.actbus.offset[1] = ivalues[1];
      CommandData.actbus.offset[2] = ivalues[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_OFFSET;
      break;
    case lvdt_limit:
      CommandData.actbus.lvdt_delta = rvalues[0];
      if (rvalues[1] > rvalues[2]) {
        CommandData.actbus.lvdt_low = rvalues[2];
        CommandData.actbus.lvdt_high = rvalues[1];
      } else {
        CommandData.actbus.lvdt_low = rvalues[1];
        CommandData.actbus.lvdt_high = rvalues[2];
      }
      break;
    case thermo_param:
      CommandData.actbus.tc_spread = rvalues[0];
      CommandData.actbus.tc_prefp = ivalues[1];
      CommandData.actbus.tc_prefs = ivalues[2];
      break;

    case hwpr_vel:
      CommandData.hwpr.vel = ivalues[0];
      CommandData.hwpr.acc = ivalues[1];
      break;
    case hwpr_i:
      CommandData.hwpr.move_i = ivalues[0];
      CommandData.hwpr.hold_i = ivalues[1];
      break;
    //TODO probably want hwpr moves calibrated into degrees
    case hwpr_goto:
      CommandData.hwpr.target = ivalues[0];
      CommandData.hwpr.mode = HWPR_GOTO;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_jump:
      CommandData.hwpr.target = ivalues[0];
      CommandData.hwpr.mode = HWPR_JUMP;
      CommandData.hwpr.is_new = 1;
      break;

      /* XY Stage */
    case xy_goto:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.y1 = ivalues[1];
      CommandData.xystage.xvel = ivalues[2];
      CommandData.xystage.yvel = ivalues[3];
      CommandData.xystage.mode = XYSTAGE_GOTO;
      CommandData.xystage.is_new = 1;
      break;
    case xy_jump:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.y1 = ivalues[1];
      CommandData.xystage.xvel = ivalues[2];
      CommandData.xystage.yvel = ivalues[3];
      CommandData.xystage.mode = XYSTAGE_JUMP;
      CommandData.xystage.is_new = 1;
      break;
    case xy_xscan:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.x2 = ivalues[1];
      CommandData.xystage.xvel = ivalues[2];
      CommandData.xystage.yvel = 0;
      CommandData.xystage.mode = XYSTAGE_SCAN;
      CommandData.xystage.is_new = 1;
      break;
    case xy_yscan:
      CommandData.xystage.y1 = ivalues[0];
      CommandData.xystage.y2 = ivalues[1];
      CommandData.xystage.yvel = ivalues[2];
      CommandData.xystage.xvel = 0;
      CommandData.xystage.mode = XYSTAGE_SCAN;
      CommandData.xystage.is_new = 1;
      break;
    case xy_raster:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.y1 = ivalues[1];
      CommandData.xystage.x2 = ivalues[2];
      CommandData.xystage.y2 = ivalues[3];
      CommandData.xystage.xvel = ivalues[4];
      CommandData.xystage.mode = XYSTAGE_RASTER;
      CommandData.xystage.is_new = 1;
      break;

#ifndef BOLOTEST
      /***************************************/
      /********** Balance System  ************/
    case balance_gain:
      CommandData.pumps.level_on_bal = rvalues[0] * 1990.13; /* 1990.13 DAC/Amp*/
      CommandData.pumps.level_off_bal = rvalues[1] * 1990.13;
      CommandData.pumps.level_target_bal = rvalues[2] * 1990.13;
      CommandData.pumps.gain_bal = rvalues[3];
      break;
    case balance_manual:
      CommandData.pumps.level = rvalues[0];
      CommandData.pumps.mode = bal_manual;
      break;
    case balance_tset:
      CommandData.pumps.heat_tset = rvalues[0];
      break;
    case balance_veto:
      CommandData.pumps.veto_bal = rvalues[0];
      break;



      /***************************************/
      /******** Electronics Heaters  *********/
    case t_gyro_set:  /* gyro heater setpoint */
      CommandData.gyheat.setpoint = rvalues[0];
      CommandData.gyheat.age = 0;
      break;
    case t_gyro_gain:  /* gyro heater gains */
      CommandData.gyheat.gain.P = ivalues[0];
      CommandData.gyheat.gain.I = ivalues[1];
      CommandData.gyheat.gain.D = ivalues[2];
      break;

      /***************************************/
      /*************** Misc  *****************/
//    case gyro_off:
//      CommandData.power.gyro_off[ivalues[0]-1] |= 0x01;
//      break;
//    case gyro_on:
//      CommandData.power.gyro_off[ivalues[0]-1] &= ~0x01;
//      break;
    case reset_adc:
      if (ivalues[0] < 64)
	CommandData.power.adc_reset[ivalues[0]/4] = RESET_ADC_LEN;
      break;
    case timeout:       /* Set timeout */
      CommandData.timeout = rvalues[0];
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
      /* used to be multiplied by 2 here, but screw up prev_satus */
      /* need to multiply later instead */
    case bias_level_500:    /* Set bias 1 (500) */
      CommandData.Bias.bias[0] = ivalues[0];
      CommandData.Bias.setLevel[0] = 1;
      break;
    case bias_level_350:   /* Set bias 2 (350) */
      CommandData.Bias.bias[1] = ivalues[0];
      CommandData.Bias.setLevel[1] = 1;
      break;
    case bias_level_250:   /* Set bias 3 (250) */
      CommandData.Bias.bias[2] = ivalues[0];
      CommandData.Bias.setLevel[2] = 1;
      break;
    case bias_level_rox:   /* Set bias 4 (ROX) */
      CommandData.Bias.bias[3] = ivalues[0];
      CommandData.Bias.setLevel[3] = 1;
      break;
    case bias_level_x:   /* Set bias 5 (spare) */
      CommandData.Bias.bias[4] = ivalues[0];
      CommandData.Bias.setLevel[4] = 1;
      break;
    case phase:
      if (ivalues[0] >= DAS_START && ivalues[0] <= DAS_START + DAS_CARDS*4/3
	  && ivalues[0]%4 != 0)
        CommandData.Phase[(ivalues[0] - DAS_START)*3/4] = ivalues[1];
      else if (ivalues[0] == 0)
        for (i = 0; i < DAS_CARDS; ++i)
          CommandData.Phase[i] = ivalues[1];
      else if (ivalues[0] == 13)
        CommandData.Phase[DAS_CARDS] = ivalues[1];

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
    case jfet_set:
      CommandData.Cryo.JFETSetOn = rvalues[0];
      CommandData.Cryo.JFETSetOff = rvalues[1];
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
    case isc_max_age:
      CommandData.ISCControl[0].max_age = ivalues[0]/10; //convert from ms to frames
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
    case osc_max_age:
      CommandData.ISCControl[1].max_age = ivalues[0]/10; //convert from ms to frames
      break;
#endif
    default:
      bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
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

#ifndef USE_FIFO_CMD
static void GPSPosition (unsigned char *indata)
{
  /* Send new information to CommandData */

  SIPData.GPSpos.lon = -ParseGPS(indata); /* sip sends east lon */
  SIPData.GPSpos.lat = ParseGPS(indata + 4);
  SIPData.GPSpos.alt = ParseGPS(indata + 8);
  SIPData.GPSstatus1 = *(indata + 12);
  SIPData.GPSstatus2 = *(indata + 13);

  WritePrevStatus();
}
#endif

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
      else if (type == 'l') /* 30 bit unsigned integer */
        bprintf(info, "Commands:   param%02i: long   : %i\n", i,
            event->ivalues[i]);
      else if (type == 'f') /* 15 bit floating point */
        bprintf(info, "Commands:   param%02i: float  : %f\n", i,
            event->rvalues[i]);
      else if (type == 'd') /* 30 bit floating point */
        bprintf(info, "Commands:   param%02i: double : %f\n", i,
            event->rvalues[i]);
      else
        bprintf(err,
            "Commands: Unknown parameter type ('%c') param%02i: ignored", type,
            i);
    }
    MultiCommand(event->command, event->rvalues, event->ivalues, event->svalues,
        1);

  } else {
    bprintf(info, "Commands: Executing Scheduled Command: %i (%s)\n",
        event->command, SName(event->command));
    SingleCommand(event->command, 1);
  }
}

#ifndef USE_FIFO_CMD
static void GPSTime (unsigned char *indata)
{
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

static void MKSAltitude (unsigned char *indata)
{

  SIPData.MKSalt.hi = ((unsigned short *)indata)[0];;
  SIPData.MKSalt.med = ((unsigned short *)indata)[1];;
  SIPData.MKSalt.lo = ((unsigned short *)indata)[2];;

  WritePrevStatus();
}

/* Send TDRSS Low Rate Packet */

static void SendDownData(char tty_fd)
{
  unsigned char buffer[3 + SLOWDL_LEN + 1], data[3 + SLOWDL_LEN + 1];
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
  memcpy(buffer +  3, data, SLOWDL_LEN);
  buffer[3 + SLOWDL_LEN] = SLOWDL_ETX;

  if (write(tty_fd, buffer, 3 + SLOWDL_LEN + 1) < 0)
    berror(warning, "Error writing to SlowDL\n");
#if 0
  for (i=0; i<3 + SLOWDL_LEN + 1; i++) {
    bprintf(info, "%d %2x", i, buffer[i]);
  }
#endif
}

/* compute the size of the data queue for the given command */
static int DataQSize(int index)
{
  int i, size = mcommands[index].numparams;

  for (i = 0; i < mcommands[index].numparams; ++i)
    if (mcommands[index].params[i].type == 'd'
        || mcommands[index].params[i].type == 'l')
      size++;
    else if (mcommands[index].params[i].type == 's')
      size += (mcommands[index].params[i].max - 1) / 2;

  return size;
}
#endif

#ifdef USE_FIFO_CMD
void WatchFIFO ()
{
  unsigned char buf[1];
  char command[100];
  char pbuf[30];
  int fifo;

  int mcommand = -1;
  int mcommand_count = 0;
  char *mcommand_data[DATA_Q_SIZE];

  int i;
  nameThread("SIPSS");
  for (i = 0; i < DATA_Q_SIZE; ++i) {
    mcommand_data[i] = NULL;
  }

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];

  int index, pindex = 0;

  bputs(startup, "WatchFIFO startup\n");

  if ((fifo = open("/tmp/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
    berror(tfatal, "Unable to open FIFO");

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    bprintf(info, "Command received: %s\n", command);
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
    bprintf(info, "%i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SCommand(command);
      SingleCommand(mcommand, 0);
      mcommand = -1;
    } else {
      mcommand = MCommand(command);
      bputs(info, "Multi word command received\n");
      if (mcommand_count == mcommands[MIndex(mcommand)].numparams) {
        SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues, ivalues,
            svalues);
        MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
      } else {
        bputs(warning, "Ignoring mal-formed command!\n");
      }
      mcommand = -1;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);

  }
}

#else
void WatchPort (void* parameter)
{
  const char *COMM[] = {"/dev/ttyS0", "/dev/ttyS1"};

  unsigned char buf;
  unsigned short *indatadumper;
  unsigned char indata[20];
  int readstage = 0;
  int tty_fd;

  int port = (int)parameter;

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];

  int mcommand = -1;
  int mcommand_count = 0;
  int dataqsize = 0;
  unsigned short mcommand_data[DATA_Q_SIZE];
  unsigned char mcommand_time = 0;

  int timer = 0;
  int bytecount = 0;

  char tname[6];
  sprintf(tname, "COMM%1d", port+1);
  nameThread(tname);
  bprintf(startup, "WatchPort startup\n");

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
        bprintf(info, "Request SIP Position\n");
#endif
        pthread_mutex_unlock(&mutex);
      } else if (timer == 1700) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_TIME, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Request SIP Time\n");
#endif
        pthread_mutex_unlock(&mutex);	
      } else if (timer > 2500) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_ALTITUDE, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Request SIP Altitude\n");
#endif
        pthread_mutex_unlock(&mutex);
        timer = 0;
      }
      usleep(10000); /* sleep for 10ms */
    }
#ifdef VERBOSE_SIP_CHATTER
    bprintf(info, "read SIP byte %02x\n", buf);
#endif

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
          bprintf(info, "Data request\n");
#endif
        } else if (buf == 0x14) { /* Command */
          readstage = 2;
#ifdef SIP_CHATTER
          bprintf(info, "Command\n");
#endif
        } else if (buf == 0x10) { /* GPS Position */
          readstage = 4;
#ifdef SIP_CHATTER
          bprintf(info, "GPS Position\n");
#endif
        } else if (buf == 0x11) { /* GPS Time */
          readstage = 5;
#ifdef SIP_CHATTER
          bprintf(info, "GPS Time\n");
#endif
        } else if (buf == 0x12) { /* MKS Altitude */
          readstage = 6;
#ifdef SIP_CHATTER
          bprintf(info, "MKS Altitude\n");
#endif
        } else {
          bprintf(warning, "Bad packet received: "
              "Unrecognised Packet Type: %02X\n", buf);
          readstage = 0;
        }
        break;
      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf == 0x02)
            bytecount = 1;
          else {
            readstage = 0;
            bprintf(warning, "Bad command packet: Unsupported Length: %02X\n", 
		buf);
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
            /* Recall:    101? ???? = 0xA0 = single command */
            /*            100? ???? = 0x80 = begin multi command */
            /*            110? ???? = 0xC0 = end multi command */
            /*            0??? ???? = 0x00 = data packet in multi command */


            if ((indata[1] & 0xE0) == 0xA0) {
              /*** Single command ***/
              bprintf(info, "Single command received\n");
              SingleCommand(indata[0], 0);
              mcommand = -1;
            } else if ((indata[1] & 0xE0) == 0x80) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(MIndex(mcommand));
              bprintf(info, "Multi word command %d (%s) started\n",
                  mcommand, MName(mcommand));

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;
            } else if (((indata[1] & 0x80) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              bprintf(info, "Multi word command continues...\n");
              mcommand_count++;
            } else if (((indata[1] & 0xE0) == 0xC0) && (mcommand == indata[0])
                && ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              bprintf(info, "Multi word command ends \n");
              SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues,
                  ivalues, svalues);
              MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              bprintf(warning, "Command packet discarded: Bad Encoding: %04X\n",
		  indata[1]);
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
          bprintf(warning, "Bad encoding: Bad packet terminator: %02X\n", buf);
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
            bprintf(warning, "Bad encoding in GPS Position: "
                "Bad packet terminator: %02X\n", buf);
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
            bprintf(warning, "Bad encoding in GPS Time: "
                "Bad packet terminator: %02X\n", buf);
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
            bprintf(warning, "Bad encoding in MKS Altitude: "
                "Bad packet terminator: %02X\n", buf);
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
/*  Initialize CommandData: read last valid state: if there */
/*   is no previous state file, set to default              */
/*                                                          */
/************************************************************/
void InitCommandData()
{
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

  CommandData.actbus.off = 0;
  CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
  CommandData.actbus.lock_goal = LS_DRIVE_OFF;
  CommandData.actbus.force_repoll = 0;
  CommandData.actbus.cindex = 0;
  CommandData.actbus.caddr[0] = 0;
  CommandData.actbus.caddr[1] = 0;
  CommandData.actbus.caddr[2] = 0;

  CommandData.xystage.is_new = 0;
  CommandData.xystage.force_repoll = 0;
  CommandData.hwpr.is_new = 0;
  CommandData.hwpr.force_repoll = 0;

  CommandData.Bias.dont_do_anything = 0;
  CommandData.Bias.biasRamp = 0;

  //forces reload of saved bias values
  CommandData.Bias.setLevel[0] = 1;
  CommandData.Bias.setLevel[1] = 1;
  CommandData.Bias.setLevel[2] = 1;
  CommandData.Bias.setLevel[3] = 1;
  CommandData.Bias.setLevel[4] = 1;

  CommandData.Temporary.setLevel[0] = 1;
  CommandData.Temporary.setLevel[1] = 1;
  CommandData.Temporary.setLevel[2] = 1;
  CommandData.Temporary.setLevel[3] = 1;
  CommandData.Temporary.setLevel[4] = 1;

  CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_NONE;
  CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_NONE;

  CommandData.power.sc_tx.rst_count = 0;
  CommandData.power.sc_tx.set_count = 0;
  CommandData.power.das.rst_count = 0;
  CommandData.power.das.set_count = 0;
  CommandData.power.isc.rst_count = 0;
  CommandData.power.isc.set_count = 0;
  CommandData.power.osc.rst_count = 0;
  CommandData.power.osc.set_count = 0;
  CommandData.power.gps.rst_count = 0;
  CommandData.power.gps.set_count = 0;
  CommandData.power.rw.rst_count = 0;
  CommandData.power.rw.set_count = 0;
  CommandData.power.piv.rst_count = 0;
  CommandData.power.piv.set_count = 0;
  CommandData.power.elmot.rst_count = 0;
  CommandData.power.elmot.set_count = 0;
  CommandData.power.bi0.rst_count = 0;
  CommandData.power.bi0.set_count = 0;
  CommandData.power.preamp.rst_count = 0;
  CommandData.power.preamp.set_count = 0;
  CommandData.power.bias.rst_count = 0;
  CommandData.power.bias.set_count = 0;
  CommandData.power.hk.rst_count = 0;
  CommandData.power.hk.set_count = 0;
  CommandData.power.um250.rst_count = 0;
  CommandData.power.um250.set_count = 0;
  CommandData.power.um350.rst_count = 0;
  CommandData.power.um350.set_count = 0;
  CommandData.power.um500.rst_count = 0;
  CommandData.power.um500.set_count = 0;
  CommandData.power.heat.rst_count = 0;
  CommandData.power.heat.set_count = 0;
  CommandData.power.gybox_off = 0;
  CommandData.power.gyro_off[0] = 0;
  CommandData.power.gyro_off[1] = 0;
  CommandData.power.gyro_off[2] = 0;
  CommandData.power.gyro_off[3] = 0;
  CommandData.power.gyro_off[4] = 0;
  CommandData.power.gyro_off[5] = 0;
  CommandData.power.hub232_off = 0;
  CommandData.power.ss_off = 0;
  for (i=0; i<16; i++)
    CommandData.power.adc_reset[i] = 0;

  CommandData.gyheat.age = 0;

  CommandData.Cryo.BDAHeat = 0;

  CommandData.Cryo.potvalve_on = 0;
  CommandData.Cryo.potvalve_open = 0;
  CommandData.Cryo.potvalve_close = 0;
  CommandData.Cryo.lhevalve_on = 0;
  CommandData.Cryo.lvalve_open = 0;
  CommandData.Cryo.lvalve_close = 0;
  CommandData.Cryo.lnvalve_on = 0;

  /* don't use the fast gy offset calculator */
  CommandData.fast_offset_gy = 0;

  // Motors are disabled on start-up of mcp
  // TODO-lmf: not sure if we want this for flight...
  CommandData.disable_az = 1; 
  CommandData.disable_el = 1;

  CommandData.reset_rw = 0;
  CommandData.reset_piv = 0;
  CommandData.reset_elev = 0;
  CommandData.restore_piv = 0;

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
  CommandData.alice_file = 0;

  CommandData.apcu_reg = 28.0;
  CommandData.apcu_trim = 0.0;
  CommandData.apcu_auto = 1;

  CommandData.dpcu_reg = 28.0;
  CommandData.dpcu_trim = 0.0;
  CommandData.dpcu_auto = 1;

  CommandData.slew_veto = VETO_MAX; /* 10 minutes */

  CommandData.pointing_mode.nw = 0;
  CommandData.pointing_mode.mode = P_DRIFT;
  CommandData.pointing_mode.X = 0;
  CommandData.pointing_mode.Y = 0;
  CommandData.pointing_mode.vaz = 0.0;
  CommandData.pointing_mode.del = 0.0;
  CommandData.pointing_mode.w = 0;
  CommandData.pointing_mode.h = 0;
  CommandData.pointing_mode.t = mcp_systime(NULL) + CommandData.timeout;

  CommandData.ele_gain.I = 10000; /* was 8000 */
  CommandData.ele_gain.P = 10000; /* was 1200 */

  CommandData.azi_gain.P = 1000;
  CommandData.azi_gain.I = 0;

  CommandData.pivot_gain.SP = 50; // dps
  CommandData.pivot_gain.PV = 100;

  CommandData.verbose_rw = 0;
  CommandData.verbose_el = 0;
  CommandData.verbose_piv = 0;

  CommandData.gyheat.setpoint = 15.0;
  CommandData.gyheat.age = 0;
  CommandData.gyheat.gain.P = 30;
  CommandData.gyheat.gain.I = 10;
  CommandData.gyheat.gain.D = 3;

  CommandData.use_elenc = 1;
  CommandData.use_elclin = 0;
  CommandData.use_sun = 0;
  CommandData.use_isc = 1;
  CommandData.use_osc = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 1;
  CommandData.lat_range = 1;
  CommandData.sucks = 1;

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
  CommandData.offset_ifel_gy = 0;
  CommandData.offset_ifroll_gy = 0;
  CommandData.offset_ifyaw_gy = 0;
  CommandData.gymask = 0x3f;
  
  CommandData.pumps.level_on_bal = 2.0 * 1990.13;  
  CommandData.pumps.level_off_bal = 0.5 * 1900.13;
  CommandData.pumps.level_target_bal = 0.0 * 1990.13;
  CommandData.pumps.gain_bal = 0.2;
  CommandData.pumps.mode = bal_rest; // TODO: change for flight
  CommandData.pumps.heat_on = 1;
  CommandData.pumps.heat_tset = 20;
  CommandData.pumps.veto_bal = VETO_MAX;

  CommandData.Temporary.dac_out[0] = 0x8000;
  CommandData.Temporary.dac_out[1] = 0x8000;
  CommandData.Temporary.dac_out[2] = 0x8000;
  CommandData.Temporary.dac_out[3] = 0x8000;
  CommandData.Temporary.dac_out[4] = 0x8000;

  CommandData.Bias.bias[0] = 5000;   //500um
  CommandData.Bias.bias[1] = 5000;   //350um
  CommandData.Bias.bias[2] = 5000;   //250um
  CommandData.Bias.bias[3] = 1045;   //ROX
  CommandData.Bias.bias[4] = 16384;  //X

  CommandData.actbus.tc_mode = TC_MODE_VETOED;
  CommandData.actbus.tc_step = 100; /* microns */
  CommandData.actbus.tc_wait = 3000; /* = 10 minutes in 5-Hz frames */
  CommandData.actbus.tc_filter = 300; /* = 60 seconds in 5 Hz frames */
  CommandData.actbus.tc_spread = 5; /* centigrade degrees */
  CommandData.actbus.tc_prefp = 1;
  CommandData.actbus.tc_prefs = 1;

  CommandData.actbus.reset_dr = 0;
  CommandData.actbus.dead_reckon[0] = 0;
  CommandData.actbus.dead_reckon[1] = 0;
  CommandData.actbus.dead_reckon[2] = 0;
  CommandData.actbus.last_good[0] = 0;
  CommandData.actbus.last_good[1] = 0;
  CommandData.actbus.last_good[2] = 0;
  CommandData.actbus.lvdt_delta = 1000;
  CommandData.actbus.lvdt_low = 4000;
  CommandData.actbus.lvdt_high = 19000;

  /* The first is due to change in radius of curvature, the second due to
   * displacement of the secondary due to the rigid struts */

  /* Don sez:   50.23 + 9.9 and 13.85 - 2.2 */
  /* Marco sez: 56          and 10          */
  
  CommandData.actbus.g_primary = 56; /* um/deg */
  CommandData.actbus.g_secondary = 10; /* um/deg */
  CommandData.actbus.focus = 0;
  CommandData.actbus.sf_time = 0;
  CommandData.actbus.sf_offset = 0;

  CommandData.actbus.act_vel = 2000;
  CommandData.actbus.act_acc = 1;
  CommandData.actbus.act_move_i = 75;
  CommandData.actbus.act_hold_i = 0;

  CommandData.actbus.lock_vel = 110000;
  CommandData.actbus.lock_acc = 100;
  CommandData.actbus.lock_move_i = 50;
  CommandData.actbus.lock_hold_i = 0;

  CommandData.hwpr.vel = 10000;
  CommandData.hwpr.acc = 1000;
  CommandData.hwpr.move_i = 20;
  CommandData.hwpr.hold_i = 0;

  CommandData.pin_is_in = 1;

  CommandData.Cryo.charcoalHeater = 0;
  CommandData.Cryo.hsCharcoal = 0;
  //TODO enable autocycycling when FridgeCycle is reimplemented
  CommandData.Cryo.fridgeCycle = 0;
  CommandData.Cryo.force_cycle = 0;
  CommandData.Cryo.hsPot = 0;
  CommandData.Cryo.heliumLevel = 0;
  CommandData.Cryo.he4_lev_old = 0;
  CommandData.Cryo.hwprPos = 0;
  CommandData.Cryo.hwpr_pos_old = 0;
  CommandData.Cryo.JFETHeat = 0;
  CommandData.Cryo.autoJFETheat = 1;
  CommandData.Cryo.JFETSetOn = 120;
  CommandData.Cryo.JFETSetOff = 135;
  CommandData.Cryo.calibrator = repeat;
  CommandData.Cryo.calib_pulse = 13; /* = 130 ms @ 100Hz */
  CommandData.Cryo.calib_period = 3000; /* = 600 s @ 5Hz */

  CommandData.ISCState[0].useLost = 1;
  CommandData.ISCState[0].abort = 0;
  CommandData.ISCState[0].pause = 0;
  CommandData.ISCState[0].save = 0;
  CommandData.ISCState[0].eyeOn = 1;
  CommandData.ISCState[0].hold_current = 0;
  CommandData.ISCState[0].autofocus = 0;
  CommandData.ISCState[0].focus_pos = 0;
  CommandData.ISCState[0].MCPFrameNum = 0;
  CommandData.ISCState[0].focusOffset = 0;
  CommandData.ISCState[0].ap_pos = 495;
  CommandData.ISCState[0].display_mode = full;
  /* ISC-BDA offsets per Ed Chapin & Marie Rex 2006-12-09 */
  CommandData.ISCState[0].azBDA = 0.047 * DEG2RAD;
  CommandData.ISCState[0].elBDA = -0.169 * DEG2RAD;
  CommandData.ISCControl[0].max_age = 200;

  CommandData.ISCState[0].brightStarMode = 0;
  CommandData.ISCState[0].grid = 38;
  CommandData.ISCState[0].minBlobMatch =  3;
  CommandData.ISCState[0].maxBlobMatch =  7;
  CommandData.ISCState[0].sn_threshold = 4.5;
  CommandData.ISCState[0].mult_dist = 30;
  CommandData.ISCState[0].mag_limit = 9.5;
  CommandData.ISCState[0].norm_radius = 3. * DEG2RAD;
  CommandData.ISCState[0].lost_radius = 6. * DEG2RAD;
  CommandData.ISCState[0].tolerance = 10. / 3600. * DEG2RAD; /* 10 arcsec */
  CommandData.ISCState[0].match_tol = 0.5;
  CommandData.ISCState[0].quit_tol = 1;
  CommandData.ISCState[0].rot_tol = 10 * DEG2RAD;
  CommandData.ISCState[0].triggertype = ISC_TRIGGER_NEG;
  CommandData.ISCState[0].gain = 1;
  CommandData.ISCState[0].offset = 0;
  CommandData.ISCControl[0].autofocus = 0;
  CommandData.ISCControl[0].save_period = 12000; /* 120 sec */
  CommandData.ISCControl[0].pulse_width = 50; /* 500.00 msec */
  CommandData.ISCControl[0].fast_pulse_width = 8; /* 80.00 msec */

  CommandData.ISCState[1].useLost = 1;
  CommandData.ISCState[1].abort = 0;
  CommandData.ISCState[1].pause = 0;
  CommandData.ISCState[1].save = 0;
  CommandData.ISCState[1].eyeOn = 1;
  CommandData.ISCState[1].hold_current = 0;
  CommandData.ISCState[1].autofocus = 0;
  CommandData.ISCState[1].focus_pos = 0;
  CommandData.ISCState[1].MCPFrameNum = 0;
  CommandData.ISCState[1].focusOffset = 0;
  CommandData.ISCState[1].ap_pos = 495;
  CommandData.ISCState[1].display_mode = full;
  /* OSC-BDA offsets per Ed Chapin & Marie Rex 2006-12-09 */
  CommandData.ISCState[1].azBDA = 0.525 * DEG2RAD;
  CommandData.ISCState[1].elBDA = 0.051 * DEG2RAD;
  CommandData.ISCControl[1].max_age = 200;

  CommandData.ISCState[1].brightStarMode = 0;
  CommandData.ISCState[1].grid = 38;
  CommandData.ISCState[1].minBlobMatch =  3;
  CommandData.ISCState[1].maxBlobMatch =  7;
  CommandData.ISCState[1].sn_threshold = 4.5;
  CommandData.ISCState[1].mult_dist = 30;
  CommandData.ISCState[1].mag_limit = 9.5;
  CommandData.ISCState[1].norm_radius = 3. * DEG2RAD;
  CommandData.ISCState[1].lost_radius = 6. * DEG2RAD;
  CommandData.ISCState[1].tolerance = 10. / 3600. * DEG2RAD; /* 10 arcsec */
  CommandData.ISCState[1].match_tol = 0.5;
  CommandData.ISCState[1].quit_tol = 1;
  CommandData.ISCState[1].rot_tol = 10 * DEG2RAD;
  CommandData.ISCState[1].triggertype = ISC_TRIGGER_NEG;
  CommandData.ISCState[1].gain = 1;
  CommandData.ISCState[1].offset = 0;
  CommandData.ISCControl[1].autofocus = 0;
  CommandData.ISCControl[1].save_period = 12000; /* 120 sec */
  CommandData.ISCControl[1].pulse_width = 30; /* 300.00 msec */
  CommandData.ISCControl[1].fast_pulse_width = 6; /* 60.00 msec */

  CommandData.temp1 = 0;
  CommandData.temp2 = 0;
  CommandData.temp3 = 0;
  CommandData.df = 0;

  // Coordinates for Toronto AKA the centre of the universe
  // used for motor tests. 
  CommandData.lat = 43.39;
  CommandData.lon = 79.23;

  CommandData.Phase[0] = 28178;
  CommandData.Phase[1] = 28178;
  CommandData.Phase[2] = 28178;
  CommandData.Phase[3] = 28178;
  CommandData.Phase[4] = 28178;
  CommandData.Phase[5] = 28178;
  CommandData.Phase[6] = 28178;
  CommandData.Phase[7] = 28178;
  CommandData.Phase[8] = 28178;
  CommandData.Phase[9] = 28178;
  CommandData.Phase[10] = 28178;
  CommandData.Phase[11] = 28178;
  CommandData.Phase[12] = 28178;

  WritePrevStatus();
}
