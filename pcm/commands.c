/* mcp: the BLAST master control program
 *
 * commands.c: functions for listening to and processing commands
 *
 * This software is copyright (C) 2002-2010 University of Toronto
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
#include "share/channels.h"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
#define LOCK_OFFSET (-1.4) /* Updated by LMF on December 9th, 2010 */

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

#define EXT_SLOT   0
#define EXT_ICHUNK 1
#define EXT_NCHUNK 2
#define EXT_NSCHED 3
#define EXT_ROUTE  4
  
#define MAXLIB 1024

#define MAX_RTIME 65536.0
#define MAX_DAYS 21.0

void RecalcOffset(double, double);  /* actuators.c */
void actEncTrim(int, int, int);

void SetRaDec(double, double); /* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

void nameThread(const char*);  /* mcp.c */

int LoadUplinkFile(int slot); /*sched.c */

static const char *UnknownCommand = "Unknown Command";

extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];
extern short InCharge; /* tx.c */

extern int doing_schedule; /* sched.c */

extern pthread_t watchdog_id;  /* mcp.c */
extern short int BitsyIAm;
pthread_mutex_t mutex;

extern char lst0str[82];

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

int sendSBSCCommand(const char *cmd); //sbsc.cpp

/** Write the Previous Status: called whenever anything changes */
static void WritePrevStatus()
{
  int fp, n;

  /** write the default file */
  fp = open("/data/etc/spider/pcm.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "Commands: pcm.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "Commands: pcm.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "Commands: pcm.prev_status close()");
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
      sun_az = PointingData[i_point].sun_az + 250; /* point solar panels to sun */
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
    case pss_veto:      /* Veto sensors */
      CommandData.use_pss = 0;
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

    case pss_allow:      /* Un-veto sensors */
      CommandData.use_pss = 1;
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
    case vtx_off:
      CommandData.power.sc_tx.set_count = 0;
      CommandData.power.sc_tx.rst_count = LATCH_PULSE_LEN;
      break;
    case vtx_on:
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
    case rx_off:
      CommandData.power.rx_main.set_count = 0;
      CommandData.power.rx_main.rst_count = LATCH_PULSE_LEN;
      break;
    case rx_on:
      CommandData.power.rx_main.rst_count = 0;
      CommandData.power.rx_main.set_count = LATCH_PULSE_LEN;
      break;
    case rx_hk_off:
      CommandData.power.rx_hk.set_count = 0;
      CommandData.power.rx_hk.rst_count = LATCH_PULSE_LEN;
      break;
    case rx_hk_on:
      CommandData.power.rx_hk.rst_count = 0;
      CommandData.power.rx_hk.set_count = LATCH_PULSE_LEN;
      break;
    case rx_amps_off:
      CommandData.power.rx_amps.set_count = 0;
      CommandData.power.rx_amps.rst_count = LATCH_PULSE_LEN;
      break;
    case rx_amps_on:
      CommandData.power.rx_amps.rst_count = 0;
      CommandData.power.rx_amps.set_count = LATCH_PULSE_LEN;
      break;
    case charge_on:
      CommandData.power.charge.set_count = 0;
      CommandData.power.charge.rst_count = LATCH_PULSE_LEN;
      break;
    case charge_off:
      CommandData.power.charge.rst_count = 0;
      CommandData.power.charge.set_count = LATCH_PULSE_LEN;
      break;
    case charge_cycle:
      CommandData.power.charge.rst_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.charge.set_count = LATCH_PULSE_LEN;
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
    case sbsc_off:
      CommandData.power.sbsc_cam_off = -1;
      CommandData.power.sbsc_cpu_off = -1;
      break;
    case sbsc_on:
      CommandData.power.sbsc_cam_off = 0;
      CommandData.power.sbsc_cpu_off = 0;
      break;
    case sbsc_cam_cycle:
      CommandData.power.sbsc_cam_off = PCYCLE_HOLD_LEN;
      break;
    case sbsc_cpu_cycle:
      CommandData.power.sbsc_cpu_off = PCYCLE_HOLD_LEN;
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
    case actbus_off:
      CommandData.actbus.off = -1;
      break;
    case actbus_on:
      CommandData.actbus.off = 0;
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      break;
    case actbus_cycle:
      CommandData.actbus.off = PCYCLE_HOLD_LEN;
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
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
      CommandData.actbus.tc_mode = TC_MODE_VETOED;
      break;

    case hwpr_panic:
      CommandData.hwpr.mode = HWPR_PANIC;
      CommandData.hwpr.is_new = 1;
      break;

#ifndef BOLOTEST
      /***************************************/
      /********* SBSC Commanding  *************/
    case cam_expose:
      sendSBSCCommand("CtrigExp");
      break;
    case cam_autofocus:
      if (CommandData.cam.forced)
	sendSBSCCommand("CtrigFocusF");
      else sendSBSCCommand("CtrigFocus");
      break;
    case cam_settrig_ext:
      sendSBSCCommand("CsetExpInt=0");
      CommandData.cam.expInt = 0;
      break;
    case cam_force_lens:
      CommandData.cam.forced = 1;
      break;
    case cam_unforce_lens:
      CommandData.cam.forced = 0;
      break;

    case blast_rocks:
      CommandData.sucks = 0;
      CommandData.uplink_sched = 0;
      break;
    case blast_sucks:
      CommandData.sucks = 1;
      CommandData.uplink_sched = 0;
      break;

    case at_float:
      CommandData.at_float = 1;
      break;
    case not_at_float:
      CommandData.at_float = 0;
      break;
    case vtx1_isc:
      CommandData.vtx_sel[0] = vtx_isc;
      break;
    case vtx1_osc:
      CommandData.vtx_sel[0] = vtx_osc;
      break;
    case vtx1_sbsc:
      CommandData.vtx_sel[0] = vtx_sbsc;
      break;
    case vtx2_isc:
      CommandData.vtx_sel[1] = vtx_isc;
      break;
    case vtx2_osc:
      CommandData.vtx_sel[1] = vtx_osc;
      break;
    case vtx2_sbsc:
      CommandData.vtx_sel[1] = vtx_sbsc;
      break;
#endif
    case hwpr_step:
      CommandData.hwpr.mode = HWPR_STEP;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_step_off:
      CommandData.hwpr.no_step = 1;
      break;
    case hwpr_step_on:
      CommandData.hwpr.no_step = 0;
      break;
    case hwpr_pot_is_dead:
      CommandData.hwpr.use_pot = 0;
      break;
    case hwpr_pot_is_alive:
      CommandData.hwpr.use_pot = 1;
      break;

    case reap_itsy:  /* Miscellaneous commands */
    case reap_bitsy:
      if ((command == reap_itsy && !BitsyIAm) || 
	  (command == reap_bitsy && BitsyIAm)) {
	bprintf(err, "Commands: Reaping the watchdog tickle on command.");
	pthread_cancel(watchdog_id);
      }
      break;
    case halt_itsy:
    case halt_bitsy:
      if ((command == halt_itsy && !BitsyIAm) || 
	  (command == halt_bitsy && BitsyIAm)) {
        bputs(warning, "Commands: Halting the MCC\n");
        if (system("/sbin/reboot") < 0)
	  berror(fatal, "Commands: failed to reboot, dying\n");
      }
      break;
    case xy_panic:
      CommandData.xystage.mode = XYSTAGE_PANIC;
      CommandData.xystage.is_new = 1;

    //Theo heater housekeeping commands. TODO-theo: temporarily uses insert 6
    case hk_t0_heat_on:
      CommandData.hk[5].pump_heat = 1;
      break;
    case hk_t0_heat_off:
      CommandData.hk[5].pump_heat = 0;
      break;
    //NB: because heat_switch on insert 6 is NC, logic is inverted
    case hk_t1_heat_on:
      CommandData.hk[5].heat_switch = 0;
      break;
    case hk_t1_heat_off:
      CommandData.hk[5].heat_switch = 1;
      break;
    case hk_t2_heat_on:
      CommandData.hk[5].tile_heat[2] = -1;
      break;
    case hk_t2_heat_off:
      CommandData.hk[5].tile_heat[2] = 0;
      break;
    case hk_t3_heat_on:
      CommandData.hk[5].tile_heat[1] = -1;
      break;
    case hk_t3_heat_off:
      CommandData.hk[5].tile_heat[1] = 0;
      break;
    case hk_t4_heat_on:
      CommandData.hk[5].tile_heat[0] = -1;
      break;
    case hk_t4_heat_off:
      CommandData.hk[5].tile_heat[0] = 0;
      break;
    case hk_t5_heat_on:
      CommandData.hk[5].fphi_heat = 1;
      break;
    case hk_t5_heat_off:
      CommandData.hk[5].fphi_heat = 0;
      break;
    case hk_t6_heat_on:
      CommandData.hk[5].tile_heat[3] = -1;
      break;
    case hk_t6_heat_off:
      CommandData.hk[5].tile_heat[3] = 0;
      break;


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
  int i, j;
  char buf[256]; //for SBSC Commands
  int is_new;

  /* Update CommandData struct with new info
   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]
   */

  /* Pointing Modes */
  switch(command) {
#ifndef BOLOTEST
    case az_el_goto:
      if ((CommandData.pointing_mode.mode != P_AZEL_GOTO) || 
          (CommandData.pointing_mode.X != rvalues[0]) ||
          (CommandData.pointing_mode.Y != rvalues[1])) {
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }

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
      
      if ((CommandData.pointing_mode.mode != P_CAP) ||
          (CommandData.pointing_mode.X != rvalues[0]) ||  /* ra */
          (CommandData.pointing_mode.Y != rvalues[1]) ||  /* dec */
          (CommandData.pointing_mode.w != rvalues[2]) ||  /* radius */
          (CommandData.pointing_mode.vaz != rvalues[3]) ||  /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[4]) ||  /* el step size */
          (CommandData.pointing_mode.h != 0) || 
          (CommandData.pointing_mode.dith != rvalues[5])) { /* el step size */
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }


      CommandData.pointing_mode.mode = P_CAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el step size */
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.dith = rvalues[5]; /* el step size */
      break;
    case box:

      if ((CommandData.pointing_mode.mode != P_BOX) ||
          (CommandData.pointing_mode.X != rvalues[0]) || /* ra */
          (CommandData.pointing_mode.Y != rvalues[1]) || /* dec */
          (CommandData.pointing_mode.w != rvalues[2]) || /* width */
          (CommandData.pointing_mode.h != rvalues[3]) || /* height */
          (CommandData.pointing_mode.vaz != rvalues[4]) || /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[5]) || /* el step size */
          (CommandData.pointing_mode.dith != rvalues[6])) { /* el step size */
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
     
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }
      
      CommandData.pointing_mode.mode = P_BOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el step size */
      CommandData.pointing_mode.dith = rvalues[6]; /* el step size */
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
      is_new = 0;
      if ((CommandData.pointing_mode.mode != P_QUAD) ||
          (CommandData.pointing_mode.vaz != rvalues[8]) || /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[9]) || /* el step size */
          (CommandData.pointing_mode.dith != rvalues[10])) { /* el dith size */
        is_new=1;
      }
      for (i = 0; i < 4; i++) {
        if ((CommandData.pointing_mode.ra[i] != rvalues[i * 2]) ||
            (CommandData.pointing_mode.dec[i] != rvalues[i * 2 + 1])) {
          is_new = 1;
        }
      }
      
      if (is_new) {
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      CommandData.pointing_mode.X = 0; /* ra */
      CommandData.pointing_mode.Y = 0; /* dec */
      CommandData.pointing_mode.w = 0; /* width */
      CommandData.pointing_mode.h = 0; /* height */
      
      CommandData.pointing_mode.mode = P_QUAD;
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i * 2];
        CommandData.pointing_mode.dec[i] = rvalues[i * 2 + 1];
      }
      CommandData.pointing_mode.vaz = rvalues[8]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[9]; /* el step size */
      CommandData.pointing_mode.dith = rvalues[10]; /* el dith size */
      break;
    case spider_scan:
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i*2];
        CommandData.pointing_mode.dec[i] = rvalues[i*2 + 1];        
      }
      CommandData.az_accel = rvalues[8];
      CommandData.pointing_mode.Y = rvalues[9];
      CommandData.pointing_mode.mode = P_SPIDER;
      break;
    case set_az_accel:
      CommandData.az_accel = rvalues[0];
      CommandData.az_accel_max = rvalues[1];
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
            bprintf(info,"CommandData.slew_veto = %i, CommandData.pointing_mode.nw = %i", CommandData.slew_veto, CommandData.pointing_mode.nw);
      if (CommandData.pointing_mode.nw > CommandData.slew_veto) CommandData.pointing_mode.nw = CommandData.slew_veto; 
     
      break;

      /***************************************/
      /********** Pointing Motor Gains *******/
    case el_gain:  /* ele gains */
      CommandData.ele_gain.P = ivalues[0];
      CommandData.ele_gain.I = ivalues[1];
      CommandData.ele_gain.PT = ivalues[2];
      break;
    case az_gain:  /* az gains */
      CommandData.azi_gain.P = ivalues[0];
      CommandData.azi_gain.I = ivalues[1];
      CommandData.azi_gain.PT = ivalues[2];
      break;
    case pivot_gain:  /* pivot gains */
      CommandData.pivot_gain.SP = rvalues[0];
      CommandData.pivot_gain.PE = ivalues[1];
      CommandData.pivot_gain.PV = ivalues[2];
      CommandData.pivot_gain.F = rvalues[3];
      break;

      //***********DGPS tests****************/
    case cov_gps:
      CommandData.dgps_cov_limit = rvalues[0];
      break;
    case ants_gps:
      CommandData.dgps_ants_limit = rvalues[0];
      break;
      /*************************************/

      /***************************************/
      /*****           test of motor DACs ****/
    case dac2_level:
      CommandData.Temporary.dac_out[1] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[1] = 1;
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
      if (CommandData.pointing_mode.nw >= 0)
        CommandData.pointing_mode.nw = VETO_MAX;
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
    case actuator_servo:
      CommandData.actbus.goal[0] = ivalues[0] + CommandData.actbus.offset[0];
      CommandData.actbus.goal[1] = ivalues[1] + CommandData.actbus.offset[1];
      CommandData.actbus.goal[2] = ivalues[2] + CommandData.actbus.offset[2];
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
    case actuator_tol:
      CommandData.actbus.act_tol = ivalues[0];
      break;
    case act_offset:
      CommandData.actbus.offset[0] = (int)(rvalues[0]+0.5);
      CommandData.actbus.offset[1] = (int)(rvalues[1]+0.5);
      CommandData.actbus.offset[2] = (int)(rvalues[2]+0.5);
      break;
    case act_enc_trim:
      CommandData.actbus.trim[0] = rvalues[0];
      CommandData.actbus.trim[1] = rvalues[1];
      CommandData.actbus.trim[2] = rvalues[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_TRIM;
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

    case hwpr_vel:
      CommandData.hwpr.vel = ivalues[0];
      CommandData.hwpr.acc = ivalues[1];
      break;
    case hwpr_i:
      CommandData.hwpr.move_i = ivalues[0];
      CommandData.hwpr.hold_i = ivalues[1];
      break;
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
    case hwpr_repeat:
      CommandData.hwpr.n_pos = ivalues[0];
      CommandData.hwpr.repeats = ivalues[1];
      CommandData.hwpr.step_wait = ivalues[2]*5;
      CommandData.hwpr.step_size = ivalues[3];
      CommandData.hwpr.mode = HWPR_REPEAT;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_define_pos:
      CommandData.hwpr.pos[0] = rvalues[0];
      CommandData.hwpr.pos[1] = rvalues[1];
      CommandData.hwpr.pos[2] = rvalues[2];
      CommandData.hwpr.pos[3] = rvalues[3];
      break;
    case hwpr_goto_pot:
      CommandData.hwpr.pot_targ = rvalues[0];
      CommandData.hwpr.mode = HWPR_GOTO_POT;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_set_overshoot:
      CommandData.hwpr.overshoot = ivalues[0];
      break;
    case hwpr_goto_i:
      CommandData.hwpr.i_pos = ivalues[0];
      CommandData.hwpr.mode = HWPR_GOTO_I;
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
      CommandData.xystage.x2 = ivalues[1];
      CommandData.xystage.y1 = ivalues[2];
      CommandData.xystage.y2 = ivalues[3];
      CommandData.xystage.xvel = ivalues[4];
      CommandData.xystage.yvel = ivalues[5];
      CommandData.xystage.step = ivalues[6];
      CommandData.xystage.mode = XYSTAGE_RASTER;
      CommandData.xystage.is_new = 1;
      break;

#ifndef BOLOTEST
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
    case t_sbsc_set:  /* SBSC heater setpoint */
      CommandData.t_set_sbsc = rvalues[0];
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
    case tdrss_bw:
      CommandData.tdrss_bw = rvalues[0];
      break;      
    case iridium_bw:
      CommandData.iridium_bw = rvalues[0];
      break;
    case slot_sched: /* change uplinked schedule file */
      if (LoadUplinkFile(ivalues[0])) {
        CommandData.uplink_sched = 1;
        CommandData.slot_sched = ivalues[0];
      }
      break;
    case plugh:/* A hollow voice says "Plugh". */
      CommandData.plover = ivalues[0];
      break;
#endif

      /***************************************/
      /*************** Bias  *****************/
    case hk_ampl_ntd:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ntd.ampl = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ntd.ampl = rvalues[1];
      break;
    case hk_ampl_cernox:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].cernox.ampl = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].cernox.ampl = rvalues[1];
      break;
    case hk_phase_ntd:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ntd.phase = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ntd.phase = rvalues[1];
      break;
    case hk_phase_cernox:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].cernox.phase= rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].cernox.phase = rvalues[1];
      break;
    case hk_bias_freq:
      //TODO consider scaling phases as fixed time delay, when freq changes
      CommandData.hk_bias_freq = ivalues[0];
      break;

      /***************************************/
      /*************** Heat  *****************/
    case hk_pump_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].pump_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].pump_heat = 1;
      break;
    case hk_pump_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].pump_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].pump_heat = 0;
      break;
    case hk_heat_switch_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].heat_switch = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].heat_switch = 1;
      break;
    case hk_heat_switch_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].heat_switch = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].heat_switch = 0;
      break;
    case hk_fphi_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fphi_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fphi_heat = 1;
      break;
    case hk_fphi_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fphi_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fphi_heat = 0;
      break;
    case hk_tile_heat_on:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_tile_last = ivalues[1];
      for (i = ((ivalues[0] > 0) ? ivalues[0] - 1 : 0);
	   i < ((ivalues[0] > 0) ? ivalues[0]     : HK_MAX); i++) {
	for (j = ((ivalues[1] > 0) ? ivalues[1] - 1 : 0);
	     j < ((ivalues[1] > 0) ? ivalues[1]     : 4); j++) {
	  CommandData.hk[i].tile_heat[j] = -1;
	}
      }
      break;
    case hk_tile_heat_off:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_tile_last = ivalues[1];
      for (i = ((ivalues[0] > 0) ? ivalues[0] - 1 : 0);
	   i < ((ivalues[0] > 0) ? ivalues[0]     : HK_MAX); i++) {
	for (j = ((ivalues[1] > 0) ? ivalues[1] - 1 : 0);
	     j < ((ivalues[1] > 0) ? ivalues[1]     : 4); j++) {
	  CommandData.hk[i].tile_heat[j] = 0;
	}
      }
      break;
    case hk_tile_heat_pulse:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_tile_last = ivalues[1];
      CommandData.hk_pulse_last = ivalues[2];
      for (i = ((ivalues[0] > 0) ? ivalues[0] - 1 : 0);
	   i < ((ivalues[0] > 0) ? ivalues[0]     : HK_MAX); i++) {
	for (j = ((ivalues[1] > 0) ? ivalues[1] - 1 : 0);
	     j < ((ivalues[1] > 0) ? ivalues[1]     : 4); j++) {
	  CommandData.hk[i].tile_heat[j] = ivalues[2];
	}
      }
      break;
    case hk_fplo_heat_set:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_vheat_last = rvalues[1];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fplo_heat = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fplo_heat = rvalues[1];
      break;
    case hk_ssa_heat_set:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_vheat_last = rvalues[1];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ssa_heat = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ssa_heat = rvalues[1];
      break;


#ifndef BOLOTEST
      /***************************************/
      /********* SBSC Commanding  *************/ 
    case cam_any:
      sendSBSCCommand(svalues[0]);
      break;
    case cam_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendSBSCCommand(buf);
      CommandData.cam.expInt = ivalues[0];
      break;
    case cam_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.cam.expTime = ivalues[0];
      sendSBSCCommand(buf);
      break;
    case cam_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendSBSCCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendSBSCCommand(buf); 
      CommandData.cam.focusRes = ivalues[0];
      CommandData.cam.focusRng = ivalues[1];
      break;
    case cam_bad_pix:
      sprintf(buf, "IsetBadpix=%d %d %d", ivalues[0], ivalues[1], ivalues[2]);
      sendSBSCCommand(buf);
      break;
    case cam_blob_params:
      sprintf(buf, "IsetMaxBlobs=%d", ivalues[0]);
      sendSBSCCommand(buf);
      sprintf(buf, "IsetGrid=%d", ivalues[1]);
      sendSBSCCommand(buf);
      sprintf(buf, "IsetThreshold=%f", rvalues[2]);
      sendSBSCCommand(buf);
      sprintf(buf, "IsetDisttol=%d", ivalues[3]);
      sendSBSCCommand(buf);
      CommandData.cam.maxBlobs = ivalues[0];
      CommandData.cam.grid = ivalues[1];
      CommandData.cam.threshold = rvalues[2];
      CommandData.cam.minBlobDist = ivalues[3];
      break;
    case cam_lens_any:
      sprintf(buf, "L=%s", svalues[0]);
      sendSBSCCommand(buf);
      break;
    case cam_lens_move:
      if (CommandData.cam.forced)
	sprintf(buf, "Lforce=%d", ivalues[0]);
      else sprintf(buf, "Lmove=%d", ivalues[0]);
      sendSBSCCommand(buf);
      break;
    case cam_lens_params:
      sprintf(buf, "LsetTol=%d", ivalues[0]);
      sendSBSCCommand(buf);
      CommandData.cam.moveTol = ivalues[0];
      break;
    case table_move:
      CommandData.table.RelMove += rvalues[0];
      break;
    case table_move_g:
      CommandData.table.MoveGain = rvalues[0];
      break;
    case table_gain:  /* rotary table gains */
      //TODO PID loop performed in controller, figure out how to set gains
      CommandData.table.tableGain.P = ivalues[0];
      CommandData.table.tableGain.I = ivalues[1];
      CommandData.table.tableGain.D = ivalues[2];
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
  double lat;
 
  SIPData.GPSstatus1 = *(indata + 12);
  SIPData.GPSstatus2 = *(indata + 13);

  lat = ParseGPS(indata + 4);
  if (fabs(lat)>20) {

    SIPData.GPSpos.lat = lat;
    SIPData.GPSpos.lon = -ParseGPS(indata); /* sip sends east lon */
    /* end of hack */
    
    SIPData.GPSpos.alt = ParseGPS(indata + 8);
    
    WritePrevStatus();
  }
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
  GPSweek = *((unsigned short*)(indata + 4));
  offset = ParseGPS(indata + 6);
  CPUtime = ParseGPS(indata + 10);

  SIPData.GPStime.UTC = (int)(SEC_IN_WEEK * (GPSweek) + GPStime - offset) +
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
  char command[500];
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

  if ((fifo = open("/data/etc/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
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

struct LibraryStruct  {
  int n;
  int entry[MAXLIB];
  char cmd[MAXLIB][64];
  char params[MAXLIB][256];
};

struct LibraryStruct library;

void OpenLibrary() {
  FILE *fp;
  char instr[1024];
  int nf;
  int i;

  fp = fopen("/data/etc/spider/sched.library", "r");
  if (fp == NULL) {
    berror(fatal, "Could not open schedule file library.");
    exit(0);
  }
  
  i=0;
  while (fgets(instr, 256, fp) != NULL) {
    memset(library.params[i], 0, 256);
    nf = sscanf(instr, "%d %s %254c", library.entry+i, library.cmd[i], library.params[i]);
    if (nf==2) library.params[i][0] = '\n';
    if (nf>=2) i++;
  }
  library.n = i;

}


void ProcessUplinkSched(unsigned char *extdat) {
  static unsigned char slot = 0xff;
  static unsigned short sched[32][64][2];
  static unsigned long chunks_received = 0;
  static unsigned char nchunk = 0;
  static unsigned char nsched[32];
  
  unsigned char slot_in, i_chunk, nchunk_in;
  unsigned short *extdat_ui;
  unsigned short entry;
  unsigned short itime;
  double day, hour;
  
  int i, i_samp;
  
  slot_in = extdat[EXT_SLOT];
  i_chunk = extdat[EXT_ICHUNK];
  nchunk_in = extdat[EXT_NCHUNK];
  nsched[i_chunk] = extdat[EXT_NSCHED];
  
  if ((slot != slot_in) || (nchunk_in != nchunk)) {
    chunks_received = 0;
    for (i=nchunk_in; i<32; i++) {
      chunks_received |= (1 << i);
    }
    slot = slot_in;
    nchunk = nchunk_in;
  }
  
  extdat_ui = (unsigned short *)(&extdat[6]);

  for (i=0; i<nsched[i_chunk]; i++) {
    sched[i_chunk][i][0] = extdat_ui[i*2];
    sched[i_chunk][i][1] = extdat_ui[i*2+1];
  }

  chunks_received |= (1<<i_chunk);

  CommandData.parts_sched = chunks_received;
  CommandData.upslot_sched = slot;
  
  if (chunks_received == 0xffffffff) {
    FILE *fp;
    char filename[18];
   
    OpenLibrary();
    
    sprintf(filename, "/data/etc/spider/%d.sch", slot);
    fp = fopen(filename, "w");
    
    fprintf(fp, "%s", lst0str);
    
    for (i_chunk=0; i_chunk < nchunk; i_chunk++) {
      for (i_samp=0; i_samp<nsched[i_chunk]; i_samp++) {
        entry = sched[i_chunk][i_samp][1];
        itime = sched[i_chunk][i_samp][0];
        day = (double)itime*MAX_DAYS/MAX_RTIME;
        hour = (day - floor(day))*24.0;
        if (entry<library.n) {
          fprintf(fp, "%s %d %.6g %s", library.cmd[entry], (int)day , hour, library.params[entry]);
        } else {
          bprintf(warning, "entry %d not in library\n", entry);
        }
      }
    }
    fclose(fp);
  }
  
  bprintf(warning, "finished extended command\n"
                   "  slot %d chunk %d n chunk %d nsched %d route %x chunks_received: %lx\n", 
                   extdat[EXT_SLOT], extdat[EXT_ICHUNK], extdat[EXT_NCHUNK], extdat[EXT_NSCHED], extdat[EXT_ROUTE], chunks_received);
}

void WatchPort (void* parameter)
{
  const char *COMM[] = {"/dev/ttyS0", "/dev/ttyS1"};
  const unsigned char route[2] = {0x09, 0x0c};

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
  int extlen = 0;

  unsigned char extdat[256];

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
          if (buf == 0x02) {
            bytecount = 1;
	  } else {
            readstage = 7;
	    extlen = buf;
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
        break;
      case 7: // reading extended command
        if (bytecount < extlen) {
	  extdat[bytecount] = buf;
	  bytecount++;
	} else {
	  if (buf == 0x03) {
	    if (extdat[4] == route[port]) {
	      ProcessUplinkSched(extdat);
	    }
	  } else {
            bprintf(warning, "Bad encoding in extended command: "
                "Bad packet terminator: %02X\n", buf);
          }
          bytecount = 0;
          readstage = 0;
	}
        break;
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
  int fp, n_read = 0, junk, extra = 0, i, j;

  if ((fp = open("/data/etc/spider/pcm.prev_status", O_RDONLY)) < 0) {
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
  CommandData.df = 0;
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
  CommandData.hwpr.repeats = 0;

  CommandData.table.RelMove = 0;

  CommandData.Temporary.setLevel[0] = 1;
  CommandData.Temporary.setLevel[1] = 1;
  CommandData.Temporary.setLevel[2] = 1;
  CommandData.Temporary.setLevel[3] = 1;
  CommandData.Temporary.setLevel[4] = 1;

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
  CommandData.power.rx_main.rst_count = 0;
  CommandData.power.rx_main.set_count = 0;
  CommandData.power.rx_hk.rst_count = 0;
  CommandData.power.rx_hk.set_count = 0;
  CommandData.power.rx_amps.rst_count = 0;
  CommandData.power.rx_amps.set_count = 0;
  CommandData.power.gybox_off = 0;
  CommandData.power.gyro_off[0] = 0;
  CommandData.power.gyro_off[1] = 0;
  CommandData.power.gyro_off[2] = 0;
  CommandData.power.gyro_off[3] = 0;
  CommandData.power.gyro_off[4] = 0;
  CommandData.power.gyro_off[5] = 0;
  CommandData.power.hub232_off = 0;
  for (i=0; i<16; i++)
    CommandData.power.adc_reset[i] = 0;

  CommandData.gyheat.age = 0;

  /* don't use the fast gy offset calculator */
  CommandData.fast_offset_gy = 0;

  CommandData.reset_rw = 0;
  CommandData.reset_piv = 0;
  CommandData.reset_elev = 0;
  CommandData.restore_piv = 0;
  
  CommandData.slot_sched = 0x100;
  CommandData.parts_sched=0x0;

  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    bprintf(warning, "Commands: prev_status: Wanted %i bytes but got %i.\n",
        (int) sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    bputs(warning, "Commands: prev_status: Extra bytes found.\n");
  else
    return;

  bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

  /** prev_status overrides this stuff **/
  CommandData.at_float = 0;
  CommandData.timeout = 3600;
  CommandData.slot_sched = 0;
  CommandData.tdrss_bw = 6000;
  CommandData.iridium_bw = 2000;
  CommandData.vtx_sel[0] = vtx_isc;
  CommandData.vtx_sel[1] = vtx_osc;

  CommandData.slew_veto = VETO_MAX; /* 5 minutes */

  CommandData.pointing_mode.nw = 0;
  CommandData.pointing_mode.mode = P_DRIFT;
  CommandData.pointing_mode.X = 0;
  CommandData.pointing_mode.Y = 0;
  CommandData.pointing_mode.vaz = 0.0;
  CommandData.pointing_mode.del = 0.0;
  CommandData.pointing_mode.w = 0;
  CommandData.pointing_mode.h = 0;
  CommandData.pointing_mode.t = mcp_systime(NULL) + CommandData.timeout;
  CommandData.pointing_mode.dith = 0.0;

  CommandData.az_accel = 0.4; 
  CommandData.az_accel_max = 100.0;

  CommandData.ele_gain.I = 5000; /* was 8000 */
  CommandData.ele_gain.P = 5000; /* was 1200 */
  CommandData.ele_gain.PT = 3000;

  CommandData.azi_gain.P = 4000;
  CommandData.azi_gain.I = 100;
  CommandData.azi_gain.PT = 3000;

  CommandData.pivot_gain.SP = 50; // dps
  CommandData.pivot_gain.PV = 400;
  CommandData.pivot_gain.PE = 0;
  CommandData.pivot_gain.F = 0.3;

  CommandData.disable_az = 1; 
  CommandData.disable_el = 0;

  CommandData.verbose_rw = 0;
  CommandData.verbose_el = 0;
  CommandData.verbose_piv = 0;

  CommandData.gyheat.setpoint = 15.0;
  CommandData.gyheat.age = 0;
  CommandData.gyheat.gain.P = 30;
  CommandData.gyheat.gain.I = 10;
  CommandData.gyheat.gain.D = 3;

  CommandData.table.MoveGain = 10.0;
  CommandData.table.tableGain.P = 6652;  //thousandths
  CommandData.table.tableGain.I = 302;   //ten-thousandths
  CommandData.table.tableGain.D = 13520; //hundredths

  CommandData.use_elenc = 1;
  CommandData.use_elclin = 1;
  CommandData.use_pss = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 0;
  CommandData.lat_range = 1;
  CommandData.sucks = 1;

  CommandData.clin_el_trim = 0;
  CommandData.enc_el_trim = 0;
  CommandData.null_az_trim = 0;
  CommandData.mag_az_trim = 0;
  CommandData.dgps_az_trim = 0;
  CommandData.pss_az_trim = 0;

  CommandData.dgps_cov_limit = 0.3;
  CommandData.dgps_ants_limit = 0.5;

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
  
  CommandData.Temporary.dac_out[0] = 0x8000;
  CommandData.Temporary.dac_out[1] = 0x8000;
  CommandData.Temporary.dac_out[2] = 0x8000;
  CommandData.Temporary.dac_out[3] = 0x8000;
  CommandData.Temporary.dac_out[4] = 0x8000;

  for (i=0; i<6; i++) {
    CommandData.hk[i].pump_heat = 0;
    CommandData.hk[i].heat_switch = 1;	  //heat switch defaults to on
    CommandData.hk[i].fphi_heat = 0;
    for (j=0; j<4; j++) CommandData.hk[i].tile_heat[j] = 0;
    CommandData.hk[i].fplo_heat = 0.0;
    CommandData.hk[i].ssa_heat = 0.0;

    CommandData.hk[i].cernox.phase = 5.0;
    CommandData.hk[i].cernox.ampl = 0.3;
    CommandData.hk[i].ntd.phase = 5.0;
    CommandData.hk[i].ntd.ampl = 0.3;
  }
  CommandData.hk_last = 0;
  CommandData.hk_tile_last = 0;
  CommandData.hk_pulse_last = 0;
  CommandData.hk_vheat_last = 0.0;
  CommandData.hk_bias_freq = 50;

  CommandData.actbus.tc_mode = TC_MODE_VETOED;
  CommandData.actbus.tc_step = 100; /* microns */
  CommandData.actbus.tc_wait = 3000; /* = 10 minutes in 5-Hz frames */
  CommandData.actbus.tc_spread = 5; /* centigrade degrees */
  CommandData.actbus.tc_prefp = 1;
  CommandData.actbus.tc_prefs = 1;

  CommandData.actbus.lvdt_delta = 1000;
  CommandData.actbus.lvdt_low = 25000;
  CommandData.actbus.lvdt_high = 35000;

  CommandData.actbus.offset[0] = 33333;
  CommandData.actbus.offset[1] = 33333;
  CommandData.actbus.offset[2] = 33333;

  /* The first is due to change in radius of curvature, the second due to
   * displacement of the secondary due to the rigid struts */

  /* Don sez:   50.23 + 9.9 and 13.85 - 2.2 */
  /* Marco sez: 56          and 10          */
  
  CommandData.actbus.g_primary = 56; /* um/deg */
  CommandData.actbus.g_secondary = 10; /* um/deg */
  CommandData.actbus.focus = 0;
  CommandData.actbus.sf_time = 0;
  CommandData.actbus.sf_offset = 0;

  CommandData.actbus.act_vel = 10;
  CommandData.actbus.act_acc = 1;
  CommandData.actbus.act_move_i = 85;
  CommandData.actbus.act_hold_i = 40;
  CommandData.actbus.act_tol = 2;

  CommandData.actbus.lock_vel = 110000;
  CommandData.actbus.lock_acc = 100;
  CommandData.actbus.lock_move_i = 50;
  CommandData.actbus.lock_hold_i = 0;

  CommandData.hwpr.vel = 1600;
  CommandData.hwpr.acc = 4;
  CommandData.hwpr.move_i = 20;
  CommandData.hwpr.hold_i = 10;

  /* hwpr positions separated by 22.5 degs.
     Calculated by Tristan on July 19th 2010*/
  CommandData.hwpr.pos[3] = 0.263096; 
  CommandData.hwpr.pos[2] = 0.326452;
  CommandData.hwpr.pos[1] = 0.38909;
  CommandData.hwpr.pos[0] = 0.450767;

  CommandData.hwpr.overshoot = 300;
  CommandData.hwpr.i_pos = 0;
  CommandData.hwpr.no_step = 0;
  CommandData.hwpr.use_pot = 1;
  CommandData.hwpr.pot_targ = 0.5;

  CommandData.pin_is_in = 1;

  CommandData.temp1 = 0;
  CommandData.temp2 = 0;
  CommandData.temp3 = 0;

  CommandData.lat = -77.86;  //McMurdo Building 096
  CommandData.lon = -167.04; //Willy Field Dec 2010

  WritePrevStatus();
}
