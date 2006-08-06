/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2006 University of Toronto
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>

#include "mcp.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "tx.h"

/* Define this symbol to have mcp read and use the translation stage */
#undef USE_XY_STAGE

/* Define this symbol to have mcp log all actuator bus traffic */
#define ACTBUS_CHATTER
static int __inhibit_chatter = 0;

#ifdef BOLOTEST
#  define ACT_BUS "/dev/ttyS0"
#else
#  define ACT_BUS "/dev/ttyS7"
#endif

#define LVDT_RADIUS ACTUATOR_RADIUS
#define ACTUATOR_RADIUS 144.338 /* in mm */

#define ENCODER_TOL 5

#define LOCKNUM 3
#ifdef USE_XY_STAGE
#  define STAGEXNUM 4
#  define STAGEYNUM 5
#  define NACT 6
#else
#  define NACT 4
#endif
#define POLL_TIMEOUT 30000 /* 5 minutes */

/* EZ Stepper status bits */
#define EZ_ERROR  0x0F
#define EZ_READY  0x20
#define EZ_STATUS 0x40

/* ActBus status bits ... these must live in the top byte of the word */
#define ACTBUS_OK       0x0000
#define ACTBUS_OOD      0x0100
#define ACTBUS_TIMEOUT  0x0200
#define ACTBUS_CHECKSUM 0x0400

#define ACT_RECV_ABORT 3000000 /* state machine state for general parsing
                                  abort */

#define LOCK_MOTOR_DATA_TIMER 100

#define LOCK_MIN_POT 3000
#define LOCK_MAX_POT 16365
#define LOCK_POT_RANGE 1000

/* in commands.c */
double LockPosition(double elevation);

extern short int InCharge; /* tx.c */

#define LAST_ACTUATOR 2
static int bus_fd = -1;
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
  "Lock Motor"
#ifdef USE_XY_STAGE
  , "XY Stage X", "XY Stage Y"
#endif
};
static const int id[NACT] = {0x31, 0x32, 0x33, 0x35
#ifdef USE_XY_STAGE
  , 0x36, 0x37
#endif
};

static char gp_buffer[1000];
static struct stepper_struct {
  unsigned char buffer[1000];
  int status;
  int sequence;
} stepper[NACT];

static struct lock_struct {
  unsigned int pos;
  unsigned short adc[4];
  unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

static struct act_struct {
  unsigned int pos;
  unsigned int enc;
} act_data[3];

static struct lvdt_struct {
  double lvdt10, lvdt11, lvdt13;
} lvdt_data;

static struct sec_struct {
  double tilt, rotation, offset;
} sec_data[2];

#ifdef USE_XY_STAGE
static struct stage_struct {
  unsigned int xpos, ypos;
  unsigned int xlim, ylim;
  unsigned int xstp, ystp;
  unsigned int xstr, ystr;
  unsigned int xvel, yvel;
} stage_data;
#endif

static int bus_seized = -1;
static int bus_underride = -1;

static int act_setserial(char *input_tty)
{
  int fd;
  struct termios term;

  if ((fd = open(input_tty, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    berror(tfatal, "ActBus: Unable to open serial port");

  if (tcgetattr(fd, &term))
    berror(tfatal, "ActBus: Unable to get serial device attributes");

  /* Clear Character size; set no stop bits; set one parity bit */
  term.c_cflag &= ~(CSTOPB | CSIZE | PARENB);

  /* Set 8 data bits; set local port; enable receiver */
  term.c_cflag |= (CS8 | CLOCAL | CREAD);

  /* Disable all software flow control */
  term.c_iflag &= ~(IXON | IXOFF | IXANY);

  /* disable output processing (raw output) */
  term.c_oflag &= ~OPOST;

  /* disable input processing (raw input) */
  term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  if(cfsetospeed(&term, B9600))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "ActBus: Error setting serial output speed");

  if(cfsetispeed(&term, B9600))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "ActBus: Error setting serial input speed");

  if( tcsetattr(fd, TCSANOW, &term) )
    berror(tfatal, "ActBus: Unable to set serial attributes");

  return fd;
}

static int TakeBus(int who)
{
  if (bus_seized != -1 && bus_seized != bus_underride)
    return 0;

  if (bus_seized != who) {
    bprintf(info, "ActBus: Bus seized by %s.\n", name[who]);
    bus_seized = who;
  }

  return 1;
}

static inline void ReleaseBus(int who)
{
  if (bus_seized == who) {
    bprintf(info, "ActBus: Bus released by %s.\n", name[who]);
    bus_seized = -1;
  }

  if (bus_seized == -1 && bus_underride != -1) {
    bprintf(info, "ActBus: Bus underriden by %s.\n", name[bus_underride]);
    bus_seized = bus_underride;
  }
}

static inline void UnderrideBus(int who)
{
  if (bus_underride != who)
    bprintf(info, "ActBus: Bus underride for %s enabled.\n", name[who]);
  bus_underride = who;
  ReleaseBus(-2);
}

static inline void RemoveUnderride(void)
{
  int i = bus_underride;

  bprintf(info, "ActBus: Bus underride for %s disabled.\n", name[i]);
  bus_underride = -1;
  ReleaseBus(i);
}

static char hex_buffer[1000];
static const char* HexDump(const unsigned char* buffer, int len)
{
  int i;

  sprintf(hex_buffer, "%02x", buffer[0]);
  for (i = 1; i < len; ++i)
    sprintf(hex_buffer + i * 3 - 1, ".%02x", buffer[i]);

  return hex_buffer;
}

static void BusSend(int who, const char* what, int inhibit_chatter)
{
  size_t len = strlen(what) + 5;
  char *buffer = malloc(len);
  unsigned char chk = 3;
  unsigned char *ptr;

  buffer[0] = 0x2;
  buffer[1] = (who >= 0x30) ? who : id[who];
  buffer[2] = '1';
  sprintf(buffer + 3, "%s", what);
  buffer[len - 2] = 0x3;
  for (ptr = buffer; *ptr != '\03'; ++ptr)
    chk ^= *ptr;
  buffer[len - 1] = chk;
#ifdef ACTBUS_CHATTER
  if (!inhibit_chatter)
    bprintf(info, "ActBus: Request=%s", HexDump(buffer, len));
#endif
  if (write(bus_fd, buffer, len) < 0)
    berror(err, "Error writing on bus");

  free(buffer);
}

static int BusRecv(char* buffer, int nic, int inhibit_chatter)
{
  int fd, status = 0;
  fd_set rfds;
  struct timeval timeout = {2, 0};
  unsigned char byte;
  unsigned char checksum = 0;
  char* ptr = buffer;

  FD_ZERO(&rfds);
  FD_SET(bus_fd, &rfds);

  fd = select(bus_fd + 1, &rfds, NULL, NULL, &timeout);

  if (fd == -1)
    berror(err, "Error waiting for input on bus");
  else if (!fd) /* Timeout */
    return ACTBUS_TIMEOUT;
  else {
    int state = (nic) ? ACT_RECV_ABORT : 0;
    int had_errors = 0;
    int read_tries = 100;
    int len;

    for(;;) {
      len = read(bus_fd, &byte, 1);
      if (len <= 0) {
        if (state == 6 || state == ACT_RECV_ABORT)
          break;
        if (errno == EAGAIN && read_tries) {
          read_tries--;
          usleep(1000);
          continue;
        } else {
          berror(warning, "Unexpected out-of-data reading bus (%i)", state);
          return ACTBUS_OOD;
        }
      }
      checksum ^= byte;

      /* The following involves a number of semi-hidden fallthroughs which
       * attempt to recover malformed strings */
      switch (state) {
        case 0: /* RS-485 turnaround */
          state++;
          if (byte != 0xFF) { /* RS-485 turnaround */
            bputs(warning, "ActBus: RS-485 turnaround not found in response");
            had_errors++;
          } else
            break;
        case 1: /* start byte */
          state++;
          if (byte != 0x02) { /* STX */
            had_errors++;
            bputs(warning, "ActBus: Start byte not found in response");
          } else
            break;
        case 2: /* address byte */
          state++;
          if (byte != 0x30) { /* Recipient address (should be '0') */
            had_errors++;
            bputs(warning, "ActBus: Found misaddressed response");
          }
          if (had_errors > 1) {
            bputs(err,
                "ActBus: Too many errors parsing response string, aborting.");
            bprintf(err, "ActBus: Response was=%s (%x)\n", HexDump(buffer, len),
                status);
            state = ACT_RECV_ABORT;
          }
          break;
        case 3: /* state byte */
          state++;
          if (byte & EZ_STATUS)
            status = byte & (EZ_ERROR | EZ_READY);
          else {
            bputs(err,
                "ActBus: Status byte malfomed in response string, aborting.");
            state = ACT_RECV_ABORT;
          }
          break;
        case 4: /* response */
          if (byte == 0x3) /* ETX */
            state++;
          else
            *(ptr++) = byte;
          break;
        case 5: /* checksum */
          state++;
          /* Remember: the checksum here should be 0xff instead of 0 because
           * we've added the turnaround byte into the checksum */
          if (checksum != 0xff)
            bprintf(err, "ActBus: Checksum error in response (%02x).",
                checksum);
          break;
        case 6: /* End of string check */
          bputs(err, "ActBus: Malformed footer in response string, aborting.");
          state = ACT_RECV_ABORT;
        case ACT_RECV_ABORT: /* General abort: flush input */
          break;
      }
    }
  }

  if (!nic) {
    *ptr = '\0';

#ifdef ACTBUS_CHATTER
    if (!inhibit_chatter)
      bprintf(info, "ActBus: Response=%s (%x)\n", buffer, status);
#endif
  }

  return status;
}

static int ReadIntFromBus(int who, const char* cmd, int inhibit_chatter)
{
  int result;

  BusSend(who, cmd, inhibit_chatter);
  if ((result = BusRecv(gp_buffer, 0, inhibit_chatter)) & (ACTBUS_TIMEOUT
        | ACTBUS_OOD)) {
    bprintf(warning, "ActBus: Timeout waiting for response from %s (RIFB)",
        name[who]);
    CommandData.actbus.force_repoll = 1;
    return 0;
  }

  return atoi(gp_buffer);
}

static void ReadActuator(int who, int inhibit_chatter)
{
  if (stepper[who].status == -1)
    return;

  act_data[who].pos = ReadIntFromBus(who, "?0", inhibit_chatter);
  act_data[who].enc = ReadIntFromBus(who, "?8", inhibit_chatter);
}

static void DiscardBusRecv(int flag, int who, int inhibit_chatter)
{
  int i;

  /* Discard response to get it off the bus */
  if ((i = BusRecv(gp_buffer, 0, inhibit_chatter)) & (ACTBUS_TIMEOUT
        | ACTBUS_OOD))
  {
    bprintf(warning,
        "ActBus: Timeout waiting for response from %s.", name[who]);
    CommandData.actbus.force_repoll = 1;
  }
#ifndef ACTBUS_CHATTER
  else if (flag)
    bprintf(info, "ActBus: Controller response: %s\n", gp_buffer);
#endif
}

static void InitialiseActuator(int who)
{
  int enc;
  double lvdt10 = lvdt_data.lvdt10 * LVDT10_TO_MM - LVDT10_ZERO;
  double lvdt11 = lvdt_data.lvdt11 * LVDT11_TO_MM - LVDT11_ZERO;
  double lvdt13 = lvdt_data.lvdt13 * LVDT13_TO_MM - LVDT13_ZERO;
  char buffer[1000];

  ReadActuator(who, __inhibit_chatter); 

  if (act_data[who].enc < 500000) {
    /* Calculate nominal encoder position from LVDTs */
    if (who == 0) 
      enc = 2 * (int)(((lvdt10 + lvdt13) - lvdt11) / (3 * ACTENC_TO_MM));
    else if (who == 1)
      enc = 2 * (int)(((lvdt11 + lvdt10) - lvdt13) / (3 * ACTENC_TO_MM));
    else
      enc = 2 * (int)(((lvdt13 + lvdt11) - lvdt10) / (3 * ACTENC_TO_MM));

    bprintf(info, "Initialising Actuator #%i to %i", who, enc);

    /* Add a million */
    enc += 1000000;

    /* Set the encoder */
    sprintf(buffer, "z%iR", enc);  
    BusSend(who, buffer, __inhibit_chatter);
    DiscardBusRecv(0, who, __inhibit_chatter);
  }
}

static void ServoActuators(int* goal)
{
  int i;
  int act_there[3] = {0, 0, 0};
  char buffer[1000];

  while (!act_there[0] && !act_there[1] && !act_there[2])
    for (i = 0; i < 3; ++i)
      if (act_there[i] || stepper[i].status == -1) {
        act_there[i] = 1;
        continue;
      } else if (act_data[i].enc < 500000)
        InitialiseActuator(i);
      else {
        int delta = act_data[i].enc - 1000000 - goal[i];
        if (abs(delta) <= ENCODER_TOL) {
          act_there[i] = 1;
          strcpy(buffer, "T");
        } else if (delta > 0)
          sprintf(buffer, "P%iR", delta);
        else
          sprintf(buffer, "D%iR", delta);
        BusSend(i, buffer, __inhibit_chatter);
        DiscardBusRecv(0, i, __inhibit_chatter);
      }
}

static int PollBus(int rescan)
{
  int i, result;
  int all_ok = 1;

  if (rescan)
    bputs(info, "ActBus: Repolling Actuator Bus.");
  else
    bputs(info, "ActBus: Polling Actuator Bus.");

  for (i = 0; i < NACT; ++i) {
    if (rescan && stepper[i].status != -1)
      continue;
    BusSend(i, "&", __inhibit_chatter);
    if ((result = BusRecv(gp_buffer, 0, __inhibit_chatter)) & (ACTBUS_TIMEOUT
          | ACTBUS_OOD)) {
      bprintf(warning, "ActBus: No response from %s, will repoll later.",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    } else if (!strncmp(gp_buffer, "EZHR17EN AllMotion", 18)) {
      bprintf(info, "ActBus: Found type 17EN device %s at address %i.\n",
          name[i], id[i] - 0x30);
      stepper[i].status = 0;
    } else if (!strncmp(gp_buffer, "EZHR23 All Motion", 17)) {
      bprintf(info, "ActBus: Found type 23 device %s at address %i.\n", name[i],
          id[i] - 0x30);
      stepper[i].status = 0;
    } else {
      bprintf(warning,
          "ActBus: Unrecognised response from %s, will repoll later.\n",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    }

    if (stepper[i].status == 0 && i <= LAST_ACTUATOR)
      InitialiseActuator(i);
  }

  CommandData.actbus.force_repoll = 0;

  return all_ok;
}

#ifdef USE_XY_STAGE
static void ReadStage(void)
{
  static int counter = 0;
  if (stepper[STAGEXNUM].status == -1 || stepper[STAGEYNUM].status == -1)
    return;

  stage_data.xpos = ReadIntFromBus(STAGEXNUM, "?0");
  stage_data.ypos = ReadIntFromBus(STAGEYNUM, "?0");

  if (counter == 0)
    stage_data.xstr = ReadIntFromBus(STAGEXNUM, "?1");
  else if (counter == 1)
    stage_data.xstp = ReadIntFromBus(STAGEXNUM, "?3");
  else if (counter == 2)
    stage_data.xlim = ReadIntFromBus(STAGEXNUM, "?4");
  else if (counter == 3)
    stage_data.xvel = ReadIntFromBus(STAGEXNUM, "?5");
  else if (counter == 4)
    stage_data.ystr = ReadIntFromBus(STAGEYNUM, "?1");
  else if (counter == 5)
    stage_data.ystp = ReadIntFromBus(STAGEYNUM, "?3");
  else if (counter == 6)
    stage_data.yvel = ReadIntFromBus(STAGEYNUM, "?5");
  else if (counter == 7)
    stage_data.ylim = ReadIntFromBus(STAGEYNUM, "?4");

  counter = (counter + 1) % 8;
}
#endif

static void GetLockData(int mult)
{
  static int counter = 0;
  int result;

  if (stepper[LOCKNUM].status == -1)
    return;

  if (counter++ < mult * LOCK_MOTOR_DATA_TIMER)
    return;

  counter = 0;

  lock_data.pos = ReadIntFromBus(LOCKNUM, "?0", 1);

  BusSend(LOCKNUM, "?aa", 1);
  if ((result = BusRecv(gp_buffer, 0, 1)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
    bputs(warning, "ActBus: Timeout waiting for response from lock motor.");
    CommandData.actbus.force_repoll = 1;
    return;
  }

  sscanf(gp_buffer, "%hi,%hi,%hi,%hi", &lock_data.adc[0], &lock_data.adc[1],
      &lock_data.adc[2], &lock_data.adc[3]);
}

/* The NiC MCC does this via the BlastBus to give it a chance to know what's
 * going on.  The ICC reads it directly to get more promptly the answer
 * (since all these fields are slow). */
static void SetLockState(int nic)
{
  static int firsttime = 1;
  int pot = lock_data.adc[1];
  int ls = lock_data.adc[2];
  unsigned int state = lock_data.state; 

  static struct BiPhaseStruct* lockAdc1Addr;
  static struct BiPhaseStruct* lockAdc2Addr;
  static struct BiPhaseStruct* lockStateAddr;

  if (firsttime) {
    firsttime = 0;
    lockAdc1Addr = GetBiPhaseAddr("lock_adc1");
    lockAdc2Addr = GetBiPhaseAddr("lock_adc2");
    lockStateAddr = GetBiPhaseAddr("lock_state");
  }

  if (nic) {
    pot = slow_data[lockAdc1Addr->index][lockAdc1Addr->channel];
    ls = slow_data[lockAdc2Addr->index][lockAdc2Addr->channel];
    state = slow_data[lockStateAddr->index][lockStateAddr->channel];
  }

  state &= LS_DRIVE_MASK; /* zero everything but drive info */

  if (pot < LOCK_MIN_POT)
    state |= LS_CLOSED;
  else if (pot > LOCK_MAX_POT) {
    state |= LS_POT_RAIL;
    if (ls < 8000)
      state |= LS_OPEN;
  } else if ((pot < LOCK_MIN_POT + LOCK_POT_RANGE)
      || (pot > LOCK_MAX_POT - LOCK_POT_RANGE))
    state |= lock_data.state & (LS_OPEN | LS_CLOSED);

  if (fabs(ACSData.enc_elev - LockPosition(CommandData.pointing_mode.Y)) <= 0.5)
    state |= LS_EL_OK;

  /* Assume the pin is out unless we're all the way closed */
  if (state & LS_CLOSED)
    CommandData.pin_is_in = 1;
  else
    CommandData.pin_is_in = 0;

  lock_data.state = state;
}

/************************************************************************/
/*                                                                      */
/*    Do Lock Logic: check status, determine if we are locked, etc      */
/*                                                                      */
/************************************************************************/
#define LOCK_VEL   "200000" /* Max torque is 50000 */
#define SEND_SLEEP 100000 /* .1 seconds */
#define WAIT_SLEEP 50000 /* .05 seconds */
#define LA_EXIT    0
#define LA_STOP    1
#define LA_WAIT    2
#define LA_EXTEND  3
#define LA_RETRACT 4
#define LA_STEP    5
static void DoLock(void)
{
  int action = LA_EXIT;
  const char* command = NULL;

  do {
    GetLockData((bus_seized == LOCKNUM) ? 0 : 1);

    /* Fix weird states */
    if ((lock_data.state & (LS_DRIVE_EXT | LS_DRIVE_RET | LS_DRIVE_UNK)
          && lock_data.state & LS_DRIVE_OFF)
        || CommandData.actbus.lock_goal & LS_DRIVE_FORCE) {
      lock_data.state &= ~LS_DRIVE_MASK | LS_DRIVE_UNK;
      CommandData.actbus.lock_goal &= ~LS_DRIVE_FORCE;
      bprintf(info, "ActBus: Reset lock motor state.");
    }

    SetLockState(0);

    /* compare goal to current state -- only 3 goals are supported:
     * open + off, closed + off and off */
    if ((CommandData.actbus.lock_goal & 0x7) == (LS_OPEN | LS_DRIVE_OFF)) {
      /*                                       ORe -.
       * cUe -+-(stp)- cFe -(ext)- cXe -(---)- OXe -+-(stp)- OFe ->
       * cRe -'                                OUe -'
       */
      if ((lock_data.state & (LS_OPEN | LS_DRIVE_OFF))
          == (LS_OPEN | LS_DRIVE_OFF))
        action = LA_EXIT;
      else if (lock_data.state & LS_OPEN)
        action = LA_STOP;
      else if (lock_data.state & LS_DRIVE_RET)
        action = LA_WAIT;
      else if (lock_data.state & LS_DRIVE_OFF)
        action = LA_RETRACT;
      else
        action = LA_STOP;
    } else if ((CommandData.actbus.lock_goal & 0x7) == (LS_CLOSED
          | LS_DRIVE_OFF)) {
      /* oX -.         oUE -(stp)-.              CRe -(stp)-+
       * oR -+-(stp) - oF  -(---)-+- oFE -(ret)- oRE -(---)-+- CFe ->
       * oU -'         oXE -(stp)-'              CUe -(stp)-+
       *                                         CXe -(stp)-'
       */
      if ((lock_data.state & (LS_CLOSED | LS_DRIVE_OFF))
          == (LS_CLOSED | LS_DRIVE_OFF)) 
        action = LA_EXIT;
      else if (lock_data.state & LS_CLOSED)
        action = LA_STOP;
      else if (lock_data.state & LS_EL_OK
          || CommandData.actbus.lock_goal & LS_IGNORE_EL) { /* el in range */
        if ((lock_data.state & (LS_OPEN | LS_DRIVE_STP))
            == (LS_OPEN | LS_DRIVE_STP))
          action = LA_WAIT;
        else if (lock_data.state & LS_DRIVE_EXT)
          action = LA_WAIT;
        else if (lock_data.state & LS_DRIVE_STP)
          action = LA_STOP;
        else if (lock_data.state & LS_DRIVE_OFF)
          action = (lock_data.state & LS_OPEN) ? LA_STEP : LA_EXTEND;
        else
          action = LA_STOP;
      } else { /* el out of range */
        action = (lock_data.state & LS_DRIVE_OFF) ? LA_WAIT : LA_STOP;
      }
    } else if ((CommandData.actbus.lock_goal & 0x7) == LS_DRIVE_OFF)
      /* ocXe -.
       * ocRe -+-(stp)- ocFe ->
       * ocUe -+
       * ocSe -'
       */
      action = (lock_data.state & LS_DRIVE_OFF) ? LA_EXIT : LA_STOP;
    else {
      bprintf(warning, "ActBus: Unhandled lock goal (%x) ignored.",
          CommandData.actbus.lock_goal);
      CommandData.actbus.lock_goal = LS_DRIVE_OFF;
    }

    /* Seize the bus */
    if (action == LA_EXIT)
      ReleaseBus(LOCKNUM);
    else
      TakeBus(LOCKNUM);

    /* Figure out what to do... */
    switch (action) {
      case LA_STOP:
        bputs(info, "ActBus: Stopping lock motor.");
        command = "T"; /* terminate all strings */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_OFF;
        break;
      case LA_EXTEND:
        bputs(info, "ActBus: Extending lock motor.");
        command = "V" LOCK_VEL "P0R"; /* move out forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_EXT;
        break;
      case LA_RETRACT:
        bputs(info, "ActBus: Retracting lock motor.");
        command = "V" LOCK_VEL "D0R"; /* move in forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_RET;
        break;
      case LA_STEP:
        bputs(info, "ActBus: Stepping lock motor.");
        command = "V" LOCK_VEL "P50000R"; /* move away from the limit switch */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_STP;
        break;
      default:
        command = NULL;
    }

    /* ... and do it! */
    if (command != NULL) {
      BusSend(LOCKNUM, command, __inhibit_chatter);
      DiscardBusRecv(0, LOCKNUM, __inhibit_chatter);
      usleep(SEND_SLEEP); /* wait for a bit */
    } else if (action == LA_WAIT)
      usleep(WAIT_SLEEP); /* wait for a bit */
  } while (action != LA_EXIT);
}

static inline struct NiosStruct* GetActNiosAddr(int i, const char* field)
{
  sprintf(gp_buffer, "act%i_%s", i, field);

  return GetNiosAddr(gp_buffer);
}

/* This function is called by the frame control thread */
void StoreActBus(void)
{
  int i, j;
  static int firsttime = 1;

  static struct NiosStruct* actbusResetAddr;
  static struct NiosStruct* lockPosAddr;
  static struct NiosStruct* lockStateAddr;
  static struct NiosStruct* lockGoalAddr;
  static struct NiosStruct* seizedBusAddr;
  static struct NiosStruct* lockAdcAddr[4];
  static struct NiosStruct* lokmotPinAddr;

  static struct NiosStruct* actPosAddr[3];
  static struct NiosStruct* actEncAddr[3];

  static struct NiosStruct* secEtiltAddr;
  static struct NiosStruct* secErotAddr;
  static struct NiosStruct* secEoffAddr;
  static struct NiosStruct* secLtiltAddr;
  static struct NiosStruct* secLrotAddr;
  static struct NiosStruct* secLoffAddr;

  static struct NiosStruct* gTPrimAddr;
  static struct NiosStruct* gTSecAddr;
  static struct NiosStruct* secFocusPosAddr;
  static struct NiosStruct* secTiltGoalAddr;
  static struct NiosStruct* secRotGoalAddr;
  static struct NiosStruct* focusVetoAddr;

#ifdef USE_XY_STAGE
  static struct NiosStruct* stageXAddr;
  static struct NiosStruct* stageXLimAddr;
  static struct NiosStruct* stageXStpAddr;
  static struct NiosStruct* stageXStrAddr;
  static struct NiosStruct* stageXVelAddr;

  static struct NiosStruct* stageYAddr;
  static struct NiosStruct* stageYLimAddr;
  static struct NiosStruct* stageYStpAddr;
  static struct NiosStruct* stageYStrAddr;
  static struct NiosStruct* stageYVelAddr;
#endif

  if (firsttime) {
    firsttime = 0;
    actbusResetAddr = GetNiosAddr("actbus_reset");
    lokmotPinAddr = GetNiosAddr("lokmot_pin");
    lockPosAddr = GetNiosAddr("lock_pos");
    lockStateAddr = GetNiosAddr("lock_state");
    seizedBusAddr = GetNiosAddr("seized_bus");
    lockGoalAddr = GetNiosAddr("lock_goal");
    lockAdcAddr[0] = GetNiosAddr("lock_adc0");
    lockAdcAddr[1] = GetNiosAddr("lock_adc1");
    lockAdcAddr[2] = GetNiosAddr("lock_adc2");
    lockAdcAddr[3] = GetNiosAddr("lock_adc3");

    for (i = 0; i < 3; ++i) {
      actPosAddr[i] = GetActNiosAddr(i, "pos");
      actEncAddr[i] = GetActNiosAddr(i, "enc");
    }

    secEtiltAddr = GetNiosAddr("sec_etilt");
    secErotAddr = GetNiosAddr("sec_erot");
    secEoffAddr = GetNiosAddr("sec_eoff");
    secLtiltAddr = GetNiosAddr("sec_ltilt");
    secLrotAddr = GetNiosAddr("sec_lrot");
    secLoffAddr = GetNiosAddr("sec_loff");

    gTPrimAddr = GetNiosAddr("g_t_prim");
    gTSecAddr = GetNiosAddr("g_t_sec");
    secFocusPosAddr = GetNiosAddr("sec_focus_pos");
    secTiltGoalAddr = GetNiosAddr("sec_tilt_goal");
    secRotGoalAddr = GetNiosAddr("sec_rot_goal");
    focusVetoAddr = GetNiosAddr("focus_veto");

#ifdef USE_XY_STAGE
    stageXAddr = GetNiosAddr("stage_x");
    stageXLimAddr = GetNiosAddr("stage_x_lim");
    stageXStrAddr = GetNiosAddr("stage_x_str");
    stageXStpAddr = GetNiosAddr("stage_x_stp");
    stageXVelAddr = GetNiosAddr("stage_x_vel");
    stageYAddr = GetNiosAddr("stage_y");
    stageYLimAddr = GetNiosAddr("stage_y_lim");
    stageYStrAddr = GetNiosAddr("stage_y_str");
    stageYStpAddr = GetNiosAddr("stage_y_stp");
    stageYVelAddr = GetNiosAddr("stage_y_vel");
#endif
  }

  WriteData(actbusResetAddr, CommandData.actbus.off, NIOS_QUEUE);

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);

  for (j = 0; j < 3; ++j) {
    WriteData(actPosAddr[j], act_data[j].pos, NIOS_QUEUE);
    WriteData(actEncAddr[j], act_data[j].enc, NIOS_QUEUE);
  }

  for (i = 0; i < 4; ++i)
    WriteData(lockAdcAddr[i], lock_data.adc[i], NIOS_QUEUE);
  WriteData(lockStateAddr, lock_data.state, NIOS_QUEUE);
  WriteData(seizedBusAddr, bus_seized, NIOS_QUEUE);
  WriteData(lockGoalAddr, CommandData.actbus.lock_goal, NIOS_QUEUE);
  WriteData(lockPosAddr, lock_data.pos, NIOS_FLUSH);

  WriteData(secEtiltAddr, sec_data[0].tilt * RAD2I * 40, NIOS_QUEUE);
  WriteData(secErotAddr, sec_data[0].rotation * RAD2I, NIOS_QUEUE);
  WriteData(secEoffAddr, sec_data[0].offset * 400, NIOS_QUEUE);
  WriteData(secLtiltAddr, sec_data[1].tilt * RAD2I * 40, NIOS_QUEUE);
  WriteData(secLrotAddr, sec_data[1].rotation * RAD2I, NIOS_QUEUE);
  WriteData(secLoffAddr, sec_data[1].offset * 400, NIOS_QUEUE);

  WriteData(gTPrimAddr, CommandData.actbus.g_primary, NIOS_QUEUE);
  WriteData(gTSecAddr, CommandData.actbus.g_secondary, NIOS_QUEUE);
  WriteData(focusVetoAddr, CommandData.actbus.autofocus_vetoed, NIOS_QUEUE);
  WriteData(secFocusPosAddr, CommandData.actbus.focus * 3000, NIOS_QUEUE);
  WriteData(secTiltGoalAddr, CommandData.actbus.tilt * 30000, NIOS_QUEUE);
  WriteData(secRotGoalAddr, CommandData.actbus.rotation * DEG2I, NIOS_QUEUE);

#ifdef USE_XY_STAGE
  WriteData(stageXAddr, stage_data.xpos, NIOS_QUEUE);
  WriteData(stageXLimAddr, stage_data.xlim, NIOS_QUEUE);
  WriteData(stageXStrAddr, stage_data.xstr, NIOS_QUEUE);
  WriteData(stageXStpAddr, stage_data.xstp, NIOS_QUEUE);
  WriteData(stageXVelAddr, stage_data.xvel, NIOS_QUEUE);
  WriteData(stageYAddr, stage_data.ypos, NIOS_QUEUE);
  WriteData(stageYStrAddr, stage_data.ystr, NIOS_QUEUE);
  WriteData(stageYStpAddr, stage_data.ystp, NIOS_QUEUE);
  WriteData(stageYVelAddr, stage_data.yvel, NIOS_QUEUE);
  WriteData(stageYLimAddr, stage_data.ylim, NIOS_FLUSH);
#endif
}

static void SolveSecondary(void)
{
  static int firsttime = 1;

  static struct BiPhaseStruct* lvdt10Addr;
  static struct BiPhaseStruct* lvdt11Addr;
  static struct BiPhaseStruct* lvdt13Addr;

  if (firsttime) {
    firsttime = 0;
    lvdt10Addr = GetBiPhaseAddr("lvdt_10");
    lvdt11Addr = GetBiPhaseAddr("lvdt_11");
    lvdt13Addr = GetBiPhaseAddr("lvdt_13");
  }

  lvdt_data.lvdt10 = slow_data[lvdt10Addr->index][lvdt10Addr->channel];
  lvdt_data.lvdt11 = slow_data[lvdt11Addr->index][lvdt11Addr->channel];
  lvdt_data.lvdt13 = slow_data[lvdt13Addr->index][lvdt13Addr->channel];

  /* encoder based solution */
  double alpha = act_data[0].enc * ACTENC_TO_MM;
  double beta = act_data[1].enc * ACTENC_TO_MM;
  double gamma = act_data[2].enc * ACTENC_TO_MM;
  double A = 2 * sqrt(alpha * alpha + beta * beta + gamma * gamma
      - alpha * beta - beta * gamma - gamma * alpha) / 3;
  sec_data[0].offset = (alpha + beta + gamma) / 3;
  sec_data[0].rotation = atan2(sqrt(3) * (alpha - gamma), 2 * beta - gamma
      + alpha);
  sec_data[0].tilt = asin(A / ACTUATOR_RADIUS); 

  /* lvdt based solution */
  alpha = lvdt_data.lvdt10 * LVDT10_TO_MM - LVDT10_ZERO;
  beta = lvdt_data.lvdt13 * LVDT13_TO_MM - LVDT13_ZERO;
  gamma = lvdt_data.lvdt11 * LVDT11_TO_MM - LVDT11_ZERO;
  A = 2 * sqrt(alpha * alpha + beta * beta + gamma * gamma
      - alpha * beta - beta * gamma - gamma * alpha) / 3;
  sec_data[1].offset = (alpha + beta + gamma) / 3;
  sec_data[1].rotation = atan2(sqrt(3) * (alpha - gamma), 2 * beta - gamma
      + alpha) + M_PI / 6;
  sec_data[1].tilt = asin(A / LVDT_RADIUS); 
}

void ActuatorBus(void)
{
  int poll_timeout = POLL_TIMEOUT;
  int all_ok = 0;
  int i;
  int my_cindex = 0;

  bputs(startup, "ActBus: ActuatorBus startup.");

  for (i = 0; i < NACT; ++i)
    stepper[i].sequence = 1;

  bus_fd = act_setserial(ACT_BUS);

  all_ok = PollBus(0);

  for (;;) {
    while (!InCharge) { /* NiC MCC traps here */
      CommandData.actbus.force_repoll = 1; /* repoll bus as soon as gaining
                                              control */
      BusRecv(NULL, 1, 1); /* this is a blocking call - clear the recv buffer */
      SetLockState(1); /* to ensure the NiC MCC knows the pin state */
      /* no need to sleep -- BusRecv does that for us */
    }

    /* Repoll bus if necessary */
    if (CommandData.actbus.force_repoll) {
      poll_timeout = 0;
      all_ok = 0;
      for (i = 0; i < NACT; ++i)
        stepper[i].status = -1;
    }

    if (poll_timeout == 0 && !all_ok) {
      all_ok = PollBus(1);
      poll_timeout = POLL_TIMEOUT;
    } else if (poll_timeout > 0)
      poll_timeout--;

    /* Send the uplinked command, if any */
    my_cindex = GETREADINDEX(CommandData.actbus.cindex);
    if (CommandData.actbus.caddr[my_cindex] != 0) {
      BusSend(CommandData.actbus.caddr[my_cindex],
          CommandData.actbus.command[my_cindex], __inhibit_chatter);
      /* Discard response to get it off the bus */
      if (CommandData.actbus.caddr[my_cindex] < NACT)
        DiscardBusRecv(1, CommandData.actbus.caddr[my_cindex],
            __inhibit_chatter);
      CommandData.actbus.caddr[my_cindex] = 0;
    }

#ifdef USE_XY_STAGE
    UnderrideBus(STAGEXNUM);
#endif

    DoLock(); /* Lock motor stuff -- this will seize the bus until
                 the lock motor's state has settled */

#ifdef USE_XY_STAGE
    if (bus_seized == STAGEXNUM)
      ReadStage();
#endif

    for (i = 0; i < 3; ++i)
      ReadActuator(i, 1);

    SolveSecondary();

    usleep(10000);
  }
}
