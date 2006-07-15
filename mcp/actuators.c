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
#undef ACTBUS_CHATTER
int __inhibit_chatter = 0;

#ifdef BOLOTEST
#  define ACT_BUS "/dev/ttyS0"
#else
#  define ACT_BUS "/dev/ttyS7"
#endif

#define LVDT_RADIUS ACTUATOR_RADIUS
#define ACTUATOR_RADIUS 144.338 /* in mm */

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

struct lock_struct {
  unsigned int pos;
  unsigned short adc[4];
  unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

struct act_struct {
  unsigned int pos;
  unsigned int enc;
  unsigned short adc[4];
} act_data[3];

struct sec_struct {
  double tilt, rotation, offset;
} sec_data[2];

#ifdef USE_XY_STAGE
struct stage_struct {
  unsigned int xpos, ypos;
  unsigned int xlim, ylim;
  unsigned int xstp, ystp;
  unsigned int xstr, ystr;
  unsigned int xvel, yvel;
} stage_data;
#endif

int bus_seized = -1;
int bus_underride = -1;

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

static void BusSend(int who, const char* what)
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
  if (!__inhibit_chatter)
    bprintf(info, "ActBus: Request=%s", HexDump(buffer, len));
#endif
  if (write(bus_fd, buffer, len) < 0)
    berror(err, "Error writing on bus");

  free(buffer);
}

static int BusRecv(char* buffer, int nic)
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

#if 0
      bprintf(info, "%02x. (%i/%i)", byte, state, len);
#endif

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
            bprintf(err, "ActBus: Response was=%s (%x)\n", HexDump(buffer, len), status);
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
    if (!__inhibit_chatter)
      bprintf(info, "ActBus: Response=%s (%x)\n", buffer, status);
#endif
  }

  return status;
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
    BusSend(i, "&");
    if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
      bprintf(warning, "ActBus: No response from %s, will repoll later.",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    } else if (!strncmp(gp_buffer, "EZHR17EN AllMotion", 18)) {
      bprintf(info, "ActBus: Found type 17EN device %s at address %i.\n",
          name[i], i + 1);
      stepper[i].status = 0;
    } else if (!strncmp(gp_buffer, "EZHR23 All Motion", 17)) {
      bprintf(info, "ActBus: Found type 23 device %s at address %i.\n", name[i],
          i + 1);
      stepper[i].status = 0;
    } else {
      bprintf(warning,
          "ActBus: Unrecognised response from %s, will repoll later.\n",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    }
  }

  CommandData.actbus.force_repoll = 0;

  return all_ok;
}

int ReadIntFromBus(int who, const char* cmd)
{
  int result;

  BusSend(who, cmd);
  if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
    bprintf(warning, "ActBus: Timeout waiting for response from %s (RIFB)",
        name[who]);
    stepper[who].status = -1;
    return 0;
  }

  return atoi(gp_buffer);
}

static void ReadActuator(int who)
{
  int result;
  if (stepper[who].status == -1)
    return;

  act_data[who].pos = ReadIntFromBus(who, "?0");
  act_data[who].enc = ReadIntFromBus(who, "?8");
  BusSend(who, "?aa");
  if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
    bprintf(warning, "ActBus: Timeout waiting for response from Actuator #%i",
        who);
    stepper[who].status = -1;
    return;
  }

  sscanf(gp_buffer, "%hi,%hi,%hi,%hi", &act_data[who].adc[0], &act_data[who].adc[1],
      &act_data[who].adc[2], &act_data[who].adc[3]);
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

  lock_data.pos = ReadIntFromBus(LOCKNUM, "?0");

  BusSend(LOCKNUM, "?aa");
  if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
    bputs(warning, "ActBus: Timeout waiting for response from lock motor.");
    stepper[LOCKNUM].status = -1;
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
  else if (pot > LOCK_MAX_POT || ls < 8000)
    state |= LS_OPEN;
  else if ((pot < LOCK_MIN_POT + LOCK_POT_RANGE)
      || (pot > LOCK_MAX_POT - LOCK_POT_RANGE))
    state |= lock_data.state & (LS_OPEN | LS_CLOSED);

  if (fabs(ACSData.enc_elev - LockPosition(ACSData.enc_elev)) <= 0.2)
    state |= LS_EL_OK;

  /* Assume the pin is in unless we're all the way open */
  if (state & LS_OPEN)
    CommandData.pin_is_in = 0;
  else
    CommandData.pin_is_in = 1;

  lock_data.state = state;
}

/************************************************************************/
/*                                                                      */
/*    Do Lock Logic: check status, determine if we are locked, etc      */
/*                                                                      */
/************************************************************************/
#define SEND_SLEEP 100000 /* .1 seconds */
#define WAIT_SLEEP 50000 /* .05 seconds */
#define LA_EXIT    0
#define LA_STOP    1
#define LA_WAIT    2
#define LA_EXTEND  3
#define LA_RETRACT 4
static void DoLock(void)
{
  int action = LA_EXIT;
  int result;
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
        if (lock_data.state & LS_DRIVE_EXT)
          action = LA_WAIT;
        else if (lock_data.state & LS_DRIVE_OFF)
          action = LA_EXTEND;
        else
          action = LA_STOP;
      } else { /* el out of range */
        action = (lock_data.state & LS_DRIVE_OFF) ? LA_WAIT : LA_STOP;
      }
    } else if ((CommandData.actbus.lock_goal & 0x7) == LS_DRIVE_OFF)
      /* ocXe -.
       * ocRe -+-(stp)- ocFe ->
       * ocUe -'
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
        command = "V50000P0R"; /* move out forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_EXT;
        break;
      case LA_RETRACT:
        bputs(info, "ActBus: Retracting lock motor.");
        command = "V50000D0R"; /* move in forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_RET;
        break;
      default:
        command = NULL;
    }

    /* ... and do it! */
    if (command != NULL) {
      BusSend(LOCKNUM, command);
      if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD))
        bputs(warning,
            "ActBus: Timeout waiting for response from lock motor.");
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

  static struct NiosStruct* lockPosAddr;
  static struct NiosStruct* lockStateAddr;
  static struct NiosStruct* lockGoalAddr;
  static struct NiosStruct* seizedBusAddr;
  static struct NiosStruct* lockAdcAddr[4];
  static struct NiosStruct* lokmotPinAddr;
  
  static struct NiosStruct* actPosAddr[3];
  static struct NiosStruct* actEncAddr[3];
#if 0
  static struct NiosStruct* actAdcAddr[3][4];
#endif

  static struct NiosStruct* secEtiltAddr;
  static struct NiosStruct* secErotAddr;
  static struct NiosStruct* secEoffAddr;
  static struct NiosStruct* secLtiltAddr;
  static struct NiosStruct* secLrotAddr;
  static struct NiosStruct* secLoffAddr;

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
#if 0
      actAdcAddr[i][0] = GetActNiosAddr(i, "adc0");
      actAdcAddr[i][1] = GetActNiosAddr(i, "adc1");
      actAdcAddr[i][2] = GetActNiosAddr(i, "adc2");
      actAdcAddr[i][3] = GetActNiosAddr(i, "adc3");
#endif
    }

    secEtiltAddr = GetNiosAddr("sec_etilt");
    secErotAddr = GetNiosAddr("sec_erot");
    secEoffAddr = GetNiosAddr("sec_eoff");
    secLtiltAddr = GetNiosAddr("sec_ltilt");
    secLrotAddr = GetNiosAddr("sec_lrot");
    secLoffAddr = GetNiosAddr("sec_loff");

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

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);

  for (j = 0; j < 3; ++j) {
#if 0
    for (i = 0; i < 4; ++i)
      WriteData(actAdcAddr[j][i], act_data[j].adc[i], NIOS_QUEUE);
#endif
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

static int MegaKill(void)
{
  if (CommandData.actbus.megakill) {
    BusSend('_', "T");
    return 1;
    CommandData.actbus.megakill = 0;
  }
  return 0;
}

static void DiscardBusRecv(int flag)
{
  int i;

  /* Discard response to get it off the bus */
  if ((i = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD))
    bputs(warning,
        "ActBus: Timeout waiting for response after uplinked command.");
#ifndef ACTBUS_CHATTER
  else if (flag)
    bprintf(info, "ActBus: Controller response: %s\n", gp_buffer);
#endif
}

void SolveSecondary(void)
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

  double lvdt10 = slow_data[lvdt10Addr->index][lvdt10Addr->channel];
  double lvdt11 = slow_data[lvdt11Addr->index][lvdt11Addr->channel];
  double lvdt13 = slow_data[lvdt13Addr->index][lvdt13Addr->channel];

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
  alpha = lvdt10 * LVDT10_TO_MM - LVDT10_ZERO;
  beta = lvdt13 * LVDT13_TO_MM - LVDT13_ZERO;
  gamma = lvdt11 * LVDT11_TO_MM - LVDT11_ZERO;
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
      BusRecv(NULL, 1); /* this is a blocking call -- clear the recv buffer */
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
          CommandData.actbus.command[my_cindex]);
      /* Discard response to get it off the bus */
      DiscardBusRecv(1);
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
      ReadActuator(i);

    SolveSecondary();

    if (MegaKill())
      return;
    usleep(10000);
  }
}
