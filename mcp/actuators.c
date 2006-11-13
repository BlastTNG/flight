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

/* Define this symbol to have mcp log all actuator bus traffic */
#define ACTBUS_CHATTER
static int __inhibit_chatter = 0;

#if defined USE_FIFO_CMD && ! defined USE_XY_THREAD
#  define ACT_BUS "/dev/ttyS0"
#else
#  define ACT_BUS "/dev/ttyS7"
#endif

#define LVDT_RADIUS ACTUATOR_RADIUS
#define ACTUATOR_RADIUS 144.338 /* in mm */

#define ENCODER_TOL 0

#define ALL_ACT 0x51 /* All actuators */
#define LOCKNUM 3
#ifdef USE_XY_STAGE
#  define STAGEXNUM 4
#  define STAGEYNUM 5
#  define NACT 6
#else
#  define NACT 4
#endif
#define POLL_TIMEOUT 30000 /* 5 minutes */

/* EZ Stepper status bit masks */
#define EZ_ERROR  0x0F
#define EZ_READY  0x20
#define EZ_STATUS 0x40

/* EZ Stepper error numbers */
#define EZ_ERR_OK      0 /* No error */
#define EZ_ERR_INIT    1 /* Initialisation error */
#define EZ_ERR_BADCMD  2 /* Bad command */
#define EZ_ERR_BADOP   3 /* Bad operand */
#define EZ_ERR_COMM    5 /* Communications error */
#define EZ_ERR_NOINIT  7 /* Not initialised */
#define EZ_ERR_OVER    9 /* overload error */
#define EZ_ERR_NOMOVE 11 /* Move Not allowed */
#define EZ_ERR_BUSY   15 /* Command overflow */

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

static char bus_buffer[1000];
static struct stepper_struct {
  int status;
  int sequence;
} stepper[NACT];

static struct lock_struct {
  unsigned int pos;
  unsigned short adc[4];
  unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

static struct act_struct {
  int pos;
  int enc;
} act_data[3];

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

/* Secondary focus crap */
static double focus = -ACTENC_OFFSET; /* set in ab thread, read in fc thread */
static double correction = 0;         /* set in fc thread, read in ab thread */

static int act_setserial(char *input_tty)
{
  int fd;
  struct termios term;

  if ((fd = open(input_tty, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    berror(tfatal, "ActBus: Unable to open serial port (%s)", input_tty);

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

#ifdef USE_XY_STAGE
static inline void UnderrideBus(int who)
{
  if (bus_underride != who)
    bprintf(info, "ActBus: Bus underride for %s enabled.\n", name[who]);
  bus_underride = who;
  ReleaseBus(-2);
}
#endif

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
  static int sequence = 1;

  sequence = (sequence + 1) % 7;
  buffer[0] = 0x2;
  buffer[1] = who;
  buffer[2] = 0x30 + sequence;
  sprintf(buffer + 3, "%s", what);
  buffer[len - 2] = 0x3;
  for (ptr = buffer; *ptr != '\03'; ++ptr)
    chk ^= *ptr;
  buffer[len - 1] = chk;
#ifdef ACTBUS_CHATTER
  if (!inhibit_chatter)
    bprintf(info, "ActBus: Request=%s (%s)", HexDump(buffer, len), what);
#endif
  if (write(bus_fd, buffer, len) < 0)
    berror(err, "ActBus: Error writing on bus");

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
        case 3: /* status byte */
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

static int BusComm(int who, const char* what, int naive, int inhibit_chatter)
{
  int result = 0;
  int ok;

  if (who < 0x30)
    who = id[who];

  do {
    ok = 1;
    BusSend(who, what, inhibit_chatter);

    /* Muliple motors */
    if (who >= 0x40)
      break;

    if ((result = BusRecv(bus_buffer, 0, inhibit_chatter)) & (ACTBUS_TIMEOUT
          | ACTBUS_OOD))
    {
      bprintf(warning,
          "ActBus: Timeout waiting for response from %#x.", who);
      CommandData.actbus.force_repoll = 1;
#ifndef ACTBUS_CHATTER
    } else if (naive) {
      bprintf(info, "ActBus: Controller response: %s\n", buffer);
#endif
    } else {
      switch (result & EZ_ERROR) {
        case EZ_ERR_INIT:
          bprintf(warning, "ActBus: Controller %#x: initialisation error.\n",
              who);
          break;
        case EZ_ERR_BADCMD:
          bprintf(warning, "ActBus: Controller %#x: bad command.\n", who);
          break;
        case EZ_ERR_BADOP:
          bprintf(warning, "ActBus: Controller %#x: bad operand.\n", who);
          break;
        case EZ_ERR_COMM:
          bprintf(warning, "ActBus: Controller %#x: communications error.\n",
              who);
          break;
        case EZ_ERR_NOINIT:
          bprintf(warning, "ActBus: Controller %#x: not initialied.\n", who);
          break;
        case EZ_ERR_OVER:
          bprintf(warning, "ActBus: Controller %#x: overload.\n", who);
          break;
        case EZ_ERR_NOMOVE:
          bprintf(warning, "ActBus: Controller %#x: move not allowed.\n", who);
          break;
        case EZ_ERR_BUSY:
          bprintf(warning, "ActBus: Controller %#x: command overflow.\n", who);
          usleep(10000);
          ok = 0;
          break;
      }
    }
  } while (!ok && !naive);

  return result;
}

static int ReadIntFromBus(int who, const char* cmd, int inhibit_chatter)
{
  BusComm(who, cmd, 0, inhibit_chatter);

  return atoi(bus_buffer);
}

static void ReadActuator(int who, int inhibit_chatter)
{
  if (stepper[who].status == -1)
    return;

  act_data[who].pos = ReadIntFromBus(who, "?0", inhibit_chatter);
  act_data[who].enc = ReadIntFromBus(who, "?8", inhibit_chatter);
}

static char* __attribute__((format(printf,2,3))) ActCommand(char* buffer,
    const char* fmt, ...)
{
  va_list argptr;
  char* ptr;

  sprintf(buffer, "aE25600V%iL%im%ih%i", CommandData.actbus.act_vel,
      CommandData.actbus.act_acc, CommandData.actbus.act_move_i,
      CommandData.actbus.act_hold_i);

  for(ptr = buffer; *ptr != '\0'; ++ptr);

  va_start(argptr, fmt);
  vsprintf(ptr, fmt, argptr);
  va_end(argptr);

  return buffer;
}

static void InitialiseActuator(int who)
{
  int enc;
  char buffer[1000];

  ReadActuator(who, 1); 

  if (act_data[who].enc < ACTENC_OFFSET / 2) {
    bprintf(info, "Initialising Actuator #%i...", who);

    /* Bug workaround -- can't set zero after controller boot until
     * after having moved. */
    BusComm(who, ActCommand(buffer, "%s", "P10R"), 0, __inhibit_chatter);

    ReadActuator(who, 1); 
    sleep(1);

    /* Add offset */
    enc = ACTENC_OFFSET + act_data[who].enc;

    /* Set the encoder */
    ActCommand(buffer, "z%iR", enc);  
    BusComm(who, buffer, 0, __inhibit_chatter);
  }
}

#define THERE_WAIT 10
static void ServoActuators(int* goal)
{
  int i;
  int act_there[3] = {0, 0, 0};
  int act_wait[3] = {0, 0, 0};
  char buffer[1000];

  if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
    return;

  TakeBus(0);

  while (
      act_there[0] < THERE_WAIT ||
      act_there[1] < THERE_WAIT ||
      act_there[2] < THERE_WAIT) {
    for (i = 0; i < 3; ++i)
      if (act_there[i] >= THERE_WAIT || stepper[i].status == -1) {
        act_there[i] = THERE_WAIT;
        continue;
      } else if (act_data[i].enc < ACTENC_OFFSET / 2)
        InitialiseActuator(i);
      else if (act_wait[i] > 0)
        act_wait[i]--;
      else {
        int delta = goal[i] - act_data[i].enc + ACTENC_OFFSET;
        if (abs(delta) <= ENCODER_TOL) {
          act_there[i]++;
        } else if (delta > 0) {
          ActCommand(buffer, "P%iR", delta);
          bprintf(info, "Servo: %i => %s\n", i, buffer);
          act_wait[i] = delta * (6. * 26. / CommandData.actbus.act_vel) + 1;
          BusComm(i, buffer, 0, __inhibit_chatter);
        } else {
          delta = 10 - delta;
          ActCommand(buffer, "D%iR", delta);
          bprintf(info, "Servo: %i => %s\n", i, buffer);
          act_wait[i] = delta * (6. * 26. / CommandData.actbus.act_vel) + 1;
          BusComm(i, buffer, 0, __inhibit_chatter);
        }
      }
    for (i = 0; i < 3; ++i)
      ReadActuator(i, 1);
    bprintf(info, "%i %i %i / %i %i %i", act_wait[0], act_wait[1], act_wait[2],
        act_there[0], act_there[1], act_there[2]);
    if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
      break;
  }

  ReleaseBus(0);
}

static void DeltaActuators(void)
{
  int i, goal[3];
  
  for (i = 0; i < 3; ++i)
    goal[i] = CommandData.actbus.delta[i] + act_data[i].enc - ACTENC_OFFSET;

  ServoActuators(goal);
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
    BusSend(id[i], "&", __inhibit_chatter);
    if ((result = BusRecv(bus_buffer, 0, __inhibit_chatter)) & (ACTBUS_TIMEOUT
          | ACTBUS_OOD)) {
      bprintf(warning, "ActBus: No response from %s, will repoll later.",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    } else if (!strncmp(bus_buffer, "EZHR17EN AllMotion", 18)) {
      bprintf(info, "ActBus: Found type 17EN device %s at address %i.\n",
          name[i], id[i] - 0x30);
      stepper[i].status = 0;
    } else if (!strncmp(bus_buffer, "EZHR23 All Motion", 17)) {
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

    sleep(1);
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

  if (stepper[LOCKNUM].status == -1)
    return;

  if (counter++ < mult * LOCK_MOTOR_DATA_TIMER)
    return;

  counter = 0;

  lock_data.pos = ReadIntFromBus(LOCKNUM, "?0", 1);

  BusComm(LOCKNUM, "?aa", 0, 1);

  sscanf(bus_buffer, "%hi,%hi,%hi,%hi", &lock_data.adc[0], &lock_data.adc[1],
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

static void SetOffsets(int* offset)
{
  int i;
  char buffer[1000];

  for (i = 0; i < 3; ++i) {
    sprintf(buffer, "z%iR", offset[i] + 1000000);
    BusComm(i, buffer, 0, __inhibit_chatter);
    sleep(1);
  }
}

static void SetNewFocus(void)
{
  char buffer[1000];
  int i, j;
  int delta = CommandData.actbus.focus - focus;

  if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
    return;

  TakeBus(0);

  bprintf(info, "Old: %i %i %i -> %i %i\n", act_data[0].enc, act_data[1].enc,
      act_data[2].enc, CommandData.actbus.focus, delta);
  CommandData.actbus.goal[0] = act_data[0].enc + delta - ACTENC_OFFSET;
  CommandData.actbus.goal[1] = act_data[1].enc + delta - ACTENC_OFFSET;
  CommandData.actbus.goal[2] = act_data[2].enc + delta - ACTENC_OFFSET;

  if (abs(delta) <= ENCODER_TOL) {
    ;
  } else if (delta > 0) {
    ActCommand(buffer, "V2000P%iR", delta);
    bprintf(info, "Servo: %i => %s\n", ALL_ACT, buffer);
    BusComm(ALL_ACT, buffer, 0, __inhibit_chatter);
  } else {
    delta = 10 - delta;
    ActCommand(buffer, "V2000D%iR", delta);
    bprintf(info, "Servo: %i => %s\n", ALL_ACT, buffer);
    BusComm(ALL_ACT, buffer, 0, __inhibit_chatter);
  }
  bprintf(info, "New: %i %i %i\n", CommandData.actbus.goal[0],
      CommandData.actbus.goal[1], CommandData.actbus.goal[2]);
  for (i = 0; i < (1 + abs(delta) * 26 / CommandData.actbus.act_vel) * 6; ++i) {
    for (j = 0; j < 3; ++j)
      ReadActuator(j, 1);
    if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
      return;
  }
}

static double CalibrateAD590(int counts)
{
  double t = I2T_M * counts + I2T_B + 273.15;

  /* if t < -73C or t > 67C, assume AD590 is broken */
  if (t < 200)
    t = -1;
  else if (t > 340)
    t = -2;
  
  return t;
}

static int ThermalCompensation(void)
{
  focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3
    - ACTENC_OFFSET;
  
  /* Do nothing if vetoed or autovetoed */
  if (CommandData.actbus.tc_mode != TC_MODE_ENABLED)
    return ACTBUS_FM_SLEEP;

  /* Do nothing if we haven't timed out */
  if (CommandData.actbus.sf_time < CommandData.actbus.tc_wait)
    return ACTBUS_FM_SLEEP;

  /* Do nothing if the offset is below the threshold */
  if (correction < CommandData.actbus.tc_step
      && -correction < CommandData.actbus.tc_step)
    return ACTBUS_FM_SLEEP;

  /* Do something! */
  CommandData.actbus.focus = focus - correction; 
  CommandData.actbus.sf_time = 0;

  return ACTBUS_FM_THERMO;
}

/* This function is called by the frame control thread */
void SecondaryMirror(void)
{
  static int firsttime = 1;

  static struct NiosStruct* sfCorrectionAddr;
  static struct NiosStruct* sfAgeAddr;
  static struct NiosStruct* sfPositionAddr;
  static struct NiosStruct* sfTPrimAddr;
  static struct NiosStruct* sfTSecAddr;
  static struct NiosStruct* tpAddr;
  static struct NiosStruct* tsAddr;

  static struct BiPhaseStruct* tPrimary1Addr;
  static struct BiPhaseStruct* tSecondary1Addr;
  static struct BiPhaseStruct* tPrimary2Addr;
  static struct BiPhaseStruct* tSecondary2Addr;
  double t_primary, t_secondary;
  double t_primary1, t_secondary1;
  double t_primary2, t_secondary2;
  static int ctr = 0;

  if (firsttime) {
    firsttime = 0;
    tPrimary1Addr = GetBiPhaseAddr("t_primary_1");
    tSecondary1Addr = GetBiPhaseAddr("t_secondary_1");
    tPrimary2Addr = GetBiPhaseAddr("t_primary_2");
    tSecondary2Addr = GetBiPhaseAddr("t_secondary_2");
    sfCorrectionAddr = GetNiosAddr("sf_correction");
    sfAgeAddr = GetNiosAddr("sf_age");
    sfPositionAddr = GetNiosAddr("sf_position");
    sfTPrimAddr = GetNiosAddr("sf_t_prim");
    sfTSecAddr = GetNiosAddr("sf_t_sec");
    tpAddr = GetNiosAddr("tp");
    tsAddr = GetNiosAddr("ts");
  }

  /* Do nothing if we haven't heard from the actuators */
  if (focus < -ACTENC_OFFSET / 2)
    return;

  t_primary1 = CalibrateAD590(
      slow_data[tPrimary1Addr->index][tPrimary1Addr->channel]
      );
  t_primary2 = CalibrateAD590(
      slow_data[tPrimary2Addr->index][tPrimary2Addr->channel]
      );
  t_secondary1 = CalibrateAD590(
      slow_data[tSecondary1Addr->index][tSecondary1Addr->channel]
      );

  t_secondary2 = CalibrateAD590(
      slow_data[tSecondary2Addr->index][tSecondary2Addr->channel]
      );

  t_primary = (t_primary1 + t_primary2) / 2;
  t_secondary = (t_secondary1 + t_secondary2) / 2;

  ctr++;
  if (ctr == 280000)
    ctr = 0;

  if (CommandData.actbus.tc_mode != TC_MODE_VETOED && (t_primary < 0
        || t_secondary < 0)) {
    if (CommandData.actbus.tc_mode == TC_MODE_ENABLED)
      bputs(info, "Thermal Compensation: Autoveto raised.");
    CommandData.actbus.tc_mode = TC_MODE_AUTOVETO;
  } else if (CommandData.actbus.tc_mode == TC_MODE_AUTOVETO) {
    bputs(info, "Thermal Compensation: Autoveto lowered.");
    CommandData.actbus.tc_mode = TC_MODE_ENABLED;
  }

  if (CommandData.actbus.sf_in_focus) {
    CommandData.actbus.sf_position = focus;
    CommandData.actbus.sf_t_primary = t_primary;
    CommandData.actbus.sf_t_secondary = t_secondary;
    CommandData.actbus.sf_time = 0;
    CommandData.actbus.sf_in_focus = 0;
    correction = 0;
  } else {
    correction = CommandData.actbus.g_primary * (t_primary -
        CommandData.actbus.sf_t_primary) - CommandData.actbus.g_secondary *
      (t_secondary - CommandData.actbus.sf_t_secondary) + (focus -
          CommandData.actbus.sf_position);
    if (CommandData.actbus.sf_time < 1000000)
      CommandData.actbus.sf_time++;
  }

  WriteData(tpAddr, (t_primary - 273.15) * 500, NIOS_QUEUE);
  WriteData(tsAddr, (t_secondary - 273.15) * 500, NIOS_QUEUE);
  WriteData(sfCorrectionAddr, correction, NIOS_QUEUE);
  WriteData(sfAgeAddr, CommandData.actbus.sf_time / 20., NIOS_QUEUE);
  WriteData(sfPositionAddr, CommandData.actbus.sf_position, NIOS_QUEUE);
  WriteData(sfTPrimAddr, (CommandData.actbus.sf_t_primary - 273.15) * 100.,
      NIOS_QUEUE);
  WriteData(sfTSecAddr, (CommandData.actbus.sf_t_secondary - 273.15) * 100.,
      NIOS_QUEUE);
}


static void DoActuators(void)
{
  switch(CommandData.actbus.focus_mode) {
    case ACTBUS_FM_PANIC:
      bputs(warning, "ActBus: Actuator Panic");
      BusComm(ALL_ACT, "T", 0, __inhibit_chatter);
      CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
      break;
    case ACTBUS_FM_DELTA:
      DeltaActuators();
      break;
    case ACTBUS_FM_THERMO:
    case ACTBUS_FM_FOCUS:
      SetNewFocus();
      /* fallthrough */
    case ACTBUS_FM_SERVO:
      ServoActuators(CommandData.actbus.goal);
      break;
    case ACTBUS_FM_OFFSET:
      SetOffsets(CommandData.actbus.offset);
      /* fallthrough */
    case ACTBUS_FM_SLEEP:
      break;
    default:
      bputs(err, "ActBus: Unknown Focus Mode (%i), sleeping");
  }
  if (CommandData.actbus.focus_mode != ACTBUS_FM_PANIC)
    CommandData.actbus.focus_mode = ThermalCompensation();
}

static inline char* LockCommand(char* buffer, const char* cmd)
{
  sprintf(buffer, "j256V%iL%im%ih%i%s", CommandData.actbus.lock_vel,
      CommandData.actbus.lock_acc, CommandData.actbus.lock_move_i,
      CommandData.actbus.lock_hold_i, cmd);

  return buffer;
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
#define LA_STEP    5
static void DoLock(void)
{
  int action = LA_EXIT;
  char command[2000] = "";

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
//        LockCommand(command, "T"); /* terminate all strings */
        strcpy(command, "T"); /* terminate all strings */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_OFF;
        break;
      case LA_EXTEND:
        bputs(info, "ActBus: Extending lock motor.");
        LockCommand(command, "P0R"); /* move out forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_EXT;
        break;
      case LA_RETRACT:
        bputs(info, "ActBus: Retracting lock motor.");
        LockCommand(command, "D0R"); /* move in forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_RET;
        break;
      case LA_STEP:
        bputs(info, "ActBus: Stepping lock motor.");
        LockCommand(command, "P100000R"); /* move away from the limit switch */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_STP;
        break;
      default:
        command[0] = 0;
    }

    /* ... and do it! */
    if (command[0] != 0) {
      BusComm(LOCKNUM, command, 0, __inhibit_chatter);
      usleep(SEND_SLEEP); /* wait for a bit */
    } else if (action == LA_WAIT)
      usleep(WAIT_SLEEP); /* wait for a bit */
  } while (action != LA_EXIT);
}

static inline struct NiosStruct* GetActNiosAddr(int i, const char* field)
{
  sprintf(bus_buffer, "act%i_%s", i, field);

  return GetNiosAddr(bus_buffer);
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

  static struct NiosStruct* lockVelAddr;
  static struct NiosStruct* lockAccAddr;
  static struct NiosStruct* lockMoveIAddr;
  static struct NiosStruct* lockHoldIAddr;

  static struct NiosStruct* actVelAddr;
  static struct NiosStruct* actAccAddr;
  static struct NiosStruct* actMoveIAddr;
  static struct NiosStruct* actHoldIAddr;

  static struct NiosStruct* actPosAddr[3];
  static struct NiosStruct* actEncAddr[3];

  static struct NiosStruct* tcGPrimAddr;
  static struct NiosStruct* tcGSecAddr;
  static struct NiosStruct* tcStepAddr;
  static struct NiosStruct* tcWaitAddr;
  static struct NiosStruct* tcModeAddr;
  static struct NiosStruct* secGoalAddr;
  static struct NiosStruct* secFocusAddr;

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

    tcGPrimAddr = GetNiosAddr("tc_g_prim");
    tcGSecAddr = GetNiosAddr("tc_g_sec");
    tcStepAddr = GetNiosAddr("tc_step");
    tcWaitAddr = GetNiosAddr("tc_wait");
    tcModeAddr = GetNiosAddr("tc_mode");
    secGoalAddr = GetNiosAddr("sec_goal");
    secFocusAddr = GetNiosAddr("sec_focus");

    actVelAddr = GetNiosAddr("act_vel");
    actAccAddr = GetNiosAddr("act_acc");
    actMoveIAddr = GetNiosAddr("act_move_i");
    actHoldIAddr = GetNiosAddr("act_hold_i");

    lockVelAddr = GetNiosAddr("lock_vel");
    lockAccAddr = GetNiosAddr("lock_acc");
    lockMoveIAddr = GetNiosAddr("lock_move_i");
    lockHoldIAddr = GetNiosAddr("lock_hold_i");

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
    WriteData(actEncAddr[j], act_data[j].enc - ACTENC_OFFSET, NIOS_QUEUE);
  }
  WriteData(secFocusAddr, focus, NIOS_FLUSH);

  for (i = 0; i < 4; ++i)
    WriteData(lockAdcAddr[i], lock_data.adc[i], NIOS_QUEUE);
  WriteData(lockStateAddr, lock_data.state, NIOS_QUEUE);
  WriteData(seizedBusAddr, bus_seized, NIOS_QUEUE);
  WriteData(lockGoalAddr, CommandData.actbus.lock_goal, NIOS_QUEUE);
  WriteData(lockPosAddr, lock_data.pos, NIOS_FLUSH);

  WriteData(actVelAddr, CommandData.actbus.act_vel, NIOS_QUEUE);
  WriteData(actAccAddr, CommandData.actbus.act_acc, NIOS_QUEUE);
  WriteData(actMoveIAddr, CommandData.actbus.act_move_i, NIOS_QUEUE);
  WriteData(actHoldIAddr, CommandData.actbus.act_hold_i, NIOS_QUEUE);

  WriteData(lockVelAddr, CommandData.actbus.lock_vel, NIOS_QUEUE);
  WriteData(lockAccAddr, CommandData.actbus.lock_acc, NIOS_QUEUE);
  WriteData(lockMoveIAddr, CommandData.actbus.lock_move_i, NIOS_QUEUE);
  WriteData(lockHoldIAddr, CommandData.actbus.lock_hold_i, NIOS_QUEUE);

  WriteData(tcGPrimAddr, CommandData.actbus.g_primary * 100., NIOS_QUEUE);
  WriteData(tcGSecAddr, CommandData.actbus.g_secondary * 100., NIOS_QUEUE);
  WriteData(tcModeAddr, CommandData.actbus.tc_mode, NIOS_QUEUE);
  WriteData(tcStepAddr, CommandData.actbus.tc_step, NIOS_QUEUE);
  WriteData(tcWaitAddr, CommandData.actbus.tc_wait / 20., NIOS_QUEUE);
  WriteData(secGoalAddr, CommandData.actbus.focus, NIOS_QUEUE);

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
      CommandData.actbus.caddr[my_cindex] = 0;
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
    if (CommandData.actbus.caddr[my_cindex] != 0
#ifdef USE_XY_THREAD
        && CommandData.actbus.caddr[my_cindex] != 0x36
        && CommandData.actbus.caddr[my_cindex] != 0x37
#endif
       ) {
      BusComm(CommandData.actbus.caddr[my_cindex],
          CommandData.actbus.command[my_cindex], 1, __inhibit_chatter);
      CommandData.actbus.caddr[my_cindex] = 0;
    }

#ifdef USE_XY_STAGE
    UnderrideBus(STAGEXNUM);
#endif

    DoLock(); /* Lock motor stuff -- this will seize the bus until
                 the lock motor's state has settled */

    DoActuators(); /* Actuator stuff -- this may seize the bus */

#ifdef USE_XY_STAGE
    if (bus_seized == STAGEXNUM)
      ReadStage();
#endif

    for (i = 0; i < 3; ++i)
      ReadActuator(i, 1);

    focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3
      - ACTENC_OFFSET;

    usleep(10000);
  }
}
