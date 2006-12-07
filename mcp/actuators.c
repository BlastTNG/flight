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

/* Thermal model numbers, from MD and MV */
#define T_PRIMARY_FOCUS   258.15 /* = -15C */
#define T_SECONDARY_FOCUS 243.15 /* = -30C */
#define POSITION_FOCUS     11953 /* absolute counts */

#define LVDT_RADIUS ACTUATOR_RADIUS
#define ACTUATOR_RADIUS 143.71 /* mm */

#define ENCODER_TOL 0
#define FOCUS_TOL 30
#define MAX_STEP 100
#define STEP_MIN 70

#define ALL_ACT 0x51 /* 'Q' = All actuators */
#define LOCKNUM 3
#define NACT 4
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
#define DRIVE_TIMEOUT 300 /* 1 minute @ 5Hz */

#define LOCK_MIN_POT 3000
#define LOCK_MAX_POT 16365
#define LOCK_POT_RANGE 1000

/* in commands.c */
double LockPosition(double elevation);

extern short int InCharge; /* tx.c */

#define LAST_ACTUATOR 2
static int bus_fd = -1;
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
  "Lock Motor" };
static const int id[NACT] = {0x31, 0x32, 0x33, 0x35};

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

static double lvdt[3];

static int bus_seized = -1;
static int actbus_flags = 0;
static int bad_move = 0;

#define ACTBUS_FL_LOST     0x001
#define ACTBUS_FL_DR_PS    0x002
#define ACTBUS_FL_DR_LG    0x004
#define ACTBUS_FL_LG_PS    0x008
#define ACTBUS_FL_DR_LV    0x010
#define ACTBUS_FL_LG_LV    0x020
#define ACTBUS_FL_PS_LV    0x040
#define ACTBUS_FL_BAD_MOVE 0x080
#define ACTBUS_FL_FAIL0    0x100
#define ACTBUS_FL_FAIL1    0x200
#define ACTBUS_FL_FAIL2    0x400

/* Secondary focus crap */
static double t_primary = -1, t_secondary = -1;
static double focus = -ACTENC_OFFSET; /* set in ab thread, read in fc thread */
static double correction = 0;         /* set in fc thread, read in ab thread */
static int fail[3] = {0, 0, 0};
static int lost[3] = {0, 0, 0};

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
  if (bus_seized != -1)
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

static int ReadIntFromBus(int who, const char* cmd, int old,
    int inhibit_chatter)
{
  if (BusComm(who, cmd, 0, inhibit_chatter) & (ACTBUS_TIMEOUT | ACTBUS_OOD))
    return old;

  return atoi(bus_buffer);
}

static void ReadActuator(int who, int inhibit_chatter)
{
  if (stepper[who].status == -1)
    return;

  act_data[who].pos = ReadIntFromBus(who, "?0", act_data[who].pos,
      inhibit_chatter);
  act_data[who].enc = ReadIntFromBus(who, "?8", act_data[who].enc,
      inhibit_chatter);

  if (act_data[who].enc > ACTENC_OFFSET / 2)
    CommandData.actbus.last_good[who] = act_data[who].enc;
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

    /* Go Back */
    BusComm(who, ActCommand(buffer, "%s", "D10R"), 0, __inhibit_chatter);

    /* Add offset */
    enc = CommandData.actbus.last_good[who];

    /* Set the encoder */
    ActCommand(buffer, "z%iR", enc);  
    BusComm(who, buffer, 0, __inhibit_chatter);
  }
}

/* Barth made this up */
static int CheckMove(int delta0, int delta1, int delta2)
{
  double maxE, minE;

  double lvdt_low = CommandData.actbus.lvdt_low;
  double lvdt_high = CommandData.actbus.lvdt_high;
  double lvdt_delta = CommandData.actbus.lvdt_delta;

  double X = (lvdt[0]+lvdt[1]+lvdt[2] + (double)(delta0 + delta1
        + delta2)) / 3.0;

  double A = (-lvdt[1] + 2 * lvdt[0] + 2 * lvdt[2]) / 3. + (double)delta0;
  double B = (-lvdt[2] + 2 * lvdt[1] + 2 * lvdt[0]) / 3. + (double)delta1;
  double C = (-lvdt[0] + 2 * lvdt[2] + 2 * lvdt[1]) / 3. + (double)delta2;

  if (A < B) {
   maxE = B;
   minE = A;
  } else {
   maxE = A;
   minE = B;
  }

  if (C > maxE)
    maxE = C;
  else if (C < minE)
    minE = C;

  bprintf(info, "%i %i %i | %f %f %f %f | %f %f | %f %f | %f %f", delta0,
      delta1, delta2, X, A, B, C, minE, maxE,
      lvdt_low, lvdt_high, maxE - minE, lvdt_delta);

  if (X < lvdt_low || X > lvdt_high || maxE - minE > lvdt_delta) {
    bputs(warning, "ActBus: Move Out of Range.");
    bad_move = ACTBUS_FL_BAD_MOVE;
  } else
    bad_move = 0;

  return bad_move;
}

/* Update flags */
static void UpdateFlags(void)
{
  int i;
  int new_flags = bad_move;

  if (fail[0]) 
    new_flags |= ACTBUS_FL_FAIL0;
  if (fail[1]) 
    new_flags |= ACTBUS_FL_FAIL1;
  if (fail[2]) 
    new_flags |= ACTBUS_FL_FAIL2;

  for (i = 0; i < 3; ++i) {
    if (lost[i])
      new_flags |= ACTBUS_FL_LOST;
    if (CommandData.actbus.dead_reckon[i] != CommandData.actbus.last_good[i])
      new_flags |= ACTBUS_FL_DR_LG;
    if (CommandData.actbus.dead_reckon[i] != act_data[i].pos +
        CommandData.actbus.pos_trim[i])
      new_flags |= ACTBUS_FL_DR_PS;
    if (CommandData.actbus.last_good[i] != act_data[i].pos +
        CommandData.actbus.pos_trim[i])
      new_flags |= ACTBUS_FL_LG_PS;
  }
  actbus_flags = new_flags;
}

#define THERE_WAIT 10
static void ServoActuators(int* goal, int update_dr)
{
  int i;
  int act_there[3] = {0, 0, 0};
  int act_wait[3] = {0, 0, 0};
  int delta_dr[3] = {0, 0, 0};
  int delta[3] = {0, 0, 0};
  int start[3] = {0, 0, 0};
  char buffer[1000];

  for (i = 0; i < 3; ++i) {
    fail[i] = lost[i] = 0;
    delta_dr[i] = goal[i] - act_data[i].enc + ACTENC_OFFSET;
  }

  if (CheckMove(delta_dr[0], delta_dr[1], delta_dr[2]))
    return;

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
      } else if (act_data[i].enc < ACTENC_OFFSET / 2) {
        lost[i] = 1;
        CommandData.actbus.force_repoll = 1;
      } else if (act_wait[i] > 0)
        act_wait[i]--;
      else if (act_wait[i] == 0) {
        act_wait[i]--;
        /* Here is the check */
        if (abs(delta[i]) == MAX_STEP && abs(act_data[i].enc - start[i]) <
            STEP_MIN)
        {
          bprintf(err, "ActBus: Encoder failure detected, actuator %i\n", i);
          if (CommandData.actbus.tc_mode == TC_MODE_ENABLED) {
            bputs(err, "ActBus: Fully vetoing thermal correction");
            CommandData.actbus.tc_mode = TC_MODE_VETOED;
          }
          act_there[0] = act_there[1] = act_there[2] = THERE_WAIT;
          fail[i] = 1;
        }
      } else {
        start[i] = act_data[i].enc - ACTENC_OFFSET;
        delta[i] = goal[i] - start[i];

        if (delta[i] > MAX_STEP)
          delta[i] = MAX_STEP;
        else if (delta[i] < -MAX_STEP)
          delta[i] = -MAX_STEP;

        if (abs(delta[i]) <= ENCODER_TOL || lost[i]) {
          act_there[i]++;
        } else if (delta[i] > 0) {
          ActCommand(buffer, "P%iR", delta[i]);
          bprintf(info, "Servo: %i => %s\n", i, buffer);
          act_wait[i] = delta[i] * (6. * 26. / CommandData.actbus.act_vel) + 1;
          BusComm(i, buffer, 0, __inhibit_chatter);
        } else {
          /* Always overshoot on the downswing, so we can come up to the goal
           * Random to damp oscillations */
          delta[i] = 8 + (rand() % 10) - delta[i];
          ActCommand(buffer, "D%iR", delta[i]);
          bprintf(info, "Servo: %i => %s\n", i, buffer);
          act_wait[i] = delta[i] * (6. * 26. / CommandData.actbus.act_vel) + 1;
          BusComm(i, buffer, 0, __inhibit_chatter);
        }
      }
    for (i = 0; i < 3; ++i)
      ReadActuator(i, 1);

    focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3
      - ACTENC_OFFSET;

    bprintf(info, "%i %i %i / %i %i %i", act_wait[0], act_wait[1], act_wait[2],
        act_there[0], act_there[1], act_there[2]);
    if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
      break;
  }

  if (update_dr)
    for (i = 0; i < 3; ++i)
      CommandData.actbus.dead_reckon[i] += delta_dr[i];

  ReleaseBus(0);
}

static void DeltaActuators(void)
{
  int i, goal[3];

  for (i = 0; i < 3; ++i)
    goal[i] = CommandData.actbus.delta[i] + act_data[i].enc - ACTENC_OFFSET;

  ServoActuators(goal, 1);
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

static void GetLockData(int mult)
{
  static int counter = 0;

  if (stepper[LOCKNUM].status == -1)
    return;

  if (counter++ < mult * LOCK_MOTOR_DATA_TIMER)
    return;

  counter = 0;

  lock_data.pos = ReadIntFromBus(LOCKNUM, "?0", lock_data.pos, 1);

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

  static struct BiPhaseStruct* lockPotAddr;
  static struct BiPhaseStruct* lockLimSwAddr;
  static struct BiPhaseStruct* lockStateAddr;

  if (firsttime) {
    firsttime = 0;
    lockPotAddr = GetBiPhaseAddr("lock_pot");
    lockLimSwAddr = GetBiPhaseAddr("lock_lim_sw");
    lockStateAddr = GetBiPhaseAddr("lock_state");
  }

  if (nic) {
    pot = slow_data[lockPotAddr->index][lockPotAddr->channel];
    ls = slow_data[lockLimSwAddr->index][lockLimSwAddr->channel];
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

static int SetNewFocus(void)
{
  char buffer[1000];
  int i, j;
  int deferred = CommandData.actbus.focus - focus;
  int delta_dr = deferred;
  int delta = 0;
  int start[3];

  for (i = 0; i < 3; ++i) {
    fail[i] = 0;
    start[i] = act_data[i].enc;
  }

  if (CheckMove(deferred, deferred, deferred))
    return 0;
  
  if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
    return 0;

  TakeBus(0);

  bprintf(info, "Old: %i %i %i -> %i %i\n", act_data[0].enc, act_data[1].enc,
      act_data[2].enc, CommandData.actbus.focus, deferred);
  CommandData.actbus.goal[0] = act_data[0].enc + deferred - ACTENC_OFFSET;
  CommandData.actbus.goal[1] = act_data[1].enc + deferred - ACTENC_OFFSET;
  CommandData.actbus.goal[2] = act_data[2].enc + deferred - ACTENC_OFFSET;

  while (deferred != 0) {
    if (deferred > MAX_STEP)
      delta = MAX_STEP;
    else if (deferred < -MAX_STEP)
      delta = -MAX_STEP;
    else
      delta = deferred;
    bprintf(info, "D/d - %i %i", deferred, delta);

    deferred -= delta;

    if (abs(delta) <= FOCUS_TOL) {
      ;
    } else if (delta > 0) {
      ActCommand(buffer, "P%iR", delta);
      bprintf(info, "Servo: %i => %s\n", ALL_ACT, buffer);
      BusComm(ALL_ACT, buffer, 0, __inhibit_chatter);
    } else {
      delta = 10 - delta;
      ActCommand(buffer, "D%iR", delta);
      bprintf(info, "Servo: %i => %s\n", ALL_ACT, buffer);
      BusComm(ALL_ACT, buffer, 0, __inhibit_chatter);
    }
    bprintf(info, "New: %i %i %i\n", CommandData.actbus.goal[0],
        CommandData.actbus.goal[1], CommandData.actbus.goal[2]);
    for (i = 0; i < (1 + abs(delta) * 26 / CommandData.actbus.act_vel) * 6; ++i)
    {
      for (j = 0; j < 3; ++j)
        ReadActuator(j, 1);

      focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3
        - ACTENC_OFFSET;

      if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
        return 0;
    }
  }

  /* Here is the check */
  if (abs(delta) == MAX_STEP)
    for (i = 0; i < 3; ++i)
      if (abs(act_data[i].enc - start[i]) < STEP_MIN) {
          bprintf(err, "ActBus: Encoder failure detected, actuator %i\n", i);
          if (CommandData.actbus.tc_mode == TC_MODE_ENABLED) {
            bputs(err, "ActBus: Fully vetoing thermal correction");
            CommandData.actbus.tc_mode = TC_MODE_VETOED;
          }
          deferred = 0;
          fail[i] = 1;
        }

  for (i = 0; i < 3; ++i)
    CommandData.actbus.dead_reckon[i] += delta_dr;

  return 1;
}

static double CalibrateAD590(int counts)
{
  double t = I2T_M * counts + I2T_B + 273.15;

  /* if t < -73C or t > 67C, assume AD590 is broken */
  if (t < 170)
    t = -1;
  else if (t > 360)
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

void RecalcOffset(double new_gp, double new_gs)
{
  if (t_primary < 0 || t_secondary < 0)
    return;

  CommandData.actbus.sf_offset = (new_gp - CommandData.actbus.g_primary) *
    (t_primary - T_PRIMARY_FOCUS) - (new_gs - CommandData.actbus.g_secondary) *
    (t_secondary - T_SECONDARY_FOCUS) + CommandData.actbus.sf_offset;
}

/* Some sort of lame tmeperature filter */
static double TFilter(double old, double new, double t)
{
  if (old < 0)
    return new;

  return new / t + (1 - 1 / t) * old;
}

/* This function is called by the frame control thread */
void SecondaryMirror(void)
{
  static int firsttime = 1;

  static struct NiosStruct* sfCorrectionAddr;
  static struct NiosStruct* sfAgeAddr;
  static struct NiosStruct* sfOffsetAddr;
  static struct NiosStruct* tPrimeFidAddr;
  static struct NiosStruct* tSecondFidAddr;

  static struct BiPhaseStruct* tPrimary1Addr;
  static struct BiPhaseStruct* tSecondary1Addr;
  static struct BiPhaseStruct* tPrimary2Addr;
  static struct BiPhaseStruct* tSecondary2Addr;
  const int filter_len = CommandData.actbus.tc_filter;
  double t_primary1, t_secondary1;
  double t_primary2, t_secondary2;

  if (firsttime) {
    firsttime = 0;
    tPrimary1Addr = GetBiPhaseAddr("t_primary_1");
    tSecondary1Addr = GetBiPhaseAddr("t_secondary_1");
    tPrimary2Addr = GetBiPhaseAddr("t_primary_2");
    tSecondary2Addr = GetBiPhaseAddr("t_secondary_2");
    sfCorrectionAddr = GetNiosAddr("sf_correction");
    sfAgeAddr = GetNiosAddr("sf_age");
    sfOffsetAddr = GetNiosAddr("sf_offset");
    tPrimeFidAddr = GetNiosAddr("t_prime_fid");
    tSecondFidAddr = GetNiosAddr("t_second_fid");
  }

  /* Do nothing if we haven't heard from the actuators */
  if (focus < -ACTENC_OFFSET / 2)
    return;

  t_primary1 = CalibrateAD590(
      slow_data[tPrimary1Addr->index][tPrimary1Addr->channel]
      ) + AD590_CALIB_PRIMARY_1;
  t_primary2 = CalibrateAD590(
      slow_data[tPrimary2Addr->index][tPrimary2Addr->channel]
      ) + AD590_CALIB_PRIMARY_2;

  t_secondary1 = CalibrateAD590(
      slow_data[tSecondary1Addr->index][tSecondary1Addr->channel]
      ) + AD590_CALIB_SECONDARY_1;
  t_secondary2 = CalibrateAD590(
      slow_data[tSecondary2Addr->index][tSecondary2Addr->channel]
      ) + AD590_CALIB_SECONDARY_2;

  if (t_primary1 < 0 || t_primary2 < 0)
    t_primary = -1; /* autoveto */
  else if (fabs(t_primary1 - t_primary2) < CommandData.actbus.tc_spread) {
    if (t_primary1 >= 0 && t_primary2 >= 0)
      t_primary = TFilter(t_primary, (t_primary1 + t_primary2) / 2, filter_len);
    else if (t_primary1 >= 0)
      t_primary = TFilter(t_primary, t_primary1, filter_len);
    else
      t_primary = TFilter(t_primary, t_primary2, filter_len);
  } else {
    if (t_primary1 >= 0 && CommandData.actbus.tc_prefp == 1)
      t_primary = TFilter(t_primary, t_primary1, filter_len);
    else if (t_primary2 >= 0 && CommandData.actbus.tc_prefp == 2)
      t_primary = TFilter(t_primary, t_primary2, filter_len);
    else
      t_primary = -1; /* autoveto */
  }

  if (t_secondary1 < 0 || t_secondary2 < 0)
    t_secondary = -1; /* autoveto */
  else if (fabs(t_secondary1 - t_secondary2) < CommandData.actbus.tc_spread) {
    if (t_secondary1 >= 0 && t_secondary2 >= 0)
      t_secondary = TFilter(t_secondary, (t_secondary1 + t_secondary2) / 2,
          filter_len);
    else if (t_secondary1 >= 0)
      t_secondary = TFilter(t_secondary, t_secondary1, filter_len);
    else
      t_secondary = TFilter(t_secondary, t_secondary2, filter_len);
  } else {
    if (t_secondary1 >= 0 && CommandData.actbus.tc_prefs == 1)
      t_secondary = TFilter(t_secondary, t_secondary1, filter_len);
    else if (t_secondary2 >= 0 && CommandData.actbus.tc_prefs == 2)
      t_secondary = TFilter(t_secondary, t_secondary2, filter_len);
    else
      t_secondary = -1; /* autoveto */
  }

  if (CommandData.actbus.tc_mode != TC_MODE_VETOED &&
      (t_primary < 0 || t_secondary < 0)) {
    if (CommandData.actbus.tc_mode == TC_MODE_ENABLED)
      bputs(info, "Thermal Compensation: Autoveto raised.");
    CommandData.actbus.tc_mode = TC_MODE_AUTOVETO;
  } else if (CommandData.actbus.tc_mode == TC_MODE_AUTOVETO) {
    bputs(info, "Thermal Compensation: Autoveto lowered.");
    CommandData.actbus.tc_mode = TC_MODE_ENABLED;
  }

  correction = CommandData.actbus.g_primary * (t_primary - T_PRIMARY_FOCUS) -
    CommandData.actbus.g_secondary * (t_secondary - T_SECONDARY_FOCUS);

  /* convert to counts */
  correction /= ACTENC_TO_UM;

  /* re-adjust */
  correction = correction + focus - POSITION_FOCUS -
    CommandData.actbus.sf_offset;
  
  if (CommandData.actbus.sf_time < CommandData.actbus.tc_wait)
    CommandData.actbus.sf_time++;

  WriteData(tPrimeFidAddr, (t_primary - 273.15) * 500, NIOS_QUEUE);
  WriteData(tSecondFidAddr, (t_secondary - 273.15) * 500, NIOS_QUEUE);
  WriteData(sfCorrectionAddr, correction, NIOS_QUEUE);
  WriteData(sfAgeAddr, CommandData.actbus.sf_time / 10., NIOS_QUEUE);
  WriteData(sfOffsetAddr, CommandData.actbus.sf_offset, NIOS_FLUSH);
}


static void DoActuators(void)
{
  int update_dr = 1;

  switch(CommandData.actbus.focus_mode) {
    case ACTBUS_FM_PANIC:
      bputs(warning, "ActBus: Actuator Panic");
      BusComm(ALL_ACT, "T", 0, __inhibit_chatter);
      CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
      break;
    case ACTBUS_FM_DELTA:
      DeltaActuators();
      break;
    case ACTBUS_FM_DELFOC:
      CommandData.actbus.focus += focus;
      /* Fallthough */
    case ACTBUS_FM_THERMO:
    case ACTBUS_FM_FOCUS:
      update_dr = 0;
      if (SetNewFocus())
        /* fallthrough */
    case ACTBUS_FM_SERVO:
        ServoActuators(CommandData.actbus.goal, update_dr);
        break;
    case ACTBUS_FM_OFFSET:
        SetOffsets(CommandData.actbus.offset);
        /* fallthrough */
    case ACTBUS_FM_SLEEP:
        break;
    default:
        bputs(err, "ActBus: Unknown Focus Mode (%i), sleeping");
  }

  UpdateFlags();

  if (CommandData.actbus.focus_mode != ACTBUS_FM_PANIC)
    CommandData.actbus.focus_mode = ThermalCompensation();

  if (CommandData.actbus.reset_dr) {
    int i;

    for (i = 0; i < 3; ++i)
      CommandData.actbus.dead_reckon[i] = act_data[i].enc;

    CommandData.actbus.reset_dr = 0;
  }
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
#define SEND_SLEEP 100000 /* 100 miliseconds */
#define WAIT_SLEEP 50000 /* 50 miliseconds */
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
  static int drive_timeout = 0;

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

    /* Timeout check */
    if (drive_timeout == 1) {
      bputs(warning, "ActBus: Lock Motor drive timeout.");
      action = LA_STOP;
    }
    if (drive_timeout > 0)
      --drive_timeout;

    /* Figure out what to do... */
    switch (action) {
      case LA_STOP:
        drive_timeout = 0;
        bputs(info, "ActBus: Stopping lock motor.");
        strcpy(command, "T"); /* terminate all strings */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_OFF;
        break;
      case LA_EXTEND:
        drive_timeout = DRIVE_TIMEOUT;
        bputs(info, "ActBus: Extending lock motor.");
        LockCommand(command, "P0R"); /* move out forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_EXT;
        break;
      case LA_RETRACT:
        drive_timeout = DRIVE_TIMEOUT;
        bputs(info, "ActBus: Retracting lock motor.");
        LockCommand(command, "D0R"); /* move in forever */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_RET;
        break;
      case LA_STEP:
        drive_timeout = DRIVE_TIMEOUT;
        bputs(info, "ActBus: Stepping lock motor.");
        LockCommand(command, "P200000R"); /* move away from the limit switch */
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

void ActPotTrim(void)
{
  int i;

  for (i = 0; i < 3; ++i)
    if (act_data[i].enc > ACTENC_OFFSET / 2)
      CommandData.actbus.pos_trim[i] = act_data[i].enc - act_data[i].pos;
}

/* This function is called by the frame control thread */
void StoreActBus(void)
{
  int j;
  static int firsttime = 1;

  static struct BiPhaseStruct* lvdt10Addr;
  static struct BiPhaseStruct* lvdt11Addr;
  static struct BiPhaseStruct* lvdt13Addr;

  static struct NiosStruct* actbusResetAddr;
  static struct NiosStruct* lockPosAddr;
  static struct NiosStruct* lockStateAddr;
  static struct NiosStruct* lockGoalAddr;
  static struct NiosStruct* seizedBusAddr;
  static struct NiosStruct* lockPotAddr;
  static struct NiosStruct* lockLimSwAddr;
  static struct NiosStruct* lokmotPinAddr;

  static struct NiosStruct* lockVelAddr;
  static struct NiosStruct* lockAccAddr;
  static struct NiosStruct* lockMoveIAddr;
  static struct NiosStruct* lockHoldIAddr;

  static struct NiosStruct* actVelAddr;
  static struct NiosStruct* actAccAddr;
  static struct NiosStruct* actMoveIAddr;
  static struct NiosStruct* actHoldIAddr;
  static struct NiosStruct* actFlagsAddr;

  static struct NiosStruct* actPosAddr[3];
  static struct NiosStruct* actPostrimAddr[3];
  static struct NiosStruct* actEncAddr[3];
  static struct NiosStruct* actDeadRecAddr[3];
  static struct NiosStruct* actLGoodAddr[3];

  static struct NiosStruct* lvdtSpreadAddr;
  static struct NiosStruct* lvdtLowAddr;
  static struct NiosStruct* lvdtHighAddr;

  static struct NiosStruct* tcGPrimAddr;
  static struct NiosStruct* tcGSecAddr;
  static struct NiosStruct* tcStepAddr;
  static struct NiosStruct* tcWaitAddr;
  static struct NiosStruct* tcFilterAddr;
  static struct NiosStruct* tcModeAddr;
  static struct NiosStruct* tcSpreadAddr;
  static struct NiosStruct* tcPrefTpAddr;
  static struct NiosStruct* tcPrefTsAddr;
  static struct NiosStruct* secGoalAddr;
  static struct NiosStruct* absFocusAddr;

  if (firsttime) {
    firsttime = 0;

    lvdt10Addr = GetBiPhaseAddr("lvdt_10");
    lvdt11Addr = GetBiPhaseAddr("lvdt_11");
    lvdt13Addr = GetBiPhaseAddr("lvdt_13");

    actbusResetAddr = GetNiosAddr("actbus_reset");
    lokmotPinAddr = GetNiosAddr("lokmot_pin");
    lockPosAddr = GetNiosAddr("lock_pos");
    lockStateAddr = GetNiosAddr("lock_state");
    seizedBusAddr = GetNiosAddr("seized_bus");
    lockGoalAddr = GetNiosAddr("lock_goal");
    lockPotAddr = GetNiosAddr("lock_pot");
    lockLimSwAddr = GetNiosAddr("lock_lim_sw");

    for (j = 0; j < 3; ++j) {
      actPosAddr[j] = GetActNiosAddr(j, "pos");
      actPostrimAddr[j] = GetActNiosAddr(j, "postrim");
      actEncAddr[j] = GetActNiosAddr(j, "enc");
      actLGoodAddr[j] = GetActNiosAddr(j, "l_good");
      actDeadRecAddr[j] = GetActNiosAddr(j, "dead_rec");
    }

    tcGPrimAddr = GetNiosAddr("tc_g_prim");
    tcGSecAddr = GetNiosAddr("tc_g_sec");
    tcStepAddr = GetNiosAddr("tc_step");
    tcWaitAddr = GetNiosAddr("tc_wait");
    tcFilterAddr = GetNiosAddr("tc_filter");
    tcModeAddr = GetNiosAddr("tc_mode");
    tcSpreadAddr = GetNiosAddr("tc_spread");
    tcPrefTpAddr = GetNiosAddr("tc_pref_tp");
    tcPrefTsAddr = GetNiosAddr("tc_pref_ts");
    secGoalAddr = GetNiosAddr("sec_goal");
    absFocusAddr = GetNiosAddr("abs_focus");

    lvdtSpreadAddr = GetNiosAddr("lvdt_spread");
    lvdtLowAddr = GetNiosAddr("lvdt_low");
    lvdtHighAddr = GetNiosAddr("lvdt_high");

    actVelAddr = GetNiosAddr("act_vel");
    actAccAddr = GetNiosAddr("act_acc");
    actMoveIAddr = GetNiosAddr("act_move_i");
    actHoldIAddr = GetNiosAddr("act_hold_i");
    actFlagsAddr = GetNiosAddr("act_flags");

    lockVelAddr = GetNiosAddr("lock_vel");
    lockAccAddr = GetNiosAddr("lock_acc");
    lockMoveIAddr = GetNiosAddr("lock_move_i");
    lockHoldIAddr = GetNiosAddr("lock_hold_i");
  }

  lvdt[0] = slow_data[lvdt10Addr->index][lvdt10Addr->channel] *
    LVDT10_ADC_TO_ENC + LVDT10_ZERO;
  lvdt[1] = slow_data[lvdt11Addr->index][lvdt11Addr->channel] *
    LVDT11_ADC_TO_ENC + LVDT11_ZERO;
  lvdt[2] = slow_data[lvdt13Addr->index][lvdt13Addr->channel] *
    LVDT13_ADC_TO_ENC + LVDT13_ZERO;

  WriteData(actbusResetAddr, CommandData.actbus.off, NIOS_QUEUE);

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);

  for (j = 0; j < 3; ++j) {
    WriteData(actPosAddr[j], act_data[j].pos + CommandData.actbus.pos_trim[j],
        NIOS_QUEUE);
    WriteData(actPostrimAddr[j], CommandData.actbus.pos_trim[j], NIOS_QUEUE);
    WriteData(actEncAddr[j], act_data[j].enc, NIOS_QUEUE);
    WriteData(actLGoodAddr[j], CommandData.actbus.last_good[j] - ACTENC_OFFSET,
        NIOS_QUEUE);
    WriteData(actDeadRecAddr[j], CommandData.actbus.dead_reckon[j]
        - ACTENC_OFFSET, NIOS_QUEUE);
  }
  WriteData(absFocusAddr, focus, NIOS_QUEUE);

  WriteData(lockPotAddr, lock_data.adc[1], NIOS_QUEUE);
  WriteData(lockLimSwAddr, lock_data.adc[2], NIOS_QUEUE);
  WriteData(lockStateAddr, lock_data.state, NIOS_QUEUE);
  WriteData(seizedBusAddr, bus_seized, NIOS_QUEUE);
  WriteData(lockGoalAddr, CommandData.actbus.lock_goal, NIOS_QUEUE);
  WriteData(lockPosAddr, lock_data.pos, NIOS_QUEUE);

  WriteData(actVelAddr, CommandData.actbus.act_vel, NIOS_QUEUE);
  WriteData(actAccAddr, CommandData.actbus.act_acc, NIOS_QUEUE);
  WriteData(actMoveIAddr, CommandData.actbus.act_move_i, NIOS_QUEUE);
  WriteData(actHoldIAddr, CommandData.actbus.act_hold_i, NIOS_QUEUE);
  WriteData(actFlagsAddr, actbus_flags, NIOS_QUEUE);

  WriteData(lvdtSpreadAddr, CommandData.actbus.lvdt_delta, NIOS_QUEUE);
  WriteData(lvdtLowAddr, CommandData.actbus.lvdt_low, NIOS_QUEUE);
  WriteData(lvdtHighAddr, CommandData.actbus.lvdt_high, NIOS_QUEUE);

  WriteData(lockVelAddr, CommandData.actbus.lock_vel, NIOS_QUEUE);
  WriteData(lockAccAddr, CommandData.actbus.lock_acc, NIOS_QUEUE);
  WriteData(lockMoveIAddr, CommandData.actbus.lock_move_i, NIOS_QUEUE);
  WriteData(lockHoldIAddr, CommandData.actbus.lock_hold_i, NIOS_QUEUE);

  WriteData(tcGPrimAddr, CommandData.actbus.g_primary * 100., NIOS_QUEUE);
  WriteData(tcGSecAddr, CommandData.actbus.g_secondary * 100., NIOS_QUEUE);
  WriteData(tcModeAddr, CommandData.actbus.tc_mode, NIOS_QUEUE);
  WriteData(tcStepAddr, CommandData.actbus.tc_step, NIOS_QUEUE);
  WriteData(tcSpreadAddr, CommandData.actbus.tc_spread * 500., NIOS_QUEUE);
  WriteData(tcPrefTpAddr, CommandData.actbus.tc_prefp, NIOS_QUEUE);
  WriteData(tcPrefTsAddr, CommandData.actbus.tc_prefs, NIOS_QUEUE);
  WriteData(tcWaitAddr, CommandData.actbus.tc_wait / 10., NIOS_QUEUE);
  WriteData(tcFilterAddr, CommandData.actbus.tc_filter, NIOS_QUEUE);
  WriteData(secGoalAddr, CommandData.actbus.focus, NIOS_FLUSH);
}

void CopyActuators(void)
{
  static struct BiPhaseStruct* act0LGoodAddr;
  static struct BiPhaseStruct* act1LGoodAddr;
  static struct BiPhaseStruct* act2LGoodAddr;
  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    act0LGoodAddr = GetBiPhaseAddr("act0_l_good");
    act1LGoodAddr = GetBiPhaseAddr("act1_l_good");
    act2LGoodAddr = GetBiPhaseAddr("act2_l_good");
  }

  CommandData.actbus.last_good[0] = CommandData.actbus.dead_reckon[0] =
    slow_data[act0LGoodAddr->index][act0LGoodAddr->channel];
  CommandData.actbus.last_good[1] = CommandData.actbus.dead_reckon[1] =
    slow_data[act1LGoodAddr->index][act1LGoodAddr->channel];
  CommandData.actbus.last_good[2] = CommandData.actbus.dead_reckon[2] =
    slow_data[act2LGoodAddr->index][act2LGoodAddr->channel];
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
      CopyActuators(); /* let the NiC MCC know what's going on */
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

    DoLock(); /* Lock motor stuff -- this will seize the bus until
                 the lock motor's state has settled */

    DoActuators(); /* Actuator stuff -- this may seize the bus */

    for (i = 0; i < 3; ++i)
      ReadActuator(i, 1);

    focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3
      - ACTENC_OFFSET;

    usleep(10000);
  }
}
