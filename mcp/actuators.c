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

#include "mcp.h"
#include "command_struct.h"
#include "tx.h"

/* Define this symbol to have mcp log all actuator bus traffic */
#undef ACTBUS_CHATTER

#define ACT_BUS "/dev/ttyS7"

#define LOCKNUM 3
#define NACT 4
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

#define ACT_RECV_ABORT 3000000

#define LOCK_MOTOR_DATA_TIMER 100

/* in commands.c */
double LockPosition(double elevation);

extern short int InCharge; /* tx.c */

static int bus_fd = -1;
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
  "Lock Motor"};

static char gp_buffer[1000];
static struct stepper_struct {
  unsigned char buffer[1000];
  int status;
  int sequence;
} stepper[NACT];

struct lock_struct {
  unsigned int pos;
  unsigned short adc[4];
} lock_data;

/****************************************************************/
/* Read the state of the lock motor pin (or guess it, whatever) */
/****************************************************************/
int pinIsIn(void)
{
  return(CommandData.pin_is_in);
}

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

#ifdef ACTBUS_CHATTER
static char hex_buffer[1000];
static const char* HexDump(const unsigned char* buffer, int len)
{
  int i;

  sprintf(hex_buffer, "%02x", buffer[0]);
  for (i = 1; i < len; ++i)
    sprintf(hex_buffer + i * 3 - 1, ".%02x", buffer[i]);

  return hex_buffer;
}
#endif

static void BusSend(int who, const char* what)
{
  size_t len = strlen(what) + 5;
  char *buffer = malloc(len);
  unsigned char chk = 3;
  unsigned char *ptr;

  buffer[0] = 0x2;
  buffer[1] = who + 0x31;
  buffer[2] = '1';
  sprintf(buffer + 3, "%s", what);
  buffer[len - 2] = 0x3;
  for (ptr = buffer; *ptr != '\03'; ++ptr)
    chk ^= *ptr;
  buffer[len - 1] = chk;
#ifdef ACTBUS_CHATTER
  bprintf(info, "ActBus: Request=%s", HexDump(buffer, len));
#endif
  if (write(bus_fd, buffer, len) < 0)
    berror(err, "Error writing on bus");

  free(buffer);
}

static int BusRecv(char* buffer, int nic)
{
  int i, fd, status = 0;
  fd_set rfds;
  struct timeval timeout = {1, 0};
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

    for(;;) {
      i = read(bus_fd, &byte, 1);
      if (i <= 0) {
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
      bprintf(info, "%02x. (%i/%i)", byte, state, i);
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
      bprintf(info, "ActBus: Found %s at address %i.\n", name[i], i + 1);
      stepper[i].status = 0;
    } else {
      bprintf(warning,
          "ActBus: Unrecognised response from %s, will repoll later.\n",
          gp_buffer);
      stepper[i].status = -1;
      all_ok = 0;
    }
  }

  CommandData.actbus.force_repoll = 0;

  return all_ok;
}

static void GetLockData(void)
{
  static int counter = 0;
  int result;

  if (stepper[LOCKNUM].status == -1)
    return;

  if (counter++ < LOCK_MOTOR_DATA_TIMER)
    return;

  counter = 0;

  BusSend(LOCKNUM, "?0");
  if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
    bputs(warning, "ActBus: Timeout waiting for response from lock motor.");
    stepper[LOCKNUM].status = -1;
    return;
  }

  lock_data.pos = atoi(gp_buffer);

  BusSend(LOCKNUM, "?aa");
  if ((result = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD)) {
    bputs(warning, "ActBus: Timeout waiting for response from lock motor.");
    stepper[LOCKNUM].status = -1;
    return;
  }

  sscanf(gp_buffer, "%hi,%hi,%hi,%hi", &lock_data.adc[0], &lock_data.adc[1],
      &lock_data.adc[2], &lock_data.adc[3]);
}

#if 0
/************************************************************************/
/*                                                                      */
/*    Do Lock Logic: check status, determine if we are locked, etc      */
/*                                                                      */
/************************************************************************/
#define PULSE_LENGTH 400   /* 4 seconds */
#define SEARCH_COUNTS 500 /* 5 seconds */
static int GetLockBits(unsigned short lockBits)
{
  static int is_closing = 0;
  static int is_opening = 0;
  static int is_searching = 0;

  /* check for commands from CommandData */
  if (CommandData.pumps.lock_in) {
    CommandData.pumps.lock_in = 0;
    is_opening = 0;
    is_closing = PULSE_LENGTH;
    is_searching = 0;
  } else if (CommandData.pumps.lock_out) {
    CommandData.pumps.lock_out = 0;
    is_opening = PULSE_LENGTH;
    is_closing = 0;
    is_searching = 0;
  } else if (CommandData.pumps.lock_point) {
    CommandData.pumps.lock_point = 0;
    is_searching = SEARCH_COUNTS;
    is_opening = 0;
    is_closing = 0;
  } else if (CommandData.pumps.lock_off) {
    CommandData.pumps.lock_off = 0;
    is_opening = 0;
    is_closing = 0;
    is_searching = 0;
  }

  /* elevation searching stuff */
  if (is_searching > 1) {
    if (fabs(ACSData.enc_elev - LockPosition(ACSData.enc_elev)) > 0.2)
      is_searching = SEARCH_COUNTS;
    else
      is_searching--;
  } else if (is_searching == 1) {
    is_searching = 0;
    is_closing = PULSE_LENGTH;
  }

  /* check limit switches -- if both bits are set, the motor is running
   * -- if we have reached a limit we can stop going in the direction
   * of the limitswitch that is active */
  if ((lockBits & LOKMOT_ISIN) && (~lockBits & LOKMOT_ISOUT)) {
    CommandData.pin_is_in = 1;
    is_closing = 0;
  } else if ((lockBits & LOKMOT_ISOUT) && (~lockBits & LOKMOT_ISIN)) {
    CommandData.pin_is_in = 0;
    is_opening = 0;
  }

  /* set lock bits */
  if (is_closing) {
    is_closing--;
    return(LOKMOT_IN | LOKMOT_ON);
  } else if (is_opening) {
    is_opening--;
    return(LOKMOT_OUT | LOKMOT_ON);
  }


  return 0;
}
#endif

/* This function is called by the frame control thread */
void StoreActBus(void)
{
  int i;
  static int firsttime = 1;

  static struct NiosStruct* lockPosAddr;
  static struct NiosStruct* lockAdcAddr[4];
  static struct NiosStruct* lokmotPinAddr;

  if (firsttime) {
    firsttime = 0;
    lokmotPinAddr = GetNiosAddr("lokmot_pin");
    lockPosAddr = GetNiosAddr("lock_pos");
    lockAdcAddr[0] = GetNiosAddr("lock_adc0");
    lockAdcAddr[1] = GetNiosAddr("lock_adc1");
    lockAdcAddr[2] = GetNiosAddr("lock_adc2");
    lockAdcAddr[3] = GetNiosAddr("lock_adc3");
  }

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);

  for (i = 0; i < 4; ++i)
    WriteData(lockAdcAddr[i], lock_data.adc[i], NIOS_QUEUE);
  WriteData(lockPosAddr, lock_data.pos, NIOS_FLUSH);
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
    while (!InCharge) /* NiC MCC traps here */
      BusRecv(NULL, 1); /* this is a blocking call */

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
      if ((i = BusRecv(gp_buffer, 0)) & (ACTBUS_TIMEOUT | ACTBUS_OOD))
        bputs(warning,
            "ActBus: Timeout waiting for response after uplinked command.");
      CommandData.actbus.caddr[my_cindex] = 0;
    }

    /* Get data from lock motor */
    GetLockData();

    usleep(10000);
  }
}
