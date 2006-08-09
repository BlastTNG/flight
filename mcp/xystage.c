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

#include "mcp.h"

#ifdef USE_XY_THREAD

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

#include "command_struct.h"
#include "pointing_struct.h"
#include "tx.h"

#ifdef USE_XY_STAGE
#  error Both USE_XY_STAGE and USE_XY_THREAD defined.
#endif

/* Define this symbol to have mcp log all actuator bus traffic */
#define ACTBUS_CHATTER

#define ACT_BUS "/dev/ttyS1"

#define STAGEXNUM 0
#define STAGEYNUM 1
#define NACT 2
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

/* in commands.c */
double LockPosition(double elevation);

extern short int InCharge; /* tx.c */

static int bus_fd = -1;
static const char *name[NACT] = {"XY Stage X", "XY Stage Y"};
static const int id[NACT] = {0x36, 0x37};

static char gp_buffer[1000];
static struct stepper_struct {
  unsigned char buffer[1000];
  int status;
  int sequence;
} stepper[NACT];

static struct stage_struct {
  unsigned int xpos, ypos;
  unsigned int xlim, ylim;
  unsigned int xstp, ystp;
  unsigned int xstr, ystr;
  unsigned int xvel, yvel;
} stage_data;

static int act_setserial(char *input_tty)
{
  int fd;
  struct termios term;

  if ((fd = open(input_tty, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    berror(tfatal, "StageBus: Unable to open serial port");

  if (tcgetattr(fd, &term))
    berror(tfatal, "StageBus: Unable to get serial device attributes");

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
    berror(tfatal, "StageBus: Error setting serial output speed");

  if(cfsetispeed(&term, B9600))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "StageBus: Error setting serial input speed");

  if( tcsetattr(fd, TCSANOW, &term) )
    berror(tfatal, "StageBus: Unable to set serial attributes");

  return fd;
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
    bprintf(info, "StageBus: Request=%s (%s)", HexDump(buffer, len), what);
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
            bputs(warning, "StageBus: RS-485 turnaround not found in response");
            had_errors++;
          } else
            break;
        case 1: /* start byte */
          state++;
          if (byte != 0x02) { /* STX */
            had_errors++;
            bputs(warning, "StageBus: Start byte not found in response");
          } else
            break;
        case 2: /* address byte */
          state++;
          if (byte != 0x30) { /* Recipient address (should be '0') */
            had_errors++;
            bputs(warning, "StageBus: Found misaddressed response");
          }
          if (had_errors > 1) {
            bputs(err,
                "StageBus: Too many errors parsing response string, aborting.");
            bprintf(err, "StageBus: Response was=%s (%x)\n", HexDump(buffer, len),
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
                "StageBus: Status byte malfomed in response string, aborting.");
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
            bprintf(err, "StageBus: Checksum error in response (%02x).",
                checksum);
          break;
        case 6: /* End of string check */
          bputs(err, "StageBus: Malformed footer in response string, aborting.");
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
      bprintf(info, "StageBus: Response=%s (%x)\n", buffer, status);
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
    bprintf(warning, "StageBus: Timeout waiting for response from %s (RIFB)",
        name[who]);
    CommandData.actbus.force_repoll = 1;
    return 0;
  }

  return atoi(gp_buffer);
}

static void DiscardBusRecv(int flag, int who, int inhibit_chatter)
{
  int i;

  /* Discard response to get it off the bus */
  if ((i = BusRecv(gp_buffer, 0, inhibit_chatter)) & (ACTBUS_TIMEOUT
        | ACTBUS_OOD))
  {
    i = who;
    bprintf(warning,
        "StageBus: Timeout waiting for response from %s.", name[who]);
    CommandData.actbus.force_repoll = 1;
  }
#ifndef ACTBUS_CHATTER
  else if (flag)
    bprintf(info, "StageBus: Controller response: %s\n", gp_buffer);
#endif
}

static int PollBus(int rescan)
{
  int i, result;
  int all_ok = 1;

  if (rescan)
    bputs(info, "StageBus: Repolling Stage Bus.");
  else
    bputs(info, "StageBus: Polling Stage Bus.");

  for (i = 0; i < NACT; ++i) {
    if (rescan && stepper[i].status != -1)
      continue;
    BusSend(i, "&", 0);
    if ((result = BusRecv(gp_buffer, 0, 0)) & (ACTBUS_TIMEOUT
          | ACTBUS_OOD)) {
      bprintf(warning, "StageBus: No response from %s, will repoll later.",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    } else if (!strncmp(gp_buffer, "EZHR17EN AllMotion", 18)) {
      bprintf(info, "StageBus: Found type 17EN device %s at address %i.\n",
          name[i], id[i] - 0x30);
      stepper[i].status = 0;
    } else if (!strncmp(gp_buffer, "EZHR23 All Motion", 17)) {
      bprintf(info, "StageBus: Found type 23 device %s at address %i.\n", name[i],
          id[i] - 0x30);
      stepper[i].status = 0;
    } else {
      bprintf(warning,
          "StageBus: Unrecognised response from %s, will repoll later.\n",
          name[i]);
      stepper[i].status = -1;
      all_ok = 0;
    }
  }

  CommandData.actbus.force_repoll = 0;

  return all_ok;
}

static void ReadStage(void)
{
  static int counter = 0;
  if (stepper[STAGEXNUM].status == -1 || stepper[STAGEYNUM].status == -1)
    return;

  stage_data.xpos = ReadIntFromBus(STAGEXNUM, "?0", 1);
  stage_data.ypos = ReadIntFromBus(STAGEYNUM, "?0", 1);

  if (counter == 0)
    stage_data.xstr = ReadIntFromBus(STAGEXNUM, "?1", 1);
  else if (counter == 1)
    stage_data.xstp = ReadIntFromBus(STAGEXNUM, "?3", 1);
  else if (counter == 2)
    stage_data.xlim = ReadIntFromBus(STAGEXNUM, "?4", 1);
  else if (counter == 3)
    stage_data.xvel = ReadIntFromBus(STAGEXNUM, "?5", 1);
  else if (counter == 4)
    stage_data.ystr = ReadIntFromBus(STAGEYNUM, "?1", 1);
  else if (counter == 5)
    stage_data.ystp = ReadIntFromBus(STAGEYNUM, "?3", 1);
  else if (counter == 6)
    stage_data.yvel = ReadIntFromBus(STAGEYNUM, "?5", 1);
  else if (counter == 7)
    stage_data.ylim = ReadIntFromBus(STAGEYNUM, "?4", 1);

  counter = (counter + 1) % 8;
}

/* This function is called by the frame control thread */
void StoreStageBus(void)
{
  static int firsttime = 1;

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

  if (firsttime) {
    firsttime = 0;

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
  }

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
}

void StageBus(void)
{
  int poll_timeout = POLL_TIMEOUT;
  int all_ok = 0;
  int i;
  int my_cindex = 0;

  bputs(startup, "StageBus: StageBus startup.");

  for (i = 0; i < NACT; ++i)
    stepper[i].sequence = 1;

  bus_fd = act_setserial(ACT_BUS);

  all_ok = PollBus(0);

  for (;;) {
    while (!InCharge) { /* NiC MCC traps here */
      CommandData.actbus.force_repoll = 1; /* repoll bus as soon as gaining
                                              control */
      BusRecv(NULL, 1, 1); /* this is a blocking call - clear the recv buffer */
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
    if (CommandData.actbus.caddr[my_cindex] == 0x36 ||
        CommandData.actbus.caddr[my_cindex] == 0x37) {
      BusSend(CommandData.actbus.caddr[my_cindex],
          CommandData.actbus.command[my_cindex], 0);
      /* Discard response to get it off the bus */
      DiscardBusRecv(1, CommandData.actbus.caddr[my_cindex] - 0x30, 0);
      CommandData.actbus.caddr[my_cindex] = 0;
    }

    if (CommandData.xystage.is_new) {
      if (CommandData.xystage.mode == XYSTAGE_PANIC) {
        bputs(info, "StageBus: Panic");
        BusSend(STAGEXNUM, "T", 0);
        DiscardBusRecv(0, STAGEXNUM, 0);
        BusSend(STAGEYNUM, "T", 0);
        DiscardBusRecv(0, STAGEYNUM, 0);
      } else if (CommandData.xystage.mode == XYSTAGE_GOTO) {
        if (CommandData.xystage.xvel > 0) {
          sprintf(gp_buffer, "L2m30l30V%iA%iR", CommandData.xystage.xvel,
              CommandData.xystage.x1);
          bprintf(info, "StageBus: Move X to %i at speed %i",
              CommandData.xystage.x1, CommandData.xystage.xvel);
          BusSend(STAGEXNUM, gp_buffer, 0);
          DiscardBusRecv(0, STAGEXNUM, 0);
        }
        if (CommandData.xystage.yvel > 0) {
          sprintf(gp_buffer, "L2m30l30V%iA%iR", CommandData.xystage.yvel,
              CommandData.xystage.y1);
          bprintf(info, "StageBus: Move Y to %i at speed %i",
              CommandData.xystage.y1, CommandData.xystage.yvel);
          BusSend(STAGEYNUM, gp_buffer, 0);
          DiscardBusRecv(0, STAGEYNUM, 0);
        }
      } else if (CommandData.xystage.mode == XYSTAGE_JUMP) {
        if (CommandData.xystage.xvel > 0 && CommandData.xystage.x1 != 0) {
          bprintf(info, "StageBus: Jump X by %i at speed %i",
              CommandData.xystage.x1, CommandData.xystage.xvel);
          sprintf(gp_buffer, "L2m30l30V%i%c%iR", CommandData.xystage.xvel,
              (CommandData.xystage.x1 > 0) ? 'P' : 'D',
              abs(CommandData.xystage.x1));
          BusSend(STAGEXNUM, gp_buffer, 0);
          DiscardBusRecv(0, STAGEXNUM, 0);
        }
        if (CommandData.xystage.yvel > 0 && CommandData.xystage.y1 != 0) {
          bprintf(info, "StageBus: Jump Y by %i at speed %i",
              CommandData.xystage.y1, CommandData.xystage.yvel);
          sprintf(gp_buffer, "L2m30l30V%i%c%iR", CommandData.xystage.yvel,
              (CommandData.xystage.y1 > 0) ? 'P' : 'D',
              abs(CommandData.xystage.y1));
          BusSend(STAGEYNUM, gp_buffer, 0);
          DiscardBusRecv(0, STAGEYNUM, 0);
        }
      } else if (CommandData.xystage.mode == XYSTAGE_SCAN) {
        if (CommandData.xystage.xvel > 0 && CommandData.xystage.x1 > 0) {
        } else if (CommandData.xystage.yvel > 0 && CommandData.xystage.y1 > 0) {
        }
      }
      CommandData.xystage.is_new = 0;
    }

    ReadStage();

    usleep(10000);
  }
}
#endif
