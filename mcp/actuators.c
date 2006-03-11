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

/* Define this symbol to have mcp log all actuator bus traffic */
#define ACTBUS_CHATTER

#define ACT_BUS "/dev/ttyS1"

#define NACT 4

/* EZ Stepper status bits */
#define EZ_ERROR  0x0F
#define EZ_READY  0x20
#define EZ_STATUS 0x40

static int bus_fd = -1;
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
  "Lock Motor"};

int act_setserial(char *input_tty) {
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

void BusSend(int who, const char* what)
{
  size_t len = strlen(what) + 7;
  char *buffer = malloc(len);

  snprintf(buffer, len, "/%i%s\r\n", who + 1, what);
#ifdef ACTBUS_CHATTER
  bprintf(info, "Wrote: %s", buffer);
#endif
  if (write(bus_fd, buffer, strlen(buffer)) < 0)
    berror(err, "Error writing on bus");

  free(buffer);
}

int BusRecv(char* buffer)
{
  int i, fd, status = 0;
  fd_set rfds;
  struct timeval timeout = {1, 0};
  unsigned char byte;
  char* ptr = buffer;

  FD_ZERO(&rfds);
  FD_SET(bus_fd, &rfds);

  fd = select(bus_fd + 1, &rfds, NULL, NULL, &timeout);

  if (fd == -1)
    berror(err, "Error waiting for input on bus");
  else if (!fd) /* Timeout */
    return -1;
  else {
    int state = 0;
    int had_errors = 0;
    int read_tries = 100;

    for(;;) {
      i = read(bus_fd, &byte, 1);
      if (i <= 0) {
        if (state == 7 || state == 13)
          break;
        if (errno == EAGAIN && read_tries) {
          read_tries--;
          usleep(1000);
          continue;
        } else {
          *ptr = 0;
          berror(warning, "Unexpected out-of-data reading bus (%i %s)", state,
              buffer);
          return -2;
        }
      }
      //      bprintf(info, "%02x. (%i/%i)", byte, state, i);

      /* The following involves a number of semi-hidden fallthroughs which
       * attempt to recover malformed strings */
      switch (state) {
        case 0: /* RS-485 turnaround */
          state++;
          if (byte != 0xFF) { /* RS-485 turnaround */
            bputs(warning, "RS-485 turnaround not found in response");
            had_errors++;
          } else
            break;
        case 1: /* start byte */
          state++;
          if (byte != 0x2F) { /* Start character == '/' */
            had_errors++;
            bputs(warning, "Start byte not found in response");
          } else
            break;
        case 2: /* address byte */
          state++;
          if (byte != 0x30) { /* Recipient address (should be '0') */
            had_errors++;
            bputs(warning, "Found misaddressed response");
          }
          if (had_errors > 1) {
            bputs(err, "Too many errors parsing response string, aborting.");
            state = 13;
          }
          break;
        case 3: /* state byte */
          state++;
          if (byte & EZ_STATUS)
            status = byte & (EZ_ERROR | EZ_READY);
          else {
            bputs(err, "Status byte malfomed in response string, aborting.");
            state = 13;
          }
          break;
        case 4: /* response */
          if (byte == 0x3) /* ETX */
            state++;
          else
            *(ptr++) = byte;
          break;
        case 5:
          state++;
          if (byte != 0x0D) { /* \r */
            bputs(err, "Malformed footer in response string, aborting.");
            state = 13;
          }
          break;
        case 6:
          state++;
          if (byte != 0x0A) { /* \n */
            bputs(err, "Malformed footer in response string, aborting.");
            state = 13;
          }
          break;
        case 7: /* End of string check */
          bputs(err, "Malformed footer in response string, aborting.");
          state = 13;
        case 13: /* General abort: flush input */
          break;
      }
    }
  }

  *ptr = '\0';

  return status;
}

void PollBus(int rescan, int *status)
{
  int i, result;
  char buffer[1000];

  if (rescan)
    bputs(info, "ActBus: Repolling Actuator Bus.");
  else
    bputs(info, "ActBus: Polling Actuator Bus.");

  for (i = 0; i < NACT; ++i) {
    BusSend(i, "&");
    if ((result = BusRecv(buffer)) < 0) {
      bprintf(warning, "ActBus: No response from %s, will repoll later.",
          name[i]);
      status[i] = 1;
    } else {
      bprintf(info, "Read: %i = %s\n", result, buffer);
    }
  }
}

void ActuatorBus(void)
{
  unsigned int controller_status[NACT];

  bputs(startup, "ActBus: ActuatorBus startup.");

  bus_fd = act_setserial(ACT_BUS);

  PollBus(0, controller_status);
}
