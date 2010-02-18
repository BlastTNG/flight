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
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include "ezstep.h"
#include "blast.h"

//TODO need to improve the error handling situation

static int ez_setserial(struct ezbus* bus, const char* input_tty)
{
  int fd;
  struct termios term;

  if ((fd = open(input_tty, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    berror(tfatal, "%sUnable to open serial port (%s)", bus->name, input_tty);

  if (tcgetattr(fd, &term))
    berror(tfatal, "%sUnable to get serial device attributes", bus->name);

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
    berror(tfatal, "%sError setting serial output speed", bus->name);

  if(cfsetispeed(&term, B9600))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "%sError setting serial input speed", bus->name);

  if( tcsetattr(fd, TCSANOW, &term) )
    berror(tfatal, "%sUnable to set serial attributes", bus->name);

  return fd;
}

//ensure 'who' is an integer stepper index
//NB: do not use when who is a multi-stepper character
static inline int iWho(int who)
{
  return (who <= EZ_BUS_NACT) ? (who - 1) : (who - 0x31);
}

//make sure 'who' becomes the correct ASCII character
static inline int cWho(int who)
{
  return (who <= EZ_BUS_NACT) ? (who+0x30) : who;
}

//lookup/generate a name for stepper 'who'
static char hidden_buffer[EZ_BUS_NAME_LEN];
static char* stepName(struct ezbus* bus, int who)
{
  int cwho = cWho(who);
  int iwho = iWho(who);
  if (cwho < 0x30 || cwho >= 0x60) {	  //invalid who
    sprintf(hidden_buffer, "Invalid Stepper");
    return hidden_buffer;
  }
  else if (cwho == 0x5f) {		  //all steppers
    sprintf(hidden_buffer, "All Steppers");
    return hidden_buffer;
  }
  else if (cwho > 0x50) {		  //groups of 4
    sprintf(hidden_buffer, "4-Stepper Group \'%c\'", (char)cwho);
    return hidden_buffer;
  }
  else if (cwho > 0x40) {		  //groups of 2
    sprintf(hidden_buffer, "2-Stepper Group \'%c\'", (char)cwho);
    return hidden_buffer;
  }
  if (cwho > 0x30) {			  //single controller
    if (bus->step[iwho].name[0] != '\0')
      return bus->step[iwho].name;
    else {
      sprintf(hidden_buffer, "Stepper #%c", cwho);
      return hidden_buffer;
    }
  }
  bprintf(err, "EZ-Stepper: stepName(): Unexpected Trap!\n");
  sprintf(hidden_buffer, "Invalid Stepper");
  return hidden_buffer;
}

int EZBus_Init(struct ezbus* bus,const char* tty,const char* name,int chatter)
{
  int i;
  strncpy(bus->name, name, EZ_BUS_NAME_LEN);
  bus->name[EZ_BUS_NAME_LEN-1] = '\0';
  bus->seized = -1;
  bus->chatter = chatter;
  bus->fd = ez_setserial(bus, tty);
  for (i=0; i<EZ_BUS_NACT; i++) {
    bus->step[i].status = 0;
    bus->step[i].name[0] = '\0';
  }
  return bus->fd;
}

void EZBus_Add(struct ezbus* bus, int who, const char* name)
{
  int iwho = iWho(who);
  if (iwho >= EZ_BUS_NACT) {
    if (bus->chatter > 1) 
      bprintf(warning, "%sFailed to add \'%c\'\n", bus->name, cWho(who));
    return;
  }
  bus->step[iwho].status |= EZ_STEP_USED;
  strncpy(bus->step[iwho].name, name, EZ_BUS_NAME_LEN);
  bus->step[iwho].name[EZ_BUS_NAME_LEN-1] = '\0';
}

int EZBus_Take(struct ezbus* bus, int who)
{
  if (bus->seized != -1)
    return 0;

  if (bus->seized != cWho(who)) {
    if (bus->chatter > 0)
      bprintf(info, "%sBus seized by %s.\n", bus->name, stepName(bus,who));
    bus->seized = cWho(who);
  }

  return 1;
}

void EZBus_Release(struct ezbus* bus, int who)
{
  if (bus->seized == cWho(who)) {
    if (bus->chatter > 0)
      bprintf(info, "%sBus released by %s.\n", bus->name, stepName(bus,who));
    bus->seized = -1;
  }
}

static const char* HexDump(const unsigned char* from, char* to, int len)
{
  int i;
  sprintf(to, "%02x", from[0]);
  for (i = 1; i < len; ++i)
    sprintf(to + i * 3 - 1, ".%02x", from[i]);
  return to;
}

void EZBus_Send(struct ezbus *bus, int who, const char* what)
{
  size_t len = strlen(what) + 5;
  char* buffer = malloc(len);
  unsigned char chk = 3;
  unsigned char *ptr;
  static int sequence = 1;
  char* hex_buffer;

  sequence = (sequence + 1) % 7;
  buffer[0] = 0x2;
  buffer[1] = cWho(who);
  buffer[2] = 0x30 + sequence;
  sprintf(buffer + 3, "%s", what);
  buffer[len - 2] = 0x3;
  for (ptr = (unsigned char *)buffer; *ptr != '\03'; ++ptr)
    chk ^= *ptr;
  buffer[len - 1] = chk;
  if (bus->chatter > 1) {
    hex_buffer = malloc(2*len+1);
    bprintf(info, "%sRequest=%s (%s)", bus->name, 
	HexDump((unsigned char *)buffer, hex_buffer, len), what);
    free(hex_buffer);
  }
  if (write(bus->fd, buffer, len) < 0)
    berror(err, "%sError writing on bus", bus->name);

  free(buffer);
}

#define EZ_BUS_RECV_ABORT 3000000 /* state for general parsing abort */

int EZBus_Recv(struct ezbus* bus)
{
  int fd, status = 0;
  fd_set rfds;
  struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
  unsigned char byte;
  unsigned char checksum = 0;
  char* ptr = bus->buffer;
  char* hex_buffer;

  FD_ZERO(&rfds);
  FD_SET(bus->fd, &rfds);

  fd = select(bus->fd + 1, &rfds, NULL, NULL, &timeout);

  if (fd == -1)
    berror(err, "%sError waiting for input on bus",bus->name);
  else if (!fd) /* Timeout */
    return EZ_BUS_TIMEOUT;
  else {
    int state = 0;
    int had_errors = 0;
    int read_tries = 100;
    int len;

    for(;;) {
      len = read(bus->fd, &byte, 1);
      if (len <= 0) {
        if (state == 6 || state == EZ_BUS_RECV_ABORT)
          break;
        if (errno == EAGAIN && read_tries) {
          read_tries--;
          usleep(1000);
          continue;
        } else {
          berror(warning, "%sUnexpected out-of-data reading bus (%i)", 
	      bus->name, state);
          return EZ_BUS_OOD;
        }
      }
      checksum ^= byte;

      /* The following involves a number of semi-hidden fallthroughs which
       * attempt to recover malformed strings */
      switch (state) {
        case 0: /* RS-485 turnaround */
          state++;
          if (byte != 0xFF) { /* RS-485 turnaround */
            bprintf(warning, "%sRS-485 turnaround not found in response", 
		bus->name);
            had_errors++;
          } else
            break;
        case 1: /* start byte */
          state++;
          if (byte != 0x02) { /* STX */
            had_errors++;
            bprintf(warning, "%sStart byte not found in response", bus->name);
          } else
            break;
        case 2: /* address byte */
          state++;
          if (byte != 0x30) { /* Recipient address (should be '0') */
            had_errors++;
            bprintf(warning, "%sFound misaddressed response", bus->name);
          }
          if (had_errors > 1) {
            bprintf(err, "%sToo many errors parsing response string, aborting.",
	       	bus->name);
            state = EZ_BUS_RECV_ABORT;
          }
          break;
        case 3: /* status byte */
          state++;
          if (byte & EZ_STATUS)
            status = byte & (EZ_ERROR | EZ_READY);
          else {
            bprintf(err, "%sStatus byte malfomed in response string, aborting.",
	       	bus->name);
            state = EZ_BUS_RECV_ABORT;
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
            bprintf(err, "%sChecksum error in response (%02x).", bus->name,
                checksum);
          break;
        case 6: /* End of string check */
          bprintf(err, "%sMalformed footer in response string, aborting.",
	      bus->name);
          state = EZ_BUS_RECV_ABORT;
        case EZ_BUS_RECV_ABORT: /* General abort: flush input */
          break;
      }
    }
  }

  *ptr = '\0';

  if (bus->chatter > 1) {
    size_t len;
    len = strlen(bus->buffer);
    hex_buffer = malloc(2*len+1);
    bprintf(info, "%sResponse=%s (%s) Status=%x\n", bus->name, 
	HexDump((unsigned char *)bus->buffer, hex_buffer, len), 
	bus->buffer, status);
    free(hex_buffer);
  }

  return status;
}

int EZBus_Comm(struct ezbus* bus, int who, const char* what, int naive)
{
  int result = 0;
  int ok;

  do {
    ok = 1;
    EZBus_Send(bus, who, what);

    /* Muliple motors */
    if (cWho(who) >= 0x40)
      break;

    if ((result = EZBus_Recv(bus)) & (EZ_BUS_TIMEOUT | EZ_BUS_OOD))
    {
      bprintf(warning, "%sTimeout waiting for response from %s\n.", 
	  bus->name, stepName(bus,who));
      //TODO there used to be a forced repoll here, rethink with error handling
      return result;
    } else {
      switch (result & EZ_ERROR) {
	case EZ_ERR_INIT:
	  bprintf(warning, "%s%s: initialisation error.\n", 
	      bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_BADCMD:
	  bprintf(warning, "%s%s: bad command.\n", 
	      bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_BADOP:
	  bprintf(warning, "%s%s: bad operand.\n", 
	      bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_COMM:
	  bprintf(warning, "%s%s: communications error.\n", 
	      bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_NOINIT:
	  bprintf(warning, "%s%s: not initialied.\n", 
	      bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_OVER:
	  bprintf(warning, "%s%s: overload.\n", bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_NOMOVE:
	  bprintf(warning, "%s%s: move not allowed.\n", 
	      bus->name, stepName(bus,who));
	  break;
	case EZ_ERR_BUSY:
	  bprintf(warning, "%s%s: command overflow.\n", 
	      bus->name, stepName(bus,who));
	  usleep(10000);
	  ok = 0;
	  break;
      }
    }
  } while (!ok && !naive);

  return result;
}

int EZBus_ReadInt(struct ezbus* bus, int who, const char* what, int old)
{
  if (EZBus_Comm(bus, who, what, 0) & (EZ_BUS_TIMEOUT | EZ_BUS_OOD))
    return old;

  return atoi(bus->buffer);
}

int EZBus_Poll(struct ezbus* bus)
{
  int i, result;
  int all_ok = 1;

  if (bus->chatter > 0) {
      bprintf(info, "%sPolling EZStepper Bus.", bus->name);
  }

  for (i = 0; i < EZ_BUS_NACT; ++i) {
    if ( !(bus->step[i].status & EZ_STEP_USED)   //skip if unused or okay
	|| (bus->step[i].status & EZ_STEP_OKAY) )
      continue;
    EZBus_Send(bus, i+1, "&");
    if ((result = EZBus_Recv(bus)) & (EZ_BUS_TIMEOUT | EZ_BUS_OOD)) {
      bprintf(warning, "%sNo response from %s, will repoll later.", 
	  bus->name, stepName(bus,i+1));
      bus->step[i].status &= ~EZ_STEP_OKAY;
      all_ok = 0;
    } else if (!strncmp(bus->buffer, "EZStepper AllMotion", 19)) {
      bprintf(info, "%sFound EZStepper device %s at address %c.\n", bus->name,
          stepName(bus,i+1), cWho(i+1));
      bus->step[i].status |= EZ_STEP_OKAY;
    } else if (!strncmp(bus->buffer, "EZHR17EN AllMotion", 18)) {
      bprintf(info, "%sFound type 17EN device %s at address %c.\n", bus->name,
          stepName(bus,i+1), cWho(i+1));
      bus->step[i].status |= EZ_STEP_OKAY;
    } else if (!strncmp(bus->buffer, "EZHR23 All Motion", 17)) {
      bprintf(info, "%sFound type 23 device %s at address %c.\n", bus->name, 
	  stepName(bus,i+1), cWho(i+1));
      bus->step[i].status |= EZ_STEP_OKAY;
    } else {
      bprintf(warning, "%sUnrecognised response from %s, will repoll later.\n",
	  bus->name, stepName(bus,i+1));
      bus->step[i].status &= ~EZ_STEP_OKAY;
      all_ok = 0;
    }

    /* TODO if this is needed, this about how to accomplish it
    if (stepper[i].status == 0 && i <= LAST_ACTUATOR) {
      InitialiseActuator(i);
      sleep(1);
    }
    */
  }

  return all_ok;
}

