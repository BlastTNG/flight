/* pcm: the Spider master control program
 *
 * This software is copyright (C) 2012 University of Toronto
 *
 * This file is part of pcm.
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
#include <limits.h>
#include <math.h>
#include "phytron.h"
#include "blast.h"


//allowed baud rates. Scanned during Poll(). First entry is default
#define N_ALLOWED_BAUDS 5
const unsigned int allowed_bauds[N_ALLOWED_BAUDS] = {B115200, B57600, B38400,
  B19200, B9600};
const char* allowed_bauds_s[N_ALLOWED_BAUDS] = {"115200", "57600", "38400",
  "19200", "9600"};


//terminating snprintf and strncpy, for convenience
//assumes 'str' is an array of length at least 'size'
static int  __attribute__((format(printf,3,4))) stprintf(char* str,
    size_t size, const char* fmt, ...) {
  va_list argptr;
  int ret;

  va_start(argptr, fmt);
  ret = vsnprintf(str, size, fmt, argptr);
  str[size-1] = '\0';
  va_end(argptr);

  return ret;
}

static char* strtcpy(char* dest, const char* src, size_t n)
{
  char* ret = strncpy(dest, src, n);
  dest[n-1] = '\0';
  return ret;
}

//set up serial port.
//Set bus->fd negative before call from Phytron_Init()
//Otherwise, can be used to change speed (baud rate) of already-opened port
static int phytron_setserial(struct phytron* bus, speed_t speed)
{
  int fd;
  struct termios term;
  static int err_flag = 0;

  if (bus->fd < 0) {  //open device if it is not yet open
    if ((fd = open(bus->tty, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
      if (bus->chatter >= PH_CHAT_ERR && err_flag == 0)
        berror(err, "%sUnable to open serial port (%s)", bus->name, bus->tty);
      err_flag = 1;
      return -1;
    }
  } else fd = bus->fd;


  if (tcgetattr(fd, &term)) {
    if (bus->chatter >= PH_CHAT_ERR && err_flag == 0)
      berror(err, "%sUnable to get serial device attributes", bus->name);
    err_flag = 1;
    return -1;
  }

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

  if(cfsetospeed(&term, speed)) {        /*  <======= SET THE SPEED HERE */
    if (bus->chatter >= PH_CHAT_ERR && err_flag == 0)
      berror(err, "%sError setting serial output speed", bus->name);
    err_flag = 1;
    return -1;
  }

  if(cfsetispeed(&term, speed)) {        /*  <======= SET THE SPEED HERE */
    if (bus->chatter >= PH_CHAT_ERR && err_flag == 0)
      berror(err, "%sError setting serial input speed", bus->name);
    err_flag = 1;
    return -1;
  }

  if( tcsetattr(fd, TCSANOW, &term) ) {
    if (bus->chatter >= PH_CHAT_ERR && err_flag == 0)
      berror(err, "%sUnable to set serial attributes", bus->name);
    err_flag = 1;
    return -1;
  }
  err_flag = 0;
  return fd;
}

int Phytron_Init(struct phytron* bus, const char* tty, const char* name,
    int chatter)
{
  int i;
  int retval = PH_ERR_OK;
  if (name[0] != '\0') stprintf(bus->name, PH_BUS_NAME_LEN, "%s: ", name);
  else bus->name[0] = '\0';
  bus->tty = tty;  //NB: Init argument should not be deallocated
  bus->seized = -1;
  bus->chatter = chatter;
  bus->fd = -1;
  if ( (bus->fd = phytron_setserial(bus, allowed_bauds[0])) < 0 )
    retval |= PH_ERR_TTY;
  Phytron_Recv_Flush(bus);
  for (i = 0; i < PH_BUS_NACT; ++i) {
    bus->stepper[i].status = 0;
    bus->stepper[i].name[0] = '\0';
    bus->stepper[i].addr = '\0';
    bus->stepper[i].axis = '\0';
    bus->stepper[i].baud = allowed_bauds[0];
    bus->stepper[i].usteps = 1;
    bus->stepper[i].gear_teeth = 1;
    bus->stepper[i].vel = 0;
    //bus->stepper[i].acc = 0;
    bus->stepper[i].ihold = 0;
    bus->stepper[i].imove = 0;
  }
  return retval;
}

int Phytron_Add(struct phytron* bus, int who, const char* addr,
    const char* name)
{
  if (who >= PH_BUS_NACT || who < 0) {
    if (bus->chatter >= PH_CHAT_ERR) 
      bprintf(err, "%sFailed to add stepper \'%d\'\n", bus->name, who);
    return PH_ERR_BAD_WHO;
  }

  strtcpy(bus->stepper[who].name, name, PH_BUS_NAME_LEN);
  if (bus->stepper[who].name[0] == '\0')  //assign name, if blank
    stprintf(bus->stepper[who].name, PH_BUS_NAME_LEN, "Stepper #%d", who);

  if (addr[0] >= '0' && addr[0] <= '9') bus->stepper[who].addr = addr[0];
  else return PH_ERR_PARAM;
  if (addr[1] == 'X' || addr[1] == 'Y') bus->stepper[who].axis = addr[1];
  else return PH_ERR_PARAM;

  //set default values for the stepper move paramters
  bus->stepper[who].gear_teeth = 464;
  bus->stepper[who].usteps = 4;
  bus->stepper[who].vel = 0.5;
  //bus->stepper[who].acc = ??;
  bus->stepper[who].ihold = 0.0;
  bus->stepper[who].imove = 0.8;

  bus->stepper[who].status |= PH_STEP_USED;

  return PH_ERR_OK;
}

int Phytron_Take(struct phytron* bus, int who)
{
  if (bus->seized != -1 && bus->seized != who)
    return PH_ERR_BUSY;

  if (!Phytron_IsUsable(bus, who))
    return PH_ERR_POLL;

  if (bus->seized != who) {
    if (bus->chatter >= PH_CHAT_SEIZE) 
      bprintf(info, "%sBus seized by %s.\n", bus->name, bus->stepper[who].name);
    bus->seized = who;
  }

  return PH_ERR_OK;
}

int Phytron_Release(struct phytron* bus, int who)
{
  if (bus->seized == who) {
    if (bus->chatter >= PH_CHAT_SEIZE)
      bprintf(info, "%sBus released by %s.\n", bus->name, bus->stepper[who].name);
    bus->seized = -1;
  }
  return PH_ERR_OK;
}

int Phytron_IsTaken(struct phytron* bus, int who)
{
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL;
  if (bus->seized == who) return PH_ERR_OK;
  return PH_ERR_BUSY;
}

//deals with serial port error accumulation
static void count_errors(struct phytron* bus, int retval, const char* f)
{
  if((retval & PH_ERR_MASK) > 0) {
    bus->err_count++;
    if(bus->chatter >= PH_CHAT_BUS)
      bprintf(err,"%s: Serial madness! err_count=%i", f, bus->err_count);
  } else {
    bus->err_count = 0;
    //if(bus->chatter >= PH_CHAT_BUS) bprintf(err,"%s: No serial error! Resetting error count to 0.", f);
  }
}

//fills string to with stringified hex values of the string from
static const char* HexDump(const unsigned char* from, char* to, int len)
{
  int i;
  if (from[0] == '\0') {
    to[0] = '\0';
    return to;
  }
  sprintf(to, "%02x", from[0]);
  for (i = 1; i < len; ++i)
    sprintf(to + i * 3 - 1, ".%02x", from[i]);
  return to;
}

#define STX (char)0x02
#define ETX (char)0x03
#define ACK (char)0x06
#define NAK (char)0x15

static int Phytron_DoSend(struct phytron* bus, int who, int useaxis,
    const char* what)
{
  size_t len = strnlen(what, PH_BUS_BUF_LEN) + 3 + ((useaxis)?1:0);
  char* buffer = malloc(len);
  //TODO should I use checksums on sends?
  char* hex_buffer;
  int i = 0, j=0;
  int retval = PH_ERR_OK;
  char addrstr[3] = {'\0', '\0', '\0'};

  buffer[i++] = STX;
  buffer[i++] = bus->stepper[who].addr;
  if (useaxis) buffer[i++] = bus->stepper[who].axis;
  while (what[j] != '\0' && i < len-1) buffer[i++] = what[j++];
  buffer[i++] = ETX;
  if (bus->chatter >= PH_CHAT_BUS) {
    hex_buffer = malloc(3*len+1);
    addrstr[0] = bus->stepper[who].addr;
    if (useaxis) addrstr[1] = bus->stepper[who].axis;
    bprintf(info, "%sRequest=%s (%s%s)", bus->name, 
        HexDump((unsigned char *)buffer, hex_buffer, len),
        addrstr, what);
    free(hex_buffer);
  }
  //update baud rate to that used by stepper 'who'
  if (phytron_setserial(bus, bus->stepper[who].baud) < 0) {
    if (bus->chatter >= PH_CHAT_ERR)
      berror(err, "%sError setting baud rate. File descriptor = %i",
          bus->name, bus->fd);
    retval |= PH_ERR_BAUD;
  }
  if (write(bus->fd, buffer, len) < 0) {
    if (bus->chatter >= PH_CHAT_ERR)
      berror(err, "%sError writing on bus. File descriptor = %i",
          bus->name, bus->fd);
    retval |= PH_ERR_TTY;
  }

  free(buffer);

  count_errors(bus, retval, "Phytron_Send");

  return (bus->error=retval);
}

int Phytron_Send(struct phytron *bus, int who, const char* what)
{
  return Phytron_DoSend(bus, who, 1, what);
}

int Phytron_NASend(struct phytron *bus, int who, const char* what)
{
  return Phytron_DoSend(bus, who, 0, what);
}

#define PH_BUS_RECV_ABORT 3000000 /* state for general parsing abort */
#define READ_TRIES  100

int Phytron_Recv(struct phytron* bus)
{
  int fd;
  fd_set rfds;
  struct timeval timeout = {.tv_sec = 0, .tv_usec = 500000};
  unsigned char byte;
  //unsigned char checksum = 0;
  char full_response[PH_BUS_BUF_LEN];
  char* ptr = bus->buffer;
  char* fullptr = full_response;
  char* hex_buffer;
  int retval = PH_ERR_OK;

  FD_ZERO(&rfds);
  FD_SET(bus->fd, &rfds);

  fd = select(bus->fd + 1, &rfds, NULL, NULL, &timeout);

  if (fd == -1) {
    //on error, don't return immediately, allow response to be terminated
    if (bus->chatter >= PH_CHAT_ERR)
      berror(err, "%sError waiting for input on bus",bus->name);
    retval |= PH_ERR_TTY;
  } else if (!fd) { /* Timeout */
    retval |= PH_ERR_TIMEOUT;
  } else {
    int state = 1;
    int had_errors = 0;
    int read_tries = READ_TRIES;
    int len;

    for(;;) {
      len = read(bus->fd, &byte, 1);
      if (len <= 0) {
        if (state == 4 || state == PH_BUS_RECV_ABORT)
          break;
        if (errno == EAGAIN && read_tries > 0) {
          read_tries--;
          usleep(1000);
          continue;
        } else {
          if (bus->chatter >= PH_CHAT_ERR)
            berror(warning, "%sUnexpected out-of-data reading bus (%i)", 
                bus->name, state);
          return PH_ERR_OOD;
        }
      }
      *(fullptr++) = byte;
      //checksum ^= byte;

      /* The following involves a number of semi-hidden fallthroughs which
       * attempt to recover malformed strings */
      switch (state) {
        case 1: /* start byte */
          state++;
          if (byte != STX) {
            had_errors++;
            if (bus->chatter >= PH_CHAT_ERR)
              bprintf(warning, "%sSTX not found in response", 
                  bus->name);
          } else break;
        case 2: /* acknowledge */
          state++;
          if (byte != ACK && byte != NAK) {
            had_errors++;
            if (bus->chatter >= PH_CHAT_ERR)
              bprintf(warning, "%sNo ACK in response", bus->name);
          }
          if (had_errors > 1) {
            if (bus->chatter >= PH_CHAT_ERR)
              bprintf(err, "%sToo many errors parsing response, aborting.",
                  bus->name);
            retval|= PH_ERR_RESPONSE;
            state = PH_BUS_RECV_ABORT;
          }
          if (byte == NAK) retval |= PH_ERR_NAK;
          break;
        case 3: /* response */
          if (byte == ETX)
            state++;
          else
            *(ptr++) = byte;
          break;
        case 4: /* End of string check */
          if (bus->chatter >= PH_CHAT_ERR)
            bprintf(err, "%sMalformed footer in response string, aborting.",
                bus->name);
          retval|= PH_ERR_RESPONSE;
          state = PH_BUS_RECV_ABORT;
        case PH_BUS_RECV_ABORT: /* General abort: flush input */
          break;
      }
    }
  }

  *ptr = '\0';  //terminate response string
  *fullptr = '\0';

  if (bus->chatter >= PH_CHAT_BUS) {
    size_t len;
    len = strnlen(full_response, PH_BUS_BUF_LEN);
    hex_buffer = malloc(3*len+1);
    bprintf(info, "%sResponse=%s (%s%s)\n", bus->name, 
        HexDump((unsigned char *)full_response, hex_buffer, len), 
        (full_response[1]==NAK)?"<NAK>":(full_response[1]==ACK)?"<ACK>":"",
        bus->buffer);
    free(hex_buffer);
  }

  count_errors(bus, retval, "Phytron_Recv");

  return (bus->error=retval);
}

int Phytron_Recv_Flush(struct phytron* bus)
{
  ssize_t len;
  char byte;
  int read_tries = READ_TRIES;

  while (1) {
    len = read(bus->fd, &byte, 1);
    if (len < 0) {
      if (errno == EAGAIN) {
        if (read_tries <= 0) return PH_ERR_OK;  //out of data. done
        read_tries--;
        usleep(1000);
      } else return PH_ERR_TTY;                 //serial error. abort
    } else if (len == 0) return PH_ERR_OK;      //out of data. done
  }
}

static int Phytron_DoCommRetry(struct phytron* bus, int who, int useaxis,
    const char* what, int retries)
{
  int ok;
  int retval = PH_ERR_OK;
  int retry_count = 0;

  do {
    ok = 1;
    if (useaxis) retval = Phytron_Send(bus, who, what);
    else retval = Phytron_NASend(bus, who, what);
    if (retval != PH_ERR_OK) {
      if (bus->chatter >= PH_CHAT_ERR)
        berror(warning, "%sFailed to send command\n", bus->name);
      break; //return retval;
    }

    retval = Phytron_Recv(bus);
    if (retval & PH_ERR_NAK) {        //retry on NAK
      if (bus->chatter >= PH_CHAT_ERR && retry_count == 0) {
        bprintf(warning, "%s%s: command error (NAK) on %s.\n", 
            bus->name, bus->stepper[who].name, what);
      }
      ok = 0;
      retry_count++;
      sleep(1);
    } else if (retval != PH_ERR_OK) { //communication error, give up
      if (bus->chatter >= PH_CHAT_ERR) 
        bprintf(warning, "%sError waiting for response from %s (%s)\n.", 
            bus->name, bus->stepper[who].name, what);

      Phytron_ForceRepoll(bus, who);
      break; //return retval;
    }
  } while (!ok && retry_count < retries);


  count_errors(bus, retval, "Phytron_Comm");

  return (bus->error=retval);
}

int Phytron_Comm(struct phytron* bus, int who, const char* what)
{
  return Phytron_DoCommRetry(bus, who, 1, what, PH_BUS_COMM_RETRIES);
}

int Phytron_NAComm(struct phytron* bus, int who, const char* what)
{
  return Phytron_DoCommRetry(bus, who, 0, what, PH_BUS_COMM_RETRIES);
}

int __attribute__((format(printf,3,4))) Phytron_CommVarg(struct phytron* bus,
    int who, const char* fmt, ...)
{
  va_list argptr;
  char buf[PH_BUS_BUF_LEN];

  va_start(argptr, fmt);
  vsnprintf(buf, PH_BUS_BUF_LEN, fmt, argptr);
  buf[PH_BUS_BUF_LEN-1] = '\0';
  va_end(argptr);

  return Phytron_Comm(bus, who, buf);
}

int __attribute__((format(printf,3,4))) Phytron_NACommVarg(struct phytron* bus,
    int who, const char* fmt, ...)
{
  va_list argptr;
  char buf[PH_BUS_BUF_LEN];

  va_start(argptr, fmt);
  vsnprintf(buf, PH_BUS_BUF_LEN, fmt, argptr);
  buf[PH_BUS_BUF_LEN-1] = '\0';
  va_end(argptr);

  return Phytron_NAComm(bus, who, buf);
}

int Phytron_ReadInt(struct phytron* bus, int who, const char* what, int* val)
{
  //TODO update ReadInt
  int retval;
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL;
  else {
    retval = Phytron_Comm(bus, who, what);
    if ( !(retval & (PH_ERR_TIMEOUT | PH_ERR_OOD | PH_ERR_TTY)) )
      *val = atoi(bus->buffer);

    return retval;
  }
}

int Phytron_ForceRepoll(struct phytron* bus, int who)
{
  bus->stepper[who].status &= ~(PH_STEP_OK | PH_STEP_INIT);
  return PH_ERR_OK;
}

//dummy function to use to perform no initialization when using Phytron_Poll
static int noInit(struct phytron* bus, int who)
{
  return 1;
}

int Phytron_Poll(struct phytron* bus)
{
  return Phytron_PollInit(bus, &noInit);
}

int Phytron_PollInit(struct phytron* bus, int (*phinit)(struct phytron*,int))
{
  int i, b, result;
  int retval = PH_ERR_OK;
  int bus_chatter;    //the normal value of bus->chatter

  bus_chatter = bus->chatter;
  bus->chatter = PH_CHAT_NONE;

  if (bus_chatter >= PH_CHAT_ACT) {
    bprintf(info, "%sPolling Phytron Bus.", bus->name);
  }

  for (i=0; i<PH_BUS_NACT; ++i) {
    if ( !(bus->stepper[i].status & PH_STEP_USED)   //skip if unused
        || ((bus->stepper[i].status & PH_STEP_OK)
          && (bus->stepper[i].status & PH_STEP_INIT)) ) //skip if ok, init
      continue;

    for (b=0; b<N_ALLOWED_BAUDS; b++) {
      bus->stepper[i].baud = allowed_bauds[b];
      Phytron_NASend(bus, i, "IVR");
      if ((result = Phytron_Recv(bus)) & (PH_ERR_TIMEOUT | PH_ERR_OOD)) {
        if (b == N_ALLOWED_BAUDS-1) {
          if (bus_chatter >= PH_CHAT_ACT)
            bprintf(warning, "%sNo response from %s, will repoll later.", 
                bus->name, bus->stepper[i].name);
          bus->stepper[i].baud = allowed_bauds[0];  //reset to default
          retval |= PH_ERR_POLL;
        }
        bus->stepper[i].status &= ~PH_STEP_OK;
        retval |= result;	  //include in retval results from Recv
      } else if (!strncmp(bus->buffer, "MCC Minilog V", 13)) {
        if (bus_chatter >= PH_CHAT_ACT)
          bprintf(info, "%sFound Phytron MCC V%.2f device \"%s\","
              "address \"%c%c\" baud rate %s (%d)\n",
              bus->name, atof(bus->buffer+13), bus->stepper[i].name,
              bus->stepper[i].addr, bus->stepper[i].axis, allowed_bauds_s[b],
              allowed_bauds[b]);
        bus->stepper[i].status |= PH_STEP_OK;
        retval &= ~(PH_ERR_TIMEOUT | PH_ERR_OOD); //clear previous error bits
        break;
      } else {
        if (b == N_ALLOWED_BAUDS-1) {
          if (bus_chatter >= PH_CHAT_ACT)
            bprintf(warning, "%sUnrecognised response from %s, "
                "will repoll later.\n", bus->name, bus->stepper[i].name);
          bus->stepper[i].baud = allowed_bauds[0];  //reset to default
          retval |= PH_ERR_POLL;
        }
        bus->stepper[i].status &= ~PH_STEP_OK;
        retval |= result;	  //include in retval results from Recv
      }
    }

    if ( bus->stepper[i].status & PH_STEP_OK &&
        !(bus->stepper[i].status & PH_STEP_INIT) ) {
      if (phinit(bus,i)) bus->stepper[i].status |= PH_STEP_INIT;
      else bus->stepper[i].status &= ~PH_STEP_INIT;
      usleep(1000); //may prevent many-init...what does this mean?
    }

  }

  bus->chatter = bus_chatter; //restore chatter level
  return retval;
}

int Phytron_IsUsable(struct phytron* bus, int who)
{
  if ( (bus->stepper[who].status &  (PH_STEP_USED | PH_STEP_OK | PH_STEP_INIT))
      != (PH_STEP_USED | PH_STEP_OK | PH_STEP_INIT) ) return 0;
  else return 1;
}

#if 0
//TODO update readiness check command, or remove function
int Phytron_IsBusy(struct phytron* bus, int who)
{
  int retval;
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL | PH_READY;

  retval = Phytron_DoCommRetry(bus, who, 1, "Q", 0);
  if ( (retval & ~PH_READY) != PH_ERR_OK ) {
    //on error return code with busy bit set
    return retval | PH_READY;
  }

  //otherwise just flip busy bit which usually means the opposite
  return retval ^ PH_READY;
}
#endif

/*******************************************************************************
 * Simple motion commands                                                      *
 ******************************************************************************/

int Phytron_SetIHold(struct phytron* bus, int who, double current)
{
  if (current > 1.2 || current < 0.0) {
    //bprintf(err, "%sInvalid current command %g", bus->name, current);
    return PH_ERR_PARAM;
  }
  bus->stepper[who].ihold = current;
  return PH_ERR_OK;
}

int Phytron_SetIMove(struct phytron* bus, int who, double current)
{
  if (current > 1.2 || current < 0.0) {
    //bprintf(err, "%sInvalid current command %g", bus->name, current);
    return PH_ERR_PARAM;
  }
  bus->stepper[who].imove = current;
  return PH_ERR_OK;
}

int Phytron_SetVel(struct phytron* bus, int who, double vel)
{
  if (vel < 0.0) {
    //bprintf(err, "%sInvalid velocity command %g", bus->name, vel);
    return PH_ERR_PARAM;
  }
  bus->stepper[who].vel = vel;
  return PH_ERR_OK;
}

int Phytron_SetGear(struct phytron* bus, int who, int teeth)
{
  bus->stepper[who].gear_teeth = teeth;
  return PH_ERR_OK;
}

#if 0
int Phytron_SetAccel(struct phytron* bus, int who, int acc)
{
  bus->stepper[who].acc = acc;
  return PH_ERR_OK;
}
#endif

inline int Phytron_D2S(struct phytron* bus, int who, double degrees)
{
  return (int)round(( PH_STEPS_PER_REV * (double)bus->stepper[who].usteps
      * (double)bus->stepper[who].gear_teeth * degrees / 360.));
}

inline double Phytron_S2D(struct phytron* bus, int who, int steps)
{
  return 360. * (double)steps / PH_STEPS_PER_REV
    / (double)bus->stepper[who].usteps / (double)bus->stepper[who].gear_teeth;
}

int Phytron_SendParams(struct phytron* bus, int who)
{
  int retval;

  //microstep resolution
  retval = Phytron_CommVarg(bus, who, "P45S%i", bus->stepper[who].usteps);
  if (retval != PH_ERR_OK) return retval;
  //hold current
  retval = Phytron_CommVarg(bus, who, "P40S%i",
      (int)(bus->stepper[who].ihold*10.+0.5));
  if (retval != PH_ERR_OK) return retval;
  //move (and boost) current
  retval = Phytron_CommVarg(bus, who, "P41S%i",
      (int)(bus->stepper[who].imove*10.+0.5));
  if (retval != PH_ERR_OK) return retval;
  retval = Phytron_CommVarg(bus, who, "P42S%i",
      (int)(bus->stepper[who].imove*10.+0.5));
  if (retval != PH_ERR_OK) return retval;
  //velocity
  retval = Phytron_CommVarg(bus, who, "P14S%i",
      Phytron_D2S(bus, who, bus->stepper[who].vel));
  return retval;
}

int Phytron_Stop(struct phytron* bus, int who)
{
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL;
  else return Phytron_Comm(bus, who, "S");
}

int Phytron_Stop_All(struct phytron* bus)
{
  //TODO check into multi-stepper version of stop command
  int i;
  int ret = PH_ERR_OK;
  for (i=0; i<PH_BUS_NACT; i++)
    if (bus->stepper[i].status & PH_STEP_USED) ret |= Phytron_Send(bus, i, "S");
  return ret | Phytron_Recv_Flush(bus);
}

int Phytron_Move(struct phytron* bus, int who, double delta)
{
  int ret;
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL;
  else {
    ret = Phytron_SendParams(bus, who);
    if (ret != PH_ERR_OK) return ret;
    return Phytron_CommVarg(bus, who, "%+i", Phytron_D2S(bus, who, delta));
  }
}

int Phytron_MoveVel(struct phytron* bus, int who, double delta, double vel)
{
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL;
  else {
    Phytron_SetVel(bus, who, vel);
    return Phytron_Move(bus, who, delta);
  }
}

int Phytron_ContinuousVel(struct phytron* bus, int who, double vel)
{
  int ret;
  char dir = (vel >= 0) ? '+' : '-';
  if (!Phytron_IsUsable(bus, who)) return PH_ERR_POLL;
  else {
    Phytron_SetVel(bus, who, fabs(vel));
    ret = Phytron_SendParams(bus, who);
    if (ret != PH_ERR_OK) return ret;
    return Phytron_CommVarg(bus, who, "L%c", dir);
  }
}

