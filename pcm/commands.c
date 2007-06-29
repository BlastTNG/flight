/* mcp: the Spider master control program
 *
 * commands.c: functions for listening to and processing commands
 *
 * This software is copyright (C) 2002-2007 University of Toronto
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
#include "channels.h"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
#define LOCK_OFFSET (3.34)

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

/* based on isc_protocol.h */
#define ISC_SHUTDOWN_NONE     0
#define ISC_SHUTDOWN_HALT     1
#define ISC_SHUTDOWN_REBOOT   2
#define ISC_SHUTDOWN_CAMCYCLE 3

#define ISC_TRIGGER_INT  0
#define ISC_TRIGGER_EDGE 1
#define ISC_TRIGGER_POS  2
#define ISC_TRIGGER_NEG  3

void NormalizeAngle(double*);  //pointing.c

static const char *UnknownCommand = "Unknown Command";

extern short InCharge; /* tx.c */

pthread_mutex_t mutex; //init'd in mcp.c

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

/** Write the Previous Status: called whenever anything changes */
static void WritePrevStatus()
{
  int fp, n;

  /** write the default file */
  fp = open("/tmp/mcp.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "Commands: mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "Commands: mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "Commands: mcp.prev_status close()");
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

  write(tty_fd, buffer, 3);
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
  if (!scheduled)
    bprintf(info, "Commands: Single command: %d (%s)\n", command,
        SName(command));

  /* Update CommandData structure with new info */

  switch (command) {
    case test:
      bputs(info, "This has beeen a succesful single command test");
      break;
    default:
      bputs(warning, "Commands: ***Invalid Single Word Command***\n");
      return; /* invalid command - no write or update */
  }

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

  /* Update CommandData struct with new info
   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]
   */

  switch(command) {
      /***************************************/
      /********** Pointing Motor Gains *******/
    case table_gain:  /* rotary table gains */
      CommandData.table_gain.P = ivalues[0];
      CommandData.table_gain.I = ivalues[1];
      break;
    default:
      bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
      return; /* invalid command - don't update */
  }

  WritePrevStatus();
}

#ifndef USE_FIFO_CMD
static void GPSPosition (unsigned char *indata)
{
  /* Send new information to CommandData */

  SIPData.GPSpos.lon = -ParseGPS(indata); /* sip sends east lon */
  SIPData.GPSpos.lat = ParseGPS(indata + 4);
  SIPData.GPSpos.alt = ParseGPS(indata + 8);
  SIPData.GPSstatus1 = *(indata + 12);
  SIPData.GPSstatus2 = *(indata + 13);

  WritePrevStatus();
}
#endif

const char* CommandName(int is_multi, int command)
{
  return (is_multi) ? MName(command) : SName(command);
}

#ifndef USE_FIFO_CMD
static void GPSTime (unsigned char *indata)
{
  float GPStime, offset;
  int CPUtime, GPSweek;

  /* Send new information to CommandData */

  GPStime = ParseGPS(indata);
  GPSweek = (unsigned short)(*(indata + 4));
  offset = ParseGPS(indata + 6);
  CPUtime = ParseGPS(indata + 10);

  SIPData.GPStime.UTC = (int)(SEC_IN_WEEK * (GPSweek+1024) + GPStime - offset) +
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

#if 0
/* Send TDRSS Low Rate Packet */

static void SendDownData(char tty_fd)
{
  unsigned char buffer[SLOWDL_LEN], data[3 + SLOWDL_LEN + 1];
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
  memcpy(buffer + 3, data, SLOWDL_LEN);
  buffer[3 + SLOWDL_LEN] = SLOWDL_ETX;

  write(tty_fd, buffer, 3 + SLOWDL_LEN + 1);
#if 0
  for (i=0; i<3 + SLOWDL_LEN + 1; i++) {
    bprintf(info, "%d %2x", i, buffer[i]);
  }
#endif
}
#endif //if 0

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
  char command[100];
  char pbuf[30];
  int fifo;

  int mcommand = -1;
  int mcommand_count = 0;
  char *mcommand_data[DATA_Q_SIZE];

  int i;
  for (i = 0; i < DATA_Q_SIZE; ++i) {
    mcommand_data[i] = NULL;
  }

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];

  int index, pindex = 0;

  bputs(startup, "Commands: WatchFIFO startup\n");

  if ((fifo = open("/tmp/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
    berror(tfatal, "Commands: Unable to open FIFO");

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    bprintf(info, "Commands: Command received: %s\n", command);
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
    bprintf(info, "Commands: %i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SCommand(command);
      SingleCommand(mcommand, 0);
      mcommand = -1;
    } else {
      mcommand = MCommand(command);
      bputs(info, "Commands:  Multi word command received\n");
      SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues, ivalues,
          svalues);
      MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
      mcommand = -1;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);

  }
}

#else
void WatchPort (void* parameter)
{
  const char *COMM[] = {"/dev/ttyS0", "/dev/ttyS4"};

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

  bprintf(startup, "Commands: WatchPort(%i) startup\n", port);

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
        bprintf(info, "Commands: COMM%i: Request SIP Position\n", port + 1);
#endif
        pthread_mutex_unlock(&mutex);
      } else if (timer == 1700) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_TIME, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Commands: COMM%i: Request SIP Time\n", port + 1);
#endif
        pthread_mutex_unlock(&mutex);	
      } else if (timer > 2500) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_ALTITUDE, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Commands: COMM%i: Request SIP Altitude\n", port + 1);
#endif
        pthread_mutex_unlock(&mutex);
        timer = 0;
      }
      usleep(10000); /* sleep for 10ms */
    }

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
          bprintf(info, "Commands: COMM%i: Data request\n", port + 1);
#endif
        } else if (buf == 0x14) { /* Command */
          readstage = 2;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: Command\n", port + 1);
#endif
        } else if (buf == 0x10) { /* GPS Position */
          readstage = 4;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: GPS Position\n", port + 1);
#endif
        } else if (buf == 0x11) { /* GPS Time */
          readstage = 5;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: GPS Time\n", port + 1);
#endif
        } else if (buf == 0x12) { /* MKS Altitude */
          readstage = 6;
#ifdef SIP_CHATTER
          bprintf(info, "Commands: COMM%i: MKS Altitude\n", port + 1);
#endif
        } else {
          bprintf(warning, "Commands: COMM%i: Bad packet received: "
              "Unrecognised Packet Type: %02X\n", port + 1, buf);
          readstage = 0;
        }
        break;
      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf == 0x02)
            bytecount = 1;
          else {
            readstage = 0;
            bprintf(warning, "Commands: COMM%i: Bad command packet: "
                "Unsupported Length: %02X\n", port + 1, buf);
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
              bprintf(info, "Commands: COMM%i:  Single command received\n",
                  port + 1);
              SingleCommand(indata[0], 0);
              mcommand = -1;
            } else if ((indata[1] & 0xE0) == 0x80) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(MIndex(mcommand));
              bprintf(info,
                  "Commands: COMM%i:  Multi word command %d (%s) started\n",
                  port + 1, mcommand, MName(mcommand));

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;
            } else if (((indata[1] & 0x80) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              bprintf(info, "Commands: COMM%i:  Multi word command "
                  "continues...\n", port + 1);
              mcommand_count++;
            } else if (((indata[1] & 0xE0) == 0xC0) && (mcommand == indata[0])
                && ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              bprintf(info, "Commands: COMM%i:  Multi word command ends \n",
                  port + 1);
              SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues,
                  ivalues, svalues);
              MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              bprintf(warning, "Commands: COMM%i: Command packet discarded: "
                  "Bad Encoding: %04X\n", port + 1, indata[1]);
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
          bprintf(warning,
              "Commands: COMM%i: Bad encoding: Bad packet terminator: %02X\n",
              port + 1, buf);
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
            bprintf(warning, "Commands: COMM%i: Bad encoding in GPS Position: "
                "Bad packet terminator: %02X\n", port + 1, buf);
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
            bprintf(warning, "Commands: COMM%i: Bad encoding in GPS Time: "
                "Bad packet terminator: %02X\n", port + 1, buf);
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
            bprintf(warning, "Commands: COMM%i: Bad encoding in MKS Altitude: "
                "Bad packet terminator: %02X\n", port + 1, buf);
          }
        }
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
  int fp, n_read = 0, junk, extra = 0;

  if ((fp = open("/tmp/mcp.prev_status", O_RDONLY)) < 0) {
    berror(err, "Commands: Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      berror(err, "Commands: prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      berror(err, "Commands: extra prev_status read()");
    if (close(fp) < 0)
      berror(err, "Commands: prev_status close()");
  }

  /** stuff here overrides prev_status **/

  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    bprintf(warning, "Commands: prev_status: Wanted %i bytes but got %i.\n",
        sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    bputs(warning, "Commands: prev_status: Extra bytes found.\n");
  else
    return;

  bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

  /** prev_status overrides this stuff **/
  CommandData.table_gain.I = 302;
  CommandData.table_gain.P = 834;

  WritePrevStatus();
}
