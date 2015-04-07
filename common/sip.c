/* mcp: the BLAST master control program
 *
 * commands.c: functions for listening to and processing commands
 *
 * This software is copyright (C) 2002-2010 University of Toronto
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

/* Define this symbol to have mcp log all SIP traffic */
#undef SIP_CHATTER
#undef VERBOSE_SIP_CHATTER

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
#include <limits.h>

#include <netcmd.h>
#include "sip.h"
#include "blast.h"
#include "command_common.h"
#include "command_struct.h"
#include "mcp.h"
#include "sys.h"

#define REQ_POSITION    0x50
#define REQ_TIME        0x51
#define REQ_ALTITUDE    0x52
#define SLOWDL_SYNC     0x53

#define SLOWDL_LEN          255

#define SLOWDL_DLE          0x10
#define SLOWDL_ETX          0x03

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

#define EXT_SLOT   1
#define EXT_ICHUNK 2
#define EXT_NCHUNK 3
#define EXT_NSCHED 4
#define EXT_ROUTE  5
  
#define MAXLIB 1024

#define MAX_RTIME 65536.0
#define MAX_DAYS 21.0

void nameThread(const char*);  /* mcp.c */
void fillDLData(unsigned char *b, int len); /* slowdl.c */

extern pthread_mutex_t mutex;

//TODO:lst0str is in the scheduler
 char lst0str[82];

void SingleCommand (enum singleCommand command, int scheduled); // commands.c
void MultiCommand(enum multiCommand command, double *rvalues,
    int *ivalues, char svalues[][CMD_STRING_LEN], int scheduled); // commands.c
    


#ifdef USE_SIP_CMD
int sip_setserial(const char *input_tty)
{
  int fd;
  struct termios term;

  if ((fd = open(input_tty, O_RDWR)) < 0)
    berror(tfatal, "Unable to open serial port");

  if (tcgetattr(fd, &term))
    berror(tfatal, "Unable to get serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if(cfsetospeed(&term, B1200))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "Error setting serial output speed");

  if(cfsetispeed(&term, B1200))          /*  <======= SET THE SPEED HERE */
    berror(tfatal, "Error setting serial input speed");

  if( tcsetattr(fd, TCSANOW, &term) )
    berror(tfatal, "Unable to set serial attributes");

  return fd;
}
#endif    // USE_SIP_CMD

#ifdef USE_SIP_CMD
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
  
#ifdef VERBOSE_SIP_CHATTER
  bprintf(info,"sending to SIP %02x %02x %02x\n",
      buffer[0],buffer[1],buffer[2]);
#endif

  if (write(tty_fd, buffer, 3) < 0)
    berror(warning, "error sending SIP request\n");
}
#endif    // USE_SIP_CMD



enum singleCommand SCommand(char *cmd) //---------------------
{
  int i;

  for (i = 0; i < N_SCOMMANDS; i++) {
    if (strcmp(scommands[i].name, cmd) == 0)
      return scommands[i].command;
  }

  return -1;
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

static void SetParametersFifo(enum multiCommand command, unsigned short *dataq,
    double* rvalues, int* ivalues, char svalues[][CMD_STRING_LEN])
{
  int i, dataqind;
  char type;
  int index = MIndex(command);

  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  bprintf(info, "Multiword Command: %s (%d)\n", MName(command), command);

  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'l')  /* 30 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "param%02i: long   : %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "param%02i: float  : %f\n", i, rvalues[i]);
    } else if (type == 'd') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "param%02i: double : %f\n", i, rvalues[i]);
    } else if (type == 's') { /* string */
      strncpy(svalues[i], dataqc[dataqind++], CMD_STRING_LEN - 1);
      svalues[i][CMD_STRING_LEN - 1] = 0;
      bprintf(info, "param%02i: string: %s\n", i, svalues[i]);
    } else
      bprintf(err,
          "Unknown parameter type ('%c') param%02i: ignored", type,
          i);
  }


}

static void SetParameters(enum multiCommand command, unsigned short *dataq,
    double* rvalues, int* ivalues, char svalues[][CMD_STRING_LEN])
{
  int i, dataqind;
  char type;
  int index = MIndex(command);
  double min;

  /* compute renormalised values */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    min = mcommands[index].params[i].min;
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 16 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++] + mcommands[index].params[i].min;
      bprintf(info, "param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'l')  /* 32 bit unsigned integer */ {
      ivalues[i] = dataq[dataqind++] + mcommands[index].params[i].min;
      ivalues[i] += (dataq[dataqind++] << 16);
      bprintf(info, "param%02i: long   : %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 16 bit floating point */ {
      rvalues[i] = (float)dataq[dataqind++] * (mcommands[index].params[i].max
          - min) / USHRT_MAX + min;
      bprintf(info, "param%02i: float  : %f\n", i, rvalues[i]);
    } else if (type == 'd') { /* 32 bit floating point */
      rvalues[i] = (float)((unsigned)dataq[dataqind++] << 16); /* upper 16 bits */
      rvalues[i] += (float)dataq[dataqind++];             /* lower 16 bits */
      rvalues[i] = rvalues[i] * (mcommands[index].params[i].max - min) /
        UINT_MAX + min;
      bprintf(info, "param%02i: double : %f\n", i, rvalues[i]);
    } else if (type == 's') { /* string of 7-bit characters */
      int j;
      for (j = 0; j < mcommands[index].params[i].max; ++j)
        svalues[i][j] = ((j % 2) ? dataq[dataqind++] : dataq[dataqind] >> 8)
          & 0x7f;
      bprintf(info, "param%02i: string: %s\n", i, svalues[i]);
    } else
      bprintf(err,
          "Unknown parameter type ('%c') param%02i: ignored", type,
          i);
  }
}

#ifdef USE_SIP_CMD
static void GPSPosition (unsigned char *indata)
{
  double lat;
 
  SIPData.GPSstatus1 = *(indata + 12);
  SIPData.GPSstatus2 = *(indata + 13);

  lat = ParseGPS(indata + 4);
  if (fabs(lat)>20) {

    SIPData.GPSpos.lat = lat;
    SIPData.GPSpos.lon = -ParseGPS(indata); /* sip sends east lon */
    /* end of hack */
    
    SIPData.GPSpos.alt = ParseGPS(indata + 8);
    
    WritePrevStatus();
  }
}
#endif    // USE_SIP_CMD

void ScheduledCommand(struct ScheduleEvent *event)
{
  if (event->is_multi) {
    int i;
    int index = MIndex(event->command);

    bprintf(info, "Executing Scheduled Command: %s (%i)\n",
        MName(event->command), event->command);
    for (i = 0; i < mcommands[index].numparams; ++i) {
      int type = mcommands[index].params[i].type;
      if (type == 'i') /* 15 bit unsigned integer */
        bprintf(info, "param%02i: integer: %i\n", i,
            event->ivalues[i]);
      else if (type == 'l') /* 30 bit unsigned integer */
        bprintf(info, "param%02i: long   : %i\n", i,
            event->ivalues[i]);
      else if (type == 'f') /* 15 bit floating point */
        bprintf(info, "param%02i: float  : %f\n", i,
            event->rvalues[i]);
      else if (type == 'd') /* 30 bit floating point */
        bprintf(info, "param%02i: double : %f\n", i,
            event->rvalues[i]);
      else
        bprintf(err,
            "Unknown parameter type ('%c') param%02i: ignored", type,
            i);
    }
    MultiCommand(event->command, event->rvalues, event->ivalues, event->svalues,
        1);

  } else {
    bprintf(info, "Executing Scheduled Command: %s (%i)\n",
        SName(event->command), event->command);
    SingleCommand(event->command, 1);
  }
}

#ifdef USE_SIP_CMD
static void GPSTime (unsigned char *indata)
{
  float GPStime, offset;
  int CPUtime, GPSweek;

  /* Send new information to CommandData */

  GPStime = ParseGPS(indata);
  GPSweek = *((unsigned short*)(indata + 4));
  offset = ParseGPS(indata + 6);
  CPUtime = ParseGPS(indata + 10);

  SIPData.GPStime.UTC = (int)(SEC_IN_WEEK * (GPSweek) + GPStime - offset) +
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

/* Send TDRSS Low Rate Packet */

static void SendDownData(char tty_fd)
{
  unsigned char buffer[3 + SLOWDL_LEN + 1];

//  buffer[0] = SLOWDL_DLE;
//  buffer[1] = SLOWDL_SYNC;
//  buffer[2] = SLOWDL_LEN;
//  fillDLData(buffer+3, SLOWDL_LEN);
//
//  buffer[3 + SLOWDL_LEN] = SLOWDL_ETX;
//  if (write(tty_fd, buffer, 3 + SLOWDL_LEN + 1) < 0) {
//    berror(warning, "Error writing to SlowDL\n");
//  }
}

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
#endif    // USE_SIP_CMD


#ifdef USE_FIFO_CMD
void WatchFIFO (void* void_other_ip)
{
  unsigned char buf[1];
  char command[100];
  char pbuf[30];
  int fifo;

  int mcommand = -1;
  int mcommand_count = 0;
  char *mcommand_data[DATA_Q_SIZE];

  int i;
  int send_to_other;
  char other_ip[100] = "@localhost";

  if (void_other_ip)
    sprintf(other_ip, "@%s", (char*)void_other_ip);

  nameThread("FIFO");
  for (i = 0; i < DATA_Q_SIZE; ++i) {
    mcommand_data[i] = NULL;
  }

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];

  int index, pindex = 0;

  //bputs(startup, "WatchFIFO startup\n");

  if ((fifo = open("/data/etc/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
    berror(tfatal, "Unable to open FIFO");

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    //bprintf(info, "Command received: %s\n", command);
    
#ifndef TEST_RUN
    if (void_other_ip != NULL) {
      // if the command has an _ prepended, then it came from the other computer
      // and should not be sent back.
      if (command[0] == '_') {
        for (i=1; i<index; i++) {
          command[i-1] = command[i];
        }
        send_to_other = 0;
      } else {
        send_to_other = 1;
      }
      
      if (send_to_other) {
        char arg_command[100] = "_";
        char *ptr = arg_command;
        char *argv[16] = { "/usr/local/bin/spidercmd", other_ip, arg_command };
        int argc = 3;
        strcpy(arg_command + 1, command);
        for (ptr = arg_command; *ptr; ++ptr) {
          if (*ptr == ' ') {
            *ptr = 0;
            if (*(ptr + 1) != ' ' && *(ptr + 1))
              argv[argc++] = ptr + 1;
          }
        }
        argv[argc] = 0;

        exec_and_wait(err, none, argv[0], argv, 100, 1, NULL);
      }    
    }
#endif

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
    //bprintf(info, "%i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SCommand(command);
      SingleCommand(mcommand, 0);
      mcommand = -1;
    } else {
      mcommand = MCommand(command);
      //bputs(info, "Multi word command received\n");
      if (mcommand_count == mcommands[MIndex(mcommand)].numparams) {
        SetParametersFifo(mcommand, (unsigned short*)mcommand_data, rvalues,
            ivalues, svalues);
        MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
      } else {
        bputs(warning, "Ignoring mal-formed command!\n");
      }
      mcommand = -1;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);

  }
}
#endif  // USE_FIFO_CMD

#ifdef USE_SIP_CMD

struct LibraryStruct  {
  int n;
  int entry[MAXLIB];
  char cmd[MAXLIB][64];
  char params[MAXLIB][256];
};

struct LibraryStruct library;

void OpenLibrary() {
  FILE *fp;
  char instr[1024];
  int nf;
  int i;

  fp = fopen("/data/etc/blast/sched.library", "r");
  if (fp == NULL) {
    berror(fatal, "Could not open schedule file library.");
    exit(0);
  }
  
  i=0;
  while (fgets(instr, 256, fp) != NULL) {
    memset(library.params[i], 0, 256);
    nf = sscanf(instr, "%d %s %254c", library.entry+i, library.cmd[i], library.params[i]);
    if (nf==2) library.params[i][0] = '\n';
    if (nf>=2) i++;
  }
  library.n = i;

}


void ProcessUplinkSched(unsigned char *extdat) {
  static unsigned char slot = 0xff;
  static unsigned short sched[32][64][2];
  static unsigned long chunks_received = 0;
  static unsigned char nchunk = 0;
  static unsigned char nsched[32];
  
  unsigned char slot_in, i_chunk, nchunk_in;
  unsigned short *extdat_ui;
  unsigned short entry;
  unsigned short itime;
  double day, hour;
  
  int i, i_samp;
  
  slot_in = extdat[EXT_SLOT];
  i_chunk = extdat[EXT_ICHUNK];
  nchunk_in = extdat[EXT_NCHUNK];

  bprintf(info,"slot: %d chunk: %d nchunk: %d", slot_in, i_chunk, nchunk_in);

  nsched[i_chunk] = extdat[EXT_NSCHED];
  
  if ((slot != slot_in) || (nchunk_in != nchunk)) {
    chunks_received = 0;
    for (i=nchunk_in; i<32; i++) {
      chunks_received |= (1 << i);
    }
    slot = slot_in;
    nchunk = nchunk_in;
  }
  
  extdat_ui = (unsigned short *)(&extdat[6]);

  for (i=0; i<nsched[i_chunk]; i++) {
    sched[i_chunk][i][0] = extdat_ui[i*2];
    sched[i_chunk][i][1] = extdat_ui[i*2+1];
  }

  chunks_received |= (1<<i_chunk);

  CommandData.parts_sched = chunks_received;
  CommandData.upslot_sched = slot;
  
  if (chunks_received == 0xffffffff) {
    FILE *fp;
    char filename[128];
    OpenLibrary();

    sprintf(filename, "/data/etc/blast/%d.sch", slot);
    fp = fopen(filename, "w");
    
    fprintf(fp, "%s", lst0str);
    
    for (i_chunk=0; i_chunk < nchunk; i_chunk++) {
      for (i_samp=0; i_samp<nsched[i_chunk]; i_samp++) {
        entry = sched[i_chunk][i_samp][1];
        itime = sched[i_chunk][i_samp][0];
        day = (double)itime*MAX_DAYS/MAX_RTIME;
        hour = (day - floor(day))*24.0;
        if (entry<library.n) {
          fprintf(fp, "%s %d %.6g %s", library.cmd[entry], (int)day , hour, library.params[entry]);
        } else {
          bprintf(warning, "entry %d not in library\n", entry);
        }
      }
    }
    fclose(fp);
  }
  
  bprintf(warning, "finished extended command\n"
                   "  slot %d chunk %d n chunk %d nsched %d route %x chunks_received: %lx\n", 
                   extdat[EXT_SLOT], extdat[EXT_ICHUNK], extdat[EXT_NCHUNK], extdat[EXT_NSCHED], extdat[EXT_ROUTE], chunks_received);
}

void WatchPort (void* parameter)
{
  const char *COMM[] = {"/dev/ttyS0", "/dev/ttyS1"};
  const unsigned char route[2] = {0x09, 0x0c};

  unsigned char buf;
  unsigned short *indatadumper;
  unsigned char indata[20];
  int readstage = 0;
  int tty_fd;

  intptr_t port = (intptr_t)parameter;

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
  int extlen = 0;

  unsigned char extdat[256];

  char tname[6];
  sprintf(tname, "COMM%1d", port+1);
  nameThread(tname);
  //bprintf(startup, "WatchPort startup\n");

  tty_fd = sip_setserial(COMM[port]);

  for(;;) {
    /* Loop until data come in */
    while (read(tty_fd, &buf, 1) <= 0) {
      timer++;
      /** Request updated info every 50 seconds */
      if (timer == 800) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_POSITION, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Request SIP Position\n");
#endif
        pthread_mutex_unlock(&mutex);
      } else if (timer == 1700) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_TIME, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Request SIP Time\n");
#endif
        pthread_mutex_unlock(&mutex);	
      } else if (timer > 2500) {
        pthread_mutex_lock(&mutex);
        SendRequest (REQ_ALTITUDE, tty_fd);
#ifdef SIP_CHATTER
        bprintf(info, "Request SIP Altitude\n");
#endif
        pthread_mutex_unlock(&mutex);
        timer = 0;
      }
      usleep(10000); /* sleep for 10ms */
    }
#ifdef VERBOSE_SIP_CHATTER
    bprintf(info, "read SIP byte %02x\n", buf);
#endif

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
          bprintf(info, "Data request\n");
#endif
        } else if (buf == 0x14) { /* Command */
          readstage = 2;
#ifdef SIP_CHATTER
          bprintf(info, "Command\n");
#endif
        } else if (buf == 0x10) { /* GPS Position */
          readstage = 4;
#ifdef SIP_CHATTER
          bprintf(info, "GPS Position\n");
#endif
        } else if (buf == 0x11) { /* GPS Time */
          readstage = 5;
#ifdef SIP_CHATTER
          bprintf(info, "GPS Time\n");
#endif
        } else if (buf == 0x12) { /* MKS Altitude */
          readstage = 6;
#ifdef SIP_CHATTER
          bprintf(info, "MKS Altitude\n");
#endif
        } else {
          bprintf(warning, "Bad packet received: "
              "Unrecognised Packet Type: %02X\n", buf);
          readstage = 0;
        }
        break;
      case 2: /* waiting for command packet datum */
        if (bytecount == 0) {  /* Look for 2nd byte of command packet = 0x02 */
          if (buf == 0x02) {
            bytecount = 1;
          } else {
            readstage = 7;
            extlen = buf;
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
              //bprintf(info, "Single command received\n");
              //FIXME: this limits # single commands to 255. use indata[1] too
              SingleCommand(indata[0], 0);
              mcommand = -1;
            } else if ((indata[1] & 0xE0) == 0x80) {
              /*** Beginning of multi command ***/
              /*Grab first five bits of second byte containing command number*/
              mcommand = indata[0];
              mcommand_count = 0;
              dataqsize = DataQSize(MIndex(mcommand));
              bprintf(info, "UNSUPPORTED: Multi word command %s (%d) started\n",
                  MName(mcommand), mcommand);

              /* The time of sending, a "unique" number shared by the first */
              /* and last packed of a multi-command */
              mcommand_time = indata[1] & 0x1F;
            } else if (((indata[1] & 0x80) == 0) && (mcommand >= 0) &&
                (mcommand_count < dataqsize)) {
              /*** Parameter values in multi-command ***/
              indatadumper = (unsigned short *) indata;
              mcommand_data[mcommand_count] = *indatadumper;
              bprintf(info, "Multi word command continues...\n");
              mcommand_count++;
            } else if (((indata[1] & 0xE0) == 0xC0) && (mcommand == indata[0])
                && ((indata[1] & 0x1F) == mcommand_time) &&
                (mcommand_count == dataqsize)) {
              /*** End of multi-command ***/
              bprintf(info, "Multi word command ends \n");
              SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues,
                  ivalues, svalues);
              MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
              mcommand = -1;
              mcommand_count = 0;
              mcommand_time = 0;
            } else {
              mcommand = -1;
              mcommand_count = 0;
              bprintf(warning, "Command packet discarded: Bad Encoding: %04X\n",
                  indata[1]);
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
          bprintf(warning, "Bad encoding: Bad packet terminator: %02X\n", buf);
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
            bprintf(warning, "Bad encoding in GPS Position: "
                "Bad packet terminator: %02X\n", buf);
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
            bprintf(warning, "Bad encoding in GPS Time: "
                "Bad packet terminator: %02X\n", buf);
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
            bprintf(warning, "Bad encoding in MKS Altitude: "
                "Bad packet terminator: %02X\n", buf);
          }
        }
        break;
      case 7: // reading extended command
        if (bytecount < extlen) {
          extdat[bytecount] = buf;
          bytecount++;
        } else {
          if (buf == 0x03) {
            if (extdat[0] == sched_packet) {
              if (extdat[EXT_ROUTE] == route[port]) {
                bprintf(info, "Schedule file uplink packet detected\n");
                ProcessUplinkSched(extdat);
              } else {
                bprintf(info,
                    "Schedule file uplink packet bad route %d != %d\n",
                    extdat[EXT_ROUTE], route[port]);
              }		
            } else {
              if (MIndex(extdat[0])<0) {
                bprintf(warning, "ignoring unknown extended command (%d)",
                    extdat[0]);
              } else {
                bprintf(info, "extended command %s (%d)", MName(extdat[0]),
                    extdat[0]);
                SetParameters(extdat[0], (unsigned short*)(extdat+2), rvalues,
                    ivalues, svalues);
                MultiCommand(extdat[0], rvalues, ivalues, svalues, 0);
              }
            }
          } else {
            bprintf(warning, "Bad encoding in extended command: "
                "Bad packet terminator: %02X\n", buf);
          }
          bytecount = 0;
          readstage = 0;
        }
        break;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);
  }
}

#endif    // USE_SIP_CMD
