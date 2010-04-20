/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/statvfs.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include "bbc_pci.h"

#include "blast.h"
#include "command_list.h"
#include "command_struct.h"
#include "crc.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "slow_dl.h"
#include "starpos.h"
#include "channels.h"
#include "tx.h"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#define STARTUP_VETO_LENGTH 250 /* "frames" */
#define BI0_VETO_LENGTH 5 /* seconds */

#define BI0_FIFO_MARGIN 8750 /* bytes = 7 frames */
#define BI0_FIFO_MINIMUM (BI0_FIFO_MARGIN / 2)

#ifdef BOLOTEST
#  define FRAME_MARGIN (-12)
#else
#  define FRAME_MARGIN (-2)
#endif

#define BI0_FRAME_BUFLEN (400)
/* Define global variables */
int bbc_fp = -1;
unsigned int debug = 0;
short int SouthIAm;
struct ACSDataStruct ACSData;

unsigned int RxFrameFastSamp;
unsigned short* slow_data[FAST_PER_SLOW];
pthread_t watchdog_id;

int StartupVeto = STARTUP_VETO_LENGTH + 1;

static int bi0_fp = -2;
static int Death = -STARTUP_VETO_LENGTH * 2;
static int RxFrameIndex;

extern short int InCharge; /* tx.c */
extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];
extern pthread_mutex_t mutex;

void Pointing();
void WatchPort(void*);
void WatchDGPS(void);
void IntegratingStarCamera(void);
void ActuatorBus(void);
void WatchFIFO(void);
void FrameFileWriter(void);
void TDRSSWriter(void);
void StageBus(void);
void HWPRBus(void);

void InitialiseFrameFile(char);
void dirFileWriteFrame(unsigned short *RxFrame);
void pushDiskFrame(unsigned short *RxFrame);

void SunSensor(void);

void InitSched();

static FILE* logfile = NULL;

#ifndef BOLOTEST
static struct {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
} bi0_buffer;
#endif

unsigned short *tdrss_data[3];
unsigned int tdrss_index = 0;
time_t biphase_timer;
int biphase_is_on = 0;

struct chat_buf chatter_buffer;

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 3                /* start-of-line marker */ \
  - 24               /* date "YYYY-MM-DD HH:MM:SS.mmm " */ \
  - 2                /* Newline and NUL */ \
)

#if (TEMPORAL_OFFSET != 0)
#warning TEMPORAL_OFFSET NON-ZERO; FIX FOR FLIGHT
#endif

#ifndef BOLOTEST
void openMotors(); //motors.c
void closeMotors();
#endif

time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}

/* tid to name lookup list */
/* TODO setting/reading tid names not quite thread safe. is this a problem? */
#define TID_NAME_LEN  6	      //always change with TID_NAME_FMT
#define TID_NAME_FMT  "%6s"   //always change with TID_NAME_LEN
struct tid_name {
  int tid;
  char name[TID_NAME_LEN+1];
  struct tid_name* next;
};

struct tid_name* threadNames = NULL;

void nameThread(const char* name)
{
  struct tid_name* new_node = (struct tid_name*)malloc(sizeof(struct tid_name));
  new_node->tid = syscall(SYS_gettid);
  strncpy(new_node->name, name, TID_NAME_LEN);
  new_node->name[TID_NAME_LEN] = '\0';
  new_node->next = threadNames;
  threadNames = new_node;
}

char failed_lookup_buffer[TID_NAME_LEN+1];
char* threadNameLookup(int tid)
{
  struct tid_name* p;
  for(p=threadNames; p->next != NULL; p = p->next)
    if (p->tid == tid) return p->name;
  //not found, just print tid
  snprintf(failed_lookup_buffer, TID_NAME_LEN, "%u", (unsigned)tid);
  failed_lookup_buffer[TID_NAME_LEN] = '\0';
  return failed_lookup_buffer;
}
  

void mputs(buos_t flag, const char* message) {
  char buffer[MPRINT_BUFFER_SIZE];
  struct timeval t;
  struct timezone tz; /* We never use this, but gettimeofday won't let us
                         give it a NULL -- also it's obsolete under linux */
  struct tm now;
  char local[1024];
  char *bufstart, *bufptr, *lastchr, *firstchr;
  int len;
  char marker[4];

  /* time */
  gettimeofday(&t, &tz);
  t.tv_sec += TEMPORAL_OFFSET;

  switch(flag) {
    case err:
      strcpy(marker, "* ");
      break;
    case fatal:
      strcpy(marker, "! ");
      break;
    case info:
      strcpy(marker, "- ");
      break;
    case sched:
      strcpy(marker, "# ");
      break;
    case startup:
      strcpy(marker, "> ");
      break;
    case tfatal:
      strcpy(marker, "$ ");
      break;
    case warning:
      strcpy(marker, "= ");
      break;
    case mem:
      return;  /* don't record mem messages at all */
      strcpy(marker, "m ");
      break;
    default:
      strcpy(marker, "? ");
      break;
  }
  strcpy(buffer, marker);

  /* we need a writable copy of the string */
  strncpy(local, message, 1023);
  local[1023] = '\0';

  for(bufstart = buffer; *bufstart != '\0' && bufstart < buffer
      + 1024; ++bufstart);


  strftime(bufstart, 1023, "%F %T", gmtime_r(&t.tv_sec, &now));

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, ".%03li ", t.tv_usec/1000);
  strcat(buffer, marker);

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, TID_NAME_FMT ": ", threadNameLookup(syscall(SYS_gettid)));

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

    /* output the formatted string line by line */
    for (firstchr = bufptr = local; *bufptr != '\0' &&
        bufptr < local + 1023; ++bufptr) {

      /* writeout the string when we find a newline or the EOS */
      if (*bufptr == '\n' || *(bufptr + 1) == '\0') {
        lastchr = (*bufptr == '\n') ? bufptr : bufptr + 1;
        *lastchr = '\0';

        /* compute length of string to writeout */
        len = lastchr - firstchr + 1;
        if (len > MAX_MPRINT_STRING - 1)
          len = MAX_MPRINT_STRING - 1;

        /* append string part and a newline to preamble */
        strncpy(bufstart, firstchr, len);
        *(bufstart + len + 1) = '\0';
        strcat(bufstart, "\n");
        if (logfile != NULL) {
          fputs(buffer, logfile);
          fflush(logfile);
        }
        if (logfile == NULL || flag != mem) {
          fputs(buffer, stdout);
          fflush(stdout);
        }

        firstchr = bufptr + 1;
      }
    }

  if (flag == fatal) {
    if (logfile != NULL) {
      fputs("!! Last error is FATAL.  Cannot continue.\n", logfile);
      fflush(logfile);
    }
    fputs("!! Last error is FATAL.  Cannot continue.\n", stdout);
    fflush(stdout);

    exit(1);
  }

  if (flag == tfatal) {
    if (logfile != NULL) {
      fprintf(logfile,
          "$$ Last error is THREAD FATAL.  Thread [%5u] exits.\n", (unsigned)syscall(SYS_gettid));
      fflush(logfile);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [%5u] exits.\n", (unsigned)syscall(SYS_gettid));
    fflush(stdout);

    pthread_exit(NULL);
  }
}

#ifndef BOLOTEST
static void FillSlowDL(unsigned short *RxFrame)
{
  int i;
  unsigned short msb, lsb;

  for (i = 0, msb = 0; i < SLOWDL_NUM_DATA; i++) {
    if (SlowDLInfo[i].mindex == NOT_MULTIPLEXED) {
      lsb = RxFrame[SlowDLInfo[i].chnum];
      if (SlowDLInfo[i].wide)
        msb = RxFrame[SlowDLInfo[i].chnum + 1];
      else
        msb = 0;
      SlowDLInfo[i].value = (double)((msb << 16) | lsb);
    } else {
      lsb = slow_data[SlowDLInfo[i].mindex][SlowDLInfo[i].chnum];
      if (SlowDLInfo[i].wide)
        msb = slow_data[SlowDLInfo[i].mindex][SlowDLInfo[i].chnum + 1];
      else
        msb = 0;
      SlowDLInfo[i].value = (double)((msb << 16) | lsb);
    }
  }
}

static void SensorReader(void)
{
  int data;
  int nr;
  struct statvfs vfsbuf;
  int sensor_error = 0;

  FILE *stream;

  nameThread("Sensor");
  bputs(startup, "Startup\n");

  while (1) {
    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp1_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp1 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp1 from I2C bus");
      sensor_error = 5;
    }

    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp2_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp2 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp2 from I2C bus");
      sensor_error = 5;
    }

    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp3 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp3 from I2C bus");
      sensor_error = 5;
    }

    if (statvfs("/data", &vfsbuf))
      berror(warning, "Cannot stat filesystem");
    else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bavail / 1000;
    }

    if (sensor_error)
      sensor_error--;

    usleep(100000);
  }
}

static void Chatter(void* arg)
{
  int fd;
  char ch;
  ssize_t ch_got;
  off_t fpos;

  fpos = *(off_t*)arg;

  nameThread("Chat");

  bprintf(startup, "Thread startup\n");

  fd = open("/data/etc/mcp.log", O_RDONLY|O_NONBLOCK);

  if (fd == -1)
  {
    bprintf(tfatal, "Failed to open /data/etc/mcp.log for reading (%d)\n", errno);
  }

  if (fpos == -1) {
    if (lseek(fd, -500, SEEK_END) == -1)
    {
      if (errno == EINVAL)
      {
	if (lseek(fd, 0, SEEK_SET) == -1)
	{
	  bprintf(tfatal, "Failed to rewind /data/etc/mcp.log (%d)\n", errno);
	}
      } else {
	bprintf(tfatal, "Failed to seek /data/etc/mcp.log (%d)\n", errno);
      }
    }
  } else {
    if (lseek(fd, fpos, SEEK_SET) == -1)
    {
      if (lseek(fd, 0, SEEK_END) == -1)
      {
	bprintf(tfatal, "Failed to rewind /data/etc/mcp.log (%d)\n", errno);
      }
    }
  }

  while (read(fd, &ch, 1) == 1 && ch != '\n'); /* Find start of next message */

  chatter_buffer.reading = chatter_buffer.writing = 0;
      /* decimal 22 is "Synchronous Idle" in ascii */
  memset(chatter_buffer.msg, 22, sizeof(char) * FAST_PER_SLOW * 2 * 4);

  while (1)
  {
    if (chatter_buffer.writing != ((chatter_buffer.reading - 1) & 0x3))
    {
      ch_got = read(fd, chatter_buffer.msg[chatter_buffer.writing], 2 * FAST_PER_SLOW * sizeof(char));
      if (ch_got == -1)
      {
        bprintf(tfatal, "Error reading from /data/etc/mcp.log (%d)\n", errno);
      }
      if (ch_got < (2 * FAST_PER_SLOW * sizeof(char)))
      {
        memset(&(chatter_buffer.msg[chatter_buffer.writing][ch_got]), 22, (2 * FAST_PER_SLOW * sizeof(char)) - ch_got);
      }
      chatter_buffer.writing = ((chatter_buffer.writing + 1) & 0x3);
    }
    usleep(100000);
  }
}


static void GetACS(unsigned short *RxFrame)
{
  double enc_raw_el, ifel_gy, ifroll_gy, ifyaw_gy;
  double x_comp, y_comp, z_comp;
  double vel_raw_rw;
  double res_raw_piv;
  static struct BiPhaseStruct* ifElgyAddr;
  static struct BiPhaseStruct* ifRollgyAddr;
  static struct BiPhaseStruct* ifYawgyAddr;
  static struct BiPhaseStruct* encElevAddr;
  static struct BiPhaseStruct* clinElevAddr;
  static struct BiPhaseStruct* magXAddr;
  static struct BiPhaseStruct* magYAddr;
  static struct BiPhaseStruct* magZAddr;
  static struct BiPhaseStruct* velRawRWAddr;
  static struct BiPhaseStruct* resRawPivAddr;

  unsigned int rx_frame_index = 0;
  int i_ss;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    encElevAddr = GetBiPhaseAddr("enc_raw_el");
    clinElevAddr = GetBiPhaseAddr("clin_elev");
    ifElgyAddr = GetBiPhaseAddr("ifel_gy");
    ifRollgyAddr = GetBiPhaseAddr("ifroll_gy");
    ifYawgyAddr = GetBiPhaseAddr("ifyaw_gy");
    magXAddr = GetBiPhaseAddr("mag_x");
    magYAddr = GetBiPhaseAddr("mag_y");
    magZAddr = GetBiPhaseAddr("mag_z");
    velRawRWAddr = GetBiPhaseAddr("vel_raw_rw");
    resRawPivAddr = GetBiPhaseAddr("res_raw_piv");
  }

  rx_frame_index = ((RxFrame[1] & 0x0000ffff) |
      (RxFrame[2] & 0x0000ffff) << 16);

  enc_raw_el = (((double)RxFrame[encElevAddr->channel])/DEG2I);
  vel_raw_rw = (((double)((short)RxFrame[velRawRWAddr->channel]))*4.0/DEG2I);
  res_raw_piv = (((double)((short)RxFrame[resRawPivAddr->channel]))/DEG2I);
  ifel_gy = (double)((RxFrame[ifElgyAddr->channel])-GY16_OFFSET) * GY16_TO_DPS;
  ifroll_gy = (double)(RxFrame[ifRollgyAddr->channel]-GY16_OFFSET) * GY16_TO_DPS;
  ifyaw_gy = (double)(RxFrame[ifYawgyAddr->channel]-GY16_OFFSET) * GY16_TO_DPS;

  x_comp = (double)(RxFrame[magXAddr->channel]);
  y_comp = (double)(RxFrame[magYAddr->channel]);
  z_comp = (double)(RxFrame[magZAddr->channel]);
  
  i_ss = ss_index;

  ACSData.t = mcp_systime(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.enc_raw_el = enc_raw_el;
  ACSData.ifel_gy = ifel_gy;
  ACSData.ifroll_gy = ifroll_gy;
  ACSData.ifyaw_gy = ifyaw_gy;
  ACSData.mag_x = x_comp;
  ACSData.mag_y = y_comp;
  ACSData.mag_z = z_comp;
  ACSData.vel_raw_rw = vel_raw_rw;
  ACSData.res_raw_piv = res_raw_piv;

  ACSData.clin_elev = (double)RxFrame[clinElevAddr->channel];

}
#endif

/* fill_Rx_frame: places one 32 bit word into the RxFrame. Returns true on
 * success */
static int fill_Rx_frame(unsigned int in_data, unsigned short *RxFrame)
{
  static int n_not_found = 0;
  struct BiPhaseStruct BiPhaseData;

  if (in_data == 0xffffffff)
    return 1;

  /* discard ADC sync words */
  if (in_data & BBC_ADC_SYNC)
    return 1;

  /* words with no write flag are ignored, don't process them */
  if (~in_data & BBC_WRITE)
    return 1;

  /* BBC reverse address lookup */
  BiPhaseData = BiPhaseLookup[BI0_MAGIC(in_data)];

  /* Return error if the lookup failed */
  if (BiPhaseData.index == -1) {
    if (++n_not_found > 2)
      return 0;
    else
      return 1;
  }

  n_not_found = 0;

  /* Discard words we're not saving */
  if (BiPhaseData.index == DISCARD_WORD)
    return 1;

  /* Write the data to the local buffers */
  if (BiPhaseData.index == NOT_MULTIPLEXED)
    RxFrame[BiPhaseData.channel] = BBC_DATA(in_data);
  else
    slow_data[BiPhaseData.index][BiPhaseData.channel] = BBC_DATA(in_data);

  return(1);
}

#ifndef BOLOTEST
static void WatchDog (void)
{
  nameThread("WDog");
  bputs(startup, "Startup\n");

  /* Allow other threads to kill this one at any time */
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

  if (ioperm(0x378, 0x0F, 1) != 0)
    berror(tfatal, "Error setting watchdog permissions");
  ioperm(0x80, 1, 1);

  for (;;) {
    outb(0xAA, 0x378);
    usleep(40000);
    outb(0x55, 0x378);
    usleep(40000);
  }
}

static int write_to_biphase(unsigned short *RxFrame, int i_in, int i_out)
{
  int i;
  static unsigned short nothing[BI0_FRAME_SIZE];
  static unsigned short sync = 0xEB90;
  static unsigned int do_skip = 0;

  if (bi0_fp == -2) {
    bi0_fp = open("/dev/bbc_bi0", O_RDWR);
    if (bi0_fp == -1)
      berror(tfatal, "Error opening biphase device");

    for (i = 0; i < BI0_FRAME_SIZE; i++)
      nothing[i] = 0xEEEE;
  }
  i_out = (i_out + 1) % BI0_FRAME_BUFLEN;

  if (bi0_fp >= 0 && InCharge) {

    RxFrame[0] = 0xEB90;
    nothing[0] = CalculateCRC(0xEB90, RxFrame, BiPhaseFrameWords);

    RxFrame[0] = sync;
    sync = ~sync;

//    unsigned short buffer[BI0_FRAME_SIZE];
//    static unsigned int fc = 0;
//    for (i = 0; i < BI0_FRAME_SIZE; ++i) {
//      buffer[i] = i;
//    }
//    *(unsigned int*)(&buffer[1]) = fc++;
//    buffer[0] = RxFrame[0];
//    i = write(bi0_fp, buffer, BI0_FRAME_SIZE * sizeof(unsigned short));

    if (do_skip) {
      --do_skip;
    } else {
      i = write(bi0_fp, RxFrame, BiPhaseFrameWords * sizeof(unsigned short));
      if (i < 0)
        berror(err, "bi-phase write for RxFrame failed");
      else if (i != BiPhaseFrameWords * sizeof(unsigned short))
        bprintf(err, "Short write for RxFrame: %i of %u", i,
            BiPhaseFrameWords * sizeof(unsigned short));

      i = write(bi0_fp, nothing, (BI0_FRAME_SIZE - BiPhaseFrameWords) *
          sizeof(unsigned short));
      if (i < 0)
        berror(err, "bi-phase write for padding failed");
      else if (i != (BI0_FRAME_SIZE - BiPhaseFrameWords)
          * sizeof(unsigned short))
        bprintf(err, "Short write for padding: %i of %u", i,
            (BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(unsigned short));
    }

    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);
#if 0
    if (do_skip == 0 && CommandData.bi0FifoSize > BI0_FIFO_MARGIN) {
      do_skip = (CommandData.bi0FifoSize - BI0_FIFO_MINIMUM) / BI0_FRAME_SIZE;
      bprintf(warning, "Excess data in FIFO, discarding "
          "%i frames out of hand.", do_skip);
    }
#endif
  }

  return i_out;
}

static void InitBi0Buffer()
{
  int i;

  bi0_buffer.i_in = 10; /* preload the fifo */
  bi0_buffer.i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++) {
    bi0_buffer.framelist[i] = balloc(fatal, BiPhaseFrameWords *
        sizeof(unsigned short));
    ///*
    //TODO this initialization does not appear to resolve valgrind errors
    memset(bi0_buffer.framelist[i],0,BiPhaseFrameWords*sizeof(unsigned short));
    //*/
  }
}

static void PushBi0Buffer(unsigned short *RxFrame)
{
  int i, fw, i_in;

  i_in = bi0_buffer.i_in + 1;
  if (i_in>=BI0_FRAME_BUFLEN)
    i_in = 0;

  fw = BiPhaseFrameWords;

  for (i = 0; i<fw; i++) {
    bi0_buffer.framelist[i_in][i] = RxFrame[i];
  }

  bi0_buffer.i_in = i_in;
}
#endif

static void zero(unsigned short *RxFrame)
{
  int i;

  for (i = 0; i < SLOW_OFFSET + slowsPerBi0Frame; i++)
    RxFrame[i] = 0;
}

#ifndef BOLOTEST
static void BiPhaseWriter(void)
{
  int i_out, i_in;

  nameThread("Bi0");
  bputs(startup, "Startup\n");

  while (!biphase_is_on)
    usleep(10000);

  bputs(info, "Veto has ended.  Here we go.\n");

  while (1) {
    i_in = bi0_buffer.i_in;
    i_out = bi0_buffer.i_out;
    if (i_out == i_in) {
      /* Death meausres how long the BiPhaseWriter has gone without receiving
       * any data -- an indication that we aren't receiving FSYNCs from the
       * BLASTBus anymore */
      if (InCharge) {
        if (++Death == 25) {
          bprintf(err, "Death is reaping the watchdog tickle.");
          pthread_cancel(watchdog_id);
        }
      }
    } else
      while (i_out != i_in) {
        i_out = write_to_biphase(bi0_buffer.framelist[i_out], i_in, i_out);
        if (Death > 0)
          Death = 0;
      }
    bi0_buffer.i_out = i_out;
    usleep(10000);
  }
}
#endif

/******************************************************************/
/*                                                                */
/* IsNewFrame: returns true if d is a begining of frame marker,   */
/*    unless this is the first beginning of frame.                */
/*                                                                */
/******************************************************************/
static int IsNewFrame(unsigned int d)
{
  static int first_bof = 1;
  int is_bof;
  is_bof = (d == (BBC_FSYNC | BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | 0xEB90));
  if (is_bof && first_bof) {
    is_bof = first_bof = 0;
  }

  return is_bof;
}

/* Polarity crisis: am I north or south? */
static int AmISouth(void)
{
  char buffer[2];

  if (gethostname(buffer, 1) == -1 && errno != ENAMETOOLONG) {
    berror(err, "System: Unable to get hostname");
  }

  return (buffer[0] == 's') ? 1 : 0;
}

/* Signal handler called when we get a hup, int or term */
static void CloseBBC(int signo)
{
  bprintf(err, "System: Caught signal %i; stopping NIOS", signo);
#ifndef BOLOTEST
  closeMotors();
#endif
  RawNiosWrite(0, BBC_ENDWORD, NIOS_FLUSH);
  RawNiosWrite(BBCPCI_MAX_FRAME_SIZE, BBC_ENDWORD, NIOS_FLUSH);
  bprintf(err, "System: Closing BBC and Bi0");
  if (bi0_fp >= 0)
    close(bi0_fp);
  if (bbc_fp >= 0)
    close(bbc_fp);

  /* restore default handler and raise the signal again */
  signal(signo, SIG_DFL);
  raise(signo);
}

#if 0
static char segvregs[100];
static int segvcnt = 0;
static void SegV(int signo)
{
  fprintf(stderr, "SEGV caught: %s\n", segvregs);
  raise(SIGTERM);
}
#endif

int main(int argc, char *argv[])
{
  unsigned int in_data, i;
  unsigned short* RxFrame;
  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t abus_id;

#ifndef USE_FIFO_CMD
  pthread_t CommandDatacomm2;
#endif

#ifndef BOLOTEST
  pthread_t sunsensor_id;
  pthread_t tdrss_id;
  pthread_t bi0_id;
  pthread_t sensors_id;
  pthread_t dgps_id;
  pthread_t isc_id;
  pthread_t osc_id;
#endif
#ifdef USE_XY_THREAD
  pthread_t xy_id;
#endif
  pthread_t chatter_id;
  pthread_t hwpr_id;
  struct stat fstats;

  if (argc == 1) {
    fprintf(stderr, "Must specify file type:\n"
        "p  pointing\n"
        "m  maps\n"
        "c  cryo\n"
        "n  noise\n"
        "x  software test\n"
        "f  flight\n");
    exit(0);
  }

  umask(0);  /* clear umask */

  if ((logfile = fopen("/data/etc/mcp.log", "a")) == NULL) {
    berror(err, "System: Can't open log file");
    fstats.st_size = -1;
  } else {
    if (fstat(fileno(logfile), &fstats) < 0)
      fstats.st_size = -1;
    fputs("!!!!!! LOG RESTART !!!!!!\n", logfile);
  }

  biphase_timer = mcp_systime(NULL) + BI0_VETO_LENGTH;

  /* register the output function */
  nameThread("Dummy"); //insert dummy sentinel node first
  nameThread("Main");
  buos_use_func(mputs);

#if (TEMPORAL_OFFSET > 0)
  bprintf(warning, "System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  bputs(startup, "System: Startup");

#ifndef BOLOTEST
  /* Watchdog */
  pthread_create(&watchdog_id, NULL, (void*)&WatchDog, NULL);
#endif

  if ((bbc_fp = open("/dev/bbcpci", O_RDWR)) < 0)
    berror(fatal, "System: Error opening BBC");

  InitCommandData();

  pthread_mutex_init(&mutex, NULL);

  MakeAddressLookups();

  bprintf(info, "Commands: MCP Command List Version: %s", command_list_serial);
#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
  ReductionInit();

  bprintf(info, "System: Slow Downlink Initialisation");
  InitSlowDL();

  InitBi0Buffer();

  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif

  InitialiseFrameFile(argv[1][0]);

  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  /* Allocate the local data buffers */
  RxFrame = balloc(fatal, BiPhaseFrameSize);

  for (i = 0; i < 3; ++i)
    tdrss_data[i] = (unsigned short *)balloc(fatal, BiPhaseFrameSize);

  for (i = 0; i < FAST_PER_SLOW; ++i) {
    slow_data[i] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));
    //TODO fix "uninitialised value" valgrind errors. Ensure not more serious.
    memset(slow_data[i], 0, slowsPerBi0Frame * sizeof(unsigned short));
  }

  /* Find out whether I'm north or south */
  SouthIAm = AmISouth();

  if (SouthIAm)
    bputs(info, "System: I am South.\n");
  else
    bputs(info, "System: I am not South.\n");

#ifndef BOLOTEST
  InitSched();
  openMotors();  //open communications with peripherals, creates threads
                 // in motors.c
#endif

  bputs(info, "System: Finished Initialisation, waiting for BBC to come up.\n");

  /* mcp used to wait here for a semaphore from the BBC, which makes the
   * presence of these messages somewhat "historical" */

  bputs(info, "System: BBC is up.\n");

  InitTxFrame(RxFrame);

#ifndef BOLOTEST
  pthread_create(&chatter_id, NULL, (void*)&Chatter, (void*)&(fstats.st_size));
#endif

#ifdef USE_XY_THREAD
  pthread_create(&xy_id, NULL, (void*)&StageBus, NULL);
#endif
#ifndef BOLOTEST
  pthread_create(&dgps_id, NULL, (void*)&WatchDGPS, NULL);
  pthread_create(&isc_id, NULL, (void*)&IntegratingStarCamera, (void*)0);
  pthread_create(&osc_id, NULL, (void*)&IntegratingStarCamera, (void*)1);

  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  pthread_create(&sunsensor_id, NULL, (void*)&SunSensor, NULL);

  pthread_create(&tdrss_id, NULL, (void*)&TDRSSWriter, NULL);
  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
#endif
  pthread_create(&abus_id, NULL, (void*)&ActuatorBus, NULL);
  pthread_create(&hwpr_id, NULL, (void*)&HWPRBus, NULL);

  while (1) {
    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0)
      berror(err, "System: Error on BBC read");

    if (!fill_Rx_frame(in_data, RxFrame))
      bprintf(err, "System: Unrecognised word received from BBC (%08x)",
          in_data);

    if (IsNewFrame(in_data)) {
      if (StartupVeto > 1) {
        --StartupVeto;
      } else {
#ifndef BOLOTEST
        GetACS(RxFrame);
        Pointing();

        /* Copy data to tdrss thread. */
        memcpy(tdrss_data[tdrss_index], RxFrame, BiPhaseFrameSize);
        tdrss_index = INC_INDEX(tdrss_index);
#endif

        /* Frame sequencing check */
        if (StartupVeto) {
          bputs(info, "System: Startup Veto Ends\n");
          StartupVeto = 0;
          Death = 0;
        } else if (RxFrame[3] != (RxFrameIndex + 1) % FAST_PER_SLOW
            && RxFrameIndex >= 0) {
          bprintf(err, "System: Frame sequencing error detected: wanted %i, "
              "got %i\n", RxFrameIndex + 1, RxFrame[3]);
	}
        RxFrameIndex = RxFrame[3];

        /* Save current fastsamp */
        RxFrameFastSamp = (RxFrame[1] + RxFrame[2] * 0x10000);

        UpdateBBCFrame(RxFrame);
        CommandData.bbcFifoSize = ioctl(bbc_fp, BBCPCI_IOC_BBC_FIONREAD);

        /* pushDiskFrame must be called before PushBi0Buffer to get the slow
           data right */
        pushDiskFrame(RxFrame);
#ifndef BOLOTEST
        if (biphase_is_on)
          PushBi0Buffer(RxFrame);
        else if (biphase_timer < mcp_systime(NULL))
          biphase_is_on = 1;

        FillSlowDL(RxFrame);
#endif
        zero(RxFrame);
      }
    }
  }
  return(0);
}
