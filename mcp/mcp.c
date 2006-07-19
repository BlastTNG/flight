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

#define STARTUP_VETO_LENGTH 250

#ifdef BOLOTEST
#  define FRAME_MARGIN (-12)
#else
#  define FRAME_MARGIN (-2)
#endif

#define BI0_FRAME_BUFLEN (40)
/* Define global variables */
int bbc_fp = -1;
unsigned int debug = 0;
short int SamIAm;
struct ACSDataStruct ACSData;
unsigned int RxFrameFastSamp;
unsigned short* slow_data[FAST_PER_SLOW];
pthread_t watchdog_id;

static int bi0_fp = -2;
static int StartupVeto = STARTUP_VETO_LENGTH + 1;
static int Death = -STARTUP_VETO_LENGTH;
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

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 3                /* start-of-line marker */ \
  - 31               /* date "YYYY-MM-DD HH:MM:SS.mmmmmm GMT " */ \
  - 4                /* marker again plus NUL */ \
)

#if (TEMPORAL_OFFSET != 0)
#warning TEMPORAL_OFFSET NON-ZERO; FIX FOR FLIGHT
#endif

time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
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
      strcpy(marker, "** ");
      break;
    case fatal:
      strcpy(marker, "!! ");
      break;
    case info:
      strcpy(marker, "-- ");
      break;
    case sched:
      strcpy(marker, "## ");
      break;
    case startup:
      strcpy(marker, ">> ");
      break;
    case tfatal:
      strcpy(marker, "$$ ");
      break;
    case warning:
      strcpy(marker, "== ");
      break;
    case mem:
      return;  /* don't record mem messages at all */
      strcpy(marker, "mm ");
      break;
    default:
      strcpy(marker, "?? ");
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

  sprintf(bufstart, ".%06li GMT ", t.tv_usec);
  strcat(buffer, marker);

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, "[%5i] ", getpid());

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
          "$$ Last error is THREAD FATAL.  Thread [%5i] exits.\n", getpid());
      fflush(logfile);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [%5i] exits.\n", getpid());
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

  FILE *stream;

  bputs(startup, "Sensor Reader: Startup\n");

  while (1) {
    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/temp1_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp1 = data / 10;
      fclose(stream);
    } else
      berror(warning, "Sensor Reader: Cannot read temp1 from I2C bus");
    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/temp2_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp2 = data / 10;
      fclose(stream);
    } else
      berror(warning, "Sensor Reader: Cannot read temp2 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/temp3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp3 = data / 10;
      fclose(stream);
    } else
          berror(warning, "Sensor Reader: Cannot read temp3 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/fan3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.fan = data;
      fclose(stream);
    } else
          berror(warning, "Sensor Reader: Cannot read fan3 from I2C bus");

    if (statvfs("/data", &vfsbuf))
      berror(warning, "Sensor Reader: Cannot stat filesystem");
    else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bavail / 1000;
    }
    sleep(1);
  }
}

static void GetACS(unsigned short *RxFrame)
{
  double enc_elev, gyro1, gyro2, gyro3;
  double x_comp, y_comp, z_comp;
  static struct BiPhaseStruct* gyro1Addr;
  static struct BiPhaseStruct* gyro2Addr;
  static struct BiPhaseStruct* gyro3Addr;
  static struct BiPhaseStruct* encElevAddr;
  static struct BiPhaseStruct* clinElevAddr;
  static struct BiPhaseStruct* magXAddr;
  static struct BiPhaseStruct* magYAddr;
  static struct BiPhaseStruct* magZAddr;
  
  unsigned int rx_frame_index = 0;
  int i_ss;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    encElevAddr = GetBiPhaseAddr("enc_elev");
    clinElevAddr = GetBiPhaseAddr("clin_elev");
    gyro1Addr = GetBiPhaseAddr("gyro1");
    gyro2Addr = GetBiPhaseAddr("gyro2");
    gyro3Addr = GetBiPhaseAddr("gyro3");
    magXAddr = GetBiPhaseAddr("mag_x");
    magYAddr = GetBiPhaseAddr("mag_y");
    magZAddr = GetBiPhaseAddr("mag_z");
  }

  rx_frame_index = ((RxFrame[1] & 0x0000ffff) |
      (RxFrame[2] & 0x0000ffff) << 16);

  if ((enc_elev = ((double)RxFrame[encElevAddr->channel] * (-360.0 / 65536.0)
          + ENC_ELEV_OFFSET)) < 0)
    enc_elev += 360;

  gyro1 = (double)(RxFrame[gyro1Addr->channel]-GY16_OFFSET) * GY16_TO_DPS;
  gyro2 = (double)(RxFrame[gyro2Addr->channel]-GY16_OFFSET) * GY16_TO_DPS;
  gyro3 = (double)(RxFrame[gyro3Addr->channel]-GY16_OFFSET) * GY16_TO_DPS;

  x_comp = (double)(RxFrame[magXAddr->channel]);
  y_comp = (double)(RxFrame[magYAddr->channel]);
  z_comp = (double)(RxFrame[magZAddr->channel]);
  
  i_ss = ss_index;

  ACSData.t = mcp_systime(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.enc_elev = enc_elev;
  ACSData.gyro1 = gyro1;
  ACSData.gyro2 = gyro2;
  ACSData.gyro3 = gyro3;
  ACSData.mag_x = x_comp;
  ACSData.mag_y = y_comp;
  ACSData.mag_z = z_comp;

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
  bputs(startup, "Watchdog: Startup\n");

  /* Allow other threads to kill this one at any time */
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

  if (ioperm(0x378, 0x0F, 1) != 0)
    berror(tfatal, "Watchdog: Error setting watchdog permissions");
  ioperm(0x80, 1, 1);

  for (;;) {
    outb(0xAA, 0x378);
    usleep(40000);
    outb(0x55, 0x378);
    usleep(40000);
  }
}

static void write_to_biphase(unsigned short *RxFrame)
{
  int i;
  static unsigned short nothing[BI0_FRAME_SIZE];
  static unsigned short sync = 0xEB90;

  if (bi0_fp == -2) {
    bi0_fp = open("/dev/bi0_pci", O_RDWR);
    if (bi0_fp == -1)
      berror(tfatal, "BiPhase Writer: Error opening biphase device");

    for (i = 0; i < BI0_FRAME_SIZE; i++)
      nothing[i] = 0xEEEE;
  }

  nothing[0] = CalculateCRC(0xEB90, RxFrame, BiPhaseFrameWords);

  if (bi0_fp >= 0) {
    RxFrame[0] = sync;
    sync = ~sync;
    if (write(bi0_fp, RxFrame, BiPhaseFrameWords * sizeof(unsigned short)) < 0)
      berror(err, "BiPhase Writer: bi-phase write (RxFrame) failed");
    if (write(bi0_fp, nothing,
          (BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(unsigned short)) < 0)
      berror(err, "BiPhase Writer: bi-phase write (padding) failed");
    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);
  }
}

static void InitBi0Buffer()
{
  int i;

  bi0_buffer.i_in = 10; /* preload the fifo */
  bi0_buffer.i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++)
    bi0_buffer.framelist[i] = balloc(fatal, BiPhaseFrameWords *
        sizeof(unsigned short));
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

  bputs(startup, "Biphase Writer: Startup\n");

  while (1) {
    i_in = bi0_buffer.i_in;
    i_out = bi0_buffer.i_out;
    if (i_out == i_in) {
      /* Death meausres how long the BiPhaseWriter has gone without receiving
       * any data -- an indication that we aren't receiving FSYNCs from the
       * BLASTBus anymore */
      if (InCharge) {
        if (++Death == 25) {
          bprintf(err, "BiPhase Writer: Death is reaping the watchdog tickle.");
          pthread_cancel(watchdog_id);
        }
      }
    } else
      while (i_out != i_in) {
        i_out = (i_out + 1) % BI0_FRAME_BUFLEN;
        write_to_biphase(bi0_buffer.framelist[i_out]);
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

/* Identity crisis: am I frodo or sam? */
static int AmISam(void)
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

  if ((logfile = fopen("/data/etc/mcp.log", "a")) == NULL)
    berror(err, "System: Can't open log file");
  else
    fputs("----- LOG RESTART -----\n", logfile);

  /* register the output function */
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

  for (i = 0; i < FAST_PER_SLOW; ++i)
    slow_data[i] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));

  /* Find out whether I'm frodo or sam */
  SamIAm = AmISam();

  if (SamIAm)
    bputs(info, "System: I am Sam.\n");
  else
    bputs(info, "System: I am not Sam.\n");

#ifndef BOLOTEST
  InitSched();
#endif

  bputs(info, "System: Finished Initialisation, waiting for BBC to come up.\n");

  /* mcp used to wait here for a semaphore from the BBC, which makes the
   * presence of these messages somewhat "historical" */

  bputs(info, "System: BBC is up.\n");

  InitTxFrame(RxFrame);

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

  while (1) {
    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0)
      berror(err, "System: Error on BBC read");

#if 0
    static int mycounter = 0;
    static int mycounter2 = 0;

    if(GET_NODE(in_data) == 6 && GET_CH(in_data) == 6 && GET_STORE(in_data))
      printf("%08x\n", in_data);

    // DEBUG TOOLS
    if(GET_NODE(in_data) == 0x27) {
      if(GET_STORE(in_data) ) {
        mycounter++;
      } else {
        mycounter2++;
      }
    }
    if (in_data == 0xdf80eb90)  {
      if( (mycounter != 12) || (mycounter2 != 12) ) {
        printf("++++++++++++++>>>>>> mycounter = %d mycounter2 = %d\n",
            mycounter, mycounter2);
      }
      mycounter  = 0;
      mycounter2 = 0;
    }
    // END DEBUG TOOL
#endif

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
            && RxFrameIndex >= 0)
          bprintf(err, "System: Frame sequencing error detected: wanted %i, "
              "got %i\n", RxFrameIndex + 1, RxFrame[3]);
        RxFrameIndex = RxFrame[3];

        /* Save current fastsamp */
        RxFrameFastSamp = (RxFrame[1] + RxFrame[2] * 0x10000);

        UpdateBBCFrame(RxFrame);
        CommandData.bbcFifoSize = ioctl(bbc_fp, BBCPCI_IOC_BBC_FIONREAD);

        /* pushDiskFrame must be called before PushBi0Buffer to get the slow
           data right */
        pushDiskFrame(RxFrame);
#ifndef BOLOTEST
        PushBi0Buffer(RxFrame);
        FillSlowDL(RxFrame);
#endif
        zero(RxFrame);
      }
    }
  }
  return(0);
}
