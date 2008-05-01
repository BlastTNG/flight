/* mcp: the Spider master control program
 *
 * mcp.c: contains the main loop and creates all threads used by mcp
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
#include "mcp.h"
#include "channels.h"
#include "tx.h"
#include "command_list.h"
#include "command_struct.h"
#include "slow_dl.h"
#include "pointing_struct.h"
#include "crc.h"

# define NOSTARCAM

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#define STARTUP_VETO_LENGTH 250 /* "frames" */
#define BI0_VETO_LENGTH 5 /* seconds */

#define BI0_FIFO_MARGIN 8750 /* bytes = 7 frames */
#define BI0_FIFO_MINIMUM (BI0_FIFO_MARGIN / 2)

#define FRAME_MARGIN (-2)

#define BI0_FRAME_BUFLEN (400)
/* Define global variables */
int bbc_fp = -1;
unsigned int debug = 0;
short int SamIAm;
unsigned int RxFrameFastSamp;
unsigned short* slow_data[FAST_PER_SLOW];
pthread_t watchdog_id;

static int bi0_fp = -2;
static int StartupVeto = STARTUP_VETO_LENGTH + 1;
static int Death = -STARTUP_VETO_LENGTH * 2;
static int RxFrameIndex;

extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];

extern pthread_mutex_t mutex; //commands.c

struct ReactWheelStruct RWData;
#define RW_FILTER_LEN 20
 
#ifdef HAVE_ACS
struct ACSDataStruct ACSData;
void Pointing();    //pointing.c
void openMotors(); //motors.c
void closeMotors();
void openCamera();  //starcamera.c
#endif

void WatchFIFO();   //commands.c
void WatchPort(void* param);

void FrameFileWriter(void);  //framefile.c
void InitialiseFrameFile(char);
void dirFileWriteFrame(unsigned short *RxFrame);
void pushDiskFrame(unsigned short *RxFrame);

static FILE* logfile = NULL;

static struct {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
} bi0_buffer;

time_t biphase_timer;
int biphase_is_on = 0;

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

/* gives system time (in s) */
time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}

/* I/O function to be used by bputs, bprintf, etc.
 * it is assigned this purpose in main
 */
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


#ifdef HAVE_ACS
/* fills ACSData struct from current frame */
static void GetACS(unsigned short *RxFrame)
{
  double enc_table;
  double gyro1, gyro2, gyro3, gyro4, gyro5, gyro6;
  unsigned int uTab, ugy1, ugy2, ugy3, ugy4, ugy5, ugy6;
  static struct BiPhaseStruct* gyro1Addr;
  static struct BiPhaseStruct* gyro2Addr;
  static struct BiPhaseStruct* gyro3Addr;
  static struct BiPhaseStruct* gyro4Addr;
  static struct BiPhaseStruct* gyro5Addr;
  static struct BiPhaseStruct* gyro6Addr;
  static struct BiPhaseStruct* encTableAddr;
  
  unsigned int rx_frame_index = 0;
  
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    encTableAddr = GetBiPhaseAddr("enc_table");
#if 0
    //using raw (4-stage filtered) gyros adds 0.02s latency
    gyro1Addr = GetBiPhaseAddr("raw_gy1");
    gyro2Addr = GetBiPhaseAddr("raw_gy2");
    gyro3Addr = GetBiPhaseAddr("raw_gy3");
    gyro4Addr = GetBiPhaseAddr("raw_gy4");
    gyro5Addr = GetBiPhaseAddr("raw_gy5");
    gyro6Addr = GetBiPhaseAddr("raw_gy6");
#endif
    gyro1Addr = GetBiPhaseAddr("gyro1");
    gyro2Addr = GetBiPhaseAddr("gyro2");
    gyro3Addr = GetBiPhaseAddr("gyro3");
    gyro4Addr = GetBiPhaseAddr("gyro4");
    gyro5Addr = GetBiPhaseAddr("gyro5");
    gyro6Addr = GetBiPhaseAddr("gyro6");
  }

  rx_frame_index = ((RxFrame[1] & 0x0000ffff) |
      (RxFrame[2] & 0x0000ffff) << 16);

  uTab = (RxFrame[encTableAddr->channel+1] << 16 | 
      RxFrame[encTableAddr->channel]);
  if ((enc_table = ((double)uTab * (360.0 / 144000.0)
          + ENC_ELEV_OFFSET)) < 0)
    enc_table += 360;
  else if (enc_table > 360.0) enc_table -= 360;

#if 0
  ugy1 = (RxFrame[gyro1Addr->channel+1] << 16 | RxFrame[gyro1Addr->channel]);
  ugy2 = (RxFrame[gyro2Addr->channel+1] << 16 | RxFrame[gyro2Addr->channel]);
  ugy3 = (RxFrame[gyro3Addr->channel+1] << 16 | RxFrame[gyro3Addr->channel]);
  ugy4 = (RxFrame[gyro4Addr->channel+1] << 16 | RxFrame[gyro4Addr->channel]);
  ugy5 = (RxFrame[gyro5Addr->channel+1] << 16 | RxFrame[gyro5Addr->channel]);
  ugy6 = (RxFrame[gyro6Addr->channel+1] << 16 | RxFrame[gyro6Addr->channel]);
  gyro1 = (double)(ugy1-DGY32_OFFSET) * DGY32_TO_DPS;
  gyro2 = (double)(ugy2-DGY32_OFFSET) * DGY32_TO_DPS;
  gyro3 = (double)(ugy3-DGY32_OFFSET) * DGY32_TO_DPS;
  gyro4 = (double)(ugy4-DGY32_OFFSET) * DGY32_TO_DPS;
  gyro5 = (double)(ugy5-DGY32_OFFSET) * DGY32_TO_DPS;
  gyro6 = (double)(ugy6-DGY32_OFFSET) * DGY32_TO_DPS;
#endif
  ugy1 = RxFrame[gyro1Addr->channel];
  ugy2 = RxFrame[gyro2Addr->channel];
  ugy3 = RxFrame[gyro3Addr->channel];
  ugy4 = RxFrame[gyro4Addr->channel];
  ugy5 = RxFrame[gyro5Addr->channel];
  ugy6 = RxFrame[gyro6Addr->channel];
  gyro1 = (double)(ugy1-GY16_OFFSET) * GY16_TO_DPS;
  gyro2 = (double)(ugy2-GY16_OFFSET) * GY16_TO_DPS;
  gyro3 = (double)(ugy3-GY16_OFFSET) * GY16_TO_DPS;
  gyro4 = (double)(ugy4-GY16_OFFSET) * GY16_TO_DPS;
  gyro5 = (double)(ugy5-GY16_OFFSET) * GY16_TO_DPS;
  gyro6 = (double)(ugy6-GY16_OFFSET) * GY16_TO_DPS;

  ACSData.t = mcp_systime(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.enc_table = enc_table;
  ACSData.gyro1 = gyro1;
  ACSData.gyro2 = gyro2;
  ACSData.gyro3 = gyro3;
  ACSData.gyro4 = gyro4;
  ACSData.gyro5 = gyro5;
  ACSData.gyro6 = gyro6;

}
#endif //HAVE_ACS
static void GetRW(unsigned short *RxFrame)
{
  double rwCur, rwVelD;
  short int uCur, uVel;
  static double buf_rwVel[RW_FILTER_LEN]; // Buffer for RW boxcar filter
  static unsigned int ib_last=0;
  static double a=0.0; // Boxcar filter accumulator
  static struct BiPhaseStruct* rwCurAddr;
  static struct BiPhaseStruct* rwVelAddr;
  static int firsttime = 1;
  char fouts[12*(RW_FILTER_LEN+1)+2];
  int i;

  if (firsttime) {
    firsttime = 0;
    rwCurAddr = GetBiPhaseAddr("rwheel_cur");
    rwVelAddr = GetBiPhaseAddr("rwheel_vel");
    // Initialize the buffer.  Assume all zeros to begin
    for(i=0;i>(RW_FILTER_LEN-1);i++) buf_rwVel[i]=0.0;
  }
  uCur = RxFrame[rwCurAddr->channel];
  uVel = RxFrame[rwVelAddr->channel];

  rwCur = ((double)(uCur)) * RWCUR_TO_DPS+RWCUR_OFFSET;
  rwVelD = ((double)(uVel)) * RWCTS_TO_VEL;
  RWData.i=rwCur;

  //  Boxcar Filter the RW Velocity
  a+=(rwVelD-buf_rwVel[ib_last]);
  buf_rwVel[ib_last]=rwVelD;
  ib_last=(ib_last+RW_FILTER_LEN+1)%RW_FILTER_LEN;
  RWData.vel=a/((double)RW_FILTER_LEN); 
}

/* fill_Rx_frame: places one 32 bit word into the RxFrame. 
 * Returns true on success 
 */
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

static int write_to_biphase(unsigned short *RxFrame, int i_in, int i_out)
{
  int i;
  static unsigned short nothing[BI0_FRAME_SIZE];
  static unsigned short sync = 0xEB90;
  static unsigned int do_skip = 0;

  if (bi0_fp == -2) {
    bi0_fp = open("/dev/bi0_pci", O_RDWR);
    if (bi0_fp == -1)
      berror(tfatal, "BiPhase Writer: Error opening biphase device");

    for (i = 0; i < BI0_FRAME_SIZE; i++)
      nothing[i] = 0xEEEE;
  }
  i_out = (i_out + 1) % BI0_FRAME_BUFLEN;

  nothing[0] = CalculateCRC(0xEB90, RxFrame, BiPhaseFrameWords);

  //  if (bi0_fp >= 0 && InCharge) {
  if (bi0_fp >= 0) { // lmf: We don't have an in charge computer right now.  Later we'll 
                     // have to add this in.
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
        berror(err, "BiPhase Writer: bi-phase write for RxFrame failed");
      else if (i != BiPhaseFrameWords * sizeof(unsigned short))
        bprintf(err, "BiPhase Writer: Short write for RxFrame: %i of %i", i,
		BiPhaseFrameWords * sizeof(unsigned short));

      i = write(bi0_fp, nothing, (BI0_FRAME_SIZE - BiPhaseFrameWords) *
		sizeof(unsigned short));
      if (i < 0)
        berror(err, "BiPhase Writer: bi-phase write for padding failed");
      else if (i != (BI0_FRAME_SIZE - BiPhaseFrameWords)
	       * sizeof(unsigned short))
        bprintf(err, "BiPhase Writer: Short write for padding: %i of %i", i,
		(BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(unsigned short));
    }

#if 0
    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);
    if (do_skip == 0 && CommandData.bi0FifoSize > BI0_FIFO_MARGIN) {
      do_skip = (CommandData.bi0FifoSize - BI0_FIFO_MINIMUM) / BI0_FRAME_SIZE;
      bprintf(warning, "BiPhase Writer: Excess data in FIFO, discarding "
	      "%i frames out of hand.", do_skip);
    }
#endif
  }

  return i_out;
}

/* initialize the bi0 buffer */
static void InitBi0Buffer()
{
  int i;

  bi0_buffer.i_in = 10; /* preload the fifo */
  bi0_buffer.i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++)
    bi0_buffer.framelist[i] = balloc(fatal, BiPhaseFrameWords *
        sizeof(unsigned short));
}

/* assigns RxFrame to next entry in bi0 buffer's framelist */
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

/* zeros RxFrame */
static void zero(unsigned short *RxFrame)
{
  int i;

  for (i = 0; i < SLOW_OFFSET + slowsPerBi0Frame; i++)
    RxFrame[i] = 0;
}

static void BiPhaseWriter(void)
{
  int i_out, i_in;

  bputs(startup, "Biphase Writer: Startup\n");

  while (!biphase_is_on)
    usleep(10000);

  bputs(info, "BiPhase Writer: Veto has ended.  Here we go.\n");

  while (1) {
    i_in = bi0_buffer.i_in;
    i_out = bi0_buffer.i_out;
    if (i_out == i_in) {
      /* Death meausres how long the BiPhaseWriter has gone without receiving
       * any data -- an indication that we aren't receiving FSYNCs from the
       * BLASTBus anymore */
      //      if (InCharge) {
        if (++Death == 25) {
          bprintf(err, "BiPhase Writer: Death is reaping the watchdog tickle.");
        //  pthread_cancel(watchdog_id);
        }
	//      }
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
#ifdef HAVE_ACS
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
  //close peripheral communications
  
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

  pthread_t disk_id;
  pthread_t CommandDatacomm1;
  pthread_t CommandDatacomm2;     //2nd needed for SIP, not for FIFO
  pthread_t bi0_id;

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

  biphase_timer = mcp_systime(NULL) + BI0_VETO_LENGTH;

  /* register the output function */
  buos_use_func(mputs);

#if (TEMPORAL_OFFSET > 0)
  bprintf(warning, "System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  bputs(startup, "System: Startup");


  if ((bbc_fp = open("/dev/bbcpci", O_RDWR)) < 0)
    berror(fatal, "System: Error opening BBC");

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  MakeAddressLookups();  //nios addresses, based off of tx_struct, derived

#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);
#else
#warning "Trying to talk with SIP(ulator), do you have it set up?"
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

  InitSlowDL();

  InitBi0Buffer();

  InitialiseFrameFile(argv[1][0]);

  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  /* Allocate the local data buffers */
  RxFrame = balloc(fatal, BiPhaseFrameSize);

  for (i = 0; i < FAST_PER_SLOW; ++i)
    slow_data[i] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));

  /* Find out whether I'm frodo or sam */
  SamIAm = AmISam();

  if (SamIAm)
    bputs(info, "System: I am Sam.\n");
  else
    bputs(info, "System: I am not Sam.\n");

#ifdef HAVE_ACS
  openMotors();  //open communications with peripherals, creates threads
  openCamera();  //also creates a thread
#endif

  bputs(info, "System: Finished Initialisation, waiting for BBC to come up.\n");

  /* mcp used to wait here for a semaphore from the BBC, which makes the
   * presence of these messages somewhat "historical" */

  bputs(info, "System: BBC is up.\n");

  InitTxFrame(RxFrame);
  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);

  FILE* logstream = fopen("/data/etc/bbc_log", "wt");

  while (1) {  //main loop
    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0)
      berror(err, "System: Error on BBC read");

    fprintf(logstream, "%08x\n", in_data);

    if (!fill_Rx_frame(in_data, RxFrame))
      bprintf(err, "System: Unrecognised word received from BBC (%08x)",
          in_data);

    if (IsNewFrame(in_data)) {
      if (StartupVeto > 1) {
        --StartupVeto;
      } else {
#ifdef HAVE_ACS
        GetACS(RxFrame);
        Pointing();
#endif

        /* Read in the Reaction Wheel velocity and current */     
        GetRW(RxFrame);

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

        UpdateBBCFrame(RxFrame);  //major part of tx.c
//        CommandData.bbcFifoSize = ioctl(bbc_fp, BBCPCI_IOC_BBC_FIONREAD);

        /* pushDiskFrame must be called before PushBi0Buffer to get the slow
           data right */
        pushDiskFrame(RxFrame);
        if (biphase_is_on)
          PushBi0Buffer(RxFrame);
        else if (biphase_timer < mcp_systime(NULL))
          biphase_is_on = 1;

	FillSlowDL(RxFrame);

        zero(RxFrame);
      }
    }
  }
  return(0);
}
