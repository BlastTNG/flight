/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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

#include "command_struct.h"
#include "crc.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "small_c.h"
#include "slow_dl.h"
#include "starpos.h"
#include "tx_struct.h"
#include "tx.h"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#define STARTUP_VETO_LENGTH 250

#ifdef BOLOTEST
#  define FRAME_MARGIN (-12)
#  define USE_FIFO_CMD
#else
#  define FRAME_MARGIN (-2)
#endif

#define BI0_FRAME_BUFLEN (40)
/* Define global variables */
int bbc_fp = -1;
int bi0_fp = -2;
unsigned int debug = 0;
short int SamIAm;
pthread_key_t identity;

struct ACSDataStruct ACSData;

int RxFrameIndex;

unsigned short* slow_data[FAST_PER_SLOW];

extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];

extern pthread_mutex_t mutex;

void Pointing();
void WatchPort(void*);
void WatchDGPS(void);
void IntegratingStarCamera(void);
void WatchFIFO(void);
void FrameFileWriter(void);

void InitialiseFrameFile(char);
void dirFileWriteFrame(unsigned short *RxFrame);
void pushDiskFrame(unsigned short *RxFrame);

void SunSensor(void);

void InitSched();

FILE* logfile = NULL;

struct {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
} bi0_buffer;

unsigned short *tdrss_data[3];
unsigned int tdrss_index = 0;

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 3                /* start-of-line marker */ \
  - 24               /* date "YYYY-MM-DD HH:MM:SS GMT " */ \
  - 4                /* marker again plus NUL */ \
)

void mputs(int flag, const char* message) {
  char buffer[MPRINT_BUFFER_SIZE];
  time_t t = time(NULL);
  struct tm now;
  char local[1024];
  char *bufstart, *bufptr, *lastchr, *firstchr;
  int len;
  char marker[4];

  switch(flag) {
    case MCP_ERROR:
      strcpy(marker, "** ");
      break;
    case MCP_FATAL:
      strcpy(marker, "!! ");
      break;
    case MCP_INFO:
      strcpy(marker, "-- ");
      break;
    case MCP_SCHED:
      strcpy(marker, "## ");
      break;
    case MCP_STARTUP:
      strcpy(marker, ">> ");
      break;
    case MCP_TFATAL:
      strcpy(marker, "$$ ");
      break;
    case MCP_WARNING:
      strcpy(marker, "== ");
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

  strftime(bufstart, 1023, "%F %T GMT ", gmtime_r(&t, &now));
  strcat(buffer, marker);

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, "[%s] ", (char*)pthread_getspecific(identity));

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
      fputs(buffer, stdout);
      fflush(stdout);

      firstchr = bufptr + 1;
    }
  }

  if (flag == MCP_FATAL) {
    if (logfile != NULL) {
      fputs("!! Last error is FATAL.  Cannot continue.\n", logfile);
      fflush(logfile);
    }
    fputs("!! Last error is FATAL.  Cannot continue.\n", stdout);
    fflush(stdout);

    _exit(1);
  }

  if (flag == MCP_TFATAL) {
    if (logfile != NULL) {
      fprintf(logfile,
          "$$ Last error is THREAD FATAL.  Thread [%s] exits.\n",
          (char*)pthread_getspecific(identity));
      fflush(logfile);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [%s] exits.\n",
        (char*)pthread_getspecific(identity));
    fflush(stdout);

    pthread_exit(NULL);
  }
}

void mprintf(int flag, char* fmt, ...) {
  char message[1024];
  va_list argptr;

  va_start(argptr, fmt);
  vsnprintf(message, 1023, fmt, argptr);
  va_end(argptr);

  mputs(flag, message);
}

void merror(int flag, char* fmt, ...) {
  char message[1024];
  va_list argptr;
  int error = errno;

  va_start(argptr, fmt);
  vsnprintf(message, 1023, fmt, argptr);
  va_end(argptr);

  /* add a colon */
  strncat(message, ": ", 1023 - strlen(message));

  /* ensure the string is null terminated */
  message[1023] = '\0';

  /* copy error message into remainder of string -- Note: sterror is reentrant
   * despite what STRERROR(3) insinuates (and strerror_r is horribly b0rked) */
  strcat(message, strerror(error));

  mputs(flag, message);
}

void SensorReader(void) {
  int data;
  int nr;
  struct statvfs vfsbuf;

  FILE *stream;

  pthread_setspecific(identity, "sens");
  mputs(MCP_STARTUP, "SensorReader startup\n");

  while (1) {
    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/temp1_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp1 = data / 10;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read temp1 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/temp2_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp2 = data / 10;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read temp2 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/temp3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp3 = data / 10;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read temp3 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/0-0290/fan3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.fan = data;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read fan3 from I2C bus");

    if (statvfs("/data", &vfsbuf))
      merror(MCP_WARNING, "Cannot stat filesystem");
    else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bavail / 1000;
    }
    sleep(1);
  }
}

void GetACS(unsigned short *RxFrame){
  double enc_elev, gyro1, gyro2, gyro3;
  double x_comp, y_comp, bias;
  static struct BiPhaseStruct* gyro1Addr;
  static struct BiPhaseStruct* gyro2Addr;
  static struct BiPhaseStruct* gyro3Addr;
  static struct BiPhaseStruct* encElevAddr;
  static struct BiPhaseStruct* clinElevAddr;
  static struct BiPhaseStruct* magXAddr;
  static struct BiPhaseStruct* magYAddr;
  static struct BiPhaseStruct* magBiasAddr;
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
    magBiasAddr = GetBiPhaseAddr("mag_bias");
  }

  rx_frame_index = ((RxFrame[1] & 0x0000ffff) |
      (RxFrame[2] & 0x0000ffff) << 16);

  if ((enc_elev = ((double)RxFrame[encElevAddr->channel] * (-360.0 / 65536.0)
          + ENC_ELEV_OFFSET)) < 0)
    enc_elev += 360;

  gyro1 = (double)(RxFrame[gyro1Addr->channel]-GYRO1_OFFSET) * ADU1_TO_DPS;
  gyro2 = (double)(RxFrame[gyro2Addr->channel]-GYRO2_OFFSET) * ADU2_TO_DPS;
  gyro3 = (double)(RxFrame[gyro3Addr->channel]-GYRO3_OFFSET) * ADU3_TO_DPS;

  bias = (double)(RxFrame[magBiasAddr->channel]);
  x_comp = (double)(RxFrame[magXAddr->channel]);
  y_comp = (double)(RxFrame[magYAddr->channel]);

  i_ss = ss_index;

  ACSData.t = time(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.enc_elev = enc_elev;
  ACSData.gyro1 = gyro1;
  ACSData.gyro2 = gyro2;
  ACSData.gyro3 = gyro3;
  ACSData.mag_x = x_comp;
  ACSData.mag_y = y_comp;

  ACSData.clin_elev = (double)RxFrame[clinElevAddr->channel];

}

void FillSlowDL(unsigned short *RxFrame) {
  static char firsttime = 1;
  int i;
  unsigned short msb, lsb;
  struct NiosStruct *address;

  if (firsttime) {
    firsttime = 0;
    for (i = 0; i < SLOWDL_NUM_DATA; i++) {
      address = GetNiosAddr(SlowDLInfo[i].src);
      SlowDLInfo[i].wide = address->wide;
      SlowDLInfo[i].mindex = BiPhaseLookup[BI0_MAGIC(address->bbcAddr)].index;
      SlowDLInfo[i].chnum = BiPhaseLookup[BI0_MAGIC(address->bbcAddr)].channel;
    }
  }

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
        lsb = slow_data[SlowDLInfo[i].mindex][SlowDLInfo[i].chnum + 1];
      else
        msb = 0;
      SlowDLInfo[i].value = (double)((msb << 16) | lsb);
    }
  }
}

/* fill_Rx_frame: places one 32 bit word into the RxFrame. Returns true on
 * success */
int fill_Rx_frame(unsigned int in_data,
    unsigned short *RxFrame) {
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

void WatchDog (void) {
  pthread_setspecific(identity, "wdog");
  mputs(MCP_STARTUP, "Watchdog startup\n");

  if (ioperm(0x378, 0x0F, 1) != 0)
    merror(MCP_TFATAL, "Error setting watchdog permissions");
  ioperm(0x80, 1, 1);

  for (;;) {
    outb(0xAA, 0x378);
    usleep(4000);
    outb(0x55, 0x378);
    usleep(4000);
  }
}

void write_to_biphase(unsigned short *RxFrame) {
  int i;
  static unsigned short nothing[BI0_FRAME_SIZE];
  static unsigned short sync = 0xEB90;

  if (bi0_fp == -2) {
    bi0_fp = open("/dev/bi0_pci", O_RDWR);
    if (bi0_fp == -1)
      merror(MCP_TFATAL, "Error opening biphase device");

    for (i = 0; i < 1024; i++)
      nothing[i] = 0xEEEE;
  }

  nothing[0] = CalculateCRC(0xEB90, RxFrame, BiPhaseFrameWords);

  if (bi0_fp >= 0) {
    RxFrame[0] = sync;
    sync = ~sync;
    if (write(bi0_fp, RxFrame, BiPhaseFrameWords * sizeof(unsigned short)) < 0)
      merror(MCP_ERROR, "bi-phase write (RxFrame) failed");
    if (write(bi0_fp, nothing,
          (BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(unsigned short)) < 0)
      merror(MCP_ERROR, "bi-phase write (padding) failed");
    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);
  }
}

void InitBi0Buffer() {
  int i;

  bi0_buffer.i_in = 10; /* preload the fifo */
  bi0_buffer.i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++) {
    if ((bi0_buffer.framelist[i] = malloc(BiPhaseFrameWords *
            sizeof(unsigned short))) == NULL)
      merror(MCP_FATAL, "Unable to malloc for bi-phase");
  }
}

void PushBi0Buffer(unsigned short *RxFrame) {
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

void zero(unsigned short *RxFrame) {
  int i;

  for (i = 0; i < SLOW_OFFSET + slowsPerBi0Frame; i++)
    RxFrame[i] = 0;
}

void BiPhaseWriter(void) {
  int i_out, i_in;

  pthread_setspecific(identity, "bi0 ");
  mputs(MCP_STARTUP, "Biphase writer startup\n");

  while (1) {
    i_in = bi0_buffer.i_in;
    i_out = bi0_buffer.i_out;
    while (i_out != i_in) {
      i_out++;
      if (i_out>=BI0_FRAME_BUFLEN)
        i_out = 0;
      write_to_biphase(bi0_buffer.framelist[i_out]);
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
int IsNewFrame(unsigned int d) {
  static int first_bof = 1;
  int is_bof;
  is_bof = (d == (BBC_FSYNC | BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | 0xEB90));
  if (is_bof && first_bof) {
    is_bof = first_bof = 0;
  }

  return is_bof;
}

/* Identity crisis: am I frodo or sam? */
int AmISam(void) {
  char buffer[2];

  if (gethostname(buffer, 1) == -1 && errno != ENAMETOOLONG) {
    merror(MCP_ERROR, "Unable to get hostname");
  }

  return (buffer[0] == 's') ? 1 : 0;
}

/* Signal handler called when we get a hup, int or term */
void CloseBBC(int signo) {
  mprintf(MCP_ERROR, "Caught signal %i, closing BBC and Bi0", signo);
  if (bi0_fp >= 0)
    close(bi0_fp);
  if (bbc_fp >= 0)
    close(bbc_fp);

  /* restore default handler and raise the signal again */
  signal(signo, SIG_DFL);
  raise(signo);
}

char segvregs[100];
int segvcnt = 0;
void SegV(int signo) {
  fprintf(stderr, "SEGV caught: %s\n", segvregs);
  raise(SIGTERM);
}

int main(int argc, char *argv[]) {
  unsigned int in_data, i;
  unsigned short* RxFrame;
  int StartupVeto = STARTUP_VETO_LENGTH + 1;

  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t watchdog_id;

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

  /********** DEBUG TOOL ***************/
  int mycounter = 0;
  int mycounter2 = 0;
  /********** DEBUG TOOL ***************/


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

  pthread_key_create(&identity, NULL);
  pthread_setspecific(identity, "mcp ");

  if ((logfile = fopen("/data/etc/mcp.log", "a")) == NULL)
    merror(MCP_ERROR, "Can't open log file");
  else
    fputs("----- LOG RESTART -----\n", logfile);

  mputs(MCP_STARTUP, "MCP startup");

  /* Watchdog */
  pthread_create(&watchdog_id, NULL, (void*)&WatchDog, NULL);

  if ((bbc_fp = open("/dev/bbcpci", O_RDWR)) < 0)
    merror(MCP_FATAL, "Error opening BBC");

  /* Initialize the Ephemeris */
  ReductionInit();

  InitCommandData();

  pthread_mutex_init(&mutex, NULL);

  MakeAddressLookups();

#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

#ifndef BOLOTEST
  pthread_create(&dgps_id, NULL, (void*)&WatchDGPS, NULL);
  pthread_create(&isc_id, NULL, (void*)&IntegratingStarCamera, (void*)0);
  pthread_create(&osc_id, NULL, (void*)&IntegratingStarCamera, (void*)1);

  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  pthread_create(&sunsensor_id, NULL, (void*)&SunSensor, NULL);

  InitBi0Buffer();
#endif

  InitialiseFrameFile(argv[1][0]);

  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  memset(PointingData, 0, 3*sizeof(struct PointingDataStruct));

  /* Allocate the local data buffers */
  if ((RxFrame = malloc(BiPhaseFrameSize)) == NULL)
    merror(MCP_FATAL, "Unable to malloc RxFrame");

  for (i = 0; i < 3; ++i)
    if ((tdrss_data[i] = (unsigned short *)malloc(BiPhaseFrameSize)) == NULL)
      merror(MCP_FATAL, "Unable to malloc tdrss data buffer");

  for (i = 0; i < FAST_PER_SLOW; ++i)
    if ((slow_data[i] = malloc(slowsPerBi0Frame * sizeof(unsigned short)))
        == NULL)
      merror(MCP_FATAL, "Unable to malloc slow data buffer");

  CommandData.tdrssVeto = 0;
#ifndef BOLOTEST
  if (CommandData.tdrssVeto)
    mputs(MCP_WARNING, "The TDRSS writer has been VETOed.");
  else
    pthread_create(&tdrss_id, NULL, (void*)&TDRSSWriter, NULL);
#endif

  /* Find out whether I'm frodo or sam */
  SamIAm = AmISam();

  if (SamIAm)
    mputs(MCP_INFO, "I am Sam.\n");
  else 
    mputs(MCP_INFO, "I am not Sam.\n");

  InitSched();

  mputs(MCP_INFO, "Finished Initialisation, waiting for BBC to come up.\n");

  mputs(MCP_INFO, "BBC is up.\n");

#ifndef BOLOTEST
  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
#endif

  InitTxFrame();

  while (1) {

    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0) 
      merror(MCP_ERROR, "Error on BBC read");


    //if(GET_NODE(in_data) == 6 && GET_CH(in_data) == 6 && GET_STORE(in_data))
    //  printf("%08x\n", in_data);

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
//        printf("++++++++++++++>>>>>> mycounter = %d mycounter2 = %d\n", mycounter, mycounter2);
      }
      mycounter  = 0;
      mycounter2 = 0;
    }
    // END DEBUG TOOL

    if (!fill_Rx_frame(in_data, RxFrame))
      mputs(MCP_ERROR, "Unrecognised word received from BBC");

    if (IsNewFrame(in_data)) {
      if (StartupVeto > 1)
        --StartupVeto;
      else {
#ifndef BOLOTEST
        GetACS(RxFrame);
        Pointing();

        /* Copy data to tdrss thread. */
        memcpy(tdrss_data[tdrss_index], RxFrame, BiPhaseFrameSize);
        tdrss_index = INC_INDEX(tdrss_index);
#endif

        /* Frame sequencing check */
        if (StartupVeto) {
          mputs(MCP_INFO, "Startup Veto Ends\n");
          StartupVeto = 0;
        } else if (RxFrame[3] != (RxFrameIndex + 1) % FAST_PER_SLOW
            && RxFrameIndex >= 0)
          mprintf(MCP_ERROR,
              "Frame sequencing error detected: wanted %i, got %i\n",
              RxFrameIndex + 1, RxFrame[3]);
        RxFrameIndex = RxFrame[3];

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
