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
#include "bbc_pci.h"

#include "tx_struct.h"
#include "tx.h"

#include "pointing_struct.h"
#include "command_struct.h"
#include "starpos.h"
#include "mcp.h"
#include "small_c.h"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#ifdef BOLOTEST
#  define FRAME_MARGIN (-12)
#  define USE_FIFO_CMD
#else
#  define FRAME_MARGIN (-2)
#endif

#define BI0_FRAME_BUFLEN (40)
/* Define global variables */
/* new comment */

int bbc_fp = -1;
int bi0_fp = -2;
int frames_in = 0;

struct ACSDataStruct ACSData;

int RxFrameIndex;

short int SamIAm;

unsigned short* slow_data[FAST_PER_SLOW];

extern struct SlowDLStruct SlowDLInfo[N_SLOWDL];

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

unsigned short *smalldata[3];
unsigned int small_index = 0;

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
  int len, pid;
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

  sprintf(bufstart, "[%li] ", pthread_self());

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
    pid = getpid();
    if (logfile != NULL) {
      fprintf(logfile,
          "$$ Last error is THREAD FATAL.  Thread [pid = %i] exits.\n",
          pid);
      fflush(logfile);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [pid = %i] exits.\n", pid);
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

  mputs(MCP_STARTUP, "SensorReader startup\n");

  while (1) {
    if ((stream = fopen("/sys/bus/i2c/devices/3-0290/temp1_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp1 = data / 10;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read temp1 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/3-0290/temp2_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp2 = data / 10;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read temp2 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/3-0290/temp3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp3 = data / 10;
      fclose(stream);
    } else
      merror(MCP_WARNING, "Cannot read temp3 from I2C bus");

    if ((stream = fopen("/sys/bus/i2c/devices/3-0290/fan3_input", "r"))
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

#if 0
void FillSlowDL(unsigned short *RxFrame) {
  static char firsttime = 1;
  int i, j, k;

  pthread_mutex_lock(&mutex);

  if (firsttime) {
    firsttime = 0;
    for (i = 0; i < N_SLOWDL; i++) {
      for (j = 0; j < N_FASTCHLIST; j++) {
        if (strcmp(FastChList[j].field, SlowDLInfo[i].src) == 0) {
          SlowDLInfo[i].m_c2e = FastChList[j].m_c2e;
          SlowDLInfo[i].b_e2e = FastChList[j].b_e2e;
          SlowDLInfo[i].ctype = FastChList[j].type;
          SlowDLInfo[i].ind1 = j + FAST_OFFSET;
        }
      }

      for (j = 0; j < N_SLOW; j++) {
        for (k = 0; k < FAST_PER_SLOW; k++) {
          if (strcmp(SlowChList[j][k].field, SlowDLInfo[i].src) == 0) {
            SlowDLInfo[i].m_c2e = SlowChList[j][k].m_c2e;
            SlowDLInfo[i].b_e2e = SlowChList[j][k].b_e2e;
            SlowDLInfo[i].ctype = SlowChList[j][k].type;
            SlowDLInfo[i].ind1 = j;
            SlowDLInfo[i].ind2 = k;
          }
        }
      }
    }
  }

  for (i = 0; i < N_SLOWDL; i++) {
    if ((SlowDLInfo[i].ind2 >= 0 && RxFrame[3] == SlowDLInfo[i].ind2)
        || SlowDLInfo[i].ind2 < 0) {
      switch (SlowDLInfo[i].ctype) {
        case 's':
          SlowDLInfo[i].value =
            (double)((signed short) RxFrame[SlowDLInfo[i].ind1 + 4]);
          break;
        case 'u':
          SlowDLInfo[i].value = (double)(RxFrame[SlowDLInfo[i].ind1 + 4]);
          break;
        case 'i': case 'S':
          SlowDLInfo[i].value =
            (double)((signed int) ((RxFrame[SlowDLInfo[i].ind1 + 4] << 16) +
                                   RxFrame[SlowDLInfo[i].ind1 + 5]));
          break;
        case 'U':
          SlowDLInfo[i].value =
            (double)((unsigned int) ((RxFrame[SlowDLInfo[i].ind1 + 4] << 16) +
                                     RxFrame[SlowDLInfo[i].ind1 + 5]));
          break;
      }
      SlowDLInfo[i].value = SlowDLInfo[i].value * SlowDLInfo[i].m_c2e
        + SlowDLInfo[i].b_e2e;
    }
  }
  pthread_mutex_unlock(&mutex);
}
#endif

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

  if (bi0_fp == -2) {
    bi0_fp = open("/dev/bi0_pci", O_RDWR);
    if (bi0_fp == -1)
      merror(MCP_TFATAL, "Error opening biphase device");

    for (i = 0; i < 1024; i++)
      nothing[i] = 0xEEEE;
  }

  int n;
  if (bi0_fp >= 0) {
    RxFrame[0] = 0xEB90;
    if ((n = write(bi0_fp, RxFrame, BiPhaseFrameWords * sizeof(unsigned short))) < 0)
      merror(MCP_ERROR, "bi-phase write (RxFrame) failed");
    if ((n = write(bi0_fp, nothing,
          (BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(unsigned short))) < 0)
      merror(MCP_ERROR, "bi-phase write (padding) failed");
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
  int i, fw;

  fw = FAST_OFFSET;

  for (i = 0; i<fw; i++) {
    RxFrame[i] = 0;
  }
}

void BiPhaseWriter(void) {
  int i_out, i_in;

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

  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t sunsensor_id;
  pthread_t watchdog_id;

#ifndef USE_FIFO_CMD
  pthread_t CommandDatacomm2;
#endif

#ifndef BOLOTEST
  pthread_t small_id;
  pthread_t bi0_id;
  pthread_t sensors_id;
  pthread_t dgps_id;
  pthread_t isc_id;
  pthread_t osc_id;
#endif

  struct CommandDataStruct CommandData_loc;

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
    merror(MCP_ERROR, "Can't open log file");
  else
    fputs("----- LOG RESTART -----\n", logfile);

  mputs(MCP_STARTUP, "MCP startup");

  if ((bbc_fp = open("/dev/bbcpci", O_RDWR)) < 0)
    merror(MCP_FATAL, "Error opening BBC");


  //Initialize the Ephemeris
  ReductionInit();

  InitCommandData();

  pthread_mutex_init(&mutex, NULL);

  MakeAddressLookups();

  /* Watchdog */
  pthread_create(&watchdog_id, NULL, (void*)&WatchDog, NULL);

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

  //  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
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
    if ((smalldata[i] = (unsigned short *)malloc(BiPhaseFrameSize)) == NULL)
      merror(MCP_FATAL, "Unable to malloc smalldata");

  for (i = 0; i < FAST_PER_SLOW; ++i)
    if ((slow_data[i] = malloc(slowsPerBi0Frame * sizeof(unsigned short)))
        == NULL)
      merror(MCP_FATAL, "Unable to malloc slow data buffer");

#ifndef BOLOTEST
  pthread_create(&small_id, NULL, (void*)&smallinit, NULL);
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
    pthread_mutex_lock(&mutex);
    CommandData_loc = CommandData;
    pthread_mutex_unlock(&mutex);

    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) < 0)
      merror(MCP_ERROR, "Error on BBC read");

    if (!fill_Rx_frame(in_data, RxFrame))
      mputs(MCP_ERROR, "Unrecognised word received from BBC");

    if (IsNewFrame(in_data)) {
      frames_in++;
#ifndef BOLOTEST
      GetACS(RxFrame);
      Pointing();

      /* Copy data to small. */
      memcpy(smalldata[small_index], RxFrame, BiPhaseFrameSize);
      small_index = INC_INDEX(small_index);
#endif

      /* Frame sequencing check */
      if (RxFrame[3] != (RxFrameIndex + 1) % FAST_PER_SLOW && RxFrameIndex >= 0)
        mprintf(MCP_ERROR,
            "Frame sequencing error detected: wanted %i, got %i\n",
            RxFrameIndex + 1, RxFrame[3]);
      RxFrameIndex = RxFrame[3];

      UpdateBBCFrame(RxFrame);

#ifndef BOLOTEST
      PushBi0Buffer(RxFrame);
#endif
#if 0
      FillSlowDL(RxFrame);
#endif
      pushDiskFrame(RxFrame);
      zero(RxFrame);
    }
  }
  return(0);
}
