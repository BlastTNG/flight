#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include "bbc.h"

#include "tx_struct.h"
#include "tx.h"

#include "pointing_struct.h"
#include "command_struct.h"
#include "starpos.h"
#include "mcp.h"

#define ARIEN "192.168.1.99"
#define ARIEN_PORT 8645

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#ifdef BOLOTEST
#  define FRAME_MARGIN (-12)
#else
#  define FRAME_MARGIN (-2)
#endif

#define BI0_FRAME_BUFLEN (40)
/* Define global variables */

int bbc_fp;
int frames_in = 0;

struct SunSensorDataStruct SunSensorData[3];

struct ACSDataStruct ACSData;

int ss_index = 0;
int sigPipeRaised = 0;
int RxFrameIndex;

short int SamIAm;

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

void MakeTxFrame(void);

void InitSched();

FILE* logfile = NULL;

struct {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
} bi0_buffer;

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
  
  sprintf(bufstart, "[%i] ", getpid());
  
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

    exit(1);
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

  /* copy error message into remainder of string -- Note: sterror is thread-safe despite
   * what STRERROR(3) insinuates (and strerror_r is horribly b0rked) */
  strcat(message, strerror(error));

  mputs(flag, message);
}

void SensorReader(void) {
  int fan, T;
  int nr, df;

  FILE *fp;

  mputs(MCP_STARTUP, "SensorReader startup\n");

  while (1) {
    fp = fopen("/data/rawdir/sensors.dat", "r");
    if (fp !=NULL) {
      nr = fscanf(fp, "%d %d\n", &fan, &T);
      if (nr == 2) {
        CommandData.T = T;
        CommandData.fan = fan;
      }
      fclose(fp);
    }
    if ((fp = fopen("/data/rawdir/df.dat", "r")) != NULL) {
      nr = fscanf(fp, "%d\n", &df);
      if (nr == 1) {
        CommandData.df = df;
      }
      fclose(fp);
    }
    sleep(1);
  }
}

/* The SIG is raised in the SunSensor thread, which doesn't have to do anything
 * else when it gets the signal but die (and tell someone that it did)... */
void SigPipe(int signal) {
  sigPipeRaised = 1;

  /* exit so mcp can respawn the thread */
  mputs(MCP_TFATAL, "SIGPIPE raised.\n");
}

void GetACS(unsigned short *RxFrame){
  double enc_elev, gyro1, gyro2, gyro3;
  double x_comp, y_comp, bias;
  static int i_GYRO1 = -1;
  static int i_GYRO2 = -1;
  static int i_GYRO3 = -1;
  static int i_enc_elev = -1;
  static int i_clin_elev = -1;
  static int i_mag_x = -1;
  static int i_mag_y = -1;
  static int i_mag_bias = -1;
  unsigned int rx_frame_index = 0;
  int i_ss;

  if (i_enc_elev == -1) {
    FastChIndex("enc_elev", &i_enc_elev);
    FastChIndex("clin_elev", &i_clin_elev);
    FastChIndex("gyro1", &i_GYRO1);
    FastChIndex("gyro2", &i_GYRO2);
    FastChIndex("gyro3", &i_GYRO3);
    FastChIndex("mag_x", &i_mag_x);
    FastChIndex("mag_y", &i_mag_y);
    FastChIndex("mag_bias", &i_mag_bias);
  }

  rx_frame_index = ((RxFrame[1] & 0x0000ffff) |
      (RxFrame[2] & 0x0000ffff) << 16);

  enc_elev = ((double)RxFrame[i_enc_elev] *
      (-360.0 / 65536.0) + ENC_ELEV_OFFSET);

  gyro1 = (double)(RxFrame[i_GYRO1]-GYRO1_OFFSET)*ADU1_TO_DPS;
  gyro2 = (double)(RxFrame[i_GYRO2]-GYRO2_OFFSET)*ADU2_TO_DPS;
  gyro3 = (double)(RxFrame[i_GYRO3]-GYRO3_OFFSET)*ADU3_TO_DPS;

  bias = (double)(RxFrame[i_mag_bias]);
  x_comp = (double)(RxFrame[i_mag_x]) - (26303.0+16881.0)/2.0;
  y_comp = (double)(RxFrame[i_mag_y]) - (24792.0+15615.0)/2.0;

  i_ss = ss_index;

  ACSData.t = time(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.enc_elev = enc_elev;
  ACSData.gyro1 = gyro1;
  ACSData.gyro2 = gyro2;
  ACSData.gyro3 = gyro3;
  ACSData.mag_x = x_comp;
  ACSData.mag_y = y_comp;

  ACSData.clin_elev = (double)RxFrame[i_clin_elev];

}

void SunSensor(void) {
  int sock = -1, n, prin;
  int ars, ers;
  double az_rel_sun, el_rel_sun;

  fd_set fdr;
  struct timeval timeout;

  struct sockaddr_in addr;

  char buff[256];
  const char P[] = "P";

  mputs(MCP_STARTUP, "SunSensor startup\n");

  /* we don't want mcp to die of a broken pipe, so catch the
   * SIGPIPEs which are raised when the ssc drops the connection */
  signal(SIGPIPE, SigPipe);

  while (1) {
    if (sock != -1) 
      if (close(sock) == -1)
        merror(MCP_ERROR, "SunSensor close()");


    /* create an empty socket connection */
    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == -1)
      merror(MCP_TFATAL, "SunSensor socket()");

    /* set options */
    n = 1;
    if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) == -1)
      merror(MCP_TFATAL, "SunSensor setsockopt()");

    /* Connect to Arien */
    inet_aton(ARIEN, &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(ARIEN_PORT);
    while ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
        < 0) {
      merror(MCP_ERROR, "SunSensor connect()");
      sleep(10);
    };

    mputs(MCP_INFO, "Connected to Arien\n");
    sigPipeRaised = n = 0;
    //lastIndex = RxFrameIndex;
    while (n != -1) {
      //curIndex = RxFrameIndex;
      //if (curIndex > lastIndex) {
      //lastIndex = curIndex;
      usleep(10000);
      if (send(sock, P, 2, 0) == -1) {
        merror(MCP_ERROR, "SunSensor send()");
      }

      FD_ZERO(&fdr);
      FD_SET(sock, &fdr);

      timeout.tv_sec = 10;
      timeout.tv_usec = 0;

      n = select(sock + 1, &fdr, NULL, NULL, &timeout);

      if (n == -1 && errno == EINTR) {
        mputs(MCP_WARNING, "timeout on Sun Sensor\n");
        continue;
      }
      if (n == -1) {
        merror(MCP_ERROR, "SunSensor select()");
        continue;
      }

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, buff, (socklen_t)255, MSG_DONTWAIT);

        if (n != -1) {
          buff[n] = 0;
          if (sscanf(buff, "%lf %lf %i %i %i", &az_rel_sun, &el_rel_sun, &ars,
                &ers, &prin) == 5) {
            SunSensorData[ss_index].raw_el = (short int)(ers);
            SunSensorData[ss_index].raw_az = (short int)(ars);
            SunSensorData[ss_index].prin = prin;
            ss_index = INC_INDEX(ss_index + 1);
          }
        } else {
          merror(MCP_ERROR, "SunSensor recv()");
        }
      } else {
        mputs(MCP_WARNING, "Connection to Arien timed out.\n");
        n = -1;
      }
      //} else {
      //	usleep(10000);
      //}
    }
  }
}

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

int fill_Rx_frame(unsigned int in_data,
    unsigned int *TxFrame,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW],
    unsigned short *RxFrame) {
  static int i = 0;
  static int n_not_found=0;
  int start, done = 0;
  int j;

  if (in_data == 0xffffffff) {
    return (0);
  }

  start = i;

  do {
    if ((i < 4 + N_SLOW) && (i >= 4)) {

      /*      if ((GET_NODE(in_data)==1) && (GET_CH(in_data)==56)) {
              printf("%d %08x %08x r: %d w: %d node: %2d ch: %2d\n", i,
              TxFrame[i], in_data,
              GET_READ(in_data), GET_STORE(in_data),
              GET_NODE(in_data),
              GET_CH(in_data));
              } */

      for (j = 0; j < FAST_PER_SLOW; j++) {
        if ((in_data & 0x3fff0000) == (slowTxFields[i - 4][j] & 0x3fff0000)) {
          RxFrame[i] = (unsigned short)(in_data & 0xffff);
          done = 1;
        }
      }
      if (!done) {
        if ((in_data & (BBC_ADC_SYNC | 0xffff))==(BBC_ADC_SYNC | 0xa5a3)) {
          done = 1;
        }
      }
    } else {
      if ((in_data & 0x3fff0000) == (TxFrame[i] & 0x3fff0000)) {
        RxFrame[i] = (unsigned short)(in_data & 0xffff);
        done = 1;
      }
    }

    i++;

    if (i >= FRAME_WORDS)
      i = 0;

    if (i == start) {
      //if (n_not_found==0) {
      //  printf("%d %08x %08x r: %d w: %d node: %2d ch: %2d\n", start,
      //     TxFrame[start], in_data,
      //     GET_READ(in_data), GET_STORE(in_data),
      //     GET_NODE(in_data),
      //     GET_CH(in_data));
      //}

      n_not_found++;
      if (n_not_found>2) {
        return(0);
      } else {
        return(1);
      }
    }
  } while(!done);

  n_not_found = 0;

  return(1);
}

void write_to_biphase(unsigned short *RxFrame) {
  static int fp = -2;
  static unsigned short sync = 0xeb90;
  /* static unsigned short nothing = 0x00; */
  int i;
  static unsigned short nothing[1024];

  if (fp == -2) {
    fp = open("/dev/bi0", O_RDWR);
    if (fp == -1)
      merror(MCP_TFATAL, "Error opening biphase device");

    for (i = 0; i < 1024; i++) nothing[i] = 0xeeee;
  }

  if (fp >= 0) {
    /* Write the sync word */
    if (write(fp, &sync, 2) < 0)
      merror(MCP_ERROR, "bi-phase write (sync) failed");
    if (write(fp, RxFrame + 1, 2 * (FRAME_WORDS - 1)) < 0)
      merror(MCP_ERROR, "bi-phase write (RxFrame) failed");
    if (write(fp, nothing + FRAME_WORDS, 2 * (624 - FRAME_WORDS)) < 0)
      merror(MCP_ERROR, "bi-phase write (padding) failed");
  }
}

void InitBi0Buffer() {
  int i;

  bi0_buffer.i_in = 10; /* preload the fifo */
  bi0_buffer.i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++) {
    if ((bi0_buffer.framelist[i] = malloc(FRAME_WORDS *
            sizeof(unsigned short))) == NULL)
      merror(MCP_FATAL, "Unable to malloc for bi-phase");
  }
}

void PushBi0Buffer(unsigned short *RxFrame) {
  int i, fw, i_in;

  i_in = bi0_buffer.i_in + 1;
  if (i_in>=BI0_FRAME_BUFLEN)
    i_in = 0;

  fw = FRAME_WORDS;

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
  mprintf(MCP_ERROR, "Caught signal %i, closing BBC", signo);
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
  int i, j;
  unsigned int in_data, n_to_read = 0, status;
  unsigned short* RxFrame;
  unsigned int *TxFrame;
  int last_frames = FRAME_MARGIN, frames;
  unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW];
  int n_reset=0;

  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t sunsensor_id;

#ifndef BOLOTEST
  pthread_t CommandDatacomm2;
  pthread_t bi0_id;
  pthread_t sensors_id;
  pthread_t dgps_id;
  pthread_t isc_id;
#endif

  struct CommandDataStruct CommandData_loc;
  int nread = 0, last_nread = 0, last_read = 0, df = 0;

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

  if ((logfile = fopen("/data/etc/mcp.log", "a")) == NULL)
    merror(MCP_ERROR, "Can't open log file");
  else
    fputs("----- LOG RESTART -----\n", logfile);

  mputs(MCP_STARTUP, "MCP startup");

  //Initialize the Ephemeris
  ReductionInit();

  InitCommandData();

  if ((bbc_fp = open("/dev/bbc", O_RDWR)) < 0)
    merror(MCP_FATAL, "Error opening BBC");

  pthread_mutex_init(&mutex, NULL);

#ifdef BOLOTEST
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);

  pthread_create(&dgps_id, NULL, (void*)&WatchDGPS, NULL);
  pthread_create(&isc_id, NULL, (void*)&IntegratingStarCamera, NULL);

  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  pthread_create(&sunsensor_id, NULL, (void*)&SunSensor, NULL);

  InitBi0Buffer();

  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
#endif
  InitialiseFrameFile(argv[1][0]);

  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  memset(PointingData, 0, 3*sizeof(struct PointingDataStruct));

  mprintf(MCP_INFO, "Tx Frame Bytes: %d.  Allowed: 2220\n", TX_FRAME_SIZE);
  if (TX_FRAME_SIZE > 2220)
    merror(MCP_FATAL, "TxFrame too big\n");

  if ((TxFrame = malloc(TX_FRAME_SIZE)) == NULL)
    merror(MCP_FATAL, "Unable to malloc TxFrame");

  if ((RxFrame = malloc(RX_FRAME_SIZE)) == NULL)
    merror(MCP_FATAL, "Unable to malloc RxFrame");

  /* Find out whether I'm frodo or sam */
  SamIAm = AmISam();

  if (SamIAm)
    mputs(MCP_INFO, "I am Sam.\n");
  else 
    mputs(MCP_INFO, "I am not Sam.\n");

  do_Tx_frame(bbc_fp, TxFrame, slowTxFields, RxFrame, 0);

  MakeTxFrame();

  InitSched();

  last_frames =  ioctl(bbc_fp, BBC_IOC_FRAMES) + FRAME_MARGIN + 1;

  while (1) {
    pthread_mutex_lock(&mutex);
    CommandData_loc = CommandData;
    pthread_mutex_unlock(&mutex);
    status = ioctl(bbc_fp, BBC_IOC_STATUS);

    if (!(BBC_TX_NOT_EMPTY(status))) {
      frames =  ioctl(bbc_fp, BBC_IOC_FRAMES);
      df = frames-last_frames;
      close(bbc_fp);
      bbc_fp = open("/dev/bbc", O_RDWR);
      if (bbc_fp < 0)
        merror(MCP_FATAL, "Error opening BBC");

      frame_num = frames_in;
      mprintf(MCP_WARNING, "EMPTY %d %d %d\n", df, frames, frame_num);
      last_frames = FRAME_MARGIN;
    }

    frames =  ioctl(bbc_fp, BBC_IOC_FRAMES);
    df = frames-last_frames;
    for ( j = last_frames; j < frames; j++ ) {
      do_Tx_frame(bbc_fp, TxFrame, slowTxFields, RxFrame, 0);
    }
    last_frames = frames;

    n_to_read = ioctl(bbc_fp, BBC_IOC_RX_SW_COUNT);
    for (i = 0; i < n_to_read; i++) {
      if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) < 0) {
        merror(MCP_ERROR, "Error on BBC read");
      }

      nread++;
      if (nread == last_nread)
        last_read = in_data;
      if (!fill_Rx_frame(in_data, TxFrame, slowTxFields, RxFrame)) {
        close(bbc_fp);
        bbc_fp = open("/dev/bbc", O_RDWR);
        if (bbc_fp < 0)
          merror(MCP_FATAL, "Error opening BBC");

        frame_num = frames_in;
        do_Tx_frame(bbc_fp, TxFrame, slowTxFields, RxFrame, 0);
        last_frames = FRAME_MARGIN + 1;
        n_reset++;
        mprintf(MCP_WARNING, "RESET (%.4f %% cumulative loss)\n",
            100.0*(double)n_reset/((double)frame_num));
        break;
      }

      if (IsNewFrame(in_data)) {
        frames_in++;
#ifndef BOLOTEST
        GetACS(RxFrame);
        Pointing();
#endif

        RxFrameIndex = RxFrame[3];

#ifndef BOLOTEST
        PushBi0Buffer(RxFrame);
#endif
        //FillSlowDL(RxFrame);
        pushDiskFrame(RxFrame);
        zero(RxFrame);

        last_nread = nread;
        nread = 0;
      }
    }

    if (sigPipeRaised) {
      mputs(MCP_ERROR, "SIGPIPE caught, recreating sunsensor thread.\n");
      pthread_create(&sunsensor_id, NULL, (void*)&SunSensor, NULL);
      sigPipeRaised = 0;
    }
    usleep(10000);
  }
  /* atexit(bc_close); */
  return(0);
}
