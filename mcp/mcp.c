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
#include <errno.h>
#include <signal.h>
#include <math.h>
#include "bbc.h"

#include "tx_struct.h"
#include "tx.h"

#include "pointing_struct.h"
#include "command_struct.h"
#include "starpos.h"

#define ARIEN "192.168.1.99"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#ifdef BOLOTEST
  #define FRAME_MARGIN (-12)
#else
  #define FRAME_MARGIN (-30)
#endif

#define BI0_FRAME_BUFLEN (40)
/* Define global variables */

int bbc_fp;
int frames_in = 0;

extern void starfind();

struct SunSensorDataStruct SunSensorData[3];

struct ACSDataStruct ACSData;

int ss_index = 0;
int sigPipeRaised = 0;
int RxframeIndex;

short int SamIAm;

extern struct SlowDLStruct SlowDLInfo[N_SLOWDL];
extern struct CommandDataStruct CommandData;
extern struct SIPDataStruct SIPData;

extern pthread_mutex_t mutex;

void Pointing();
void WatchPortC1(void);
void WatchPortC2(void);
void WatchFIFO(void);
void DirFileWriter(void);

void InitializeDirfile(char type);
void dirFileWriteFrame(unsigned short *Rxframe);
void pushDiskFrame(unsigned short *Rxframe);

void MakeTxFrame(void);

/* Functions in the file 'geomag.c' */
void MagModelInit(int maxdeg);
void GetMagModel(float alt, float glat, float glon, float time,
                 float *dec, float *dip, float *ti, float *gv);

void InitSched();

struct {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
} bi0_buffer;

void SensorReader(void) {
  int fan, T;
  int nr;

  FILE *fp;
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
    sleep(1);
  }
}

/* The SIG is raised in the SunSensor thread, which doesn't have to do anything
 * else but die (and tell someone that it did)... */
void SigPipe(int signal) {
  sigPipeRaised = 1;

  /* exit so mcp can respawn the thread */
  pthread_exit(NULL);
}

/************************************************************************\
|*                                                                      *|
|*   MagRead:  readout magnetometer, subtracting bais from x and        *|
|*             y and determine angle                                    *|
|*                                                                      *|
\************************************************************************/
double MagRead(unsigned short *Rxframe) {
  static int i_mag_x = -1;
  static int i_mag_y = -1;

  double mag_az, x_comp, y_comp;
  float year;
  static float dec, dip, ti, gv;
  static time_t t, oldt;
  struct tm now;

  /******** Obtain correct indexes the first time here ***********/
  if (i_mag_x == -1) {
    FastChIndex("mag_x", &i_mag_x);
    FastChIndex("mag_y", &i_mag_y);

    /* Initialise magnetic model reader: I'm not sure what the '12' is, but */
    /* I think it has something to do with the accuracy of the modelling -- */
    /* probably shouldn't change this value.  (Adam H.) */
    MagModelInit(12);
    oldt = -1;
  }

  x_comp = (M_PI / 180.0) *
    ( ( (double)(Rxframe[i_mag_x]) )
      * (-360.0 / 65536.0) );

  y_comp = (M_PI / 180.0) *
    ( ( (double)(Rxframe[i_mag_y]) )
      * (-360.0 / 65536.0) );

  /* Every 300 s = 5 min, get new data from the magnetic model. */
  /* */
  /* dec = magnetic declination (field direction in az) */
  /* dip = magnetic inclination (field direction in ele) */
  /* ti  = intensity of the field in nT */
  /* gv  = modified form of dec used in polar reasons -- haven't researched */
  /*       this one */
  /* */
  /* The year must be between 2000.0 and 2005.0 with current model data */
  /* */
  /* The functions called are in 'geomag.c' (Adam. H) */
  if ((t = time(NULL)) > oldt + 300) {
    oldt = t;
    gmtime_r(&t, &now);
    year = 1900 + now.tm_year + now.tm_yday / 365.25;

    GetMagModel(SIPData.GPSpos.alt / 1000.0, SIPData.GPSpos.lat,
        SIPData.GPSpos.lon, year, &dec, &dip, &ti, &gv);

    dec *= M_PI / 180.0;
    dip *= M_PI / 180.0;
  }

  /* The dec is the correction to the azimuth of the magnetic field. */
  /* If negative is west and positive is east, then: */
  /* */
  /*   true bearing = magnetic bearing + dec */
  /* */
  /* Thus, depending on the sign convention, you have to either add or */
  /* subtract dec from az to get the true bearing. (Adam H.) */

  mag_az = atan2(y_comp, x_comp) + dec + MAG_ALIGNMENT + M_PI;
  mag_az *= 180.0/M_PI;

  return mag_az;
}

void GetACS(unsigned short *Rxframe){
  double enc_elev, gyro1, gyro2, gyro3;
  static int i_GYRO1 = -1;
  static int i_GYRO2 = -1;
  static int i_GYRO3 = -1;
  static int i_enc_elev = -1;
  unsigned int rx_frame_index = 0;
  int i_ss;

  if (i_enc_elev == -1) {
    FastChIndex("enc_elev", &i_enc_elev);
    FastChIndex("gyro1", &i_GYRO1);
    FastChIndex("gyro2", &i_GYRO2);
    FastChIndex("gyro3", &i_GYRO3);
  }

  rx_frame_index = ((Rxframe[1] & 0x0000ffff) |
                    (Rxframe[2] & 0x0000ffff) << 16);
  
  enc_elev = ((double)Rxframe[i_enc_elev] *
  	      (-360.0 / 65536.0) + ENC_ELEV_OFFSET);

  gyro1 = (double)(Rxframe[i_GYRO1]-25794.0)*0.00091506980885;
  gyro2 = (double)(Rxframe[i_GYRO2]-25535.0)*0.00091506980885;
  gyro3 = (double)(Rxframe[i_GYRO3]-25600.0)*0.00091506980885;

  i_ss = ss_index;
  
  ACSData.t = time(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.mag_az = MagRead(Rxframe);
  ACSData.enc_elev = enc_elev;
  ACSData.gyro1 = gyro1;
  ACSData.gyro2 = gyro2;
  ACSData.gyro3 = gyro3;

}

void SunSensor(void) {
  int sock, n, prin, attempts, lastIndex, curIndex;
  int ars, ers;

  struct sockaddr_in addr;

  char buff[256];
  const char P[] = "P";

  /* we don't want mcp to die of a broken pipe, so catch the
   * SIGPIPEs which are raised when the ssc drops the connection */
  signal(SIGPIPE, SigPipe);

  /* create an empty socket connection */
  sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

  /* set options */
  n = 1;
  setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n));

  /* Connect to Arien */
  inet_aton(ARIEN, &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(2718);
  fprintf(stderr, "Attempting to connect to Arien...\n");
  while (connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) < 0) {
    fprintf(stderr, "Connection attempt to Arien failed.\n");
    sleep(1);
    fprintf(stderr, "Attempting to connect to Arien...\n");
  };

  fprintf(stderr, "Connected to Arien\n");
  sigPipeRaised = n = 0;
  lastIndex = RxframeIndex;
  while (n != -1) {
    curIndex = RxframeIndex;
    if (curIndex < lastIndex) {
      attempts = 0;
      //fprintf(stderr, "P\n");
      send(sock, P, 2, 0);

      do {
        n = recv(sock, buff, (socklen_t)255, MSG_DONTWAIT);
        if (n == -1) {
          attempts++;
          usleep(10);
        }
      } while ((n == -1) && (attempts < 200));

      if (n != -1) {
        buff[n] = 0;
        if (sscanf(buff, "%*f %*f %i %i %i", &ars, &ers, &prin) == 3) {
          SunSensorData[ss_index].raw_el = (short int)(ars);
          SunSensorData[ss_index].raw_az = (short int)(ers);
          SunSensorData[ss_index].prin = prin;
          ss_index = (ss_index + 1) % 3;
          fprintf(stderr, "%i %i  %i  %i\n", attempts * 10, ars, ers, prin);
        }
      }
    }

    lastIndex = curIndex;
  }

  /* connection timed out .. raise SIGPIPE to force a restart */
  fprintf(stderr, "Connection to Arien timed out.\n");
  raise(SIGPIPE);
}

void FillSlowDL(unsigned short *Rxframe) {
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
    if ((SlowDLInfo[i].ind2 >= 0 && Rxframe[3] == SlowDLInfo[i].ind2)
        || SlowDLInfo[i].ind2 < 0) {
      switch (SlowDLInfo[i].ctype) {
        case 's':
          SlowDLInfo[i].value =
            (double)((signed short) Rxframe[SlowDLInfo[i].ind1 + 4]);
          break;
        case 'u':
          SlowDLInfo[i].value = (double)(Rxframe[SlowDLInfo[i].ind1 + 4]);
          break;
        case 'i': case 'S':
          SlowDLInfo[i].value =
            (double)((signed int) ((Rxframe[SlowDLInfo[i].ind1 + 4] << 16) +
                                   Rxframe[SlowDLInfo[i].ind1 + 5]));
          break;
        case 'U':
          SlowDLInfo[i].value =
            (double)((unsigned int) ((Rxframe[SlowDLInfo[i].ind1 + 4] << 16) +
                                     Rxframe[SlowDLInfo[i].ind1 + 5]));
          break;
      }
      SlowDLInfo[i].value = SlowDLInfo[i].value * SlowDLInfo[i].m_c2e
        + SlowDLInfo[i].b_e2e;
    }
  }
  pthread_mutex_unlock(&mutex);
}

int fill_Rx_frame(unsigned int in_data,
    unsigned int *Txframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW],
    unsigned short *Rxframe) {
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
      for (j = 0; j < FAST_PER_SLOW; j++) {
        if ((in_data & 0x3fff0000) == (slowTxFields[i - 4][j] & 0x3fff0000)) {
          Rxframe[i] = (unsigned short)(in_data & 0xffff);
          done = 1;
        }
      }
      if (!done) {
        if ((in_data & (BBC_ADC_SYNC | 0xffff))==(BBC_ADC_SYNC | 0xa5a3)) {
          done = 1;
        }
      }
    } else {
      if ((in_data & 0x3fff0000) == (Txframe[i] & 0x3fff0000)) {
        Rxframe[i] = (unsigned short)(in_data & 0xffff);
        done = 1;
      }
    }

    i++;

    if (i >= FrameWords()) i = 0;

    if (i == start) {
      //if (n_not_found==0) {
      //  printf("%d %08x %08x r: %d w: %d node: %2d ch: %2d\n", start,
      //     Txframe[start], in_data,
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

void write_to_biphase(unsigned short *Rxframe) {
  static int fp = -2;
  static unsigned short sync = 0xeb90;
  /* static unsigned short nothing = 0x00; */
  int i;
  static unsigned short nothing[1024];

  if (fp == -2) {
    fp = open("/dev/bi0", O_RDWR);
    if (fp == -1) {
      fprintf(stderr, "Error opening biphase device\n");
    }
    for (i = 0; i < 1024; i++) nothing[i] = 0xeeee;
  }

  if (fp >= 0) {
    write(fp, &sync, 2); /* Write the sync word */
    write(fp, Rxframe + 1, 2 * (FrameWords() - 1));
    write(fp, nothing+FrameWords(), 2 * (624 - FrameWords()));
  }
}

void InitBi0Buffer() {
  int i;

  bi0_buffer.i_in = 10; /* preload the fifo */
  bi0_buffer.i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++) {
    bi0_buffer.framelist[i] = malloc(FrameWords() * sizeof(unsigned short));
  }
}

void PushBi0Buffer(unsigned short *Rxframe) {
  int i, fw, i_in;

  i_in = bi0_buffer.i_in + 1;
  if (i_in>=BI0_FRAME_BUFLEN) i_in = 0;

  fw = FrameWords();

  for (i = 0; i<fw; i++) {
    bi0_buffer.framelist[i_in][i] = Rxframe[i];
  }
  bi0_buffer.i_in = i_in;
}

void zero(unsigned short *Rxframe) {
  int i, fw;

  fw = FAST_OFFSET; //FrameWords();

  for (i = 0; i<fw; i++) {
    Rxframe[i] = 0;
  }
}

void BiphaseWriter(void) {
  int i_out, i_in;


  while (1) {
    i_in = bi0_buffer.i_in;
    i_out = bi0_buffer.i_out;
    while (i_out != i_in) {
      i_out++;
      if (i_out>=BI0_FRAME_BUFLEN) i_out = 0;
      write_to_biphase(bi0_buffer.framelist[i_out]);
    }
    bi0_buffer.i_out = i_out;
    usleep(10000);
  }
} 

/* Identity crisis: am I frodo or sam? */
int AmISam(void) {
  char buffer[2];

  gethostname(buffer, 1);

  return (buffer[0] == 's') ? 1 : 0;
}

void WatchFIFO(void);

int main(int argc, char *argv[]) {
  int i, j;
  unsigned int in_data, n_to_read = 0, status;
  /* int bbc_fp; */
  unsigned short* Rxframe;
  unsigned int *Txframe;
  int last_frames = FRAME_MARGIN, frames;
  unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW];
  int n_reset=0;

  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t sunsensor_id;

#ifndef BOLOTEST
  pthread_t starfind_id;
  pthread_t CommandDatacomm2;
  pthread_t bi0_id;
  pthread_t sensors_id;
#endif

  struct CommandDataStruct CommandData_loc;
  int nread = 0, last_nread = 0, last_read = 0, df = 0;

  //Initialize the Ephemeris
  ReductionInit();

//  pthread_create(&starfind_id, NULL, (void*)&starfind, NULL);
   
  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

#ifdef BOLOTEST
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPortC1, NULL);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPortC2, NULL);

  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  //pthread_create(&sunsensor_id, NULL, (void*)&SunSensor, NULL);

  InitBi0Buffer();
  pthread_create(&bi0_id, NULL, (void*)&BiphaseWriter, NULL);
#endif

  fprintf(stderr, "Tx Frame Bytes: %d.  Allowed: 2220\n", TxFrameBytes());
  if (TxFrameBytes() > 2220) exit(1);

  Txframe = malloc(TxFrameBytes());
  Rxframe = malloc(FrameWords() * sizeof(unsigned short));

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

  /* Find out whether I'm frodo or sam */
  SamIAm = AmISam();

  if (SamIAm)
    fprintf(stderr, "I am Sam.\n");
  else 
    fprintf(stderr, "I am not Sam.\n");

  MakeTxFrame();

  bbc_fp = open("/dev/bbc", O_RDWR);
  if (bbc_fp < 0) {
    fprintf(stderr, "Error opening BBC\n");
    exit(0);
  }

  InitializeDirfile(argv[1][0]);

  InitSched();
  
  pthread_create(&disk_id, NULL, (void*)&DirFileWriter, NULL);

  do_Tx_frame(bbc_fp, Txframe, slowTxFields, Rxframe, 0);

  last_frames = FRAME_MARGIN + 1;

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
      frame_num = frames_in;
      fprintf(stderr, "EMPTY %d %d %d\n", df, frames, frame_num);
      last_frames = FRAME_MARGIN;
    }

    frames =  ioctl(bbc_fp, BBC_IOC_FRAMES);
    df = frames-last_frames;
    for ( j = last_frames; j < frames; j++ ) {
      do_Tx_frame(bbc_fp, Txframe, slowTxFields, Rxframe, 0);
    }
    last_frames = frames;

    n_to_read = ioctl(bbc_fp, BBC_IOC_RX_SW_COUNT);
    //printf("%d\n", n_to_read);
    for (i = 0; i < n_to_read; i++) {
      read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int));

      nread++;
      if (nread == last_nread) last_read = in_data;
      if (!fill_Rx_frame(in_data, Txframe, slowTxFields, Rxframe)) {
        close(bbc_fp);
        bbc_fp = open("/dev/bbc", O_RDWR);
        frame_num = frames_in;
        do_Tx_frame(bbc_fp, Txframe, slowTxFields, Rxframe, 0);
        last_frames = FRAME_MARGIN + 1;
        n_reset++;
        fprintf(stderr, "RESET (%.4f %% cumulative loss)\n",
            100.0*(double)n_reset/((double)frame_num));
        break;
      }

      if (IsNewFrame(in_data)) {
        frames_in++;

#ifndef BOLOTEST
        GetACS(Rxframe);
        Pointing();
#endif
        
        RxframeIndex = Rxframe[3];
        
#ifndef BOLOTEST
        PushBi0Buffer(Rxframe);
#endif
        //FillSlowDL(Rxframe);
        pushDiskFrame(Rxframe);
        zero(Rxframe);

        last_nread = nread;
        nread = 0;
      }
    }

    if (sigPipeRaised) {
      fprintf(stderr, "SIGPIPE caught, recreating sunsensor thread.\n");
      pthread_create(&sunsensor_id, NULL, (void*)&SunSensor, NULL);
      sigPipeRaised = 0;
    }
    usleep(10000);
  }
  /* atexit(bc_close); */
  return(0);
}
