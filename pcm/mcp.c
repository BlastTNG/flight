/* mcp: the Spider master control program
 *
 * mcp.c: contains the main loop and creates all threads used by mcp
 *
 * This software is copyright (C) 2002-2011 University of Toronto
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
#include "share/bbc_pci.h"

#include "share/blast.h"
#include "command_list.h"
#include "command_struct.h"
#include "share/crc.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "slow_dl.h"
#include "share/starpos.h"
#include "share/channels.h"
#include "tx.h"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#define STARTUP_VETO_LENGTH 250 /* "frames" */
#define BI0_VETO_LENGTH 5 /* seconds */

#define BI0_FRAME_BUFLEN (400)

/* Define global variables */
int bbc_fp = -1;
unsigned int debug = 0;
short int BitsyIAm;
struct ACSDataStruct ACSData;

unsigned int BBFrameIndex;
unsigned short* slow_data[FAST_PER_SLOW];
unsigned short* RxFrame;
pthread_t watchdog_id;

int StartupVeto = STARTUP_VETO_LENGTH + 1;
int UsefulnessVeto = 2*STARTUP_VETO_LENGTH;
int BLASTBusUseful = 0;

static int bi0_fp = -2;
static int Death = -STARTUP_VETO_LENGTH * 2;
static int RxFrameMultiplexIndex;

extern short int InCharge; /* tx.c */
extern struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA];
extern pthread_mutex_t mutex;       //commands.c

void Pointing();
void WatchPort(void*);
void WatchDGPS(void);
void IntegratingStarCamera(void);
void ActuatorBus(void);
void WatchFIFO(void);               //commands.c
void FrameFileWriter(void);         //framefile.c
void CompressionWriter(void);
void StageBus(void);
void openSBSC(void);

void InitialiseFrameFile(char);
void pushDiskFrame(unsigned short *RxFrame);
void ShutdownFrameFile();

void SunSensor(void);

void InitSched();

static FILE* logfile = NULL;

#ifndef BOLOTEST
struct frameBuffer {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
  unsigned short** slow_data_list[BI0_FRAME_BUFLEN];
};

static struct frameBuffer bi0_buffer;
struct frameBuffer hiGain_buffer;

#endif

time_t biphase_timer;
int biphase_is_on = 0;

struct chat_buf chatter_buffer;

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 6                /* 2*(marker+space) + EOL + NUL */ \
  - 24               /* date "YYYY-MM-DD HH:MM:SS.mmm " */ \
  - 8                /* thread name "ThName: " */ \
)

#if (TEMPORAL_OFFSET != 0)
#warning TEMPORAL_OFFSET NON-ZERO; FIX FOR FLIGHT
#endif

#ifndef BOLOTEST
void openMotors();    // motors.c
void closeMotors();

void startChrgCtrl(); // chrgctrl.c
void endChrgCtrl();
#endif

/* gives system time (in s) */
time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}

/* tid to name lookup list */
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
  bprintf(startup, "New thread name for tid %d", new_node->tid);
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
      fprintf(logfile, "$$ Last error is THREAD FATAL. Thread [" 
          TID_NAME_FMT " (%5u)] exits.\n",threadNameLookup(syscall(SYS_gettid)),
          (unsigned)syscall(SYS_gettid));
      fflush(logfile);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [" 
          TID_NAME_FMT " (%5u)] exits.\n",threadNameLookup(syscall(SYS_gettid)),
          (unsigned)syscall(SYS_gettid));
    fflush(stdout);

    pthread_exit(NULL);
  }
}

#ifndef BOLOTEST
static void FillSlowDL()
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

  fd = open("/data/etc/spider/pcm.log", O_RDONLY|O_NONBLOCK);

  if (fd == -1)
  {
    bprintf(tfatal, "Failed to open /data/etc/spider/pcm.log for reading (%d)\n", errno);
  }

  if (fpos == -1) {
    if (lseek(fd, -500, SEEK_END) == -1)
    {
      if (errno == EINVAL)
      {
	if (lseek(fd, 0, SEEK_SET) == -1)
	{
	  bprintf(tfatal, "Failed to rewind /data/etc/spider/pcm.log (%d)\n", errno);
	}
      } else {
	bprintf(tfatal, "Failed to seek /data/etc/spider/pcm.log (%d)\n", errno);
      }
    }
  } else {
    if (lseek(fd, fpos, SEEK_SET) == -1)
    {
      if (lseek(fd, 0, SEEK_END) == -1)
      {
	bprintf(tfatal, "Failed to rewind /data/etc/spider/pcm.log (%d)\n", errno);
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
        bprintf(tfatal, "Error reading from /data/etc/spider/pcm.log (%d)\n", errno);
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


static void GetACS()
{
  double enc_raw_el, ifel_gy, ifroll_gy, ifyaw_gy;
  double x_comp, y_comp, z_comp;
  double pss1_i1, pss1_i2, pss1_i3, pss1_i4;
  double pss2_i1, pss2_i2, pss2_i3, pss2_i4;
  double vel_rw;
  double res_piv;
  int hwpr_pot;

  
  static struct BiPhaseStruct* ifElgyAddr;
  static struct BiPhaseStruct* ifRollgyAddr;
  static struct BiPhaseStruct* ifYawgyAddr;
  static struct BiPhaseStruct* elRawEncAddr;
  static struct BiPhaseStruct* elRawIfClinAddr;
  static struct BiPhaseStruct* xMagAddr;
  static struct BiPhaseStruct* yMagAddr;
  static struct BiPhaseStruct* zMagAddr;
  static struct BiPhaseStruct* velRWAddr;
  static struct BiPhaseStruct* resPivAddr;
  static struct BiPhaseStruct* v11PssAddr;
  static struct BiPhaseStruct* v21PssAddr;
  static struct BiPhaseStruct* v31PssAddr;
  static struct BiPhaseStruct* v41PssAddr;
  static struct BiPhaseStruct* v12PssAddr;
  static struct BiPhaseStruct* v22PssAddr;
  static struct BiPhaseStruct* v32PssAddr;
  static struct BiPhaseStruct* v42PssAddr;
  static struct BiPhaseStruct* potHwprAddr;

  unsigned int rx_frame_index = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elRawEncAddr = GetBiPhaseAddr("el_raw_enc");
    elRawIfClinAddr = GetBiPhaseAddr("el_raw_if_clin");
    ifElgyAddr = GetBiPhaseAddr("ifel_gy");
    ifRollgyAddr = GetBiPhaseAddr("ifroll_gy");
    ifYawgyAddr = GetBiPhaseAddr("ifyaw_gy");
    xMagAddr = GetBiPhaseAddr("x_mag");
    yMagAddr = GetBiPhaseAddr("y_mag");
    zMagAddr = GetBiPhaseAddr("z_mag");
    velRWAddr = GetBiPhaseAddr("vel_ser_rw");
    resPivAddr = GetBiPhaseAddr("res_piv");
    v11PssAddr = GetBiPhaseAddr("v1_1_pss");
    v21PssAddr = GetBiPhaseAddr("v2_1_pss");
    v31PssAddr = GetBiPhaseAddr("v3_1_pss");
    v41PssAddr = GetBiPhaseAddr("v4_1_pss");
    v12PssAddr = GetBiPhaseAddr("v1_2_pss");
    v22PssAddr = GetBiPhaseAddr("v2_2_pss");
    v32PssAddr = GetBiPhaseAddr("v3_2_pss");
    v42PssAddr = GetBiPhaseAddr("v4_2_pss");
    potHwprAddr = GetBiPhaseAddr("pot_hwpr");
  }

  rx_frame_index = ((RxFrame[1] & 0x0000ffff) |
      (RxFrame[2] & 0x0000ffff) << 16);

  enc_raw_el = (((double)RxFrame[elRawEncAddr->channel])/DEG2I);
  vel_rw = (((double)((short)RxFrame[velRWAddr->channel]))*4.0/DEG2I);
  ifel_gy = (double)((RxFrame[ifElgyAddr->channel])-GY16_OFFSET)*GY16_TO_DPS;
  ifroll_gy = (double)(RxFrame[ifRollgyAddr->channel]-GY16_OFFSET)*GY16_TO_DPS;
  ifyaw_gy = (double)(RxFrame[ifYawgyAddr->channel]-GY16_OFFSET)*GY16_TO_DPS;

  res_piv = (((double)
	((short)slow_data[resPivAddr->index][resPivAddr->channel]))/DEG2I);

  x_comp = (double)(slow_data[xMagAddr->index][xMagAddr->channel]);
  y_comp = (double)(slow_data[yMagAddr->index][yMagAddr->channel]);
  z_comp = (double)(slow_data[zMagAddr->index][zMagAddr->channel]);

  pss1_i1 = (double)(slow_data[v11PssAddr->index][v11PssAddr->channel]);
  pss1_i2 = (double)(slow_data[v21PssAddr->index][v21PssAddr->channel]);
  pss1_i3 = (double)(slow_data[v31PssAddr->index][v31PssAddr->channel]);
  pss1_i4 = (double)(slow_data[v41PssAddr->index][v41PssAddr->channel]);
  pss2_i1 = (double)(slow_data[v12PssAddr->index][v12PssAddr->channel]);
  pss2_i2 = (double)(slow_data[v22PssAddr->index][v22PssAddr->channel]);
  pss2_i3 = (double)(slow_data[v32PssAddr->index][v32PssAddr->channel]);
  pss2_i4 = (double)(slow_data[v42PssAddr->index][v42PssAddr->channel]);
  hwpr_pot = (double)(slow_data[potHwprAddr->index][potHwprAddr->channel]);

  ACSData.clin_elev = (double)(slow_data[elRawIfClinAddr->index][elRawIfClinAddr->channel]);

  ACSData.t = mcp_systime(NULL);
  ACSData.mcp_frame = rx_frame_index;
  ACSData.enc_raw_el = enc_raw_el;
  ACSData.ifel_gy = ifel_gy;
  ACSData.ifroll_gy = ifroll_gy;
  ACSData.ifyaw_gy = ifyaw_gy;
  ACSData.mag_x = x_comp;
  ACSData.mag_y = y_comp;
  ACSData.mag_z = z_comp;
  ACSData.vel_rw = vel_rw;
  ACSData.res_piv = res_piv;
  ACSData.pss1_i1 = pss1_i1;
  ACSData.pss1_i2 = pss1_i2;
  ACSData.pss1_i3 = pss1_i3;
  ACSData.pss1_i4 = pss1_i4;
  ACSData.pss2_i1 = pss2_i1;
  ACSData.pss2_i2 = pss2_i2;
  ACSData.pss2_i3 = pss2_i3;
  ACSData.pss2_i4 = pss2_i4;
  ACSData.hwpr_pot = hwpr_pot; // keep this as an integer, 
                               // so it can be read in one atomic cycle...

}

/* sole purpose of following function is to add a field that reads the total current */

static void GetCurrents()
{

  double i_trans;
  double i_das;
  double i_acs;
  double i_rec;
  double i_sc;
  double i_dgps;
  double i_step;
  double i_flc;
  double i_gy;
  double i_rw;
  double i_el;
  double i_piv;
  double i_tot;

  static struct BiPhaseStruct* i_transAddr;
  static struct BiPhaseStruct* i_dasAddr;
  static struct BiPhaseStruct* i_acsAddr;
  static struct BiPhaseStruct* i_recAddr;
  static struct BiPhaseStruct* i_scAddr;
  static struct BiPhaseStruct* i_dgpsAddr;
  static struct BiPhaseStruct* i_stepAddr;
  static struct BiPhaseStruct* i_flcAddr;
  static struct BiPhaseStruct* i_gyAddr;
  static struct BiPhaseStruct* i_rwAddr;
  static struct BiPhaseStruct* i_elAddr;
  static struct BiPhaseStruct* i_pivAddr;

  static struct NiosStruct* i_transNios;
  static struct NiosStruct* i_dasNios;
  static struct NiosStruct* i_acsNios;
  static struct NiosStruct* i_recNios;
  static struct NiosStruct* i_scNios;
  static struct NiosStruct* i_dgpsNios;
  static struct NiosStruct* i_stepNios;
  static struct NiosStruct* i_flcNios;
  static struct NiosStruct* i_gyNios;
  static struct NiosStruct* i_rwNios;
  static struct NiosStruct* i_elNios;
  static struct NiosStruct* i_pivNios;

  static struct NiosStruct* i_totNios;

  static int firsttime = 1;

  if (firsttime) {
  
    firsttime = 0;

    i_transAddr = GetBiPhaseAddr("i_trans");
    i_dasAddr = GetBiPhaseAddr("i_das");
    i_acsAddr = GetBiPhaseAddr("i_acs");
    i_recAddr = GetBiPhaseAddr("i_rec");
    i_scAddr = GetBiPhaseAddr("i_sc");
    i_dgpsAddr = GetBiPhaseAddr("i_dgps");
    i_stepAddr = GetBiPhaseAddr("i_step");
    i_flcAddr = GetBiPhaseAddr("i_flc");
    i_gyAddr = GetBiPhaseAddr("i_gy");
    i_rwAddr = GetBiPhaseAddr("i_rw");
    i_elAddr = GetBiPhaseAddr("i_el");
    i_pivAddr = GetBiPhaseAddr("i_piv");

    i_transNios = GetNiosAddr("i_trans");
    i_dasNios = GetNiosAddr("i_das");
    i_acsNios = GetNiosAddr("i_acs");
    i_recNios = GetNiosAddr("i_rec");
    i_scNios = GetNiosAddr("i_sc");
    i_dgpsNios = GetNiosAddr("i_dgps");
    i_stepNios = GetNiosAddr("i_step");
    i_flcNios  = GetNiosAddr("i_flc");
    i_gyNios = GetNiosAddr("i_gy");
    i_rwNios = GetNiosAddr("i_rw");
    i_elNios = GetNiosAddr("i_el");
    i_pivNios = GetNiosAddr("i_piv");

    i_totNios = GetNiosAddr("i_tot");

  }

  i_trans = (double)(slow_data[i_transAddr->index][i_transAddr->channel])*i_transNios->m + i_transNios->b;
  i_das = (double)(slow_data[i_dasAddr->index][i_dasAddr->channel])*i_dasNios->m + i_dasNios->b;
  i_acs = (double)(slow_data[i_acsAddr->index][i_acsAddr->channel])*i_acsNios->m + i_acsNios->b;
  i_rec = (double)(slow_data[i_recAddr->index][i_recAddr->channel])*i_recNios->m + i_recNios->b;
  i_sc = (double)(slow_data[i_scAddr->index][i_scAddr->channel])*i_scNios->m + i_scNios->b;
  i_dgps = (double)(slow_data[i_dgpsAddr->index][i_dgpsAddr->channel])*i_dgpsNios->m + i_dgpsNios->b;
  i_step = (double)(slow_data[i_stepAddr->index][i_stepAddr->channel])*i_stepNios->m + i_stepNios->b;
  i_flc = (double)(slow_data[i_flcAddr->index][i_flcAddr->channel])*i_flcNios->m + i_flcNios->b;
  i_gy = (double)(slow_data[i_gyAddr->index][i_gyAddr->channel])*i_gyNios->m + i_gyNios->b;
  i_rw = (double)(slow_data[i_rwAddr->index][i_rwAddr->channel])*i_rwNios->m + i_rwNios->b;
  i_el = (double)(slow_data[i_elAddr->index][i_elAddr->channel])*i_elNios->m + i_elNios->b;
  i_piv = (double)(slow_data[i_pivAddr->index][i_pivAddr->channel])*i_pivNios->m + i_pivNios->b;

  i_tot = i_trans + i_das + i_acs + i_rec + i_sc + i_dgps + i_step + i_flc + i_gy + i_rw + i_el + i_piv;

  WriteData(i_totNios, 1000*i_tot, NIOS_QUEUE);

}

#endif

/* fill_Rx_frame: places one 32 bit word into the RxFrame. 
 * Returns true on success */
static int fill_Rx_frame(unsigned int in_data)
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

static void write_to_biphase()
{
  int i;
  static unsigned short nothing[BI0_FRAME_SIZE];
  static unsigned short sync = 0xEB90;
  static unsigned int do_skip = 0;

  if (bi0_fp == -2) {
    bi0_fp = open("/dev/bbc_bi0", O_RDWR);
    if (bi0_fp == -1) {
      berror(tfatal, "Error opening biphase device");
    }

    for (i = 0; i < BI0_FRAME_SIZE; i++) {
      nothing[i] = 0xEEEE;
    }
  }

  if ((bi0_fp >= 0) && InCharge) {

    RxFrame[0] = 0xEB90;
    nothing[0] = CalculateCRC(0xEB90, RxFrame, BiPhaseFrameWords);

    RxFrame[0] = sync;
    sync = ~sync;

    if (do_skip) {
      --do_skip;
    } else {
      i = write(bi0_fp, RxFrame, BiPhaseFrameWords * sizeof(unsigned short));
      if (i < 0) {
        berror(err, "bi-phase write for RxFrame failed");
      } else if (i != BiPhaseFrameWords * sizeof(unsigned short)) {
        bprintf(err, "Short write for RxFrame: %i of %u", i,
            (unsigned int)(BiPhaseFrameWords * sizeof(unsigned short)));
      }

      i = write(bi0_fp, nothing, (BI0_FRAME_SIZE - BiPhaseFrameWords) *
          sizeof(unsigned short));
      if (i < 0) 
        berror(err, "bi-phase write for padding failed");
      else if (i != (BI0_FRAME_SIZE - BiPhaseFrameWords)
          * sizeof(unsigned short))
        bprintf(err, "Short write for padding: %i of %u", i,
            (unsigned int)
              ((BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(unsigned short)));
    }

    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);
  }
}

static void InitFrameBuffer(struct frameBuffer *buffer) {
  int i,j;

  buffer->i_in = 0;
  buffer->i_out = 0;
  for (i = 0; i<BI0_FRAME_BUFLEN; i++) {
    buffer->framelist[i] = balloc(fatal, BiPhaseFrameWords *
        sizeof(unsigned short));

    memset(buffer->framelist[i],0,BiPhaseFrameWords*sizeof(unsigned short));

    buffer->slow_data_list[i] = (unsigned short **)balloc(fatal, FAST_PER_SLOW*sizeof(unsigned short *));
    for (j = 0; j < FAST_PER_SLOW; ++j) {
      buffer->slow_data_list[i][j] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));
      memset(buffer->slow_data_list[i][j], 0, slowsPerBi0Frame * sizeof(unsigned short));
    }
  }
}

// warning: there is no checking for an overfull buffer.  Just make sure it is big
// enough that it can never happen.
static void PushFrameBuffer(struct frameBuffer *buffer) {
  int i, j, fw, i_in;

  i_in = buffer->i_in + 1;
  if (i_in>=BI0_FRAME_BUFLEN)
    i_in = 0;

  fw = BiPhaseFrameWords;

  for (i = 0; i<fw; i++) {
    buffer->framelist[i_in][i] = RxFrame[i];
  }
  
  for (i=0; i<FAST_PER_SLOW; i++) {
    for (j=0; j<slowsPerBi0Frame; j++) {
      buffer->slow_data_list[i_in][i][j] = slow_data[i][j];
    }
  }
  
  buffer->i_in = i_in;
}

void ClearBuffer(struct frameBuffer *buffer) {
  buffer->i_out = buffer->i_in;
}

unsigned short *PopFrameBufferAndSlow(struct frameBuffer *buffer, unsigned short ***slow) {
  unsigned short *frame;
  int i_out = buffer->i_out;
  
  if (buffer->i_in == i_out) { // no data
    return (NULL);
  }
  frame = buffer->framelist[i_out];
  
  *slow = buffer->slow_data_list[i_out];
  
  i_out++;
  if (i_out>=BI0_FRAME_BUFLEN) {
    i_out = 0;
  }
  buffer->i_out = i_out;
  return (frame);
}

unsigned short *PopFrameBuffer(struct frameBuffer *buffer) {
  unsigned short *frame;
  int i_out = buffer->i_out;
  
  if (buffer->i_in == i_out) { // no data
    return (NULL);
  }
  frame = buffer->framelist[i_out];
  i_out++;
  if (i_out>=BI0_FRAME_BUFLEN) {
    i_out = 0;
  }
  buffer->i_out = i_out;
  return (frame);
}
  
#endif

static void zero()
{
  int i;

  for (i = 0; i < SLOW_OFFSET + slowsPerBi0Frame; i++)
    RxFrame[i] = 0;
}

#ifndef BOLOTEST
static void BiPhaseWriter(void)
{
  unsigned short *frame;
  
  nameThread("Bi0");
  bputs(startup, "Startup\n");

  while (!biphase_is_on)
    usleep(10000);

  bputs(info, "Veto has ended.  Here we go.\n");

  while (1) {
    frame = PopFrameBuffer(&bi0_buffer);
    
    if (!frame) { 
      /* Death meausres how long the BiPhaseWriter has gone without receiving
       * any data -- an indication that we aren't receiving FSYNCs from the
       * BLASTBus anymore */
      if (InCharge && (++Death > 25)) {
        bprintf(err, "Death is reaping the watchdog tickle.");
        pthread_cancel(watchdog_id);
      }
      usleep(10000); // 100 Hz
    } else {
      write_to_biphase(frame);
      if (Death > 0) {
        Death = 0;
      }
    }
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
static int AmIBitsy()
{
  char buffer[16];

  if (gethostname(buffer, 15) == -1 && errno != ENAMETOOLONG) {
    berror(err, "System: Unable to get hostname");
  }

#ifdef TEST_RUN
  //check that flight code is not compiled as a test run
  if (strncmp(buffer, "itsy", 15) == 0 || strncmp(buffer, "bitsy", 15) == 0)
    bputs(fatal, "Flight code can't be compiled with TEST_RUN\n");
#endif

  return (buffer[0] == 'b') ? 1 : 0;
}

/* Signal handler called when we get a hup, int or term */
static void CloseBBC(int signo)
{
  bprintf(err, "System: Caught signal %i; stopping NIOS", signo);
#ifndef BOLOTEST
  closeMotors();

  endChrgCtrl();  // is this needed?
#endif
  RawNiosWrite(0, BBC_ENDWORD, NIOS_FLUSH);
  RawNiosWrite(BBCPCI_MAX_FRAME_SIZE, BBC_ENDWORD, NIOS_FLUSH);
  bprintf(err, "System: Closing BBC and Bi0");
  if (bi0_fp >= 0)
    close(bi0_fp);
  if (bbc_fp >= 0)
    close(bbc_fp);

  ShutdownFrameFile();

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
  int startup_test = 0;
  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t abus_id;

#ifndef USE_FIFO_CMD
  pthread_t CommandDatacomm2;
#endif

#ifndef BOLOTEST
  pthread_t compression_id;
  pthread_t bi0_id;
  pthread_t sensors_id;
  pthread_t dgps_id;
#endif
#ifdef USE_XY_THREAD
  pthread_t xy_id;
#endif
  pthread_t chatter_id;
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

  if ((logfile = fopen("/data/etc/spider/pcm.log", "a")) == NULL) {
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

  //both modes want interrupts enabled (?)
  if (ioctl(bbc_fp, BBCPCI_IOC_ON_IRQ) < 0) startup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_SYNC) < 0) startup_test = -1;
  usleep(100000);  //TODO: needed after sync? shorter?

#ifdef USE_EXT_SERIAL
  bprintf(startup, "System: BBC using external synchronization/serial numbers");
  if (ioctl(bbc_fp, BBCPCI_IOC_EXT_SER_ON) < 0) startup_test = -1;
  //external mode rates in units of DV pulse rate (200Hz on test box)
  if (ioctl(bbc_fp, BBCPCI_IOC_IRQ_RATE, 1) < 0) startup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_FRAME_RATE, SERIAL_PER_FRAME) < 0)
    startup_test = -1;
#else
  bprintf(startup, "System: BBC generating internal serial numbers");
  if (ioctl(bbc_fp, BBCPCI_IOC_EXT_SER_OFF) < 0) startup_test = -1;
  //internal mode rates in units of bbc clock rate (32MHz or 4MHz) (?)
  //frame rate doesn't need to be specified int his mode
  //TODO don't think this irq rate is right; doesn't matter much in this mode
  if (ioctl(bbc_fp, BBCPCI_IOC_IRQ_RATE, 320000) < 0) startup_test = -1;
#endif
  if (startup_test < 0)
    bprintf(fatal, "System: BBC failed to set synchronization mode");

  //populate nios addresses, based off of tx_struct, derived
  MakeAddressLookups("/data/etc/spider/Nios.map");

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  bprintf(info, "Commands: MCP Command List Version: %s", command_list_serial);
#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
  ReductionInit("/data/etc/spider/ephem.2000");

  bprintf(info, "System: Slow Downlink Initialisation");
  InitSlowDL();

  InitFrameBuffer(&bi0_buffer);
  InitFrameBuffer(&hiGain_buffer);

  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif

  InitialiseFrameFile(argv[1][0]);
  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  /* Allocate the local data buffers */
  RxFrame = balloc(fatal, BiPhaseFrameSize);

  for (i = 0; i < FAST_PER_SLOW; ++i) {
    slow_data[i] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));
    memset(slow_data[i], 0, slowsPerBi0Frame * sizeof(unsigned short));
  }

  /* Find out whether I'm itsy or bitsy */
  BitsyIAm = AmIBitsy();
  if (BitsyIAm) bputs(info, "System: I am Bitsy.\n");
  else bputs(info, "System: I am not Bitsy.\n");

#ifndef BOLOTEST
  pthread_create(&chatter_id, NULL, (void*)&Chatter, (void*)&(fstats.st_size));

  InitSched();
  openMotors();  //open communications with peripherals, creates threads
                 // in motors.c
  openSBSC();  // SBSC - creates thread in sbsc.cpp


  startChrgCtrl(); // create charge controller serial thread
                   // defined in chrgctrl.c
#endif

  bputs(info, "System: Finished Initialisation, waiting for BBC to come up.\n");

  /* mcp used to wait here for a semaphore from the BBC, which makes the
   * presence of these messages somewhat "historical" */

  bputs(info, "System: BBC is up.\n");

  InitTxFrame();

#ifdef USE_XY_THREAD
  pthread_create(&xy_id, NULL, (void*)&StageBus, NULL);
#endif
#ifndef BOLOTEST
  pthread_create(&dgps_id, NULL, (void*)&WatchDGPS, NULL);
#ifndef TEST_RUN
  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
#endif
  //TODO CompressionWriter segfaults on (currently) empty compressstruct
  //pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);
  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
#endif
  pthread_create(&abus_id, NULL, (void*)&ActuatorBus, NULL);

  while (1) {
    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0)
      berror(err, "System: Error on BBC read");

    if (!fill_Rx_frame(in_data))
      bprintf(err, "System: Unrecognised word received from BBC (%08x)",
          in_data);

    if (IsNewFrame(in_data)) {
      if (UsefulnessVeto > 0) {
        UsefulnessVeto--;
        BLASTBusUseful = 0;
      } else BLASTBusUseful = 1;
      if (StartupVeto > 1) {
        --StartupVeto;
      } else {
#ifndef BOLOTEST
        GetACS();
        GetCurrents();
        Pointing();

#endif

        /* Frame sequencing check */
        if (StartupVeto) {
          bputs(info, "System: Startup Veto Ends\n");
          StartupVeto = 0;
          Death = 0;
        } else if (RxFrame[3] != (RxFrameMultiplexIndex + 1) % FAST_PER_SLOW
            && RxFrameMultiplexIndex >= 0) {
          bprintf(err, "System: Frame sequencing error detected: wanted %i, "
              "got %i\n", RxFrameMultiplexIndex + 1, RxFrame[3]);
        }
        RxFrameMultiplexIndex = RxFrame[3];

        /* Save current fastsamp */
        /* NB: internal frame numbers from PCI don't work. TODO: fix! */
        BBFrameIndex = (RxFrame[1] + RxFrame[2] * 0x10000);

        UpdateBBCFrame();
        CommandData.bbcFifoSize = ioctl(bbc_fp, BBCPCI_IOC_BBC_FIONREAD);

        /* pushDiskFrame must be called before PushBi0Buffer to get the slow
           data right */
        pushDiskFrame(RxFrame);
#ifndef BOLOTEST
        if (biphase_is_on) {
          PushFrameBuffer(&bi0_buffer);
          PushFrameBuffer(&hiGain_buffer);
        } else if (biphase_timer < mcp_systime(NULL)) {
          biphase_is_on = 1;
        }

        FillSlowDL();
#endif
        zero();
      }
    }
  }
  return(0);
}
