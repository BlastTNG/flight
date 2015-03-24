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

#include "blast.h"
#include "command_list.h"
#include "command_struct.h"
#include "crc.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "starpos.h"
#include "channels_tng.h"
#include "tx.h"
#include "flcdataswap.h"
#include "lut.h"



/* Define global variables */
int StartupVeto = 20;
//flc_ip[0] = south, flc_ip[1] = north, so that flc_ip[SouthIAm] gives other flc
char* flc_ip[2] = {"192.168.1.6", "192.168.1.5"};

int bbc_fp = -1;
unsigned int debug = 0;
short int SouthIAm;
short int InCharge = 0;
short int InChargeSet=0;
struct ACSDataStruct ACSData;

pthread_t watchdog_id;

extern pthread_mutex_t mutex;  //commands.c

void Pointing();
void WatchPort(void*);
void WatchDGPS(void);
void IntegratingStarCamera(void);
void ActuatorBus(void);
void WatchFIFO(void*);          //commands.c
void FrameFileWriter(void);
void CompressionWriter(void);
void StageBus(void);
void openSBSC(void);

void InitialiseFrameFile(char);
void pushDiskFrame(unsigned short *RxFrame);
void ShutdownFrameFile();

void updateSlowDL(); // common/slowdl.c

void InitSched();

static FILE* logfile = NULL;

//#ifndef BOLOTEST
//struct frameBuffer {
//  int i_in;
//  int i_out;
//  unsigned short *framelist[BI0_FRAME_BUFLEN];
//  unsigned short** slow_data_list[BI0_FRAME_BUFLEN];
//};
//
//static struct frameBuffer bi0_buffer;
//struct frameBuffer hiGain_buffer;
//
//#endif

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


double CalibrateAD590(int counts)
{
  double t = M_16_AD590 * (counts + B_16_AD590);

  /* if t < -73C or t > 67C, assume AD590 is broken */
  if (t < 170)
    t = -1;
  else if (t > 360)
    t = -2;

  return t;
}

double CalibrateThermister(int counts)
{
  static struct LutType temperature_lut =
     {"/data/etc/blast/thermistor.lut", 0, NULL, NULL, 0};
  static int firsttime = 1;
  
  double vt;
  double t;
  
  if (firsttime) {
    firsttime =0;
    LutInit(&temperature_lut);
  }
  
  vt = M_16T * (counts + B_16T);
  t = LutCal(&temperature_lut, vt) + 273.15;

  /* if t < -73C or t > 67C, assume AD590 is broken */
  if (t < 170)
    t = -1;
  else if (t > 360)
    t = -2;

  return t;
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

  fd = open("/data/etc/blast/mcp.log", O_RDONLY|O_NONBLOCK);

  if (fd == -1)
  {
    bprintf(tfatal, "Failed to open /data/etc/blast/mcp.log for reading (%d)\n", errno);
  }

  if (fpos == -1) {
    if (lseek(fd, -500, SEEK_END) == -1)
    {
      if (errno == EINVAL)
      {
	if (lseek(fd, 0, SEEK_SET) == -1)
	{
	  bprintf(tfatal, "Failed to rewind /data/etc/blast/mcp.log (%d)\n", errno);
	}
      } else {
	bprintf(tfatal, "Failed to seek /data/etc/blast/mcp.log (%d)\n", errno);
      }
    }
  } else {
    if (lseek(fd, fpos, SEEK_SET) == -1)
    {
      if (lseek(fd, 0, SEEK_END) == -1)
      {
	bprintf(tfatal, "Failed to rewind /data/etc/blast/mcp.log (%d)\n", errno);
      }
    }
  }

  while (read(fd, &ch, 1) == 1 && ch != '\n'); /* Find start of next message */

  chatter_buffer.reading = chatter_buffer.writing = 0;
      /* decimal 22 is "Synchronous Idle" in ascii */
  memset(chatter_buffer.msg, 22, sizeof(char) * 20 * 2 * 4);

  while (1)
  {
    if (chatter_buffer.writing != ((chatter_buffer.reading - 1) & 0x3))
    {
      ch_got = read(fd, chatter_buffer.msg[chatter_buffer.writing], 2 * 20 * sizeof(char));
      if (ch_got == -1)
      {
        bprintf(tfatal, "Error reading from /data/etc/blast/mcp.log (%d)\n", errno);
      }
      if (ch_got < (2 * 20 * sizeof(char)))
      {
        memset(&(chatter_buffer.msg[chatter_buffer.writing][ch_got]), 22, (2 * 20 * sizeof(char)) - ch_got);
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
  double pss1_i1, pss1_i2, pss1_i3, pss1_i4;
  double pss2_i1, pss2_i2, pss2_i3, pss2_i4;
  double pss3_i1, pss3_i2, pss3_i3, pss3_i4;
  double pss4_i1, pss4_i2, pss4_i3, pss4_i4;
  double vel_rw;
  double res_piv;
  int hwpr_pot;

  static channel_t* ifElgyAddr;
  static channel_t* ifRollgyAddr;
  static channel_t* ifYawgyAddr;
  static channel_t* elRawEncAddr;
  static channel_t* elRawIfClinAddr;
  static channel_t* xMagAddr;
  static channel_t* yMagAddr;
  static channel_t* zMagAddr;
  static channel_t* velRWAddr;
  static channel_t* resPivAddr;
  static channel_t* v11PssAddr;
  static channel_t* v21PssAddr;
  static channel_t* v31PssAddr;
  static channel_t* v41PssAddr;
  static channel_t* v12PssAddr;
  static channel_t* v22PssAddr;
  static channel_t* v32PssAddr;
  static channel_t* v42PssAddr;
  static channel_t* v13PssAddr;
  static channel_t* v23PssAddr;
  static channel_t* v33PssAddr;
  static channel_t* v43PssAddr;
  static channel_t* v14PssAddr;
  static channel_t* v24PssAddr;
  static channel_t* v34PssAddr;
  static channel_t* v44PssAddr;
  static channel_t* potHwprAddr;

  unsigned int rx_frame_index = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elRawEncAddr = channels_find_by_name("el_raw_enc");
    elRawIfClinAddr = channels_find_by_name("el_raw_if_clin");
    ifElgyAddr = channels_find_by_name("ifel_gy");
    ifRollgyAddr = channels_find_by_name("ifroll_gy");
    ifYawgyAddr = channels_find_by_name("ifyaw_gy");
    xMagAddr = channels_find_by_name("x_mag");
    yMagAddr = channels_find_by_name("y_mag");
    zMagAddr = channels_find_by_name("z_mag");
    velRWAddr = channels_find_by_name("vel_rw");
    resPivAddr = channels_find_by_name("res_piv");
    v11PssAddr = channels_find_by_name("v1_1_pss");
    v21PssAddr = channels_find_by_name("v2_1_pss");
    v31PssAddr = channels_find_by_name("v3_1_pss");
    v41PssAddr = channels_find_by_name("v4_1_pss");
    v12PssAddr = channels_find_by_name("v1_2_pss");
    v22PssAddr = channels_find_by_name("v2_2_pss");
    v32PssAddr = channels_find_by_name("v3_2_pss");
    v42PssAddr = channels_find_by_name("v4_2_pss");
    v13PssAddr = channels_find_by_name("v1_3_pss");
    v23PssAddr = channels_find_by_name("v2_3_pss");
    v33PssAddr = channels_find_by_name("v3_3_pss");
    v43PssAddr = channels_find_by_name("v4_3_pss");
    v14PssAddr = channels_find_by_name("v1_4_pss");
    v24PssAddr = channels_find_by_name("v2_4_pss");
    v34PssAddr = channels_find_by_name("v3_4_pss");
    v44PssAddr = channels_find_by_name("v4_4_pss");
    potHwprAddr = channels_find_by_name("pot_hwpr");
  }

  ///TODO: Add MAG read functions
  ///TODO: Add Clin read functions
  ///TODO: Add PSS read functions

}

/* sole purpose of following function is to add a field that reads the total current */

static void GetCurrents(unsigned short *RxFrame)
{

  double i_trans;
  double i_das;
  double i_acs;
  double i_rec;
  double i_sc;
  double i_sbsc;
  double i_step;
  double i_flc;
  double i_gy;
  double i_rw;
  double i_el;
  double i_piv;
  double i_tot;

  static channel_t* i_transAddr;
  static channel_t* i_dasAddr;
  static channel_t* i_acsAddr;
  static channel_t* i_recAddr;
  static channel_t* i_scAddr;
  static channel_t* i_sbscAddr;
  static channel_t* i_stepAddr;
  static channel_t* i_flcAddr;
  static channel_t* i_gyAddr;
  static channel_t* i_rwAddr;
  static channel_t* i_elAddr;
  static channel_t* i_pivAddr;

  static channel_t* i_transNios;
  static channel_t* i_dasNios;
  static channel_t* i_acsNios;
  static channel_t* i_recNios;
  static channel_t* i_scNios;
  static channel_t* i_sbscNios;
  static channel_t* i_stepNios;
  static channel_t* i_flcNios;
  static channel_t* i_gyNios;
  static channel_t* i_rwNios;
  static channel_t* i_elNios;
  static channel_t* i_pivNios;

  static channel_t* i_totNios;

  static int firsttime = 1;

  if (firsttime) {
  
    firsttime = 0;

    i_transAddr = channels_find_by_name("i_trans");
    i_dasAddr = channels_find_by_name("i_das");
    i_acsAddr = channels_find_by_name("i_acs");
    i_recAddr = channels_find_by_name("i_rec");
    i_scAddr = channels_find_by_name("i_sc");
    i_sbscAddr = channels_find_by_name("i_sbsc");
    i_stepAddr = channels_find_by_name("i_step");
    i_flcAddr = channels_find_by_name("i_flc");
    i_gyAddr = channels_find_by_name("i_gy");
    i_rwAddr = channels_find_by_name("i_rw");
    i_elAddr = channels_find_by_name("i_el");
    i_pivAddr = channels_find_by_name("i_piv");

    i_transNios = channels_find_by_name("i_trans");
    i_dasNios = channels_find_by_name("i_das");
    i_acsNios = channels_find_by_name("i_acs");
    i_recNios = channels_find_by_name("i_rec");
    i_scNios = channels_find_by_name("i_sc");
    i_sbscNios = channels_find_by_name("i_sbsc");
    i_stepNios = channels_find_by_name("i_step");
    i_flcNios  = channels_find_by_name("i_flc");
    i_gyNios = channels_find_by_name("i_gy");
    i_rwNios = channels_find_by_name("i_rw");
    i_elNios = channels_find_by_name("i_el");
    i_pivNios = channels_find_by_name("i_piv");

    i_totNios = channels_find_by_name("i_tot");

  }

  i_trans = ReadCalData(i_transAddr);
  i_das = ReadCalData(i_dasAddr);
  i_acs = ReadCalData(i_acsAddr);
  i_rec = ReadCalData(i_recAddr);
  i_sc = ReadCalData(i_scAddr);
  i_sbsc = ReadCalData(i_sbscAddr);
  i_step = ReadCalData(i_stepAddr);
  i_flc = ReadCalData(i_flcAddr);
  i_gy = ReadCalData(i_gyAddr);
  i_rw = ReadCalData(i_rwAddr);
  i_el = ReadCalData(i_elAddr);
  i_piv = ReadCalData(i_pivAddr);

  i_tot = i_trans + i_das + i_acs + i_rec + i_sc + i_sbsc + i_step + i_flc + i_gy + i_rw + i_el + i_piv;

  SET_VALUE(i_totNios, 1000*i_tot);

}

#endif


//#ifndef BOLOTEST
//static void WatchDog (void)
//{
//  nameThread("WDog");
//  bputs(startup, "Startup\n");
//
//  /* Allow other threads to kill this one at any time */
//  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
//
//  if (ioperm(0x378, 0x0F, 1) != 0)
//    berror(tfatal, "Error setting watchdog permissions");
//  ioperm(0x80, 1, 1);
//
//  for (;;) {
//    outb(0xAA, 0x378);
//    usleep(10000);
//    outb(0x55, 0x378);
//    usleep(10000);
//  }
//}
//
//
//void ClearBuffer(struct frameBuffer *buffer) {
//  buffer->i_out = buffer->i_in;
//}
//
//unsigned short *PopFrameBufferAndSlow(struct frameBuffer *buffer, unsigned short ***slow) {
//  unsigned short *frame;
//  int i_out = buffer->i_out;
//
//  if (buffer->i_in == i_out) { // no data
//    return (NULL);
//  }
//  frame = buffer->framelist[i_out];
//
//  *slow = buffer->slow_data_list[i_out];
//
//  i_out++;
//  if (i_out>=BI0_FRAME_BUFLEN) {
//    i_out = 0;
//  }
//  buffer->i_out = i_out;
//  return (frame);
//}
//
//unsigned short *PopFrameBuffer(struct frameBuffer *buffer) {
//  unsigned short *frame;
//  int i_out = buffer->i_out;
//
//  if (buffer->i_in == i_out) { // no data
//    return (NULL);
//  }
//  frame = buffer->framelist[i_out];
//  i_out++;
//  if (i_out>=BI0_FRAME_BUFLEN) {
//    i_out = 0;
//  }
//  buffer->i_out = i_out;
//  return (frame);
//}
//
//#endif

//#ifndef BOLOTEST
//static void BiPhaseWriter(void)
//{
//  unsigned short *frame;
//
//  nameThread("Bi0");
//  bputs(startup, "Startup\n");
//
//  while (!biphase_is_on)
//    usleep(10000);
//
//  bputs(info, "Veto has ended.  Here we go.\n");
//
//  while (1) {
//    frame = PopFrameBuffer(&bi0_buffer);
//
//    if (!frame) {
//      /* Death meausres how long the BiPhaseWriter has gone without receiving
//       * any data -- an indication that we aren't receiving FSYNCs from the
//       * BLASTBus anymore */
//      if (InCharge && (++Death > 25)) {
//        bprintf(err, "Death is reaping the watchdog tickle.");
//        pthread_cancel(watchdog_id);
//      }
//      usleep(10000); // 100 Hz
//    } else {
//      write_to_biphase(frame);
//      if (Death > 0) {
//        Death = 0;
//      }
//    }
//  }
//}
//
//#endif


/* Polarity crisis: am I north or south? */
static int AmISouth(int *not_cryo_corner)
{
  char buffer[2];
  *not_cryo_corner = 1;

  if (gethostname(buffer, 1) == -1 && errno != ENAMETOOLONG) {
    berror(err, "System: Unable to get hostname");
  } else if (buffer[0] == 'p') {
    *not_cryo_corner = 0;
    bprintf(info, "System: Cryo Corner Mode Activated\n");
  }

  return (buffer[0] == 's') ? 1 : 0;
}


int main(int argc, char *argv[])
{
  unsigned int in_data, i;
  unsigned short* RxFrame;
  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  pthread_t abus_id;
  int use_starcams = 1;

#ifndef USE_FIFO_CMD
  pthread_t CommandDatacomm2;
#endif

#ifndef BOLOTEST
  pthread_t compression_id;
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

  if ((logfile = fopen("/data/etc/blast/mcp.log", "a")) == NULL) {
    berror(err, "System: Can't open log file");
    fstats.st_size = -1;
  } else {
    if (fstat(fileno(logfile), &fstats) < 0)
      fstats.st_size = -1;
    fputs("!!!!!! LOG RESTART !!!!!!\n", logfile);
  }


  /* register the output function */
  nameThread("Dummy"); //insert dummy sentinel node first
  nameThread("Main");
  buos_use_func(mputs);

#if (TEMPORAL_OFFSET > 0)
  bprintf(warning, "System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  bputs(startup, "System: Startup");

  /* Find out whether I'm north or south */
//  SouthIAm = AmISouth(&use_starcams);

  if (SouthIAm)
    bputs(info, "System: I am South.\n");
  else
    bputs(info, "System: I am not South.\n");


//#ifndef BOLOTEST
//  /* Watchdog */
//  pthread_create(&watchdog_id, NULL, (void*)&WatchDog, NULL);
//#endif


  //populate nios addresses, based off of tx_struct, derived
  channels_initialize("/data/etc/blast/channel.map");

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  bprintf(info, "Commands: MCP Command List Version: %s", command_list_serial);
#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, (void*)flc_ip[SouthIAm]);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
//  ReductionInit("/data/etc/blast/ephem.2000");

  bprintf(info, "System: Slow Downlink Initialisation");

//  InitFrameBuffer(&bi0_buffer);
//  InitFrameBuffer(&hiGain_buffer);

  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif

//  InitialiseFrameFile(argv[1][0]);
//  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

//  signal(SIGHUP, CloseBBC);
//  signal(SIGINT, CloseBBC);
//  signal(SIGTERM, CloseBBC);
//  signal(SIGPIPE, SIG_IGN);


#ifndef BOLOTEST
  pthread_create(&chatter_id, NULL, (void*)&Chatter, (void*)&(fstats.st_size));

//  InitSched();
  openMotors();  //open communications with peripherals, creates threads
                 // in motors.c

#endif

  bputs(info, "System: Finished Initialisation, waiting for BBC to come up.\n");

  /* mcp used to wait here for a semaphore from the BBC, which makes the
   * presence of these messages somewhat "historical" */

  bputs(info, "System: BBC is up.\n");

//  InitTxFrame(RxFrame);

#ifdef USE_XY_THREAD
  pthread_create(&xy_id, NULL, (void*)&StageBus, NULL);
#endif
#ifndef BOLOTEST
//  pthread_create(&dgps_id, NULL, (void*)&WatchDGPS, NULL);
//  if (use_starcams) {
//    pthread_create(&isc_id, NULL, (void*)&IntegratingStarCamera, (void*)0);
//    pthread_create(&osc_id, NULL, (void*)&IntegratingStarCamera, (void*)1);
//  }
//
//  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);

//  pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);
//  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
#endif
//  pthread_create(&abus_id, NULL, (void*)&ActuatorBus, NULL);

//  start_flc_data_swapper(flc_ip[SouthIAm]);

  while (1) {
      sleep(1);
#ifndef BOLOTEST
//        GetACS(RxFrame);
//        GetCurrents(RxFrame);
        Pointing();

#endif


        /* pushDiskFrame must be called before PushBi0Buffer to get the slow
           data right */
//        pushDiskFrame(RxFrame);
#ifndef BOLOTEST
//        if (biphase_is_on) {
//          PushFrameBuffer(&bi0_buffer, RxFrame);
//          PushFrameBuffer(&hiGain_buffer, RxFrame);
//        } else if (biphase_timer < mcp_systime(NULL)) {
//          biphase_is_on = 1;
//        }
//        updateSlowDL();

#endif
//        zero(RxFrame);


  }
  return(0);
}
