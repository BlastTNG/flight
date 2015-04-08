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

#include <mputs.h>
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

#include <framing.h>
#include <dsp1760.h>
#include <ec_motors.h>
#include <blast_comms.h>
#include <blast_sip_interface.h>

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
extern channel_t channel_list[]; //tx_struct_tng.c


void Pointing();
void WatchPort(void*);
void WatchDGPS(void);
void IntegratingStarCamera(void);
void ActuatorBus(void);
void WatchFIFO(void*);          //commands.c
void FrameFileWriter(void);
void CompressionWriter(void);
void StageBus(void);

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

/* gives system time (in s) */
time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}

#ifndef BOLOTEST



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
  channels_initialize(channel_list);

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  bprintf(info, "Commands: MCP Command List Version: %s", command_list_serial);
  initialize_blast_comms();
  initialize_sip_interface();
  initialize_dsp1760_interface();
#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, (void*)flc_ip[SouthIAm]);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
//  ReductionInit("/data/etc/blast/ephem.2000");

  framing_init(channel_list);

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
  initialize_motors();

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
