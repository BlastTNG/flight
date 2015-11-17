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
#include "command_list.h"
#include "command_struct.h"
#include "crc.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "channels_tng.h"
#include "tx.h"
#include "lut.h"

#include <acs.h>
#include <actuators.h>
#include <blast.h>
#include <blast_comms.h>
#include <blast_sip_interface.h>
#include <blast_time.h>
#include <computer_sensors.h>
#include <data_sharing.h>
#include <dsp1760.h>
#include <ec_motors.h>
#include <framing.h>
#include <hwpr.h>
#include <motors.h>
#include <watchdog.h>
#include <xsc_network.h>
#include <xsc_pointing.h>

/* Define global variables */
int StartupVeto = 20;
//flc_ip[0] = south, flc_ip[1] = north, so that flc_ip[SouthIAm] gives other flc
char* flc_ip[2] = {"192.168.1.3", "192.168.1.4"};

int bbc_fp = -1;
unsigned int debug = 0;
short int SouthIAm;
short int InCharge = 0;
short int InChargeSet=0;
struct ACSDataStruct ACSData;

pthread_t watchdog_id;

extern pthread_mutex_t mutex;  //commands.c
extern channel_t channel_list[]; //tx_struct_tng.c
extern derived_tng_t derived_list[];

void Pointing();
void WatchPort(void*);
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

  blast_startup("Thread startup\n");

  fd = open("/data/etc/blast/mcp.log", O_RDONLY|O_NONBLOCK);

  if (fd == -1)
  {
    blast_tfatal("Failed to open /data/etc/blast/mcp.log for reading (%d)\n", errno);
  }

  if (fpos == -1) {
    if (lseek(fd, -500, SEEK_END) == -1)
    {
      if (errno == EINVAL)
      {
	if (lseek(fd, 0, SEEK_SET) == -1)
	{
	  blast_tfatal("Failed to rewind /data/etc/blast/mcp.log (%d)\n", errno);
	}
      } else {
	blast_tfatal("Failed to seek /data/etc/blast/mcp.log (%d)\n", errno);
      }
    }
  } else {
    if (lseek(fd, fpos, SEEK_SET) == -1)
    {
      if (lseek(fd, 0, SEEK_END) == -1)
      {
	blast_tfatal("Failed to rewind /data/etc/blast/mcp.log (%d)\n", errno);
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
        blast_tfatal("Error reading from /data/etc/blast/mcp.log (%d)\n", errno);
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
//        blast_err("Death is reaping the watchdog tickle.");
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

static void close_mcp(int m_code)
{
    fprintf(stderr, "Closing MCP with signal %d\n", m_code);
    watchdog_close();

}

/* Polarity crisis: am I north or south? */
static int AmISouth(int *not_cryo_corner)
{
  char buffer[2];
  *not_cryo_corner = 1;

  if (gethostname(buffer, 1) == -1 && errno != ENAMETOOLONG) {
    berror(err, "System: Unable to get hostname");
  } else if (buffer[0] == 'p') {
    *not_cryo_corner = 0;
    blast_info("System: Cryo Corner Mode Activated\n");
  }

  return (buffer[0] == 's') ? 1 : 0;
}

static void mcp_200hz_routines(void)
{
    store_200hz_acs();
    command_motors();
    write_motor_channels_200hz();

    framing_publish_200hz();
}
static void mcp_100hz_routines(void)
{
    read_100hz_acs();
    Pointing();
//    DoSched();
    update_axes_mode();
    store_100hz_acs();
//    CryoControl(index);
//    BiasControl();
    WriteChatter();
    xsc_control_triggers();

    framing_publish_100hz();
}
static void mcp_5hz_routines(void)
{
    watchdog_ping();
    read_5hz_acs();
    store_5hz_acs();
    write_motor_channels_5hz();
    store_axes_mode_data();
    WriteAux();
    StoreActBus();
    SecondaryMirror();
//    PhaseControl();
    StoreHWPRBus();
    SetGyroMask();
//    ChargeController();
//    ControlPower();
//    VideoTx();
//    cameraFields();

    framing_publish_5hz();
}
static void mcp_1hz_routines(void)
{
    blast_store_cpu_health();
    blast_store_disk_space();
    xsc_write_data(0);
    xsc_write_data(1);
    framing_publish_1hz();
}

int main(int argc, char *argv[])
{
  pthread_t CommandDatacomm1;
  int use_starcams = 0;

  int counter_100hz = 0;
  int counter_5hz=0;
  int counter_1hz=0;
  struct timespec ts;
  struct timespec interval_ts = { .tv_sec = 0,
                                  .tv_nsec = 5000000}; /// 200HZ interval

#ifndef USE_FIFO_CMD
  pthread_t CommandDatacomm2;
#endif

//  pthread_t compression_id;
//  pthread_t isc_id;
//  pthread_t osc_id;
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

  if(geteuid() != 0) {
      fprintf(stderr, "Sorry!  MCP needs to be run with root privileges.  Try `sudo ./mcp`\n");
      exit(0);
  }
  umask(0);  /* clear umask */

  /**
   * Begin logging
   */
  {
      struct tm start_time;
      time_t start_time_s;
      char log_file_name[PATH_MAX];

      start_time_s = time(&start_time_s);
      gmtime_r(&start_time_s, &start_time);

      snprintf(log_file_name, PATH_MAX, "/data/etc/blast/mcp_%02d-%02d-%02d_%02d:%02d.log",
              start_time.tm_mday, start_time.tm_mon + 1 , start_time.tm_year + 1900,
              start_time.tm_hour, start_time.tm_min);

      openMCElog(log_file_name);
  }

  /* register the output function */
  nameThread("Dummy"); //insert dummy sentinel node first
  nameThread("Main");
  buos_use_func(mputs);

#if (TEMPORAL_OFFSET > 0)
  blast_warn("System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  bputs(startup, "System: Startup");

  /* Find out whether I'm north or south */
  SouthIAm = AmISouth(&use_starcams);

  if (SouthIAm)
    bputs(info, "System: I am South.\n");
  else
    bputs(info, "System: I am not South.\n");

  //populate nios addresses, based off of tx_struct, derived
  channels_initialize(channel_list);

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  blast_info("Commands: MCP Command List Version: %s", command_list_serial);
  initialize_blast_comms();
//  initialize_sip_interface();
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

  framing_init(channel_list, derived_list);

  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif

//  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, close_mcp);
  signal(SIGINT, close_mcp);
  signal(SIGTERM, close_mcp);
  signal(SIGPIPE, SIG_IGN);


#ifndef BOLOTEST
  pthread_create(&chatter_id, NULL, (void*)&Chatter, (void*)&(fstats.st_size));

//  InitSched();
  initialize_motors();

#endif

  initialize_CPU_sensors();

  if (use_starcams) {
      xsc_networking_init(0);
      xsc_networking_init(1);
  }


//  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);

//  pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);
//  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
//  pthread_create(&abus_id, NULL, (void*)&ActuatorBus, NULL);

  initialize_data_sharing();
  initialize_watchdog(2);

  clock_gettime(CLOCK_REALTIME, &ts);
  while (1) {
      int ret;
      /// Set our wakeup time
      ts = timespec_add(ts, interval_ts);
      ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

      if (ret && ret != -EINTR)
      {
          blast_err("error while sleeping, code %d (%s)\n", ret, strerror(-ret));
          break;
      }

      if (!counter_1hz--) {
          counter_1hz = 199;
          mcp_1hz_routines();
      }
      if (!counter_5hz--) {
          counter_5hz = 39;
          mcp_5hz_routines();
      }
      if (!counter_100hz--) {
          counter_100hz = 1;
          mcp_100hz_routines();
      }
      mcp_200hz_routines();

  }
  return(0);
}
