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

#include "phenom/job.h"
#include "phenom/log.h"
#include "phenom/sysutil.h"

#include "cryostat.h"
#include "chrgctrl.h"
#include "mputs.h"
#include "command_list.h"
#include "command_struct.h"
#include "crc.h"
#include "magnetometer.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "channels_tng.h"
#include "tx.h"
#include "lut.h"
#include "labjack.h"
#include "multiplexed_labjack.h"
#include "sensor_updates.h"

#include "acs.h"
#include "actuators.h"
#include "bias_tone.h"
#include "balance.h"
#include "blast.h"
#include "blast_comms.h"
#include "blast_sip_interface.h"
#include "blast_time.h"
#include "computer_sensors.h"
#include "data_sharing.h"
#include "dsp1760.h"
#include "ec_motors.h"
#include "framing.h"
#include "bi0.h"
#include "hwpr.h"
#include "motors.h"
#include "roach.h"
#include "watchdog.h"
#include "xsc_network.h"
#include "xsc_pointing.h"
#include "xystage.h"
#include "diskmanager_tng.h"

/* Define global variables */
char* flc_ip[2] = {"192.168.1.3", "192.168.1.4"};

int16_t SouthIAm;
int16_t InCharge = 0;
int16_t InChargeSet = 0;

bool shutdown_mcp = false;

void Pointing();
void WatchFIFO(void*);          // commands.c
#ifdef USE_XY_THREAD
void StageBus(void);
#endif

struct chat_buf chatter_buffer;
struct tm start_time;

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

// static void Chatter(void* arg)
// {
//  int fd;
//  char ch;
//  ssize_t ch_got;
//  off_t fpos;
//
//  fpos = *(off_t*)arg;
//
//  nameThread("Chat");
//
//  blast_startup("Thread startup\n");
//
//  fd = open("/data/etc/blast/mcp.log", O_RDONLY|O_NONBLOCK);
//
//  if (fd == -1)
//  {
//    blast_tfatal("Failed to open /data/etc/blast/mcp.log for reading (%d)\n", errno);
//  }
//
//  if (fpos == -1) {
//    if (lseek(fd, -500, SEEK_END) == -1)
//    {
//      if (errno == EINVAL)
//      {
//  if (lseek(fd, 0, SEEK_SET) == -1)
//  {
//    blast_tfatal("Failed to rewind /data/etc/blast/mcp.log (%d)\n", errno);
//  }
//      } else {
//  blast_tfatal("Failed to seek /data/etc/blast/mcp.log (%d)\n", errno);
//      }
//    }
//  } else {
//    if (lseek(fd, fpos, SEEK_SET) == -1)
//    {
//      if (lseek(fd, 0, SEEK_END) == -1)
//      {
//  blast_tfatal("Failed to rewind /data/etc/blast/mcp.log (%d)\n", errno);
//      }
//    }
//  }
//
//  while (read(fd, &ch, 1) == 1 && ch != '\n'); /* Find start of next message */
//
//  chatter_buffer.reading = chatter_buffer.writing = 0;
//      /* decimal 22 is "Synchronous Idle" in ascii */
//  memset(chatter_buffer.msg, 22, sizeof(char) * 20 * 2 * 4);
//
//  while (1)
//  {
//    if (chatter_buffer.writing != ((chatter_buffer.reading - 1) & 0x3))
//    {
//      ch_got = read(fd, chatter_buffer.msg[chatter_buffer.writing], 2 * 20 * sizeof(char));
//      if (ch_got == -1)
//      {
//        blast_tfatal("Error reading from /data/etc/blast/mcp.log (%d)\n", errno);
//      }
//      if (ch_got < (2 * 20 * sizeof(char)))
//      {
//        memset(&(chatter_buffer.msg[chatter_buffer.writing][ch_got]), 22, (2 * 20 * sizeof(char)) - ch_got);
//      }
//      chatter_buffer.writing = ((chatter_buffer.writing + 1) & 0x3);
//    }
//    usleep(100000);
//  }
// }



// void ClearBuffer(struct frameBuffer *buffer) {
//  buffer->i_out = buffer->i_in;
// }
//
// uint16_t  *PopFrameBufferAndSlow(struct frameBuffer *buffer, uint16_t  ***slow) {
//  uint16_t  *frame;
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
// }
//
// uint16_t  *PopFrameBuffer(struct frameBuffer *buffer) {
//  uint16_t  *frame;
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
// }
//
// #endif

// #ifndef BOLOTEST
// static void BiPhaseWriter(void)
// {
//  uint16_t  *frame;
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
// }
//
// #endif

static void close_mcp(int m_code)
{
    fprintf(stderr, "Closing MCP with signal %d\n", m_code);
    shutdown_mcp = true;
    watchdog_close();
    shutdown_bias_tone();
#ifndef NO_KIDS_ROACH
    shutdown_roaches();
#endif
    diskmanager_shutdown();
    ph_sched_stop();
}

/* Polarity crisis: am I north or south? */
/* Right now fc2 == south */
static int AmISouth(int *not_cryo_corner)
{
    char buffer[4];
    *not_cryo_corner = 1;

    if (gethostname(buffer, 3) == -1 && errno != ENAMETOOLONG) {
      berror(err, "System: Unable to get hostname");
    } else if (buffer[0] == 'p') {
      *not_cryo_corner = 0;
      blast_info("System: Cryo Corner Mode Activated\n");
    }

    return ((buffer[0] == 'f') && (buffer[1] == 'c') && (buffer[2] == '2')) ? 1 : 0;
}

static void mcp_488hz_routines(void)
{
#ifndef NO_KIDS_TEST
    write_roach_channels_488hz();
#endif
    framing_publish_488hz();
}

static void mcp_244hz_routines(void)
{
//    write_roach_channels_244hz();

    framing_publish_244hz();
}

static void mcp_200hz_routines(void)
{
    store_200hz_acs();
    command_motors();
    write_motor_channels_200hz();
    #ifdef USE_XY_THREAD
    	read_chopper();
    #endif
    cal_control();

    framing_publish_200hz();
    store_data_200hz();
    build_biphase_frame_200hz(channel_data[RATE_200HZ]);
}
static void mcp_100hz_routines(void)
{
    read_100hz_acs();
    Pointing();
//    DoSched();
    update_axes_mode();
    store_100hz_acs();
//    BiasControl();
    WriteChatter();
    store_100hz_xsc(0);
    store_100hz_xsc(1);
    xsc_control_triggers();
    xsc_decrement_is_new_countdowns(&CommandData.XSC[0].net);
    xsc_decrement_is_new_countdowns(&CommandData.XSC[1].net);

    framing_publish_100hz();
    store_data_100hz();
    build_biphase_frame_1hz(channel_data[RATE_1HZ]);
    build_biphase_frame_100hz(channel_data[RATE_100HZ]);
    push_bi0_buffer();
    // test_dio();
}
static void mcp_5hz_routines(void)
{
    watchdog_ping();
    // update_sun_sensors();
    read_5hz_acs();
    store_5hz_acs();
    write_motor_channels_5hz();
    store_axes_mode_data();
    WriteAux();
    ControlBalance();
    StoreActBus();
    level_control();
    #ifdef USE_XY_THREAD
    StoreStageBus(0);
    #endif
    SecondaryMirror();
//    PhaseControl();
    StoreHWPRBus();
    SetGyroMask();
//    ChargeController();
//    ControlPower();
//    VideoTx();
//    cameraFields();
#ifndef NO_KIDS_TEST
    write_roach_channels_5hz();
#endif

    framing_publish_5hz();
    store_data_5hz();
}
static void mcp_2hz_routines(void)
{
    xsc_write_data(0);
    xsc_write_data(1);
}
static void mcp_1hz_routines(void)
{
    // rec_control();
    // of_control();
    // if_control();
    // heater_control();
    // test_labjacks(0);
    // read_thermometers();
    // test_read();
    blast_store_cpu_health();
    blast_store_disk_space();
    xsc_control_heaters();
    store_1hz_xsc(0);
    store_1hz_xsc(1);
    store_charge_controller_data();
    framing_publish_1hz();
    store_data_1hz();
    // query_mult(0, 48);
    // query_mult(0, 49);
}

static void *mcp_main_loop(void *m_arg)
{
#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))

    int counter_488hz = 1;
    int counter_244hz = 1;
    int counter_200hz = 1;
    int counter_100hz = 1;
    int counter_5hz = 1;
    int counter_2hz = 1;
    int counter_1hz = 1;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    nameThread("Main");

    while (!shutdown_mcp) {
        int ret;
        const struct timespec interval_ts = { .tv_sec = 0,
                                        .tv_nsec = MCP_NS_PERIOD}; /// 200HZ interval
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
        if (ret == EINTR) {
            blast_info("Exiting MCP!");
            break;
        }

        if (ret) {
            blast_err("error while sleeping, code %d (%s)\n", ret, strerror(ret));
            break;
        }

        if (!--counter_1hz) {
            counter_1hz = HZ_COUNTER(1);
            mcp_1hz_routines();
        }
        if (!--counter_2hz) {
            counter_2hz = HZ_COUNTER(2);
            mcp_2hz_routines();
        }
        if (!--counter_5hz) {
            counter_5hz = HZ_COUNTER(5);
            mcp_5hz_routines();
        }
        if (!--counter_100hz) {
            counter_100hz = HZ_COUNTER(100);
            mcp_100hz_routines();
        }
        if (!--counter_200hz) {
            counter_200hz = HZ_COUNTER(200);
            mcp_200hz_routines();
        }
        if (!--counter_244hz) {
            counter_244hz = HZ_COUNTER(244);
            mcp_244hz_routines();
        }
        if (!--counter_488hz) {
            counter_488hz = HZ_COUNTER(488);
            mcp_488hz_routines();
        }
    }

    return NULL;
}

int main(int argc, char *argv[])
{
  ph_thread_t *main_thread = NULL;
  ph_thread_t *act_thread = NULL;
  ph_thread_t *disk_thread = NULL;

  pthread_t CommandDatacomm1;
  pthread_t biphase_writer_id;
  int use_starcams = 0;

#ifndef USE_FIFO_CMD
  pthread_t CommandDatacomm2;
#endif
#ifdef USE_XY_THREAD /* Define should be set in mcp.h */
  // pthread_t xy_id;
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

  if (geteuid() != 0) {
      fprintf(stderr, "Sorry!  MCP needs to be run with root privileges.  Try `sudo ./mcp`\n");
      exit(0);
  }
  umask(0);  /* clear umask */

  ph_library_init();
  ph_nbio_init(4);

  /**
   * Begin logging
   */
  {
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
  nameThread("Dummy"); // insert dummy sentinel node first
  nameThread("Scheduling");

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

#ifdef NO_KIDS_TEST
    blast_warn("Warning: NO_KIDS_TEST flag is set.  No detector functions will be called!");
#endif

  // populate nios addresses, based off of tx_struct, derived
  channels_initialize(channel_list);

  InitCommandData();

  blast_info("Commands: MCP Command List Version: %s", command_list_serial);

  initialize_blast_comms();
// initialize_sip_interface();
  initialize_dsp1760_interface();

#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, (void*)flc_ip[SouthIAm]);
#else
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif
#ifdef USE_XY_THREAD
  // pthread_create(&xy_id, NULL, (void*)&StageBus, NULL);
#endif

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
//  ReductionInit("/data/etc/blast/ephem.2000");

  framing_init(channel_list, derived_list);
  initialize_biphase_buffer();
  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif

#ifndef NO_KIDS_TEST
blast_info("Initializing ROACHes from MCP...");
init_roach();
blast_info("Finished initializing ROACHes...");
#endif

/* blast_info("Initializing Beaglebones from MCP...");
init_beaglebone();
blast_info("Finished initializing Beaglebones..."); */

//  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);
  disk_thread = ph_thread_spawn(diskmanager_thread, NULL);
  signal(SIGHUP, close_mcp);
  signal(SIGINT, close_mcp);
  signal(SIGTERM, close_mcp);
  signal(SIGPIPE, SIG_IGN);

//  InitSched();
  initialize_motors();
  labjack_networking_init(LABJACK_CRYO_1, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
  labjack_networking_init(LABJACK_CRYO_2, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
  // labjack_networking_init(LABJACK_OF_1, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
  // labjack_networking_init(LABJACK_OF_2, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
  // labjack_networking_init(LABJACK_OF_3, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
  // mult_labjack_networking_init(0, 84, 1);

  initialize_labjack_commands(LABJACK_CRYO_1);
  initialize_labjack_commands(LABJACK_CRYO_2);
  // initialize_labjack_commands(LABJACK_OF_1);
  // initialize_labjack_commands(LABJACK_OF_2);
  // initialize_labjack_commands(LABJACK_OF_3);
  // mult_initialize_labjack_commands(0);

  initialize_CPU_sensors();

  if (use_starcams) {
      xsc_networking_init(0);
      xsc_networking_init(1);
  }
  initialize_magnetometer();

  // pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  // pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);

  pthread_create(&biphase_writer_id, NULL, (void*)&biphase_writer, NULL);

  act_thread = ph_thread_spawn(ActuatorBus, NULL);

  initialize_data_sharing();
  initialize_watchdog(2);
  initialize_bias_tone();
  startChrgCtrl(0);

  main_thread = ph_thread_spawn(mcp_main_loop, NULL);
#ifdef USE_XY_THREAD
  ph_thread_t *xy_thread = ph_thread_spawn(StageBus, NULL);
#endif
  ph_sched_run();

  ph_thread_join(main_thread, NULL);

  return(0);
}
