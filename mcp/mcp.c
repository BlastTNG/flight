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
#include <openssl/md5.h>

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
#include "labjack_functions.h"
#include "multiplexed_labjack.h"
#include "sensor_updates.h"

#include "acs.h"
#include "actuators.h"
#include "bias_tone.h"
#include "balance.h"
#include "cryovalves.h"
#include "blast.h"
// #include "blast_comms.h"
#include "blast_time.h"
#include "computer_sensors.h"
#include "diskmanager_tng.h"
#include "dsp1760.h"
#include "ec_motors.h"
#include "framing.h"
#include "gps.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "pilot.h"
#include "highrate.h"
#include "bitserver.h"
#include "bi0.h"
#include "biphase_hardware.h"
#include "data_sharing_server.h"
#include "FIFO.h"
#include "hwpr.h"
#include "motors.h"
#include "roach.h"
#include "relay_control.h"
#include "outer_frame.h"
#include "store_data.h"
#include "watchdog.h"
#include "xsc_network.h"
#include "xsc_pointing.h"
#include "xystage.h"
#include "sip.h"

/* Define global variables */
char* flc_ip[2] = {"192.168.1.3", "192.168.1.4"};

int16_t SouthIAm;
int16_t InCharge = 0;
int16_t InChargeSet = 0;

extern labjack_state_t state[NUM_LABJACKS];

bool shutdown_mcp = false;
bool ready_to_close = false;

void Pointing();
void WatchFIFO(void*);          // commands.c
#ifdef USE_XY_THREAD
void StageBus(void);
#endif

struct chat_buf chatter_buffer;
struct tm start_time;

linklist_t * linklist_array[MAX_NUM_LINKLIST_FILES] = {NULL};
linklist_t * telemetries_linklist[NUM_TELEMETRIES] = {NULL, NULL, NULL};
uint8_t * master_superframe_buffer = NULL;
struct Fifo * telem_fifo[NUM_TELEMETRIES] = {&pilot_fifo, &bi0_fifo, &highrate_fifo};
extern linklist_t * ll_hk;
extern char * ROACH_TYPES[NUM_RTYPES];

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

void close_mcp(int m_code)
{
    fprintf(stderr, "Closing MCP with signal %d\n", m_code);
    shutdown_mcp = true;
    while (!ready_to_close) usleep(10000);
    watchdog_close();
    shutdown_bias_tone();
    // diskmanager_shutdown();
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

void * lj_connection_handler(void *arg) {
    while (!InCharge) {
        sleep(1);
    }
    // LABJACKS
    blast_info("I am now in charge, initializing LJs");
    // init labjacks, first 2 args correspond to the cryo LJs, the next 3 are OF LJs
    // last argument turns commanding on/off
    // arguments are 1/0 0 off 1 on
    // order is CRYO1 CRYO2 OF1 OF2 OF3
    init_labjacks(1, 1, 1, 1, 1, 1);
    mult_labjack_networking_init(LABJACK_MULT_OF, 84, 1);
    // 7 is for highbay labjack
    labjack_networking_init(7, 14, 1);
    initialize_labjack_commands(7);
    // initializes an array of voltages for load curves
    init_array();
    // labjack_networking_init(8, 14, 1);
    // initialize_labjack_commands(8);
    // switch to this thread for flight
    ph_thread_t *cmd_thread = mult_initialize_labjack_commands(6);
    ph_thread_join(cmd_thread, NULL);

    return NULL;
}

unsigned int superframe_counter[RATE_END] = {0};

// distributes and multiplexes commanded roach channels to compressed telemetry fields
void add_roach_tlm_488hz()
{
  static channel_t * tlm[NUM_ROACH_TLM] = {NULL};
  static channel_t * tlm_index[NUM_ROACH_TLM] = {NULL};
  static int first_time = 1;
  static unsigned int RoachId[NUM_ROACH_TLM] = {0};
  static unsigned int KidId[NUM_ROACH_TLM] = {0};
  static unsigned int TypeId[NUM_ROACH_TLM] = {0};
  static unsigned int roach_indices[NUM_ROACH_TLM] = {0};

  int i;

  if (first_time) {
    for (i = 0; i < NUM_ROACH_TLM; i++) {
			char tlm_name[64] = {0};
			snprintf(tlm_name, sizeof(tlm_name), "kid%c_roachN", 65+i);
      tlm[i] = channels_find_by_name(tlm_name);
			snprintf(tlm_name, sizeof(tlm_name), "kid%c_roachN_index", 65+i);
      tlm_index[i] = channels_find_by_name(tlm_name);
    }

    memset(roach_indices, 0xff, NUM_ROACH_TLM*sizeof(unsigned int));
    first_time = 0;
  }

  for (i = 0; i < NUM_ROACH_TLM; i++) {
    if ((roach_indices[i] != CommandData.roach_tlm[i].index) && (strlen(CommandData.roach_tlm[i].name))) {
      roach_indices[i] = CommandData.roach_tlm[i].index;
      read_roach_index(&RoachId[i], &KidId[i], &TypeId[i], roach_indices[i]);
      RoachId[i] -= 1; // switch from 1 to 0 indexing

      if (tlm[i]) blast_info("Telemetering \"%s\" -> \"%s\"", CommandData.roach_tlm[i].name, tlm[i]->field);
    }

    unsigned int i_udp_read = GETREADINDEX(roach_udp[RoachId[i]].index);
    data_udp_packet_t *m_packet = &(roach_udp[RoachId[i]].last_pkts[i_udp_read]);

    // write the roach data to the multiplexed field
    if (tlm[i]) {
      double value = -3.14159;
      if (strcmp(ROACH_TYPES[TypeId[i]], "i") == 0) { // I comes from the UDP packet directly
        value = m_packet->Ival[KidId[i]];
      } else if (strcmp(ROACH_TYPES[TypeId[i]], "q") == 0) { // Q comes from the UDP packet directly
        value = m_packet->Qval[KidId[i]];
      } else if (strcmp(ROACH_TYPES[TypeId[i]], "df") == 0) { // df comes from the frame
        GET_VALUE(channels_find_by_name(CommandData.roach_tlm[i].name), value);
      }

      SET_FLOAT(tlm[i], value);
    }
    // write the multiplex index
    if (tlm_index[i]) {
      SET_INT32(tlm_index[i], roach_indices[i]);
    }
  }
}

static void mcp_488hz_routines(void)
{
#ifndef NO_KIDS_TEST
    write_roach_channels_488hz();
#endif
    add_roach_tlm_488hz();

    share_data(RATE_488HZ);
    framing_publish_488hz();
    add_frame_to_superframe(channel_data[RATE_488HZ], RATE_488HZ, master_superframe_buffer,
                            &superframe_counter[RATE_488HZ]);
}

static void mcp_244hz_routines(void)
{
//    write_roach_channels_244hz();

    share_data(RATE_244HZ);
    framing_publish_244hz();
    add_frame_to_superframe(channel_data[RATE_244HZ], RATE_244HZ, master_superframe_buffer,
                            &superframe_counter[RATE_244HZ]);
}

static void mcp_200hz_routines(void)
{
    store_200hz_acs();
    command_motors();
    write_motor_channels_200hz();
    // read_chopper();
    periodic_cal_control();

    share_data(RATE_200HZ);
    framing_publish_200hz();
    add_frame_to_superframe(channel_data[RATE_200HZ], RATE_200HZ, master_superframe_buffer,
                            &superframe_counter[RATE_200HZ]);
    cryo_200hz(1);
}
static void mcp_100hz_routines(void)
{
    read_100hz_acs();
    Pointing();
//    DoSched();
    update_axes_mode();
    store_100hz_acs();
//   BiasControl();
    WriteChatter();
    store_100hz_xsc(0);
    store_100hz_xsc(1);
    xsc_control_triggers();
    xsc_decrement_is_new_countdowns(&CommandData.XSC[0].net);
    xsc_decrement_is_new_countdowns(&CommandData.XSC[1].net);
    share_data(RATE_100HZ);
    framing_publish_100hz();
    add_frame_to_superframe(channel_data[RATE_100HZ], RATE_100HZ, master_superframe_buffer,
                            &superframe_counter[RATE_100HZ]);
}
static void mcp_5hz_routines(void)
{
    watchdog_ping();
    // Tickles software WD 2.5x as fast as timeout

    // update_sun_sensors();
    read_5hz_acs();
    store_5hz_acs();
    write_motor_channels_5hz();
    store_axes_mode_data();
    WriteAux();
    ControlBalance();
    StoreActBus();
    level_control();
    level_toggle();
    #ifdef USE_XY_THREAD
    StoreStageBus(0);
    #endif
    SecondaryMirror();
//    PhaseControl();
    StoreHWPRBus();
    SetGyroMask();
//    ChargeController();
//    VideoTx();
//    cameraFields();

    share_data(RATE_5HZ);
    framing_publish_5hz();
    add_frame_to_superframe(channel_data[RATE_5HZ], RATE_5HZ, master_superframe_buffer,
                            &superframe_counter[RATE_5HZ]);
}
static void mcp_2hz_routines(void)
{
    if (InCharge) {
      xsc_write_data(0);
      xsc_write_data(1);
    }
}

static void mcp_1hz_routines(void)
{
    int ready = !superframe_counter[RATE_488HZ];
    // int ready = 1;
    // int i = 0;
    // for (i = 0; i < RATE_END; i++) ready = ready && !superframe_counter[i];
    if (ready) {
      for (int i = 0; i < NUM_TELEMETRIES; i++) {
         memcpy(getFifoWrite(telem_fifo[i]), master_superframe_buffer, superframe->size);
         incrementFifo(telem_fifo[i]);
      }
    }
    share_superframe(master_superframe_buffer);
    labjack_choose_execute();
    auto_cycle_mk2();
    // all 1hz cryo monitoring 1 on 0 off
    cryo_1hz(1);
    // out frame monitoring (current loops and thermistors) 1 on 0 off
    outer_frame(1);
    // relays arg defines found in relay.h
    relays(3);
    // highbay will be rewritten as all on or off when box is complete
    highbay(1);
    // thermal_vac();
    // blast_info("value is %f", labjack_get_value(6, 3));
    blast_store_cpu_health();
    // blast_store_disk_space();
    xsc_control_heaters();
    store_1hz_xsc(0);
    store_1hz_xsc(1);
    store_charge_controller_data();
    share_data(RATE_1HZ);
    framing_publish_1hz();
    store_data_hk(master_superframe_buffer);

    add_frame_to_superframe(channel_data[RATE_1HZ], RATE_1HZ, master_superframe_buffer,
                            &superframe_counter[RATE_1HZ]);
    // roach_timestamp_init(1);
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

    while (true) {
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

            // only break out of main loop after all data has been written to mqtt
            if (shutdown_mcp) {
                ready_to_close = true;
                blast_info("Main loop is ready for shutdown\n");
                break;
            }
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
  ph_thread_t *mag_thread = NULL;
  ph_thread_t *gps_thread = NULL;
	ph_thread_t *lj_init_thread = NULL;

  pthread_t CommandDatacomm1;
  pthread_t CommandDatacomm2;
  pthread_t CommandDataFIFO;
  pthread_t DiskManagerID;
  pthread_t pilot_send_worker;
  pthread_t highrate_send_worker;
  pthread_t bi0_send_worker;
  // pthread_t biphase_writer_id;
  int use_starcams = 0;

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
  /* Note that South == fc2 */
  SouthIAm = AmISouth(&use_starcams);

  if (SouthIAm)
    bputs(info, "System: I am South.\n");
  else
    bputs(info, "System: I am not South.\n");

  // populate nios addresses, based off of tx_struct, derived
  channels_initialize(channel_list);

  InitCommandData(); // This should happen before all other threads

  blast_info("Commands: MCP Command List Version: %s", command_list_serial);


//  initialize_blast_comms();
//  initialize_sip_interface();
  initialize_dsp1760_interface();

  pthread_create(&CommandDataFIFO, NULL, (void*)&WatchFIFO, (void*)flc_ip[SouthIAm]);
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#ifdef USE_XY_THREAD
  // pthread_create(&xy_id, NULL, (void*)&StageBus, NULL);
#endif

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
//  ReductionInit("/data/etc/blast/ephem.2000");
  framing_init(channel_list, derived_list);
  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif

#ifndef NO_KIDS_TEST
  blast_info("Initializing ROACHes from MCP...");
  roach_udp_networking_init();
  init_roach(0);
  init_roach(1);
  init_roach(2);
  init_roach(3);
  init_roach(4);
  blast_info("Finished initializing ROACHes...");
#endif

/* blast_info("Initializing Beaglebones from MCP...");
init_beaglebone();
blast_info("Finished initializing Beaglebones..."); */

  // initialize superframe FIFO
  master_superframe_buffer = calloc(1, superframe->size);
  for (int i = 0; i < NUM_TELEMETRIES; i++) { // initialize all fifos
    allocFifo(telem_fifo[i], 3, superframe->size);
  }

  // load all the linklists
  load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, linklist_array, 0);
  generate_housekeeping_linklist(linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array), ALL_TELEMETRY_NAME);
  linklist_generate_lookup(linklist_array);
  ll_hk = linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);

  // load the latest linklist into telemetry
  telemetries_linklist[PILOT_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
  telemetries_linklist[BI0_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.bi0_linklist_name, linklist_array);
  telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.highrate_linklist_name, linklist_array);

  linklist_t * testll = generate_roach_udp_linklist("roach1.ll", 0);
  write_superframe_format(testll->superframe, "roach1.sf");

  pthread_create(&pilot_send_worker, NULL, (void *) &pilot_compress_and_send, (void *) telemetries_linklist);
  pthread_create(&highrate_send_worker, NULL, (void *) &highrate_compress_and_send, (void *) telemetries_linklist);
  pthread_create(&bi0_send_worker, NULL, (void *) &biphase_writer, (void *) telemetries_linklist);

//  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);
  signal(SIGHUP, close_mcp);
  signal(SIGINT, close_mcp);
  signal(SIGTERM, close_mcp);
  signal(SIGPIPE, SIG_IGN);

  // pthread_create(&DiskManagerID, NULL, (void*)&initialize_diskmanager, NULL);

//  InitSched();
  initialize_motors();

// LJ THREAD
  lj_init_thread = ph_thread_spawn(lj_connection_handler, NULL);

  initialize_CPU_sensors();

  // force incharge for test cryo
  // force_incharge();

  if (use_starcams) {
       xsc_networking_init(0);
       xsc_networking_init(1);
  }

  initialize_magnetometer();

  mag_thread = ph_thread_spawn(monitor_magnetometer, NULL);
  gps_thread = ph_thread_spawn(GPSMonitor, &GPSData);

  // pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  // pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);

  act_thread = ph_thread_spawn(ActuatorBus, NULL);

//  Turns on software WD 2, which reboots the FC if not tickled
//  initialize_watchdog(2); // Don't want this for testing but put BACK FOR FLIGHT

//  initialize_bias_tone();
  startChrgCtrl(0);
  startChrgCtrl(1);

//  initialize the data sharing server
  data_sharing_init(linklist_array);

  main_thread = ph_thread_spawn(mcp_main_loop, NULL);
#ifdef USE_XY_THREAD
  ph_thread_t *xy_thread = ph_thread_spawn(StageBus, NULL);
#endif
  ph_sched_run();

  blast_info("Joining main thread.");
  ph_thread_join(main_thread, NULL);

  return(0);
}
