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
#include "log.h"
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
#include "scheduler_tng.h"
#include "hawkeyeir.h"
#include "microscroll.h"

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

struct LOGGER logger = {0};
uint8_t * logger_buffer = NULL;
struct tm start_time;
int ResetLog = 0;

linklist_t * linklist_array[MAX_NUM_LINKLIST_FILES] = {NULL};
linklist_t * telemetries_linklist[NUM_TELEMETRIES] = {NULL, NULL, NULL, NULL};
uint8_t * master_superframe_buffer = NULL;
struct Fifo * telem_fifo[NUM_TELEMETRIES] = {&pilot_fifo, &bi0_fifo, &highrate_fifo, &sbd_fifo};
extern linklist_t * ll_hk;

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


void close_mcp(int m_code)
{
    fprintf(stderr, "Closing MCP with signal %d\n", m_code);
    shutdown_mcp = true;
    while (!ready_to_close) usleep(10000);
    watchdog_close();
    shutdown_bias_tone();
    diskmanager_shutdown();
    closeLogger(&logger);
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
    // Set the queue to allow new set
    CommandData.Labjack_Queue.set_q = 1;
    CommandData.Labjack_Queue.lj_q_on = 0;
    for (int h = 0; h < NUM_LABJACKS; h++) {
        CommandData.Labjack_Queue.which_q[h] = 0;
    }
    // init labjacks, first 2 args correspond to the cryo LJs, the next 3 are OF LJs
    // last argument turns commanding on/off
    // arguments are 1/0 0 off 1 on
    // order is CRYO1 CRYO2 OF1 OF2 OF3
    init_labjacks(1, 1, 1, 1, 1, 1);
    mult_labjack_networking_init(LABJACK_MULT_OF, 84, 1);
    mult_labjack_networking_init(LABJACK_MULT_PSS, 84, 1);
    // 7 is for highbay labjack
    labjack_networking_init(7, 14, 1);
    initialize_labjack_commands(7);
    // initializes an array of voltages for load curves
    init_array();
    labjack_networking_init(9, 14, 1);
    initialize_labjack_commands(9);
    // switch to this thread for flight
    mult_initialize_labjack_commands(5);
    // labjack_networking_init(10, 14, 1);
    // initialize_labjack_commands(10);
    ph_thread_t *cmd_thread = mult_initialize_labjack_commands(6);
    ph_thread_join(cmd_thread, NULL);

    return NULL;
}

unsigned int superframe_counter[RATE_END] = {0};

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
    hawkeye_control(1);
    outer_frame_200hz(1);
    process_sun_sensors();
    store_200hz_acs();
    command_motors();
    write_motor_channels_200hz();
    // read_chopper();
    // periodic_cal_control();
    SetGyroMask();
    share_data(RATE_200HZ);
    framing_publish_200hz();
    add_frame_to_superframe(channel_data[RATE_200HZ], RATE_200HZ, master_superframe_buffer,
                            &superframe_counter[RATE_200HZ]);
    cryo_200hz(1);
}
static void mcp_100hz_routines(void)
{
    int i_point = GETREADINDEX(point_index);
    read_100hz_acs();
    PointingData[i_point].recv_shared_data = recv_fast_data();
    Pointing();
    DoSched();
    update_axes_mode();
    store_100hz_acs();
    send_fast_data();
//   BiasControl();
    store_100hz_xsc(0);
    store_100hz_xsc(1);
    write_motor_channels_100hz();
    xsc_control_triggers();
    xsc_decrement_is_new_countdowns(&CommandData.XSC[0].net);
    xsc_decrement_is_new_countdowns(&CommandData.XSC[1].net);
    // write the logs to the frame
    if (logger_buffer) {
        if (ResetLog) {
            resetLogger(&logger);
            ResetLog = 0;
        }
        readLogger(&logger, logger_buffer);
    }

    share_data(RATE_100HZ);
    framing_publish_100hz();
    add_frame_to_superframe(channel_data[RATE_100HZ], RATE_100HZ, master_superframe_buffer,
                            &superframe_counter[RATE_100HZ]);
}
static void mcp_5hz_routines(void)
{
    watchdog_ping();
    // Tickles software WD 2.5x as fast as timeout

    update_sun_sensors();
    // hawkeye_spewer();
    read_5hz_acs();
    store_5hz_acs();
    store_5hz_xsc(0);
    store_5hz_xsc(1);
    write_motor_channels_5hz();
    write_roach_channels_5hz();
    store_axes_mode_data();
    WriteAux();
	WriteAalborgs();
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
    ReadHWPREnc();
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
    if (ready && InCharge) {
        for (int i = 0; i < NUM_TELEMETRIES; i++) {
           memcpy(getFifoWrite(telem_fifo[i]), master_superframe_buffer, superframe->size);
           incrementFifo(telem_fifo[i]);
        }
    }
    share_superframe(master_superframe_buffer);
    labjack_choose_execute();
    auto_cycle_mk2();
    execute_microscroll_functions();
    // all 1hz cryo monitoring 1 on 0 off
    cryo_1hz(1);
    // out frame monitoring (current loops and thermistors) 1 on 0 off
    outer_frame_1hz(1);
    // update_mult_vac();
    // relays arg defines found in relay.h
    relays(3);
    // highbay will be rewritten as all on or off when box is complete
    highbay(1);
    store_1hz_acs();
    // thermal_vac();
    // blast_info("value is %f", labjack_get_value(6, 3));
    blast_store_cpu_health();
    // blast_store_disk_space();
    xsc_control_heaters();
    store_1hz_xsc(0);
    store_1hz_xsc(1);
    write_roach_channels_1hz();
    store_charge_controller_data();
    share_data(RATE_1HZ);
    framing_publish_1hz();
    store_data_hk(master_superframe_buffer);
	for (int i = 0; i < N_AALBORG_VALVES; i++) {
		ControlAalborg(i);
	}
	TestLjWrites();

    add_frame_to_superframe(channel_data[RATE_1HZ], RATE_1HZ, master_superframe_buffer,
                            &superframe_counter[RATE_1HZ]);
}

static void *mcp_main_loop(void *m_arg)
{
#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))


    // Start values indicate the phase (in MCP_FREQ counts) of each channel relative to zero.
    //
    // For 24400 mcp counts per second, there are 50, 100, 122, 244, 4880, 12200, and 24400
    // mcp counts per 488 Hz, 244 Hz, 200 Hz, 100 Hz, 5 Hz, and 1 Hz routine, respectively.
    //
    // Start values are chosen so that all the routines are spaced over the 50 mcp pulses per
    // 488 Hz routine, which is the fastest rate.
    int counter_488hz = 40;
    int counter_244hz = 39; // 10;
    int counter_200hz = 33; // 11;
    int counter_100hz = 27; // 17;
    int counter_5hz = 20; // 23;
    int counter_2hz = 19; // 30;
    int counter_1hz = 1; // 31;

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
  ph_thread_t *dgps_thread = NULL;
  ph_thread_t *lj_init_thread = NULL;
  ph_thread_t *DiskManagerID = NULL;
  ph_thread_t *bi0_send_worker = NULL;

  pthread_t CommandDatacomm1;
  pthread_t CommandDatacomm2;
  pthread_t CommandDataFIFO;
  pthread_t pilot_send_worker;
  pthread_t highrate_send_worker;
  // pthread_t bi0_send_worker;
  int use_starcams = 0;

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
  char log_file_name[PATH_MAX];
  {
      time_t start_time_s;

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

  // telemetry logger
	initLogger(&logger, log_file_name, 1);
	logger_buffer = channels_find_by_name("chatter")->var;

//  initialize_blast_comms();
//  initialize_sip_interface();
  initialize_dsp1760_interface();

  pthread_create(&CommandDataFIFO, NULL, (void*)&WatchFIFO, (void*)flc_ip[SouthIAm]);
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);

#ifndef BOLOTEST
  /* Initialize the Ephemeris */
//  ReductionInit("/data/etc/blast/ephem.2000");
  // framing_init(channel_list, derived_list);
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
  start_cycle_checker();
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
  telemetries_linklist[SBD_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.sbd_linklist_name, linklist_array);

  pthread_create(&pilot_send_worker, NULL, (void *) &pilot_compress_and_send, (void *) telemetries_linklist);
  pthread_create(&highrate_send_worker, NULL, (void *) &highrate_compress_and_send, (void *) telemetries_linklist);
//  pthread_create(&bi0_send_worker, NULL, (void *) &biphase_writer, (void *) telemetries_linklist);
  bi0_send_worker = ph_thread_spawn((void *) &biphase_writer, (void *) telemetries_linklist);


//  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);
  signal(SIGHUP, close_mcp);
  signal(SIGINT, close_mcp);
  signal(SIGTERM, close_mcp);
  signal(SIGPIPE, SIG_IGN);

  DiskManagerID = ph_thread_spawn((void *) &initialize_diskmanager, (void *) NULL);

  InitSched();
  initialize_motors();

// LJ THREAD
  lj_init_thread = ph_thread_spawn(lj_connection_handler, NULL);

  initialize_CPU_sensors();

  // force incharge for test cryo
  // force_incharge();

  if (use_starcams) {
       xsc_networking_init(0);
       xsc_networking_init(1);
       xsc_trigger(0, 0);
       xsc_trigger(1, 0);
  }

  initialize_magnetometer();
  // inits heater setup to nominal operating conditions, used for roach testing safety
  CommandData.Cryo.heater_300mk = 0;
  CommandData.Cryo.charcoal_hs = 1;
  CommandData.Cryo.charcoal = 0;
  CommandData.Cryo.lna_250 = 1;
  CommandData.Cryo.lna_350 = 1;
  CommandData.Cryo.lna_500 = 1;
  CommandData.Cryo.heater_1k = 0;
  CommandData.Cryo.heater_update = 1;


  mag_thread = ph_thread_spawn(monitor_magnetometer, NULL);

  // This is our (BLAST) GPS, used for timing and position.
  gps_thread = ph_thread_spawn(GPSMonitor, &GPSData);

  // This is the DPGS we get over serial from CSBF

  dgps_thread = ph_thread_spawn(DGPSMonitor, NULL);

  // pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  // pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);
#ifndef USE_XY_THREAD
  // for now put ActBus inside ifndef so that only one of Actbus thread and XYbus thread run
  act_thread = ph_thread_spawn(ActuatorBus, NULL);
#endif
//  Turns on software WD 2, which reboots the FC if not tickled
//  initialize_watchdog(2); // Don't want this for testing but put BACK FOR FLIGHT

//  initialize_bias_tone();
  startChrgCtrl(0);
  startChrgCtrl(1);

//  initialize the data sharing server
  data_sharing_init(linklist_array);

// Get attitude and position information from the CSBF GPS
//  initialize_csbf_gps_monitor();

  main_thread = ph_thread_spawn(mcp_main_loop, NULL);
#ifdef USE_XY_THREAD // define should be set in mcp.h
  ph_thread_t *xy_thread = ph_thread_spawn(StageBus, NULL);
#endif
  ph_sched_run();

  blast_info("Joining main thread.");
  ph_thread_join(main_thread, NULL);

  return(0);
}
