/* MPC: MCE-PCM communicator
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include "mpc.h"
#include "mputs.h"
#include "mpc_proto.h"
#include "udp.h"
#include "tes.h"

#include <sys/statvfs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

/* some timing constants; in the main loop, timing is done using the udp poll
 * timeout.  As a result, all timings are approximate (typically lower bounds)
 * and the granularity of timing is the UDP_TIMEOUT
 */
#define UDP_TIMEOUT    100 /* milliseconds */

/* wait time between sending the ping to an unresponsive PCM */
#define INIT_TIMEOUT 60000 /* milliseconds */

/* sets the rate at which slow data are sent to PCM.  Since PCM multiplexes
 * these over the MCE count into a slow channel, this can happen "rarely".
 * If we assume the PCM frame rate is 100 Hz, it's once every 1.2 seconds;
 * this is slightly faster than that, probably.
 */
#define SLOW_TIMEOUT   800 /* milliseconds */

/* Semeuphoria */

/* UDP socket */
static int sock;

/* The number of the attached MCE */
int nmce = 0;

/* Are we running in fake mode */
int fake;

/* The current MCE data mode */
int data_mode = -1;

/* Initialisation veto */
static int init = 1;

/* Data return veto */
static int veto = 0;

/* The slow data struct */
static struct mpc_slow_data slow_dat;

/* the list of bolometers to send to PCM */
static uint16_t bset_num = 0xFFFF;
static int ntes = 0;
static int16_t tes[NUM_ROW * NUM_COL];

/* the turnaround flag */
int in_turnaround = 0;

/* time to ask PCM to power cycle the MCE */
int power_cycle_mce = 0;

/* command "veto" -- actually, all this does is veto the proactive actions
 * associated with failed commands */
int command_veto = 0; 

/* make and send fake data -- we only make data mode 11 data, which is static */
#define HEADER_SIZE 43
#define FAKE_DATA_RATE   6667 /* microseconds -- this is approximate */
static void *fake_data(void *dummy)
{
  uint32_t framenum = 0x37183332;
  int i, j;
  uint32_t mode11[NUM_COL * NUM_ROW + HEADER_SIZE];

  char data[UDP_MAXSIZE];

  nameThread("FAKE");
  bprintf(info, "Running FAKE data at %.1f Hz", 1e6 / FAKE_DATA_RATE);

  /* generate the mode 11 data -- this is simply:
   *
   *  0000 0000 0000 0000 0000 000r rrrr rccc
   *
   *  where :
   *
   *  r = the row number
   *  c = the column number
   */
  for (i = 0; i < NUM_COL; ++i)
    for (j = 0; j < NUM_ROW; ++j)
      mode11[HEADER_SIZE + i + j * NUM_COL] = (i & 0x7) | ((j & 0x3F) << 3);

  /* wait for initialisation from PCM */
  while (init)
    sleep(1);

  /* send data packets at half the "frame rate" */
  for (;;) {
    do_frame(mode11, (HEADER_SIZE + NUM_COL * NUM_ROW) * sizeof(uint32_t));
    /* we just don't bother sending anything if nothing is requested */
    if (!veto && ntes > 0) {
      size_t len = mpc_compose_tes(mode11, framenum, bset_num, nmce, ntes, tes,
          data);
      udp_bcast(sock, MCESERV_PORT, len, data);
    }
    framenum += 2;
    usleep(FAKE_DATA_RATE * 2);
  }

  return NULL;
}

/* ask PCM to do something, maybe;
 * also deal with PCM having done something, maybe
 */
static void pcm_special(size_t len, const char *data_in)
{
  static int power_cycle_wait = 0;

  if (data_in) {
    /* reply acknowledgement from PCM */
    int power_state = MPCPROTO_POWER_NOP;

    if (mce_decompose_notice(nmce, &in_turnaround, &power_state, len, data_in))
      return;

    if (power_state == MPCPROTO_POWER_OFF) {
      /* MCE power turned off; if we were power cycling we should stop */
      power_cycle_mce = 0;
      power_cycle_wait = 0;
      /* "veto" commanding for a while */
      command_veto = 1000000; /* microseconds */
    } else if (power_state == MPCPROTO_POWER_CYC) {
      /* MCE power is cycling; if we were power cycling we should stop */
      power_cycle_mce = 0;
      power_cycle_wait = 0;
      /* "veto" commanding for a while */
      command_veto = 1000000; /* microseconds */
    } else if (power_state == MPCPROTO_POWER_ON) {
      /* MCE power on; disable the command veto, if any */
      command_veto = 0;
    }
  } else {
    /* send a request, if necessary */
    char data[UDP_MAXSIZE];
    int power_cycle = 0;
    int need_send = 0;

    /* collect pending requests */
    if (power_cycle_mce && power_cycle_wait <= 0) {
      power_cycle = 1;
      need_send = 1;
      power_cycle_wait = 1000000; /* microseconds */
    } else if (power_cycle_wait > 0)
      power_cycle_wait -= UDP_TIMEOUT;

    if (need_send) {
      len = mpc_compose_pcmreq(nmce, power_cycle, data);
      udp_bcast(sock, MCESERV_PORT, len, data);
    }
  }
}

/* send slow data to PCM */
static void send_slow_data(char *data)
{
  struct timeval tv;
  struct statvfs buf;
  size_t len;

  slow_dat.data_mode = data_mode;

  /* disk free -- units are 2**24 bytes = 16 MB */
  if (statvfs(MAS_DATA_ROOT, &buf) == 0)
    slow_dat.df =
      (uint16_t)(((unsigned long long)buf.f_bfree * buf.f_bsize) >> 24);

  /* time -- this wraps around ~16 months after the epoch */
  gettimeofday(&tv, NULL);
  slow_dat.time = (tv.tv_sec - MPC_EPOCH) * 100 + tv.tv_usec / 10000;

  /* make packet and send */
  len = mpc_compose_slow(&slow_dat, nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data);
}

/* Figure out which MCE we're attached to, also whether we're running in fake
 * mode */
static void get_nmce(int argc, const char **argv)
{
  int i;
  char array_id[1024] = {0};

#ifdef FAKE_MAS
  fake = 1;
#else
  fake = 0;
#endif

  /* Look for "fake" on the command line */
  for (i = 1; i < argc; ++i) {
    if (strncmp(argv[i], "fake", 4) == 0) {
      fake = 1;
      if (argv[i][4] == '=') {
        nmce = argv[i][5] - '0';
        if (nmce < 0 || nmce > 5) {
          bprintf(warning, "Ignoring bad fake MCE number on command line.\n");
          nmce = 0;
        }
      }
      break;
    }
  }

  /* figure out which MCE we're attached to via the array_id */
  if (!fake) {
    char map_entry[1024];
    const char *file = MAS_DATA_ROOT "/array_id";
    FILE *stream = fopen(file, "r");

    if (stream == NULL) {
      berror(err, "Unable to open array ID file: %s", file);
      goto BAD_ARRAY_ID;
    } else if (fgets(array_id, 1024, stream) == NULL) {
      berror(err, "Can't read array id");
      fclose(stream);
      goto BAD_ARRAY_ID;
    }
    fclose(stream);

    /* look this array_id up in the map file */
    file = MPC_ETC_DIR "/mpc_array_map";
    stream = fopen(file, "r");
    if (stream == NULL) {
      berror(err, "Unable to open array ID map: %s", file);
      goto BAD_ARRAY_ID;
    }

    for (i = 0; i < 6; ++i) {
      if (fgets(map_entry, 1024, stream) == NULL) {
        bprintf(err, "Missing entry #%i in array map", i);
        fclose(stream);
        goto BAD_ARRAY_ID;
      }
      if (strncmp(map_entry, array_id, 1024) == 0)
        break;
    }
    fclose(stream);

    if (i < 6) {
      nmce = i;
    } else {
      bprintf(err, "Array ID not found in map file: %s", array_id);
BAD_ARRAY_ID:
      bprintf(err, "Reverting to FAKE MCE");
      fake = 1;
    }
  }

  if (fake) {
    bprintf(info, "Running FAKE MCE #%i", nmce);
  } else {
    bprintf(info, "Controlling MCE #%i.  Array ID: %s", nmce, array_id);
  }
}

int main(int argc, const char **argv)
{
  int port, type;
  ssize_t n;
  size_t len;

  pthread_t data_thread;

  int init_timer = 0;
  int slow_timer = 0;

  char peer[UDP_MAXHOST];
  char data[UDP_MAXSIZE];

  buos_use_func(mputs);
  nameThread("Main");

  if ((logfile = fopen(MPC_ETC_DIR "/mpc.log", "a")) == NULL)
    berror(err, "Can't open log file");
  else
    fputs("!!!!!! LOG RESTART !!!!!!\n", logfile);

  bputs(startup, "This is MPC.\n");
  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC subsystem.");

  /* bind to the UDP port */
  sock = udp_bind_port(MPC_PORT, 1);

  /* Figure out our MCE number and check for fake mode */
  get_nmce(argc, argv);

  /* create the data thread */
  pthread_create(&data_thread, NULL, fake ? fake_data : mas_data, NULL);

  /* main loop */
  for (;;) {
    struct ScheduleEvent ev;

    /* check inbound packets */
    n = udp_recv(sock, UDP_TIMEOUT, peer, &port, UDP_MAXSIZE, data);

    /* decrement timers */
    if (command_veto > 0) {
      if ((command_veto -= UDP_TIMEOUT) <= 0) {
        bputs(info, "Commanding unvetoed\n");
        command_veto = 0;
      }
    }

    /* n == 0 on timeout */
    type = (n > 0) ? mpc_check_packet(n, data, peer, port) : -1;

    if (init) {
      if (init_timer <= 0) {
        /* send init packet */
        len = mpc_compose_init(nmce, data);

        /* Broadcast this to everyone */
        if (udp_bcast(sock, MCESERV_PORT, len, data) == 0)
          bputs(info, "Broadcast awake ping.\n");

        init_timer = INIT_TIMEOUT;
      } else if (n == 0)
        init_timer -= UDP_TIMEOUT;
    } else {
      /* finished the init; slow data */
      if (slow_timer <= 0) {
        send_slow_data(data);
        slow_timer = SLOW_TIMEOUT;
      } else if (n == 0)
        slow_timer -= UDP_TIMEOUT;

      /* PCM requests */
      pcm_special(0, NULL);
    }

    /* skip bad packets */
    if (type < 0)
      continue;

    /* do something based on packet type */
    switch (type) {
      case 'C': /* command packet */
        if (mpc_decompose_command(&ev, n, data)) {
          /* command decomposition failed */
          break;
        }
        /* run the command here */
        break;
      case 'F': /* field set packet */
        veto = 1; /* veto transmission */
        if ((n = mpc_decompose_bset(&bset_num, tes, nmce, n, data)) >= 0)
          ntes = n;
        veto = 0; /* unveto transmission */
        init = 0;
        break;
      case 'N': /* PCM notice packet */
        pcm_special(n, data);
        break;
      default:
        bprintf(err, "Unintentionally dropping unhandled packet of type 0x%X\n",
            type);
    }
  }

  close(sock);
  return 0;
}
