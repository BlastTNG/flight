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
#include "mce_frame.h"
#include "mpc.h"
#include "mputs.h"
#include "mpc_proto.h"
#include "udp.h"

#include <sys/types.h>
#include <sys/statvfs.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

/* some timing constants; in the main loop, timing is done using the udp poll
 * timeout.  As a result, all timings are approximate (typically lower bounds)
 * and the granularity of timing is the UDP_TIMEOUT
 */
#define UDP_TIMEOUT    10  /* milliseconds */

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
int init = 1;

/* Data return veto */
int veto = 0;

/* PCM/MPC divisor */
int divisor = 2;

/* The slow data struct */
static struct mpc_slow_data slow_dat;

/* the list of bolometers to send to PCM */
uint16_t bset_num = 0xFFFF;
int ntes = 0;
int16_t tes[NUM_ROW * NUM_COL];

/* the data frame to ship out */
uint16_t pcm_data[NUM_COL * NUM_ROW];

/* the turnaround flag */
int in_turnaround = 0;

/* time to ask PCM to power cycle the MCE or MCC */
int power_cycle_mce = 0;
int power_cycle_cmp = 0;

/* ping */
static int pcm_pong = 0;
   
/* handles the divide-by-two frequency scaling for PCM transfer */
int pcm_strobe = 0;

/* Semaphore for data packet ready for tranmission to PCM */
int pcm_ret_dat = 0;

/* Frame number of PCM data */
uint32_t pcm_frameno = 0;

static void set_data_mode_bits(int i, int both, const char *dmb)
{
  if (data_modes[i][0].first_bit != dmb[0] ||
      data_modes[i][0].num_bits != dmb[1] ||
      (both && (data_modes[i][1].first_bit != dmb[2] ||
                data_modes[i][1].num_bits != dmb[3])))
  {
    /* sanity checks */
    if (dmb[0] + dmb[1] > 32)
      goto BAD_DMB;

    if (both)
      if (dmb[1] + dmb[3] != 16 || dmb[2] + dmb[3] > 32 ||
          dmb[2] + dmb[3] > dmb[0])
        goto BAD_DMB;

    /* update */
    if (both)
      bprintf(info, "Set extraction bits for data_mode %i to: %i:%i/%i:%i",
          i, dmb[0], dmb[1], dmb[2], dmb[3]);
    else
      bprintf(info, "Set extraction bits for data_mode %i to: %i:%i/-:-",
          i, dmb[0], dmb[1]);

    data_modes[i][0].first_bit = dmb[0];
    data_modes[i][0].num_bits = dmb[1];
    if (both) {
      data_modes[i][1].first_bit = dmb[2];
      data_modes[i][1].num_bits = dmb[3];
    }

    return;

BAD_DMB:
    if (both)
      bprintf(info, "Ignoring bad bits for data_mode %i: %i:%i/%i:%i",
          i, dmb[0], dmb[1], dmb[2], dmb[3]);
    else
      bprintf(info, "Ignoring bad bits for data_mode %i: %i:%i/-:-",
          i, dmb[0], dmb[1]);

  }
}

/* ask PCM to do something, maybe;
 * also deal with PCM having done something, maybe
 */
static void pcm_special(size_t len, const char *data_in, const char *peer,
    int port)
{
  if (data_in) {
    /* reply acknowledgement from PCM */
    const char *data_mode_bits;

    if (mpc_decompose_notice(nmce, &data_mode_bits, &in_turnaround,
          &divisor, len, data_in, peer, port))
      return;

    /* update the data_modes definition */
    set_data_mode_bits(0, 0, data_mode_bits + 0);
    set_data_mode_bits(1, 0, data_mode_bits + 2);
    set_data_mode_bits(2, 0, data_mode_bits + 4);
    set_data_mode_bits(4, 1, data_mode_bits + 6);
    set_data_mode_bits(5, 1, data_mode_bits + 10);
    set_data_mode_bits(10, 1, data_mode_bits + 14);
    set_data_mode_bits(12, 0, data_mode_bits + 18);
  } else {
    /* send a request, if necessary */
    char data[UDP_MAXSIZE];
    int power_cycle = 0;
    int pong = 0;
    int need_send = 0;

    /* XXX */
    if (power_cycle_mce) {
      bprintf(warning, "WANTED TO POWER CYCLE MCE!");
      power_cycle_mce = 0;
    }

    /* collect pending requests */
    if (power_cycle_mce) {
      power_cycle = 1;
      need_send = 1;
      power_cycle_mce = 0;
    }
    
    /* force send */
    if (pcm_pong) {
      pong = need_send = 1;
      pcm_pong = 0;
    }

    if (need_send) {
      len = mpc_compose_pcmreq(nmce, power_cycle, data);
      udp_bcast(sock, MCESERV_PORT, len, data, 0);
      bprintf(info, "PCM request: :%s%s",
          power_cycle ? "POW:" : "",
          pong ? "PNG:" : ""
          );
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
  if (statvfs("/data0/mce", &buf) == 0)
    slow_dat.df0 =
      (uint16_t)(((unsigned long long)buf.f_bfree * buf.f_bsize) >> 24);

  /* time -- this wraps around ~16 months after the epoch */
  gettimeofday(&tv, NULL);
  slow_dat.time = (tv.tv_sec - MPC_EPOCH) * 100 + tv.tv_usec / 10000;

  /* meta stuff */
  slow_dat.goal = goal;
  slow_dat.task = start_tk - stop_tk;
  slow_dat.dtask = data_tk;
  slow_dat.state = state;

  /* make packet and send */
  len = mpc_compose_slow(&slow_dat, nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data, 0);
}

static int nmce_from_ip(void)
{
  struct ifreq ifr;
  int inet_sock;
  unsigned long addr;

  /* connect to kINET */
  inet_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (inet_sock < 0)
    bprintf(fatal, "Unable to connect to kernel INET subsystem!");

  /* get eth0's IP address */
  strcpy(ifr.ifr_name, "eth0");
  if (ioctl(inet_sock, SIOCGIFADDR, &ifr))
    bprintf(fatal, "SIOCGIFADDR failed!");
  close(inet_sock);
  
  /* Flight computers have addresses of the form 192.168.1.8x for x in [1,6].
   * IPv4 addresses are encoded as a big endian number in s_addr, so we check
   * whether the lower three bytes are 0x01a8c0 (= 1.168.192) and then use the
   * top byte minus 80 as the mce number, if in range. */
  addr =
    (unsigned long)((struct sockaddr_in*)&ifr.ifr_netmask)->sin_addr.s_addr;
  if (((addr & 0xFFFFF) != 0x01A8C0) || ((addr >> 24) < 81) ||
      ((addr >> 24) > 86))
  {
    return -1;
  }

  return (addr >> 24) - 80;
}

/* Figure out which MCE we're attached to, also whether we're running in fake
 * mode */
static void get_nmce(int argc, const char **argv)
{
  int i;
  char array_id[1024] = {'x', 0, 0};

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

  /* figure out which MCE we're attached to via the IP address */
  if (!fake) {
    int n;
    /* get the MCE number from the IP address. */
    nmce = nmce_from_ip();

    const char *file = MAS_DATA_ROOT "/array_id";
    FILE *stream = fopen(file, "w");

    if (stream == NULL) {
      berror(err, "Unable to open array ID file: %s", file);
      goto BAD_ARRAY_ID;
    }

    if (nmce == -1) {
      bputs(warning, "Using 'default' configuration and running as MCE0");
      nmce = 0;
      strcpy(array_id, "default");
    } else 
      array_id[1] = '0' + nmce;

    n = fprintf(stream, "%s\n", array_id);

    if (n < 0) {
      berror(err, "Can't write array id");
BAD_ARRAY_ID:
      if (nmce == -1)
        nmce = 0;
      bprintf(err, "Reverting to FAKE MCE");
      fake = 1;
    }
    fclose(stream);
  }

  if (fake) {
    bprintf(info, "Running FAKE MCE #%i", nmce);
  } else {
    bprintf(info, "Controlling MCE #%i.  Array ID: %s", nmce, array_id);
  }
}

/* Send a TES data packet */
static void ForwardData(void)
{
  char data[UDP_MAXSIZE];
  size_t len = mpc_compose_tes(pcm_data, pcm_frameno, bset_num, nmce, ntes, tes,
      data);
  if (pcm_ret_dat) { /* race condition avoidance */
    udp_bcast(sock, MCESERV_PORT, len, data, 0);
    pcm_ret_dat = 0;
  }
}

/* Execute a command */
static void do_ev(const struct ScheduleEvent *ev, const char *peer, int port)
{
  if (ev->is_multi) {
    switch (ev->command) {
      default:
        bprintf(warning, "Unrecognised multi command #%i from %s/%i\n",
            ev->command, peer, port);
        break;
    }
  } else {
    switch (ev->command) {
      case mpc_ping:
        pcm_pong = 1;
        break;
      default:
        bprintf(warning, "Unrecognised single command #%i from %s/%i\n",
            ev->command, peer, port);
        break;
    }
  }
}

int main(int argc, const char **argv)
{
  int port, type;
  ssize_t n;
  size_t len;

  pthread_t data_thread;
  pthread_t task_thread;

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

  /* create the threads */
  pthread_create(&data_thread, NULL, fake ? fake_data : mas_data, NULL);
  pthread_create(&task_thread, NULL, task, NULL);

  /* main loop */
  for (;;) {
    struct ScheduleEvent ev;

    /* iterate the director */
    meta();

    /* check inbound packets */
    n = udp_recv(sock, UDP_TIMEOUT, peer, &port, UDP_MAXSIZE, data);

    /* n == 0 on timeout */
    type = (n > 0) ? mpc_check_packet(n, data, peer, port) : -1;

    /* skip bad packets */
    if (type > 0) {
      /* do something based on packet type */
      switch (type) {
        case 'C': /* command packet */
          if (mpc_decompose_command(&ev, n, data)) {
            /* command decomposition failed */
            break;
          }
          /* execute */
          do_ev(&ev, peer, port);
          break;
        case 'F': /* field set packet */
          veto = 1; /* veto transmission */
          if ((n = mpc_decompose_bset(&bset_num, tes, nmce, n, data)) >= 0)
            ntes = n;
          veto = 0; /* unveto transmission */
          init = 0;
          break;
        case 'N': /* PCM notice packet */
          pcm_special(n, data, peer, port);
          break;
        default:
          bprintf(err,
              "Unintentionally dropping unhandled packet of type 0x%X\n", type);
      }
    }

    /* Now do outbound packets */
    if (init) {
      if (init_timer <= 0) {
        /* send init packet */
        len = mpc_compose_init(nmce, data);

        /* Broadcast this to everyone */
        if (udp_bcast(sock, MCESERV_PORT, len, data, 0) == 0)
          bputs(info, "Broadcast awake ping.\n");

        init_timer = INIT_TIMEOUT;
      } else if (n == 0)
        init_timer -= UDP_TIMEOUT;
    } else if (!power_cycle_cmp) {
      /* finished the init; slow data */
      if (slow_timer <= 0) {
        send_slow_data(data);
        slow_timer = SLOW_TIMEOUT;
      } else if (n == 0)
        slow_timer -= UDP_TIMEOUT;

      if (pcm_ret_dat)
        ForwardData();

      /* PCM requests */
      pcm_special(0, NULL, NULL, 0);
    }
  }

  close(sock);
  return 0;
}
