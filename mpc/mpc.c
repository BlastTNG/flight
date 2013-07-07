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
int sock;

/* The number of the attached MCE */
int nmce = 0;

/* The current MCE data mode */
int cur_dm = -1;

/* The requested data mode */
int req_dm = 10;

/* Initialisation veto */
int init = 1;

/* Data return veto */
int veto = 1;

/* PCM/MPC divisor */
int divisor = 2;

/* The slow data struct */
static struct mpc_slow_data slow_dat;

/* the list of bolometers to send to PCM */
uint16_t bset_num = 0xFFFF;
int ntes = 0;
int16_t tes[NUM_ROW * NUM_COL];

/* DV is comming from the sync box */
int sync_dv = 0;

/* the data frame to ship out */
static uint16_t pcm_data[NUM_COL * NUM_ROW * PB_SIZE];

/* the turnaround flag */
int in_turnaround = 0;

/* time to ask PCM to power cycle the MCE or MCC */
int power_cycle_mce = 0;
int power_cycle_cmp = 0;

/* ping */
static int pcm_pong = 0;
   
/* handles the divide-by-two frequency scaling for PCM transfer */
int pcm_strobe = 0;

/* ret_dat counter */
int rd_count = 0;

/* Send mcestat to PCM */
int send_mcestat = 0;

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
    int ssreq;

    if (mpc_decompose_notice(nmce, &data_mode_bits, &in_turnaround,
          &divisor, &ssreq, len, data_in, peer, port))
      return;
    
    if (ssreq)
      send_mcestat = 1;

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

static int disk_bad[4] = { 0, 0, 0, 0 };

#define BAD_DISK_TIMEOUT 30 /* in units of SLOW_TIMEOUT =~ 4 minutes */
/* get disk free, also determine whether a disk is healthy or not */
static int check_disk(int n)
{
  static int check_timeout[4] = { 0, 0, 0, 0 };

  struct statvfs buf;
  char path[] = "/data0/mce";
  path[5] = '0' + n;

  if (check_timeout[n]) {
    check_timeout[n]--;
    return disk_bad[n];
  }

  if (statvfs(path, &buf) == 0) {
    /* disk free -- units are 2**24 bytes = 16 MB */
    slow_dat.df[n] = (uint16_t)(((unsigned long long)buf.f_bfree * buf.f_bsize)
        >> 24);
    if (disk_bad[n]) {
      bprintf(info, "/data%i now reporting", n);
      disk_bad[n] = 0;
    }
    check_timeout[n] = 0;
  } else {
    slow_dat.df[n] = -1;
    if (!disk_bad[n]) {
      bprintf(warning, "/data%i has failed", n);
      disk_bad[n] = 1;
    }
    check_timeout[n] = BAD_DISK_TIMEOUT;
  }

  return disk_bad[n];
}

/* send slow data to PCM */
static void send_slow_data(char *data)
{
  struct timeval tv;
  size_t len;
  FILE *stream;
  int datum;

  slow_dat.data_mode = cur_dm;

  check_disk(0);
  check_disk(1);
  check_disk(2);
  check_disk(3);

  /* time -- this wraps around ~16 months after the epoch */
  gettimeofday(&tv, NULL);
  slow_dat.time = (tv.tv_sec - MPC_EPOCH) * 100 + tv.tv_usec / 10000;

  /* temperature */
  if ((stream = fopen("/sys/devices/platform/coretemp.0/temp2_input", "r"))) {
    if (fscanf(stream, "%i\n", &datum) == 1)
      slow_dat.t_mcc = datum / 10;
    fclose(stream);
  } else
    slow_dat.t_mcc = 0xFFFF;

  /* state stuff */
  slow_dat.state = state;

  slow_dat.goal = goal;
  slow_dat.task = (stop_tk == st_idle) ? start_tk : (0x8000 | stop_tk);
  slow_dat.dtask = data_tk;

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

/* Figure out which MCE we're attached to */
static void get_nmce(void)
{
  int n;
  char array_id[1024] = {'x', 0, 0};

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
    berror(err, "Can't write array id!");
BAD_ARRAY_ID:
    if (nmce == -1)
      nmce = 0;
    bprintf(fatal, "Unable to find MCE.");
  }
  fclose(stream);

  bprintf(info, "Controlling MCE #%i.  Array ID: %s", nmce, array_id);
}

/* Send a TES data packet */
static void ForwardData(const uint32_t *frameno)
{
  char data[UDP_MAXSIZE];
  size_t len = mpc_compose_tes(pcm_data, frameno, bset_num, PB_SIZE, sync_dv,
      nmce, ntes, tes, data);
  udp_bcast(sock, MCESERV_PORT, len, data, 0);
}

/* do the frequency division and 16-bit conversion based on the data mode
 * definitions */
static int16_t coadd(uint32_t datum, uint16_t old_datum)
{
  uint16_t rec[2];
  uint32_t mask[2];
  int i;

  /* 16-bit-space masks */
  mask[0] = ((1 << data_modes[cur_dm][0].num_bits) - 1)
    << data_modes[cur_dm][1].num_bits;
  mask[1] = ((1 << data_modes[cur_dm][1].num_bits) - 1);

  /* extract the subfield(s) */
  rec[0] = (uint16_t)(datum >> (data_modes[cur_dm][0].first_bit -
          data_modes[cur_dm][1].num_bits)) & mask[0];
  rec[1] = (uint16_t)(datum >> data_modes[cur_dm][1].first_bit) & mask[1];

  if (pcm_strobe) { /* coadd */
    /* split the old datum, if neccessary */
    uint16_t old_rec[2];
    
    old_rec[0] = (data_modes[cur_dm][0].coadd_how != first)
      ? old_datum & mask[0] : 0;

    old_rec[1] = (data_modes[cur_dm][1].num_bits > 0 &&
        data_modes[cur_dm][1].coadd_how != first) ?  old_datum & mask[1] : 0;

    /* coadd */
    for (i = 0; i < 2; ++i)
      switch (data_modes[cur_dm][i].coadd_how) {
        case first:
          break; /* nothing to do */
        case sum:
          rec[i] = ((uint32_t)rec[i] + old_rec[i]) & mask[i];
          break;
        case mean:
          rec[i] = (((uint32_t)rec[i] + old_rec[i]) / 2) & mask[i];
          break;
      }

  }

  /* mush back together and return */
  return rec[0] | rec[1];
}

static void pushback(void)
{
  int n;
  size_t i, ndata = frame_size / sizeof(uint32_t) - MCE_HEADER_SIZE;

  /* the buffer containing the first frame */
  static uint16_t data[NUM_ROW * NUM_COL];
  uint32_t frameno[PB_SIZE];

  if (ntes > 0 || 1) { /* not sending any TES data */
    for (n = 0; n < PB_SIZE; ++n) {
      /* this frame */
      uint32_t *fr = frame[(n + pb_last) % FB_SIZE];

      const struct mas_header *header = (const struct mas_header *)fr;
      frameno[n] = sync_dv ? header->syncno : header->cc_frameno;

      if (ndata > NUM_COL * NUM_ROW)
        ndata = NUM_COL * NUM_ROW;

      if (divisor == 1) {
        pcm_strobe = 0;
        for (i = 0; i < ndata; ++i) 
          pcm_data[i] = coadd(fr[i + MCE_HEADER_SIZE], 0);
      } else {
        if (pcm_strobe) { /* coadd */
          for (i = 0; i < ndata; ++i) 
            pcm_data[i] = coadd(fr[i + MCE_HEADER_SIZE], data[i]);
          pcm_strobe = 0;
        } else {/* just copy */
          for (i = 0; i < ndata; ++i) 
            data[i] = coadd(fr[i + MCE_HEADER_SIZE], 0);
          pcm_strobe = 1;
          return;
        }
      }
    }

    /* pushback */
    ForwardData(frameno);
  }
  pb_last = (pb_last + PB_SIZE) % FB_SIZE;
}

/* Send a super-slow data packet */
static void ForwardMCEStat(void)
{
  /* send it! */
  char data[UDP_MAXSIZE];
  size_t len = mpc_compose_stat(mce_stat, nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data, 0);
  send_mcestat = 0;
}

const static inline int check_cmd_mce(int w)
{
  return (w == 0) || (w == (nmce + 1));
}

/* Execute a command */
static void do_ev(const struct ScheduleEvent *ev, const char *peer, int port)
{
  if (ev->is_multi) {
    switch (ev->command) {
      case data_mode:
        if (check_cmd_mce(ev->ivalues[0]))
          req_dm = ev->ivalues[1];
      case reset_acq:
        if (check_cmd_mce(ev->ivalues[0]))
          comms_lost = 1;
        break;
      case start_acq:
        if (check_cmd_mce(ev->ivalues[0]))
          goal = op_acq;
        break;
      case stop_acq:
        if (check_cmd_mce(ev->ivalues[0]))
          goal = op_ready;
        break;
      case tune_array:
        if (check_cmd_mce(ev->ivalues[0]))
          goal = op_tune;
        break;
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

int main(void)
{
  int port, type;
  ssize_t n;
  size_t len;

  pthread_t data_thread;
  pthread_t task_thread;
  pthread_t acq_thread;

  int init_timer = 0;
  int slow_timer = 0;

  char peer[UDP_MAXHOST];
  char data[UDP_MAXSIZE];

  buos_use_func(mputs);
  nameThread("Main");

  /* for scripts */
  setenv("PYTHONPATH", "/data/mas/mce_script/python:/data/mas/python", 1);
  setenv("PATH", "usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:"
      "/data/mas/bin:/data/mas/mce_script/script:"
      "/data/mas/mce_script/test_suite", 1);

  if ((logfile = fopen(MPC_ETC_DIR "/mpc.log", "a")) == NULL)
    berror(err, "Can't open log file");
  else
    fputs("!!!!!! LOG RESTART !!!!!!\n", logfile);

  bputs(startup, "This is MPC.\n");
  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC subsystem.");

  /* bind to the UDP port */
  sock = udp_bind_port(MPC_PORT, 1);
  if (sock < 0)
    bprintf(fatal, "Unable to bind port.");

  /* Figure out our MCE number and check for fake mode */
  get_nmce();

  /* create the threads */
  pthread_create(&data_thread, NULL, mas_data, NULL);
  pthread_create(&task_thread, NULL, task, NULL);
  pthread_create(&acq_thread, NULL, acquer, NULL);

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

      if (send_mcestat)
        ForwardMCEStat();

      /* send data */
      if ((fb_top - pb_last + FB_SIZE) % FB_SIZE > PB_SIZE)
        pushback();

      /* PCM requests */
      pcm_special(0, NULL, NULL, 0);
    }
  }

  close(sock);
  return 0;
}
