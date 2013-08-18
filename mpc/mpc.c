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
#include "command_list.h"
#include "mce_frame.h"
#include "mputs.h"
#include "mpc_proto.h"
#include "udp.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/socket.h>
#include <signal.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <dirent.h>
#include <glob.h>
#include <math.h>

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

/* sets the rate at which array synopses are sent to PCM.  PCM sends these down
 * at a rate of 16-bits per frame, seriallising MCEs.  There are ~3000 channels,
 * 3 statistics at 8-bits = ~4500 frames of data.  So we can send this down
 * rarely.
 */
#define SYNOP_TIMEOUT 10000 /* milliseconds */

/* non-volatile memory */
int mem_dirty;
struct memory_t memory;

/* UDP socket */
int sock;

/* The number of the attached MCE */
int nmce = 0;

/* The current MCE data mode */
int cur_dm = -1;

/* The requested data mode */
int req_dm = 10;

/* Drive check semaphore */
int drives_checked;

/* Kill switch */
int terminate = 0;

/* Initialisation veto */
static int init = 1;

/* boot time */
time_t btime;

/* uuid map */
char *uuid[4];

/* reset the array statistics */
int stat_reset = 1;

/* Data return veto */
int veto = 1;

/* MCE temperature poll */
int mas_get_temp = 0;

/* slow veto -- if vetoed, eventually PCM will reboot the MCC; ergo, we use this
 * as a watchdog */
int slow_veto = 0;

time_t start_time;

/* The slow data struct */
struct mpc_slow_data slow_dat;

struct block_q blockq[BLOCKQ_SIZE];
int blockq_head = 0, blockq_tail = 0;

/* drive map */
uint8_t drive_map = DRIVE0_UNMAP | DRIVE1_UNMAP | DRIVE2_UNMAP;
int data_drive[3] = {-1, -1, -1};
int drive_error[4]; /* high level error on drive */
int disk_bad[4]; /* disk marked bad */


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

/* array_id */
char array_id[100] = {'x', 0, 0};

/* time to ask PCM to power cycle the MCE or MCC */
int power_cycle_mce = 0;
int power_cycle_cmp = 0;

/* ping */
static int pcm_pong = 0;

/* ret_dat counter */
int rd_count = 0;

/* Send mce parameters to PCM */
int send_mceparam = 0;

/* Send blobs */
int send_blob = 0;
uint16_t blob[MCE_BLOB_MAX];

/* box temperature */
int32_t box_temp = 32767;

/* DV timing parameters */
int num_rows = -1, row_len = -1, data_rate = -1;

static void set_data_mode_bits(int i, const char *dmb)
{
  if (data_modes[i][0].first_bit != dmb[0] ||
      data_modes[i][0].num_bits != dmb[1] ||
      (data_modes[i][1].first_bit != dmb[2] ||
       data_modes[i][1].num_bits != dmb[3]))
  {
    /* sanity checks */
    if (dmb[0] + dmb[1] > 32)
      goto BAD_DMB;

    if (dmb[1] + dmb[3] != 16 || dmb[2] + dmb[3] > 32 ||
        dmb[2] + dmb[3] > dmb[0])
      goto BAD_DMB;

    /* update */
    bprintf(info, "Set extraction bits for data_mode %i to: %i:%i/%i:%i",
        i, dmb[0], dmb[1], dmb[2], dmb[3]);

    data_modes[i][0].first_bit = dmb[0];
    data_modes[i][0].num_bits = dmb[1];
    data_modes[i][1].first_bit = dmb[2];
    data_modes[i][1].num_bits = dmb[3];

    return;

BAD_DMB:
    bprintf(info, "Ignoring bad bits for data_mode %i: %i:%i/%i:%i",
        i, dmb[0], dmb[1], dmb[2], dmb[3]);
  }
}

/* ask PCM to do something, maybe;
 * also deal with PCM having done something, maybe
 */
static void pcm_special(size_t len, const char *data_in, const char *peer,
    int port)
{
  int new_row_len = -1, new_num_rows = -1, new_data_rate = -1, squidveto = 0;
  int divisor;
  uint16_t new_bolo_filt_len;
  double new_bolo_filt_freq, new_bolo_filt_bw;

  if (data_in) {
    /* reply acknowledgement from PCM */
    const char *data_mode_bits;
    int ssreq;

    if (mpc_decompose_notice(nmce, &data_mode_bits, &in_turnaround,
          &divisor, &ssreq, &req_dm, &new_row_len, &new_num_rows,
          &new_data_rate, &squidveto, &new_bolo_filt_freq, &new_bolo_filt_bw,
          &new_bolo_filt_len, len, data_in, peer, port))
      return;

    if (ssreq)
      send_mceparam = 1;

    squidveto = (squidveto & (1U << nmce)) ? 1 : 0;
    if (squidveto != memory.squidveto) {
      if (squidveto)
        bprintf(info, "Squids vetoed");
      else
        bprintf(info, "Squids unvetoed");
      memory.squidveto = squidveto;
      mem_dirty = 1;
    }

    if (new_bolo_filt_len != memory.bolo_filt_len || 
        new_bolo_filt_freq != memory.bolo_filt_freq ||
        new_bolo_filt_bw != memory.bolo_filt_bw) {
      memory.bolo_filt_len = new_bolo_filt_len;
      memory.bolo_filt_freq = new_bolo_filt_freq;
      memory.bolo_filt_bw = new_bolo_filt_bw;
      mem_dirty = 1;
      stat_reset = 1;
    }

    if (divisor != 1)
      divisor = 2;

    if (divisor != memory.divisor) {
      bprintf(info, "New divisor: %i", divisor);
      memory.divisor = divisor;
      mem_dirty = 1;
    }

    if (new_row_len > 1 && new_data_rate > 1 && new_num_rows > 1) {
      row_len = new_row_len;
      num_rows = new_num_rows;
      data_rate = new_data_rate;
      cfg_update_timing(new_row_len, new_num_rows, new_data_rate);
    }

    /* update the data_modes definition */
    set_data_mode_bits(req_dm, data_mode_bits);
  } else {
    /* send a request, if necessary */
    char data[UDP_MAXSIZE];
    int power_cycle = 0;
    int pong = 0;
    int need_send = 0;

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

#define BAD_DISK_TIMEOUT 30 /* in units of SLOW_TIMEOUT =~ 4 minutes */
/* get disk free, also determine whether a disk is healthy or not */
static int check_disk(int n)
{
  static int check_timeout[4] = { 0, 0, 0, 0 };

  struct statvfs buf;
  char path[] = "/data0/mce";
  int drive_fail = 0;
  path[5] = '0' + n;

  if (disk_bad[n] && check_timeout[n]) {
    check_timeout[n]--;
    return 1;
  }

  if (statvfs(path, &buf) == 0) {
    /* disk free -- units are 2**24 bytes = 16 MB */
    slow_dat.df[n] = (uint16_t)(((unsigned long long)buf.f_bfree * buf.f_bsize)
        >> 24);
    if (disk_bad[n] && !drive_error[n]) {
      bprintf(info, "/data%i now available", n);
      disk_bad[n] = 0;
    }
    check_timeout[n] = 0;
  } else
    drive_fail = 1;

  if (drive_fail || drive_error[n]) {
    slow_dat.df[n] = 0;
    if (!disk_bad[n]) {
      bprintf(warning, "/data%i has failed", n);
      disk_bad[n] = 1;
    }
    check_timeout[n] = BAD_DISK_TIMEOUT;
  }

  return disk_bad[n];
}

/* send array synopses; nothing spectacular here... */
static void send_array_synopsis(void)
{
  char data[UDP_MAXSIZE];
  size_t len = mpc_compose_synop(&bolo_stat_buff[0][0], nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data, 0);
}

/* send slow data to PCM */
static void send_slow_data(char *data, int send)
{
  struct timeval tv;
  size_t len;
  FILE *stream;
  int datum;

  /* get temperature data from MCE */
  mas_get_temp = 1;

  check_disk(0);
  check_disk(1);
  check_disk(2);
  check_disk(3);
  drives_checked = 1;

  /* time -- this wraps around ~16 months after the epoch */
  gettimeofday(&tv, NULL);
  slow_dat.time = (tv.tv_sec - MPC_EPOCH) * 100 + tv.tv_usec / 10000;

  /* uptime -- this wraps around every ~30 days */
  if ((tv.tv_sec - start_time) > 65535 * 40)
    slow_dat.uptime = 65535;
  else
    slow_dat.uptime = (tv.tv_sec - start_time) / 40;

  /* mcc temperature */
  if ((stream = fopen("/sys/devices/platform/coretemp.0/temp2_input", "r"))) {
    if (fscanf(stream, "%i\n", &datum) == 1)
      slow_dat.t_mcc = datum / 10;
    fclose(stream);
  } else
    slow_dat.t_mcc = 0x8000;

  /* sync veto */
  slow_dat.sync_veto = memory.sync_veto;

  /* mce temp */
  slow_dat.t_mce = box_temp;

  /* state stuff */
  slow_dat.state = state | (moda << MODA_SHIFT);

  slow_dat.goal = goal.goal;
  slow_dat.task = meta_tk;
  slow_dat.dtask = data_tk;

  /* drive map */
  slow_dat.drive_map = drive_map;

  /* tuning and iv curves */
  slow_dat.last_tune = memory.last_tune;
  slow_dat.last_iv = memory.last_iv;
  slow_dat.used_tune = memory.used_tune;
  slow_dat.ref_tune = memory.ref_tune;
  slow_dat.tune_stat = tuning_status;

  /* make packet and send */
  if (send) {
    len = mpc_compose_slow(&slow_dat, nmce, data);
    udp_bcast(sock, MCESERV_PORT, len, data, 0);
  }
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

  /* Flight computers have addresses of the form 192.168.1.24x for x in [1,6].
   * IPv4 addresses are encoded as a big endian number in s_addr, so we check
   * whether the lower three bytes are 0x01a8c0 (= 1.168.192) and then use the
   * top byte minus 240 as the mce number, if in range. */
  addr =
    (unsigned long)((struct sockaddr_in*)&ifr.ifr_netmask)->sin_addr.s_addr;
  if (((addr & 0xFFFFF) != 0x01A8C0) || ((addr >> 24) < 241) ||
      ((addr >> 24) > 246))
  {
    return -1;
  }

  return (addr >> 24) - 241;
}

/* do the frequency division and 16-bit conversion based on the data mode
 * definitions */
static int16_t coadd(uint32_t datum1, uint16_t datum2)
{
  uint16_t rec[2];
  uint32_t mask[2];
  int i;

  /* 16-bit-space masks */
  mask[0] = ((1 << data_modes[cur_dm][0].num_bits) - 1)
    << data_modes[cur_dm][1].num_bits;
  mask[1] = ((1 << data_modes[cur_dm][1].num_bits) - 1);

  /* extract the subfield(s) */
  rec[0] = (uint16_t)(datum1 >> (data_modes[cur_dm][0].first_bit -
        data_modes[cur_dm][1].num_bits)) & mask[0];
  rec[1] = (uint16_t)(datum1 >> data_modes[cur_dm][1].first_bit) & mask[1];

  /* if divisor is two, we have to add two frames together */
  if (memory.divisor != 1) {
    /* split dataum2, if neccessary */
    uint16_t old_rec[2];

    old_rec[0] = (data_modes[cur_dm][0].coadd_how != coadd_first)
      ? datum2 & mask[0] : 0;

    old_rec[1] = (data_modes[cur_dm][1].num_bits > 0 &&
        data_modes[cur_dm][1].coadd_how != coadd_first) ? datum2 & mask[1] : 0;

    /* coadd */
    for (i = 0; i < 2; ++i)
      switch (data_modes[cur_dm][i].coadd_how) {
        case coadd_first:
          break; /* nothing to do */
        case coadd_sum:
          rec[i] = ((uint32_t)rec[i] + old_rec[i]) & mask[i];
          break;
        case coadd_mean:
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
  size_t i, ndata = frame_size / sizeof(uint32_t) - MCE_HEADER_SIZE - 1;
  uint32_t frameno[PB_SIZE];
  char data[UDP_MAXSIZE];
  size_t len;

  if (ntes > 0) { /* not sending any TES data */
    for (n = 0; n < PB_SIZE * memory.divisor; n += memory.divisor) {
      /* this frame */
      uint32_t *frA = frame[(n + pb_last) % FB_SIZE];

      const struct mas_header *header = (const struct mas_header *)frA;
      frameno[n / memory.divisor]
        = sync_dv ? header->syncno : header->cc_frameno;

      if (ndata > NUM_COL * NUM_ROW)
        ndata = NUM_COL * NUM_ROW;

      if (memory.divisor == 1)
        for (i = 0; i < ndata; ++i) 
          pcm_data[i + n * NUM_ROW * NUM_COL]
            = coadd(frA[i + MCE_HEADER_SIZE], 0);
      else {
        /* previous frame */
        uint32_t *frB = frame[(n + pb_last + FB_SIZE - 1) % FB_SIZE];
        for (i = 0; i < ndata; ++i) 
          pcm_data[i + n * NUM_ROW * NUM_COL / 2]
            = coadd(frA[i + MCE_HEADER_SIZE], frB[i + MCE_HEADER_SIZE]);
      }
    }

    /* pushback */
    len = mpc_compose_tes(pcm_data, frameno, bset_num, PB_SIZE, sync_dv,
        nmce, ntes, tes, data);
    udp_bcast(sock, MCESERV_PORT, len, data, 0);
  }
  pb_last = (pb_last + PB_SIZE * memory.divisor) % FB_SIZE;
}

/* Send GP data to PCM */
static void ForwardBlob(void)
{
  char data[UDP_MAXSIZE];
  size_t len = mpc_compose_gpdata(blob_type, blob_size, blob, nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data, 0);
  send_blob = 0;
}

/* Send a super-slow data packet */
static void ForwardMCEParam(void)
{
  char data[UDP_MAXSIZE];

  /* update expt config stuff */
  serialise_experiment_cfg();

  size_t len = mpc_compose_param(mce_stat, nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data, 0);
  send_mceparam = 0;
}

/* push something onto the mas block queue */
static void push_blockr(const char *c, const char *p, int o, const uint32_t *d,
    int n, int raw)
{
  int new_head;

  /* if it's full, just discard, I guess */
  if (((blockq_head - blockq_tail + BLOCKQ_SIZE) % BLOCKQ_SIZE) ==
      BLOCKQ_SIZE - 1)
  {
    return;
  }

  /* if we're vetoed, just discard it */
  if (~state & st_active)
    return;

  new_head = (blockq_head + 1) % BLOCKQ_SIZE;

  blockq[new_head].c = strdup(c);
  blockq[new_head].p = strdup(p);
  if (n)
    memcpy(blockq[new_head].d, d, sizeof(uint32_t) * n);
  blockq[new_head].n = n;
  blockq[new_head].o = o;
  blockq[new_head].raw = raw;

#if 0
  {
    int i;
    char *ptr, params[1000];
    ptr = params;
    for (i = 0; i < blockq[new_head].n; ++i)
      ptr += sprintf(ptr, "%u ", blockq[new_head].d[i]);
    bprintf(info, "pushblock%s: %s/%s+%i(%i) [ %s]",
        raw ? " (r)" : "", blockq[new_head].c,
        blockq[new_head].p, blockq[new_head].o, blockq[new_head].n, params);
  }
#endif

  blockq_head = new_head;
}

static void push_kick(double heater_bias, double time, int kick_bias)
{
  uint32_t data[3];
  memcpy(data, &time, sizeof(double));
  data[2] = kick_bias;
  push_blockr("", "", heater_bias * 32767. / 5, data, 3, 2);
}

/* cooked push */
static void push_block(const char *c, const char *p, int o, const uint32_t *d,
    int n)
{
  push_blockr(c, p, o, d, n, 0);
}

static void push_block_raw(const char *c, const char *p, int o,
    const uint32_t *d, int n)
{
  push_blockr(c, p, o, d, n, 1);
}

static void apply_cmd(int p, const char *name, int n, uint32_t v)
{
  const char *card[] = {NULL, NULL, NULL, NULL};
  const char *param = name;
  char buffer[30];
  int i, c;

  switch (p) {
    case sample_num:
    case fb_dly:
      card[0] = "rc1";
      card[1] = "rc2";
      break;
    case row_dly:
      card[0] = "ac";
      break;
    case flux_jumping_on:
    case flux_jumping_off:
      card[0] = "rc1";
      card[1] = "rc2";
      param = "en_fb_jump";
      break;
    case bias_tes_col:
      card[0] = "tes";
      param = "bias";
      break;
    case sq1_bias:
      card[0] = "ac";
      param = "on_bias";
      break;
    case sq1_off_bias:
      card[0] = "ac";
      param = "off_bias";
      break;
    case sq2_bias:
      card[0] = "sq2";
      param = "bias";
      break;
    case sq2_fb:
      card[0] = "sq2";
      c = n % 41;
      n /= 41;
      sprintf(buffer, "fb_col%i", c);
      param = buffer;
      break;
    case sa_bias:
      if (n > 8) {
        card[0] = "rc2";
        n -= 8;
      } else 
        card[0] = "rc1";
      break;
    case sa_fb:
      card[0] = "sa";
      param = "fb";
      break;
    case sa_offset:
      if (n > 8) {
        card[0] = "rc2";
        n -= 8;
      } else
        card[0] = "rc1";
      param = "offset";
      break;
    case adc_offset:
      c = n % 41;
      n /= 41;
      if (c > 8) {
        card[0] = "rc2";
        c -= 8;
      } else
        card[0] = "rc1";

      sprintf(buffer, "adc_offset%i", c);
      param = buffer;
      break;
    default:
      bprintf(err, "Unhandled parameter %s (%i) in apply_cmd", name, p);
      return;
  }

  for (i = 0; card[i]; ++i)
    push_block(card[i], param, n, &v, 1);
}

static void prm_set_int(int p, const char *name, int n, int v, int a)
{
  /* apply */
  if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY)
    apply_cmd(p, name, n, v);

  /* record default */
  if (a == PRM_DEFAULT_ONLY) {
    char dflt[100] = "default_";
    strcpy(dflt + sizeof("default_") - 1, name);
    cfg_set_int(dflt, n, v);
  } else if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY
      || a == PRM_RECORD_RCONF)
  {
    cfg_set_int(name, n, v);
    if (a == PRM_RECORD_RCONF)
      state &= ~st_config;
  }
}

static void prm_set_int_cr(int p, const char *name, int c, int r, int v, int a)
{
  prm_set_int(p, name, c * 41 + r, v, a);
}

static void prm_set_servo(int c, int r, char l, uint32_t v, int a)
{
  int i;

  /* apply */
  if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
    int col = c;
    char param[] = "gain?#";
    char rc[] = "rc1";
    if (col >= 8) {
      col -= 8;
      rc[2] = '2';
    }
    param[4] = l;
    param[5] = col + '0';

    if (r == -1) {
      /* vet via dead and frail masks */
      uint32_t buffer[NUM_ROW];
      for (i = 0; i < NUM_ROW; ++i) {
        enum det_types det_type = cfg_det_type(c, i);
        switch (det_type) {
          case det_healthy:
            buffer[i] = v;
            break;
          case det_frail:
            buffer[i] = cfg_frail_pid(l);
            break;
          case det_dead:
            buffer[i] = 0;
            break;
        }
      }
      push_block(rc, param, 0, buffer, NUM_ROW);
      if (l == 'i')
        memcpy(igain + c * NUM_ROW, buffer, sizeof(uint32_t) * NUM_ROW);
    } else 
      push_block(rc, param, r, &v, 1);
  }

  /* record default */
  if (a == PRM_DEFAULT_ONLY) {
    char name[] = "default_servo_?";
    name[14] = l;
    cfg_set_int(name, c, v);
  } else if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY
      || a == PRM_RECORD_RCONF)
  {
    char name[] = "servo_?";
    name[6] = l;
    cfg_set_int(name, c, v);
  }
}

static void prm_set_det_type(int c, int r, enum det_types h, int a)
{
  char name[] = "servo_?";

  switch (h) {
    case det_healthy:
      if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
        name[6] = 'p';
        prm_set_servo(c, r, 'p', cfg_get_int(name, r), PRM_APPLY_ONLY);
        name[6] = 'i';
        prm_set_servo(c, r, 'i', cfg_get_int(name, r), PRM_APPLY_ONLY);
        name[6] = 'd';
        prm_set_servo(c, r, 'd', cfg_get_int(name, r), PRM_APPLY_ONLY);
      }

      if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY
          || a == PRM_RECORD_RCONF)
      {
        cfg_set_int_cr("dead_detectors", c, r, 0);
        cfg_set_int_cr("frail_detectors", c, r, 0);
      }
      break;
    case det_dead:
      if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
        prm_set_servo(c, r, 'p', 0, PRM_APPLY_ONLY);
        prm_set_servo(c, r, 'i', 0, PRM_APPLY_ONLY);
        prm_set_servo(c, r, 'd', 0, PRM_APPLY_ONLY);
      }

      if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY
          || a == PRM_RECORD_RCONF)
      {
        cfg_set_int_cr("dead_detectors", c, r, 1);
        cfg_set_int_cr("frail_detectors", c, r, 0);
      }
      break;
    case det_frail:
      if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
        prm_set_servo(c, r, 'p', cfg_get_int("frail_servo_p", 0),
            PRM_APPLY_ONLY);
        prm_set_servo(c, r, 'i', cfg_get_int("frail_servo_i", 0),
            PRM_APPLY_ONLY);
        prm_set_servo(c, r, 'd', cfg_get_int("frail_servo_d", 0),
            PRM_APPLY_ONLY);
      }

      if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY
          || a == PRM_RECORD_RCONF)
      {
        cfg_set_int_cr("dead_detectors", c, r, 0);
        cfg_set_int_cr("frail_detectors", c, r, 1);
      }
      break;
  }

  if (a == PRM_RECORD_RCONF)
    state &= ~st_config;
}

/* integral clamp */
static void prm_integral_clamp(double v, int a)
{
  /* apply -- in this case we have to do the maths oursevles */
  if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
    int i, r, c;
    uint32_t fj = 0, ic = 0;
    uint32_t data[NUM_ROW], servo[8];
    char card[] = "rc1";
    for (r = 1; r <= 2; ++r) {
      int max_i = 0;

      card[2] = r + '0';

      /* calculate max_i */
      read_param(card, "servo_mode", 0, servo, 8);
      for (c = 0; c < 8; ++c) {
        /* if servo_mode != 3, ignore this column */
        if (servo[c] == 3) {
          for (i = 0; i < NUM_ROW; ++i) {
            if (max_i < igain[(8 * (r - 1) + c) * NUM_ROW + i])
              max_i = igain[(8 * (r - 1) + c) * NUM_ROW + i];
          }
        }
      }

      if (max_i == 0)
        ic = 0;
      else {
        read_param(card, "en_fb_jump", 0, &fj, 1);
        if (fj) {
          /* calcualte min_fq */
          int min_fq = 999999;
          char flx_quanta[] = "flx_quanta0";
          for (c = 0; c < 8; ++c) {
            flx_quanta[10] = c + '0';
            read_param(card, flx_quanta, 0, data, NUM_ROW);
            for (i = 0; i < NUM_ROW; ++i)
              if (min_fq > data[i] && data[i] > 0)
                min_fq = data[i];
          }

          ic = v * 127 * 4096 * min_fq / max_i;
        } else {
          ic = v * 8192 * 4096 / max_i;
        }
      }
      push_block(card, "integral_clamp", 0, &ic, 1);
      iclamp[r - 1] = ic;
    }
  }

  /* record */
  if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY || a == PRM_RECORD_RCONF) {
    cfg_set_float("integral_clamp_factor", 0, v);
    if (a == PRM_RECORD_RCONF)
      state &= ~st_config;
  }
}

/* ignore this command if the column is off */
static void prm_vet_bias(int p, const char *name, int c, int v, int a)
{
  if (cfg_get_int("columns_off", c))
    return;
  prm_set_int(p, name, c, v, a);
}

static void q_column_on(int c, uint32_t sa, uint32_t sq2, int a)
{
  /* column on means: SA and SQ2 are biased and servo mode is 3 */

  /* set biases -- we're synchronous with the meta director here so 
   * we don't have to worry about multiple PRM_RECORD_RCONFs */
  prm_set_int(sa_bias, "sa_bias", c, sa, a);
  prm_set_int(sq2_bias, "sq2_bias", c, sq2, a);

  if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
    /* apply servo mode */
    uint32_t three = 3;
    char rc[] = "rc1";
    int col = c;

    if (c > 8) {
      rc[3] = '2';
      col -= 8;
    }

    push_block(rc, "servo_mode", col, &three, 1);
  }

  /* record columns off */
  if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY || a == PRM_RECORD_RCONF) {
    cfg_set_int("columns_off", c, 0);
    if (a == PRM_RECORD_RCONF)
      state &= ~st_config;
  }
}

static void q_column_off(int c, int a)
{
  /* column off means: SA and SQ2 biases are zeroed and servo mode is 1 */

  /* zero biases -- we're synchronous with the meta director here so 
   * we don't have to worry about multiple PRM_RECORD_RCONFs */
  prm_set_int(sa_bias, "sa_bias", c, 0, a);
  prm_set_int(sq2_bias, "sq2_bias", c, 0, a);

  if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
    /* apply servo mode */
    uint32_t one = 1;
    char rc[] = "rc1";
    int col = c;

    if (c > 8) {
      rc[3] = '2';
      col -= 8;
    }

    push_block(rc, "servo_mode", col, &one, 1);
  }

  /* record columns off */
  if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY || a == PRM_RECORD_RCONF) {
    cfg_set_int("columns_off", c, 1);
    if (a == PRM_RECORD_RCONF)
      state &= ~st_config;
  }
}

static void q_servo_reset(int c, int r)
{
  const char *rc = "rc1";
  char block[] = "servo_rst_col#";
  uint32_t one = 1;
  uint32_t zero = 0;
  uint32_t bits[41] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  if (c > 8) {
    rc = "rc2";
    c -= 8;
  }

  bits[r] = 1;
  block[13] = c + '0';
  push_block_raw(rc, block, 0, bits, 41);
  push_block_raw(rc, "servo_rst_arm", 0, &one, 1);

  /* reset */
  bits[r] = 0;
  push_block_raw(rc, block, 0, bits, 41);
  push_block_raw(rc, "servo_rst_arm", 0, &zero, 1);
}

static void q_bias_tess(double k, int o, uint32_t *data, int n, int a)
{
  /* apply */
  if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
    push_block("tes", "bias", o, data, n);
    push_kick(k, 0, memory.bias_kick_bias);
    state |= st_biased;
  }

  /* record value */
  if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY || a == PRM_RECORD_RCONF) {
    cfg_set_intarr("tes_bias", o, data, n);
    if (a == PRM_RECORD_RCONF)
      state &= ~st_config;
  }
}

#define CFG_TOGGLE(on,off,name) \
  case on: cfg_set_int(name, 0, 1); break; \
case off: cfg_set_int(name, 0, 0); break
#define CFG_SETFLT(name) \
  case name: cfg_set_float(#name, 0, ev->rvalues[1]); break
#define CFG_SETFLTC(name) \
  case name: cfg_set_float(#name, ev->ivalues[1], ev->rvalues[2]); break
#define CFG_SETINT(cmd,name) \
  case cmd: cfg_set_int(name, 0, ev->ivalues[1]); break
#define CFG_SETINTC(cmd,name) \
  case cmd: cfg_set_int(name, ev->ivalues[1], ev->ivalues[2]); break
#define CFG_SETINTR CFG_SETINTC /* does the same thing */
#define CFG_SETINTCR(cmd,name) \
  case cmd: cfg_set_int_cr(name, ev->ivalues[1], ev->ivalues[2], \
                ev->ivalues[3]); break
#define CFG_SETSCS(name) \
  case name: \
cfg_set_int(#name "_start", 0, ev->ivalues[1]); \
cfg_set_int(#name "_count", 0, ev->ivalues[2]); \
cfg_set_int(#name "_step",  0, ev->ivalues[3]); \
break

#define PRM_TOGGLE(on,off,name) \
  case on: prm_set_int(on, name, 0, 1, ev->ivalues[1]); break; \
case off: prm_set_int(off, name, 0, 0, ev->ivalues[1]); break
#define PRM_TOGGLEC(on,off,name) \
  case on: prm_set_int(on, name, ev->ivalues[1], 1, ev->ivalues[2]); break; \
case off: prm_set_int(off, name, ev->ivalues[1], 0, ev->ivalues[2]); break
#define PRM_SETINT(cmd,name) \
  case cmd: prm_set_int(cmd, name, 0, ev->ivalues[1], ev->ivalues[2]); break
#define PRM_SETINTC(cmd,name) \
  case cmd: prm_set_int(cmd, name, ev->ivalues[1], ev->ivalues[2], \
                ev->ivalues[3]); break
#define PRM_SETINTR PRM_SETINTC /* does the same thing */
#define PRM_SETINTCR(cmd,name) \
  case cmd: prm_set_int_cr(cmd, name, ev->ivalues[1], ev->ivalues[2], \
                ev->ivalues[3], ev->ivalues[4]); break

/* Execute a command */
static void do_ev(const struct ScheduleEvent *ev, const char *peer, int port)
{
  uint32_t data[16];
  int i = 0;

  if (ev->is_multi) {
    switch (ev->command) {
      /* goal switching */
      case drive_check:
        try_mount = 1;
        state &= ~st_drives;
        /* reset lookback */
        if (ev->ivalues[1]) {
          memory.dmesg_lookback = time(NULL);
          mem_dirty = 1;
        }
        break;
      case reconfig:
        state &= ~st_config;
        new_goal.goal = gl_acq;
        change_goal = 1;
        break;
      case acquire:
        new_goal.goal = gl_acq;
        change_goal = 1;
        break;
      case cycle_acq:
        new_goal.goal = gl_cycle;
        change_goal = 1;
        break;
      case pause_acq:
        new_goal.goal = gl_pause;
        change_goal = 1;
        break;
      case lcloop:
        new_goal.goal = gl_lcloop;
        change_goal = 1;
        break;
      case bias_step:
        new_goal.step = ev->ivalues[1];
        new_goal.wait = ev->rvalues[2]; /* this is full period */
        new_goal.total = ev->rvalues[3];
        new_goal.goal = gl_bstep;
        change_goal = 1;
        break;
      case tune_biases:
      case tune_array:
        new_goal.force = (ev->command == tune_biases) ? 1 : 0;
        new_goal.apply = ev->ivalues[1];
        new_goal.goal = gl_tune;
        change_goal = 1;
        break;
      case partial_iv_curve:
        new_goal.kick = ev->rvalues[1] * 32767 / 5.;
        new_goal.kicktime = ev->rvalues[2];
        new_goal.kickwait = ev->rvalues[3];
        new_goal.start = ev->ivalues[4];
        new_goal.step = ev->ivalues[5];
        /* fix sign */
        if (new_goal.step > 0)
          new_goal.step = -new_goal.step;
        new_goal.wait = ev->rvalues[6];
        new_goal.goal = gl_partial;
        change_goal = 1;
        break;
      case auto_iv_params:
        memory.auto_iv_kick = ev->rvalues[1] * 32767 / 5.;
        memory.auto_iv_kicktime = ev->rvalues[2];
        memory.auto_iv_kickwait = ev->rvalues[3];
        memory.auto_iv_start = ev->ivalues[4];
        memory.auto_iv_stop = ev->ivalues[5];
        memory.auto_iv_step = ev->ivalues[6];
        /* fix sign */
        if ((memory.auto_iv_stop - memory.auto_iv_start) * memory.auto_iv_step <
            0)
        {
          memory.auto_iv_step = -memory.auto_iv_step;
        }
        memory.auto_iv_wait = ev->rvalues[7];
        break;
      case acq_iv_curve:
      case bias_ramp:
        new_goal.kick = ev->rvalues[1] * 32767 / 5.;
        new_goal.kicktime = ev->rvalues[2];
        new_goal.kickwait = ev->rvalues[3];
        new_goal.start = ev->ivalues[4];
        new_goal.stop = ev->ivalues[5];
        new_goal.step = ev->ivalues[6];
        /* fix sign */
        if ((new_goal.stop - new_goal.start) * new_goal.step < 0)
          new_goal.step = -new_goal.step;
        new_goal.wait = ev->rvalues[7];
        if (ev->command == acq_iv_curve) {
          new_goal.goal = gl_iv;
          new_goal.apply = ev->ivalues[8];
        } else
          new_goal.goal = gl_bramp;
        change_goal = 1;
        break;

      case sq1_ramp_check_params:
        i++;
        /* FALLTHROUGH */
      case sq1_servo_check_params:
        i++;
        /* FALLTHROUGH */
      case sq2_servo_check_params:
        i++;
        /* FALLTHROUGH */
      case sa_ramp_check_params:
        memory.p2p_abs_thresh[i] = ev->rvalues[1];
        memory.p2p_rel_thresh[i] = ev->rvalues[2];
        memory.slope_abs_thresh[i] = ev->rvalues[3];
        memory.slope_rel_thresh[i] = ev->rvalues[4];
        memory.range_abs_thresh[i] = ev->rvalues[5];
        memory.range_rel_thresh[i] = ev->rvalues[6];
        memory.count_thresh[i] = ev->ivalues[7];
        memory.ramp_shift[i] = ev->rvalues[8];
        memory.ramp_buffer[i] = ev->ivalues[9];
        memory.fail_thresh[i] = ev->ivalues[10];
        break;

      case sq1_ramp_check_crit:
        i++;
        /* FALLTHROUGH */
      case sq1_servo_check_crit:
        i++;
        /* FALLTHROUGH */
      case sq2_servo_check_crit:
        i++;
        /* FALLTHROUGH */
      case sa_ramp_check_crit:
        memory.check_p2p[i] = ev->ivalues[1];
        memory.check_slope[i] = ev->ivalues[2];
        memory.check_range[i] = ev->ivalues[3];
        memory.check_count[i] = ev->ivalues[4];
        break;

      case tuning_tries:
        memory.tune_global_tries = ev->ivalues[1];
        for (i = 0; i < 4; ++i)
          memory.tune_tries[i] = ev->ivalues[2 + i];
        mem_dirty = 1;
        break;
      case tuning_check_off:
        memory.tune_check_off = 1;
        mem_dirty = 1;
      case tuning_check_on:
        memory.tune_check_off = 0;
        mem_dirty = 1;
        break;
      case array_monitor:
        memory.ramp_max = ev->ivalues[1];
        memory.rst_wait = ev->ivalues[2] * 100; /* in units of 10 msec */
        memory.rst_tries = ev->ivalues[3];
        memory.off_trans_max = ev->ivalues[4];
        memory.off_trans_wait = ev->ivalues[5] * 100;
        memory.off_trans_thresh = ev->ivalues[6];
        mem_dirty = 1;
        reset_array_health();
        break;

        /* Experiment config commands */
      case column_off:
        q_column_off(ev->ivalues[1], ev->ivalues[2]);
        break;
      case column_on:
        q_column_on(ev->ivalues[1], ev->ivalues[2], ev->ivalues[3],
            ev->ivalues[4]);
        CFG_SETFLT(sa_offset_bias_ratio);
        CFG_TOGGLE(sa_ramp_bias_on, sa_ramp_bias_off, "sa_ramp_bias");
        CFG_SETSCS(sa_ramp_flux);
        CFG_SETSCS(sa_ramp_bias);
        CFG_SETINTC(sq2_tuning_row, "sq2_rows");
        CFG_SETFLTC(sq2_servo_gain);
        CFG_SETFLTC(sq1_servo_gain);
        CFG_TOGGLE(sq1_servo_bias_on, sq1_servo_bias_off,
            "sq1_servo_bias_ramp");
        CFG_SETSCS(sq1_servo_flux);
        CFG_SETSCS(sq1_servo_bias);
        CFG_TOGGLE(sq2_servo_bias_on, sq2_servo_bias_off,
            "sq2_servo_bias_ramp");
        CFG_SETSCS(sq2_servo_flux);
        CFG_SETSCS(sq2_servo_bias);
        CFG_SETSCS(sq1_ramp_flux);
        CFG_TOGGLE(sq1_ramp_tes_bias_on, sq1_ramp_tes_bias_off,
            "sq1_ramp_tes_bias");
        CFG_SETSCS(sq1_ramp_tes_bias);
        CFG_SETINTC(tes_bias_idle, "tes_bias_idle");
        CFG_SETINTC(tes_bias_normal, "tes_bias_normal");
        CFG_SETFLT(tes_bias_normal_time);
        CFG_TOGGLE(tuning_check_bias_on, tuning_check_bias_off,
            "tuning_check_bias");
        CFG_SETINT(tuning_therm_time, "tuning_therm_time");
        CFG_TOGGLE(tuning_do_plots_on, tuning_do_plots_off, "tuning_do_plots");
        CFG_SETINTC(sq2servo_safb_init, "sq2servo_safb_init");
        CFG_SETINTC(sq1servo_sq2fb_init, "sq1servo_sq2fb_init");
        PRM_SETINT(sample_num, "sample_num");
        PRM_SETINT(fb_dly, "fb_dly");
        PRM_SETINT(row_dly, "row_dly");
        PRM_TOGGLE(flux_jumping_on, flux_jumping_off, "flux_jumping");
        PRM_SETINTC(bias_tes_col, "tes_bias");
        CFG_SETINTC(sa_flux_quantum, "sa_flux_quanta");
        CFG_SETINTC(sq2_flux_quantum, "sq2_flux_quanta");
        CFG_SETINTCR(sq1_flux_quantum, "flux_quanta_all");
        PRM_SETINTR(sq1_bias, "sq1_bias");
        PRM_SETINTR(sq1_off_bias, "sq1_bias_off");
        PRM_SETINTCR(sq2_fb, "sq2_fb_set");
      case sa_bias:
        prm_vet_bias(sa_bias, "sa_bias", ev->ivalues[1], ev->ivalues[2],
            ev->ivalues[3]);
        break;
      case sq2_bias:
        prm_vet_bias(sq2_bias, "sq2_bias", ev->ivalues[1], ev->ivalues[2],
            ev->ivalues[3]);
        break;
        PRM_SETINTC(sa_fb, "sa_fb");
        PRM_SETINTC(sa_offset, "sa_offset");
        PRM_SETINTCR(adc_offset, "adc_offset_cr");

      case servo_pid_col:
        prm_set_servo(ev->ivalues[1], -1, 'p', ev->ivalues[2], ev->ivalues[5]);
        prm_set_servo(ev->ivalues[1], -1, 'i', ev->ivalues[3], ev->ivalues[5]);
        prm_set_servo(ev->ivalues[1], -1, 'd', ev->ivalues[4], ev->ivalues[5]);
        if (ev->ivalues[5] == PRM_RECORD_RCONF)
          state &= ~st_config;
        break;
      case servo_pid_pixel:
        prm_set_servo(ev->ivalues[1], ev->ivalues[2], 'p', ev->ivalues[2],
            PRM_APPLY_ONLY);
        prm_set_servo(ev->ivalues[1], ev->ivalues[2], 'i', ev->ivalues[3],
            PRM_APPLY_ONLY);
        prm_set_servo(ev->ivalues[1], ev->ivalues[2], 'd', ev->ivalues[4],
            PRM_APPLY_ONLY);
        break;
      case servo_pid_frail:
        cfg_set_int("frail_servo_p", 0, ev->ivalues[1]);
        cfg_set_int("frail_servo_i", 0, ev->ivalues[2]);
        cfg_set_int("frail_servo_d", 0, ev->ivalues[3]);
        break;
      case dead_detector:
        prm_set_det_type(ev->ivalues[1], ev->ivalues[2], det_dead,
            ev->ivalues[3]);
        break;
      case frail_detector:
        prm_set_det_type(ev->ivalues[1], ev->ivalues[2], det_frail,
            ev->ivalues[3]);
        break;
      case healthy_detector:
        prm_set_det_type(ev->ivalues[1], ev->ivalues[2], det_healthy,
            ev->ivalues[3]);
        break;
      case send_exptcfg:
        new_blob_type = BLOB_EXPCFG;
        break;
      case send_iv_curve:
        new_blob_type = BLOB_IV;
        blob_data[0] = ev->ivalues[1];
        blob_data[1] = ev->ivalues[2];
        blob_data[2] = ev->ivalues[3];
        break;
      case use_tuning:
        cfg_apply_tuning(ev->ivalues[1] ? ev->ivalues[1] : memory.last_tune, 1);
        break;
      case ref_tuning:
        memory.ref_tune = ev->ivalues[1] ? ev->ivalues[1] : memory.last_tune;
        mem_dirty = 1;
        break;
      case send_tuning:
        new_blob_type = BLOB_TUNECFG + ev->ivalues[2];
        blob_data[0] = ev->ivalues[1] ? ev->ivalues[1] : memory.last_tune;
        break;
      case bias_tes_rc1:
      case bias_tes_rc2:
        data[0] = ev->ivalues[2];
        data[1] = ev->ivalues[3];
        data[2] = ev->ivalues[4];
        data[3] = ev->ivalues[5];
        data[4] = ev->ivalues[6];
        data[5] = ev->ivalues[7];
        data[6] = ev->ivalues[8];
        data[7] = ev->ivalues[9];
        q_bias_tess(ev->rvalues[1], (ev->command == bias_tes_rc2) ? 8 : 0,
            data, 8, ev->ivalues[10]);
        break;
      case bias_tes_all:
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] =
          data[7] = data[8] = data[9] = data[10] = data[11] = data[12] =
          data[13] = data[14] = data[15] = ev->ivalues[2];
        q_bias_tess(ev->rvalues[1], 0, data, 16, ev->ivalues[3]);
        break;
      case tile_heater_on:
        data[0] = ev->ivalues[1] * 32767 / 5;
        push_block("heater", "bias", 0, data, 1);
        slow_dat.tile_heater = data[0];
        break;
      case tile_heater_off:
        data[0] = 0;
        push_block("heater", "bias", 0, data, 1);
        slow_dat.tile_heater = 0;
        break;
      case tile_heater_kick:
        push_kick(ev->rvalues[1], ev->rvalues[3], ev->ivalues[2]);
        break;
      case servo_reset:
        q_servo_reset(ev->ivalues[1], ev->ivalues[2]);
        break;
      case reload_mce_config:
        cfg_load_template();
        cfg_load_dead_masks();
        break;
      case flux_loop_init:
        data[0] = 1;
        bprintf(info, "Flux loop init");
        push_block_raw("rca", "flx_lp_init", 0, data, 1);
        stat_reset = 1;
        break;
      case integral_clamp:
        prm_integral_clamp(ev->rvalues[1], ev->ivalues[2]);
        break;
      case force_config:
        state |= st_config | st_mcecom;
        break;
      case mce_clock_int:
        if (!memory.sync_veto) {
          memory.sync_veto = 1;
          mem_dirty = 1;
        }
        break;
      case mce_clock_ext:
        if (memory.sync_veto) {
          memory.sync_veto = 0;
          mem_dirty = 1;
        }
        break;
      case bolo_stat_reset:
        stat_reset = 1;
        break;
      case bolo_stat_gains:
        for (i = 0; i < N_STAT_TYPES; i++) {
          memory.bolo_stat_gain[i] = 1. / log(1. + ev->rvalues[2 * i + 1]);
          memory.bolo_stat_offset[i] = ev->ivalues[2 * i + 2];
        }
        mem_dirty = 1;
        stat_reset = 1;
        break;
      case pick_biases:
        data[0] = ev->ivalues[2]; /* darks? */
        push_blockr("", "", ev->ivalues[1], data, 1, 3);
        break;
      case ref_biases:
        memory.ref_iv = ev->ivalues[1] ? ev->ivalues[1] : memory.last_iv;
        mem_dirty = 1;
        ref_biases_lite_ok = ref_biases_dark_ok = 0;
        break;
      case bias_kick_params:
        memory.bias_kick_val = ev->rvalues[1] * 32767 / 5.;
        memory.bias_kick_bias = ev->ivalues[2];
        memory.bias_kick_time = ev->rvalues[3] * 1e6;
        memory.bias_kick_wait = ev->ivalues[4];
        mem_dirty = 1;
        break;

      default:
        bprintf(warning, "Unrecognised multi command #%i from %s/%i\n",
            ev->command, peer, port);
        break;
    }

    /* flush the experiment.cfg if it has changed */
    flush_experiment_cfg(0);
  } else {
    switch (ev->command) {
      case restart_reset_on:
        memory.restart_reset = 1;
        mem_dirty = 1;
        break;
      case restart_reset_off:
        memory.restart_reset = 0;
        mem_dirty = 1;
        break;
      default:
        bprintf(warning, "Unrecognised single command #%i from %s/%i\n",
            ev->command, peer, port);
    }
  }
}

/* find the highest numbered thing */
int find_last_dirent(const char *what, int skip_chars)
{
  DIR *dir;
  struct dirent *ent;
  char path[100];
  char *endptr;
  int d, n, last = 0;

  sprintf(path, "/data#/mce/%s", what);

  for (d = 0; d < 4; ++d) {
    path[5] = d + '0';
    dir = opendir(path);
    if (dir == NULL)
      continue;
    while ((ent = readdir(dir))) {
      n = (int)strtol(ent->d_name + skip_chars, &endptr, 10);
      if (n < 0 || *endptr)
        continue;
      else if (last < n)
        last = n;
    }
    closedir(dir);
  }

  return last;
}

/* save memory data */
static void save_mem(void)
{
  char name[] = {"/data#/mce/mpc_mem"};
  int d;

  memory.time = time(NULL);
  for (d = 0; d < 4; ++d) 
    if (slow_dat.df[d]) {
      char template[] = {"/data#/mce/mpc_mem.XXXXXX"};
      name[5] = template[5] = d + '0';
      int fd = mkstemp(template);
      if (fd < 0)
        continue;
      ssize_t n = write(fd, &memory, sizeof(memory));
      close(fd);
      if (n < sizeof(memory))
        unlink(template);
      else {
        rename(template, name);
        bprintf(info, "Wrote memory to %s\n", name);
      }
    }
  mem_dirty = 0;
}

/* read or regenerate memory data -- this must run before thread creation */
static int read_mem(void)
{
  int i, d, have_mem = -1;
  struct memory_t new_memory;

  char name[] = {"/data#/mce/mpc_mem"};

  /* find the newest memory file */
  for (d = 0; d < 4; ++d) {
    name[5] = d + '0';
    /* try opening */
    int fd = open(name, O_RDONLY);
    if (fd < 0)
      continue;

    ssize_t n = read(fd, &new_memory, sizeof(new_memory));
    close(fd);

    if (n < sizeof(new_memory)) /* too small */
      continue;

    if (new_memory.version != MEM_VERS) /* too old */
      continue;

    /* newer than the current one? */
    if (new_memory.time > memory.time) {
      memcpy(&memory, &new_memory, sizeof(memory));
      have_mem = d;
    }
  }

  if (have_mem == -1) {
    /* no valid memory */
    bprintf(warning, "Regenerating memory.");
    memory.version = MEM_VERS;
    memory.last_tune = find_last_dirent("tuning", 0);
    memory.ref_tune = memory.last_tune; /* why not...? */
    memory.last_iv = find_last_dirent("ivcurves", 3);
    memory.ref_iv = memory.last_iv; /* I guess...? */
    memory.squidveto = 0;
    memory.used_tune = 0xFFFF; /* don't know */
    memory.sync_veto = 0;
    memory.divisor = 1;
    memory.dmesg_lookback = btime;
    memory.bias_kick_val = 3 /* Volts */ * 32767 / 5;
    memory.bias_kick_bias = 0; /* off */
    memory.bias_kick_time = 500000; /* microseconds */
    memory.bias_kick_wait = 30; /* seconds */
    memory.bolo_filt_len = 120;
    memory.bolo_filt_freq = 5.0;
    memory.bolo_filt_bw = 1.2;
    memory.bolo_stat_gain[bs_mean] = 1. / log( 1.07);
    memory.bolo_stat_gain[bs_sigma] = 1. / log( 1.03);
    memory.bolo_stat_gain[bs_noise] = 1. / log( 1.03);
    memory.bolo_stat_offset[bs_mean] = 5000;
    memory.bolo_stat_offset[bs_sigma] = 1000;
    memory.bolo_stat_offset[bs_noise] = 200;
    memory.restart_reset = 1;
    memory.tune_check_off = 0;
    memory.tune_tries[0] = 3;
    memory.tune_tries[1] = 3;
    memory.tune_tries[2] = 3;
    memory.tune_tries[3] = 3;
    memory.tune_global_tries = 1;
    memory.ramp_max = 20;
    memory.rst_wait = 60000; /* ten minutes */
    memory.off_trans_max = 20;
    memory.off_trans_wait = 60000; /* ten minutes */
    memory.off_trans_thresh = 0;
    memory.rst_tries = 5;
    for (i = 0; i < 4; ++i) {
      memory.check_count[i] = 1;
      memory.check_range[i] = 1;
      memory.check_slope[i] = 1;
      memory.check_p2p[i] = 1;
      memory.p2p_abs_thresh[i] = 0.3;
      memory.p2p_rel_thresh[i] = 0.1;
      memory.slope_abs_thresh[i] = 0.5;
      memory.slope_rel_thresh[i] = 0.25;
      memory.range_abs_thresh[i] = 0.5;
      memory.range_rel_thresh[i] = 0.25;
      memory.count_thresh[i] = 1;
      memory.fail_thresh[i] = 10;
      memory.ramp_shift[i] = 0.5;
      memory.ramp_buffer[i] = 3;
    }
    memory.auto_iv_kick = 19660; /* 3 volts */
    memory.auto_iv_kicktime = 0.3;
    memory.auto_iv_kickwait = 300;
    memory.auto_iv_start = 8000;
    memory.auto_iv_stop = 0;
    memory.auto_iv_step = -5;
    memory.auto_iv_wait = 0.2;
  } else
    bprintf(info, "Restored memory from /data%i", have_mem);

  return 1; /* always re-write memory on start-up */
}

/* find a tuning file -- returns 1 on error */
int tuning_filename(const char *stage, const char *file, int n, char *buffer)
{
  int r;
  glob_t pglob;
  char pattern[100];
  sprintf(pattern, "/data?/mce/tuning/%04i/%s/%s", n, stage, file);

  /* glob search */
  r = glob(pattern, GLOB_NOSORT, NULL, &pglob);
  if (r) {
    globfree(&pglob);
    return 1;
  }

  /* use the first one */
  strcpy(buffer, pglob.gl_pathv[0]);
  globfree(&pglob);

  return 0;
}

/* figure out the boot time (needed for dmesg scraping) */
static void set_btime(void)
{
  struct sysinfo sinfo;
  sysinfo(&sinfo);

  btime = time(NULL) - sinfo.uptime;
  bprintf(info, "System boot time: %s", asctime(gmtime(&btime)));
}

/* load uuids for the /data drives */
static void get_uuids(void)
{
  FILE *stream;
  char buffer[1024];
  char data[1024], c;

  stream = fopen("/etc/fstab", "r");
  if (stream == NULL) {
    bprintf(err, "Unable to read /etc/fstab");
    return;
  }

  /* look for "^UUID=[^ ]* /data[0-3]" */
  while (fgets(buffer, 1024, stream)) {
    if (buffer[0] != 'U')
      continue;

    if (sscanf(buffer, "UUID=%s /data%c", data, &c) == 2) {
      if (c >= '0' && c <= '3') {
        bprintf(info, "Found UUID=%s mapped to /data%c", data, c);
        uuid[(int)c - '0'] = strdup(data);
      }
    }
  }

  fclose(stream);
}

int main(void)
{
  int port, type;
  ssize_t n;
  size_t len;

  pthread_t mas_thread;
  pthread_t task_thread;
  pthread_t acq_thread;
  pthread_t blob_thread;

  int init_timer = 0;
  int slow_timer = 0;
  int synop_timer = 0;

  char peer[UDP_MAXHOST];
  char data[UDP_MAXSIZE];

  umask(0);

  buos_use_func(mputs);
  nameThread("Main");

  /* for scripts */
  setenv("PYTHONPATH", "/data/mas/mce_script/python:/data/mas/python", 1);
  setenv("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:"
      "/data/mas/bin:/data/mas/mce_script/script:"
      "/data/mas/mce_script/test_suite", 1);

  /* logapalooza */
  openMCElog("/data0/mce/mpc.log");
  openMCElog("/data1/mce/mpc.log");
  openMCElog("/data2/mce/mpc.log");
  openMCElog("/data3/mce/mpc.log");

  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC subsystem.");

  /* start time */
  start_time = time(NULL);

  /* figure out the boot time */
  set_btime();

  /* load non-volatile memory */
  mem_dirty = read_mem();

  /* hack for getting rid fo the initial reset */
  if (!memory.restart_reset)
    state = st_mcecom | st_biased;

  /* bind to the UDP port */
  sock = udp_bind_port(MPC_PORT, 1);
  if (sock < 0)
    bprintf(fatal, "Unable to bind port.");

  /* Figure out our MCE number */
  nmce = nmce_from_ip();

  /* compose array id */
  if (nmce == -1) {
    bputs(warning, "Using 'default' configuration");
    nmce = 4; /* be X5 */
    strcpy(array_id, "default");
  } else 
    array_id[1] = '1' + nmce;

  bprintf(info, "Controlling MCE #%i.  Array ID: %s", nmce, array_id);

  /* get the UUIDs of the /data drives */
  get_uuids();

  /* create the threads */
  pthread_create(&mas_thread, NULL, mas_data, NULL);
  pthread_create(&task_thread, NULL, task, NULL);
  pthread_create(&blob_thread, NULL, blobber, NULL);
  pthread_create(&acq_thread, NULL, acquer, NULL);

  signal(SIGHUP, crash_stop);
  signal(SIGINT, crash_stop);
  signal(SIGTERM, crash_stop);

  /* main loop */
  for (;;) {
    struct ScheduleEvent ev;
    int target;

    /* write non-volatile memory, if dirty */
    if (mem_dirty)
      save_mem();

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
          if ((target = mpc_decompose_command(&ev, nmce, n, data)) < 0) {
            /* command decomposition failed */
            break;
          }
          /* execute, if it's for us */
          if (target == 0 || target == 1 + nmce)
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

        if (udp_bcast(sock, MCESERV_PORT, len, data, 0) == 0)
          bputs(info, "Broadcast awake ping.\n");

        init_timer = INIT_TIMEOUT;
      } else if (n == 0)
        init_timer -= UDP_TIMEOUT;

      /* update but don't send slow data */
      send_slow_data(data, 0);
    } else if (!power_cycle_cmp) {
      /* finished the init; slow data */
      if (slow_timer <= 0) {
        send_slow_data(data, !slow_veto);
        slow_timer = SLOW_TIMEOUT;
      } else if (n == 0)
        slow_timer -= UDP_TIMEOUT;

      /* array stats */
      if (synop_timer <= 0) {
        send_array_synopsis();
        synop_timer = SYNOP_TIMEOUT;
      } else if (n == 0)
        synop_timer -= UDP_TIMEOUT;

      if (send_mceparam)
        ForwardMCEParam();

      if (send_blob)
        ForwardBlob();

      /* send data */
      while ((fb_top - pb_last + FB_SIZE) % FB_SIZE > PB_SIZE * memory.divisor)
        pushback();

      /* PCM requests */
      pcm_special(0, NULL, NULL, 0);
    }
  }

  close(sock);
  return 0;
}
