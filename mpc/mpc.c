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

/* Heater-based kick depths */
#define KICK_1V  6550
#define KICK_2V 26200

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

/* Kill switch */
int terminate = 0;

/* Initialisation veto */
static int init = 1;

/* tuning stages */
int tune_first = 0, tune_last = 0;

/* iv curve parameters */
int iv_start, iv_step, iv_last;
uint32_t iv_kick;
double iv_kickwait, iv_wait;

/* reset the array statistics */
int stat_reset = 1;

/* Data return veto */
int veto = 1;

/* MCE temperature poll */
int mas_get_temp = 0;

/* PCM/MPC divisor */
int divisor = 2;

/* slow veto -- if vetoed, eventually PCM will reboot the MCC; ergo, we use this
 * as a watchdog */
int slow_veto = 0;

/* The slow data struct */
struct mpc_slow_data slow_dat;

struct block_q blockq[BLOCKQ_SIZE];
int blockq_head = 0, blockq_tail = 0;

/* drive map */
uint8_t drive_map = DRIVE0_UNMAP | DRIVE1_UNMAP | DRIVE2_UNMAP;
int data_drive[3] = {-1, -1, -1};

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
   
/* handles the divide-by-two frequency scaling for PCM transfer */
static int pcm_strobe = 0;

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

/* bias_tess values */
uint32_t bias_tess_val[8];
int bias_tess_card;

/* tile heater kick timeout */
static int tile_heater_timeout = -1;

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
  int new_row_len = -1, new_num_rows = -1, new_data_rate = -1;

  if (data_in) {
    /* reply acknowledgement from PCM */
    const char *data_mode_bits;
    int ssreq;

    if (mpc_decompose_notice(nmce, &data_mode_bits, &in_turnaround,
          &divisor, &ssreq, &new_row_len, &new_num_rows, &new_data_rate, len,
          data_in, peer, port))
      return;

    if (ssreq)
      send_mceparam = 1;
    
    if (new_row_len > 1 && new_data_rate > 1 && new_num_rows > 1) {
      row_len = new_row_len;
      num_rows = new_num_rows;
      data_rate = new_data_rate;
      cfg_update_timing(new_row_len, new_num_rows, new_data_rate);
    }

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
      bprintf(info, "/data%i now available", n);
      disk_bad[n] = 0;
    }
    check_timeout[n] = 0;
  } else {
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
  size_t len = mpc_compose_synop(mean, sigma, noise, nmce, data);
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

  slow_dat.data_mode = cur_dm;

  check_disk(0);
  check_disk(1);
  check_disk(2);
  check_disk(3);

  /* time -- this wraps around ~16 months after the epoch */
  gettimeofday(&tv, NULL);
  slow_dat.time = (tv.tv_sec - MPC_EPOCH) * 100 + tv.tv_usec / 10000;

  /* mcc temperature */
  if ((stream = fopen("/sys/devices/platform/coretemp.0/temp2_input", "r"))) {
    if (fscanf(stream, "%i\n", &datum) == 1)
      slow_dat.t_mcc = datum / 10;
    fclose(stream);
  } else
    slow_dat.t_mcc = 0x8000;

  /* mce temp */
  slow_dat.t_mce = box_temp;

  /* state stuff */
  slow_dat.state = state;

  slow_dat.goal = goal;
  slow_dat.task = (stop_tk == st_idle) ? start_tk : (0x8000 | stop_tk);
  slow_dat.dtask = data_tk;

  /* drive map */
  slow_dat.drive_map = drive_map;

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

  return (addr >> 24) - 81;
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
    
    old_rec[0] = (data_modes[cur_dm][0].coadd_how != coadd_first)
      ? old_datum & mask[0] : 0;

    old_rec[1] = (data_modes[cur_dm][1].num_bits > 0 &&
        data_modes[cur_dm][1].coadd_how != coadd_first) ? old_datum & mask[1] :
      0;

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

//    bprintf(info, "PB: %i > %i", frameno[0], frameno[PB_SIZE - 1]);

    /* pushback */
    ForwardData(frameno);
  }
  pb_last = (pb_last + PB_SIZE) % FB_SIZE;
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

  new_head = (blockq_head + 1) % BLOCKQ_SIZE;

  blockq[new_head].c = c;
  blockq[new_head].p = p;
  memcpy(blockq[new_head].d, d, sizeof(uint32_t) * n);
  blockq[new_head].n = n;
  blockq[new_head].o = o;
  blockq[new_head].raw = raw;

  blockq_head = new_head;
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
    case column_off:
      if (n > 8) {
        card[0] = "rc2";
        n -= 8;
      } else
        card[0] = "rc1";
      v = 1;
      param = "servo_mode";
      break;
    case column_on:
      if (n > 8) {
        card[0] = "rc2";
        n -= 8;
      } else
        card[0] = "rc1";
      v = 3;
      param = "servo_mode";
      break;
    case readout_row_index:
    case sample_dly:
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
    case num_rows_reported:
      card[0] = "cc";
      card[1] = "rc1";
      card[2] = "rc2";
      break;
    case tes_bias:
      card[0] = "tes";
      param = "bias";
      break;
    case sq1_bias:
      card[0] = "ac";
      param = "on_bias";
      break;
    case sq1_bias_off:
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
    char param[] = "gain?#";
    char rc[] = "rc1";
    if (c >= 8) {
      c -= 8;
      rc[2] = '2';
    }
    param[4] = l;
    param[5] = c + '0';

    if (r == -1) {
      uint32_t buffer[33];
      for (i = 0; i < 33; ++i)
        buffer[i] = v;
      push_block(rc, param, 0, buffer, 33);
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

static void prm_set_pixel(int c, int r, int h, int a)
{
  char name[] = "servo_?";

  switch (h) {
    case 0: /* healthy */
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
    case 1: /* dead */
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
    case 2: /* frail */
      if (a == PRM_APPLY_RECORD || a == PRM_APPLY_ONLY) {
        prm_set_servo(c, r, 'p', cfg_get_int("frail_servo_p", r),
            PRM_APPLY_ONLY);
        prm_set_servo(c, r, 'i', cfg_get_int("frail_servo_i", r),
            PRM_APPLY_ONLY);
        prm_set_servo(c, r, 'd', cfg_get_int("frail_servo_d", r),
            PRM_APPLY_ONLY);
      }

      if (a == PRM_APPLY_RECORD || a == PRM_RECORD_ONLY
          || a == PRM_RECORD_RCONF)
      {
        cfg_set_int_cr("dead_detectors", c, r, 0);
        cfg_set_int_cr("frail_detectors", c, r, 1);
      }
      break;
      break;
  }

  if (a == PRM_RECORD_RCONF)
    state &= ~st_config;
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

  if (ev->is_multi) {
    switch (ev->command) {
      case data_mode:
        req_dm = ev->ivalues[1];
        break;
      case reconfig:
        state &= ~st_config;
        goal = op_acq;
        break;
      case start_acq:
        goal = op_acq;
        break;
      case stop_acq:
        goal = op_ready;
        break;
      case tune_array:
        tune_first = ev->ivalues[1];
        tune_last = ev->ivalues[2];
        goal = op_tune;
        break;

        /* Experiment config commands */
        PRM_TOGGLEC(column_off, column_on, "columns_off");
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
        CFG_SETSCS(sq1_ramp_bias);
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
        CFG_SETSCS(ramp_tes);
        CFG_SETINT(ramp_tes_final_bias, "ramp_tes_final_bias");
        CFG_SETINT(ramp_tes_initial_pause, "ramp_tes_initial_pause");
        PRM_SETINT(num_rows_reported, "num_rows_reported");
        PRM_SETINT(readout_row_index, "readout_row_index");
        PRM_SETINT(sample_dly, "sample_dly");
        PRM_SETINT(sample_num, "sample_num");
        PRM_SETINT(fb_dly, "fb_dly");
        PRM_SETINT(row_dly, "row_dly");
        PRM_TOGGLE(flux_jumping_on, flux_jumping_off, "flux_jumping");
        CFG_SETINT(mce_servo_mode, "servo_mode");
        PRM_SETINTC(tes_bias, "tes_bias");
        CFG_SETINTC(sa_flux_quantum, "sa_flux_quanta");
        CFG_SETINTC(sq2_flux_quantum, "sq2_flux_quanta");
        CFG_SETINTCR(sq1_flux_quantum, "flux_quanta_all");
        PRM_SETINTR(sq1_bias, "sq1_bias");
        PRM_SETINTR(sq1_bias_off, "sq1_bias_off");
        PRM_SETINTC(sq2_bias, "sq2_bias");
        PRM_SETINTCR(sq2_fb, "sq2_fb_set");
        PRM_SETINTC(sa_bias, "sa_bias");
        PRM_SETINTC(sa_fb, "sa_fb");
        PRM_SETINTC(sa_offset, "sa_offset");
        PRM_SETINTCR(adc_offset, "adc_offset_cr");

      case mce_servo_pid:
        prm_set_servo(ev->ivalues[1], -1, 'p', ev->ivalues[2], ev->ivalues[5]);
        prm_set_servo(ev->ivalues[1], -1, 'i', ev->ivalues[3], ev->ivalues[5]);
        prm_set_servo(ev->ivalues[1], -1, 'd', ev->ivalues[4], ev->ivalues[5]);
        if (ev->ivalues[5] == PRM_RECORD_RCONF)
          state &= ~st_config;
        break;
      case pixel_servo_pid:
        prm_set_servo(ev->ivalues[1], ev->ivalues[2], 'p', ev->ivalues[2],
            PRM_APPLY_ONLY);
        prm_set_servo(ev->ivalues[1], ev->ivalues[2], 'i', ev->ivalues[3],
            PRM_APPLY_ONLY);
        prm_set_servo(ev->ivalues[1], ev->ivalues[2], 'd', ev->ivalues[4],
            PRM_APPLY_ONLY);
        break;
      case frail_servo_pid:
        cfg_set_int("frail_servo_p", 0, ev->ivalues[1]);
        cfg_set_int("frail_servo_i", 0, ev->ivalues[2]);
        cfg_set_int("frail_servo_d", 0, ev->ivalues[3]);
        break;
      case dead_detector:
        prm_set_pixel(ev->ivalues[1], ev->ivalues[2], 1, ev->ivalues[3]);
        break;
      case frail_detector:
        prm_set_pixel(ev->ivalues[1], ev->ivalues[2], 2, ev->ivalues[3]);
        break;
      case healthy_detector:
        prm_set_pixel(ev->ivalues[1], ev->ivalues[2], 0, ev->ivalues[3]);
        break;
      case send_exptcfg:
        new_blob_type = BLOB_EXPCFG;
        break;
      case acq_iv_curve:
        goal = op_iv;
        iv_kick = (ev->ivalues[1] == 1) ? KICK_1V :
          (ev->ivalues[2] == 2) ? KICK_2V : 0;
        iv_kickwait = ev->rvalues[2];
        iv_start = ev->ivalues[3];
        iv_last = ev->ivalues[4];
        iv_step = ev->ivalues[5];
        /* fix sign */
        if ((iv_last - iv_start) * iv_step < 0)
          iv_step = -iv_step;
        iv_wait = ev->rvalues[6];
        break;
      case send_iv_curve:
        new_blob_type = BLOB_IV;
        blob_data[0] = ev->ivalues[1];
        blob_data[1] = ev->ivalues[2];
        break;
      case use_tuning:
        cfg_apply_tuning(ev->ivalues[1]);
        break;
      case send_tuning:
        new_blob_type = BLOB_TUNECFG;
        blob_data[0] = ev->ivalues[1];
        break;
      case bias_tess:
        data[0] = ev->ivalues[2];
        data[1] = ev->ivalues[3];
        data[2] = ev->ivalues[4];
        data[3] = ev->ivalues[5];
        data[4] = ev->ivalues[6];
        data[5] = ev->ivalues[7];
        data[6] = ev->ivalues[8];
        data[7] = ev->ivalues[9];
        push_block("tes", "bias", ev->ivalues[1] ? 8 : 0, data, 8);
        break;
      case bias_tess_all:
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] =
          data[7] = data[8] = data[9] = data[10] = data[11] = data[12] =
          data[13] = data[14] = data[15] = ev->ivalues[1];
        push_block("tes", "bias", 0, data, 16);
        break;
      case stop_mce:
        task_special = TSPEC_STOP_MCE;
        break;
      case tile_heater_on:
        data[0] = ev->ivalues[1];
        push_block("heater", "bias", 0, data, 1);
        tile_heater_timeout = -1;
        break;
      case tile_heater_off:
        data[0] = 0;
        push_block("heater", "bias", 0, data, 1);
        tile_heater_timeout = -1;
        break;
      case tile_heater_kick:
        data[0] = ev->ivalues[1];
        push_block("heater", "bias", 0, data, 1);
        tile_heater_timeout = ev->rvalues[2] * 1000;
        break;
      case servo_reset:
        q_servo_reset(ev->ivalues[1], ev->ivalues[2]);
        break;

      default:
        bprintf(warning, "Unrecognised multi command #%i from %s/%i\n",
            ev->command, peer, port);
        break;
    }

    /* flush the experiment.cfg if it has changed */
    flush_experiment_cfg();
  } else {
    bprintf(warning, "Unrecognised single command #%i from %s/%i\n",
        ev->command, peer, port);
  }
}

/* find the highest numbered thing */
int find_last_dirent(const char *what)
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
      n = (int)strtol(ent->d_name, &endptr, 10);
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
  int d, have_mem = -1;
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
    memory.last_tune = find_last_dirent("tuning");
    memory.last_iv = find_last_dirent("ivcurve");
    return 1;
  }

  bprintf(info, "Restored memory from /data%i", have_mem);
  return 0;
}

/* find a tuning -- returns 1 on error */
int tuning_filename(const char *file, int n, char *buffer)
{
  struct stat statbuf;
  int d = 0;
  sprintf(buffer, "/data#/mce/tuning/%04i/%s", n, file);
  for (d = 0; d < 4; ++d) {
    buffer[5] = d + '0';
    if (stat(buffer, &statbuf) == 0)
      return 0;
  }

  return 1;
}

int main(void)
{
  int port, type;
  ssize_t n;
  size_t len;

  pthread_t data_thread;
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
  setenv("PATH", "usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:"
      "/data/mas/bin:/data/mas/mce_script/script:"
      "/data/mas/mce_script/test_suite", 1);

  /* logapalooza */
  openMCElog("/data0/mce/mpc.log");
  openMCElog("/data1/mce/mpc.log");
  openMCElog("/data2/mce/mpc.log");
  openMCElog("/data3/mce/mpc.log");

  bputs(startup, "This is MPC.\n");
  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC subsystem.");

  /* load non-volatile memory */
  mem_dirty = read_mem();

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

  /* init slow data */
  send_slow_data(data, 0);

  /* create the threads */
  pthread_create(&data_thread, NULL, mas_data, NULL);
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

        /* Broadcast this to everyone */
        if (udp_bcast(sock, MCESERV_PORT, len, data, 0) == 0)
          bputs(info, "Broadcast awake ping.\n");

        init_timer = INIT_TIMEOUT;
      } else if (n == 0)
        init_timer -= UDP_TIMEOUT;
    } else if (!power_cycle_cmp) {
      /* finished the init; slow data */
      if (slow_timer <= 0) {
        if (!slow_veto)
          send_slow_data(data, 1);
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
      while ((fb_top - pb_last + FB_SIZE) % FB_SIZE > PB_SIZE)
        pushback();

      /* PCM requests */
      pcm_special(0, NULL, NULL, 0);
    }

    /* tile heater timer */
    if (tile_heater_timeout >= 0) {
      if ((tile_heater_timeout -= UDP_TIMEOUT) <= 0) {
        uint32_t zero = 0;
        push_block("heater", "bias", 0, &zero, 1);
        tile_heater_timeout = -1;
      }
    }
  }

  close(sock);
  return 0;
}
