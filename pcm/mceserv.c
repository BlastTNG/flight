/* pcm: the Spider master control program
 *
 * mceserv.c: the MCE flight computer network server
 *
 * This software is copyright (C) 2012-2013 D. V. Wiebe
 *
 * pcm is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * pcm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pcm; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include "mceserv.h"
#include "mcp.h"
#include "crc.h"
#include "mce_counts.h"
#include "command_struct.h"
#include "mpc_proto.h"
#include "tx.h"
#include "sync_comms.h"
#include "udp.h"
#include "tes.h"

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

extern int StartupVeto; /* mcp.c */

/* empty the fifo on start-up */
int empty_tes_fifo = 1;

/* number of data packets with the wrong bset to allow before triggering a
 * resend of the bset */
#define BAD_BSET_THRESHOLD 2

static int sock = 0;

/* semeuphoria */
static int sent_bset = -1; /* the last field set that was sent */
static int last_turnaround = -1; /* the last turnaround flag sent */

/* general purpose datagram buffer */
static char udp_buffer[UDP_MAXSIZE];

/* slow data */
int mce_slow_index[NUM_MCE];
struct mpc_slow_data mce_slow_dat[NUM_MCE][3];

/* mcc status */
uint16_t mccs_alive, mccs_reporting;

/* super slow data */
uint32_t mce_param[N_MCE_STAT * NUM_MCE];
int request_ssdata = 1; /* start-up request */

/* array statistics */
uint8_t array_statistics[NUM_ARRAY_STAT];

/* general purpose blobs */

static const uint16_t blob_leadout[BLOB_LEADOUT_LEN] = BLOB_LEADOUT

uint16_t mce_blob_envelope[MCE_BLOB_ENVELOPE_MAX] = BLOB_LEADIN;

/* The payload always starts a fixed distance into the envelope */
static uint16_t *mce_blob_payload = mce_blob_envelope + BLOB_LEADIN_LEN + 2;

size_t mce_blob_size = 0; /* size of the blob, including envelope */

/* TES reconstruction buffer */
#define MCE_PRESENT(m) (1 << (m))
#define TES_FRAME_FULL ((1 << NUM_MCE) - 1) /* lowest NUM_MCE bits set */
#define TR_SIZE (PB_SIZE * 8) /* size of the reconstruction buffer */
static struct tes_frame tes_recon[TR_SIZE];
static int tes_recon_top = 0, tes_recon_bot = 0;

/* TES data FIFO */
static int tes_fifo_bottom = 0;
static int tes_fifo_top = 0;

static struct tes_frame tes_fifo[TES_FIFO_DEPTH];
pthread_mutex_t tes_mex = PTHREAD_MUTEX_INITIALIZER;

/* send a command if one is pending */
static int ForwardCommand(int sock)
{
  size_t len;
  const int cmd_idx = GETREADINDEX(CommandData.mcecmd_index);
  struct ScheduleEvent ev;

  /* all this saves us is potentially a useless memcpy */
  if (CommandData.mcecmd[cmd_idx].done || CommandData.mcecmd[cmd_idx].t == -1)
    return 0;

  memcpy(&ev, CommandData.mcecmd + cmd_idx, sizeof(ev));

  /* mitigate race conditions */
  if (ev.done || ev.t == -1)
    return 0;

  /* compose the command for transfer. */
  len = mpc_compose_command(&ev, udp_buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, udp_buffer, !InCharge))
    bprintf(warning, "Error broadcasting command.\n");
  else /* mark as written */
    CommandData.mcecmd[cmd_idx].done = 1;

  /* indicate something has been sent */
  return 1;
}

/* Send useful trivia to the MPC */
static void ForwardNotices(int sock)
{
  static int last_dr = -1, last_nr = -1, last_rl = -1;
  static int last_dmb = 0;
  static int last_divisor = -1;
  static int last_squidveto = -1;
  static int last_data_mode = -1;
  static double last_bolo_filt_freq;
  static double last_bolo_filt_bw;
  static int last_bolo_filt_len;

  /* Veto bits for MCE banks: 0 = X2 & X3; 1 = X4 & X6; 2 = X1 & X5 */
  const uint8_t pow_veto_bits[3] = { 0x6, 0x28, 0x11 };

  int this_divisor = CommandData.bbcIsExt ? CommandData.bbcExtFrameRate : 1;
  int this_turnaround = CommandData.pointing_mode.is_turn_around;
  int this_squidveto = CommandData.squidveto;
  int this_rl = CommandData.sync_box.rl_value;
  int this_nr = CommandData.sync_box.nr_value;
  int this_dr = CommandData.sync_box.fr_value;
  int this_dmb = CommandData.data_mode_bits_serial;
  int this_data_mode = CommandData.data_mode;
  double this_bolo_filt_freq = CommandData.bolo_filt_freq;
  double this_bolo_filt_bw = CommandData.bolo_filt_bw;
  int this_bolo_filt_len = CommandData.bolo_filt_len;
  size_t len;

  /* or with the mce_power-induced veto */
  if (CommandData.mce_power & 1)
    this_squidveto |= pow_veto_bits[0];
  if (CommandData.mce_power & 2)
    this_squidveto |= pow_veto_bits[1];
  if (CommandData.mce_power & 4)
    this_squidveto |= pow_veto_bits[2];

  this_squidveto |= CommandData.thermveto;

  /* edge triggers */
  if ((last_turnaround != -1 && last_turnaround == this_turnaround) &&
      (last_divisor == this_divisor) && (last_dmb == this_dmb) &&
      (last_rl == this_rl) && (last_nr == this_nr) && (last_dr == this_dr) &&
      (last_squidveto == this_squidveto) && (request_ssdata == 0) &&
      (last_data_mode == this_data_mode) &&
      (this_bolo_filt_freq == last_bolo_filt_freq) &&
      (this_bolo_filt_bw == last_bolo_filt_bw) &&
      (this_bolo_filt_len == last_bolo_filt_len)
     )
  {
    /* no notices */
    return;
  }

  len = mpc_compose_notice(this_divisor, this_turnaround, request_ssdata,
      this_data_mode, this_rl, this_nr, this_dr, this_squidveto,
      this_bolo_filt_freq, this_bolo_filt_bw, this_bolo_filt_len,
      CommandData.data_mode_bits[this_data_mode], udp_buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, udp_buffer, !InCharge))
    bprintf(warning, "Error broadcasting notifications\n");
  else {
    last_dmb = this_dmb;
    last_turnaround = this_turnaround;
    last_divisor = this_divisor;
    last_rl = this_rl;
    last_nr = this_nr;
    last_dr = this_dr;
    last_squidveto = this_squidveto;
    last_data_mode = this_data_mode;
    last_bolo_filt_freq = this_bolo_filt_freq;
    last_bolo_filt_bw = this_bolo_filt_bw;
    last_bolo_filt_len = this_bolo_filt_len;
    request_ssdata = 0;
  }
}

/* read the current bset num from the data return; if it has changed, update it
 */
static int get_bset(struct bset *local_set)
{
  static struct BiPhaseStruct *bsetAddr = NULL;
  int new_bset;

  if (bsetAddr == NULL)
    bsetAddr = GetBiPhaseAddr("bset");

  /* check for a bset change */
  new_bset = slow_data[bsetAddr->index][bsetAddr->channel];

  /* if it's different than the stored address, try loading the new number */
  if ((new_bset & 0xFF) != (curr_bset.num & 0xFF))
    CommandData.bset_num = change_bset(new_bset);

  /* return the bset */
  memcpy(local_set, &curr_bset, sizeof(curr_bset));
  return curr_bset.num;
}

static void ForwardBSet(int sock)
{
  size_t len;
  int num;
  struct bset set;

  num = get_bset(&set);

  /* nothing to do */
  if (sent_bset == num)
    return;

  /* compose the packet */
  len = mpc_compose_bset(set.v, set.n, (uint16_t)num, udp_buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, udp_buffer, !InCharge))
    bprintf(warning, "Error broadcasting BSet 0x%04X\n", num);
  else {
    sent_bset = num;
  }
}

/* push tes_recon[n] on the tes fifo; discards the new frame if FIFO full
 * also updates the list of non-reporting mces (nrx)
 */
#define NRX_THRESH (PB_SIZE * 10)
static uint16_t tes_push(uint16_t nrx, int *nrx_c)
{
  int i;

  tes_recon_bot = (tes_recon_bot + 1) % TR_SIZE;

  /* update nrx */
  for (i = 0; i < NUM_MCE; ++i)
    if (nrx & MCE_PRESENT(i)) { /* currently set as non-reporting */
      if (nrx_c[i] < PB_SIZE) {
        bprintf(info, "MCE%i reporting", i);
        nrx_c[i] = 0;
        nrx &= ~MCE_PRESENT(i);
      }
    } else { /* currently set as reporting */
      if (!(tes_recon[tes_recon_bot].present & MCE_PRESENT(i))) {
        if (nrx_c[i]++ > NRX_THRESH) {
          bprintf(warning, "MCE%i not reporting", i);
          nrx |= MCE_PRESENT(i);
        }
      }
    }

  /* discard if full */
  if (tes_fifo_top < 0)
    return nrx;

  /* do the push */
  memcpy(tes_fifo + tes_fifo_top, tes_recon + tes_recon_bot,
      sizeof(*tes_recon));

  pthread_mutex_lock(&tes_mex);

  /* increment fifo top */
  tes_fifo_top = (tes_fifo_top + 1) % TES_FIFO_DEPTH;

  /* flag as full */
  if (tes_fifo_top == tes_fifo_bottom)
    tes_fifo_top = -1;

  pthread_mutex_unlock(&tes_mex);

  return nrx;
}

/* this is a complement of empties, containing non-reporting MCEs */
int nrx_c[NUM_MCE] = {0, 0, 0, 0, 0, 0};
int last_mce_no[NUM_MCE] = {-1, -1, -1, -1, -1, -1};

  struct bset local_set;
/* do TES data frame reconstruction and push the data into the fifo */
static int insert_tes_data(int bad_bset_count, size_t len, const char *data,
    const char *peer, int port)
{

  uint16_t datain[MAX_BSET * PB_SIZE];
  uint16_t bset_num = get_bset(&local_set);
  uint32_t frameno_in[PB_SIZE];
  int f, mce, sync, n, nf, ntes, i, new = 1;

  int divisor = 1;
  if (CommandData.bbcIsExt && (CommandData.bbcExtFrameRate > 1))
    divisor = 2;

  /* decode input datagram */
  mce = mpc_decompose_tes(&nf, frameno_in, datain, len, data, bset_num,
      local_set.nm, &bad_bset_count, peer, port);

  /* sync bit on */
  sync = (mce & 0x80);
  mce &= 0x7F;

  if (mce < 0 || mce >= NUM_MCE)
    return bad_bset_count;

  ntes = local_set.nm[mce];

  for (f = 0; f < nf; ++f) {
    /* check frame sequencing */
    if (last_mce_no[mce] != -1 && frameno_in[f] - divisor != last_mce_no[mce])
      bprintf(warning, "Sequencing error, MCE%i: %i -> %i\n", mce,
          last_mce_no[mce], frameno_in[f]);
    last_mce_no[mce] = frameno_in[f];

    new = 1;
    if (tes_recon_top == tes_recon_bot) { /* buffer empty */
      ;
    } else if (sync) { /* synchronous mode: find the right buffer */
      for (n = tes_recon_top; n != (tes_recon_bot + 1) % TR_SIZE;
          n = (n + TR_SIZE - 1) % TR_SIZE)
      {
        if (tes_recon[n].frameno == frameno_in[f]) {
          new  = 0;
          break;
        }
      }
    } else { /* asynchronouse mode: find the oldest frame missing data from
                this MCE */
      for (n = (tes_recon_bot + 1) % TR_SIZE; n != tes_recon_top;
          n = (n + 1) % TR_SIZE)
      {
        if (!(tes_recon[n].present & MCE_PRESENT(mce))) {
          new = 0;
          break;
        }
      }
    }

    if (new) { /* new framenumber */
      /* in synchronous mode, frame numbers that have just recently happened
       * are discarded */
      if (sync && frameno_in[f] < tes_recon[tes_recon_bot].frameno &&
          frameno_in[f] + PB_SIZE > tes_recon[tes_recon_bot].frameno) {
        bprintf(info, "discarding late frame from MCE%i (%i < %i)", mce,
            frameno_in[f], tes_recon[tes_recon_bot].frameno);
        continue;
      }

      /* otherwise, new frame */
      n = tes_recon_top = (tes_recon_top + 1) % TR_SIZE;
      if ((n + 1) % TR_SIZE == tes_recon_bot) { /* buffer full */
        mccs_reporting = tes_push(mccs_reporting, nrx_c); /* push old frame */
      }

      /* zero the new buffer */
      memset(tes_recon + n, 0, sizeof(tes_recon[n]));

      /* ignore non-reporting MCEs */
      tes_recon[n].present = local_set.empties | mccs_reporting;

      /* record new framenum */
      tes_recon[n].frameno = frameno_in[f];
    } else if (tes_recon[n].present & MCE_PRESENT(mce)) {
      /* discard duplicates */
      bprintf(info, "Duplicate frame# 0x%08X from MCE%i", tes_recon[n].frameno,
          mce);
      continue;
    }

    /* insert the new data */
    for (i = 0; i < ntes; ++i)
      tes_recon[n].data[local_set.im[mce][i]] = datain[i + ntes * f];

    nrx_c[mce] = 0;

    /* remember that we got a frame from this MCE */
    tes_recon[n].present |= MCE_PRESENT(mce);

    /* if it's finished, and at the bottom of the buffer, push and then reset */
    if (n == (tes_recon_bot + 1) % TR_SIZE &&
        tes_recon[n].present == TES_FRAME_FULL)
    {
      mccs_reporting = tes_push(mccs_reporting, nrx_c);
      memset(tes_recon + n, 0, sizeof(tes_recon[0]));
    }
  }

  return bad_bset_count;
}

static void handle_pcm_request(size_t n, const char *peer, int port)
{
  static struct BiPhaseStruct* mcePowerAddr = NULL;

  int bank;

  if (mcePowerAddr == NULL)
    mcePowerAddr = GetBiPhaseAddr("mce_power");

  int power_cycle = 0;

  int mce = mpc_decompose_pcmreq(&power_cycle, n, udp_buffer, peer, port);
  if (mce < 0 || mce >= NUM_MCE) /* bad packet */
    return;

  /* These MCE numbers are zero based */
  bank =
    (mce == 1 || mce == 2) ? 0 :  /* MCE2 and MCE3 are on bank 0 */
    ((mce == 3 || mce == 5) ? 1 : /* MCE4 and MCE6 are on bank 1 */
     2);                          /* MCE1 and MCE5 are on bank 2 */

  if (power_cycle) {
    bprintf(info, "%s/%i req: cycle MCE#%i on bank %i", peer, port, mce, bank);

    /* Don't cycle the power if someone else turned it off */
    int mce_is_off = slow_data[mcePowerAddr->index][mcePowerAddr->channel] &
      (1 << bank);
    if (!mce_is_off && CommandData.ifpower.mce_mpcveto[bank] == 0) {
      CommandData.ifpower.mce_op[bank] = mce_pow_cyc;
      CommandData.ifpower.mce_mpcveto[bank] = MPC_POWER_VETO;
    }
  }
}

/* receiver routinee */
void *mcerecv(void *unused)
{
  int port, type;
  int bad_bset_count = 0;
  ssize_t n;
  char peer[UDP_MAXHOST];
  int mccnum;

  nameThread("MCE->");

  /* wait for resumption of sanity */
  while (StartupVeto)
    usleep(10000);

  sock = udp_bind_port(MCESERV_PORT, 1);

  if (sock == -1)
    bprintf(tfatal, "Unable to bind to port");

  /* service loop */
  for (;;) {
    /* check inbound packets - blocks */
    n = udp_recv(sock, 0, peer, &port, UDP_MAXSIZE, udp_buffer);

    type = (n > 0) ? mpc_check_packet(n, udp_buffer, peer, port) : -1;

    if (type < 0)
      continue;

    /* do something based on packet type */
    switch (type) {
      case 'A': /* "awake" packet from somebody, trigger retransmission of
                   "interesting" things. */

        /* this returns -1 on error.  */
        if (mpc_decompose_init(n, udp_buffer, peer, port) >= 0) {
          sent_bset = -1; /* resend bset */
          last_turnaround = -1; /* resend notices */
        }
        break;
      case 'G': /* GP data (blob) packet */

        /* because we do everything in-place here, we'll corrupt any blob
         * being sent out; but we'd truncate it anyways, so it doesn't really
         * matter
         */
        n = mpc_decompose_gpdata(mce_blob_payload, n, udp_buffer, peer, port);
        if (n > 0) {
          bprintf(info, "Received GP blob, from MCE %i, size %zu",
              mce_blob_payload[0] + 1, n);

          /* compute CRC -- goes immediately before and after the playload */
          mce_blob_payload[-1] = mce_blob_payload[n] = CalculateCRC(CRC_SEED,
              mce_blob_payload, n);

          /* the blob serial number */
          mce_blob_payload[-2] = CommandData.mce_blob_num + 1;

          /* append the leadout */
          memcpy(mce_blob_payload + n + 1, blob_leadout,
              BLOB_LEADOUT_LEN * sizeof(uint16_t));

          /* set the size */
          mce_blob_size = n + BLOB_LEADOUT_LEN + BLOB_LEADIN_LEN + 3;

          /* trigger send */
          CommandData.mce_blob_num++;
        }
        break;
      case 'Q': /* array synopsis */
        mpc_decompose_synop(array_statistics, n, udp_buffer, peer, port);
        break;
      case 'R': /* PCM command request */
        handle_pcm_request(n, peer, port);
        break;
      case 'S': /* slow data */
        mccnum = mpc_decompose_slow(mce_slow_dat, mce_slow_index, n, udp_buffer,
            peer, port);
        if (mccnum >= 0 && mccnum < NUM_MCE) {
          mccSlowCount[mccnum] = 0;
          mccs_alive |= (1U << mccnum);
        }

        break;
      case 'T': /* TES (fast) data */
        bad_bset_count = insert_tes_data(bad_bset_count, n, udp_buffer, peer,
            port);
        if (bad_bset_count > BAD_BSET_THRESHOLD) {
          bprintf(warning, "Resending BSet: %i bad packets", bad_bset_count);
          sent_bset = -1; /* resend bset */
          bad_bset_count = 0;
        }
        break;
      case 'Z': /* Super slow data */
        mpc_decompose_param(mce_param, n, udp_buffer, peer, port);
        break;
      default:
        bprintf(err, "Dropping packet with bad type 0x%X\n", type);
    }
  }

  return NULL;
}

/* send routine */
void *mcesend(void *unused)
{
  int mceserv_InCharge = 0; /* to look for edges */
  nameThread("->MCE");

  /* wait for socket initialisation */
  while (sock == 0)
    sleep(1);

  for (;;) {
    /* check for a change in in-chargeness */
    if (mceserv_InCharge != InCharge) {
      request_ssdata = 1;
      sent_bset = -1; /* resend bset */
      mceserv_InCharge = InCharge;
    }

    /* broadcast MCE commands */
    ForwardCommand(sock);

    /* Broadcast notices */
    ForwardNotices(sock);

    /* broadcast the field set, when necessary */
    ForwardBSet(sock);

    usleep(10000);
  }

  return NULL;
};

/* ===== TES fifo handling for tx.c ===== */

/* return the number of frames in the fifo */
int tes_nfifo(void)
{
  int tes_fifo_len;

  pthread_mutex_lock(&tes_mex);
  tes_fifo_len = tes_fifo_top < 0 ? TES_FIFO_DEPTH
    : (tes_fifo_top + TES_FIFO_DEPTH - tes_fifo_bottom) % TES_FIFO_DEPTH;
  pthread_mutex_unlock(&tes_mex);

  return tes_fifo_len;
}

/* return the oldest record in the TES fifo */
const struct tes_frame *tes_data(void)
{
  /* empty -- return error */
  if (tes_fifo_top == tes_fifo_bottom)
    return NULL;

  return tes_fifo + tes_fifo_bottom;
}

/* pop the oldest thing from the TES fifo */
void tes_pop(void)
{
  /* empty -- do nothing */
  if (tes_fifo_top == tes_fifo_bottom)
    return;

  pthread_mutex_lock(&tes_mex);

  /* no longer full */
  if (tes_fifo_top < 0)
    tes_fifo_top = tes_fifo_bottom;

  /* increment fifo bottom */
  tes_fifo_bottom = (tes_fifo_bottom + 1) % TES_FIFO_DEPTH;

  pthread_mutex_unlock(&tes_mex);
}
