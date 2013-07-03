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
#include "mce_counts.h"
#include "command_struct.h"
#include "mpc_proto.h"
#include "udp.h"
#include "tes.h"

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

#define UDP_TIMEOUT 100 /* milliseconds */

/* number of data packets with the wrong bset to allow before triggering a
 * resend of the bset */
#define BAD_BSET_THRESHOLD 2

extern int StartupVeto; /* in mcp.c */

/* semeuphoria */
static int sent_bset = -1; /* the last field set that was sent */
static int last_turnaround = -1; /* the last turnaround flag sent */

/* general purpose datagram buffer */
static char udp_buffer[UDP_MAXSIZE];

/* slow data */
int mce_slow_index[NUM_MCE];
struct mpc_slow_data mce_slow_dat[NUM_MCE][3];

/* super slow data */
uint32_t mce_param[N_MCE_STAT * NUM_MCE];
static int request_ssdata = 1; /* start-up request */
static int mceserv_InCharge; /* to look for edges */

/* TES reconstruction buffer */
#define MCE_PRESENT(m) (1 << (m))
#define TES_FRAME_FULL ((1 << NUM_MCE) - 1) /* lowest NUM_MCE bits set */

static struct tes_frame tes_buffer[2];

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
  static int last_dmb = 0;
  static int last_divisor = -1;
  int this_divisor = CommandData.bbcExtFrameRate;
  int this_turnaround = CommandData.pointing_mode.is_turn_around;
  size_t len;

  /* edge triggers */
  if ((last_turnaround != -1 && last_turnaround == this_turnaround) &&
      (this_divisor == last_divisor) &&
      (last_dmb == CommandData.data_mode_bits_serial) &&
      (request_ssdata == 0)
     )
  {
    /* no notices */
    return;
  }

  len = mpc_compose_notice(this_divisor, this_turnaround, request_ssdata,
      CommandData.data_mode_bits, udp_buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, udp_buffer, !InCharge))
    bprintf(warning, "Error broadcasting notifications\n");
  else {
    bprintf(warning, "Broadcast notifications\n");
    last_dmb = CommandData.data_mode_bits_serial;
    last_turnaround = this_turnaround;
    last_divisor = this_divisor;
    request_ssdata = 0;
  }
}

static void ForwardBSet(int sock)
{
  size_t len;
  int num;
  struct bset set;

  num = get_bset(&set);

  /* compose the packet */
  len = mpc_compose_bset(set.v, set.n, (uint16_t)num, udp_buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, udp_buffer, !InCharge))
    bprintf(warning, "Error broadcasting BSet 0x%04X\n", num);
  else {
    bprintf(warning, "Broadcast BSet 0x%04X\n", num);
    sent_bset = num;
  }
}

/* push tes_buffer[n] on the tes fifo; discards the new frame if FIFO full */
static void tes_push(int n)
{
  /* discard if full */
  if (tes_fifo_top < 0)
    return;

  pthread_mutex_lock(&tes_mex);

  /* do the push */
  memcpy(tes_fifo + tes_fifo_top, tes_buffer + n, sizeof(*tes_buffer));

  /* increment fifo top */
  tes_fifo_top = (tes_fifo_top + 1) % TES_FIFO_DEPTH;

  /* flag as full */
  if (tes_fifo_top == tes_fifo_bottom)
    tes_fifo_top = -1;
//  bprintf(warning, "TES_FIFO: %i:%i", tes_fifo_top, tes_fifo_bottom);

  pthread_mutex_unlock(&tes_mex);
}

/* do TES data frame reconstruction and push the data into the fifo */
static int insert_tes_data(int bad_bset_count, size_t len, const char *data,
    const char *peer, int port)
{
  /* are neither, one or both of the reconstruction buffers full? */
  static enum {empty, half, full } tes_recon_status = empty;

  struct bset local_set;
  uint16_t datain[MAX_BSET];
  uint16_t bset_num = get_bset(&local_set);
  uint32_t frameno_in;
  int i, mce, n;

  /* decode input datagram */
  mce = mpc_decompose_tes(&frameno_in, datain, len, data, bset_num,
      local_set.nm, &bad_bset_count, peer, port);

  if (mce < 0)
    return bad_bset_count;

  /* find framenum -- here 0 is the newer frame, 1 the older */
  n = (tes_buffer[0].frameno == frameno_in) ? 0
    : (tes_buffer[1].frameno == frameno_in) ? 1 : -1;

  /* new framenumber -- look for or make an empty frame */
  if (n == -1) {
    switch (tes_recon_status) {
      case full:
        /* push the oldest frame to the fifo */
        tes_push(1);

        /* FALLTHROUGH */
      case half:
        /* advance zero to one and reset zero */
        memcpy(tes_buffer + 1, tes_buffer, sizeof(tes_buffer[0]));
        memset(tes_buffer, 0, sizeof(tes_buffer[0]));

        tes_recon_status = full;
        break;
      case empty:
        tes_recon_status = half;
    }
    /* whatever happened above, we're now using buffer zero */
    n = 0;

    /* ignore non-reporting MCEs */
    tes_buffer[0].present = local_set.empties;

    /* record new framenum */
    tes_buffer[0].frameno = frameno_in;
  } else if (tes_buffer[n].present & MCE_PRESENT(mce)) {
    /* discard duplicates */
    bprintf(info, "Duplicate frame# 0x%08X from MCE%i", tes_buffer[n].frameno,
        mce);
    return bad_bset_count;
  }

  /* insert the new data */
  for (i = 0; i < local_set.nm[mce]; ++i)
    tes_buffer[n].data[local_set.im[mce][i] + 1] = datain[i + 1];

  /* remember that we got a frame from this MCE */
  tes_buffer[n].present |= MCE_PRESENT(mce);

  /* if it's finished, push and then reset it */
  if (tes_buffer[n].present == TES_FRAME_FULL) {
    tes_push(n);
    memset(tes_buffer + n, 0, sizeof(tes_buffer[0]));
    tes_recon_status = n ? half : empty;
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
  if (mce < 0) /* bad packet */
    return;

  /* These MCE numbers are zero based */
  bank =
    (mce == 0 || mce == 5) ? 0 :  /* MCE1 and MCE6 are on bank 0 */
    ((mce == 1 || mce == 2) ? 1 : /* MCE2 and MCE3 are on bank 1 */
     2);                          /* MCE4 and MCE5 are on bank 2 */

  if (power_cycle) {
    bprintf(info, "%s/%i req: cycle MCE#%i on bank %i", peer, port, mce, bank);

    /* Don't cycle the power if someone else turned it off */
    int mce_is_on = slow_data[mcePowerAddr->index][mcePowerAddr->channel] &
      (1 << bank);
    if (mce_is_on)
      CommandData.ifpower.mce_op[bank] = cyc;
  }
}

/* main routine */
void *mceserv(void *unused)
{
  int sock, port, type;
  int bad_bset_count = 0;
  ssize_t n;
  char peer[UDP_MAXHOST];
  int mccnum;

  nameThread("MCE");

  /* initialise the MPC protocol suite */
  if (mpc_init())
    bprintf(tfatal, "Unable to initialise MPC protocol subsystem.");

  sock = udp_bind_port(MCESERV_PORT, 1);

  if (sock == -1)
    bprintf(tfatal, "Unable to bind to port");

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
    if (sent_bset != CommandData.bset_num)
      ForwardBSet(sock);

    /* check inbound packets */
    n = udp_recv(sock, UDP_TIMEOUT, peer, &port, UDP_MAXSIZE, udp_buffer);

    /* n == 0 on timeout */
    type = (n > 0) ? mpc_check_packet(n, udp_buffer, peer, port) : -1;

    if (type < 0)
      continue;

    /* do something based on packet type */
    switch (type) {
      case 'A': /* "awake" packet from somebody, trigger retransmission of
                   "interesting" things. */

        /* this returns the mce number of the trasmitting computer, which we
         * promptly forget, or -1 on error.
         */
        if (mpc_decompose_init(n, udp_buffer, peer, port) >= 0) {
          sent_bset = -1; /* resend bset */
          last_turnaround = -1; /* resend notices */
        }
        break;
      case 'R': /* PCM command request */
        handle_pcm_request(n, peer, port);
        break;
      case 'S': /* slow data */
        mccnum = mpc_decompose_slow(mce_slow_dat, mce_slow_index, n, udp_buffer,
            peer, port);
        if (mccnum >= 0) {
          mccSlowCount[mccnum] = 0;
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
        mpc_decompose_stat(mce_param, n, udp_buffer, peer, port);
        break;
      default:
        bprintf(err, "Dropping packet with bad type 0x%X\n", type);
    }
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
//  bprintf(warning, "TES_FIFO: %i:%i", tes_fifo_top, tes_fifo_bottom);

  pthread_mutex_unlock(&tes_mex);
}
