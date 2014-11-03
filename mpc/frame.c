/* frame.c: Frame processing stuff
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
#include "mce_frame.h"
#include "mpc_proto.h"
#include "udp.h"

#include <stdio.h>
#include <string.h>

#define BSA_MAX  166667
#define BSA_MIN -333333

uint32_t igain[NUM_ROW * NUM_COL];

static double find_delta(int i, double delta, double dv, uint32_t ramp_val,
    int nd)
{
  delta = (nd * delta + 8 * dv / ramp_val) / (nd + 1);

  /* store */
  if (delta >= BSA_MAX)
    bolo_stat_buff[bs_step][i] = 255;
  else if (delta <= BSA_MIN)
    bolo_stat_buff[bs_step][i] = 0;
  else
    bolo_stat_buff[bs_step][i] = 256 * (delta - BSA_MIN) / (BSA_MAX - BSA_MIN);

  return delta;
}

int bsa_init = 1;
static void bias_step_analysis(const int32_t *frame, size_t frame_size,
    uint32_t frameno)
{
  static int is_on = 0;
  static int n;
  static double von[NUM_COL * NUM_ROW], voff[NUM_COL * NUM_ROW];
  static double delta[NUM_COL * NUM_ROW];
  static int nd;

  static uint32_t ramp_val;

  int i;

  const struct mas_header *header = (const struct mas_header *)frame;

  frame += MCE_HEADER_SIZE;

  /* initialisation */
  if (bsa_init) {
    /* wait for bias step on */
    if (header->ramp_val == 0)
      return;

    ramp_val = header->ramp_val;

    bsa_init = 0;

    n = nd = 0;
    is_on = 1;
    memset(von, 0, sizeof(double) * NUM_COL * NUM_ROW);
    memset(voff, 0, sizeof(double) * NUM_COL * NUM_ROW);
    memset(delta, 0, sizeof(double) * NUM_COL * NUM_ROW);
  }

  /* check whether we're on or not */
  if (header->ramp_val) {
    if (!is_on) {
      if (n > 0) {
        for (i = 0; i < NUM_ROW * NUM_COL; ++i) {
          voff[i] /= n;

          delta[i] = find_delta(i, delta[i], von[i] - voff[i], ramp_val, nd);
        }
        nd++;
      }
      is_on = 1;
      n = 0;
    }

    for (i = 0; i < NUM_ROW * NUM_COL; ++i)
      von[i] += (frame[i] >> 7);
    n++;
  } else {
    if (is_on) {
      if (n > 0) {
        for (i = 0; i < NUM_ROW * NUM_COL; ++i) {
          von[i] /= n;

          delta[i] = find_delta(i, delta[i], von[i] - voff[i], ramp_val, nd);
        }
      }
      is_on = 0;
      n = 0;
    }

    for (i = 0; i < NUM_ROW * NUM_COL; ++i)
      voff[i] += (frame[i] >> 7);
    n++;
  }
}

/* count raming detectors */
#define RAMP_LOOKBACK_LEN 10
#define RAMP_THRESHOLD (1U << 24)
static void count_ramped(const int32_t *frame)
{
  static int counter = 0;
  static int32_t vmin[NUM_COL * NUM_ROW];
  static int32_t vmax[NUM_COL * NUM_ROW];
  int new_ramp_count = 0;
  int i;

  /* skip header */
  frame += MCE_HEADER_SIZE;

  /* reset */
  if (stat_reset)
    counter = 0;

  if (counter == 0) { /* initialise */
    for (i = 0; i < NUM_ROW * NUM_COL; ++i)
      vmin[i] = vmax[i] = (frame[i] >> 7);
  } else {
    /* update */
    for (i = 0; i < NUM_ROW * NUM_COL; ++i) {
      int32_t v = frame[i] >> 7;
      if (v < vmin[i])
        vmin[i] = v;
      else if (v > vmax[i])
        vmax[i] = v;
    }

    /* check */
    if (counter == RAMP_LOOKBACK_LEN - 1) {
      for (i = 0; i < NUM_ROW * NUM_COL; ++i)
        if (vmax[i] - vmin[i] > RAMP_THRESHOLD)
          new_ramp_count++;

      slow_dat.ramp_count = new_ramp_count;
    }
  }

  counter = (counter + 1) % RAMP_LOOKBACK_LEN;
}

/* count clamped detectors */
static uint32_t cval[NUM_ROW * NUM_COL];
static int ccount[NUM_ROW * NUM_COL];
#define MIN_CLAMP_COUNT 500
static void count_clamped(const int32_t *frame)
{
  int i;
  int new_clamp_count = 0;

  /* skip header */
  frame += MCE_HEADER_SIZE;

  for (i = 0; i < NUM_COL * NUM_ROW; ++i)
    if (cval[i] && frame[i] == cval[i]) {
      if (ccount[i] > MIN_CLAMP_COUNT)
        new_clamp_count++;
      else
        ccount[i]++;
    } else {
      cval[i] = frame[i];
      ccount[i] = 0;
    }

  slow_dat.clamp_count = new_clamp_count;
}

static void do_frame(const uint32_t *frame, size_t frame_size, uint32_t frameno)
{
  const struct mas_header *header = (const struct mas_header *)frame;

  /* "Helpful" messages */
  if (header->status & MCE_FSB_LAST)
    bprintf(info, "LAST bit in CC frame %u", header->cc_frameno);
  if (header->status & MCE_FSB_STOP)
    bprintf(info, "STOP bit in CC frame %u", header->cc_frameno);

  /* clamp counting */
  if (iclamp[0] > 0 || iclamp[1] > 0)
    count_clamped((const int32_t*)frame);

  /* ramp counting */
  count_ramped((const int32_t*)frame);

  /* update frame statistics */
  if (!stat_veto) {
    if (moda == md_bstep)
      bias_step_analysis((const int32_t *)frame, frame_size, frameno);
    else {
      update_stats(frame, frame_size, frameno);
      bsa_init = 1;
    }
  }
}

/* the rambuff callback */
int frame_acq(unsigned long user_data, int frame_size, uint32_t *buffer)
{
  /* Initialise */
  const struct mas_header *header = (const struct mas_header *)buffer;
  sync_dv = (header->status & MCE_FSB_ACT_CLK) ? 1 : 0;
  const uint32_t frameno = sync_dv ? header->syncno : header->cc_frameno;
  static uint32_t last_frameno = 0;

  if (acq_init) {
    /* synchronise packets to frameno */
    if (frameno % PB_SIZE)
      return 0; /* skip */
    acq_init = 0;
  }

  /* copy to the frame buffer */
  memcpy(frame[fb_top], buffer, frame_size * sizeof(uint32_t));

  /* sequencing check */
  if (last_frameno && frameno - last_frameno != 1) {
    bprintf(warning, "Sequencing error: %u, %u %c | %u %u\n", last_frameno,
        frameno, sync_dv ? 's' : ' ', header->syncno, header->cc_frameno);
  }
  last_frameno = frameno;

  /* do stuff */
  do_frame(frame[fb_top], (size_t)(sizeof(uint32_t) * frame_size), frameno);

  fb_top = (fb_top + 1) % FB_SIZE;
  rd_count++;

  return 0; /* MAS ignores this */
}
