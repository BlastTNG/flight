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

int bsa_init = 1;
static void bias_step_analysis(const uint32_t *frame, size_t frame_size,
    uint32_t frameno)
{
}

/* count clamped detectors */
static void count_clamped(const int32_t *frame, size_t frame_size)
{
  int i;
  int new_clamp_count = 0;

  /* skip header */
  frame += MCE_HEADER_SIZE;

  /* rc1 */
  if (iclamp[0] > 0)
    for (i = 0; i < 8 * NUM_ROW; ++i)
      if ((frame[i] & DATA_MASK) == iclamp[0] ||
          ((-frame[i]) & DATA_MASK) == iclamp[0])
        new_clamp_count++;

  frame += 8 * NUM_ROW;

  /* rc2 */
  if (iclamp[1] > 0)
    for (i = 0; i < 8 * NUM_ROW; ++i)
      if ((frame[i] & DATA_MASK) == iclamp[1] ||
          ((-frame[i]) & DATA_MASK) == iclamp[1])
        new_clamp_count++;

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

  /* update frame statistics */
  if (!stat_veto) {
    if (moda == md_bstep)
      bias_step_analysis(frame, frame_size, frameno);
    else {
      update_stats(frame, frame_size, frameno);
      bsa_init = 1;
    }
  }

  /* clamp counting */
  if (iclamp[0] > 0 || iclamp[1] > 0)
    count_clamped((const int32_t*)frame, frame_size);
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
