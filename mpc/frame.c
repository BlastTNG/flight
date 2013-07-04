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

#include <stdio.h>

#pragma pack(1)
struct mas_header {
  uint32_t status, cc_frameno, row_len, num_rows_rep, data_rate, arz_count,
           header_vers, ramp_val;
  uint16_t ramp_card, ramp_param;
  uint32_t num_rows, syncno, runid, user_word;
  /* more stuff */
};
#pragma pack()

/* do the frequency division and 16-bit conversion based on the data mode
 * definitions */
static int16_t coadd(uint32_t datum, uint16_t old_datum)
{
  uint16_t rec[2];
  uint32_t mask[2];
  int i;

  /* 16-bit-space masks */
  mask[0] = ((1 << data_modes[data_mode][0].num_bits) - 1)
    << data_modes[data_mode][1].num_bits;
  mask[1] = ((1 << data_modes[data_mode][1].num_bits) - 1);

  /* extract the subfield(s) */
  rec[0] = (uint16_t)(datum >> (data_modes[data_mode][0].first_bit -
          data_modes[data_mode][1].num_bits)) & mask[0];
  rec[1] = (uint16_t)(datum >> data_modes[data_mode][1].first_bit) & mask[1];

  if (pcm_strobe) { /* coadd */
    /* split the old datum, if neccessary */
    uint16_t old_rec[2];
    
    old_rec[0] = (data_modes[data_mode][0].coadd_how != first)
      ? old_datum & mask[0] : 0;

    old_rec[1] = (data_modes[data_mode][1].num_bits > 0 &&
        data_modes[data_mode][1].coadd_how != first) ?  old_datum & mask[1] : 0;

    /* coadd */
    for (i = 0; i < 2; ++i)
      switch (data_modes[data_mode][i].coadd_how) {
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

static void extract_data(uint32_t frameno, const uint32_t *frame,
    size_t frame_size)
{
  /* the buffer containing the first frame */
  static uint16_t data[NUM_ROW * NUM_COL];
  size_t i, ndata = frame_size / sizeof(uint32_t) - MCE_HEADER_SIZE;

  if (ndata > NUM_COL * NUM_ROW)
    ndata = NUM_COL * NUM_ROW;

  if (divisor == 1) {
    pcm_strobe = 0;
    for (i = 0; i < ndata; ++i) 
      pcm_data[i] = coadd(frame[i + MCE_HEADER_SIZE], 0);

    /* signal for transmission to PCM */
    pcm_frameno = frameno;
    pcm_ret_dat = 1;
  } else {
    if (pcm_strobe) { /* coadd */
      pcm_ret_dat = 0; /* avoid race condition */
      for (i = 0; i < ndata; ++i) 
        pcm_data[i] = coadd(frame[i + MCE_HEADER_SIZE], data[i]);

      /* signal for transmission to PCM */
      pcm_frameno = frameno;
      pcm_ret_dat = 1;
    } else /* just copy */
      for (i = 0; i < ndata; ++i) 
        data[i] = coadd(frame[i + MCE_HEADER_SIZE], 0);

    pcm_strobe = !pcm_strobe;
  }
}

static void do_frame(const uint32_t *frame, size_t frame_size)
{
  /* Initialise */
  const struct mas_header *header = (const struct mas_header *)frame;

  int sync_on = header->status & MCE_FSB_ACT_CLK;
  static uint32_t last_frameno = 0;
  static int last_veto = 1000;
  uint32_t frameno = sync_on ? header->syncno : header->cc_frameno;

  /* sequencing check */
  if (last_frameno && frameno - last_frameno != 1)
    bprintf(warning, "Sequencing error: %u, %u\n", last_frameno, frameno);
  last_frameno = frameno;

  /* "Helpful" messages */
  if (last_veto > 0) {
    if (header->status & MCE_FSB_LAST) {
      bprintf(info, "LAST bit in CC frame %u", header->cc_frameno);
      state &= ~st_retdat;
    }
    if (header->status & MCE_FSB_STOP) {
      bprintf(info, "STOP bit in CC frame %u", header->cc_frameno);
      state &= ~st_retdat;
    }
  }

  /* do more stuff here, probably */

  /* do the data extraction for PCM */
  extract_data(frameno, frame, frame_size);
}

/* the rambuff callback */
int frame_acq(unsigned long user_data, int frame_size, uint32_t *buffer)
{
  do_frame(buffer, (size_t)frame_size);

  return 0; /* MAS ignores this */
}
