/* MPC: MCE-PCM communicator
 *
 * mce_frame: Describe the MCE frame here.
 *
 * This software is copyright (C) 2013 D. V. Wiebe
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

#include <stdint.h>

/* The size (in 32-bit words) of the MCE frame header */
#define MCE_HEADER_SIZE 43

/* Frame status bits */
#define MCE_FSB_LAST    0x0000001 /* last frame bit */
#define MCE_FSB_STOP    0x0000002 /* stop acq bit */
#define MCE_FSB_SYNC_FR 0x0000004 /* sync box free run */
#define MCE_FSB_SYNC_ER 0x0000008 /* sync box error */
#define MCE_FSB_ACT_CLK 0x0000010 /* active clock: set if using sync box */
/* bits 5-8 are reserved */
/* bit 9 isn't relevant to Spider */
#define MCE_FSB_RC1     0x0000400 /* RC1 reporting */
#define MCE_FSB_RC2     0x0000800 /* RC2 reporting */
/* bits 12-13 aren't relevant to Spider */
/* bits 14-15 are reserved */
#define MCE_FSB_NUM_COL 0x000F000 /* number of columns per RC */
#define MCE_FSB_DT_ERR  0x0010000 /* data timing error */
/* bits 21-31 are reserved */

#pragma pack(1)
struct mas_header {
  uint32_t status, cc_frameno, row_len, num_rows_rep, data_rate, arz_count,
           header_vers, ramp_val;
  uint16_t ramp_card, ramp_param;
  uint32_t num_rows, syncno, runid, user_word, errno1;
  uint32_t fpga_temp[9], errno2;
  uint32_t card_temp[9], errno3;
  uint32_t reserved[7], errno4;
   int32_t box_temp;
};
#pragma pack()

