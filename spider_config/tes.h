/* tes.c: some TES stuff for PCM/MPC
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef TES_H
#define TES_H

#include <stdint.h>

/* this following three numbers define the addressable TES space in the flight
 * code; they do not affect MCE operation */
#define NUM_MCE 6
#define NUM_COL 16
#define NUM_ROW 33

#define N_STAT_TYPES 4
#define NUM_ARRAY_STAT (NUM_MCE*N_STAT_TYPES*NUM_COL*NUM_ROW)

enum bolo_stat {
  bs_mean,
  bs_sigma,
  bs_noise,
  bs_step
};

/* make a 16-bit TES number from a (subrack/column/row) triplet; returns -1 on
 * out-of-range values.
 */
const static inline int16_t TESNumber(int x, int r, int c)
{
  if (x < 0 || x >= NUM_MCE ||
      c < 0 || c >= NUM_COL ||
      r < 0 || r >= NUM_ROW)
  {
    return -1;
  }

  return (x << 12) + (c + r * NUM_COL);
}

/* mask off the mce number */
#define TES_LOCAL(t) ((t) & 0x0FFF)

/* do the reverse */
#define TES_MCE(t) ((t) >> 12)
#define TES_COL(t) (TES_LOCAL((t)) % NUM_COL)
#define TES_ROW(t) (TES_LOCAL((t)) / NUM_COL)

/* the offset into a COL * ROW length array -- simply just the local part */
#define TES_OFFSET(t) TES_LOCAL((t))

/* The number of MCE fast data channels in the downlink.  See fset.h for
 * caveats involved with making this larger than 255 */
#define NUM_MCE_FIELDS 128

#define PB_SIZE 10 /* number of frames in a TES packet */

#endif
