/* MPC: MCE-PCM communicator
 *
 * data_mode.c: Describe MCE data_modes
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
#include "mpc.h"

/* the bit numbers here are never actually used (the real numbers are uplinked
 * from PCM, which remembers them */

/* ignore the second subfield */
#define IGNORED {0,0,coadd_first}
/* for unsupported data modes -- just take the bottom 16-bits, i guess */
#define OBSOLETE {{0,16,coadd_first},IGNORED}
struct data_mode_def data_modes[N_DATA_MODES][2] = {
  { { 16, 16, coadd_mean }, IGNORED },        /* 0: [31: 0] = error */
  { { 16, 16, coadd_mean }, IGNORED },        /* 1: [31: 0] = sq1_fb */
  { { 16, 16, coadd_mean }, IGNORED },        /* 2: [31: 0] = sq1_fb_filtered */
  OBSOLETE,                             /* 3 -- obsolete */
  { { 14,  9, coadd_mean }, { 0, 7, coadd_mean } }, /* 4: [31:14] = sq1_fb
                                           -  [13: 0] = error */
  { { 8, 12, coadd_mean }, { 0, 4, coadd_sum } },   /* 5: [31: 8] = sq1_fb
                                           -   [7: 0] = num_flux_jumps */
  OBSOLETE,                             /* 6 -- obsolete */
  OBSOLETE,                             /* 7 -- unsupported */
  OBSOLETE,                             /* 8 -- obsolete */
  OBSOLETE,                             /* 9 -- obsolete */
  { { 16, 16, coadd_mean }, { 0, 0, coadd_sum } }, /* 10:
                                           -  [31:7] = sq1_fb_filtered
                                           -   [6:0] = num_flux_jumps */
  { { 0, 16, coadd_first }, IGNORED },        /* 11: [9:3] row_index
                                           [2:0] column_index */
  { { 16, 16, coadd_mean }, IGNORED }
};
