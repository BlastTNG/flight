/* MPC: MCE-PCM communicator
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
#ifndef MPC_H
#define MPC_H

#include "blast.h"
#include "tes.h"
#include <stdlib.h>
#include <stdint.h>

/* MPC globals */
extern int nmce;
extern int in_turnaround;
extern int fake;
extern int init;
extern int data_mode;
extern int power_cycle_mce;
extern int command_veto;
extern int veto;
extern uint16_t bset_num;
extern int ntes;
extern int16_t tes[NUM_ROW * NUM_COL];
extern uint16_t pcm_data[NUM_COL * NUM_ROW];
extern int pcm_strobe;
extern uint32_t pcm_frameno;
extern int pcm_ret_dat;

/* MCE data mode definitions */
#define N_DATA_MODES 13
extern struct data_mode_def {
  int first_bit, num_bits;
  enum { first, mean, sum } coadd_how;
} data_modes[N_DATA_MODES][2];

#define MPC_ETC_DIR "/data/mas/etc"

/* the MAS-MPC interface and its fake counterpart */
void *mas_data(void *dummy);
void *fake_data(void *dummy);

/* The frame-processing loop */
void do_frame(const uint32_t *frame, size_t frame_size);

/* Script routines */
int run_simple_script(const char *path, char *argv[]);
#endif
