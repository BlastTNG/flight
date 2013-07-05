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
#include "mce_counts.h"
#include <stdlib.h>
#include <stdint.h>

#define FB_SIZE 40 /* number of frames in the frame buffer */
#define PB_SIZE 10 /* number of frames in PCM packet */

#pragma pack(1)
struct mas_header {
  uint32_t status, cc_frameno, row_len, num_rows_rep, data_rate, arz_count,
           header_vers, ramp_val;
  uint16_t ramp_card, ramp_param;
  uint32_t num_rows, syncno, runid, user_word;
  /* more stuff */
};
#pragma pack()

/* MPC globals */
extern int nmce;
extern int sock;
extern int in_turnaround;
extern int fake;
extern int init;
extern int cur_dm, req_dm;
extern int power_cycle_mce;
extern int power_cycle_cmp;
extern int command_veto;
extern int send_mcestat;
extern int divisor;
extern int veto;
extern int leech_veto;
extern uint16_t bset_num;
extern int ntes;
extern int16_t tes[NUM_ROW * NUM_COL];
extern int rd_count;
extern int pcm_strobe;
extern uint32_t pcm_frameno;
extern int pcm_ret_dat;
extern uint32_t mce_stat[N_MCE_STAT];

extern size_t frame_size;
extern int pb_last;
extern int fb_top;
extern uint32_t *frame[FB_SIZE];

/* The director */
void meta(void);

/* MCE data mode definitions */
#define N_DATA_MODES 13
extern struct data_mode_def {
  int first_bit, num_bits;
  enum { first, mean, sum } coadd_how;
} data_modes[N_DATA_MODES][2];

/* mpc statūs -- try to keep these in start order */
enum status {
  st_idle   = 0x0000, /* Not a status bit, just a "task" */

  st_drive0 = 0x0001, /* The primary drive is ready */
  st_drive1 = 0x0002, /* The secondary drive is ready */
  st_drive2 = 0x0004, /* The tertiary drive is ready */
  st_mcecom = 0x0008, /* MCE is talking */ 
  st_config = 0x0010, /* MCE is configured */
  st_acqcnf = 0x0020, /* Acquisition is configured */
  st_retdat = 0x0040  /* MCE is returning data */
};

extern unsigned int state;

/* operating modes */
enum modes { op_init = 0, op_ready, op_acq };
#define MODE_STRINGS "init", "ready", "acq"

/* high-level tasks */
extern enum status start_tk;
extern enum status  stop_tk;
extern enum modes      goal;

/* task handler */
void *task(void *dummy);

/* data tasklets */
enum dtask {
  dt_idle = 0, dt_setdir, dt_dsprs, dt_mcers, dt_reconfig, dt_startacq,
  dt_stopacq, dt_killacq, dt_fakestop, dt_empty, dt_status, dt_acqcnf,
};
#define DT_STRINGS \
  "idle", "setdir", "dsprs", "mcers", "reconfig", "startacq", \
  "stopacq", "killacq", "fakestop", "empty", "status", "acqcnf"
extern enum dtask data_tk;
extern int dt_error;
extern int comms_lost;

#define MPC_ETC_DIR "/data/mas/etc"

void *mas_data(void *dummy);
void *acquer(void *dummy);

/* The frame acq callback */
int frame_acq(unsigned long user_data, int frame_size, uint32_t *buffer);

#endif
