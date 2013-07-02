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
extern int power_cycle_cmp;
extern int command_veto;
extern int veto;
extern uint16_t bset_num;
extern int ntes;
extern int16_t tes[NUM_ROW * NUM_COL];
extern uint16_t pcm_data[NUM_COL * NUM_ROW];
extern int pcm_strobe;
extern uint32_t pcm_frameno;
extern int pcm_ret_dat;

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
  st_mcecmd = 0x0008, /* mce_cmd is running */
  st_mcecom = 0x0010, /* MCE is talking */ 
  st_config = 0x0020, /* MCE is configured */
  st_acqcnf = 0x0040, /* Acquisition is configured */
  st_retdat = 0x0080  /* MCE is returning data */
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
  dt_stopacq, dt_killacq, dt_mcecmd_init, dt_mcecmd_fini, dt_fakestop,
  dt_empty, dt_status, dt_acqcnf, dt_multiend
};
#define DT_STRINGS "idle", "setdir", "dsprs", "mcers", "reconfig", "startacq", \
  "stopacq", "killacq", "mcecmd_init", "mcecmd_fini", "fakestop", "empty", \
"status", "acqcnf", "multiend"
extern enum dtask data_tk;
extern int dt_error;
extern int comms_lost;

#define MPC_ETC_DIR "/data/mas/etc"

/* the MAS-MPC interface and its fake counterpart */
void *mas_data(void *dummy);
void *fake_data(void *dummy);

/* The frame-processing loop */
void do_frame(const uint32_t *frame, size_t frame_size);

#endif
