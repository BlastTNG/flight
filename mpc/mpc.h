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
#include "mpc_proto.h"
#include "mce_counts.h"
#include <stdlib.h>
#include <stdint.h>

#define FB_SIZE 5000 /* number of frames in the frame buffer */

/* MPC globals */
extern int nmce;
extern int mas_get_temp;
extern int sock;
extern int in_turnaround;
extern int init;
extern int cur_dm, req_dm;
extern int power_cycle_mce;
extern int power_cycle_cmp;
extern int command_veto;
extern int send_mcestat;
extern int divisor;
extern int veto;
extern int leech_veto;
extern int data_drive[3];
extern char array_id[100];
extern uint16_t bset_num;
extern struct mpc_slow_data slow_dat;
extern int slow_veto;
extern int ntes;
extern int acq_init;
extern int sync_dv;
extern int16_t tes[NUM_ROW * NUM_COL];
extern int rd_count;
extern int pcm_strobe;
extern uint32_t pcm_frameno;
extern int pcm_ret_dat;
extern uint32_t mce_stat[N_MCE_STAT];
extern int32_t box_temp;
extern uint8_t drive_map;

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

/* mpc statūs -- try to keep these in start-up order */
enum status {
  st_idle   = 0x0000, /* Not a status bit, just a "task" */

  st_drive0 = 0x0001, /* The primary drive is ready */
  st_drive1 = 0x0002, /* The secondary drive is ready */
  st_drive2 = 0x0004, /* The tertiary drive is ready */
  st_mcecom = 0x0008, /* MCE is talking */ 
  st_config = 0x0010, /* MCE is configured */
  st_acqcnf = 0x0020, /* Acquisition is configured */
  st_retdat = 0x0040, /* MCE is returning data */
  st_tuning = 0x0080, /* auto_setup in progress */
};

extern unsigned int state;

/* operating modes -- op_acq must be the last mode */
enum modes { op_init = 0, op_ready, op_tune, op_acq };
#define MODE_STRINGS "init", "ready", "tune", "acq"

/* drive mapping */
#define DRIVE0_DATA0 0x0000
#define DRIVE0_DATA1 0x0001
#define DRIVE0_DATA2 0x0002
#define DRIVE0_DATA3 0x0003
#define DRIVE0_UNMAP 0x0004
#define DRIVE0_MASK  0x0007

#define DRIVE1_DATA0 0x0000
#define DRIVE1_DATA1 0x0008
#define DRIVE1_DATA2 0x0010
#define DRIVE1_DATA3 0x0018
#define DRIVE1_UNMAP 0x0020
#define DRIVE1_MASK  0x0038

/* can only be SSD */
#define DRIVE2_DATA0 0x0000
#define DRIVE2_DATA1 0x0040
#define DRIVE2_UNMAP 0x0080
#define DRIVE2_MASK  0x00C0

/* high-level tasks */
extern enum status start_tk;
extern enum status  stop_tk;
extern enum modes      goal;

/* task handler */
void *task(void *dummy);

/* data tasklets */
enum dtask {
  dt_idle = 0, dt_setdir, dt_dsprs, dt_mcers, dt_reconfig, dt_startacq,
  dt_fakestop, dt_empty, dt_status, dt_acqcnf, dt_autosetup, dt_delacq,
};
#define DT_STRINGS \
  "idle", "setdir", "dsprs", "mcers", "reconfig", "startacq", \
  "fakestop", "empty", "status", "acqcnf", "autosetup", "delacq"
extern enum dtask data_tk;
extern int dt_error;
extern int comms_lost;

#define MPC_ETC_DIR "/data/mas/etc"

void *mas_data(void *dummy);
void *acquer(void *dummy);

/* The frame acq callback */
int frame_acq(unsigned long user_data, int frame_size, uint32_t *buffer);

#endif
