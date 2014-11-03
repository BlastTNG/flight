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
#include "mce_blob.h"
#include "mce_counts.h"
#include <stdlib.h>
#include <stdint.h>

#define FB_SIZE 5000 /* number of frames in the frame buffer */

/* Command actions -- these must be the same as the list in command_list.c */
#define PRM_APPLY_RECORD 0
#define PRM_APPLY_ONLY   1
#define PRM_RECORD_RCONF 2
#define PRM_RECORD_ONLY  3
#define PRM_DEFAULT_ONLY 4

/* upper 25 bits */
#define DATA_MASK 0xFFFFFF80

/* non-volatile memory */
#define MEM_VERS 1
struct memory_t {
  int version;
  time_t time;
  int last_tune;
  int last_iv;
  int squidveto;
  int used_tune;
  int sync_veto;
  int divisor;
  time_t dmesg_lookback;
  int bias_kick_val;
  int bias_kick_bias;
  int bias_kick_time;
  int bias_kick_wait;
  double bolo_filt_freq;
  double bolo_filt_bw;
  int bolo_filt_len;
  double bolo_stat_gain[N_STAT_TYPES];
  int bolo_stat_offset[N_STAT_TYPES];
  int restart_reset;
  int ref_tune;
  int tune_tries[4];
  int tune_global_tries;
  int tune_check_off;
  int ramp_max;
  int off_trans_max;
  int off_trans_wait;
  int off_trans_thresh;
  int rst_wait;
  int rst_tries;
  int ref_iv;
  int ref_iv_dark;
  int check_count[4];
  int check_range[4];
  int check_slope[4];
  int check_p2p[4];
  double p2p_abs_thresh[4];
  double p2p_rel_thresh[4];
  double slope_abs_thresh[4];
  double slope_rel_thresh[4];
  double range_abs_thresh[4];
  double range_rel_thresh[4];
  int count_thresh[4];
  double ramp_shift[4];
  int ramp_buffer[4];
  int fail_thresh[4];
  int auto_iv_kick;
  double auto_iv_kicktime;
  int auto_iv_kickwait;
  int auto_iv_start;
  int auto_iv_stop;
  int auto_iv_step;
  double auto_iv_wait;
};

extern struct memory_t memory;
extern int mem_dirty;

/* MPC globals */
extern time_t btime;
extern int stat_veto;
extern int nmce;
extern int mas_get_temp;
extern int sock;
extern int in_turnaround;
extern int cur_dm, req_dm;
extern uint16_t tuning_status;
extern int power_cycle_mce;
extern int power_cycle_cmp;
extern int send_mceparam;
extern char *uuid[4];
extern int divisor;
extern int veto;
extern int try_mount;
extern int wait_state;
extern int kill_special;
extern uint32_t iclamp[2];
extern uint32_t igain[NUM_ROW * NUM_COL];
extern int dt_going;
extern uint8_t bolo_stat_buff[N_STAT_TYPES][NUM_ROW * NUM_COL];
extern int drives_checked;
extern int data_drive[3];
extern int drive_error[4];
extern int disk_bad[4];

extern char array_id[100];
extern uint16_t bset_num;
extern struct mpc_slow_data slow_dat;
extern int slow_veto;
extern int ntes;
extern int acq_init;
extern int stat_reset;
extern int terminate;
extern int sync_dv;
extern int16_t tes[NUM_ROW * NUM_COL];
extern int rd_count;
extern int ref_biases_dark_ok;
extern int ref_biases_lite_ok;
extern uint32_t pcm_frameno;
extern int pcm_ret_dat;
extern uint32_t mce_stat[N_MCE_STAT];
extern int32_t box_temp;
extern uint8_t drive_map;
extern int num_rows, row_len, data_rate;

extern size_t frame_size;
extern int pb_last;
extern int fb_top;
extern uint32_t *frame[FB_SIZE];

/* MCE data mode definitions */
#define N_DATA_MODES 13
extern struct data_mode_def {
  int first_bit, num_bits;
  enum { coadd_first, coadd_mean, coadd_sum } coadd_how;
} data_modes[N_DATA_MODES][2];

/* mpc statūs -- try to keep these in start-up order */
enum status {
  st_idle   = 0x0000, /* Not a status bit, just a "task" */

  st_drives = 0x0001, /* The drives are ready */
  st_active = 0x0002, /* MCE ops are active */
  st_mcecom = 0x0004, /* MCE is talking */ 
  st_syncon = 0x0008, /* the sync box is active */
  st_config = 0x0010, /* MCE is configured */
  st_biased = 0x0020, /* TESes are biased */
  st_acqcnf = 0x0040, /* Acquisition is configured */
  st_retdat = 0x0080, /* MCE is returning data */
};
/* state that can be on when reconfiguring the data drives */
#define ST_DRIVE_IGNORE (st_active | st_syncon | st_mcecom)
#define STOP_TK 0x8000

extern unsigned int state;

#define MODA_SHIFT 8 /* bias to prevent modas from clashing with the states */
enum modas {
  md_none = 0,
  md_tuning, /* auto_setup in progress */
  md_iv_curve, /* normal IV curve in progress */
  md_lcloop, /* lc-looping */

  md_running, /* normal data acquisition */
  md_bstep, /* bias step (during acquisition) */
  md_bramp, /* stepped bias ramp */
  md_partial, /* partial load curve */
};
#define MODA_STRINGS "none", "tuning", "iv_curve", "lcloop", \
  "running", "bstep", "bramp", "partial"

extern enum modas moda;

/* operating goals */
enum goals { gl_pause, gl_tune, gl_iv, gl_stop, gl_lcloop, gl_cycle,
  /* goals >= gd_acq involve normal data acquisition */
  gl_acq, gl_bstep, gl_bramp, gl_partial};
#define GOAL_STRINGS "pause", "tune", "iv", "stop", "lcloop", "cycle", \
  "acq", "bstep", "bramp", "partial"

/* goal data */
struct gl_data {
  enum goals goal;

  /* general purpose registers */
  int start, stop, force;
  int step, kick, apply;
  double kicktime, kickwait, wait, total;
};
extern struct gl_data goal, new_goal;
extern int change_goal;

/* The director */
void meta(void);
void meta_goal_complete(int reconfig);

/* meta <-> task communication */
extern uint32_t meta_tk;

/* drive mapping */
#define DRIVE0_DATA0 0x0000
#define DRIVE0_DATA1 0x0001
#define DRIVE0_DATA2 0x0002
#define DRIVE0_DATA3 0x0003
#define DRIVE0_UNMAP 0x0004
#define DRIVE0_MASK  0x000F

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

/* the block update queue */
#define BLOCKQ_SIZE 100
struct block_q {
  char *c, *p;
  uint32_t d[41];
  int n, o, raw;
};
extern struct block_q blockq[BLOCKQ_SIZE];
extern int blockq_head, blockq_tail;

/* blob creator */
#define N_BLOB_DATA 5
extern int send_blob;
extern int blob_size;
extern int blob_type;
extern int new_blob_type;
extern char blob_source[1024];
extern int blob_data[N_BLOB_DATA];
extern uint16_t blob[MCE_BLOB_MAX];
extern int iv_blob_start;
extern int iv_blob_count;
void *blobber(void *dummy);

/* task handler */
void *task(void *dummy);

/* data tasklets */
enum dtask {
  dt_idle = 0, dt_setdir, dt_dsprs, dt_mcers, dt_reconfig, dt_startacq,
  dt_fakestop, dt_empty, dt_status, dt_acqcnf, dt_autosetup, dt_delacq,
  dt_ivcurve, dt_stop, dt_stopmce, dt_lcloop, dt_bstep, dt_bramp, dt_kick,
  dt_partial, dt_rstsrvo
};
#define DT_STRINGS \
  "idle", "setdir", "dsprs", "mcers", "reconfig", "startacq", \
"fakestop", "empty", "status", "acqcnf", "autosetup", "delacq", \
"ivcurve", "stop", "stopmce", "lcloop", "bstep", "bramp", "kick", \
"partial", "rstsrvo"
extern enum dtask data_tk;
extern int dt_error;
extern int comms_lost;

#define MPC_ETC_DIR "/data/mas/etc"

void *mas_data(void *dummy);
void *acquer(void *dummy);
void crash_stop(int);

/* reset the array monitoring */
void reset_array_health(void);

/* figure out tuning paths */
int tuning_filename(const char *stage, const char *file, int n, char *buffer);

/* The frame acq callback */
int frame_acq(unsigned long user_data, int frame_size, uint32_t *buffer);

/* detector types */
enum det_types {det_dead, det_frail, det_healthy};

/* cfg stuff */
int load_experiment_cfg(void);
void cfg_apply_tuning(int, int);
int cfg_get_int(const char *, int);
int cfg_set_float(const char *, int, double);
int cfg_set_int(const char *, int, int);
int cfg_set_int_cr(const char *, int, int, int);
int cfg_set_intarr(const char *name, int o, uint32_t *d, int n);
int flush_experiment_cfg(int);
int serialise_experiment_cfg(void);
void cfg_update_timing(int, int, int);
void cfg_load_template(void);
void cfg_load_dead_masks(void);
enum det_types cfg_det_type(int, int);
int cfg_frail_pid(char);

/* fake read from the MCE */
void read_param(const char *card, const char *param, int offset, uint32_t *data,
    int count);

/* frame statistics */
void update_stats(const uint32_t *curr_frame, size_t frame_size,
    uint32_t frameno);

#endif
