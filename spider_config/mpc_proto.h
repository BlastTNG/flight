/* MCP: the MPC communication protocol
 *
 * Copyright (C) 2012-2013, D. V. Wiebe
 * All rights reserved.
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 */

#ifndef MPC_PROTO_H
#define MPC_PROTO_H

#include "mcp_sched.h" /* for ScheduleEvent */
#include "tes.h" /* for NUM_MCE */

#include <sys/types.h>
#include <stdint.h>

#define MCESERV_PORT 1729
#define MPC_PORT     9271

/* the MPC epoch */
#define MPC_EPOCH 1376000000 /* 2013-08-08 22:13:20 UTC */

/* power states */
#define MPCPROTO_POWER_NOP -1
#define MPCPROTO_POWER_ON   0
#define MPCPROTO_POWER_OFF  1
#define MPCPROTO_POWER_CYC  2

/* This make a very large UDP packet.... */
#define MCE_BLOB_MAX 20000 /* in WORDS */

/* the MPC slow data structure */
struct mpc_slow_data {
  uint32_t time; /* system time */

  uint16_t df[4]; /* data disk free in units of 2**24 bytes (16 MB) */
  uint16_t state; /* running state */
  uint16_t goal; /* operating goal */
  uint16_t task; /* current high-level task */
  uint16_t dtask; /* data tasklet */
  uint16_t t_mcc; /* mcc temp */
  uint16_t t_mce; /* MCE box temp */
  uint16_t drive_map; /* current drive map */
  uint16_t dead_count; /* count of dead and frail pixels */
  uint16_t last_tune; /* last tuning number */
  uint16_t last_iv; /* last iv curve number */
  uint16_t tile_heater; /* current tile heater level (in counts) */
  uint16_t used_tune; /* the last tuning applied */
  uint16_t sync_veto; /* CC forced into internal mode */
  uint16_t clamp_count; /* count of clamped detectors */
  uint16_t ramp_count; /* count of ramping detectors */
  uint16_t ref_tune; /* reference tuning */
  uint16_t tune_stat; /* tuning status */
  uint16_t off_trans; /* count of off-transition detectors */
  uint16_t uptime; /* time since program start (in units of 40 seconds) */
};

extern int16_t mpc_proto_rev;
extern int mpc_cmd_rev;

int mpc_init(void);
int mpc_check_packet(size_t len, const char *data, const char *peer, int port);

size_t mpc_compose_command(struct ScheduleEvent *, char *);
int mpc_decompose_command(struct ScheduleEvent *ev, int mce, size_t len,
    const char *data);

size_t mpc_compose_bset(const int16_t *set, int set_len, uint16_t num,
    char *buffer);
int mpc_decompose_bset(uint16_t *bset_num, int16_t *set, int mce, size_t len,
    const char *data);

size_t mpc_compose_init(int mce, char *buffer);
int mpc_decompose_init(size_t len, const char *data, const char *peer,
    int port);

size_t mpc_compose_slow(const struct mpc_slow_data *dat, int mce, char *buffer);
int mpc_decompose_slow(struct mpc_slow_data dat[NUM_MCE][3], int ind[NUM_MCE],
    size_t len, const char *data, const char *peer, int port);

size_t mpc_compose_tes(const uint16_t *data, const uint32_t *framenum,
    uint16_t bset_num, int16_t nf, int sync, int nmce, int ntes,
    const int16_t *tesind, char *buffer);
int mpc_decompose_tes(int *nf, uint32_t *frameno, uint16_t *tes_data,
    size_t len, const char *data, uint16_t bset_num, int set_len[NUM_MCE],
    int *bad_bset_count, const char *peer, int port);

size_t mpc_compose_pcmreq(int nmce, int power_cycle, char *buffer);
int mpc_decompose_pcmreq(int *power_cycle, size_t len, const char *data,
    const char *peer, int port);

size_t mpc_compose_notice(int divisor, int turnaround, int request_ssdata,
    int data_mode, int row_len, int num_rows, int data_rate, uint8_t squidveto,
    double bolo_filt_freq, double bolo_filt_bw, uint16_t bolo_filt_len,
    char data_mode_bits[2][2], char *buffer);
int mpc_decompose_notice(int nmce, const char **data_mode_bits, int *turnaround,
    int *divisor, int *ssdata_req, int *data_mode, int *row_len, int *num_rows,
    int *data_rate, int *squidveto, double *bolo_filt_freq,
    double *bolo_filt_bw, uint16_t *bolo_filt_len, size_t len, const char *data,
    const char *peer, int port);

size_t mpc_compose_param(const uint32_t *stat, int nmce, char *buffer);
int mpc_decompose_param(uint32_t *stat, size_t len, const char *data,
    const char *peer, int port);

size_t mpc_compose_synop(const uint8_t *data, int nmce, char *buffer);
int mpc_decompose_synop(uint8_t *synop, size_t len, const char *data,
    const char *peer, int port);

size_t mpc_compose_gpdata(uint16_t type, uint16_t len, const uint16_t *data,
    int nmce, char *buffer);
ssize_t mpc_decompose_gpdata(uint16_t *serial, size_t len, const char *data,
    const char *peer, int port);

#endif
