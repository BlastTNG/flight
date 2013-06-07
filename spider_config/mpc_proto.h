/* MCP: the MPC communication protocol
 *
 * Copyright (c) 2012-2013, D. V. Wiebe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * This software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. In no event shall the copyright holder or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */

#ifndef MPC_PROTO_H
#define MPC_PROTO_H

#include "command_struct.h" /* for ScheduleEvent */
#include "tes.h" /* for NUM_MCE */

#include <sys/types.h>
#include <stdint.h>

#define MCESERV_PORT 1729
#define MPC_PORT     9271

/* the MPC epoch */
#define MPC_EPOCH 1363903385

/* power states */
#define MPCPROTO_POWER_NOP -1
#define MPCPROTO_POWER_ON   0
#define MPCPROTO_POWER_OFF  1
#define MPCPROTO_POWER_CYC  2

/* the MPC slow data structure */
struct mpc_slow_data {
  uint32_t time; /* system time */

  uint16_t df; /* disk free in units of 2**24 bytes (16 MB) */
  uint16_t data_mode; /* MCE data mode */
};

int mpc_init(void);
int mpc_check_packet(size_t len, const char *data, const char *peer, int port);

size_t mpc_compose_command(struct ScheduleEvent *, char *);
int mpc_decompose_command(struct ScheduleEvent *ev, size_t len,
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

size_t mpc_compose_tes(const uint16_t *data, uint32_t framenum,
    uint16_t bset_num, int nmce, int ntes, const int16_t *tesind, char *buffer);
int mpc_decompose_tes(uint32_t *frameno, uint16_t *tes_data, size_t len,
    const char *data, uint16_t bset_num, int nm[NUM_MCE], int *bad_bset_count,
    const char *peer, int port);

size_t mpc_compose_pcmreq(int nmce, int power_cycle, char *buffer);
int mpc_decompose_pcmreq(int *power_cycle, size_t len, const char *data,
    const char *peer, int port);

size_t mpc_compose_notice(int mce1_power, int mce2_power, int mce3_power,
    int turnaround, char data_mode_bits[13][2][2], char *buffer);
int mce_decompose_notice(int nmce, const char **data_mode_bits, int *turnaround,
    int *power_on, size_t len, const char *data, const char *peer, int port);

#endif
