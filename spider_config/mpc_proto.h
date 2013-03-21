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

#include <sys/types.h>
#include <stdint.h>
#include "command_struct.h"
#include "fset.h"
#include "tes.h"

#define MCESERV_PORT 1729
#define MPC_PORT     9271

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

size_t mpc_compose_fset(const struct fset *set, uint16_t num, char *buffer);
int mpc_decompose_fset(uint16_t *fset_num, int16_t *array, int mce, size_t len,
    const char *data);

size_t mpc_compose_init(int mce, char *buffer);
int mpc_decompose_init(size_t len, const char *data, const char *peer,
    int port);

size_t mpc_compose_slow(const struct mpc_slow_data *dat, int mce, char *buffer);
int mpc_decompose_slow(struct mpc_slow_data dat[NUM_MCE][3], int ind[NUM_MCE],
    size_t len, const char *data);

#endif
