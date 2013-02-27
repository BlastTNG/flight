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
#include "blast.h"
#include "mpc_proto.h"

#include <string.h>

static int mpc_cmd_rev = -1, mpc_proto_rev = -1;

/* desecrate the C preprocessor to extract this file's SVN revision */

/* Fun fact: GCC's CPP allows $ in identifier names */
#define $Rev (0?0 /* eat a : here */
#define $    )

static const inline int mpc_proto_revision(void) { return $Rev$; };

#undef $Rev
#undef $
/* end preprocessor desecration */

/* initialisation routine: sets up the environment; returns non-zero on error */
int mpc_init(void)
{
  mpc_proto_rev = mpc_proto_revision();
  mpc_cmd_rev = command_list_serial_as_int();
  if (mpc_cmd_rev < 0) {
    bputs(err, "MPC: Unable to parse the command list revision.");
    return -1;
  }

  bprintf(info, "MPC: Protocol revision: %i/%i\n", mpc_proto_rev, mpc_cmd_rev);
  return 0;
}

/* compose a command for transfer to the mpc
 *
 * Command packet looks like:
 *
 * RRCTZZNN...
 *
 * where
 * 
 * R = 16-bit protocol revision
 * C = 'C', indicating command packet
 * T = 'm' or 's' indicating single or multiword command
 * Z = 16-bit command list revision
 * N = 16-bit command number
 *
 * followed by command parameters, if any.  Like the rest of the protocol all
 * numbers a little-endian */
size_t mpc_compose_command(struct ScheduleEvent *ev, char *buffer)
{
  size_t len = 2 + 1 + 2 + 2 + 1; /* 16-bit protocol revision + 'C' + 16-bit
                                     command list revision + 16-bit command
                                     number + 'm'/'s' */
  int32_t i32;
  int16_t i16 = mpc_proto_rev;
  char *ptr;
  int i;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
  buffer[2] = 'C'; /* command packet */
  buffer[3] = ev->is_multi ? 'm' : 's'; /* multi/single command */
  i16 = mpc_cmd_rev;
  memcpy(buffer + 4, &i16, sizeof(i16)); /* 16-bit command list revision */
  i16 = ev->command;
  memcpy(buffer + 6, &i16, sizeof(i16)); /* 16-bit command number */
  if (ev->is_multi) {
    ptr = buffer + 8;
    for (i = 0; i < mcommands[ev->t].numparams; ++i) {
      switch (mcommands[ev->t].params[i].type) {
        case 'i':
        case 'l':
          *(ptr++) = 'N'; /* 32-bit integer */
          i32 = (int32_t)ev->ivalues + i;
          memcpy(ptr, &i32, sizeof(i32));
          ptr += sizeof(i32);
          break;
        case 'f':
        case 'd':
          *(ptr++) = 'R'; /* 4-bit double */
          memcpy(ptr, ev->rvalues + i, sizeof(double));
          ptr += sizeof(double);
          break;
        case 's':
          *(ptr++) = 'T'; /* 32-byte NUL-terminated string */
          memcpy(ptr, ev->svalues + i, 32);
          ptr += 32;
          break;
      }
    }
    len = ptr - buffer;
  }

  return len;
}
