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
#include "mpc_proto.h"

#include <string.h>

/* desecrate the C preprocessor to extract this file's SVN revision */

/* Fun fact: GCC's CPP allows $ in identifier names */
#define $Rev (0?0 /* eat a : here */
#define $    )

const int mpc_proto_revision(void) { return $Rev$; };

#undef $Rev
#undef $
/* end preprocessor desecration */

/* compose a command for transfer to the mpc */
size_t mpc_compose_command(struct ScheduleEvent *ev, char *buffer)
{
  size_t len;
  int32_t i32;
  char *ptr;
  int i;

  if (ev->is_multi) {
    sprintf(buffer, "CMDm%3i", ev->command);
    ptr = buffer + 7;
    for (i = 0; i < mcommands[ev->t].numparams; ++i) {
      switch (mcommands[ev->t].params[i].type) {
        case 'i':
        case 'l':
          *(ptr++) = 'N';
          i32 = (int32_t)ev->ivalues + i;
          memcpy(ptr, &i32, sizeof(i32));
          ptr += sizeof(i32);
          break;
        case 'f':
        case 'd':
          *(ptr++) = 'R';
          memcpy(ptr, ev->rvalues + i, sizeof(double));
          ptr += sizeof(double);
          break;
        case 's':
          *(ptr++) = 'T';
          memcpy(ptr, ev->svalues + i, 32);
          ptr += 32;
          break;
      }
    }
    len = ptr - buffer;
  } else {
    sprintf(buffer, "CMDs%3i", ev->command);
    len = 7;
  }

  return len;
}
