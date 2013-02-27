/* mceserv: the MCE flight computer network server
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
#include "mcp.h"
#include "command_struct.h"
#include "mpc_proto.h"
#include "udp.h"

#include <unistd.h>
#include <string.h>

#if 0
/* reverse lookup on an unsorted integer array */
static inline int FindInt(int v, const int *a, size_t l)
{
  size_t i;
  for (i = 0; i < l; ++i)
    if (v == a[i])
      return v;
  return -1;
}

/* check a client revision number.  Also isolate the hostname */
static int GetRev(char *buffer)
{
  char *end = NULL;
  int rev;
  char *ptr = buffer + 5;

  /* find first space */
  while (*ptr && *ptr != ' ')
    ptr++;

  /* no space */
  if (!*ptr)
    return -1;

  *(ptr++) = 0;

  /* skip all the spaces */
  while (*ptr && *ptr == ' ')
    ptr++;

  if (!*ptr)
    return -1;

  /* convert revision number */
  rev = (int)strtol(ptr, &end, 10);
  if (rev <= 0)
    return -1;

  /* revision number should be ended by a \r\n */
  if (*end != '\r')
    return -1;

  return rev;
}
#endif

/* send a command if one is pending */
static int ForwardCommand(int sock)
{
  size_t len;
  char buffer[10000];
  const int cmd_idx = GETREADINDEX(CommandData.mcecmd_index);
  struct ScheduleEvent ev;

  /* all this saves us is potentially a useless memcpy */
  if (CommandData.mcecmd[cmd_idx].done || CommandData.mcecmd[cmd_idx].t == -1)
    return 0;

  memcpy(&ev, CommandData.mcecmd + cmd_idx, sizeof(ev));

  /* mitigate race conditions */
  if (ev.done || ev.t == -1)
    return 0;

  /* compose the command for transfer. */
  len = mpc_compose_command(&ev, buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, buffer) == 0) {
    bprintf(info, "Broadcast %s command #%i.\n",
        ev.is_multi ? "multi" : "single", ev.command);
  }

  /* mark as written */
  CommandData.mcecmd[cmd_idx].done = 1;

  /* indicate something has been sent */
  return 1;
}

/* main routine */
void *mceserv(void *unused)
{
  const int proto_rev = mpc_proto_revision();
  int sock;

  nameThread("MCE");
  bprintf(startup, "Startup");
  bprintf(info, "Protocol revision: %i", proto_rev);

  sock = udp_bind_port(MCESERV_PORT, 1);

  if (sock == -1)
    bprintf(tfatal, "Unable to bind to port");

  for (;;) {
    /* broadcast MCE commands */
    ForwardCommand(sock);
    usleep(10000);
  }

  return NULL;
};
