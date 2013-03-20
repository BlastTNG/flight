/* pcm: the Spider master control program
 *
 * mceserv.c: the MCE flight computer network server
 *
 * This software is copyright (C) 2012-2013 D. V. Wiebe
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
#include "mceserv.h"
#include "fset.h"
#include "mcp.h"
#include "command_struct.h"
#include "mpc_proto.h"
#include "udp.h"

#include <unistd.h>
#include <string.h>
#include <stdlib.h>

/* semeuphoria */
int send_fset = 0; /* field sets have changed and need re-sending */

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

static void ForwardFSet(int sock)
{
  char buffer[10000];
  size_t len;
  int num = CommandData.fset_num;
  struct fset set = {0, NULL};

  send_fset = 0;
  
  set.n = CommandData.fset.n;
  if (set.n > 0) {
    set.f = malloc(sizeof(struct fset_item) * set.n);
    memcpy(set.f, CommandData.fset.f, set.n * sizeof(struct fset_item));
  }

  /* double check -- on error, reraise the semaphore and return to try again */
  if (CommandData.fset_num != num || CommandData.fset.n != set.n) {
    send_fset = 1;
    goto FWD_FSET_DONE;
  }

  /* compose the packet */
  len = mpc_compose_fset(&set, buffer);

  /* Broadcast this to everyone */
  if (udp_bcast(sock, MPC_PORT, len, buffer) == 0)
    bprintf(info, "Broadcast FSET%03i\n", num);
  else
    send_fset = 1;

FWD_FSET_DONE:
  free(set.f);
}

/* main routine */
void *mceserv(void *unused)
{
  int sock;

  nameThread("MCE");
  bprintf(startup, "Startup");

  /* initialise the MPC protocol suite */
  if (mpc_init())
    bprintf(tfatal, "Unable to initialise MPC protocol subsystem.");

  sock = udp_bind_port(MCESERV_PORT, 1);

  if (sock == -1)
    bprintf(tfatal, "Unable to bind to port");

  for (;;) {
    /* broadcast MCE commands */
    ForwardCommand(sock);

    /* broadcast the field set, when necessary */
    if (send_fset)
      ForwardFSet(sock);
    usleep(10000);
  }

  return NULL;
};
