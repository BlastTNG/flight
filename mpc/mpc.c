/* MPC: MCE-PCM communicator
 *
 * Copyright (c) 2013, D. V. Wiebe
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
#include "udp.h"

#include <stdio.h>
#include <unistd.h>
#include "tes.h"

#define UDP_TIMEOUT    100 /* milliseconds */
#define INIT_TIMEOUT 60000 /* milliseconds */

int main(void)
{
  int sock, port, type;
  ssize_t n;
  size_t len;

  int init = 1, init_timer = 0;
  int nmce = 0;

  int ntes = 0;
  uint16_t fset_num = 0xFFFF;
  int16_t tes[NUM_ROW * NUM_COL];

  char peer[UDP_MAXHOST];
  char data[65536];

  printf("This is MPC.\n");
  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC subsystem.");

  /* bind to the UDP port */
  sock = udp_bind_port(MPC_PORT, 1);

  /* main loop */
  for (;;) {
    struct ScheduleEvent ev;

    /* check inbound packets */
    n = udp_recv(sock, UDP_TIMEOUT, peer, &port, 65536, data);

    /* n == 0 on timeout */
    type = (n > 0) ? mpc_check_packet(n, data, peer, port) : -1;

    if (init) {
      if (init_timer <= 0) {
        /* send init packet */
        len = mpc_compose_init(nmce, data);

        /* Broadcast this to everyone */
        if (udp_bcast(sock, MCESERV_PORT, len, data) == 0)
          bputs(info, "Broadcast awake ping.\n");

        init_timer = INIT_TIMEOUT;
      } else
        init_timer -= UDP_TIMEOUT;
    }

    if (type < 0)
      continue;

    /* do something based on packet type */
    switch (type) {
      case 'C': /* command packet */
        if (mpc_decompose_command(&ev, n, data)) {
          /* command decomposition failed */
          break;
        }
        /* run the command here */
        break;
      case 'F': /* field set packet */
        if ((n = mpc_decompose_fset(&fset_num, tes, nmce, n, data)) >= 0)
          ntes = n;
        init = 0;
        break;
      default:
        bprintf(err, "Unintentionally dropping unhandled packet of type 0x%X\n",
            type);
    }
  }

  close(sock);
  return 0;
}
