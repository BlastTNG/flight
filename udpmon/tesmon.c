/* udpmon: monitors a udp port.
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

#include "udp.h"
#include "blast.h"
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>

/* handle signals */
static int done = 0;
static void handle(int sig)
{
  done = 1;
}

int main(int argc, const char **argv)
{
  sigset_t signals;
  struct sigaction action;
  int port, remport, sock;
  ssize_t n;

  char peer[UDP_MAXHOST];
  char data[65536];

  /* BUOS set up */
  buos_use_stdio();

  if (argc < 2) {
    fprintf(stderr, "Usage:\n   udpmon <port>\n\n");
    return 1;
  }

  port = atoi(argv[1]);

  if (port < 0 || port > 65536) {
    fprintf(stderr, "Bad port: %i\n", port);
    return 1;
  }

  /* handle signals */
  sigemptyset(&signals);
  sigaddset(&signals, SIGHUP);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);

  action.sa_handler = handle;
  action.sa_mask = signals;
  action.sa_flags = 0;
  sigaction(SIGHUP, &action, NULL);
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  /* bind */
  sock = udp_bind_port(port, 0);

  if (sock < 1)
    return 1;

  /* service loop */
  int i, last_no[6] = {-1, -1, -1, -1, -1, -1};
  while (!done) {
    n = udp_recv(sock, 0, peer, &remport, 65536, data);
    if (n > 3 && data[2] == 'T') {
      int mce = data[3];
      int nf = *(int16_t*)(data + 6);
      for (i = 0; i < nf; ++i) {
        int this_no = *(int32_t*)(data + 8 + i * sizeof(uint32_t));
        if (last_no[mce] != -1 && this_no - 1 != last_no[mce])
          printf("%i -> %i\n", last_no[mce], this_no);
        last_no[mce] = this_no;
        if ((this_no % 1000) == 0)
          printf("[X%i %i]\n", mce + 1, this_no);
      }
    }
  }

  /* quitsies */
  close(sock);
  printf("\nReleased udp/%i\n", port);
  return 0;
}
