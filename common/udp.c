/* udp: generic UDP stuff for flight code
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* create and bind a socket to a UDP port on the local interfaces */
int udp_bind_port(int port, int bcast)
{
  int r, sock;

  char service[20];

  struct addrinfo *res, *addr;
  struct addrinfo hints = {
    .ai_flags = AI_PASSIVE | AI_NUMERICSERV,
    .ai_family = AF_INET,
    .ai_socktype = SOCK_DGRAM,
    .ai_protocol = 0,
    .ai_addrlen = 0,
    .ai_addr = NULL,
    .ai_canonname = NULL,
    .ai_next = NULL
  };

  /* deal with the weirdness */
  sprintf(service, "%i", port);

  /* fill res */
  r = getaddrinfo(NULL, service, &hints, &res);
  if (r) {
    bprintf(err, "Service look-up failed for udp/%i: %s\n", port,
        gai_strerror(r));
    return -1;
  }

  /* try to bind a socket somewhere */
  for (addr = res; addr; addr = addr->ai_next) {
    sock = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (sock >= 0) {
      if (bind(sock, addr->ai_addr, addr->ai_addrlen) == 0) {
        /* turn on broadcast, if requested */
        if (bcast) {
          int one = 1;
          r = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
          if (r) {
            berror(err, "setsockopt");
            close(sock);
            return -1;
          }
        }

        /* success */
#if 0
        bprintf(info, bcast ? "Bound to udp/%i (broadcast enabled)" :
            "Bound to udp/%i", port);
#endif
        freeaddrinfo(res);
        return sock;
      }
      close(sock);
    }
  }

  freeaddrinfo(res);
  berror(err, "Unable to bind to udp/%i", port);

  return -1;
}

/* Wait (with optional timeout) for a datagram on a bound port. */
ssize_t udp_recv(int sock, int msec, char *peer, int *port, size_t len,
    char *data)
{
  char local_peer[UDP_MAXHOST];
  char service[32];
  char *ptr = peer ? peer : local_peer;

  struct sockaddr addr;
  socklen_t addr_len = sizeof(addr);
  ssize_t n;

  if (msec > 0) {
    /* poll */
    int r;
    struct pollfd fds = { .fd = sock, .events = POLLIN, .revents = 0};
    r = poll(&fds, 1, msec);
    if (r < 0) {
      berror(err, "poll");
      return -1;
    } else if (r == 0) {
      return 0; /* timeout */
    }
  }

  n = recvfrom(sock, data, len, 0, &addr, &addr_len);

  if (n < 0) {
    berror(err, "recvfrom");
    return -1;
  }

  /* caller wants to know something about the sender */
  if (peer || port) {
    if (getnameinfo((struct sockaddr*)&addr, addr_len, ptr, UDP_MAXHOST,
          service, 32, NI_NUMERICSERV))
    {
      /* return ""/-1 on error */
      if (peer)
        peer[0] = 0;
      if (port)
        *port = -1;
    }
    if (port)
      *port = atoi(service);
  }

  return n;
}

/* broadcast a datagram */
int udp_bcast(int sock, int port, size_t len, const char *data, int veto)
{
  struct sockaddr_in addr;

  if (veto)
    return 0;

  memset(&addr, 0, sizeof(addr));

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

  if (sendto(sock, data, len, 0, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    berror(err, "broadcast error");
    return -1;
  }

  return 0;
}
