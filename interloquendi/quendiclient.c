/* quendiclient: quendi client routines
 *
 * This software is copyright (C) 2004-2005 D. V. Wiebe
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>       /* ANSI C std library (atoi) */
#include <arpa/inet.h>    /* IP4 specification (inet_aton, inet_ntoa) */
#include <netinet/tcp.h>  /* TCP specification (SOL_TCP, TCP_NODELAY) */
#include <netdb.h>        /* DNS queries (gethostbyname, hstrerror, h_errno) */
#include <string.h>       /* ANSI C strings (strcat, strdup, &c.)  */

#include "blast.h"

#define QUENDI_PORT 44144

int MakeSock(void)
{
  int sock, n;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    berror(fatal, "socket");

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0)
    berror(fatal, "setsockopt");

  n = 1;
  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0)
    berror(fatal, "setsockopt");

  return sock;
}

const char* ResolveHost(const char* host, struct sockaddr_in* addr, int forced)
{
  struct hostent* the_host;
  char* ptr;

  if ((ptr = strchr(host, ':')) != NULL) {
    if ((addr->sin_port = htons(atoi(ptr + 1))) == htons(0))
      addr->sin_port = htons(QUENDI_PORT);
    *ptr = '\0';
  } else
    addr->sin_port = htons(QUENDI_PORT);

  the_host = gethostbyname(host);

  if (the_host == NULL) {
    if (forced)
      bprintf(fatal, "host lookup failed: %s\n", hstrerror(h_errno));

    return hstrerror(h_errno);
  }

  addr->sin_family = AF_INET;
  memcpy(&(addr->sin_addr.s_addr), the_host->h_addr, the_host->h_length);

  return NULL;
}
