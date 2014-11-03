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
#include <errno.h>        /* ANSI C library errors (errno) */
#include <netinet/in.h>   /* For IPPROTO_TCP */
#include <netinet/tcp.h>  /* For TCP_NODELAY */
#include <netdb.h>        /* DNS queries (gethostbyname, hstrerror, h_errno) */
#include <string.h>       /* ANSI C strings (strcat, strdup, &c.)  */
#include <unistd.h>       /* UNIX std library (read, write, close, sleep) */

#include "blast.h"
#include "quenya.h"

#define QUENDI_PORT 44144

int GetServerResponse(int sock, char* buffer)
{
  static char extra[2000] = "";
  char cbuf[4000];
  char* ptr;
  int n, overrun;
  char* response;

//  if (buffer != NULL) printf("<-- %s\n", buffer);

  strcpy(cbuf, extra);
  response = cbuf + strlen(cbuf);
  
  if (strchr(cbuf, '\n') == NULL) {
    n = read(sock, response, 2000);

    if (n < 0)
      berror(fatal, "Read error");

    if (n == 0 && errno != EAGAIN) {
      bprintf(err, "Unexpected server disconnect.\n");
      return -3;
    }

    response[1999] = 0;
  } else
    n = 0;

  for (ptr = cbuf; *ptr; ++ptr)
    if (*ptr == '\r') {
      *ptr = '\0';
      overrun = n - (ptr - response) - 2;
      if (overrun > 0) {
        memcpy(extra, ptr + 2, overrun);
        extra[n - (ptr - response) - 2] = 0;
      } else
        extra[0] = 0;

      break;
    }

  if (buffer != NULL)
    strcpy(buffer, cbuf);

  if (cbuf[3] != ' ')
    return -2;

  n = atoi(cbuf);

//  printf("--> %s\n", cbuf);

  if (n == 0)
    bprintf(fatal, "Indecypherable server response: %s\n", cbuf);

  if (buffer != NULL)
    strcpy(buffer, cbuf + 4);

  return n;
}

int MakeSock(void)
{
  int sock, n;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    berror(fatal, "socket");

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0)
    berror(fatal, "setsockopt");

  n = 1;
  if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &n, sizeof(n)) != 0)
    berror(fatal, "setsockopt");

  return sock;
}

int OpenDataPort(int csock, const struct sockaddr_in *rc_addr, int* dsock)
{
  char buffer[2000];
  int n;
  char *ptr1 = NULL, *ptr2;
  struct sockaddr_in dp_addr;

  *dsock = MakeSock();
  strcpy(buffer, "OPEN\r\n");
  if (write(csock, buffer, strlen(buffer)) < 0)
    berror(err, "Failed to write to socket");
  switch (n = GetServerResponse(csock, buffer)) {
    case -3:
      return -1;
    case QUENYA_RESPONSE_LISTENING:
      for (ptr1 = buffer; *ptr1 != '@'; ++ptr1);
      *(ptr1++) = 0;
      for (ptr2 = ptr1; *ptr2 != ':'; ++ptr2);
      *(ptr2++) = 0;
      *(strchr(ptr2, ' ')) = 0;

      memcpy(&dp_addr, rc_addr, sizeof(dp_addr));
      dp_addr.sin_family = AF_INET;
      dp_addr.sin_port = htons(atoi(ptr2));
//      inet_aton(ptr1, &dp_addr.sin_addr);
      sleep(1);

      if ((n = connect(*dsock, (struct sockaddr*)&dp_addr, sizeof(dp_addr)))
          != 0)
        berror(fatal, "d-Connect failed");
      break;
    default:
      bprintf(fatal, "Unexpected response from server after OPEN: %i\n", n);
      return -1;
  }

  switch (n = GetServerResponse(csock, buffer)) {
    case -3: /* disconnect */
      return -1;
    case QUENYA_RESPONSE_PORT_OPENED:
      break;
    default:
      bprintf(err, "Unexpected response from server after OPEN/2: %i\n", n);
      return -1;
  }

  return 0;
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
