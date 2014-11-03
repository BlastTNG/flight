/* interloquendi: copies a mcp frame file to a TCP port
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of interloquendi.
 * 
 * interloquendi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * interloquendi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with interloquendi; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "quendiclient.h"
#include "quenya.h"
#include "blast.h"

/* In interloquendi.c */
int MakeListener(int);

int InitRendezvous(const char* host, int port, const char* masq_in)
{
  char buffer[2000];
  struct sockaddr_in addr;
  const char* herr;
  int n, sock, rsock;
  char *ptr1 = NULL, *ptr2;
  char masq[2000];
  struct hostent* thishost;
  socklen_t addrlen = sizeof(addr);

  /* No rendezvous host means no rendezvousing */
  if (host[0] == '\0')
    return 0;

  sock = MakeSock();

  if ((herr = ResolveHost(host, &addr, 0)) != NULL) 
    bprintf(fatal, "Unable to resolve upstream rendezvous server: %s", herr);

  bprintf(info, "Rendezvousing with %s:%i...\n", inet_ntoa(addr.sin_addr),
      ntohs(addr.sin_port));

  if ((n = connect(sock, (struct sockaddr*)&addr, sizeof(addr))) != 0)
    berror(fatal, "Rendezvous failed");

  if (masq_in[0] == '\0') {
    getsockname(sock, (struct sockaddr*)&addr, &addrlen);
    thishost = gethostbyaddr((const char*)&addr.sin_addr, sizeof(addr.sin_addr),
        AF_INET);
    if (thishost == NULL && h_errno) {
      bprintf(warning, "gethostbyaddr: %s", hstrerror(h_errno));
      thishost = NULL;
    }
    strcpy(masq, thishost ? thishost->h_name : inet_ntoa(addr.sin_addr));
  }

  n = 0;
  for (ptr2 = masq; *ptr2 != '\0'; ++ptr2)
    if (*ptr2 == ':') {
      n = 1;
      break;
    }

  if (!n)
    sprintf(ptr2, ":%i", port);

  switch(n = GetServerResponse(sock, buffer)) {
    case -3:
      bprintf(fatal, "Unexpected disconnect by upstream server.\n");
    case QUENYA_RESPONSE_SERVICE_READY:
      for (ptr1 = buffer; *ptr1 != ' '; ++ptr1);
      *(ptr1++) = 0;
      for (ptr2 = ptr1; *ptr2 != '/'; ++ptr2);
      *(ptr2++) = 0;
      *(strchr(ptr2, ' ')) = 0;

      bprintf(info, "Connected to %s on %s speaking quenya version %s.\n",
          ptr1, buffer, ptr2);
      break;
    default:
      bprintf(fatal, "Unexpected response from server on connect: %i\n", n);
  }

  strcpy(buffer, "IDEN interloquendi\r\n");
  if (write(sock, buffer, strlen(buffer)) < 0)
    berror(err, "Failed to write to socket");
  switch (n = GetServerResponse(sock, buffer)) {
    case -3:
      bprintf(fatal, "Unexpected disconnect by upstream server.\n");
    case QUENYA_RESPONSE_ACCESS_GRANTED:
      break;
    default:
      bprintf(fatal, "Unexpected response from server after IDEN: %i\n", n);
  }

  /* Open Rendezvous Port */
  rsock = MakeListener(port);

  /* Negotiate Rendezvous */
  sprintf(buffer, "RDVS %s\r\n", masq);
  if (write(sock, buffer, strlen(buffer)) < 0)
    berror(err, "Failed to write to socket");
  switch (n = GetServerResponse(sock, buffer)) {
    case -3:
      bprintf(fatal, "Unexpected disconnect by upstream server.\n");
    case QUENYA_RESPONSE_OPEN_ERROR:
      bprintf(fatal, "Unable to establish rendezvous with upstream server.\n");
    case QUENYA_RESPONSE_PORT_OPENED:
      break;
    default:
      bprintf(fatal, "Unexpected response from server after RDVS: %i\n", n);
  }

  return 0;
}
