/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004 D. V. Wiebe
 * 
 * This file is part of defile.
 * 
 * defile is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * defile is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with defile; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>

#include "channels.h"
#include "defile.h"
#include "blast.h"

int MakeSock(void)
{
  int sock, n;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    berror(fatal, "socket");

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0)
    berror(fatal, "setsockopt");

  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0)
    berror(fatal, "setsockopt");

  return sock;
}

int GetServerResponse(char* buffer)
{
  char response[2000];
  char* ptr;
  int n;
  
  n = read(rc.csock, response, 2000);
  if (n < 0)
    return -1;

  if (n == 0)
    return -3;

  response[1999] = 0;

  for (ptr = response; *ptr; ++ptr)
    if (*ptr == '\r') {
      *ptr = '\0';
      break;
    }

  if (buffer != NULL)
    strcpy(buffer, response);

  if (response[3] != ' ')
    return -2;

  n = atoi(response);

  printf("%s\n", response);

  if (n == 0)
    return -2;

  if (buffer != NULL)
    strcpy(buffer, response + 4);

  return n;
}

void OpenDataPort(void)
{
  char buffer[2000];
  int n;
  char *ptr1 = NULL, *ptr2;
  struct sockaddr_in dp_addr;

  rc.dsock = MakeSock();
  strcpy(buffer, "OPEN\r\n");
  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 123:
      for (ptr1 = buffer; *ptr1 != '@'; ++ptr1);
      *(ptr1++) = 0;
      for (ptr2 = ptr1; *ptr2 != ':'; ++ptr2);
      *(ptr2++) = 0;
      *(strchr(ptr2, ' ')) = 0;

      dp_addr.sin_family = AF_INET;
      dp_addr.sin_port = htons(atoi(ptr2));
      inet_aton(ptr1, &dp_addr.sin_addr);
      sleep(1);

      printf("Connecting to %s:%i...\n", inet_ntoa(dp_addr.sin_addr),
          ntohs(dp_addr.sin_port));
      if ((n = connect(rc.dsock, (struct sockaddr*)&dp_addr, sizeof(dp_addr)))
          != 0)
        berror(fatal, "Connect failed");
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 222:
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }
}

void InitClient(void)
{
  char buffer[2000];
  int n;
  char *ptr1 = NULL, *ptr2;
  char *hostname = NULL;
  rc.csock = MakeSock();
  FILE* stream;

  printf("Connecting to %s:%i...\n", inet_ntoa(rc.addr.sin_addr),
      ntohs(rc.addr.sin_port));

  if ((n = connect(rc.csock, (struct sockaddr*)&rc.addr, sizeof(rc.addr))) != 0)
    berror(fatal, "Connect failed");

  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 220:
      for (ptr1 = buffer; *ptr1 != ' '; ++ptr1);
      *(ptr1++) = 0;
      for (ptr2 = ptr1; *ptr2 != '/'; ++ptr2);
      *(ptr2++) = 0;
      *(strchr(ptr2, ' ')) = 0;

      bprintf(info, "Connected to %s on %s speaking quenya version %s.\n",
          ptr1, buffer, ptr2);

      hostname = strdup(buffer);
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  strcpy(buffer, "IDEN defile\r\n");
  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 230:
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  OpenDataPort();

  strcpy(buffer, "QNOW\r\n");
  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 251:
      for (ptr1 = buffer; *ptr1 != ':'; ++ptr1);
      *(ptr1++) = 0;
      *(strchr(ptr1, ' ')) = 0;
      rc.chunk = bstrdup(fatal, ptr1);
      rc.resume_at = atoi(buffer);
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  if (rc.output_dirfile != NULL)
    strncpy(rc.dirfile, rc.output_dirfile, FILENAME_LEN);
  else
    GetDirFile(rc.dirfile, ptr1, rc.dest_dir, rc.resume_at);

  rc.chunk = (char*)balloc(fatal, FILENAME_LEN);
  sprintf(rc.chunk, "%s:%s", hostname, ptr1);

  strcpy(buffer, "SPEC\r\n");
  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 150:
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  if ((stream = fdopen(rc.dsock, "w+")) == NULL)
    berror(fatal, "fdopen");

  ReadSpecificationFile(stream);

  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 252:
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  strcpy(buffer, "CLOS\r\n");
  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(buffer)) {
    case -3:
      bprintf(fatal, "Unexpected server disconnect.\n");
    case -2:
      bprintf(fatal, "Indecypherable server response: %s\n", buffer);
    case -1:
      berror(fatal, "Read error");
    case 250:
      break;
    default:
      bprintf(fatal, "Unexpected response from server: %i\n", n);
  }

  fclose(stream);

  MakeAddressLookups();
  bprintf(info, "Frame size: %i bytes\n", DiskFrameSize);

  OpenDataPort();

  bfree(fatal, hostname);
}

void QuenyaClient(void)
{
}
