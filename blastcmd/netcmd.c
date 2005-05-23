/* blastcmd: groundstation BLAST command software
 *
 * This software is copyright (C) 2005 University of Toronto
 * 
 * This file is part of blastcmd.
 * 
 * blastcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * blastcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>

#include <errno.h>
#include <netdb.h>
#include <pwd.h>
#include <signal.h>
#include <string.h>
#include <tcpd.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "netcmd.h"
#include "command_list.h"

unsigned short client_n_scommands = 0;
unsigned short client_n_mcommands = 0;
struct scom *client_scommands;
struct mcom *client_mcommands;
char client_command_list_serial[1024];

char me[1024];
char owner[1024];
char banner[1024];
int is_free = -1;
int sock;

int ReadLine(int sock, char* buffer, int bufflen)
{
  static int prebuffer_size = 0;
  static char prebuffer[2048] = "";
  int i;

  i = recv(sock, prebuffer + prebuffer_size, 2048 - prebuffer_size,
      MSG_DONTWAIT);
  
  if (i < 0) {
    if (errno == EAGAIN && prebuffer_size != 0)
      i = 0;
    else
      return i;
  }

  if (i + prebuffer_size == 0)
    return 0;

  prebuffer_size += i;

  buffer[bufflen - 1] = 0;
  for (i = 0; i < bufflen - 1; ++i) {
    if (prebuffer[i] == '\r' || prebuffer[i] == '\n' || prebuffer[i] == '\0') {
      buffer[i] = 0;
      break;
    } else
      buffer[i] = prebuffer[i];
  }

  while ((prebuffer[i] == '\r' || prebuffer[i] == '\n') && i < 2048)
    i++;

  memmove(prebuffer, prebuffer + i, 2048 - i);
  prebuffer_size -= i;

  return strlen(buffer);
}

void SetOwner(char* buffer)
{
  int i;

  buffer[1023] = 0;
  for (i = 0; i < 1023; ++i)
    if (buffer[i] == '\n' || buffer[i] == '\r')
      buffer[i] = 0;

  printf("so %s\n", buffer);

  if (strcmp(buffer, ":::free:::") == 0) {
    is_free = 1;
  } else if (strncmp(buffer, ":::conn:::", 10) == 0) {
    strcpy(owner, buffer + 10);
    is_free = 0;
  }
}

void NetCmdDrop(void)
{
  shutdown(sock, SHUT_RDWR);
  close(sock);
}

int NetCmdGetAck(int *ack, int silent)
{
  char buffer[1024] = "\0";
  int i;

  i = ReadLine(sock, buffer, 1024);

  printf("ga %s\n", buffer);

  if (i < 0) {
    if (errno == EAGAIN)
      return 0;
    else {
      perror("Unable to receive");
      *ack = 14;
      return 1;
    }
  } else if (strncmp(buffer, ":::ack:::", 9) == 0) {
    *ack = atoi(buffer + 9);
    return 1;
  } else if (strncmp(buffer, ":::limit:::", 11) == 0) {
    if (!silent)
      puts(buffer + 11);
  } else if (strncmp(buffer, ":::sent:::", 10) == 0) {
    if (!silent)
      printf("Packet: %s", buffer + 10);
  } else
    printf("%s", buffer);

  return 0;
}

void NetCmdSend(const char* buffer)
{
  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);
}

int NetCmdSubmitCommand(char t_link, char t_route, int nc, char *command[],
    int silent)
{
  int i;
  int ack;
  char buffer[1024] = {t_link, t_route, ' ', 0};

  for (i = 0; i < nc; ++i) {
    strcat(buffer, command[i]);
    strcat(buffer, " ");
  }

  if (!silent)
    printf("Submitting %s\n\n", buffer);

  strcat(buffer, "\r\n");

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  do {
    if (NetCmdGetAck(&ack, silent))
      return ack;
  } while (1);

  fprintf(stderr, "Unexpected trap in NetCmdSubmitCommand. Stop.\n");
  exit(-1);
}

int NetCmdGetCmdList(void)
{
  int i, n, c = 0;
  char buffer[1024] = "::list::\r\n";

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  strcpy(buffer, ":::rev:::");

  for (i = 0; i < strlen(buffer); ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (buffer[i] != c) {
      fprintf(stderr, "Protocol error from daemon.\n");
      exit(14);
    }
  }

  for (i = 0; i < 100; ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (i == 999 && c != '\n') {
      fprintf(stderr, "Protocol error from daemon.\n");
      exit(14);
    }

    if (c == '\n') {
      client_command_list_serial[i] = '\0';
      break;
    } else if (c != '\r')
      client_command_list_serial[i] = c;
  }

  read(sock, &client_n_scommands, sizeof(client_n_scommands));
  read(sock, &client_n_mcommands, sizeof(client_n_mcommands));

  if (client_n_scommands > 255 || client_n_mcommands > 255 ||
      client_n_scommands * client_n_mcommands == 0) {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }

  client_scommands = (struct scom*)malloc(client_n_scommands
      * sizeof(struct scom));
  client_mcommands = (struct mcom*)malloc(client_n_mcommands
      * sizeof(struct mcom));
  recv(sock, client_scommands, client_n_scommands * sizeof(struct scom),
      MSG_WAITALL);
  recv(sock, client_mcommands, client_n_mcommands * sizeof(struct mcom),
      MSG_WAITALL);

  return 0;
}

void NetCmdUpdateConn(void)
{
  char buffer[1024] = "";
  int i;

  do {
    i = ReadLine(sock, buffer, 1024);

    if (i == -1 && errno == EAGAIN) {
      break;
    } else if (i == -1) {
      perror("Error on network read");
      exit(1);
    } else if (i == 0) {
      fprintf(stderr, "Connection reset by peer.\n");
      exit(1);
    } else
      SetOwner(buffer);
  } while (1);
}

const char* NetCmdBanner(void)
{
  if (is_free)
    return "The conn is free.";
  else if (strncmp(owner, me, strlen(me)) == 0)
    return "I have the conn.";
  else {
    sprintf(banner, "%s has the conn.", owner);
    return banner;
  }
}

int NetCmdTakeConn(void)
{
  int i;
  char buffer[1024] = "::take::\r\n";

  /* don't take it if we already have it */
  if (is_free == 0 && strncmp(owner, me, strlen(me)) == 0)
    return 1;

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  if ((i = recv(sock, buffer, 1024, 0)) <= 0) {
    perror("Unable to receive");
    exit(14);
  } else if (buffer[0] != ':' || buffer[1] != ':' || buffer[2] != ':') {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }

  SetOwner(buffer);

  return (is_free == 0 && strncmp(owner, me, strlen(me)) == 0);
}

void NetCmdConnect(const char* host, int silent, int silenter)
{
  int i;
  char buffer[1024];
  struct hostent* the_host;
  struct sockaddr_in addr;
  struct passwd pw;
  struct passwd *pwptr;

  /* get remote host IP */
  the_host = gethostbyname(host);

  if (the_host == NULL) {
    fprintf(stderr, "host lookup failed for `%s': %s\n", host,
        hstrerror(h_errno));
    exit(14);
  }

  addr.sin_port = htons(SOCK_PORT);
  addr.sin_family = AF_INET;
  memcpy(&(addr.sin_addr.s_addr), the_host->h_addr, the_host->h_length);

  /* set user */
  getpwuid_r(getuid(), &pw, buffer, 1024, &pwptr);
  strcpy(me, pw.pw_name);
  strcat(me, "@");
  i = strlen(me);
  gethostname(me + i, 1023 - i);
  sprintf(me + strlen(me), ".%i", getpid());

  sprintf(buffer, "::user::%s\r\n", me);

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    perror("Unable to create socket");
    exit(14);
  }

  if (!silenter)
    printf("Connecting to %s (%s) port %i ...\n", host,
        inet_ntoa(addr.sin_addr), SOCK_PORT);

  if ((i = connect(sock, (struct sockaddr*)&addr, sizeof(addr))) == -1) {
    perror("Unable to connect to blastcmd daemon");
    exit(14);
  }

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);
  if ((i = recv(sock, buffer, 1024, 0)) <= 0) {
    perror("Unable to receive");
    exit(14);
  } else if (buffer[0] != ':' || buffer[1] != ':' || buffer[2] != ':') {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }

  SetOwner(buffer);

  if (is_free == -1) {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }

  if (!silenter)
    printf("Connected.\n");

  if (!silent) {
    if (is_free)
      printf("Conn is free.\n");
    else
      printf("%s has the conn.\n", owner);
  }
}
