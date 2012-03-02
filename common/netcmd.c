/* cow: groundstation BLAST command software
 *
 * This software is copyright (C) 2005 University of Toronto
 * Parts of this software are copyright 2010 Matthew Truch
 * 
 * This file is part of cow.
 * 
 * cow is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * cow is distributed in the hope that it will be useful,
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
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
#include <pwd.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "netcmd.h"

unsigned short client_n_scommands = 0;
unsigned short client_n_mcommands = 0;
struct scom *client_scommands;
struct mcom *client_mcommands;
char client_command_list_serial[1024];

int client_n_groups = 0;
char **client_group_names;

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
  } else if (i == 0)
    return 0;

  prebuffer_size += i;

  buffer[bufflen - 1] = '\0';
  for (i = 0; i < bufflen - 1; ++i) {
    if (prebuffer[i] == '\r' || prebuffer[i] == '\n' || prebuffer[i] == '\0') {
      buffer[i] = '\0';
      break;
    } else {
      buffer[i] = prebuffer[i];
    }
  }

  while ((prebuffer[i] == '\r' || prebuffer[i] == '\n') && i < 2048)
    i++;

  memmove(prebuffer, prebuffer + i, 2048 - i);
  prebuffer_size -= i;

  if (prebuffer[0] == '\0')
    prebuffer_size = 0;

  return strlen(buffer);
}

int SetOwner(char* buffer)
{
  int i;

  buffer[1023] = 0;
  for (i = 0; i < 1023; ++i)
    if (buffer[i] == '\n' || buffer[i] == '\r')
      buffer[i] = 0;

  if (strcmp(buffer, ":::free:::") == 0) {
    is_free = 1;
  } else if (strncmp(buffer, ":::conn:::", 10) == 0) {
    strcpy(owner, buffer + 10);
    is_free = 0;
  } else if (strcmp(buffer, ":::nope:::") == 0) {
    fprintf(stderr, "Connexion refused from this host.\n");
    return -16;
  }
  return 0;
}

void NetCmdDrop(void)
{
  shutdown(sock, SHUT_RDWR);
  close(sock);
}

/* Does all non-blocking receiving.                           *
 * Returns integer: lower byte is which command it received.  *
 *                  upper bytes are optional status.          */
int NetCmdReceive(int silent)
{
  char buffer[1024] = "\0";
  int i;

  i = ReadLine(sock, buffer, 1024);
  buffer[1023] = '\0';

  if (i < 0) {
    if (errno == EAGAIN)
      return CMD_NONE;
    else {
      perror("Unable to receive");
      return CMD_ERRR + (14 << 8);
    }
  } else if (strncmp(buffer, ":::ack:::", 9) == 0) {
    return CMD_BCMD + (atoi(buffer + 9) << 8);
  } else if (strncmp(buffer, ":::limit:::", 11) == 0) {
    if (!silent)
      puts(buffer + 11);
    return CMD_LIMT;
  } else if (strncmp(buffer, ":::sent:::", 10) == 0) {
    if (!silent)
      printf("Packet: %s\n", buffer + 10);
    return CMD_SENT;
  } else if (strncmp(buffer, ":::pong:::", 10) == 0) {
    if (!silent)
      printf("Pong received: %s\n", buffer);
    return CMD_PING;
  } else if (strncmp(buffer, ":::slink:::", 11) == 0) {
    if (!silent)
      printf("Slinking: %s\n", buffer);
    return CMD_LURK;
  } else if (strncmp(buffer, ":::sender:::", 12) == 0) {
    if (!silent)
      printf("Sender received: %s\n", buffer);
    return CMD_LURK;
  } else if (strncmp(buffer, ":::cmd:::", 9) == 0) {
    if (!silent)
      printf("Sent Command received: %s\n", buffer);
    return CMD_LURK;
  } else if (strncmp(buffer, ":::rep:::", 9) == 0) {
    if (!silent)
      printf("Sent Command Response received: %s\n", buffer);
    return CMD_LURK + (atoi(buffer + 9) << 8);
  } else if (strncmp(buffer, ":::free:::", 10) == 0) {
    if (!silent)
      printf("Free received: %s\n", buffer);
    SetOwner(buffer);
    return CMD_CONN;
  } else if (strncmp(buffer, ":::conn:::", 10) == 0) {
    if (!silent)
      printf("Conn received: %s\n", buffer);
    SetOwner(buffer);
    return CMD_CONN;
  } else {
    buffer[1023] = '\0';
    printf("Unknown Reponse: %s\n", buffer);
  }

  return 0;
}

void NetCmdSend(const char* buffer)
{
  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);
}

int NetCmdSendAndReceive(const char *buffer, int silent)
{
  int ack;

  NetCmdSend(buffer);

  do {
    ack = NetCmdReceive(silent);
    if ((ack & 0xff) == CMD_BCMD) return ack >> 8;
    else if ((ack & 0xff) == CMD_NONE) usleep(1000);
    else {
      fprintf(stderr, "Protocol error from daemon.\n");
      exit(14);
    }
  } while (1);

  fprintf(stderr, "Unexpected trap in NetCmdSendAndReceive. Stop.\n");
  exit(-1);
}

//Blocks on reading until list comes through.
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

  if (read(sock, &client_n_scommands, sizeof(client_n_scommands)) < 0)
    printf("Warning: NetCmdGetCmdList failed to read n_scommands\n");
  if (read(sock, &client_n_mcommands, sizeof(client_n_mcommands)) < 0)
    printf("Warning: NetCmdGetCmdList failed to read n_mcommands\n");

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

//Blocks on reading until list comes through.
int NetCmdGetGroupNames(void)
{
  int i, j, n, c = 0;
  char buffer[128] = "::group::\r\n";

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  if (read(sock, &client_n_groups, sizeof(client_n_groups)) < 0)
    printf("Warning: NetCmdGetGroupNames failed to read n_groups\n");
  if (client_n_groups > 31) {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }
  client_group_names = (char**)malloc(client_n_groups * sizeof(char*));

  for (j=0; j < client_n_groups; ++j) {
    for (i=0; i<128; i++) {
      if ((n = read(sock, &c, 1)) <= 0) {
	perror("Unable to receive");
	exit(14);
      } else if (i == 127 && c != '\n') {
	fprintf(stderr, "Protocol error from daemon.\n");
	exit(14);
      }

      if (c == '\n') {
	buffer[i] = '\0';
	client_group_names[j] = (char*)malloc(strlen(buffer)+1);
	strcpy(client_group_names[j], buffer);
	break;
      } else buffer[i] = c;
    }
  }

  return 0;
}

int NetCmdTakeConn(int silent)
{
  int ack;
  char buffer[1024] = "::take::\r\n";

  /* don't take it if we already have it */
  if (is_free == 0 && strncmp(owner, me, strlen(me)) == 0)
    return 1;

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  //cow didn't wait for a response before. Maybe there was a reason for that
  do {
    ack = NetCmdReceive(silent);
    if ((ack & 0xff) == CMD_CONN) break;
    else if ((ack & 0xff) == CMD_NONE) usleep(1000);
    else {
      fprintf(stderr, "Protocol error from daemon.\n");
      exit(14);
    }
  } while (1);

  return (is_free == 0 && strncmp(owner, me, strlen(me)) == 0);
}

const char* NetCmdBanner(void)
{
  if (is_free)
    return "The conn is free.";
  else if (strncmp(owner, me, strlen(me)) == 0)
    return "I have the conn.";
  else {
    snprintf(banner, 1000, "%s has the conn.", owner);
    return banner;
  }
}

int NetCmdPing(void)
{
  char buffer[1024] = "::ping::\r\n";
  ssize_t sent;

  sent = send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  if (sent == -1)
    return 0;

  return 1;
}

// Initialization Function... All blocking network i/o.
int NetCmdConnect(const char* host, int silent, int silenter)
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
    return -14;
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
    return -141;
  }

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);
  if ((i = recv(sock, buffer, 1024, 0)) <= 0) {
    perror("Unable to receive");
    exit(14);
  } else if (buffer[0] != ':' || buffer[1] != ':' || buffer[2] != ':') {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }

  if ((i = SetOwner(buffer)) < 0) return -i;

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

  return 0;
}
