/* netcmd: BLAST command protocol client
 *
 * This software is copyright (C) 2005 D. V. Wiebe and others
 * Parts of this software are copyright 2010 Matthew Truch
 * 
 * This file is part of the BLAST flight code.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
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
static int sock;

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
int NetCmdReceive(int silent, size_t oob_buflen, char *oob_message)
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
    int ack = atoi(buffer + 9);

    if (oob_message) {
      /* look for a message */
      char *ptr = strstr(buffer + 10, ":::");
      if (ptr)
        strncpy(oob_message, ptr + 3, oob_buflen);
      else
        oob_message[0] = 0;
    }

    return CMD_BCMD + (ack << 8);
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

int NetCmdSendAndReceive(const char *inbuf, int silent, size_t buflen,
    char *outbuf)
{
  int ack;

  NetCmdSend(inbuf);

  do {
    ack = NetCmdReceive(silent, buflen, outbuf);
    if ((ack & 0xff) == CMD_BCMD) {
      return ack >> 8;
    } else if ((ack & 0xff) == CMD_SENT) { /* ignore packet info */
      continue;
    } else if ((ack & 0xff) == CMD_NONE)
      usleep(1000);
    else {
      fprintf(stderr, "Protocol error from daemon (0x%04X).\n", ack);
      exit(14);
    }
  } while (1);

  fprintf(stderr, "Unexpected trap in NetCmdSendAndReceive. Stop.\n");
  exit(-1);
}

/* read a \n-terminated string from the server, returning a malloc'd buffer */
static char *read_string(int max)
{
  char *s = malloc(max);
  int i, n;
  char c;

  /* max includes the '\0' */
  for (i = 0; i < max; ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (i == max - 1 && c != '\n')
      break;
    else if (c == '\n') {
      s[i] = 0;
      return s;
    } else
      s[i] = c;
  }

  /* overrun */
  fprintf(stderr, "Protocol error from daemon.\n");
  exit(14);
}

// get back default of one command parameter
// cmdstr should be <cmdname>;<parname>
double NetCmdGetDefault(char *cmdstr) {
  char buffer[1024];
  char defstr[50];
  int len;
  int i, n, c = 0;

  sprintf(buffer, "::cmddef::%s\r\n", cmdstr);
  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  strcpy(buffer, ":::cmddef:::");

  len = strlen(buffer);
  for (i = 0; i < len; ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (buffer[i] != c)
      goto CMDDEF_READ_ERROR;
  }

  for (i = 0; i < 100; ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (i == 99 && c != '\n')
      goto CMDDEF_READ_ERROR;

    if (c == '\n') {
      defstr[i] = '\0';
      break;
    } else if (c != '\r')
      defstr[i] = c;
  }

  return atof(defstr);

CMDDEF_READ_ERROR:
  fprintf(stderr, "Protocol error from daemon.\n");
  return (0);
  //exit(14);
}

//Blocks on reading until list comes through.
int NetCmdGetCmdList(void)
{
  uint16_t u16;
  int n, c = 0;
  size_t i, len;
  char buffer[1024] = "::list::\r\n";

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  strcpy(buffer, ":::rev:::");

  len = strlen(buffer);
  for (i = 0; i < len; ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (buffer[i] != c) 
      goto CMDLIST_READ_ERROR;
  }

  for (i = 0; i < 100; ++i) {
    if ((n = read(sock, &c, 1)) <= 0) {
      perror("Unable to receive");
      exit(14);
    } else if (i == 99 && c != '\n')
      goto CMDLIST_READ_ERROR;

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
      client_n_scommands * client_n_mcommands == 0)
  {
    goto CMDLIST_READ_ERROR;
  }

  client_scommands = (struct scom*)malloc(client_n_scommands
      * sizeof(struct scom));
  client_mcommands = (struct mcom*)malloc(client_n_mcommands
      * sizeof(struct mcom));
  recv(sock, client_scommands, client_n_scommands * sizeof(struct scom),
      MSG_WAITALL);
  recv(sock, client_mcommands, client_n_mcommands * sizeof(struct mcom),
      MSG_WAITALL);

  /* read parameter value tables */
  for (;;) {
    int cmd, prm;
    if (read(sock, &u16, sizeof(uint16_t)) < 1)
      goto CMDLIST_READ_ERROR;

    if (u16 == 0xFFFF) /* end */
      break;
    cmd = u16 >> 8;
    prm = u16 & 0xFF;

    if (cmd >= client_n_mcommands || prm >= client_mcommands[cmd].numparams)
      goto CMDLIST_READ_ERROR;

    if (read(sock, &u16, sizeof(uint16_t)) < 1)
      goto CMDLIST_READ_ERROR;

    client_mcommands[cmd].params[prm].nt = malloc((u16 + 1) * sizeof(char*));
    for (i = 0; i < u16; ++i)
      client_mcommands[cmd].params[prm].nt[i] = read_string(80);
    client_mcommands[cmd].params[prm].nt[u16] = NULL;
  }

  return 0;

CMDLIST_READ_ERROR:
  fprintf(stderr, "Protocol error from daemon.\n");
  exit(14);
}

//Blocks on reading until list comes through.
int NetCmdGetGroupNames(void)
{
  int j;
  char buffer[128] = "::group::\r\n";

  send(sock, buffer, strlen(buffer), MSG_NOSIGNAL);

  if (read(sock, &client_n_groups, sizeof(client_n_groups)) < 0)
    printf("Warning: NetCmdGetGroupNames failed to read n_groups\n");
  if (client_n_groups > 31) {
    fprintf(stderr, "Protocol error from daemon.\n");
    exit(14);
  }
  client_group_names = (char**)malloc(client_n_groups * sizeof(char*));

  for (j = 0; j < client_n_groups; ++j)
    client_group_names[j] = read_string(128);

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
    ack = NetCmdReceive(silent, 0, NULL);
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
int NetCmdConnect(const char* host_in, int silent, int silenter)
{
  int i;
  char buffer[1024];
  struct hostent* the_host;
  struct sockaddr_in addr;
  struct passwd pw;
  struct passwd *pwptr;
  char host[255];
  int i_ch;
  int port;

  for (i_ch = 0; (host_in[i_ch]!='\0') && (host_in[i_ch]!=':'); i_ch++) {
    host[i_ch] = host_in[i_ch];
  }
  host[i_ch] = '\0';
  if (host_in[i_ch]==':') {
    port = atoi(host_in+i_ch+1);
  } else {
    port = SOCK_PORT;
  }

  /* get remote host IP */
  the_host = gethostbyname(host);

  if (the_host == NULL) {
    fprintf(stderr, "host lookup failed for `%s': %s\n", host,
        hstrerror(h_errno));
    return -14;
  }

  addr.sin_port = htons(port);
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
        inet_ntoa(addr.sin_addr), port);

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
