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

#include <stdlib.h>

#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <tcpd.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "blast.h"
#include "frameread.h"
#include "quendi.h"

#define VERSION   "0.9.0"
#define SOCK_PORT 44144
#define PID_FILE "/var/run/interloquendi.pid"
#define CONFIG_FILE "/usr/home/dwiebe/cvs/interloquendi/interloquendi.conf"

#define DEBUG

struct {
  union {
    char* as_string;
    int   as_int;
  } value;
  char type;
  const char name[48];
} options[] = {
  {{NULL}, 's', "Directory"},
  {{NULL}, 's', "PidFile"},
  {{NULL}, 's', "CurFile"},
  {{NULL}, 'i', "SuffixLength"},
  {{NULL}, '\0', ""}
};

#define CFG_DIRECTORY      0
#define CFG_PID_FILE       1
#define CFG_CUR_FILE       2
#define CFG_SUFFIX_LENGTH  3

struct data_connection {
  int sock;
  unsigned frame_size;
  unsigned long pos;
  char name[PATH_MAX];
};

/* for libwrap */
int allow_severity = LOG_INFO;
int deny_severity = LOG_WARNING;

char* GetCurFile(char *buffer, int buflen)
{
  FILE* stream = NULL;
  char* ptr;
  
  if ((stream = fopen(options[CFG_CUR_FILE].value.as_string, "rt")) == NULL)
    syslog(LOG_ERR, "can't open curfile: %m");
  else if (fgets(buffer, buflen, stream) == NULL) {
    syslog(LOG_ERR, "read error on curfile: %m");
    fclose(stream);
  } else {
    if ((ptr = strchr(buffer, '\n')) != NULL)
      *ptr = '\0';
    fclose(stream);
    return buffer;
  }

  return NULL;
}

void Connection(int csock)
{
  struct sockaddr_in addr;
  int addrlen = sizeof(addr);
  struct hostent* thishost;
  struct request_info req;
  struct quendi_data QuendiData;

  char buffer[QUENDI_COMMAND_LENGTH];
  int np;
  int n;
  char* params;
  struct data_connection data;

  data.sock = -1;

  /* tcp wrapper check */
  request_init(&req, RQ_DAEMON, "interloquendi", RQ_FILE, csock, 0);
  fromhost(&req);

  if (!hosts_access(&req)) {
    refuse(&req);
    exit(1); /* can't get here */
  }

  getsockname(csock, (struct sockaddr*)&addr, &addrlen);
  thishost = gethostbyaddr((const char*)&addr.sin_addr, sizeof(addr.sin_addr),
      AF_INET);
  if (thishost == NULL && h_errno) {
    syslog(LOG_WARNING, "gethostbyaddr: %s", hstrerror(h_errno));
    thishost = NULL;
  }

  QuendiData.server_version = VERSION;
  QuendiData.server_name = "interloquendi";
  QuendiData.server_host = thishost ? thishost->h_name
    : inet_ntoa(addr.sin_addr);
  QuendiData.csock = csock;
  QuendiData.access_level = 0;
  QuendiData.directory = (char*)options[CFG_DIRECTORY].value.as_string;

  quendi_server_init(&QuendiData);

  /* Service Loop */
  for (;;) {
    if ((n = quendi_get_cmd(buffer)) == 0)
      switch(quendi_parse(buffer, &np, &params)) {
        case -2:
          quendi_respond(QUENDR_PARAM_ERROR, "Parameter Missing");
          break;
        case -1:
          quendi_respond(QUENDR_SYNTAX_ERROR, "Unrecognised Command");
          break;
        case QUENDC_FORM:
          if (data.sock < 1)
            quendi_respond(QUENDR_PORT_NOT_OPEN, NULL);
          else {
            quendi_respond(QUENDR_CMD_NOT_IMPL, NULL);
          }
          break;
        case QUENDC_IDEN:
          QuendiData.access_level = 1;
          quendi_respond(QUENDR_ACCESS_GRANTED, NULL);
          break;
        case QUENDC_SYNC:
        case QUENDC_NOOP:
          quendi_respond(QUENDR_OK, NULL);
          break;
        case QUENDC_OPEN:
          if (quendi_access_ok(1)) {
            if (data.sock < 1) {
              data.sock = quendi_dp_connect();
              if (data.sock < 1)
                quendi_respond(QUENDR_OPEN_ERROR, NULL);
              else
                quendi_respond(QUENDR_PORT_OPENED, NULL);
            } else
              quendi_respond(QUENDR_OPEN_ERROR, "Too Many Open Ports");
          }
          break;
        case QUENDC_QNOW:
          if (quendi_access_ok(1)) {
            if (GetCurFile(data.name, QUENDI_COMMAND_LENGTH) == NULL)
              quendi_respond(QUENDR_NO_CUR_DATA, NULL);
            else
              quendi_stage_data(data.name,
                  data.pos = GetFrameFileSize(data.name,
                    options[CFG_SUFFIX_LENGTH].value.as_int) /
                  (data.frame_size = ReconstructChannelLists(data.name, NULL)));
          }
          break;
        case QUENDC_QUIT:
          quendi_respond(QUENDR_GOODBYE, NULL);
          close(csock);
          exit(0);
        case QUENDC_ASYN:
        default:
          quendi_respond(QUENDR_CMD_NOT_IMPL, NULL);
          break;
      }
    else if (n == -2) {
      printf("connection dropped\n");
      shutdown(csock, SHUT_RDWR);
      close(csock);
      quendi_server_shutdown();
    }
  }
}

int MakeSock(void)
{
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    syslog(LOG_CRIT, "socket: %m");
    exit(1);
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
    syslog(LOG_CRIT, "setsockopt: %m");
    exit(1);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    syslog(LOG_CRIT, "bind: %m");
    exit(1);
  }

  if (listen(sock, 10) == -1) {
    syslog(LOG_CRIT, "listen: %m");
    exit(1);
  }

  syslog(LOG_INFO, "listening on port %i.", SOCK_PORT);

  return sock;
}

void CleanUp(void)
{
  unlink(options[CFG_PID_FILE].value.as_string);
  closelog();
}

void ReadConfig(FILE* stream)
{
  char buffer[1024];
  char *option, *value;
  int i, found;

  while (fgets(buffer, sizeof(buffer), stream)) {
    /* remove comments */
    if ((option = strchr(buffer, '#')) != NULL)
      *option = '\0';

    /* strip newline */
    if ((option = strchr(buffer, '\n')) != NULL)
      *option = '\0';

    /* strip leading whitespace */
    option = buffer + strspn(buffer, " \t");
    if ((value = strchr(option, ' ')) != NULL) {
      *(value++) = '\0';
      value += strspn(value, " \t");
    }
    printf("o: %s\nv: %s\n", option, value);

    found = 0;
    for (i = 0; options[i].type != '\0'; ++i)
      if (strcasecmp(option, options[i].name) == 0) {
        found = 1;
        switch (options[i].type) {
          case 's':
            options[i].value.as_string = strdup(value);
            break;
          case 'i':
            options[i].value.as_int = atoi(value);
            break;
          default:
            printf("Unknown option type\n");
            exit(1);
        }
        break;
      }

    if (!found)
      syslog(LOG_WARNING, "Unknown option `%s'", option);
  }
}

void LoadDefaultConfig(void)
{
  int i;

  for (i = 0; options[i].type; ++i)
    if (options[i].value.as_string == NULL)
      switch (i) {
        case CFG_DIRECTORY:
          options[i].value.as_string = strdup("/mnt/decom/rawdir");
          break;
        case CFG_PID_FILE:
          options[i].value.as_string = strdup(PID_FILE);
          break;
        case CFG_CUR_FILE:
          options[i].value.as_string = strdup("/mnt/decom/etc/decom.cur");
          break;
        case CFG_SUFFIX_LENGTH:
          options[i].value.as_int = 3;
        default:
          syslog(LOG_WARNING, "No default value for option `%s'",
              options[i].name);
      }
}

int main(void)
{
  int pid;
  FILE* stream;
  int sock, addrlen, csock;
  struct sockaddr_in addr;

  /* set up our outputs */
  openlog("interloquendi", LOG_PID, LOG_DAEMON);
  blog_use_syslog();

  /* read config file */
  if ((stream = fopen(CONFIG_FILE, "rt")) != NULL) {
    ReadConfig(stream);
    fclose(stream);
  } else
    syslog(LOG_WARNING, "unable to open config file `%s': %m", CONFIG_FILE);

  /* fill uninitialised options with default values */
  LoadDefaultConfig();

#ifndef DEBUG
  /* Fork to background */
  if ((pid = fork()) != 0) {
    if (pid == -1) {
      syslog(LOG_CRIT, "unable to fork to background: %m");
      exit(1);
    }

    if ((stream = fopen(options[CFG_PID_FILE].value, "w")) == NULL)
      syslog(LOG_ERR, "unable to write PID to disk: %m");
    else {
      fprintf(stream, "%i\n", pid);
      fflush(stream);
      fclose(stream);
    }
    closelog();
    printf("PID = %i\n", pid);
    exit(0);
  }
  atexit(CleanUp);

  /* Daemonise */
  chdir("/");
  freopen("/dev/null", "r", stdin);
  freopen("/dev/null", "w", stdout);
  freopen("/dev/null", "w", stderr);
  setsid();
#endif

  /* initialise listener socket */
  sock = MakeSock();

  /* accept loop */
  for (;;) {
    addrlen = sizeof(addr);
    csock = accept(sock, (struct sockaddr*)&addr, &addrlen);

    if (csock == -1 && errno == EINTR)
      continue;
    else if (csock == -1)
      syslog(LOG_ERR, "accept: %m");
    else {
      /* fork child */
      if ((pid = fork()) == 0) {
        close(sock);
        Connection(csock);
      }

      syslog(LOG_INFO, "spawned %i to handle connect from %s", pid,
          inet_ntoa(addr.sin_addr));
      close(csock);
    }
  }
}
