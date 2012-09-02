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
#include <signal.h>
#include <string.h>
#include <tcpd.h>
#include <unistd.h>

#include <sys/syslog.h>

#include <arpa/inet.h>

#include "blast.h"
#include "frameread.h"
#include "quendi.h"
#include "quendiclient.h"

#define VERSION   "1.2.0"
#define SERVER_PORT 44144
#define RENDEZ_PORT 14141
#define PID_FILE "/var/run/interloquendi.pid"
#define CONFIG_FILE "/etc/interloquendi.conf"

#define DEBUG

/* in rendezvous.c */
int InitRendezvous(const char*, int, const char*);

struct {
  union {
    char* as_string;
    int   as_int;
  } value;
  char type;
  const char name[48];
} options[] = {
  {{NULL}, 's', "CurFile"},
  {{NULL}, 's', "Directory"},
  {{NULL}, 's', "LocalNet"},
  {{NULL}, 'i', "MaxConnect"},
  {{NULL}, 's', "PidFile"},
  {{NULL}, 's', "RendezvousAs"},
  {{NULL}, 'i', "RendezvousAt"},
  {{NULL}, 's', "RendezvousWith"},
  {{NULL}, 'i', "SuffixLength"},
  {{NULL}, '\0', ""}
};

#define CFG_CUR_FILE       0
#define CFG_DIRECTORY      1
#define CFG_LOCAL_NET      2
#define CFG_MAX_CONNECT    3
#define CFG_PID_FILE       4
#define CFG_RENDEZ_AS      5
#define CFG_RENDEZ_AT      6
#define CFG_RENDEZ_WITH    7
#define CFG_SUFFIX_LENGTH  8

/* for libwrap */
int allow_severity = LOG_INFO;
int deny_severity = LOG_WARNING;

char* GetCurFile(char *buffer, int buflen)
{
  FILE* stream = NULL;
  char* ptr;

  if ((stream = fopen(options[CFG_CUR_FILE].value.as_string, "rt")) == NULL)
    berror(err, "can't open curfile %s",options[CFG_CUR_FILE].value.as_string);
  else if (fgets(buffer, buflen, stream) == NULL) {
    berror(err, "read error on curfile");
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
  socklen_t addrlen = sizeof(addr);
  struct hostent* thishost;
  struct request_info req;
  struct quendi_server_data_t QuendiData;

  char buffer[QUENDI_COMMAND_LENGTH];
  int np;
  int n;
  char params[1][QUENDI_COMMAND_LENGTH];
  struct quendi_data_port_t data;

  data.sock = -1;
  data.port_active = 0;
  data.sending_data = 0;

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
    bprintf(warning, "gethostbyaddr: %s", hstrerror(h_errno));
    thishost = NULL;
  }

  QuendiData.server_version = VERSION;
  QuendiData.server_name = "interloquendi";
  QuendiData.server_host = thishost ? thishost->h_name
    : inet_ntoa(addr.sin_addr);
  QuendiData.csock = csock;
  QuendiData.access_level = 1;
  QuendiData.directory = (char*)options[CFG_DIRECTORY].value.as_string;

  quendi_server_init(&QuendiData);
  quendi_add_data_port(&data);

  /* Service Loop */
  for (;;) {
    if ((n = quendi_get_cmd(buffer)) == 0) {
      n = quendi_parse(buffer, &np, params);
      switch(n) {
        case -2:
          quendi_respond(QUENYA_RESPONSE_PARAM_ERROR, "Parameter Missing");
          break;
        case -1:
          quendi_respond(QUENYA_RESPONSE_SYNTAX_ERROR, "Unrecognised Command");
          break;
        case QUENYA_COMMAND_ABOR:
          if (!data.sending_data)
            quendi_respond(QUENYA_RESPONSE_PORT_INACTIVE, NULL);
          else {
            quendi_reader_shutdown(data.fd, 1);
            data.sending_data = 0;
          }
          break;
        case QUENYA_COMMAND_CLOS:
          if (data.sock < 0)
            quendi_respond(QUENYA_RESPONSE_PORT_NOT_OPEN, NULL);
          else if (data.sending_data)
            quendi_respond(QUENYA_RESPONSE_PORT_ACTIVE, NULL);
          else {
            shutdown(data.sock, SHUT_RDWR);
            close(data.sock);
            data.sock = -1;
            quendi_respond(QUENYA_RESPONSE_OK, NULL);
          }
          break;
        case QUENYA_COMMAND_DATA:
          if (data.sock < 0) {
            quendi_respond(QUENYA_RESPONSE_PORT_NOT_OPEN, NULL);
            break;
          } else if (!data.staged) {
            quendi_respond(QUENYA_RESPONSE_NO_DATA_STAGED, NULL);
            break;
          } else if (data.sending_data) {
            quendi_respond(QUENYA_RESPONSE_PORT_ACTIVE, NULL);
            break;
          }

          data.seek_to = quendi_reader_init(data.frame_size, data.pos,
              data.chunk, data.name, options[CFG_SUFFIX_LENGTH].value.as_int);

          /* The first read from the file always involves a new chunk */
          data.new_chunk = 1;
          data.chunk_total = 0;
          data.frames_read = 0;
          data.sending_data = 1;
          data.remainder = 0;

          /* Fallthrough */
        case QUENYA_COMMAND_CONT:
          if (n == QUENYA_COMMAND_CONT && !data.sending_data) {
            quendi_respond(QUENYA_RESPONSE_PORT_INACTIVE, NULL);
            break;
          }

          data.port_active = 1;
          do {
            if (n != QUENYA_COMMAND_DATA) {
              n = quendi_advance_data(data.persist, data.chunk,
                  options[CFG_SUFFIX_LENGTH].value.as_int, &data.chunk_total,
                  options[CFG_CUR_FILE].value.as_string, data.name,
                  data.block_length, data.remainder);

              switch (n) {
                case FR_DONE:
                  quendi_reader_shutdown(data.fd, 1);
                  data.sending_data = 0;
                  data.staged = 0;
                  data.port_active = 0;
                  break;
                case FR_MORE_IN_FILE:
                  data.new_chunk = 0;
                  break;
                case FR_NEW_CHUNK:
                  close(data.fd);
                  data.new_chunk = 1;
                  break;
                case FR_CURFILE_CHANGED:
                  quendi_reader_shutdown(data.fd, 0);
                  data.sending_data = 0;
                  data.port_active = 0;
                  if (GetCurFile(data.name, QUENDI_COMMAND_LENGTH) == NULL)
                    quendi_respond(QUENYA_RESPONSE_TRANS_COMPLETE, NULL);
                  else {
                    data.frame_size = ReconstructChannelLists(data.name, NULL);
                    data.staged = quendi_stage_data(data.name, 0,
                        options[CFG_SUFFIX_LENGTH].value.as_int, 1,
                        data.frame_size);
                  }
                  break;
                default:
                  printf("huh? (%i)\n", n);
                  exit(1);
              }
            } else
              n = FR_NEW_CHUNK;

            if (n == FR_NEW_CHUNK || n == FR_MORE_IN_FILE) {
              /* read a block */
              data.block_length = quendi_read_data(data.new_chunk,
                  &data.fd, data.chunk, data.seek_to, &data.chunk_total,
                  data.frame_size, &data.frames_read, &data.remainder);

              data.seek_to = 0;
              data.new_chunk = 0;
//              printf("block size = %i\n", data.block_length);
            } else
              break;
          } while (data.block_length <= 0);

          if (n == FR_NEW_CHUNK || n == FR_MORE_IN_FILE)
            /* send the block */
            quendi_send_data(data.sock, data.frame_size, data.block_length);
          break;
        case QUENYA_COMMAND_IDEN:
          QuendiData.access_level = 1;
          quendi_respond(QUENYA_RESPONSE_ACCESS_GRANTED, NULL);
          break;
        case QUENYA_COMMAND_ASYN:
        case QUENYA_COMMAND_NOOP:
          quendi_respond(QUENYA_RESPONSE_OK, NULL);
          break;
        case QUENYA_COMMAND_OPEN:
          if (quendi_access_ok(1)) {
            if (data.sock < 0) {
              data.sock = quendi_dp_connect();
              if (data.sock == -2)
                quendi_respond(QUENYA_RESPONSE_OPEN_ERROR,
                    "Timeout Opening Data Port");
              else if (data.sock == -3)
                quendi_respond(QUENYA_RESPONSE_OPEN_ERROR,
                    "Access from Unauthorised Client on Data Port");
              else if (data.sock < 0)
                quendi_respond(QUENYA_RESPONSE_OPEN_ERROR, NULL);
              else {
                quendi_respond(QUENYA_RESPONSE_PORT_OPENED, NULL);
              }
            } else
              quendi_respond(QUENYA_RESPONSE_OPEN_ERROR, "Too Many Open Ports");
          }
          break;
        case QUENYA_COMMAND_QNOW:
          if (quendi_access_ok(1)) {
            if (data.sending_data)
              quendi_respond(QUENYA_RESPONSE_PORT_ACTIVE, NULL);
            else if (GetCurFile(data.name, QUENDI_COMMAND_LENGTH) == NULL)
              quendi_respond(QUENYA_RESPONSE_NO_CUR_DATA, NULL);
            else {
              data.persist = 1;
              data.frame_size = ReconstructChannelLists(data.name, NULL);
              data.pos = GetFrameFileSize(data.name,
                  options[CFG_SUFFIX_LENGTH].value.as_int) / data.frame_size;
              data.staged = quendi_stage_data(data.name, data.pos,
                  options[CFG_SUFFIX_LENGTH].value.as_int, 0, data.frame_size);
            }
          }
          break;
        case QUENYA_COMMAND_QUIT:
          quendi_respond(QUENYA_RESPONSE_GOODBYE, NULL);
          close(csock);
          exit(0);
        case QUENYA_COMMAND_RDVS:
          if (quendi_access_ok(1)) {
            QuendiData.rendezvous_name = bstrdup(fatal, params[0]);
            if (data.sock < 0) {
              data.sock = quendi_rp_connect(QuendiData.rendezvous_name);
              if (data.sock == -2)
                quendi_respond(QUENYA_RESPONSE_OPEN_ERROR,
                    "Unable to Resolve Rendezvous Host");
              else if (data.sock < 0)
                quendi_respond(QUENYA_RESPONSE_OPEN_ERROR,
                    "Error Opening Rendezvous Port");
              else
                quendi_respond(QUENYA_RESPONSE_PORT_OPENED,
                    "Connected to Rendezvous Port");
            } else
              quendi_respond(QUENYA_RESPONSE_OPEN_ERROR, "Too Many Open Ports");
          }
          break;
        case QUENYA_COMMAND_RTBK:
          if (!data.sending_data)
            quendi_respond(QUENYA_RESPONSE_PORT_INACTIVE, NULL);
          else {
            data.port_active = 1;
            quendi_send_data(data.sock, data.frame_size, data.block_length);
          }
          break;
        case QUENYA_COMMAND_SIZE:
          if (!data.staged)
            quendi_respond(QUENYA_RESPONSE_NO_DATA_STAGED, NULL);
          else
            quendi_respond(QUENYA_RESPONSE_FRAME_SIZE, NULL);
          break;
        case QUENYA_COMMAND_SPEC:
          if (data.sock < 0)
            quendi_respond(QUENYA_RESPONSE_PORT_NOT_OPEN, NULL);
          else if (!data.staged)
            quendi_respond(QUENYA_RESPONSE_NO_DATA_STAGED, NULL);
          else if (data.sending_data)
            quendi_respond(QUENYA_RESPONSE_PORT_ACTIVE, NULL);
          else {
            data.port_active = 1;
            quendi_send_spec(data.sock, data.name);
            data.port_active = 0;
          }
          break;
        case QUENYA_COMMAND_SYNC:
        default:
          quendi_respond(QUENYA_RESPONSE_CMD_NOT_IMPL, NULL);
          break;
      }
    } else if (n == -2) {
      bprintf(warning, "connection dropped\n");
      shutdown(csock, SHUT_RDWR);
      close(csock);
      if (data.sock >= 0) {
        bprintf(warning, "shutdown data port\n");
        shutdown(data.sock, SHUT_RDWR);
        close(data.sock);
      }
      quendi_server_shutdown();
      return;
    } else if (n == -3) {
      if (!data.port_active) {
        quendi_respond(QUENYA_RESPONSE_TIMEOUT, NULL);
        shutdown(csock, SHUT_RDWR);
        close(csock);
        quendi_server_shutdown();
        return;
      }
    } else if (n == -4) {
      quendi_respond(QUENYA_RESPONSE_PORT_CLOSE_ERR,
          "Unexpected Close on Data Port");
      shutdown(data.sock, SHUT_RDWR);
      close(data.sock);
      data.sock = -1;
    }
  }
}

int MakeListener(int port)
{
  int sock;
  struct sockaddr_in addr;

  sock = MakeSock();

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1)
    berror(fatal, "bind");

  if (listen(sock, 10) == -1)
    berror(fatal, "listen");

  bprintf(info, "listening on port %i.", port);

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

    found = 0;
    for (i = 0; options[i].type != '\0'; ++i)
      if (strcasecmp(option, options[i].name) == 0) {
        found = 1;
        switch (options[i].type) {
          case 's':
            options[i].value.as_string = bstrdup(fatal, value);
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
      bprintf(warning, "Unknown option `%s'", option);
  }
}

void LoadDefaultConfig(void)
{
  int i;

  for (i = 0; options[i].type; ++i)
    if (options[i].value.as_string == NULL)
      switch (i) {
        case CFG_CUR_FILE:
          options[i].value.as_string
            = bstrdup(fatal, "/mnt/decom/etc/decom.cur");
          break;
        case CFG_DIRECTORY:
          options[i].value.as_string = bstrdup(fatal, "/mnt/decom/rawdir");
          break;
        case CFG_LOCAL_NET:
        case CFG_RENDEZ_AS:
        case CFG_RENDEZ_WITH:
          options[i].value.as_string = "";
          break;
        case CFG_MAX_CONNECT:
          options[i].value.as_int = 0;
          break;
        case CFG_PID_FILE:
          options[i].value.as_string = bstrdup(fatal, PID_FILE);
          break;
        case CFG_RENDEZ_AT:
          options[i].value.as_int = RENDEZ_PORT;
          break;
        case CFG_SUFFIX_LENGTH:
          options[i].value.as_int = 3;
          break;
        default:
          bprintf(warning, "No default value for option `%s'",
              options[i].name);
      }
}

int main(void)
{
  int pid;
  FILE* stream;
  int sock, csock;
  socklen_t addrlen;
  struct sockaddr_in addr;

  /* set up our outputs */
#ifdef DEBUG
  buos_use_stdio();
#else
  openlog("interloquendi", LOG_PID, LOG_DAEMON);
  buos_use_syslog();
#endif

  /* read config file */
  if ((stream = fopen(CONFIG_FILE, "rt")) != NULL) {
    ReadConfig(stream);
    fclose(stream);
  } else
    berror(warning, "unable to open config file `%s'", CONFIG_FILE);

  /* fill uninitialised options with default values */
  LoadDefaultConfig();

#ifndef DEBUG
  /* Fork to background */
  if ((pid = fork()) != 0) {
    if (pid == -1)
      berror(fatal, "unable to fork to background");

    if ((stream = fopen(options[CFG_PID_FILE].value.as_string, "w")) == NULL)
      berror(err, "unable to write PID to disk");
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

  /* Autoreap children */
  signal(SIGCHLD, SIG_IGN);

  /* Set up Rendezvous, if necessary */
  if (InitRendezvous(options[CFG_RENDEZ_WITH].value.as_string,
        options[CFG_RENDEZ_AT].value.as_int,
        options[CFG_RENDEZ_AS].value.as_string))
    bputs(fatal, "Unable to rendezvous with upstream host.");

  /* initialise listener socket */
  sock = MakeListener(SERVER_PORT);

  /* accept loop */
  for (;;) {
    addrlen = sizeof(addr);
    csock = accept(sock, (struct sockaddr*)&addr, &addrlen);

    if (csock == -1 && errno == EINTR)
      continue;
    else if (csock == -1)
      berror(err, "accept");
    else {
      /* fork child */
      if ((pid = fork()) == 0) {
        close(sock);
        Connection(csock);
        exit(0);
      }

      bprintf(info, "spawned %i to handle connect from %s", pid,
          inet_ntoa(addr.sin_addr));
      close(csock);
    }
  }
}
