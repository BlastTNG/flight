/* quendi protocol: an implementation of the QUendi Protocol
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * quendi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * quendi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with quendi; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>

#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <tcpd.h>

#include <sys/socket.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "quendi.h"

/* internals */
char *_quendi_server_version = NULL;
char *_quendi_server_host = NULL;
char *_quendi_server_name = NULL;
const char _quendi_version[] = "1.0";
int _quendi_dp = -1;
int _quendi_access_level = 0;

int quendi_access_ok(int sock, int level) {
  if (_quendi_access_level == 0) {
    quendi_respond(sock, QUENDR_NOT_IDENTIFIED, NULL);
    return 0;
  } else if (_quendi_access_level < level) {
    quendi_respond(sock, QUENDR_NO_ACCESS, NULL);
    return 0;
  }

  return 1;
}

int quendi_cmdnum(char* buffer)
{
  int i;

  /* command name to lower case */
  for (i = 0; i < 4; ++i)
    buffer[i] |= 0x20;

  switch(htonl(*(int*)buffer)) {
    case 0x6173796e: return QUENDC_ASYN;
    case 0x666f726d: return QUENDC_FORM;
    case 0x68657265: return QUENDC_HERE;
    case 0x6e6f6f70: return QUENDC_NOOP;
    case 0x6f70656e: return QUENDC_OPEN;
    case 0x716e6f77: return QUENDC_QNOW;
    case 0x71756974: return QUENDC_QUIT;
    case 0x73796e63: return QUENDC_SYNC;
  }

  printf("cmd = %04x\n", htonl(*(int*)buffer));
  return -1;
}

int quendi_get_next_param(char *buffer, int *nparams, char **params)
{
  char* ptr = buffer;
  if (*ptr == '\0')
    return 1;

  if (*++ptr == '\0')
    return 1;

  params[(*nparams)++] = ptr;

  while (*ptr != '\0' && *ptr != ' ')
    ++ptr;

  *ptr = '\0';

  printf("Param %i: %s\n", *nparams - 1, params[*nparams - 1]);

  return 0;
}

int quendi_dp_open(int sock)
{
  struct sockaddr_in addr;
  int addrlen;
  int dsock;

  addrlen = sizeof(addr);
  getsockname(sock, (struct sockaddr*)&addr, &addrlen);
  if ((dsock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    syslog(LOG_WARNING, "socket: %m");
    return -1;
  }

  addr.sin_port = htons(0);
  if (bind(dsock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    syslog(LOG_WARNING, "bind: %m");
    return -1;
  }

  return dsock;
}

int quendi_dp_connect(int csock)
{
  int dsock;
  int i;
  char buffer[QUENDI_RESPONSE_LENGTH];
  struct sockaddr_in addr;
  int addrlen;
  struct request_info req;

  dsock = quendi_dp_open(csock);
  if (dsock < 1) {
    quendi_respond(csock, QUENDR_OPEN_ERROR, NULL);
    return -1;
  }

  addrlen = sizeof(addr);
  getsockname(dsock, (struct sockaddr*)&addr, &addrlen);
  snprintf(buffer, QUENDI_RESPONSE_LENGTH - 1,
      "%i@%s:%i Listening on Port", 1, inet_ntoa(addr.sin_addr),
      ntohs(addr.sin_port));
  quendi_respond(csock, QUENDR_LISTENING, buffer);

  if (listen(dsock, 1)) {
    syslog(LOG_WARNING, "dp listen: %m");
    return 0;
  }

  i = accept(dsock, (struct sockaddr*)&addr, &addrlen);
  if (i == -1)
    syslog(LOG_WARNING, "dp accept: %m");

  /* tcp wrapper check */
  request_init(&req, RQ_DAEMON, "interloquendi", RQ_FILE, csock, 0);
  fromhost(&req);

  if (!hosts_access(&req)) {
    close(dsock);
    return -1;
  }

  return i;
}

char* quendi_make_response(char* buffer, int response_num, const char* message)
{
  int size;

  if (message)
    size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i %s\r\n", response_num,
        message);
  else
    switch (response_num) {
      case QUENDR_LISTENING: /* 123 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Listening on Data Port for Connection\r\n", response_num);
        break;
      case QUENDR_SERVICE_READY: /* 220 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %s %s %s/%s Service Ready\r\n", response_num,
            _quendi_server_host, _quendi_server_name,
            _quendi_server_version, _quendi_version);
        break;
      case QUENDR_GOODBYE: /* 221 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %s Closing Connection\r\n", response_num,
            _quendi_server_host);
        break;
      case QUENDR_PORT_OPENED: /* 222 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Opened\r\n", response_num);
        break;
      case QUENDR_ACCESS_GRANTED: /* 222 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Access Granted\r\n", response_num);
        break;
      case QUENDR_OK: /* 250 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i OK\r\n",
            response_num);
        break;
      case QUENDR_OPEN_ERROR: /* 420 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Error Opening Data Port\r\n", response_num);
        break;
      case QUENDR_SYNTAX_ERROR: /* 500 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i Syntax Error\r\n",
            response_num);
        break;
      case QUENDR_PARAM_ERROR: /* 501 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Parameter Error\r\n", response_num);
        break;
      case QUENDR_CMD_NOT_IMPL: /* 502 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Command Not Implemented\r\n", response_num);
        break;
      case QUENDR_PORT_NOT_OPEN: /* 520 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Not Open\r\n", response_num);
        break;
      case QUENDR_NOT_IDENTIFIED: /* 530 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Client Not Identified\r\n", response_num);
        break;
      case QUENDR_NO_ACCESS: /* 531 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Insufficient Privilege\r\n", response_num);
        break;
      default:
        return NULL;
    } 

  if (size <= 0 || size > QUENDI_RESPONSE_LENGTH)
    return NULL;

  return buffer;
}

int quendi_parse(char *buffer, int *nparams, char **params)
{
  int cmd = quendi_cmdnum(buffer);

  if (cmd == QUENDC_HERE) {
    if (quendi_get_next_param(buffer + 4, nparams, params))
      return -2;
  } else {
    *nparams = 0;
  }

  return cmd;
}

int quendi_respond(int sock, int response_num, const char *message)
{
  char buffer[QUENDI_RESPONSE_LENGTH + 1];

  if (quendi_make_response(buffer, response_num, message) == NULL)
    return -1;

  write(sock, buffer, strlen(buffer));

  return 0;
}

void quendi_server_init(const char* server_version, const char* server_name,
    const char* server_host)
{
  _quendi_server_version = strdup(server_version);
  _quendi_server_name = strdup(server_name);
  _quendi_server_host = strdup(server_host);
}

int quendi_server_start(int sock, const char* server_version,
    const char* server_name, const char* server_host)
{
  char buffer[QUENDI_COMMAND_LENGTH + 1];
  int i, n, np;
  int overrun;
  int eolfound;
  char* params;

  quendi_server_init(server_version, server_name, server_host);

  quendi_respond(sock, QUENDR_SERVICE_READY, NULL);

  /* Service Loop */
  for (;;) {
    for(overrun = eolfound = 0; !eolfound; ) {

      n = read(sock, buffer, QUENDI_COMMAND_LENGTH);

      if (n == 0) {
        printf("connection dropped\n");
        shutdown(sock, SHUT_RDWR);
        close(sock);
        quendi_server_shutdown();
        return 0;
      }

      eolfound = 0;
      /* check for buffer overrun */
      for (i = 0; i < QUENDI_COMMAND_LENGTH; ++i)
        if (buffer[i] == '\n') {
          buffer[i] = '\0';
          eolfound = 1;
          break;
        }

      if (!eolfound)
        overrun = 1;
    }

    if (overrun)
      quendi_respond(sock, QUENDR_SYNTAX_ERROR, "Command Line too Long");
    else if (i <= 4 || buffer[i - 1] != '\r')
      quendi_respond(sock, QUENDR_SYNTAX_ERROR, NULL);
    else if (buffer[4] != ' ' && buffer[4] != '\r') {
      printf("'%c' == %i\n", buffer[4], buffer[4]);
      quendi_respond(sock, QUENDR_SYNTAX_ERROR, NULL);
    } else {
      buffer[i - 1] = '\0';
      printf("Got: %s\n", buffer);
      switch(quendi_parse(buffer, &np, &params)) {
        case -2:
          quendi_respond(sock, QUENDR_PARAM_ERROR, "Parameter Missing");
          break;
        case -1:
          quendi_respond(sock, QUENDR_SYNTAX_ERROR, "Unrecognised Command");
          break;
        case QUENDC_FORM:
          if (_quendi_dp < 1)
            quendi_respond(sock, QUENDR_PORT_NOT_OPEN, NULL);
          else {
            quendi_respond(sock, QUENDR_CMD_NOT_IMPL, NULL);
          }
          break;
        case QUENDC_HERE:
          _quendi_access_level = 1;
          quendi_respond(sock, QUENDR_ACCESS_GRANTED, NULL);
          break;
        case QUENDC_SYNC:
        case QUENDC_NOOP:
          quendi_respond(sock, QUENDR_OK, NULL);
          break;
        case QUENDC_OPEN:
          if (quendi_access_ok(sock, 1)) {
            if (_quendi_dp < 1) {
              _quendi_dp = quendi_dp_connect(sock);
              if (_quendi_dp < 1)
                quendi_respond(sock, QUENDR_OPEN_ERROR, NULL);
              else
                quendi_respond(sock, QUENDR_PORT_OPENED, NULL);
            } else
              quendi_respond(sock, QUENDR_OPEN_ERROR, "Too Many Open Ports");
          }
          break;
        case QUENDC_QNOW:
          if (quendi_access_ok(sock, 1)) {
          }
          break;
        case QUENDC_QUIT:
          quendi_respond(sock, QUENDR_GOODBYE, NULL);
          close(sock);
          return 0;
        default:
          quendi_respond(sock, QUENDR_CMD_NOT_IMPL, NULL);
          break;
      }
    }
  }
  return 1;
}

void quendi_server_shutdown(void)
{
  if (_quendi_server_version) {
    free(_quendi_server_version);
    _quendi_server_version = NULL;
  }
  if (_quendi_server_host) {
    free(_quendi_server_host);
    _quendi_server_host = NULL;
  }
  if (_quendi_server_name) {
    free(_quendi_server_name);
    _quendi_server_name = NULL;
  }
}
