/* quendi.c: an implementation of the Quenya Protocol
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
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <tcpd.h>

#include <sys/socket.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "crc.h"
#include "blast.h"
#include "quendi.h"
#include "quendiclient.h"
#include "frameread.h"

/* upped from 50 to 200 to handle high latencies of rsyncing frame files */
#define INPUT_BUF_SIZE 200 /* Frames are big (~1 kb) and we take a big
                           * performance hit if we read more than 64k at a
                           * time, so we keep this small */

/* internals */
static const char quendi_version[] = "1.1";
static const struct quendi_server_data_t *quendi_server_data = NULL;
static const struct quendi_data_port_t *quendi_data_port = NULL;
static unsigned short* quendi_input_buffer[INPUT_BUF_SIZE];
static unsigned quendi_frame_size;

/* functions */
int quendi_access_ok(int level) {
  if (quendi_server_data->access_level == 0) {
    quendi_respond(QUENYA_RESPONSE_NOT_IDENTIFIED, NULL);
    return 0;
  } else if (quendi_server_data->access_level < level) {
    quendi_respond(QUENYA_RESPONSE_NO_ACCESS, NULL);
    return 0;
  }

  return 1;
}

void quendi_add_data_port(const struct quendi_data_port_t* data_port)
{
  quendi_data_port = data_port;
}

int quendi_cmdnum(char* buffer)
{
  int i;

  /* command name to lower case */
  for (i = 0; i < 4; ++i)
    buffer[i] |= 0x20;

  switch(htonl(*(int*)buffer)) {
    case 0x61626f72: return QUENYA_COMMAND_ABOR;
    case 0x6173796e: return QUENYA_COMMAND_ASYN;
    case 0x636c6f73: return QUENYA_COMMAND_CLOS;
    case 0x636f6e74: return QUENYA_COMMAND_CONT;
    case 0x64617461: return QUENYA_COMMAND_DATA; 
    case 0x6964656e: return QUENYA_COMMAND_IDEN;
    case 0x6e6f6f70: return QUENYA_COMMAND_NOOP;
    case 0x6f70656e: return QUENYA_COMMAND_OPEN;
    case 0x716e6f77: return QUENYA_COMMAND_QNOW;
    case 0x71756974: return QUENYA_COMMAND_QUIT;
    case 0x72647673: return QUENYA_COMMAND_RDVS;
    case 0x7274626b: return QUENYA_COMMAND_RTBK;
    case 0x73697a65: return QUENYA_COMMAND_SIZE;
    case 0x73706563: return QUENYA_COMMAND_SPEC;
    case 0x73796e63: return QUENYA_COMMAND_SYNC;
  }

  printf("cmd = %04x\n", htonl(*(int*)buffer));
  return -1;
}

int quendi_get_cmd(char* buffer)
{
  int overrun;
  int eolfound;
  int n, i;
  fd_set sock_set;
  struct timeval timeout;
  int did_timeout;

  timeout.tv_sec = 60;
  timeout.tv_usec = 0;

  FD_ZERO(&sock_set);
  FD_SET(quendi_server_data->csock, &sock_set);
  if (quendi_data_port->sock >= 0)
    FD_SET(quendi_data_port->sock, &sock_set);

  i = quendi_server_data->csock;
  if (i < quendi_data_port->sock)
    i = quendi_data_port->sock;

  n = select(i + 1, &sock_set, NULL, NULL, &timeout);

  did_timeout = 1;
  if (quendi_data_port->sock > 0 &&
      FD_ISSET(quendi_data_port->sock, &sock_set)) {
    did_timeout = 0;
    n = read(quendi_data_port->sock, buffer, QUENDI_COMMAND_LENGTH);

    if (n == 0) {
      bprintf(warning, "data port close\n");
      return -4;
    }
  }

  if (FD_ISSET(quendi_server_data->csock, &sock_set)) {
    did_timeout = 0;
    for (overrun = eolfound = 0; !eolfound; ) {
      n = read(quendi_server_data->csock, buffer, QUENDI_COMMAND_LENGTH);

      if (n == 0)
        return -2;

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
      quendi_respond(QUENYA_RESPONSE_SYNTAX_ERROR, "Command Line too Long");
    else if (i <= 4 || buffer[i - 1] != '\r')
      quendi_respond(QUENYA_RESPONSE_SYNTAX_ERROR, NULL);
    else if (buffer[4] != ' ' && buffer[4] != '\r') {
      printf("'%c' == %i\n", buffer[4], buffer[4]);
      quendi_respond(QUENYA_RESPONSE_SYNTAX_ERROR, NULL);
    } else {
      buffer[i - 1] = '\0';
      printf("Got: %s\n", buffer);
      return 0;
    }
  }

  if (did_timeout)
    return -3;
  else 
    return -1;
}

int quendi_get_next_param(char *buffer, int *nparams,
    char params[][QUENDI_COMMAND_LENGTH])
{
  char* ptr = buffer;
  if (*ptr == '\0')
    return 1;

  if (*++ptr == '\0')
    return 1;

  strncpy(params[(*nparams)++], ptr, QUENDI_COMMAND_LENGTH);

  while (*ptr != '\0' && *ptr != ' ')
    ++ptr;

  *ptr = '\0';

  printf("Param %i: %s\n", *nparams - 1, params[*nparams - 1]);

  return 0;
}

int quendi_dp_open(void)
{
  struct sockaddr_in addr;
  socklen_t addrlen;
  int dsock;

  addrlen = sizeof(addr);
  getsockname(quendi_server_data->csock, (struct sockaddr*)&addr, &addrlen);
  if ((dsock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    berror(warning, "socket");

  addr.sin_port = htons(0);
  if (bind(dsock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1)
    berror(warning, "bind");

  return dsock;
}

int quendi_dp_connect(void)
{
  int dsock;
  int i;
  char buffer[QUENDI_RESPONSE_LENGTH];
  struct sockaddr_in addr;
  socklen_t addrlen;
  struct request_info req;
  fd_set sock_set;
  struct timeval timeout;

  dsock = quendi_dp_open();
  if (dsock < 1) {
    quendi_respond(QUENYA_RESPONSE_OPEN_ERROR, NULL);
    return -1;
  }

  addrlen = sizeof(addr);
  getsockname(dsock, (struct sockaddr*)&addr, &addrlen);
  snprintf(buffer, QUENDI_RESPONSE_LENGTH - 1,
      "%i@%s:%i Listening on Port", 1, inet_ntoa(addr.sin_addr),
      ntohs(addr.sin_port));
  quendi_respond(QUENYA_RESPONSE_LISTENING, buffer);

  if (listen(dsock, 1)) {
    syslog(LOG_WARNING, "dp listen: %m");
    return 0;
  }

  FD_ZERO(&sock_set);
  FD_SET(dsock, &sock_set);

  timeout.tv_sec = 60;
  timeout.tv_usec = 0;

  i = select(dsock + 1, &sock_set, NULL, NULL, &timeout);
  if (i == -1)
    syslog(LOG_WARNING, "dp select: %m");
  else if (FD_ISSET(dsock, &sock_set)) {
    i = accept(dsock, (struct sockaddr*)&addr, &addrlen);
    if (i == -1)
      syslog(LOG_WARNING, "dp accept: %m");
  } else {
    close(dsock);
    return -2; /* timeout */
  }

  if (i == -1) {
    close(dsock);
    return -1; /* select or accept error */
  }

  /* tcp wrapper check */
  request_init(&req, RQ_DAEMON, quendi_server_data->server_name, RQ_FILE,
      quendi_server_data->csock, 0);
  fromhost(&req);

  if (!hosts_access(&req)) {
    close(dsock);
    return -3; /* host access error */
  }

  close(dsock);
  return i;
}

int quendi_rp_connect(const char* host)
{
  int rsock;
  struct sockaddr_in addr;

  rsock = MakeSock();

  if (ResolveHost(host, &addr, 0) != NULL)
    return -2;

  if (connect(rsock, (struct sockaddr*)&addr, sizeof(addr)) != 0)
    return -1;

  return rsock;
}

char* quendi_make_response(char* buffer, int response_num, const char* message)
{
  int size;

  if (message)
    size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i %s\r\n", response_num,
        message);
  else
    switch (response_num) {
      case QUENYA_RESPONSE_LISTENING: /* 123 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Listening on Data Port for Connection\r\n", response_num);
        break;
      case QUENYA_RESPONSE_SENDING_DATA: /* 152 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Sending Data\r\n", response_num);
        break;
      case QUENYA_RESPONSE_SENDING_SPEC: /* 153 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Sending Spec File\r\n", response_num);
        break;
      case QUENYA_RESPONSE_OK: /* 200 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i OK\r\n",
            response_num);
        break;
      case QUENYA_RESPONSE_DATA_STAGED: /* 211 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Staged\r\n", response_num);
        break; 
      case QUENYA_RESPONSE_SERVICE_READY: /* 220 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %s %s %s/%s Service Ready\r\n", response_num,
            quendi_server_data->server_host, quendi_server_data->server_name,
            quendi_server_data->server_version, quendi_version);
        break;
      case QUENYA_RESPONSE_GOODBYE: /* 221 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %s Closing Connection\r\n", response_num,
            quendi_server_data->server_host);
        break;
      case QUENYA_RESPONSE_PORT_OPENED: /* 223 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Opened\r\n", response_num);
        break;
      case QUENYA_RESPONSE_ACCESS_GRANTED: /* 230 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Access Granted\r\n", response_num);
        break;
      case QUENYA_RESPONSE_FRAME_SIZE: /* 242 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %u Frame Bytes\r\n", response_num, quendi_frame_size);
        break;
      case QUENYA_RESPONSE_TRANS_COMPLETE: /* 250 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Transfer Complete\r\n", response_num);
        break;
      case QUENYA_RESPONSE_BLOCK_CRC: /* 316 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i 0x0000 Chunk CRC\r\n", response_num);
        break; 
      case QUENYA_RESPONSE_OPEN_ERROR: /* 423 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Error Opening Data Port\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NO_CUR_DATA: /* 451 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i No Current Data Writer\r\n", response_num);
        break;
      case QUENYA_RESPONSE_SYNTAX_ERROR: /* 500 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i Syntax Error\r\n",
            response_num);
        break;
      case QUENYA_RESPONSE_PARAM_ERROR: /* 501 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Parameter Error\r\n", response_num);
        break;
      case QUENYA_RESPONSE_CMD_NOT_IMPL: /* 502 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Command Not Implemented\r\n", response_num);
        break;
      case QUENYA_RESPONSE_PORT_INACTIVE: /* 503 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Invalid Command: Data Port Not Active\r\n", response_num);
        break;
      case QUENYA_RESPONSE_PORT_NOT_OPEN: /* 523 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Not Open\r\n", response_num);
        break;
      case QUENYA_RESPONSE_PORT_ACTIVE: /* 524 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Already Active.  Send ABOR First.\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NOT_IDENTIFIED: /* 530 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Client Not Identified\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NO_ACCESS: /* 531 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Insufficient Privilege\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NO_DATA_STAGED: /* 551 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i No Data Staged\r\n", response_num);
        break;
      case QUENYA_RESPONSE_TIMEOUT: /* 621 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Timeout Waiting for Command\r\n", response_num);
        break;
      case QUENYA_RESPONSE_PORT_CLOSE_ERR: /* 623 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Closed\r\n", response_num);
        break;
      default:
        return NULL;
    } 

  if (size <= 0 || size > QUENDI_RESPONSE_LENGTH)
    return NULL;

  return buffer;
}

int quendi_parse(char *buffer, int *nparams,
    char params[][QUENDI_COMMAND_LENGTH])
{
  int cmd = quendi_cmdnum(buffer);

  *nparams = 0;

  switch(cmd) {
    case QUENYA_COMMAND_IDEN:
    case QUENYA_COMMAND_RDVS:
      if (quendi_get_next_param(buffer + 4, nparams, params))
        return -2;
      break;
  }

  return cmd;
}

unsigned long quendi_reader_init(unsigned frame_size, unsigned long pos,
    char* chunk, const char* name, int sufflen)
{
  int i;

  strcpy(chunk, name);

  quendi_input_buffer[0] = (unsigned short*)balloc(fatal,
      frame_size * INPUT_BUF_SIZE);

  for (i = 1; i < INPUT_BUF_SIZE; ++i)
    quendi_input_buffer[i] = (void*)quendi_input_buffer[0] + i * frame_size;

  return SetStartChunk(pos, chunk, sufflen) * frame_size;
}

int quendi_respond(int response_num, const char *message)
{
  char buffer[QUENDI_RESPONSE_LENGTH + 1];

  if (quendi_make_response(buffer, response_num, message) == NULL)
    return -1;

  send(quendi_server_data->csock, buffer, strlen(buffer), MSG_DONTWAIT);
  printf("Replied: %s", buffer);

  return 0;
}

int quendi_read_data(int new_chunk, int* fd, const char* chunk,
    unsigned long seek_to, int *chunk_total, unsigned frame_size,
    unsigned long *frames_read, unsigned *remainder)
{
  int n, block_size;
  struct stat chunk_stat;

  if (new_chunk) {
    printf("New chunk ok\n");
    if ((*fd = open(chunk, O_RDONLY)) < 0)
      berror(fatal, "cannot open `%s'", chunk);

    printf("New chunk fopen ok\n");
    if (seek_to > 0) {
      lseek(*fd, seek_to, SEEK_SET);
      seek_to = 0;
      printf("New chunk fseek ok\n");
    }

    if (stat(chunk, &chunk_stat))
      berror(fatal, "cannot stat `%s'", chunk);

    *chunk_total = chunk_stat.st_size / frame_size;
    printf("New chunk stat ok (%i)\n", *chunk_total);

  }

  if ((n = read(*fd, (void*)quendi_input_buffer[0] + *remainder, frame_size *
          INPUT_BUF_SIZE - *remainder)) < 1) {
    if (n == 0)
      return 0;
    else {
      berror(err, "error reading `%s'", chunk);

      close(*fd);
      if ((*fd = open(chunk, O_RDONLY)) < 0)
        berror(fatal, "cannot open `%s'", chunk);

      lseek(*fd, *frames_read * frame_size, SEEK_SET);
      n = 0;
    }
  }

  block_size = (*remainder + n) / frame_size;
  *remainder = (*remainder + n) % frame_size;
  *frames_read += block_size;

  return block_size;
}

void quendi_send_data(int dsock, unsigned frame_size, int block_size)
{
  int i;
  char buffer[100];
  unsigned short crc;

  if (block_size > 0) {
    snprintf(buffer, 100, "%i Frame Block Transfer Starting", block_size);
    quendi_respond(QUENYA_RESPONSE_SENDING_DATA, buffer);

    for (i = 0; i < block_size; ++i) {
      if (write(dsock, quendi_input_buffer[i], frame_size) < 0)
	berror(err, "failed to write to socket");
    }

    crc = CalculateCRC(CRC_SEED, quendi_input_buffer[0],
        frame_size * block_size);

    sprintf(buffer, "0x%04X Block CRC", crc);
    quendi_respond(QUENYA_RESPONSE_BLOCK_CRC, buffer);
  }

  return;
}

int quendi_advance_data(int persist, char* chunk, int sufflen, int *chunk_total,
    const char* curfile_name, char* curfile_val, int block_size,
    unsigned remainder)
{
  if (block_size > 0 && remainder > 0)
    memcpy(quendi_input_buffer[0], quendi_input_buffer[block_size + 1],
        remainder);

  return StreamToNextChunk(persist, chunk, sufflen, chunk_total, curfile_name,
      curfile_val);
}

void quendi_reader_shutdown(int fd, int flag)
{
  bfree(fatal, quendi_input_buffer[0]);

  close(fd);

  if (flag)
    quendi_respond(QUENYA_RESPONSE_TRANS_COMPLETE, NULL);
}

void quendi_send_spec(int dsock, const char* name)
{
  char spec_file[200];
  char buffer[100];
  unsigned length;
  void *spec;
  struct stat stat_buf;
  int fd;

  GetSpecFile(spec_file, name, NULL);

  if (stat(spec_file, &stat_buf))
    berror(fatal, "cannot stat spec file `%s'", spec_file);

  length = stat_buf.st_size;

  if ((fd = open(spec_file, O_RDONLY)) < 0)
    berror(fatal, "cannot open spec file `%s'", spec_file);

  snprintf(buffer, 100, "%i Sending Spec File", length);
  quendi_respond(QUENYA_RESPONSE_SENDING_SPEC, buffer);

  spec = balloc(fatal, length);

  if (read(fd, spec, length) < 0)
    berror(err, "failed to read from spec file");
  if (write(dsock, spec, length) < 0)
    berror(err, "failed to write to spec file");

  close(fd);
  bfree(fatal, spec);

  quendi_respond(QUENYA_RESPONSE_TRANS_COMPLETE, NULL);
}

void quendi_server_init(const struct quendi_server_data_t* server_data)
{
  quendi_server_data = server_data;
  quendi_respond(QUENYA_RESPONSE_SERVICE_READY, NULL);
}

void quendi_server_shutdown(void)
{
}

int quendi_stage_data(const char* file, unsigned long pos, int sufflen,
    int streamed_here, unsigned frame_size)
{
  char buffer[NAME_MAX + 60];
  char source[NAME_MAX];

  quendi_frame_size = frame_size;

  PathSplit_r(file, NULL, buffer);
  StaticSourcePart(source, buffer, NULL, sufflen);
  if (streamed_here) {
    snprintf(buffer, NAME_MAX + 60, "%lu:%s Data Continues: New Data Staged",
        pos, source);
    quendi_respond(QUENYA_RESPONSE_STAGED_NEXT, buffer);
  } else {
    snprintf(buffer, NAME_MAX + 60, "%lu:%s Data Staged", pos, source);
    quendi_respond(QUENYA_RESPONSE_DATA_STAGED, buffer);
  }
  return 1;
}
