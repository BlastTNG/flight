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
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <tcpd.h>

#include <sys/socket.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "blast.h"
#include "quendi.h"
#include "frameread.h"

#define INPUT_BUF_SIZE 50 /* Frames are big (~1 kb) and we take a big
                           * performance hit if we read more than 64k at a
                           * time, so we keep this small */

/* internals */
const char _quendi_version[] = "1.0";
struct quendi_data *_quendi_server_data = NULL;

/* functions */
int quendi_access_ok(int level) {
  if (_quendi_server_data->access_level == 0) {
    quendi_respond(QUENYA_RESPONSE_NOT_IDENTIFIED, NULL);
    return 0;
  } else if (_quendi_server_data->access_level < level) {
    quendi_respond(QUENYA_RESPONSE_NO_ACCESS, NULL);
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
    case 0x6173796e: return QUENYA_COMMAND_ASYN;
    case 0x64617461: return QUENYA_COMMAND_DATA; 
    case 0x6964656e: return QUENYA_COMMAND_IDEN;
    case 0x6e6f6f70: return QUENYA_COMMAND_NOOP;
    case 0x6f70656e: return QUENYA_COMMAND_OPEN;
    case 0x716e6f77: return QUENYA_COMMAND_QNOW;
    case 0x71756974: return QUENYA_COMMAND_QUIT;
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

  for (overrun = eolfound = 0; !eolfound; ) {

    n = read(_quendi_server_data->csock, buffer, QUENDI_COMMAND_LENGTH);

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

int quendi_dp_open(void)
{
  struct sockaddr_in addr;
  int addrlen;
  int dsock;

  addrlen = sizeof(addr);
  getsockname(_quendi_server_data->csock, (struct sockaddr*)&addr, &addrlen);
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
  int addrlen;
  struct request_info req;

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

  i = accept(dsock, (struct sockaddr*)&addr, &addrlen);
  if (i == -1)
    syslog(LOG_WARNING, "dp accept: %m");

  /* tcp wrapper check */
  request_init(&req, RQ_DAEMON, _quendi_server_data->server_name, RQ_FILE,
      _quendi_server_data->csock, 0);
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
      case QUENYA_RESPONSE_LISTENING: /* 123 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Listening on Data Port for Connection\r\n", response_num);
        break;
      case QUENYA_RESPONSE_SENDING_DATA: /* 150 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Sending Data\r\n", response_num);
        break;
      case QUENYA_RESPONSE_SENDING_SPEC: /* 151 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Sending Spec File\r\n", response_num);
        break;
      case QUENYA_RESPONSE_SERVICE_READY: /* 220 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %s %s %s/%s Service Ready\r\n", response_num,
            _quendi_server_data->server_host, _quendi_server_data->server_name,
            _quendi_server_data->server_version, _quendi_version);
        break;
      case QUENYA_RESPONSE_GOODBYE: /* 221 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i %s Closing Connection\r\n", response_num,
            _quendi_server_data->server_host);
        break;
      case QUENYA_RESPONSE_PORT_OPENED: /* 222 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Opened\r\n", response_num);
        break;
      case QUENYA_RESPONSE_ACCESS_GRANTED: /* 222 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Access Granted\r\n", response_num);
        break;
      case QUENYA_RESPONSE_OK: /* 250 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH, "%i OK\r\n",
            response_num);
        break;
      case QUENYA_RESPONSE_DATA_STAGED: /* 251 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Staged\r\n", response_num);
        break; 
      case QUENYA_RESPONSE_TRANS_COMPLETE: /* 252 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Transfer Complete\r\n", response_num);
        break;
      case QUENYA_RESPONSE_OPEN_ERROR: /* 420 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Error Opening Data Port\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NO_CUR_DATA: /* 450 */
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
      case QUENYA_RESPONSE_PORT_NOT_OPEN: /* 520 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Data Port Not Open\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NOT_IDENTIFIED: /* 530 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Client Not Identified\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NO_ACCESS: /* 531 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i Insufficient Privilege\r\n", response_num);
        break;
      case QUENYA_RESPONSE_NO_DATA_STAGED: /* 550 */
        size = snprintf(buffer, QUENDI_RESPONSE_LENGTH,
            "%i No Data Staged\r\n", response_num);
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

  nparams = 0;

  if (cmd == QUENYA_COMMAND_IDEN) {
    if (quendi_get_next_param(buffer + 4, nparams, params))
      return -2;
  }

  return cmd;
}

int quendi_respond(int response_num, const char *message)
{
  char buffer[QUENDI_RESPONSE_LENGTH + 1];

  if (quendi_make_response(buffer, response_num, message) == NULL)
    return -1;

  write(_quendi_server_data->csock, buffer, strlen(buffer));

  return 0;
}

void quendi_send_data(int dsock, const char* name, unsigned long pos,
    unsigned frame_size, int sufflen, int persist)
{
  FILE* stream = NULL;
  int new_chunk = 1;
  long int seek_to = 0;
  char* chunk = bstrdup(fatal, name);
  int chunk_total = 0;
  struct stat chunk_stat;
  int i, n;
  long int frames_read = 0;
  char buffer[100];

  unsigned short* InputBuffer[INPUT_BUF_SIZE];

  InputBuffer[0] = (unsigned short*)balloc(fatal, frame_size * INPUT_BUF_SIZE);

  for (i = 1; i < INPUT_BUF_SIZE; ++i)
    InputBuffer[i] = (void*)InputBuffer[0] + i * frame_size;

  seek_to = SetStartChunk(pos, chunk, sufflen) * frame_size;

  snprintf(buffer, 100, "%i Data Transfer Starts", frame_size);
  quendi_respond(QUENYA_RESPONSE_SENDING_DATA, buffer);

  do {
    if (new_chunk) {
      if ((stream = fopen(chunk, "r")) == NULL)
        berror(fatal, "cannot open `%s'", chunk);

      if (seek_to > 0) {
        fseek(stream, seek_to, SEEK_SET);
        seek_to = 0;
      }

      if (stat(chunk, &chunk_stat))
        berror(fatal, "cannot stat `%s'", chunk);

      chunk_total = chunk_stat.st_size / frame_size;
    }

    do {
      clearerr(stream);
      if ((n = fread(InputBuffer[0], frame_size, INPUT_BUF_SIZE, stream)) < 1) {
        if (feof(stream))
          break;
        else if ((i = ferror(stream))) {
          berror(err, "error reading `%s' (%i)", chunk, i);

          fclose(stream);
          if ((stream = fopen(chunk, "r")) == NULL)
            berror(fatal, "cannot open `%s'", chunk);

          fseek(stream, frames_read * frame_size, SEEK_SET);
          n = 0;
        }
      }
      frames_read += n;

      for (i = 0; i < n; ++i)
        write(dsock, InputBuffer[i], frame_size);

    } while (!feof(stream));

    n = StreamToNextChunk(persist, chunk, sufflen, &chunk_total, NULL, NULL);

    if (n == FR_NEW_CHUNK) {
      fclose(stream);
      new_chunk = 1;
    } else if (n == FR_CURFILE_CHANGED) {
      /* do something */;
    } else
      new_chunk = 0;

  } while (n != FR_DONE);

  bfree(fatal, InputBuffer);
  bfree(fatal, chunk);

  quendi_respond(QUENYA_RESPONSE_TRANS_COMPLETE, NULL);

  return;
}

void quendi_send_spec(int dsock, const char* name, unsigned long pos)
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

  read(fd, spec, length);
  write(dsock, spec, length);

  close(fd);
  bfree(fatal, spec);

  quendi_respond(QUENYA_RESPONSE_TRANS_COMPLETE, NULL);
}

void quendi_server_init(struct quendi_data* server_data)
{
  _quendi_server_data = server_data;
  quendi_respond(QUENYA_RESPONSE_SERVICE_READY, NULL);
}

void quendi_server_shutdown(void)
{
}

int quendi_stage_data(const char* file, unsigned long pos, int sufflen)
{
  char buffer[NAME_MAX + 60];
  char source[NAME_MAX];

  PathSplit_r(file, NULL, buffer);
  StaticSourcePart(source, buffer, NULL, sufflen);
  snprintf(buffer, NAME_MAX + 60, "%lu:%s Data Staged", pos, source);
  quendi_respond(QUENYA_RESPONSE_DATA_STAGED, buffer);
  return 1;
}
