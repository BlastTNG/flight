/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2018-2019 Javier Romualdez, University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/io.h>
#include <string.h>
#include <time.h>

#include "log.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifdef __cplusplus
extern "C" {
#endif

void initLogger(struct LOGGER * log, char * filename, int size)
{
  closeLogger(log);

  log->f = fopen(filename, "r");

  if (log->f != NULL) {
    log->buffer = (char *) calloc(1, MAX(2048, size));
    log->loc = 0;
    log->blksize = size;
    log->n = 0;
    log->isinit = 1;

    // seek to the end of the file
    resetLogger(log);
  }
}
int readLogger(struct LOGGER * log, char * buffer)
{
  if (!log->isinit) return -1;

  int cpy = 0;
  if (log->loc < log->n) { // data in the buffer still
    memset(buffer, 0, log->blksize); // reset
    cpy = MIN((int) log->n-log->loc, (int) log->blksize); // amount to cpy

    // printf("%d %d %d %d\n",log->loc, log->n, log->cpy, log->blksize);
    memcpy(buffer, log->buffer+log->loc, cpy);
    log->loc += cpy;
  } else { // data can be read from file
    size_t lenl = 2048;
    memset(buffer, 0, log->blksize); // reset again
    if (log->f != NULL) {
      fflush(log->f); // flush the buffer
      log->n = getline(&log->buffer, &lenl, log->f);
    }
    log->loc = 0;
  }
  return cpy;
}

void resetLogger(struct LOGGER * log)
{
  if (log->isinit) {
    fseek(log->f, 0, SEEK_END);
  }
}

void closeLogger(struct LOGGER * log)
{
  if (log->f) fclose(log->f);
  if (log->buffer) free(log->buffer);
  log->isinit = 0;
}

#ifdef __cplusplus
}
#endif

