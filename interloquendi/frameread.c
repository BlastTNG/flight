/* frameread: reads mcp-style framefiles
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * frameread is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * frameread is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with frameread; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "frameread.h"
#include "tx_struct.h"

/* given a source filename, fills in the part of it which is static from chunk
 * to chunk, the value of the counter, returns the length of the non-static
 * suffix */
int StaticSourcePart(char* output, const char* source, chunkindex_t* value,
    int sufflen)
{
  char* buffer;
  char* ptr;
  int counter = 0;
  long number = 0;

  if ((buffer = strdup(source)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  /* walk backwards through source looking for first non-hex digit */
  for (ptr = buffer + strlen(buffer) - 1; counter < sufflen && ptr != buffer;
      --ptr)
    if (*ptr >= '0' && *ptr <= '9') {
      number += (*ptr - '0') << 4 * counter++;
      *ptr = '\0';
    } else if (*ptr >= 'a' && *ptr <= 'f') {
      number += (*ptr - 'a' + 10) << 4 * counter++;
      *ptr = '\0';
    } else if (*ptr >= 'A' && *ptr <= 'F') {
      number += (*ptr - 'A' + 10) << 4 * counter++;
      *ptr = '\0';
    } else
      break;

    if (value != NULL)
      *value = number;
    strcpy(output, buffer);

    free(buffer);

    return counter;
}

int GetNextChunk(char* chunk, int sufflen)
{
  char* buffer;
  char* newchunk;
  int s;
  chunkindex_t chunknum;
  struct stat chunk_stat;

  /* allocate our buffers */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }
  if ((newchunk = (char*)malloc(FILENAME_LEN)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  /* get current chunk name */
  s = StaticSourcePart(buffer, chunk, &chunknum, sufflen);

  /* if incrementing chunknum causes it to wrap around, we're out of space
   * on our suffix -- no more chunks are possible */
  if (chunknum + 1 < chunknum) {
    free(newchunk);
    free(buffer);
    return 0;
  }

  /* if incrementing chunknum causes it to be more than sufflen bytes,
   * we're out of space on our suffix -- no more chunks are possible */
  if (chunknum + 1 >= (chunkindex_t)1 << (4 * sufflen)) {
    free(newchunk);
    free(buffer);
    return 0;
  }
  
  /* generate new filename */
  snprintf(newchunk, FILENAME_LEN, "%s%0*llX", buffer, s,
      (unsigned long long)(chunknum + 1));

  /* stat it to see if it exists */
  if (stat(newchunk, &chunk_stat)) {
    free(newchunk);
    free(buffer);
    return 0;
  }

  /* stat worked, it's our new chunk */
  strcpy(chunk, newchunk);

  free(newchunk);
  free(buffer);

  return 1;
}

/* find the filename and position of the place where we're supposed to start */
long int SetResumeChunk(long int resume_at, char* chunk, int sufflen)
{
  long int left_to_read = resume_at;
  int chunk_total;
  int new_chunk;
  struct stat chunk_stat;
  char gpb[GPB_LEN];

  /* Loop until we get to the right chunk */
  for (;;) {
    /* Stat the current chunk file to get its size */
    if (stat(chunk, &chunk_stat)) {
      snprintf(gpb, GPB_LEN, "defile: cannot stat `%s'", chunk);
      perror(gpb);
      exit(1);
    }

    chunk_total = chunk_stat.st_size / DiskFrameSize;

    /* if there's more than we need, we're done */
    if (chunk_total > left_to_read)
      return left_to_read;

    /* Otherwise, try to get a new chunk */
    if ((new_chunk = GetNextChunk(chunk, sufflen)) == 0) {
      /* no new chunk -- complain and exit */
      fprintf(stderr, "defile: source file is smaller than destination.\n"
          "defile: cannot resume.\n");
      exit(1);
    }

    /* there is another chunk, decrement the total needed and try again */
    left_to_read -= chunk_total;
  }
}
