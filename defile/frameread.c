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
#include <syslog.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdarg.h>

#include "blast.h"
#include "frameread.h"
#include "channels.h"

/* splits path into dname and bname */
void PathSplit_r(const char* path, char* dname, char* bname)
{
  char the_base[NAME_MAX];
  char the_path[PATH_MAX];
  char* base = NULL, *ptr;
  char* buffer;

  if ((buffer = strdup(path)) == NULL)
    berror(fatal, "strdup");

  for (ptr = buffer; *ptr != '\0'; ++ptr)
    if (*ptr == '/')
      base = ptr + 1;

  if (base == NULL) { /* this is "foo" */
    strncpy(the_base, buffer, NAME_MAX);
    strcpy(the_path, ".");
  } else if (base == buffer + 1) {
    if (base[0] == '\0') { /* this is "/" */
      strncpy(the_path, buffer, NAME_MAX);
      strcpy(the_base, ".");
    } else { /* this is "/foo" */
      strcpy(the_path, "/");
      strncpy(the_base, base, NAME_MAX);
    }
  } else { /* this is "foo/bar" */
    *(base - 1) = '\0';
    strncpy(the_base, base, NAME_MAX);
    strncpy(the_path, buffer, PATH_MAX);
  }

  if (dname != NULL)
    strcpy(dname, the_path);

  if (bname != NULL)
    strcpy(bname, the_base);

  free(buffer);
}

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

  if ((buffer = strdup(source)) == NULL)
    berror(fatal, "strdup");

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

/* Returns the length of a framefile */
unsigned long GetFrameFileSize(const char* file, int sufflen)
{
  char *chunk = strdup(file);
  struct stat chunk_stat;
  unsigned long length = 0;

  /* stat it to see if it exists */
  if (stat(chunk, &chunk_stat) == 0) {
    length = chunk_stat.st_size / DiskFrameSize;

    while (GetNextChunk(chunk, sufflen))
      if (stat(chunk, &chunk_stat) == 0)
        length += chunk_stat.st_size / DiskFrameSize;
  }

  free(chunk);

  return length;
}

/* Increments the chunk name.  Returns true on success. On failure returns
 * false and chunk isn't changed */
int GetNextChunk(char* chunk, int sufflen)
{
  char* buffer;
  char* newchunk;
  int s;
  chunkindex_t chunknum;
  struct stat chunk_stat;

  /* allocate our buffers */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL) 
    berror(fatal, "malloc");

  if ((newchunk = (char*)malloc(FILENAME_LEN)) == NULL)
    berror(fatal, "malloc");

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
long int SetStartChunk(long int framenum, char* chunk, int sufflen)
{
  long int left_to_read = framenum;
  int chunk_total;
  int new_chunk;
  struct stat chunk_stat;
  char gpb[GPB_LEN];

  /* Loop until we get to the right chunk */
  for (;;) {
    /* Stat the current chunk file to get its size */
    if (stat(chunk, &chunk_stat)) {
      snprintf(gpb, GPB_LEN, "stat `%s'", chunk);
      berror(fatal, gpb);
    }

    chunk_total = chunk_stat.st_size / DiskFrameSize;

    /* if there's more than we need, we're done */
    if (chunk_total > left_to_read)
      return left_to_read;

    /* Otherwise, try to get a new chunk */
    if ((new_chunk = GetNextChunk(chunk, sufflen)) == 0)
#ifdef __DEFILE__
      /* no new chunk -- complain and exit */
      bprintf(fatal, "source file is smaller than destination.\n"
          "cannot resume.\n");
#else
    /* start at end of last chunk */
    return chunk_total;
#endif

    /* there is another chunk, decrement the total needed and try again */
    left_to_read -= chunk_total;
  }
}

int StreamToNextChunk(int keepalive, char* chunk, int sufflen, int *chunk_total,
    const char* curfile_name, char* curfile_val)
{
  FILE *curfile = NULL;
  struct stat chunk_stat;
  char gpb[GPB_LEN];
  int n, i;

  if (keepalive) {
    for (;;) {
      /* persistent: first check to see if we have more data in the file */
      if (stat(chunk, &chunk_stat)) {
        snprintf(gpb, GPB_LEN, "stat `%s'", chunk);
        berror(fatal, gpb);
      }

      /* new frame total */
      n = chunk_stat.st_size / DiskFrameSize;
      if (n < *chunk_total)
        bprintf(err, "chunk `%s' has shrunk.", chunk);

      if (n > *chunk_total) {
        *chunk_total = n;
        return FR_MORE_IN_FILE;
      }

      /* nothing more in file, check to see if we have a new chunk */
      if (GetNextChunk(chunk, sufflen))
        return FR_NEW_CHUNK;

      /* no new chunk either, check for a change in SOURCE curfile if
       * we're using one */
      if (curfile_name != NULL) {
        if ((curfile = fopen(curfile_name, "r")) == NULL) {
          snprintf(gpb, GPB_LEN, "open `%s'", curfile_name);
          berror(fatal, gpb);
        }

        fgets(gpb, PATH_MAX, curfile);

        fclose(curfile);

        i = strlen(gpb) - 1;
        if (gpb[i] == '\n') {
          gpb[i] = '\0';
          if (strcmp(gpb, curfile_val) != 0) {
            /* curfile has changed, reinitialise source */
            strcpy(curfile_val, gpb);
            return FR_CURFILE_CHANGED;
          }
        }
      }

      /* no changes wait and try again */
      usleep(10000);
    }
  } else
    return (GetNextChunk(chunk, sufflen)) ? FR_NEW_CHUNK : FR_DONE;
}
