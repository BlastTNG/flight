/* frameread: reads mcp-style framefiles
 *
 * This software is copyright (C) 2004-2005 University of Toronto
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

#include <string.h>     /* ANSI C strings (strlen, strncpy, strcmp, &c.)  */
#include <sys/stat.h>   /* SYSV stat (stat, struct stat S_IS(FOO)) */
#include <unistd.h>     /* UNIX std library (read, write, close, sleep) */

#include "blast.h"
#include "frameread.h"
#include "channels.h"

/* splits path into dname and bname */
void PathSplit_r(const char* path, char* dname, char* bname)
{
  char the_base[NAME_MAX];
  char the_path[FR_PATH_MAX];
  char* base = NULL, *ptr;
  char* buffer;

  buffer = bstrdup(fatal, path);

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
    strncpy(the_path, buffer, FR_PATH_MAX);
  }

  if (dname != NULL)
    strcpy(dname, the_path);

  if (bname != NULL)
    strcpy(bname, the_base);

  bfree(fatal, buffer);
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

  buffer = bstrdup(fatal, source);

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

  bfree(fatal, buffer);

  return counter;
}

/* Figures out the name of the channel specification file, and then tries to
 * open and read it. */
char* GetSpecFile(char* buffer, const char* chunk, const char *spec_file)
{
  struct stat stat_buf;
  char* ptr;

  /* if spec_file exists, the user has specified a spec file name, use it */
  if (spec_file != NULL) {
    /* check for buffer overrun */
    if (strlen(spec_file) >= 200)
      blast_fatal("specification file path too long\n");
    strcpy(buffer, spec_file);
  } else {
    /* if the chunk is 923488378.x000, the spec file will be 923488378.x.spec */
    strcpy(buffer, chunk);
    ptr = buffer + strlen(buffer);
    while (*ptr != '.' && ptr != buffer)
      --ptr;
    if (ptr == buffer)
      ptr += strlen(buffer);
    if (*ptr != '\0') {
      ++ptr;
      if (*ptr != '\0')
        ++ptr;
    }

    /* check for buffer overrun */
    if (ptr - buffer > 190)
      blast_fatal("specification file path too long\n");
    strcpy(ptr, ".spec");
  }

  /* first attempt to stat spec file to see if it is indeed a regular file */
  if (stat(buffer, &stat_buf))
    berror(fatal, "cannot stat spec file `%s'", buffer);

  /* the stat worked.  Now is this a regular file? */
  if (!S_ISREG(stat_buf.st_mode))
    blast_fatal("spec file `%s' is not a regular file\n", buffer);

  return buffer;
}

/* Read spec file and make channel lists */
int ReconstructChannelLists(const char* chunk, const char *spec_file)
{
  char buffer[200];
  FILE* stream;

  /* attempt to open the file */
  if ((stream = fopen(GetSpecFile(buffer, chunk, spec_file), "r")) == NULL)
    berror(fatal, "cannot open spec file `%s'", buffer);

  ReadSpecificationFile(stream);

  fclose(stream);

  /* Make the Channel Struct */
  MakeAddressLookups(NULL);

  return DiskFrameSize;
}

/* Returns the length of a framefile */
unsigned long long GetFrameFileSize(const char* file, int sufflen)
{
  char *chunk = bstrdup(fatal, file);
  struct stat chunk_stat;
  unsigned long long length = 0;

  /* stat it to see if it exists */
  if (stat(chunk, &chunk_stat) == 0) {
    length = chunk_stat.st_size;

    while (GetNextChunk(chunk, sufflen))
      if (stat(chunk, &chunk_stat) == 0)
        length += chunk_stat.st_size;
  }

  bfree(fatal, chunk);

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
  buffer = (char*)balloc(fatal, FILENAME_LEN);
  newchunk = (char*)balloc(fatal, FILENAME_LEN);

  /* get current chunk name */
  s = StaticSourcePart(buffer, chunk, &chunknum, sufflen);

  /* if incrementing chunknum causes it to wrap around, we're out of space
   * on our suffix -- no more chunks are possible */
  if (chunknum + 1 < chunknum) {
    bfree(fatal, newchunk);
    bfree(fatal, buffer);
    return 0;
  }

  /* if incrementing chunknum causes it to be more than sufflen bytes,
   * we're out of space on our suffix -- no more chunks are possible */
  if (chunknum + 1 >= (chunkindex_t)1 << (4 * sufflen)) {
    bfree(fatal, newchunk);
    bfree(fatal, buffer);
    return 0;
  }

  /* generate new filename */
  snprintf(newchunk, FILENAME_LEN, "%s%0*llX", buffer, s,
      (unsigned long long)(chunknum + 1));

  /* stat it to see if it exists */
  if (stat(newchunk, &chunk_stat)) {
    bfree(fatal, newchunk);
    bfree(fatal, buffer);
    return 0;
  }

  /* stat worked, it's our new chunk */
  strcpy(chunk, newchunk);

  bfree(fatal, newchunk);
  bfree(fatal, buffer);

  return 1;
}

/* find the filename and position of the place where we're supposed to start */
long int SetStartChunk(long int framenum, char* chunk, int sufflen)
{
  long int left_to_read = framenum;
  int chunk_total;
  int new_chunk;
  struct stat chunk_stat;

  /* Loop until we get to the right chunk */
  for (;;) {
    /* Stat the current chunk file to get its size */
    if (stat(chunk, &chunk_stat)) {
      berror(fatal, "stat `%s'", chunk);
    }

    chunk_total = chunk_stat.st_size / DiskFrameSize;

    /* if there's more than we need, we're done */
    if (chunk_total > left_to_read)
      return left_to_read;

    /* Otherwise, try to get a new chunk */
    if ((new_chunk = GetNextChunk(chunk, sufflen)) == 0) {
#ifdef __DEFILE__
      /* no new chunk -- complain and exit */
      blast_fatal("source file is smaller than destination.\n"
          "cannot resume.\n");
#else
      /* start at end of last chunk */
      return chunk_total;
#endif
    }

    /* there is another chunk, decrement the total needed and try again */
    left_to_read -= chunk_total;
  }
}

int StreamToNextChunk(int keepalive, char* chunk, int sufflen, int *chunk_total,
    const char* curfile_name, char* curfile_val)
{
  FILE *curfile = NULL;
  char* new_chunk;
  struct stat chunk_stat;
  char gpb[GPB_LEN];
  int n, i;

  if (keepalive) {
    for (;;) {
      /* persistent: first check to see if we have more data in the file */
      if (stat(chunk, &chunk_stat)) {
        berror(fatal, "stat `%s'", chunk);
      }

      /* new frame total */
      n = chunk_stat.st_size / DiskFrameSize;
      if (n < *chunk_total)
        blast_err("chunk `%s' has shrunk.", chunk);

      /* Don't read the last frame in the file */
      if (n - 75 > *chunk_total) {
        *chunk_total = n - 75;
        return FR_MORE_IN_FILE;
      }

      /* nothing more in file, check to see if we have a new chunk */
      new_chunk = bstrdup(fatal, chunk);

      if (GetNextChunk(new_chunk, sufflen)) {
        if (n > *chunk_total) {
          bfree(fatal, new_chunk);
          *chunk_total = n;
          return FR_MORE_IN_FILE;
        } else {
          strcpy(chunk, new_chunk);
          bfree(fatal, new_chunk);
          return FR_NEW_CHUNK;
        }
      }

      bfree(fatal, new_chunk);

      /* no new chunk either, check for a change in SOURCE curfile if
       * we're using one */
      if (curfile_val != NULL) {
        if ((curfile = fopen(curfile_name, "r")) == NULL) {
          berror(fatal, "open `%s'", curfile_name);
        }

        if (fgets(gpb, FR_PATH_MAX, curfile) == NULL)
	  blast_fatal("Failed to read curfile");

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
