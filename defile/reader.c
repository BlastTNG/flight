/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004 D. V. Wiebe
 * 
 * This file is part of defile.
 * 
 * defile is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * defile is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with defile; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include "tx_struct.h"
#include "defile.h"
#include "frameread.h"

#define INPUT_BUF_SIZE 50 /* Frames are big (~1 kb) and we take a big
                           * performance hit if we read more than 64k at a
                           * time, so we keep this small */

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
  s = StaticSourcePart(buffer, chunk, &chunknum);

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

void FrameFileReader(void)
{
  FILE *stream = NULL;
  FILE *curfile = NULL;
  char gpb[GPB_LEN];
  int i, n;
  int frames_read = 0;
  int new_chunk = 1;
  int more_in_file = 0;
  unsigned short* InputBuffer[INPUT_BUF_SIZE];
  struct stat chunk_stat;
  long int seek_to = 0;

  if ((InputBuffer[0] = (unsigned short*)malloc(DiskFrameSize
          * INPUT_BUF_SIZE)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }                 

  for(i = 1; i < INPUT_BUF_SIZE; ++i)
    InputBuffer[i] = (void*)InputBuffer[0] + i * DiskFrameSize;

  if (rc.resume_at >= 0) {
    ri.read = SetResumeChunk(rc.resume_at, rc.chunk, rc.sufflen);
    seek_to = ri.read * DiskFrameWords;
  }

  do {
    if (new_chunk) {
      printf("\nDefiling chunk `%s'\n", rc.chunk);

      frames_read = 0;

      /* open the chunk */
      if ((stream = fopen(rc.chunk, "r")) == NULL) {
        snprintf(gpb, GPB_LEN, "defile: cannot open `%s'", rc.chunk);
        perror(gpb);
        exit(1);
      }

      if (seek_to > 0) {
        fseek(stream, seek_to, SEEK_SET);
        seek_to = 0;
      }
    }

    do {
      if (!more_in_file) {
        /* stat file to find its size */
        if (stat(rc.chunk, &chunk_stat)) {
          snprintf(gpb, GPB_LEN, "defile: cannot stat `%s'", rc.chunk);
          perror(gpb);
          exit(1);
        }

        ri.chunk_total = chunk_stat.st_size / DiskFrameSize;
      }

      /* read some frames */
      clearerr(stream);
      if ((n = fread(InputBuffer[0], DiskFrameSize, INPUT_BUF_SIZE, stream))
          < 1) {
        if (feof(stream))
          break;
        else if ((i = ferror(stream))) {
          snprintf(gpb, GPB_LEN, "defile: error reading `%s' (%i)",
              rc.chunk, errno);
          perror(gpb);

          /* reopen file and try again */
          fclose(stream);
          if ((stream = fopen(rc.chunk, "r")) == NULL) {
            snprintf(gpb, GPB_LEN, "defile: cannot open `%s'", rc.chunk);
            perror(gpb);
            exit(1);
          }

          /* seek to our last position */
          fseek(stream, frames_read * DiskFrameSize, SEEK_SET);
          n = 0;
        }
      }
      frames_read += n;

      for (i = 0; i < n; ++i) {
        /* increment counter */
        ri.read++;

        /* push frame */
        PushFrame(InputBuffer[i]);
      }
    } while (!feof(stream));

    more_in_file = 0;

    if (rc.persist) {
      do {
        more_in_file = new_chunk = 0;
        /* persistent: first check to see if we have more data in the file */
        if (stat(rc.chunk, &chunk_stat)) {
          snprintf(gpb, GPB_LEN, "defile: cannot stat `%s'", rc.chunk);
          perror(gpb);
          exit(1);
        }

        /* new frame total */
        n = chunk_stat.st_size / DiskFrameSize;
        if (n < ri.chunk_total)
          fprintf(stderr, "defile: warning: chunk `%s' has shrunk.\n",
              rc.chunk);

        if (n > ri.chunk_total) {
          more_in_file = 1;
          ri.chunk_total = n;
        } else {
          /* nothing more in file, check to see if we have a new chunk */
          if ((new_chunk = GetNextChunk(rc.chunk, rc.sufflen))) {
            fclose(stream);
            ri.old_total += ri.chunk_total;
          } else
            /* no new chunk either, check for a change in SOURCE curfile if
             * we're using one */
            if (rc.source_is_curfile) {
              if ((curfile = fopen(rc.source, "r")) == NULL) {
                snprintf(gpb, GPB_LEN, "defile: cannot open `%s'", rc.source);
                perror(gpb);
                exit(1);
              }

              fgets(gpb, PATH_MAX, curfile);

              fclose(curfile);

              i = strlen(gpb) - 1;
              if (gpb[i] == '\n') {
                gpb[i] = '\0';
                if (strcmp(gpb, rc.curfile_val) != 0) {
                  /* curfile has changed, reinitialise source */
                  strcpy(rc.curfile_val, gpb);

                  /* fixup remounting */
                  if (rc.remount)
                    Remount(rc.source, gpb);

                  strcpy(rc.chunk, gpb);

                  /* remake the destination dirfile (if necessary) */
                  if (rc.output_dirfile == NULL) {
                    strcpy(gpb, rc.dirfile);
                    GetDirFile(rc.dirfile, rc.chunk, rc.dest_dir);

                    /* if the dirfile has changed, signal the writer to cycle */
                    ri.dirfile_init = 0;
                    fclose(stream);
                  }

                  ri.old_total += ri.chunk_total;
                  new_chunk = 1;
                } else
                  /* no changes wait and try again */
                  usleep(10000);
              }
            }
        }
      } while (!more_in_file && !new_chunk);
    } else {
      if ((new_chunk = GetNextChunk(rc.chunk, rc.sufflen))) {
        fclose(stream);
        ri.old_total += ri.chunk_total;
      }
    }
  } while (more_in_file || new_chunk);

  ri.reader_done = 1;

  return;
}
