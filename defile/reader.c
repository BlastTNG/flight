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
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>

#include "channels.h"
#include "defile.h"
#include "frameread.h"

#define INPUT_BUF_SIZE 50 /* Frames are big (~1 kb) and we take a big
                           * performance hit if we read more than 64k at a
                           * time, so we keep this small */

extern sigset_t signals;

void ReaderDone(int signo) {
  fprintf(stderr, "\nCaught signal %d; exiting...\n", signo);
  ri.reader_done = 1;
  pthread_exit(0);
}

void FrameFileReader(void)
{
  FILE *stream = NULL;
  char gpb[GPB_LEN];
  int i, n;
  int frames_read = 0;
  int new_chunk = 1;
  unsigned short* InputBuffer[INPUT_BUF_SIZE];
  struct stat chunk_stat;
  long int seek_to = 0;

  struct sigaction action;

  /* set up signal masks */
  sigemptyset(&signals);
  sigaddset(&signals, SIGHUP);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);

  /* set up signal handlers */
  action.sa_handler = ReaderDone;
  action.sa_mask = signals;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGHUP, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  /* enable signals */
  pthread_sigmask(SIG_UNBLOCK, &signals, NULL);

  if ((InputBuffer[0] = (unsigned short*)malloc(DiskFrameSize
          * INPUT_BUF_SIZE)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }                 

  for (i = 1; i < INPUT_BUF_SIZE; ++i)
    InputBuffer[i] = (void*)InputBuffer[0] + i * DiskFrameSize;

  if (rc.resume_at >= 0) {
    ri.read = SetStartChunk(rc.resume_at, rc.chunk, rc.sufflen);
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

      /* stat file to find its size */
      if (stat(rc.chunk, &chunk_stat)) {
        snprintf(gpb, GPB_LEN, "defile: cannot stat `%s'", rc.chunk);
        perror(gpb);
        exit(1);
      }

      ri.chunk_total = chunk_stat.st_size / DiskFrameSize;
    }

    do {
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
        /* push frame */
        PushFrame(InputBuffer[i]);

	/* increment counter */
	ri.read++;
      }
    } while (!feof(stream));

    n = StreamToNextChunk(rc.persist, rc.chunk, rc.sufflen, &ri.chunk_total,
            rc.source, rc.curfile_val);

    if (n == FR_NEW_CHUNK) {
      fclose(stream);
      ri.old_total += ri.chunk_total;
      new_chunk = 1;
    } else if (n == FR_CURFILE_CHANGED) {
      /* fixup remounting */
      strcpy(gpb, rc.curfile_val);
      if (rc.remount)
        Remount(rc.source, gpb);

      strcpy(rc.chunk, gpb);

      /* remake the destination dirfile (if necessary) */
      if (rc.output_dirfile == NULL) {
        GetDirFile(rc.dirfile, rc.chunk, rc.dest_dir);

        /* if the dirfile has changed, signal the writer to cycle */
        ri.dirfile_init = 0;
        fclose(stream);
      }

      ri.old_total += ri.chunk_total;
      new_chunk = 1;
    } else 
      new_chunk = 0;

  } while (n != FR_DONE);

  ri.reader_done = 1;

  return;
}
