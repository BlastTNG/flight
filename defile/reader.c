/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004-2005 D. V. Wiebe
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

#include <fcntl.h>      /* File control (open) */
#include <pthread.h>    /* POSIX threads (pthread_create, pthread_join) */
#include <string.h>     /* C string library (strcpy, strncpy)  */
#include <signal.h>     /* ANSI C signals (SIG(FOO), sigemptyset, sigaddset) */
#include <sys/stat.h>   /* SYSV stat (stat, struct stat S_IS(FOO)) */
#include <unistd.h>     /* UNIX std library (lseek, read, close) */

#include "blast.h"
#include "channels.h"
#include "defile.h"
#include "frameread.h"

#define FLAKEY_MAX 9      /* Maximum number of attempts to make with a flakey
                           * source */
#define FLAKEY_WAIT 10    /* Number of seconds to wait before retying a flakey
                             source */
#define FLAKEY(x) ((!rc.flakey_source || ++x >= FLAKEY_MAX) ? fatal : err)

void ReaderDone(int signo) {
  int i;

  bprintf(warning, "Caught deadly signal %d; exiting...\n", signo);
  ri.reader_done = 1;
  for (i = 0; i < 30; ++i) {
    sleep(1);
    if (ri.writer_done)
      pthread_exit(0);
  }
  bputs(err, "Timeout waiting for writer to exit.  Stop.\n");
  raise(SIGKILL);
}

void InitReader(void)
{
  if (rc.output_dirfile != NULL)
    strncpy(rc.dirfile, rc.output_dirfile, FILENAME_LEN);
  else
    GetDirFile(rc.dirfile, rc.chunk, rc.dest_dir, 0);

  /* Attempt to open the Specification file and read the channel lists */
  ReconstructChannelLists(rc.chunk, rc.spec_file);
  bprintf(info, "Frame size: %i bytes\n", DiskFrameSize);

}

void FrameFileReader(void)
{
  int fd = -1;
  int old_frame_size;
  char gpb[GPB_LEN];
  int i, n, fullframes, remainder = 0;
  int flakey_count = 0;
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

  /* enable signals -- they were blocked in main before this thread was
   * spawned */
  pthread_sigmask(SIG_UNBLOCK, &signals, NULL);

  InputBuffer[0] = (unsigned short*)balloc(fatal, DiskFrameSize
      * INPUT_BUF_SIZE);

  for (i = 1; i < INPUT_BUF_SIZE; ++i)
    InputBuffer[i] = (void*)InputBuffer[0] + i * DiskFrameSize;

  if (rc.resume_at >= 0) {
    seek_to = SetStartChunk(rc.resume_at, rc.chunk, rc.sufflen)
      * DiskFrameSize;
    ri.read = rc.resume_at;
  }

  do {
    if (new_chunk) {
      remainder = 0;
      if (!rc.silent)
        printf("\nDefiling chunk `%s'\n", rc.chunk);

      frames_read = 0;

      /* open the chunk */
      flakey_count = 0;
      while ((fd = open(rc.chunk, O_RDONLY)) < 0) {
        berror(FLAKEY(flakey_count), "cannot open `%s'", rc.chunk);
        bprintf(warning, "Sleeping for %i seconds...", FLAKEY_WAIT);
        sleep(FLAKEY_WAIT);
      }

      if (seek_to > 0) {
        lseek(fd, seek_to, SEEK_SET);
        seek_to = 0;
      }

      /* stat file to find its size */
      flakey_count = 0;
      while (stat(rc.chunk, &chunk_stat)) {
        berror(FLAKEY(flakey_count), "cannot stat `%s'", rc.chunk);
        bprintf(warning, "Sleeping for %i seconds...", FLAKEY_WAIT);
        sleep(FLAKEY_WAIT);
      }

      ri.chunk_total = chunk_stat.st_size / DiskFrameSize;
    }

    do {
      /* read some frames */
      if ((n = read(fd, (void*)InputBuffer[0] + remainder, DiskFrameSize
              * INPUT_BUF_SIZE - remainder)) < 1) {
        if (n == 0)
          break;
        else {
          berror(err, "error reading `%s'", rc.chunk);

          /* reopen file and try again */
          close(fd);

          flakey_count = 0;
          while ((fd = open(rc.chunk, O_RDONLY)) < 0) {
            berror(FLAKEY(flakey_count), "cannot open `%s'", rc.chunk);
            bprintf(warning, "Sleeping for %i seconds...", FLAKEY_WAIT);
            sleep(FLAKEY_WAIT);
          }

          /* seek to our last position */
          lseek(fd, frames_read * DiskFrameSize, SEEK_SET);
          n = 0;
        }
      }
      n += remainder;
      fullframes = n / DiskFrameSize;
      remainder = n % DiskFrameSize;
      frames_read += fullframes;

      for (i = 0; i < fullframes; ++i) {
        /* push frame */
        PushFrame(InputBuffer[i]);

        /* increment counter */
        ri.read++;
      }

      /* Copy remainder to the bottom of the buffer */
      if (fullframes > 0 && remainder > 0)
        memcpy(InputBuffer[0], InputBuffer[fullframes + 1], remainder);
    } while (n != 0);

    n = StreamToNextChunk(rc.persist, rc.chunk, rc.sufflen, &ri.chunk_total,
        rc.source, rc.curfile_val);

    if (n == FR_NEW_CHUNK) {
      close(fd);
      ri.old_total += ri.chunk_total;
      new_chunk = 1;
    } else if (n == FR_CURFILE_CHANGED) {
      /* fixup remounting */
      strcpy(gpb, rc.curfile_val);
      if (rc.remount)
        Remount(rc.source, gpb);

      strcpy(rc.chunk, gpb);

      /* Read the new Spec file */
      old_frame_size = DiskFrameSize;
      ReconstructChannelLists(rc.chunk, rc.spec_file);
      bprintf(info, "\nFrame size: %i bytes\n", DiskFrameSize);

      /* remake the destination dirfile (if necessary) */
      if (rc.output_dirfile == NULL) {
        GetDirFile(rc.dirfile, rc.chunk, rc.dest_dir, 0);

        /* Re-allocate InputBuffer */
        bfree(fatal, InputBuffer[0]);
        InputBuffer[0] = (unsigned short*)balloc(fatal, DiskFrameSize
            * INPUT_BUF_SIZE);

        for (i = 1; i < INPUT_BUF_SIZE; ++i)
          InputBuffer[i] = (void*)InputBuffer[0] + i * DiskFrameSize;

        /* if the dirfile has changed, signal the writer to cycle */
        ri.dirfile_init = 0;
        close(fd);
      } else if (old_frame_size != DiskFrameSize) {
        printf("Frame size has changed.  Unable to continue writing to "
            "current file.\n");
        n = FR_DONE;
      }

      ri.old_total += ri.chunk_total;
      new_chunk = 1;
    } else
      new_chunk = 0;

  } while (n != FR_DONE);

  ri.reader_done = 1;

  return;
}
