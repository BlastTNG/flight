/* framefile.c: writes raw frame streams to disk
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "blast.h"
#include "channels.h"
#ifdef __MCP__
#include "mcp.h"
#include "command_struct.h"
#endif

#ifdef __DECOMD__
#  define CURFILE "/mnt/decom/etc/decom.cur"
#  define RAWDIR  "/mnt/decom/rawdir"
#else
#  define CURFILE "/data/etc/datafile.cur"
#  define RAWDIR  "/data/rawdir"
#endif

unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

static int shutdown_now = 0;

#ifdef __MCP__
void nameThread(const char*);	/* mcp.c */
#endif
void WriteSpecificationFile(FILE*); /* in tx_struct.c */

struct file_info {
  int fd;            /* current file descriptor */
  int chunk;         /* current chunk number */
  int frames;        /* number of frames writen so far to current chunk */
  char type;         /* run type */
  time_t time;       /* file timestamp */
  char name[200];    /* filename to write to */
  void* buffer;      /* frame buffer */
  void* b_write_to;  /* buffer write-to pointer */
  void* b_read_from; /* buffer read-from pointer */
  void* buffer_end;  /* end of frame buffer */
} framefile;

#define BUFFER_SIZE 3000 /* 30 seconds of frames */
#define FRAMES_PER_FILE 90000 /* 15 minutes of data per file */

/* OpenNextChunk: closes the currently open chunk (if any), increments the
 * chunk index and opens creates the next chunk */
static void OpenNextChunk(void)
{
  if (framefile.fd > -1)
    if (close(framefile.fd) == -1)
      berror(err, "Error closing chunk");

  sprintf(framefile.name, RAWDIR "/%lu.%c%03X%c", framefile.time,
      framefile.type, ++framefile.chunk, '\0');

  bprintf(info, "Writing to %s\n", framefile.name);

  if ((framefile.fd = creat(framefile.name, 0644)) == -1)
    berror(err, "Error opening chunk");

  framefile.frames = 0;
}

/* This function is only used by the decom daemon */
void ShutdownFrameFile(void)
{
  shutdown_now = 1;
}

/*********************************************************************/
/*                                                                   */
/*     Initialize framefile                                          */
/*                                                                   */
/*********************************************************************/
void InitialiseFrameFile(char type)
{
  FILE* fp;
  char buffer[200];

  shutdown_now = 0;

  /* filename */
  framefile.fd = framefile.chunk = -1;
  framefile.type = type;
  framefile.time = time(NULL);
  OpenNextChunk();
  sprintf(buffer, RAWDIR "/%lu.%c.spec", framefile.time, framefile.type);

  if ((fp = fopen(buffer,"w")) == NULL)
    berror(err, "Unable to write spec file");
  else {
    WriteSpecificationFile(fp);
    fclose(fp);
  }

  /* alloc frame buffer */
  framefile.buffer = balloc(tfatal, BUFFER_SIZE * DiskFrameSize);
  framefile.buffer_end = framefile.buffer + BUFFER_SIZE * DiskFrameSize;
  framefile.b_write_to = framefile.b_read_from = framefile.buffer;

  fp = fopen(CURFILE,"w");
  if (fp == NULL) {
    berror(err, "Error opening curfile");
    return;
  }

  /* defile likes the newline */
  fprintf(fp, "%s\n", framefile.name);

  if (fclose(fp) < 0)
    berror(err, "Error while closing curfile");
}

static void* advance_in_buffer(void* ptr)
{
  void* tmp;
  tmp = ((char*)ptr + DiskFrameSize);
  return (tmp >= framefile.buffer_end) ? framefile.buffer : (void*)tmp;
}

/*************************************************************/
/* pushDiskFrame: called from main thread: puts rxframe into */
/* individual buffers                                        */
/*************************************************************/
void pushDiskFrame(unsigned short *RxFrame)
{
  void* new_write_to = advance_in_buffer(framefile.b_write_to);

#ifdef __MCP__
  int i_slow, i_mindex;

  /*******************************************************************/
  /* fill the Rx slow data from the MCP internal slow data structure */
  /*******************************************************************/
  i_mindex = RxFrame[3];
  if (i_mindex < FAST_PER_SLOW) {
    for (i_slow = 0; i_slow < slowsPerBi0Frame; i_slow++) {
      RxFrame[SLOW_OFFSET + i_slow] = slow_data[i_mindex][i_slow];
    }
  }
#endif

  /* ****************************************************************** */
  /* First make sure there is enough space in the buffer                */
  /* We discard the full frame if there is no space                     */
  /* ****************************************************************** */

  if (new_write_to == framefile.b_read_from) {
    bputs(warning, "Buffer overflow (frame discarded)\n");
    return;
  }	

  /*********************/
  /* SHIP OUT RX FRAME */
  /*********************/
  memcpy(framefile.b_write_to, RxFrame, DiskFrameSize);

  /* advance write-to pointer */
  framefile.b_write_to = new_write_to;
}

/***************************************************************/
/* FrameFileWriter: separate thread: writes each frame to disk */
/***************************************************************/
void FrameFileWriter(void)
{
  void* writeout_buffer;
  void* b_write_to;
  int write_len;

#ifdef __MCP__
  nameThread("FFWrit");
  //bputs(startup, "Startup\n");
#endif

  /* alloc output_buffer */
  writeout_buffer = balloc(tfatal, BUFFER_SIZE * DiskFrameSize);

  while (1) {
#ifdef __MCP__
    //Stop writing framefile with less than 200MB of disk space
    if (CommandData.df < 50 && CommandData.df > 0)
      bprintf(tfatal, "Insufficient disk space (%d) to write frame file, exiting", CommandData.df);
#endif
    write_len = 0;
    b_write_to = framefile.b_write_to;

    /* fill the write buffer from the transfer buffer */
    while (b_write_to != framefile.b_read_from) {
      memcpy(writeout_buffer + write_len, framefile.b_read_from,
          DiskFrameSize);
      framefile.b_read_from = advance_in_buffer(framefile.b_read_from);
      write_len += DiskFrameSize;

      /* increment file frame counter and check to see if we're at the end
       * of a file.  If so, writeout what we've accumulated and reset everything
       * for the next file */
      if (++framefile.frames >= FRAMES_PER_FILE) {
        if (framefile.fd >= 0)
          if (write(framefile.fd, writeout_buffer, write_len) < 0)
            berror(err, "Error while writing frame");

        OpenNextChunk();
        write_len = 0;
      }
    }

    /* check to see if the decomd has been signaled to idle.  We encase this
     * in a define for pananoiac reasons: don't wan't mcp to accidentally decide
     * to shutdown the disk writer */
#ifdef __DECOMD__
    if (shutdown_now) {
      if (framefile.fd > -1)
        if (close(framefile.fd) == -1)
          berror(err, "Error closing chunk");
      framefile.fd = -1;

      bfree(fatal, framefile.buffer);
      bfree(fatal, writeout_buffer);

      return;
    }
#endif

    if ((write_len > 0) && (framefile.fd >= 0))
      if (write(framefile.fd, writeout_buffer, write_len) < 0)
        berror(err, "Error while writing frame");

    usleep(400000);
  }
}
