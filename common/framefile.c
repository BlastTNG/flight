#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "tx_struct.h"
#include "mcp.h"

unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

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

char *StringToLower(char *s) {
  int i, len;
  static char ls[256];

  len = strlen(s);
  if (len > 255)
    len = 255;

  for (i = 0; i < len; i++)
    ls[i] = tolower(s[i]);

  ls[len] = '\0';
  return(ls);
}

char *StringToUpper(char *s) {
  int i, len;
  static char us[256];

  len = strlen(s);

  for (i = 0; i < len; i++)
    us[i] = toupper(s[i]);

  us[len] = '\0';
  return(us);
}

/* OpenNextChunk: closes the currently open chunk (if any), increments the
 * chunk index and opens creates the next chunk */
void OpenNextChunk(void) {
  if (framefile.fd > -1)
    if (close(framefile.fd) == -1)
      merror(MCP_ERROR, "Error closing chunk");

  sprintf(framefile.name, "/data/rawdir/%lu.%c%03X", framefile.time,
      framefile.type, ++framefile.chunk);

  mprintf(MCP_INFO, "Writing to framefile %s\n", framefile.name);

  if ((framefile.fd = creat(framefile.name, 0644)) == -1)
    merror(MCP_ERROR, "Error opening chunk\n");

  framefile.frames = 0;
}

/*********************************************************************/
/*                                                                   */
/*     Initialize framefile                                          */
/*                                                                   */
/*********************************************************************/
void InitialiseFrameFile(char type) {
  FILE* fp;

  /* filename */
  framefile.fd = framefile.chunk = -1;
  framefile.type = type;
  framefile.time = time(NULL);
  OpenNextChunk();

  /* malloc frame buffer */
  if ((framefile.buffer = malloc(BUFFER_SIZE * RX_FRAME_SIZE)) == NULL)
    mputs(MCP_TFATAL, "Unable to malloc framefile buffer\n");
  framefile.buffer_end = framefile.buffer + BUFFER_SIZE * RX_FRAME_SIZE;
  framefile.b_write_to = framefile.b_read_from = framefile.buffer;

  fp = fopen("/data/etc/datafile.cur","w");
  if (fp == NULL) {
    merror(MCP_ERROR, "Error opening curfile");
    return;
  }

  /* defile likes the newline */
  fprintf(fp, "%s\n", framefile.name);

  if (fclose(fp) < 0)
    merror(MCP_ERROR, "Error while closing curfile");
}

void* advance_in_buffer(void* ptr) {
  void* tmp;
  tmp = ((char*)ptr + RX_FRAME_SIZE);
  return (tmp > framefile.buffer_end) ? framefile.buffer : (void*)tmp;
}

/*************************************************************/
/* pushDiskFrame: called from main thread: puts rxframe into */
/* individual buffers                                        */
/*************************************************************/
void pushDiskFrame(unsigned short *RxFrame) {
  unsigned int i_slow;
  int i_ch;

  /*********************************************/
  /* fill the MCP internal slow data structure */
  /*********************************************/
  i_ch = RxFrame[3];
  if (i_ch < FAST_PER_SLOW) {
    for (i_slow = 0; i_slow<N_SLOW; i_slow++) {
      slow_data[i_slow][i_ch] = RxFrame[4 + i_slow];
    }
  }

  /* ****************************************************************** */
  /* First make sure there is enough space in the buffer                */
  /* We discard the full frame id there is no space                     */
  /* ****************************************************************** */

  if (advance_in_buffer(framefile.b_write_to) == framefile.b_read_from) {
    mputs(MCP_WARNING, "Framefile buffer overflow (frame discarded)\n");
    return;
  }	

  /*********************/
  /* SHIP OUT RX FRAME */
  /*********************/
  memcpy(framefile.b_write_to, RxFrame, RX_FRAME_SIZE);

  /* advance write-to pointer */
  framefile.b_write_to = advance_in_buffer(framefile.b_write_to);
}

/***************************************************************/
/* FrameFileWriter: separate thread: writes each frame to disk */
/***************************************************************/
void FrameFileWriter(void) {
  void* writeout_buffer;
  void* b_write_to;
  int write_len;

  mputs(MCP_STARTUP, "FrameFileWriter startup\n");

  /* malloc output_buffer */
  if ((writeout_buffer = malloc(BUFFER_SIZE * RX_FRAME_SIZE)) == NULL)
    mputs(MCP_TFATAL, "Unable to malloc write out buffer\n");

  while (1) {
    write_len = 0;
    b_write_to = framefile.b_write_to;

    while (b_write_to != framefile.b_read_from) {
      memcpy(writeout_buffer + write_len, framefile.b_read_from, RX_FRAME_SIZE);
      framefile.b_read_from = advance_in_buffer(framefile.b_read_from);
      write_len += RX_FRAME_SIZE;

      /* increment file frame counter and check to see if we're at the end
       * of a file.  If so, writeout what we've accumulated and reset everything
       * for the next file */
      if (++framefile.frames >= FRAMES_PER_FILE) {
        if (framefile.fd >= 0)
          if (write(framefile.fd, writeout_buffer, write_len) < 0)
            merror(MCP_ERROR, "Error while writing frame");

        OpenNextChunk();
        write_len = 0;
      }
    }

    if ((write_len > 0) && (framefile.fd >= 0)) 
      if (write(framefile.fd, writeout_buffer, write_len) < 0)
        merror(MCP_ERROR, "Error while writing frame");

    usleep(400000);
  }
}
