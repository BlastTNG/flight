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

/* if compiling MCP load the real mputs function prototypes, otherwise, just
 * make up a fake one */
#ifdef __MCP__
#  include "mcp.h"
#else
#  define mprintf(x, ...) \
     do {  /* encase in a do {} while(0) loop to properly swallow the ; */ \
       printf(__VA_ARGS__); \
       if (strcmp(#x, "MCP_FATAL") == 0) \
         exit(1); \
     } while (0)
#  define mputs(x,s) \
     do {  /* encase in a do {} while(0) loop to properly swallow the ; */ \
       puts(s); \
       if (strcmp(#x, "MCP_FATAL") == 0) \
         exit(1); \
     } while (0)
#  define merror(x,s) \
     do {  /* encase in a do {} while(0) loop to properly swallow the ; */ \
       perror(s); \
       if (strcmp(#x, "MCP_FATAL") == 0) \
         exit(1); \
     } while (0)
#endif

unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];


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
void OpenNextChunk(void) {
  if (framefile.fd > -1)
    if (close(framefile.fd) == -1)
      merror(MCP_ERROR, "Error closing chunk");

  sprintf(framefile.name, "/data/rawdir/%lu.%c%03X", framefile.time,
      framefile.type, ++framefile.chunk);

  mprintf(MCP_INFO, "Writing to framefile %s\n", framefile.name);

  if ((framefile.fd = creat(framefile.name, 0644)) == -1)
    merror(MCP_ERROR, "Error opening chunk");

  framefile.frames = 0;
}

/*********************************************************************/
/*                                                                   */
/*     Initialize framefile                                          */
/*                                                                   */
/*********************************************************************/
void InitialiseFrameFile(char type) {
  FILE* fp;
  char buffer[200];

  /* filename */
  framefile.fd = framefile.chunk = -1;
  framefile.type = type;
  framefile.time = time(NULL);
  OpenNextChunk();
  sprintf(buffer, "/data/rawdir/%lu.%c.spec", framefile.time, framefile.type);

  if ((fp = fopen(buffer,"w")) == NULL)
    merror(MCP_ERROR, "Unable to write spec file");
  else {
    WriteSpecificationFile(fp);
    fclose(fp);
  }

  /* malloc frame buffer */
  if ((framefile.buffer = malloc(BUFFER_SIZE * BiPhaseFrameSize)) == NULL)
    merror(MCP_TFATAL, "Unable to malloc framefile buffer");
  framefile.buffer_end = framefile.buffer + BUFFER_SIZE * BiPhaseFrameSize;
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
  tmp = ((char*)ptr + BiPhaseFrameSize);
  return (tmp >= framefile.buffer_end) ? framefile.buffer : (void*)tmp;
}

/*************************************************************/
/* pushDiskFrame: called from main thread: puts rxframe into */
/* individual buffers                                        */
/*************************************************************/
void pushDiskFrame(unsigned short *RxFrame) {
  void* new_write_to = advance_in_buffer(framefile.b_write_to);

#ifdef __MCP__
  int i_slow, i_mindex;

  /*******************************************************************/
  /* fill the Rx slow data from the MCP internal slow data structure */
  /*******************************************************************/
  i_mindex = RxFrame[3];
  if (i_mindex < FAST_PER_SLOW) {
    for (i_slow = 0; i_slow < slowsPerBi0Frame; i_slow++) {
      RxFrame[4 + i_slow] = slow_data[i_mindex][i_slow];
    }
  }
#endif

  /* ****************************************************************** */
  /* First make sure there is enough space in the buffer                */
  /* We discard the full frame if there is no space                     */
  /* ****************************************************************** */

  if (new_write_to == framefile.b_read_from) {
    mputs(MCP_WARNING, "Framefile buffer overflow (frame discarded)\n");
    return;
  }	

  /*********************/
  /* SHIP OUT RX FRAME */
  /*********************/
  memcpy(framefile.b_write_to, RxFrame, BiPhaseFrameSize);

  /* advance write-to pointer */
  framefile.b_write_to = new_write_to;
}

/***************************************************************/
/* FrameFileWriter: separate thread: writes each frame to disk */
/***************************************************************/
void FrameFileWriter(void) {
  void* writeout_buffer;
  void* b_write_to;
  int write_len;

  pthread_setspecific(identity, "disk");
  mputs(MCP_STARTUP, "FrameFileWriter startup\n");

  /* malloc output_buffer */
  if ((writeout_buffer = malloc(BUFFER_SIZE * BiPhaseFrameSize)) == NULL)
    mputs(MCP_TFATAL, "Unable to malloc write out buffer\n");

  while (1) {
    write_len = 0;
    b_write_to = framefile.b_write_to;

    while (b_write_to != framefile.b_read_from) {
      memcpy(writeout_buffer + write_len, framefile.b_read_from,
          BiPhaseFrameSize);
      framefile.b_read_from = advance_in_buffer(framefile.b_read_from);
      write_len += BiPhaseFrameSize;

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
