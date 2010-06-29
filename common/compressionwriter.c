#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "mcp.h"
#include "compressstruct.h"

// Structure:
// FASTFRAMES: 100.16 Hz
// SLOWFRAMES: FASTFRAMES/20 - multiplex repeated at this rate
// STREAMFRAMES: FASTFRAMES/100 - minimum speed from streams.  
// SUPERFRAMES: FASTFRAMES/2000 - slow fields and stream offsets


#define N_PORTS 1

#define HIGAIN_TTY "/dev/ttySI2"

void nameThread(const char*);               /* mcp.c */

extern char *frameList[];
extern struct fieldStreamStruct streamList[];
extern short int InCharge;

int n_framelist;
int n_streamlist;
struct NiosStruct **frameNiosList;
struct BiPhaseStruct **frameBi0List;
struct NiosStruct **streamNiosList;
struct BiPhaseStruct **streamBi0List;
struct streamDataStruct *streamData;

int higain_bytes_per_streamframe = -1;
int omni_bytes_per_streamframe = -1;
int dialup_bytes_per_streamframe = -1;


static int OpenHiGainSerial(void) {
  static int report_state = -1; // -1 = no reports.  0 == reported error.  1 == reported success
  int fd;
  struct termios term;

  if ((fd = open(HIGAIN_TTY, O_RDWR | O_NOCTTY)) < 0) {
    if (report_state!=0) {
      bprintf(err, "Could not open tdrss higain serial port.  Retrying...");
      report_state = 0;
    }
    return (fd);
  }

  if (tcgetattr(fd, &term))
    berror(tfatal, "Unable to get higain tdrss serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  //term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  cfmakeraw(&term);
   
  if (cfsetospeed(&term, B115200))
    berror(tfatal, "Error setting higain tdrss serial output speed");

  if (cfsetispeed(&term, B115200))
    berror(tfatal, "Error setting higain tdrss serial input speed");

  if (tcsetattr(fd, TCSANOW, &term))
    berror(tfatal, "Unable to set higain tdrss serial attributes");

  if (report_state!=1) {
    bprintf(info, "TDRSS higain serial port opened");
    report_state = 1;
  }
  
  return fd;
}

void writeHiGainData(char *x, int size) {
  static int fp = -1;
  
  if (fp < 0) {
    fp = OpenHiGainSerial();
  }

  if (fp>=0) {
    if (write(fp, x, size)!=size) {
      close(fp);
    }
  }
}

void BufferStreamData(int i_streamframe, int readindex) {
  int i_field;
  int isWide;
  unsigned int x;

  // record stream data in buffer
  for (i_field=0; i_field < n_streamlist; i_field++) {
    if (streamNiosList[i_field]->fast) {
      if (streamNiosList[i_field]->wide) {
        isWide = 1;
        x = (unsigned int)tdrss_data[readindex][streamBi0List[i_field]->channel] +
        ((unsigned int)tdrss_data[readindex][streamBi0List[i_field]->channel+1] << 16);
      } else {
        isWide = 0;
        x = tdrss_data[readindex][streamBi0List[i_field]->channel];
      }
    } else { // slow
      if (streamNiosList[i_field]->wide) {
        isWide = 1;
        x = (unsigned int)slow_data[streamBi0List[i_field]->index][streamBi0List[i_field]->channel] +
        ((unsigned int)slow_data[streamBi0List[i_field]->index][streamBi0List[i_field]->channel+1] <<16);
      } else {
        isWide = 0;
        x = slow_data[streamBi0List[i_field]->index][streamBi0List[i_field]->channel];
      }
    }
    streamData[i_field].x[i_streamframe] = x;
    streamData[i_field].sum += x;
    streamData[i_field].n_sum++;
  }
}


void WriteSuperFrame(int readindex) {
  int frame_bytes_written = 0;
  int i_field;
  int isWide;
  unsigned x;
  int size;
  unsigned gain, offset;

  x=SYNCWORD;
  writeHiGainData((char *)(&x), 4);

  // write superframe data
  for (i_field = 0; i_field<n_framelist; i_field++) {
    if (frameNiosList[i_field]->fast) {
      if (frameNiosList[i_field]->wide) {
        isWide = 1;
        x = (unsigned int)tdrss_data[readindex][frameBi0List[i_field]->channel] +
        ((unsigned int)tdrss_data[readindex][frameBi0List[i_field]->channel+1] << 16);
      } else {
        isWide = 0;
        x = tdrss_data[readindex][frameBi0List[i_field]->channel];
      }
    } else { // slow
      if (frameNiosList[i_field]->wide) {
        isWide = 1;
        x = (unsigned int)slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel] +
        ((unsigned int)slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel+1] <<16);
      } else {
        isWide = 0;
        x = slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel];
      }
    }
    size = (1 + isWide)*sizeof(unsigned short);
    writeHiGainData((char*)&x, size);

    frame_bytes_written += size;
  }
  
  // figure out how many bytes per stream frame we have room for...
  if (higain_bytes_per_streamframe <0) {
    higain_bytes_per_streamframe = (HIGAIN_BYTES_PER_FRAME-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    omni_bytes_per_streamframe = (OMNI_BYTES_PER_FRAME-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    dialup_bytes_per_streamframe = (DIALUP_BYTES_PER_FRAME-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    bprintf(info, "Bytes per stream frame - High gain: %d  tdrss omni: %d  iridium dialup: %d",
            higain_bytes_per_streamframe, omni_bytes_per_streamframe, dialup_bytes_per_streamframe);

    BufferStreamData(0, readindex); // fill buffer with first value;

    //FIXME: calculate how many fields you can actually fit...
  }
 
  // set and write stream gains and offsets
  for (i_field=0; i_field<n_streamlist; i_field++) {
    short soffset;
    int ioffset;
    
    streamData[i_field].offset = streamData[i_field].sum/(double)streamData[i_field].n_sum;
    streamData[i_field].sum = streamData[i_field].n_sum = 0;
    gain = streamData[i_field].gain;
    ioffset = soffset = streamData[i_field].offset;
    writeHiGainData((char *)&gain, sizeof(unsigned short));
    if (frameNiosList[i_field]->wide) {
      writeHiGainData((char *)&ioffset, 2*sizeof(unsigned short));
    } else {
      writeHiGainData((char *)&soffset, sizeof(unsigned short));
    }
  }
  
  return;
}

void WriteStreamFrame() {
  int i_field, i_samp, i_fastsamp;
  int n=1;
  double x, dx;
  int xi;
  signed char streambuf[FASTFRAME_PER_STREAMFRAME];
  int n_streambuf;
  
  for (i_field = 0; i_field<n_streamlist; i_field++) {
    n_streambuf = 0;
    for (i_samp = 0; i_samp < streamList[i_field].samples_per_frame; i_samp++) {
      if (streamList[i_field].doAverage) {
        // filter
        n = FASTFRAME_PER_STREAMFRAME/streamList[i_field].samples_per_frame;
        x = 0.0;
        for (i_fastsamp = 0; i_fastsamp < n ; i_fastsamp++) {
          x+=streamData[i_field].x[i_fastsamp + i_samp * n];
        }
        x /= (double)n;
      } else {
        x=streamData[i_field].x[i_samp * n];
      }
      // differentiate
      if (streamList[i_field].doDifferentiate) {
        dx = x - streamData[i_field].last;
        streamData[i_field].last = x;
        x = dx;
      }
      // apply gain
      x = (x-streamData[i_field].offset)/(double)streamData[i_field].gain;

      //preserve integral
      xi = (int)(x + streamData[i_field].residual);
      streamData[i_field].residual =(x + streamData[i_field].residual) - (double)xi; // preserve integral

      if (streamList[i_field].bits == 4) {
      } else if (streamList[i_field].bits == 8) {
        streambuf[i_samp] = (signed char)xi;
        n_streambuf++;
      } else if (streamList[i_field].bits == 16) {
        n_streambuf+=2;
        ((short *)streambuf)[i_samp] = (short)xi;
      }
    }
    writeHiGainData((char *)streambuf, n_streambuf);
  }
}

void CompressionWriter() {
  int readindex, lastreadindex = 2;
  int n_higainstream = -1;
  int n_omnistream = -1;
  int n_dailupstream = -1;
  
  int i_fastframe = -1;
  int i_field;
  int i_streamframe=0;

  nameThread("DOWN");

  bputs(startup, "Startup.\n");

  // determine frameList length
  for (n_framelist=0; frameList[n_framelist][0]!='\0'; n_framelist++);
  
  frameNiosList = (struct NiosStruct **)malloc(n_framelist * sizeof(struct NiosStruct *));
  frameBi0List = (struct BiPhaseStruct **)malloc(n_framelist * sizeof(struct BiPhaseStruct *));
  
  for (i_field =0; i_field < n_framelist; i_field++) {
    frameNiosList[i_field] = GetNiosAddr(frameList[i_field]);
    frameBi0List[i_field] = GetBiPhaseAddr(frameList[i_field]);
  }

  // determine streamlist length
  for (n_streamlist = 0; streamList[n_streamlist].name[0] != '\0'; n_streamlist++);
  streamNiosList = (struct NiosStruct **)malloc(n_streamlist * sizeof(struct NiosStruct *));
  streamBi0List = (struct BiPhaseStruct **)malloc(n_streamlist * sizeof(struct BiPhaseStruct *));
  streamData = (struct streamDataStruct *)malloc(n_streamlist*sizeof(struct streamDataStruct));
  
  for (i_field = 0; i_field < n_streamlist; i_field++) {
    streamNiosList[i_field] = GetNiosAddr(streamList[i_field].name);
    streamBi0List[i_field] = GetBiPhaseAddr(streamList[i_field].name);
    streamData[i_field].last = 0;
    streamData[i_field].residual = 0;
    streamData[i_field].gain = streamList[i_field].gain;
    streamData[i_field].offset = 0;
    streamData[i_field].sum = 0.0;
    streamData[i_field].n_sum = 0;
  }
  
  bprintf(startup, "frame list length: %d  stream list length: %d", n_framelist, n_streamlist);

  while (!InCharge) {  // wait to be the boss to open the port!
    usleep(10000);
  }
  
  //*****************************************************
  //     The Infinite Loop....
  //*****************************************************
  while (1) {
    readindex = GETREADINDEX(tdrss_index);
    if (readindex != lastreadindex) {
      lastreadindex = readindex;
      ++i_fastframe;
      
      //********************************
      // Process the superframe 
      if (i_fastframe >= FASTFRAME_PER_SUPERFRAME) {
        i_fastframe = 0;
        WriteSuperFrame(readindex);
      }

      BufferStreamData(i_streamframe, readindex);

      if (++i_streamframe >= FASTFRAME_PER_STREAMFRAME) { // end of streamframe - lets write!
        // Write the frames here...
        WriteStreamFrame();

        i_streamframe = 0;
      }
      
        // write stream header data (gains and offsets)
      
    } else {
      usleep(10000);
    }
  }
  
}
