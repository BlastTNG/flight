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
#include <string.h>
#include <limits.h>
#include "mcp.h"
#include "compressstruct.h"

// Structure:
// FASTFRAMES: 100.16 Hz
// SLOWFRAMES: FASTFRAMES/20 - multiplex repeated at this rate
// STREAMFRAMES: FASTFRAMES/100 - minimum speed from streams.  
// SUPERFRAMES: FASTFRAMES/2000 - slow fields and stream offsets

struct streamDataStruct {
  double x[FASTFRAME_PER_STREAMFRAME];
  double last;
  double residual;
  double sum;
  double n_sum;
  unsigned gain;
  long long offset;
  int isFast; // bool
  int isSigned; // bool
  int slowIndex; // from bi phase struct
  int slowChannel; // from bi phase struct
  unsigned mask; // used to set 16, 24, or 32 bits
  int lsw; // offset in bytes in frame_char to the lsw
  int msw; // offset in bytes in frame_char to the msw
};


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
struct streamDataStruct *streamData;

int higain_bytes_per_streamframe = -1;
int omni_bytes_per_streamframe = -1;
int dialup_bytes_per_streamframe = -1;

unsigned short *PopFrameBuffer(struct frameBuffer *buffer); // mcp.c

//*********************************************************
// Open High Gain Serial port
//*********************************************************
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

//*********************************************************
// Write data to the high gain serial port
//*********************************************************
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
//*********************************************************
// Buffer streamed data to be later compressed
//*********************************************************
void BufferStreamData(int i_streamframe, unsigned short *frame) {
  int i_field;
  unsigned int xu;
  double xd;
  int i,c;
  unsigned char *frame_char = (unsigned char *)frame;

  // record stream data in buffer
  for (i_field=0; i_field < n_streamlist; i_field++) {
    if (streamData[i_field].isFast) {
      int i_l, i_m;
      unsigned short *lsw;
      unsigned short *msw;
      unsigned mask;
      
      i_l = streamData[i_field].lsw;
      i_m = streamData[i_field].msw;
      mask = streamData[i_field].mask;

      lsw = (unsigned short *) (frame_char + i_l);
      if (mask) {
        msw = (unsigned short *) (frame_char + i_m);
        xu = (unsigned)(*lsw) | ((unsigned)(*msw & mask)<<16);
      } else {
         xu = (unsigned)(*lsw);
      }
    } else { // slow
      i = streamData[i_field].slowIndex;
      c = streamData[i_field].slowChannel;
      if (streamData[i_field].mask) { // wide slow
        xu = (unsigned int)slow_data[i][c] + ((unsigned int)slow_data[i][c+1] <<16);
      } else { // narrow slow
        xu = (unsigned int)slow_data[i][c];
      }
    }
    xd = xu;
    // deal with signed data
    if (streamData[i_field].isSigned) {
      if (streamData[i_field].mask) { // mask is only set if its wide
        if (xd>=INT_MAX) {
          xd -= UINT_MAX;
        }
      } else {
        if (xd >= SHRT_MAX) {
          xd -= USHRT_MAX;
        }
      }
    }
    streamData[i_field].x[i_streamframe] = xd;
    if (streamList[i_field].doDifferentiate) {
      if (i_streamframe==FASTFRAME_PER_STREAMFRAME-1) {
        streamData[i_field].sum = xd;
        streamData[i_field].n_sum=1;
      }
    } else {
      streamData[i_field].sum += xd;
      streamData[i_field].n_sum++;
    }
  }
}


//*********************************************************
// Write data that comes once per superframe
//*********************************************************
void WriteSuperFrame(unsigned short *frame) {
  int frame_bytes_written = 0;
  int i_field;
  int isWide;
  unsigned x;
  int size;
  unsigned gain;

  x=SYNCWORD;
  writeHiGainData((char *)(&x), 4);

  // write superframe data
  for (i_field = 0; i_field<n_framelist; i_field++) {
    if (frameNiosList[i_field]->fast) {
      if (frameNiosList[i_field]->wide) {
        isWide = 1;
        x = (unsigned int)frame[frameBi0List[i_field]->channel] +
        ((unsigned int)frame[frameBi0List[i_field]->channel+1] << 16);
      } else {
        isWide = 0;
        x = frame[frameBi0List[i_field]->channel];
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

    BufferStreamData(FASTFRAME_PER_STREAMFRAME-1, frame); // fill buffer with first value;

    //FIXME: calculate how many fields you can actually fit...
  }
 
  // set and write stream gains and offsets
  for (i_field=0; i_field<n_streamlist; i_field++) {
    long long unsigned lloffset;

    gain = streamData[i_field].gain;
    writeHiGainData((char *)&gain, sizeof(unsigned short));

    lloffset = streamData[i_field].sum/(double)streamData[i_field].n_sum;
    streamData[i_field].offset = lloffset;

    if (streamData[i_field].mask) {
      if (streamData[i_field].isSigned) { // 32 bit in field
        int offset;
        offset = lloffset;
        writeHiGainData((char *)&offset, sizeof(int));
      } else {
        unsigned offset;
        offset = lloffset;
        writeHiGainData((char *)&offset, sizeof(unsigned));
      }
    } else {
      if (streamData[i_field].isSigned) { // 16 bit signed field
        short offset;
        offset = lloffset;
        writeHiGainData((char *)&offset, sizeof(short));
      } else {
        unsigned short offset;
        offset = lloffset;
        writeHiGainData((char *)&offset, sizeof(unsigned short));
      }
    }
    streamData[i_field].sum = streamData[i_field].n_sum = 0;
  }
  
  return;
}
//*********************************************************
// write stream field
//*********************************************************
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
        x = dx/(double)streamData[i_field].gain;
      } else {
        // apply gain
        x = (x-streamData[i_field].offset)/(double)streamData[i_field].gain;
      }
      
      //preserve integral
      xi = (int)(x + streamData[i_field].residual);
      streamData[i_field].residual =(x + streamData[i_field].residual) - (double)xi; // preserve integral

      if (streamList[i_field].bits == 4) {
      } else if (streamList[i_field].bits == 8) {
        if (xi>127) xi = 127; // truncate overage
        if (xi<-127) xi = -127;
        streambuf[i_samp] = (signed char)xi;
        n_streambuf++;
      } else if (streamList[i_field].bits == 16) {
        if (xi > 32767) xi = 32767; // truncate
        if (xi<-32767) xi = -32767;
        n_streambuf+=2;
        ((short *)streambuf)[i_samp] = (short)xi;
      }
    }
    writeHiGainData((char *)streambuf, n_streambuf);
  }
}

//*********************************************************
// The main compression writer thread
//*********************************************************
void CompressionWriter() {
  int n_higainstream = -1;
  int n_omnistream = -1;
  int n_dailupstream = -1;
  
  int i_fastframe = -1;
  int i_field;
  int i_streamframe=0;
  unsigned short *frame;

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
  for (n_streamlist = 0; streamList[n_streamlist].name[0] != '\0'; n_streamlist++); // count
  streamData = (struct streamDataStruct *)malloc(n_streamlist*sizeof(struct streamDataStruct));
  
  for (i_field = 0; i_field < n_streamlist; i_field++) {
    if (isBoloField(streamList[i_field].name)) {
      struct BiPhaseStruct *l_bi0;
      struct BiPhaseStruct *m_bi0;
      char l_name[10];
      char m_name[10];
      
      sprintf(l_name, "%slo", streamList[i_field].name);
      sprintf(m_name, "%shi", streamList[i_field].name);

      // OK: bolometers are 24 bit fields.  The msw for odd bolometer
      // channels are stored in the high byte of the even channels.
      if (m_name[5]%2) m_name[5]--; // make sure msw referes to an even field
        
      l_bi0 = GetBiPhaseAddr(l_name);
      m_bi0 = GetBiPhaseAddr(m_name);

      streamData[i_field].isFast = 1;
      streamData[i_field].isSigned = 0;
      streamData[i_field].mask = 0x00ff;
      streamData[i_field].lsw = l_bi0->channel*2;
      if (streamList[i_field].name[5]%2) {
        streamData[i_field].msw = m_bi0->channel*2;
      } else {
        streamData[i_field].msw = m_bi0->channel*2+1;
      }
    } else {
      struct NiosStruct *nios;
      struct BiPhaseStruct *bi0;
      struct ChannelStruct *channel;

      nios = GetNiosAddr(streamList[i_field].name);
      bi0 = GetBiPhaseAddr(streamList[i_field].name);
      channel = GetChannelStruct(streamList[i_field].name);

      if ((channel->type == 's') || (channel->type == 'S')) {
        streamData[i_field].isSigned = 1;
      }
      if (nios->fast) {
        streamData[i_field].isFast = 1;
        if (nios->wide) {
          streamData[i_field].mask = 0xffff;
        } else {
          streamData[i_field].mask = 0;
        }
        streamData[i_field].lsw = bi0->channel*2;
        streamData[i_field].msw = bi0->channel*2 + 2;
      } else { // slow
        streamData[i_field].isFast = 0;
        streamData[i_field].slowIndex = bi0->index;
        streamData[i_field].slowChannel = bi0->channel;
        if (nios->wide) {
          streamData[i_field].mask = 0xffff;
        } else {
          streamData[i_field].mask = 0;
        }
      }
    }      
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
    frame = PopFrameBuffer(&hiGain_buffer);
    if (frame) {
      ++i_fastframe;
      
      //********************************
      // Process the superframe 
      if (i_fastframe >= FASTFRAME_PER_SUPERFRAME) {
        i_fastframe = 0;
        WriteSuperFrame(frame);
      }

      BufferStreamData(i_streamframe, frame);

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
