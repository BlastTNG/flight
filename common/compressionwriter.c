/****************************************************************
 * Arranges code for transfer from Flight Computer
 *
 *
 *
 ****************************************************************/

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
#include <stdint.h>
#include "mcp.h"
#include "compressstruct.h"
#include "compressconst.h"
#include "command_struct.h"
#include "pointing_struct.h"

/* MWG: This include was added manually */
#include "channels.h"


// Structure:
// FASTFRAMES: 100.16 Hz
// SLOWFRAMES: FASTFRAMES/20 - multiplex repeated at this rate
// STREAMFRAMES: FASTFRAMES/100 - minimum speed from streams.  
// SUPERFRAMES: FASTFRAMES/2000 - slow fields and stream offsets

//#define N_ARRAY_STATS_PER_SUPERFRAME 100
#define MAX_BLOB_WORDS_PER_SUPERFRAME 100


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

/* File locations */
#define HIGAIN_TTY "/dev/ttySI2"
#define PILOT_FILE "/data/etc/highgain.lnk"
#define OMNI1_TTY "/dev/ttySI1"
#define OMNI2_TTY "/dev/ttySI7"

void nameThread(const char*);            // In mputs.c

extern char *frameList[];
extern struct fieldStreamStruct streamList[N_OTH_SETS][MAX_OTH_STREAM_FIELDS];
extern short int InCharge;

int n_framelist;
unsigned short n_streamlist;  // number of fields in the stream list
unsigned short n_higain_stream = 0;  // number of stream fields written to highgain
unsigned short n_omni1_stream = 0;
unsigned short n_omni2_stream = 0;

struct NiosStruct **frameNiosList;
struct BiPhaseStruct **frameBi0List;
struct streamDataStruct *streamData;


unsigned short *PopFrameBufferAndSlow(struct frameBuffer *buffer,  unsigned short ***slow); // mcp.c

void ClearBuffer(struct frameBuffer *buffer);

unsigned short **current_slow_data;

int cycle_highgain_file = 0;

//*********************************************************
// Open High Gain Serial port
//*********************************************************
static int OpenSerial(char *tty) {

// MWG:
//  OpenSerial tries to access:
//  HIGAIN_TTY: /dev/ttySI2
//  OMNI1_TTY: /dev/ttySI1
//  OMNI2_TTY: /dev/ttySI7
//
//  With the error message:
//  "Compressionwriter.c: Could not open downlink serial port X.  Retrying..."
//  Because fd = 0 through "fd = open(tty, O_RDWR | O_NOCTTY))"


  static int report_state = -1; // -1 = no reports.  0 == reported error.  1 == reported success
  int fd;

  struct termios term;

  if ((fd = open(tty, O_RDWR | O_NOCTTY)) < 0) {
    if (report_state != 0) {
      bprintf(err, "Compressionwriter.c: Could not open downlink serial port %s.  Retrying...", tty);
      report_state = 0;
    }
    return (fd);
  }

  if (tcgetattr(fd, &term)) {
    berror(tfatal, "Compressionwriter.c: Unable to get downlink serial port %s attributes", tty);
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  cfmakeraw(&term);
   
  if (cfsetospeed(&term, B115200))
    berror(tfatal, "Error setting downlink serial port %s output speed", tty);

  if (cfsetispeed(&term, B115200))
    berror(tfatal, "Error setting downlink serial port %s input speed", tty);

  if (tcsetattr(fd, TCSANOW, &term))
    berror(tfatal, "Unable to set downlink serial port %s attributes", tty);

  report_state = 1;
  
  return fd;
}

//*********************************************************
// Write data to the high gain serial port
//*********************************************************
void writeHiGainData(char *x, int size) {
  static int fp_port = -1;
  static int fp_file = -1;

  // write to serial port  
  if (fp_port < 0) {
    fp_port = OpenSerial(HIGAIN_TTY);
  }

  if (fp_port>=0) {
    if (write(fp_port, x, size)!=size) {
      if (close(fp_port)==0) {
        fp_port = -1;
      }
    }
  }

  // write to file
  if (cycle_highgain_file && (fp_file>=0)) {
    close(fp_file);
    fp_file = -1;
  }
  
  if (fp_file < 0) {
    fp_file = creat(PILOT_FILE, 00666);
    cycle_highgain_file = 0;
  }

  if (fp_file>=0) {
    if (write(fp_file, x, size)!=size) {
      if (close(fp_file)==0) {
        fp_file = -1;            
      }
    }
  }
}

//*********************************************************
// Write data to the omni1 serial port
//*********************************************************
void writeOmni1Data(char *x, int size) {
  static int fp = -1;
  
  if (fp < 0) {
    fp = OpenSerial(OMNI1_TTY);
  }

  if (fp>=0) {
    
    if (write(fp, x, size)!=size) {
      close(fp);
    }
  }
}

//*********************************************************
// Write data to the omni2 serial port
//*********************************************************
void writeOmni2Data(char *x, int size) {
  static int fp = -1;

  if (fp < 0) {
    fp = OpenSerial(OMNI2_TTY);
  }

  if (fp>=0) {
    if (write(fp, x, size)!=size) {
      close(fp);
    }
  }
}

//*********************************************************
// Write data to ports if there is room
//*********************************************************
void writeData(char *x, int size, int i_field) {
  if (i_field < n_higain_stream) {
    writeHiGainData(x,size);
  }
  if (i_field < n_omni1_stream) {
    writeOmni1Data(x,size);
  }
  if (i_field < n_omni2_stream) {
    writeOmni2Data(x,size);
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
  // buffer all stream channels, even ones that might not get written
  for (i_field = 0; i_field < n_streamlist; i_field++) {
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
        xu = (unsigned)(*lsw) | ((unsigned)(*msw & mask) << 16);
      } else {
         xu = (unsigned)(*lsw);
      }
    } else { // slow
      i = streamData[i_field].slowIndex;
      c = streamData[i_field].slowChannel;
      if (streamData[i_field].mask) { // wide slow
        xu = (unsigned int)current_slow_data[i][c] + ((unsigned int)current_slow_data[i][c+1] <<16);
      } else { // narrow slow
        xu = (unsigned int)current_slow_data[i][c];
      }
    }
    xd = xu;
    // deal with signed data
    if (streamData[i_field].isSigned) {
      if (streamData[i_field].mask) { // mask is only set if it's wide
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
    if (streamList[CommandData.channelset_oth][i_field].doDifferentiate) {
      if (i_streamframe==FASTFRAME_PER_STREAMFRAME-1) {
        streamData[i_field].sum = xd;
        streamData[i_field].n_sum = 1;
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
  int higain_bytes_per_streamframe = 0;
  int omni1_bytes_per_streamframe = 0;
  int omni2_bytes_per_streamframe = 0;
  // initialize required bytes per streamframe to account for stream sync bytes.
  double higain_requested_bytes_per_streamframe = STREAMFRAME_PER_SUPERFRAME;
  double omni1_requested_bytes_per_streamframe = STREAMFRAME_PER_SUPERFRAME;
  double omni2_requested_bytes_per_streamframe = STREAMFRAME_PER_SUPERFRAME;

  static int first_time = 1;
  static int reset_rates = 1;
  static int omni1_rate = 0;
  static int omni2_rate = 0;
  static int highgain_rate = 0;
  static int n_arrays_stats_per_superframe = 0;

  int frame_bytes_written = 0;
  int i_field;
  int isWide;
  unsigned x;
  int size;
  unsigned gain;
  static unsigned int last_set = 65535;
  char set = CommandData.channelset_oth;

  if (last_set != set) {
    last_set = set;
    reset_rates = 1;
  }
  
  x=SYNCWORD;
  writeData((char *)(&x), 4, 0);
  frame_bytes_written += 4;
  
  writeData(&set, 1, 0);
  frame_bytes_written++;
  
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
        x = (unsigned int)current_slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel] +
        ((unsigned int)current_slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel+1] <<16);
      } else {
        isWide = 0;
        x = current_slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel];
      }
    }
    size = (1 + isWide)*sizeof(unsigned short);
    writeData((char*)&x, size, 0);

    frame_bytes_written += size;
  }

  frame_bytes_written += sizeof(unsigned short); // account for nfields field;
  
  if (first_time) {
    first_time = 0;
    BufferStreamData(FASTFRAME_PER_STREAMFRAME - 1, frame); // fill buffer with first value;
  }

  if (highgain_rate != CommandData.pilot_bw) {
    highgain_rate = CommandData.pilot_bw;
    reset_rates = 1;
  }
  if (omni1_rate != CommandData.tdrss_bw) {
    omni1_rate = CommandData.tdrss_bw;
    reset_rates = 1;
  }
  if (omni2_rate != CommandData.iridium_bw) {
    omni2_rate = CommandData.iridium_bw;
    reset_rates = 1;
  }


  // Calculate number of stream fields given data rates
  if (reset_rates) {
    reset_rates = 0;
    cycle_highgain_file = 1;
    //higain_bytes_per_streamframe = (HIGAIN_BYTES_PER_FRAME-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    higain_bytes_per_streamframe = (highgain_rate*FASTFRAME_PER_SUPERFRAME/(8*SR)-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    omni1_bytes_per_streamframe =  (   omni1_rate*FASTFRAME_PER_SUPERFRAME/(8*SR)-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    omni2_bytes_per_streamframe =  (   omni2_rate*FASTFRAME_PER_SUPERFRAME/(8*SR)-frame_bytes_written)/STREAMFRAME_PER_SUPERFRAME;
    //bprintf(info, "Bytes per stream frame - High gain: %d  omni1: %d  omni2: %d",
    //        higain_bytes_per_streamframe, omni1_bytes_per_streamframe, omni2_bytes_per_streamframe);

    n_higain_stream = 0;
    n_omni1_stream = 0;
    n_omni2_stream = 0;
    
    for (i_field = 0; i_field<n_streamlist; i_field++) {
      double delta = 0;
      delta = streamList[CommandData.channelset_oth][i_field].samples_per_frame*streamList[CommandData.channelset_oth][i_field].bits/8;
      if (streamData[i_field].mask) {
        delta+=6.0/(double)STREAMFRAME_PER_SUPERFRAME;
      } else {
        delta+=4.0/(double)STREAMFRAME_PER_SUPERFRAME;
      }
        
      if (higain_requested_bytes_per_streamframe+delta>higain_bytes_per_streamframe) {
        if (n_higain_stream==0) {
          n_higain_stream = i_field;
        }
      } else {
        higain_requested_bytes_per_streamframe+=delta;
      }
      
      if (omni1_requested_bytes_per_streamframe+delta>omni1_bytes_per_streamframe) {
        if (n_omni1_stream==0) {
          n_omni1_stream = i_field;
        }
      } else {
        omni1_requested_bytes_per_streamframe+=delta;
      }
      
      if (omni2_requested_bytes_per_streamframe+delta>omni2_bytes_per_streamframe) {
        if (n_omni2_stream==0) {
          n_omni2_stream = i_field;
        }
      } else {
        omni2_requested_bytes_per_streamframe+=delta;
      }
    }
    
    if (n_higain_stream==0) {
      n_higain_stream = n_streamlist;
    }
    if (n_omni1_stream==0) {
      n_omni1_stream = n_streamlist;
    }
    if (n_omni2_stream==0) {
      n_omni2_stream = n_streamlist;
    }
    
    bprintf(info, "High gain: %u stream fields use %.0f out of %d bytes per stream frame (%.0f free)",
            n_higain_stream,
            higain_requested_bytes_per_streamframe, higain_bytes_per_streamframe,
            higain_bytes_per_streamframe - higain_requested_bytes_per_streamframe);
    bprintf(info, "Omni1: %u stream fields use %.0f out of %d bytes per stream frame (%.0f free)",
            n_omni1_stream,
            omni1_requested_bytes_per_streamframe, omni1_bytes_per_streamframe,
            omni1_bytes_per_streamframe - omni1_requested_bytes_per_streamframe);
    bprintf(info, "Omni2: %u stream fields use %.0f out of %d bytes per stream frame (%.0f free)",
            n_omni2_stream,
            omni2_requested_bytes_per_streamframe, omni2_bytes_per_streamframe,
            omni2_bytes_per_streamframe - omni2_requested_bytes_per_streamframe);
  }

  // write fields size
  writeHiGainData((char*)&n_higain_stream, sizeof(unsigned short));
  writeOmni1Data((char*)&n_omni1_stream, sizeof(unsigned short));
  writeOmni2Data((char*)&n_omni2_stream, sizeof(unsigned short));
  
  // set and write stream gains and offsets
  for (i_field=0; i_field<n_streamlist; i_field++) {
    long long unsigned lloffset;
    streamData[i_field].residual = 0;

    gain = streamData[i_field].gain;

    writeData((char *)&gain, sizeof(unsigned short), i_field);

    lloffset = streamData[i_field].sum/(double)streamData[i_field].n_sum;
    streamData[i_field].offset = lloffset;

    if (streamData[i_field].mask) {
      if (streamData[i_field].isSigned) { // 32 bit in field
        int offset;
        offset = lloffset;
        writeData((char *)&offset, sizeof(int), i_field);
      } else {
        unsigned offset;
        offset = lloffset;
        writeData((char *)&offset, sizeof(unsigned), i_field);
      }
    } else {
      if (streamData[i_field].isSigned) { // 16 bit signed field
        short offset;
        offset = lloffset;
        writeData((char *)&offset, sizeof(short), i_field);
      } else {
        unsigned short offset;
        offset = lloffset;
        writeData((char *)&offset, sizeof(unsigned short), i_field);
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
  int n;

  /* x either passes the value itself OR delta-x down to the user. */
  double x, dx;
  int xi;
  signed char streambuf[FASTFRAME_PER_STREAMFRAME];
  int n_streambuf;

  for (i_field = 0; i_field < n_streamlist; i_field++) {
    n_streambuf = 0;
    n = FASTFRAME_PER_STREAMFRAME/streamList[CommandData.channelset_oth][i_field].samples_per_frame;

    for (i_samp = 0; i_samp < streamList[CommandData.channelset_oth][i_field].samples_per_frame; i_samp++) {
      if (streamList[CommandData.channelset_oth][i_field].doAverage) {

        /* x = average over i_field of all streamData[i_field].x[i_fastsamp + i_samp * n] */
        // filter
        x = 0.0;
        for (i_fastsamp = 0; i_fastsamp < n ; i_fastsamp++) {
          x += streamData[i_field].x[i_fastsamp + i_samp * n];
        }
        x /= (double)n;
      } else {
        x = streamData[i_field].x[i_samp * n];
      }
      // differentiate
      if (streamList[CommandData.channelset_oth][i_field].doDifferentiate) {
        dx = x - streamData[i_field].last;
        streamData[i_field].last = x;
        x = dx/(double)streamData[i_field].gain;
      } else {
        // apply gain
        x = (x - streamData[i_field].offset) / (double)streamData[i_field].gain;
      }
      /* Better perhaps: x /= (double)streamData.gain at the end */

      /* So streamData[i_field].residual holds on to the value of x when x = dx instead */
      //preserve integral
      xi = (int)(x + streamData[i_field].residual);
      streamData[i_field].residual = x;

      if (streamList[CommandData.channelset_oth][i_field].bits == 8) {
        // if the derivative exceeds the maximum, truncate, but preserve the
        // integral for later application.  It gets cleared at the slow frame.
        if (streamList[CommandData.channelset_oth][i_field].doDifferentiate) {
          if (xi > 127) {
            streamData[i_field].residual += xi - 127;
            xi = 127;
          } else if (xi < -127) {
            streamData[i_field].residual += xi + 127;
            xi = -127;
          }            
        } else {
          if (xi>127) xi = 127;
          if (xi<-127) xi = -127;
        }
        streambuf[i_samp] = (signed char)xi;
        n_streambuf++;
      } else if (streamList[CommandData.channelset_oth][i_field].bits == 16) {
        if (xi > 32767) xi = 32767; // truncate
        if (xi<-32767) xi = -32767;
        n_streambuf+=2;
        ((short *)streambuf)[i_samp] = (short)xi;
      }
    }
    writeData((char *)streambuf, n_streambuf, i_field);
  }
  
  streambuf[0] = 0xa5;
  writeData((char *)streambuf, 1, 0); // write stream frame sync byte.

}


//*********************************************************
// The main compression writer thread
//*********************************************************
void CompressionWriterFunction(int channelset) {
  int i_field;
  int i_streamframe = 0;
  unsigned short *frame;
  int i_fastframe = -1;
  
  // Determine frameList length by running through frameList until the \0 is reached.
  for (n_framelist = 0; frameList[n_framelist][0] != '\0'; n_framelist++);
  
  frameNiosList = (struct NiosStruct **)malloc(n_framelist * sizeof(struct NiosStruct *));
  frameBi0List = (struct BiPhaseStruct **)malloc(n_framelist * sizeof(struct BiPhaseStruct *));

  for (i_field = 0; i_field < n_framelist; i_field++) {
      frameNiosList[i_field] = GetNiosAddr(frameList[i_field]);
      frameBi0List[i_field] = GetBiPhaseAddr(frameList[i_field]);
  }

  // Determine streamlist length just as with framelist length.
  bprintf(info, "Compressionwriter.c: channelset oth == %d, field 1 == %s", CommandData.channelset_oth, streamList[1][1].name);
  for (n_streamlist = 0; streamList[CommandData.channelset_oth][n_streamlist].name[0] != '\0'; n_streamlist++)

  streamData = (struct streamDataStruct *)malloc(n_streamlist*sizeof(struct streamDataStruct));

  for (i_field = 0; i_field < n_streamlist; i_field++) {
    //printf ("Compressionwriter.c: (bolostuff) %s\n",streamList[CommandData.channelset_oth][i_field].name);
    if (isBoloField(streamList[CommandData.channelset_oth][i_field].name)) {

        /* MWG: I commented out the following section to get rid
         * of bolometers - this is a clumsy hack, but I lack more
         * elegant methods for making the code run right now.
         * To make the program run properly with bolometers, set
         * DAS_CARDS to 12 in channels.h, and uncomment this next code.
         */

        /*
      struct BiPhaseStruct *l_bi0;
      struct BiPhaseStruct *m_bi0;
      char l_name[10];
      char m_name[10];
      
      sprintf(l_name, "%slo", streamList[CommandData.channelset_oth][i_field].name);
      sprintf(m_name, "%shi", streamList[CommandData.channelset_oth][i_field].name);

      // OK: bolometers are 24 bit fields.  The msw for odd bolometer
      // channels are stored in the high byte of the even channels.
      if (m_name[5]%2) m_name[5]--; // make sure msw referes to an even field

      printf ("Compresionwriter.c Flag0: looking for field %s\n", l_name);
      l_bi0 = GetBiPhaseAddr(l_name);
      printf ("Compresionwriter.c Flag1: looking for field %s\n", m_name);
      m_bi0 = GetBiPhaseAddr(m_name);

      streamData[i_field].isFast = 1;
      streamData[i_field].isSigned = 0;
      streamData[i_field].mask = 0x00ff;
      streamData[i_field].lsw = l_bi0->channel*2;
      if (streamList[CommandData.channelset_oth][i_field].name[5]%2) {
        streamData[i_field].msw = m_bi0->channel*2;
      } else {
        streamData[i_field].msw = m_bi0->channel*2+1;
      }
    */
    } else {
      struct NiosStruct *nios;
      struct BiPhaseStruct *bi0;
      struct ChannelStruct *channel;

      nios = GetNiosAddr(streamList[CommandData.channelset_oth][i_field].name);
      bi0 = GetBiPhaseAddr(streamList[CommandData.channelset_oth][i_field].name);
      channel = GetChannelStruct(streamList[CommandData.channelset_oth][i_field].name);

      if ((channel->type == 's') || (channel->type == 'S')) {
        streamData[i_field].isSigned = 1;
      } else {
        streamData[i_field].isSigned = 0;
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
    streamData[i_field].gain = streamList[CommandData.channelset_oth][i_field].gain;
    streamData[i_field].offset = 0;
    streamData[i_field].sum = 0.0;
    streamData[i_field].n_sum = 0;
  }
  
  bprintf(startup, "Compressionwriter.c, CompressionWriterFunction: frame list length == %d; stream list length == %d", n_framelist, n_streamlist);

  while (!InCharge) {  // wait to be the boss to open the port!
    usleep(10000);
  }
  

  // clear the fifo so we have current data
  ClearBuffer(&hiGain_buffer);
  
  //*****************************************************
  //     The Infinite Loop....
  //*****************************************************
  while (channelset == CommandData.channelset_oth) {
    frame = PopFrameBufferAndSlow(&hiGain_buffer, &current_slow_data);
    if (frame) {
        printf ("frame == %d",frame);
        ++i_fastframe;
      
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

void CompressionWriter() {  

  nameThread("DOWN");

  //bprintf(startup, "Starting with oth channel set %d", CommandData.channelset_oth);

  while (1) {
    CompressionWriterFunction(CommandData.channelset_oth);
    usleep(10000);
    //bprintf(info, "restarting with oth channel set %d", CommandData.channelset_oth);
  }
}


/* MWG: This wrapper function works with test.c to call a static function.
 * It only works with my test program, and may be removed without affecting
 * functionality in compressionwriter.c. */
#ifdef TEST
int OpenSerialTest(){
    char *tty = '\0'; // For later
    return OpenSerial(tty);
    }
#endif

