#ifndef COMPRESSSTRUCT_H
#define COMPRESSSTRUCT_H

#include "channels.h"

#define END_OF_STREAM {"", 0, 0, 0, 0, 0,0}
#define NOAVG 0
#define AVG 1
#define NODX 0
#define DX 1
#define SPIKE 0
#define SLOW 1

#define SYNCWORD 0xeb90a5a5

#define FASTFRAME_PER_SUPERFRAME 2000
#define FASTFRAME_PER_STREAMFRAME 100
#define STREAMFRAME_PER_SUPERFRAME (FASTFRAME_PER_SUPERFRAME/FASTFRAME_PER_STREAMFRAME)
#define OMNI2_BYTES_PER_FRAME (2000/9 * FASTFRAME_PER_SUPERFRAME/SR)
#define OMNI1_BYTES_PER_FRAME (6000/9 * FASTFRAME_PER_SUPERFRAME/SR)
#define HIGAIN_BYTES_PER_FRAME (92000/8 * FASTFRAME_PER_SUPERFRAME/SR)

struct fieldStreamStruct {
  char name[256];
  int gain; // x_down = x/gain
  int samples_per_frame; // samples per ~1Hz frame
  int doAverage; // boxcar average before decimation
  int doDifferentiate; // send down derivative, not value
  int bits; // number of bits in stream: 4, 8, 16, 32
  int spikeMode; // SLOW: skip samples  SPIKE: report enlarged data
};

int isBoloField(char *field);
struct ChannelStruct *GetChannelStruct(char *name);

#endif
