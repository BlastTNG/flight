#define END_OF_STREAM {"", 0, 0, 0, 0, 0}
#define NOAVG 0
#define AVG 1
#define NODX 0
#define DX 1
#define SPIKE 0
#define SLOW 1


#define SYNCWORD 0xeb90a5a5

struct fieldStreamStruct {
  char name[256];
  int gain; // x_down = x/gain
  int samples_per_frame; // samples per ~1Hz frame
  int doAverage; // boxcar average before decimation
  int differentiate; // send down derivative, not value
  int bits; // number of bits in stream: 4, 8, 16, 32
};
