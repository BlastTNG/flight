#define END_OF_STREAM {"", 0, 0, 0, 0, 0}
#define NOAVG 0
#define AVG 1
#define NODX 0
#define DX 1

#define SYNCWORD 0xeb90a5a5

struct fieldStreamStruct {
  char name[256];
  int rightShift; // number of bits to rightshift in compression.  type will be reduced if possible.
  int skip; // 1 sample every skip fast frames.  skip < 20 makes no sense for slow fields!
  int doAverage; // boxcar average before decimation
  int differentiate; // send down derivative, not value
  int offsetBits; // number of bits in stream
};