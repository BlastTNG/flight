#define SLOWDL_NUM_DATA     7

#define SLOWDL_LEN          29

#define SLOWDL_DLE          0x10
#define SLOWDL_SYNC         0x53
#define SLOWDL_ETX          0x03

#define SLOWDL_TAKE_BIT     0
#define SLOWDL_FORCE_INT    1
#define SLOWDL_U_MASK       2

struct SlowDLStruct {
  char src[20];
  char type;
  int numbits;
  double value;
  int wide;
  int mindex;
  int chnum;
};
