#define EXPOSURE2I (65536. / 5000000.)  /* ISC exposure time to int */

#define NIOS_QUEUE  0
#define NIOS_FLUSH -1

#define WHICH 0

#include "channels.h"

extern int frame_num;

void InitTxFrame(void);
void UpdateBBCFrame(unsigned short*);

void RawNiosWrite(unsigned int, unsigned int, int);
void WriteData(struct NiosStruct*, unsigned int, int);
