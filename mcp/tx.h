#define FILETYPE 0x0001

#define EXPOSURE2I (65536. / 5000000.)  /* ISC exposure time to int */

extern int frame_num;

void InitTxFrame(void);
void UpdateBBCFrame(unsigned short *RxFrame);

void RawNiosWrite(unsigned int addr, unsigned int data);
void WriteData(struct NiosStruct*, unsigned int);
