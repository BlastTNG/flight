#define FILETYPE 0x0001

#define EXPOSURE2I (65536. / 5000000.)  /* ISC exposure time to int */

extern int frame_num;

void do_Tx_frame(int bbc_fp, unsigned int *TxFrame,
                 unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW],
                 unsigned short *RxFrame, int reset);

#define WriteSlow(c, i, v) slowTxFields[c][i] = (slowTxFields[c][i] & 0xffff0000) | ((v) & 0xffff)
#define WriteFast(c, v) TxFrame[c] = (TxFrame[c] & 0xffff0000) | ((v) & 0xffff)


int IsNewFrame();
