
#define FILETYPE 0x0001

extern int frame_num;

void do_Tx_frame(int bbc_fp, unsigned int *Txframe,
                 unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW],
                 unsigned short *Rxframe, int reset);

/************************************************************************ 
 *                                                                      * 
 *   FrameWords: Returns the size of the rx and downlink frames         * 
 *                                                                      * 
 ************************************************************************/
#define FrameWords() \
      (1   /* FILETYPE */ \
      + 2 /* FRAMENUM */ \
      + 1 /* MP_INDEX */ \
      + N_SLOW /* slow channel spots */ \
      + N_FASTCHLIST)

#define TxFrameBytes() (FrameWords() * sizeof(unsigned int))

#define WriteSlow(c, i, v) slowTxFields[c][i] = (slowTxFields[c][i] & 0xffff0000) | ((v) & 0xffff)
#define WriteFast(c, v) Txframe[c] = (Txframe[c] & 0xffff0000) | ((v) & 0xffff)


int IsNewFrame();

