#include <unistd.h>
#include <sys/ioctl.h>
#include "bbc_pci.h"
#include "decom_pci.h"
#include "channels.h"
#include "crc.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.9977

#define SYNC_LEN 5

extern int decom;

extern unsigned short FrameBuf[BI0_FRAME_SIZE+3];
extern unsigned short AntiFrameBuf[BI0_FRAME_SIZE+3];
extern int du; // num_unlocked: info only
extern int wfifo_size; // write fifo size: info only
extern int status; // unlocked, searching. or locked.
extern unsigned short polarity;
extern int system_idled; // if 1, don't write to disk
extern unsigned short crc_ok; // 1 if crc was ok
extern unsigned long frame_counter;
extern double dq_bad;  // data quality - fraction of bad crcs


void pushDiskFrame(unsigned short *RxFrame);

void ReadDecom (void)
{
  unsigned short word_in, raw_word_in;
  int i_word = BiPhaseFrameWords+1;
  unsigned short crc_pos;
  unsigned short syncwords[SYNC_LEN] = {0xeb90, 0xc5c5, 0x3a3a, 0x146f,0};

  int syncstate = 0, sync_pol=0;
  int polarity = 0;
  int since_last = 0;
  
  syncwords[SYNC_LEN-1] = BiPhaseFrameWords;
  /* Stuff we need to ad to the loop besides the main stuff
*/
  
  for (;;) {
    while ((read(decom, &word_in, sizeof(unsigned short))) > 0) {
      raw_word_in = word_in;
      if (polarity) word_in = ~word_in; //flip if inverse polarity
      // Look for sync word...
      if (syncstate == 0) {
        if (raw_word_in == syncwords[0]) {
	  //printf("found syncword0 (%d)\n", i_word);
          syncstate = 1;
          sync_pol = 0;
        } else if (raw_word_in == (unsigned short)~syncwords[0]) {
	  //printf("found ~syncword0 (%d)\n", i_word);
          syncstate = 1;
          sync_pol = 1;
        } else {
	  since_last++;
	  if (since_last>2*BiPhaseFrameWords) {
	    status = 1;// searching
	  }
	}
      } else if (syncstate > 0 && raw_word_in == (unsigned short)
          (sync_pol ? ~syncwords[syncstate] : syncwords[syncstate])) {
        syncstate++;
        //printf("%x syncstate now %d\n",word_in, syncstate);
      } else if (syncstate > 1) {
	unsigned short jj = (sync_pol ? ~raw_word_in : raw_word_in);
	//printf("error %x instead of %x i_word: %d syncstate: %d\n", jj, syncwords[syncstate], i_word, syncstate);
	syncstate = 0;
      } else {
	syncstate = 0;
      }
      
      if (syncstate == SYNC_LEN) {  //full sync word detected
	//printf("new frame: i_word: %d polarity: %d\n", i_word, sync_pol);
	i_word = 1;
	polarity = sync_pol;
	FrameBuf[0] = 0xeb90;
	syncstate = 0;
	since_last = 0;
	status = 2; // locked
      } else if (i_word<BiPhaseFrameWords) {
	FrameBuf[i_word] = word_in;
	i_word++;
      } else if (i_word == BiPhaseFrameWords) {
        du = ioctl(decom, DECOM_IOC_NUM_UNLOCKED);
        wfifo_size = ioctl(decom, DECOM_IOC_FIONREAD);
        crc_pos = CalculateCRC(0, FrameBuf+1, 
			       sizeof(short)*(BiPhaseFrameWords-1));
        crc_ok = (word_in == crc_pos);
	if (crc_ok) {
          dq_bad *= DQ_FILTER;
        } else {
          dq_bad = dq_bad * DQ_FILTER + (1.0 - DQ_FILTER);
	}
        FrameBuf[BiPhaseFrameWords] = crc_ok;
        FrameBuf[BiPhaseFrameWords + 1] = polarity;
        FrameBuf[BiPhaseFrameWords + 2] = du;

        if (!system_idled) {
          pushDiskFrame(FrameBuf);
          frame_counter++;
        }

	i_word++;
      }
    }
    usleep(5000);
    since_last += BiPhaseFrameWords/2;
    if (since_last>2*BiPhaseFrameWords) {
      status = 0; // no data coming in: lost
    }
  }
}
