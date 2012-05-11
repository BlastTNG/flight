#include <unistd.h>
#include <sys/ioctl.h>
#include "bbc_pci.h"
#include "decom_pci.h"
#include "channels.h"
#include "crc.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.9977

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
  unsigned short buf;
  int i_word = 0;
  int read_data = 0;
  unsigned short crc_pos;
  unsigned short crc_neg;
  unsigned short crc_buf;

  for (;;) {
    while ((read(decom, &buf, sizeof(unsigned short))) > 0) {
#ifdef DEBUG
      fwrite(&buf, sizeof(unsigned short), 1, dump);
#endif
      read_data = 1;
      FrameBuf[i_word] = buf;
      AntiFrameBuf[i_word] = ~buf;
      if (i_word % BI0_FRAME_SIZE == 0) { /* begining of frame */
        du = ioctl(decom, DECOM_IOC_NUM_UNLOCKED);
        wfifo_size = ioctl(decom, DECOM_IOC_FIONREAD);
        if ((buf != FRAME_SYNC_WORD) && ((~buf & 0xffff) != FRAME_SYNC_WORD)) {
          status = 0;
          i_word = 0;
        } else {
          if (status < 2) {
            status++;
          } else {
            if (polarity) {
              FrameBuf[BiPhaseFrameWords] = crc_ok;
              FrameBuf[BiPhaseFrameWords + 1] = polarity;
              FrameBuf[BiPhaseFrameWords + 2] = du;
              if (!system_idled) {
                pushDiskFrame(FrameBuf);
                frame_counter++;
              }
            } else {
              AntiFrameBuf[BiPhaseFrameWords] = crc_ok;
              AntiFrameBuf[BiPhaseFrameWords + 1] = polarity;
              AntiFrameBuf[BiPhaseFrameWords + 2] = du;
              if (!system_idled) {
                pushDiskFrame(AntiFrameBuf);
                frame_counter++;
              }
            }
          }

          if (crc_ok==1) {
            dq_bad *= DQ_FILTER;
          } else {
            dq_bad = dq_bad * DQ_FILTER + (1.0 - DQ_FILTER);
	  }

          i_word++;
        }
      } else {
        if (++i_word >= BI0_FRAME_SIZE) {
          i_word = 0;
	}

        if (i_word - 1 == BiPhaseFrameWords) {
          FrameBuf[0] = AntiFrameBuf[0] = 0xEB90;

          crc_pos = CalculateCRC(CRC_SEED, FrameBuf, BiPhaseFrameWords);
          crc_neg = CalculateCRC(CRC_SEED, AntiFrameBuf, BiPhaseFrameWords);
	  crc_buf = buf;
          if (buf == crc_pos) {
            crc_ok = 1;
            polarity = 1;
          } else if ((unsigned short)~buf == crc_neg) {
            polarity = 0;
            crc_ok = 1;
          } else {
            crc_ok = 0;
	  }
        }
      }
    }

    if (!read_data) {
      status = 0;
    } else {
      read_data = 0;
    }

    usleep(5000);
  }
}
