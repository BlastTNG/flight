#include <unistd.h>
#include <sys/ioctl.h>
#include "bbc_pci.h"
#include "decom_pci.h"
#include "channels_tng.h"
#include "crc.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.9977

extern int decom;

extern uint32_t FrameBuf[BI0_FRAME_SIZE+3];
extern uint32_t AntiFrameBuf[BI0_FRAME_SIZE+3];
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
  uint32_t buf;
  int i_word = 0;
  int read_data = 0;
  uint32_t crc_buf;

  for (;;) {
    while ((read(decom, &buf, sizeof(unsigned int))) > 0) {
#ifdef DEBUG
      fwrite(&buf, sizeof(uint32_t), 1, dump);
#endif
      read_data = 1;
      FrameBuf[i_word] = buf;
      AntiFrameBuf[i_word] = ~buf;
      if (i_word % BI0_FRAME_SIZE == 0) { /* begining of frame */
        du = ioctl(decom, DECOM_IOC_NUM_UNLOCKED);
        wfifo_size = ioctl(decom, DECOM_IOC_FIONREAD);
        if ((buf != FRAME_SYNC_WORD) && ((~buf & 0xffffffff) != FRAME_SYNC_WORD)) { /* wait for frame sync word */
          status = 0;
          i_word = 0;
        } else { /* We have found the sync word */
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

	  uint32_t crc_pos = 0xFFFFFFFF;
	  uint32_t crc_neg = 0xFFFFFFFF;
	  crc_pos = crc32_be(crc_pos, (uint8_t*)FrameBuf, BiPhaseFrameWords);
	  crc_neg = crc32_be(crc_neg, (uint8_t*)AntiFrameBuf, BiPhaseFrameWords);
	  crc_buf = buf;
          if (buf == crc_pos) {
            crc_ok = 1;
            polarity = 1;
          } else if ((uint32_t)~buf == crc_neg) {
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
