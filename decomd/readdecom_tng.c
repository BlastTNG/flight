#include <unistd.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include "bbc_pci.h"
#include "decom_pci.h"
#include "channels_tng.h"
#include "crc.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.9977

extern int decom_fp;

extern uint16_t out_frame[BI0_FRAME_SIZE+3];
extern uint16_t anti_out_frame[BI0_FRAME_SIZE+3];
extern int16_t du; // num_unlocked: info only
extern int status; // unlocked, searching. or locked.
extern uint16_t polarity;
extern int system_idled; // if 1, don't write to disk
extern uint16_t crc_ok; // 1 if crc was ok
extern unsigned long frame_counter;
extern double dq_bad;  // data quality - fraction of bad crcs

void pushDiskFrame(unsigned short *RxFrame);


void ReadDecom (void)
{
  uint16_t raw_word_in;
  int i_word = 0;
  int read_data = 0;
  uint16_t crc_pos, crc_neg;
  const uint16_t sync_word = 0xeb90;

  while(true) {
      while ((read(decom_fp, &raw_word_in, sizeof(uint16_t))) > 0) {
          read_data = 1;
          out_frame[i_word] = raw_word_in;
          anti_out_frame[i_word] = ~raw_word_in;
          if (i_word % BI0_FRAME_SIZE == 0) { /* begining of frame */
              du = ioctl(decom_fp, DECOM_IOC_NUM_UNLOCKED);
              if ((raw_word_in != sync_word) && (raw_word_in != (uint16_t) ~sync_word)) {
                  status = 0;
                  i_word = 0;
              } else {
                  if (status < 2) {
                      status++;
                  } else {
                      if (polarity) {
                          out_frame[BI0_FRAME_SIZE] = crc_ok;
                          out_frame[BI0_FRAME_SIZE + 1] = polarity;
                          out_frame[BI0_FRAME_SIZE + 2] = du;
                          // if (!system_idled) {
                          //   pushDiskFrame(out_frame);
                          //   frame_counter++;
                          // }
                      } else {
                          anti_out_frame[BI0_FRAME_SIZE] = crc_ok;
                          anti_out_frame[BI0_FRAME_SIZE + 1] = polarity;
                          anti_out_frame[BI0_FRAME_SIZE + 2] = du;
                          // if (!system_idled) {
                          //   pushDiskFrame(anti_out_frame);
                          //   frame_counter++;
                          // }
                      }
                  }
                  if (crc_ok == 1) {
                      dq_bad *= DQ_FILTER;
                  } else {
                      dq_bad = dq_bad * DQ_FILTER + (1.0 - DQ_FILTER);
                  }
                  i_word++;
              }
          } else {
              if ((i_word) == (BI0_FRAME_SIZE-1)) {
                    out_frame[0] = anti_out_frame[0] = 0xEB90;
  
                    crc_pos = crc16(CRC16_SEED, (uint8_t*)out_frame, BI0_FRAME_SIZE-1);
                    crc_neg = crc16(CRC16_SEED, (uint8_t*)anti_out_frame, BI0_FRAME_SIZE-1);
                    if (raw_word_in == crc_pos) {
                        crc_ok = 1;
                        polarity = 1;
                    } else if ((uint16_t)(~raw_word_in) == crc_neg) {
                        polarity = 0;
                        crc_ok = 1;
                    } else {
                        crc_ok = 0;
                    }
              }
              if (++i_word >= BI0_FRAME_SIZE) {
                i_word = 0;
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
