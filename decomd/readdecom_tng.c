#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include "bbc_pci.h"
#include "decom_pci.h"
#include "decomd.h"
#include "channels_tng.h"
#include "crc.h"
#include "blast.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.9977
#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)

extern int decom_fp;

extern uint16_t out_frame[BI0_FRAME_SIZE+3];
extern uint16_t anti_out_frame[BI0_FRAME_SIZE+3];
extern frames_list_t frames;

extern int16_t du; // num_unlocked: info only
extern int status; // unlocked, searching. or locked.
extern uint16_t polarity;
extern int system_idled; // if 1, don't write to disk
extern uint16_t crc_ok; // 1 if crc was ok
extern unsigned long frame_counter;
extern double dq_bad;  // data quality - fraction of bad crcs

bool debug_rate = true;

// void pushDiskFrame(unsigned short *RxFrame);

void initialize_biphase_frame(void)
{
    int i;
    frames.i_in = 0;
    frames.i_out = 0;
    for (i = 0; i < NUM_FRAMES; i++) {
        frames.framelist[i] = calloc(1, BIPHASE_FRAME_SIZE_BYTES);
        memset(frames.framelist[i], 0, BIPHASE_FRAME_SIZE_BYTES);
    }
}

void push_biphase_frame(const void *m_frame)
{
    int i_in;
    i_in = (frames.i_in + 1) & BI0_FRAME_BUFMASK;
    frames.framesize[i_in] = BIPHASE_FRAME_SIZE_BYTES;
    memcpy(frames.framelist[i_in], m_frame, BIPHASE_FRAME_SIZE_BYTES);
    frames.i_in = i_in;
}


void ReadDecom (void)
{
  uint16_t raw_word_in;
  int i_word = 0;
  int read_data = 0;
  uint16_t crc_pos, crc_neg;
  const uint16_t sync_word = 0xeb90;

  initialize_biphase_frame();

  while(true) {
      while ((read(decom_fp, &raw_word_in, sizeof(uint16_t))) > 0) {
          read_data = 1;
          out_frame[i_word] = raw_word_in;
          anti_out_frame[i_word] = ~raw_word_in;
          if (debug_rate) {
            printf("i_word=%d, raw_word_in = %04x\n", i_word, raw_word_in);
          }
          if (i_word % BI0_FRAME_SIZE == 0) { /* begining of frame */
            if (debug_rate) {
                printf("=================i_word=%d==============================\n", i_word);
                printf("This should be frame start: it's been BIO_FRAME_SIZE_WORDS since last sync word\n");
            }
            du = ioctl(decom_fp, DECOM_IOC_NUM_UNLOCKED);
            if ((raw_word_in != sync_word) && (raw_word_in != (uint16_t) ~sync_word)) {
                status = 0;
                i_word = 0;
            } else {
                if (debug_rate) {
                    printf("=== FRAME START! i_word=%d===\n== Got sync word %04x ==\n", i_word, raw_word_in);
                }
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
                    if (debug_rate) {
                        printf("== This is the last word: i_word=%d, and BI0_FRAME_SIZE=%d\n", i_word, BI0_FRAME_SIZE);
                        printf("The last word received (normally the CRC) is %04x\n", raw_word_in); 
                        printf("The last word received negative is (normally the CRC) is %04x\n", (~raw_word_in)&0xffff); 
                        if (false) {
                            printf("==frame==\n");
                            for (int j=0; j<BI0_FRAME_SIZE; j++) {
                                printf("%02x ", out_frame[j]);
                            }
                            printf("========\n== anti frame==\n");
                            for (int j=0; j<BI0_FRAME_SIZE; j++) {
                                printf("%04x ", anti_out_frame[j]);
                            }
                            printf("\n-------------------\n");
                        }
                    }
                    out_frame[0] = anti_out_frame[0] = 0xEB90;
                    crc_pos = crc16(CRC16_SEED, out_frame, BI0_FRAME_SIZE*sizeof(uint16_t)-2);
                    crc_neg = crc16(CRC16_SEED, anti_out_frame, BI0_FRAME_SIZE*sizeof(uint16_t)-2);
                    if (debug_rate) {
                        printf("The CRC_POS computed is %04x\n", crc_pos);
                        printf("The CRC_NEG computed is %04x\n", crc_neg);
                    }
                    if (raw_word_in == crc_pos) {
                        crc_ok = 1;
                        polarity = 1;
                        push_biphase_frame(out_frame);
                    } else if ((uint16_t)(~raw_word_in) == crc_neg) {
                        crc_ok = 1;
                        polarity = 0;
                        push_biphase_frame(anti_out_frame);
                    } else {
                        crc_ok = 0;
                    }
                    if (debug_rate) {
                        blast_dbg("Last word of frame, is crc ok? %d\n======================\n", (int) crc_ok);
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
