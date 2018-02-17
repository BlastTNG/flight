#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <math.h>
#include "bbc_pci.h"
#include "decom_pci.h"
#include "decomd.h"
#include "channels_tng.h"
#include "crc.h"
#include "blast.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.4
#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)

extern int decom_fp;

extern uint16_t out_frame[BI0_FRAME_SIZE+3];
extern uint16_t anti_out_frame[BI0_FRAME_SIZE+3];
extern superframes_list_t superframes;

extern int16_t du; // num_unlocked: info only
extern int status; // unlocked, searching. or locked.
extern uint16_t polarity;
extern int system_idled; // if 1, don't write to disk
extern uint16_t crc_ok; // 1 if crc was ok
extern unsigned long frame_counter;
extern double dq_bad;  // data quality - fraction of bad crcs


bool debug_rate = false;
uint16_t debug_counter = 0;
uint16_t previous_counter = 0;


void biphase_receive(void)
{
  uint16_t raw_word_in;
  int i_word = 0;
  int read_data = 0;
  uint16_t crc_pos, crc_neg;
  const uint16_t sync_word = 0xeb90;

  initialize_circular_superframes();

  while(true) {
      while ((read(decom_fp, &raw_word_in, sizeof(uint16_t))) > 0) {
          read_data = 1;
          out_frame[i_word] = raw_word_in;
          anti_out_frame[i_word] = ~raw_word_in;
          if (debug_rate) {
            printf("i_word=%d, raw_word_in = %04x\n", i_word, raw_word_in);
          }
          if (i_word == 2 && false) {
            if (abs(raw_word_in - previous_counter) != 2) {
                printf("We have missed %d frames\n", abs(raw_word_in - previous_counter));
            }
            previous_counter = raw_word_in;
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
                debug_counter += 1;
                if (debug_counter % 10 == 0) {
                    printf("fraction of bad crc weighted: %f\n", dq_bad);
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
                        push_superframe(out_frame);
                    } else if ((uint16_t)(~raw_word_in) == crc_neg) {
                        crc_ok = 1;
                        polarity = 0;
                        push_superframe(anti_out_frame);
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

void biphase_publish(){

    static char frame_name[RATE_END][32];
    int frame_offset = 0;
    void *biphase_data[RATE_END] = {0};

    uint16_t    read_frame;
    uint16_t    write_frame;

    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
        biphase_data[rate] = calloc(1, allocated_size);
    }
 
    for (int rate = 0; rate < RATE_END; rate++) {
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/biphase/%s", RATE_LOOKUP_TABLE[rate].text);
        blast_info("there will be a topic with name %s", frame_name[rate]);
    }

    while (true) {
        write_frame = superframes.i_out;
        read_frame = superframes.i_in;

        if (read_frame == write_frame) {
            usleep(10000);
            continue;
        }
        while (read_frame != write_frame) {
            for (int rate = 0; rate < RATE_END; rate++) {
                int freq = groundhog_get_rate(rate);
                for (int i = 0; i < freq; i++) {
                    frame_offset = i*frame_size[rate];
                    memcpy(biphase_data[rate], superframes.framelist[write_frame]+frame_offset, frame_size[rate]);
                    framing_publish_200hz(biphase_data[rate], "biphase");
                }
            }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
        }
        superframes.i_out = write_frame;
        usleep(10000);
    }
}
