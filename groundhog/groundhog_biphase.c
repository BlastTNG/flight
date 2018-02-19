#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <libgen.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/statvfs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>

#include "bbc_pci.h"
#include "decom_pci.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "channels_tng.h"
#include "crc.h"
#include "blast.h"
#include "blast_time.h"
#include "groundhog_framing.h"
#include "groundhog.h"

#define FRAME_SYNC_WORD 0xEB90
#define DQ_FILTER 0.4
#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)

#define DEV "/dev/decom_pci"

superframes_list_t superframes;

void biphase_receive(void *args)
{
  int decom_fp;

  uint16_t out_frame[BI0_FRAME_SIZE+3];
  uint16_t anti_out_frame[BI0_FRAME_SIZE+3];

  int16_t du; // num_unlocked: info only
  int status; // unlocked, searching. or locked.
  uint16_t polarity;
  int system_idled; // if 1, don't write to disk
  uint16_t crc_ok; // 1 if crc was ok
  // unsigned long frame_counter;
  double dq_bad;  // data quality - fraction of bad crcs

  bool debug_rate = false;
  uint16_t debug_counter = 0;
  uint16_t previous_counter = 0;

  uint16_t raw_word_in;
  int i_word = 0;
  int read_data = 0;
  uint16_t crc_pos, crc_neg;
  const uint16_t sync_word = 0xeb90;

  buos_use_stdio();

  /* Open Decom */
  if ((decom_fp = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
      berror(fatal, "fatal error opening " DEV);
  }

  /* Initialise Decom */
  ioctl(decom_fp, DECOM_IOC_RESET);
  ioctl(decom_fp, DECOM_IOC_FRAMELEN, BI0_FRAME_SIZE-1);

  /* set up our outputs */
  openlog("decomd", LOG_PID, LOG_DAEMON);
  // buos_use_syslog();


  initialize_circular_superframes(&superframes);

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
                        // push_superframe(out_frame, &superframes);
                    } else if ((uint16_t)(~raw_word_in) == crc_neg) {
                        crc_ok = 1;
                        polarity = 0;
                        // push_superframe(anti_out_frame, &superframes);
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


#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))
void biphase_publish(void *args){

    static char frame_name[RATE_END][32];
    void *biphase_data[RATE_END] = {0};

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
        biphase_data[rate] = calloc(1, allocated_size);
    }
 
    for (int rate = 0; rate < RATE_END; rate++) {
        char rate_name[16];
        strcpy(rate_name, RATE_LOOKUP_TABLE[rate].text);
        rate_name[strlen(rate_name)-1] = 'z';
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/biphase/%s", rate_name);
        blast_info("there will be a topic with name %s", frame_name[rate]);
    }

    while (true) {
        write_frame = superframes.i_out;
        read_frame = superframes.i_in;

        if (read_frame == write_frame) {
            usleep(100);
            continue;
        }
        while (read_frame != write_frame) {
            int counter_488hz = 1;
            int counter_244hz = 1;
            int counter_200hz = 1;
            int counter_100hz = 1;
            int counter_5hz = 1;
            int counter_1hz = 1;
            int frame_488_counter = 0;

            while(frame_488_counter < 488) {
                const struct timespec interval_ts = {.tv_sec = 0, .tv_nsec = MCP_NS_PERIOD};
                ts = timespec_add(ts, interval_ts);
                clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

                if (!--counter_1hz) {
                    counter_1hz = HZ_COUNTER(1);
                    extract_frame_from_superframe(biphase_data[RATE_1HZ], RATE_1HZ, superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_1HZ], "biphase", RATE_1HZ);
                    //printf("1Hz\n");
                }
                  if (!--counter_5hz) {
                    counter_5hz = HZ_COUNTER(5);
                    extract_frame_from_superframe(biphase_data[RATE_5HZ], RATE_5HZ, superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_5HZ], "biphase", RATE_5HZ);
                    //printf("5Hz\n");
                }
                if (!--counter_100hz) {
                    counter_100hz = HZ_COUNTER(100);
                    extract_frame_from_superframe(biphase_data[RATE_100HZ], RATE_100HZ, superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_100HZ], "biphase", RATE_100HZ);
                    //printf("100Hz\n");
                }
                if (!--counter_200hz) {
                    counter_200hz = HZ_COUNTER(200);
                    extract_frame_from_superframe(biphase_data[RATE_200HZ], RATE_200HZ, superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_200HZ], "biphase", RATE_200HZ);
                    //printf("200Hz\n");
                }
                if (!--counter_244hz) {
                    counter_244hz = HZ_COUNTER(244);
                    extract_frame_from_superframe(biphase_data[RATE_244HZ], RATE_244HZ, superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_244HZ], "biphase", RATE_244HZ);
                    //printf("244Hz\n");
                }
                if (!--counter_488hz) {
                    counter_488hz = HZ_COUNTER(488);
                    extract_frame_from_superframe(biphase_data[RATE_488HZ], RATE_488HZ, superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_488HZ], "biphase", RATE_488HZ);
                    frame_488_counter++;
                    //printf("488Hz\n");
                }
            }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
            superframes.i_out = write_frame;
        }
    }
}
