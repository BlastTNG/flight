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
#include "linklist.h"
#include "bitserver.h"
#include "FIFO.h"

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)
#define BIPHASE_PACKET_SIZE (BIPHASE_FRAME_SIZE_BYTES-2-PACKET_HEADER_SIZE)
#define BIPHASE_PACKET_WORD_START (7)

#define MAX_LL_SIZE (250000)

#define DEV "/dev/decom_pci"

superframes_list_t biphase_superframes;

void print_packet(uint8_t * packet, size_t length) {
    printf("Packet Received is: \n");
    for (int i = 0; i < length; i++) {
        printf("%04x ", *(packet + i));
    }
}

void get_stripped_packet(bool *normal_polarity, uint8_t *receive_buffer_stripped, const uint16_t *receive_buffer, 
                         const uint16_t *anti_receive_buffer, linklist_t **ll, uint16_t **i_pkt, uint16_t **n_pkt) {

  uint32_t *serial;
  uint32_t *frame_number;

  if (*normal_polarity) {
      // Reading the header
      readHeader((uint8_t *) (receive_buffer+1), &serial, &frame_number, i_pkt, n_pkt);
      // Checking for polarity
      if ((*ll = linklist_lookup_by_serial(*serial))) {
          //blast_dbg("\n=== I found a linklist with serial %x, counter: %x, i_pkt: %d, n_pkt: %d==\n", *serial, *frame_number, **i_pkt, **n_pkt);
          *normal_polarity = true;
          memcpy(receive_buffer_stripped, receive_buffer+BIPHASE_PACKET_WORD_START, BIPHASE_PACKET_SIZE);
          // print_packet(receive_buffer_stripped, BIPHASE_PACKET_SIZE);
      } else if ((*ll = linklist_lookup_by_serial((uint16_t) ~(*serial)))) {
          *normal_polarity = false;
      }
  }
  if (!(*normal_polarity)) {
      // Reading the header
      readHeader((uint8_t *) (anti_receive_buffer+1), &serial, &frame_number, i_pkt, n_pkt);
      // Checking for polarity
      if ((*ll = linklist_lookup_by_serial(*serial))) {
          //blast_dbg("Inverted 1: I found a linklist with serial %x, counter: %d, i_pkt: %d, n_pkt: %d\n", *serial, *frame_number, **i_pkt, **n_pkt);
          *normal_polarity = false;
          memcpy(receive_buffer_stripped, anti_receive_buffer+BIPHASE_PACKET_WORD_START, BIPHASE_PACKET_SIZE);
          // print_packet(receive_buffer_stripped, BIPHASE_PACKET_SIZE);
      } else if ((*ll = linklist_lookup_by_serial((uint16_t) ~(*serial)))) {
          *normal_polarity = true;
          readHeader((uint8_t *) (receive_buffer+1), &serial, &frame_number, i_pkt, n_pkt);
          //blast_dbg("Inverted 2: I found a linklist with serial %x, counter: %d, i_pkt: %d, n_pkt: %d\n", *serial, *frame_number, **i_pkt, **n_pkt);
          memcpy(receive_buffer_stripped, receive_buffer+BIPHASE_PACKET_WORD_START, BIPHASE_PACKET_SIZE);
          // print_packet(receive_buffer_stripped, BIPHASE_PACKET_SIZE);
      }
  }
}

void biphase_receive(void *args)
{

  int decom_fp;
  int i_word = 0;
  int16_t du; // num_unlocked: info only
  const uint16_t sync_word = 0xeb90;
  uint16_t raw_word_in;

  uint16_t receive_buffer[BI0_FRAME_SIZE];
  uint16_t anti_receive_buffer[BI0_FRAME_SIZE];
  uint8_t receive_buffer_stripped[BIPHASE_PACKET_SIZE];
  uint8_t *compressed_linklist = calloc(1, MAX_LL_SIZE);
  uint32_t compressed_linklist_size = 0;

  linklist_t *ll = NULL;
  uint16_t *i_pkt;
  uint16_t *n_pkt;
  int retval;
  uint8_t *local_superframe = allocate_superframe();
  bool normal_polarity = true;

  // buos_use_stdio();

  /* Open Decom */
  if ((decom_fp = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
      berror(fatal, "fatal error opening " DEV);
  }

  /* Initialise Decom */
  ioctl(decom_fp, DECOM_IOC_RESET);
  // ioctl(decom_fp, DECOM_IOC_FRAMELEN, BI0_FRAME_SIZE-1);
  ioctl(decom_fp, DECOM_IOC_FRAMELEN, 2*BI0_FRAME_SIZE-1);

  /* set up our outputs */
  openlog("decomd", LOG_PID, LOG_DAEMON);
  // buos_use_syslog();

  initialize_circular_superframes(&biphase_superframes);

  while(true) {
      while ((read(decom_fp, &raw_word_in, sizeof(uint16_t))) > 0) {
          // Fill receive and anti receive
          receive_buffer[i_word] = raw_word_in;
          anti_receive_buffer[i_word] = ~raw_word_in;
          if (i_word == 0) {
              // Beginning of packet?
              du = ioctl(decom_fp, DECOM_IOC_NUM_UNLOCKED);
              if ((raw_word_in != sync_word) && (raw_word_in != (uint16_t) ~sync_word)) {
                  // This was not the beginning actually
                  i_word = 0;
                  continue;
              }
           //   printf("\n=== Frame Start ==\n");
          } else if ((i_word) == (BI0_FRAME_SIZE-1)) {
              get_stripped_packet(&normal_polarity, receive_buffer_stripped, receive_buffer, anti_receive_buffer, 
                                 &ll, &i_pkt, &n_pkt);
              retval = depacketizeBuffer(compressed_linklist, &compressed_linklist_size, BIPHASE_PACKET_SIZE, 
                                       i_pkt, n_pkt, receive_buffer_stripped);
              memset(receive_buffer_stripped, 0, BIPHASE_PACKET_SIZE);
              if ((retval == 0) && (ll != NULL)){
                  // The compressed linklist has been fully reconstructed
                  printf("holy fucking shit we have a linklist!! Serial: %x\n", *(uint32_t *) ll->serial);
                  decompress_linklist(local_superframe, ll, compressed_linklist);
                  push_superframe(local_superframe, &biphase_superframes);
                  memset(compressed_linklist, 0, compressed_linklist_size);
                  compressed_linklist_size = 0;
              }
          }
          i_word++;
          i_word = (i_word % BI0_FRAME_SIZE);
          //printf("%04x ", raw_word_in);
      }
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
        write_frame = biphase_superframes.i_out;
        read_frame = biphase_superframes.i_in;

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
                    extract_frame_from_superframe(biphase_data[RATE_1HZ], RATE_1HZ, biphase_superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_1HZ], "biphase", RATE_1HZ);
                    //printf("1Hz\n");
                }
                  if (!--counter_5hz) {
                    counter_5hz = HZ_COUNTER(5);
                    extract_frame_from_superframe(biphase_data[RATE_5HZ], RATE_5HZ, biphase_superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_5HZ], "biphase", RATE_5HZ);
                    //printf("5Hz\n");
                }
                if (!--counter_100hz) {
                    counter_100hz = HZ_COUNTER(100);
                    extract_frame_from_superframe(biphase_data[RATE_100HZ], RATE_100HZ, biphase_superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_100HZ], "biphase", RATE_100HZ);
                    //printf("100Hz\n");
                }
                if (!--counter_200hz) {
                    counter_200hz = HZ_COUNTER(200);
                    extract_frame_from_superframe(biphase_data[RATE_200HZ], RATE_200HZ, biphase_superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_200HZ], "biphase", RATE_200HZ);
                    //printf("200Hz\n");
                }
                if (!--counter_244hz) {
                    counter_244hz = HZ_COUNTER(244);
                    extract_frame_from_superframe(biphase_data[RATE_244HZ], RATE_244HZ, biphase_superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_244HZ], "biphase", RATE_244HZ);
                    //printf("244Hz\n");
                }
                if (!--counter_488hz) {
                    counter_488hz = HZ_COUNTER(488);
                    extract_frame_from_superframe(biphase_data[RATE_488HZ], RATE_488HZ, biphase_superframes.framelist[write_frame]);
                    framing_publish(biphase_data[RATE_488HZ], "biphase", RATE_488HZ);
                    frame_488_counter++;
                    //printf("488Hz\n");
                }
            }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
            biphase_superframes.i_out = write_frame;
        }
    }
}
