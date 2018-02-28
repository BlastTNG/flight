#include <math.h>
#include <fcntl.h>
#include <stdbool.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // threads
#include <openssl/md5.h>
#include <float.h>
#include <time.h>
#include <sys/time.h>

#include "bitserver.h"
#include "linklist.h" // This gives access to channel_list and frame_size
#include "linklist_compress.h"
#include "blast.h"
#include "blast_time.h"
#include "groundhog_framing.h"
#include "highrate.h"
#include "comms_serial.h"


superframes_list_t tdrss_superframes;

int blocking_read(int fd, uint8_t * buffer, unsigned int num_bytes)
{
    int i = 0;
    while (1) { 
        if (read(fd, buffer+i, 1)) {
            i++;
        } else {
            usleep(1000);
        }
        if (i >= num_bytes) break;
    }
    return num_bytes;
}

void tdrss_receive(void *arg) {

  linklist_t * ll = NULL;

  uint8_t *local_superframe = allocate_superframe();
  uint8_t *compressed_buffer = calloc(1, HIGHRATE_MAX_SIZE);

  uint8_t *header_buffer = calloc(1, PACKET_HEADER_SIZE);

  uint32_t *serial_number = 0;
  uint16_t *i_pkt, *n_pkt;
  uint32_t *frame_number;
  uint32_t transmit_size;

  initialize_circular_superframes(&tdrss_superframes);

  // Open serial port
  comms_serial_t *serial = comms_serial_new(NULL);
  comms_serial_connect(serial, HIGHRATE_PORT);
  comms_serial_setspeed(serial, B115200);
  int fd = serial->sock->fd; 

  uint8_t byte = 0;

  while (true) {
      int byte_pos = 0;
      // perform a search for a valid serial
      while (1) {
          if (read(fd, &byte, 1)) {
              if (byte_pos >= 3) { // shift the bytes
                  for (int i = 0; i < 3; i++) {
                      header_buffer[i] = header_buffer[i+1];
                  }
              } else {
                  byte_pos++;
              }
              header_buffer[byte_pos] = byte;
              if ((ll = linklist_lookup_by_serial(*(uint32_t *) header_buffer))) break;
          } else {
              usleep(1000);
          }
      }
       
      // read the rest of the header
      blocking_read(fd, header_buffer+4, PACKET_HEADER_SIZE-4);
    
      readHeader(header_buffer, &serial_number, &frame_number, &i_pkt, &n_pkt);
 
      // hijack frame number for transmit size
      transmit_size = *frame_number;
      if (transmit_size > ll->blk_size) {
          blast_err("Transmit size larger than assigned linklist");
          transmit_size = ll->blk_size;
      }

      // blast_info("Transmit size=%d, blk_size=%d", transmit_size, ll->blk_size);

      blocking_read(fd, compressed_buffer, transmit_size);

      // decompress the linklist
      if (!read_allframe(local_superframe, compressed_buffer)) {
          blast_info("[TDRSS HGA] Received linklist with serial_number 0x%x\n", *serial_number);
          if (!decompress_linklist_by_size(local_superframe, ll, compressed_buffer, transmit_size)) { 
              continue;
          }
          push_superframe(local_superframe, &tdrss_superframes);
      } else {
          blast_info("[TDRSS HGA] Received an allframe :)\n");
      }
      memset(header_buffer, 0, PACKET_HEADER_SIZE);
  }
}

#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))
void tdrss_publish(void *arg) {

    static char frame_name[RATE_END][32];
    void *tdrss_data[RATE_END] = {0};

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
        tdrss_data[rate] = calloc(1, allocated_size);
    }
 
    for (int rate = 0; rate < RATE_END; rate++) {
        char rate_name[16];
        strcpy(rate_name, RATE_LOOKUP_TABLE[rate].text);
        rate_name[strlen(rate_name)-1] = 'z';
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/tdrss/%s", rate_name);
        blast_info("there will be a topic with name %s", frame_name[rate]);
    }

    while (true) {
        write_frame = tdrss_superframes.i_out;
        read_frame = tdrss_superframes.i_in;

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
                    extract_frame_from_superframe(tdrss_data[RATE_1HZ], RATE_1HZ, tdrss_superframes.framelist[write_frame]);
                    framing_publish(tdrss_data[RATE_1HZ], "tdrss", RATE_1HZ);
                    //printf("1Hz\n");
                }
                  if (!--counter_5hz) {
                    counter_5hz = HZ_COUNTER(5);
                    extract_frame_from_superframe(tdrss_data[RATE_5HZ], RATE_5HZ, tdrss_superframes.framelist[write_frame]);
                    framing_publish(tdrss_data[RATE_5HZ], "tdrss", RATE_5HZ);
                    //printf("5Hz\n");
                }
                if (!--counter_100hz) {
                    counter_100hz = HZ_COUNTER(100);
                    extract_frame_from_superframe(tdrss_data[RATE_100HZ], RATE_100HZ, tdrss_superframes.framelist[write_frame]);
                    framing_publish(tdrss_data[RATE_100HZ], "tdrss", RATE_100HZ);
                    //printf("100Hz\n");
                }
                if (!--counter_200hz) {
                    counter_200hz = HZ_COUNTER(200);
                    extract_frame_from_superframe(tdrss_data[RATE_200HZ], RATE_200HZ, tdrss_superframes.framelist[write_frame]);
                    framing_publish(tdrss_data[RATE_200HZ], "tdrss", RATE_200HZ);
                    //printf("200Hz\n");
                }
                if (!--counter_244hz) {
                    counter_244hz = HZ_COUNTER(244);
                    extract_frame_from_superframe(tdrss_data[RATE_244HZ], RATE_244HZ, tdrss_superframes.framelist[write_frame]);
                    framing_publish(tdrss_data[RATE_244HZ], "tdrss", RATE_244HZ);
                    //printf("244Hz\n");
                }
                if (!--counter_488hz) {
                    counter_488hz = HZ_COUNTER(488);
                    extract_frame_from_superframe(tdrss_data[RATE_488HZ], RATE_488HZ, tdrss_superframes.framelist[write_frame]);
                    framing_publish(tdrss_data[RATE_488HZ], "tdrss", RATE_488HZ);
                    frame_488_counter++;
                    //printf("488Hz\n");
                }
            }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
            tdrss_superframes.i_out = write_frame;
        }
    }
}

