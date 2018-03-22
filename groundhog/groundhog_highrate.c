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

#include "FIFO.h"
#include "bitserver.h"
#include "linklist.h" // This gives access to channel_list and frame_size
#include "linklist_compress.h"
#include "blast.h"
#include "blast_time.h"
#include "groundhog_framing.h"
#include "highrate.h"
#include "comms_serial.h"

superframes_list_t highrate_superframes;

int blocking_read(int fd, uint8_t * buffer, unsigned int num_bytes)
{
    int i = 0;
    while (1) { 
        if (read(fd, buffer+i, 1) > 0) {
            i++;
        } else {
            usleep(1000);
        }
        if (i >= num_bytes) break;
    }
    return num_bytes;
}

void highrate_receive(void *arg) {
  linklist_t * ll = NULL;

  uint8_t *local_superframe = allocate_superframe();
  unsigned int payload_packet_size = HIGHRATE_DATA_PACKET_SIZE+CSBF_HEADER_SIZE+1;

  uint8_t *csbf_packet = calloc(1, payload_packet_size);
  uint16_t datasize = HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE;
  uint32_t buffer_size = ((HIGHRATE_MAX_SIZE-1)/datasize+1)*datasize;
  uint8_t *compressed_buffer = calloc(1, buffer_size);

  // buffer allocations
  uint8_t *csbf_header = csbf_packet+0;
  uint8_t *header_buffer = csbf_header+CSBF_HEADER_SIZE;
  uint8_t *data_buffer = header_buffer+PACKET_HEADER_SIZE;
  uint8_t *csbf_checksum = header_buffer+HIGHRATE_DATA_PACKET_SIZE;

  // header variables
  uint32_t *serial_number = 0;
  uint16_t *i_pkt, *n_pkt;
  uint32_t *frame_number;
  uint32_t transmit_size;

  initialize_circular_superframes(&highrate_superframes);

  // Open serial port
  comms_serial_t *serial = comms_serial_new(NULL);
  comms_serial_connect(serial, HIGHRATE_PORT);
  comms_serial_setspeed(serial, B115200);
  int fd = serial->sock->fd; 

  char source_str[32] = {0};
  int retval = 0;
  uint32_t recv_size = 0;

  unsigned int lock = 0;
  unsigned int sync = 0;
  unsigned int syncswitch = 0; // 0 = gse, 1 = payload
  uint8_t byte = 0;
  uint8_t csbf_origin = 0;
  uint8_t csbf_sync2 = 0;
  uint8_t checksum = 0;

  unsigned int gse_read = 0;
  uint16_t gse_size = 42;

  unsigned int payload_read = 0;
  uint16_t payload_size = 42;

  while (true) {
      if (read(fd, &byte, 1) == 1) {
          if ((sync == 0) && (byte == HIGHRATE_SYNC1)) {
              sync++;
          } else if (sync == 1) {
              switch (byte) {
                  case LOWRATE_COMM1_SYNC2 :
                  case LOWRATE_COMM2_SYNC2 :
                  case HIGHRATE_TDRSS_SYNC2 :
                  case HIGHRATE_IRIDIUM_SYNC2 :
                      sync++;
                      break;
                  default :
                      sync = 0;
              }
          } else if (sync == 2) {
              csbf_origin = byte;
              sync++;
          } else if (sync == 3) {
              if (byte) {
                  payload_read = 3; // start of a new csbf payload packet
                  syncswitch = 1;
              } else {
                  *csbf_checksum = byte+csbf_origin;
                  syncswitch = 0;
              }
              sync++;
          } else if (sync == 4) { // size MSB
              if (syncswitch) payload_size = byte << 8;
              else gse_size = byte << 8;
              sync++;
          } else if (sync == 5) { // size LSB
              if (syncswitch) payload_size += byte;
              else gse_size += byte;
              sync++;
          } else if (sync == CSBF_HEADER_SIZE) { // end of gse/payload header
              if (syncswitch) {
                  // printf("Have a sync from payload (read = %d, gse_read = %d)\n", payload_read, gse_read);
              } else {
                  //printf("\nHave a sync from gse (size = %d)\n", gse_size);
                  payload_read = (payload_read+payload_packet_size-6)%payload_packet_size; // overwrite the header
                  gse_read = 0; // reset the number of bytes read since last syncword
                  lock = 1; // locked onto a gse packet
              }
              sync = 0; // done reading header
          } 

          if (gse_read == gse_size) { // got all the bytes from the gse
              // check the checksum
              //printf("Received all %d bytes from gse (0x%.2x == 0x%.2x)\n", gse_read, byte, *csbf_checksum);
              if (gse_size == 255) { 
                  blast_info("Got 255 byte hk packet");
              }
              *csbf_checksum = 0; 
              gse_read = 0;
              lock = 0;
          }

          // add byte to the packet
          if (lock || sync) {
              csbf_packet[payload_read++] = byte;
              *csbf_checksum += byte;
              gse_read++;
              if (gse_size == 255) payload_read--;
          }

          if ((payload_read == (payload_size+CSBF_HEADER_SIZE)) && // got all the bytes from the payload
              (ll = linklist_lookup_by_serial(*(uint32_t *) header_buffer))) {

              // process the packet
              readHeader(header_buffer, &serial_number, &frame_number, &i_pkt, &n_pkt);
              transmit_size = *frame_number;

              // blast_info("Transmit size=%d, blk_size=%d, payload_size=%d, datasize=%d, i=%d, n=%d", transmit_size, ll->blk_size, payload_size, datasize, *i_pkt, *n_pkt);

              retval = depacketizeBuffer(compressed_buffer, &recv_size, 
                                   HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE,
                                   i_pkt, n_pkt, data_buffer);

              memset(csbf_packet, 0, payload_packet_size);

              // the packet is complete, so decompress
              if ((retval == 0) && (ll != NULL))
              {
                  // hijack frame number for transmit size
                  if (transmit_size > ll->blk_size) {
                      blast_err("Transmit size larger than assigned linklist");
                      transmit_size = ll->blk_size;
                  }

                  // decompress the linklist
                  if (read_allframe(local_superframe, compressed_buffer)) {
                      blast_info("[TDRSS HGA] Received an allframe :)\n");
                  } else {
                      blast_info("%s Received linklist \"%s\"", source_str, ll->name);
                      // blast_info("%s Received linklist with serial_number 0x%x\n", source_str, *serial_number);
                      decompress_linklist_by_size(local_superframe, ll, compressed_buffer, transmit_size);
                      push_superframe(local_superframe, &highrate_superframes);
                  }
                  memset(compressed_buffer, 0, buffer_size);
                  recv_size = 0;
              }

              payload_read = 0;
          }

          if (payload_read >= payload_packet_size) { // somehow read more than the max packet size
             payload_read = 0;
          }


      } else {
          usleep(1000);
      }

  }
}

#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))
void highrate_publish(void *arg) {

    static char frame_name[RATE_END][32];
    void *highrate_data[RATE_END] = {0};

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
        highrate_data[rate] = calloc(1, allocated_size);
    }
 
    for (int rate = 0; rate < RATE_END; rate++) {
        char rate_name[16];
        strcpy(rate_name, RATE_LOOKUP_TABLE[rate].text);
        rate_name[strlen(rate_name)-1] = 'z';
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/highrate/%s", rate_name);
        blast_info("there will be a topic with name %s", frame_name[rate]);
    }

    while (true) {
        write_frame = highrate_superframes.i_out;
        read_frame = highrate_superframes.i_in;

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
                    extract_frame_from_superframe(highrate_data[RATE_1HZ], RATE_1HZ, highrate_superframes.framelist[write_frame]);
                    framing_publish(highrate_data[RATE_1HZ], "highrate", RATE_1HZ);
                    //printf("1Hz\n");
                }
                  if (!--counter_5hz) {
                    counter_5hz = HZ_COUNTER(5);
                    extract_frame_from_superframe(highrate_data[RATE_5HZ], RATE_5HZ, highrate_superframes.framelist[write_frame]);
                    framing_publish(highrate_data[RATE_5HZ], "highrate", RATE_5HZ);
                    //printf("5Hz\n");
                }
                if (!--counter_100hz) {
                    counter_100hz = HZ_COUNTER(100);
                    extract_frame_from_superframe(highrate_data[RATE_100HZ], RATE_100HZ, highrate_superframes.framelist[write_frame]);
                    framing_publish(highrate_data[RATE_100HZ], "highrate", RATE_100HZ);
                    //printf("100Hz\n");
                }
                if (!--counter_200hz) {
                    counter_200hz = HZ_COUNTER(200);
                    extract_frame_from_superframe(highrate_data[RATE_200HZ], RATE_200HZ, highrate_superframes.framelist[write_frame]);
                    framing_publish(highrate_data[RATE_200HZ], "highrate", RATE_200HZ);
                    //printf("200Hz\n");
                }
                if (!--counter_244hz) {
                    counter_244hz = HZ_COUNTER(244);
                    extract_frame_from_superframe(highrate_data[RATE_244HZ], RATE_244HZ, highrate_superframes.framelist[write_frame]);
                    framing_publish(highrate_data[RATE_244HZ], "highrate", RATE_244HZ);
                    //printf("244Hz\n");
                }
                if (!--counter_488hz) {
                    counter_488hz = HZ_COUNTER(488);
                    extract_frame_from_superframe(highrate_data[RATE_488HZ], RATE_488HZ, highrate_superframes.framelist[write_frame]);
                    framing_publish(highrate_data[RATE_488HZ], "highrate", RATE_488HZ);
                    frame_488_counter++;
                    //printf("488Hz\n");
                }
            }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
            highrate_superframes.i_out = write_frame;
        }
    }
}

