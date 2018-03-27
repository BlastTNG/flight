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

enum HeaderType{NONE, IRID_OMNI, TD_OMNI, IRID_SLOW, PAYLOAD};

struct CSBFHeader {
  uint8_t route;
  uint8_t origin;
  uint8_t zero;
  uint16_t size;

  uint8_t checksum;
  enum HeaderType mode;
  uint8_t sync;

  char namestr[80];
};


enum HeaderType read_csbf_header(struct CSBFHeader * header, uint8_t byte) {
  header->mode = NONE;

  if ((header->sync == 0) && (byte == HIGHRATE_SYNC1)) {
      header->sync++;
  } else if (header->sync == 1) {
      switch (byte) {
          case LOWRATE_COMM1_SYNC2 :
          case LOWRATE_COMM2_SYNC2 :
          case HIGHRATE_TDRSS_SYNC2 :
          case HIGHRATE_IRIDIUM_SYNC2 :
              header->route = byte;
              header->sync++;
              break;
          default :
              header->sync = 0;
      }
  } else if (header->sync == 2) {
      header->origin = byte;
      header->checksum = byte;             

      header->sync++;
  } else if (header->sync == 3) {
      header->zero = byte;
      header->checksum += byte;

      header->sync++; 
  } else if (header->sync == 4) { // size MSB
      header->size = byte << 8;
      header->checksum += byte;

      header->sync++;
  } else if (header->sync == 5) { // size LSB
      header->size += byte;
      header->checksum += byte;       

      if (header->zero) {
          header->mode = PAYLOAD;
      } else if (header->route == HIGHRATE_IRIDIUM_SYNC2) {
          if ((header->origin & 0x03) == 0x01) {
              header->mode = IRID_SLOW;
              sprintf(header->namestr, "Iridium HK");
          } else if ((header->origin & 0x03) == 0x02) {
              header->mode = IRID_OMNI;
              sprintf(header->namestr, "Iridium Omni");
          }
      } else if (header->route == HIGHRATE_TDRSS_SYNC2) {
          sprintf(header->namestr, "TDRSS");
          header->mode = TD_OMNI;
      }
      header->sync = 0;
  }
  return header->mode;
}

// grabs a packet from the gse stripped of it gse header
void read_gse_sync_frame(int fd, uint8_t * buffer, struct CSBFHeader * header) {
  uint8_t byte = 0;
  unsigned int bytes_read = 0;
  memset(header, 0, sizeof(struct CSBFHeader));

  while (true) {
      if (read(fd, &byte, 1) == 1) {
          if (header->mode == NONE) {
              read_csbf_header(header, byte);
          } else if (header->mode != PAYLOAD) { // end of gse/payload header
              if (bytes_read < header->size) { // keep reading to the buffer 
                  buffer[bytes_read++] = byte;
                  header->checksum += byte;
              } else {
                  if (header->checksum == byte) { // done reading 
                      // printf("Received gse packet size %d (0x%x == 0x%x)\n", header->size, byte, header->checksum); 
                      return;
                  } else {
                      blast_info("[%s] Bad checksum 0x%.2x != 0x%.2x", header->namestr, header->checksum, byte);
                  }
                  bytes_read = 0;
                  memset(header, 0, sizeof(struct CSBFHeader));
              }
          } else { // payload packet, so ignore
              bytes_read = 0;
              memset(header, 0, sizeof(struct CSBFHeader));
          }
      } else { // nothing read
          usleep(1000);
      }
  }
}


void highrate_receive(void *arg) {
  // Open serial port
  comms_serial_t *serial = comms_serial_new(NULL);
  comms_serial_connect(serial, HIGHRATE_PORT);
  comms_serial_setspeed(serial, B115200);
  int fd = serial->sock->fd; 

  linklist_t * ll = NULL;

  // packet sizes
  unsigned int payload_packet_size = HIGHRATE_DATA_PACKET_SIZE+CSBF_HEADER_SIZE+1;
  uint16_t datasize = HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE;
  uint32_t buffer_size = ((HIGHRATE_MAX_SIZE-1)/datasize+1)*datasize;

  // buffer allocations
  uint8_t *payload_packet = calloc(1, payload_packet_size);
  uint8_t *csbf_header = payload_packet+0;
  uint8_t *header_buffer = csbf_header+CSBF_HEADER_SIZE;
  uint8_t *data_buffer = header_buffer+PACKET_HEADER_SIZE;
  uint8_t *csbf_checksum = header_buffer+HIGHRATE_DATA_PACKET_SIZE;

  // packetization variables
  uint32_t *serial_number = 0;
  uint16_t *i_pkt, *n_pkt;
  uint32_t *frame_number;
  uint32_t transmit_size;
  uint8_t *compressed_buffer = calloc(1, buffer_size);

  // superframe allocations
  uint8_t *local_superframe = allocate_superframe();
  initialize_circular_superframes(&highrate_superframes);

  struct CSBFHeader gse_packet_header = {0};
  uint8_t * gse_packet = calloc(1, 2048);
  unsigned int gse_read = 0;

  struct CSBFHeader payload_packet_header = {0};
  unsigned int payload_read = 0;
  unsigned int payload_copy = 0;
  uint8_t payload_packet_lock = 0;
  uint16_t payload_size = 0;

  int retval = 0;
  uint32_t recv_size = 0;

  char * source_str = NULL;

  while (true) {
      // printf("-------------START (lock = %d)---------\n", payload_packet_lock);

      // get the sync frame from the gse
      read_gse_sync_frame(fd, gse_packet, &gse_packet_header);
      gse_read = 0;
      source_str = gse_packet_header.namestr;
/* 
      for (int i = 0; i < gse_packet_header.size; i++) {
          if (i%32 == 0) printf("\n%.3d : ", i/32);
          printf("0x%.2x ", gse_packet[i]);
      }
      printf("\n");
*/
      // printf("Got a GSE packet size %d origin 0x%x\n", gse_packet_header.size, gse_packet_header.origin);
 
      while (gse_read < gse_packet_header.size) { // read all the gse data

          if (gse_packet_header.origin & 0xe) { // packet not from the hk stack
              if (payload_packet_lock) { // locked onto payload header     
                  payload_copy = MIN(payload_size-payload_read, gse_packet_header.size-gse_read);
                  memcpy(payload_packet+payload_read, gse_packet+gse_read, payload_copy);

                  gse_read += payload_copy; // update bytes read from gse
                  // printf("Copied %d bytes to location %d. %d bytes left to read\n", payload_copy, payload_read, gse_packet_header.size-gse_read);
                  payload_read += payload_copy; // update bytes read to to payload packet

                  if (payload_read == payload_size) { // read all of the payload packet
                      // printf("Got the entire payload message\n");
                      // reset bookkeeping variables
                      payload_read = 0;
                      payload_packet_lock = 0;
                      memset(&payload_packet_header, 0, sizeof(struct CSBFHeader));

                      // process the packet
                      readHeader(header_buffer, &serial_number, &frame_number, &i_pkt, &n_pkt);

                      // hijack frame number for transmit size
                      transmit_size = *frame_number;

                      if (!(ll = linklist_lookup_by_serial(*serial_number))) {
                          blast_err("Could not find linklist with serial 0x%.4x", *serial_number);
                          continue; 
                      }

                      // blast_info("Transmit size=%d, blk_size=%d, payload_size=%d, datasize=%d, i=%d, n=%d", transmit_size, ll->blk_size, payload_size, datasize, *i_pkt, *n_pkt);

                      retval = depacketizeBuffer(compressed_buffer, &recv_size, 
                                           HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE,
                                           i_pkt, n_pkt, data_buffer);

                      memset(payload_packet, 0, payload_packet_size);

                      // the packet is complete, so decompress
                      if ((retval == 0) && (ll != NULL))
                      {
                          // decompress the linklist
                          if (read_allframe(local_superframe, compressed_buffer)) {
                              blast_info("[%s] Received an allframe :)\n", source_str);
                          } else {
                              if (transmit_size > ll->blk_size) {
                                  blast_err("Transmit size %d larger than assigned linklist size %d", transmit_size, allframe_size);
                                  transmit_size = ll->blk_size;
                              }
                              blast_info("[%s] Received linklist \"%s\"", source_str, ll->name);
                              // blast_info("[%s] Received linklist with serial_number 0x%x\n", source_str, *serial_number);
                              decompress_linklist_by_size(local_superframe, ll, compressed_buffer, transmit_size);
                              push_superframe(local_superframe, &highrate_superframes);
                          }
                          memset(compressed_buffer, 0, buffer_size);
                          recv_size = 0;
                      }
                  }
              } else if (!payload_packet_lock) { // search for payload header
                  read_csbf_header(&payload_packet_header, gse_packet[gse_read++]);
                  if (payload_packet_header.mode == PAYLOAD) {
                      payload_size = payload_packet_header.size+CSBF_HEADER_SIZE+1; // payload includes the header and checksum
                      payload_packet_lock = 1;
                      payload_read = CSBF_HEADER_SIZE;
                      // printf("Found a payload header @ %d size %d bytes\n", gse_read, payload_size);
                  }
              }    
     
          } else { // housekeeping packet
              // TODO(javier): deal with housekeeping packets
              blast_info("[%s] Received packet from HK stack size=%d\n", source_str, gse_packet_header.size);
              gse_read += gse_packet_header.size;
          }
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

