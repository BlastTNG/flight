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


superframes_list_t tdrss_superframes;

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

void tdrss_receive(void *arg) {

  linklist_t * ll = NULL;

  uint8_t *local_superframe = allocate_superframe();
  uint16_t datasize = HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE;
  uint32_t buffer_size = ((HIGHRATE_MAX_SIZE-1)/datasize+1)*datasize;
  uint8_t *compressed_buffer = calloc(1, buffer_size);
  uint8_t *data_buffer = calloc(1, datasize+1);

  uint8_t *header_buffer = calloc(1, PACKET_HEADER_SIZE);
  uint8_t *csbf_header = calloc(1, CSBF_HEADER_SIZE);

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
  uint8_t csbf_origin = 0;
  uint8_t csbf_sync2 = 0;
  uint16_t csbf_size = 0;
  uint8_t csbf_checksum = 0;
  uint8_t checksum = 0;
  int byte_pos = 0;
  char source_str[32] = {0};
  int retval = 0;
  uint32_t recv_size = 0;

  while (true) {
      // perform a search for a valid serial
      byte_pos = 0;
      while (1) {
          if (read(fd, &byte, 1)) {
              // handle sync
              if (byte_pos == 0) { // check first sync byte
                  if (byte == HIGHRATE_SYNC1) { // check first sync byte
                      csbf_header[byte_pos++] = byte;
                      // printf("Tentative 1\n");
                  }
              } else if (byte_pos == 1) { // check 2nd sync byte
                  if ((byte == HIGHRATE_TDRSS_SYNC2) || (byte == HIGHRATE_IRIDIUM_SYNC2)) { 
                  csbf_header[byte_pos++] = byte;
                  // printf("Definite 0\n");
                  } else { // false alarm, lost sync
                      byte_pos = 0;
                  }
              } else if (byte_pos < CSBF_HEADER_SIZE) { // fill the CSBF header
                  //printf("Filling csbf header %d 0x%x\n", byte_pos, byte);
                  csbf_header[byte_pos++] = byte;
              } else if (byte_pos < (CSBF_HEADER_SIZE+4)) { // fill our header
                  //printf("Filling our header %d 0x%x\n", byte_pos, byte);
                  header_buffer[(byte_pos++)-CSBF_HEADER_SIZE] = byte;
              } else {
                  byte_pos = 0;
              }
               
              // handle potentially valid header data
              if (byte_pos == (CSBF_HEADER_SIZE+4)) { // check the serial to see if there's a match
                 //printf("Checking serial 0x%x\n", *(uint32_t *) header_buffer);
                 if ((ll = linklist_lookup_by_serial(*(uint32_t *) header_buffer))) {
                     //printf("Found serial\n");
                     break; // the only way out is with a valid serial
                 } else {
                     byte_pos = 0; // not a valid serial, so lose sync
                 }
              }
          } else {
              usleep(1000);
          }
      }
 
      // get the CSBF header info
      csbf_sync2 = csbf_header[1];
      csbf_origin = csbf_header[2];
      csbf_size = (csbf_header[4] << 8) + (csbf_header[5]);
      if (csbf_sync2 == HIGHRATE_TDRSS_SYNC2) {
          sprintf(source_str, "[TDRSS]");
      } else if (csbf_sync2 == HIGHRATE_IRIDIUM_SYNC2) {
          sprintf(source_str, "[Iridium]");
      } else {
          sprintf(source_str, "[Unknown]");
      }

      // read the rest of the header
      blocking_read(fd, header_buffer+4, PACKET_HEADER_SIZE-4);
      blocking_read(fd, data_buffer, datasize+1); // +1 for the crc footer
      csbf_checksum = data_buffer[datasize]; // grab the received checksum

      // compute checksum
      checksum = 0;
      for (int i = 2; i < CSBF_HEADER_SIZE; i++) checksum += csbf_header[i];
      for (int i = 0; i < PACKET_HEADER_SIZE; i++) checksum += header_buffer[i];
      for (int i = 0; i < datasize; i++) checksum += data_buffer[i];

      if (checksum != csbf_checksum) {
        blast_info("Checksum error 0x%.2x != 0x%.2x", checksum, csbf_checksum);
      }
      if (csbf_size != (datasize+PACKET_HEADER_SIZE)) {
        blast_info("Data size error %d != %d", datasize, csbf_size);
      }
      readHeader(header_buffer, &serial_number, &frame_number, &i_pkt, &n_pkt);
      transmit_size = *frame_number;

      // blast_info("Transmit size=%d, blk_size=%d, csbf_size=%d, datasize=%d, i=%d, n=%d", transmit_size, ll->blk_size, csbf_size, datasize, *i_pkt, *n_pkt);

      retval = depacketizeBuffer(compressed_buffer, &recv_size, 
                               HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE,
                               i_pkt, n_pkt, data_buffer);

      memset(data_buffer, 0, datasize+1);

      if ((retval == 0) && (ll != NULL))
      {
 
        // hijack frame number for transmit size
        if (transmit_size > ll->blk_size) {
            blast_err("Transmit size larger than assigned linklist");
            transmit_size = ll->blk_size;
        }


        // decompress the linklist
        if (!read_allframe(local_superframe, compressed_buffer)) {
            blast_info("%s Received linklist \"%s\"", source_str, ll->name);
            // blast_info("%s Received linklist with serial_number 0x%x\n", source_str, *serial_number);
            if (!decompress_linklist_by_size(local_superframe, ll, compressed_buffer, transmit_size)) { 
                continue;
            }
            push_superframe(local_superframe, &tdrss_superframes);
        } else {
            blast_info("[TDRSS HGA] Received an allframe :)\n");
        }
        memset(compressed_buffer, 0, buffer_size);
        recv_size = 0;
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

