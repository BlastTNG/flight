#include <math.h>
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
#include "pilot.h"
#include "groundhog.h"
#include "groundhog_framing.h"

superframes_list_t pilot_superframes;

void pilot_receive(void *arg) {

  struct UDPSetup * udpsetup = (struct UDPSetup *) arg;

  struct BITRecver pilotrecver = {0};
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0, prev_serial = 0;
  linklist_t * ll = NULL;
  uint32_t blk_size = 0;
  uint32_t transmit_size = 0;

  // open a file to save all the raw linklist data
  FILE * rawsave = NULL;

  uint8_t *local_superframe = allocate_superframe();
  uint8_t *compbuffer = calloc(1, PILOT_MAX_SIZE);

  // initialize UDP connection via bitserver/BITRecver
  initBITRecver(&pilotrecver, udpsetup->addr, udpsetup->port, 10, udpsetup->maxsize, udpsetup->packetsize);
  initialize_circular_superframes(&pilot_superframes);

  while (true) {
    do {
      // get the linklist serial for the data received
      recvbuffer = getBITRecverAddr(&pilotrecver, &blk_size);
      serial = *(uint32_t *) recvbuffer;
      if (!(ll = linklist_lookup_by_serial(serial))) {
        removeBITRecverAddr(&pilotrecver);
      } else {
        break;
      }
    } while (true);

#if 0
    if (serial != prev_serial) {
      char fname[80] = {0}, basename[80] = {0};

      if (rawsave) {
        fclose(rawsave);
        rawsave = NULL;
      }
      sprintf(basename, "%s/pilot/%s_%s", FILE_SAVE_DIR, datestring, ll->name);

      // open a new file to write raw linklist data to
      sprintf(fname, "%s.gnddata", basename);
      blast_info("Opening file \"%s\"", fname);
      rawsave = fpreopenb(fname);
      if (!rawsave) blast_err("Failed to open file");
      
      // copy linklist file to the directory
      blast_info("Generating linklist format file to \"%s\"", basename);
      linklist_to_file(ll, basename);
    }
#endif

    prev_serial = serial;

    // set the linklist serial
    setBITRecverSerial(&pilotrecver, serial);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&pilotrecver, compbuffer, PILOT_MAX_SIZE, 0);

    // hijacking frame number for transmit size
    transmit_size = pilotrecver.frame_num; 

    // printf("Transmit size = %d, blk_size = %d\n", transmit_size, blk_size);
 
    if (blk_size < 0) {
      blast_info("Malformed packed received on Pilot\n");
      continue;
    } else if (blk_size != transmit_size) {
      blast_info("Packet size mismatch blk_size=%d, transmit_size=%d", blk_size, transmit_size);
    }

    // TODO(javier): deal with blk_size < ll->blk_size
    // decompress the linklist
    if (read_allframe(local_superframe, compbuffer)) { // just a regular frame
      blast_info("[%s] Received an allframe :)\n", udpsetup->name);
    } else {
      blast_info("[%s] Received linklist \"%s\"", udpsetup->name, ll->name);
      // blast_info("[Pilot] Received linklist with serial 0x%x\n", serial);

      // write the linklist data to disk
      if (rawsave) {
        fwrite(compbuffer, 1, ll->blk_size, rawsave);
        fflush(rawsave); 
      }

      // decompress
      decompress_linklist_by_size(local_superframe, ll, compbuffer, transmit_size); 
      /*
      printf("Start\n");
      for (int i = 0; i < 5; i++) {
        printf("%d\n", (int32_t) be32toh(*((int32_t *) (compbuffer+119+i*4))));
      }
      */
      push_superframe(local_superframe, &pilot_superframes);
    }
  }
}

#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))
void pilot_publish(void *arg) {

    static char frame_name[RATE_END][32];
    void *pilot_data[RATE_END] = {0};

    uint16_t    read_frame;
    uint16_t    write_frame;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
        pilot_data[rate] = calloc(1, allocated_size);
    }
 
    for (int rate = 0; rate < RATE_END; rate++) {
        char rate_name[16];
        strcpy(rate_name, RATE_LOOKUP_TABLE[rate].text);
        rate_name[strlen(rate_name)-1] = 'z';
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/pilot/%s", rate_name);
        blast_info("there will be a topic with name %s", frame_name[rate]);
    }

    while (true) {
        write_frame = pilot_superframes.i_out;
        read_frame = pilot_superframes.i_in;

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
                    extract_frame_from_superframe(pilot_data[RATE_1HZ], RATE_1HZ, pilot_superframes.framelist[write_frame]);
                    framing_publish(pilot_data[RATE_1HZ], "pilot", RATE_1HZ);
                    //printf("1Hz\n");
                }
                  if (!--counter_5hz) {
                    counter_5hz = HZ_COUNTER(5);
                    extract_frame_from_superframe(pilot_data[RATE_5HZ], RATE_5HZ, pilot_superframes.framelist[write_frame]);
                    framing_publish(pilot_data[RATE_5HZ], "pilot", RATE_5HZ);
                    //printf("5Hz\n");
                }
                if (!--counter_100hz) {
                    counter_100hz = HZ_COUNTER(100);
                    extract_frame_from_superframe(pilot_data[RATE_100HZ], RATE_100HZ, pilot_superframes.framelist[write_frame]);
                    framing_publish(pilot_data[RATE_100HZ], "pilot", RATE_100HZ);
                    //printf("100Hz\n");
                }
                if (!--counter_200hz) {
                    counter_200hz = HZ_COUNTER(200);
                    extract_frame_from_superframe(pilot_data[RATE_200HZ], RATE_200HZ, pilot_superframes.framelist[write_frame]);
                    framing_publish(pilot_data[RATE_200HZ], "pilot", RATE_200HZ);
                    //printf("200Hz\n");
                }
                if (!--counter_244hz) {
                    counter_244hz = HZ_COUNTER(244);
                    extract_frame_from_superframe(pilot_data[RATE_244HZ], RATE_244HZ, pilot_superframes.framelist[write_frame]);
                    framing_publish(pilot_data[RATE_244HZ], "pilot", RATE_244HZ);
                    //printf("244Hz\n");
                }
                if (!--counter_488hz) {
                    counter_488hz = HZ_COUNTER(488);
                    extract_frame_from_superframe(pilot_data[RATE_488HZ], RATE_488HZ, pilot_superframes.framelist[write_frame]);
                    framing_publish(pilot_data[RATE_488HZ], "pilot", RATE_488HZ);
                    frame_488_counter++;
                    //printf("488Hz\n");
                }
            }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
            pilot_superframes.i_out = write_frame;
        }
    }
}

