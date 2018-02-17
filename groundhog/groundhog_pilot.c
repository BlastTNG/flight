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

#include "bitserver.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "blast.h"
#include "pilot.h"
#include "groundhog_framing.h"

superframes_list_t superframes;

void pilot_receive(void *arg) {

  struct BITRecver pilotrecver = {0};
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0;
  linklist_t * ll = NULL;
  uint32_t blk_size = 0;

  uint8_t *local_superframe = allocate_superframe();
  uint8_t *compbuffer = calloc(1, PILOT_MAX_SIZE);

  // initialize UDP connection via bitserver/BITRecver
  initBITRecver(&pilotrecver, PILOT_ADDR, PILOT_PORT, 10, PILOT_MAX_SIZE, PILOT_MAX_PACKET_SIZE);
  initialize_circular_superframes(&superframes);

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

    // set the linklist serial
    setBITRecverSerial(&pilotrecver, serial);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&pilotrecver, compbuffer, PILOT_MAX_SIZE, 0);

    if (blk_size < 0) {
      blast_info("Malformed packed received on Pilot\n");
      continue;
    }

    // TODO(javier): deal with blk_size < ll->blk_size
    // decompress the linklist
    if (!read_allframe(local_superframe, compbuffer)) { // just a regular frame
      blast_info("Received linklist with serial 0x%x\n", serial);
      if (!decompress_linklist(local_superframe, ll, compbuffer)) { 
        continue;
      }
    } else {
      blast_info("\nReceived an all frame :)\n");
    }
    push_superframe(local_superframe, &superframes);
  }
}

void pilot_publish(void *arg) {

    static char frame_name[RATE_END][32];
    int frame_offset = 0;
    void *pilot_data[RATE_END] = {0};

    uint16_t    read_frame;
    uint16_t    write_frame;

    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
        pilot_data[rate] = calloc(1, allocated_size);
    }
 
    for (int rate = 0; rate < RATE_END; rate++) {
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/pilot/%s", RATE_LOOKUP_TABLE[rate].text);
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
            // loop below to be replaced by linklist code
            // for (int rate = 0; rate < RATE_END; rate++) {
            //     int freq = groundhog_get_rate(rate);
            //     for (int i = 0; i < freq; i++) {
            //         frame_offset = i*frame_size[rate];
            //         memcpy(pilot_data[rate], superframes.framelist[write_frame]+frame_offset, frame_size[rate]);
            //         framing_publish_200hz(pilot_data[rate], "pilot");
            //     }
            // }
            write_frame = (write_frame + 1) & (NUM_FRAMES-1);
        }
        superframes.i_out = write_frame;
        usleep(10000);
    }
}

