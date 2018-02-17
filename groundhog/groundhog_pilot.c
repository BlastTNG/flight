#include <math.h>
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

  // initialize UDP connection via bitserver/BITRecver
  struct BITRecver pilotrecver = {0};
  initBITRecver(&pilotrecver, PILOT_ADDR, PILOT_PORT, 10, PILOT_MAX_PACKET_SIZE, PILOT_MAX_PACKET_SIZE);
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0;
  linklist_t * ll = NULL;
  uint32_t blk_size = 0;

  initialize_circular_superframes(&superframes);
  uint8_t *local_superframe = allocate_superframe(); // Joy: do we still need this? Seems like the circular buffer is taking care of it

  while (1) {
    blast_info("Waiting for data..\n");
    do {
      // get the linklist serial for the data received
      recvbuffer = getBITRecverAddr(&pilotrecver, &blk_size);
      serial = *(uint32_t *) recvbuffer;
      if (!(ll = linklist_lookup_by_serial(serial))) {
        removeBITRecverAddr(&pilotrecver);
      } else {
        break;
      }
    } while (1);

    // set the linklist serial
    setBITRecverSerial(&pilotrecver, serial);
    blast_info("Received linklist with serial 0x%x\n", serial);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&pilotrecver, ll->compframe, PILOT_MAX_PACKET_SIZE, 0);

    if (blk_size < 0) {
      blast_info("Malformed packed received on Pilot\n");
      continue;
    }

    // TODO(javier): deal with blk_size < ll->blk_size
    // decompress the linklist
    // TODO(joy): line below needs to fill local_superframe
    if (!decompress_linklist(NULL, ll, NULL)) continue;
    push_superframe(local_superframe, &superframes);

    // set the superframe ready flag
    ll->data_ready |= SUPERFRAME_READY;
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

