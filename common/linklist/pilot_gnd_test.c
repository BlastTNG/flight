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
#include "../../mcp/include/pilot.h"

void pilot_recv_and_decompress(void *arg) {
  // initialize UDP connection via bitserver/BITRecver
  struct BITRecver pilotrecver = {0};
  initBITRecver(&pilotrecver, PILOT_ADDR, PILOT_PORT, 10, PILOT_MAX_PACKET_SIZE, PILOT_MAX_PACKET_SIZE);
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0;
  linklist_t * ll = NULL;
  uint32_t blk_size = 0;

  // allocate buffers
  uint8_t * compbuffer = calloc(1, PILOT_MAX_PACKET_SIZE); // compressed buffer
  uint8_t * superframe = allocate_superframe(); // uncompressed superframe
  uint8_t * pilot_channel_data[RATE_END] = {NULL};

  int i = 0, rate = 0;
  for (rate = 0; rate < RATE_END; rate++) {
    pilot_channel_data[rate] = calloc(1, frame_size[rate]);
  }  

  int count = 0;
  while (1) {
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
    blast_info("Received linklist with serial 0x%x (packet %d)\n", serial, count++);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&pilotrecver, compbuffer, PILOT_MAX_PACKET_SIZE, 0);

    if (blk_size < 0) {
      blast_info("Malformed packed received on Pilot\n");
      continue;
    }

    if (!read_allframe(superframe, compbuffer)) { // just a regular frame
      for (i = 0; i < blk_size; i++) {
        if ((i % 16) == 0) printf("\n");
        printf("0x%x ", compbuffer[i]);
      }
      printf("\n");

      // TODO(javier): deal with blk_size < ll->blk_size
      // decompress the linklist
      if (!decompress_linklist(superframe, ll, compbuffer)) continue;
    }
    else {
      printf("\nReceived all frame :)\n");
    }

    // set the superframe ready flag
    ll->data_ready |= SUPERFRAME_READY;

    // extract data from the superframe for each rate
    for (rate = 0; rate < RATE_END; rate++) {
      int next_frame = 0; // the next frame to be extracted from the superframe
      for (i = 0; i < get_spf(rate); i++) {
        next_frame = extract_frame_from_superframe(pilot_channel_data[rate], rate, superframe);
      }
    }
  }
}

int main(int argc, char * argv[]) {
  channels_initialize(channel_list);
  linklist_t * ll_list[2] = {parse_linklist("test.ll"), NULL};
  linklist_generate_lookup(ll_list);  
 

  pthread_t recv_worker;
  pthread_create(&recv_worker, NULL, (void *) &pilot_recv_and_decompress, NULL);
  pthread_join(recv_worker,NULL);

  return 0;
}


