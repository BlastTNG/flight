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

uint8_t * superframe = NULL;

void pilot_recv_and_decompress(void *arg) {
  // initialize UDP connection via bitserver/BITRecver
  struct BITRecver pilotrecver = {0};
  initBITRecver(&pilotrecver, PILOT_ADDR, PILOT_PORT, 10, PILOT_MAX_PACKET_SIZE, PILOT_MAX_PACKET_SIZE);
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0;
  linklist_t * ll = NULL;
  uint32_t blk_size = 0;

  uint8_t * compbuffer = calloc(1, PILOT_MAX_PACKET_SIZE);
  int i = 0;

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
    blast_info("Received linklist with serial 0x%x (packet %d)\n", serial, i++);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&pilotrecver, compbuffer, PILOT_MAX_PACKET_SIZE, 0);

    if (blk_size < 0) {
      blast_info("Malformed packed received on Pilot\n");
      continue;
    }

    // TODO(javier): deal with blk_size < ll->blk_size
    // decompress the linklist
    if (!decompress_linklist(NULL, ll, compbuffer)) continue;

    // set the superframe ready flag
    ll->data_ready |= SUPERFRAME_READY;
  }
}

int main(int argc, char * argv[]) {
  channels_initialize(channel_list);
  linklist_t * ll_list[2] = {parse_linklist("test.ll"), NULL};
  linklist_generate_lookup(ll_list);  
 
  superframe = allocate_superframe();
  assign_superframe_to_linklist(ll_list[0], superframe);

  pthread_t recv_worker;
  pthread_create(&recv_worker, NULL, (void *) &pilot_recv_and_decompress, NULL);
  pthread_join(recv_worker,NULL);

  return 0;
}


