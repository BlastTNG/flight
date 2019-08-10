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
#include <ctype.h>
#include <sys/time.h>

#include "groundhog.h"

extern struct TlmReport ll_report[MAX_NUM_LINKLIST_FILES+1];
struct LinklistState ll_state[MAX_NUM_LINKLIST_FILES+1] = {{0}};

void udp_receive(void *arg) {

  struct UDPSetup * udpsetup = (struct UDPSetup *) arg;

  struct BITRecver udprecver = {0};
  uint8_t * recvbuffer = NULL;
  uint16_t serial = 0, prev_serial = 0;
  linklist_t * ll = NULL;
  int32_t blk_size = 0;
  uint32_t recv_size = 0;
  uint64_t transmit_size = 0;
  int64_t framenum = 0;
  uint64_t recv_framenum = 0;

  uint8_t *local_allframe = calloc(1, superframe->allframe_size);

  // raw linklist data and fileblocks
  struct LinklistState * state = NULL;
  struct TlmReport * report = NULL;
  uint8_t * compbuffer = calloc(1, udpsetup->maxsize);

  // initialize UDP connection via bitserver/BITRecver
  initBITRecver(&udprecver, udpsetup->addr, udpsetup->port, 10, udpsetup->maxsize, udpsetup->packetsize);

  int bad_serial_count = 0;

  while (true) {
    do {
      // get the linklist serial for the data received
      recvbuffer = getBITRecverAddr(&udprecver, &recv_size);
      serial = *(uint16_t *) recvbuffer;
      if (!(ll = linklist_lookup_by_serial(serial))) {
        removeBITRecverAddr(&udprecver);
        if (verbose) groundhog_info("[%s] Receiving bad serial packets (0x%x)\n", udpsetup->name, serial);
        bad_serial_count++;
      } else {
        bad_serial_count = 0;
        break;
      }
    } while (true);

    // set the linklist serial
    setBITRecverSerial(&udprecver, serial);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&udprecver, compbuffer, udpsetup->maxsize, 0);
    if (blk_size < 0) {
			groundhog_info("Malformed packet received on Pilot\n");
			continue;
    }

    // process the auxiliary data into transmit size and frame number
    get_aux_packet_data(udprecver.frame_num, &transmit_size, &recv_framenum); 

    // get the linklist state struct 
    int flags = 0;
		int i;
    if (serial != prev_serial) {
			for (i=0; i<MAX_NUM_LINKLIST_FILES; i++) {
				if (!ll_state[i].serial || (serial == ll_state[i].serial)) break;
			}
			ll_state[i].serial = serial;
			state = &ll_state[i];
    }

    // if this serial has not be recv'd yet, a new state will be allocated
    if (!state->ll_rawfile) {
      // set the flags to open a new rawfile
      flags |= GROUNDHOG_OPEN_NEW_RAWFILE;

      // reuse the rawfile if it had been accessed before
      if (ll->internal_id) flags |= GROUNDHOG_REUSE_VALID_RAWFILE;
      ll->internal_id = 42;

			// build the symlink name based on linklist name
			for (i = 0; i < strlen(ll->name); i++) {
				if (ll->name[i] == '.') break;
				state->symname[i] = toupper(ll->name[i]);
			}
			state->symname[i] = '\0'; // terminate
    }

    unsigned int bytes_unpacked = 0;
    while (bytes_unpacked < transmit_size) {
      // Received <= the data expected for the linklist, so received a single linklist packet or a
      // bandwidth-limited linklist.
      unsigned int comp_size = MIN(ll->blk_size, transmit_size-bytes_unpacked);
      framenum = groundhog_process_and_write(ll, comp_size, compbuffer+bytes_unpacked, 
                                             local_allframe, state->symname, udpsetup->name, 
                                             &state->ll_rawfile, flags); 
      // only open a new rawfile once
      flags &= ~GROUNDHOG_OPEN_NEW_RAWFILE;

      // check to see how many bytes were unpacked
      // (negative framenum indicates allframe
      bytes_unpacked += (framenum > 0) ? ll->blk_size : ll->superframe->allframe_size;
    }

    // get the telemetry report
    if (serial != prev_serial) {
			for (i=0; i<MAX_NUM_LINKLIST_FILES; i++) {
				if (!ll_report[i].ll || (serial == *(uint16_t *) ll_report[i].ll->serial)) break;
			}
      report = &ll_report[i];
		  report->ll = ll;
      report->type = 0; // pilot report type
    }

    // fill out the telemetry report
    report->framenum = abs(framenum); 
    report->allframe = (framenum < 0);

    prev_serial = serial;
    memset(compbuffer, 0, udpsetup->maxsize);
 
  }
}
