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

#include "groundhog_funcs.h"
#include "bitserver.h"

struct TlmReport pilot_report = {0};

void udp_receive(void *arg) {

  struct UDPSetup * udpsetup = (struct UDPSetup *) arg;

  struct BITRecver udprecver = {0};
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0, prev_serial = 0;
  linklist_t * ll = NULL;
  int32_t blk_size = 0;
  uint32_t recv_size = 0;
  uint32_t transmit_size = 0;
  int64_t framenum = 0;
  int af = 0;

  uint8_t *local_allframe = calloc(1, superframe->allframe_size);

  // open a file to save all the raw linklist data
  linklist_rawfile_t * ll_rawfile = NULL;

  uint8_t * compbuffer = calloc(1, udpsetup->maxsize);

  // initialize UDP connection via bitserver/BITRecver
  initBITRecver(&udprecver, udpsetup->addr, udpsetup->port, 10, udpsetup->maxsize, udpsetup->packetsize);

  int bad_serial_count = 0;

  while (true) {
    do {
      // get the linklist serial for the data received
      recvbuffer = getBITRecverAddr(&udprecver, &recv_size);
      serial = *(uint32_t *) recvbuffer;
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
        groundhog_info("Malformed packed received on Pilot\n");
        continue;
    }

    // hijacking frame number for transmit size
    transmit_size = udprecver.frame_num; 

    if (groundhog_check_for_fileblocks(ll, FILE_LINKLIST)) {
        // unpack and extract to disk
        framenum = groundhog_unpack_fileblocks(ll, transmit_size, compbuffer, NULL,
                                               NULL, NULL, NULL, GROUNDHOG_EXTRACT_TO_DISK);
    } else { // write the linklist data to disk
        // set flags for data extraction
        unsigned int flags = 0;
        if (serial != prev_serial) flags |= GROUNDHOG_OPEN_NEW_RAWFILE;
        prev_serial = serial;

        // process the linklist and write the data to disk
        framenum = groundhog_process_and_write(ll, transmit_size, compbuffer, local_allframe,
                                               udpsetup->name, udpsetup->name, &ll_rawfile, flags);
    }

    // fill out the telemetry report
    pilot_report.ll = ll;
    pilot_report.framenum = abs(framenum); 
    pilot_report.allframe = af; 

    memset(compbuffer, 0, udpsetup->maxsize);
 
  }
}
