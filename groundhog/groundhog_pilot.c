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

#include "linklist.h" // This gives access to channel_list and frame_size
#include "linklist_compress.h"
#include "linklist_writer.h"

#include "bitserver.h"
#include "blast.h"
#include "blast_time.h"
#include "pilot.h"
#include "groundhog.h"

void udp_receive(void *arg) {

  struct UDPSetup * udpsetup = (struct UDPSetup *) arg;

  int id = udpsetup->downlink_index;

  struct BITRecver udprecver = {0};
  uint8_t * recvbuffer = NULL;
  uint32_t serial = 0, prev_serial = 0;
  linklist_t * ll = NULL;
  uint32_t blk_size = 0;
  uint32_t transmit_size = 0;

  uint8_t *local_superframe = calloc(1, superframe->size);
  uint8_t *local_allframe = calloc(1, superframe->allframe_size);

  // open a file to save all the raw linklist data
  linklist_rawfile_t * ll_rawfile = NULL;

  uint8_t * compbuffer = calloc(1, udpsetup->maxsize);
  uint8_t * dummy_buffer = calloc(1, superframe->size);

  // initialize UDP connection via bitserver/BITRecver
  initBITRecver(&udprecver, udpsetup->addr, udpsetup->port, 10, udpsetup->maxsize, udpsetup->packetsize);

  int bad_serial_count = 0;
  int good_serial_count = 0;

  while (true) {
    do {
      // get the linklist serial for the data received
      recvbuffer = getBITRecverAddr(&udprecver, &blk_size);
      serial = *(uint32_t *) recvbuffer;
      if (!(ll = linklist_lookup_by_serial(serial))) {
        removeBITRecverAddr(&udprecver);
        if (verbose) blast_info("[%s] Receiving bad serial packets (0x%x)", udpsetup->name, serial);
        bad_serial_count++;
      } else {
        bad_serial_count = 0;
        break;
      }
    } while (true);

    // set the linklist serial
    setBITRecverSerial(&udprecver, serial);

    // receive the data from payload via bitserver
    blk_size = recvFromBITRecver(&udprecver, compbuffer, PILOT_MAX_SIZE, 0);

    // hijacking frame number for transmit size
    transmit_size = udprecver.frame_num; 
    // printf("Transmit size = %d, blk_size = %d\n", transmit_size, blk_size);

    if (verbose) blast_info("[%s] Received linklist \"%s\"", udpsetup->name, ll->name);
    // blast_info("[%s] Received linklist \"%s\"", udpsetup->name, ll->name);
    // blast_info("[Pilot] Received linklist with serial 0x%x\n", serial);

    // this is a file that has been downlinked, so unpack and extract to disk
    if (!strcmp(ll->name, FILE_LINKLIST)) {
        unsigned int bytes_unpacked = 0;
        while ((bytes_unpacked+ll->blk_size) <= transmit_size) {
            decompress_linklist(dummy_buffer, ll, compbuffer+bytes_unpacked);
            bytes_unpacked += ll->blk_size;
            usleep(1000);
        }

    } else { // write the linklist data to disk
        // decompress the linklist
        if (read_allframe(local_superframe, superframe, compbuffer)) { // just a regular frame
            if (verbose) blast_info("[%s] Received an allframe :)\n", udpsetup->name);
            memcpy(local_allframe, compbuffer, superframe->allframe_size);
        } else {
            if (serial != prev_serial) {
                ll_rawfile = groundhog_open_new_rawfile(ll_rawfile, ll, udpsetup->name);
            }
            prev_serial = serial;

            if (blk_size < 0) {
                blast_info("Malformed packed received on Pilot\n");
                continue;
            } else if (blk_size != transmit_size) {
                blast_info("Packet size mismatch blk_size=%d, transmit_size=%d", blk_size, transmit_size);
            }

            // write the linklist data to disk
            if (ll_rawfile) {
                memcpy(compbuffer+ll->blk_size, local_allframe, superframe->allframe_size);
                write_linklist_rawfile(ll_rawfile, compbuffer);
                flush_linklist_rawfile(ll_rawfile);
            }
        }
    }
 
  }
}
