/* mcp: the BLAST flight control program
 *
 * This software is copyright (C) 2018 Penn University
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>

#include "linklist.h"
#include "linklist_compress.h"
#include "blast.h"
#include "mcp.h"
#include "command_struct.h"
#include "FIFO.h"
#include "bitserver.h"
#include "mputs.h"
#include "comms_serial.h"
#include "highrate.h"

struct Fifo highrate_fifo = {0};

void highrate_compress_and_send(void *arg) {

  linklist_t * ll = NULL, * ll_old = NULL;
  linklist_t ** ll_array = arg;

  unsigned int fifosize = MAX(HIGHRATE_MAX_SIZE, allframe_size);

  comms_serial_t * serial = comms_serial_new(NULL);

  uint8_t * csbf_header = calloc(1, CSBF_HEADER_SIZE);
  uint8_t csbf_checksum = 0;
  uint8_t * header_buffer = calloc(1, PACKET_HEADER_SIZE);
  uint16_t datasize = HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE;
  unsigned int buffer_size =  ((fifosize-1)/datasize+1)*datasize;
  uint8_t * compressed_buffer = calloc(1, buffer_size);
  int allframe_count = 0;
  uint32_t bandwidth = 0, transmit_size = 0;
  int i;
  int get_serial_fd = 1;

  // packetization variables
  uint16_t i_pkt = 0;
  uint16_t n_pkt = 0;

  nameThread("Highrate");

  while (true) {
    while (get_serial_fd) {
      if ((comms_serial_connect(serial, HIGHRATE_PORT) == NETSOCK_OK) &&
           comms_serial_setspeed(serial, B115200)) {
        break;
      }
      sleep(5);
    }
    get_serial_fd = 0;

    // get the current pointer to the pilot linklist
    ll = ll_array[HIGHRATE_TELEMETRY_INDEX];
		if (ll != ll_old) {
				if (ll) blast_info("Highrate linklist set to \"%s\"", ll->name);
				else blast_info("Highrate linklist set to NULL");
		}
		ll_old = ll;

    // get the current bandwidth
    bandwidth = CommandData.highrate_bw;

    if (!fifoIsEmpty(&highrate_fifo) && ll) { // data is ready to be sent
      // send allframe if necessary
      if (!allframe_count) {
      //  write_allframe(compressed_buffer, getFifoRead(&highrate_fifo));
      //  sendToBITSender(&pilotsender, compressed_buffer, allframe_size, 0);
      }

      // compress the linklist
      int retval = compress_linklist(compressed_buffer, ll, getFifoRead(&highrate_fifo));
      decrementFifo(&highrate_fifo);

      if (!retval) continue;

      // compute the transmite size based on bandwidth
      transmit_size = MIN(ll->blk_size, bandwidth); // frames are 1 Hz, so bandwidth == size

      // set initialization for packetization
      uint8_t * chunk = NULL;
      uint32_t chunksize = datasize;
      i_pkt = 0;
      n_pkt = 1;

      // write the CSBF header
      csbf_header[0] = HIGHRATE_SYNC1;
      csbf_header[1] = (CommandData.highrate_through_tdrss) ? HIGHRATE_TDRSS_SYNC2 : HIGHRATE_IRIDIUM_SYNC2;
      csbf_header[2] = HIGHRATE_ORIGIN_COMM1; // TODO(javier): check if this needs to be commanded 
      csbf_header[3] = 0x00; // zero
      csbf_header[4] = ((datasize+PACKET_HEADER_SIZE) >> 8) & 0xff; // msb of size
      csbf_header[5] = (datasize+PACKET_HEADER_SIZE) & 0xff;  // lsb of size

      while ((i_pkt < n_pkt) && (chunk = packetizeBuffer(compressed_buffer, transmit_size,
                                    &chunksize, &i_pkt, &n_pkt))) {

        // have packet header serials match the linklist serials
        writeHeader(header_buffer, *(uint32_t *) ll->serial, transmit_size, i_pkt, n_pkt);

        // compute checksum
        csbf_checksum = 0;
        for (i = 2; i < CSBF_HEADER_SIZE; i++) csbf_checksum += csbf_header[i];
        for (i = 0; i < PACKET_HEADER_SIZE; i++) csbf_checksum += header_buffer[i];
        for (i = 0; i < datasize; i++) csbf_checksum += chunk[i];

        // send the headers
        if (write(serial->sock->fd, csbf_header, CSBF_HEADER_SIZE) < 0) { // send csbf header 
          get_serial_fd = 1;
          break;
        }
        if (write(serial->sock->fd, header_buffer, PACKET_HEADER_SIZE) < 0) { // send our header
          get_serial_fd = 1;
          break;
        }

        // send the data to the ground station via ttyHighRate
        if (write(serial->sock->fd, chunk, datasize) < 0) {
          get_serial_fd = 1;
          break;
        }

        // send the checksum as the last byte
        if (write(serial->sock->fd, &csbf_checksum, 1) < 0) {
          get_serial_fd = 1;
          break;
        }

        i_pkt++;
        usleep(1000);
      }

      memset(compressed_buffer, 0, buffer_size);
      allframe_count = (allframe_count + 1) % HIGHRATE_ALLFRAME_PERIOD;
    } else {
      usleep(100000); // zzz...
    }
  }
}
