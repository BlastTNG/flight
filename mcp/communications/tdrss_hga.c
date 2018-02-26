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
// #include "command_struct.h"
#include "FIFO.h"
#include "bitserver.h"
#include "mputs.h"
#include "comms_serial.h"
#include "tdrss_hga.h"

struct Fifo tdrss_hga_fifo = {0};

void tdrss_hga_compress_and_send(void *arg) {

  linklist_t * ll = NULL;
  linklist_t ** ll_array = arg;

  unsigned int fifosize = MAX(TDRSS_HGA_MAX_SIZE, allframe_size);

  comms_serial_t * serial = comms_serial_new(NULL);
  comms_serial_connect(serial, TDRSS_HGA_PORT);
  comms_serial_setspeed(serial, B115200);

  uint8_t * header_buffer = calloc(1, PACKET_HEADER_SIZE);
  uint8_t * compressed_buffer = calloc(1, fifosize);
  int allframe_count = 0;

  nameThread("TDRSS_HGA");

  while (true) {

    // get the current pointer to the pilot linklist
    ll = ll_array[2];

    if (!fifoIsEmpty(&tdrss_hga_fifo) && ll) { // data is ready to be sent
      // send allframe if necessary
      if (!allframe_count) {
      //  write_allframe(compressed_buffer, getFifoRead(&tdrss_hga_fifo));
      //  sendToBITSender(&pilotsender, compressed_buffer, allframe_size, 0);
      }

      // compress the linklist
      int retval = compress_linklist(compressed_buffer, ll, getFifoRead(&tdrss_hga_fifo));
      decrementFifo(&tdrss_hga_fifo);

      if (!retval) continue;

      // have packet header serials match the linklist serials
      writeHeader(header_buffer, *(uint32_t *) ll->serial, 0, 0, 1);
      // comms_serial_write(serial, header_buffer, PACKET_HEADER_SIZE);
      write(serial->sock->fd, header_buffer, PACKET_HEADER_SIZE);

      // TODO(javier): make send size commandable (e.g. MIN(ll->blk_size, cmd_tdrss_hga_bw))
      // send the data to the ground station via ttyHighRate
      // comms_serial_write(serial, compressed_buffer, ll->blk_size);
      write(serial->sock->fd, compressed_buffer, ll->blk_size);

      memset(compressed_buffer, 0, TDRSS_HGA_MAX_SIZE);
      allframe_count = (allframe_count + 1) % TDRSS_HGA_ALLFRAME_PERIOD;
    } else {
      usleep(100000); // zzz...
    }
  }
}
