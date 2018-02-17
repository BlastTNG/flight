/* 
 * linklist.c: 
 *
 * This software is copyright 
 *  (C) 2015-2018 University of Toronto, Toronto, ON
 *
 * This file is part of the SuperBIT project, modified and adapted for BLAST-TNG.
 *
 * linklist is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * linklist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Feb 15, 2018 by Javier Romualdez
 */
/**
 * Description:
 *
 * This file contains functions for compressing, sending, receiving, and
 * decompressing pilot data using linklists. 
 *
*/
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

#include "FIFO.h"
#include "bitserver.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "pilot.h"
#include "blast.h"

uint8_t pilot_idle = 0;

void pilot_compress_and_send(void *arg) {
  // initialize UDP connection using bitserver/BITSender
  struct BITSender pilotsender = {0};
  initBITSender(&pilotsender, PILOT_ADDR, PILOT_PORT, 10, PILOT_MAX_PACKET_SIZE, PILOT_MAX_PACKET_SIZE);
  linklist_t * ll = NULL;

  uint8_t * compbuffer = calloc(1, PILOT_MAX_PACKET_SIZE);

  while (1) {
    // get the current pointer to the pilot linklist
    ll = *(linklist_t **) arg;

    if (ll->data_ready & SUPERFRAME_READY) { // data is ready to be sent
      // unset the data ready bit
      ll->data_ready &= ~SUPERFRAME_READY;

      // compress the linklist
      int retval = compress_linklist(compbuffer, ll, NULL);
      pilot_idle = 1; // set the FIFO flag in mcp
      if (!retval) continue;

      // have packet header serials match the linklist serials
      setBITSenderSerial(&pilotsender, *(uint32_t *) ll->serial);

      // TODO(javier): make send size commandable (e.g. MIN(ll->blk_size, cmd_pilot_bw))
      // send the data to the ground station via bitsender
      sendToBITSender(&pilotsender, compbuffer, ll->blk_size, 0);

      memset(compbuffer, 0, PILOT_MAX_PACKET_SIZE);
    } else {
      usleep(100000); // zzz...
    }
  }
}
