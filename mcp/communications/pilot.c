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

#include "bitserver.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "pilot.h"

void pilot_compress_and_send(void *arg) {
  linklist_t * ll = (linklist_t *) arg;

  // initialize UDP connection using bitserver
  struct BITSender pilotsender = {0};
  initBITSender(&pilotsender, PILOT_ADDR, PILOT_PORT, 10, 200000, 200000);
  uint32_t blk_size = 0;

  while (1) {
    if (ll->data_ready & SUPERFRAME_READY) { // data is ready to be sent
      // unset the data ready bit
      ll->data_ready &= ~SUPERFRAME_READY;

      // compress the linklist
      blk_size = compress_linklist(NULL, ll, NULL);

      // send the data to the ground station via bitsender
      sendToBITSender(&pilotsender, ll->compframe, blk_size, 0);

    } else {
      usleep(100000); // zzz...
    }
  }
}

void pilot_recv_and_decompress(void *arg) {
}
