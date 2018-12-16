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
 * This file contains functions for compressing and sending pilot data using linklists. 
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

#include <linklist.h>
#include <linklist_compress.h>

#include "mcp.h"
#include "FIFO.h"
#include "bitserver.h"
#include "pilot.h"
#include "blast.h"
#include "mputs.h"
#include "command_struct.h"

extern int16_t InCharge;

#define NUM_PILOT_TARGETS 4

struct Fifo pilot_fifo = {0};

void pilot_compress_and_send(void *arg) {
  // initialize UDP connection using bitserver/BITSender
  struct BITSender pilotothsender[4] = {{0}};
  unsigned int fifosize = MAX(PILOT_MAX_SIZE, superframe->allframe_size);
  for (int i = 0; i < NUM_PILOT_TARGETS; i++) {  
    initBITSender(&pilotothsender[i], pilot_target_names[i], PILOT_PORT, 10, fifosize, PILOT_MAX_PACKET_SIZE);
  }
  linklist_t * ll = NULL, * ll_old = NULL, * ll_saved = NULL;
  linklist_t ** ll_array = arg;

  uint8_t * compbuffer = calloc(1, fifosize);
  unsigned int allframe_bytes = 0;
  double bandwidth = 0;
  uint32_t transmit_size = 0;

  nameThread("Pilot");

  while (1) {
    // get the current pointer to the pilot linklist
    ll = ll_array[PILOT_TELEMETRY_INDEX];
    if (ll != ll_old) {
        if (ll) blast_info("Pilot linklist set to \"%s\"", ll->name);
        else blast_info("Pilot linklist set to NULL");
    }
    ll_old = ll;

    // get the current bandwidth
    if ((bandwidth != CommandData.pilot_bw) ||
         (CommandData.pilot_allframe_fraction < 0.0001)) allframe_bytes = 0;
    bandwidth = CommandData.pilot_bw;

    if (!fifoIsEmpty(&pilot_fifo) && ll && InCharge) { // data is ready to be sent

      if (!strcmp(CommandData.pilot_linklist_name, FILE_LINKLIST)) { // special file downlinking 
        // done sending, so revert to other linklist
        if (ll->blocks[0].i >= ll->blocks[0].n) {
          ll_array[PILOT_TELEMETRY_INDEX] = ll_saved;
          if (ll_saved) strcpy(CommandData.pilot_linklist_name, ll_saved->name);
          continue;
        }

				// use the full bandwidth
				transmit_size = bandwidth;

				// fill the downlink buffer as much as the downlink will allow 
				unsigned int bytes_packed = 0;
				while ((bytes_packed+ll->blk_size) <= transmit_size) {
						compress_linklist(compbuffer+bytes_packed, ll, getFifoRead(&pilot_fifo));
						bytes_packed += ll->blk_size;
				} 
				decrementFifo(&pilot_fifo);

      } else { // normal linklist
        ll_saved = ll;

				// send allframe if necessary
				if (allframe_bytes >= superframe->allframe_size) {
					transmit_size = write_allframe(compbuffer, superframe, getFifoRead(&pilot_fifo));
					allframe_bytes = 0;
				} else {
					transmit_size = MIN(ll->blk_size, bandwidth*(1.0-CommandData.pilot_allframe_fraction));  

					// compress the linklist
					compress_linklist(compbuffer, ll, getFifoRead(&pilot_fifo));

					// bandwidth limit; frames are 1 Hz, so bandwidth == size
					allframe_bytes += bandwidth*CommandData.pilot_allframe_fraction;
				  decrementFifo(&pilot_fifo);
				}
      }

			// no packetization if there is nothing to transmit
			if (!transmit_size) continue;

			// send the data to pilot oth via bitsender
      int ind = CommandData.pilot_oth;

			// have packet header serials match the linklist serials
			setBITSenderSerial(&pilotothsender[ind], *(uint32_t *) ll->serial);

			// commendeer the framenum for total transmit size
			setBITSenderFramenum(&pilotothsender[ind], transmit_size);

			// send the data over pilot via bitsender
			sendToBITSender(&pilotothsender[ind], compbuffer, transmit_size, 0);

      memset(compbuffer, 0, PILOT_MAX_SIZE);
    } else {
      usleep(100000); // zzz...
    }
  }
}
