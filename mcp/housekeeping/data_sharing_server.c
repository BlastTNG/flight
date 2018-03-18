/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/statvfs.h>

#include "mputs.h"

#include "channels_tng.h"
#include "command_struct.h"
#include "FIFO.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "bitserver.h"
#include "mcp.h"
#include "data_sharing_server.h"

struct Fifo shared_data_fifo = {0};
struct Fifo shared_cmddata_fifo = {0};

extern int16_t InCharge;
extern int16_t SouthIAm;

linklist_t * shared_ll = NULL;

void data_sharing_routine(void *arg) {
  struct BITSender data_sender = {0};
  struct BITRecver data_recver = {0};
  linklist_t ** ll_array = (linklist_t **) arg;

  nameThread("DataShare");

  linklist_t * temp_ll = linklist_find_by_name("shared.ll",ll_array);

  if (!temp_ll) {
    blast_err("Cannot find shared.ll for data sharing");
    return;
  }

  unsigned int packet_size = temp_ll->blk_size+sizeof(struct CommandDataStruct);
  uint8_t * recv_buffer = calloc(1, packet_size);
  uint8_t * cmddata = recv_buffer+temp_ll->blk_size;

  // allocate fifos
  allocFifo(&shared_data_fifo, 3, superframe_size);
  allocFifo(&shared_cmddata_fifo, 3, sizeof(struct CommandDataStruct));

  // initialize bitserver
  initBITSender(&data_sender, (SouthIAm) ? NORTH_IP : SOUTH_IP, DATA_SHARING_PORT, 3, 
                  packet_size, packet_size);
  initBITRecver(&data_recver, (SouthIAm) ? SOUTH_IP : NORTH_IP, DATA_SHARING_PORT, 3, 
                  packet_size, packet_size);

  setBITSenderSerial(&data_sender, *(uint32_t *) temp_ll->serial);
  setBITRecverSerial(&data_recver, *(uint32_t *) temp_ll->serial);

  int recv_size = 0;

  sleep(4);
  shared_ll = temp_ll;

  // if not in charge, receive data from the InCharge computer
  while (!InCharge) {
    if (peekBITRecver(&data_recver)) {
      recv_size = recvFromBITRecver(&data_recver, recv_buffer, packet_size, 0);
      if (recv_size != packet_size) {
        if (recv_size >= 0) blast_err("Received %d bytes (expected %d bytes)", recv_size, packet_size);
        continue;
      } 
      decompress_linklist(getFifoWrite(&shared_data_fifo), shared_ll, recv_buffer);
      memcpy(getFifoWrite(&shared_cmddata_fifo), cmddata, sizeof(struct CommandDataStruct));
      
      incrementFifo(&shared_data_fifo);
      incrementFifo(&shared_cmddata_fifo);
    } else {
      usleep(100000);
    }
  }

  // clear all data in the fifo for command switchover
  clearFifo(&shared_data_fifo);
  clearFifo(&shared_cmddata_fifo);

  // now we're in charge, so send data to not InCharge computer 
  blast_info("Now in charge! Switching to data sharing server mode.");
  while (true) {
    if (!fifoIsEmpty(&shared_data_fifo)) {
      compress_linklist(recv_buffer, shared_ll, getFifoRead(&shared_data_fifo));
      decrementFifo(&shared_data_fifo);
      
      memcpy(cmddata, &CommandData, sizeof(struct CommandDataStruct));

      sendToBITSender(&data_sender, recv_buffer, packet_size, 0);
      // blast_info("Sending shared data\n");
    } else {
      usleep(100000);
    }

  }

}

void share_superframe(uint8_t * superframe) {
  if (!shared_ll) return;

  static int write_prevstatus_counter = 0;

  if (InCharge) { // in charge, so send data to not in charge computer
    memcpy(getFifoWrite(&shared_data_fifo), superframe, superframe_size);
    incrementFifo(&shared_data_fifo);
  } else { // not in charge, so copy command data and decrement the read fifos
    if (!fifoIsEmpty(&shared_data_fifo)) {
      decrementFifo(&shared_data_fifo);
    }
    if (!fifoIsEmpty(&shared_cmddata_fifo)) { 
      memcpy(&CommandData, getFifoRead(&shared_cmddata_fifo), sizeof(struct CommandDataStruct));
      decrementFifo(&shared_cmddata_fifo);
      if (write_prevstatus_counter%10 == 0) {
        WritePrevStatus();
        write_prevstatus_counter = 0;
      }
    }
  }
  write_prevstatus_counter++;
}

// grabs shared data from the fifo for the specified field at the specified rate
void share_data(E_RATE rate) {
  if (!shared_ll) return;  
  if (InCharge) return;

  static unsigned int frame_location[RATE_END] = {0};

  int i;
  channel_t * chan = NULL;
  uint8_t * data_loc = NULL;
  uint8_t * buffer = getFifoRead(&shared_data_fifo);

  for (i = 0; i < shared_ll->n_entries; i++) {
    chan = shared_ll->items[i].tlm;

    if (!chan) continue; 
    if (chan->rate != rate) continue;

    data_loc = buffer+get_channel_start_in_superframe(chan)+superframe_skip[rate]*frame_location[rate];
    memcpy(chan->var, data_loc, channel_size(chan));

    // blast_info("Copying shared data \"%s\"=%.4x", chan->field, *(uint16_t *) chan->var);
  }
  // increment the frame location
  frame_location[rate] = (frame_location[rate]+1)%get_spf(rate);
}
