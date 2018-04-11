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
#include "linklist.h"
#include "linklist_compress.h"
#include "bitserver.h"
#include "mcp.h"
#include "data_sharing_server.h"

struct BITSender shared_data_sender = {0};
struct BITRecver shared_data_recver = {0};
unsigned int shared_packet_size = 0;
uint8_t * shared_recv_buffer = NULL;
uint8_t * shared_cmddata = NULL;
uint8_t * shared_superframe = NULL;
int shared_data_recvd = 0;

extern int16_t InCharge;
extern int16_t SouthIAm;

linklist_t * shared_ll = NULL;

void data_sharing_init(linklist_t ** ll_array) {
  linklist_t * temp_ll = linklist_find_by_name("shared.ll", ll_array);

  if (!temp_ll) {
    blast_err("Cannot find shared.ll for data sharing");
    return;
  }

  // initialize sizes and buffers
  shared_packet_size = temp_ll->blk_size+sizeof(struct CommandDataStruct);
  shared_recv_buffer = calloc(1, shared_packet_size);
  shared_cmddata = shared_recv_buffer+temp_ll->blk_size;
  shared_superframe = calloc(1, superframe_size);

  // initialize bitserver
  initBITSender(&shared_data_sender, (SouthIAm) ? NORTH_IP : SOUTH_IP, DATA_SHARING_PORT, 3,
                  shared_packet_size, shared_packet_size);
  initBITRecver(&shared_data_recver, (SouthIAm) ? SOUTH_IP : NORTH_IP, DATA_SHARING_PORT, 3,
                  shared_packet_size, shared_packet_size);

  setBITSenderSerial(&shared_data_sender, *(uint32_t *) temp_ll->serial);
  setBITRecverSerial(&shared_data_recver, *(uint32_t *) temp_ll->serial);

  shared_ll = temp_ll;
}

void share_superframe(uint8_t * superframe) {
  static int write_prevstatus_counter = 0;
  int retval;

  if (!shared_ll) return;

  // reset the new data flag to prevent stagnant data
  shared_data_recvd = 0;

  // make new shared data frame and send to other flc
  compress_linklist(shared_recv_buffer, shared_ll, superframe);
  memcpy(shared_cmddata, &CommandData, sizeof(struct CommandDataStruct));
  retval = sendToBITSender(&shared_data_sender, shared_recv_buffer, shared_packet_size, 0);

  // blast_info("Sending shared data");

  if (retval != shared_packet_size) {
    blast_err("Could only send %d/%d bytes of shared data", retval, shared_packet_size);
  }

  // recv shared data frome of the other flc
  if (peekBITRecver(&shared_data_recver)) {
    retval = recvFromBITRecver(&shared_data_recver, shared_recv_buffer, shared_packet_size, RECV_TIMEOUT);
    if (retval != shared_packet_size) {
      if (retval > 0) blast_err("Received %d bytes, expected %d bytes of shared data", retval, shared_packet_size);
      return;
    }

    // have valid data, so decompress to superframe
    decompress_linklist(shared_superframe, shared_ll, shared_recv_buffer);

    // overwrite command data if not in charge
		if (!InCharge) {
      memcpy(&CommandData, shared_cmddata, sizeof(struct CommandDataStruct));
      // blast_info("Overwriting command data");

      // save command data to prev_status file
		  if (write_prevstatus_counter%10 == 0) {
			  WritePrevStatus();
			  write_prevstatus_counter = 0;
        // blast_info("Saving recvd command data to disk");
	 	  }
      write_prevstatus_counter++;
    }

    // set the flag that fresh shared data has been received
    shared_data_recvd = 1;
  }
}

// grabs shared data from the fifo for the specified field at the specified rate
void share_data(E_RATE rate) {
  static unsigned int frame_location[RATE_END] = {0};

  if (!shared_ll || !shared_data_recvd) return;

  int i;
  channel_t * chan = NULL;
  uint8_t * data_loc = NULL;
  const char which_flc[2] = {'n', 's'};

  for (i = 0; i < shared_ll->n_entries; i++) {
    chan = shared_ll->items[i].tlm;

    // don't process null channels or channels not at this rate
    if (!chan) continue;
    if (chan->rate != rate) continue;

    // logic for flc specified channels
    int len = strlen(chan->field);
    bool swi = (chan->field[len-2] == '_');
    bool this_flc = swi && (chan->field[len-1] == which_flc[SouthIAm]);
    bool that_flc = swi && (chan->field[len-1] == which_flc[!SouthIAm]);

    // only overwrite data if
    // a) I am not in charge and the channel is not specific to this flc OR
    // b) The channel is specific to the other flc
    if ((!InCharge && !this_flc) || that_flc) {
      // overwrite data with shared data
      data_loc = shared_superframe+get_channel_start_in_superframe(chan)+get_channel_skip_in_superframe(chan)*frame_location[rate];
      memcpy(chan->var, data_loc, channel_size(chan));
     //  if (!frame_location[rate]) blast_info("Copying shared data \"%s\"=%.4x", chan->field, *(uint16_t *) chan->var);
    }
  }
  // increment the frame location
  frame_location[rate] = (frame_location[rate]+1)%get_spf(rate);
}
