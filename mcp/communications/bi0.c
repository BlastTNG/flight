/* fcp: the EBEX flight control program
 *
 * This software is copyright (C) 2009 Columbia University
 *                            (C) 2016 University of Pennsylvania
 *
 * This file is part of fcp.
 *
 * fcp is free software; you can redistribute it and/or modify
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
#include <unistd.h>
#include <stdint.h>
#include <packet_slinger.h>

#include <blast.h>

#include "bi0.h"
#include "channels_tng.h"


bi0_buffer_t bi0_buffer;

void initialize_biphase_buffer(void)
{
	int i;
	size_t max_rate_size = 0;

	//for (int rate = RATE_100HZ; rate < RATE_END; rate++) {
	//    if (max_rate_size < frame_size[rate]) max_rate_size = frame_size[rate];
	//}
    max_rate_size = frame_size[RATE_100HZ]; // Temporary, for now only implementing biphase on the 100 Hz framerate
	bi0_buffer.i_in = 10; /* preload the fifo */
	bi0_buffer.i_out = 0;
	for (i = 0; i < BI0_FRAME_BUFLEN; i++)
	{
		bi0_buffer.framelist[i] = calloc(1, max_rate_size);
		memset(bi0_buffer.framelist[i], 0, max_rate_size);
	}

}

void push_bi0_buffer(const void *m_frame)  // what was there before uint16_t *m_frame
{
	int i_in;

	i_in = (bi0_buffer.i_in + 1) & BI0_FRAME_BUFMASK;    // Does this automatically wrap when i_in becomes greater than the number of frames? I think so from BUFMASK
    bi0_buffer.framesize[i_in] = frame_size[RATE_100HZ]; // Joy added this, later will have to be modified for any rate
    memcpy(bi0_buffer.framelist[i_in], m_frame, frame_size[RATE_100HZ]; // Later will have to adapt to various sizes for various frequencies
	bi0_buffer.i_in = i_in;
}

void biphase_writer(void)
{

// BLAST CODE
//  uint16_t  *frame;
//
//  nameThread("Bi0");
//  bputs(startup, "Startup\n");
//
//  while (!biphase_is_on)
//    usleep(10000);
//
//  bputs(info, "Veto has ended.  Here we go.\n");
//
//  while (1) {
//    frame = PopFrameBuffer(&bi0_buffer);
//    write_to_biphase(frame);
//    }
//  }
// }
}
// EBEX CODE for biphase writer
/*
	uint32_t 	ack_val = 0;

	size_t		node_remaining = 0;
	size_t		write_len;
	size_t		frame_offset = 2;
	size_t		node_offset = 0;
	uint16_t	bi0_frame[BI0_FRAME_SIZE];
	uint16_t	sync_word = 0xEB90;

	uint16_t	read_frame;
	uint16_t	write_frame;

	size_t 			slinger_frame_bytes;
	const size_t	bi0_frame_bytes = (BI0_FRAME_SIZE - 1) * 2;

	slinger_frame_bytes = (BI0_FRAME_SIZE - BiPhaseFrameWords) * sizeof(uint16_t);

	while (link_state[slinger_link_biphase].is_active)
	{
		write_frame = bi0_buffer.i_out;
		read_frame = bi0_buffer.i_in;
		if (read_frame == write_frame)
		{
			usleep(10000);
			continue;
		}

		while (read_frame != write_frame)
		{
			memcpy(bi0_frame, bi0_buffer.framelist[write_frame], BiPhaseFrameWords * sizeof(uint16_t));
			e_memset(&bi0_frame[BiPhaseFrameWords], 0xEE, slinger_frame_bytes);
			frame_offset = BiPhaseFrameWords * sizeof(uint16_t);

			while (frame_offset < bi0_frame_bytes)
			{
				 //If we have no more data to cache, break to the write
				if (!node_remaining &&
						!(current_node = ebex_fifo_pop(link_state[slinger_link_biphase].downlink_fifo)))
				{
					break;
				}

				 //Initialize a new node structure for downlink
				if (!node_offset)
				{
					node_remaining = EBEX_MASTER_PACKET_FULL_LENGTH(current_node);
					if ((ack_val = (uint32_t)((intptr_t)ebex_fifo_pop(link_state[slinger_link_biphase].control_fifo_queue))))
					{
						current_node->has_ack = true;
						node_remaining += sizeof(uint32_t);
						EBEX_MASTER_PACKET_ACK(current_node) = ack_val;
						ack_val = 0;
					}

					EBEX_MASTER_PACKET_CRC(current_node) =
							ebex_crc32(EBEX_MASTER_PACKET_PAYLOAD(current_node), current_node->length, CRC32_SEED);
				}

				if (node_remaining)
				{
					write_len = min(bi0_frame_bytes - frame_offset, node_remaining);

					memcpy(((uint8_t*)bi0_frame) + frame_offset,
							((uint8_t*)current_node) + node_offset, write_len);
					node_remaining -= write_len;
					frame_offset += write_len;
					node_offset += write_len;
				};

				if (!node_remaining)
				{
					 //After sending, if we want confirmation of the packet's arrival, we store the packet for re-transmission
					if (current_node->qos)
					{
						ebex_fifo_push(link_state[slinger_link_biphase].resend_fifo, current_node);
					}
					else
					{
						slinger_free_fifo_node(current_node);
					}

					node_offset = 0;
					current_node = NULL;
				}
			}

			bi0_frame[0] = 0xEB90;
			bi0_frame[BI0_FRAME_SIZE - 1] = ebex_crc16(bi0_frame, bi0_frame_bytes, CRC16_SEED);

			bi0_frame[0] = sync_word;
			sync_word = ~sync_word;

			if (write(fd, bi0_frame, 2 * BI0_FRAME_SIZE) != 2 * BI0_FRAME_SIZE)
			{
				ebex_err("Error writing frame to Biphase, discarding.");
				node_remaining = 0;
			}

			write_frame = (write_frame + 1) & BI0_FRAME_BUFMASK;

		}

		bi0_buffer.i_out = write_frame;
		usleep(10000);
	}

	slinger_free_fifo_node(current_node);
	close(fd);

	/// Empty the FIFOs at the end of the thread
	while (ebex_fifo_pop(link_state[slinger_link_biphase].control_fifo_queue));
	while ((current_node = ebex_fifo_pop(link_state[slinger_link_biphase].downlink_fifo)))
		slinger_free_fifo_node(current_node);

	ebex_info("Stopped Packet Slinger Biphase Downlink Thread");
	return NULL;

}
*/
