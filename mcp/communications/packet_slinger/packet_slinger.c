/**
 * @file packet_slinger.c
 *
 * @date Jan 31, 2011
 * @author seth
 * 
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2010 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/** For the convenience macros in sys/time.h */
#ifndef __USE_BSD
#define __USE_BSD
#endif

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <blast.h>
#include <tx.h>
#include <command_struct.h>
#include <bbc_pci.h>
#include <crc.h>

#include <lookup.h>
#include <fifo.h>
#include <atomic.h>

#include <blast_packet_format.h>
#include <slinger_frame.h>
#include <slinger_data.h>
#include <packet_slinger.h>


BLAST_LOOKUP_TABLE(BLAST_PORT_DEF, static);

typedef blast_master_packet_t slinger_fifo_node_t;

#define SLINGER_MAX_QUEUE_BYTES 1125
#define SLINGER_MAX_BIT_RATE 92160

slinger_link_state_t link_state[3];
static struct timeval		slinger_sleep_us = {0, 100000};

static bool			slinger_initialized = false;
int64_t 			slinger_tick_diff = 0;

static inline void slinger_set_packet_header(blast_master_packet_t *m_header, bool m_segmented, bool m_qos);
static inline size_t slinger_packetize_normal(slinger_dl_node_t *m_node, slinger_link_state_t *m_link);
static size_t slinger_packetize_small(slinger_dl_node_t *m_node, slinger_fifo_node_t *m_tempnode, slinger_link_state_t *m_state);
static inline size_t slinger_queue_small_packets(slinger_fifo_node_t *m_tempnode, slinger_link_state_t *m_state);
static void *slinger_highrate_downlink(void *m_arg __attribute__((unused)));
static void *slinger_bi0_downlink(void *m_arg __attribute__((unused)));
static void slinger_free_fifo_node(slinger_fifo_node_t *m_node);

bool slinger_link_is_active(void)
{
	return (link_state[0].is_active || link_state[1].is_active);
}

/**
 * Updates the downlink rate, setting a new maximum number of bits to attempt to
 * downlink in 1 second.  This is only applicable to the TDRSS downlink
 * @param m_bps New maximum number of bits per second
 */
void set_slinger_dl_rate(uint32_t m_bps)
{

	static channel_t *slinger_rate = NULL;
	m_bps = min(m_bps, SLINGER_MAX_BIT_RATE);

	if (!slinger_rate)
	{
		slinger_rate = channels_find_by_name("slinger_rate");
	}
	link_state[slinger_link_highrate].bps = m_bps;

	SET_SCALED_VALUE(slinger_rate, m_bps);

	blast_info("New rate for slinger downlink is %u", link_state[slinger_link_highrate].bps);
}

bool slinger_start_link(e_slinger_link m_link)
{
	if (!slinger_initialized)
	{
		blast_err("Packet Slinger not initialized!");
		return false;
	}

	if (link_state[m_link].is_active) return true;

	blast_info("Starting downlink on %s", link_state[m_link].device);

	pthread_create(&(link_state[m_link].monitor_thread), NULL, slinger_monitor, &link_state[m_link]);
	pthread_detach(link_state[m_link].monitor_thread);

	if (m_link == slinger_link_highrate)
		pthread_create(&(link_state[m_link].downlink_thread), NULL, slinger_highrate_downlink, &(link_state[m_link]));
	else
		pthread_create(&(link_state[m_link].downlink_thread), NULL, slinger_bi0_downlink, &(link_state[m_link]));
	pthread_detach(link_state[m_link].downlink_thread);

	link_state[m_link].is_active = true;

	blast_info("Started downlink on %s", link_state[m_link].device);
	return true;
}

bool slinger_stop_link(e_slinger_link m_link)
{
	blast_info("Stopping downlink on %s", link_state[m_link].device?link_state[m_link].device:"UNK");

	link_state[m_link].is_active = false;

	return true;
}

bool initialize_packet_slinger()
{
	if (slinger_initialized) return true;

	initialize_slinger_frame_lookup();

	e_memset(link_state, 0, 2 * sizeof(slinger_link_state_t));

	link_state[slinger_link_highrate].link_enum = slinger_link_highrate;
	link_state[slinger_link_biphase].link_enum = slinger_link_biphase;

	link_state[slinger_link_highrate].downlink_pq = create_pq(SLINGER_MAX_PRIORITY);
	link_state[slinger_link_biphase].downlink_pq = create_pq(SLINGER_MAX_PRIORITY);
	link_state[slinger_link_highrate].resend_fifo = fifo_new();
	link_state[slinger_link_biphase].resend_fifo = fifo_new();

	link_state[slinger_link_highrate].control_fifo_queue = fifo_new();
	link_state[slinger_link_highrate].downlink_fifo = fifo_new();
	link_state[slinger_link_biphase].control_fifo_queue = fifo_new();
	link_state[slinger_link_biphase].downlink_fifo = fifo_new();

	link_state[slinger_link_highrate].fifo_pause_us = 50000;
	link_state[slinger_link_biphase].fifo_pause_us = 9984;

	link_state[slinger_link_highrate].device = bstrdup(err, "/dev/ttyHighRate");
	link_state[slinger_link_biphase].device = bstrdup(err, "/dev/bi0_pci");

	link_state[slinger_link_biphase].bps = 1000000;

	/// Initialize the downlink rate for TDRSS Omni-directional
	set_slinger_dl_rate(CommandData.packet_slinger.downlink_rate_bps);

	initialize_slinger_tables();
	if (!slinger_xml_load_preferences("/data/etc/packetslinger.xml")) return false;

	slinger_initialized = true;
	return true;
}

static void slinger_free_fifo_node(slinger_fifo_node_t *m_node)
{
	BLAST_SAFE_FREE(m_node);
}



/**
 * Monitor thread for each packet slinger.  Takes data from the priority-ordered skip list and places it into a
 * packetized FIFO queue
 * @param m_arg Pointer to the link state structure
 */
void *slinger_monitor(void *m_arg)
{
	int32_t increment;
	slinger_dl_node_t *dl_node = NULL;
	slinger_fifo_node_t *temp_packet = NULL;
	slinger_link_state_t *link = (slinger_link_state_t *)m_arg;
	struct timeval last_downlink;
	struct timeval delta_t;
	struct timeval current_time;

	size_t level;
	size_t loops_to_clear = SLINGER_LOOPS_TO_CLEAR;

	if (!(temp_packet = balloc(err, sizeof(slinger_fifo_node_t) + SLINGER_MAX_SMALL_PAYLOAD_SIZE + 2 * sizeof(uint32_t))))
	{
		blast_tfatal("Could not create temporary packet");
		return NULL;
	}
	e_memset(temp_packet, 0, sizeof(slinger_fifo_node_t) + SLINGER_MAX_SMALL_PAYLOAD_SIZE + sizeof(uint32_t));

	gettimeofday(&current_time, NULL);
	timersub(&current_time, &slinger_sleep_us, &last_downlink);

	blast_startup("Started Packet Slinger monitor thread");
	while(link->is_active)
	{
		gettimeofday(&current_time, NULL);
		timersub(&current_time, &last_downlink, &delta_t);
		last_downlink.tv_sec = current_time.tv_sec;
		last_downlink.tv_usec = current_time.tv_usec;

		if (link->bytes_to_write <= 0)
		{
			increment = (uint64_t)link->bps * delta_t.tv_usec / 8000000;
			ADD_TO(link->bytes_to_write, increment);
			link->bytes_to_write = min_safe(link->bytes_to_write, increment);
		}

		if ( (link->bytes_to_write > 0)
				&& (dl_node = dequeue_pq(link->downlink_pq, &level)) )
		{
			if ((float)level > link->max_priority) link->max_priority = (float)level;


			if (!dl_node->data)
			{
				if (!slinger_compress_node(dl_node))
				{
					blast_warn("Could not compress node of length %zu", dl_node->size);
					slinger_free_dl_node(dl_node);
					continue;
				}
			}

			/**
			 * Assign the packet data header type to be the dl_node type's
			 * bottom three bits.  This is hacky but used to quickly implement
			 * a mandated LOS change
			 */
			dl_node->data->type = (dl_node->type & 0b111);

			/**
			 * Process the highest-priority node in the skip list.  Small packets get lumped together to
			 * save overhead.  Larger packets are sent individually.
			 */
			if (dl_node->size < SLINGER_MAX_SEGMENT_SIZE && dl_node->data->type == BLAST_PORT_DEF_DATA)
			{
				slinger_packetize_small(dl_node, temp_packet, link);
			}
			else if (dl_node->size < SLINGER_MAX_PAYLOAD_SIZE)
			{
				slinger_packetize_normal(dl_node, link);
			}
			else
			{
				blast_err("Packet size %zu larger than maximum downlink size %u", dl_node->size, SLINGER_MAX_PAYLOAD_SIZE);
			}

			slinger_free_dl_node(dl_node);
		}

		if (!dl_node || link->bytes_to_write <= 0)
		{
			if (!dl_node)
			{
				link->max_priority = min(link->max_priority + 1., (float)SLINGER_MAX_PRIORITY);
			}
			else if (link->max_priority > (float)level)
			{
				link->max_priority = link->max_priority - SLINGER_PRIORITY_ADJUST_RATE * (link->max_priority - (float)level);
			}

			if (!loops_to_clear--)
			{
				clear_levels_pq(link->downlink_pq, ceilf(link->max_priority), slinger_free_dl_node);
				loops_to_clear = SLINGER_LOOPS_TO_CLEAR;
			}
			usleep(slinger_sleep_us.tv_usec);
		}

	};

	blast_info("Stopped Packet Slinger monitor thread");
	BLAST_SAFE_FREE(temp_packet);

	clear_levels_pq(link->downlink_pq, 0, slinger_free_dl_node);
	return NULL;
}

/**
 * Main monitor loop for TDRSS slinger instance.  Provides the interface between the FIFO queue and the
 * High-Rate serial port
 * @param m_arg
 */
void *slinger_highrate_downlink(void *m_arg __attribute__((unused)))
{
	int slinger_serial_fd;
	struct termios slinger_termios;
	uint32_t ack_val = 0;

	slinger_fifo_node_t *current_node = NULL;

	blast_master_packet_t blank_header =
				{	.magic = BLAST_MAGIC8,
				 	.version = 1
				};

	slinger_serial_fd = open(link_state[slinger_link_highrate].device, O_NOCTTY | O_RDWR );
	if (slinger_serial_fd == -1)
	{
		blast_tfatal("Could not connect to %s", link_state[slinger_link_highrate].device);
		return NULL;
	}
	e_memset(&slinger_termios, 0, sizeof(struct termios));
	if (cfsetospeed(&slinger_termios, B115200) || cfsetispeed(&slinger_termios, B115200))
	{
		blast_tfatal("Could not set speed to %d", (int)B115200);
		close(slinger_serial_fd);
		return NULL;
	}

	slinger_termios.c_cflag |= (CS8 | CLOCAL | CREAD);
	tcsetattr(slinger_serial_fd, TCSANOW, &slinger_termios);


	blast_startup("Started Packet Slinger Downlink Thread on %s", link_state[slinger_link_highrate].device);

	while (link_state[slinger_link_highrate].is_active)
	{
		/**
		 * Start by fetching the oldest ack packet (if one exists).  We'll downlink this whether or not we have a
		 * data packet.
		 */
		ack_val = (uint32_t)((intptr_t)fifo_pop(link_state[slinger_link_highrate].control_fifo_queue));

		/**
		 * Grab the oldest data packet
		 */
		if ((current_node = fifo_pop(link_state[slinger_link_highrate].downlink_fifo)))
		{
			ssize_t written_bytes;

			current_node->has_ack = !!ack_val;

			BLAST_MASTER_PACKET_CRC(current_node) =
			        crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(current_node), current_node->length);

			written_bytes = write(slinger_serial_fd, current_node, BLAST_MASTER_PACKET_FULL_LENGTH(current_node));
			if ( written_bytes != (ssize_t)BLAST_MASTER_PACKET_FULL_LENGTH(current_node))
			{
				blast_strerror("Error writing packet to link.  Tried to write %zu, wrote %zd",
						BLAST_MASTER_PACKET_FULL_LENGTH(current_node), written_bytes);
			}

			if (ack_val && (write(slinger_serial_fd, &ack_val, sizeof(ack_val)) != sizeof(ack_val)))
			{
				blast_strerror("Error writing ack to link");
			}

			/**
			 * After sending, if we want confirmation of the packet's arrival, we store the packet for re-transmission
			 */
			if (current_node->qos)
			{
				fifo_push(link_state[slinger_link_highrate].resend_fifo, current_node);
			}
			else
			{
				slinger_free_fifo_node(current_node);
			}

		}
		else
		{
			if (ack_val)
			{
				blast_dbg("No data.  Sending empty Ack packet");

				blank_header.has_ack = true;

				blank_header.length = 0;
				write(slinger_serial_fd, &blank_header, sizeof(blast_master_packet_t));
				write(slinger_serial_fd, (uint32_t[]){0}, sizeof(uint32_t));
				write(slinger_serial_fd, &ack_val, sizeof(ack_val));
			}

			if (usleep(link_state[slinger_link_highrate].fifo_pause_us) == -1) break;
		}
	}

	blast_info("Stopped Packet Slinger Downlink Thread");

	close(slinger_serial_fd);
	/// Empty the FIFOs at the end of the thread
	while (fifo_pop(link_state[slinger_link_highrate].control_fifo_queue));
	while ((current_node = fifo_pop(link_state[slinger_link_highrate].downlink_fifo)))
		slinger_free_fifo_node(current_node);

	return NULL;
}

/**
 * Main monitor loop for each slinger instance.  Provides the interface between the FIFO queue and the
 * Biphase character device (/dev/bi0)
 * @param m_arg
 */
static void *slinger_bi0_downlink(void *m_arg __attribute__((unused)))
{
	int			fd = -1;
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

	slinger_fifo_node_t *current_node = NULL;


	if ((fd = open(link_state[slinger_link_biphase].device, O_RDWR)) < 0)
	{
		blast_tfatal("Could not allocated new serial structure");
		return NULL;
	}

	blast_startup("Started Packet Slinger Downlink Thread on %s", link_state[slinger_link_biphase].device);

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
			memcpy(bi0_frame, bi0_buffer.framelist[write_frame], bi0_buffer.framesize[write_frame]);
			memset(bi0_frame + bi0_buffer.framesize[write_frame], 0xEE, BI0_FRAME_SIZE * sizeof(uint16_t) - bi0_buffer.framesize[write_frame]);
			frame_offset = bi0_buffer.framesize[write_frame];

			while (frame_offset < bi0_frame_bytes)
			{
				/**
				 * If we have no more data to cache, break to the write
				 */
				if (!node_remaining &&
						!(current_node = fifo_pop(link_state[slinger_link_biphase].downlink_fifo)))
				{
					break;
				}

				/**
				 * Initialize a new node structure for downlink
				 */
				if (!node_offset)
				{
					node_remaining = BLAST_MASTER_PACKET_FULL_LENGTH(current_node);
					if ((ack_val = (uint32_t)((intptr_t)fifo_pop(link_state[slinger_link_biphase].control_fifo_queue))))
					{
						current_node->has_ack = true;
						node_remaining += sizeof(uint32_t);
						BLAST_MASTER_PACKET_ACK(current_node) = ack_val;
						ack_val = 0;
					}

					BLAST_MASTER_PACKET_CRC(current_node) =
							crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(current_node), current_node->length);
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
					/**
					 * After sending, if we want confirmation of the packet's arrival, we store the packet for re-transmission
					 */
					if (current_node->qos)
					{
						fifo_push(link_state[slinger_link_biphase].resend_fifo, current_node);
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
			bi0_frame[BI0_FRAME_SIZE - 1] = crc16(BLAST_MAGIC16, bi0_frame, bi0_frame_bytes);

			bi0_frame[0] = sync_word;
			sync_word = ~sync_word;

			if (write(fd, bi0_frame, 2 * BI0_FRAME_SIZE) != 2 * BI0_FRAME_SIZE)
			{
				blast_err("Error writing frame to Biphase, discarding.");
				node_remaining = 0;
			}

			write_frame = (write_frame + 1) & 7;

		}

		bi0_buffer.i_out = write_frame;
		usleep(10000);
	}

	slinger_free_fifo_node(current_node);
	close(fd);

	/// Empty the FIFOs at the end of the thread
	while (fifo_pop(link_state[slinger_link_biphase].control_fifo_queue));
	while ((current_node = fifo_pop(link_state[slinger_link_biphase].downlink_fifo)))
		slinger_free_fifo_node(current_node);

	blast_info("Stopped Packet Slinger Biphase Downlink Thread");
	return NULL;
}

void slinger_set_packet_header(blast_master_packet_t *m_header, bool m_segmented, bool m_qos)
{
	m_header->version = BLAST_SLINGER_VERSION;
	m_header->magic = BLAST_SLINGER_MAGIC;
	m_header->qos = m_qos;
	m_header->multi_packet = m_segmented;
}

static inline size_t slinger_packetize_normal(slinger_dl_node_t *m_node, slinger_link_state_t *m_link)
{
	slinger_fifo_node_t *new_fifo_node;

    new_fifo_node = m_node->data;
    m_node->data = NULL;

	slinger_set_packet_header(new_fifo_node, false, m_node->qos);
	if (m_link->bytes_to_write > 0)
	{
		ADD_TO(m_link->bytes_to_write, -(BLAST_MASTER_PACKET_FULL_LENGTH(new_fifo_node) + sizeof(uint32_t)));
		fifo_push(m_link->downlink_fifo, new_fifo_node);
	}

	return m_node->size;

}

static inline size_t slinger_queue_small_packets(slinger_fifo_node_t *m_tempnode, slinger_link_state_t *m_state)
{
	slinger_fifo_node_t *new_fifo_node = NULL;
	size_t retval = 0;

	if (m_tempnode->length)
	{
		new_fifo_node = memdup(err, m_tempnode, BLAST_MASTER_PACKET_FULL_LENGTH(m_tempnode));
		slinger_set_packet_header(new_fifo_node, true, false);
		ADD_TO(m_state->bytes_to_write, -(BLAST_MASTER_PACKET_FULL_LENGTH(new_fifo_node) + sizeof(uint32_t)));
		fifo_push(m_state->downlink_fifo, new_fifo_node);

		retval = BLAST_MASTER_PACKET_FULL_LENGTH(m_tempnode);
		memset(m_tempnode, 0, BLAST_MASTER_PACKET_FULL_LENGTH(m_tempnode));
	}
	return retval;
}

static size_t slinger_packetize_small(slinger_dl_node_t *m_node, slinger_fifo_node_t *m_tempnode, slinger_link_state_t *m_state)
{
	uint8_t *data;
	uint8_t *input;
	size_t retval = 0;

	data = (uint8_t*)&BLAST_MASTER_PACKET_CRC(m_tempnode);
	input = BLAST_MASTER_PACKET_PAYLOAD(m_node->data);

	*data++ = (uint8_t)m_node->data->length;

	memcpy(data, input, m_node->data->length);
	m_tempnode->length += (m_node->data->length + sizeof(uint8_t));

	if (m_tempnode->length > SLINGER_MAX_SMALL_PAYLOAD_SIZE -
			(SLINGER_MAX_SEGMENT_SIZE + sizeof(uint32_t) + sizeof(uint8_t)))
	{
		retval = slinger_queue_small_packets(m_tempnode, m_state);
	}

	return retval;
}

/**
 * Prepares and queues an acknowledgment header packet for packets that request confirmation.  These are stored
 * in a FIFO for each link to be sent on an opportunistic basis
 * @param m_header Pointer to the packet header that should be acknowledged
 */
void slinger_acknowledge_packet(blast_master_packet_t *m_header)
{
	intptr_t ackval = BLAST_MASTER_PACKET_CRC(m_header);

	for (int i = 0; i < 2; i++)
	{
		if (link_state[i].is_active) fifo_push(link_state[i].control_fifo_queue, (void*)ackval);
	}
}


void slinger_receive_ack_packet(int m_link, uint32_t m_packet_id)
{
	slinger_fifo_node_t *temp_node = NULL;

	while ((temp_node = fifo_pop(link_state[m_link].resend_fifo)))
	{
		if (BLAST_MASTER_PACKET_CRC(temp_node) != m_packet_id)
		{
			/**
			 * If we find a node in the resend list that matches the first element (we should!) then
			 * clear the QoS flag so we don't get a packet storm and then re-queue in the FIFO
			 */
			temp_node->qos = false;
			temp_node->has_ack = false;
			BLAST_MASTER_PACKET_CRC(temp_node) = crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(temp_node), temp_node->length);

			if (link_state[m_link].is_active)
			{
				ADD_TO(link_state[m_link].bytes_to_write, -BLAST_MASTER_PACKET_FULL_LENGTH(temp_node));
				fifo_push(link_state[m_link].downlink_fifo, temp_node);
			}
			else
				slinger_free_fifo_node(temp_node);
		}
		else
		{
			slinger_free_fifo_node(temp_node);
			break;
		}
	}

}

