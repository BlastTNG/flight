/**
 * @file data_sharing.c
 *
 * @date Dec 25, 2012
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
 * Revisions copyright (C) 2015 Seth Hillbrand
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

#include <stdint.h>
#include <stdbool.h>

#include <command_struct.h>

#include <blast.h>
#include <blast_comms.h>
#include <blast_packet_format.h>
#include <crc.h>
#include <comms_netsock.h>

#include <data_sharing.h>


typedef struct
{
	blast_master_packet_t header;
	data_sharing_t payload;
	uint32_t crc;
} __attribute__((packed)) sharing_packet_t;

typedef enum
{
    type_frame_data,
    type_command_data,
    type_ethercat_data
} e_sharing_type;

static data_sharing_t received_data = {0};
static comms_socket_t *sharing_sock = NULL;

static int data_sharing_process_packet(const void *m_data, size_t m_len, void *m_userdata );
static int data_sharing_net_cleanup(const void *m_data, size_t m_len, void *m_userdata );
static void data_sharing_net_error(int m_code, void *m_userdata);

void initialize_data_sharing(void)
{
	sharing_sock = comms_sock_new();

	comms_sock_multicast_listen(sharing_sock, "224.0.0.0", 1300);

	sharing_sock->callbacks.priv = sharing_sock;
	sharing_sock->callbacks.data = data_sharing_process_packet;
	sharing_sock->callbacks.finished = data_sharing_net_cleanup;
	sharing_sock->callbacks.error = data_sharing_net_error;

	blast_comms_add_socket(sharing_sock);
}

void data_sharing_send_data(const data_sharing_t *m_data)
{
    sharing_packet_t packet =
        {
                .header.length = sizeof(data_sharing_t),
                .header.type = type_frame_data,
                .header.version = 1,
                .header.magic = BLAST_MAGIC8,
        };

    memcpy(&packet.payload, m_data, sizeof(data_sharing_t));

    packet.crc = crc32(BLAST_MAGIC32, &packet.payload, packet.header.length);
    comms_sock_write(sharing_sock, &packet, sizeof(packet));
}

void data_sharing_get_data(data_sharing_t *m_data)
{
    if (m_data) memcpy(m_data, &received_data, sizeof(data_sharing_t));
}

void data_sharing_request_commanddata(void)
{
	struct
	{
		blast_master_packet_t header;
		uint32_t crc;
	} __attribute__((packed))  packet =
	    {
	            .header.length = 0,
	            .header.type = type_command_data,
	            .header.version = 1,
	            .header.magic = BLAST_MAGIC8,
	            .crc = BLAST_MAGIC32
	    };

    comms_sock_write(sharing_sock, &packet, sizeof(packet));
}

void data_sharing_send_commanddata(void)
{
	struct
	{
		blast_master_packet_t header;
		struct CommandDataStruct payload;
		uint32_t crc;
	} __attribute__((packed)) packet =
        {
                .header.length = sizeof(struct CommandDataStruct),
                .header.type = type_command_data,
                .header.version = 1,
                .header.magic = BLAST_MAGIC8,
        };

	memcpy(&packet.payload, &CommandData, sizeof(struct CommandDataStruct));
	packet.crc = crc32(BLAST_MAGIC32, &packet.payload, packet.header.length);
    comms_sock_write(sharing_sock, &packet, sizeof(packet));
}

/**
 * Handles the UDP sharing packet
 * @param m_data Pointer to the data queue
 * @param m_len length in bytes of the data queue
 * @param m_userdata unused
 * @return Number of bytes processed (that should be removed from the queue)
 */
static int data_sharing_process_packet(const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)) )
{
	const uint8_t *packet_string = (const uint8_t *)m_data;
	blast_master_packet_t *header = NULL;
	data_sharing_t *sharing_packet = NULL;
	size_t consumed = 0;

	while (consumed < m_len)
	{
		if (packet_string[consumed] == BLAST_MAGIC8) break;
		consumed++;
	}

	if (consumed >= m_len)
	{
		blast_dbg("Could not find start byte in %d bytes", (int)m_len);
		return m_len;
	}

	if (m_len - consumed < sizeof(blast_master_packet_t))
	{
		blast_dbg("Waiting for full header packet");
		return consumed;
	}

	header = (blast_master_packet_t*)&packet_string[consumed];
	if (sizeof(blast_master_packet_t) + header->length > m_len - consumed)
	{
		blast_dbg("Partially filled packet received (%zu bytes).  Waiting for full packet (%u bytes)", m_len, header->length);
		return consumed;
	}

	if (header->version != 1)
	{
		blast_err("Unknown version %d.  Discarding packet", (int)header->version);
	}

	if (BLAST_MASTER_PACKET_CRC(header) != crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(header), header->length))
	{
		blast_err("Invalid CRC.  Discarding packet");
		return 1;
	}

	/**
	 * Process data in packet
	 */
	switch (header->type)
	{
		case type_frame_data:
			sharing_packet = (data_sharing_t*)BLAST_MASTER_PACKET_PAYLOAD(header);
			memcpy(&received_data, sharing_packet, sizeof(received_data));
			break;
		case type_command_data:
			if (header->length == sizeof(struct CommandDataStruct))
				memcpy(&CommandData, BLAST_MASTER_PACKET_PAYLOAD(header), sizeof(CommandData));
			else if (header->length == 0)
				data_sharing_send_commanddata();
			break;
		case type_ethercat_data:
		    break;
	}

	consumed += BLAST_MASTER_PACKET_FULL_LENGTH(header);

	return consumed;
}

/**
 * Handles the network command hangup.
 * @param m_data Pointer to the data queue
 * @param m_len length in bytes of the data queue
 * @param m_userdata socket pointer
 * @return Number of bytes processed (that should be removed from the queue)
 */
static int data_sharing_net_cleanup(const void *m_data, size_t m_len, void *m_userdata )
{
	comms_socket_t *socket = (comms_socket_t*)m_userdata;
	size_t consumed = 0;

	if (m_len)
		consumed = data_sharing_process_packet(m_data, m_len, m_userdata);

	if (consumed < m_len) blast_err("Did not receive full packet from %s", socket->host);

	return consumed;
}

/**
 * Currently unused function to process errors received on the Network socket
 * @param m_code
 * @param m_userdata
 */
static void data_sharing_net_error(int m_code, void *m_userdata)
{
	comms_socket_t *socket = (comms_socket_t*)m_userdata;
	blast_err("Got error %d on %s", m_code, socket->host);
	return;
}
