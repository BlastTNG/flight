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

#include <phenom/listener.h>
#include <phenom/socket.h>
#include <phenom/buffer.h>

#include <stdint.h>
#include <stdbool.h>

#include <command_struct.h>

#include <blast.h>
#include <blast_comms.h>
#include <blast_packet_format.h>
#include <crc.h>

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

static ph_listener_t *lstn;


static void data_sharing_process_packet(ph_sock_t *sock, ph_iomask_t why, void *arg)
{
    ph_buf_t *pkt_buf;
    blast_master_packet_t *packet;
    size_t packet_len;
    char delim[2] = {0};

    if (why & (PH_IOMASK_ERR | PH_IOMASK_TIME)) {
        ph_sock_shutdown(sock, PH_SOCK_SHUT_RDWR);
        ph_sock_free(sock);
        return;
    }

    delim[0] = BLAST_MAGIC8;
    if (!ph_bufq_discard_until(sock->rbuf, delim, 1, 0)) return;

    if (!(pkt_buf = ph_bufq_peek_bytes(sock->rbuf, sizeof(blast_master_packet_t)))) return;
    packet = (blast_master_packet_t*)ph_buf_mem(pkt_buf);
    packet_len = BLAST_MASTER_PACKET_FULL_LENGTH(packet);

    if (ph_bufq_len(sock->rbuf) < packet_len) return;
    ph_buf_delref(pkt_buf);

    /**
     * Now, read the full packet and remove from the socket queue
     */
    pkt_buf = ph_sock_read_bytes_exact(sock, packet_len);
    packet = (blast_master_packet_t*)ph_buf_mem(pkt_buf);
    if (packet->version != 1) {
        blast_err("Unknown version %d.  Discarding packet", (int)packet->version);
        goto packet_err;
    }

    if (BLAST_MASTER_PACKET_CRC(packet) != crc32(BLAST_MAGIC32, BLAST_MASTER_PACKET_PAYLOAD(packet), packet->length)) {
        blast_err("Invalid CRC.  Discarding packet");
        goto packet_err;
    }

    /**
     * Process data in packet
     */
    switch (packet->type) {
        case type_frame_data:
            // TODO(seth): implement ring buffer for frame_data exchange
            memcpy(&received_data, BLAST_MASTER_PACKET_PAYLOAD(packet), sizeof(received_data));
            break;
        case type_command_data:
            // TODO(seth): implement queue for receiving CommandDataStruct
            if (packet->length == sizeof(struct CommandDataStruct))
                memcpy(&CommandData, BLAST_MASTER_PACKET_PAYLOAD(packet), sizeof(CommandData));
            else if (packet->length == 0)
                data_sharing_send_commanddata(sock);
            break;
        case type_ethercat_data:
            break;
    }

packet_err:
    ph_buf_delref(pkt_buf);
}

// Called each time the listener has accepted a client connection
static void acceptor(ph_listener_t *lstn, ph_sock_t *sock)
{
  ph_unused_parameter(lstn);

  // Tell it how to dispatch
  sock->callback = data_sharing_process_packet;

  ph_sock_enable(sock, true);
}

void initialize_data_sharing(void)
{
    ph_sockaddr_t addr;
    struct addrinfo request = { .ai_protocol = IPPROTO_UDP,
                                .ai_socktype = SOCK_DGRAM };
    struct addrinfo *result_ai;
    getaddrinfo("224.0.0.0", "1300", &request, &result_ai);

    ph_sockaddr_set_from_addrinfo(&addr, result_ai);
    freeaddrinfo(result_ai);

    lstn = ph_listener_new("data-share", acceptor);
    ph_listener_bind(lstn, &addr);
    ph_listener_enable(lstn, true);
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

void data_sharing_send_commanddata(ph_sock_t *m_sock)
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
    ph_stm_write(m_sock->stream, &packet, sizeof(packet), NULL);
}

