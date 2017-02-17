/*
 * roach.c
 *
 * This software is copyright (C) 2013-2016 University of Pennsylvania
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 *  Created on: July 14, 2015
 *      Author: sam + laura + seth
 */

#include "roach.h"
#include <complex.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <fftw3.h>
// include "portable_endian.h"
#include "mcp.h"
#include "katcp.h"
#include "katcl.h"
#include "blast.h"
#include "crc.h"
#include "netc.h"
#include "qdr.h"
#include "remote_serial.h"
#include "valon.h"
// #include "command_struct.h"
#undef I
#include "phenom/defs.h"
#include "phenom/buffer.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"
#include "phenom/string.h"

typedef void (*roach_callback_t)(uint8_t*, size_t);

// TODO(laura/sam): This should be changed to a read checksum once Sam and Adrian
// incorporate a checksum into the roach packets.
#define ROACH_CHECKSUM 42

// number of roach channels that will be published to the server
const uint16_t n_publish_roaches[5] = {1016, 64, 0, 0, 0};

uint16_t check_udp_packet(data_udp_packet_t* m_packet, roach_handle_data_t* m_roach_udp)
{
    uint16_t retval = 0;
    if (m_packet->packet_count != (m_roach_udp->seq_number + 1)) {
        blast_warn("roach%i: Packet sequence number is %i. Last sequence number was =%i!",
                  m_roach_udp->which, m_packet->packet_count, m_roach_udp->seq_number);
        m_roach_udp->seq_error_count++;
	    retval |= ROACH_UDP_SEQ_ERR;
    }
    if (m_packet->checksum != ROACH_CHECKSUM) {
        blast_err("roach%i: checksum =%i failed!", m_roach_udp->which, m_packet->checksum);
        m_roach_udp->crc_error_count++;
	    retval |= ROACH_UDP_CRC_ERR;
    }
    m_roach_udp->seq_number = m_packet->packet_count;

    if (retval > 0) {
        m_roach_udp->roach_invalid_packet_count++;
        m_roach_udp->roach_packet_count++;
    } else {
        m_roach_udp->roach_valid_packet_count++;
        m_roach_udp->roach_packet_count++;
    }
    return retval;
}

void parse_udp_packet(data_udp_packet_t* m_packet)
{
	static uint64_t i_packet = 0;

// Get a pointer to the start of the raw buffer.
    uint8_t* buf = ph_buf_mem(m_packet->rcv_buffer);

// Note that we don't get ethernet or IP packet header info if we use SOCK_DGRAM.
// TODO(laura): is there a way to recover this information using libphenom?
// Might be useful for error checking.

//  	m_packet->eth  = (struct ethhdr *)(buf);
// 	uint16_t iphdrlen;
// 	struct sockaddr_in source, dest;
// 	m_packet->ip = (struct iphdr *)(buf + sizeof(struct ethhdr));
// 	memset(&source, 0, sizeof(source));
// 	source.sin_addr.s_addr = m_packet->ip->saddr;
// 	memset(&dest, 0, sizeof(dest));
// 	dest.sin_addr.s_addr = m_packet->ip->daddr;

	/* if ((i_packet % 6000) == 0) {
	    for (int i = 8160; i < ROACH_UDP_DATA_LEN; i = i + 8) blast_info("%i: %.2x%.2x %.2x%.2x %.2x%.2x %.2x%.2x",
	    i, buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
    	} */

	m_packet->checksum = (buf[8176] << 24) | (buf[8177] << 16) | (buf[8178] << 8) | buf[8179];
	m_packet->pps_count = (buf[8180] << 24) | (buf[8181] << 16) | (buf[8182] << 8) | buf[8183];
	m_packet->clock_count = (buf[8184] << 24) | (buf[8185] << 16) | (buf[8186] << 8) | buf[8187];
	m_packet->packet_count = (buf[8188] << 24) | (buf[8189] << 16) | (buf[8190] << 8) | buf[8191];
	// I, Q
	for (int i = 0;	i < 1016; i += 1) {
		int j;
		int k;
		if ((i % 2) == 0) {
			j = ((i*4) /2);
			k = 512*4 + ((i*4) /2);
		} else {
			j = 1024*4 + (((i*4) - 1) / 2) - 1;
			k = 1536*4 + (((i*4) - 1) / 2) - 1;
		}
		m_packet->Ival[i] = (float)(int32_t)(ntohl((buf[j] << 24) |
					(buf[j + 1] << 16) | (buf[j + 2] << 8) | (buf[j + 3])));
		m_packet->Qval[i] = (float)(int32_t)(ntohl((buf[k] << 24) |
					(buf[k + 1] << 16) | (buf[k + 2] << 8) | (buf[k + 3])));
        /*if ((i == 0) && (i_packet % 60) == 0) {
             blast_info("i = %i, I = %f, Q = %f", i, m_packet->Ival[i], m_packet->Qval[i]);
        }*/
	}
	/* if ((i_packet % 6000) == 0) {
        blast_info("checksum = %i, pps_count = %i, clock_count = % i, packet_count = %i",
                m_packet->checksum, m_packet->pps_count, m_packet->clock_count, m_packet->packet_count);
	} */
	i_packet++;
}


void udp_store_to_structure(data_udp_packet_t* m_packet, roach_handle_data_t* m_roach_udp)
{
    static uint64_t packet_count = 0;
    data_udp_packet_t* local_packet;

    if (m_roach_udp->first_packet) {
        // blast_info("checksum = %i, pps_count = %i, clock_count = % i, packet_count = %i",
        //  m_packet->checksum, m_packet->pps_count, m_packet->clock_count, m_packet->packet_count);
    	for (int i = 0; i < 3; i++) {
            local_packet = &m_roach_udp->last_pkts[i];
    	    local_packet = balloc(fatal, sizeof(*m_packet) + m_packet->buffer_len);
        }
        // blast_info("roach%i: Allocated packet structures of size %lu", m_roach_udp->which, sizeof(*m_packet));
        m_roach_udp->first_packet = FALSE;
    }
    /* if (packet_count < 100) {
        blast_info("roach%i: Write index = %i", m_roach_udp->which, m_roach_udp->index);
    }*/
    memcpy(&(m_roach_udp->last_pkts[m_roach_udp->index]), m_packet, sizeof(*m_packet));
    m_roach_udp->index = INC_INDEX(m_roach_udp->index);
    /* if (packet_count < 100) {
        blast_info("roach%i: After incrementing write index = %i", m_roach_udp->which, m_roach_udp->index);
        blast_info("roach%i: last_packet: checksum = %i, pps_count = %i, clock_count = % i, packet_count = %i ",
        m_roach_udp->which, m_packet->checksum, m_packet->pps_count, m_packet->clock_count, m_packet->packet_count);
    }*/
    packet_count++;
}
/**
 * Called every time we receive a roach udp packet.
 *
 */
static void roach_process_stream(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    roach_handle_data_t *m_roach_udp = (roach_handle_data_t*) m_data;
// First check for errors...
    if (m_why & PH_IOMASK_ERR) {
        if (!m_roach_udp->have_warned) {
            blast_err("roach%i: IO error. ", m_roach_udp->which);
            m_roach_udp->have_warned = 1;
        }
    	return;
    } else if (m_why & PH_IOMASK_TIME) {
        if (!m_roach_udp->have_warned) {
            blast_err("roach%i: Timeout. ", m_roach_udp->which);
            m_roach_udp->have_warned = 1;
        }
    	return;
    } else if (m_why & PH_IOMASK_WAKEUP) {
         if (!m_roach_udp->have_warned) {
            blast_err("roach%i: Triggered by ph_job_wakeup. ", m_roach_udp->which);
            m_roach_udp->have_warned = 1;
        }
    	return;
    } else if (m_why & PH_IOMASK_NONE) {
        if (!m_roach_udp->have_warned) {
            blast_info("roach%i: NBIO is disabled or not applicable! ", m_roach_udp->which);
            m_roach_udp->have_warned = 1;
        }
    	return;
    }
    if (m_why & PH_IOMASK_WRITE) {
        if (!m_roach_udp->have_warned) {
            blast_info("roach%i: Write flag set! ", m_roach_udp->which);
            m_roach_udp->have_warned = 1;
        }
    }
    if (!(m_why & PH_IOMASK_READ)) {
        if (!m_roach_udp->have_warned) {
            blast_err("roach%i: Triggered but not for reading. m_why = %i. ", m_roach_udp->which, m_why);
            m_roach_udp->have_warned = 1;
        }
        return;
    }

// At this point we know that a read callback was triggered.

// Now read from the buffer
//    data_packet_t m_packet;
    data_udp_packet_t m_packet;

    m_packet.rcv_buffer = ph_sock_read_bytes_exact(m_roach_udp->udp_socket, ROACH_UDP_DATA_LEN);

    uint64_t bytes_read = ph_buf_len(m_packet.rcv_buffer);
    if (bytes_read < ROACH_UDP_DATA_LEN) {
    	blast_err("roach%i: Read only %lu bytes.", m_roach_udp->which, bytes_read);
        m_roach_udp->roach_invalid_packet_count++;
        m_roach_udp->roach_packet_count++;
        return;
    }
    m_packet.buffer_len = bytes_read;
    parse_udp_packet(&m_packet);

    uint16_t udperr = check_udp_packet(&m_packet, m_roach_udp);

// Store udp packets to the harddrive even if there were errors in the packet format.
//    if (CommandData.udp_roach[m_roach_udp->i_which].store_udp) {
// This is where we will write the packets to the flight harddisks.
// TODO(laura): Write harddrive storage function.
//    }

    if (udperr > 0) return;

// The next set of commands are only executed if there were no packet errors.
//    if (CommandData.udp_roach[m_roach_udp->i_which].publish_udp) {
        udp_store_to_structure(&m_packet, m_roach_udp);
//    }

//    if (CommandData.udp_roach[m_roach_udp->i_which].publish_udp) {
//    }

    m_roach_udp->have_warned = 0;
}
/**
 * Initialize the roach udp packet routine.  The state variable tracks each
 * udp socket and is passed to the connect job.
 *
 * @param m_which
 */

void roach_udp_networking_init(int m_which, roach_state_t* m_roach_state, size_t m_numchannels)
{
    ph_sockaddr_t addr;
    uint32_t origaddr;
	ph_result_t test = 0; // Used to test the status of some phenom calls. 0 = OK
    ph_string_t sockaddr_str;

    roach_handle_data_t *m_roach_udp = (roach_handle_data_t*)&roach_udp[m_which-1];

    // Initialize counts
    m_roach_udp->roach_invalid_packet_count = 0;
    m_roach_udp->roach_packet_count = 0;
    m_roach_udp->seq_error_count = 0;
    m_roach_udp->crc_error_count = 0;

    m_roach_udp->index = 0; // Write index for the udp circular buffer.
    m_roach_udp->which = m_which;
    m_roach_udp->i_which = m_which-1;
    m_roach_udp->first_packet = TRUE;

    snprintf(m_roach_udp->address, sizeof(m_roach_udp->address), "roach%i-udp", m_which);
	m_roach_udp->port = m_roach_state->dest_port;

	blast_info("roach%i: Configuring roach information corresponding to %s", m_which, m_roach_udp->address);
    struct hostent *udp_origin = gethostbyname(m_roach_udp->address);
    origaddr = *(uint32_t*)(udp_origin->h_addr_list[0]);
    snprintf(m_roach_udp->ip, sizeof(m_roach_udp->ip), "%d.%d.%d.%d",
            (origaddr & 0xff), ((origaddr >> 8) & 0xff),
            ((origaddr >> 16) & 0xff), ((origaddr >> 24) & 0xff));
    blast_info("Expecting UDP packets for ROACH%d from IP %s on port %i", m_which,
    m_roach_udp->ip, m_roach_udp->port);

    blast_info("Initializing ROACH UDP packet reading.");

    m_roach_udp->opened = false;
    m_roach_udp->have_warned = false;

//    struct hostent *udp_ent = gethostbyaddr(udp_dest, sizeof(udp_dest), AF_INET6);
    struct hostent *udp_ent = gethostbyname(udp_dest_name);
    if (!udp_ent) {
        blast_err("roach%i: Could not resolve broadcast IP %s!", m_which, udp_dest_name);
        return;
    }
    uint32_t destaddr;
    char destaddr_char[32];
    destaddr = *(uint32_t*)(udp_ent->h_addr_list[0]);

    snprintf(destaddr_char, sizeof(destaddr_char), "%d.%d.%d.%d",
                 (destaddr & 0xff), ((destaddr >> 8) & 0xff),
                 ((destaddr >> 16) & 0xff), ((destaddr >> 24) & 0xff));
    blast_info("Will listen on address %s corresponds to IP %s", udp_dest_name, destaddr_char);

	test = ph_sockaddr_set_from_hostent(&addr, udp_ent);
    if (test) {
    	blast_err("roach%i: Could not read hostent for UDP socket! Error code = %u. %s",
    	          m_which, test, strerror(errno));
    }

    // test = ph_sockaddr_print(&addr, &sockaddr_str, TRUE);
	// if (test) {
	// 	blast_info("roach%i: sockaddr string is %s", m_which, sockaddr_str.buf);
	// } else {
	//    blast_err("roach%i: Could not parse sockaddr structure. %s", m_which, strerror(errno));
	// }

	ph_sockaddr_set_port(&addr, m_roach_udp->port);

    // Open the unix socket file descriptor
    ph_socket_t sock = ph_socket_for_addr(&addr, SOCK_DGRAM, PH_SOCK_NONBLOCK);
    m_roach_udp->udp_socket_fd = sock;
    blast_info("roach%d: Socket file descriptor is %i", m_which, (int) sock);
    if (!sock) blast_err("roach%d: Failed to open Socket. %s", errno);

    // Allocate a phenom socket pointer
	m_roach_udp->udp_socket = ph_sock_new_from_socket(sock, &addr, NULL);

    m_roach_udp->udp_socket->callback = roach_process_stream;
    m_roach_udp->udp_socket->job.data = m_roach_udp;

    m_roach_udp->opened = 1;

    ph_sock_enable(m_roach_udp->udp_socket, TRUE);
}

void write_roach_channels_488hz(void)
{
    uint8_t i_udp_read;
	static channel_t *RoachQAddr[NUM_ROACHES][NUM_ROACH_UDP_CHANNELS];
	static channel_t *RoachIAddr[NUM_ROACHES][NUM_ROACH_UDP_CHANNELS];
    char channel_name_i[128] = {0};
    char channel_name_q[128] = {0};
	int i, j;
    static int firsttime = 1;

    if (firsttime) {
        blast_info("Starting write_roach_channels_488hz");
        firsttime = 0;
        for (i = 0; i < 1; i++) { // Only write ROACH1 channels for now.
//        for (i = 0; i < NUM_ROACHES; i++) {
            for (j = 0; j < n_publish_roaches[i]; j++) {
                snprintf(channel_name_i, sizeof(channel_name_i), "roach%d_kid%04d_i", i+1, j);
                RoachIAddr[i][j] = channels_find_by_name(channel_name_i);
                snprintf(channel_name_q, sizeof(channel_name_q), "roach%d_kid%04d_q", i+1, j);
                RoachQAddr[i][j] = channels_find_by_name(channel_name_q);
            }
        }
    }
//    for (i = 0; i < NUM_ROACHES; i++) {
    for (i = 0; i < 1; i++) {
        i_udp_read = GETREADINDEX(roach_udp[i].index);
        data_udp_packet_t* m_packet = &(roach_udp[i].last_pkts[i_udp_read]);
        for (j = 0; j < n_publish_roaches[i]; j++) {
            SET_FLOAT(RoachIAddr[i][j], m_packet->Ival[j]);
            SET_FLOAT(RoachQAddr[i][j], m_packet->Qval[j]);
//            if ((j == 0) && ((roach_udp[i].roach_packet_count % 500) == 0)) {
//                blast_info("Ival = %f, Qval = %f", m_packet->Ival[j], m_packet->Qval[j]);
//            }
        }
    }
}
