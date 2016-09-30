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
#undef I
#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"
#include "phenom/string.h"

typedef void (*roach_callback_t)(uint8_t*, size_t);

typedef struct {
    int roach;
    bool            opened;
    bool            have_warned;
    bool            want_reset;
    uint8_t         which;
    uint8_t         seq_error_count;
    uint8_t         crc_error_count;
    uint8_t         seq_number;
    uint16_t        num_channels;
    uint16_t        roach_invalid_packet_count;
    uint32_t        roach_packet_count;
    uint32_t        roach_valid_packet_count;
    uint8_t         index;
    uint16_t        port;
    char            address[16];
    char            listen_ip[16];
    char            ip[16];
    udp_packet_t    last_pkt;
    ph_sock_t *udp_socket;
    roach_callback_t process_data;
    struct timeval  timeout;
} roach_handle_data_t;

roach_handle_data_t roach_udp[NUM_ROACHES];


/**
 * Called every time we receive a roach udp packet.
 *
 */
static void roach_process_stream(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    roach_handle_data_t *roach_udp = (roach_handle_data_t*) m_data;
    blast_info("roach%i: roach_process_stream called!", roach_udp->which+1);
    if (m_why & PH_IOMASK_ERR) {
        if (!roach_udp->have_warned) {
            blast_err("roach%i: IO error. ", roach_udp->which+1);
            roach_udp->have_warned = 1;
        }
    	return;
    }
    if (m_why & PH_IOMASK_TIME) {
        if (!roach_udp->have_warned) {
            blast_err("roach%i: Timeout. ", roach_udp->which+1);
            roach_udp->have_warned = 1;
        }
    	return;
    }
    if (m_why & PH_IOMASK_WAKEUP) {
        if (!roach_udp->have_warned) {
            blast_err("roach%i: Triggered by ph_job_wakeup. ", roach_udp->which+1);
            roach_udp->have_warned = 1;
        }
    	return;
    }
    if (m_why & PH_IOMASK_READ) {
//        blast_info("roach%i: There is data to be read! ", roach_udp->which+1);
    }
    if (m_why & PH_IOMASK_NONE) {
        if (!roach_udp->have_warned) {
            blast_info("roach%i: NBIO is disabled or not applicable! ", roach_udp->which+1);
            roach_udp->have_warned = 1;
        }
    }
    if (m_why & PH_IOMASK_WRITE) {
        if (!roach_udp->have_warned) {
            blast_info("roach%i: Write flag set! ", roach_udp->which+1);
            roach_udp->have_warned = 1;
        }
    }
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

    roach_handle_data_t *m_roach_udp = (roach_handle_data_t*)&roach_udp[m_which];
    m_roach_udp->which = m_which;

    snprintf(m_roach_udp->address, sizeof(m_roach_udp->address), "roach%i-udp", m_which+1);
	m_roach_udp->port = m_roach_state->dest_port;

	blast_info("roach%i: Configuring roach information corresponding to %s", m_which+1, m_roach_udp->address);
    struct hostent *udp_origin = gethostbyname(m_roach_udp->address);
    origaddr = *(uint32_t*)(udp_origin->h_addr_list[0]);
    snprintf(m_roach_udp->ip, sizeof(m_roach_udp->ip), "%d.%d.%d.%d",
            (origaddr & 0xff), ((origaddr >> 8) & 0xff),
            ((origaddr >> 16) & 0xff), ((origaddr >> 24) & 0xff));
    blast_info("Expecting UDP packets for ROACH%d from IP %s on port %i", m_which+1,
    m_roach_udp->ip, m_roach_udp->port);

    blast_info("Initializing ROACH UDP packet reading.");

    m_roach_udp->opened = false;
    m_roach_udp->have_warned = false;

//    struct hostent *udp_ent = gethostbyaddr(udp_dest, sizeof(udp_dest), AF_INET6);
    struct hostent *udp_ent = gethostbyname(udp_dest_name);
    if (!udp_ent) {
        blast_err("roach%i: Could not resolve broadcast IP %s!", m_which + 1, udp_dest_name);
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
    	          m_which + 1, test, strerror(errno));
    }

    // test = ph_sockaddr_print(&addr, &sockaddr_str, TRUE);
	// if (test) {
	// 	blast_info("roach%i: sockaddr string is %s", m_which+1, sockaddr_str.buf);
	// } else {
	//    blast_err("roach%i: Could not parse sockaddr structure. %s", m_which+1, strerror(errno));
	// }

	ph_sockaddr_set_port(&addr, m_roach_udp->port);

    // Open the unix socket file descriptor
    ph_socket_t sock = ph_socket_for_addr(&addr, SOCK_DGRAM, PH_SOCK_NONBLOCK);
    blast_info("roach%d: Socket file descriptor is %i", m_which+1, (int) sock);
    if (!sock) blast_err("roach%d: Failed to open Socket. %s", errno);

    // Allocate a phenom socket pointer
	m_roach_udp->udp_socket = ph_sock_new_from_socket(sock, &addr, NULL);

    m_roach_udp->udp_socket->callback = roach_process_stream;
    m_roach_udp->udp_socket->job.data = m_roach_udp;

    m_roach_udp->opened = 1;

    ph_sock_enable(m_roach_udp->udp_socket, TRUE);
}
