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
 *  Last edited: July 15, 2017
 *      Author: sam + laura + seth
 */

#include "roach.h"
#include <complex.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/poll.h>
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
#include <linux/filter.h>
#include <net/if.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <fftw3.h>
#include <pthread.h>
// include "portable_endian.h"
#include "mcp.h"
#include "katcp.h"
#include "katcl.h"
#include "blast.h"
#include "crc.h"
#include "netc.h"
#include "qdr.h"
#include "remote_serial.h"
#include "store_data.h"
#include "valon.h"
// #include "command_struct.h"
#undef I

#define ROACH_CHECKSUM 42
#define HEADER_LEN 42
/* Number of Roach channels for each module that will be published to the server */
const uint16_t n_publish_roaches[5] = {1016, 1016, 1016, 1016, 1016};
/* The shared UDP socket file descriptor */
extern int roach_sock_fd;

/* Function: check_udp_packets
 * ----------------------------
 * Verifies checksum and count of received packet
 * Throws error if either one is incorrect
 *
 * @param m_packet UDP packet structure
 * @param m_roach_udp Roach UDP structure
 */
uint16_t check_udp_packet(data_udp_packet_t* m_packet, roach_handle_data_t* m_roach_udp)
{
    // blast_info("R%d packet count = %d", m_roach_udp->which, m_packet->packet_count);
    uint16_t retval = 0;
    if (m_packet->packet_count != (m_roach_udp->seq_number + 1)) {
        blast_warn("roach%i: Packet sequence number is %i. Last sequence number was =%i!",
                  m_roach_udp->which, m_packet->packet_count, m_roach_udp->seq_number);
        m_roach_udp->seq_error_count++;
	    retval |= ROACH_UDP_SEQ_ERR;
    }
    if (m_packet->checksum != ROACH_CHECKSUM) {
        blast_err("roach%i: checksum = %i failed!", m_roach_udp->which, m_packet->checksum);
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

/* Function: filter_udp
 * -------------------------
 * Filters packets of type UDP
 *
 * @param m_sock socket file descriptor
 */
void filter_udp(int m_sock)
{
    struct sock_fprog prog;
    struct sock_filter filter[] = {
        { BPF_LD + BPF_H + BPF_ABS,  0, 0,     12 },
        { BPF_JMP + BPF_JEQ + BPF_K, 0, 1,  0x800 },
        { BPF_LD + BPF_B + BPF_ABS,  0, 0,     23 },
        { BPF_JMP + BPF_JEQ + BPF_K, 0, 1,   0x11 },
        { BPF_RET + BPF_K,           0, 0, 0xffff },
        { BPF_RET + BPF_K,           0, 0, 0x0000 },
    };
    prog.len = sizeof(filter)/sizeof(filter[0]);
    prog.filter = filter;
    if (setsockopt(m_sock, SOL_SOCKET, SO_ATTACH_FILTER, &prog, sizeof(prog)) < 0) {
        blast_err("Socket filter error\n");
    }
}

/* Function init_roach_socket
 * -------------------------------
 * Creates RAW socket, binds it to chosen eth interface and applies UDP filter
 */
int init_roach_socket(void)
{
    int retval = -1;
    char *eth_iface;
    if ((roach_sock_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_IP))) < 0) {
        blast_err("Cannot create socket\n");
    }
    // Bind our raw socket to this interface
    eth_iface = "eth0";
    if (setsockopt(roach_sock_fd, SOL_SOCKET, SO_BINDTODEVICE, eth_iface, 4) < 0) {
        blast_err("setsockopt failed:\n");
    } else { retval = 1;}
    // Filter only UDP packets
    filter_udp(roach_sock_fd);
    return retval;
}

/* Function: parse_udp_packet
 * --------------------------
 * Parses packet data (I, Q values) and timing information
 * Stores result to packet structure.
 *
 * @param m_packet UDP packet structure
 */
void parse_udp_packet(data_udp_packet_t* m_packet)
{
    // static uint64_t i_packet = 0;
    uint8_t *payload = (uint8_t *)(m_packet->rcv_buffer);
    uint8_t *buf = (payload + HEADER_LEN);
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
        /* if ((i == 0) && (i_packet % 60) == 0) {
        blast_info("i = %i, I = %f, Q = %f, checksum = %u, count = %u", i, m_packet->Ival[i], m_packet->Qval[i],
        m_packet->checksum, m_packet->packet_count);
        } */
    }
    // i_packet++;
}

/* Function: udp_store_to_structure
 * -----------------------------------
 * Cycles current data packet through a circular buffer (last_pkts)
 *
 * @param m_roach_udp roach UDP structure
 * @param m_packet UDP packet structure
 * @param
 *
 */
void udp_store_to_structure(roach_handle_data_t* m_roach_udp, data_udp_packet_t* m_packet)
{
    static uint64_t packet_count = 0;
    data_udp_packet_t* local_packet;
    if (m_roach_udp->first_packet) {
        // blast_info("checksum = %i, pps_count = %i, clock_count = % i, packet_count = %i",
        // m_packet->checksum, m_packet->pps_count, m_packet->clock_count, m_packet->packet_count);
    	for (int i = 0; i < 3; i++) {
            local_packet = &m_roach_udp->last_pkts[i];
    	    local_packet = balloc(fatal, sizeof(*m_packet) + ROACH_UDP_LEN);
        }
        blast_info("roach%i: Allocated packet structures of size %lu", m_roach_udp->which, sizeof(*m_packet));
        m_roach_udp->first_packet = FALSE;
    }
    /* if (packet_count < 100) {
        blast_info("roach%i: Write index = %i", m_roach_udp->which, m_roach_udp->index);
    } */
    memcpy(&(m_roach_udp->last_pkts[m_roach_udp->index]), m_packet, sizeof(*m_packet));
    m_roach_udp->index = INC_INDEX(m_roach_udp->index);
    // blast_info("ROACH UDP IDX = %d", m_roach_udp->index);
    packet_count++;
}

/* Function: roach_process_stream
 * ---------------------------------
 * Called every time packet is received
 * Parses the packet data, checks its validity and stores to structure
 *
 * @param m_roach_udp roach UDP structure
 * @param m_packet UDP packet structure
 */
void roach_process_stream(roach_handle_data_t *m_roach_udp, data_udp_packet_t *m_packet)
{
    parse_udp_packet(m_packet);
    uint16_t udperr = check_udp_packet(m_packet, m_roach_udp);
    // store_roach_udp_packet(&m_packet, m_roach_udp); // Writes packet to harddrive.
    if (udperr > 0) {
        blast_info("UDPERR = %d", udperr);
        // continue;
        udp_store_to_structure(m_roach_udp, m_packet);
        m_roach_udp->have_warned = 0;
    }
}

/* Function: poll_socket
 * ------------------------
 * Poll the raw socket for data. When data is received, call roach_process_stream
 */
void poll_socket(void)
{
    init_roach_socket();
    blast_info("Roach socket file descriptor is %i", roach_sock_fd);
    if (!roach_sock_fd) {
       blast_err("Failed to open Socket");
    } else {
        blast_info("Roach socket file descriptor is %i", roach_sock_fd);
    }
    blast_info("Creating poll thread...");
    int rv;
    struct pollfd ufds[1];
    ufds[0].fd = roach_sock_fd;
    ufds[0].events = POLLIN; // Check for data on socket
    rv = poll(ufds, 1, 0.002); // Wait for event, 2 ns timeout
    while (1) {
        usleep(10);
        data_udp_packet_t m_packet;
        rv = poll(ufds, 1, 0.002); // Wait for event, 2 ns timeout
        if (rv == -1) {
            blast_err("Roach socket poll error");
        } else {
	    // blast_info("I am waiting for a poll event...");
            if (ufds[0].revents & POLLIN) { // check for events on socket
                uint64_t bytes_read = recv(roach_sock_fd, m_packet.rcv_buffer,
                ROACH_UDP_LEN, 0);
                m_packet.udp_header = (struct udphdr *)(m_packet.rcv_buffer
			+ sizeof(struct ethhdr)
			+ sizeof(struct iphdr));
                /* Filter destination address */
		// blast_info("Before filt: R%d\t%d\t%d", m_roach_udp->which,
                          // m_roach_udp->port, ntohs(m_packet.udp_header->dest));
                for (int ind = 0; ind < NUM_ROACHES; ind++) {
                    roach_handle_data_t *m_roach_udp = (roach_handle_data_t*)&roach_udp[ind];
                    // blast_info("Roach udp %d, port = %d", m_roach_udp->which, m_roach_udp->port);
                    if ((m_roach_udp->port != ntohs(m_packet.udp_header->dest))) {
                        continue;
                    } else if (bytes_read < ROACH_UDP_DATA_LEN) {
                        blast_err("Roach%i: Read only %lu bytes.", m_roach_udp->which, bytes_read);
                        m_roach_udp->roach_invalid_packet_count++;
                        m_roach_udp->roach_packet_count++;
                        continue;
                        // blast_info("%d: These should match: %d\t%d", m_roach_udp->which,
                         // m_roach_udp->port, ntohs(m_packet.udp_header->dest));}
                    } else { roach_process_stream(m_roach_udp, &m_packet);}
                }
           }
       }
    }
}

/* Function: roach_udp_networking_init
 * ------------------------------------
 * The UDP data thread. Called in mcp.c
 */
void roach_udp_networking_init(void)
{
    pthread_t poll_thread;
    uint32_t origaddr;
    for (int ind = 0; ind < NUM_ROACHES; ind++) {
        roach_handle_data_t *m_roach_udp = (roach_handle_data_t*)&roach_udp[ind];

        // Initialize counts
        m_roach_udp->roach_invalid_packet_count = 0;
        m_roach_udp->roach_packet_count = 0;
        m_roach_udp->seq_error_count = 0;
        m_roach_udp->crc_error_count = 0;

        m_roach_udp->index = 0; // Write index for the udp circular buffer.
        m_roach_udp->which = ind + 1;
        m_roach_udp->i_which = ind;
        m_roach_udp->first_packet = TRUE;

        snprintf(m_roach_udp->address, sizeof(m_roach_udp->address), "roach%i-udp", m_roach_udp->which);
        m_roach_udp->port = 64000 + ind;

        blast_info("roach%i: Configuring roach information corresponding to %s",
            m_roach_udp->which, m_roach_udp->address);
        struct hostent *udp_origin = gethostbyname(m_roach_udp->address);
        origaddr = *(uint32_t*)(udp_origin->h_addr_list[0]);
        snprintf(m_roach_udp->ip, sizeof(m_roach_udp->ip), "%d.%d.%d.%d",
            (origaddr & 0xff), ((origaddr >> 8) & 0xff),
            ((origaddr >> 16) & 0xff), ((origaddr >> 24) & 0xff));
        blast_info("Expecting UDP packets for ROACH%d from IP %s on port %i", m_roach_udp->which,
            m_roach_udp->ip, m_roach_udp->port);

        blast_info("Initializing ROACH UDP packet reading.");

        m_roach_udp->opened = 1;
        m_roach_udp->have_warned = false;

        struct hostent *udp_ent = gethostbyname(udp_dest_name);
        if (!udp_ent) {
            blast_err("roach%i: Could not resolve broadcast IP %s!", m_roach_udp->which, udp_dest_name);
            return;
        }
        uint32_t destaddr;
        char destaddr_char[32];
        destaddr = *(uint32_t*)(udp_ent->h_addr_list[0]);

        snprintf(destaddr_char, sizeof(destaddr_char), "%d.%d.%d.%d",
                 (destaddr & 0xff), ((destaddr >> 8) & 0xff),
                 ((destaddr >> 16) & 0xff), ((destaddr >> 24) & 0xff));
        blast_info("Will listen on address %s corresponds to IP %s", udp_dest_name, destaddr_char);
    }
    blast_info("Creating poll thread...");
    if (pthread_create(&poll_thread, NULL, (void*)&poll_socket, NULL)) {
        blast_err("Error creating recv_thread");
    }
}

/* Function: write_roach_channels_488hz
 * ------------------------------------
 * Populates 488 Hz frame fields
 */
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
	for (i = 0; i < NUM_ROACHES; i++) {
            for (j = 0; j < n_publish_roaches[i]; j++) {
                snprintf(channel_name_i, sizeof(channel_name_i), "i_kid%04d_roach%d", j, i+1);
                RoachIAddr[i][j] = channels_find_by_name(channel_name_i);
                snprintf(channel_name_q, sizeof(channel_name_q), "q_kid%04d_roach%d", j, i+1);
                RoachQAddr[i][j] = channels_find_by_name(channel_name_q);
            }
        }
    }
    for (i = 0; i < NUM_ROACHES; i++) {
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
