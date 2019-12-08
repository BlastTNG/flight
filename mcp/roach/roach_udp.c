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
#include <linklist.h>

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
// const uint16_t n_publish_roaches[5] = {1016, 1016, 1016, 1016, 1016};
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
//        blast_warn("roach%i: Packet sequence number is %i. Last sequence number was =%i!",
//                  m_roach_udp->which, m_packet->packet_count, m_roach_udp->seq_number);
        m_roach_udp->seq_error_count++;
        retval |= ROACH_UDP_SEQ_ERR;
    }
    // TODO(laura/adrian): Implement the new roach UDP packet checksum test.
    /* if (m_packet->checksum != ROACH_CHECKSUM) {
        blast_err("roach%i: checksum = %i failed!", m_roach_udp->which, m_packet->checksum);
        m_roach_udp->crc_error_count++;
	    retval |= ROACH_UDP_CRC_ERR;
    }*/
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
void parse_udp_packet(data_udp_packet_t* m_packet, uint8_t* m_buf)
{
    static int debug_count = 0;
    if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
        blast_info("starting parse_udp_packet");
    }
    // static uint64_t i_packet = 0;
    // uint8_t *payload = (uint8_t *)(m_packet->rcv_buffer);
    uint8_t *buf = (m_buf + HEADER_LEN);
    m_packet->ctime = (buf[8172] << 24) | (buf[8173] << 16) | (buf[8174] << 8) | buf[8175];
    m_packet->pps_count = (buf[8176] << 24) | (buf[8177] << 16) | (buf[8178] << 8) | buf[8179];
    m_packet->clock_count = (buf[8180] << 24) | (buf[8181] << 16) | (buf[8182] << 8) | buf[8183];
    m_packet->packet_count = (buf[8184] << 24) | (buf[8185] << 16) | (buf[8186] << 8) | buf[8187];
    m_packet->status_reg = (buf[8188] << 24) | (buf[8189] << 16) | (buf[8190] << 8) | buf[8191];
    if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
        blast_info("ctime =%u, pps_count =%u, clock_count =%u, packet_count =%u, status_reg =%u",
                   m_packet->ctime, m_packet->pps_count, m_packet->clock_count,
                   m_packet->packet_count, m_packet->status_reg);
    }
    // I, Q
    for (int i = 0; i < 1016; i += 1) {
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
        blast_info("i = %i, I = %f, Q = %f, ctime = %u, count = %u, status reg = %u",
                    i_packet, m_packet->Ival[i], m_packet->Qval[i], m_packet->ctime,
                    m_packet->packet_count, m_packet->status_reg);
        }*/
    }
    if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) debug_count++;
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
    static uint64_t blksize = 0;
    if (m_roach_udp->first_packet) {
        // blast_info("checksum = %i, pps_count = %i, clock_count = % i, packet_count = %i",
        //  m_packet->checksum, m_packet->pps_count, m_packet->clock_count, m_packet->packet_count);
        blksize = ((uint64_t) (&(m_packet->status_reg)))+sizeof(m_packet->status_reg)
                   -((uint64_t) m_packet);
        m_roach_udp->first_packet = FALSE;
    }
    /* if (packet_count < 100) {
        blast_info("roach%i: Write index = %i", m_roach_udp->which, m_roach_udp->index);
    } */
    memcpy(&(m_roach_udp->last_pkts[m_roach_udp->index]), m_packet, sizeof(*m_packet));

    store_data_roach_udp(m_packet, blksize, m_roach_udp->i_which);

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
void roach_process_stream(roach_handle_data_t *m_roach_udp, data_udp_packet_t *m_packet, uint8_t *m_buf)
{
    static int debug_count = 0;
    if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
        blast_info("starting roach_process_stream");
    }
    parse_udp_packet(m_packet, m_buf);
    uint16_t udperr = check_udp_packet(m_packet, m_roach_udp);
    if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
        blast_info("check_udp_packet = %u", udperr);
    }
    // store_roach_udp_packet(m_packet, m_roach_udp, udperr); // Writes packet to harddrive.
    if (udperr > 0) return;
    udp_store_to_structure(m_roach_udp, m_packet);
    m_roach_udp->have_warned = 0;
    if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
        debug_count++;
    }
}

/* Function: poll_socket
 * ------------------------
 * Poll the raw socket for data. When data is received, call roach_process_stream
 */
void poll_socket(void)
{
    init_roach_socket();
    static data_udp_packet_t m_packet;

    int debug_count = 0;
    // blast_info("Roach socket file descriptor is %i", roach_sock_fd);
    if (!roach_sock_fd) {
       blast_err("Failed to open Socket");
    } else {
       // blast_info("Roach socket file descriptor is %i", roach_sock_fd);
    }
    // blast_info("Creating poll thread...");
    int rv;
    struct pollfd ufds[1];
    ufds[0].fd = roach_sock_fd;
    ufds[0].events = POLLIN; // Check for data on socket

    uint16_t num_packets = 0;
    uint8_t buf[ROACH_UDP_BUF_LEN];

    while (1) {
        rv = poll(ufds, 1, -1);     // Wait for event, block forever (timeout == -1).
                                    // timeout == 0 means don't block at all.
                                    // timeout == n means block for n milliseconds.
        if (rv == -1) {
            blast_err("Roach socket poll error");
        } else {
            if (ufds[0].revents & POLLIN) { // check for events on socket
                if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) blast_info("roach_udp poll event!");
                uint32_t bytes_read = recv(roach_sock_fd, buf, ROACH_UDP_BUF_LEN, 0);

                if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) blast_info("bytes read = %u!", bytes_read);

                // Don't even try to read the header if we read less than the size of a udp header struct.
                if (bytes_read < (sizeof(struct udphdr) + sizeof(struct iphdr) + sizeof(struct ethhdr))) {
                    blast_err("We read only %ud", bytes_read);
                    continue;
                }
                num_packets = bytes_read/ROACH_UDP_LEN; // How many packets did we read?
                if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) blast_info("num_packets = %u!", num_packets);
                for (int i_pkt =0; i_pkt < num_packets; i_pkt++) {
                    m_packet.udp_header = (struct udphdr *)(buf
                        + sizeof(struct ethhdr)
                        + sizeof(struct iphdr) + ROACH_UDP_LEN*i_pkt);
                /* Filter destination address */
		// blast_info("Before filt: R%d\t%d\t%d", m_roach_udp->which,
                          // m_roach_udp->port, ntohs(m_packet.udp_header->dest));
                    for (int ind = 0; ind < NUM_ROACHES; ind++) { // Figure out how to process this packet.
                        roach_handle_data_t *m_roach_udp = (roach_handle_data_t*)&roach_udp[ind];
                        if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
                            blast_info("i_pkt = %d, roach = %d, port = %u, header dest = %u",
                                       i_pkt, ind+1, m_roach_udp->port, ntohs(m_packet.udp_header->dest));
                        }
                    // blast_info("Roach udp %d, port = %d", m_roach_udp->which, m_roach_udp->port);
                        if ((m_roach_udp->port != ntohs(m_packet.udp_header->dest))) {
                            continue;
                        } else if (bytes_read < ROACH_UDP_DATA_LEN) {
                        // This only works if the truncated packet is the only thing read,
                        // Right now if, for example, the 3rd packet read is truncated we will be
                        // out of sequence for the rest of the subsequent packets read out in buf.
                        // TODO(laura): Test to see if this happens and if so write code to search
                        // for the next packet.
                            blast_err("Roach%i: Read only %u bytes.", m_roach_udp->which, bytes_read);
                            m_roach_udp->roach_invalid_packet_count++;
                            m_roach_udp->roach_packet_count++;
                            continue;
                        // blast_info("%d: These should match: %d\t%d", m_roach_udp->which,
                         // m_roach_udp->port, ntohs(m_packet.udp_header->dest));}
                        } else {
                            if (bytes_read < ROACH_UDP_DATA_LEN*(i_pkt+1)) {
                                blast_err("We are being asked to read beyond the end of the data length of %d.",
                                          ROACH_UDP_DATA_LEN*(i_pkt+1));
                            } else {
                                if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) {
                                    blast_info("Sending packet for processing: ptr offset = %d",
                                               i_pkt * ROACH_UDP_DATA_LEN);
                                }
                                roach_process_stream(m_roach_udp, &m_packet , (buf + i_pkt * ROACH_UDP_DATA_LEN));
                            }
                        }
                    }
                }
                if (debug_count < ROACH_UDP_DEBUG_PRINT_COUNT) debug_count++;
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

        // blast_info("roach%i: Configuring roach information corresponding to %s",
        //     m_roach_udp->which, m_roach_udp->address);
        struct hostent *udp_origin = gethostbyname(m_roach_udp->address);
        origaddr = *(uint32_t*)(udp_origin->h_addr_list[0]);
        snprintf(m_roach_udp->ip, sizeof(m_roach_udp->ip), "%d.%d.%d.%d",
            (origaddr & 0xff), ((origaddr >> 8) & 0xff),
            ((origaddr >> 16) & 0xff), ((origaddr >> 24) & 0xff));
        blast_info("UDP packets for ROACH%d from IP %s on port %i", m_roach_udp->which,
            m_roach_udp->ip, m_roach_udp->port);

        // blast_info("Initializing ROACH UDP packet reading.");

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

/* Function: generate_roach_udp_linklist_format
 * --------------------------------------------
 * Generates a text file such that the raw binary roach udp data is
 * readable from linklists.
 */

// this is a very specific macro written to loop over the roach udp structure
#define ADD_STRUCT_ENTRY_TO_LINKLIST(_FIELD, _NAME, _TYPE)                     \
({                                                                             \
    void *__ptr = &(m_packet._FIELD);                                          \
    uint64_t pad = ((uint64_t) __ptr)-base-loc;                                \
    if (pad) fprintf(fp, "PADDING_AT_%" PRIu64 " B %" PRIu64 "\n", loc, pad);  \
    loc += pad;                                                                \
                                                                               \
    strncpy(sfe[entry_i].field, _NAME, FIELD_LEN-1);                           \
    sfe[entry_i].type = _TYPE;                                                 \
    sfe[entry_i].spf = 1;                                                      \
    sfe[entry_i].start = loc;                                                  \
    sfe[entry_i].skip = sizeof(m_packet._FIELD);                               \
    entry_i++;                                                                 \
                                                                               \
    fprintf(fp, "%s\n", _NAME);                                                \
    loc += sizeof(m_packet._FIELD);                                            \
})

#define ROACH_STRUCT_SF_NUM_ENTRIES (MAX_CHANNELS_PER_ROACH*2+6)

linklist_t * generate_roach_udp_linklist(char * filename, int roach)
{
    FILE * fp = fopen(filename, "w+");
    int j = 0;

    linklist_t * ll = NULL;
    superframe_t * roach_sf = NULL;
    superframe_entry_t * sfe = NULL;

    int entry_i = 0;

    char fieldname[128] = {0};

    if (!fp) {
        blast_err("Unable to open roach %d format file at %s\n", roach, filename);
        return NULL;
    }

    // Generate the superframe
    // Actually want to calloc because the pointer is stored in the superframe and
    // linklist, which is returned.
    sfe = calloc(ROACH_STRUCT_SF_NUM_ENTRIES+1, sizeof(superframe_entry_t));

    // dummy udp packet for mapping
    data_udp_packet_t m_packet;
    uint64_t base = (uint64_t) &m_packet;
    uint64_t loc = 0;
    uint64_t blksize = ((uint64_t) (&(m_packet.status_reg)))+sizeof(m_packet.status_reg)-((uint64_t) &m_packet);

    // --- STEP 1: generate the linklist format file --- //

    // set the no checksum flag
    fprintf(fp, LINKLIST_FILE_SERIAL_IND "%.08x\n", 0xaddfaded); // format specifier
    fprintf(fp, LINKLIST_FILE_SIZE_IND "%" PRIu64 "\n", blksize+4); // blk_size = bulk size (+4 for checksums)
    fprintf(fp, LINKLIST_FRAMES_PER_FILE_IND "%d\n", 488*STORE_DATA_FRAMES_PER_FILE); // number of frames per file
    fprintf(fp, "#\n");
    fprintf(fp, "%s\n", STR(LL_NO_AUTO_CHECKSUM));

    // write the I channel fields to the file
    for (j = 0; j < n_publish_roaches[roach]; j++) {
      snprintf(fieldname, sizeof(fieldname), "i_kid%04d_roach%d", j, roach+1);
      ADD_STRUCT_ENTRY_TO_LINKLIST(Ival[j], fieldname, SF_FLOAT32);
    }

    // write the Q channel fields to the file
    for (j = 0; j < n_publish_roaches[roach]; j++) {
      snprintf(fieldname, sizeof(fieldname), "q_kid%04d_roach%d", j, roach+1);
      ADD_STRUCT_ENTRY_TO_LINKLIST(Qval[j], fieldname, SF_FLOAT32);
    }
    snprintf(fieldname, sizeof(fieldname), "header_roach%d", roach+1);
    if (sizeof(m_packet.udp_header) == 8) {
      ADD_STRUCT_ENTRY_TO_LINKLIST(udp_header, fieldname, SF_UINT64);
    } else {
      ADD_STRUCT_ENTRY_TO_LINKLIST(udp_header, fieldname, SF_UINT32);
    }

    snprintf(fieldname, sizeof(fieldname), "ctime_roach%d", roach+1);
    ADD_STRUCT_ENTRY_TO_LINKLIST(ctime, fieldname, SF_UINT32);

    snprintf(fieldname, sizeof(fieldname), "pps_count_roach%d", roach+1);
    ADD_STRUCT_ENTRY_TO_LINKLIST(pps_count, fieldname, SF_UINT32);

    snprintf(fieldname, sizeof(fieldname), "clock_count_roach%d", roach+1);
    ADD_STRUCT_ENTRY_TO_LINKLIST(clock_count, fieldname, SF_UINT32);

    snprintf(fieldname, sizeof(fieldname), "packet_count_roach%d", roach+1);
    ADD_STRUCT_ENTRY_TO_LINKLIST(packet_count, fieldname, SF_UINT32);

    snprintf(fieldname, sizeof(fieldname), "status_reg_roach%d", roach+1);
    ADD_STRUCT_ENTRY_TO_LINKLIST(status_reg, fieldname, SF_UINT32);

    // the final entry must be NULL terminated
    sfe[entry_i].field[0] = '\0';

    fclose(fp);

    // --- STEP 2: generate the linklist --- //
    // build the superframe
    roach_sf = linklist_build_superframe(sfe, &channel_data_to_double, &channel_double_to_data, 0);

    // parse the newly made linklist file
    ll = parse_linklist_format(roach_sf, filename);

    return ll;
}

/* Function: write_roach_channels_488hz
 * ------------------------------------
 * Populates 488 Hz frame fields
 */
void write_roach_channels_488hz(void)
{
    uint8_t i_udp_read;
    static channel_t *RoachCTimeAddr[NUM_ROACHES];
    static channel_t *RoachPPSCountAddr[NUM_ROACHES];
    static channel_t *RoachClockCountAddr[NUM_ROACHES];
    static channel_t *RoachPacketCountAddr[NUM_ROACHES];
    char channel_name_ctime[128] = {0};
    char channel_name_pps_count[128] = {0};
    char channel_name_clock_count[128] = {0};
    char channel_name_packet_count[128] = {0};
    int i;
    static int firsttime = 1;

    if (firsttime) {
        blast_info("Starting write_roach_channels_488hz");
        for (i = 0; i < NUM_ROACHES; i++) {
            snprintf(channel_name_ctime, sizeof(channel_name_ctime), "ctime_packet_roach%d", i+1);
            RoachCTimeAddr[i] = channels_find_by_name(channel_name_ctime);
            snprintf(channel_name_pps_count, sizeof(channel_name_pps_count), "pps_count_roach%d", i+1);
            RoachPPSCountAddr[i] = channels_find_by_name(channel_name_pps_count);
            snprintf(channel_name_clock_count, sizeof(channel_name_clock_count), "clock_count_roach%d", i+1);
            RoachClockCountAddr[i] = channels_find_by_name(channel_name_clock_count);
            snprintf(channel_name_packet_count, sizeof(channel_name_packet_count), "packet_count_roach%d", i+1);
            RoachPacketCountAddr[i] = channels_find_by_name(channel_name_packet_count);
        }
        firsttime = 0;
    }
    for (i = 0; i < NUM_ROACHES; i++) {
        i_udp_read = GETREADINDEX(roach_udp[i].index);
        data_udp_packet_t* m_packet = &(roach_udp[i].last_pkts[i_udp_read]);
        SET_UINT32(RoachCTimeAddr[i], m_packet->ctime);
        SET_UINT32(RoachPPSCountAddr[i], m_packet->pps_count);
        SET_UINT32(RoachClockCountAddr[i], m_packet->clock_count);
        SET_UINT32(RoachPacketCountAddr[i], m_packet->packet_count);
    }
}
