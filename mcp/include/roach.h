/*
 * roach.h
 *
 * This software is copyright (C) 2013-2014 University of Pennsylvania
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
 *  Created on: Apr 26, 2016
 *      Author: seth
 */

#ifndef INCLUDE_ROACH_H_
#define INCLUDE_ROACH_H_

#include <stddef.h>
#include <stdint.h>
#include <glib.h>
#include <math.h>
#include "phenom/socket.h"

typedef struct {
    int32_t I;
    int32_t Q;
} __attribute__((packed)) udp_element_t;

typedef struct {
    udp_element_t data[1024];
    uint32_t cycle_count;
    uint32_t pps_count:24;
    uint32_t pkt_count:8;
} __attribute__((packed)) udp_packet_t;

typedef enum {
    ROACH_STATUS_BOOT = 0,
    ROACH_STATUS_CONNECTED,
    ROACH_STATUS_PROGRAMMED,
    ROACH_STATUS_CONFIGURED,
    ROACH_STATUS_CALIBRATED,
    ROACH_STATUS_TONE,
    ROACH_STATUS_STREAMING,
    ROACH_STATUS_VNA,
    ROACH_STATUS_ARRAY_FREQS,
    ROACH_STATUS_TARG,
    ROACH_STATUS_ACQUIRING,
} e_roach_status;

typedef enum {
    ROACH_UPLOAD_RESULT_WORKING = 0,
    ROACH_UPLOAD_CONN_REFUSED,
    ROACH_UPLOAD_RESULT_TIMEOUT,
    ROACH_UPLOAD_RESULT_ERROR,
    ROACH_UPLOAD_RESULT_SUCCESS
} e_roach_upload_result;

typedef struct {
    size_t len;
    double *I;
    double *Q;
} roach_lut_t;

typedef struct {
    size_t len;
    uint16_t *I;
    uint16_t *Q;
} roach_uint16_lut_t;

typedef struct roach_state {
    int which;
    int katcp_fd;
    e_roach_status status;
    e_roach_status desired_status;

    int has_error;
    const char *last_err;
    const char *address;
    uint16_t port;

    double *freq_residuals;

    double *freq_comb;
    size_t freqlen;
    double *kid_freqs;
    size_t num_kids;

    // First two LUTs are for building
    roach_lut_t DDS;
    roach_lut_t DAC;
    // This LUT is what gets written
    roach_uint16_lut_t LUT;

    char *vna_path;
    char *targ_path;
    char *channels_path;
    uint16_t dest_port;

    // PPC link
    struct katcl_line *rpc_conn;
} roach_state_t;

typedef struct {
    const char *firmware_file;
    uint16_t port;
    struct timeval timeout;
    e_roach_upload_result result;
    roach_state_t *roach;
    ph_sock_t *sock;
} firmware_state_t;

// Called each time a packet is received
typedef struct data_packet {
	unsigned char *rcv_buffer;
	struct ethhdr *eth;
	struct iphdr *ip;
	float *I;
	float *Q;
	uint32_t checksum;
	uint32_t pps_count;
	uint32_t clock_count;
	uint32_t packet_count;
} data_packet_t;

#define NUM_ROACHES 4
#define NUM_ROACH_UDP_CHANNELS 1024

static const char roach_name[4][32] = {"roach1", "roach2", "roach3", "roach4"};

// Destination IP for UDP packets
// This is the destination IP on Lazarus:
// static const char udp_dest[32] = "192.168.42.1";
// static uint32_t dest_ip = 192*pow(2, 24) + 168*pow(2, 16) + 42*pow(2, 8) + 1;

// Destination IP for fc1
static const char udp_dest[32] = "192.168.1.3";
static uint32_t dest_ip = 192*pow(2, 24) + 168*pow(2, 16) + 40*pow(2, 8) + 3;
static const char udp_dest_name[32] = "fc1.blast";

// TODO(laura/sam): Set up either a multicast address or arrange for the switch to mirror
// the packet broadcast so that both FCs can receive the UDP packets.

const char *roach_get_name(roach_state_t *m_roach);
int roach_write_data(roach_state_t *m_roach, const char *m_register, uint8_t *m_data,
                            size_t m_len, uint32_t m_offset, int m_timeout);
int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size, int ms_timeout);
int roach_write_int(roach_state_t *m_roach, const char *m_register, uint32_t m_val, uint32_t m_offset);
int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename);

// Defined in roach_udp.c
void roach_udp_networking_init(int m_which, roach_state_t* m_roach_state, size_t m_numchannels);

void write_roach_channels_244hz(void);
void shutdown_roaches(void);

#endif /* INCLUDE_ROACH_H_ */
