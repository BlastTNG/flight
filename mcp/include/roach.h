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
 *  Last edited: September 22, 2017
 *      Author: sam, laura, seth
 */

#ifndef INCLUDE_ROACH_H_
#define INCLUDE_ROACH_H_

#include <stddef.h>
#include <stdint.h>
#include <glib.h>
#include <math.h>
#include <netinet/in.h>
#include <fftw3.h>
#include "phenom/socket.h"
#include "phenom/buffer.h"
#include "remote_serial.h"
#include "linklist.h"
#include "channels_tng.h"

#define ROACH_UDP_CRC_ERR 0x01
#define ROACH_UDP_SEQ_ERR 0x02

#define MAX_CHANNELS_PER_ROACH 1016
#define NUM_ROACHES 5
#define NUM_ROACH_UDP_CHANNELS 1024
#define ROACH_UDP_LEN 8234
#define ROACH_DF_FILT_LEN 10
#define ROACH_UDP_DATA_LEN NUM_ROACH_UDP_CHANNELS * 4 * 2
#define IPv4(a, b, c, d) ((uint32_t)(((a) & 0xff) << 24) | \
                                            (((b) & 0xff) << 16) | \
                                            (((c) & 0xff) << 8)  | \
                                            ((d) & 0xff))
#define NC1_PORT 12345
#define NC2_PORT 12346

static const char roach_name[5][32] = {"roach1", "roach2", "roach3", "roach4", "roach5"};
// Destination IP for fc2
static const char udp_dest[32] = "239.1.1.234";
static const char udp_dest_name[32] = "roach-udp-dest";
int roach_sock_fd;

typedef struct {
    int32_t Ival;
    int32_t Qval;
} __attribute__((packed)) udp_element_t;

typedef struct {
    udp_element_t data[1024];
    uint32_t cycle_count;
    uint32_t pps_count:24;
    uint32_t pkt_count:8;
} __attribute__((packed)) udp_packet_t;

typedef enum {
    ROACH_STATE_BOOT = 0,
    ROACH_STATE_CONNECTED = 1,
    ROACH_STATE_PROGRAMMED = 2,
    ROACH_STATE_CONFIGURED = 3,
    ROACH_STATE_STREAMING = 4,
} e_roach_state;

typedef enum {
    PI_STATE_BOOT = 0,
    PI_STATE_CONNECTED = 1,
    PI_STATE_INIT = 2,
} e_pi_state;

typedef enum {
    ROACH_UPLOAD_RESULT_WORKING = 0,
    ROACH_UPLOAD_CONN_REFUSED,
    ROACH_UPLOAD_RESULT_TIMEOUT,
    ROACH_UPLOAD_RESULT_ERROR,
    ROACH_UPLOAD_RESULT_SUCCESS
} e_roach_upload_result;

typedef struct {
    size_t len;
    double *Ival;
    double *Qval;
} roach_lut_t;

typedef struct {
    size_t len;
    uint16_t *Ival;
    uint16_t *Qval;
} roach_uint16_lut_t;

typedef struct roach_state {
    char *fpg;
    int array;
    int which;
    int katcp_fd;
    e_roach_state state;
    e_roach_state desired_state;

    int has_error;
    const char *last_err;
    char *address;
    uint16_t port;
    bool has_qdr_cal;
    bool has_tones;
    bool has_targ_tones;
    bool is_streaming;
    bool is_sweeping;
    bool has_vna_sweep;
    bool has_targ_sweep;
    bool has_amp_cal;
    bool has_adc_cal;
    bool write_flag;

    double *freq_residuals;
    double *targ_tones; // kid frequencies found with get_targ_freqs()
    double lo_freq_req;
    size_t current_ntones; // number of current kid frequencies
    size_t num_kids; // number of current kid frequencies
    double lo_centerfreq;

    // First two LUTs are for building
    roach_lut_t DDC;
    roach_lut_t DAC;
    // This LUT is what gets written
    roach_uint16_lut_t LUT;

    // VNA sweep
    double *vna_comb;
    double vna_sweep_span;
    size_t vna_comb_len;
    char *vna_path_root;
    char *chop_path_root;
    double p_max_freq;
    double p_min_freq;
    double n_max_freq;
    double n_min_freq;
    // VNA/TARG/CAL sweep file paths
    char *sweep_root_path;
    char *targ_path_root;
    char *last_vna_path;
    char *last_targ_path;
    char *channels_path;

    // For detector retune decision
    int nflag_thresh; // num channels which need to be out of range for retune
    int has_ref; /* If 1, ref grads exist */
    int retune_flag; // 1 if retune is recommended
    bool out_of_range[MAX_CHANNELS_PER_ROACH]; // 1 if kid df is out of range

    double ref_grads[MAX_CHANNELS_PER_ROACH][2]; // The reference grad values
    double ref_vals[MAX_CHANNELS_PER_ROACH][2]; // reference I,Q values for df calculation
    double df[MAX_CHANNELS_PER_ROACH];

    char *last_cal_path;
    // path to the last master chop directory
    char *last_chop_path;
    char *cal_path_root;
    // array of tone amplitudes used for calibration
    double cal_amps[MAX_CHANNELS_PER_ROACH][5];
    // Python logs (for saving/reading response)
    char *qdr_log;
    char *find_kids_log;
    char *opt_tones_log;
    uint16_t dest_port;
    uint16_t src_port;
    uint32_t src_ip;
    // Path to tone amplitudes file
    char *amps_path[2];
    double *last_amps;
    double *last_phases;
    char *vna_amps_path[2];
    char *targ_amps_path[2];
    char *random_phase_path;
    char *phase_centers_path;
    char *freqlist_path;
    fftw_plan comb_plan;

    // PPC link
    struct katcl_line *rpc_conn;
} roach_state_t;

typedef struct pi_state {
    int which;
    e_pi_state state;
    e_pi_state desired_state;
    uint16_t port;
    bool has_valon;
    bool has_input_atten;
    bool has_output_atten;
    remote_serial_t *pi_comm;
} pi_state_t;

typedef struct {
    const char *firmware_file;
    uint16_t port;
    struct timeval timeout;
    e_roach_upload_result result;
    roach_state_t *roach;
    ph_sock_t *sock;
} firmware_state_t;

// Called each time a packet is received
typedef struct data_udp_packet {
    float Ival[MAX_CHANNELS_PER_ROACH];
    float Qval[MAX_CHANNELS_PER_ROACH];
    struct udphdr *udp_header; // will filter on udp dest port
    uint32_t ctime;
    uint32_t pps_count;
    uint32_t clock_count;
    uint32_t packet_count;
    uint32_t status_reg;
    unsigned char rcv_buffer[ROACH_UDP_LEN];
} data_udp_packet_t;

typedef struct {
    int roach;
    bool            opened;
    bool            have_warned;
    bool            want_reset;
    uint8_t         which;
    uint8_t         i_which;
    uint32_t        seq_error_count;
    uint32_t        crc_error_count;
    uint32_t        seq_number;
    uint16_t        num_channels;
    uint32_t        roach_invalid_packet_count;
    uint32_t        roach_packet_count;
    uint32_t        roach_valid_packet_count;
    uint8_t         index;
    uint16_t        port;
    char            address[16];
    char            listen_ip[16];
    char            ip[16];
    bool            first_packet;
    data_udp_packet_t last_pkts[3];
    int sock; // the socket file descriptor
    struct timeval  timeout;
} roach_handle_data_t;

typedef struct {
    float ibuf[ROACH_DF_FILT_LEN];
    float qbuf[ROACH_DF_FILT_LEN];
    int ind_last;
    int ind_roach;
    int ind_kid;
    float i_sum;
    float q_sum;
    float i_cur;
    float q_cur;
    float df;
    int first_call;
} roach_df_calc_t;

roach_df_calc_t roach_df_telem[NUM_ROACHES];

roach_handle_data_t roach_udp[NUM_ROACHES];

// This structure is used for writing a header for each roach-udp packet to disk.
typedef struct {
    uint32_t        roach_packet_count;
    uint16_t packet_err_code; // Zero if there was no error.
    time_t write_time; // Time before we call write to harddrive.
    uint32_t packet_crc; // CRC of the packet
    uint8_t         which;
    bool            want_reset;
    uint16_t        port;
} roach_packet_header_out_t;


// TODO(laura/sam): Set up either a multicast address or arrange for the switch to mirror
// the packet broadcast so that both FCs can receive the UDP packets.

const char *roach_get_name(roach_state_t *m_roach);
int roach_write_data(roach_state_t *m_roach, const char *m_register, uint8_t *m_data,
                            size_t m_len, uint32_t m_offset, int m_timeout);
int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size, int ms_timeout);
int roach_write_int(roach_state_t *m_roach, const char *m_register, uint32_t m_val, uint32_t m_offset);
int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename);
int init_roach(uint16_t ind);
void write_roach_channels_5hz(void);
int get_roach_state(uint16_t ind);
void roach_timestamp_init(uint16_t ind);

// Defined in roach_udp.c
void roach_udp_networking_init(void);
void write_roach_channels_244hz(void);
void write_roach_channels_488hz(void);
void shutdown_roaches(void);

linklist_t * generate_roach_udp_linklist(char *, int);

#endif /* INCLUDE_ROACH_H_ */
