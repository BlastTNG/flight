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
 *  Created on: Apr 5, 2015
 *      Author: seth
 */

#include <stdint.h>
#include <complex.h>
#include <math.h>

// N.B. fftw3.h needs to be AFTER complex.h
#include <fftw3.h>

#include <katcp.h>
#include <katcl.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "blast.h"
#include "crc.h"
#include "roach.h"

static int roach_fft_shift = 255;
static double dac_samp_freq = 512.0e6;
static double fpga_samp_freq = 256.0e6;
static int dds_shift = 304; // This varies b/t fpg/bof files
static int f_base = 300;
static int fft_len = 1024;

typedef enum {
    ROACH_STATUS_BOOT,
    ROACH_STATUS_CONNECTED,
    ROACH_STATUS_PROGRAMMED,
    ROACH_STATUS_TONE,
    ROACH_STATUS_DDS,
    ROACH_STATUS_STREAMING,
} e_roach_status;

typedef struct {
    int32_t II;
    int32_t QQ;
} __attribute__((packed)) udp_element_t;

typedef struct {
    udp_header_t header;
    udp_element_t data[1024];
    uint64_t timestamp;
    uint32_t checksum;
} __attribute__((packed)) udp_packet_t;

typedef struct {
    size_t len;
    double *II;
    double *QQ;
} roach_lut_t;

typedef struct {
    int status;
    int has_error;
    const char *last_err;
    const char *address;
    uint16_t port;
    int ms_cmd_timeout;

    double dac_freq_res;
    roach_lut_t DDS;
    roach_lut_t DAC;
    roach_lut_t LUT;

    struct katcl_line *rpc_conn;
} roach_state_t;

typedef struct {
    char *firmware_file;
    uint16_t port;
    struct timeval timeout;
    roach_state_t *roach;
    ph_sock_t *sock;
} firmware_state_t;

static ph_thread_t *katcp_thread = NULL;

static void roach_buffer_ntohl(uint32_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohl(m_buffer[i]);
    }
}

static void roach_buffer_ntohs(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohs(m_buffer[i]);
    }
}

static int roach_write_int(roach_state_t *m_roach, const char *m_register, unsigned long m_val, unsigned long m_offset)
{
    unsigned long sendval = htonl(m_val);
    return send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?write",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_BUFFER | KATCP_FLAG_LAST, &sendval, sizeof(sendval),
                   NULL);
}

static int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size)
{
    int retval = send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "read",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_size,
                   NULL);

    if (retval < 0) {
        blast_err("Could not read data '%s' from %s: Internal Error", m_register, m_roach->address);
        return -1;
    }
    if (retval > 0) {
        char *ret = arg_string_katcl(m_roach->rpc_conn, 1);
        blast_err("Could not read data '%s' from %s: ROACH Error '%s'", m_register, m_roach->address, ret?ret:"");
        return -1;
    }

    if (arg_count_katcl(m_roach->rpc_conn) < 2) {
        blast_err("Expecting 2 return values.  Recevied %d", arg_count_katcl(m_roach->rpc_conn));
        return -1;
    }

    uint32_t bytes_copied = arg_buffer_katcl(m_roach->rpc_conn, 1, m_dest, m_size);

    if (bytes_copied != m_size) {
        blast_err("Expecting %ul bytes but only received %ul bytes", m_size, bytes_copied);
        return -1;
    }
    return 0;
}

static int roach_reset_dac(roach_state_t *m_roach,)
{
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
}

static int roach_read_mixer_snaps(roach_state_t *m_roach, uint32_t m_shift, uint32_t m_chan,
                                  double *m_mixer_in, double *m_mixer_out)
{
    size_t buffer_len = 8 * (1<<14);
    int16_t *temp_data;

    if (roach_write_int(m_roach, "dds_shift", m_shift, 0)) return -1;
    if (roach_write_int(m_roach, "chan_select", (m_chan & (UINT32_MAX - 1)) >> 1, 0)) return -1;
    if (roach_write_int(m_roach, "rawfftbin_ctrl", 0, 0)) return -1;
    if (roach_write_int(m_roach, "mixerout_ctrl", 0, 0)) return -1;
    if (roach_write_int(m_roach, "rawfftbin_ctrl", 1, 0)) return -1;
    if (roach_write_int(m_roach, "mixerout_ctrl", 1, 0)) return -1;

    temp_data = calloc(sizeof(int16_t), buffer_len);
    if (roach_read_data(m_roach, temp_data, "rawfftbin_bram", 0, buffer_len * sizeof(uint16_t))) {
        free(temp_data);
        return -1;
    }
    roach_buffer_ntohs(temp_data, buffer_len);
    for (size_t i = 0; i < buffer_len; i++) {
        m_mixer_in[i] = temp_data[i] / ((float)(1<<15));
    }

    if (m_mixer_out) {
        if (roach_read_data(m_roach, temp_data, "mixerout_bram", 0, buffer_len)) return -1;
        roach_buffer_ntohs(temp_data, buffer_len / 2);
        for (size_t i = 0; i < buffer_len / 2; i++) {
            m_mixer_out[i] = temp_data[i] / ((double)(1<<14));
        }
    }
    return 0;
}

static void roach_init_LUT(roach_state_t * m_roach, size_t m_len)
{
    m_roach->LUT.len = m_len;
    m_roach->LUT.II = calloc(m_len, sizeof(double));
    m_roach->LUT.QQ = calloc(m_len, sizeof(double));
}

static inline int roach_fft_bin_index(double *m_freqs, int m_index, size_t m_fft_len, double m_samp_freq)
{
    return (int)lround(m_freqs[m_index] / m_samp_freq * m_fft_len);
}
static int roach_freq_comb(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen,
                            int m_samp_freq, bool m_random_phase, bool m_DAQ_LUT, double *m_amplitudes,
                            double *m_I, double *m_Q)
{
    size_t comb_fft_len;

    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / m_roach->dac_freq_res) * m_roach->dac_freq_res;
    }

    if (m_DAQ_LUT) {
        comb_fft_len = m_roach->LUT.len;
    } else {
        comb_fft_len = m_roach->LUT.len / fft_len;
    }

    double complex *spec = fftw_malloc(comb_fft_len * sizeof(double complex));
    double complex *wave = fftw_malloc(comb_fft_len * sizeof(double complex));
    double max_val = 0.0;


    srand48(time(NULL));
    for (size_t i = 0; i < comb_fft_len; i++) {
        spec[roach_fft_bin_index(i)] = cexp(I * drand48() * 2.0 * M_PI);
    }
    fftw_execute(fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE));
    for (size_t i = 0; i < comb_fft_len; i++) {
        if (cabs(spec[i]) > max_val) max_val = cabs(spec[i]);
    }
    for (size_t i = 0; i < comb_fft_len; i++) {
        m_I[i] = creal(wave) / max_val * ((1<<15)-1);
        m_Q[i] = cimag(wave) / max_val * ((1<<15)-1);
    }

    fftw_free(spec);
    fftw_free(wave);
    return 0;
}

static int roach_return_shift(roach_state_t *m_roach, uint32_t m_chan)
{


}

static int roach_save_1d(const char *m_filename, double *m_data, size_t m_len)
{
    uint32_t channel_crc;
    FILE *fp;

    channel_crc = crc32(BLAST_MAGIC32, m_data, sizeof(double) * m_len);
    fp = fopen(m_filename, "w");
    fwrite(&m_len, sizeof(size_t), 1, fp);
    fwrite(m_data, sizeof(double), m_len, fp);
    fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);
    return 0;
}

static ssize_t roach_load_1d(const char *m_filename, double **m_data)
{
    size_t len;
    FILE *fp;
    struct stat fp_stat;
    uint32_t channel_crc;

    if (stat(m_filename, &fp_stat)) {
        blast_err("Could not get file data for %s: %s", m_filename, strerror(errno));
        return -1;
    }
    if (!(fp = fopen(m_filename, "r"))) {
        blast_err("Could not open %s for reading: %s", m_filename, strerror(errno));
        return -1;
    }
    if (fread(&len, sizeof(len), 1, fp) != 1) {
        blast_err("Could not read data length from %s: %s", m_filename, strerror(errno));
        fclose(fp);
        return -1;
    }
    if ((len * sizeof(double)) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
        blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu",
                  (len * sizeof(double)) + sizeof(channel_crc) + sizeof(len), fp_stat.st_size);
        fclose(fp);
        return -1;
    }

    *m_data = calloc(len, sizeof(double));
    fread(*m_data, sizeof(double), len, fp);
    fread(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);

    if (channel_crc != crc32(BLAST_MAGIC32, *m_data, sizeof(double) * len)) {
        free(*m_data);
        *m_data = NULL;
        blast_err("Mismatched CRC for '%s'.  File corrupted?");
        len = -1;
    }
    return len;
}


static int roach_save_3d(const char *m_filename, size_t m_len, double m_data[3][m_len])
{
    uint32_t channel_crc;
    FILE *fp;

    channel_crc = crc32(BLAST_MAGIC32, m_data[0], sizeof(double) * m_len);
    channel_crc = crc32(channel_crc, m_data[1], sizeof(double) * m_len);
    channel_crc = crc32(channel_crc, m_data[2], sizeof(double) * m_len);
    fp = fopen(m_filename, "w");
    fwrite(&m_len, sizeof(size_t), 1, fp);
    fwrite(m_data[0], sizeof(double), m_len, fp);
    fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);
    return 0;
}

static ssize_t roach_load_3d(const char *m_filename, double ***m_data)
{
    size_t len;
    FILE *fp;
    struct stat fp_stat;
    uint32_t channel_crc;

    if (stat(m_filename, &fp_stat)) {
        blast_err("Could not get file data for %s: %s", m_filename, strerror(errno));
        return -1;
    }
    if (!(fp = fopen(m_filename, "r"))) {
        blast_err("Could not open %s for reading: %s", m_filename, strerror(errno));
        return -1;
    }
    if (fread(&len, sizeof(len), 1, fp) != 1) {
        blast_err("Could not read data length from %s: %s", m_filename, strerror(errno));
        fclose(fp);
        return -1;
    }
    if ((3 * len * sizeof(double)) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
        blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu",
                  (3 * len * sizeof(double)) + sizeof(channel_crc) + sizeof(len), fp_stat.st_size);
        fclose(fp);
        return -1;
    }

    *m_data = calloc(3, sizeof(double*));
    (*m_data)[0] = calloc(len, sizeof(double));
    (*m_data)[1] = calloc(len, sizeof(double));
    (*m_data)[2] = calloc(len, sizeof(double));
    fread((*m_data)[0], sizeof(double), len, fp);
    fread((*m_data)[1], sizeof(double), len, fp);
    fread((*m_data)[2], sizeof(double), len, fp);
    fclose(fp);

    channel_crc = crc32(BLAST_MAGIC32, (*m_data)[0], sizeof(double) * len);
    channel_crc = crc32(channel_crc, (*m_data)[1], sizeof(double) * len);
    if (channel_crc != crc32(channel_crc, (*m_data)[2], sizeof(double) * len)) {
        free(*m_data);
        *m_data = NULL;
        blast_err("Mismatched CRC for '%s'.  File corrupted?");
        len = -1;
    }
    return len;
}


/**
 * If we have an error, we'll disable the socket and schedule a reconnection attempt.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void firmware_upload_process_stream(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    ph_buf_t *buf;
    firmware_state_t *state = (firmware_state_t*) m_data;

    /**
     * If we have an error, or do not receive data from the LabJack in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & (PH_IOMASK_ERR|PH_IOMASK_TIME)) {
      blast_err("disconnecting from firmware upload at %s due to connection issue", state->address);
      ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
      ph_sock_enable(m_sock, 0);


      return;
    }

    buf = ph_sock_read_bytes_exact(m_sock, sizeof(labjack_data_header_t) + state_data->num_channels * 2);
    if (!buf) return; /// We do not have enough data

    ph_buf_delref(buf);
}

/**
 * Handle a connection callback from @connect_lj.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our LabJack State variable
 */
static void firmware_upload_connected(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    firmware_state_t *state = (firmware_state_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", state->roach->address, state->port, gai_strerror(m_errcode));

            return;

        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->roach->address, state->port, m_errcode, strerror(m_errcode));

            return;
    }

    blast_info("Connected to ROACH at %s", state->roach->address);

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    state->connected = true;
    state->backoff_sec = min_backoff_sec;
    m_sock->callback = (state->port == cmd_port)?labjack_process_cmd_resp:labjack_process_stream;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);
}

/**
 * Handles the connection job.  Formatted this way to allow us to schedule
 * a future timeout in the PH_JOB infrastructure
 *
 * @param m_job Unused
 * @param m_why Unused
 * @param m_data Pointer to the labjack State variable
 */
static void connect_roach_firmware(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    firmware_state_t *state = (firmware_state_t*)m_data;

    blast_info("Connecting to %s", state->roach->address);
    ph_sock_resolve_and_connect(state->roach->address, state->port,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, firmware_upload_connected, m_data);
}

int init_roach(void) {

    katcp_
}
