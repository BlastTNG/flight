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

#include <complex.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <sys/stat.h>

// N.B. fftw3.h needs to be AFTER complex.h
#include <fftw3.h>

// N.B. "I" is a terrible definition as many headers (OpenSSL!) use it.  We'll expand to _Complex_I
#undef I

#include "katcp.h"
#include "katcl.h"

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "blast.h"
#include "crc.h"
#include "roach.h"

static double dac_samp_freq = 512.0e6;
static double fpga_samp_freq = 256.0e6;
static int fft_len = 1024; /// Refers to the FFT length in firmware NOT software

typedef enum {
    ROACH_STATUS_BOOT,
    ROACH_STATUS_CONNECTED,
    ROACH_STATUS_PROGRAMMED,
    ROACH_STATUS_TONE,
    ROACH_STATUS_DDS,
    ROACH_STATUS_STREAMING,
} e_roach_status;

typedef enum {
    ROACH_UPLOAD_RESULT_WORKING = 0,
    ROACH_UPLOAD_RESULT_TIMEOUT,
    ROACH_UPLOAD_RESULT_ERROR,
    ROACH_UPLOAD_RESULT_SUCCESS
} e_roach_upload_result;

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
    int status;
    int desired_status;

    int has_error;
    const char *last_err;
    const char *address;
    uint16_t port;
    int ms_cmd_timeout;

    double dac_freq_res;
    double *freq_residuals;
    size_t lut_buffer_len;

    roach_lut_t DDS;
    roach_lut_t DAC;
    roach_uint16_lut_t LUT;

    char *vna_path;
    char *channels_path;

    struct katcl_line *rpc_conn;

    ph_sock_t *udp_socket;
} roach_state_t;

typedef struct {
    const char *firmware_file;
    uint16_t port;
    struct timeval timeout;
    int result;
    roach_state_t *roach;
    ph_sock_t *sock;
} firmware_state_t;

static ph_thread_t *roach_state = NULL;

static int roach_abs_cmp(const void* pa, const void* pb) {
    double a = fabs(*(const double*)pa);
    double b = fabs(*(const double*)pb);

    return (a > b) - (a < b);
}

static void roach_buffer_ntohs(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohs(m_buffer[i]);
    }
}

const char *roach_get_name(roach_state_t *m_roach)
{
    if (!m_roach || !m_roach->address) return "INVALID ROACH";
    return m_roach->address;
}

int roach_write_data(roach_state_t *m_roach, const char *m_register, uint8_t *m_data,
                            size_t m_len, uint32_t m_offset)
{
    return send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?write",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_BUFFER, m_data, m_len,
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_len,
                   NULL);
}

int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size)
{
    int retval = send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?read",
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

int roach_write_int(roach_state_t *m_roach, const char *m_register, uint32_t m_val, uint32_t m_offset)
{
    uint32_t sendval = htonl(m_val);
    return roach_write_data(m_roach, m_register, (uint8_t*)&sendval, sizeof(sendval), m_offset);
}




static int roach_reset_dac(roach_state_t *m_roach)
{
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
    return 0;
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
    if (roach_read_data(m_roach, (uint8_t*)temp_data, "rawfftbin_bram", 0, buffer_len * sizeof(uint16_t))) {
        free(temp_data);
        return -1;
    }
    roach_buffer_ntohs((uint16_t*)temp_data, buffer_len);
    for (size_t i = 0; i < buffer_len; i++) {
        m_mixer_in[i] = temp_data[i] / ((float)(1<<15));
    }

    if (m_mixer_out) {
        if (roach_read_data(m_roach, (uint8_t*)temp_data, "mixerout_bram", 0, buffer_len)) return -1;
        roach_buffer_ntohs((uint16_t*)temp_data, buffer_len / 2);
        for (size_t i = 0; i < buffer_len / 2; i++) {
            m_mixer_out[i] = temp_data[i] / ((double)(1<<14));
        }
    }
    return 0;
}

static void roach_init_LUT(roach_state_t * m_roach, size_t m_len)
{
    m_roach->LUT.len = m_len;
    m_roach->LUT.I = calloc(m_len, sizeof(uint16_t));
    m_roach->LUT.Q = calloc(m_len, sizeof(uint16_t));
}

static inline int roach_fft_bin_index(double *m_freqs, int m_index, size_t m_fft_len, double m_samp_freq)
{
    return (int)lround(m_freqs[m_index] / m_samp_freq * m_fft_len);
}
static int roach_freq_comb(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen,
                            int m_samp_freq, bool m_random_phase, bool m_DAQ_LUT,
                            double *m_I, double *m_Q)
{
    size_t comb_fft_len;
    fftw_plan comb_plan;

    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / m_roach->dac_freq_res) * m_roach->dac_freq_res;
    }

    if (m_DAQ_LUT) {
        comb_fft_len = m_roach->lut_buffer_len;
    } else {
        comb_fft_len = m_roach->lut_buffer_len / fft_len;
    }

    complex double *spec = (complex double*)fftw_malloc(comb_fft_len * sizeof(complex double));
    complex double *wave = (complex double*)fftw_malloc(comb_fft_len * sizeof(complex double));
    double max_val = 0.0;


    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        spec[roach_fft_bin_index(m_freqs, i, comb_fft_len, m_samp_freq)] =
                cexp(_Complex_I * drand48() * 2.0 * M_PI);
    }
    comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(comb_plan);
    fftw_destroy_plan(comb_plan);

    for (size_t i = 0; i < comb_fft_len; i++) {
        if (cabs(spec[i]) > max_val) max_val = cabs(spec[i]);
    }
    for (size_t i = 0; i < comb_fft_len; i++) {
        m_I[i] = creal(wave[i]) / max_val * ((1<<15)-1);
        m_Q[i] = cimag(wave[i]) / max_val * ((1<<15)-1);
    }

    fftw_free(spec);
    fftw_free(wave);
    return 0;
}


static inline size_t roach_get_max_index(complex double *m_vals, size_t m_len)
{
    int retval = 0;
    double max_val = 0.0;
    for (size_t i = 0; i < m_len; i++) {
        if (cabs(m_vals[i]) > max_val) {
            max_val = cabs(m_vals[i]);
            retval = i;
        }
    }
    return retval;
}

static int roach_return_shift(roach_state_t *m_roach, uint32_t m_chan)
{
    int n = 1024;
    int istride = 1024;
    int ostride = 1;
    int retval = 0;
    size_t dds_index = 0;

    complex double *dds_spec = (complex double*)fftw_malloc(n * sizeof(complex double));
    double *mixer_snaps = (double *)fftw_malloc((1<<17) * sizeof(double));
    double *dds_in = (double *)fftw_malloc((1<<14) * sizeof(double));

    fftw_plan shift_plan = fftw_plan_many_dft_r2c(1, &n, 1, m_roach->DDS.I, &n, istride, 0,
                                                dds_spec, &n, ostride, 0, FFTW_ESTIMATE);
    fftw_execute(shift_plan);
    fftw_destroy_plan(shift_plan);

    dds_index = roach_get_max_index(dds_spec, n);

    shift_plan = fftw_plan_dft_r2c_1d(n, dds_in, dds_spec, FFTW_ESTIMATE);
    for (int i = 0; i < 512; i++) {
        roach_read_mixer_snaps(m_roach, i, m_chan, mixer_snaps, NULL);
        for (size_t j = 2, k = 0; j < (1<<14); j += 8, k++) {
            dds_in[k] = (mixer_snaps[j] > INT16_MAX) ? mixer_snaps[j] - UINT16_MAX : mixer_snaps[j];
        }
        fftw_execute(shift_plan);
        if (roach_get_max_index(dds_spec, n) == dds_index) {
            blast_info("Found LUT shift of %d for %s", i, m_roach->address);
            retval = i;
            break;
        }
    }

    fftw_destroy_plan(shift_plan);
    fftw_free(dds_spec);
    fftw_free(mixer_snaps);
    fftw_free(dds_in);

    return retval;
}

static int roach_save_1d(const char *m_filename, void *m_data, size_t m_element_size, size_t m_len)
{
    uint32_t channel_crc;
    FILE *fp;

    channel_crc = crc32(BLAST_MAGIC32, m_data, m_element_size * m_len);
    fp = fopen(m_filename, "w");
    fwrite(&m_len, sizeof(size_t), 1, fp);
    fwrite(m_data, m_element_size, m_len, fp);
    fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);
    return 0;
}

static ssize_t roach_load_1d(const char *m_filename, void **m_data, size_t m_element_size)
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
    if ((len * m_element_size) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
        blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu", m_filename,
                  (len * m_element_size) + sizeof(channel_crc) + sizeof(len), fp_stat.st_size);
        fclose(fp);
        return -1;
    }

    *m_data = calloc(len, m_element_size);
    fread(*m_data, m_element_size, len, fp);
    fread(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);

    if (channel_crc != crc32(BLAST_MAGIC32, *m_data, m_element_size * len)) {
        free(*m_data);
        *m_data = NULL;
        blast_err("Mismatched CRC for '%s'.  File corrupted?", m_filename);
        len = -1;
    }
    return len;
}


static int roach_save_2d(const char *m_filename, size_t m_len, double *m_data)
{
    uint32_t channel_crc;
    FILE *fp;

    channel_crc = crc32(BLAST_MAGIC32, m_data, sizeof(double) * 2 * m_len);
    fp = fopen(m_filename, "w");
    fwrite(&m_len, sizeof(size_t), 1, fp);
    fwrite(m_data, sizeof(double), 2 * m_len, fp);
    fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);
    return 0;
}

static ssize_t roach_load_2d(const char *m_filename, double **m_data)
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
    if ((2 * len * sizeof(double)) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
        blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu", m_filename,
                  (size_t)(2 * len * sizeof(double)) + sizeof(channel_crc) + sizeof(len),
                  (size_t)fp_stat.st_size);
        fclose(fp);
        return -1;
    }

    *m_data = calloc(2 * len, sizeof(double));
    fread(*m_data, sizeof(double), 2 * len, fp);
    fread(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);

    if (channel_crc != crc32(BLAST_MAGIC32, *m_data, sizeof(double) * 2 * len)) {
        free(*m_data);
        *m_data = NULL;
        blast_err("Mismatched CRC for '%s'.  File corrupted?", m_filename);
        len = -1;
    }
    return len;
}

static void roach_caclulate_amps(roach_state_t *m_roach, double **m_mags,
                                 double **m_avgs, double **m_offsets, double **m_amps)
{
    double *tmp_data;
    int *channels;
    ssize_t chan_len;
    ssize_t vna_len;

    double *Is;
    double *Qs;

    if ((vna_len = roach_load_2d(m_roach->vna_path, &tmp_data)) <= 0) {
        blast_err("Could not VNA data from %s", m_roach->vna_path);
        *m_amps = NULL;
        return;
    }

    Is = tmp_data;
    Qs = Is + vna_len;

    if ((chan_len = roach_load_1d(m_roach->channels_path, (void**)&channels, sizeof(int))) <= 0) {
        blast_err("Could not load channels data from %s", m_roach->channels_path);
        free(tmp_data);
        *m_amps = NULL;
        return;
    }

    double *chan_mags = calloc(vna_len, sizeof(double));
    double *chan_avgs = calloc(chan_len, sizeof(double));
    double *offsets = calloc(chan_len, sizeof(double));
    double *amps = calloc(chan_len, sizeof(double));
    int sweep_len = vna_len / chan_len;

    for (int chan = 0; chan < chan_len; chan++) {
        double total_mag = 0.0;
        double total_offset = 0.0;
        for (int offset = 0; offset < sweep_len; offset++) {
            int i = offset + chan * sweep_len;
            chan_mags[i]= (10.0 * log10(sqrt(pow(Is[i], 2.0) + pow(Qs[i], 2.0))));
            total_mag += chan_mags[i];
            if (offset < (sweep_len / 2 - 1) || offset > (sweep_len / 2 + 1)) total_offset += chan_mags[i];
        }
        chan_avgs[chan] = total_mag / sweep_len;
        offsets[chan] = chan_avgs[chan] - total_offset / (sweep_len - 3);
        amps[chan] = 2.0 - sqrt(pow10(offsets[chan]/10.0));
    }

    *m_amps = amps;
    *m_avgs = chan_avgs;
    *m_mags = chan_mags;
    *m_offsets = offsets;
}

static void roach_select_bins(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int bins[fft_len];;
    double bin_freqs[fft_len];
    int last_bin = -1;
    int ch = 0;

    for (int i = 0; i < m_freqlen; i++) {
        bins[i] = roach_fft_bin_index(m_freqs, i, fft_len, dac_samp_freq);
        bin_freqs[i] = bins[i] * dac_samp_freq / fft_len;
        m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / m_roach->dac_freq_res)
                                        * m_roach->dac_freq_res;
    }
    for (int i = 0; i < m_freqlen; i++) {
        if (bins[i] == last_bin) continue;
        roach_write_int(m_roach, "bins", bins[i], 0);
        roach_write_int(m_roach, "load_bins", 2 * ch + 1, 0);
        roach_write_int(m_roach, "load_bins", 0, 0);
        ch++;
    }
    /**
     * Fill any remaining of the 1024 channelizer addresses with '0'
     */
    for (int i = ch; i < fft_len; i++) {
        roach_write_int(m_roach, "bins", 0, 0);
        roach_write_int(m_roach, "load_bins", 2 * ch + 1, 0);
        roach_write_int(m_roach, "load_bins", 0, 0);
        ch++;
    }
}

void roach_define_DDS_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_select_bins(m_roach, m_freqs, m_freqlen);

    if (m_roach->DDS.len > 0 && m_roach->DDS.len != m_roach->lut_buffer_len) {
        free(m_roach->DDS.I);
        free(m_roach->DDS.Q);
        m_roach->DDS.len = 0;
    }
    if (m_roach->DDS.len == 0) {
        m_roach->DDS.I = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DDS.Q = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DDS.len = m_roach->lut_buffer_len;
    }

    for (int i = 0; i < m_freqlen; i++) {
        double I[fft_len];
        double Q[fft_len];
        roach_freq_comb(m_roach, &m_roach->freq_residuals[i], 1,
                        fpga_samp_freq / (fft_len / 2), false, false,
                        I, Q);
        for (int j = i, k = 1; j < fft_len * fft_len; j += fft_len, k++) {
            m_roach->DDS.I[j] = I[k];
            m_roach->DDS.Q[j] = Q[k];
        }
    }
}

static void roach_define_DAC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    if (m_roach->DAC.len > 0 && m_roach->DAC.len != m_roach->lut_buffer_len) {
        free(m_roach->DAC.I);
        free(m_roach->DAC.Q);
        m_roach->DAC.len = 0;
    }
    if (m_roach->DAC.len == 0) {
        m_roach->DAC.I = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DAC.Q = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DAC.len = m_roach->lut_buffer_len;
    }
    roach_freq_comb(m_roach, m_freqs, m_freqlen,
                    dac_samp_freq, false, false,
                    m_roach->DAC.I, m_roach->DAC.Q);
}

void roach_pack_LUTs(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_define_DDS_LUT(m_roach, m_freqs, m_freqlen);
    roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);

    if (m_roach->LUT.len != 2 * m_roach->lut_buffer_len) {
        if (m_roach->LUT.len) {
            free(m_roach->LUT.I);
            free(m_roach->LUT.Q);
        }
        roach_init_LUT(m_roach, 2 * m_roach->lut_buffer_len);
    }

    for (size_t i = 0; i < m_roach->lut_buffer_len; i += 2) {
        m_roach->LUT.I[2 * i + 0] = htons(m_roach->DAC.I[i + 1]);
        m_roach->LUT.I[2 * i + 1] = htons(m_roach->DAC.I[i]);
        m_roach->LUT.I[2 * i + 2] = htons(m_roach->DDS.I[i + 1]);
        m_roach->LUT.I[2 * i + 3] = htons(m_roach->DDS.I[i]);
        m_roach->LUT.Q[2 * i + 0] = htons(m_roach->DAC.Q[i + 1]);
        m_roach->LUT.Q[2 * i + 1] = htons(m_roach->DAC.Q[i]);
        m_roach->LUT.Q[2 * i + 2] = htons(m_roach->DDS.Q[i + 1]);
        m_roach->LUT.Q[2 * i + 3] = htons(m_roach->DDS.Q[i]);
    }
}

void roach_write_QDR(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_pack_LUTs(m_roach, m_freqs, m_freqlen);
    roach_reset_dac(m_roach);
    roach_write_int(m_roach, "start_dac", 0, 0);
    roach_write_data(m_roach, "qdr0_memory", (uint8_t*)m_roach->LUT.I, m_roach->LUT.len * sizeof(uint16_t), 0);
    roach_write_data(m_roach, "qdr1_memory", (uint8_t*)m_roach->LUT.Q, m_roach->LUT.len * sizeof(uint16_t), 0);
    roach_write_int(m_roach, "start_dac", 1, 0);
}

void roach_vna_sweep(roach_state_t *m_roach, double m_centerfreq, const char *m_savepath)
{
    char fullpath[FILENAME_MAX];
    double *bb_freqs;
    double *rf_freqs;
    int *channels;

    double min_freq = -200e6;
    double max_freq = 200e6;
    size_t num_freqs = 1000;

    double delta_f = (max_freq - min_freq) / num_freqs;
    bb_freqs = calloc(num_freqs, sizeof(double));
    rf_freqs = calloc(num_freqs, sizeof(double));
    channels = calloc(num_freqs, sizeof(int));

    bb_freqs[0] = min_freq;
    for (size_t i = 1; i < num_freqs; i++) {
        bb_freqs[i] = bb_freqs[i - 1] + delta_f;
    }

    for (size_t i = 0; i < num_freqs; i++) {
        channels[i] = i;
        rf_freqs[i] = bb_freqs[i] + m_centerfreq;
    }

    snprintf(fullpath, sizeof(fullpath), "%s/last_bb_freqs.dat", m_savepath);
    roach_save_1d(fullpath, bb_freqs, sizeof(*bb_freqs), num_freqs);
    snprintf(fullpath, sizeof(fullpath), "%s/last_rf_freqs.dat", m_savepath);
    roach_save_1d(fullpath, rf_freqs, sizeof(*rf_freqs), num_freqs);
    snprintf(fullpath, sizeof(fullpath), "%s/last_channels.dat", m_savepath);
    roach_save_1d(fullpath, channels, sizeof(*channels), num_freqs);


    roach_write_QDR(m_roach, bb_freqs, num_freqs);
}

/**
 * If we have an error, we'll disable the socket and schedule a reconnection attempt.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void firmware_upload_process_return(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    firmware_state_t *state = (firmware_state_t*) m_data;

    /**
     * If we have an error, or do not receive data from the Roach in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & (PH_IOMASK_ERR)) {
        blast_err("disconnecting from firmware upload at %s due to connection issue", state->roach->address);
        state->result = ROACH_UPLOAD_RESULT_ERROR;
    } else if (m_why & PH_IOMASK_TIME) {
        blast_err("Timeout uploading firmware to %s", state->roach->address);
        state->result = ROACH_UPLOAD_RESULT_TIMEOUT;
    }

    if (!ph_bufq_len(m_sock->wbuf)) {
        blast_info("Successfully uploaded firmware to %s", state->roach->address);
        state->result = ROACH_UPLOAD_RESULT_SUCCESS;
    }

    ph_sock_enable(m_sock, 0);
    ph_sock_free(m_sock);
}

/**
 * Handle a connection callback.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our ROACH firmware upload state variable
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
    m_sock->callback = firmware_upload_process_return;
    m_sock->timeout_duration.tv_sec = 30;
    m_sock->job.data = state;

    /**
     * We have enabled the socket and now we buffer the firmware file into the network
     */
    {
        size_t number_bytes;
        ph_stream_t *firmware_stm = ph_stm_file_open(state->firmware_file, O_RDONLY, 0);
        if (!ph_stm_copy(firmware_stm, m_sock->stream, PH_STREAM_READ_ALL, NULL, &number_bytes)) {
            blast_err("Error getting data from %s: %s", state->firmware_file,
                      strerror(ph_stm_errno(firmware_stm)));
            ph_sock_shutdown(state->sock, PH_SOCK_SHUT_RDWR);
            ph_sock_enable(state->sock, false);
            ph_sock_free(state->sock);
            return;
        } else {
            blast_info("Loading %s with %zu bytes", state->firmware_file, number_bytes);
            ph_sock_enable(state->sock, true);
        }
        ph_stm_close(firmware_stm);
    }
}

int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename)
{
    firmware_state_t state = {
                              .firmware_file = m_filename,
                              .port = (uint16_t) (drand48() * 500.0 + 5000),
                              .timeout.tv_sec = 5,
                              .timeout.tv_usec = 0,
                              .roach = m_roach
    };

    int retval = send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                       KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?progremote",
                       KATCP_FLAG_ULONG | KATCP_FLAG_LAST, state.port,
                       NULL);
    if (retval != KATCP_RESULT_OK) {
        blast_err("Could not request upload port for ROACH firmware on %s!", m_roach->address);
        return -1;
    }
    ph_sock_resolve_and_connect(state.roach->address, state.port,
        &state.timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, firmware_upload_connected, &state);

    while (state.result == ROACH_UPLOAD_RESULT_WORKING) {
        usleep(1000);
    }

    if (state.result != ROACH_UPLOAD_RESULT_SUCCESS) return -1;

    return 0;
}

int init_roach(void)
{
    return 0;
}


