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
 *  Created on: July 7, 2015
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

#undef I
#define WRITE_INT_TIMEOUT 1000
#define UPLOAD_TIMEOUT 20000
#define QDR_TIMEOUT 20000
#define LUT_BUFFER_LEN 2097152
#define FPGA_SAMP_FREQ 256.0e6
#define DAC_FREQ_RES DAC_SAMP_FREQ / LUT_BUFFER_LEN

static double DAC_SAMP_FREQ = 512.0e6;
static int fft_len = 1024;
static double accum_len = (1 << 19) - 1;
// Test frequencies for troubleshooting (arbitrary values)
double freqs[2] = {10.3285242e6, -70.0832511e6};
size_t freqlen = 2;
// Firmware image files
const char roach_fpg[5][11] = {"roach1.fpg", "roach2.fpg", "roach3.fpg", "roach4.fpg", "roach5.fpg"};
const char test_fpg[] = "/data/etc/blast/roach2_8tap_wide_2016_Jun_25_2016.fpg";

static roach_state_t roach_state_table[NUM_ROACHES];

static ph_thread_t *roach_state = NULL;

static void roach_buffer_ntohs(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohs(m_buffer[i]);
    }
}

static void roach_buffer_ntohl(uint32_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
	m_buffer[i] = ntohl(m_buffer[i]);
    }
}

const char *roach_get_name(roach_state_t *m_roach)
{
    if (!m_roach || !m_roach->address) return "INVALID ROACH";
    return m_roach->address;
}

int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size, int ms_timeout)
{
    int retval = send_rpc_katcl(m_roach->rpc_conn, ms_timeout,
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
        char *ret = arg_string_katcl(m_roach->rpc_conn, 0);
        blast_err("Could not read data '%s' from %s: ROACH Error '%s'", m_register, m_roach->address,
            ret?ret:"");
	return -1;
    }

    if (arg_count_katcl(m_roach->rpc_conn) < 3) {
        blast_err("Expecting 2 return values.  Recevied %d", arg_count_katcl(m_roach->rpc_conn));
	return -1;
	}

    uint32_t bytes_copied = arg_buffer_katcl(m_roach->rpc_conn, 2, m_dest, m_size);

    if (bytes_copied != m_size) {
        blast_err("Expecting %ul bytes but only received %ul bytes", m_size, bytes_copied);
	return -1;
    }
    return 0;
}

int roach_read_int(roach_state_t *m_roach, const char *m_register)
{
  	uint32_t m_data;
	roach_read_data(m_roach, (uint8_t*) &m_data, m_register, 0, sizeof(m_data), 100);
	m_data = ntohl(m_data);
	blast_info("%s = %d", m_register, m_data);
	return 0;
}

int roach_write_data(roach_state_t *m_roach, const char *m_register, uint8_t *m_data,
					size_t m_len, uint32_t m_offset, int m_timeout)
{
    return send_rpc_katcl(m_roach->rpc_conn, m_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?write",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_BUFFER, m_data, m_len,
		   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_len, NULL);
}

int roach_write_int(roach_state_t *m_roach, const char *m_register, uint32_t m_val, uint32_t m_offset)
{
    uint32_t sendval = htonl(m_val);
    return roach_write_data(m_roach, m_register, (uint8_t*)&sendval, sizeof(sendval), m_offset, WRITE_INT_TIMEOUT);
}

static void roach_init_LUT(roach_state_t *m_roach, size_t m_len)
{
    m_roach->LUT.len = m_len;
    m_roach->LUT.I = calloc(m_len, sizeof(uint16_t));
    m_roach->LUT.Q = calloc(m_len, sizeof(uint16_t));
}

static void roach_init_DACDDS_LUTs(roach_state_t *m_roach, size_t m_len)
{
    m_roach->DAC.len = m_len;
    m_roach->DAC.I = calloc(m_len, sizeof(double));
    m_roach->DAC.Q = calloc(m_len, sizeof(double));
    m_roach->DDS.len = m_len;
    m_roach->DDS.I = calloc(m_len, sizeof(double));
    m_roach->DDS.Q = calloc(m_len, sizeof(double));
}

static inline int roach_fft_bin_index(double *m_freqs, size_t m_index, size_t m_fft_len, double m_samp_freq)
{
	return (int)lround(m_freqs[m_index] / m_samp_freq *  m_fft_len);
}

// Builds DAC frequency comb

static int roach_dac_comb(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen,
                            int m_samp_freq, double *m_I, double *m_Q)
{
    size_t comb_fft_len;
    fftw_plan comb_plan;

    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
    }

    comb_fft_len = LUT_BUFFER_LEN;
    complex double *spec = calloc(comb_fft_len,  sizeof(complex double));
    complex double *wave = calloc(comb_fft_len,  sizeof(complex double));
    double max_val = 0.0;
    srand48(time(NULL));
    for (size_t i = 1; i < m_freqlen; i++) {
    	int bin = roach_fft_bin_index(m_freqs, i, comb_fft_len, m_samp_freq);
	if (bin < 0) {
		bin += comb_fft_len;
	}
	spec[bin] = cexp(_Complex_I * drand48() * 2.0 * M_PI);
    }
    	/*FILE *f1 = fopen("./dac_spec.txt", "w");
    	for (size_t i = 0; i < comb_fft_len; i++){
	fprintf(f1,"%f, %f\n", creal(spec[i]), cimag(spec[i])); 
    	} 
     	fclose(f1); 
    FILE *f2 = fopen("./dac_wave.txt", "w");*/
    comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(comb_plan);
    fftw_destroy_plan(comb_plan);
    /*for (size_t i = 0; i < comb_fft_len; i++) {
    fprintf(f2,"%f, %f\n", creal(wave[i]), cimag(wave[i])); 
        if (cabs(wave[i]) > max_val) max_val = cabs(wave[i]);
    }
    
    fclose(f2);*/
    for (size_t i = 0; i < comb_fft_len; i++) {
    	m_I[i] = creal(wave[i]) / max_val * ((1<<15)-1);
        m_Q[i] = cimag(wave[i]) / max_val * ((1<<15)-1);
    }
    free(spec);
    free(wave);
    return 0;
}

// Build DDS LUT

static int roach_dds_comb(roach_state_t *m_roach, double m_freqs, size_t m_freqlen,
                            int m_samp_freq, int m_bin, double *m_I, double *m_Q)
{
	size_t comb_fft_len;
    	fftw_plan comb_plan;

	comb_fft_len = LUT_BUFFER_LEN / fft_len;
    	complex double *spec = calloc(comb_fft_len,  sizeof(complex double));
    	complex double *wave = calloc(comb_fft_len,  sizeof(complex double));
    	double max_val = 0.0;
	spec[m_bin] = cexp(_Complex_I * 0.);
/*	FILE *f3 = fopen("./dds_spec.txt", "w");
	for (size_t i = 0; i < comb_fft_len; i++){
	fprintf(f3,"%f, %f\n", creal(spec[i]), cimag(spec[i]));
     	 }	
	fclose(f3); 

    	FILE *f4 = fopen("./dds_wave.txt", "w");*/
    	comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    	fftw_execute(comb_plan);
    	fftw_destroy_plan(comb_plan);
	for (size_t i = 0; i < comb_fft_len; i++) {
		// fprintf(f4,"%f, %f\n", creal(wave[i]), cimag(wave[i]));
        	if (cabs(wave[i]) > max_val) max_val = cabs(wave[i]);
	}
	// fclose(f4);
    	for (size_t i = 0; i < comb_fft_len; i++) {
		m_I[i] = creal(wave[i]) / max_val * ((1<<15)-1);
        	m_Q[i] = cimag(wave[i]) / max_val * ((1<<15)-1);
    	}
    	free(spec);
    	free(wave);
    	return 0;
}

static void roach_define_DAC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    if (m_roach->DAC.len > 0 && m_roach->DAC.len != LUT_BUFFER_LEN) {
	free(m_roach->DAC.I);
        free(m_roach->DAC.Q);
        m_roach->DAC.len = 0;
    }
    if (m_roach->DAC.len == 0) {
        m_roach->DAC.I = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DAC.Q = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DAC.len = LUT_BUFFER_LEN;
    }
    roach_dac_comb(m_roach, m_freqs, m_freqlen,
                    DAC_SAMP_FREQ, m_roach->DAC.I, m_roach->DAC.Q);
}

static void roach_select_bins(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int bins[fft_len];
    double bin_freqs[fft_len];

    m_roach->freq_residuals = malloc(m_freqlen * sizeof(size_t));
    for (size_t i = 0; i < m_freqlen; i++) {
	bins[i] = roach_fft_bin_index(m_freqs, i, fft_len, DAC_SAMP_FREQ);
	bin_freqs[i] = bins[i] * DAC_SAMP_FREQ / fft_len;
	if (bins[i] < 0) {
		bins[i] += 1024;
	}
	if (m_freqs[i] < 0 && m_freqs[i]+ 512.0e6 >= 511.5e6) {
		bins[i] = 1023;
	}

	m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / DAC_FREQ_RES) * DAC_FREQ_RES;
    }
    for (int ch = 0; ch < m_freqlen; ch++) {
        roach_write_int(m_roach, "bins", bins[ch], 0);
        roach_write_int(m_roach, "load_bins", 2*ch + 1, 0);
        roach_write_int(m_roach, "load_bins", 0, 0);
    }
}

void roach_define_DDS_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_select_bins(m_roach, m_freqs, m_freqlen);
    if (m_roach->DDS.len > 0 && m_roach->DDS.len != LUT_BUFFER_LEN) {
        free(m_roach->DDS.I);
        free(m_roach->DDS.Q);
        m_roach->DDS.len = 0;
    }
    if (m_roach->DDS.len == 0) {
        m_roach->DDS.I = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DDS.Q = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DDS.len = LUT_BUFFER_LEN;
    }

    for (size_t i = 0; i < m_freqlen; i++) {
	double I[2 * fft_len];
    	double Q[2 * fft_len];
	int bin = roach_fft_bin_index(m_roach->freq_residuals,
		i, LUT_BUFFER_LEN / fft_len, FPGA_SAMP_FREQ / (fft_len / 2));
	if (bin < 0) {
		bin += 2048;
	}
	roach_dds_comb(m_roach, m_roach->freq_residuals[i], m_freqlen,
                        FPGA_SAMP_FREQ / (fft_len / 2), bin, I, Q);
	for (int j = i, k = 0; k < 2*fft_len; j += fft_len, k++) {
		m_roach->DDS.I[j] = I[k];
        	m_roach->DDS.Q[j] = Q[k];
    	}
    }
}

void roach_pack_LUTs(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    	roach_define_DDS_LUT(m_roach, m_freqs, m_freqlen);
    	roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);
// Commented section below checks to see if space is allocated. Needs error checking.
   /* if (m_roach->LUT.len > 0 && m_roach->LUT.len != LUT_BUFFER_LEN)  {
    	blast_info("Attempting to free luts\t");
	free(m_roach->LUT.I);
        free(m_roach->LUT.Q);
    }*/
	roach_init_LUT(m_roach, 2 * LUT_BUFFER_LEN);
	for (size_t i = 0; i < LUT_BUFFER_LEN; i += 2) {
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
// Useful for error checking LUTs, NOT NEEDED FOR FLIGHT
void save_luts(roach_state_t *m_roach)
{
	FILE *f1 = fopen("./DACI.txt", "w");
    	FILE *f2 = fopen("./DACQ.txt", "w");
	FILE *f3 = fopen("./DDSI.txt", "w");
	FILE *f4 = fopen("./DDSQ.txt", "w");
	for (size_t i = 0; i < LUT_BUFFER_LEN; i++) {
    		fprintf(f1, "%g\n", m_roach->DAC.I[i]);
		fprintf(f2, "%g\n", m_roach->DAC.Q[i]);
    		fprintf(f3, "%g\n", m_roach->DDS.I[i]);
    		fprintf(f4, "%g\n", m_roach->DDS.Q[i]);
    	}
    	fclose(f1);
    	fclose(f2);
    	fclose(f3);
    	fclose(f4);
	blast_info("LUTs written to disk\n");
}

// Useful for error checking LUTs, NOT NEEDED FOR FLIGHT
void save_packed_luts(roach_state_t *m_roach)
{
	FILE *f1 = fopen("./QDRI.txt", "w");
	FILE *f2 = fopen("./QDRQ.txt", "w");
	for (size_t i = 0; i < 2 * LUT_BUFFER_LEN; i++) {
    		fprintf(f1, "%d\n", ntohs(m_roach->LUT.I[i]));
    		fprintf(f2, "%d\n", ntohs(m_roach->LUT.Q[i]));
    	}
    	fclose(f1);
    	fclose(f2);
}

void roach_write_QDR(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{   roach_pack_LUTs(m_roach, m_freqs, m_freqlen);
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
    roach_write_int(m_roach, "start_dac", 0, 0);
    if (roach_write_data(m_roach, "qdr0_memory", (uint8_t*)m_roach->LUT.I,
    		m_roach->LUT.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
		blast_info("Could not write to qdr0!");
	}
    if (roach_write_data(m_roach, "qdr1_memory", (uint8_t*)m_roach->LUT.Q,
    		m_roach->LUT.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
		blast_info("Could not write to qdr1!");
    	}
    sleep(0.3);
    roach_write_int(m_roach, "start_dac", 1, 0);
    roach_write_int(m_roach, "sync_accum_reset", 0, 0);
    roach_write_int(m_roach, "sync_accum_reset", 1, 0);
}

void roach_write_tones(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
	blast_info("Allocating memory for LUTs, ROACH%d...", m_roach->which);
	roach_init_DACDDS_LUTs(m_roach, LUT_BUFFER_LEN);
	blast_info("Defining DAC LUT, ROACH%d...", m_roach->which);
	roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);
	blast_info("Defining DDS LUT, ROACH%d...", m_roach->which);
	roach_define_DDS_LUT(m_roach, m_freqs, m_freqlen);
	blast_info("Uploading LUTs to QDR RAM on ROACH%d...", m_roach->which);
	roach_write_QDR(m_roach, m_freqs, m_freqlen);
}

static int roach_read_QDR(roach_state_t *m_roach, uint16_t *m_qdr_I, uint16_t *m_qdr_Q)
{
    size_t buffer_len = (1<<22);
    int16_t *temp_data_I;
    int16_t *temp_data_Q;
    temp_data_I = calloc(sizeof(int16_t), buffer_len);
    if (roach_read_data(m_roach, (uint8_t*)temp_data_I, "qdr0_memory", 0, buffer_len * sizeof(uint16_t), QDR_TIMEOUT)) {
        free(temp_data_I);
        return -1;
    }
    temp_data_Q = calloc(sizeof(int16_t), buffer_len);
    if (roach_read_data(m_roach, (uint8_t*)temp_data_Q, "qdr1_memory", 0, buffer_len * sizeof(uint16_t), QDR_TIMEOUT)) {
        free(temp_data_Q);
        return -1;
    }
    roach_buffer_ntohs((uint16_t*)temp_data_I, buffer_len);
    roach_buffer_ntohs((uint16_t*)temp_data_Q, buffer_len);
    for (size_t i = 0; i < buffer_len; i++) {
    	m_qdr_I[i] = temp_data_I[i];
    	m_qdr_Q[i] = temp_data_Q[i];
    }
    free(temp_data_I);
    free(temp_data_Q);
    return 0;
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

// Not currently being used, may change or eliminate
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

// UDP Socket, replace with phenom socket
int init_socket(void)
{
	int rawsock;
	struct sockaddr saddr;
	int saddr_len = sizeof(saddr);
	if ((rawsock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_IP))) < 0) {
		perror("cannot create socket\n");
		return -1;
	}
	struct sockaddr_ll sll;
	struct ifreq ifr;
	bzero(&sll, sizeof(sll));
	bzero(&ifr, sizeof(ifr));
	/* First Get the Interface Index */
	strncpy((char *)ifr.ifr_name, "eth0", IFNAMSIZ);
	if((ioctl(rawsock, SIOCGIFINDEX, &ifr)) == -1) {
		perror("Error getting Interface index !\n");
		exit(-1);
	}
	/* Bind our raw socket to this interface */
	sll.sll_family = AF_PACKET;
	sll.sll_ifindex = ifr.ifr_ifindex;
	sll.sll_protocol = htons(ETH_P_ALL);
	if((bind(rawsock, (struct sockaddr *)&sll, sizeof(sll)))== -1) {
		perror("Error binding raw socket to interface\n");
		exit(-1);
	}
	return rawsock;
}

int fill_packet_buffer(data_packet_t *m_packet, int m_sock_desc)
{
	int buflen;
	// m_packet->rcv_buffer = (unsigned char *) malloc(8234);
	m_packet->rcv_buffer = calloc(8234, sizeof(uint8_t));
	memset(m_packet->rcv_buffer, 0, 8234);
	buflen = recvfrom(m_sock_desc, m_packet->rcv_buffer, 8234, 0, NULL, NULL);
	if(buflen < 0) {
		blast_info("error in reading recvfrom function\n");
		return -1;
	}
	// blast_info("Received %d bytes\n", buflen);
	return 0;
}

// Revisit this function
void parse_packet(data_packet_t *m_packet)
{
	m_packet->I = calloc(1024, sizeof(float));
        m_packet->Q = calloc(1024, sizeof(float));
	uint8_t *payload = (uint8_t *)(m_packet->rcv_buffer);
	uint8_t *data = (payload + 42);
	m_packet->checksum = (data[8176] << 24) | (data[8177] << 16) | (data[8178] << 8) | data[8179];
	m_packet->pps_count = (data[8180] << 24) | (data[8181] << 16) | (data[8182] << 8) | data[8183];
	m_packet->clock_count = (data[8184] << 24) | (data[8185] << 16) | (data[8186] << 8) | data[8187];
	m_packet->packet_count = (data[8188] << 24) | (data[8189] << 16) | (data[8190] << 8) | data[8191];
	// I, Q
	for (int i = 0;	i < 1024; i += 1) {
		int j;
		int k;
		if ((i % 2) == 0) {
			j = ((i*4) /2);
			k = 512*4 + ((i*4)/2);
		} else {
			j = 1024*4 + (((i*4) - 1) / 2) - 1;
			k = 1536*4 + (((i*4) - 1) / 2) - 1;
		}
		m_packet->I[i] = (float)(ntohl((data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3])));
		m_packet->Q[i] = (float)(ntohl((data[k] << 24) | (data[k + 1] << 16) | (data[k + 2] << 8) | (data[k + 3])));
		// blast_info("%d\t %d\t %d\t %d\t %d\t\n", i, j, j + 1, j + 2, j + 3);
		// blast_info("%d\t %d\t %d\t %d\t %d\t\n", i, k, k + 1, k + 2, k + 3);
	}
}

// streams packets for troubleshooting, DOES NOT SAVE DATA
int stream_packets(size_t m_num_packets, int m_socket_desc, int m_chan)
{
	for (size_t i = 0; i < m_num_packets; i++) {
		data_packet_t m_packet;
		fill_packet_buffer(&m_packet, m_socket_desc);
		parse_packet(&m_packet);
		fflush(stdout);
		blast_info("%u\t", m_packet.packet_count);
		blast_info("%d\t", m_chan);
		blast_info("%f\t%f\t", m_packet.I[m_chan], m_packet.Q[m_chan]);
		double chan_phase = atan2((double)m_packet.Q[m_chan], (double)m_packet.I[m_chan]);
		blast_info("%g\t\n", chan_phase);
		free(m_packet.I);
		free(m_packet.Q);
	}
	return 0;
}

void write_packet(data_packet_t *m_packet, FILE *m_fd, int m_sock_desc)
{
	int rc = fill_packet_buffer(m_packet, m_sock_desc);
	m_packet->eth  = (struct ethhdr *)(m_packet->rcv_buffer);
	fprintf(m_fd, "Source MAC : %.2X-%.2X-%.2X-%.2X-%.2X-%.2X\n", m_packet->eth->h_source[0],
	m_packet->eth->h_source[1], m_packet->eth->h_source[2], m_packet->eth->h_source[3],
	m_packet->eth->h_source[4], m_packet->eth->h_source[5]);
	uint16_t iphdrlen;
	struct sockaddr_in source, dest;
	m_packet->ip = (struct iphdr *)(m_packet->rcv_buffer + sizeof(struct ethhdr));
	memset(&source, 0, sizeof(source));
	source.sin_addr.s_addr = m_packet->ip->saddr;
	memset(&dest, 0, sizeof(dest));
	dest.sin_addr.s_addr = m_packet->ip->daddr;
	fprintf(m_fd, "Version : %d\n", (unsigned int)m_packet->ip->version);
	fprintf(m_fd, "Internet Header Length : %d DWORDS or %d Bytes\n",
	(unsigned int)m_packet->ip->ihl, ((unsigned int)(m_packet->ip->ihl))*4);
	fprintf(m_fd, "TOS : %d\n", (unsigned int)m_packet->ip->tos);
	fprintf(m_fd, "Total Length : %d Bytes\n", ntohs(m_packet->ip->tot_len));
	fprintf(m_fd, "ID : %d\n", ntohs(m_packet->ip->id));
	fprintf(m_fd, "TTL : %d\n", (unsigned int)m_packet->ip->ttl);
	fprintf(m_fd, "Protocol : %d\n", (unsigned int)m_packet->ip->protocol);
	fprintf(m_fd, "Header Checksum : %d\n", ntohs(m_packet->ip->check));
	fprintf(m_fd, "Source IP : %s\n", inet_ntoa(source.sin_addr));
	fprintf(m_fd, "Dest IP : %s\n\n", inet_ntoa(dest.sin_addr));
	parse_packet(m_packet);
	fprintf(m_fd, "Firmware Checksum : %u\n", m_packet->checksum);
	fprintf(m_fd, "PPS Count : %u\n", m_packet->pps_count);
	fprintf(m_fd, "Clock Count : %u\n", m_packet->clock_count);
	fprintf(m_fd, "Packet Num : %u\n\n", m_packet->packet_count);
	for (int i = 0; i < 1024; i++) {
		fprintf(m_fd, "%d, %f\t, %f\n", i, m_packet->I[i], m_packet->Q[i]);
	}
	free(m_packet->I);
	free(m_packet->Q);
}

void save_packets(size_t m_num_packets, int m_sock_desc, double m_filetag,
				const char *m_savepath, const char *m_pathtag)
{
	char fname[FILENAME_MAX];
	for (size_t i = 0; i < m_num_packets; i++) {
		data_packet_t packet;
		snprintf(fname, sizeof(fname), "%s/%s/p%3g.dat", m_savepath, m_pathtag, m_filetag);
		FILE *fd = fopen(fname, "w");
		write_packet(&packet, fd, m_sock_desc);
		fclose(fd);
	}
}

void roach_freq_comb(roach_state_t *m_roach)
{
	size_t m_freqlen = 300;
	double p_max_freq = 255.021234e6;
	double p_min_freq = 5.2342e6;
	double n_max_freq = -5.2342e6+5.0e4;
	double n_min_freq = -255.021234e6+5.0e4;
	m_roach->freq_comb = calloc(m_freqlen, sizeof(double));
	m_roach->freqlen = m_freqlen;
	/* positive freqs */
	double p_delta_f = (n_max_freq - n_min_freq) / ((m_freqlen/2) - 1);
	for (size_t i = m_freqlen/2; i-- > 0;) {
		m_roach->freq_comb[m_freqlen/2 - (i + 1)] = p_max_freq - i*p_delta_f;
	}
	/* negative freqs */
	double n_delta_f = (n_max_freq - n_min_freq) / ((m_freqlen/2) - 1);
	for (size_t i = 0; i < m_freqlen/2; i++) {
		m_roach->freq_comb[i + m_freqlen/2] = n_min_freq + i*n_delta_f;
	}
	for (size_t i = 0; i < m_freqlen; i++) {
	}
}

// Valon commands should be switched to C, Beaglebone/Remote Serial
void sweep_lo(double m_centerfreq, double m_span, double delta_f, int m_sock_desc,
				bool save, const char *m_savepath, const char *m_filetag)
{
	double min_freq = m_centerfreq - (m_span/2);
	double max_freq = m_centerfreq + (m_span/2);
	double *sweep_freqs;
	double megahertz = 1.0e6;
	size_t num_freqs = (max_freq - min_freq) / delta_f;
	char command[FILENAME_MAX];
	sweep_freqs = calloc(num_freqs, sizeof(double));
	sweep_freqs[0] = min_freq;
	for (size_t i = 1; i < num_freqs; i++) {
		sweep_freqs[i] = sweep_freqs[i - 1] + delta_f;
	}
	for (size_t i = 0; i < num_freqs; i++) {
		snprintf(command, sizeof(command), "python /data/etc/blast/set_lo.py %g", sweep_freqs[i]/megahertz);
		fflush(stdout);
		system(command);
		blast_info("LO @ %g\n", sweep_freqs[i]);
		if (save) {
			save_packets(1, m_sock_desc, sweep_freqs[i], m_savepath, m_filetag);
		}
		snprintf(command, sizeof(command), "python /data/etc/blast/set_lo.py %g", m_centerfreq/megahertz);
		system(command);
	}
	free(sweep_freqs);
}

// Revisit, may have too many arguments
void roach_sweep(roach_state_t *m_roach, double m_centerfreq, const char *m_savepath,
		const char *m_packetdir, int m_sock_desc, bool vna, bool write, bool plot)
{
	if (vna) {
		char fullpath[FILENAME_MAX];
		double *rf_freqs;
		int *channels;
		rf_freqs = calloc(m_roach->freqlen, sizeof(double));
		channels = calloc(m_roach->freqlen, sizeof(int));
		roach_freq_comb(m_roach);
		for (size_t i = 0; i < m_roach->freqlen; i++) {
			channels[i] = i;
			rf_freqs[i] = m_roach->freq_comb[i] + m_centerfreq;
		}
		snprintf(fullpath, sizeof(fullpath), "%s/last_bb_freqs.dat", m_savepath);
		roach_save_1d(fullpath, m_roach->freq_comb, sizeof(*m_roach->freq_comb), m_roach->freqlen);
		snprintf(fullpath, sizeof(fullpath), "%s/last_rf_freqs.dat", m_savepath);
		roach_save_1d(fullpath, rf_freqs, sizeof(*rf_freqs), m_roach->freqlen);
		snprintf(fullpath, sizeof(fullpath), "%s/last_channels.dat", m_savepath);
		roach_save_1d(fullpath, channels, sizeof(*channels), m_roach->freqlen);
		if (write) {
			roach_write_tones(m_roach, m_roach->freq_comb, m_roach->freqlen);
		}
		sweep_lo(m_centerfreq, 5.0e5, 5.0e3, m_sock_desc, 1, m_savepath, m_packetdir);
	} else {
		FILE *m_fd;
		m_fd = fopen("/data/etc/blast/kid_freqs.dat", "r");
		double m_kid_freqs[m_roach->num_kids];
		size_t i = 0;
		while(!feof(m_fd)) {
			fscanf(m_fd, "%lg\n", &m_kid_freqs[i]);
			blast_info("%g\n", m_kid_freqs[i]);
			// m_roach->kid_freqs[i] = m_kid_freqs[i];
			i++;
		}
		fclose(m_fd);
		// reorder frequencies
		if (write) {
			roach_write_tones(m_roach, m_kid_freqs, m_roach->num_kids);
		}
		sweep_lo(m_centerfreq, 1.0e5, 2.5e3, m_sock_desc, 1, m_savepath, m_packetdir);
	}
	if (plot) {
		char command[FILENAME_MAX];
		snprintf(command, sizeof(command), "python -i /data/etc/blast/plot_sweep.py %s/%s", m_savepath, m_packetdir);
		blast_info("%s\n", command);
		fflush(stdout);
		system(command);
	}
}

// Need to replace with C, locates resonances and stores new array of frequencies
void get_kid_freqs(roach_state_t *m_roach, const char *m_packetpath)
{
	// char fullpath[FILENAME_MAX];
	char command[FILENAME_MAX];
	snprintf(command, sizeof(command), "python -i /data/etc/blast/find_kids_blast.py %s", m_packetpath);
	blast_info("%s\n", command);
	fflush(stdout);
	system(command);
	// size_t num_freqs = sizeof(kid_freqs)/sizeof(double);
	// snprintf(fullpath, sizeof(fullpath), "%s/last_kid_freqs.dat", m_savepath);
	// blast_info("Wrote %s\n", fullpath);
	// roach_save_1d(fullpath, kid_freqs, sizeof(*kid_freqs), num_freqs);
}

/** Phenom/Callback functions to upload firmware**/
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
    } else {
        blast_err("Write buffer not empty: %i", ph_bufq_len(m_sock->wbuf));
        return;
    }
    ph_sock_enable(m_sock, 0);
    ph_sock_free(m_sock);
}

static void firmware_upload_connected(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    firmware_state_t *state = (firmware_state_t*) m_data;
    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", state->roach->address, state->port, gai_strerror(m_errcode));
            state->result = ROACH_UPLOAD_RESULT_ERROR;
            return;
        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->roach->address, state->port, m_errcode, strerror(m_errcode));
            state->result = ROACH_UPLOAD_CONN_REFUSED;
            return;
    }
    blast_info("Connected to ROACH at %s:%u", state->roach->address, state->port);
    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);
    state->sock = m_sock;
    m_sock->callback = firmware_upload_process_return;
    m_sock->timeout_duration.tv_sec = 10;
    m_sock->job.data = state;
    /**
     * We have enabled the socket and now we buffer the firmware file into the network
     */
    {
        size_t number_bytes;
	ph_stream_t *firmware_stm = ph_stm_file_open(state->firmware_file, O_RDONLY, 0);
        state->result = ROACH_UPLOAD_RESULT_WORKING;
        if (!ph_stm_copy(firmware_stm, m_sock->stream, PH_STREAM_READ_ALL, NULL, &number_bytes)) {
            blast_err("Error getting data from %s: %s", state->firmware_file,
                      strerror(ph_stm_errno(firmware_stm)));
            ph_sock_shutdown(state->sock, PH_SOCK_SHUT_RDWR);
            ph_sock_enable(state->sock, false);
            ph_sock_free(state->sock);
            state->result = ROACH_UPLOAD_RESULT_ERROR;
            return;
        } else {
            blast_info("Loading %s with %zu bytes", state->firmware_file, number_bytes);
	    ph_sock_enable(state->sock, true);
        }
	ph_stm_close(firmware_stm);
    }
}

int roach_upload_status(roach_state_t *m_roach)
{
    int success_val = send_rpc_katcl(m_roach->rpc_conn, 1000,
    	KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?fpgastatus",
	KATCP_FLAG_LAST | KATCP_FLAG_STRING, "",
	NULL);
    while (success_val != KATCP_RESULT_OK) {
    	char *ret = arg_string_katcl(m_roach->rpc_conn, 1);
	blast_info("**********%s", ret);
	usleep(1000);
    }
    char *ret = arg_string_katcl(m_roach->rpc_conn, 1);
    blast_info("FPGA %s", ret);
    return 0;
}

int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename)
{
    srand48(time(NULL));
    firmware_state_t state = {
                              .firmware_file = m_filename,
                              .port = (uint16_t) (drand48() * 500.0 + 5000),
                              .timeout.tv_sec = 20,
                              .timeout.tv_usec = 0,
                              .roach = m_roach
    };
    int retval = send_rpc_katcl(m_roach->rpc_conn, UPLOAD_TIMEOUT,
                       KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?progremote",
                       KATCP_FLAG_ULONG | KATCP_FLAG_LAST, state.port,
                       NULL);
    if (retval != KATCP_RESULT_OK) {
        blast_err("Could not request upload port for ROACH firmware on %s!", m_roach->address);
        return -1;
    }
    for (int loop = 0; loop < 2; loop++) {
        ph_sock_resolve_and_connect(state.roach->address, state.port, 0,
            &state.timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, firmware_upload_connected, &state);
        while (state.result == ROACH_UPLOAD_RESULT_WORKING) {
            usleep(1000);
        }
        if (state.result != ROACH_UPLOAD_CONN_REFUSED) break;
	usleep(100000);
    }
    if (state.result != ROACH_UPLOAD_RESULT_SUCCESS) return -1;
    if (state.result = ROACH_UPLOAD_RESULT_SUCCESS) {
	sleep(3);
	roach_upload_status(m_roach);
    return 0;
    }
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

void shutdown_roaches(void)
{
	for (int i = 0; i < NUM_ROACHES; i++) {
		if (roach_state_table[i].katcp_fd > 0) {
			destroy_rpc_katcl(roach_state_table[i].katcp_fd);
			blast_info("Closing KATCP on ROACH%d", i + 1);
		}
	}
}

void *roach_cmd_loop(void)
{
	char tname[] = "rchcmd";
	ph_thread_set_name(tname);
	nameThread(tname);
	blast_info("Starting Roach Commanding Thread");
	for (int i = 0; i < NUM_ROACHES; i++) {
		roach_state_table[i].status = ROACH_STATUS_BOOT;
		roach_state_table[i].desired_status = ROACH_STATUS_STREAMING;
	}
	while (!shutdown_mcp) {
		// TODO(SAM/LAURA): Fix Roach 1/Add error handling
		char *cal_command;
		for (int i = 1; i < 2; i++) {
		// for (int i = 0; i < NUM_ROACHES; i++) {
			if (roach_state_table[i].status == ROACH_STATUS_BOOT && roach_state_table[i].desired_status > ROACH_STATUS_BOOT) {
				blast_info("Attempting to connect to %s", roach_state_table[i].address);
				roach_state_table[i].katcp_fd = net_connect(roach_state_table[i].address,
					0, NETC_VERBOSE_ERRORS | NETC_VERBOSE_STATS);
				blast_info("fd:%d ", roach_state_table[i].katcp_fd);
				roach_state_table[i].rpc_conn = create_katcl(roach_state_table[i].katcp_fd);
				if (roach_state_table[i].katcp_fd > 0) {
					roach_state_table[i].status = ROACH_STATUS_CONNECTED;
				blast_info("KATCP up on ROACH%d", i + 1);
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_CONNECTED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_PROGRAMMED) {
				blast_info("Uploading firmware to ROACH%d", i + 1);
				if (roach_upload_fpg(&roach_state_table[i], test_fpg) == 0) {
					roach_state_table[i].status = ROACH_STATUS_PROGRAMMED;
					roach_state_table[i].desired_status = ROACH_STATUS_CONFIGURED;
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_PROGRAMMED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_CONFIGURED) {
				blast_info("Configuring Valon and software registers on ROACH%d", i + 1);
				system("sudo python /data/etc/blast/set_valon.py");
				roach_write_int(&roach_state_table[i], "dds_shift", 305, 0);/* DDS LUT shift, in clock cycles */
				roach_read_int(&roach_state_table[i], "dds_shift");
				roach_write_int(&roach_state_table[i], "fft_shift", 255, 0);/* FFT shift schedule */
				roach_read_int(&roach_state_table[i], "fft_shift");
				roach_write_int(&roach_state_table[i], "sync_accum_len", 1048575, 0);/* Number of accumulations */
				roach_read_int(&roach_state_table[i], "sync_accum_len");
				roach_write_int(&roach_state_table[i], "tx_destip", dest_ip, 0);/* UDP destination IP */
				roach_write_int(&roach_state_table[i], "tx_destport", roach_state_table[i].dest_port, 0); /* UDP port */
				roach_state_table[i].status = ROACH_STATUS_CONFIGURED;
				roach_state_table[i].desired_status = ROACH_STATUS_CALIBRATED;
			}
			if (roach_state_table[i].status == ROACH_STATUS_CONFIGURED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_CALIBRATED) {
				roach_write_int(&roach_state_table[i], "dac_reset", 1, 0);
				// TODO(Sam/Laura): replace with C function?
				blast_info("Calibrating QDRs... ");
				// qdr_cal2(&roach_state_table[i], 0);
				asprintf(&cal_command, "python /data/etc/blast/cal_roach_qdr.py %s", roach_state_table[i].address);
				if (system(cal_command) > -1) {
					blast_info("Calibration completed on ROACH%d", i + 1);
					roach_write_int(&roach_state_table[i], "tx_rst", 0, 0);
					roach_write_int(&roach_state_table[i], "tx_rst", 1, 0);
					roach_write_int(&roach_state_table[i], "tx_rst", 0, 0);
					roach_write_int(&roach_state_table[i], "pps_start", 1, 0);
					roach_state_table[i].status = ROACH_STATUS_CALIBRATED;
					roach_state_table[i].desired_status = ROACH_STATUS_TONE;
				} else {
					blast_info("Calibration failed on ROACH%d", i + 1);
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_CALIBRATED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_TONE) {
				blast_info("Generating frequency comb for ROACH%d", i + 1);
				roach_freq_comb(&roach_state_table[i]);
				roach_write_tones(&roach_state_table[i], roach_state_table[i].freq_comb, roach_state_table[i].freqlen);
				blast_info("Frequency comb written to ROACH%d", i + 1);
				roach_state_table[i].status = ROACH_STATUS_TONE;
				roach_state_table[i].desired_status = ROACH_STATUS_STREAMING;
			}
			/*if (roach_state_table[i].status == ROACH_STATUS_TONE &&
				roach_state_table[i].desired_status >= ROACH_STATUS_STREAMING) {
				// TODO(Sam): Streaming starts automatically if write is successful -> 
				// Find a way to verify that packets are streaming
			}	
			if (roach_state_table[i].status == ROACH_STATUS_STREAMING &&
				roach_state_table[i].desired_status >= ROACH_STATUS_VNA) {
				blast_info("ROACH%d, starting VNA sweep", i + 1);
				roach_sweep(&roach_state_table[i], -225.1213e6, 225.21234e6, 80, 750e6, roach_state_table[i].vna_path, "test", sock, 1, 0, 1);
			}
			if (roach_state_table[i].status == ROACH_STATUS_VNA &&
				roach_state_table[i].desired_status >= ROACH_STATUS_ARRAY_FREQS) {
			
			}
			if (roach_state_table[i].status == ROACH_STATUS_ARRAY_FREQS &&
				roach_state_table[i].desired_status >= ROACH_STATUS_TARG) {
			
			}
			if (roach_state_table[i].status == ROACH_STATUS_ARRAY_FREQS &&
				roach_state_table[i].desired_status >= ROACH_STATUS_ACQUIRING) {
			
			}*/
		}
	}
}

int init_roach(void)
{
    memset(roach_state_table, 0, sizeof(roach_state_t) * NUM_ROACHES);
    for (int i = 0; i < NUM_ROACHES; i++) {
    	 asprintf(&roach_state_table[i].address, "roach%d", i + 1);
    	 // asprintf(&roach_state_table[i].vna_path, "/data/etc/blast/r%d/vna", i + 1);
    	 // asprintf(&roach_state_table[i].targ_path, "/data/etc/blast/r%d/targ", i + 1);
    	 asprintf(&roach_state_table[i].vna_path, "/home/lazarus/iqstream/r%d/vna", i + 1);
    	 asprintf(&roach_state_table[i].targ_path, "/home/lazarus/iqstream/r%d/targ", i + 1);
    	 roach_state_table[i].which = i + 1;
    	 roach_state_table[i].dest_port = 60000 + i;
	    roach_udp_networking_init(i, &roach_state_table[i], NUM_ROACH_UDP_CHANNELS);
	}

    ph_thread_t *roach_cmd_thread = ph_thread_spawn(roach_cmd_loop, NULL);
    return 0;
}

