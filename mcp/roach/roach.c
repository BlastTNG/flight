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
#include <inttypes.h>
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
#include <dirent.h>
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
#include "command_struct.h"
#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#define DDS_SHIFT 305 /* Firmware version dependent */
#define FFT_SHIFT 127 /* Controls FFT overflow behavior */
#define WRITE_INT_TIMEOUT 1000 /* KATCP write timeout */
#define UPLOAD_TIMEOUT 20000 /* KATCP upload timeout */
#define QDR_TIMEOUT 20000 /* Same as above */
#define LUT_BUFFER_LEN 2097152 /* Number of samples in time domain LUTs */
#define FPGA_SAMP_FREQ 256.0e6 /* FPGA clock rate */
#define DAC_SAMP_FREQ 512.0e6 /* MUSIC board DAC/ADC clock rate */
/* Frequency resolution of DAC tones */
#define DAC_FREQ_RES (DAC_SAMP_FREQ / LUT_BUFFER_LEN)
#define LO_STEP 2.5e3 /* Freq step size for sweeps */
#define TARG_SWEEP_SPAN 10.0e3 /* Target sweep span */
#define NGRAD_POINTS 3 /* Number of points to use for gradient sweep */
#define NZEROS 14 /* Half the number of filter coefficients */
#define NC1_PORT 12345 /* Beaglebone com port for FC1 */
#define NC2_PORT 12346 /* Beaglebone com port for FC2 */

extern int16_t InCharge;
static int fft_len = 1024;
static uint32_t accum_len = (1 << 19) - 1;
// Test frequencies for troubleshooting (arbitrary values)
double freqs[1] = {50.0125};
size_t freqlen = 1;
// 27 Hann window coefficients, f_cutoff = 1.5 kHz
double zeros[14] = {0.00089543, 0.00353682, 0.00779173, 0.01344678, 0.02021843, 0.0277671, 0.03571428,
					0.04366146, 0.05121013, 0.05798178, 0.06363685, 0.06789176, 0.07053316, 0.07142859};
// double zeros[14] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};
// Firmware image files
const char roach_fpg[5][11] = {"roach1.fpg", "roach2.fpg", "roach3.fpg", "roach4.fpg", "roach5.fpg"};
const char test_fpg[] = "/data/etc/blast/blast_filt_fix_2017_Jan_30_2336.fpg";
// const char test_fpg[] = "/blast_0209_dds305_2017_Feb_09_1710.fpg";
static roach_state_t roach_state_table[NUM_ROACHES];
static bb_state_t bb_state_table[NUM_ROACHES];
// static ph_thread_t *roach_state = NULL;
char atten_init[] = "python /root/device_control/init_attenuators.py 30 30";
char valon_init[] = "python /root/device_control/init_valon.py";
char read_valon[] = "python /root/device_control/read_valon.py";

static uint32_t dest_ip = IPv4(192, 168, 40, 3);

void nameThread(const char*);

int roach_qdr_cal(roach_state_t *m_roach)
{
	char *m_cal_command;
	char m_line[256];
	blast_tmp_sprintf(m_cal_command, "python /data/etc/blast/cal_roach_qdr.py %s > %s",
					m_roach->address, m_roach->cal_log);
	system(m_cal_command);
	sleep(5);
	FILE *fd = fopen(m_roach->cal_log, "r");
	if (!fd) return 0;
	while (fgets(m_line, sizeof(m_line), fd)) {
		blast_info("%s", m_line);
	}
	fclose(fd);
	return 1;
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
    m_roach->LUT.Ival = calloc(m_len, sizeof(uint16_t));
    m_roach->LUT.Qval = calloc(m_len, sizeof(uint16_t));
}

static void roach_init_DACDDS_LUTs(roach_state_t *m_roach, size_t m_len)
{
    m_roach->DAC.len = m_len;
    m_roach->DAC.Ival = calloc(m_len, sizeof(double));
    m_roach->DAC.Qval = calloc(m_len, sizeof(double));
    m_roach->DDS.len = m_len;
    m_roach->DDS.Ival = calloc(m_len, sizeof(double));
    m_roach->DDS.Qval = calloc(m_len, sizeof(double));
}

static inline int roach_fft_bin_index(double *m_freqs, size_t m_index, size_t m_fft_len, double m_samp_freq)
{
	return (int)lround(m_freqs[m_index] / m_samp_freq *  m_fft_len);
}

static int roach_dac_comb(roach_state_t *m_roach, double *m_freqs,
        size_t m_freqlen, int m_samp_freq, double *m_I, double *m_Q)
{
    // Unless told otherwise, default attens are loaded
    double m_attens[m_freqlen];
    FILE *m_atten_file;
    size_t comb_fft_len;
    fftw_plan comb_plan;

    if (CommandData.roach[m_roach->which - 1].load_amps) {
        char *amps_file = m_roach->amps_path[CommandData.roach[m_roach->which - 1].load_amps - 1];
        blast_info("Amps file = %s", amps_file);
        if (!(m_atten_file = fopen(amps_file, "r"))) {
            blast_strerror("Could not open %s", amps_file);
            return -1;
        }
        blast_info("Opened %s", amps_file);
        // while(!feof(m_atten_file)) {
        for (size_t m_index = 0; m_index < m_freqlen; m_index++) {
            if (fscanf(m_atten_file, "%lg\n", &m_attens[m_index]) == EOF) break;
        }
        fclose(m_atten_file);
    }
    CommandData.roach[m_roach->which - 1].load_amps = 0;

    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
    }

    comb_fft_len = LUT_BUFFER_LEN;
    complex double *spec = calloc(comb_fft_len, sizeof(complex double));
    complex double *wave = calloc(comb_fft_len, sizeof(complex double));
    double max_val = 0.0;
    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        int bin = roach_fft_bin_index(m_freqs, i, comb_fft_len, m_samp_freq);
        if (bin < 0) {
            bin += comb_fft_len;
        }
        spec[bin] = m_attens[i] * cexp(_Complex_I * drand48() * 2.0 * M_PI);
    }
    /*FILE *f1 = fopen("./dac_spec.txt", "w");
     for (size_t i = 0; i < comb_fft_len; i++){
     fprintf(f1,"%f, %f\n", creal(spec[i]), cimag(spec[i]));
     }
     fclose(f1);
     FILE *f2 = fopen("./dac_wave.txt", "w");*/
    comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD,
    FFTW_ESTIMATE);
    fftw_execute(comb_plan);
    fftw_destroy_plan(comb_plan);
    /*for (size_t i = 0; i < comb_fft_len; i++) {
     fprintf(f2,"%f, %f\n", creal(wave[i]), cimag(wave[i]));
     if (cabs(wave[i]) > max_val) max_val = cabs(wave[i]);
     }

     fclose(f2);*/
    for (size_t i = 0; i < comb_fft_len; i++) {
        m_I[i] = creal(wave[i]) / max_val * ((1 << 15) - 1);
        m_Q[i] = cimag(wave[i]) / max_val * ((1 << 15) - 1);
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
	free(m_roach->DAC.Ival);
        free(m_roach->DAC.Qval);
        m_roach->DAC.len = 0;
    }
    if (m_roach->DAC.len == 0) {
        m_roach->DAC.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DAC.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DAC.len = LUT_BUFFER_LEN;
    }
    roach_dac_comb(m_roach, m_freqs, m_freqlen,
                    DAC_SAMP_FREQ, m_roach->DAC.Ival, m_roach->DAC.Qval);
}

static void roach_select_bins(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int bins[fft_len];
    double bin_freqs[fft_len];

    if (m_roach->freq_residuals) free(m_roach->freq_residuals);
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
	m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / (0.5*DAC_FREQ_RES)) * (0.5*DAC_FREQ_RES);
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
        free(m_roach->DDS.Ival);
        free(m_roach->DDS.Qval);
        m_roach->DDS.len = 0;
    }
    if (m_roach->DDS.len == 0) {
        m_roach->DDS.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DDS.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
        m_roach->DDS.len = LUT_BUFFER_LEN;
    }

    for (size_t i = 0; i < m_freqlen; i++) {
	double Ival[2 * fft_len];
    	double Qval[2 * fft_len];
	int bin = roach_fft_bin_index(m_roach->freq_residuals,
		i, LUT_BUFFER_LEN / fft_len, FPGA_SAMP_FREQ / (fft_len / 2));
	if (bin < 0) {
		bin += 2048;
	}
	roach_dds_comb(m_roach, m_roach->freq_residuals[i], m_freqlen,
                        FPGA_SAMP_FREQ / (fft_len / 2), bin, Ival, Qval);
	for (int j = i, k = 0; k < 2*fft_len; j += fft_len, k++) {
		m_roach->DDS.Ival[j] = Ival[k];
        	m_roach->DDS.Qval[j] = Qval[k];
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
	free(m_roach->LUT.Ival);
        free(m_roach->LUT.Qval);
    }*/
	roach_init_LUT(m_roach, 2 * LUT_BUFFER_LEN);
	for (size_t i = 0; i < LUT_BUFFER_LEN; i += 2) {
        	m_roach->LUT.Ival[2 * i + 0] = htons(m_roach->DAC.Ival[i + 1]);
        	m_roach->LUT.Ival[2 * i + 1] = htons(m_roach->DAC.Ival[i]);
        	m_roach->LUT.Ival[2 * i + 2] = htons(m_roach->DDS.Ival[i + 1]);
		m_roach->LUT.Ival[2 * i + 3] = htons(m_roach->DDS.Ival[i]);
		m_roach->LUT.Qval[2 * i + 0] = htons(m_roach->DAC.Qval[i + 1]);
		m_roach->LUT.Qval[2 * i + 1] = htons(m_roach->DAC.Qval[i]);
		m_roach->LUT.Qval[2 * i + 2] = htons(m_roach->DDS.Qval[i + 1]);
		m_roach->LUT.Qval[2 * i + 3] = htons(m_roach->DDS.Qval[i]);
	}
}

void roach_write_QDR(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{   roach_pack_LUTs(m_roach, m_freqs, m_freqlen);
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
    roach_write_int(m_roach, "start_dac", 0, 0);
    if (roach_write_data(m_roach, "qdr0_memory", (uint8_t*)m_roach->LUT.Ival,
    		m_roach->LUT.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
		blast_info("Could not write to qdr0!");
	}
    if (roach_write_data(m_roach, "qdr1_memory", (uint8_t*)m_roach->LUT.Qval,
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
	roach_init_DACDDS_LUTs(m_roach, LUT_BUFFER_LEN);
	roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);
	roach_define_DDS_LUT(m_roach, m_freqs, m_freqlen);
	blast_info("ROACH%d, Uploading Tone LUTs...", m_roach->which);
	roach_write_QDR(m_roach, m_freqs, m_freqlen);
}

void load_fir(roach_state_t *m_roach, double *m_zeros)
{
	char *reg;
	for (int i = 0; i < NZEROS; ++i) {
		m_zeros[i] *= ((pow(2, 31) - 1));
		blast_tmp_sprintf(reg, "b%d", i + 1);
		roach_write_int(m_roach, reg, (int32_t)m_zeros[i], 0);
		blast_info("b%d = %d", i + 1, (int32_t)m_zeros[i]);
	}
	blast_info("ROACH%d, Uploaded FIR coefficients", m_roach->which);
}

int bb_read_string(bb_state_t *m_bb)
{
	unsigned char m_read_buffer[4096];
	int bytes_read;
	int retval = -1;
	while (ph_bufq_len(m_bb->bb_comm->input_buffer)) {
		size_t m_size = (size_t)ph_bufq_len(m_bb->bb_comm->input_buffer);
		bytes_read = remote_serial_read_data(m_bb->bb_comm, m_read_buffer, m_size);
		m_read_buffer[bytes_read++] = '\0';
		blast_info("BB%d: %s", m_bb->which, m_read_buffer);
		retval = bytes_read;
	}
	// blast_info("Read Retval = %i", retval);
	return retval;
}

int bb_write_string(bb_state_t *m_bb, uint8_t *m_data, size_t m_len)
{
	int bytes_wrote;
	int retval = -1;
	m_data[m_len++] = '\n';
	m_data[m_len] = 0;
	bytes_wrote = remote_serial_write_data(m_bb->bb_comm, m_data, m_len);
	if (!bytes_wrote) {
		return retval;
	} else { retval = bytes_wrote;
	        // blast_info("Wrote %i bytes", bytes_wrote);
	        return retval;
	}
}

double roach_read_adc(roach_state_t *m_roach)
{
    size_t buffer_len = (1<<12);
    int16_t *temp_data;
    double rms_voltage;
    double sum = 0;
    temp_data = calloc((uint16_t)buffer_len, sizeof(uint16_t));
    roach_write_int(m_roach, "adc_snap_ctrl", 0, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_ctrl", 1, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_trig", 0, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_trig", 1, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_trig", 0, 0);
    usleep(1000);
    roach_read_data(m_roach, (uint8_t*)temp_data, "adc_snap_bram", 0, buffer_len * sizeof(uint16_t), QDR_TIMEOUT);
    roach_buffer_ntohs((uint16_t*)temp_data, buffer_len);
    for (size_t i = 0; i < buffer_len; i++) {
    	sum += (double)((int16_t)temp_data[i])*(double)((int16_t)temp_data[i]);
    }
    rms_voltage = sqrt(sum / (double)buffer_len);
    rms_voltage /= ((1<<16) - 1);
    rms_voltage *= (1100);
    free(temp_data);
    blast_info("ROACH%d, ADC V_rms = %g\n", m_roach->which, rms_voltage);
    return rms_voltage;
}

/* Adjust attenuators based on ADC rms voltage */
int set_atten(roach_state_t *m_roach)
{
	int pre_amp = 20; /* Pre-amp attenuator */
	// int pre_adc = 20; /* Pre-ADC attenuator */
	// int lower = 0.0;
	double upper = 0.0;
	double rms_voltage = roach_read_adc(m_roach);
	while (rms_voltage < upper) {
	    char *atten_command;
		pre_amp += 0.5;
		// TODO(Sam) /* Make sure the order of attenuators is correct here */
		blast_tmp_sprintf(atten_command, "python ~/init_attenuators.py %d %d \n", pre_amp, 30);
		blast_info("Setting BB%d pre-amp attenuation: %d dB", m_roach->which - 1, pre_amp);
		bb_write_string(&bb_state_table[m_roach->which - 1], (unsigned char*)atten_command, strlen(atten_command));
		while (bb_read_string(&bb_state_table[m_roach->which - 1]) <= 0) {
		sleep(3);
		}
		rms_voltage = roach_read_adc(m_roach);
	}
	return 0;
}

// static int roach_save_1d(const char *m_filename, void *m_data, size_t m_element_size, size_t m_len)
// {
//     uint32_t channel_crc;
//     FILE *fp;
//     channel_crc = crc32(BLAST_MAGIC32, m_data, m_element_size * m_len);
//     fp = fopen(m_filename, "w");
//     fwrite(&m_len, sizeof(size_t), 1, fp);
//     fwrite(m_data, m_element_size, m_len, fp);
//     fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
//     fclose(fp);
//     return 0;
// }

// Not currently being used, may change or eliminate
// static ssize_t roach_load_1d(const char *m_filename, void **m_data, size_t m_element_size)
// {
//     size_t len;
//     FILE *fp;
//     struct stat fp_stat;
//     uint32_t channel_crc;
//     if (stat(m_filename, &fp_stat)) {
//         blast_err("Could not get file data for %s: %s", m_filename, strerror(errno));
//         return -1;
//     }
//     if (!(fp = fopen(m_filename, "r"))) {
//         blast_err("Could not open %s for reading: %s", m_filename, strerror(errno));
//         return -1;
//     }
//     if (fread(&len, sizeof(len), 1, fp) != 1) {
//         blast_err("Could not read data length from %s: %s", m_filename, strerror(errno));
//         fclose(fp);
//         return -1;
//     }
//     if ((len * m_element_size) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
//         blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu", m_filename,
//                   (len * m_element_size) + sizeof(channel_crc) + sizeof(len), fp_stat.st_size);
//         fclose(fp);
//         return -1;
//     }
//     *m_data = calloc(len, m_element_size);
//     fread(*m_data, m_element_size, len, fp);
//     fread(&channel_crc, sizeof(channel_crc), 1, fp);
//     fclose(fp);
//     if (channel_crc != crc32(BLAST_MAGIC32, *m_data, m_element_size * len)) {
//         free(*m_data);
//         *m_data = NULL;
//         blast_err("Mismatched CRC for '%s'.  File corrupted?", m_filename);
//         len = -1;
//     }
//     return len;
// }

/* Check if UDP streaming is successful */
int roach_check_streaming(roach_state_t *m_roach)
{
	int m_last_packet_count = roach_udp[m_roach->which - 1].roach_packet_count;
	while ((roach_state_table[m_roach->which - 1].is_streaming != 1)) {
		/* Run for 10 seconds and check to see if packet count has incremented */
		sleep(10);
		if (roach_udp[m_roach->which - 1].roach_packet_count > m_last_packet_count) {
			roach_state_table[m_roach->which - 1].is_streaming = 1;
			return 0;
		} else { blast_err("Data stream error on ROACH%d", m_roach->which);
			return -1;
		}
	}
	return 0;
}

void roach_vna_comb(roach_state_t *m_roach)
{
	size_t m_freqlen = 1000;
	double p_max_freq = 250.021234e6;
	double p_min_freq = 10.2342e6;
	double n_max_freq = -10.2342e6+5.0e4;
	double n_min_freq = -250.021234e6+5.0e4;
	m_roach->vna_comb = calloc(m_freqlen, sizeof(double));
	m_roach->freqlen = m_freqlen;
	/* positive freqs */
	double p_delta_f = (p_max_freq - p_min_freq) / ((m_freqlen/2) - 1);
	/* Store delta f, to be used as 'span' in roach_do_sweep */
	m_roach->delta_f = p_delta_f;
	for (size_t i = m_freqlen/2; i-- > 0;) {
		m_roach->vna_comb[m_freqlen/2 - (i + 1)] = p_max_freq - i*p_delta_f;
	}
	/* negative freqs */
	double n_delta_f = (n_max_freq - n_min_freq) / ((m_freqlen/2) - 1);
	for (size_t i = 0; i < m_freqlen/2; i++) {
		m_roach->vna_comb[i + m_freqlen/2] = n_min_freq + i*n_delta_f;
	}
	m_roach->num_kids = m_roach->freqlen;
}

/* Save the current list of baseband frequencies */
void save_bb_freqs(roach_state_t *m_roach)
{
	char *bb_fname;
	blast_tmp_sprintf(bb_fname, "%s/vna_freqs.dat", m_roach->last_vna_path);
	FILE *m_bb_fd = fopen(bb_fname, "w");
	if (!m_bb_fd) {
	    blast_strerror("Could not open %s for writing", bb_fname);
	    return;
	}
	for (size_t i = 0; i < m_roach->freqlen; i++) {
		// blast_info("delta f = %g, freq = %g\n", m_roach->delta_f, m_roach->vna_comb[i]);
		fprintf(m_bb_fd, "%.10f\n", (float)m_roach->vna_comb[i]);
	}
	fclose(m_bb_fd);
	blast_info("ROACH%d VNA freqs written to %s", m_roach->which, bb_fname);
}

void roach_save_sweep_packet(roach_state_t *m_roach, float m_sweep_freq, char *m_sweep_save_path)
{
	/* Grab a set number of packets, average I and Q for each chan,
	and save the result to file:fname in the sweep dir (/data/etc/blast/sweeps/) */
	/* Save I_avg, Q_avg, to sweep dir */
	int m_num_to_avg = 100;
	int m_num_received = 0;
	int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
	uint8_t i_udp_read;
	char *fname;
	blast_tmp_sprintf(fname, "%s/%f.dat", m_sweep_save_path, m_sweep_freq);
	FILE *m_fd = fopen(fname, "w");
	if (!m_fd) {
        blast_strerror("Could not open %s for writing", fname);
        return;
	}
	float *I_sum = calloc(m_roach->num_kids, sizeof(float)); // Array to store I values to be summed
	float *Q_sum = calloc(m_roach->num_kids, sizeof(float)); // Array to store Q values to be summed
	float *I_avg = calloc(m_roach->num_kids, sizeof(float)); // Array to store averaged I values
	float *Q_avg = calloc(m_roach->num_kids, sizeof(float)); // Array to store averaged Q values
	while (m_num_received < m_num_to_avg) {
		if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
		   m_num_received++;
		   i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
		   data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
		   for (size_t chan = 0; chan < m_roach->num_kids; chan ++) {
			I_sum[chan] +=  m_packet.Ival[chan];
			Q_sum[chan] +=  m_packet.Qval[chan];
		   }
                }
		m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
	}
	for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
		I_avg[chan] = (I_sum[chan] / m_num_to_avg);
		Q_avg[chan] = (Q_sum[chan] / m_num_to_avg);
		/* Save I_avg, Q_avg, to sweep dir */
		fprintf(m_fd, "%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
		// blast_info("%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
	}
	free(I_sum);
	free(Q_sum);
	free(I_avg);
	free(Q_avg);
	fclose(m_fd);
}

/* void sweep_lo(roach_state_t *m_roach, double *m_sweep_freqs, size_t m_num_sweep_freqs)
{
	char ssh_command[FILENAME_MAX];
	for (size_t i = 0; i < m_num_sweep_freqs; i++) {
		snprintf(ssh_command, sizeof(ssh_command), "python ~/device_control/set_lo.py %g\n", m_sweep_freqs[i]/1.0e6);
		fputs(ssh_command, bb_state_table[m_roach->which - 1].bb_ssh_pipe);
		blast_info("LO @ %g\n", m_sweep_freqs[i]/1.0e6);
		sleep(0.5);
		roach_save_sweep_packet(m_roach, m_sweep_freqs[i]/1.0e6);
	}
	fputs("python ~/device_control/set_lo.py 750\n", bb_state_table[m_roach->which - 1].bb_ssh_pipe);
}*/

void get_targ_freqs(roach_state_t *m_roach)
{
    char *py_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    char m_line[256];
    blast_tmp_sprintf(py_command,
            "python /data/etc/blast/find_kids_blast.py %s %s > %s",
            m_roach->last_vna_path, "/home/fc1user/sam_tests",
            m_roach->find_kids_log);
    blast_info("%s", py_command);
    system(py_command);
    sleep(3);
    FILE *fd = fopen(m_roach->find_kids_log, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_roach->find_kids_log);
        return;
    }
    while (fgets(m_line, sizeof(m_line), fd)) {
        blast_info("%s", m_line);
    }
    fclose(fd);
    blast_tmp_sprintf(m_targ_freq_path,
            "/home/fc1user/sam_tests/bb_targ_freqs.dat");

    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return;
    }
    size_t m_freqlen = 0;
    while (m_freqlen < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &m_temp_freqs[m_freqlen++]) != EOF) {
    }
    fclose(fd);
    m_roach->num_kids = m_freqlen;
    m_roach->targ_tones = calloc(m_roach->num_kids, sizeof(double));
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j];
        blast_info("KID freq = %lg", m_roach->targ_tones[j]);
    }
}

void get_time(char *time_buffer)
{
	/* Get current time to append to sweep directory root */
	time_t rawtime;
	time(&rawtime);
	ctime_r(&rawtime, time_buffer);
	/* Convert spaces in ctime to underscores */
	char *p = time_buffer;
	for (; *p; ++p) {
		if (*p == ' ') *p = '_';
		if (*p == ':') *p = '_';
	}
	time_buffer[strlen(time_buffer) - 1] = 0;
}

char* make_dir(roach_state_t *m_roach, char *m_dir_root)
{
	char *mkdir_command;
	char *perm_path;
	char time_buffer[FILENAME_MAX];
	char *return_path;
	get_time(time_buffer);
	asprintf(&return_path, "%s/%s", m_dir_root, time_buffer);
	blast_tmp_sprintf(mkdir_command, "mkdir -p %s", return_path);
	system(mkdir_command);
	blast_tmp_sprintf(perm_path, "sudo chown -R fc1user:blast %s", return_path);
	system(perm_path);
	return return_path;
}

void roach_do_sweep(roach_state_t *m_roach, int type)
{
	bb_state_t m_bb = bb_state_table[m_roach->which - 1];
	/*type = {0:VNA, 1:TARG} */
	double m_span;
	/* Create VNA and TARG directories on FC1 */
	char *sweep_freq_fname;
	if ((type == 0)) {
		/* Get current time to append to sweep directory root */
		m_roach->last_vna_path = make_dir(m_roach, m_roach->vna_path_root);
		blast_info("VNA dir to copy: %s", m_roach->last_vna_path);
		m_roach->last_targ_path = make_dir(m_roach, m_roach->targ_path_root);
		blast_info("TARG dir to copy: %s", m_roach->last_targ_path);
		// m_span = m_roach->delta_f;
		m_span = 1.0e4;
		blast_info("ROACH%d, VNA sweep will be saved in %s", m_roach->which, m_roach->last_vna_path);
		blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_vna_path);
		save_bb_freqs(m_roach);
	} else {
		m_span = TARG_SWEEP_SPAN;
		m_roach->last_targ_path = make_dir(m_roach, m_roach->targ_path_root);
		blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_targ_path);
		blast_info("Sweep freq fname = %s", sweep_freq_fname);
		blast_info("ROACH%d, TARGET sweep will be saved in %s", m_roach->which, m_roach->last_targ_path);
		blast_info("ROACH%d, Calculating KID freqs...", m_roach->which);
		get_targ_freqs(m_roach);
		blast_info("Uploading TARGET comb...");
		roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
		blast_info("ROACH%d, TARGET comb uploaded", m_roach->which);
	}
	/* Determine sweep frequencies */
	FILE *m_sweep_fd = fopen(sweep_freq_fname, "w");
	if (!m_sweep_fd) {
        blast_strerror("Could not open %s for writing", sweep_freq_fname);
        return;
	}
	double m_min_freq = m_roach->lo_centerfreq - (m_span/2);
	double m_max_freq = m_roach->lo_centerfreq + (m_span/2);
	size_t m_num_sweep_freqs = (m_max_freq - m_min_freq) / LO_STEP;
	char *lo_command; /* BB command */
	double *m_sweep_freqs = calloc(m_num_sweep_freqs, sizeof(double));
	m_sweep_freqs[0] = m_min_freq;
	for (size_t i = 1; i < m_num_sweep_freqs; i++) {
		m_sweep_freqs[i] = m_sweep_freqs[i - 1] + LO_STEP;
	}
	for (size_t i = 0; i < m_num_sweep_freqs; i++) {
		m_sweep_freqs[i] = round(m_sweep_freqs[i] / LO_STEP) * LO_STEP;
		fprintf(m_sweep_fd, "%.6f\n", (float)m_sweep_freqs[i]);
	}
	fclose(m_sweep_fd);
	blast_info("ROACH%d, Sweep freqs written to %s", m_roach->which, sweep_freq_fname);
	blast_info("ROACH%d Starting new sweep...", m_roach->which);
	/* Sweep and save data */
	for (size_t i = 0; i < m_num_sweep_freqs; i++) {
		if (CommandData.roach[m_roach->which - 1].do_sweeps) {
			blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n", m_sweep_freqs[i]/1.0e6);
			bb_write_string(&m_bb, (unsigned char*)lo_command, strlen(lo_command));
			while (bb_read_string(&m_bb) <= 0) {
				usleep(2000);
			}
			if ((type == 0)) {
				roach_save_sweep_packet(m_roach, m_sweep_freqs[i]/1.0e6, m_roach->last_vna_path);
			} else {
				roach_save_sweep_packet(m_roach, m_sweep_freqs[i]/1.0e6, m_roach->last_targ_path);
			}
		} else {
			break;
			blast_info("Sweep interrupted by command");
		}
	}
	blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n", m_roach->lo_centerfreq/1.0e6);
	bb_write_string(&m_bb, (unsigned char*)lo_command, strlen(lo_command));
	free(m_sweep_freqs);
	m_roach->status = ROACH_STATUS_STREAMING;
}

/* Small sweep to calculate I/Q gradient */
void grad_sweep(roach_state_t *m_roach)
{
	bb_state_t m_bb = bb_state_table[m_roach-> which - 1];
	m_roach->last_grad_path = make_dir(m_roach, m_roach->grad_path_root);
	if (CommandData.roach[m_roach->which - 1].do_calc_grad == 1) {
		blast_tmp_sprintf(m_roach->ref_sweep_path, m_roach->last_grad_path);
	}
	char *lo_command; /* BB command */
	double m_sweep_freqs[NGRAD_POINTS];
	m_sweep_freqs[0] = m_roach->lo_centerfreq - (((NGRAD_POINTS - 1)/2)*LO_STEP);
	for (size_t i = 0; i < NGRAD_POINTS; i++) {
		if (CommandData.roach[m_roach->which - 1].do_sweeps) {
			m_sweep_freqs[i] = round(m_sweep_freqs[i] / LO_STEP) * LO_STEP;
			blast_info("Sweep freq = %.7g", m_sweep_freqs[i]/1.0e6);
			// m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
			blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %.7g\n", m_sweep_freqs[i]/1.0e6);
			bb_write_string(&m_bb, (unsigned char*)lo_command, strlen(lo_command));
			while (bb_read_string(&m_bb) <= 0) {
				usleep(1000);
			roach_save_sweep_packet(m_roach, m_sweep_freqs[i]/1.0e6, m_roach->last_grad_path);
			}
			m_sweep_freqs[i + 1] = m_sweep_freqs[i] + LO_STEP;
		} else {
			break;
			blast_info("Sweep interrupted by command");
		}
	blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n", m_roach->lo_centerfreq/1.0e6);
	bb_write_string(&m_bb, (unsigned char*)lo_command, strlen(lo_command));
	m_roach->status = ROACH_STATUS_STREAMING;
	}
}

static int roach_check_retune(roach_state_t *m_roach)
{
    /* Calculate (delta_I, delta_Q), project onto ref grad, (dI/df, dQ/df) */
    /* Retune if N resonators exceeds threshold */
    // int nflags;
    // int threshold = m_roach->num_kids/4;
    FILE* fp;
    float I_ref[m_roach->num_kids][NGRAD_POINTS];
    float Q_ref[m_roach->num_kids][NGRAD_POINTS];
    int ref_chan[m_roach->num_kids];
    int comp_chan[m_roach->num_kids];
    float dIdf[m_roach->num_kids];
    float dQdf[m_roach->num_kids];
    float I_comp[m_roach->num_kids][NGRAD_POINTS];
    float Q_comp[m_roach->num_kids][NGRAD_POINTS];
    float delta_I[m_roach->num_kids];
    float delta_Q[m_roach->num_kids];
    float dot_I[m_roach->num_kids];
    float dot_Q[m_roach->num_kids];
    float delta_f[m_roach->num_kids];
    char *read_filename;
    char *dirs[2];
    dirs[0] = m_roach->ref_grad_path;
    dirs[1] = m_roach->last_grad_path;
    for (int n = 0; n < 2; ++n) {
        struct dirent **file_list;
        int nfiles;
        int freq_idx = 0;
        nfiles = scandir(dirs[n], &file_list, NULL, alphasort);
        if (nfiles < 0) {
            blast_strerror("scandir");
            continue;
        }

        for (int i = 0; i < nfiles; free(file_list[i++])) {
            if (file_list[i]->d_type != DT_REG && file_list[i]->d_type != DT_LNK) continue;

            blast_tmp_sprintf(read_filename, "%s/%s", dirs[n], file_list[i]->d_name);
            if (!(fp = fopen(read_filename, "r"))) {
                blast_strerror("Could not open %s for reading", read_filename);
                continue;
            }
            for (int kid = 0; kid < m_roach->num_kids; ++kid) {
                if (n == 0) {
                    fscanf(fp, "%d\t%f\t%f\n", &ref_chan[kid],
                            &I_ref[kid][freq_idx], &Q_ref[kid][freq_idx]);
                } else {
                    fscanf(fp, "%d\t%f\t%f\n", &comp_chan[kid],
                            &I_comp[kid][freq_idx], &Q_comp[kid][freq_idx]);
                }
            }
            ++freq_idx;
            printf("Read: %s\n", read_filename);
            fclose(fp);
        }
    }
    /* Load ref grads */
    if (!(fp = fopen(m_roach->ref_grad_path, "r"))) {
        blast_strerror("Could not open %s for reading", m_roach->ref_grad_path);
        return -1;
    }
    for (int kid = 0; kid < m_roach->num_kids; ++kid) {
        fscanf(fp, "%f,%f\n", &dIdf[kid], &dQdf[kid]);
    }
    fclose(fp);
    /* Calculate: delta_f using (delta_I, delta_Q) dot (dIdf, dQdf) */
    for (int kid = 0; kid < m_roach->num_kids; ++kid) {
        delta_I[kid] = I_comp[kid][(NGRAD_POINTS - 1) / 2]
                - I_ref[kid][(NGRAD_POINTS - 1) / 2];
        delta_Q[kid] = Q_comp[kid][(NGRAD_POINTS - 1) / 2]
                - Q_ref[kid][(NGRAD_POINTS - 1) / 2];
        dot_I[kid] = delta_I[kid] * dIdf[kid];
        dot_Q[kid] = delta_Q[kid] * dQdf[kid];
        delta_f[kid] = ((dot_I[kid] + dot_Q[kid])
                / (dIdf[kid] * dIdf[kid] + dQdf[kid] * dQdf[kid])) * LO_STEP;
        blast_info("Chan %d, df = %f", kid, delta_f[kid]);
    }
    return 0;
}

void grad_calc(roach_state_t *m_roach)
{
	/* Calculates a reference gradient, (dI/df, dQ/df) */
	/* Used to determine whether or not to retune resonators */
	blast_info("ROACH%d, Calculating KID GRADIENTS", m_roach->which);
	int chan[m_roach->num_kids];
	float dIdf[m_roach->num_kids];
	float dQdf[m_roach->num_kids];
	float delta_f[m_roach->num_kids];
	float Ival[m_roach->num_kids][NGRAD_POINTS];
	// float rf_freq[m_roach->num_kids][3];
	float Qval[m_roach->num_kids][NGRAD_POINTS];
	char *read_filename;
	char *grad_out_filename;
	char *df_out_filename;
	char m_time_buffer[4096];
	get_time(m_time_buffer);
	/* open files for reading */
	struct dirent **file_list;
	int nfiles;
	int freq_idx = 0;
	nfiles = scandir(m_roach->last_grad_path, &file_list, NULL, alphasort);
	if (nfiles < 0) {
		blast_strerror("Could read %s", m_roach->last_grad_path);
	} else {
		for (int i = 0; i < nfiles; ++i) {
			if (!strcmp(file_list[i]->d_name, ".") || !strcmp(file_list[i]->d_name, "..")) {
				free(file_list[i]);
			} else {
				blast_tmp_sprintf(read_filename, "%s/%s", m_roach->last_grad_path, file_list[i]->d_name);
         			free(file_list[i]);
				FILE* fd = fopen(read_filename, "r");
				for(int kid = 0; kid < m_roach->num_kids; ++kid) {
					fscanf(fd, "%d\t%f\t%f\n", &chan[kid], &Ival[kid][freq_idx], &Qval[kid][freq_idx]);
				}
				++freq_idx;
				printf("Read: %s\n", read_filename);
				fclose(fd);
			}
		}
		if(file_list) free(file_list);
    }

	if (CommandData.roach[m_roach->which - 1].do_calc_grad == 1) {
		// TODO(Sam) save grads in different dir
		asprintf(&m_roach->ref_grad_path, "/home/fc1user/sam_tests/roach%d_ref_grads.dat", m_roach->which);
		grad_out_filename = m_roach->ref_grad_path;
		asprintf(&m_roach->ref_df_path, "/home/fc1user/sam_tests/roach%d_ref_df.dat", m_roach->which);
		df_out_filename = m_roach->ref_df_path;
		blast_info("ROACH%d REF grads will be saved in: %s", m_roach->which, m_roach->ref_grad_path);
		blast_info("ROACH%d REF grads will be saved in: %s", m_roach->which, m_roach->ref_df_path);
	}
	if (CommandData.roach[m_roach->which - 1].do_calc_grad == 2) {
		asprintf(&m_roach->last_comp_df, "/home/fc1user/sam_tests/roach%d_comp_grads.dat", m_roach->which);
		df_out_filename = m_roach->last_comp_df;
		asprintf(&m_roach->last_comp_df, "/home/fc1user/sam_tests/roach%d_comp_df.dat", m_roach->which);
		df_out_filename = m_roach->last_comp_df;
		blast_info("ROACH%d comp grads will be saved in: %s", m_roach->which, m_roach->last_comp_grads);
		blast_info("ROACH%d REF grads will be saved in: %s", m_roach->which, m_roach->ref_df_path);
		roach_check_retune(m_roach);
		CommandData.roach[m_roach->which - 1].do_calc_grad = 0;
	}
	FILE *grads_out = fopen(grad_out_filename, "w");
	if (!grads_out) {
        blast_strerror("Could not open %s for writing", grad_out_filename);
        return;
    }
	FILE *delta_f_out = fopen(df_out_filename, "w");
    if (!delta_f_out) {
        blast_strerror("Could not open %s for writing", df_out_filename);
        return;
    }
	for (int kid = 0; kid < m_roach->num_kids; ++kid) {
		// printf("%.10f\t%.10f\t%.10f\n", Qval[kid][0], Qval[kid][1], Qval[kid][2]);
		dIdf[kid] = (Ival[kid][NGRAD_POINTS - 1] - Ival[kid][0]);
		dQdf[kid] = (Qval[kid][NGRAD_POINTS - 1] - Qval[kid][0]);
		delta_f[kid] = ((Ival[kid][(NGRAD_POINTS - 1) / 2]*dIdf[kid] + Qval[kid][(NGRAD_POINTS - 1) / 2]
									*dQdf[kid])/(dIdf[kid]*dIdf[kid] + dQdf[kid]*dQdf[kid]))*LO_STEP;
		fprintf(grads_out, "%.10f,%.10f\n", dIdf[kid], dQdf[kid]);
		fprintf(delta_f_out, "%d\t%.10f\n", kid, delta_f[kid]);
	}

	fclose(grads_out);
	fclose(delta_f_out);
	blast_info("ROACH%d, IQ GRADIENTS saved in %s", m_roach->which, grad_out_filename);
	blast_info("ROACH%d, DELTA Fs saved in %s", m_roach->which, df_out_filename);
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
        blast_err("Write buffer not empty: %lu", ph_bufq_len(m_sock->wbuf));
        return;
    }
    ph_sock_enable(m_sock, 0);
    ph_sock_free(m_sock);
}

static void firmware_upload_connected(ph_sock_t *m_sock, int m_status,
        int m_errcode, const ph_sockaddr_t *m_addr, struct timeval *m_elapsed,
        void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    firmware_state_t *state = (firmware_state_t*) m_data;
    size_t number_bytes;
    ph_stream_t *firmware_stm;
    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", state->roach->address,
                    state->port, gai_strerror(m_errcode));
            state->result = ROACH_UPLOAD_RESULT_ERROR;
            return;
        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->roach->address, state->port, m_errcode,
                    strerror(m_errcode));
            state->result = ROACH_UPLOAD_CONN_REFUSED;
            return;
    }
    blast_info("Connected to ROACH at %s:%u", state->roach->address,
            state->port);
    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);
    state->sock = m_sock;
    m_sock->callback = firmware_upload_process_return;
    m_sock->timeout_duration.tv_sec = 10;
    m_sock->job.data = state;
    /**
     * We have enabled the socket and now we buffer the firmware file into the network
     */

    if ((firmware_stm = ph_stm_file_open(state->firmware_file, O_RDONLY, 0))) {
        state->result = ROACH_UPLOAD_RESULT_WORKING;
        if (!ph_stm_copy(firmware_stm, m_sock->stream, PH_STREAM_READ_ALL,
        NULL, &number_bytes)) {
            blast_err("Error getting data from %s: %s", state->firmware_file,
                    strerror(ph_stm_errno(firmware_stm)));
            ph_sock_shutdown(state->sock, PH_SOCK_SHUT_RDWR);
            ph_sock_free(state->sock);
            state->result = ROACH_UPLOAD_RESULT_ERROR;
        } else {
            blast_info("Loading %s with %zu bytes", state->firmware_file,
                    number_bytes);
            ph_sock_enable(state->sock, true);
        }
        ph_stm_close(firmware_stm);
    } else {
        blast_err("Could not load firmware file \"%s\"", state->firmware_file);
        ph_sock_shutdown(state->sock, PH_SOCK_SHUT_RDWR);
        ph_sock_free(state->sock);
        state->result = ROACH_UPLOAD_RESULT_ERROR;
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
        blast_err("Could not request upload port for ROACH firmware on %s! retval = %i", m_roach->address, retval);
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
    if ((state.result != ROACH_UPLOAD_RESULT_SUCCESS)) return -1;
    if ((state.result = ROACH_UPLOAD_RESULT_SUCCESS)) {
    	sleep(5);
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
		roach_write_int(&roach_state_table[i], "tx_rst", 1, 0);
		if (roach_state_table[i].katcp_fd > 0) {
			destroy_rpc_katcl(roach_state_table[i].rpc_conn);
			blast_info("Closing KATCP on ROACH%d", i + 1);
            		ph_sock_shutdown(bb_state_table[i].bb_comm->sock, PH_SOCK_SHUT_RDWR);
            		ph_sock_enable(bb_state_table[i].bb_comm->sock, false);
            		ph_sock_free(bb_state_table[i].bb_comm->sock);
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
		bb_state_table[i].status = BB_STATUS_BOOT;
		bb_state_table[i].desired_status = BB_STATUS_INIT;
		roach_state_table[i].status = ROACH_STATUS_BOOT;
		roach_state_table[i].desired_status = ROACH_STATUS_STREAMING;
	}
	while (!shutdown_mcp) {
		// TODO(SAM/LAURA): Fix Roach 1/Add error handling
		for (int i = 0; i < 1; i++) {
		// Check for new roach status commands
		    if (CommandData.roach[i].change_state) {
                roach_state_table[i].status = CommandData.roach[i].new_state;
                CommandData.roach[i].change_state = 0;
		    }
		// for (int i = 0; i < NUM_ROACHES; i++) {
			if (bb_state_table[i].status == BB_STATUS_BOOT && bb_state_table[i].desired_status > BB_STATUS_BOOT) {
				blast_info("Initializing BB%d ...", i + 1);
				bb_state_table[i].which = i + 1;
				bb_state_table[i].bb_comm = remote_serial_init(i, NC2_PORT);
				while (!bb_state_table[i].bb_comm->connected || !InCharge) {
				usleep(3000);
			}
				blast_info("BB%d Initialized", i + 1);
				blast_info("Initializing Valon%d...", i + 1);
				bb_write_string(&bb_state_table[i], (unsigned char*)valon_init, strlen(valon_init));
				bb_write_string(&bb_state_table[i], (unsigned char*)read_valon, strlen(read_valon));
				while (bb_read_string(&bb_state_table[i]) <= 0) {
				sleep(3);
			}
				blast_info("Initializing BB%d RUDATs...", i + 1);
				bb_write_string(&bb_state_table[i], (unsigned char*)atten_init, strlen(atten_init));
				while (bb_read_string(&bb_state_table[i]) <= 0) {
				sleep(3);
			}
				bb_state_table[i].status = BB_STATUS_INIT;
				bb_state_table[i].desired_status = BB_STATUS_RUNNING;
			}
			// if (bb_state_table[i].status == BB_STATUS_INIT && bb_state_table[i].desired_status >= BB_STATUS_RUNNING) {
			if (roach_state_table[i].status == ROACH_STATUS_BOOT && roach_state_table[i].desired_status > ROACH_STATUS_BOOT) {
				blast_info("Attempting to connect to %s", roach_state_table[i].address);
				roach_state_table[i].katcp_fd = net_connect(roach_state_table[i].address,
					0, NETC_VERBOSE_ERRORS | NETC_VERBOSE_STATS);
				blast_info("fd:%d ", roach_state_table[i].katcp_fd);
				roach_state_table[i].rpc_conn = create_katcl(roach_state_table[i].katcp_fd);
				if (roach_state_table[i].katcp_fd > 0) {
					roach_state_table[i].status = ROACH_STATUS_CONNECTED;
				blast_info("ROACH%d, KATCP up", i + 1);
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_CONNECTED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_PROGRAMMED) {
				blast_info("ROACH%d, Firmware uploaded", i + 1);
				if (roach_upload_fpg(&roach_state_table[i], test_fpg) == 0) {
					roach_state_table[i].status = ROACH_STATUS_PROGRAMMED;
					roach_state_table[i].desired_status = ROACH_STATUS_CONFIGURED;
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_PROGRAMMED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_CONFIGURED) {
				blast_info("ROACH%d, Configuring software registers...", i + 1);
				roach_write_int(&roach_state_table[i], "dds_shift", DDS_SHIFT, 0);/* DDS LUT shift, in clock cycles */
				roach_read_int(&roach_state_table[i], "dds_shift");
				roach_write_int(&roach_state_table[i], "fft_shift", FFT_SHIFT, 0);/* FFT shift schedule */
				roach_read_int(&roach_state_table[i], "fft_shift");
				roach_write_int(&roach_state_table[i], "sync_accum_len", accum_len, 0);/* Number of accumulations */
				roach_read_int(&roach_state_table[i], "sync_accum_len");
				roach_write_int(&roach_state_table[i], "tx_destip", dest_ip, 0);/* UDP destination IP */
				roach_write_int(&roach_state_table[i], "tx_destport", roach_state_table[i].dest_port, 0); /* UDP port */
				load_fir(&roach_state_table[i], zeros);
				roach_state_table[i].status = ROACH_STATUS_CONFIGURED;
				roach_state_table[i].desired_status = ROACH_STATUS_CALIBRATED;
			}
			if (roach_state_table[i].status == ROACH_STATUS_CONFIGURED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_CALIBRATED) {
				roach_write_int(&roach_state_table[i], "dac_reset", 1, 0);
				blast_info("ROACH%d, Calibrating QDR RAM", i + 1);
				if (roach_qdr_cal(&roach_state_table[i])) {
					blast_info("ROACH%d, Calibration complete", i + 1);
					roach_write_int(&roach_state_table[i], "tx_rst", 0, 0);
					roach_write_int(&roach_state_table[i], "tx_rst", 1, 0);
					roach_write_int(&roach_state_table[i], "tx_rst", 0, 0);
					roach_write_int(&roach_state_table[i], "pps_start", 1, 0);
					roach_state_table[i].status = ROACH_STATUS_CALIBRATED;
					roach_state_table[i].desired_status = ROACH_STATUS_TONE;
				} else {
					blast_info("ROACH%d, Calibration failed", i + 1);
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_CALIBRATED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_TONE) {
				blast_info("ROACH%d, Generating search comb...", i + 1);
				roach_vna_comb(&roach_state_table[i]);
				roach_write_tones(&roach_state_table[i], roach_state_table[i].vna_comb, roach_state_table[i].freqlen);
				blast_info("ROACH%d, Search comb uploaded", i + 1);
				roach_state_table[i].status = ROACH_STATUS_TONE;
				roach_state_table[i].desired_status = ROACH_STATUS_STREAMING;
			}
			if (roach_state_table[i].status == ROACH_STATUS_TONE &&
				roach_state_table[i].desired_status >= ROACH_STATUS_STREAMING) {
				blast_info("ROACH%d, Checking stream status...", i + 1);
				if (roach_check_streaming(&roach_state_table[i]) == 0) {
					roach_state_table[i].status = ROACH_STATUS_STREAMING;
					blast_info("ROACH%d, streaming SUCCESS", i + 1);
					roach_state_table[i].desired_status = ROACH_STATUS_ATTENUATION;
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_STREAMING &&
				roach_state_table[i].desired_status >= ROACH_STATUS_ATTENUATION) {
				// blast_info("ROACH%d, Setting Attenuators...", i + 1);
				// roach_read_adc(&roach_state_table[i]);
				// set_atten(&roach_state_table[i]); /* Set attenuators based on ADC voltage */
				roach_state_table[i].status = ROACH_STATUS_ATTENUATION;
				roach_state_table[i].desired_status = ROACH_STATUS_VNA;
			}
			if (roach_state_table[i].status == ROACH_STATUS_ATTENUATION &&
				roach_state_table[i].desired_status >= ROACH_STATUS_VNA) {
				blast_info("Initializing Beaglebone%d socket...", i + 1);
				blast_info("ROACH%d, Initializing VNA sweep", i + 1);
				blast_info("ROACH%d, Starting VNA sweep...", i + 1);
				roach_do_sweep(&roach_state_table[i], 0);
				blast_info("ROACH%d, VNA sweep complete", i + 1);
				roach_state_table[i].status = ROACH_STATUS_ARRAY_FREQS;
				roach_state_table[i].desired_status = ROACH_STATUS_TARG;
			}
			if (roach_state_table[i].status == ROACH_STATUS_ARRAY_FREQS &&
				roach_state_table[i].desired_status >= ROACH_STATUS_TARG) {
				blast_info("ROACH%d, Initializing TARG sweep", i + 1);
				roach_do_sweep(&roach_state_table[i], 1);
				blast_info("ROACH%d, TARG sweep complete", i + 1);
				CommandData.roach[i].do_calc_grad = 1;
				blast_info("ROACH%d, Starting GRADIENT sweep...", i + 1);
				grad_sweep(&roach_state_table[i]);
				blast_info("ROACH%d, GRADIENT sweep complete", i + 1);
				grad_calc(&roach_state_table[i]);
				if (roach_check_streaming(&roach_state_table[i]) == 0) {
					roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
					roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
				}
			}
			/* if (roach_state_table[i].status == ROACH_STATUS_ARRAY_FREQS &&
				roach_state_table[i].desired_status >= ROACH_STATUS_ACQUIRING) {
			}*/
		// Check for any additional roach commands
			/* if (CommandData.roach[i].do_calc_grad == 1) {
            			grad_sweep(&roach_state_table[i]);
				grad_calc(&roach_state_table[i]);
			}
			if (CommandData.roach[i].do_calc_grad == 2) {
            			grad_sweep(&roach_state_table[i]);
				grad_calc(&roach_state_table[i]);
				roach_check_retune(&roach_state_table[i]);
				CommandData.roach[i].do_calc_grad = 0;
			}*/
		}
	}
	return NULL;
}

int init_roach(void)
{
    memset(roach_state_table, 0, sizeof(roach_state_t) * NUM_ROACHES);
    memset(bb_state_table, 0, sizeof(bb_state_t) * NUM_ROACHES);
    for (int i = 0; i < NUM_ROACHES; i++) {
    	 asprintf(&roach_state_table[i].address, "roach%d", i + 1);
    	 asprintf(&roach_state_table[i].vna_path_root, "/home/fc1user/sam_tests/sweeps/roach%d/vna", i + 1);
    	 asprintf(&roach_state_table[i].targ_path_root, "/home/fc1user/sam_tests/sweeps/roach%d/targ", i + 1);
    	 asprintf(&roach_state_table[i].grad_path_root, "/home/fc1user/sam_tests/sweeps/roach%d/grad", i + 1);
    	 asprintf(&roach_state_table[i].amps_path[0], "/home/fc1user/sam_tests/roach%d_default_amps.dat", i + 1);
    	 asprintf(&roach_state_table[i].cal_log, "/home/fc1user/sam_tests/roach%d_qdr_cal.log", i + 1);
    	 asprintf(&roach_state_table[i].find_kids_log, "/home/fc1user/sam_tests/roach%d_find_kids.log", i + 1);
	 roach_state_table[i].lo_centerfreq = 750.0e6;
	 roach_state_table[i].which = i + 1;
    	 roach_state_table[i].dest_port = 64000 + i;
	 roach_state_table[i].is_streaming = 0;
	    roach_udp_networking_init(roach_state_table[i].which, &roach_state_table[i], NUM_ROACH_UDP_CHANNELS);
	}

    ph_thread_spawn((ph_thread_func)roach_cmd_loop, NULL);
    return 0;
}

void write_roach_channels_5hz(void)
{
    int i;
    static int firsttime = 1;
    static channel_t *RoachPktCtAddr[NUM_ROACHES];
    static channel_t *RoachValidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachInvalidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachStatusAddr[NUM_ROACHES];
    static channel_t *RoachStateAddr[NUM_ROACHES];
//  static channel_t *RoachReqLOFreqAddr[NUM_ROACHES];
    char channel_name_pkt_ct[128] = { 0 };
    char channel_name_valid_pkt_ct[128] = { 0 };
    char channel_name_invalid_pkt_ct[128] = { 0 };
    char channel_name_roach_status[128] = { 0 };
    char channel_name_roach_state[128] = { 0 };
    char channel_name_roach_req_lo[128] = { 0 };
    if (firsttime) {
        firsttime = 0;
        for (i = 0; i < NUM_ROACHES; i++) {
            snprintf(channel_name_pkt_ct, sizeof(channel_name_pkt_ct),
                    "packet_count_roach%d", i + 1);
            snprintf(channel_name_valid_pkt_ct,
                    sizeof(channel_name_valid_pkt_ct),
                    "packet_count_valid_roach%d", i + 1);
            snprintf(channel_name_invalid_pkt_ct,
                    sizeof(channel_name_invalid_pkt_ct),
                    "packet_count_invalid_roach%d", i + 1);
            snprintf(channel_name_roach_status,
                    sizeof(channel_name_roach_status), "status_roach%d", i + 1);
            snprintf(channel_name_roach_state, sizeof(channel_name_roach_state),
                    "stream_state_roach%d", i + 1);
            snprintf(channel_name_roach_req_lo,
                    sizeof(channel_name_roach_state), "freq_lo_req_roach%d",
                    i + 1);
            RoachPktCtAddr[i] = channels_find_by_name(channel_name_pkt_ct);
            RoachValidPktCtAddr[i] = channels_find_by_name(
                    channel_name_valid_pkt_ct);
            RoachInvalidPktCtAddr[i] = channels_find_by_name(
                    channel_name_invalid_pkt_ct);
            RoachStatusAddr[i] = channels_find_by_name(
                    channel_name_roach_status);
            RoachStateAddr[i] = channels_find_by_name(channel_name_roach_state);
        }
    }
    for (i = 0; i < NUM_ROACHES; i++) {
        SET_UINT32(RoachPktCtAddr[i], roach_udp[i].roach_packet_count);
        SET_UINT32(RoachValidPktCtAddr[i],
                roach_udp[i].roach_valid_packet_count);
        SET_UINT32(RoachInvalidPktCtAddr[i],
                roach_udp[i].roach_invalid_packet_count);
        SET_UINT16(RoachStatusAddr[i], roach_state_table[i].status);
        // TODO(laura/sam): Replace next write with a streaming status bitfield.
        SET_UINT16(RoachStateAddr[i], roach_state_table[i].is_streaming);
// This next statement causes a segfault for some reason.
// TODO(laura): Fix it!
//        SET_SCALED_VALUE(RoachReqLOFreqAddr[i], roach_state_table[i].lo_freq_req);
    }
}
