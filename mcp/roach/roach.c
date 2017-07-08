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
#include <linux/filter.h>
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
#include <errno.h>
#include <pthread.h>
// include "portable_endian.h"
#include <fftw3.h>
#include "mcp.h"
#include "katcp.h"
#include "katcl.h"
#include "blast.h"
#include "blast_time.h"
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

#define DDS_SHIFT 330 /* Firmware version dependent */
#define VNA_FFT_SHIFT 31 /* Controls FFT overflow behavior, for VNA SWEEP */
#define TARG_FFT_SHIFT 127
#define VNA 0 /* Sweep type */
#define TARG 1 /*Sweep type */
#define WRITE_INT_TIMEOUT 1000 /* KATCP write timeout */
#define UPLOAD_TIMEOUT 20000 /* KATCP upload timeout */
#define QDR_TIMEOUT 20000 /* Same as above */
#define LUT_BUFFER_LEN 2097152 /* Number of samples in time domain LUTs */
#define FPGA_SAMP_FREQ 256.0e6 /* FPGA clock rate */
#define DAC_SAMP_FREQ 512.0e6 /* MUSIC board DAC/ADC clock rate */
/* Frequency resolution of DAC tones */
#define DAC_FREQ_RES (2*DAC_SAMP_FREQ / LUT_BUFFER_LEN)
#define LO_STEP 1000 /* Freq step size for sweeps */
#define TARG_SWEEP_SPAN 200.0e3 /* Target sweep span */
#define NGRAD_POINTS 6 /* Number of points to use for gradient sweep */
#define NZEROS 12 /* Half the number of filter coefficients */
#define N_AVG 50 /* Number of packets to average for each sweep point */
#define NC1_PORT 12345 /* Beaglebone com port for FC1 */
#define NC2_PORT 12346 /* Beaglebone com port for FC2 */
#define SWEEP_INTERRUPT (-1)
#define SWEEP_SUCCESS (1)
#define SWEEP_FAIL (0)
#define BB_READ_NTRIES 10 /* Number of times to attempt BB buffer read */
#define BB_READ_TIMEOUT (1000000) /* BB read timeout, usec */
#define LO_READ_TIMEOUT (500000) /* BB read timeout, usec */
#define INIT_VALON_TIMEOUT (500000) /* BB read timeout, usec */

extern int16_t InCharge;
static int fft_len = 1024;
static uint32_t accum_len = (1 << 19) - 1;
// Test frequencies for troubleshooting (arbitrary values)
double test_freq[] = {20.0125e6};

// Order 22 (length 23) Hann window coefficients, f_cutoff = 10 kHz
double zeros[12] = {0.0, 0.00145215, 0.0060159, 0.01373059, 0.02425512, 0.03688533, 0.05061838, 0.06425732,
		0.07654357, 0.08630093, 0.09257286, 0.09473569};
// double zeros[14] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};
// Firmware image files
const char roach_fpg[5][50] = {"/data/etc/blast/r1_fir_dds330.fpg", "/data/etc/blast/r2_fir_dds330.fpg",
		"/data/etc/blast/r3_fir_dds330.fpg", "/data/etc/blast/r4_fir_dds330.fpg",
					"/data/etc/blast/r5_fir_dds330.fpg"};
static roach_state_t roach_state_table[NUM_ROACHES];
static bb_state_t bb_state_table[NUM_ROACHES];
static rudat_state_t rudat_state_table[NUM_ROACHES];
static valon_state_t valon_state_table[NUM_ROACHES];
// static ph_thread_t *roach_state = NULL;
char valon_init_bb[] = "python /root/device_control/init_valon.py";
char read_valon_bb[] = "python /root/device_control/read_valon.py";
char valon_init_pi[] = "python /home/pi/device_control/init_valon.py";
char read_valon_pi[] = "python /home/pi/device_control/read_valon.py";
char vna_search_path[] = "/home/fc1user/sam_tests/sweeps/roach1/vna/Sun_Feb_19_19_20_00_2017";
char targ_search_path[] = "/home/fc1user/sam_tests/sweeps/roach1/targ/Sun_Feb_19_19_20_00_2017";

/* Controls access to the fftw3 */
static pthread_mutex_t fft_mutex;

static uint32_t dest_ip = IPv4(192, 168, 40, 4);

void nameThread(const char*);

int roach_qdr_cal(roach_state_t *m_roach)
{
	char *m_cal_command;
	char m_line[256];
    char *freq;
    blast_tmp_sprintf(m_cal_command, "python /data/etc/blast/cal_roach_qdr.py %s > %s",
					m_roach->address, m_roach->qdr_log);
	system(m_cal_command);
	sleep(5);
	FILE *fd = fopen(m_roach->qdr_log, "r");
	if (!fd) return 0;
	// while (fgets(m_line, sizeof(m_line), fd)) {
	// blast_info("%s", m_line);
	// }
	if (fgets(m_line, sizeof(m_line), fd)) {
	    blast_info("%s", m_line);
       freq = strchr(m_line, '=');
       freq += 2; // Remove equals sign and space from string
    }
    fclose(fd);
    double clock_freq = atof(freq);
    if ((clock_freq < 254.0) || (clock_freq > 258.0)) {
        blast_info("ROACH%d FPGA CLOCK NOT SET: Reinitialize Valon%d ?", m_roach->which, m_roach->which);
    }
    // blast_info("Freq = %f", clock_freq);
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
    size_t comb_fft_len;
    /*
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
    */
    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
	// blast_info("m_freq = %g", m_freqs[i]);
    }

    comb_fft_len = LUT_BUFFER_LEN;
    complex double *spec = calloc(comb_fft_len, sizeof(complex double));
    complex double *wave = calloc(comb_fft_len, sizeof(complex double));
    double alpha;
    double max_val = 0.0;
    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        int bin = roach_fft_bin_index(m_freqs, i, comb_fft_len, m_samp_freq);
        if (bin < 0) {
            bin += comb_fft_len;
	}
	alpha = drand48() * 2.0 * M_PI;
        spec[bin] = cexp(_Complex_I * alpha);
	// blast_info("bin = %d, r(spec[bin]) = %f, im(spec[bin]) = %f",
						// bin, creal(spec[bin]), cimag(spec[bin]));
    }
    pthread_mutex_lock(&fft_mutex);
    m_roach->comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD,
    FFTW_ESTIMATE);
    fftw_execute(m_roach->comb_plan);
    fftw_destroy_plan(m_roach->comb_plan);
    pthread_mutex_unlock(&fft_mutex);
    for (size_t i = 0; i < comb_fft_len; i++) {
    	wave[i] /= comb_fft_len;
    	// fprintf(f2,"%f, %f\n", creal(wave[i]), cimag(wave[i]));
	if (cabs(wave[i]) > max_val) max_val = cabs(wave[i]);
     }
    // fclose(f2);
    for (size_t i = 0; i < comb_fft_len; i++) {
        m_I[i] = creal(wave[i]) / max_val * ((1 << 15) - 1);
        m_Q[i] = cimag(wave[i]) / max_val * ((1 << 15) - 1);
    }
    free(spec);
    free(wave);
    return 0;
    // TODO(Sam/Laura) This is the newer FFTW3 implementation - the dds_comb() function returns NAN
    /* size_t comb_fft_len;
    comb_fft_len = LUT_BUFFER_LEN;
    fftw_complex *spec = (fftw_complex*) fftw_malloc(comb_fft_len * sizeof(fftw_complex));
    fftw_complex *wave = (fftw_complex*) fftw_malloc(comb_fft_len * sizeof(fftw_complex));
    double alpha;
    double max_val = 0.0;
    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        int bin = roach_fft_bin_index(m_freqs, i, comb_fft_len, m_samp_freq);
        if (bin < 0) {
            bin += comb_fft_len;
	    }
	    alpha = drand48() * 2.0 * M_PI;
        spec[bin][0] = creal(cexp(_Complex_I * alpha));
        spec[bin][1] = cimag(cexp(_Complex_I * alpha));
	// blast_info("bin = %d, r(spec[bin]) = %f, im(spec[bin]) = %f",
						// bin, creal(spec[bin]), cimag(spec[bin]));
    }
    pthread_mutex_lock(&fft_mutex);
    m_roach->comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(m_roach->comb_plan);
    fftw_destroy_plan(m_roach->comb_plan);
    pthread_mutex_unlock(&fft_mutex);
    for (size_t i = 0; i < comb_fft_len; i++) {
    	wave[i][0] /= comb_fft_len;
    	wave[i][1] /= comb_fft_len;
	if (cabs(wave[i][0] + wave[i][1]* _Complex_I) > max_val) max_val = cabs(wave[i][0] + wave[i][1] * _Complex_I);
    }
    for (size_t i = 0; i < comb_fft_len; i++) {
        m_I[i] = wave[i][0] / max_val * ((1 << 15) - 1);
        m_Q[i] = wave[i][1] / max_val * ((1 << 15) - 1);
    }
    fftw_free(spec);
    fftw_free(wave);
    return 0; */
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
	// blast_info("DDS spec = %f, %f", creal(spec[m_bin]), cimag(spec[m_bin]));
    pthread_mutex_lock(&fft_mutex);
    comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(comb_plan);
    fftw_destroy_plan(comb_plan);
    pthread_mutex_unlock(&fft_mutex);
    for (size_t i = 0; i < comb_fft_len; i++) {
	// blast_info("%f, %f\n", creal(wave[i]), cimag(wave[i]));
       	if (cabs(wave[i]) > max_val) max_val = cabs(wave[i]);
    }
    for (size_t i = 0; i < comb_fft_len; i++) {
	m_I[i] = creal(wave[i]) / max_val * ((1<<15)-1);
       	m_Q[i] = cimag(wave[i]) / max_val * ((1<<15)-1);
    }
    free(spec);
    free(wave);
    return 0;
    // TODO(Sam/Laura) This is the newer FFTW3 implementation - wave[i] is all NANs?
    /* size_t comb_fft_len;
    comb_fft_len = LUT_BUFFER_LEN / fft_len;
    fftw_complex *spec = (fftw_complex*) fftw_malloc(comb_fft_len * sizeof(fftw_complex));
    fftw_complex *wave = (fftw_complex*) fftw_malloc(comb_fft_len * sizeof(fftw_complex));
    double alpha = 0.0;
    double max_val = 0.0;
    spec[m_bin][0] = creal(cexp(_Complex_I * alpha));
    spec[m_bin][1] = cimag(cexp(_Complex_I * alpha));
	// blast_info("DDS spec = %f, %f", creal(spec[m_bin]), cimag(spec[m_bin]));
    pthread_mutex_lock(&fft_mutex);
    fftw_plan comb_plan;
    comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(comb_plan);
    fftw_destroy_plan(comb_plan);
    pthread_mutex_unlock(&fft_mutex);
    for (size_t i = 0; i < comb_fft_len; i++) {
        blast_info("%f, %f\n", wave[i][0], wave[i][1]);
        if (cabs(wave[i][0] + wave[i][1]* _Complex_I) > max_val) max_val = cabs(wave[i][0] + wave[i][1]* _Complex_I);
	}
	// fclose(f4);
    for (size_t i = 0; i < comb_fft_len; i++) {
	m_I[i] = wave[i][0] / max_val * ((1<<15)-1);
	// blast_info("I = %g", m_I[i]);
        m_Q[i] = wave[i][1] / max_val * ((1<<15)-1);
	// blast_info("Q = %g", m_Q[i]);
    }
    fftw_free(spec);
    fftw_free(wave);
    return 0; */
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
	/* if (m_freqs[i] < 0 && m_freqs[i]+ 512.0e6 >= 511.5e6) {
		bins[i] = 1023;
	}*/
	m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / (DAC_FREQ_RES)) * (DAC_FREQ_RES);
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
    	// roach_define_DDS_LUT(m_roach, m_freqs, m_freqlen);
    	// roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);
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
    roach_write_int(m_roach, "downsamp_sync_accum_reset", 0, 0);
    roach_write_int(m_roach, "downsamp_sync_accum_reset", 1, 0);
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
		blast_tmp_sprintf(reg, "FIR_h%d", i);
		roach_write_int(m_roach, reg, (int32_t)m_zeros[i], 0);
	}
	blast_info("ROACH%d, Uploaded FIR coefficients", m_roach->which);
}

int bb_read_string(bb_state_t *m_bb, int ntries, int timeout)
{
    int retval = -1;
    unsigned char m_read_buffer[4096];
    int bytes_read;
	int count = 0;
    while ((count < ntries)) {
	    if (!ph_bufq_len(m_bb->bb_comm->input_buffer)) {
            usleep(BB_READ_TIMEOUT);
            count += 1;
        } else {
            while (ph_bufq_len(m_bb->bb_comm->input_buffer)) {
                size_t m_size = (size_t)ph_bufq_len(m_bb->bb_comm->input_buffer);
                bytes_read = remote_serial_read_data(m_bb->bb_comm, m_read_buffer, m_size);
                m_read_buffer[bytes_read++] = '\0';
                blast_info("BB%d: %s", m_bb->which, m_read_buffer);
	        }
            retval = 1;
            break;
        }
    }
    return retval;
}

// returns either 0 (fail) or 1 (success) as an integer, from Python on BB or Pi
int bb_response(bb_state_t *m_bb)
{
	int retval = -1;
	unsigned char m_read_buffer[4096];
	int bytes_read;
	while (ph_bufq_len(m_bb->bb_comm->input_buffer)) {
		size_t m_size = (size_t)ph_bufq_len(m_bb->bb_comm->input_buffer);
		bytes_read = remote_serial_read_data(m_bb->bb_comm, m_read_buffer, m_size);
		m_read_buffer[bytes_read++] = '\0';
		// blast_info("BB%d: %s", m_bb->which, m_read_buffer);
		retval = atoi((const char*)m_read_buffer);
	}
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
/* int cal_atten(roach_state_t *m_roach)
{
	int pre_amp = 20;
	// int pre_adc = 20;
	// int lower = 0.0;
	double upper = 0.0;
	double rms_voltage = roach_read_adc(m_roach);
	while (rms_voltage < upper) {
	    char *atten_command;
		pre_amp += 0.5;
		// TODO(Sam) Make sure the order of attenuators is correct here
		blast_tmp_sprintf(atten_command, "python ~/init_attenuators.py %d %d", pre_amp, 30);
		blast_info("Setting BB%d pre-amp attenuation: %d dB", m_roach->which - 1, pre_amp);
		bb_write_string(&bb_state_table[m_roach->which - 1], (unsigned char*)atten_command, strlen(atten_command));
		while (bb_read_string(&bb_state_table[m_roach->which - 1]) <= 0) {
		sleep(3);
		}
		rms_voltage = roach_read_adc(m_roach);
	}
	return 0;
} */

int set_atten(rudat_state_t *m_rudat)
{
	// TODO(Sam) Put in error handling
    int retval = -1;
    char *m_command;
    char *m_command2;
    int ind = m_rudat->which - 1;
    if ((ind >= 3)) {
	blast_info("BB%d, attempting to set RUDATs...", ind + 1);
        blast_tmp_sprintf(m_command, "sudo ./dual_RUDAT %g %g > rudat.log",
		CommandData.roach_params[m_rudat->which - 1].in_atten,
		CommandData.roach_params[m_rudat->which - 1].out_atten);
        blast_tmp_sprintf(m_command2, "cat rudat.log");
        bb_write_string(&bb_state_table[ind], (unsigned char*)m_command, strlen(m_command));
        bb_write_string(&bb_state_table[ind], (unsigned char*)m_command2, strlen(m_command2));
    	if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, BB_READ_TIMEOUT) < 1) {
		    blast_info("Error setting Atten... reboot BB%d?", ind + 1);
        } else {
            retval = 1;
        }
    } else {
        blast_info("BB%d, attempting to set RUDATs...", ind + 1);
        blast_tmp_sprintf(m_command, "python /root/device_control/init_attenuators.py %g %g",
		CommandData.roach_params[ind].in_atten,
		CommandData.roach_params[ind].out_atten);
        bb_write_string(&bb_state_table[ind], (unsigned char*)m_command, strlen(m_command));
    	if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, BB_READ_TIMEOUT) < 1) {
		    blast_info("Error setting Atten... reboot BB%d?", ind + 1);
        } else {
            retval = 1;
	    }
    }
	return retval;
}

/*int init_valon(roach_state_t *m_roach, int ntries)
{
    int retval = -1;
    char *m_command;
    int count = 0;
    int success = 0; // read from nc service (equals 0 or 1)
    int ind = m_roach->which - 1;
    if (ind == 3) {
        blast_info("Initializing Valon%d...", ind + 1);
        // try to initialize the Valon: Python returns 0 on fail, 1 on success, read value from log
        while ((count < ntries)) {
            bb_write_string(&bb_state_table[ind], (unsigned char*)valon_init_4, strlen(valon_init_4));
            blast_tmp_sprintf(m_command, "cat /home/pi/device_control/init_valon.log");
            bb_write_string(&bb_state_table[ind], (unsigned char*)m_command, strlen(m_command));
	        success = bb_response(&bb_state_table[ind]);
            if (success < 1) {
                usleep(INIT_VALON_TIMEOUT);
                count += 1;
            } else {
                retval = 1;
                break;
            }
        }
    } else {
        blast_info("Initializing Valon%d...", ind + 1);
        bb_write_string(&bb_state_table[ind], (unsigned char*)valon_init_command, strlen(valon_init_command));
        retval = 1;
        }
    return retval;
}*/

int init_valon(roach_state_t *m_roach)
{
    int retval = 1;
    int ind = m_roach->which - 1;
    blast_info("Initializing Valon%d...", ind + 1);
    if (ind >= 3) {
        bb_write_string(&bb_state_table[ind], (unsigned char*)valon_init_pi, strlen(valon_init_pi));
    } else {
	bb_write_string(&bb_state_table[ind], (unsigned char*)valon_init_bb, strlen(valon_init_bb));
    }
    return retval;
}

/*int read_valon(roach_state_t *m_roach, int ntries)
{
	int retval = -1;
    char *m_command;
	char *m_command2;
	int count = 0;
    int success = 0; // read from nc service (equals 0 or 1)
    int ind = m_roach->which - 1;
    if (ind == 3) {
        // try to initialize the Valon: Python returns 0 on fail, 1 on success, read value from log
        while ((count < ntries)) {
            bb_write_string(&bb_state_table[ind], (unsigned char*)read_valon_4, strlen(read_valon_4));
            usleep(1000);
            blast_tmp_sprintf(m_command, "cat /home/pi/device_control/read_valon.log");
            bb_write_string(&bb_state_table[ind], (unsigned char*)m_command, strlen(m_command));
	        success = bb_response(&bb_state_table[ind]);
            if (success < 1) {
                count += 1;
                sleep(2);
            } else {
                blast_tmp_sprintf(m_command2, "cat /home/pi/device_control/valon_response.log");
                bb_write_string(&bb_state_table[ind], (unsigned char*)m_command2, strlen(m_command2));
                if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, BB_READ_TIMEOUT) < 1) {
                    blast_info("Error reading Valon... reboot BB%d?", ind + 1);
                } else {
                    retval = 1;
                    break;
                }
            }
        }
     } else {
        bb_write_string(&bb_state_table[ind], (unsigned char*)read_valon_command, strlen(read_valon_command));
        if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, BB_READ_TIMEOUT) < 1) {
		    blast_info("Error reading Valon... reboot BB%d?", ind + 1);
        } else {
            retval = 1;
            }
        }
    return retval;
}*/

int read_valon(roach_state_t *m_roach, int ntries)
{
    int retval = -1;
    int ind = m_roach->which - 1;
    if (ind >= 3) {
        bb_write_string(&bb_state_table[ind], (unsigned char*)read_valon_pi, strlen(read_valon_pi));
    } else {
        bb_write_string(&bb_state_table[ind], (unsigned char*)read_valon_bb, strlen(read_valon_bb));
    }
    if (bb_read_string(&bb_state_table[ind], ntries, BB_READ_TIMEOUT) < 1) {
        blast_info("Error reading Valon... reboot BB%d?", ind + 1);
    } else {
    	retval = 1;
    }
    return retval;
}
/* Check if UDP streaming is successful */
int roach_check_streaming(roach_state_t *m_roach)
{
	int m_last_packet_count = roach_udp[m_roach->which - 1].roach_packet_count;
	blast_info("m_last_packet_count = % d", m_last_packet_count);
	while ((m_roach->is_streaming != 1)) {
		/* Run for 10 seconds and check to see if packet count has incremented */
		blast_info("roach_packet_count = % d", roach_udp[m_roach->which - 1].roach_packet_count);
		sleep(10);
		if (roach_udp[m_roach->which - 1].roach_packet_count > m_last_packet_count) {
			m_roach->is_streaming = 1;
			return 0;
		} else { blast_err("Data stream error on ROACH%d", m_roach->which);
			return -1;
		}
	}
	return 0;
}

void roach_vna_comb(roach_state_t *m_roach)
{
	double p_delta_f;
	double n_delta_f;
	m_roach->vna_comb = calloc(m_roach->vna_comb_len, sizeof(double));
	/* positive freqs */
	p_delta_f = (m_roach->p_max_freq - m_roach->p_min_freq) / ((m_roach->vna_comb_len/2.) - 1);
	/* Store delta f, to be used as 'span' in roach_do_sweep */
	m_roach->vna_sweep_span = p_delta_f;
	// blast_info("p_max = %g, p_min = %g, n_max = %g, n_min = %g",
			// m_roach->p_max_freq, m_roach->p_min_freq, m_roach->n_max_freq, m_roach->n_min_freq);
	// blast_info("Sweep p_delta_f = %g, delta f = %g", p_delta_f, m_roach->vna_sweep_span);
	for (size_t i = m_roach->vna_comb_len/2; i-- > 0;) {
		m_roach->vna_comb[m_roach->vna_comb_len/2 - (i + 1)] = m_roach->p_max_freq - i*p_delta_f;
	}
	/* negative freqs */
	n_delta_f = (m_roach->n_max_freq - m_roach->n_min_freq) / ((m_roach->vna_comb_len/2.) - 1);
	for (size_t i = 0; i < m_roach->vna_comb_len/2; i++) {
		m_roach->vna_comb[i + m_roach->vna_comb_len/2] = m_roach->n_min_freq + i*n_delta_f;
	}
	for (size_t i = 0; i < m_roach->vna_comb_len; i++) {
		// blast_info("ROACH%d vna freq = %g", m_roach->which, m_roach->vna_comb[i]);
	}
    /* for (size_t i = 0; i < m_roach->vna_comb_len; i++) {
        m_roach->vna_comb[i] = round(m_roach->vna_comb[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
	    blast_info("VNA freq = %g", m_roach->vna_comb[i]/DAC_FREQ_RES);
    }*/
}

/* Save list of either VNA or TARG frequencies */
void save_freqs(roach_state_t *m_roach, char *m_save_path, double *m_freqs, size_t m_Nfreqs)
{
	FILE *fd = fopen(m_save_path, "w");
	if (!fd) {
	    blast_strerror("Could not open %s for writing", m_save_path);
	    return;
	}
	for (size_t i = 0; i < m_Nfreqs; i++) {
        fprintf(fd, "%.10f\n", (float)m_freqs[i]);
	}
	fclose(fd);
	blast_info("ROACH%d Freqs written to %s", m_roach->which, m_save_path);
}

void roach_save_sweep_packet(roach_state_t *m_roach, uint32_t m_sweep_freq, char *m_sweep_save_path, size_t m_Nfreqs)
{
	/* Grab a set number of packets, average I and Q for each chan,
	and save the result to file:fname in the sweep dir (/data/etc/blast/sweeps/) */
	/* Save I_avg, Q_avg, to sweep dir */
	int m_num_received = 0;
	int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
	uint8_t i_udp_read;
	char *fname;
	blast_tmp_sprintf(fname, "%s/%d.dat", m_sweep_save_path, m_sweep_freq);
	FILE *m_fd = fopen(fname, "w");
	if (!m_fd) {
        blast_strerror("Could not open %s for writing", fname);
        return;
	}
	double *I_sum = calloc(m_Nfreqs, sizeof(double)); // Array to store I values to be summed
	double *Q_sum = calloc(m_Nfreqs, sizeof(double)); // Array to store Q values to be summed
	double *I_avg = calloc(m_Nfreqs, sizeof(double)); // Array to store averaged I values
	double *Q_avg = calloc(m_Nfreqs, sizeof(double)); // Array to store averaged Q values
	while (m_num_received < N_AVG) {
		usleep(3000);
		if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
		   m_num_received++;
		   i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
		   data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
		   for (size_t chan = 0; chan < m_Nfreqs; chan ++) {
			I_sum[chan] +=  m_packet.Ival[chan];
			Q_sum[chan] +=  m_packet.Qval[chan];
		   }
		}
		m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
	}
	for (size_t chan = 0; chan < m_Nfreqs; chan++) {
		I_avg[chan] = (I_sum[chan] / N_AVG);
		Q_avg[chan] = (Q_sum[chan] / N_AVG);
		/* Save I_avg, Q_avg, to sweep dir */
		fprintf(m_fd, "%zd\t %g\t %g\n", chan, I_avg[chan], Q_avg[chan]);
		// blast_info("%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
	}
	free(I_sum);
	free(Q_sum);
	free(I_avg);
	free(Q_avg);
	fclose(m_fd);
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

char* get_path(roach_state_t *m_roach, char *m_dir_root)
{
    char time_buffer[FILENAME_MAX];
    char *save_path;
    get_time(time_buffer);
    asprintf(&save_path, "%s/%s", m_dir_root, time_buffer);
    return save_path;
}

static int mkdir_recursive(char *m_directory)
{
    char current_path[PATH_MAX];
    char *path_chunk = NULL;
    char *strtok_ref = NULL;
    int	ret = 0;
    int stat_return = 0;
    char *path_ptr;
    size_t path_chunk_len = 0;
    size_t current_path_len = 0;
    struct stat dir_stat;
    current_path[0]=0;
    path_ptr = current_path;  // We use this to keep track of where we should be writing in current_path;

    // Takes a chunk of the directory path at a time
    path_chunk = strtok_r(m_directory, "/", &strtok_ref);
    if (m_directory[0] == '/') {
	snprintf(path_ptr, sizeof("/"), "/");
	path_ptr++;
    }
    while (path_chunk != NULL) {
        current_path_len = strlen(current_path);
        path_chunk_len = strlen(path_chunk);
	if (current_path_len + path_chunk_len + 2 > PATH_MAX) {
	    blast_err("Path too long");
            return -1;
        }
	snprintf(path_ptr, path_chunk_len + 1, path_chunk); // The +1 is for the '\0'
	path_ptr += path_chunk_len;
	snprintf(path_ptr, sizeof("/"), "/");
	path_ptr++;
	current_path_len = strlen(current_path);
        stat_return = stat(current_path, &dir_stat);
        if (stat_return != 0) {
	    blast_info("Making path %s", current_path);
	    ret = mkdir(current_path, ACCESSPERMS);
	    if (ret < 0) {
	        blast_strerror("Could not make %s", current_path);
	        return ret;
	    } else { ret = 1; }
        }
	path_chunk = strtok_r(NULL, "/", &strtok_ref);
    }
    return ret;
}

int create_sweepdir(roach_state_t *m_roach, int sweep_type)
{
    int retval = -1;
    int ind = m_roach->which - 1;
    char* new_path;
    char *path_root;
    char *type;
    if ((sweep_type == VNA)) {
        path_root = m_roach->vna_path_root;
	type = "VNA";
    }
    if ((sweep_type = TARG)) {
        path_root = m_roach->targ_path_root;
	type = "TARGET";
    }
    new_path = get_path(m_roach, path_root);
    if (sweep_type == VNA) {
	    asprintf(&m_roach->last_vna_path, new_path);
    } else { asprintf(&m_roach->last_targ_path, new_path); }
    blast_info("ROACH%d, New %s sweep will be saved in %s", ind + 1, type, new_path);
    if (mkdir_recursive(new_path)) {
        retval = 1;
    } else {
        blast_strerror("Could not create new directory: %s", new_path);
    }
    return retval;
}

int get_targ_freqs(roach_state_t *m_roach, char *m_last_vna_path, char* m_last_targ_path)
{
    char *py_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    char m_line[256];
    blast_tmp_sprintf(py_command,
            "python /data/etc/blast/find_kids_blast.py %s %s %g %g %g > %s",
		m_last_vna_path,
		m_last_targ_path,
		CommandData.roach_params[m_roach->which - 1].smoothing_scale,
		CommandData.roach_params[m_roach->which - 1].peak_threshold,
		CommandData.roach_params[m_roach->which - 1].spacing_threshold,
		m_roach->find_kids_log);
    blast_info("%s", py_command);
    system(py_command);
    sleep(3);
    FILE *log = fopen(m_roach->find_kids_log, "r");
    if (!log) {
        blast_strerror("Could not open %s for reading", m_roach->find_kids_log);
        return -1;
    }
    while (fgets(m_line, sizeof(m_line), log)) {
        blast_info("%s", m_line);
    }
    fclose(log);
    /*blast_tmp_sprintf(py_command, "python /data/etc/blast/copy_file.py %s", m_last_targ_path);
    blast_info("%s", py_command);
    system(py_command);*/
    blast_tmp_sprintf(m_targ_freq_path, "%s/bb_targ_freqs.dat", m_last_targ_path);
    FILE *fd;
    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return -1;
    }
    m_roach->num_kids = 0;
    while (m_roach->num_kids < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &m_temp_freqs[(m_roach->num_kids)++]) != EOF) {
    }
    fclose(fd);
    if (m_roach->num_kids > 0) {
    	(m_roach->num_kids)--;
    } else {
    	return -1;
    }
    blast_info("NUM kids = %zd", m_roach->num_kids);
    m_roach->targ_tones = calloc(m_roach->num_kids, sizeof(double));
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j];
        // m_roach->targ_tones[j] = 1.0e7;
	blast_info("KID freq = %lg", m_roach->targ_tones[j] + m_roach->lo_centerfreq);
    }
    if (CommandData.roach[m_roach->which - 1].find_kids) {
    	CommandData.roach[m_roach->which - 1].find_kids = 0;
    }
    return 1;
}

int optimize_targ_tones(roach_state_t *m_roach, char *m_last_targ_path)
{
    char *py_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    char m_line[256];
    blast_tmp_sprintf(py_command,
            "python /data/etc/blast/optimize_freqs_mcp.py %s > %s",
		m_last_targ_path, m_roach->opt_tones_log);
    blast_info("%s", py_command);
    system(py_command);
    sleep(3);
    FILE *log = fopen(m_roach->opt_tones_log, "r");
    if (!log) {
        blast_strerror("Could not open %s for reading", m_roach->opt_tones_log);
        return -1;
    }
    while (fgets(m_line, sizeof(m_line), log)) {
        blast_info("%s", m_line);
    }
    fclose(log);
    blast_tmp_sprintf(m_targ_freq_path, "%s/bb_targ_freqs.dat", m_last_targ_path);
    FILE *fd;
    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return -1;
    }
    m_roach->num_kids = 0;
    while (m_roach->num_kids < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &m_temp_freqs[(m_roach->num_kids)++]) != EOF) {
    }
    fclose(fd);
    if (m_roach->num_kids > 0) {
    	(m_roach->num_kids)--;
    } else {
    	return -1;
    }
    blast_info("NUM kids = %zd", m_roach->num_kids);
    m_roach->targ_tones = calloc(m_roach->num_kids, sizeof(double));
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j];
        // m_roach->targ_tones[j] = 1.0e7;
	blast_info("Optimized KID freq = %lg", m_roach->targ_tones[j]);
    }
    if (CommandData.roach[m_roach->which - 1].find_kids) {
    	CommandData.roach[m_roach->which - 1].find_kids = 0;
    }
    return 1;
}

/* int type: 0 for VNA sweep, 1 for TARG sweep */
int roach_do_sweep(roach_state_t *m_roach, int sweep_type)
{
    int retval = SWEEP_SUCCESS;
    int ind = m_roach->which - 1;
    bb_state_t *m_bb = &bb_state_table[ind];
    double m_span;
    char *sweep_freq_fname;
    char *save_path;
    size_t comb_len;
    struct stat dir_stat;
    int stat_return;
    if ((sweep_type == VNA)) {
        char *vna_freq_fname;
        if (create_sweepdir(m_roach, VNA) && create_sweepdir(m_roach, TARG)) {
	    m_span = m_roach->vna_sweep_span;
	    blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_vna_path);
	    blast_tmp_sprintf(vna_freq_fname, "%s/vna_freqs.dat", m_roach->last_vna_path);
	    save_freqs(m_roach, vna_freq_fname, m_roach->vna_comb, m_roach->vna_comb_len);
            save_path = m_roach->last_vna_path;
	    comb_len = m_roach->vna_comb_len;
	} else {
	    return SWEEP_FAIL;
        }
    }
    if ((sweep_type == TARG)) {
        stat_return = stat(m_roach->last_targ_path, &dir_stat);
        if (stat_return != 0) {
            if (create_sweepdir(m_roach, TARG)) {
                blast_info("ROACH%d, TARGET sweep will be saved in %s", m_roach->which, m_roach->last_targ_path);
            } else {
	       return SWEEP_FAIL;
            }
        }
        save_path = m_roach->last_targ_path;
        blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_targ_path);
	blast_info("Sweep freq fname = %s", sweep_freq_fname);
	// save_freqs(m_roach, targ_freq_fname, m_roach->targ_tones, m_roach->num_kids);
	blast_info("Uploading TARGET comb...");
	roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
	comb_len = m_roach->num_kids;
	blast_info("ROACH%d, TARGET comb uploaded", m_roach->which);
    }
    /* Determine sweep frequencies */
    FILE *m_sweep_fd = fopen(sweep_freq_fname, "w");
    if (!m_sweep_fd) {
        blast_strerror("Could not open %s for writing", sweep_freq_fname);
        return SWEEP_FAIL;
    }
    double m_min_freq = m_roach->lo_centerfreq - (m_span/2.);
    double m_max_freq = m_roach->lo_centerfreq + (m_span/2.);
    size_t m_num_sweep_freqs = (m_max_freq - m_min_freq) / LO_STEP;
    char *lo_command; /* BB command */
    double *m_sweep_freqs = calloc(m_num_sweep_freqs, sizeof(double));
    m_sweep_freqs[0] = m_min_freq;
    for (size_t i = 1; i < m_num_sweep_freqs; i++) {
	m_sweep_freqs[i] = m_sweep_freqs[i - 1] + LO_STEP;
    }
    for (size_t i = 0; i < m_num_sweep_freqs; i++) {
	m_sweep_freqs[i] = round(m_sweep_freqs[i] / LO_STEP) * LO_STEP;
	fprintf(m_sweep_fd, "%d\n", (uint32_t)m_sweep_freqs[i]);
    }
    fclose(m_sweep_fd);
    blast_info("ROACH%d, Sweep freqs written to %s", m_roach->which, sweep_freq_fname);
    blast_info("ROACH%d Starting new sweep...", m_roach->which);
    /* Sweep and save data */
    for (size_t i = 0; i < m_num_sweep_freqs; i++) {
	if (CommandData.roach[ind].do_sweeps) {
        if ((ind >= 3)) {
            blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g\n",
		m_sweep_freqs[i]/1.0e6);
            } else {
                blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n",
		m_sweep_freqs[i]/1.0e6);
	    }
        m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
        bb_write_string(m_bb, (unsigned char*)lo_command, strlen(lo_command));
    	if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, LO_READ_TIMEOUT) < 1) {
	blast_info("Error setting LO... reboot BB%d?", ind + 1);
        }
        usleep(10000);
        roach_save_sweep_packet(m_roach, (uint32_t)m_sweep_freqs[i], save_path, comb_len);
	} else {
        blast_info("Sweep interrupted by command");
        retval = SWEEP_INTERRUPT;
        break;
        }
    }
    usleep(3000);
    if ((ind >= 3)) {
        blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g\n", m_roach->lo_centerfreq/1.0e6);
    } else {
        blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n", m_roach->lo_centerfreq/1.0e6);
    }
	bb_write_string(m_bb, (unsigned char*)lo_command, strlen(lo_command));
	// TODO(Sam) Put in error handling
    if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, LO_READ_TIMEOUT) < 1) {
        blast_info("Error setting LO... reboot BB%d?", ind + 1);
    }
	free(m_sweep_freqs);
	return retval;
}

/* Small sweep to calculate I/Q gradient */
void cal_sweep(roach_state_t *m_roach)
{
    char* new_path;
    int ind = m_roach->which - 1;
    bb_state_t *m_bb = &bb_state_table[ind];
    new_path = get_path(m_roach, m_roach->cal_path_root);
    asprintf(&m_roach->last_cal_path, new_path);
    if (mkdir_recursive(new_path)) {
        free(new_path);
    } else {
        blast_strerror("Could not create new directory: %s", m_roach->last_cal_path);
        free(m_roach->last_cal_path);
    }
	char *lo_command; /* BB command */
	double m_sweep_freqs[NGRAD_POINTS];
	m_sweep_freqs[0] = m_roach->lo_centerfreq - LO_STEP;
	for (size_t i = 1; i < NGRAD_POINTS; i++) {
		m_sweep_freqs[i] = m_sweep_freqs[i - 1] + LO_STEP;
	}
	for (size_t i = 0; i < NGRAD_POINTS; i++) {
		if (CommandData.roach[ind].do_sweeps) {
			// Todo(Sam) Make sure this doesn't crash sweep
			blast_info("Sweep freq = %.7g", m_sweep_freqs[i]/1.0e6);
			m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
			if ((ind >= 3)) {
                blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g\n", m_sweep_freqs[i]/1.0e6);
            } else {
                blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n", m_sweep_freqs[i]/1.0e6);
			}
			bb_write_string(m_bb, (unsigned char*)lo_command, strlen(lo_command));
            if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, BB_READ_TIMEOUT) < 1) {
                blast_info("Error setting LO... reboot BB%d?", ind + 1);
            }
			roach_save_sweep_packet(m_roach, (uint32_t)m_sweep_freqs[i], m_roach->last_cal_path, m_roach->num_kids);
				usleep(1000);
		} else {
			break;
			blast_info("Sweep interrupted by command");
		}
    if ((ind >= 3)) {
        blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g\n", m_roach->lo_centerfreq/1.0e6);
    } else {
        blast_tmp_sprintf(lo_command, "python /root/device_control/set_lo.py %g\n", m_roach->lo_centerfreq/1.0e6);
    }
	bb_write_string(m_bb, (unsigned char*)lo_command, strlen(lo_command));
    if (bb_read_string(&bb_state_table[ind], BB_READ_NTRIES, BB_READ_TIMEOUT) < 1) {
        blast_info("Error setting LO... reboot BB%d?", ind + 1);
    }
	}
}

int grad_calc(roach_state_t *m_roach)
{
	/* Calculates a reference gradient, (dI/df, dQ/df) */
	/* Used to calculate delta_f, for determining whether or not to retune resonators */
	blast_info("ROACH%d, Calculating IQ GRADIENTS", m_roach->which);
	float Ival[m_roach->num_kids][NGRAD_POINTS];
	// float rf_freq[m_roach->num_kids][3];
	float Qval[m_roach->num_kids][NGRAD_POINTS];
	char *in_file;
	char m_time_buffer[4096];
	get_time(m_time_buffer);
	/* open files for reading */
	struct dirent **file_list;
	int nfiles;
	int freq_idx = 0;
	nfiles = scandir(m_roach->last_cal_path, &file_list, NULL, alphasort);
	/* Read I/Q data from cal sweep */
	if (nfiles < 0) {
		blast_strerror("Could not read %s", m_roach->last_cal_path);
	} else {
		for (int i = 0; i < nfiles; ++i) {
			if (!strcmp(file_list[i]->d_name, ".") || !strcmp(file_list[i]->d_name, "..")) {
				free(file_list[i]);
			} else {
				blast_tmp_sprintf(in_file, "%s/%s", m_roach->last_cal_path, file_list[i]->d_name);
         			free(file_list[i]);
				FILE* fd = fopen(in_file, "r");
				for(int kid = 0; kid < m_roach->num_kids; ++kid) {
					fscanf(fd, "%d\t%g\t%g\n", &kid, &Ival[kid][freq_idx], &Qval[kid][freq_idx]);
				}
				++freq_idx;
				printf("Read: %s\n", in_file);
				fclose(fd);
			}
		}
		if(file_list) free(file_list);
    	}
	/* To calculate reference gradient and ref delta_f */
	if ((!m_roach->has_ref)) {
		for (int kid = 0; kid < m_roach->num_kids; ++kid) {
			// printf("%.10f\t%.10f\t%.10f\n", Qval[kid][0], Qval[kid][1], Qval[kid][2]);
			m_roach->ref_grad[kid][0] = (Ival[kid][NGRAD_POINTS - 1] - Ival[kid][0]);
			m_roach->ref_grad[kid][1] = (Qval[kid][NGRAD_POINTS - 1] - Qval[kid][0]);
			m_roach->ref_df[kid] = ((Ival[kid][(NGRAD_POINTS - 1) / 2] * m_roach->ref_grad[kid][0]
						+ Qval[kid][(NGRAD_POINTS - 1) / 2] * m_roach->ref_grad[kid][1])
						/ (pow(m_roach->ref_grad[kid][0], 2) + pow(m_roach->ref_grad[kid][1], 2)))*LO_STEP;
			blast_info("R%d chan%d, df = %g", m_roach->which, kid, m_roach->ref_df[kid]);
		}
		m_roach->has_ref = 1;
		blast_info("ROACH%d, stored REF grads & delta F", m_roach->which);
	} else if ((CommandData.roach[m_roach->which - 1].df_calc == 2) && (m_roach->ref_grad)) {
		/* To calculate delta_f for comparison to ref delta_f (ref grads must already exist) */
		for (int kid = 0; kid < m_roach->num_kids; ++kid) {
			// printf("%.10f\t%.10f\t%.10f\n", Qval[kid][0], Qval[kid][1], Qval[kid][2]);
			m_roach->comp_df[kid] = ((Ival[kid][(NGRAD_POINTS - 1) / 2] *m_roach->ref_grad[kid][0]
						+ Qval[kid][(NGRAD_POINTS - 1) / 2] * m_roach->ref_grad[kid][1])
						/ (pow(m_roach->ref_grad[kid][0], 2) + pow(m_roach->ref_grad[kid][1], 2)))*LO_STEP;
			blast_info("ROACH%d, stored COMP delta F", m_roach->which);
		}
	} else {
		if ((!m_roach->ref_grad)) {
			blast_info("No reference gradients found. Try setting df_calc = 1");
		}
	}
	CommandData.roach[m_roach->which - 1].df_calc = 0;
	return 1;
}

static int roach_check_retune(roach_state_t *m_roach)
{
    	/* Compare current delta_f to reference delta_f (both stored in roach state) */
    	/* Retune if nflags exceeds nflags_threshold */
    	// TODO(Sam) determine threshold
    	blast_info("ROACH%d: Checking for retune...", m_roach->which - 1);
	int nflags;
    	double df_threshold = 1.0e4; // Hz
	int nflags_threshold = 300;
	if ((CommandData.roach[m_roach->which - 1].df_calc == 3) && (m_roach->ref_df) && (m_roach->comp_df)) {
		for (int kid = 0; kid < m_roach->num_kids; ++kid) {
			// printf("%.10f\t%.10f\t%.10f\n", Qval[kid][0], Qval[kid][1], Qval[kid][2]);
			m_roach->df_diff[kid] = fabs(m_roach->comp_df[kid] - m_roach->ref_df[kid]);
			if ((m_roach->df_diff[kid] > df_threshold)) {
				nflags++;
				m_roach->out_of_range[kid] = 1;
			} else {
				m_roach->out_of_range[kid] = 0;
			}
		blast_info("ROACH%d: %d kids have drifted", m_roach->which + 1, nflags);
		}
	} else {
		blast_info("Ref DF, Ref GRADs not found!");
			CommandData.roach[m_roach->which - 1].df_calc = 0;
			return -1;
     		}
	if ((nflags >= nflags_threshold)) {
		m_roach->retune_flag = 1;
		blast_info("ROACH%d: RETUNE RECOMMENDED", m_roach->which + 1);
	}
	CommandData.roach[m_roach->which - 1].df_calc = 0;
    return m_roach->retune_flag;
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
            // blast_info("Loading %s with %zu bytes", state->firmware_file,
            //        number_bytes);
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
    if (success_val != KATCP_RESULT_OK) {
    	return 0;
    } else {
    	return 1;
    }
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
	if (!roach_upload_status(m_roach));
    		return 0;
    	} else {
		return 1;
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
        blast_info("Closing KATCP on ROACH%d", i + 1);
        if (roach_state_table[i].rpc_conn) {
            // roach_write_int(&roach_state_table[i], "tx_rst", 1, 0);
            // roach_read_int(&roach_state_table[i], "tx_rst");
            destroy_rpc_katcl(roach_state_table[i].rpc_conn);
        }
        if (bb_state_table[i].bb_comm) {
            remote_serial_shutdown(bb_state_table[i].bb_comm);
            bb_state_table[i].bb_comm = NULL;
        }
    }
}

void *roach_cmd_loop(void* ind)
{
    	int status;
    	int i = *((uint16_t*) ind);
	char tname[10];
	if (snprintf(tname, sizeof(tname), "rcmd%i", i + 1) < 5) {
        blast_tfatal("Could not name thread for roach%i", i);
	}
	ph_thread_set_name(tname);
	nameThread(tname);
	static int first_time = 1;
	while (!InCharge) {
	    if (first_time) {
	        blast_info("roach%i: Waiting until we get control...", i+1);
	        first_time = 0;
	    }
	    usleep(2000);
	}
	blast_info("Starting Roach Commanding Thread");
	bb_state_table[i].status = BB_STATUS_BOOT;
	bb_state_table[i].desired_status = BB_STATUS_INIT;
	rudat_state_table[i].status = RUDAT_STATUS_BOOT;
	rudat_state_table[i].desired_status = RUDAT_STATUS_HAS_ATTENS;
	valon_state_table[i].status = VALON_STATUS_BOOT;
	valon_state_table[i].desired_status = VALON_STATUS_HAS_FREQS;
	roach_state_table[i].status = ROACH_STATUS_BOOT;
	roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
	while (!shutdown_mcp) {
		// TODO(SAM/LAURA): Fix Roach 1/Add error handling
		// Check for new roach status commands
			if (CommandData.roach[i].change_state) {
                		roach_state_table[i].status = CommandData.roach[i].new_state;
                		CommandData.roach[i].change_state = 0;
		    	}
		// Check for any additional roach commands
			if (CommandData.roach[i].do_sweeps == 0) {
				roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
			}
			// TODO(Sam) Add error checking
			if ((CommandData.roach[i].df_calc > 0) &&
					(roach_state_table[i].status >= ROACH_STATUS_TARG)) {
				if ((CommandData.roach[i].df_calc == 1)) {
					roach_state_table[i].has_ref = 0;
				}
				cal_sweep(&roach_state_table[i]);
				grad_calc(&roach_state_table[i]);
				if ((CommandData.roach[i].df_calc == 2)) {
					roach_check_retune(&roach_state_table[i]);
				}
				roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
			}
			/*
            if (CommandData.roach[i].find_kids) {
				roach_state_table[i].last_targ_path = make_dir(&roach_state_table[i], roach_state_table[i].targ_path_root);
				if ((get_targ_freqs(&roach_state_table[i], roach_state_table[i].last_vna_path,
										roach_state_table[i].last_targ_path)) > 0) {
				CommandData.roach[i].find_kids = 0;
				} else { blast_info("ROACH%d: Failed to find kids", i + 1); }
			}*/
			if (CommandData.roach[i].set_attens) {
				if (!set_atten(&rudat_state_table[i])) {
					blast_info("ROACH%d: Failed to set RUDATs...", i + 1);
				} else {
					CommandData.roach[i].set_attens = 0;
				}
			}
			if ((CommandData.roach[i].opt_tones) && (roach_state_table[i].status >= ROACH_STATUS_TARG)) {
				if (optimize_targ_tones(&roach_state_table[i], roach_state_table[i].last_targ_path)) {
					blast_info("ROACH%d: Opt tones success", i + 1);
				} else {
					blast_info("ROACH%d: Failed to optimize target tones", i + 1);
				}
				CommandData.roach[i].opt_tones = 0;
			}
			// The following is initialization
			if (bb_state_table[i].status == BB_STATUS_BOOT && bb_state_table[i].desired_status > BB_STATUS_BOOT) {
                		blast_info("Initializing BB%d ...", i + 1);
				bb_state_table[i].which = i + 1;
				bb_state_table[i].bb_comm = remote_serial_init(i, NC2_PORT);
				while (!bb_state_table[i].bb_comm->connected) {
					// blast_info("We can't connect to bb%d.", i+1);
					usleep(2000);
				}
				bb_state_table[i].status = BB_STATUS_INIT;
				blast_info("BB%d initialized...", i + 1);
			}
			if ((bb_state_table[i].status == BB_STATUS_INIT) && (rudat_state_table[i].status == RUDAT_STATUS_BOOT)) {
				blast_info("BB%d, attempting to set RUDATs...", i + 1);
				// TODO(Sam) Put in error handling
				if (!set_atten(&rudat_state_table[i])) {
					blast_info("ROACH%d: Failed to set RUDATs...", i + 1);
				} else {
					blast_info("RUDAT%d Initialized", i + 1);
					rudat_state_table[i].status = RUDAT_STATUS_HAS_ATTENS;
				}
			}
			if ((bb_state_table[i].status == BB_STATUS_INIT) && (valon_state_table[i].status == VALON_STATUS_BOOT)) {
                		if (init_valon(&roach_state_table[i]) < 1) {
					blast_info("ROACH%d: Failed to set Valon...", i + 1);
                	} else {
                        valon_state_table[i].status = VALON_STATUS_HAS_FREQS;
			   blast_info("Finished initializing Valon%d...", i + 1);
			    }
            		}
			if (roach_state_table[i].status == ROACH_STATUS_BOOT && roach_state_table[i].desired_status > ROACH_STATUS_BOOT) {
				blast_info("Attempting to connect to %s", roach_state_table[i].address);
				roach_state_table[i].katcp_fd = net_connect(roach_state_table[i].address,
					0, NETC_VERBOSE_ERRORS | NETC_VERBOSE_STATS);
				blast_info("fd:%d ", roach_state_table[i].katcp_fd);
				roach_state_table[i].rpc_conn = create_katcl(roach_state_table[i].katcp_fd);
				if (roach_state_table[i].katcp_fd > 0) {
					blast_info("ROACH%d, KATCP up", i + 1);
				/*	blast_info("ROACH%d, Checking firmware status...", i + 1);
					if (roach_upload_status(&roach_state_table[i])) {
						roach_state_table[i].status = ROACH_STATUS_CALIBRATED;
						roach_state_table[i].desired_status = ROACH_STATUS_TONE;
				} else { roach_state_table[i].status = ROACH_STATUS_CONNECTED;
					}
				}*/
			    roach_state_table[i].status = ROACH_STATUS_CONNECTED;
                }
			}
            if (roach_state_table[i].status == ROACH_STATUS_CONNECTED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_PROGRAMMED) {
				if (roach_upload_fpg(&roach_state_table[i], roach_fpg[i]) == 0) {
					blast_info("ROACH%d, Firmware uploaded", i + 1);
					roach_state_table[i].status = ROACH_STATUS_PROGRAMMED;
					roach_state_table[i].desired_status = ROACH_STATUS_CONFIGURED;
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_PROGRAMMED &&
				roach_state_table[i].desired_status >= ROACH_STATUS_CONFIGURED) {
				blast_info("ROACH%d, Configuring software registers...", i + 1);
				roach_write_int(&roach_state_table[i], "dds_shift", DDS_SHIFT, 0);/* DDS LUT shift, in clock cycles */
				// roach_read_int(&roach_state_table[i], "dds_shift");
				roach_write_int(&roach_state_table[i], "PFB_fft_shift", VNA_FFT_SHIFT, 0);/* FFT shift schedule */
				// roach_read_int(&roach_state_table[i], "fft_shift");
				roach_write_int(&roach_state_table[i], "downsamp_sync_accum_len", accum_len, 0);/* Number of accumulations */
				// roach_read_int(&roach_state_table[i], "sync_accum_len");
				roach_write_int(&roach_state_table[i], "GbE_tx_destip", dest_ip, 0);/* UDP destination IP */
				roach_write_int(&roach_state_table[i], "GbE_tx_destport", roach_state_table[i].dest_port, 0); /* UDP port */
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
					roach_write_int(&roach_state_table[i], "GbE_tx_rst", 0, 0);
					roach_write_int(&roach_state_table[i], "GbE_tx_rst", 1, 0);
					roach_write_int(&roach_state_table[i], "GbE_tx_rst", 0, 0);
					roach_write_int(&roach_state_table[i], "GbE_pps_start", 1, 0);
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
				roach_write_tones(&roach_state_table[i], roach_state_table[i].vna_comb, roach_state_table[i].vna_comb_len);
				// roach_write_tones(&roach_state_table[i], test_freq, 1);
				blast_info("ROACH%d, Search comb uploaded", i + 1);
				roach_state_table[i].status = ROACH_STATUS_TONE;
				roach_state_table[i].desired_status = ROACH_STATUS_STREAMING;
			}
			if (roach_state_table[i].status == ROACH_STATUS_TONE &&
				roach_state_table[i].desired_status >= ROACH_STATUS_STREAMING) {
				blast_info("ROACH%d, Checking stream status...", i + 1);
				if (roach_check_streaming(&roach_state_table[i]) == 0) {
					blast_info("ROACH%d, streaming SUCCESS", i + 1);
					roach_state_table[i].status = ROACH_STATUS_STREAMING;
					roach_state_table[i].desired_status = ROACH_STATUS_VNA;
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_STREAMING &&
				roach_state_table[i].desired_status >= ROACH_STATUS_VNA) {
				roach_write_int(&roach_state_table[i], "PFB_fft_shift", VNA_FFT_SHIFT, 0);/* FFT shift schedule */
				usleep(3000);
				blast_info("ROACH%d, Initializing VNA sweep", i + 1);
				blast_info("ROACH%d, Starting VNA sweep...", i + 1);
				status = roach_do_sweep(&roach_state_table[i], VNA);
				if ((status == SWEEP_SUCCESS)) {
					blast_info("ROACH%d, VNA sweep complete", i + 1);
					roach_state_table[i].status = ROACH_STATUS_VNA;
					roach_state_table[i].desired_status = ROACH_STATUS_ARRAY_FREQS;
				} else if ((status == SWEEP_INTERRUPT)) {
					blast_info("ROACH%d, VNA sweep interrupted by blastcmd", i + 1);
					roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
					roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
				} else { blast_info("ROACH%d, VNA sweep failed, will reattempt", i + 1);
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_VNA &&
				roach_state_table[i].desired_status >= ROACH_STATUS_ARRAY_FREQS) {
				get_targ_freqs(&roach_state_table[i], roach_state_table[i].last_vna_path,
										roach_state_table[i].last_targ_path);
				roach_state_table[i].status = ROACH_STATUS_ARRAY_FREQS;
				roach_state_table[i].desired_status = ROACH_STATUS_TARG;
			}
			if (roach_state_table[i].status == ROACH_STATUS_ARRAY_FREQS &&
				roach_state_table[i].desired_status >= ROACH_STATUS_TARG) {
				if (!roach_state_table[i].targ_tones) {
					blast_info("ROACH%d, Targ comb not found, ending sweep", i + 1);
					roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
					roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
					break;
				}
				roach_write_int(&roach_state_table[i], "PFB_fft_shift", TARG_FFT_SHIFT, 0);/* FFT shift schedule */
				roach_read_int(&roach_state_table[i], "PFB_fft_shift");
				usleep(3000);
				blast_info("ROACH%d, STARTING TARG sweep", i + 1);
				status = roach_do_sweep(&roach_state_table[i], TARG);
				if ((status == SWEEP_SUCCESS)) {
					blast_info("ROACH%d, TARG sweep complete", i + 1);
					roach_state_table[i].status = ROACH_STATUS_TARG;
					roach_state_table[i].desired_status = ROACH_STATUS_GRAD;
				} else if ((status == SWEEP_INTERRUPT)) {
					blast_info("ROACH%d, TARG sweep interrupted by blastcmd", i + 1);
					roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
					roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
				} else { blast_info("ROACH%d, TARG sweep failed, will reattempt", i + 1);
				}
			}
			if (roach_state_table[i].status == ROACH_STATUS_TARG &&
				roach_state_table[i].desired_status >= ROACH_STATUS_GRAD) {
				blast_info("ROACH%d, Starting GRADIENT sweep...", i + 1);
				cal_sweep(&roach_state_table[i]);
				blast_info("ROACH%d, GRADIENT sweep complete", i + 1);
				grad_calc(&roach_state_table[i]);
				roach_state_table[i].status = ROACH_STATUS_GRAD;
				roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
			}
	}
	return NULL;
}

int init_roach(uint16_t ind)
{
	if (ind >= NUM_ROACHES) {
	    blast_err("Attempted to intialize a non-existent roach #%u", ind + 1);
	    return -1;
	}
    memset(&roach_state_table[ind], 0, sizeof(roach_state_t));
    memset(&bb_state_table[ind], 0, sizeof(bb_state_t));
    memset(&rudat_state_table[ind], 0, sizeof(rudat_state_t));
    memset(&valon_state_table[ind], 0, sizeof(valon_state_t));
    asprintf(&roach_state_table[ind].address, "roach%d", ind + 1);
    asprintf(&roach_state_table[ind].vna_path_root, "/home/fc1user/sam_tests/sweeps/roach%d/vna", ind + 1);
    asprintf(&roach_state_table[ind].targ_path_root, "/home/fc1user/sam_tests/sweeps/roach%d/targ", ind + 1);
    asprintf(&roach_state_table[ind].cal_path_root, "/home/fc1user/sam_tests/sweeps/roach%d/cal", ind + 1);
    asprintf(&roach_state_table[ind].amps_path[0], "/home/fc1user/sam_tests/roach%d_default_amps.dat", ind + 1);
    asprintf(&roach_state_table[ind].qdr_log, "/home/fc1user/sam_tests/roach%d_qdr_cal.log", ind + 1);
    asprintf(&roach_state_table[ind].find_kids_log, "/home/fc1user/sam_tests/roach%d_find_kids.log", ind + 1);
    asprintf(&roach_state_table[ind].opt_tones_log, "/home/fc1user/sam_tests/roach%d_opt_tones.log", ind + 1);
	 if ((ind == 0)) {
	 	roach_state_table[ind].lo_centerfreq = 828.0e6;
	 	roach_state_table[ind].vna_comb_len = 1000;
	 	roach_state_table[ind].p_max_freq = 246.001234e6;
	 	roach_state_table[ind].p_min_freq = 1.02342e6;
	 	roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
	 	roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
	 }
	 if ((ind == 1)) {
	 	roach_state_table[ind].lo_centerfreq = 828.0e6;
	 	roach_state_table[ind].vna_comb_len = 1000;
	 	roach_state_table[ind].p_max_freq = 246.001234e6;
	 	roach_state_table[ind].p_min_freq = 1.02342e6;
	 	roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
	 	roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
	 }
	 if ((ind == 2)) {
	 	roach_state_table[ind].lo_centerfreq = 828.0e6;
	 	roach_state_table[ind].vna_comb_len = 1000;
	 	roach_state_table[ind].p_max_freq = 246.001234e6;
	 	roach_state_table[ind].p_min_freq = 1.02342e6;
	 	roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
	 	roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
	 }
	 if ((ind == 3)) {
	 	roach_state_table[ind].lo_centerfreq = 750.0e6;
	 	roach_state_table[ind].vna_comb_len = 1000;
	 	roach_state_table[ind].p_max_freq = 246.001234e6;
	 	roach_state_table[ind].p_min_freq = 1.02342e6;
	 	roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
	    roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
	 }
	 if ((ind == 4)) {
	 	roach_state_table[ind].lo_centerfreq = 828.0e6;
	 	roach_state_table[ind].vna_comb_len = 1000;
	 	roach_state_table[ind].p_max_freq = 246.001234e6;
	 	roach_state_table[ind].p_min_freq = 1.02342e6;
	 	roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
	    roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
	 }
	 roach_state_table[ind].which = ind + 1;
	 bb_state_table[ind].which = ind + 1;
	 rudat_state_table[ind].which = ind + 1;
	 valon_state_table[ind].which = ind + 1;
         roach_state_table[ind].dest_port = 64000 + ind;
	 roach_state_table[ind].is_streaming = 0;
	 roach_udp_networking_init(roach_state_table[ind].which, &roach_state_table[ind], NUM_ROACH_UDP_CHANNELS);
    ph_thread_spawn((ph_thread_func)roach_cmd_loop, (void*) &ind);
    blast_info("Spawned command thread for roach%i", ind + 1);
    return 0;
}

void write_roach_channels_5hz(void)
{
    int i, j;
    static int firsttime = 1;
    int n_write_kids_df = 20; // For now only write the delta_freqs for the first 20 KIDs.
    static channel_t *RoachPktCtAddr[NUM_ROACHES];
    static channel_t *RoachValidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachInvalidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachStatusAddr[NUM_ROACHES];
    static channel_t *BBStatusAddr[NUM_ROACHES];
    static channel_t *RudatStatusAddr[NUM_ROACHES];
    static channel_t *ValonStatusAddr[NUM_ROACHES];
    static channel_t *RoachStateAddr[NUM_ROACHES];
    static channel_t *RoachReqLOFreqAddr[NUM_ROACHES];
    static channel_t *RoachReadLOFreqAddr[NUM_ROACHES];
    static channel_t *RoachDfAddr[NUM_ROACHES][MAX_CHANNELS_PER_ROACH];
    static channel_t *CmdRoachParSmoothAddr[NUM_ROACHES];
    static channel_t *CmdRoachParPeakThreshAddr[NUM_ROACHES];
    static channel_t *CmdRoachParSpaceThreshAddr[NUM_ROACHES];
    static channel_t *CmdRoachParInAttenAddr[NUM_ROACHES];
    static channel_t *CmdRoachParOutAttenAddr[NUM_ROACHES];
    char channel_name_pkt_ct[128] = { 0 };
    char channel_name_valid_pkt_ct[128] = { 0 };
    char channel_name_invalid_pkt_ct[128] = { 0 };
    char channel_name_roach_status[128] = { 0 };
    char channel_name_valon_status[128] = { 0 };
    char channel_name_bb_status[128] = { 0 };
    char channel_name_rudat_status[128] = { 0 };
    char channel_name_roach_state[128] = { 0 };
    char channel_name_roach_req_lo[128] = { 0 };
    char channel_name_roach_df[128] = { 0 };
    char channel_name_roach_read_lo[128] = { 0 };
    char channel_name_cmd_roach_par_smooth[128] = { 0 };
    char channel_name_cmd_roach_par_peak_thresh[128] = { 0 };
    char channel_name_cmd_roach_par_space_thresh[128] = { 0 };
    char channel_name_cmd_roach_par_in_atten[128] = { 0 };
    char channel_name_cmd_roach_par_out_atten[128] = { 0 };
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
            snprintf(channel_name_valon_status,
                    sizeof(channel_name_valon_status), "status_valon_roach%d", i + 1);
            snprintf(channel_name_bb_status,
                    sizeof(channel_name_bb_status), "status_bb_roach%d", i + 1);
            snprintf(channel_name_rudat_status,
                    sizeof(channel_name_rudat_status), "status_rudat_roach%d", i + 1);
            snprintf(channel_name_roach_state, sizeof(channel_name_roach_state),
                    "stream_state_roach%d", i + 1);
            snprintf(channel_name_roach_req_lo,
                    sizeof(channel_name_roach_req_lo), "freq_lo_req_roach%d",
                    i + 1);
            snprintf(channel_name_roach_read_lo,
                    sizeof(channel_name_roach_read_lo), "freq_lo_read_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_smooth,
                    sizeof(channel_name_cmd_roach_par_smooth), "fk_smooth_scale_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_peak_thresh,
                    sizeof(channel_name_cmd_roach_par_peak_thresh), "fk_peak_thresh_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_space_thresh,
                    sizeof(channel_name_cmd_roach_par_space_thresh), "fk_space_thresh_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_in_atten,
                    sizeof(channel_name_cmd_roach_par_in_atten), "atten_in_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_out_atten,
                    sizeof(channel_name_cmd_roach_par_out_atten), "atten_out_roach%d",
                    i + 1);
            RoachPktCtAddr[i] = channels_find_by_name(channel_name_pkt_ct);
            RoachValidPktCtAddr[i] = channels_find_by_name(
                    channel_name_valid_pkt_ct);
            RoachInvalidPktCtAddr[i] = channels_find_by_name(
                    channel_name_invalid_pkt_ct);
            RoachStatusAddr[i] = channels_find_by_name(channel_name_roach_status);
            ValonStatusAddr[i] = channels_find_by_name(channel_name_valon_status);
            BBStatusAddr[i] = channels_find_by_name(channel_name_bb_status);
            RudatStatusAddr[i] = channels_find_by_name(channel_name_rudat_status);
            RoachStateAddr[i] = channels_find_by_name(channel_name_roach_state);
            RoachReqLOFreqAddr[i] = channels_find_by_name(channel_name_roach_req_lo);
            RoachReadLOFreqAddr[i] = channels_find_by_name(channel_name_roach_read_lo);
            CmdRoachParSmoothAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_smooth);
            CmdRoachParPeakThreshAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_peak_thresh);
            CmdRoachParSpaceThreshAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_space_thresh);
            CmdRoachParInAttenAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_in_atten);
            CmdRoachParOutAttenAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_out_atten);
            for (j = 0; j < MAX_CHANNELS_PER_ROACH; j++) {
                if (j < n_write_kids_df) {
                    snprintf(channel_name_roach_df,
                             sizeof(channel_name_roach_df), "df_kid%04d_roach%d",
                             j , i + 1);
                    RoachDfAddr[i][j] = channels_find_by_name(channel_name_roach_df);
                }
            }
        }
    }
    for (i = 0; i < NUM_ROACHES; i++) {
        SET_UINT32(RoachPktCtAddr[i], roach_udp[i].roach_packet_count);
        SET_UINT32(RoachValidPktCtAddr[i],
        roach_udp[i].roach_valid_packet_count);
        SET_UINT32(RoachInvalidPktCtAddr[i],
        roach_udp[i].roach_invalid_packet_count);
        SET_UINT16(RoachStatusAddr[i], roach_state_table[i].status);
        SET_UINT16(ValonStatusAddr[i], valon_state_table[i].status);
        SET_UINT16(RudatStatusAddr[i], rudat_state_table[i].status);
        SET_UINT16(BBStatusAddr[i], bb_state_table[i].status);
        // TODO(laura/sam): Replace next write with a streaming status bitfield.
        SET_UINT16(RoachStateAddr[i], roach_state_table[i].is_streaming);
        SET_SCALED_VALUE(RoachReqLOFreqAddr[i], roach_state_table[i].lo_freq_req);
        SET_SCALED_VALUE(RoachReadLOFreqAddr[i], roach_state_table[i].lo_centerfreq);
        SET_SCALED_VALUE(CmdRoachParSmoothAddr[i], CommandData.roach_params[i].smoothing_scale);
        SET_SCALED_VALUE(CmdRoachParPeakThreshAddr[i], CommandData.roach_params[i].peak_threshold);
        SET_SCALED_VALUE(CmdRoachParSpaceThreshAddr[i], CommandData.roach_params[i].spacing_threshold);
        SET_SCALED_VALUE(CmdRoachParInAttenAddr[i], CommandData.roach_params[i].in_atten);
        SET_SCALED_VALUE(CmdRoachParOutAttenAddr[i], CommandData.roach_params[i].out_atten);
        for (j = 0; j < MAX_CHANNELS_PER_ROACH; j++) {
            if (j < n_write_kids_df) {
                SET_SCALED_VALUE(RoachDfAddr[i][j], roach_state_table[i].df_diff[j]);
            }
        }
//        if (i == 0) {
//            blast_info("roach%i, lo_freq_req = %f, lo_centerfreq = %f",
//                       i+1, roach_state_table[i].lo_freq_req, roach_state_table[i].lo_centerfreq);
//            blast_info("roach%i, smoothing_scale = %f, spacing_threshold = %f",
//                       i+1, CommandData.roach_params[i].smoothing_scale,
//                       CommandData.roach_params[i].spacing_threshold);
//            blast_info("roach%i, df[0] = %f",
//                       i+1, roach_state_table[i].df_diff[0]);
//        }
    }
}
