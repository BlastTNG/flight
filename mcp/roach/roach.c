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
 *  Last edited: March, 2018
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
#include "cryostat.h"
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

// TODO(Sam + Adrian) Add writing for both LUTs, IQ gradient calc, check lut_idx in packet

/* MCP = Master Control Program (Blast) */
/* PPC = PowerPC */
/* FW = Firmware */
/* FC1 = Flight computer 1 */
/* FC2 = Flight computer 2 */
/* Pi = Raspberry Pi */
/* Baseband = -256 to +256 MHz */
/* I = In phase. The cos (real) part of the waveform */
/* Q = Quadrature. The sine (imaginary) part of the waveform */

#define DDC_SHIFT 317 /* FW version dependent, see roach_fpg */
#define VNA_FFT_SHIFT 31 /*Controls FW FFT overflow behavior, for VNA SWEEP */
#define TARG_FFT_SHIFT 127
#define VNA 0 /* Sweep type */
#define TARG 1 /* Sweep type */
#define CAL_AMPS 2 /* Sweep type */
#define CHOP 3
#define WRITE_INT_TIMEOUT 1000 /* KATCP write timeout */
#define UPLOAD_TIMEOUT 20000 /* KATCP upload timeout */
#define QDR_TIMEOUT 20000 /* Same as above */
#define LUT_BUFFER_LEN 2097152 /* Length of time domain LUTs =  2^21 */
#define DAC_SAMP_FREQ 512.0e6 /* MUSIC board clock rate, from Valon Synth */
#define DAC_FULL_SCALE (1 << 15) - 1 /* Full scale value of the 16b DAC */
#define FPGA_SAMP_FREQ (DAC_SAMP_FREQ/2) /* FPGA clock rate */
/* Freq resolution of DAC tones = 488.28125 Hz */
#define DAC_FREQ_RES (2*DAC_SAMP_FREQ / LUT_BUFFER_LEN)
#define LO_STEP 1000 /* Freq step size for sweeps = 1 kHz */
#define VNA_SWEEP_SPAN 10.0e3 /* VNA sweep span, for testing = 10 kHz */
#define TARG_SWEEP_SPAN 120.0e3 /* Target sweep span = 200 kHz */
#define NCOEFF 12 /* 1 + Number of FW FIR coefficients */
#define N_AVG 50 /* Number of packets to average for each sweep point */
#define NC1_PORT 12345 /* Netcat port on Pi for FC1 */
#define NC2_PORT 12346 /* Netcat port on Pi for FC2 */
#define SWEEP_INTERRUPT (-1)
#define SWEEP_SUCCESS (1)
#define SWEEP_FAIL (0)
#define SWEEP_TIMEOUT 500 /* microsecond timeout between set LO and save data */
#define READ_LINE 256 /* Line length for buffer reads, bytes */
#define READ_BUFFER 4096 /* Number of bytes to read from a buffer */
#define STREAM_NTRIES 10 /* Number of times to check stream status */
#define STREAM_TIMEOUT 2 /* Seconds to wait between checks */
#define PI_READ_NTRIES 50 /* Number of times to attempt Pi read */
#define PI_READ_TIMEOUT (800*1000) /* Pi read timeout, usec */
#define LO_READ_TIMEOUT (600*1000) /* LO read timeout, usec */
#define INIT_VALON_TIMEOUT (500*1000) /* Valon init timeout, usec */
#define FLAG_THRESH 300 /* Threshold for use in function roach_check_retune */
#define LUT_FREQ_OFFSET 3000 /* Hz, Frequency offset (+/-) of LUT0 and LUT1 from f0 */
#define DF_THRESHOLD 100000 /* Hz, Delta F threshold for retuning */
#define DEFAULT_SWITCH_PERIOD 3 /* Seconds, default switching period*/
#define ADC_CAL_NTRIES 100
#define ADC_TARG_RMS_VNA 100 /* mV, Same for all Roaches */
#define ADC_TARG_RMS_250 100 /* mV, For 250 micron array TARG sweep */
#define ADC_RMS_RANGE 5 /* mV */
#define ATTEN_STEP 0.5 /* dB */
#define OUTPUT_ATTEN_VNA 1 /* dB, for VNA sweep */
#define OUTPUT_ATTEN_TARG 20 /* dB, initial level for TARG sweep */
#define READ_DATA_MS_TIMEOUT 10000 /* ms, timeout for roach_read_data */
#define EXT_REF 0 /* Valon external ref (10 MHz) */
#define NCAL_POINTS 21 /* Number of points to use for cal sweep */
#define N_CAL_CYCLES 3 /* Number of cal cycles for tone amplitudes */
#define DELTA_AMP 0.02 /* A change in tone amplitude (used for optical calibration */

extern int16_t InCharge; /* See mcp.c */
extern int roach_sock_fd; /* File descriptor for shared Roach UDP socket */
static int fft_len = 1024; /* Order of FW FFT */
static uint32_t accum_len = (1 << 19) - 1; /* Number of FW FFT accumulations */
/* FW FIR filter: Order 22 (length 23) Hann window coefficients, f_cutoff = 10 kHz */
double fir_coeffs[12] = {0.0, 0.00145215, 0.0060159, 0.01373059, 0.02425512,
                   0.03688533, 0.05061838, 0.06425732, 0.07654357, 0.08630093,
                   0.09257286, 0.09473569};

// Roach source MAC addresses
const char src_macs[5][100] = {"024402020b03", "024402020d17", "024402020D16", "02440202110c", "024402020D21"};
uint32_t srcmac0[5] = {33688323, 33688855, 33688854, 33689868, 33688865};
uint32_t srcmac1 = 580;
double test_freq[] = {10.0125e6};

// UDP destination MAC addresses

// FC1
// uint32_t destmac0 = 2877007929;
// uint32_t destmac1 = 11;

// FC2
// uint32_t destmac0 = 2876946759;
// uint32_t destmac1 = 11;

// MULTICAST
uint32_t destmac0 = 1577124330;
uint32_t destmac1 = 256;

static uint32_t dest_ip = IPv4(239, 1, 1, 234);

const char roach_fpg[] = "/data/etc/blast/roachFirmware/stable_qdr_switch_5_2018_Feb_12_1249.fpg";

/* Roach2 state structure, see roach.h */
static roach_state_t roach_state_table[NUM_ROACHES]; /* NUM_ROACHES = 5 */
/* Beaglebone/Pi state structure, see roach.h */
static pi_state_t pi_state_table[NUM_ROACHES];
/* Rudat Attenuator state structure, see roach.h */
static rudat_state_t rudat_state_table[NUM_ROACHES];
/* Valon Synthesizer state structure, see roach.h */
static valon_state_t valon_state_table[NUM_ROACHES];
/* Initialization scripts that live on Pi */
char valon_init_pi[] = "python /home/pi/device_control/init_valon.py";
char valon_init_pi_extref[] = "python /home/pi/device_control/init_valon_ext.py";
char read_valon_pi[] = "python /home/pi/device_control/read_valon.py";
/* For testing, use detector data from Feb, 2018 cooldown */
// char vna_search_path[] = "/home/fc1user/sam_tests/sweeps/roach2/vna/Fri_Mar__2_15_42_18_2018";
// char vna_search_path[] = "/home/fc1user/sam_tests/sweeps/roach1/vna/Tue_Mar_13_17_12_35_2018";
// char vna_search_path[] = "/home/fc1user/sam_tests/sweeps/roach1/vna/Wed_Mar_14_19_51_43_2018";
// char vna_search_path[] = "/home/fc1user/sam_tests/sweeps/roach1/vna/Thu_Mar_15_13_21_45_2018";
char vna_search_path[] = "/home/fc1user/sam_tests/sweeps/roach2/vna/Wed_Mar_21_16_03_45_2018";

// char targ_search_path[] = "/home/fc1user/sam_tests/sweeps/roach2/targ/Tue_Feb_27_14_30_05_2018";
char targ_search_path[] = "/home/fc1user/sam_tests/sweeps/roach1/targ/Tue_Mar_13_21_40_45_2018";

char bb_targ_freqs_path[] = "/home/fc1user/sam_tests/sweeps";
char find_kids_250[] = "/data/etc/blast/roachPython/find_kids_250.py";
char find_kids_350[] = "/data/etc/blast/roachPython/find_kids_350.py";
char center_phase_script[] = "/data/etc/blast/roachPython/center_phase.py";
char chop_snr_script[] = "/data/etc/blast/roachPython/fit_mcp_chop.py";
char refit_freqs_script[] = "/data/etc/blast/roachPython/cal_sweep_fit_res.py";

static pthread_mutex_t fft_mutex; /* Controls access to the fftw3 */

void nameThread(const char*);

int get_roach_status(uint16_t ind)
{
    int status = roach_state_table[ind].status;
    return status;
}

/* Function: enable_qdr_switch()
 * ----------------------------
 * Enables switching between tone LUTs
 * @param m_roach roach_state_structure
 * @param enable bool, on or off (0 or 1)
*/

void enable_qdr_switch(roach_state_t *m_roach, int enable)
{
    roach_write_int(m_roach, "switch_enable", enable, 0);
}

void roach_switch_LUT(uint16_t ind)
{
    // This line switches to next LUT pair (If 0, goes to 1, visa versa)
    roach_state_t m_roach = roach_state_table[ind];
    m_roach.lut_idx = m_roach.lut_idx ^ 1;
    if (m_roach.write_flag) {
        return;
    } else {
        if (m_roach.status >= ROACH_STATUS_POP_LUTS) {
            roach_write_int(&m_roach, "qdr_switch", m_roach.lut_idx, 0);
            usleep(1000);
            int result = roach_read_int(&m_roach, "qdr_switch");
            blast_info("lut idx = %d", result);
        }
    }
}

/* Function: load_fir
 * ----------------------------
 * Programs the coefficients for the FW FIR filter
 * Operates on software registers
 * @param m_roach a roach state structure
 * @param m_coeff FIR filter coefficients
*/
void load_fir(roach_state_t *m_roach, double *m_coeff)
{
    char *reg;
    for (int i = 0; i < NCOEFF; ++i) {
        m_coeff[i] *= ((pow(2, 31) - 1));
        blast_tmp_sprintf(reg, "FIR_h%d", i);
        roach_write_int(m_roach, reg, (int32_t)m_coeff[i], 0);
    }
    blast_info("ROACH%d, Uploaded FIR coefficients", m_roach->which);
}

/* Function: roach_qdr_cal
 * -----------------------
 * Runs QDR ram calibration routine for each Roach, via Python script:
 * cal_roach_qdr.py. Checks that FPGA clock ~ 256 MHz. If not, throws warning
 *
 * @param: m_roach roach state structure
*/
int roach_qdr_cal(roach_state_t *m_roach)
{
    char *m_cal_command;
    char m_line[READ_LINE];
    char *freq;
    blast_tmp_sprintf(m_cal_command, "python /data/etc/blast/roachPython/cal_roach_qdr.py %s > %s",
            m_roach->address, m_roach->qdr_log);
    system(m_cal_command);
    sleep(5);
    FILE *fd = fopen(m_roach->qdr_log, "r");
    if (!fd) {
        blast_err("Error opening QDR log file");
        return -1;
    }
    if (!fgets(m_line, sizeof(m_line), fd)) {
        blast_err("Error reading QDR log file");
        return -1;
    } else {
        blast_info("%s", m_line);
        freq = strchr(m_line, '=');
        freq += 2; // Remove equals sign and space from string
    }
    fclose(fd);
    double clock_freq = atof(freq);
    if ((clock_freq <= 255.0) || (clock_freq >= 258.0)) {
        blast_info("ROACH%d FPGA CLOCK NOT SET: Reinitialize Valon%d ?",
                   m_roach->which, m_roach->which);
        return -1;
    }
    return 0;
}

/* Function: roach_buffer_ntohs
 * ----------------------------
 * Converts a buffer of type short from big-endian to little-endian
 *
 * @param m_buffer A buffer of type short
 * @param m_len length of m_buffer
*/
static void roach_buffer_ntohs(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohs(m_buffer[i]);
    }
}

static void roach_buffer_htons(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = htons(m_buffer[i]);
    }
}

/* Function: roach_get_name
 * ----------------------------
 * Returns IP address of Roach PPC
 * Used in qdr.c (untested alternative to roach_qdr_cal
 *
 * @param m_roach roach state structure
*/
const char *roach_get_name(roach_state_t *m_roach)
{
    if (!m_roach || !m_roach->address) return "INVALID ROACH";
    return m_roach->address;
}

/* Function: roach_read_data
 * ----------------------------
 * Reads a software register with KATCP protocol
 *
 * @param m_roach roach state structure
 * @param m_dest buffer for storing KATCP response
 * @param m_register Name of software register
 * @param m_offset read offset in bytes
 * @param m_size Size of m_dest
 * @param ms_timeout millisecond timeout
*/
int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest,
    const char *m_register, uint32_t m_offset, uint32_t m_size, int ms_timeout)
{
    int retval = send_rpc_katcl(m_roach->rpc_conn, ms_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?read",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_size,
                   NULL);
    if (retval < 0) {
        blast_err("Could not read data '%s' from %s: Internal Error",
                                              m_register, m_roach->address);
        return -1;
    }
    if (retval > 0) {
        char *ret = arg_string_katcl(m_roach->rpc_conn, 0);
        blast_err("Could not read data '%s' from %s: ROACH Error '%s'",
                                              m_register, m_roach->address,
            ret?ret:"");
        return -1;
    }
    if (arg_count_katcl(m_roach->rpc_conn) < 3) {
        blast_err("Expecting 2 return values.  Received %d",
                                        arg_count_katcl(m_roach->rpc_conn));
        return -1;
    }
    uint32_t bytes_copied = arg_buffer_katcl(m_roach->rpc_conn, 2,
                                                           m_dest, m_size);
    if (bytes_copied != m_size) {
        blast_err("Expecting %ul bytes but only received %ul bytes",
                                                     m_size, bytes_copied);
        return -1;
    }
    return 0;
}

/* Function: roach_read_int
 * ----------------------------
 * Reads an integer from software register with KATCP protocol
 *
 * @param m_roach roach state structure
 * @param m_register Name of software register
*/
int roach_read_int(roach_state_t *m_roach, const char *m_register)
{
    uint32_t m_data;
    roach_read_data(m_roach, (uint8_t*) &m_data, m_register, 0,
                                          sizeof(m_data), READ_DATA_MS_TIMEOUT);
    m_data = ntohl(m_data);
    blast_info("%s = %d", m_register, m_data);
    return 0;
}

/* Function: roach_write_data
 * ----------------------------
 * Writes data to Roach software register with KATCP protocol
 *
 * @param m_roach roach state structure
 * @param m_register Name of software register
 * @param m_data data buffer, contains data to write
 * @param m_len length of m_data
 * @param m_offset write offset in bytes
 * @param m_timout timeout
*/
int roach_write_data(roach_state_t *m_roach, const char *m_register,
    uint8_t *m_data, size_t m_len, uint32_t m_offset, int m_timeout)
{
    return send_rpc_katcl(m_roach->rpc_conn, m_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?write",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_BUFFER, m_data, m_len,
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_len, NULL);
}

/* Function: roach_write_int
 * ----------------------------
 * Writes an integer to Roach software register with KATCP protocol
 *
 * @param m_roach roach state structure
 * @param m_register Name of software register
 * @param m_val integer to write
 * @param m_offset write offset in bytes
*/
int roach_write_int(roach_state_t *m_roach, const char *m_register,
                                         uint32_t m_val, uint32_t m_offset)
{
    uint32_t sendval = htonl(m_val);
    return roach_write_data(m_roach, m_register, (uint8_t*)&sendval,
                             sizeof(sendval), m_offset, WRITE_INT_TIMEOUT);
}

int roach_read_1D_file(roach_state_t *m_roach, char *m_file_path, double *m_buffer, size_t m_freqlen)
{
    int retval = -1;
    blast_info("ROACH%d, file = %s", m_roach->which, m_file_path);
    FILE *fd = fopen(m_file_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_file_path);
    } else {
        blast_info("Opened %s", m_file_path);
        for (size_t i = 0; i < m_freqlen; i++) {
            if (fscanf(fd, "%lg\n", &m_buffer[i]) != EOF) {
                // blast_info("Roach%d loaded vals: %g", m_roach->which, m_buffer[i]);
            } else {
                break;
            }
        }
        fclose(fd);
        retval = 0;
    }
    return retval;
}

/* Function: roach_init_LUT
 * ----------------------------
 * Allocates memory for Roach freq comb LUT
 *
 * @param m_roach roach state structure
 * @param m_len length of LUT
*/
static void roach_init_LUT(roach_state_t *m_roach, size_t m_len)
{
    if (m_roach->lut_idx == 0) {
        m_roach->LUT0.len = m_len;
        m_roach->LUT0.Ival = calloc(m_len, sizeof(uint16_t));
        m_roach->LUT0.Qval = calloc(m_len, sizeof(uint16_t));
    } else {
        m_roach->LUT1.len = m_len;
        m_roach->LUT1.Ival = calloc(m_len, sizeof(uint16_t));
        m_roach->LUT1.Qval = calloc(m_len, sizeof(uint16_t));
    }
}

/* Function: roach_init_DACDDC_LUTs
 * ----------------------------
 * Allocates memory for Roach DAC and DDC freq comb LUTs
 *
 * @param m_roach roach state structure
 * @param m_len length of LUT
*/
static void roach_init_DACDDC_LUTs(roach_state_t *m_roach, size_t m_len)
{
    if (m_roach->lut_idx == 0) {
       m_roach->dac0.len = m_len;
       m_roach->dac0.Ival = calloc(m_len, sizeof(double));
       m_roach->dac0.Qval = calloc(m_len, sizeof(double));
       m_roach->ddc0.len = m_len;
       m_roach->ddc0.Ival = calloc(m_len, sizeof(double));
       m_roach->ddc0.Qval = calloc(m_len, sizeof(double));
    } else {
       m_roach->dac1.len = m_len;
       m_roach->dac1.Ival = calloc(m_len, sizeof(double));
       m_roach->dac1.Qval = calloc(m_len, sizeof(double));
       m_roach->ddc1.len = m_len;
       m_roach->ddc1.Ival = calloc(m_len, sizeof(double));
       m_roach->ddc1.Qval = calloc(m_len, sizeof(double));
    }
}

/* Function: roach_fft_bin_idx
 * ----------------------------
 * Calculates an FFT bin index, for an FFT of order m_fft_len
 *
 * @param m_freq a baseband tone frequency
 * @param m_freq_idx the index of m_freq in the list of basebands freqs
 * @param m_fft_len the order of the FFT
 * @param m_samp_freq the sampling frequency
 *
 * returns: the FFT bin index
*/
static inline int roach_fft_bin_idx(double *m_freq, size_t m_freq_idx, size_t m_fft_len, double m_samp_freq)
{
    return (int)lround(m_freq[m_freq_idx] / m_samp_freq *  m_fft_len);
}

/* Function: roach_dac_comb
 * ----------------------------
 * Calculates the DAC freq comb
 *
 * @param m_roach a roach state structure
 * @param m_freqs
 * @param m_fft_len the order of the FFT
 * @param m_samp_freq the sampling frequency
 *
*/
// TODO(Sam) Update to latest FFTW3 conventions (commented out section)
static int roach_dac_comb(roach_state_t *m_roach, double *m_freqs,
        size_t m_freqlen, int m_samp_freq, double *m_I, double *m_Q)
{
    size_t comb_fft_len;
    double amps[m_freqlen];
    double phases[m_freqlen];
    // m_roach->last_amps = calloc(m_freqlen, sizeof(double));
    for (size_t i = 0; i < m_freqlen; i++) {
        amps[i] = 1.0;
        // blast_info("amps = %g", m_roach->last_amps[i]);
    }
    if (!m_roach->last_amps) {
        m_roach->last_amps = calloc(m_freqlen, sizeof(double));
        for (size_t i = 0; i < m_freqlen; i++) {
            m_roach->last_amps[i] = 1.0;
        }
    }
    // Load random phases (static file for testing)
    blast_info("Roach%d, Loading tone phases", m_roach->which);
    char *phase_path = m_roach->random_phase_path;
    // If can't load file, use random phases
    if ((roach_read_1D_file(m_roach, phase_path, phases, m_freqlen)) < 0) {
        srand48(time(NULL));
        for (size_t i = 0; i < m_freqlen; i++) {
            phases[i] = drand48() * 2.0 * M_PI;
        }
    }

    if (CommandData.roach[m_roach->which - 1].load_vna_amps) {
        blast_info("Roach%d, Loading VNA AMPS", m_roach->which);
        char *amps_path = m_roach->vna_amps_path[CommandData.roach[m_roach->which - 1].load_vna_amps - 1];
        roach_read_1D_file(m_roach, amps_path, amps, m_freqlen);
    CommandData.roach[m_roach->which - 1].load_vna_amps = 0;
    }

    if (CommandData.roach[m_roach->which - 1].load_targ_amps) {
        blast_info("Roach%d, Loading TARG AMPS", m_roach->which);
        char *amps_path = m_roach->targ_amps_path[CommandData.roach[m_roach->which - 1].load_targ_amps - 1];
        roach_read_1D_file(m_roach, amps_path, amps, m_freqlen);
    CommandData.roach[m_roach->which - 1].load_targ_amps = 0;
    }

    if (CommandData.roach[m_roach->which - 1].change_tone_amps) {
        for (size_t i = 0; i < m_freqlen; i++) {
            amps[i] = m_roach->last_amps[i];
            // blast_info("amps = %g", m_roach->last_amps[i]);
        }
    }

    // save amps to m_roach->last_targ_amps
    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
    }
    comb_fft_len = LUT_BUFFER_LEN;
    complex double *spec = calloc(comb_fft_len, sizeof(complex double));
    complex double *wave = calloc(comb_fft_len, sizeof(complex double));
    // double phi;
    double max_val = 0.0;
    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        int bin = roach_fft_bin_idx(m_freqs, i, comb_fft_len, m_samp_freq);
        if (bin < 0) {
           bin += comb_fft_len;
        }
        // blast_info("%g, %d, %d", m_freqs[i], bin, LUT_BUFFER_LEN/2);
        if (bin >= LUT_BUFFER_LEN) {
            blast_info("Write Error: DAC freqs too high, resweep recommended");
            return -1;
        }
        /* phi = drand48() * 2.0 * M_PI;
        spec[bin] = amps[i]*cexp(_Complex_I * phi); */
        spec[bin] = amps[i]*cexp(_Complex_I * phases[i]);
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
        m_I[i] = creal(wave[i]) / max_val * (DAC_FULL_SCALE);
        m_Q[i] = cimag(wave[i]) / max_val * (DAC_FULL_SCALE);
    }
    free(spec);
    free(wave);
    return 0;

    /* comb_fft_len = LUT_BUFFER_LEN;
    fftw_complex *spec = (fftw_complex*) fftw_malloc(comb_fft_len * sizeof(fftw_complex));
    fftw_complex *wave = (fftw_complex*) fftw_malloc(comb_fft_len * sizeof(fftw_complex));
    double alpha;
    double max_val = 0.0;
    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        int bin = roach_fft_bin_idx(m_freq, i, comb_fft_len, m_samp_freq);
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
        m_I[i] = wave[i][0] / max_val * (DAC_FULL_SCALE);
        m_Q[i] = wave[i][1] / max_val * (DAC_FULL_SCALE);
    }
    fftw_free(spec);
    fftw_free(wave);
    return 0; */
}

/* Function: roach_ddc_comb
 * ----------------------------
 * Calculates the DDC freq comb
 *
 * @param m_roach a roach state structure
 * @param m_freqs frequency
 * @param m_freqlen number of frequencies in comb
 * @param m_samp_freq the sampling frequency
 * @param m_bin FFT bin index
 * @param m_I an I value
 * @param m_Q a Q value
 *
*/
static int roach_ddc_comb(roach_state_t *m_roach, double m_freq, size_t m_freqlen,
                            double m_phase, int m_samp_freq, int m_bin, double *m_I, double *m_Q)
{
    size_t comb_fft_len;
    fftw_plan comb_plan;
    comb_fft_len = LUT_BUFFER_LEN / fft_len;
    complex double *spec = calloc(comb_fft_len, sizeof(complex double));
    complex double *wave = calloc(comb_fft_len, sizeof(complex double));
    double max_val = 0.0;
    spec[m_bin] = cexp(_Complex_I * m_phase);
    // blast_info("DDC spec = %f, %f", creal(spec[m_bin]), cimag(spec[m_bin]));
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
        m_I[i] = creal(wave[i]) / max_val * (DAC_FULL_SCALE);
        m_Q[i] = cimag(wave[i]) / max_val * (DAC_FULL_SCALE);
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
    // blast_info("DDC spec = %f, %f", creal(spec[m_bin]), cimag(spec[m_bin]));
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
	m_I[i] = wave[i][0] / max_val * (DAC_FULL_SCALE);
	// blast_info("I = %g", m_I[i]);
        m_Q[i] = wave[i][1] / max_val * (DAC_FULL_SCALE);
	// blast_info("Q = %g", m_Q[i]);
    }
    fftw_free(spec);
    fftw_free(wave);
    return 0; */
}

/* Function: roach_vna_comb
 * ----------------------------
 * Calculates frequencies for the VNA comb; order of freqs is:
 * Small positive -> large positive -> large negative -> small negative
 *
 * @param m_roach a roach state structure
*/
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
    /* for (size_t i = 0; i < m_roach->vna_comb_len; i++) {
        blast_info("ROACH%d vna freq = %g", m_roach->which, m_roach->vna_comb[i]);
    }*/
    /* for (size_t i = 0; i < m_roach->vna_comb_len; i++) {
           m_roach->vna_comb[i] = round(m_roach->vna_comb[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
           blast_info("VNA freq = %g", m_roach->vna_comb[i]/DAC_FREQ_RES);
    }*/
}

int save_freqs(roach_state_t *m_roach, char *m_save_path, double *m_freqs, size_t m_freqlen)
{
    FILE *fd = fopen(m_save_path, "w");
    if (!fd) {
        blast_strerror("Could not open %s for writing", m_save_path);
        return -1;
    }
    for (size_t i = 0; i < m_freqlen; i++) {
        fprintf(fd, "%.10f\n", (float)m_freqs[i]);
    }
    fclose(fd);
    blast_info("ROACH%d Freqs written to %s", m_roach->which, m_save_path);
    return 0;
}

int roach_save_1D_file(roach_state_t *m_roach, char *m_save_path, double *m_vals, size_t m_len)
{
    FILE *fd = fopen(m_save_path, "w");
    if (!fd) {
        blast_strerror("Could not open %s for writing", m_save_path);
        return -1;
    }
    for (size_t i = 0; i < m_len; i++) {
        fprintf(fd, "%.10f\n", (float)m_vals[i]);
    }
    fclose(fd);
    blast_info("ROACH%d vals to %s", m_roach->which, m_save_path);
    return 0;
}

int save_sweep_freqs(roach_state_t *m_roach, char *m_save_path, double *m_sweep_freqs, size_t m_freqlen)
{
    double freqs[m_freqlen];
    FILE *m_sweep_fd = fopen(m_save_path, "w");
    if (!m_sweep_fd) {
        blast_strerror("Could not open %s for writing", m_save_path);
        return -1;
    } else {
        for (size_t i = 0; i < m_freqlen; i++) {
            freqs[i] = round(m_sweep_freqs[i] / LO_STEP) * LO_STEP;
            fprintf(m_sweep_fd, "%d\n", (uint32_t)freqs[i]);
        }
    }
    fclose(m_sweep_fd);
    return 0;
}

/* Function: roach_define_DAC_LUT
 * ----------------------------
 * Checks to see if freq comb LUTs have been allocated,
 * calls roach_dac_comb to populate DAC LUT
 *
 * @param m_roach a roach state structure
 * @param m_freqs list of comb frequencies
 * @param m_freqlen number of frequencies in comb
*/
static void roach_define_DAC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    if (m_roach->lut_idx == 0) {
        if (m_roach->dac0.len > 0) {
            m_roach->dac0.len = 0;
            free(m_roach->dac0.Ival);
            free(m_roach->dac0.Qval);
        }
        if (m_roach->dac0.len == 0) {
            m_roach->dac0.len = LUT_BUFFER_LEN;
            m_roach->dac0.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
            m_roach->dac0.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
        }
        roach_dac_comb(m_roach, m_freqs, m_freqlen,
                    DAC_SAMP_FREQ, m_roach->dac0.Ival, m_roach->dac0.Qval);
    } else {
        if (m_roach->dac1.len > 0) {
            m_roach->dac1.len = 0;
            free(m_roach->dac1.Ival);
            free(m_roach->dac1.Qval);
        }
        if (m_roach->dac1.len == 0) {
            m_roach->dac1.len = LUT_BUFFER_LEN;
            m_roach->dac1.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
            m_roach->dac1.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
        }
        roach_dac_comb(m_roach, m_freqs, m_freqlen,
                    DAC_SAMP_FREQ, m_roach->dac1.Ival, m_roach->dac1.Qval);
    }
}

/* Function: roach_select_bins
 * ----------------------------
 * Operates on FW: Populates ROM containing list of FW FFT bins which contain detector bias tones
 *
 * @param m_roach a roach state structure
 * @param m_freqs list of comb frequencies
 * @param m_freqlen number of frequencies in comb
*/
static void roach_select_bins(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int bins[fft_len];
    double bin_freqs[fft_len];
    m_roach->freq_residuals = malloc(m_freqlen * sizeof(size_t));
    for (size_t i = 0; i < m_freqlen; i++) {
        bins[i] = roach_fft_bin_idx(m_freqs, i, fft_len, DAC_SAMP_FREQ);
        bin_freqs[i] = bins[i] * DAC_SAMP_FREQ / fft_len;
        if (bins[i] < 0) {
            bins[i] += 1024;
        }
        m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / (DAC_FREQ_RES)) * (DAC_FREQ_RES);
    }
    if (m_roach->lut_idx == 0) {
        for (int ch = 0; ch < m_freqlen; ch++) {
            roach_write_int(m_roach, "bins0", bins[ch], 0);
            usleep(100);
            roach_write_int(m_roach, "load_bins0", 2*ch + 1, 0);
            usleep(100);
            roach_write_int(m_roach, "load_bins0", 0, 0);
            usleep(100);
        }
    } else {
        for (int ch = 0; ch < m_freqlen; ch++) {
            roach_write_int(m_roach, "bins1", bins[ch], 0);
            usleep(100);
            roach_write_int(m_roach, "load_bins1", 2*ch + 1, 0);
            usleep(100);
            roach_write_int(m_roach, "load_bins1", 0, 0);
            usleep(100);
        }
    }
}

/* Function: roach_define_DDC_LUT
 * ----------------------------
 * Calls function roach_select_bins
 * Checks to see if DDC LUT has been allocated, the populates LUTs using function roach_ddc_comb
 *
 * @param m_roach a roach state structure
 * @param m_freqs list of comb frequencies
 * @param m_freqlen number of frequencies in comb
*/
void roach_define_DDC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_select_bins(m_roach, m_freqs, m_freqlen);
    if (m_roach->lut_idx == 0) {
        if (m_roach->ddc0.len > 0) {
            m_roach->ddc0.len = 0;
            free(m_roach->ddc0.Ival);
            free(m_roach->ddc0.Qval);
        }
        if (m_roach->ddc0.len == 0) {
            m_roach->ddc0.len = LUT_BUFFER_LEN;
            m_roach->ddc0.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
            m_roach->ddc0.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
        }
    } else {
        if (m_roach->ddc1.len > 0) {
            m_roach->ddc1.len = 0;
            free(m_roach->ddc1.Ival);
            free(m_roach->ddc1.Qval);
        }
        if (m_roach->ddc1.len == 0) {
            m_roach->ddc1.len = LUT_BUFFER_LEN;
            m_roach->ddc1.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
            m_roach->ddc1.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
        }
    }
    double phases[m_freqlen];
    for (size_t i = 0; i < m_freqlen; i++) {
        phases[i] = 0.0;
    }
    if (CommandData.roach[m_roach->which - 1].get_phase_centers &&
                        (m_freqlen < m_roach->vna_comb_len)) {
        double phase_centers[m_freqlen];
        // Load phase centers and subtract from original phases
        char *phase_centers_path = m_roach->phase_centers_path;
        FILE *centers_file = fopen(phase_centers_path, "r");
        if (!centers_file) {
            blast_strerror("Could not open %s for reading", phase_centers_path);
        } else {
            blast_info("Opened %s", phase_centers_path);
            for (size_t i = 0; i < m_freqlen; i++) {
                if (fscanf(centers_file, "%lg\n", &phase_centers[i]) != EOF) {
                    blast_info("Roach%d phase_centers: %g", m_roach->which, phase_centers[i]);
                } else {
                    break;
                }
            }
            fclose(centers_file);
        }
        for (size_t i = 0; i < m_freqlen; i++) {
            phases[i] = phase_centers[i];
        }
    // CommandData.roach[m_roach->which - 1].get_phase_centers = 0;
    }
    for (size_t i = 0; i < m_freqlen; i++) {
        double Ival[2 * fft_len];
        double Qval[2 * fft_len];
        int bin = roach_fft_bin_idx(m_roach->freq_residuals,
           i, LUT_BUFFER_LEN / fft_len, FPGA_SAMP_FREQ / (fft_len / 2));
        if (bin < 0) {
            bin += 2048;
        }
        roach_ddc_comb(m_roach, m_roach->freq_residuals[i], m_freqlen, phases[i],
            FPGA_SAMP_FREQ / (fft_len / 2), bin, Ival, Qval);
        if (m_roach->lut_idx == 0) {
            for (int j = i, k = 0; k < 2*fft_len; j += fft_len, k++) {
                m_roach->ddc0.Ival[j] = Ival[k];
                m_roach->ddc0.Qval[j] = Qval[k];
             }
        } else {
            for (int j = i, k = 0; k < 2*fft_len; j += fft_len, k++) {
                m_roach->ddc1.Ival[j] = Ival[k];
                m_roach->ddc1.Qval[j] = Qval[k];
            }
        }
    }
    free(m_roach->freq_residuals);
}

/* Function: roach_pack_luts
 * ----------------------------
 * Combines the DAC and DDC LUTs into a single LUT for writing to the Roach QDR ram
 *
 * @param m_roach a roach state structure
*/
void roach_pack_LUTs(roach_state_t *m_roach)
{
    roach_init_LUT(m_roach, 2 * LUT_BUFFER_LEN);
    if (m_roach->lut_idx == 0) {
        for (size_t i = 0; i < LUT_BUFFER_LEN; i += 2) {
            m_roach->LUT0.Ival[2 * i + 0] = htons(m_roach->dac0.Ival[i + 1]);
            m_roach->LUT0.Ival[2 * i + 1] = htons(m_roach->dac0.Ival[i]);
            m_roach->LUT0.Ival[2 * i + 2] = htons(m_roach->ddc0.Ival[i + 1]);
            m_roach->LUT0.Ival[2 * i + 3] = htons(m_roach->ddc0.Ival[i]);
            m_roach->LUT0.Qval[2 * i + 0] = htons(m_roach->dac0.Qval[i + 1]);
            m_roach->LUT0.Qval[2 * i + 1] = htons(m_roach->dac0.Qval[i]);
            m_roach->LUT0.Qval[2 * i + 2] = htons(m_roach->ddc0.Qval[i + 1]);
            m_roach->LUT0.Qval[2 * i + 3] = htons(m_roach->ddc0.Qval[i]);
        }
    } else {
        for (size_t i = 0; i < LUT_BUFFER_LEN; i += 2) {
            m_roach->LUT1.Ival[2 * i + 0] = htons(m_roach->dac1.Ival[i + 1]);
            m_roach->LUT1.Ival[2 * i + 1] = htons(m_roach->dac1.Ival[i]);
            m_roach->LUT1.Ival[2 * i + 2] = htons(m_roach->ddc1.Ival[i + 1]);
            m_roach->LUT1.Ival[2 * i + 3] = htons(m_roach->ddc1.Ival[i]);
            m_roach->LUT1.Qval[2 * i + 0] = htons(m_roach->dac1.Qval[i + 1]);
            m_roach->LUT1.Qval[2 * i + 1] = htons(m_roach->dac1.Qval[i]);
            m_roach->LUT1.Qval[2 * i + 2] = htons(m_roach->ddc1.Qval[i + 1]);
            m_roach->LUT1.Qval[2 * i + 3] = htons(m_roach->ddc1.Qval[i]);
        }
    }
}

/* Function: roach_write_QDR
 * ----------------------------
 * Writes Roach tone LUT to QDR ram
 *
 * @param m_roach a roach state structure
*/
void roach_write_QDR(roach_state_t *m_roach)
{
    roach_pack_LUTs(m_roach);
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
    roach_write_int(m_roach, "start_dac", 0, 0);
    // Check with LUT pair we're trying to write to
    if (m_roach->lut_idx == 0) {
        if (roach_write_data(m_roach, "qdr0_memory", (uint8_t*)m_roach->LUT0.Ival,
            m_roach->LUT0.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
            blast_info("Could not write to qdr0!");
        }
        if (roach_write_data(m_roach, "qdr1_memory", (uint8_t*)m_roach->LUT0.Qval,
            m_roach->LUT0.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
            blast_info("Could not write to qdr1!");
        }
    } else {
        if (roach_write_data(m_roach, "qdr2_memory", (uint8_t*)m_roach->LUT1.Ival,
            m_roach->LUT1.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
            blast_info("Could not write to qdr0!");
        }
        if (roach_write_data(m_roach, "qdr3_memory", (uint8_t*)m_roach->LUT1.Qval,
            m_roach->LUT1.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
            blast_info("Could not write to qdr1!");
        }
    }
    usleep(3000);
    roach_write_int(m_roach, "start_dac", 1, 0);
    roach_write_int(m_roach, "downsamp_sync_accum_reset", 0, 0);
    roach_write_int(m_roach, "downsamp_sync_accum_reset", 1, 0);
}

/* Function: roach_write_tones
 * ----------------------------
 * Calls functions to upload Roach DAC and DDS LUTs
 *
 * @param m_roach a roach state structure
 * @param m_freqs list of comb frequencies
 * @param m_freqlen number of frequencies in comb
*/
void roach_write_tones(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    m_roach->write_flag = 1;
    if (!m_roach->rpc_conn) {
        blast_info("KATCP read collision");
    } else  {
        m_roach->lut_idx = roach_read_int(m_roach, "qdr_switch");
        blast_info("ROACH%d lut idx = %d", m_roach->which, m_roach->lut_idx);
    }
    roach_init_DACDDC_LUTs(m_roach, LUT_BUFFER_LEN);
    roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);
    roach_define_DDC_LUT(m_roach, m_freqs, m_freqlen);
    blast_info("ROACH%d, Uploading Tone LUT %d...", m_roach->which, m_roach->lut_idx);
    roach_write_QDR(m_roach);
    if (m_roach->lut_idx == 0) {
        free(m_roach->LUT0.Ival);
        free(m_roach->LUT0.Qval);
    } else {
        free(m_roach->LUT1.Ival);
        free(m_roach->LUT1.Qval);
    }
    roach_write_int(m_roach, "write_comb_len", (uint32_t)m_freqlen, 0);
    usleep(1000);
    roach_read_int(m_roach, "read_comb_len");
    blast_info("ROACH%d, Uploaded tones", m_roach->which);
    m_roach->current_ntones = m_freqlen;
    m_roach->write_flag = 0;
}

/* Function: roach_check_streaming
 * ----------------------------
 * Verifies that UDP streaming is active; called after roach_write_tones
 *
 * @param m_roach roach state table
 * @param ntries number of attemps to make before erroring out
 * @param sec_timeout seconds to wait between tries
*/
int roach_check_streaming(roach_state_t *m_roach, int ntries, int sec_timeout)
{
    int retval = -1;
    int m_last_packet_count = roach_udp[m_roach->which - 1].roach_packet_count;
    // blast_info("m_last_packet_count = % d", m_last_packet_count);
    int count = 0;
    while ((count < ntries)) {
        blast_info("roach_packet_count = % d", roach_udp[m_roach->which - 1].roach_packet_count);
        sleep(sec_timeout);
        if (roach_udp[m_roach->which - 1].roach_packet_count > m_last_packet_count) {
            retval = 0;
            break;
        } else {
            count += 1;
        }
        blast_err("Data stream error on ROACH%d", m_roach->which);
    }
    return retval;
}

/* Function: pi_read_string
 * ----------------------------
 * Parses the Pi response via netcat connection, used after setting LO or attenuator
 *
 * @param m_pi Pi state structure
 * @param ntries number of tries before error
 * @param us_timeout microsecond timeout
*/
int pi_read_string(pi_state_t *m_pi, int ntries, int us_timeout)
{
    int retval = -1;
    unsigned char m_read_buffer[READ_BUFFER];
    int bytes_read;
    int count = 0;
    while ((count < ntries)) {
        if (!ph_bufq_len(m_pi->pi_comm->input_buffer)) {
            usleep(us_timeout);
            blast_info("Pi%d timeout", m_pi->which);
        } else {
            while (ph_bufq_len(m_pi->pi_comm->input_buffer)) {
                size_t m_size = (size_t)ph_bufq_len(m_pi->pi_comm->input_buffer);
                bytes_read = remote_serial_read_data(m_pi->pi_comm, m_read_buffer, m_size);
                m_read_buffer[bytes_read++] = '\0';
                usleep(1);
                blast_info("Pi%d: %s", m_pi->which, m_read_buffer);
            }
            retval = 0;
            break;
        }
        count += 1;
    }
    return retval;
}

/* Function: pi_write_string
 * ----------------------------
 * Writes a string to the Piu socket; used for Python calls
 *
 * @param m_bb Pi state structure
 * @param m_data buffer containing message to write
 * @param m_len length of m_data
 *
 * returns: number of bytes succesfully written
*/
int pi_write_string(pi_state_t *m_pi, uint8_t *m_data, size_t m_len)
{
    int bytes_wrote;
    int retval = -1;
    m_data[m_len++] = '\n';
    // m_data[m_len] = 0;
    bytes_wrote = remote_serial_write_data(m_pi->pi_comm, m_data, m_len);
    if (bytes_wrote) retval = bytes_wrote;
    return retval;
}

// TODO(Sam) test this function
/* Function: roach_read_adc
 * ----------------------------
 * Reads back values from ADC snap buffer (in FW) to calculate Vrms of digitized freq comb
 * Attenuators are set to maximize Vrms
 *
 * @param m_roach roach state table
 *
 * returns: RMS voltage of digitized freq comb
*/
double *roach_read_adc(roach_state_t *m_roach)
{
    size_t buffer_len = (1<<12);
    uint16_t *temp_data;
    char* filename;
    char save_path[] = "/home/fc1user/sam_tests/";
    double *rms = malloc(sizeof(double) * 2);
    double irms, qrms, ival, qval, isum, qsum;
    temp_data = calloc((uint16_t)buffer_len, sizeof(uint16_t));
    roach_write_int(m_roach, "adc_snap_adc_snap_ctrl", 0, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_adc_snap_ctrl", 1, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_adc_snap_trig", 0, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_adc_snap_trig", 1, 0);
    usleep(1000);
    roach_write_int(m_roach, "adc_snap_adc_snap_trig", 0, 0);
    usleep(1000);
    roach_read_data(m_roach, (uint8_t*)temp_data,
           "adc_snap_adc_snap_bram", 0, buffer_len * sizeof(uint16_t), READ_DATA_MS_TIMEOUT);
    blast_tmp_sprintf(filename, "%s/adc_vals.dat", save_path);
    FILE *fd = fopen(filename, "w");
    roach_buffer_htons((uint16_t*)temp_data, buffer_len);
    isum = 0;
    qsum = 0;
    for (size_t i = 0; i < buffer_len; i++) {
        if (i % 2 == 0) {
            ival = (double)((int16_t)temp_data[i]);
            ival *= 550.;
            ival /= pow(2, 15);
            fprintf(fd, "%f\n", ival);
            isum += pow(ival, 2);
        } else {
            qval = (double)((int16_t)temp_data[i]);
            qval *= 550.;
            qval /= pow(2, 15);
            fprintf(fd, "%f\n", qval);
            qsum += pow(qval, 2);
        }
    }
    fclose(fd);
    irms = sqrt(isum / ((double)buffer_len / 2.));
    qrms = sqrt(qsum / ((double)buffer_len / 2.));
    rms[0] = irms;
    rms[1] = qrms;
    free(temp_data);
    // blast_info("ROACH%d, ADC V_rms (I,Q) = %f %f\n", m_roach->which, rms[0], rms[0]);
    return rms;
}

void get_adc_rms(roach_state_t *m_roach)
{
    double *rms;
    rms = roach_read_adc(m_roach);
    blast_info("ROACH%d, ADC V_rms (I,Q) = %f %f\n", m_roach->which, rms[0], rms[1]);
}

// TODO(Sam) Finish and test this function
/* Function: set_atten
 * ----------------------------
 * Sets Roach attenuators; levels are in dB, between 0 and 30.
 *
 * @param m_rudat rudat state table
 *
*/
int set_atten(rudat_state_t *m_rudat)
{
    int retval = -1;
    char *m_command;
    char *m_command2;
    int ind = m_rudat->which - 1;
    // blast_info("Pi%d, attempting to set RUDATs...", ind + 1);
    blast_tmp_sprintf(m_command, "sudo ./dual_RUDAT %g %g > rudat.log",
        CommandData.roach_params[m_rudat->which - 1].out_atten,
        CommandData.roach_params[m_rudat->which - 1].in_atten);
    blast_tmp_sprintf(m_command2, "cat rudat.log");
    pi_write_string(&pi_state_table[ind], (unsigned char*)m_command, strlen(m_command));
    pi_write_string(&pi_state_table[ind], (unsigned char*)m_command2, strlen(m_command2));
    if (pi_read_string(&pi_state_table[ind], PI_READ_NTRIES, PI_READ_TIMEOUT) < 0) {
        blast_info("Error setting Atten... reboot Pi%d?", ind + 1);
    } else {
        retval = 0;
    }
    return retval;
}

// TODO(Sam) Finish and test this function
/* Function: cal_adc_rms
 * ----------------------------
 * Attempts to adjust Roach input and output attenuators until full scale of ADC is utilized
 * Attenuators are set to maximize Vrms
 *
 * @param m_roach roach state table
 *
*/

void cal_adc_rms(roach_state_t *m_roach, double targ_rms, double output_atten, int ntries)
{
    if (CommandData.roach_params[m_roach->which - 1].in_atten <= 1.0) {
        CommandData.roach_params[m_roach->which - 1].in_atten += 2.0;
    }
    if (CommandData.roach_params[m_roach->which - 1].in_atten >= 30.0) {
        CommandData.roach_params[m_roach->which - 1].in_atten -= 2.0;
    }
    double *rms;
    // For now, keep output atten at 10 dB
    CommandData.roach_params[m_roach->which - 1].out_atten = output_atten;
    int count = 0;
    double high_range = (double)targ_rms + (double)ADC_RMS_RANGE;
    double low_range = (double)targ_rms - (double)ADC_RMS_RANGE;
    while (count < ADC_CAL_NTRIES) {
        rms = roach_read_adc(m_roach);
        blast_info("ROACH%d, ADC V_rms (I,Q) = %f %f\n", m_roach->which, rms[0], rms[1]);
        blast_info("count = %d", count);
        if ((CommandData.roach_params[m_roach->which - 1].in_atten <= 1.0) ||
                     (CommandData.roach_params[m_roach->which - 1].in_atten >= 30.5)) {
            blast_info("ROACH%d, Input atten limit, aborting cal", m_roach->which);
            break;
        } else {
            // if rms is too low
            blast_info("High range, low_range = %g, %g", high_range, low_range);
            if ((rms[0] < high_range) ||
                      (rms[1] < high_range)) {
                blast_info("ROACH%d, Warning: ADC RMS < %g mV", m_roach->which, high_range);
                CommandData.roach_params[m_roach->which - 1].in_atten -= (double)ATTEN_STEP;
                blast_info("ROACH%d, Adjusting input atten...", m_roach->which);
                set_atten(&rudat_state_table[m_roach->which - 1]);
                if ((rms[0] <= high_range) &&
                      (rms[0] >= low_range)) {
                    blast_info("ROACH%d, ADC cal set", m_roach->which);
                    break;
                }
            }
            // if rms is too high
            if ((rms[0] > high_range) ||
                              (rms[1] > high_range)) {
                blast_info("ROACH%d, Warning: ADC RMS > %g mV", m_roach->which, high_range);
                CommandData.roach_params[m_roach->which - 1].in_atten += (double)ATTEN_STEP;
                blast_info("ROACH%d, Adjusting input atten...", m_roach->which);
                set_atten(&rudat_state_table[m_roach->which - 1]);
                if ((rms[0] <= high_range) &&
                      (rms[0] >= low_range)) {
                    blast_info("ROACH%d, ADC cal set", m_roach->which);
                    break;
                }
            }
        }
        count += 1;
    }
}

int set_output_atten(roach_state_t *m_roach, double new_out_atten)
{
    int retval = -1;
    // double input_atten = CommandData.roach_params[m_roach->which - 1].in_atten;
    CommandData.roach_params[m_roach->which - 1].out_atten = new_out_atten;
    if ((CommandData.roach_params[m_roach->which - 1].out_atten <= 0.5) ||
                 (CommandData.roach_params[m_roach->which - 1].out_atten >= 30.5)) {
        blast_info("ROACH%d, Output atten limit, aborting cal", m_roach->which);
    } else {
        retval = set_atten(&rudat_state_table[m_roach->which - 1]);
    }
    return retval;
}

/* Function: init_valon
 * ----------------------------
 * Sets Valon Synthesizers to their clock and center LO frequencies
 *
 * @param m_roach roach state table
 *
*/
int init_valon(roach_state_t *m_roach)
{
    char *init_command;
    if (EXT_REF) {
        blast_tmp_sprintf(init_command, valon_init_pi_extref);
    } else {
        blast_tmp_sprintf(init_command, valon_init_pi);
    }
    int retval = -1;
    // char *lo_command2;
    // blast_tmp_sprintf(lo_command2, "cat hello_world.log");
    int ind = m_roach->which - 1;
    blast_info("Initializing Valon%d...", ind + 1);
    /* if (pi_write_string(&pi_state_table[ind], (unsigned char*)valon_init_pi, strlen(valon_init_pi)) >= 0) {
        usleep(3000);
    } else {
        return retval;
    }*/
    if (pi_write_string(&pi_state_table[ind], (unsigned char*)init_command, strlen(init_command)) >= 0) {
        if (pi_read_string(&pi_state_table[ind], PI_READ_NTRIES, INIT_VALON_TIMEOUT) < 0) {
            blast_info("Error setting Atten... reboot Pi%d?", ind + 1);
        } else {
            retval = 0;
        }
    } else {
       return retval;
    }
    return retval;
}

/* Function: read_valon
 * ----------------------------
 * Reads Valon Synthesizers after initialization
 *
 * @param m_roach roach state table
 * @param ntries number of attempts before error
*/
int read_valon(roach_state_t *m_roach, int ntries)
{
    int retval = -1;
    int ind = m_roach->which - 1;
    pi_write_string(&pi_state_table[ind], (unsigned char*)read_valon_pi, strlen(read_valon_pi));
    if (pi_read_string(&pi_state_table[ind], ntries, PI_READ_TIMEOUT) < 0) {
        blast_info("Error reading Valon... reboot Pi%d?", ind + 1);
    } else {
        retval = 0;
    }
    return retval;
}

/* Function: roach_save_sweep_packet
 * ----------------------------
 * Grabs N_AVG UDP packets, averages their I and Q values, saves unscaled result to m_sweep_save_path
 * as a single dat file. Format is: int chan, I value, Q value
 *
 * @param m_roach roach state table
 * @param m_sweep_freq LO frequency
 * @param m_sweep_save_path path to save directory
 * @param m_freqlen number of channels (baseband freqs)
*/
void roach_save_sweep_packet(roach_state_t *m_roach, uint32_t m_sweep_freq,
                                     char *m_sweep_save_path, size_t m_freqlen)
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
    double *I_sum = calloc(m_freqlen, sizeof(double)); // Array to store I values to be summed
    double *Q_sum = calloc(m_freqlen, sizeof(double)); // Array to store Q values to be summed
    double *I_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged I values
    double *Q_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged Q values
    while (m_num_received < N_AVG) {
        usleep(1000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            for (size_t chan = 0; chan < m_freqlen; chan ++) {
                I_sum[chan] +=  m_packet.Ival[chan];
                Q_sum[chan] +=  m_packet.Qval[chan];
            }
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    for (size_t chan = 0; chan < m_freqlen; chan++) {
        I_avg[chan] = (I_sum[chan] / N_AVG);
        Q_avg[chan] = (Q_sum[chan] / N_AVG);
        /* Save I_avg, Q_avg, to sweep dir */
        fprintf(m_fd, "%zd\t %g\t %g\n", chan, I_avg[chan], Q_avg[chan]);
        // blast_info("Writing: %zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
    }
    free(I_sum);
    free(Q_sum);
    free(I_avg);
    free(Q_avg);
    fclose(m_fd);
}

/* Function: get_time
 * ----------------------------
 * Gets the ctime, for use in a file or directory name
 *
 * @param time_buffer a buffer for storing the return of ctime_r
*/
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

/* Function: roach_timestamp_init
 * -------------------
 * When called, begins writing absolute time in seconds (from FC2)
 * to Roach SW register 'GbE_ctime'. This timestamp is included in the UDP
 * packet which is packetized one second later.
 *
 * @param ind roach index
*/
void roach_timestamp_init(uint16_t ind)
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    char *sec;
    char *nsec;
    char sec_truncated[2];
    char nsec_truncated[2];
    char *timestamp;
    blast_tmp_sprintf(sec, "%ld", ts.tv_sec);
    blast_tmp_sprintf(nsec, "%ld", ts.tv_nsec);
    strncpy(sec_truncated, sec+4, 8);
    strncpy(nsec_truncated, nsec, 3);
    sec_truncated[6] = '\0';
    nsec_truncated[3] = '\0';
    /* blast_info("%s", sec);
    blast_info("%s", sec_truncated);
    blast_info("%s", nsec);
    blast_info("%s", nsec_truncated);
    */
    blast_tmp_sprintf(timestamp, "%s%s", sec_truncated, nsec_truncated);
    // blast_info("%s", timestamp);
    // blast_info("%u", atoi(timestamp));
    if (roach_state_table[ind].write_flag || !roach_state_table[ind].katcp_fd
                  || roach_state_table[ind].status < ROACH_STATUS_TONE) {
        return;
    } else {
        roach_write_int(&roach_state_table[ind], "GbE_ctime", atoi(timestamp), 0);
        // sleep(1);
        // roach_read_int(&roach_state_table[ind], "GbE_ctime");
        return;
    }
}

/* Function: get_path
 * ----------------------------
 * Acquires a path for file storage
 *
 * @param m_roach roach state structure
 * @param m_dir_root the path's root directory
 *
 * returns: the save_path
*/
char* get_path(roach_state_t *m_roach, char *m_dir_root)
{
    char time_buffer[FILENAME_MAX];
    char *save_path;
    get_time(time_buffer);
    asprintf(&save_path, "%s/%s", m_dir_root, time_buffer);
    return save_path;
}

/* Function: mkdir_recursive
 * ----------------------------
 * Creates a directory
 *
 * @param m_directory the directory to be created
 *
*/
static int mkdir_recursive(char *m_directory)
{
    char current_path[PATH_MAX];
    char *path_chunk = NULL;
    char *strtok_ref = NULL;
    int ret = 0;
    int stat_return = 0;
    char *path_ptr;
    size_t path_chunk_len = 0;
    size_t current_path_len = 0;
    struct stat dir_stat;
    current_path[0]=0;
    path_ptr = current_path;

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

/* Function: create_sweepdir
 * ----------------------------
 * Creates a sweep directory
 *
 * @param m_roach roach state table
 * @param sweep_type:
 *    0 = VNA
 *    1 = TARG
 *    2 = CAL_AMPS
 *    3 = CHOP
*/
int create_sweepdir(roach_state_t *m_roach, int sweep_type)
{
    int retval = -1;;
    char *new_path;
    char *path_root;
    char *type;
    if ((sweep_type == VNA)) {
        path_root = m_roach->vna_path_root;
        type = "VNA";
    } else if ((sweep_type == TARG)) {
        path_root = m_roach->targ_path_root;
        type = "TARGET";
    } else if ((sweep_type == CAL_AMPS)) {
        path_root = m_roach->cal_path_root;
        type = "CAL_AMPS";
    } else if ((sweep_type == CHOP)) {
        path_root = m_roach->chop_path_root;
        type = "CHOP";
    }
    new_path = get_path(m_roach, path_root);
    if (sweep_type == VNA) {
        asprintf(&m_roach->last_vna_path, new_path);
    } else if (sweep_type == TARG) {
        asprintf(&m_roach->last_targ_path, new_path);
    } else if (sweep_type == CAL_AMPS) {
        asprintf(&m_roach->last_cal_path, new_path);
    } else if (sweep_type == CHOP) {
        asprintf(&m_roach->last_chop_path, new_path);
    }
    blast_info("ROACH%d, %s files will be saved in %s", m_roach->which, type, new_path);
    if (mkdir_recursive(new_path)) {
        retval = 1;
    } else {
        blast_strerror("Could not create new directory: %s", new_path);
    }
    return retval;
}

/* Function: get_targ_freqs
 * ----------------------------
 * After running a VNA sweep. Calls python script find_kids_blast.py.
 * Python arguments populated with blastcmd
 * Populate m_roach->targ_tones, the TARG sweep freq comb
 *
 * @param m_roach roach state table
 * @param m_last_vna_path the path to the most recent VNA sweep files
 * @parm m_last_targ the path for saving the TARG sweep
 *
*/
int get_targ_freqs(roach_state_t *m_roach, char *m_vna_path, char* m_targ_path)
{
    /* if (!m_targ_path) {
        if (create_sweepdir(m_roach, TARG)) {
            blast_info("Roach%d, created %s", m_roach->which, m_roach->last_targ_path);
        } else {
            blast_info("Roach%d, could not create new target sweep directory", m_roach->which);
        }
    } */
    // Check for last VNA path. If doesn't exist, use hardcoded vna search path
    // (a previous sweep) as specified at top of file
    if (!m_vna_path) {
        blast_info("Roach%d, NO VNA PATH FOUND, using VNA SEARCH PATH:%s instead",
                m_roach->which, vna_search_path);
        blast_tmp_sprintf(m_vna_path, vna_search_path);
    }
    char *py_command;
    char *m_targ_freq_path;
    char *find_kids_script;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    char m_line[READ_LINE];
    if (m_roach->which <= 2) {
        blast_tmp_sprintf(find_kids_script, find_kids_350);
    } else {
        blast_tmp_sprintf(find_kids_script, find_kids_250);
    }
    blast_info("Calling Python script...");
    blast_tmp_sprintf(py_command, "python %s %d %s %g %g %g %g %g > %s",
        find_kids_script,
        m_roach->which,
        m_vna_path,
        // m_roach->last_targ_path,
        CommandData.roach_params[m_roach->which - 1].smoothing_scale,
        CommandData.roach_params[m_roach->which - 1].peak_threshold,
        CommandData.roach_params[m_roach->which - 1].spacing_threshold,
        m_roach->lo_centerfreq,
        (double)LO_STEP/1000.,
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
    blast_tmp_sprintf(m_targ_freq_path,
           "/home/fc1user/sam_tests/sweeps/roach%d/bb_targ_freqs.dat", m_roach->which);
    // blast_tmp_sprintf(m_targ_freq_path, "%s/bb_targ_freqs.dat", m_roach->last_targ_path);
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
        // blast_info("KID freq = %lg", m_roach->targ_tones[j] + m_roach->lo_centerfreq);
    }
    blast_info("Uploading TARGET comb...");
    roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
    blast_info("ROACH%d, TARGET comb uploaded", m_roach->which);
    blast_info("ROACH%d, Calibrating ADC rms voltages...", m_roach->which);
    cal_adc_rms(m_roach, ADC_TARG_RMS_250, OUTPUT_ATTEN_TARG, ADC_CAL_NTRIES);
    if (CommandData.roach[m_roach->which - 1].find_kids) {
        CommandData.roach[m_roach->which - 1].find_kids = 0;
    }
    m_roach->status = ROACH_STATUS_ARRAY_FREQS;
    return 0;
}

int calc_grad_freqs(roach_state_t *m_roach, char* m_last_targ_path)
{
    char *calc_freqs_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    m_roach->targ_tones_LUT0 = calloc(m_roach->num_kids, sizeof(double));
    m_roach->targ_tones_LUT1 = calloc(m_roach->num_kids, sizeof(double));
    blast_tmp_sprintf(calc_freqs_command,
           "python /data/etc/blast/roachPython/calc_targ_freqs.py %s %g",
           m_last_targ_path,
           m_roach->lo_centerfreq);
    blast_info("%s", calc_freqs_command);
    system(calc_freqs_command);
    sleep(6);
    blast_tmp_sprintf(m_targ_freq_path, "%s/gradient_freqs.dat", m_last_targ_path);
    blast_info("Opening gradient freqs for reading...");
    FILE *fd;
    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return -1;
    }
    for (size_t m_index = 0; m_index < m_roach->num_kids; m_index++) {
        if (fscanf(fd, "%lg\n", &m_temp_freqs[m_index]) == EOF) break;
    }
    fclose(fd);
    /* while (m_roach->num_kids < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &m_temp_freqs[(m_roach->num_kids)++]) != EOF) {
    }*/
    /* for (size_t j = 0; j < m_roach->num_kids; j++) {
        blast_info("temp_freqs: %lg", m_temp_freqs[j]);
    } */
    blast_info("populating targ_tones...");
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j] - m_roach->lo_centerfreq;
        blast_info("%g", m_roach->targ_tones[j]);
        m_roach->targ_tones_LUT0[j] = (m_temp_freqs[j] - (double)LUT_FREQ_OFFSET)
                                 - m_roach->lo_centerfreq;
        m_roach->targ_tones_LUT1[j] = (m_temp_freqs[j] + (double)LUT_FREQ_OFFSET)
                                 - m_roach->lo_centerfreq;
    }
    return 0;
}

// TODO(Sam) finish/test this function
/* Function: optimize_targ_tones
 * ----------------------------
 * Attempts to fine tune freqs located with get_targ_freqs
 *
 * @param m_roach roach state table
 * @parm m_last_targ the path for saving the TARG sweep
*/
int optimize_targ_tones(roach_state_t *m_roach, char *m_last_targ_path)
{
    char *py_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    char m_line[256];
    blast_tmp_sprintf(py_command,
            "python /data/etc/blast/roachPython/optimize_freqs_mcp.py %s > %s",
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
        blast_info("Optimized KID freq = %lg", m_roach->targ_tones[j]);
    }
    if (CommandData.roach[m_roach->which - 1].find_kids) {
        CommandData.roach[m_roach->which - 1].find_kids = 0;
    }
    return 1;
}

/* Function: recenter_lo
 * ----------------------------
 * Returns the LO to center position
 *
 * @param m_roach roach state table
*/
int recenter_lo(roach_state_t *m_roach)
{
    int ind = m_roach->which - 1;
    pi_state_t *m_pi = &pi_state_table[ind];
    char *lo_command;
    blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g",
                  m_roach->lo_centerfreq/1.0e6);
    pi_write_string(m_pi, (unsigned char*)lo_command, strlen(lo_command));
    return 0;
}

void phase_centers(roach_state_t *m_roach, char *m_targ_path)
{
    if (!m_targ_path) {
        blast_info("Roach%d, NO TARG PATH FOUND, using TARG SEARCH PATH:%s instead",
                m_roach->which, targ_search_path);
        blast_tmp_sprintf(m_targ_path, targ_search_path);
    }
    char *center_phase_command;
    blast_tmp_sprintf(center_phase_command, "python %s %d %s",
                  center_phase_script, m_roach->which, m_targ_path);
    blast_info("%s", center_phase_command);
    // Calculate phase centers from last TARG sweep, and write file to disk
    system(center_phase_command);
}

// TODO(Sam) add cal and grad sweeps as sweep types
/* Function: roach_do_sweep
 * ----------------------------
 * Performs a sweep and save data operation
 *
 * @param m_roach roach state table
 * @param sweep_type:
 *    0 = VNA
 *    1 = TARG
*/
int roach_do_sweep(roach_state_t *m_roach, int sweep_type)
{
    blast_info("ROACH%d, disabling LUT switch", m_roach->which - 1);
    enable_qdr_switch(m_roach, 0);
    int ind = m_roach->which - 1;
    if (!CommandData.roach[ind].do_sweeps) {
        return SWEEP_INTERRUPT;
    }
    pi_state_t *m_pi = &pi_state_table[ind];
    if (!CommandData.roach[ind].do_sweeps) {
        return SWEEP_INTERRUPT;
    }
    char *save_bbfreqs_command;
    char *save_vna_trf_command;
    char *save_targ_trf_command;
    double m_span;
    char *sweep_freq_fname;
    char *save_path;
    size_t comb_len;
    // struct stat dir_stat;
    // int stat_return;
    if (sweep_type == VNA) {
        char *vna_freq_fname;
        if (create_sweepdir(m_roach, VNA)) {
            // TODO(Sam) for short sweep, m_span = VNA_SWEEP_SPAN
            m_span = m_roach->vna_sweep_span;
            // m_span = VNA_SWEEP_SPAN;
            blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_vna_path);
            blast_tmp_sprintf(vna_freq_fname, "%s/vna_freqs.dat", m_roach->last_vna_path);
            if (save_freqs(m_roach, vna_freq_fname, m_roach->vna_comb, m_roach->vna_comb_len) < 0) {
                blast_err("Sweep freqs could not be saved to disk");
                return SWEEP_FAIL;
            }
            // Save last system transfer function
            blast_tmp_sprintf(save_vna_trf_command, "cp %s/roach%d/vna_trf.dat %s",
                bb_targ_freqs_path, m_roach->which, m_roach->last_vna_path);
            system(save_vna_trf_command);
            save_path = m_roach->last_vna_path;
            comb_len = m_roach->vna_comb_len;
         } else {
             return SWEEP_FAIL;
         }
    }
    if (sweep_type == TARG) {
        m_span = TARG_SWEEP_SPAN;
        // stat_return = stat(m_roach->last_targ_path, &dir_stat);
        // if (stat_return != 0) {
        // if (!m_roach->last_targ_path) {
        // Always create new TARG dir, copy bb_targ_freqs.dat into it
        if (create_sweepdir(m_roach, TARG)) {
            blast_info("ROACH%d, TARGET sweep will be saved in %s",
                           m_roach->which, m_roach->last_targ_path);
        } else {
            blast_info("ROACH%d, Could not create TARG dir", m_roach->which);
            return SWEEP_FAIL;
        }
        // }
        save_path = m_roach->last_targ_path;
        // Copy bb_targ_freqs.dat to new TARG sweep dir
        blast_tmp_sprintf(save_bbfreqs_command, "cp %s/roach%d/bb_targ_freqs.dat %s",
                        bb_targ_freqs_path, m_roach->which, m_roach->last_targ_path);
        system(save_bbfreqs_command);
        // Save initial transfer function into targ sweep dir
        blast_tmp_sprintf(save_targ_trf_command, "cp %s/roach%d/first_targ_trf.dat %s",
                        bb_targ_freqs_path, m_roach->which, m_roach->last_targ_path);
        system(save_targ_trf_command);
        blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_targ_path);
        blast_info("Sweep freq fname = %s", sweep_freq_fname);
        // TODO(Sam) if not testing, uncomment the following
        /* if (save_freqs(m_roach, targ_freq_fname, m_roach->targ_tones, m_roach->num_kids) < 0) {
            blast_err("Sweep freqs could not be saved to disk");
            return SWEEP_FAIL;
        }*/
        comb_len = m_roach->num_kids;
    }
    /* Determine sweep frequencies */
    FILE *m_sweep_fd = fopen(sweep_freq_fname, "w");
    if (!m_sweep_fd) {
        blast_strerror("Could not open %s for writing", sweep_freq_fname);
        CommandData.roach[ind].do_sweeps = 0;
        return SWEEP_FAIL;
    }
    double m_min_freq = m_roach->lo_centerfreq - (m_span/2.);
    double m_max_freq = m_roach->lo_centerfreq + (m_span/2.);
    size_t m_num_sweep_freqs = (m_max_freq - m_min_freq) / LO_STEP;
    char *lo_command; /* Pi command */
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
                blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g",
                   m_sweep_freqs[i]/1.0e6);
            m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
            pi_write_string(m_pi, (unsigned char*)lo_command, strlen(lo_command));
            // pi_write_string(m_pi, (unsigned char*)lo_command2, strlen(lo_command2));
            if (pi_read_string(&pi_state_table[ind], PI_READ_NTRIES, LO_READ_TIMEOUT) < 0) {
                blast_info("Error setting LO... reboot Pi%d?", ind + 1);
            }
            usleep(SWEEP_TIMEOUT);
            roach_save_sweep_packet(m_roach, (uint32_t)m_sweep_freqs[i], save_path, comb_len);
        } else {
            blast_info("Sweep interrupted by command");
            CommandData.roach[ind].do_sweeps = 0;
            return SWEEP_INTERRUPT;
        }
    }
    usleep(SWEEP_TIMEOUT);
    if (recenter_lo(m_roach) < 0) {
        blast_info("Error recentering LO");
    } else {
    // TODO(Sam) Put in error handling
        if (pi_read_string(&pi_state_table[ind], PI_READ_NTRIES, LO_READ_TIMEOUT) < 0) {
            blast_info("Error setting LO... reboot Pi%d?", ind + 1);
        }
    }
    free(m_sweep_freqs);
    CommandData.roach[ind].do_sweeps = 0;
    /*
    if (sweep_type == TARG) {
        phase_centers(m_roach, m_roach->last_targ_path);
    } */
    return SWEEP_SUCCESS;
}

void roach_calc_ref_grad(roach_state_t *m_roach, int m_chan, double *I0_avg,
                                                double *Q0_avg, double *I1_avg, double *Q1_avg)
{
    m_roach->ref_grad[m_chan][0] = (I1_avg[m_chan] - I0_avg[m_chan])/(2*LUT_FREQ_OFFSET);
    m_roach->ref_grad[m_chan][1] = (Q1_avg[m_chan] - Q0_avg[m_chan])/(2*LUT_FREQ_OFFSET);
    m_roach->ref_val0[m_chan][0] = I0_avg[m_chan];
    m_roach->ref_val0[m_chan][1] = Q0_avg[m_chan];
    m_roach->ref_val1[m_chan][0] = I1_avg[m_chan];
    m_roach->ref_val1[m_chan][1] = Q1_avg[m_chan];
}

void roach_calc_df(roach_state_t *m_roach)
{
    // Grab a set number of packets for each LUT pair, average I and Q for each chan
    // This must be run AFTER target sweep, to populate m_roach->num_kids
    size_t m_freqlen = m_roach->num_kids;
    // Get data from UDP
    // This is the number of packets grabbed so far1
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    double dIdf, dQdf, norm, delta_I, delta_Q;
    double *I0_sum = calloc(m_freqlen, sizeof(double)); // Array to store I0 values to be summed
    double *Q0_sum = calloc(m_freqlen, sizeof(double)); // Array to store Q0 values to be summed
    double *I0_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged I0 values
    double *Q0_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged Q0 values
    double *I1_sum = calloc(m_freqlen, sizeof(double)); // Array to store I1 values to be summed
    double *Q1_sum = calloc(m_freqlen, sizeof(double)); // Array to store Q1 values to be summed
    double *I1_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged I1 values
    double *Q1_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged Q1 values
    // Switch to first pair
    roach_write_int(m_roach, "qdr_switch", 0, 0);
    while (m_num_received < N_AVG) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            for (size_t chan = 0; chan < m_freqlen; chan ++) {
                I0_sum[chan] +=  m_packet.Ival[chan];
                Q0_sum[chan] +=  m_packet.Qval[chan];
            }
                }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    // Switch to second LUT pair
    roach_write_int(m_roach, "qdr_switch", 1, 0);
    while (m_num_received < N_AVG) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            for (size_t chan = 0; chan < m_freqlen; chan ++) {
                I1_sum[chan] +=  m_packet.Ival[chan];
                Q1_sum[chan] +=  m_packet.Qval[chan];
            }
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    for (size_t chan = 0; chan < m_freqlen; chan++) {
        I0_avg[chan] = (I0_sum[chan] / N_AVG);
        Q0_avg[chan] = (Q0_sum[chan] / N_AVG);
        I1_avg[chan] = (I1_sum[chan] / N_AVG);
        Q1_avg[chan] = (Q1_sum[chan] / N_AVG);
        // blast_info("%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
    }
    // free sum memory buffers
    free(I0_sum);
    free(Q0_sum);
    free(I1_sum);
    free(Q1_sum);
    // Calculate reference IQ gradient and delta f
    // If a reference gradient needs to be created, do so here, then calculate delta f
    if ((!m_roach->has_ref)) {
        for (int chan = 0; chan < m_roach->num_kids; ++chan) {
            roach_calc_ref_grad(m_roach, chan, I0_avg, Q0_avg, I1_avg, Q1_avg);
            blast_info("ROACH%d, stored REF grads & delta F", m_roach->which);
        }
    m_roach->has_ref = 1;
    }
    // Calculate delta f for each pixel, using the value of I0_avg as reference
    for (int chan = 0; chan < m_roach->num_kids; ++chan) {
        dIdf = m_roach->ref_grad[chan][0];
        dQdf = m_roach->ref_grad[chan][1];
        norm = (dIdf*dIdf) + (dQdf*dQdf);
        delta_I = 0.5 *((I1_avg[chan] + I0_avg[chan]) -
                 (m_roach->ref_val1[chan][0] + m_roach->ref_val0[chan][0]));
        delta_Q = 0.5 *((Q1_avg[chan] + Q0_avg[chan]) -
                 (m_roach->ref_val1[chan][1] + m_roach->ref_val0[chan][1]));
        m_roach->df[chan] = 2*LUT_FREQ_OFFSET * ((delta_I*dIdf) + (delta_Q*dQdf)) / norm;
        blast_info("R%d chan%d, df = %g", m_roach->which, chan, m_roach->df[chan]);
    }
    // free avg memory buffers
    free(I0_avg);
    free(Q0_avg);
    free(I1_avg);
    free(Q1_avg);
}

void cal_get_mags(roach_state_t *m_roach, uint32_t m_sweep_freq,
              size_t m_freqlen, double **sweep_buffer, int m_sweep_freq_idx)
{
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    double *I_sum = calloc(m_freqlen, sizeof(double)); // Array to store I values to be summed
    double *Q_sum = calloc(m_freqlen, sizeof(double)); // Array to store Q values to be summed
    double *I_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged I values
    double *Q_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged Q values
    while (m_num_received < N_AVG) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            for (size_t chan = 0; chan < m_freqlen; chan ++) {
                I_sum[chan] +=  m_packet.Ival[chan];
                Q_sum[chan] +=  m_packet.Qval[chan];
            }
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    for (int chan = 0; chan < m_freqlen; chan++) {
        I_avg[chan] = (I_sum[chan] / N_AVG);
        Q_avg[chan] = (Q_sum[chan] / N_AVG);
        sweep_buffer[m_sweep_freq_idx][chan] = sqrt(I_avg[chan]*I_avg[chan] + Q_avg[chan]*Q_avg[chan]);
        // blast_info("%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
    // Calculate minimum tone amplitudes for each channel
    }
    free(I_sum);
    free(Q_sum);
    free(I_avg);
    free(Q_avg);
}

// Save short timestreams for single channel (using for cal lamp tests outside of KST)
int save_timestream(roach_state_t *m_roach, int m_chan, double m_nsec)
{
    struct stat dir_stat;
    int stat_return;
    char *file_out;
    // create new chop directory, if doesn't exist
    stat_return = stat(m_roach->last_chop_path, &dir_stat);
    if (stat_return != 0) {
        if (create_sweepdir(m_roach, CHOP)) {
            blast_info("ROACH%d, CHOPS will be saved in %s",
                           m_roach->which, m_roach->last_chop_path);
        } else {
            blast_err("Could not create new chop directory");
            CommandData.roach[m_roach->which - 1].get_timestream = 0;
            return -1;
        }
    }
    blast_tmp_sprintf(file_out, "%s/%d.dat", m_roach->last_chop_path, m_chan);
    blast_info("chan, nsec: %d, %f", m_chan, m_nsec);
    int npoints = round(m_nsec * (double)DAC_FREQ_RES);
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    // open file for writing
    blast_info("ROACH%d, saving %d points for chan%d over %f sec", m_roach->which, npoints, m_chan, m_nsec);
    FILE *fd = fopen(file_out, "wb");
    for (int i = 0; i < npoints; i++) {
        // blast_info("i = %d", i);
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            float I = m_packet.Ival[m_chan];
            float Q = m_packet.Qval[m_chan];
            // write a 4 byte I value
            fwrite(&I, 4, 1, fd);
            // write a 4 byte Q value
            fwrite(&Q, 4, 1, fd);
            m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
        }
    }
    fclose(fd);
    // Save last chop path
    system("python /home/fc1user/sam_builds/chop_list.py");
    blast_info("ROACH%d, timestream saved", m_roach->which);
    CommandData.roach[m_roach->which - 1].get_timestream = 0;
    return 0;
}

// saves timestreams for all channels
int save_all_timestreams(roach_state_t *m_roach, double m_nsec)
{
    struct stat dir_stat;
    int stat_return;
    char *file_out;
    // create new chop directory, if doesn't exist
    stat_return = stat(m_roach->last_chop_path, &dir_stat);
    if (stat_return != 0) {
        if (create_sweepdir(m_roach, CHOP)) {
            blast_info("ROACH%d, CHOPS will be saved in %s",
                           m_roach->which, m_roach->last_chop_path);
        } else {
            blast_err("Could not create new chop directory");
            CommandData.roach[m_roach->which - 1].get_timestream = 0;
            return -1;
        }
    }
    // allocate memory to hold timestreams before saving to file
    int npoints = round(m_nsec * (double)DAC_FREQ_RES);
    int rows = m_roach->current_ntones;
    int cols = npoints;
    float *I[rows];
    float *Q[rows];
    for (int i = 0; i < rows; i++) {
         I[i] = (float *)malloc(cols * sizeof(float));
         Q[i] = (float *)malloc(cols * sizeof(float));
    }
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    blast_info("Getting data...");
    for (int i = 0; i < npoints; i++) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
            for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
                I[chan][i] = m_packet.Ival[chan];
                Q[chan][i] = m_packet.Qval[chan];
            }
        }
    }
    blast_info("Data acquired...");
    // open file for writing
    blast_info("current ntones = %zd", m_roach->current_ntones);
    for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
        // blast_info("Chan = %zd", chan);
        blast_tmp_sprintf(file_out, "%s/%zd.dat", m_roach->last_chop_path, chan);
        // blast_info("Saving %s", file_out);
        FILE *fd = fopen(file_out, "wb");
        for (int j = 0; j < npoints; j++) {
            // write a 4 byte I value
            fwrite(&I[chan][j], 4, 1, fd);
            // write a 4 byte value
            fwrite(&Q[chan][j], 4, 1, fd);
        }
        fclose(fd);
    }
    // Save last chop path
    system("python /home/fc1user/sam_builds/chop_list.py");
    blast_info("ROACH%d, timestream saved", m_roach->which);
    CommandData.roach[m_roach->which - 1].get_timestream = 0;
    for (int i = 0; i < rows; i++) {
        free(I[i]);
        free(Q[i]);
    }
    return 0;
}

// get the chop_snr for the timestream saved in save_timestream
int chop_snr(roach_state_t *m_roach, double *m_buffer)
{
    int retval = -1;
    char chop_snr_log[] = "/home/fc1user/sam_tests/chop_snr.log";
    char *py_command;
    blast_tmp_sprintf(py_command, "python %s > %s %d", chop_snr_script,
              chop_snr_log, m_roach->which);
    // Call fit_mcp_chop.py and read response from log
    system(py_command);
    FILE *fd = fopen(chop_snr_log, "r");
    if (!fd) {
        blast_err("Error opening file");
        return retval;
    }
    if (!fscanf(fd, "%lg\n", m_buffer)) {
        blast_err("Error reading chop log file");
    } else {
        retval = 0;
    }
    return retval;
}

int change_targ_amps(roach_state_t *m_roach, double *m_delta_amps, int *m_channels, size_t m_len)
{
    int retval = -1;
    /* double amps[m_roach->num_kids];
    char *amps_path = m_roach->targ_amps_path[2];
    if ((roach_read_1D_file(m_roach, amps_path, amps, m_roach->num_kids) < 0)) {
        return retval;
    } */
    if (!m_roach->last_amps) {
        m_roach->last_amps = calloc(m_roach->num_kids, sizeof(double));
    }
    for (size_t i = 0; i < m_len; i++) {
        m_roach->last_amps[m_channels[i]] += m_delta_amps[i];
        /* blast_info("ROACH%d, chan%d amp = %f", m_roach->which,
                      m_channels[i], m_roach->last_amps[m_channels[i]]); */
    }
    CommandData.roach[m_roach->which - 1].change_tone_amps = 1;
    roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
    CommandData.roach[m_roach->which - 1].change_tone_amps = 0;
    /* if ((roach_save_1D_file(m_roach, amps_path, amps, m_roach->num_kids) < 0)) {
        return retval;
    } */
    return retval;
}

// Do a lamp chop and save timestream
void chop_and_save(roach_state_t *m_roach, int m_chan, int m_num_pulse,
                   int m_length, int m_separation, double m_nsec)
{
    // chop the lamp
    CommandData.Cryo.periodic_pulse = 1;
    CommandData.Cryo.num_pulse = m_num_pulse;
    CommandData.Cryo.separation = m_separation;
    CommandData.Cryo.length = m_length;
    periodic_cal_control();
    // get the timestream (save to file for now)
    save_timestream(m_roach, m_chan, m_nsec);
}

/* Function: cal_sweep
 * ----------------------------
 * Performs a calibration sweep
 * Sweep data is saved like VNA/TARG sweeps
 *
 * @param m_roach roach state table
*/
int cal_sweep(roach_state_t *m_roach, char *subdir)
{
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 1;
    int npoints = CommandData.roach_params[m_roach->which - 1].npoints;
    blast_info("NPOINTS = %d", npoints);
    pi_state_t *m_pi = &pi_state_table[m_roach->which - 1];
    struct stat dir_stat;
    int stat_return;
    char *lo_command; /* Pi command */
    char *save_bbfreqs_command;
    char *sweep_freq_fname;
    double m_sweep_freqs[npoints];
    double span = ((double)npoints - 1.0)/2.0;
    m_sweep_freqs[0] = m_roach->lo_centerfreq - span*LO_STEP;
    for (size_t i = 1; i < npoints; i++) {
        m_sweep_freqs[i] = m_sweep_freqs[i - 1] + LO_STEP;
    }
    // If doesn't exist, create cal sweep parent directory
    stat_return = stat(m_roach->last_cal_path, &dir_stat);
    if (stat_return != 0) {
        if (create_sweepdir(m_roach, CAL_AMPS)) {
            blast_info("ROACH%d, CAL sweeps will be saved in %s",
                           m_roach->which, m_roach->last_cal_path);
        // Save bb_targ_freqs.dat in CAL dir
        blast_tmp_sprintf(save_bbfreqs_command, "cp %s/roach%d/bb_targ_freqs.dat %s",
                        bb_targ_freqs_path, m_roach->which, m_roach->last_cal_path);
        system(save_bbfreqs_command);
        } else {
            blast_info("ROACH%d, Could not create CAL dir", m_roach->which);
            return SWEEP_FAIL;
        }
    }
    char *new_path;
    char *save_path;
    // asprintf(&new_path, "%s/%s", m_roach->last_cal_path, subdir);
    blast_tmp_sprintf(new_path, "%s/%s", m_roach->last_cal_path, subdir);
    blast_tmp_sprintf(save_path, "%s/%s", m_roach->last_cal_path, subdir);
    // Create child directory to hold cal sweeps, with name of count (0,...,break)
    if (mkdir_recursive(new_path)) {
        blast_info("ROACH%d, New %s sweep will be saved in %s", m_roach->which, "CAL", save_path);
    } else {
        blast_strerror("Could not create new directory: %s", save_path);
        return SWEEP_FAIL;
    }
    // Save sweep freqs in cal subdir
    blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_cal_path);
    save_sweep_freqs(m_roach, sweep_freq_fname, m_sweep_freqs, npoints);
    // Do a series of sweeps while recalculating amplitudes
    for (size_t i = 0; i < npoints; i++) {
        if (CommandData.roach[m_roach->which - 1].do_cal_sweeps) {
                blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g",
                   m_sweep_freqs[i]/1.0e6);
            m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
            pi_write_string(m_pi, (unsigned char*)lo_command, strlen(lo_command));
            // pi_write_string(m_pi, (unsigned char*)lo_command2, strlen(lo_command2));
            if (pi_read_string(&pi_state_table[m_roach->which - 1], PI_READ_NTRIES, LO_READ_TIMEOUT) < 0) {
                blast_info("Error setting LO... reboot Pi%d?", m_roach->which);
            }
            usleep(SWEEP_TIMEOUT);
            roach_save_sweep_packet(m_roach, (uint32_t)m_sweep_freqs[i], save_path, m_roach->num_kids);
        } else {
            blast_info("Sweep interrupted by command");
            CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
            return SWEEP_INTERRUPT;
        }
    }
    usleep(SWEEP_TIMEOUT);
    if (recenter_lo(m_roach) < 0) {
        blast_info("Error recentering LO");
    } else {
    // TODO(Sam) Put in error handling
        if (pi_read_string(&pi_state_table[m_roach->which - 1], PI_READ_NTRIES, LO_READ_TIMEOUT) < 0) {
            blast_info("Error setting LO... reboot Pi%d?", m_roach->which);
        }
    }
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
    return SWEEP_SUCCESS;
}

int roach_refit_freqs(roach_state_t *m_roach)
{
    char subdir[] = "sweep_data";
    char *py_command;
    char *load_path;
    double new_freqs[m_roach->num_kids];
    int status = cal_sweep(m_roach, subdir);
    if (status == SWEEP_SUCCESS) {
        system("python /home/fc1user/sam_builds/sweep_list.py cal");
        blast_info("ROACH%d, Fit sweep complete, calculating new freqs", m_roach->which);
        blast_tmp_sprintf(py_command, "python %s %s/%s", refit_freqs_script, m_roach->last_cal_path, subdir);
        blast_info("Command: %s", py_command);
        system(py_command);
        blast_tmp_sprintf(load_path, "%s/bb_targ_freqs.dat", m_roach->last_cal_path);
        roach_read_1D_file(m_roach, load_path, new_freqs, m_roach->num_kids);
        for (size_t i = 0; i < m_roach->num_kids; i++) {
            m_roach->targ_tones[i] = new_freqs[i];
        }
        roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
        CommandData.roach[m_roach->which - 1].refit_res_freqs = 0;
        CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
        free(m_roach->last_cal_path);
        return SWEEP_SUCCESS;
        // rewrite tones!
    } else if ((status == SWEEP_INTERRUPT)) {
        blast_info("ROACH%d, Sweep interrupted by blastcmd", m_roach->which);
        m_roach->status = ROACH_STATUS_ACQUIRING;
        m_roach->desired_status = ROACH_STATUS_ACQUIRING;
        free(m_roach->last_cal_path);
        CommandData.roach[m_roach->which - 1].refit_res_freqs = 0;
        CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
        return SWEEP_INTERRUPT;
    } else {
        blast_info("ROACH%d, Sweep failed", m_roach->which);
        m_roach->status = ROACH_STATUS_ACQUIRING;
        m_roach->desired_status = ROACH_STATUS_ACQUIRING;
        free(m_roach->last_cal_path);
        CommandData.roach[m_roach->which - 1].refit_res_freqs = 0;
        CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
        return SWEEP_FAIL;
    }
    CommandData.roach[m_roach->which - 1].refit_res_freqs = 0;
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
    return SWEEP_SUCCESS;
}

int cal_sweep_amps(roach_state_t *m_roach, double **sweep_buffer)
{
    if (!CommandData.roach[m_roach->which - 1].do_cal_sweeps) {
        return SWEEP_INTERRUPT;
    }
    pi_state_t *m_pi = &pi_state_table[m_roach->which - 1];
    char *lo_command; /* Pi command */
    double m_sweep_freqs[NCAL_POINTS];
    double span = ((double)NCAL_POINTS - 1.0)/2.0;
    m_sweep_freqs[0] = m_roach->lo_centerfreq - span*LO_STEP;
    for (size_t i = 0; i < NCAL_POINTS; i++) {
        m_sweep_freqs[i] = round(m_sweep_freqs[i] / LO_STEP) * LO_STEP;
    }
    for (size_t i = 0; i < NCAL_POINTS; i++) {
        if (CommandData.roach[m_roach->which - 1].do_cal_sweeps) {
                blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g",
                   m_sweep_freqs[i]/1.0e6);
            m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
            pi_write_string(m_pi, (unsigned char*)lo_command, strlen(lo_command));
            if (pi_read_string(&pi_state_table[m_roach->which - 1], PI_READ_NTRIES, LO_READ_TIMEOUT) < 0) {
                blast_info("Error setting LO... reboot Pi%d?", m_roach->which);
            }
            usleep(SWEEP_TIMEOUT);
            cal_get_mags(m_roach, (uint32_t)m_sweep_freqs[i], m_roach->num_kids,
                    sweep_buffer, i);
        } else {
            blast_info("Sweep interrupted by command");
            CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
            return SWEEP_INTERRUPT;
        }
    }
    usleep(SWEEP_TIMEOUT);
    if (recenter_lo(m_roach) < 0) {
        blast_info("Error recentering LO");
    } else {
    // TODO(Sam) Put in error handling
        if (pi_read_string(&pi_state_table[m_roach->which - 1], PI_READ_NTRIES, LO_READ_TIMEOUT) < 0) {
            blast_info("Error setting LO... reboot Pi%d?", m_roach->which);
        }
    }
    return SWEEP_SUCCESS;
}

int cal_sweep_attens(roach_state_t *m_roach)
{
    if (CommandData.roach[m_roach->which - 1].do_cal_sweeps) {
        double atten_step;
        uint32_t npoints, ncycles;
        atten_step = CommandData.roach_params[m_roach->which - 1].atten_step;
        ncycles = (uint32_t)CommandData.roach_params[m_roach->which - 1].ncycles;
        npoints = (uint32_t)CommandData.roach_params[m_roach->which - 1].npoints;
        blast_info("NCYCLES = %u", ncycles);
        blast_info("ATTEN STEP = %f", atten_step);
        blast_info("NPOINTS = %u", npoints);
        double start_atten = (double)OUTPUT_ATTEN_TARG - (ncycles*atten_step);
        int count = 0;
        char *subdir;
        blast_info("ROACH%d, running tone amp cal for %d cycles", m_roach->which, ncycles);
        while (count < ncycles + 1) {
            blast_info("Count = %d", count);
            blast_tmp_sprintf(subdir, "%4f", start_atten + (count*atten_step));
            int status = cal_sweep(m_roach, subdir);
            if (status == SWEEP_SUCCESS) {
                blast_info("ROACH%d, CAL sweep %d complete, stepping output atten", m_roach->which, count);
                if (set_output_atten(m_roach, start_atten + (count*atten_step)) < 0) {
                    blast_info("ROACH%d: Failed to set RUDATs...", m_roach->which);
                    free(m_roach->last_cal_path);
                    return SWEEP_FAIL;
                }
            } else if ((status == SWEEP_INTERRUPT)) {
                blast_info("ROACH%d, Cal sweep interrupted by blastcmd", m_roach->which);
                m_roach->status = ROACH_STATUS_ACQUIRING;
                m_roach->desired_status = ROACH_STATUS_ACQUIRING;
                CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
                free(m_roach->last_cal_path);
                return SWEEP_INTERRUPT;
            } else {
                blast_info("ROACH%d, Cal sweep failed, will reattempt", m_roach->which);
                m_roach->status = ROACH_STATUS_ACQUIRING;
                m_roach->desired_status = ROACH_STATUS_ACQUIRING;
                CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
                free(m_roach->last_cal_path);
                return SWEEP_FAIL;
            }
        count += 1;
        }
    }
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
    free(m_roach->last_cal_path);
    return SWEEP_SUCCESS;
}

// refit resonant frequency by doing a cal sweep and calling Python
void cal_fit_res(roach_state_t *m_roach, char *m_subdir)
{
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 1;
    CommandData.roach_params[m_roach->which - 1].ncycles = 1;
    CommandData.roach_params[m_roach->which - 1].npoints = 20;
    cal_sweep(m_roach, m_subdir);
    // fit res and rewrite tones
}

void chop_all(roach_state_t *m_roach)
{
}

// Load data saved by chop_all, and average together to create master chop
int master_chop(roach_state_t *m_roach, double m_nsec)
{
    int retval = -1;
    char *file_in;
    struct stat dir_stat;
    int stat_return;
    // check to see if directory exists
    stat_return = stat(m_roach->last_chop_path, &dir_stat);
    if (stat_return != 0) {
        blast_err("Could not find chop directory");
        return retval;
    } else {
        retval = 0;
    }
    int npoints = round(m_nsec * (double)DAC_FREQ_RES);
    int rows = m_roach->current_ntones;
    int cols = npoints;
    float *I[rows];
    float *Q[rows];
    for (int i = 0; i < rows; i++) {
         I[i] = (float *)malloc(cols * sizeof(float));
         Q[i] = (float *)malloc(cols * sizeof(float));
    }
    for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
        blast_tmp_sprintf(file_in, "%s/%zd.dat", m_roach->last_chop_path, chan);
        FILE *fd = fopen(file_in, "rb");
        for (int i = 0; i < npoints; i++) {
            // write a 4 byte I value
            fread(&I[chan][i], 4, 1, fd);
            // write a 4 byte value
            fread(&Q[chan][i], 4, 1, fd);
        }
        fclose(fd);
    }
    for (int i = 0; i < rows; i++) {
        free(I[i]);
        free(Q[i]);
    }
    CommandData.roach[m_roach->which - 1].do_master_chop = 0;
    return retval;
}

// tune tone amplitude according to chop snr (starting with single channel)
// m_nsec is the length of the timestream
void chop_tune(roach_state_t *m_roach, int m_chan, double m_nsec)
{
    if (CommandData.roach[m_roach->which - 1].tune_chan) {
        int ncycles = 3; // Move to this CommandData
        CommandData.roach_params[m_roach->which - 1].npoints = 21;
        double snr;
        double current_snr;
        // start with this delta_amp for now
        double delta_amps = DELTA_AMP;
        // chop parameters
        int num_pulse = 100;
        int separation = 200;
        int length = 200;
        // chop and save timstream
        chop_and_save(m_roach, m_chan, num_pulse, length, separation, m_nsec);
        // fit the snr
        chop_snr(m_roach, &snr);
        blast_info("ROACH%d, chop SNR = %g", m_roach->which, snr);
        // Start the cycle
        int count = 0;
        while (count < ncycles) {
            blast_info("count = %d", count);
            blast_info("ROACH%d, starting chop tune on chan %d", m_roach->which, m_chan);
            // increase tone amplitude by a small amount
            change_targ_amps(m_roach, &delta_amps, &m_chan, 1);
            roach_refit_freqs(m_roach);
            // chop and save again
            chop_and_save(m_roach, m_chan, num_pulse, length, separation, m_nsec);
            // fit the snr
            chop_snr(m_roach, &current_snr);
            blast_info("ROACH%d, new chop SNR = %g", m_roach->which, current_snr);
            // Compare new snr to old
            if (current_snr < snr) {
                // if current snr < snr, change delta direction
                blast_info("ROACH%d, changing delta amp sign", m_roach->which);
                delta_amps *= -1.0;
                change_targ_amps(m_roach, &delta_amps, &m_chan, 1);
                // if current snr > snr
            }
            snr = current_snr;
            count += 1;
        }
        CommandData.roach[m_roach->which - 1].tune_chan = 0;
    }
}

int write_opt_freqs(roach_state_t *m_roach, char *m_targ_path)
{
    if (!m_targ_path) {
        blast_info("Roach%d, NO TARG PATH FOUND, using TARG SEARCH PATH:%s instead",
                m_roach->which, targ_search_path);
        blast_tmp_sprintf(m_targ_path, targ_search_path);
    }
    char *calc_freqs_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    blast_tmp_sprintf(calc_freqs_command,
           "python /data/etc/blast/roachPython/calc_targ_freqs.py %s %g",
           m_targ_path,
           m_roach->lo_centerfreq);
    blast_info("%s", calc_freqs_command);
    system(calc_freqs_command);
    sleep(6);
    blast_tmp_sprintf(m_targ_freq_path, "%s/gradient_freqs.dat", m_targ_path);
    blast_info("Opening gradient freqs for reading...");
    FILE *fd;
    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return -1;
    }
    for (size_t m_index = 0; m_index < m_roach->num_kids; m_index++) {
        if (fscanf(fd, "%lg\n", &m_temp_freqs[m_index]) == EOF) break;
    }
    fclose(fd);
    /* while (m_roach->num_kids < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &m_temp_freqs[(m_roach->num_kids)++]) != EOF) {
    }*/
    /* for (size_t j = 0; j < m_roach->num_kids; j++) {
        blast_info("temp_freqs: %lg", m_temp_freqs[j]);
    } */
    blast_info("populating targ_tones...");
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j] - m_roach->lo_centerfreq;
        blast_info("%g", m_roach->targ_tones[j]);
    }
    blast_info("Uploading TARGET comb...");
    roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
    blast_info("ROACH%d, TARGET comb uploaded", m_roach->which);
    blast_info("ROACH%d, Calibrating ADC rms voltages...", m_roach->which);
    cal_adc_rms(m_roach, ADC_TARG_RMS_250, OUTPUT_ATTEN_TARG, ADC_CAL_NTRIES);
    return 0;
}

int cal_indiv_amps(roach_state_t *m_roach, int ncycles)
{
    int status;
    double atten_step = 3.0; // dB
    // Allocate memory for buffers
    double **sweep_buffer = (double **)malloc(NCAL_POINTS * sizeof(double *));
    for (int i = 0; i < NCAL_POINTS; i++)
         sweep_buffer[i] = (double *)malloc(m_roach->num_kids * sizeof(double));
    // double first_minima[m_roach->num_kids];
    double **minima = (double **)malloc(ncycles * sizeof(double *));
    for (int i = 0; i < ncycles; i++)
         minima[i] = (double *)malloc(m_roach->num_kids * sizeof(double));

    double first_diff[m_roach->num_kids];
    // Do a cal sweep and find minimum amp for each channel
    status = cal_sweep_amps(m_roach, sweep_buffer);
    for (int chan = 0; chan < m_roach->num_kids; chan++) {
        double min_amp = 0.0;
        // Find minimum amp for each channel
        for (int point = 0; point < NCAL_POINTS; point++) {
            double amp = sweep_buffer[point][chan];
            if (amp < min_amp) min_amp = amp;
        }
        minima[0][chan] = 20.0*log10(min_amp);
    }
    for (int chan = 0; chan < m_roach->num_kids; chan++) {
        blast_info("ROACH%d, chan%d amp = %g", m_roach->which, chan, minima[0][chan]);
    }
    // raise output attenuator by 3 dB
    blast_info("ROACH%d, increasing output atten", m_roach->which);
    if (set_output_atten(m_roach, OUTPUT_ATTEN_TARG + atten_step) < 0) {
        blast_info("ROACH%d: Failed to set RUDATs...", m_roach->which);
    }
    // do second sweep
    status = cal_sweep_amps(m_roach, sweep_buffer);
    for (int chan = 0; chan < m_roach->num_kids; chan++) {
        double min_amp = 0.0;
        // Find minimum amp for each channel
        for (int point = 0; point < NCAL_POINTS; point++) {
            double amp = sweep_buffer[point][chan];
            if (amp < min_amp) min_amp = amp;
        }
        minima[1][chan] = 20.0*log10(min_amp);
    }
    // Compare new minima to first minima
    for (int chan = 0; chan < m_roach->num_kids; chan++) {
        first_diff[chan] = minima[1][chan] - minima[0][chan];
        blast_info("ROACH%d, chan%d first diff = %g", m_roach->which, chan, first_diff[chan]);
    }
    return 0;
}

// TODO(Sam/Laura) Finish/test this function
/* Function: roach_check_retune
 * ----------------------------
 * Compares reference gradients to current values to determine
 * whether or not to rewrite the targ comb. 'Retune' if nflags > FLAG_THRESH
 *
 * @param m_roach roach state table
 *
 * returns: m_roach->retune_flag
*/
/* static int roach_check_retune(roach_state_t *m_roach) */

void roach_check_retune(roach_state_t *m_roach)
{
    // Retune if nflags exceeds nflags_threshold
    // TODO(Sam) determine threshold
    blast_info("ROACH%d, calculating DF...", m_roach->which);
    roach_calc_df(m_roach);
    blast_info("ROACH%d: Checking for retune...", m_roach->which);
    int nflags = 0;
    for (int chan = 0; chan < m_roach->num_kids; chan++) {
        if ((m_roach->df[chan] > DF_THRESHOLD)) {
            m_roach->retune_mask[chan] = 1;
            nflags++;
        } else { m_roach->retune_mask[chan] = 0;}
    }
    if (nflags > FLAG_THRESH) {
        m_roach->retune_flag = 1;
        blast_info("ROACH%d: %d above threshold, RETUNE RECOMMENDED", m_roach->which, nflags);
    }
}

void roach_auto_retune(int16_t ind)
{
    roach_state_t m_roach = roach_state_table[ind];
    roach_check_retune(&m_roach);
    if (m_roach.retune_flag) {
        for (int chan = 0; chan < m_roach.num_kids; ++chan) {
            if (m_roach.retune_mask[chan] > 0) {
                m_roach.targ_tones_LUT0[chan] += m_roach.df[chan];
                m_roach.targ_tones_LUT1[chan] += m_roach.df[chan];
            }
        }
        // blast_info("ROACH%d, Writing tone LUTs...", m_roach.which - 1);
        roach_write_tones(&m_roach, m_roach.targ_tones_LUT0, m_roach.num_kids);
        roach_write_tones(&m_roach, m_roach.targ_tones_LUT1, m_roach.num_kids);
    }
}

void roach_retune_counter(uint16_t ind, int retune_period)
{
    // blast_info("retune period = %d, count = %d", retune_period, roach_state_table[ind].retune_count);
    roach_state_table[ind].retune_count += 1;
    if (roach_state_table[ind].retune_count % retune_period == 0) {
        roach_state_table[ind].retune_count = 0;
        /*blast_info("ROACH%d retune period = %d, count = %d",
              roach_state_table[ind].which, retune_period,
              roach_state_table[ind].retune_count);*/
        // blast_info("STATUS = %d", m_roach.status);
        if (CommandData.roach[ind].auto_retune &&
                     roach_state_table[ind].status >= ROACH_STATUS_POP_LUTS) {
            roach_auto_retune(ind);
        }
    }
}

/** Phenom/Callback functions to upload firmware **/
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

/* Function: firmware_upload_connected
 * -----------------------------------
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

/*
 * Function: roach_upload_status
 * -----------------------------
 * Checks if firmware was uploaded by reading KATCP field ?fpgastatus
 *
 * @param m_roach roach state structuer
*/
int roach_upload_status(roach_state_t *m_roach)
{
    int success_val = send_rpc_katcl(m_roach->rpc_conn, 1000,
        KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?fpgastatus",
        KATCP_FLAG_LAST | KATCP_FLAG_STRING, "",
        NULL);
    if (success_val != KATCP_RESULT_OK) {
        return -1;
    } else {
        return 0;
    }
}

/*
 * Function: roach_upload_fpg
 * -----------------------------
 * Uploads a firmware image file to the Roach
 *
 * @param m_roach roach state structure
 * @param m_filename name of firmware file
*/
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
        blast_err("Could not request upload port for ROACH firmware on %s! retval = %i",
               m_roach->address, retval);
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
        if (roach_upload_status(m_roach) < 0);
            return 0;
        } else {
            return 1;
    }
}

/*
 * Function: shutdown_roaches
 * -----------------------------
 * Used to shutdown Roaches, close UDP socket
 *
*/
void shutdown_roaches(void)
{
    close(roach_sock_fd); // close roach UDP socket
    for (int i = 0; i < NUM_ROACHES; i++) {
        blast_info("Closing KATCP on ROACH%d", i + 1);
        if (roach_state_table[i].rpc_conn) {
            // roach_write_int(&roach_state_table[i], "tx_rst", 1, 0);
            // roach_read_int(&roach_state_table[i], "tx_rst");
            destroy_rpc_katcl(roach_state_table[i].rpc_conn);
        }
        if (pi_state_table[i].pi_comm) {
            remote_serial_shutdown(pi_state_table[i].pi_comm);
            pi_state_table[i].pi_comm = NULL;
        }
    }
}

/*
 * Function: roach_cmd_loop
 * -----------------------------
 * Command thread for a Roach[ind], called from mcp.c
 *
 * @param ind the roach index
*/
void *roach_cmd_loop(void* ind)
{
    // auto retune by default
    int status;
    int i = *((uint16_t*) ind);
    CommandData.roach[i].auto_retune = 1;
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
    pi_state_table[i].status = PI_STATUS_BOOT;
    pi_state_table[i].desired_status = PI_STATUS_INIT;
    rudat_state_table[i].status = RUDAT_STATUS_BOOT;
    rudat_state_table[i].desired_status = RUDAT_STATUS_HAS_ATTENS;
    valon_state_table[i].status = VALON_STATUS_BOOT;
    valon_state_table[i].desired_status = VALON_STATUS_HAS_FREQS;
    roach_state_table[i].status = ROACH_STATUS_BOOT;
    roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
    while (!shutdown_mcp) {
        // TODO(SAM/LAURA): Fix Roach 1/Add error handling
        // Check for new roach status commands
        if (CommandData.roach[i].refit_res_freqs) {
            roach_refit_freqs(&roach_state_table[i]);
        }
        if (CommandData.roach[i].get_timestream == 1) {
            blast_info("Save timestream called");
            save_timestream(&roach_state_table[i], CommandData.roach[i].chan,
                          CommandData.roach_params[i].num_sec);
        }
        if (CommandData.roach[i].get_timestream == 2) {
            blast_info("Saving all timestreams");
            save_all_timestreams(&roach_state_table[i], CommandData.roach_params[i].num_sec);
        }
        if (CommandData.roach[i].do_master_chop) {
            blast_info("Creating chop template");
            master_chop(&roach_state_table[i], CommandData.roach_params[i].num_sec);
        }
        if (CommandData.roach[i].tune_chan &&
                  roach_state_table[i].status >= ROACH_STATUS_ARRAY_FREQS) {
            blast_info("Chop_tune_chan called");
            chop_tune(&roach_state_table[i], CommandData.roach[i].chan,
                          CommandData.roach_params[i].num_sec);
        }
        if (CommandData.roach[i].roach_state) {
            roach_state_table[i].status = CommandData.roach[i].roach_new_state;
            roach_state_table[i].desired_status = CommandData.roach[i].roach_desired_state;
            blast_info("CHANGE STATE: %d, %d",
                    CommandData.roach[i].roach_new_state,
                    CommandData.roach[i].roach_desired_state);
            CommandData.roach[i].roach_state = 0;
        }
        /* if (CommandData.roach[i].do_sweeps == 0) {
            roach_state_table[i].status = ROACH_STATUS_STREAMING;
        }*/
        /* if (CommandData.roach[i].switch_period) {
            roach_write_int(&roach_state_table[i], "switch_period", CommandData.roach_params[i].period, 0);
        } */
        if (CommandData.roach[i].load_vna_amps && !CommandData.roach[i].do_sweeps) {
            if (roach_state_table[i].status >= ROACH_STATUS_CALIBRATED) {
                blast_info("ROACH%d, Generating search comb", i + 1);
                roach_vna_comb(&roach_state_table[i]);
                blast_info("%d", CommandData.roach[i].load_vna_amps);
                roach_write_tones(&roach_state_table[i], roach_state_table[i].vna_comb,
                                               roach_state_table[i].vna_comb_len);
                blast_info("ROACH%d, Search comb uploaded", i + 1);
                roach_state_table[i].status = ROACH_STATUS_TONE;
            } else {
                blast_info("ROACH%d, Cannot load vna amps", i + 1);
            }
        }
        if (CommandData.roach[i].load_targ_amps && !CommandData.roach[i].do_sweeps) {
            blast_info("%d", CommandData.roach[i].load_targ_amps);
            if (roach_state_table[i].status >= ROACH_STATUS_ARRAY_FREQS) {
                roach_write_tones(&roach_state_table[i], roach_state_table[i].targ_tones,
                                    roach_state_table[i].num_kids);
            roach_state_table[i].status = ROACH_STATUS_TARG;
            } else {
                CommandData.roach[i].load_targ_amps = 0;
                blast_info("ROACH%d, Cannot load targ amps", i + 1);
            }
        }
        if (CommandData.roach[i].test_tone) {
            blast_info("Roach%d: Writing test freq %g Hz", i + 1,
                               CommandData.roach_params[i].test_freq);
            roach_write_tones(&roach_state_table[i],
                           &CommandData.roach_params[i].test_freq, 1);
            CommandData.roach[i].test_tone = 0;
        }
        if (CommandData.roach[i].adc_rms) {
            get_adc_rms(&roach_state_table[i]);
            CommandData.roach[i].adc_rms = 0;
        }
        if (CommandData.roach[i].calibrate_adc) {
            cal_adc_rms(&roach_state_table[i], ADC_TARG_RMS_VNA, OUTPUT_ATTEN_VNA, ADC_CAL_NTRIES);
            CommandData.roach[i].calibrate_adc = 0;
        }
        if (CommandData.roach[i].change_state) {
            roach_state_table[i].status = CommandData.roach[i].new_state;
            roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
            CommandData.roach[i].change_state = 0;
        }
        if (CommandData.roach[i].do_cal_sweeps &&
                roach_state_table[i].status >= ROACH_STATUS_ARRAY_FREQS) {
            roach_state_table[i].desired_status = ROACH_STATUS_CAL_AMPS;
            blast_info("NCYCLES = %f", CommandData.roach_params[i].ncycles);
            blast_info("NPOINTS = %f", CommandData.roach_params[i].npoints);
            blast_info("ATTEN STEP = %f", CommandData.roach_params[i].atten_step);
        }
        if (CommandData.roach[i].find_kids) {
            if ((get_targ_freqs(&roach_state_table[i], roach_state_table[i].last_vna_path,
                         roach_state_table[i].last_targ_path)) < 0) {
                   blast_info("ROACH%d: Failed to find kids", i + 1);
               }
            CommandData.roach[i].find_kids = 0;
        }
        if (CommandData.roach[i].set_attens) {
            if (set_atten(&rudat_state_table[i]) < 0) {
                blast_info("ROACH%d: Failed to set RUDATs...", i + 1);
            } else {
                CommandData.roach[i].set_attens = 0;
            }
        }
        if (CommandData.roach[i].new_atten) {
            if (set_output_atten(&roach_state_table[i],
                   CommandData.roach_params[i].new_out_atten) < 0) {
                blast_info("ROACH%d: Failed to set RUDATs...", i + 1);
            } else {
                CommandData.roach[i].new_atten = 0;
            }
        }
        if ((CommandData.roach[i].opt_tones) &&
                             (roach_state_table[i].status >= ROACH_STATUS_TARG)) {
            if (write_opt_freqs(&roach_state_table[i], roach_state_table[i].last_targ_path)) {
                blast_info("ROACH%d: Opt tones success", i + 1);
            } else {
                 blast_info("ROACH%d: Failed to optimize target tones", i + 1);
            }
            CommandData.roach[i].opt_tones = 0;
        }
        if (pi_state_table[i].status == PI_STATUS_BOOT &&
                             pi_state_table[i].desired_status > PI_STATUS_BOOT) {
            blast_info("Initializing Pi%d ...", i + 1);
            pi_state_table[i].which = i + 1;
            pi_state_table[i].pi_comm = remote_serial_init(i, NC1_PORT);
            while (!pi_state_table[i].pi_comm->connected) {
                // blast_info("We can't connect to bb%d.", i+1);
                usleep(2000);
            }
            pi_state_table[i].status = PI_STATUS_INIT;
            blast_info("Pi%d initialized...", i + 1);
        }
        if ((pi_state_table[i].status == PI_STATUS_INIT) &&
                              (rudat_state_table[i].status == RUDAT_STATUS_BOOT)) {
            blast_info("Pi%d, attempting to set RUDATs...", i + 1);
            // TODO(Sam) Put in error handling
            if (set_atten(&rudat_state_table[i]) < 0) {
                blast_info("ROACH%d: Failed to set RUDATs...", i + 1);
            } else {
                blast_info("RUDAT%d Initialized", i + 1);
                rudat_state_table[i].status = RUDAT_STATUS_HAS_ATTENS;
            }
        }
        if ((pi_state_table[i].status == PI_STATUS_INIT) &&
                             (valon_state_table[i].status == VALON_STATUS_BOOT)) {
            if (init_valon(&roach_state_table[i]) < 0) {
                blast_info("ROACH%d: Failed to set Valon...", i + 1);
            } else {
                valon_state_table[i].status = VALON_STATUS_HAS_FREQS;
                blast_info("Finished initializing Valon%d...", i + 1);
            }
        }
        if (roach_state_table[i].status == ROACH_STATUS_BOOT &&
                               roach_state_table[i].desired_status > ROACH_STATUS_BOOT) {
            blast_info("Attempting to connect to %s", roach_state_table[i].address);
            roach_state_table[i].katcp_fd = net_connect(roach_state_table[i].address,
                                      0, NETC_VERBOSE_ERRORS | NETC_VERBOSE_STATS);
            blast_info("fd:%d ", roach_state_table[i].katcp_fd);
            roach_state_table[i].rpc_conn = create_katcl(roach_state_table[i].katcp_fd);
            if (roach_state_table[i].katcp_fd > 0) {
                blast_info("ROACH%d, KATCP up", i + 1);
                roach_state_table[i].status = ROACH_STATUS_CONNECTED;
            } else {
                blast_err("ROACH%d, KATCP connection error", i + 1);
                sleep(3);
            }
        }
        if (roach_state_table[i].status == ROACH_STATUS_CONNECTED &&
            roach_state_table[i].desired_status >= ROACH_STATUS_PROGRAMMED) {
            if (roach_upload_fpg(&roach_state_table[i], roach_fpg) == 0) {
                blast_info("ROACH%d, Firmware uploaded", i + 1);
                roach_state_table[i].status = ROACH_STATUS_PROGRAMMED;
                roach_state_table[i].desired_status = ROACH_STATUS_CONFIGURED;
            }
        }
        if (roach_state_table[i].status == ROACH_STATUS_PROGRAMMED &&
            roach_state_table[i].desired_status >= ROACH_STATUS_CONFIGURED) {
            blast_info("ROACH%d, Configuring software registers...", i + 1);
            roach_write_int(&roach_state_table[i], "switch_period", DEFAULT_SWITCH_PERIOD, 0);
            roach_write_int(&roach_state_table[i], "dds_shift", DDC_SHIFT, 0);
            // roach_read_int(&roach_state_table[i], "dds_shift");
            roach_write_int(&roach_state_table[i], "PFB_fft_shift", VNA_FFT_SHIFT, 0);
            // roach_read_int(&roach_state_table[i], "PFB_fft_shift");
            roach_write_int(&roach_state_table[i], "downsamp_sync_accum_len", accum_len, 0);
            // roach_read_int(&roach_state_table[i], "downsamp_sync_accum_len");
            roach_write_int(&roach_state_table[i], "GbE_tx_destip", dest_ip, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_destport", roach_state_table[i].dest_port, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_srcip", roach_state_table[i].src_ip, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_srcmac0", srcmac0[i], 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_srcmac1", srcmac1, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_destmac0", destmac0, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_destmac1", destmac1, 0);
            // load_fir(&roach_state_table[i], fir_coeffs);
            roach_state_table[i].status = ROACH_STATUS_CONFIGURED;
            roach_state_table[i].desired_status = ROACH_STATUS_CALIBRATED;
        }
        if (roach_state_table[i].status == ROACH_STATUS_CONFIGURED &&
                   roach_state_table[i].desired_status >= ROACH_STATUS_CALIBRATED) {
            roach_write_int(&roach_state_table[i], "dac_reset", 1, 0);
            blast_info("ROACH%d, Calibrating QDR RAM", i + 1);
            if (roach_qdr_cal(&roach_state_table[i]) < 0) {
                blast_info("ROACH%d, Calibration failed", i + 1);
            } else {
            blast_info("ROACH%d, Calibration complete", i + 1);
            roach_write_int(&roach_state_table[i], "GbE_tx_rst", 0, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_rst", 1, 0);
            roach_write_int(&roach_state_table[i], "GbE_tx_rst", 0, 0);
            roach_write_int(&roach_state_table[i], "GbE_pps_start", 1, 0);
            roach_state_table[i].status = ROACH_STATUS_CALIBRATED;
            roach_state_table[i].desired_status = ROACH_STATUS_TONE;
            }
        }
        if (roach_state_table[i].status == ROACH_STATUS_CALIBRATED &&
            roach_state_table[i].desired_status >= ROACH_STATUS_TONE) {
            blast_info("ROACH%d, Generating search comb...", i + 1);
            blast_info("ROACH%d, Creating VNA Comb", i + 1);
            roach_vna_comb(&roach_state_table[i]);
            blast_info("ROACH%d, VNA comb created", i + 1);
            blast_info("ROACH%d, Writing tones", i + 1);
            // CommandData.roach[i].load_vna_amps = 1;
            roach_write_tones(&roach_state_table[i], roach_state_table[i].vna_comb,
                                              roach_state_table[i].vna_comb_len);
            blast_info("ROACH%d, Search comb uploaded", i + 1);
            roach_state_table[i].status = ROACH_STATUS_TONE;
            roach_state_table[i].desired_status = ROACH_STATUS_STREAMING;
        }
        if (roach_state_table[i].status == ROACH_STATUS_TONE &&
               roach_state_table[i].desired_status >= ROACH_STATUS_STREAMING) {
            if (i < 1) {
                blast_info("ROACH%d, Calibrating ADC rms voltages...", i + 1);
                cal_adc_rms(&roach_state_table[i], ADC_TARG_RMS_VNA, OUTPUT_ATTEN_VNA, ADC_CAL_NTRIES);
            }
            blast_info("ROACH%d, Checking stream status...", i + 1);
            if (roach_check_streaming(&roach_state_table[i],
                                       STREAM_NTRIES, STREAM_TIMEOUT) < 0) {
                blast_err("ROACH%d data streaming error. Reboot Roach?", i + 1);
            } else {
                blast_info("ROACH%d, streaming SUCCESS", i + 1);
                roach_state_table[i].status = ROACH_STATUS_STREAMING;
                roach_state_table[i].desired_status = ROACH_STATUS_VNA;
            }
        }
        if (roach_state_table[i].status == ROACH_STATUS_STREAMING &&
            roach_state_table[i].desired_status >= ROACH_STATUS_VNA) {
            roach_write_int(&roach_state_table[i], "PFB_fft_shift", VNA_FFT_SHIFT, 0);
            usleep(3000);
            blast_info("ROACH%d, Initializing VNA sweep", i + 1);
            blast_info("ROACH%d, disabling LUT switch", i + 1);
            enable_qdr_switch(&roach_state_table[i], 0);
            blast_info("ROACH%d, Starting VNA sweep...", i + 1);
            status = roach_do_sweep(&roach_state_table[i], VNA);
            if ((status == SWEEP_SUCCESS)) {
                blast_info("ROACH%d, VNA sweep complete", i + 1);
                system("python /home/fc1user/sam_builds/sweep_list.py vna");
                roach_state_table[i].status = ROACH_STATUS_VNA;
                roach_state_table[i].desired_status = ROACH_STATUS_ARRAY_FREQS;
            } else if ((status == SWEEP_INTERRUPT)) {
                blast_info("ROACH%d, VNA sweep interrupted by blastcmd", i + 1);
                roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
                roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
            } else { blast_info("ROACH%d, VNA sweep failed, will reattempt", i + 1);}
        }
        if (roach_state_table[i].status == ROACH_STATUS_VNA &&
              roach_state_table[i].desired_status >= ROACH_STATUS_ARRAY_FREQS) {
              if (get_targ_freqs(&roach_state_table[i], roach_state_table[i].last_vna_path,
                                    roach_state_table[i].last_targ_path) < 0) {
                  blast_info("ROACH%d, Error finding TARG freqs", i + 1);
              }
              /* if (get_targ_freqs(&roach_state_table[i], vna_search_path,
                                   roach_state_table[i].last_targ_path) < 0) {
                  blast_info("ROACH%d, Error finding TARG freqs", i + 1);
              }*/
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
            blast_info("ROACH%d, STARTING TARG sweep", i + 1);
            status = roach_do_sweep(&roach_state_table[i], TARG);
            if ((status == SWEEP_SUCCESS)) {
                /* if (roach_check_streaming(&roach_state_table[i],
                           STREAM_NTRIES, STREAM_TIMEOUT) < 0) {
                    blast_err("ROACH%d data streaming error. Reboot Roach?", i + 1);
                }*/
                blast_info("ROACH%d, TARG sweep complete", i + 1);
                // creates file for downlinking sweep path to local machine
                system("python /home/fc1user/sam_builds/sweep_list.py targ");
                roach_state_table[i].status = ROACH_STATUS_TARG;
                roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
            } else if ((status == SWEEP_INTERRUPT)) {
                blast_info("ROACH%d, TARG sweep interrupted by blastcmd", i + 1);
                roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
                roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
            } else { blast_info("ROACH%d, TARG sweep failed, will reattempt", i + 1);}
        }
        if (roach_state_table[i].status == ROACH_STATUS_TARG &&
              roach_state_table[i].desired_status >= ROACH_STATUS_POP_LUTS) {
              if (calc_grad_freqs(&roach_state_table[i], targ_search_path) < 0) {
                  blast_info("ROACH%d, Error finding GRAD freqs", i + 1);
              }
            blast_info("ROACH%d, Writing tone LUTs...", i + 1);
            roach_write_tones(&roach_state_table[i], roach_state_table[i].targ_tones_LUT0,
                                                  roach_state_table[i].num_kids);
            roach_write_tones(&roach_state_table[i], roach_state_table[i].targ_tones_LUT1,
                                                  roach_state_table[i].num_kids);
            blast_info("ROACH%d, calculating ref grads and DF...", i + 1);
            roach_calc_df(&roach_state_table[i]);
            roach_state_table[i].status = ROACH_STATUS_POP_LUTS;
            blast_info("ROACH%d, enabling LUT switch", i + 1);
            enable_qdr_switch(&roach_state_table[i], 1);
            roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
        }
        /* Do a series of small (~6 point) sweeps, check minimum amplitude of each
        sweep. If minimum is lower than previous, rewrite that tone amp and iterate
        until break condition */
        if (roach_state_table[i].status >= ROACH_STATUS_ARRAY_FREQS &&
              roach_state_table[i].desired_status == ROACH_STATUS_CAL_AMPS) {
            blast_info("ROACH%d, Starting CAL sweeps...", i + 1);
            status = cal_sweep_attens(&roach_state_table[i]);
            if ((status == SWEEP_SUCCESS)) {
                // Save the paths to last sweeps for data analysis
                system("python /home/fc1user/sam_builds/sweep_list.py cal");
                blast_info("ROACH%d, CAL sweeps complete", i + 1);
                roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
                roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
            } else if ((status == SWEEP_INTERRUPT)) {
                blast_info("ROACH%d, Cal sweep interrupted by blastcmd", i + 1);
                roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
                roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
            } else {
                blast_info("ROACH%d, Cal sweep failed, will reattempt", i + 1);
                roach_state_table[i].desired_status = ROACH_STATUS_ACQUIRING;
                roach_state_table[i].status = ROACH_STATUS_ACQUIRING;
            }
        }
        if (roach_state_table[i].status == ROACH_STATUS_POP_LUTS &&
              roach_state_table[i].desired_status >= ROACH_STATUS_ACQUIRING) {
        blast_info("ROACH%d, status = ACQUIRING", i + 1);
        }
        usleep(1000); // prevents mcp from eating up all the CPU.
    }
    return NULL;
}

/*
 * Function: init_roach
 * -----------------------------
 * Configures default parameters for Roach[ind]
 *
 * @param ind the roach index
*/
int init_roach(uint16_t ind)
{
    if (ind >= NUM_ROACHES) {
        blast_err("Attempted to intialize a non-existent roach #%u", ind + 1);
        return -1;
    }
    memset(&roach_state_table[ind], 0, sizeof(roach_state_t));
    memset(&pi_state_table[ind], 0, sizeof(pi_state_t));
    memset(&rudat_state_table[ind], 0, sizeof(rudat_state_t));
    memset(&valon_state_table[ind], 0, sizeof(valon_state_t));
    asprintf(&roach_state_table[ind].address, "roach%d", ind + 1);
    asprintf(&roach_state_table[ind].vna_path_root,
                      "/home/fc1user/sam_tests/sweeps/roach%d/vna", ind + 1);
    asprintf(&roach_state_table[ind].targ_path_root,
                      "/home/fc1user/sam_tests/sweeps/roach%d/targ", ind + 1);
    asprintf(&roach_state_table[ind].cal_path_root,
                      "/home/fc1user/sam_tests/sweeps/roach%d/cal", ind + 1);
    asprintf(&roach_state_table[ind].vna_amps_path[1],
                      "/home/fc1user/sam_tests/sweeps/roach%d/vna_trf.dat", ind + 1);
    asprintf(&roach_state_table[ind].vna_amps_path[0],
                      "/home/fc1user/sam_tests/roach%d_default_amps.dat", ind + 1);
    asprintf(&roach_state_table[ind].targ_amps_path[0],
                      "/home/fc1user/sam_tests/roach%d_default_targ_amps.dat", ind + 1);
    asprintf(&roach_state_table[ind].targ_amps_path[1],
                      "/home/fc1user/sam_tests/sweeps/roach%d/first_targ_trf.dat", ind + 1);
    asprintf(&roach_state_table[ind].targ_amps_path[2],
                      "/home/fc1user/sam_tests/sweeps/roach%d/last_targ_amps.dat", ind + 1);
    asprintf(&roach_state_table[ind].chop_path_root,
                      "/home/fc1user/sam_tests/sweeps/roach%d/chops", ind + 1);
    asprintf(&roach_state_table[ind].qdr_log,
                      "/home/fc1user/sam_tests/roach%d_qdr_cal.log", ind + 1);
    asprintf(&roach_state_table[ind].find_kids_log,
                      "/home/fc1user/sam_tests/roach%d_find_kids.log", ind + 1);
    asprintf(&roach_state_table[ind].opt_tones_log,
                      "/home/fc1user/sam_tests/roach%d_opt_tones.log", ind + 1);
    asprintf(&roach_state_table[ind].random_phase_path,
                      "/home/fc1user/sam_tests/random_phases.dat");
    asprintf(&roach_state_table[ind].phase_centers_path,
                      "/home/fc1user/sam_tests/sweeps/roach%d/phase_centers.dat", ind + 1);
    roach_state_table[ind].retune_count = 0;
    if ((ind == 0)) {
        roach_state_table[ind].lut_idx = 0;
        roach_state_table[ind].lo_centerfreq = 850.0e6;
        roach_state_table[ind].vna_comb_len = 1000;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 1)) {
        roach_state_table[ind].lut_idx = 0;
        roach_state_table[ind].lo_centerfreq = 828.0e6;
        roach_state_table[ind].vna_comb_len = 1000;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 2)) {
        roach_state_table[ind].lut_idx = 0;
        roach_state_table[ind].lo_centerfreq = 828.0e6;
        roach_state_table[ind].vna_comb_len = 1000;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 3)) {
        roach_state_table[ind].lut_idx = 0;
        roach_state_table[ind].lo_centerfreq = 828.0e6;
        roach_state_table[ind].vna_comb_len = 1000;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 4)) {
        roach_state_table[ind].lut_idx = 0;
        roach_state_table[ind].lo_centerfreq = 828.0e6;
        roach_state_table[ind].vna_comb_len = 1000;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    roach_state_table[ind].which = ind + 1;
    pi_state_table[ind].which = ind + 1;
    rudat_state_table[ind].which = ind + 1;
    valon_state_table[ind].which = ind + 1;
    roach_state_table[ind].dest_port = 64000 + ind;
    roach_state_table[ind].src_ip = IPv4(192, 168, 40, 71 + ind);
    ph_thread_spawn((ph_thread_func)roach_cmd_loop, (void*) &ind);
    blast_info("Spawned command thread for roach%i", ind + 1);
    return 0;
}

/* Function: write_roach_channels_5hz
 * ----------------------------------
 * Populates 5 Hz frame data
 */
void write_roach_channels_5hz(void)
{
    int i, j;
    static int firsttime = 1;
    int n_write_kids_df = 20; // For now only write the delta_freqs for the first 20 KIDs.
    static channel_t *RoachPktCtAddr[NUM_ROACHES];
    static channel_t *RoachValidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachInvalidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachStatusAddr[NUM_ROACHES];
    static channel_t *PiStatusAddr[NUM_ROACHES];
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
    char channel_name_pi_status[128] = { 0 };
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
            snprintf(channel_name_pi_status,
                    sizeof(channel_name_pi_status), "status_pi_roach%d", i + 1);
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
            PiStatusAddr[i] = channels_find_by_name(channel_name_pi_status);
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
        SET_UINT16(PiStatusAddr[i], pi_state_table[i].status);
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
