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

/* MCP = Master Control Program (Blast) */
/* PPC = PowerPC */
/* FW = Firmware */
/* FC1 = Flight computer 1 */
/* FC2 = Flight computer 2 */
/* Pi = Raspberry Pi */
/* Baseband = -256 to +256 MHz */
/* I = In phase. The cos (real) part of the waveform */
/* Q = Quadrature. The sine (imaginary) part of the waveform */

#define DDC_SHIFT 318 /* FW version dependent, see roach_fpg */
#define VNA_FFT_SHIFT 127 /*Controls FW FFT overflow behavior, for VNA SWEEP */
#define TARG_FFT_SHIFT 127
#define VNA 0 /* Sweep type */
#define TARG 1 /* Sweep type */
#define IQ 2
#define DF 3
#define LAMP 4
#define NOISECOMP 5
#define BB_FREQS 6
#define DS_VNA 7
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
#define DEFAULT_TARG_SWEEP_SPAN 175.0e3 /* Target sweep span */
#define NTAPS 47 /* 1 + Number of FW FIR coefficients */
#define N_AVG 30 /* Number of packets to average for each sweep point */
#define N_AVG_DF 50 /* Number of packets to average for DF calculation */
#define SWEEP_INTERRUPT (-1)
#define SWEEP_SUCCESS (0)
#define SWEEP_FAIL (-2)
#define SWEEP_TIMEOUT 3000 /* microsecond timeout between set LO and save data */
#define PI_READ_ERROR -10 /* Error code: Pi read */
#define READ_LINE 256 /* Line length for buffer reads, bytes */
#define READ_BUFFER 4096 /* Number of bytes to read from a buffer */
#define STREAM_NTRIES 100000 /* Number of times to check stream status */
#define STREAM_TIMEOUT 2 /* Seconds to wait between checks */
#define PI_READ_NTRIES 50 /* Number of times to attempt Pi read */
#define PI_READ_TIMEOUT (800*1000) /* Pi read timeout, usec */
#define LO_READ_TIMEOUT (800*1000) /* LO read timeout, usec */
#define INIT_VALON_TIMEOUT (500*1000) /* Valon init timeout, usec */
#define ADC_CAL_NTRIES 30
#define ADC_TARG_RMS_VNA 100 /* mV, Same for all Roaches */
#define ADC_TARG_RMS_250 100 /* mV, For 250 micron array TARG sweep */
#define ADC_RMS_RANGE 5 /* mV */
#define ATTEN_STEP 2. /* dB */
#define OUTPUT_ATTEN_VNA 1 /* dB, for VNA sweep */
#define OUTPUT_ATTEN_TARG 20 /* dB, initial level for TARG sweep */
#define READ_DATA_MS_TIMEOUT 10000 /* ms, timeout for roach_read_data */
#define EXT_REF 1 /* Valon external ref (10 MHz) */
#define NCAL_POINTS 200 /* Number of points to use for cal sweep */
#define N_CAL_CYCLES 3 /* Number of cal cycles for tone amplitudes */
#define START_AMP 0.90 /* Starting (digital) amplitude for carrier tones */
#define DELTA_AMP 0.02 /* A change in tone amplitude (used for linearity calibration */
#define AUTO_CAL_ADC 0 /* Choose to run the adc cal routine after initial tone write */
#define AUTO_CAL_AMPS 0
#define APPLY_VNA_TRF 1 /* Apply Roach output transfer function to vna freqs by default */
#define APPLY_TARG_TRF 1 /* Apply Roach output transfer function to targ freqs by default */
#define ATTEN_PORT 9998 /* Pi port for atten socket */
#define VALON_PORT 9999 /* Pi port for valon socket */
#define PI_TEMP_PORT 9997 /* Pi port for reading temp */
#define ROACH_WATCHDOG_PERIOD 5 /* second period to check PPC connection */
#define N_WATCHDOG_FAILS 5 /* Number of check fails before state is reset to boot */
#define MAX_PI_ERRORS_REBOOT 10 /* If there are 10 consecutive Pi errors, reboot */
#define VNA_COMB_LEN 1000 /* Number of tones in search comb */
#define ATTEN_TOTAL 7 /* In atten (dB) + out atten (dB). Number is conserved */
#define DEFAULT_OUTPUT_ATTEN 7 /* dB */
#define DEFAULT_INPUT_ATTEN 0 /* dB */
#define SWEEP_READY_TIMEOUT 5000 /* ms, time for Roaches to wait before saving data */
#define MAX_DATA_ERRORS 1000 /* Max num allowable data errors in save_sweep_packet (ms) */
#define MAX_LAMP_WAITS 5000 /* Max num times to wait for lead roach to flash lamp (1 s) */
#define MAX_FPG_UPLOAD_TRIES 10 /* Max number of fpg upload attempts */
#define MAX_PACKET_COUNT_ERRORS 5000 /* Max num packet count errors for stream checker */
#define MAX_FULL_LOOP_TRIES 2 /* Max num full loop retries after fail */

static int is_cycling = 0;
extern int16_t InCharge; /* See mcp.c */
extern int roach_sock_fd; /* File descriptor for shared Roach UDP socket */
static int fft_len = 1024; /* Order of FW FFT */
static uint32_t accum_len = (1 << 19) - 1; /* Number of FW FFT accumulations */

static double rf_pow_out[1000];
char rf_tone_power_path[100] = "/home/fc1user/roach_flight/rfPowerOut.dat";

char path_to_vna_tarball[5][100];
char path_to_targ_tarball[5][100];
char path_to_iq_tarball[5][100];
char path_to_df_tarball[5][100];
char path_to_lamp_tarball[5][100];
char path_to_dsvna[5][100];

char path_to_all_vna[] = "/home/fc1user/roach_flight/all_vna_sweeps.tar.gz";
char path_to_all_targ[] = "/home/fc1user/roach_flight/all_targ_sweeps.tar.gz";
char path_to_all_iq[] = "/home/fc1user/roach_flight/all_iq_data.tar.gz";
char path_to_all_df[] = "/home/fc1user/roach_flight/all_df_data.tar.gz";
char path_to_all_lamp[] = "/home/fc1user/roach_flight/all_lamp_data.tar.gz";
char path_to_all_noise_comp[] = "/home/fc1user/roach_flight/all_noise_comp.tar.gz";
char path_to_all_df_comp[] = "/home/fc1user/roach_flight/all_df_comp.tar.gz";
char path_to_all_bb_freqs[] = "/home/fc1user/roach_flight/all_bb_freqs.tar.gz";
char path_to_all_dsvna[] = "/home/fc1user/roach_flight/all_dsvna.tar.gz";

// Roach source MAC addresses
const char src_macs[5][100] = {"024402020b03", "024402020d17", "024402020D16", "02440202110c", "024402020D21"};
uint32_t srcmac0[5] = {33688323, 33688855, 33688854, 33689868, 33688865};
uint32_t srcmac1 = 580;
double test_freq[] = {10.0125e6};

// UDP destination MAC addresses

// MULTICAST
uint32_t destmac0 = 1577124330;
uint32_t destmac1 = 256;

static uint32_t dest_ip = IPv4(239, 1, 1, 234);

// root path for saving all roach data
char roach_root_path[] = "/home/fc1user/roach_flight";

const char roach_fpg[] = "/data/etc/blast/roachFirmware/stable_ctime_v6_2018_Feb_19_1053.fpg";
// const char roach_fpg[] = "/data/etc/blast/roachFirmware/longerfirs_2018_Apr_18_1905.fpg";

/* Roach2 state structure, see roach.h */
roach_state_t roach_state_table[NUM_ROACHES]; /* NUM_ROACHES = 5 */
/* Pi state structure, see roach.h */
static pi_state_t pi_state_table[NUM_ROACHES];
/* Initialization scripts that live on Pi */
char valon_init_pi[] = "python /home/pi/device_control/init_valon.py";
char valon_init_pi_extref[] = "python /home/pi/device_control/init_valon_ext.py";
char read_valon_pi[] = "python /home/pi/device_control/read_valon.py";

char find_kids_script[] = "/data/etc/blast/roachPython/findKidsMcp.py";
char find_kids_default_script[] = "/data/etc/blast/roachPython/findKidsDefault.py";
char center_phase_script[] = "/data/etc/blast/roachPython/center_phase.py";
char chop_snr_script[] = "/data/etc/blast/roachPython/fit_mcp_chop.py";
char refit_freqs_script[] = "/data/etc/blast/roachPython/fit_res.py";
char cal_amps_script[] = "/data/etc/blast/roachPython/nonlinearParamMcp.py";
char ref_grads_script[] = "/data/etc/blast/roachPython/saveRefparams.py";
char gen_output_trf_script[] = "/data/etc/blast/roachPython/gen_output_trf_mcp.py";
char df_from_sweeps_script[] = "/data/etc/blast/roachPython/dfSweeps.py";
char median_df_script[] = "/data/etc/blast/roachPython/median_df.py";
char noise_comp_script[] = "/data/etc/blast/roachPython/noise_comp.py";
char df_comp_script[] = "/data/etc/blast/roachPython/df_comp.py";

char rudat_input_serials[5][100] = {"11505170016", "11505170014", "11505170022", "11505170009", "11508260120"};
char rudat_output_serials[5][100] = {"11505170019", "11505170023", "11505170003", "11505170021", "11508260127"};

static pthread_mutex_t fft_mutex; /* Controls access to the fftw3 */

void nameThread(const char*);

// Roach with lamp control sets this high to alert other Roaches that it's ready to flash cal lamp
static int lead_roach_ready;
static int roach_lamp_now; // indicates Roach is turning on lamp with periodic_cal()

static char null_string[] = "";

// Load list of initial RF tone powers
int load_tone_powers()
{
    int retval = -1;
    FILE *fd = fopen(rf_tone_power_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", rf_tone_power_path);
         for (size_t i = 0; i < 1000; i++) {
            rf_pow_out[i] = -35.0;
            // blast_info("%f", rf_pow_out[i]);
        }
        return retval;
    } else {
        blast_info("Loading RF tone powers");
        for (size_t i = 0; i < 1000; i++) {
            if (fscanf(fd, "%lg\n", &rf_pow_out[i]) != EOF) {
                // blast_info("%f", rf_pow_out[i]);
            } else {
                retval = 0;
                break;
            }
        }
        fclose(fd);
    }
    return retval;
}

// Generic function to handle system calls for python scripts with additional niceness.
void pyblast_system(char* cmd)
{
    char* new_cmd;
//    blast_tmp_sprintf(new_cmd, "%s",
//                      cmd);
     blast_tmp_sprintf(new_cmd, "nice -11 %s",
                       cmd);
    system(new_cmd);
}

int get_roach_state(uint16_t ind)
{
    int state = roach_state_table[ind].state;
    return state;
}

// read in a 1D list of type float or double
int roach_read_1D_file(roach_state_t *m_roach, char *m_file_path, double *m_buffer, size_t m_freqlen)
{
    int retval = -1;
    // blast_info("ROACH%d, file = %s", m_roach->which, m_file_path);
    FILE *fd = fopen(m_file_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_file_path);
        return retval;
    } else {
        blast_info("ROACH%d, Opened %s", m_roach->which, m_file_path);
        for (size_t i = 0; i < m_freqlen; i++) {
            if (fscanf(fd, "%lg\n", &m_buffer[i]) != EOF) {
                // blast_info("Roach%d loaded vals: %g", m_roach->which, m_buffer[i]);
            } else {
                break;
            }
        }
        fclose(fd);
    }
    // blast_info("READ FILE RETVAL ======= %d", retval);
    return 0;
}

int roach_read_2D_file(roach_state_t *m_roach, char *m_file_path, double (*m_buffer)[2], size_t m_freqlen)
{
    int retval = -1;
    blast_info("ROACH%d, file = %s", m_roach->which, m_file_path);
    FILE *fd = fopen(m_file_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_file_path);
        return retval;
    } else {
        blast_info("Opened %s", m_file_path);
        for (size_t i = 0; i < m_freqlen; i++) {
            if (fscanf(fd, "%lg %lg", &m_buffer[i][0], &m_buffer[i][1]) != EOF) {
                // blast_info("Roach%d loaded vals: %g", m_roach->which, m_buffer[i]);
            } else {
                break;
            }
        }
        fclose(fd);
    }
    blast_info("ROACH%d, Saved ref params", m_roach->which);
    retval = 0;
    return retval;
}

/* Function: load_fir
 * ----------------------------
 * Programs the coefficients for the FW FIR filter
 * Operates on software registers
 * @param m_roach a roach state structure
 * @param m_coeff FIR filter coefficients
*/

/* int load_fir(roach_state_t *m_roach)
{
    int retval = -1;
    double taps[NTAPS];
    blast_info("Roach%d, Loading FIR taps", m_roach->which);
    // If can't load file, use random phases
    if ((roach_read_1D_file(m_roach, fir_taps_path, taps, NTAPS) < 0)) {
        return retval;
    }
    char *reg;
    for (int i = 0; i < NTAPS; ++i) {
        blast_info("ROACH%d, FIR tap = %g", m_roach->which, taps[i]);
        taps[i] *= ((pow(2, 31) - 1));
        blast_tmp_sprintf(reg, "FIR_h%d", i);
        roach_write_int(m_roach, reg, (int32_t)taps[i], 0);
    }
    blast_info("ROACH%d, Loaded FIR taps", m_roach->which);
    retval = 0;
    return retval;
}*/

void timestamp(roach_state_t *m_roach)
{
    char *py_command;
    char ts[READ_LINE];
    blast_tmp_sprintf(py_command,
         "python /data/etc/blast/roachPython/timestamp.py > %s", m_roach->ts);
    pyblast_system(py_command);
    FILE *fd = fopen(m_roach->ts, "r");
    if (!fd) {
        blast_err("Error opening ts file");
        return;
    }
    if (!fgets(ts, sizeof(ts), fd)) {
        fclose(fd);
        return;
    }
    blast_info("ROACH%d: ts = %s", m_roach->which, ts);
    if ((roach_write_int(m_roach, "GbE_ctime", atoi(ts), 0)) < 0) {
        blast_warn("ROACH%d: Timestamp write error", m_roach->which);
    }
    fclose(fd);
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
    int retval = -1;
    char *m_cal_command;
    char m_line[READ_LINE];
    char *freq;
    blast_tmp_sprintf(m_cal_command, "python /data/etc/blast/roachPython/cal_roach_qdr.py %s > %s",
                                    m_roach->address, m_roach->qdr_log);
    blast_info("ROACH%d, Calibrating QDR RAM", m_roach->which);
    pyblast_system(m_cal_command);
    sleep(5);
    FILE *fd = fopen(m_roach->qdr_log, "r");
    if (!fd) {
        blast_err("Error opening QDR log file");
        return retval;
    }
    if (!fgets(m_line, sizeof(m_line), fd)) {
        blast_err("Error reading QDR log file");
        blast_err("ROACH%d, Calibration failed", m_roach->which);
        return retval;
    } else {
        blast_info("%s", m_line);
        freq = strchr(m_line, '=');
        freq += 2; // Remove equals sign and space from string
    }
    fclose(fd);
    // double clock_freq = atof(freq);
    m_roach->fpga_clock_freq = atof(freq);
    if ((m_roach->fpga_clock_freq <= 255.0) || (m_roach->fpga_clock_freq >= 259.0)) {
        blast_info("ROACH%d FPGA CLOCK NOT SET: Reinitialize Valon%d ?",
                   m_roach->which, m_roach->which);
        blast_info("ROACH%d, Calibration failed", m_roach->which);
        m_roach->has_qdr_cal = 0;
        m_roach->qdr_fail = 1;
        return retval;
    }
    m_roach->has_qdr_cal = 1;
    m_roach->qdr_fail = 0;
    blast_info("ROACH%d, Calibration complete", m_roach->which);
    return 0;
}

/* Function: roach_buffer_htons
 * ----------------------------
 * Converts a buffer of type short from little-endian to big-endian
 *
 * @param m_buffer A buffer of type short
 * @param m_len length of m_buffer
*/
static void roach_buffer_htons(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = htons(m_buffer[i]);
    }
}

/* Function: roach_buffer_htonl
 * ----------------------------
 * Converts a buffer of type long from little-endian to big-endian
 *
 * @param m_buffer A buffer of type long
 * @param m_len length of m_buffer
*/
static void roach_buffer_htonl(uint32_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = htonl(m_buffer[i]);
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
    int retval;
    m_roach->katcp_is_busy = 1;
    uint32_t sendval = htonl(m_val);
    retval = roach_write_data(m_roach, m_register, (uint8_t*)&sendval,
                         sizeof(sendval), m_offset, WRITE_INT_TIMEOUT);
    m_roach->katcp_is_busy = 0;
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
    if (m_roach->LUT.Ival) free(m_roach->LUT.Ival);
    if (m_roach->LUT.Qval) free(m_roach->LUT.Qval);
    m_roach->LUT.Ival = calloc(m_len, sizeof(uint16_t));
    m_roach->LUT.Qval = calloc(m_len, sizeof(uint16_t));
    m_roach->LUT.len = m_len;
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
    if (m_roach->DAC.Ival) free(m_roach->DAC.Ival);
    if (m_roach->DAC.Qval) free(m_roach->DAC.Qval);
    m_roach->DAC.Ival = calloc(m_len, sizeof(double));
    m_roach->DAC.Qval = calloc(m_len, sizeof(double));
    m_roach->DAC.len = m_len;

    if (m_roach->DDC.Ival) free(m_roach->DDC.Ival);
    if (m_roach->DDC.Qval) free(m_roach->DDC.Qval);
    m_roach->DDC.Ival = calloc(m_len, sizeof(double));
    m_roach->DDC.Qval = calloc(m_len, sizeof(double));
    m_roach->DDC.len = m_len;
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
    int retval = -1;
    if (m_freqlen > 1000) {
        return retval;
    }
    size_t comb_fft_len;
    double amps[m_freqlen];
    double phases[m_freqlen];
    for (size_t i = 0; i < m_freqlen; i++) {
        amps[i] = 1.0;
    }
    if (!m_roach->last_amps) {
        m_roach->last_amps = calloc(m_freqlen, sizeof(double));
    }
    if (!m_roach->last_phases) {
        m_roach->last_phases = calloc(m_freqlen, sizeof(double));
    }
    if (!m_roach->last_freqs) {
        m_roach->last_freqs = calloc(m_freqlen, sizeof(double));
    }
    // Load random phases file
    blast_info("Roach%d, Loading tone phases", m_roach->which);
    // If can't load file, use random phases
    if ((roach_read_1D_file(m_roach, m_roach->random_phase_path, phases, m_freqlen)) < 0) {
        srand48(time(NULL));
        for (size_t i = 0; i < m_freqlen; i++) {
            phases[i] = drand48() * 2.0 * M_PI;
        }
    }
    if (CommandData.roach[m_roach->which - 1].change_tone_phase) {
        for (size_t i = 0; i < m_freqlen; i++) {
            phases[i] = m_roach->last_phases[i];
            blast_info("phases = %g", m_roach->last_phases[i]);
        }
    } else {
        for (size_t i = 0; i < m_freqlen; i++) {
            m_roach->last_phases[i] = phases[i];
        }
    }
    if (CommandData.roach[m_roach->which - 1].change_tone_freq) {
        for (size_t i = 0; i < m_freqlen; i++) {
            m_freqs[i] = m_roach->last_freqs[i];
            blast_info("freqs = %g", m_roach->last_freqs[i]);
        }
    } else {
        for (size_t i = 0; i < m_freqlen; i++) {
            m_roach->last_freqs[i] = m_freqs[i];
        }
    }
    if (CommandData.roach[m_roach->which - 1].load_vna_amps) {
        blast_info("Roach%d, Loading VNA AMPS", m_roach->which);
        // If can't load file, use all 1
        char *amps_path = m_roach->vna_amps_path[CommandData.roach[m_roach->which - 1].load_vna_amps - 1];
        if ((roach_read_1D_file(m_roach, amps_path, amps, m_freqlen) < 0)) {
            for (size_t i = 0; i < m_freqlen; i++) {
                amps[i] = 1.0;
            }
        }
        CommandData.roach[m_roach->which - 1].load_vna_amps = 0;
    }
    if (CommandData.roach[m_roach->which - 1].load_targ_amps) {
        blast_info("Roach%d, Loading TARG AMPS", m_roach->which);
        char *amps_path = m_roach->targ_amps_path[CommandData.roach[m_roach->which - 1].load_targ_amps - 1];
        // If can't load file, use all 1
        if ((roach_read_1D_file(m_roach, amps_path, amps, m_freqlen) < 0)) {
            for (size_t i = 0; i < m_freqlen; i++) {
                amps[i] = 1.0;
            }
        }
        CommandData.roach[m_roach->which - 1].load_targ_amps = 0;
    }
    if (CommandData.roach[m_roach->which - 1].change_tone_amps) {
        for (size_t i = 0; i < m_freqlen; i++) {
            amps[i] = m_roach->last_amps[i];
            // blast_info("amps = %g", m_roach->last_amps[i]);
        }
    } else {
        for (size_t i = 0; i < m_freqlen; i++) {
            m_roach->last_amps[i] = amps[i];
            // blast_info("m_freq = %g", m_freqs[i]);
        }
    }
    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / DAC_FREQ_RES) * DAC_FREQ_RES;
        // blast_info("m_freq = %g", m_freqs[i]);
    }
    /* for (size_t i = 0; i < m_freqlen; i++) {
        blast_info("amps = %g", amps[i]);
    }*/
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
    // blast_info("END OF DAC LUT FUNCTION");
    retval = 0;
    return retval;

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
    if (m_roach->vna_comb) free(m_roach->vna_comb);
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
static int roach_define_DAC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    if (m_roach->DAC.Ival) free(m_roach->DAC.Ival);
    if (m_roach->DAC.Qval) free(m_roach->DAC.Qval);
    m_roach->DAC.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
    m_roach->DAC.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
    m_roach->DAC.len = LUT_BUFFER_LEN;

    if ((roach_dac_comb(m_roach, m_freqs, m_freqlen,
                    DAC_SAMP_FREQ, m_roach->DAC.Ival, m_roach->DAC.Qval) < 0)) {
        return -1;
    } else {
        return 0;
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
int roach_select_bins(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int bins[fft_len];
    double bin_freqs[fft_len];

    if (m_roach->freq_residuals) free(m_roach->freq_residuals);
    m_roach->freq_residuals = calloc(m_freqlen, sizeof(size_t));

    for (size_t i = 0; i < m_freqlen; i++) {
        bins[i] = roach_fft_bin_idx(m_freqs, i, fft_len, DAC_SAMP_FREQ);
        bin_freqs[i] = bins[i] * DAC_SAMP_FREQ / fft_len;
        if (bins[i] < 0) {
            bins[i] += 1024;
        }
        m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / (DAC_FREQ_RES)) * (DAC_FREQ_RES);
    }
    for (int ch = 0; ch < m_freqlen; ch++) {
        if ((roach_write_int(m_roach, "bins", bins[ch], 0)) < 0) {
            return -1;
        }
        usleep(100);
        if ((roach_write_int(m_roach, "load_bins", 2*ch + 1, 0)) < 0) {
            return -1;
        }
        usleep(100);
        if ((roach_write_int(m_roach, "load_bins", 0, 0)) < 0) {
            return -1;
        }
    }
    return 0;
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
int roach_define_DDC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int retval = -1;
    if (roach_select_bins(m_roach, m_freqs, m_freqlen) < 0) {
        return retval;
    } else {
        retval = 0;
    }

    if (m_roach->DDC.Ival) free(m_roach->DDC.Ival);
    if (m_roach->DDC.Qval) free(m_roach->DDC.Qval);
    m_roach->DDC.Ival = calloc(LUT_BUFFER_LEN, sizeof(double));
    m_roach->DDC.Qval = calloc(LUT_BUFFER_LEN, sizeof(double));
    m_roach->DDC.len = LUT_BUFFER_LEN;

    double phases[m_freqlen];
    for (size_t i = 0; i < m_freqlen; i++) {
        phases[i] = 0.0;
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
        for (int j = i, k = 0; k < 2*fft_len; j += fft_len, k++) {
            m_roach->DDC.Ival[j] = Ival[k];
            m_roach->DDC.Qval[j] = Qval[k];
        }
    }
    return retval;
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
    for (size_t i = 0; i < LUT_BUFFER_LEN; i += 2) {
        m_roach->LUT.Ival[2 * i + 0] = htons(m_roach->DAC.Ival[i + 1]);
        m_roach->LUT.Ival[2 * i + 1] = htons(m_roach->DAC.Ival[i]);
        m_roach->LUT.Ival[2 * i + 2] = htons(m_roach->DDC.Ival[i + 1]);
        m_roach->LUT.Ival[2 * i + 3] = htons(m_roach->DDC.Ival[i]);
        m_roach->LUT.Qval[2 * i + 0] = htons(m_roach->DAC.Qval[i + 1]);
        m_roach->LUT.Qval[2 * i + 1] = htons(m_roach->DAC.Qval[i]);
        m_roach->LUT.Qval[2 * i + 2] = htons(m_roach->DDC.Qval[i + 1]);
        m_roach->LUT.Qval[2 * i + 3] = htons(m_roach->DDC.Qval[i]);
    }
}

/* Function: roach_write_QDR
 * ----------------------------
 * Writes Roach tone LUT to QDR ram
 *
 * @param m_roach a roach state structure
*/
int roach_write_QDR(roach_state_t *m_roach)
{
    int retval = -1;
    roach_pack_LUTs(m_roach);
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
    roach_write_int(m_roach, "start_dac", 0, 0);
    if (roach_write_data(m_roach, "qdr0_memory", (uint8_t*)m_roach->LUT.Ival,
        m_roach->LUT.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
        blast_info("Could not write to qdr0!");
        return retval;
    }
    if (roach_write_data(m_roach, "qdr1_memory", (uint8_t*)m_roach->LUT.Qval,
        m_roach->LUT.len * sizeof(uint16_t), 0, QDR_TIMEOUT) < 0) {
        blast_info("Could not write to qdr1!");
        return retval;
    }
    usleep(1000);
    roach_write_int(m_roach, "start_dac", 1, 0);
    usleep(100);
    if ((roach_write_int(m_roach, "downsamp_sync_accum_reset", 0, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "downsamp_sync_accum_reset", 1, 0)) < 0) {
        return retval;
    }
    retval = 0;
    return retval;
}

/* Function: roach_write_tones
 * ----------------------------
 * Calls functions to upload Roach DAC and DDS LUTs
 *
 * @param m_roach a roach state structure
 * @param m_freqs list of comb frequencies
 * @param m_freqlen number of frequencies in comb
*/
int roach_write_tones(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    // blast_info("INSIDE ROACH WRITE TONES");
    int retval = -1;
    m_roach->write_flag = 1;
    roach_init_DACDDC_LUTs(m_roach, LUT_BUFFER_LEN);
    if ((roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen) < 0)) {
        blast_info("DEFINE DAC LUT FAIL");
        m_roach->write_flag = 0;
        return retval;
    }
    if ((roach_define_DDC_LUT(m_roach, m_freqs, m_freqlen) < 0)) {
        blast_info("DEFINE DDC LUT FAIL");
        m_roach->write_flag = 0;
        return retval;
    }
    blast_info("ROACH%d, Uploading Tone LUTs...", m_roach->which);
    if ((roach_write_QDR(m_roach) < 0)) {
        m_roach->write_flag = 0;
        return retval;
    }
    if ((roach_write_int(m_roach, "write_comb_len", (uint32_t)m_freqlen, 0)) < 0) {
        m_roach->write_flag = 0;
        return retval;
    }
    m_roach->current_ntones = m_freqlen;
    m_roach->write_flag = 0;
    m_roach->has_tones = 1;
    blast_info("ROACH%d, Write complete.", m_roach->which);
    retval = 0;
    return retval;
    // blast_info("WRITE TONES RETVAL = %d", retval);
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
    blast_info("ROACH%d, Checking stream status...", m_roach->which);
    int retval = -2;
    int m_last_packet_count = roach_udp[m_roach->which - 1].roach_packet_count;
    // blast_info("m_last_packet_count = % d", m_last_packet_count);
    int count = 0;
    while ((count < ntries)) {
        // blast_info("roach_ddpacket_count = % d", roach_udp[m_roach->which - 1].roach_packet_count);
        sleep(sec_timeout);
        if (roach_udp[m_roach->which - 1].roach_packet_count > m_last_packet_count) {
            blast_info("ROACH%d, Streaming OK.", m_roach->which);
            // set streaming flag
            m_roach->is_streaming = 1;
            return 0;
        } else {
            count += 1;
        }
    }
    blast_err("Data stream error on ROACH%d", m_roach->which);
    m_roach->is_streaming = 0;
    return retval;
}

int atten_client(pi_state_t *m_pi, char *command)
{
    int status = -1;
    int s;
    struct sockaddr_in sin;
    struct hostent *hp;
    // char vcommand[strlen(argv[1]) + strlen(argv[2])];
    char buff[1024];
    // bzero(command, sizeof(command));
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        blast_err("Pi%d: Socket failed", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }
    /*
    int one = 1;
    if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int)) < 0) {
        blast_err("Pi%d: Set sock options failed", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }*/
    /* Gets, validates host; stores address in hostent structure. */
    if ((hp = gethostbyname(m_pi->address)) == NULL) {
        blast_err("Pi%d: Couldn't establish connection at given hostname", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }
    /* Assigns port number. */
    sin.sin_family = AF_INET;
    sin.sin_port = htons(ATTEN_PORT);
      /* Copies host address to socket with aide of structures. */
    bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
    /* Requests link with server and verifies connection. */
    if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
        blast_err("ROACH%d: Connection Error", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }
    bzero(buff, sizeof(buff));
    if ((status = write(s, command, strlen(command))) < 0) {
    // printf("STATUS = %d\n", status);
        blast_err("ROACH%d: Error setting Atten", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }
    if ((status = read(s, buff, sizeof(buff))) < 0) {
        blast_err("ROACH%d: Error receiving Pi response", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }
    // printf("STATUS = %d\n", status);
    // blast_info("%s", buff);
    int count = 0;
    char* line;
    char* rest = buff;
    char response[4][100];
    while ((line = strtok_r(rest, "\n", &rest))) {
        snprintf(response[count], sizeof(response[count]), line);
        // blast_info("RESPONSE = %s", response[count]);
        count++;
    }
    // blast_info("RESPONSE 0, SERIAL = %s %s", response[0], rudat_input_serials[m_pi->which - 1]);
    if (strcmp(response[0], rudat_input_serials[m_pi->which - 1]) == 0) {
        CommandData.roach_params[m_pi->which - 1].read_in_atten = atof(response[1]);
        CommandData.roach_params[m_pi->which - 1].read_out_atten = atof(response[3]);
    } else {
        CommandData.roach_params[m_pi->which - 1].read_in_atten = atof(response[3]);
        CommandData.roach_params[m_pi->which - 1].read_out_atten = atof(response[1]);
    }
    blast_info("OUT ATTEN: %f", CommandData.roach_params[m_pi->which - 1].read_out_atten);
    blast_info("IN ATTEN: %f", CommandData.roach_params[m_pi->which - 1].read_in_atten);
    close(s);
    return 0;
}

int valon_client(pi_state_t *m_pi, char *command)
{
    // If read flag = 1, parse response and store
    int status = -1;
    int s;
    struct sockaddr_in sin; // , client;
    struct hostent *hp;
    // char vcommand[strlen(argv[1]) + strlen(argv[2])];
    char buff[1024];
    // int src_port = 50000 + (m_pi->which - 1);
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        blast_err("Pi%d: Socket failed", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    }
    /* int one = 1;
    if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int)) < 0) {
        blast_err("Pi%d: Set sock options failed", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    } */
    /* bind client */
    /* client.sin_family = AF_INET;
    client.sin_addr.s_addr = INADDR_ANY;
    client.sin_port = htons(src_port);
    if (bind(s, (struct sockaddr*) &client, sizeof(struct sockaddr)) < 0) {
        blast_err("Pi%d: Unable to bind", m_pi->which);
    }*/
    /* Gets, validates host; stores address in hostent structure. */
    if ((hp = gethostbyname(m_pi->address)) == NULL) {
        blast_err("Pi%d: Couldn't establish connection at given hostname", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        close(s);
        return status;
    }
    /* Assigns port number. */
    sin.sin_family = AF_INET;
    sin.sin_port = htons(VALON_PORT);
      /* Copies host address to socket with aide of structures. */
    bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
    /* Requests link with server and verifies connection. */
    if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
        blast_err("ROACH%d: Connection Error", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        close(s);
        return status;
    }
    bzero(buff, sizeof(buff));
    if ((status = write(s, command, strlen(command))) < 0) {
    // printf("STATUS = %d\n", status);
        blast_err("ROACH%d: Error sending Pi command", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        close(s);
        return status;
    }
    if ((status = read(s, buff, sizeof(buff))) < 0) {
        blast_err("ROACH%d: Error receiving Pi response", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        close(s);
        return status;
    }
    // printf("STATUS = %d\n", status);
    // blast_info("%s", buff);
    float lo_freq_read = atof(buff);
    if (lo_freq_read > 0) {
        roach_state_table[m_pi->which - 1].lo_freq_read = atof(buff);
    } else {
        blast_err("ROACH%d: received bad freq read = %f", m_pi->which, lo_freq_read);
        close(s);
        return status;
    }
    // blast_info("%g", roach_state_table[m_pi->which - 1].lo_freq_read);
    close(s);
    return 0;
}

int set_LO(pi_state_t *m_pi, double lo_freq_MHz)
{
    int retval = -1;
    char* write_this;
    blast_tmp_sprintf(write_this, "set %g", lo_freq_MHz);
    roach_state_table[m_pi->which - 1].lo_freq_req = lo_freq_MHz;
    if (valon_client(m_pi, write_this) < 0) {
        return retval;
    } else {
        return 0;
    }
}

int read_LO(pi_state_t *m_pi)
{
    int retval = -1;
    char write_this[] = "read";
    if (valon_client(m_pi, write_this) < 0) {
        return retval;
    } else {
        return 0;
    }
}

int roach_chop_lo(roach_state_t *m_roach)
{
    int status = -1;
    if (!CommandData.roach[m_roach->which - 1].enable_chop_lo) {
        return status;
    }
    if (m_roach->is_sweeping) {
        return status;
    }
    double set_freq[3];
    double step_hz;
    if (m_roach->array == 500) {
        step_hz = 10000.0;
    } else {
        step_hz = 2500.0;
    }
    // blast_info("ROACH%d: Chopping LO", m_roach->which);
    set_freq[0] = m_roach->lo_centerfreq - step_hz;
    set_freq[1] = m_roach->lo_centerfreq + step_hz;
    set_freq[2] = m_roach->lo_centerfreq;

    struct timespec ts;
    const struct timespec interval_ts = { .tv_sec = 0, .tv_nsec = 2e8}; // 200 ms interval
    clock_gettime(CLOCK_REALTIME, &ts);

    for (int i = 0; i < 3; i++) {
        if ((status = set_LO(&pi_state_table[m_roach->which - 1], set_freq[i]/1.0e6)) < 0) {
            return status;
        }
        ts = timespec_add(ts, interval_ts);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
    }
    return 0;
}

int read_atten(pi_state_t *m_pi)
{
    int retval = -1;
    char command[] = "read";
    if (atten_client(m_pi, command) < 0) {
        return retval;
    } else {
        return 0;
    }
}

int set_atten(pi_state_t *m_pi) {
    int retval = -1;
    int ind = m_pi->which - 1;
    char *command;
    double out_in_atten[2];
    // the order of input and output attenuators is switched between PIs
    if (ind == 0) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_in_atten,
           CommandData.roach_params[ind].set_out_atten);
    }
    if (ind == 1) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_out_atten,
           CommandData.roach_params[ind].set_in_atten);
    }
    if (ind == 2) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_out_atten,
           CommandData.roach_params[ind].set_in_atten);
    }
    if (ind == 3) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_in_atten,
           CommandData.roach_params[ind].set_out_atten);
    }
    if (ind == 4) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_out_atten,
           CommandData.roach_params[ind].set_in_atten);
    }
    // blast_info("COMMAND ========= %s", command);
    if (atten_client(m_pi, command) < 0) {
        return retval;
    }
    out_in_atten[0] = CommandData.roach_params[ind].set_out_atten;
    out_in_atten[1] = CommandData.roach_params[ind].set_in_atten;
    if ((roach_save_1D_file(&roach_state_table[ind],
           roach_state_table[ind].path_to_last_attens, out_in_atten, 2) < 0)) {
        blast_info("ROACH%d, Unable to write last atten settings to file", m_pi->which);
    }
    return 0;
}

int set_atten_conserved(pi_state_t *m_pi)
{
    // Must set output attenuation with command
    int retval = -1;
    int ind = m_pi->which - 1;
    char *command;
    double out_in_atten[2];
    // CommandData.roach_params[ind].set_in_atten =
    //    ATTEN_TOTAL - CommandData.roach_params[ind].set_out_atten;
    CommandData.roach_params[ind].set_in_atten = 0;
    // the order of input and output attenuators is switched between PIs
    if (ind == 0) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_in_atten,
           CommandData.roach_params[ind].set_out_atten);
    }
    if (ind == 1) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_out_atten,
           CommandData.roach_params[ind].set_in_atten);
    }
    if (ind == 2) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_out_atten,
           CommandData.roach_params[ind].set_in_atten);
    }
    if (ind == 3) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_in_atten,
           CommandData.roach_params[ind].set_out_atten);
    }
    if (ind == 4) {
        blast_tmp_sprintf(command, "set %g %g",
           CommandData.roach_params[ind].set_out_atten,
           CommandData.roach_params[ind].set_in_atten);
    }
    if (atten_client(m_pi, command) < 0) {
        return retval;
    }
    out_in_atten[0] = CommandData.roach_params[ind].set_out_atten;
    out_in_atten[1] = CommandData.roach_params[ind].set_in_atten;
    if ((roach_save_1D_file(&roach_state_table[ind],
           roach_state_table[ind].path_to_last_attens, out_in_atten, 2) < 0)) {
        blast_info("ROACH%d, Unable to write last atten settings to file", m_pi->which);
    }
    return 0;
}

/*
int find_atten(roach_state_t *m_roach, double pow_per_tone) {
    double out_atten = -47.0 - pow_per_tone - 10.0*log10(m_roach->current_ntones/1000.0);
    // Check if out_atten is inside allowable limits
    if (out_atten < 0) {
        out_atten = 0;
        blast_err("ROACH%d: Atten request too low, setting to 30 dB (max possible)",
               m_roach->which);
    } else if (out_atten > 30) {
        out_atten = 30;
        blast_err("ROACH%d: Atten request too high, setting to 30 dB (max possible)",
               m_roach->which);
    }
    double out_atten_rounded = round(out_atten / 0.5) * 0.5;
    CommandData.roach_params[m_roach->which - 1].set_out_atten = out_atten_rounded;
    CommandData.roach_params[m_roach->which - 1].set_in_atten = ATTEN_TOTAL - out_atten_rounded;
    blast_info("ROACH%d, output atten = %f for %f dBm/tone with Ntones = %zd",
       m_roach->which, out_atten, pow_per_tone, m_roach->current_ntones);
    return 0;
}
*/

int find_atten(roach_state_t *m_roach, double pow_per_tone_desired) {
    double out_atten = rf_pow_out[m_roach->current_ntones - 1] - pow_per_tone_desired;
    // Check if out_atten is inside allowable limits
    if (out_atten < 0) {
        out_atten = 0;
        blast_err("ROACH%d: Atten request too low, setting to 0 dB (min possible)",
               m_roach->which);
    } else if (out_atten > 30) {
        out_atten = 30;
        blast_err("ROACH%d: Atten request too high, setting to 30 dB (max possible)",
               m_roach->which);
    }
    double out_atten_rounded = round(2 * out_atten) * 0.5;
    CommandData.roach_params[m_roach->which - 1].set_out_atten = out_atten_rounded;
    // CommandData.roach_params[m_roach->which - 1].set_in_atten = ATTEN_TOTAL - out_atten_rounded;
    CommandData.roach_params[m_roach->which - 1].set_in_atten = 0;
    blast_info("ROACH%d, output atten = %f for %f dBm/tone with Ntones = %zd",
       m_roach->which, out_atten, pow_per_tone_desired, m_roach->current_ntones);
    return 0;
}

int write_last_attens(roach_state_t *m_roach) {
    int retval = -1;
    double out_in_atten[2];
    if ((roach_read_1D_file(m_roach, m_roach->path_to_last_attens, out_in_atten, 2) < 0)) {
        CommandData.roach[m_roach->which - 1].set_attens = 0;
        return retval;
    }
    CommandData.roach_params[m_roach->which - 1].set_out_atten = out_in_atten[0],
    CommandData.roach_params[m_roach->which - 1].set_in_atten = out_in_atten[1];
    if (set_atten(&pi_state_table[m_roach->which - 1]) < 0) {
        CommandData.roach[m_roach->which - 1].set_attens = 0;
        return retval;
    }
    CommandData.roach[m_roach->which - 1].set_attens = 0;
    return 0;
}

int set_attens_to_default(pi_state_t *m_pi)
{
    int retval = 0;
    char *command;
    int ind = m_pi->which - 1;
    // the order of input and output attenuators is switched between PIs
    if (ind == 0) {
        blast_tmp_sprintf(command, "set %d %d",
           DEFAULT_INPUT_ATTEN,
           DEFAULT_OUTPUT_ATTEN);
    }
    if (ind == 1) {
        blast_tmp_sprintf(command, "set %d %d",
           DEFAULT_OUTPUT_ATTEN,
           DEFAULT_INPUT_ATTEN);
    }
    if (ind == 2) {
        blast_tmp_sprintf(command, "set %d %d",
           DEFAULT_OUTPUT_ATTEN,
           DEFAULT_INPUT_ATTEN);
    }
    if (ind == 3) {
        blast_tmp_sprintf(command, "set %d %d",
           DEFAULT_INPUT_ATTEN,
           DEFAULT_OUTPUT_ATTEN);
    }
    if (ind == 4) {
        blast_tmp_sprintf(command, "set %d %d",
           DEFAULT_OUTPUT_ATTEN,
           DEFAULT_INPUT_ATTEN);
    }
    if (atten_client(m_pi, command) < 0) {
        retval = -1;
    }
    return retval;
}

int set_attens_targ_output(roach_state_t *m_roach)
{
    int status = -1;
    int i = m_roach->which - 1;
    if ((status = find_atten(m_roach, CommandData.roach_params[i].dBm_per_tone)) < 0) {
        blast_err("ROACH%d: Failed to calculate output_atten...", i + 1);
        CommandData.roach[i].set_attens = 0;
        return status;
    } else {
        if ((status = set_atten(&pi_state_table[i])) < 0) {
            blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
            CommandData.roach[i].set_attens = 0;
            return status;
        }
    }
    CommandData.roach[i].set_attens = 0;
    return 0;
}

int set_output_atten(roach_state_t *m_roach, double new_out_atten)
{
    int retval = -1;
    // double input_atten = CommandData.roach_params[m_roach->which - 1].in_atten;
    CommandData.roach_params[m_roach->which - 1].set_out_atten = new_out_atten;
    if ((CommandData.roach_params[m_roach->which - 1].set_out_atten <= 0.5) ||
                 (CommandData.roach_params[m_roach->which - 1].set_out_atten >= 30.5)) {
        blast_info("ROACH%d, Output atten limit, aborting cal", m_roach->which);
    } else {
        retval = set_atten(&pi_state_table[m_roach->which - 1]);
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
/*
int roach_save_sweep_packet(roach_state_t *m_roach, uint32_t m_sweep_freq,
                                     char *m_sweep_save_path, size_t m_freqlen)
{
    // Grab a set number of packets, average I and Q for each chan,
    and save the result to file:fname in the sweep dir (/data/etc/blast/sweeps/)
    // Save I_avg, Q_avg, to sweep dir
    int retval = -1;
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    char *fname;
    blast_tmp_sprintf(fname, "%s/%d.dat", m_sweep_save_path, m_sweep_freq);
    FILE *m_fd = fopen(fname, "w");
    if (!m_fd) {
        blast_strerror("Could not open %s for writing", fname);
        return retval;
    }
    double *I_sum = calloc(m_freqlen, sizeof(double)); // Array to store I values to be summed
    double *Q_sum = calloc(m_freqlen, sizeof(double)); // Array to store Q values to be summed
    double *I_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged I values
    double *Q_avg = calloc(m_freqlen, sizeof(double)); // Array to store averaged Q values
    // m_roach->is_averaging = 1;
    int count = 0;
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
            count++;
        } else {
            break;
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    // m_roach->is_averaging = 0;
    if (!count) count = 1;
    for (size_t chan = 0; chan < m_freqlen; chan++) {
        I_avg[chan] = (I_sum[chan] / count);
        Q_avg[chan] = (Q_sum[chan] / count);
        // Save I_avg, Q_avg, to sweep dir
        fprintf(m_fd, "%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
        // blast_info("%zd\t %f\t %f\n", chan, I_avg[chan], Q_avg[chan]);
    }
    free(I_sum);
    free(Q_sum);
    free(I_avg);
    free(Q_avg);
    fclose(m_fd);
    return retval;
}*/

int roach_save_sweep_packet_binary(roach_state_t *m_roach, uint32_t m_sweep_freq,
                                     char *m_sweep_save_path, size_t m_freqlen)
{
    /* Grab a set number of packets, average I and Q for each chan,
    and save the result to disk */
    /* Save I_avg, Q_avg, to sweep dir */
    int retval = -1;
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    char *fname;
    blast_tmp_sprintf(fname, "%s/%d.dat", m_sweep_save_path, m_sweep_freq);
    FILE *fd = fopen(fname, "wb");
    if (!fd) {
        blast_strerror("Could not open %s for writing", fname);
        return retval;
    }
    float *I_sum = calloc(m_freqlen, sizeof(float)); // Array to store I values to be summed
    float *Q_sum = calloc(m_freqlen, sizeof(float)); // Array to store Q values to be summed
    // m_roach->is_averaging = 1;
    int count = 0;
    int data_error_counter = 0;
    while ((m_num_received < N_AVG) && (data_error_counter < MAX_DATA_ERRORS)) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            for (size_t chan = 0; chan < m_freqlen; chan ++) {
                I_sum[chan] +=  m_packet.Ival[chan];
                Q_sum[chan] +=  m_packet.Qval[chan];
            }
            count++;
        } else {
            data_error_counter++;
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    if (data_error_counter == MAX_DATA_ERRORS) {
        free(I_sum);
        free(Q_sum);
        fclose(fd);
        return retval;
    }
    // m_roach->is_averaging = 0;
    if (!count) count = 1;
    float I_avg = 0.0;
    float Q_avg = 0.0;
    for (size_t chan = 0; chan < m_freqlen; chan++) {
        I_avg = (I_sum[chan] / count);
        Q_avg = (Q_sum[chan] / count);
        fwrite(&I_avg, sizeof(I_avg), 1, fd);
        fwrite(&Q_avg, sizeof(Q_avg), 1, fd);
        // blast_info("%zd\t %f\t %f\n", chan, I_avg, Q_avg);
    }
    free(I_sum);
    free(Q_sum);
    fclose(fd);
    return 0;
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

/* int roach_timestamp_init(roach_state_t *m_roach)
{
    int retval = -1;
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    char *sec;
    char *nsec;
    char sec_truncated[2];
    char nsec_truncated[2];
    char *timestamp;
    blast_tmp_sprintf(sec, "%ld", ts.tv_sec);
    blast_tmp_sprintf(nsec, "%ld", ts.tv_nsec);
    strncpy(sec_truncated, sec+3, 8);
    strncpy(nsec_truncated, nsec, 3);
    sec_truncated[7] = '\0';
    nsec_truncated[3] = '\0';
    blast_tmp_sprintf(timestamp, "%s%s", sec_truncated, nsec_truncated);
    if ((roach_write_int(m_roach, "GbE_ctime", atoi(timestamp), 0) < 0)) {
        blast_warn("ROACH%d: Timestamp write error", m_roach->which);
    }
    // blast_info("TIMESTAMP ============ %s", timestamp);
    retval = 0;
    return retval;
}*/

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

/* Function: create_data_dir
 * ----------------------------
 * Creates a sweep directory
 *
 * @param m_roach roach state table
 * @param type:
 *    0 = VNA
 *    1 = TARG
 *    2 = IQ
 *    3 = DF
*/
int create_data_dir(roach_state_t *m_roach, int type)
{
    int retval = -1;;
    char *new_path;
    char *path_root;
    if ((type == VNA)) {
        path_root = m_roach->vna_path_root;
    } else if ((type == TARG)) {
        path_root = m_roach->targ_path_root;
    } else if ((type == IQ)) {
        path_root = m_roach->iq_path_root;
    } else if ((type == DF)) {
        path_root = m_roach->df_path_root;
    }
    new_path = get_path(m_roach, path_root);
    if (type == VNA) {
        asprintf(&m_roach->last_vna_path, new_path);
    } else if (type == TARG) {
        asprintf(&m_roach->last_targ_path, new_path);
    } else if (type == IQ) {
        asprintf(&m_roach->last_iq_path, new_path);
    } else if (type == DF) {
        asprintf(&m_roach->last_df_path, new_path);
    }
    blast_info("ROACH%d, new data will be saved in %s", m_roach->which, new_path);
    if (mkdir_recursive(new_path)) {
        retval = 1;
    } else {
        blast_strerror("Could not create new directory: %s", new_path);
    }
    return retval;
}

/* Function: save targ transfer function */
// TODO(Sam) error handling
int save_output_trf(roach_state_t *m_roach)
{
    char *py_command;
    blast_tmp_sprintf(py_command,
          "python %s %d %s %g", gen_output_trf_script, m_roach->which,
             m_roach->sweep_root_path, m_roach->lo_centerfreq);
    blast_info("ROACH%d, Saving output transfer function", m_roach->which);
    pyblast_system(py_command);
    return 0;
}

// For testing, if m_roach->last_targ_path doesn't exist, load it from the file
// last_targ_path in m_roach->sweep_root_path
int load_last_sweep_path(roach_state_t *m_roach, int sweep_type)
{
    int retval = -1;
    char buffer[READ_LINE];
    char *path_ref;
    char *new_path;
    if (sweep_type == VNA) {
        blast_tmp_sprintf(path_ref, m_roach->vna_path_ref);
    }
    if (sweep_type == TARG) {
        blast_tmp_sprintf(path_ref, m_roach->targ_path_ref);
    }
    FILE *fd = fopen(path_ref, "r");
    if (!fd) {
        blast_err("Error opening path ref file");
        return retval;
    }
    if (!fgets(buffer, sizeof(buffer), fd)) {
        blast_err("Error reading path ref file");
        blast_err("ROACH%d, Could not load path to last sweep", m_roach->which);
        return retval;
    } else {
        fclose(fd);
        if (sweep_type == VNA) {
            blast_tmp_sprintf(new_path, "%s", buffer);
            asprintf(&m_roach->last_vna_path, new_path);
            blast_info("ROACH%d, Last sweep path: %s", m_roach->which, m_roach->last_vna_path);
        }
        if (sweep_type == TARG) {
            blast_tmp_sprintf(new_path, "%s", buffer);
            asprintf(&m_roach->last_targ_path, new_path);
            blast_info("ROACH%d, Last sweep path: %s", m_roach->which, m_roach->last_targ_path);
        }
        retval = 0;
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
int get_targ_freqs(roach_state_t *m_roach, bool m_use_default_params)
{
    int retval = -1;
    // blast_info("ROACH%d: INSIDE GET TARG FREQS", m_roach->which);
    if (!m_roach->last_vna_path) {
        blast_info("Roach%d, NO VNA PATH FOUND, loading from ref file", m_roach->which);
        if ((load_last_sweep_path(m_roach, VNA) < 0)) {
            return retval;
        }
    }
    char *py_command;
    char *targ_freq_path;
    char *path_to_mags_and_freqs;
    double temp_freqs[MAX_CHANNELS_PER_ROACH];
    char line[READ_LINE];
    m_roach->is_finding_kids = 1;
    blast_info("Calling Python script...");
    if (!m_use_default_params) {
        blast_tmp_sprintf(py_command, "python %s %d %s %g %g %g %g %g > %s",
            find_kids_script,
            m_roach->which,
            m_roach->last_vna_path,
            CommandData.roach_params[m_roach->which - 1].smoothing_scale,
            CommandData.roach_params[m_roach->which - 1].peak_threshold,
            CommandData.roach_params[m_roach->which - 1].spacing_threshold,
            m_roach->lo_centerfreq,
            (double)LO_STEP/1000.,
            m_roach->find_kids_log);
    } else {
        blast_tmp_sprintf(py_command, "python %s %d %s %g %g > %s",
            find_kids_default_script,
            m_roach->which,
            m_roach->last_vna_path,
            m_roach->lo_centerfreq,
            (double)LO_STEP/1000.,
            m_roach->find_kids_log);
    }
    blast_info("%s", py_command);
    pyblast_system(py_command);
    FILE *log = fopen(m_roach->find_kids_log, "r");
    if (!log) {
        blast_strerror("Could not open %s for reading", m_roach->find_kids_log);
        m_roach->is_finding_kids = 0;
        return retval;
    }
    while (fgets(line, sizeof(line), log)) {
        blast_info("%s", line);
    }
    fclose(log);
    blast_tmp_sprintf(targ_freq_path, "%s/bb_targ_freqs.dat", m_roach->sweep_root_path);
    blast_tmp_sprintf(path_to_mags_and_freqs, "%s/targ_mags_and_freqs.dat", m_roach->sweep_root_path);
    // blast_tmp_sprintf(m_targ_freq_path, "%s/bb_targ_freqs.dat", m_roach->last_targ_path);
    FILE *fd;
    fd = fopen(targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", targ_freq_path);
        m_roach->is_finding_kids = 0;
        return retval;
    }
    // Set environmental variable linking to last bb targ freqs
    char *var_name;
    blast_tmp_sprintf(var_name, "R%d_LAST_BB_TARG_FREQS", m_roach->which);
    setenv(var_name, targ_freq_path, 1);
    // Set environmental variable linking to last freqs/mags array
    char *var_name1;
    blast_tmp_sprintf(var_name1, "R%d_LAST_TARG_FREQS_MAGS", m_roach->which);
    setenv(var_name1, path_to_mags_and_freqs, 1);
    m_roach->num_kids = 0;
    while (m_roach->num_kids < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &temp_freqs[(m_roach->num_kids)++]) != EOF) {
    }
    fclose(fd);
    if (m_roach->num_kids > 0) {
        (m_roach->num_kids)--;
    } else {
        blast_err("ROACH%d, Error finding TARG freqs", m_roach->which);
        m_roach->is_finding_kids = 0;
        m_roach->num_kids = 0;
        m_roach->tone_finding_error = 3;
        return retval;
    }
    // handle case where either not enough, or too many channels are found
    if (m_roach->num_kids > CommandData.roach[m_roach->which - 1].max_nkids) {
        blast_err("ROACH%d, Too many TARG freqs found, bad sweep likely", m_roach->which);
        m_roach->num_kids = 0;
        m_roach->is_finding_kids = 0;
        m_roach->tone_finding_error = 2;
        return retval;
    }
    if (m_roach->num_kids < CommandData.roach[m_roach->which - 1].min_nkids) {
        blast_err("ROACH%d, Too few TARG freqs found, bad sweep likely", m_roach->which);
        m_roach->num_kids = 0;
        m_roach->is_finding_kids = 0;
        m_roach->tone_finding_error = 1;
        return retval;
    }
    blast_info("NUM kids = %zd", m_roach->num_kids);
    if (m_roach->targ_tones) free(m_roach->targ_tones);
    m_roach->targ_tones = calloc(m_roach->num_kids, sizeof(double));
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = temp_freqs[j];
        // blast_info("KID freq = %lg", m_roach->targ_tones[j] + m_roach->lo_centerfreq);
    }
    if (CommandData.roach[m_roach->which - 1].find_kids > 0) {
        CommandData.roach[m_roach->which - 1].find_kids = 0;
    }
    save_output_trf(m_roach);
    m_roach->is_finding_kids = 0;
    m_roach->tone_finding_error = 0;
    m_roach->has_targ_tones = 1;
    return 0;
}

int roach_dfs(roach_state_t* m_roach)
{
    int retval = -1;
    // check for ref params
    if ((!m_roach->has_ref_params)) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        return retval;
    }
    // Get I and Q vals from packets. Average NUM_AVG values
    // Store in comp_vals
    double comp_vals[m_roach->num_kids][2];
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    double *I_sum = calloc(m_roach->num_kids, sizeof(double));
    double *Q_sum = calloc(m_roach->num_kids, sizeof(double));
    int count = 0;
    // m_roach->is_averaging = 1;
    int data_error_counter = 0;
    while ((m_num_received < N_AVG_DF) && (data_error_counter < MAX_DATA_ERRORS)) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            for (size_t chan = 0; chan < m_roach->num_kids; chan ++) {
                I_sum[chan] +=  m_packet.Ival[chan];
                Q_sum[chan] +=  m_packet.Qval[chan];
            }
            count++;
        } else {
            data_error_counter++;
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    if (data_error_counter == MAX_DATA_ERRORS) {
        free(I_sum);
        free(Q_sum);
        return retval;
    }
    if (!count) count = 1;
    for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
        comp_vals[chan][0] = I_sum[chan] / count;
        comp_vals[chan][1] = Q_sum[chan] / count;
    }
    free(I_sum);
    free(Q_sum);
    // calculate df for each channel
    for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
        double deltaI = comp_vals[chan][0] - m_roach->ref_vals[chan][0];
        double deltaQ = comp_vals[chan][1] - m_roach->ref_vals[chan][1];
        m_roach->df[chan] = ((m_roach->ref_grads[chan][0] * deltaI) + (m_roach->ref_grads[chan][1] * deltaQ)) /
        (m_roach->ref_grads[chan][0]*m_roach->ref_grads[chan][0] +
        m_roach->ref_grads[chan][1]*m_roach->ref_grads[chan][1]);
        // if recenter_df is false, apply df_offset to each df value
        if (!CommandData.roach[m_roach->which - 1].recenter_df) {
            m_roach->df[chan] -= m_roach->df_offset[chan];
        }
        // blast_info("**** ROACH%d, chan %zd df = %g,", m_roach->which, chan, m_roach->df[chan]);
        /* blast_info("**** ROACH%d, chan %zd df = %g, Icomp_val %g, deltaI %g, Irefval %g, offset %g",
               m_roach->which,
               chan,
               m_roach->df[chan],
               comp_vals[chan][0],
               deltaI,
               m_roach->ref_vals[chan][0],
               m_roach->df_offset[chan]);*/
    }
    // save list of dfs
    char* path_to_ts_dfs;
    blast_tmp_sprintf(path_to_ts_dfs, "%s/df.dat", m_roach->sweep_root_path);
    if ((roach_save_1D_file(m_roach, path_to_ts_dfs, m_roach->df, m_roach->current_ntones) < 0)) {
        return retval;
    }
    char *var_name;
    blast_tmp_sprintf(var_name, "R%d_DF_LIST", m_roach->which);
    // blast_tmp_sprintf(echo_command, "echo $%s", var_name);
    setenv(var_name, path_to_ts_dfs, 1);
    // m_roach->is_averaging = 0;
    retval = 0;
    return retval;
}

void center_df(roach_state_t *m_roach)
{
    CommandData.roach[m_roach->which - 1].recenter_df = 1;
    roach_dfs(m_roach);
    for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
        m_roach->df_offset[chan] = m_roach->df[chan];
    }
    blast_info("ROACH%d, zeroed dfs", m_roach->which);
    CommandData.roach[m_roach->which - 1].recenter_df = 0;
}

int save_ref_params(roach_state_t *m_roach)
{
    int retval = -1;
    if (!m_roach->last_targ_path) {
        if ((load_last_sweep_path(m_roach, TARG) < 0)) {
            return retval;
        }
    }
    char *path_to_ref_grads;
    char *path_to_ref_vals;
    char *py_command;
    blast_tmp_sprintf(path_to_ref_grads, "%s/ref_grads.dat",
                     m_roach->sweep_root_path);
    blast_tmp_sprintf(path_to_ref_vals, "%s/ref_vals.dat",
                     m_roach->sweep_root_path);
    blast_info("Roach%d, Saving ref grads", m_roach->which);
    blast_tmp_sprintf(py_command, "python %s %s %d", ref_grads_script,
            m_roach->last_targ_path, CommandData.roach[m_roach->which-1].on_res);
    blast_info("%s", py_command);
    pyblast_system(py_command);
    // get reference gradients
    if ((roach_read_2D_file(m_roach, path_to_ref_grads,
              m_roach->ref_grads, m_roach->num_kids) < 0)) {
        return retval;
    }
    if ((roach_read_2D_file(m_roach, path_to_ref_vals,
               m_roach->ref_vals, m_roach->num_kids) < 0)) {
        return retval;
    }
    /* for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
        blast_info("*************** ROACH%d, dIdf, dQdf = %g, %g", m_roach->which,
              m_roach->ref_grads[chan][0], m_roach->ref_grads[chan][0]);
    } */
    // set 'has ref' flag
    m_roach->has_ref_params = 1;
    center_df(m_roach);
    retval = 0;
    return retval;
}

int roach_write_vna(roach_state_t *m_roach)
{
    if (m_roach->num_kids > 0) {
        m_roach->prev_num_kids = m_roach->num_kids;
    }
    int retval = -1;
    blast_info("ROACH%d, Generating VNA sweep comb...", m_roach->which);
    roach_vna_comb(m_roach);
    if (APPLY_VNA_TRF) {
        CommandData.roach[m_roach->which - 1].load_vna_amps = 2;
        blast_info("ROACH%d, Applying VNA trf correction", m_roach->which);
    }
    blast_info("ROACH%d, Writing tones", m_roach->which);
    if ((roach_write_tones(m_roach, m_roach->vna_comb, m_roach->vna_comb_len) < 0)) {
        return retval;
    }
    blast_info("ROACH%d, Search comb uploaded", m_roach->which);
    m_roach->has_tones = 1;
    m_roach->has_vna_tones = 1;
    m_roach->has_targ_tones = 0;
    m_roach->num_kids = 0;
    return 0;
}

int roach_write_saved(roach_state_t *m_roach)
{
    int retval = -1;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    blast_tmp_sprintf(m_targ_freq_path,
           m_roach->freqlist_path, m_roach->which);
    FILE *fd;
    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return retval;
    }
    if (m_roach->num_kids > 0) {
        m_roach->prev_num_kids = m_roach->num_kids;
    }
    m_roach->num_kids = 0;
    while (m_roach->num_kids < MAX_CHANNELS_PER_ROACH
            && fscanf(fd, "%lg\n", &m_temp_freqs[(m_roach->num_kids)++]) != EOF) {
    }
    fclose(fd);
    if (m_roach->num_kids > 0) {
        (m_roach->num_kids)--;
    } else {
        roach_write_vna(m_roach);
        return retval;
    }
    blast_info("NUM kids = %zd", m_roach->num_kids);
    if (m_roach->targ_tones) free(m_roach->targ_tones);
    m_roach->targ_tones = calloc(m_roach->num_kids, sizeof(double));
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j];
        // blast_info("KID freq = %lg", m_roach->targ_tones[j] + m_roach->lo_centerfreq);
    }
    save_output_trf(m_roach);
    if (APPLY_TARG_TRF) {
        CommandData.roach[m_roach->which - 1].load_targ_amps = 2;
        blast_info("ROACH%d, Applying targ trf correction", m_roach->which);
    }
    blast_info("Uploading TARGET comb...");
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        roach_write_vna(m_roach);
        return retval;
    }
    blast_info("ROACH%d, TARGET comb uploaded", m_roach->which);
    save_ref_params(m_roach);
    m_roach->has_targ_tones = 1;
    m_roach->has_vna_tones = 0;
    return 0;
}

int roach_write_targ_tones(roach_state_t *m_roach)
{
    int retval = -1;
    if (!m_roach->num_kids) {
        blast_err("ROACH%d: NUM KIDS = 0!", m_roach->which - 1);
        return retval;
    } else {
        blast_info("ROACH%d, Uploading TARGET comb...", m_roach->which);
        if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
            return retval;
        }
    }
    m_roach->has_vna_tones = 0;
    m_roach->has_targ_tones = 1;
    // m_roach->tone_write_fail = 0;
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
    pyblast_system(py_command);
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
    for (size_t j = 0; j < m_roach->num_kids; j++) {
        m_roach->targ_tones[j] = m_temp_freqs[j];
        blast_info("Optimized KID freq = %lg", m_roach->targ_tones[j]);
    }
    if (CommandData.roach[m_roach->which - 1].find_kids > 0) {
        CommandData.roach[m_roach->which - 1].find_kids = 0;
    }
    return 1;
}

void roach_watchdog(void *which_roach)
{
    int s;
    struct sockaddr_in sin;
    struct hostent *hp;
    int which = *((int *) which_roach);
    while (1) {
        sleep(ROACH_WATCHDOG_PERIOD);
        // Try to open socket to KATCP server
        if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            blast_err("ROACH%d: Watchdog socket failed", roach_state_table[which].which);
            continue;
        }
        /* Gets, validates host; stores address in hostent structure. */
        if ((hp = gethostbyname(roach_state_table[which].address)) == NULL) {
            blast_err("ROACH%d: Watchdog couldn't get address", roach_state_table[which].which);
        }
        /* Assigns port number. */
        sin.sin_family = AF_INET;
        sin.sin_port = htons(7147);
        /* Copies host address to socket with aide of structures. */
        bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
        /* Requests link with server and verifies connection. */
        if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
            blast_err("ROACH%d: Connection Error", roach_state_table[which].which);
            // if n_watchdog_fails = 5, Roach is put into boot state
            roach_state_table[which].n_watchdog_fails += 1;
            blast_info("ROACH%d: N watchdog fails = %d", roach_state_table[which].which,
                             roach_state_table[which].n_watchdog_fails);
        /* else {
            blast_info("ROACH%d: Watchdog check OK", roach_state_table[which].which);
        }*/
        }
        if (roach_state_table[which].n_watchdog_fails >= N_WATCHDOG_FAILS) {
            roach_state_table[which].state = ROACH_STATE_BOOT;
            blast_info("ROACH%d: Resetting state to BOOT", roach_state_table[which].which);
            // reset n_watchdog_fails
            roach_state_table[which].n_watchdog_fails = 0;
            // kill thread
            pthread_exit(NULL);
        }
    }
}

void start_watchdog_thread(int which_roach)
{
    pthread_t watchdog_thread;
    blast_info("ROACH%d: Creating ROACH watchdog thread...", which_roach + 1);
    if (pthread_create(&watchdog_thread, NULL, (void*)&roach_watchdog, (void *)&which_roach)) {
        blast_err("ROACH%d: Error creating watchdog thread", which_roach + 1);
    }
}

int setLO_oneshot(int which_pi, double loFreq)
{
    char *lo_freq;
    int s;
    int status = -1;
    struct sockaddr_in sin;
    struct hostent *hp;
    char buff[1024];
    bzero(buff, sizeof(buff));
    blast_tmp_sprintf(lo_freq, "set %g", loFreq);
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        blast_err("Pi%d: Socket failed", which_pi + 1);
        roach_state_table[which_pi].pi_error_count += 1;
        return status;
    }
    /* Gets, validates host; stores address in hostent structure. */
    if ((hp = gethostbyname(pi_state_table[which_pi].address)) == NULL) {
        blast_err("Pi%d: Couldn't establish connection at given hostname", which_pi + 1);
        roach_state_table[which_pi].pi_error_count += 1;
        return status;
    }
    /* Assigns port number. */
    sin.sin_family = AF_INET;
    sin.sin_port = htons(VALON_PORT);
    /* Copies host address to socket with aide of structures. */
    bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
    /* Requests link with server and verifies connection. */
    if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
        blast_err("Pi%d: Connection Error", which_pi + 1);
        roach_state_table[which_pi].pi_error_count += 1;
        return status;
    }
    // blast_info("Set Freq = %f", lo_freq);
    if ((status = write(s, lo_freq, strlen(lo_freq))) < 0) {
        blast_err("Pi%d: Error setting LO", which_pi + 1);
        roach_state_table[which_pi].pi_error_count += 1;
        return status;
    }
    // if (read(s, buff, sizeof(buff)) < 0) toltec_info("read fail");
    status = read(s, buff, sizeof(buff));
    blast_info("%s\n", buff);
    close(s);
    status = 0;
    return status;
}

int pi_reboot_now(pi_state_t *m_pi)
{
    blast_info("ROACH%d, REBOOTING PI%d NOW", m_pi->which, m_pi->which);
    roach_state_table[m_pi->which - 1].pi_reboot_warning = 1;
    int s;
    int status = -1;
    struct sockaddr_in sin;
    struct hostent *hp;
    char write_this[] = "sudo reboot\n";
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        blast_err("Pi%d: Socket failed", m_pi->which);
        return status;
    }
    /* Gets, validates host; stores address in hostent structure. */
    if ((hp = gethostbyname(m_pi->address)) == NULL) {
        blast_err("Pi%d: Couldn't establish connection at given hostname", m_pi->which);
        return status;
    }
    /* Assigns port number. */
    sin.sin_family = AF_INET;
    sin.sin_port = htons(NC2_PORT);
    /* Copies host address to socket with aide of structures.*/
    bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
    /* Requests link with server and verifies connection. */
    if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
        blast_err("Pi%d: Connection Error", m_pi->which);
        return status;
    }
    if ((status = write(s, write_this, strlen(write_this))) < 0) {
        blast_err("Pi%d: Could not reboot", m_pi->which);
        return status;
    }
    roach_state_table[m_pi->which - 1].pi_reboot_warning = 0;
    roach_state_table[m_pi->which - 1].pi_error_count = 0;
    return 0;
}

int get_pi_temp(pi_state_t *m_pi)
{
    int s;
    int status = -1;
    struct sockaddr_in sin;
    struct hostent *hp;
    char buff[1024];
    char write_this[] = "1";
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        blast_err("Pi%d: Socket failed", m_pi->which);
        return status;
    }
    /*
    int one = 1;
    if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int)) < 0) {
        blast_err("Pi%d: Set sock options failed", m_pi->which);
        roach_state_table[m_pi->which - 1].pi_error_count += 1;
        return status;
    } */
    /* Gets, validates host; stores address in hostent structure. */
    if ((hp = gethostbyname(m_pi->address)) == NULL) {
        blast_err("Pi%d: Couldn't establish connection at given hostname", m_pi->which);
        return status;
    }
    /* Assigns port number. */
    sin.sin_family = AF_INET;
    sin.sin_port = htons(PI_TEMP_PORT);
    /* Copies host address to socket with aide of structures.*/
    bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
    /* Requests link with server and verifies connection. */
    if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
        blast_err("Pi%d: Connection Error", m_pi->which);
        return status;
    }
    if ((status = write(s, write_this, strlen(write_this))) < 0) {
        blast_err("Pi%d: Could not reboot", m_pi->which);
        return status;
    }
    bzero(buff, sizeof(buff));
    if ((status = read(s, buff, sizeof(buff))) < 0) {
        blast_err("ROACH%d: Error receiving Pi response", m_pi->which);
        return status;
    }
    roach_state_table[m_pi->which - 1].pi_temp = atof(buff);
    blast_info("ROACH%d: Pi Temp = %f", m_pi->which,
             roach_state_table[m_pi->which - 1].pi_temp);
    close(s);
    return 0;
}

int valon_set_ref(pi_state_t *m_pi)
{
    if (CommandData.roach[m_pi->which - 1].change_extref) {
        blast_info("ROACH%d, Changing Valon reference", m_pi->which);
        int ext_ref = CommandData.roach[m_pi->which - 1].ext_ref;
        int s;
        int status = -1;
        struct sockaddr_in sin;
        struct hostent *hp;
        char *write_this;
        blast_tmp_sprintf(write_this, "sudo echo %d > /EXTREF", ext_ref);
        if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            blast_err("Pi%d: Socket failed", m_pi->which);
            return status;
        }
        /* Gets, validates host; stores address in hostent structure. */
        if ((hp = gethostbyname(m_pi->address)) == NULL) {
            blast_err("Pi%d: Couldn't establish connection at given hostname", m_pi->which);
            close(s);
            return status;
        }
        /* Assigns port number. */
        sin.sin_family = AF_INET;
        sin.sin_port = htons(NC2_PORT);
        /* Copies host address to socket with aide of structures.*/
        bcopy(hp->h_addr, (char *) &sin.sin_addr, hp->h_length);
        /* Requests link with server and verifies connection. */
        if (connect(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
            blast_err("Pi%d: Connection Error", m_pi->which);
            close(s);
            return status;
        }
        if ((status = write(s, write_this, strlen(write_this))) < 0) {
            blast_err("Pi%d: Couldn't write to socket", m_pi->which);
            close(s);
            return status;
        }
        roach_state_table[m_pi->which - 1].pi_error_count = 0;
        blast_info("ROACH%d: Set REF = %d", m_pi->which, ext_ref);
        close(s);
    }
    return 0;
}

int recenter_lo(roach_state_t *m_roach)
{
    if (set_LO(&pi_state_table[m_roach->which - 1], m_roach->lo_centerfreq/1.0e6) < 0) {
        blast_err("PI%d, Failed to recenter LO", m_roach->which);
        return -1;
    }
    return 0;
}

int shift_lo(roach_state_t *m_roach)
{
    int retval = -1;
    double shift = (double)CommandData.roach_params[m_roach->which - 1].lo_offset;
    blast_info("LO SHIFT ======================== %g", shift);
    double set_freq = (m_roach->lo_centerfreq + shift)/1.0e6;
    if (set_LO(&pi_state_table[m_roach->which - 1], set_freq) < 0) {
        blast_err("ROACH%d LO error", m_roach->which);
        return retval;
    }
    // blast_info("LO SET FREQ ======================== %g", set_freq);
    retval = 0;
    return retval;
}

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
    int ind = m_roach->which - 1;
    // blast_info("DO SWEEPS = %d", CommandData.roach[ind].do_sweeps);
    if (!CommandData.roach[ind].do_sweeps) {
        blast_err("do_sweeps NOT SET");
        return SWEEP_INTERRUPT;
    }
    // blast_info("INSIDE DO SWEEP");
    char *save_bbfreqs_command;
    // char *save_vna_trf_command;
    double m_span;
    char *sweep_freq_fname;
    char *save_path;
    // char *path_ref;
    size_t comb_len;
    // struct stat dir_stat;
    // int stat_return;
    if (sweep_type == VNA) {
        char *vna_freq_fname;
        if (create_data_dir(m_roach, VNA)) {
            m_span = m_roach->vna_sweep_span;
            // blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_vna_path);
            // blast_tmp_sprintf(vna_freq_fname, "%s/vna_freqs.dat", m_roach->last_vna_path);
            blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_vna_path);
            blast_tmp_sprintf(vna_freq_fname, "%s/vna_freqs.dat", m_roach->last_vna_path);
            if (save_freqs(m_roach, vna_freq_fname, m_roach->vna_comb, m_roach->vna_comb_len) < 0) {
                blast_err("Sweep freqs could not be saved to disk");
                return SWEEP_FAIL;
            }
            FILE *fd = fopen(m_roach->vna_path_ref, "w");
            if (fd) {
                fprintf(fd, "%s", m_roach->last_vna_path);
                fclose(fd);
            } else {
                blast_strerror("Could not open %s for writing", m_roach->vna_path_ref);
            }
            save_path = m_roach->last_vna_path;
            comb_len = m_roach->vna_comb_len;
            } else {
                return SWEEP_FAIL;
            }
    }
    if (sweep_type == TARG) {
        // If the command turnaround targ sweep span is sane, then use it.
        // If we are not sane, or not doing a turnaound, then use the commanded targ sweep.
        // If we are not sane at all, then use the default default.
        if ((CommandData.roach_params[m_roach->which-1].trnd_sweep_span >= 75.0e3) &&
            (CommandData.roach_params[m_roach->which-1].trnd_sweep_span <= 250.0e3) &&
            (m_roach->doing_turnaround_loop)) {
            m_span = CommandData.roach_params[m_roach->which-1].trnd_sweep_span;
        } else if ((CommandData.roach_params[m_roach->which-1].targ_sweep_span >= 75.0e3) &&
            (CommandData.roach_params[m_roach->which-1].targ_sweep_span <= 250.0e3)) {
            m_span = CommandData.roach_params[m_roach->which-1].targ_sweep_span;
        } else {
            m_span = DEFAULT_TARG_SWEEP_SPAN;
        }
        m_roach->targ_sweep_span_used = m_span;
        if (create_data_dir(m_roach, TARG)) {
            blast_info("ROACH%d, TARGET sweep will be saved in %s",
                           m_roach->which, m_roach->last_targ_path);
        } else {
            blast_info("ROACH%d, Could not create TARG dir", m_roach->which);
            return SWEEP_FAIL;
        }
        // }
        save_path = m_roach->last_targ_path;
        // write last_targ_path to file in m_roach->roach_sweep_root
        FILE *fd = fopen(m_roach->targ_path_ref, "w");
        if (fd) {
            fprintf(fd, "%s", m_roach->last_targ_path);
            fclose(fd);
        } else {
            blast_strerror("Could not open %s for writing", m_roach->targ_path_ref);
        }
        // Copy bb_targ_freqs.dat to new TARG sweep dir
        blast_tmp_sprintf(save_bbfreqs_command, "cp %s/roach%d/bb_targ_freqs.dat %s",
                        roach_root_path, m_roach->which, m_roach->last_targ_path);
        pyblast_system(save_bbfreqs_command);
        blast_tmp_sprintf(sweep_freq_fname, "%s/sweep_freqs.dat", m_roach->last_targ_path);
        blast_info("Sweep freq fname = %s", sweep_freq_fname);
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
    size_t m_num_sweep_freqs = ((m_max_freq - m_min_freq) + LO_STEP)/ LO_STEP;
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
    blast_info("STARTING SWEEP");
    for (size_t i = 0; i < m_num_sweep_freqs; i++) {
        if (CommandData.roach[ind].do_sweeps) {
                blast_tmp_sprintf(lo_command, "python /home/pi/device_control/set_lo.py %g",
                   m_sweep_freqs[i]/1.0e6);
            // m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
            set_LO(&pi_state_table[ind], m_sweep_freqs[i]/1.0e6);
            if (m_roach->pi_error_count >= MAX_PI_ERRORS_REBOOT) {
                free(m_sweep_freqs);
                return SWEEP_FAIL;
            }
            if (roach_save_sweep_packet_binary(m_roach, (uint32_t)m_sweep_freqs[i], save_path, comb_len) < 0) {
                free(m_sweep_freqs);
                return SWEEP_FAIL;
            }
        } else {
            blast_info("Sweep interrupted by command");
            free(m_sweep_freqs);
            CommandData.roach[ind].do_sweeps = 0;
            return SWEEP_INTERRUPT;
        }
    }
    /* if (recenter_lo(m_roach) < 0) {
        blast_info("Error recentering LO");
    } */
    free(m_sweep_freqs);
    CommandData.roach[ind].do_sweeps = 0;
    return SWEEP_SUCCESS;
}

/*
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
}*/

// Save short timestreams for single channel (using for cal lamp tests outside of KST)
int save_timestream(roach_state_t *m_roach, int m_chan, double m_nsec)
{
    char *file_out;
    if (create_data_dir(m_roach, IQ)) {
        blast_info("ROACH%d, CHOPS will be saved in %s",
                       m_roach->which, m_roach->last_iq_path);
    } else {
        blast_err("Could not create new chop directory");
        CommandData.roach[m_roach->which - 1].get_timestream = 0;
        return -1;
    }
    blast_tmp_sprintf(file_out, "%s/%d.dat", m_roach->last_iq_path, m_chan);
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
            fwrite(&I, sizeof(I), 1, fd);
            // write a 4 byte Q value
            fwrite(&Q, sizeof(Q), 1, fd);
            m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
        }
    }
    fclose(fd);
    // Save last chop path
    // pyblast_system("python /home/fc1user/sam_builds/chop_list.py");
    blast_info("ROACH%d, timestream saved", m_roach->which);
    CommandData.roach[m_roach->which - 1].get_timestream = 0;
    return 0;
}

char* truncate_path(char *old_path, int nparents)
{
    if (old_path == NULL) {
        return null_string;
    }
    int count = 0;
    char *new_path = old_path;
    while (*new_path) {
        if (*new_path == '/') count++;
        if (count > nparents) break;
        new_path++;
    }
    new_path = new_path + 1;
    return new_path;
}

int compress_data(roach_state_t *m_roach, int type)
{
// type can be: VNA, TARG, IQ or DF, LAMP
    // int status = -1;
    char *tar_cmd;
    char *tarball;
    char *path;
    if ((type == VNA)) {
        path = truncate_path(m_roach->last_vna_path, 3);
        blast_info("PATH: %s", path);
        tarball = path_to_vna_tarball[m_roach->which - 1];
    } else if ((type == TARG)) {
        path = truncate_path(m_roach->last_targ_path, 3);
        blast_info("PATH: %s", path);
        tarball = path_to_targ_tarball[m_roach->which - 1];
    } else if ((type == IQ)) {
        path = truncate_path(m_roach->last_iq_path, 3);
        blast_info("PATH: %s", path);
        tarball = path_to_iq_tarball[m_roach->which - 1];
    } else if ((type == DF)) {
        path = truncate_path(m_roach->last_df_path, 3);
        blast_info("PATH: %s", path);
        tarball = path_to_df_tarball[m_roach->which - 1];
    } else if ((type == LAMP)) {
        path = truncate_path(m_roach->path_to_lamp_response, 3);
        blast_info("PATH: %s", path);
        tarball = path_to_lamp_tarball[m_roach->which - 1];
    }
    blast_tmp_sprintf(tar_cmd, "tar -C %s -czf %s %s &", roach_root_path, tarball, path);
    blast_info("Creating sweep tarball: %s", tar_cmd);
    for (int i = 0; i < NUM_ROACHES; i++) {
        roach_state_table[i].is_compressing_data = 1;
    }
    pyblast_system(tar_cmd);
    for (int i = 0; i < NUM_ROACHES; i++) {
        roach_state_table[i].is_compressing_data = 0;
    }
    return 0;
}

int compress_all_data(int type)
{
    // int status = -1;
    char *tar_cmd;
    char *var_name;
    char *echo_cmd;
    blast_info("Building tarball");
    blast_tmp_sprintf(tar_cmd, "tar -czvf");
    if ((type == VNA)) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_vna,
           truncate_path(path_to_vna_tarball[0], 3),
           truncate_path(path_to_vna_tarball[1], 3),
           truncate_path(path_to_vna_tarball[2], 3),
           truncate_path(path_to_vna_tarball[3], 3),
           truncate_path(path_to_vna_tarball[4], 3));
        blast_tmp_sprintf(var_name, "ALL_VNA_SWEEPS");
        var_name = "ALL_VNA_SWEEPS";
        setenv(var_name, path_to_all_vna, 1);
    } else if (type == TARG) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_targ,
           truncate_path(path_to_targ_tarball[0], 3),
           truncate_path(path_to_targ_tarball[1], 3),
           truncate_path(path_to_targ_tarball[2], 3),
           truncate_path(path_to_targ_tarball[3], 3),
           truncate_path(path_to_targ_tarball[4], 3));
        blast_tmp_sprintf(var_name, "ALL_TARG_SWEEPS");
        setenv(var_name, path_to_all_targ, 1);
    } else if (type == IQ) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_iq,
           truncate_path(path_to_iq_tarball[0], 3),
           truncate_path(path_to_iq_tarball[1], 3),
           truncate_path(path_to_iq_tarball[2], 3),
           truncate_path(path_to_iq_tarball[3], 3),
           truncate_path(path_to_iq_tarball[4], 3));
        blast_tmp_sprintf(var_name, "ALL_IQ_DATA");
        setenv(var_name, path_to_all_iq, 1);
    } else if (type == NOISECOMP) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_noise_comp,
           truncate_path(roach_state_table[0].path_to_noise_comp, 3),
           truncate_path(roach_state_table[1].path_to_noise_comp, 3),
           truncate_path(roach_state_table[2].path_to_noise_comp, 3),
           truncate_path(roach_state_table[3].path_to_noise_comp, 3),
           truncate_path(roach_state_table[4].path_to_noise_comp, 3));
        blast_tmp_sprintf(var_name, "ALL_NOISE_COMP");
        setenv(var_name, path_to_all_noise_comp, 1);
    } else if (type == BB_FREQS) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_bb_freqs,
           truncate_path(roach_state_table[0].freqlist_path, 3),
           truncate_path(roach_state_table[1].freqlist_path, 3),
           truncate_path(roach_state_table[2].freqlist_path, 3),
           truncate_path(roach_state_table[3].freqlist_path, 3),
           truncate_path(roach_state_table[4].freqlist_path, 3));
        blast_tmp_sprintf(var_name, "ALL_BB_FREQS");
        setenv(var_name, path_to_all_bb_freqs, 1);
    } else if (type == DF) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_df,
           truncate_path(path_to_df_tarball[0], 3),
           truncate_path(path_to_df_tarball[1], 3),
           truncate_path(path_to_df_tarball[2], 3),
           truncate_path(path_to_df_tarball[3], 3),
           truncate_path(path_to_df_tarball[4], 3));
        blast_tmp_sprintf(var_name, "ALL_DF_DATA");
        setenv(var_name, path_to_all_df, 1);
    } else if (type == LAMP) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_lamp,
           truncate_path(path_to_lamp_tarball[0], 3),
           truncate_path(path_to_lamp_tarball[1], 3),
           truncate_path(path_to_lamp_tarball[2], 3),
           truncate_path(path_to_lamp_tarball[3], 3),
           truncate_path(path_to_lamp_tarball[4], 3));
        blast_tmp_sprintf(var_name, "ALL_LAMP_DATA");
        setenv(var_name, path_to_all_lamp, 1);
    } else if (type == DS_VNA) {
        blast_tmp_sprintf(tar_cmd, "tar -C %s -czvf %s %s %s %s %s %s",
           roach_root_path,
           path_to_all_dsvna,
           truncate_path(path_to_dsvna[0], 3),
           truncate_path(path_to_dsvna[1], 3),
           truncate_path(path_to_dsvna[2], 3),
           truncate_path(path_to_dsvna[3], 3),
           truncate_path(path_to_dsvna[4], 3));
        blast_tmp_sprintf(var_name, "ALL_DS_VNA");
        setenv(var_name, path_to_all_dsvna, 1);
    }
    blast_info("Creating sweep tarball: %s", tar_cmd);
    for (int i = 0; i < NUM_ROACHES; i++) {
        roach_state_table[i].is_compressing_data = 1;
    }
    pyblast_system(tar_cmd);
    blast_tmp_sprintf(echo_cmd, "echo $%s", var_name);
    pyblast_system(echo_cmd);
    for (int i = 0; i < NUM_ROACHES; i++) {
        roach_state_table[i].is_compressing_data = 0;
    }
    return 0;
}

int save_roach_dfs(roach_state_t* m_roach, double m_nsec)
{
    int retval = -1;
    char *file_out;
    // check for ref params
    if ((!m_roach->has_ref_params)) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        return retval;
    }
    if ((create_data_dir(m_roach, DF)) < 0) {
        return retval;
    }
    // allocate memory to hold timestreams before saving to file
    int npoints = round(m_nsec * (double)DAC_FREQ_RES) / N_AVG_DF;
    blast_info("ROACH%d, saving %d points %f sec", m_roach->which, npoints, m_nsec);
    int rows = m_roach->current_ntones;
    char *var_name;
    int cols = npoints;
    float *dfs[rows];
    for (int i = 0; i < rows; i++) {
         dfs[i] = (float *)calloc(cols, sizeof(float));
    }
    // Get I and Q vals from packets. Average NUM_AVG values
    // Store in comp_vals
    double comp_vals[m_roach->num_kids][2];
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    // m_roach->is_averaging = 1;
    double *I_sum = calloc(m_roach->num_kids, sizeof(double));
    double *Q_sum = calloc(m_roach->num_kids, sizeof(double));

    for (int i = 0; i < npoints; i++) {
        int count = 0;
        int data_error_counter = 0;
        while ((m_num_received < N_AVG_DF) && (data_error_counter < MAX_DATA_ERRORS)) {
            usleep(3000);
            if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
                m_num_received++;
                i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
                data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
                for (size_t chan = 0; chan < m_roach->num_kids; chan ++) {
                    I_sum[chan] +=  m_packet.Ival[chan];
                    Q_sum[chan] +=  m_packet.Qval[chan];
                }
                count++;
            } else {
                data_error_counter++;
            }
            m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
        }
        if (data_error_counter == MAX_DATA_ERRORS) {
            free(I_sum);
            free(Q_sum);
            return retval;
        }
        if (!count) count = 1;
        for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
            comp_vals[chan][0] = I_sum[chan] / count;
            comp_vals[chan][1] = Q_sum[chan] / count;
        }
        // calculate df for each channel
        for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
            double deltaI = comp_vals[chan][0] - m_roach->ref_vals[chan][0];
            double deltaQ = comp_vals[chan][1] - m_roach->ref_vals[chan][1];
            m_roach->df[chan] = ((m_roach->ref_grads[chan][0] * deltaI)
                      + (m_roach->ref_grads[chan][1] * deltaQ)) /
            (m_roach->ref_grads[chan][0]*m_roach->ref_grads[chan][0] +
            m_roach->ref_grads[chan][1]*m_roach->ref_grads[chan][1]);
            // if recenter_df is false, apply df_offset to each df value
            if (!CommandData.roach[m_roach->which - 1].recenter_df) {
                m_roach->df[chan] -= m_roach->df_offset[chan];
            }
            dfs[chan][i] = m_roach->df[chan];
            // blast_info("Chan = %zd", chan);
        }
        memset(I_sum, 0, m_roach->num_kids*sizeof(double));
        memset(Q_sum, 0, m_roach->num_kids*sizeof(double));
    }
    free(I_sum);
    free(Q_sum);
    for (size_t chan = 0; chan < m_roach->num_kids; chan++) {
        blast_tmp_sprintf(file_out, "%s/%zd.dat",
             m_roach->last_df_path, chan);
        // blast_info("Saving %s", file_out);
        FILE *fd = fopen(file_out, "wb");
        if (!fd) {
            blast_err("ROACH%d: Error opening %s for writing",
                     m_roach->which, file_out);
            for (int k = 0; k < rows; k++) {
                free(dfs[k]);
            }
            return retval;
        }
        for (int j = 0; j < npoints; j++) {
            // write a 4 byte I value
            fwrite(&dfs[chan][j], sizeof(dfs[chan][j]), 1, fd);
        }
        fclose(fd);
    }
    for (int k = 0; k < rows; k++) {
        free(dfs[k]);
    }
    // m_roach->is_averaging = 0;
    blast_tmp_sprintf(var_name, "R%d_LAST_DF_PATH", m_roach->which);
    setenv(var_name, path_to_df_tarball[m_roach->which - 1], 1);
    compress_data(m_roach, DF);
    return 0;
}

// saves timestreams for all channels
int save_all_timestreams(roach_state_t *m_roach, double m_nsec)
{
    int retval = -1;
    char *var_name;
    char *file_out;
    if (create_data_dir(m_roach, IQ)) {
        blast_info("ROACH%d, IQ data will be saved in %s",
                       m_roach->which, m_roach->last_iq_path);
    } else {
        blast_err("Could not create new directory");
        CommandData.roach[m_roach->which - 1].get_timestream = 0;
        return retval;
    }
    // allocate memory to hold timestreams before saving to file
    int npoints = round(m_nsec * (double)DAC_FREQ_RES);
    blast_info("ROACH%d, saving %d points %f sec", m_roach->which, npoints, m_nsec);
    int rows = m_roach->current_ntones;
    int cols = npoints;
    float *I[rows];
    float *Q[rows];
    for (int i = 0; i < rows; i++) {
         I[i] = (float *)calloc(cols, sizeof(float));
         Q[i] = (float *)calloc(cols, sizeof(float));
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
    // blast_info("current ntones = %zd", m_roach->current_ntones);
    for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
        // blast_info("Chan = %zd", chan);
        blast_tmp_sprintf(file_out, "%s/%zd.dat", m_roach->last_iq_path, chan);
        // blast_info("Saving %s", file_out);
        FILE *fd = fopen(file_out, "wb");
        if (!fd) {
            blast_err("Error opening %s for writing", file_out);
            for (int i = 0; i < rows; i++) {
                free(I[i]);
                free(Q[i]);
            }
            return retval;
        }
        for (int j = 0; j < npoints; j++) {
            // write a 4 byte I value
            fwrite(&I[chan][j], sizeof(I[chan][j]), 1, fd);
            // write a 4 byte Q value
            fwrite(&Q[chan][j], sizeof(Q[chan][j]), 1, fd);
        }
        fclose(fd);
    }
    // Save last timestream path
    // pyblast_system("python /home/fc1user/sam_builds/chop_list.py");
    blast_tmp_sprintf(var_name, "R%d_LAST_IQ_PATH", m_roach->which);
    setenv(var_name, path_to_iq_tarball[m_roach->which - 1], 1);
    compress_data(m_roach, IQ);
    blast_info("ROACH%d, timestream saved", m_roach->which);
    for (int i = 0; i < rows; i++) {
        free(I[i]);
        free(Q[i]);
    }
    CommandData.roach[m_roach->which - 1].get_timestream = 0;
    retval = 0;
    return retval;
}

// get average IQ vals for each channel
// Call this function when the Roach which has lamp control
// is ready to pulse lamp
int avg_chan_vals(roach_state_t *m_roach, bool lamp_on)
{
    int retval = -1;
    if (lamp_on) {
        if (!CommandData.roach[m_roach->which - 1].has_lamp_control) {
            // wait for lead_roach_ready = 1
            int wait_counter = 0;
            while ((!lead_roach_ready) && (wait_counter < MAX_LAMP_WAITS)) {
                usleep(1000);
                if (wait_counter == MAX_LAMP_WAITS) {
                    return retval;
                }
                wait_counter++;
            }
        }
    }
    if (lamp_on) {
        memset(m_roach->I_on, 0, MAX_CHANNELS_PER_ROACH);
        memset(m_roach->Q_on, 0, MAX_CHANNELS_PER_ROACH);
        memset(m_roach->df_on, 0, MAX_CHANNELS_PER_ROACH);
    } else {
        memset(m_roach->I_off, 0, MAX_CHANNELS_PER_ROACH);
        memset(m_roach->Q_off, 0, MAX_CHANNELS_PER_ROACH);
        memset(m_roach->df_off, 0, MAX_CHANNELS_PER_ROACH);
    }
    float nsec = CommandData.roach_params[m_roach->which - 1].num_sec;
    int npoints = (int)round(nsec * (double)DAC_FREQ_RES);
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    // average npoints and put in m_buffer
    // Array to store I values to be summed
    double *I_sum = calloc(m_roach->current_ntones, sizeof(double));
    // Array to store Q values to be summed
    double *Q_sum = calloc(m_roach->current_ntones, sizeof(double));
    blast_info("ROACH%d, getting %u points %f sec", m_roach->which, npoints, nsec);
    // m_roach->is_averaging = 1;
    int count = 0;
    for (int i = 0; i < npoints; i++) {
        usleep(3000);
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
            for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
                I_sum[chan] += m_packet.Ival[chan];
                Q_sum[chan] += m_packet.Qval[chan];
                /* if (m_roach->which == 1 && chan == 329) {
                    blast_info("%zd, %g, %g", chan, m_packet.Ival[chan], m_packet.Qval[chan]);
                }*/
            }
            count++;
        }
    }
    // m_roach->is_averaging = 0;
    if (!count) count = 1;
    if (lamp_on) {
        for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
            m_roach->I_on[chan] = (I_sum[chan] / count);
            m_roach->Q_on[chan] = (Q_sum[chan] / count);
                /*if (m_roach->which == 1 && chan == 329) {
                    blast_info("Isum %f, Qsum %f, Iavg %f, Qavg %f",
                       I_sum[chan], Q_sum[chan],
                       m_roach->I_on[chan], m_roach->Q_on[chan]);
                }*/
            double deltaI = m_roach->I_on[chan] - m_roach->ref_vals[chan][0];
            double deltaQ = m_roach->Q_on[chan] - m_roach->ref_vals[chan][1];
            m_roach->df_on[chan] = ((m_roach->ref_grads[chan][0] * deltaI)
                           + (m_roach->ref_grads[chan][1] * deltaQ)) /
                    (m_roach->ref_grads[chan][0]*m_roach->ref_grads[chan][0] +
                              m_roach->ref_grads[chan][1]*m_roach->ref_grads[chan][1]);
            m_roach->df_on[chan] -= m_roach->df_offset[chan];
        }
    } else {
        for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
            m_roach->I_off[chan] = (I_sum[chan] / count);
            m_roach->Q_off[chan] = (Q_sum[chan] / count);
                /*if (m_roach->which == 1 && chan == 329) {
                    blast_info("Isum %f, Qsum %f, Iavg %f, Qavg %f",
                       I_sum[chan], Q_sum[chan],
                       m_roach->I_off[chan], m_roach->Q_off[chan]);
                }*/
            double deltaI = m_roach->I_off[chan] - m_roach->ref_vals[chan][0];
            double deltaQ = m_roach->Q_off[chan] - m_roach->ref_vals[chan][1];
            m_roach->df_off[chan] = ((m_roach->ref_grads[chan][0] * deltaI)
                       + (m_roach->ref_grads[chan][1] * deltaQ)) /
                    (m_roach->ref_grads[chan][0]*m_roach->ref_grads[chan][0] +
                              m_roach->ref_grads[chan][1]*m_roach->ref_grads[chan][1]);
            m_roach->df_off[chan] -= m_roach->df_offset[chan];
        }
    }
    blast_info("Data acquired...");
    // free calloc'd buffers
    free(I_sum);
    free(Q_sum);
    CommandData.roach[m_roach->which - 1].get_timestream = 0;
    m_roach->waiting_for_lamp = 0;
    return 0;
}

/*
void check_chop_data(roach_state_t *m_roach)
{
    char script_path[] = "/data/etc/blast/roachPython/chopSnrs.py";
    char save_path[] = "/home/fc1user/sam_tests";
    char *py_command;
    blast_tmp_sprintf(py_command, "python %s > %s %s", script_path,
              m_roach->last_iq_path, save_path);
    // pyblast_system(py_command);
} */

/*
// get the chop_snr for the timestream saved in save_timestream
int chop_snr(roach_state_t *m_roach, double *m_buffer)
{
    int retval = -1;
    char chop_snr_log[] = "/home/fc1user/sam_tests/chop_snr.log";
    char *py_command;
    blast_tmp_sprintf(py_command, "python %s > %s %d", chop_snr_script,
              chop_snr_log, m_roach->which);
    // Call fit_mcp_chop.py and read response from log
    pyblast_system(py_command);
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
}*/

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
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        return retval;
    }
    CommandData.roach[m_roach->which - 1].change_tone_amps = 0;
    /* if ((roach_save_1D_file(m_roach, amps_path, amps, m_roach->num_kids) < 0)) {
        return retval;
    } */
    retval = 0;
    return retval;
}

int shift_tone_amps(roach_state_t *m_roach, double *m_delta_amps)
{
    int retval = -1;
    if (!m_roach->last_amps) {
        m_roach->last_amps = calloc(m_roach->num_kids, sizeof(double));
    }
    for (size_t i = 0; i < m_roach->num_kids; i++) {
        m_roach->last_amps[i] += m_delta_amps[i];
    }
    // save last amplitudes list
    if ((roach_save_1D_file(m_roach, m_roach->targ_amps_path[2], m_roach->last_amps, m_roach->current_ntones) < 0)) {
        return retval;
    }
    CommandData.roach[m_roach->which - 1].change_tone_amps = 1;
    blast_info("ROACH%d writing new tone amps", m_roach->which);
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        return retval;
    }
    CommandData.roach[m_roach->which - 1].change_tone_amps = 0;
    retval = 0;
    return retval;
}

// shift amplitude of single tone
int shift_tone_amp(roach_state_t *m_roach)
{
    int retval;
    int chan = CommandData.roach[m_roach->which - 1].chan;
    double delta_amp = CommandData.roach_params[m_roach->which - 1].delta_amp;
    if (!m_roach->last_amps) {
        m_roach->last_amps = calloc(m_roach->num_kids, sizeof(double));
    }
    m_roach->last_amps[chan] += delta_amp;
    blast_info("ROACH%d, Shifing chan %d amp by %g", m_roach->which, chan, delta_amp);
    retval = roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
    return retval;
}

// shift phase of single tone
int shift_tone_phase(roach_state_t *m_roach)
{
    int retval;
    int chan = CommandData.roach[m_roach->which - 1].chan;
    double delta_phase = CommandData.roach_params[m_roach->which - 1].delta_phase;
    if (!m_roach->last_phases) {
        m_roach->last_phases = calloc(m_roach->num_kids, sizeof(double));
    }
    m_roach->last_phases[chan] += delta_phase;
    retval = roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
    blast_info("ROACH%d, Shifing chan %d phase by %g", m_roach->which, chan, delta_phase);
    return retval;
}

// shift freq of single tone
int shift_tone_freq(roach_state_t *m_roach)
{
    int retval;
    int chan = CommandData.roach[m_roach->which - 1].chan;
    double freq_offset = CommandData.roach_params[m_roach->which - 1].freq_offset;
    if (!m_roach->last_freqs) {
        m_roach->last_freqs = calloc(m_roach->num_kids, sizeof(double));
    }
    m_roach->last_freqs[chan] += freq_offset;
    retval = roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
    blast_info("ROACH%d, Shifing chan %d freq by %g", m_roach->which, chan, freq_offset);
    return retval;
}

/*
int optimize_amps(roach_state_t *m_roach)
{
    int retval = -1;
    char *py_command;
    blast_info("ROACH%d, Calculating nonlinearity params and new tone amps", m_roach->which);
    blast_tmp_sprintf(py_command, "python %s %s", cal_amps_script, m_roach->last_cal_path);
    blast_info("Command: %s", py_command);
    pyblast_system(py_command);
    double amps[m_roach->num_kids];
    blast_info("Roach%d, Loading new amps", m_roach->which);
    if ((roach_read_1D_file(m_roach, m_roach->last_cal_path, amps, m_roach->num_kids) < 0)) {
        free(m_roach->last_cal_path);
        return retval;
    }
    if (!m_roach->last_amps) {
        m_roach->last_amps = calloc(m_roach->num_kids, sizeof(double));
    }
    for (size_t i = 0; i < m_roach->num_kids; i++) {
        m_roach->last_amps[i] = amps[i];
    }
    CommandData.roach[m_roach->which - 1].change_tone_amps = 1;
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        free(m_roach->last_cal_path);
        return retval;
    }
    CommandData.roach[m_roach->which - 1].change_tone_amps = 0;
    free(m_roach->last_cal_path);
    retval = 0;
    return retval;
}
*/

void cal_lamp_off(roach_state_t *m_roach)
{
    if (!CommandData.roach[m_roach->which - 1].has_lamp_control) {
        return;
    }
    CommandData.Cryo.num_pulse = 1;
    CommandData.Cryo.separation = 2;
    CommandData.Cryo.length = 2;
    periodic_cal_control();
    CommandData.Cryo.periodic_pulse = 0;
}

// flash cal lamp, separation = length
void cal_pulses(roach_state_t *m_roach, float nsec, int num_pulse)
{
    if (!CommandData.enable_roach_lamp) {
        return;
    }
    m_roach->waiting_for_lamp = 1;
    if (!CommandData.roach[m_roach->which - 1].has_lamp_control) {
        return;
    } else {
        CommandData.cal_lamp_roach_hold = 1;
        lead_roach_ready = 1;
        // Lead Roach waits for other Roaches to be ready to save
        int wait_to_pulse = 1;
        for (int t = 0; (t < SWEEP_READY_TIMEOUT) && (wait_to_pulse); t++) {
            wait_to_pulse = 0;
            usleep(1000);
            for (int i = 0; i < NUM_ROACHES; i++) {
                if (roach_state_table[i].waiting_for_lamp) {
                    wait_to_pulse = 1;
                    break;
                }
            }
       }
    }
    CommandData.Cryo.periodic_pulse = 1;
    CommandData.Cryo.num_pulse = 1;
    CommandData.Cryo.separation = (int)(200 * nsec);
    CommandData.Cryo.length = (int)(200 * nsec);
    roach_lamp_now = 1;
    periodic_cal_control();
    roach_lamp_now = 0;
    usleep(50000);
    if (CommandData.roach[m_roach->which - 1].has_lamp_control) {
        lead_roach_ready = 0;
    }
    CommandData.cal_lamp_roach_hold = 0;
}

// Compare diff in I and Q of each channel with
// lamp on and lamp off. Store 2D array of diff values
int get_lamp_response(roach_state_t *m_roach)
{
    blast_info("ROACH%d: Checking response to cal lamp", m_roach->which);
    int retval = -1;
    char *file_out;
    // get 'off' timestream (save in buffer)
    avg_chan_vals(m_roach, 0);
    // chop the lamp
    float lamp_sec = CommandData.roach_params[m_roach->which - 1].num_sec;
    // If Roach doesn't have cal lamp (has_lamp_control), cal_pulses does nothing
    cal_pulses(m_roach, lamp_sec, 1);
    // get 'on' timestream (save in buffer)
    avg_chan_vals(m_roach, 1);
    // stop the lamp
    // cal_lamp_off(m_roach);
    // get diff between two channel arrays
    blast_tmp_sprintf(file_out, "%s/lamp_response.dat", m_roach->sweep_root_path);
    FILE *fd = fopen(file_out, "wb");
    if (!fd) {
        blast_err("Error opening %s for writing", file_out);
        return retval;
    }
    char *var_name;
    blast_tmp_sprintf(var_name, "R%d_LAST_LAMP_DATA", m_roach->which);
    setenv(var_name, path_to_lamp_tarball[m_roach->which - 1], 1);
    compress_data(m_roach, LAMP);
    double df_diff_sum = 0;
    for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
        m_roach->I_diff[chan] = m_roach->I_on[chan] - m_roach->I_off[chan];
        m_roach->Q_diff[chan] = m_roach->Q_on[chan] - m_roach->Q_off[chan];
        m_roach->df_diff[chan] = m_roach->df_on[chan] - m_roach->df_off[chan];
        df_diff_sum += m_roach->df_diff[chan];
        // blast_info("**** ROACH%d, chan %zd df_diff %g,", m_roach->which, chan, m_roach->df_diff[chan]);
        fwrite(&m_roach->I_diff[chan], sizeof(m_roach->I_diff[chan]), 1, fd);
        fwrite(&m_roach->Q_diff[chan], sizeof(m_roach->Q_diff[chan]), 1, fd);
        fwrite(&m_roach->df_diff[chan], sizeof(m_roach->df_diff[chan]), 1, fd);
    }
    m_roach->avg_df_diff = df_diff_sum/m_roach->current_ntones;
    fclose(fd);
    return 0;
}

/* Function: cal_sweep
 * ----------------------------
 * Performs a calibration sweep
 * Sweep data is saved like VNA/TARG sweeps
 *
 * @param m_roach roach state table
*/
/*
int cal_sweep(roach_state_t *m_roach, char *subdir)
{
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 1;
    // int npoints = CommandData.roach_params[m_roach->which - 1].npoints;
    int npoints = NCAL_POINTS;
    blast_info("NPOINTS = %d", npoints);
    struct stat dir_stat;
    int stat_return;
    char *lo_command;
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
        if (create_data_dir(m_roach, CAL_AMPS)) {
            blast_info("ROACH%d, CAL sweeps will be saved in %s",
                           m_roach->which, m_roach->last_cal_path);
        // Save bb_targ_freqs.dat in CAL dir
        blast_tmp_sprintf(save_bbfreqs_command, "cp %s/roach%d/bb_targ_freqs.dat %s",
                        roach_root_path, m_roach->which, m_roach->last_cal_path);
        pyblast_system(save_bbfreqs_command);
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
                blast_tmp_sprintf(lo_command, "%g",
                   m_sweep_freqs[i]/1.0e6);
            m_roach->lo_freq_req = m_sweep_freqs[i]/1.0e6;
            if (setLO_oneshot(m_roach->which - 1, m_roach->lo_freq_req) < 0) {
                blast_err("ROACH%d LO error", m_roach->which);
                return SWEEP_FAIL;
            }
            usleep(SWEEP_TIMEOUT);
            if (roach_save_sweep_packet(m_roach, (uint32_t)m_sweep_freqs[i], save_path, m_roach->num_kids) < 0) {
                return SWEEP_FAIL;
            }
        } else {
            blast_info("Sweep interrupted by command");
            CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
            return SWEEP_INTERRUPT;
        }
    }
    if (recenter_lo(m_roach) < 0) {
        blast_info("ROACH%d, Error recentering LO", m_roach->which);
    }
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
    return SWEEP_SUCCESS;
}*/

/* Function: roach_read_adc
 * ----------------------------
 * Reads back values from ADC snap buffer (in FW) to calculate Vrms of digitized freq comb
 * Attenuators are set to maximize Vrms
 *
 * @param m_roach roach state table
 *
 * returns: RMS voltage of digitized freq comb
*/
float *roach_read_adc(roach_state_t *m_roach)
{
    size_t buffer_len = (1<<12);
    uint16_t *temp_data;
    // char* filename;
    float *rms = calloc(2, sizeof(float));
    float irms, qrms, ival, qval, isum, qsum;
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
    // blast_tmp_sprintf(filename, "%s/adc_vals.dat", m_roach->sweep_root_path);
    // FILE *fd = fopen(filename, "w");
    roach_buffer_htons((uint16_t*)temp_data, buffer_len);
    isum = 0;
    qsum = 0;
    for (size_t i = 0; i < buffer_len; i++) {
        if (i % 2 == 0) {
            ival = (float)((int16_t)temp_data[i]);
            ival *= 550.;
            ival /= pow(2, 15);
            // fprintf(fd, "%f\n", ival);
            isum += pow(ival, 2);
        } else {
            qval = (float)((int16_t)temp_data[i]);
            qval *= 550.;
            qval /= pow(2, 15);
            // fprintf(fd, "%f\n", qval);
            qsum += pow(qval, 2);
        }
    }
    // fclose(fd);
    irms = sqrt(isum / ((float)buffer_len / 2.));
    qrms = sqrt(qsum / ((float)buffer_len / 2.));
    rms[0] = irms;
    rms[1] = qrms;
    free(temp_data);
    // blast_info("ROACH%d, ADC V_rms (I,Q) = %f %f\n", m_roach->which, rms[0], rms[0]);
    return rms;
}

void get_adc_rms(roach_state_t *m_roach)
{
    float *rms;
    rms = roach_read_adc(m_roach);
    m_roach->adc_rms[0] = rms[0];
    m_roach->adc_rms[1] = rms[1];
    blast_info("ROACH%d, ADC V_rms (I,Q) = %f %f\n", m_roach->which, rms[0], rms[1]);
}

int read_accum_snap(roach_state_t *m_roach)
{
    int retval = -1;
    size_t buffer_len = (1<<16);
    uint32_t *temp_data;
    temp_data = calloc((uint32_t)buffer_len, sizeof(uint32_t));
    char *filename;
    float Is[MAX_CHANNELS_PER_ROACH];
    float Qs[MAX_CHANNELS_PER_ROACH];
    if ((roach_write_int(m_roach, "accum_snap_accum_snap_ctrl", 0, 0)) < 0) {
        return retval;
    }
    if ((roach_write_int(m_roach, "accum_snap_accum_snap_ctrl", 1, 0)) < 0) {
        return retval;
    }
    if ((roach_read_data(m_roach, (uint8_t*)temp_data,
           "accum_snap_accum_snap_bram", 0, buffer_len * sizeof(uint32_t), READ_DATA_MS_TIMEOUT) != 0)) {
        return retval;
    }
    roach_buffer_htonl(temp_data, buffer_len);
    int count = 0;
    for (size_t i = 0; i < buffer_len; i++) {
        if (i % 2 == 0) {
            Is[count] = (float)temp_data[i];
        } else {
            Qs[count] = (float)temp_data[i];
        }
        count++;
    }
    blast_tmp_sprintf(filename, "%s/accumulator.dat", m_roach->sweep_root_path);
    FILE *fd = fopen(filename, "w");
    if (!fd) {
        blast_strerror("Could not open %s for writing", filename);
        return retval;
    }
    for (size_t i = 0; i < MAX_CHANNELS_PER_ROACH; i++) {
        fprintf(fd, "%f\t%f\n", Is[i], Qs[i]);
    }
    fclose(fd);
    retval = 0;
    return retval;
}

int roach_targ_sweep(roach_state_t *m_roach)
{
    int status = -1;
    char* var_name;
    if (m_roach->has_vna_tones) {
        blast_err("ROACH%d: Search comb is loaded. Must write TARG tones before TARG sweep", m_roach->which);
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        return status;
    }
    if (CommandData.roach[m_roach->which - 1].do_sweeps != 2) {
        blast_info("ROACH%d: Can't do target sweep- do sweeps not set", m_roach->which);
        return status;
    }
    m_roach->is_sweeping = 2;
    blast_info("ROACH%d, STARTING TARG sweep", m_roach->which);
    status = roach_do_sweep(m_roach, TARG);
    if ((status == SWEEP_SUCCESS)) {
        blast_info("ROACH%d, TARG sweep complete", m_roach->which);
        // creates file for downlinking sweep path to local machine
        // pyblast_system("python /home/fc1user/sam_builds/sweep_list.py targ");
        // save params for delta f calculation
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        save_ref_params(m_roach);
        // check lamp response
        get_adc_rms(m_roach);
        // write environment variable linking to last sweep
        blast_tmp_sprintf(var_name, "R%d_LAST_TARG_SWEEP", m_roach->which);
        // blast_tmp_sprintf(echo_command, "echo $%s", var_name);
        setenv(var_name, path_to_targ_tarball[m_roach->which - 1], 1);
        compress_data(m_roach, TARG);
        m_roach->is_sweeping = 0;
        m_roach->sweep_fail = 0;
        m_roach->has_targ_sweep = 1;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        center_df(m_roach);
        return status;
    /*} else if ((retval == SWEEP_INTERRUPT)) {
        m_roach->has_targ_sweep = 0;
        blast_info("ROACH%d, TARG sweep interrupted by blastcmd", m_roach->which);*/
    } else if (status == SWEEP_INTERRUPT) {
        blast_info("ROACH%d, TARG sweep interrupted by blastcmd", m_roach->which);
        m_roach->sweep_fail = 1;
        m_roach->is_sweeping = 0;
        m_roach->has_targ_sweep = 0;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        return status;
    } else if (status == SWEEP_FAIL) {
        blast_info("ROACH%d, TARG sweep failed", m_roach->which);
        m_roach->sweep_fail = 1;
        m_roach->is_sweeping = 0;
        m_roach->has_targ_sweep = 0;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        return status;
    }
    return 0;
}

int roach_df_targ_sweeps(roach_state_t *m_roach)
{
    int retval = -1;
    char *pycommand;
    char *ref_sweep;
    char *new_sweep;
    char *path_to_sweep_dfs;
    char *var_name;
    // original sweep
    if (!m_roach->last_targ_path) {
        if ((load_last_sweep_path(m_roach, TARG) < 0)) {
            return retval;
        }
    } else {
        ref_sweep = m_roach->last_targ_path;
    }
    // new_sweep = m_roach->last_targ_path;
    // do new sweep
    CommandData.roach[m_roach->which - 1].do_sweeps = 2;
    if (roach_targ_sweep(m_roach) < 0) {
        m_roach->sweep_fail = 1;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        return retval;
    }
    CommandData.roach[m_roach->which - 1].do_sweeps = 0;
    new_sweep = m_roach->last_targ_path;
    blast_tmp_sprintf(pycommand, "python %s %s %s %s", df_from_sweeps_script,
          ref_sweep, new_sweep, m_roach->sweep_root_path);
    blast_info("%s", pycommand);
    pyblast_system(pycommand);
    blast_tmp_sprintf(path_to_sweep_dfs, "%s/sweep_df.dat", m_roach->sweep_root_path);
    if ((roach_read_1D_file(m_roach, path_to_sweep_dfs, m_roach->sweep_df, m_roach->num_kids) < 0)) {
        return retval;
    }
    blast_tmp_sprintf(var_name, "R%d_SWEEP_DF_LIST", m_roach->which);
    // blast_tmp_sprintf(echo_command, "echo $%s", var_name);
    setenv(var_name, path_to_sweep_dfs, 1);
    for (size_t i = 0; i < m_roach->num_kids; i++) {
        blast_info("chan %zd, df = %g", i, m_roach->sweep_df[i]);
    }
    return 0;
}

int roach_median_sweep_df(roach_state_t *m_roach)
{
    int retval = -1;
    char *pycommand;
    char *ref_sweep;
    char *new_sweep;
    char *path_to_median_df;
    // original sweep
    if (!m_roach->last_targ_path) {
        if ((load_last_sweep_path(m_roach, TARG) < 0)) {
            return retval;
        }
    } else {
        ref_sweep = m_roach->last_targ_path;
    }
    // new_sweep = m_roach->last_targ_path;
    // do new sweep
    CommandData.roach[m_roach->which - 1].do_sweeps = 2;
    if (roach_targ_sweep(m_roach) < 0) {
        m_roach->sweep_fail = 1;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        return retval;
    }
    CommandData.roach[m_roach->which - 1].do_sweeps = 0;
    new_sweep = m_roach->last_targ_path;
    blast_tmp_sprintf(pycommand, "python %s %s %s %s", median_df_script,
          ref_sweep, new_sweep, m_roach->sweep_root_path);
    blast_info("%s", pycommand);
    pyblast_system(pycommand);
    blast_tmp_sprintf(path_to_median_df, "%s/median_df.dat", m_roach->sweep_root_path);
    if ((roach_read_1D_file(m_roach, path_to_median_df, &m_roach->median_df, 1) < 0)) {
        return retval;
    }
    return 0;
}

int roach_noise_comp(roach_state_t *m_roach)
{
    int status = -1;
    char *pycommand;
    char *path_to_ts_on;
    char *path_to_ts_off;
    char *var_name;
    int i = m_roach->which - 1;
    // with output atten at current setting
    if ((status = save_all_timestreams(m_roach,
            CommandData.roach_params[i].num_sec)) < 0) {
        blast_err("ROACH%d: Error saving I/Q timestream", i + 1);
        return status;
    }
    blast_tmp_sprintf(path_to_ts_on, "%s", m_roach->last_iq_path);
    // output atten at max
    CommandData.roach_params[i].set_out_atten = 30.0;
    CommandData.roach[i].set_attens = 1;
    if ((status = set_atten(&pi_state_table[i])) < 0) {
        blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
        CommandData.roach[i].set_attens = 0;
        return status;
    }
    if ((status = save_all_timestreams(m_roach,
            CommandData.roach_params[i].num_sec)) < 0) {
        blast_err("ROACH%d: Error saving I/Q timestream", i + 1);
        return status;
    }
    CommandData.roach[i].set_attens = 5;
    if ((status = set_attens_targ_output(m_roach)) < 0) {
        blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
        CommandData.roach[i].set_attens = 0;
        return status;
    }
    CommandData.roach[i].set_attens = 0;
    blast_tmp_sprintf(path_to_ts_off, "%s", m_roach->last_iq_path);
    blast_tmp_sprintf(pycommand, "python %s %s %s %s &", noise_comp_script,
         path_to_ts_on, path_to_ts_off, m_roach->sweep_root_path);
    blast_info("%s", pycommand);
    pyblast_system(pycommand);
    blast_tmp_sprintf(var_name, "R%d_LAST_NOISE_COMP", m_roach->which);
    // blast_tmp_sprintf(echo_command, "echo $%s", var_name);
    setenv(var_name, m_roach->path_to_noise_comp, 1);
    return 0;
}

int roach_df_comp(roach_state_t *m_roach)
{
    int status = -1;
    char *pycommand;
    char *path_to_df_on;
    char *path_to_df_off;
    char *var_name;
    int i = m_roach->which - 1;
    // with output atten at current setting
    if ((status = save_roach_dfs(m_roach,
            CommandData.roach_params[i].num_sec)) < 0) {
        blast_err("ROACH%d: Error saving df timestream", i + 1);
        return status;
    }
    blast_tmp_sprintf(path_to_df_on, "%s", m_roach->last_df_path);
    // output atten at max
    CommandData.roach_params[i].set_out_atten = 30.0;
    CommandData.roach[i].set_attens = 1;
    if ((status = set_atten(&pi_state_table[i])) < 0) {
        blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
        CommandData.roach[i].set_attens = 0;
        return status;
    }
    if ((status = save_roach_dfs(m_roach,
            CommandData.roach_params[i].num_sec)) < 0) {
        blast_err("ROACH%d: Error saving df timestream", i + 1);
        return status;
    }
    CommandData.roach[i].set_attens = 5;
    if ((status = set_attens_targ_output(m_roach)) < 0) {
        blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
        CommandData.roach[i].set_attens = 0;
        return status;
    }
    CommandData.roach[i].set_attens = 0;
    blast_tmp_sprintf(path_to_df_off, "%s", m_roach->last_df_path);
    blast_tmp_sprintf(pycommand, "python %s %s %s %s &", df_comp_script,
         path_to_df_on, path_to_df_off, m_roach->sweep_root_path);
    blast_info("%s", pycommand);
    pyblast_system(pycommand);
    blast_tmp_sprintf(var_name, "R%d_LAST_DF_COMP", m_roach->which);
    // blast_tmp_sprintf(echo_command, "echo $%s", var_name);
    setenv(var_name, m_roach->path_to_df_comp, 1);
    return 0;
}

int roach_refit_freqs(roach_state_t *m_roach, int m_on_res)
{
    int retval = -1;
    char *py_command;
    char *copy_command;
    char *load_path;
    double new_freqs[m_roach->num_kids];
    /* if (!m_roach->last_targ_path) {
        load_last_sweep_path(m_roach, TARG);
    } */
    if (!CommandData.roach[m_roach->which - 1].refit_res_freqs) {
        return retval;
    }
    CommandData.roach[m_roach->which - 1].do_sweeps = 2;
    if (roach_targ_sweep(m_roach) < 0) {
        m_roach->sweep_fail = 1;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        return retval;
    }
    blast_info("ROACH%d, Fit sweep complete, calculating new freqs", m_roach->which);
    blast_tmp_sprintf(py_command, "python %s %s %d %d", refit_freqs_script,
      m_roach->last_targ_path, m_roach->which, m_on_res);
    blast_info("Command: %s", py_command);
    pyblast_system(py_command);
    // system(py_command);
    sleep(5);
    if (m_on_res) {
        blast_tmp_sprintf(load_path, "%s/new_res_freqs.dat", m_roach->last_targ_path);
    } else {
        blast_tmp_sprintf(load_path, "%s/grad_freqs.dat", m_roach->last_targ_path);
    }
    if ((roach_read_1D_file(m_roach, load_path, new_freqs, m_roach->num_kids) < 0)) {
        return retval;
    }
    if (APPLY_TARG_TRF) {
        CommandData.roach[m_roach->which - 1].load_targ_amps = 2;
        blast_info("ROACH%d, Applying targ trf correction", m_roach->which);
    }
    for (size_t i = 0; i < m_roach->num_kids; i++) {
        m_roach->targ_tones[i] = new_freqs[i];
        // blast_info("new freq = %g", new_freqs[i]);
    }
    blast_tmp_sprintf(copy_command, "cp %s %s/bb_targ_freqs.dat",
            load_path, m_roach->sweep_root_path);
    pyblast_system(copy_command);
    // system(copy_command);
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        return retval;
    }
    /*
    // if Roach has df ref params, recenter df
    if (m_roach->has_ref_params) {
        center_df(m_roach);
    } */
    // Do another targ sweep, which becomes the new reference sweep
    CommandData.roach[m_roach->which - 1].do_sweeps = 2;
    if (roach_targ_sweep(m_roach) < 0) {
        m_roach->sweep_fail = 1;
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        return retval;
    }
    CommandData.roach[m_roach->which - 1].do_sweeps = 0;
    CommandData.roach[m_roach->which -1].refit_res_freqs = 0;
    return 0;
}

/*
int cal_sweep_amps(roach_state_t *m_roach, double **sweep_buffer)
{
    if (!CommandData.roach[m_roach->which - 1].do_cal_sweeps) {
        return SWEEP_INTERRUPT;
    }
    pi_state_t *m_pi = &pi_state_table[m_roach->which - 1];
    char *lo_command;
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
*/
/*
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
                CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
                free(m_roach->last_cal_path);
                return SWEEP_INTERRUPT;
            } else {
                blast_info("ROACH%d, Cal sweep failed, will reattempt", m_roach->which);
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
}*/

/*
int cal_sweep_amps(roach_state_t *m_roach)
{
    double atten_step;
    uint32_t npoints, ncycles;
    atten_step = CommandData.roach_params[m_roach->which - 1].atten_step;
    ncycles = (uint32_t)CommandData.roach_params[m_roach->which - 1].ncycles;
    npoints = (uint32_t)CommandData.roach_params[m_roach->which - 1].npoints;
    blast_info("NCYCLES = %u", ncycles);
    blast_info("AMP STEP = %f", atten_step);
    blast_info("NPOINTS = %u", npoints);
    int count = 0;
    char *subdir;
    double new_amps[ncycles + 1];
    blast_info("ROACH%d, running tone amp cal for %d cycles", m_roach->which, ncycles);
    while (count < ncycles + 1) {
        blast_info("Count = %d", count);
        new_amps[count] = (double)START_AMP + count*(double)DELTA_AMP;
        blast_tmp_sprintf(subdir, "%4f", new_amps[count]);
        int status = cal_sweep(m_roach, subdir);
        // If sweep worked, offset tone amplitudes by DELTA_AMP and sweep again
        if (status == SWEEP_SUCCESS) {
            blast_info("ROACH%d, CAL sweep %d complete", m_roach->which, count);
            // shift_tone_amps(m_roach);
        }
        // If sweep interrupted or failed, write original tone amplitudes
        if ((status == SWEEP_INTERRUPT)) {
            blast_info("ROACH%d, Cal sweep interrupted by blastcmd, writing default amps",
                   m_roach->which);
            roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
            CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
            free(m_roach->last_cal_path);
            return SWEEP_INTERRUPT;
        }
        if ((status == SWEEP_FAIL)) {
            blast_info("ROACH%d, Cal sweep failed, writing default amps", m_roach->which);
            roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids);
            CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
            free(m_roach->last_cal_path);
            return SWEEP_FAIL;
        }
    count += 1;
    }
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 0;
    // free(m_roach->last_cal_path);
    // save list of test amps which were written
    char *new_amps_savepath;
    blast_tmp_sprintf(new_amps_savepath, "%s/%s", m_roach->last_cal_path, "new_amps.dat");
    if ((roach_save_1D_file(m_roach, m_roach->last_cal_path, new_amps, count) < 0)) {
        free(m_roach->last_cal_path);
        return SWEEP_FAIL;
    }
    return SWEEP_SUCCESS;
}*/

/*
// refit resonant frequency by doing a cal sweep and calling Python
void cal_fit_res(roach_state_t *m_roach, char *m_subdir)
{
    CommandData.roach[m_roach->which - 1].do_cal_sweeps = 1;
    CommandData.roach_params[m_roach->which - 1].ncycles = 1;
    CommandData.roach_params[m_roach->which - 1].npoints = 20;
    cal_sweep(m_roach, m_subdir);
    // fit res and rewrite tones
}*/

// Load data saved by chop_all, and average together to create master chop
int master_chop(roach_state_t *m_roach, double m_nsec)
{
    int retval = -1;
    char *file_in;
    struct stat dir_stat;
    int stat_return;
    // check to see if directory exists
    stat_return = stat(m_roach->last_iq_path, &dir_stat);
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
         I[i] = (float *)calloc(cols, sizeof(float));
         Q[i] = (float *)calloc(cols, sizeof(float));
    }
    for (size_t chan = 0; chan < m_roach->current_ntones; chan++) {
        blast_tmp_sprintf(file_in, "%s/%zd.dat", m_roach->last_iq_path, chan);
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

int nudge_amps(roach_state_t *m_roach, double *m_delta_amps, double (*m_chop_diff)[3])
{
    int retval = -1;
    for (int chan = 0; chan < m_roach->current_ntones; chan++) {
        // if a chan response is less than chop_targ, add to delta_amps
        if (m_chop_diff[chan][2] <= CommandData.roach_params[m_roach->which - 1].resp_thresh) {
            m_delta_amps[chan] += CommandData.roach_params[m_roach->which - 1].delta_amp;
            blast_info("ROACH%d chan %d will be adjusted", m_roach->which, chan);
        }
    }
    // write new tone amps
    if ((shift_tone_amps(m_roach, m_delta_amps)) < 0) {
        return retval;
    }
    retval = 0;
    return retval;
}

/*
// tune tone amplitudes to maximize I,Q response to cal lamp
int lamp_cal_amps(roach_state_t *m_roach)
{
    int retval = -1;
    double diff[m_roach->current_ntones][3];
    double delta_amps[m_roach->current_ntones];
    int count = 0;
    if (CommandData.roach[m_roach->which - 1].tune_amps) {
        int ncycles = CommandData.roach_params[m_roach->which - 1].ncycles;
        blast_info("ROACH%d: Running amp cal cycle for %u cycles", m_roach->which, ncycles);
        // start with this delta_amp for now
        // get baseline diff for each chan
        get_lamp_response(m_roach, diff);
        // shift the tone amps based on initial response
        if ((nudge_amps(m_roach, delta_amps, diff) < 0)) {
            return retval;
        }
        blast_info("count, ncycles: %d, %d", count, ncycles);
        while (count < ncycles) {
            blast_info("ROACH%d, starting amp tune", m_roach->which);
            // chop again
            get_lamp_response(m_roach, diff);
            if ((nudge_amps(m_roach, delta_amps, diff) < 0)) {
                return retval;
            } else {
                count += 1;
            }
        }
    }
    CommandData.roach[m_roach->which - 1].tune_amps = 0;
    retval = 0;
    return retval;
}*/

int calc_grad_freqs(roach_state_t *m_roach, char *m_targ_path)
{
    int retval = -1;
    if (!m_targ_path) {
        if ((load_last_sweep_path(m_roach, TARG) < 0)) {
            return retval;
        }
    }
    char *calc_freqs_command;
    char *m_targ_freq_path;
    double m_temp_freqs[MAX_CHANNELS_PER_ROACH];
    blast_tmp_sprintf(calc_freqs_command,
           "python /data/etc/blast/roachPython/calc_targ_freqs.py %s %g",
           m_targ_path,
           m_roach->lo_centerfreq);
    blast_info("%s", calc_freqs_command);
    pyblast_system(calc_freqs_command);
    // system(calc_freqs_command);
    sleep(6);
    blast_tmp_sprintf(m_targ_freq_path, "%s/gradient_freqs.dat", m_targ_path);
    blast_info("Opening gradient freqs for reading...");
    FILE *fd;
    fd = fopen(m_targ_freq_path, "r");
    if (!fd) {
        blast_strerror("Could not open %s for reading", m_targ_freq_path);
        return retval;
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
    retval = 0;
    return retval;
}

/*
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
}*/

int roach_df(roach_state_t* m_roach)
{
    int retval = -1;
    // check for ref params
    if ((!m_roach->has_ref_params)) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        return retval;
    }
    int chan = CommandData.roach[m_roach->which - 1].chan;
    // Store in comp_vals
    // Get I and Q vals from packets. Average NUM_AVG values
    // Store in comp_vals
    double comp_vals[2];
    int num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    double I_sum = 0;
    double Q_sum = 0;
    // m_roach->is_averaging = 1;
    int data_error_counter = 0;
    while ((num_received < N_AVG_DF) && (data_error_counter < MAX_DATA_ERRORS)) {
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            I_sum +=  m_packet.Ival[chan];
            Q_sum +=  m_packet.Qval[chan];
        } else {
            data_error_counter++;
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    if (data_error_counter == MAX_DATA_ERRORS) {
        return retval;
    }
    if (!num_received) num_received = 1;
    comp_vals[0] = I_sum / num_received;
    comp_vals[1] = Q_sum / num_received;
    // calculate df for each selected channel
    double deltaI = comp_vals[0] - m_roach->ref_vals[chan][0];
    double deltaQ = comp_vals[1] - m_roach->ref_vals[chan][1];
    m_roach->df[chan] = ((m_roach->ref_grads[chan][0] * deltaI) + (m_roach->ref_grads[chan][1] * deltaQ)) /
            (m_roach->ref_grads[chan][0]*m_roach->ref_grads[chan][0] +
                      m_roach->ref_grads[chan][1]*m_roach->ref_grads[chan][1]);
    if (!CommandData.roach[m_roach->which - 1].recenter_df) {
        m_roach->df[chan] -= m_roach->df_offset[chan];
    }
    blast_info("****** ROACH%d, chan %d df = %g", m_roach->which, chan, m_roach->df[chan]);
    /* blast_info("**** ROACH%d, chan %zd df = %g, Icomp_val %g, deltaI %g, Irefval %g, offset %g",
          m_roach->which,
          chan,
          m_roach->df[chan],
          comp_vals[0],
          deltaI,
          m_roach->ref_vals[chan][0],
          m_roach->df_offset[chan]);*/
    // m_roach->is_averaging = 0;
    return 0;
}

float roach_df_continuous(roach_df_calc_t* m_roach_df, float inew, float qnew, int i_roach, int i_kid)
{
    int i;
    double df;
    // static uint32_t i_ct = 0;
    // Check to make sure i_roach and i_kid are within range
    /* if ((i_roach >= NUM_ROACHES) || (i_kid >= MAX_CHANNELS_PER_ROACH)
                                 || (i_roach < 0) || (i_kid < 0)) {
        blast_info("i_roach = %d, i_kid = %d are out of range", i_roach, i_kid);
    }*/
    roach_state_t* m_roach = &(roach_state_table[i_roach]);
    if (m_roach_df->first_call) { // initialize structure
        for (i = 0; i < ROACH_DF_FILT_LEN; i++) m_roach_df->ibuf[i] = 0.0;
        for (i = 0; i < ROACH_DF_FILT_LEN; i++) m_roach_df->qbuf[i] = 0.0;
        m_roach_df->ind_last = 0;
        m_roach_df->i_sum = 0.0;
        m_roach_df->q_sum = 0.0;
        m_roach_df->first_call = 0;
    }
    int retval = -1.0;
    // check for ref params
    if ((!m_roach->has_ref_params)) {
//         if ((i_ct % ROACH_FILT_DEBUG_FREQ) == 0) {
//             blast_info("roach%d ikid%d The references aren't set...exiting roach_df_continuous",
//                        m_roach_df->ind_roach, m_roach_df->ind_kid);
//         }
//         // Don't try to calculate df until the references are set.
//         i_ct++;
        return(retval);
    }
    m_roach_df->i_sum = m_roach_df->i_sum - m_roach_df->ibuf[m_roach_df->ind_last] + inew;
    m_roach_df->q_sum = m_roach_df->q_sum - m_roach_df->qbuf[m_roach_df->ind_last] + qnew;
    m_roach_df->qbuf[m_roach_df->ind_last] = qnew;
    m_roach_df->ibuf[m_roach_df->ind_last] = inew;
    /* if ((i_ct % ROACH_FILT_DEBUG_FREQ) < 5) {
        blast_info("roach%d ikid%d i_sum = %f, i_cur = %f, ind_last = %f",
                   i_roach, i_kid,
                   m_roach_df->i_sum, m_roach_df->ibuf[m_roach_df->ind_last], inew);
        blast_info("roach%d ikid%d q_sum = %f, q_cur = %f, qbuf last = %f, new_ind = %d",
                   i_roach, i_kid,
                   m_roach_df->q_sum, m_roach_df->qbuf[m_roach_df->ind_last], qnew);
    }*/
    m_roach_df->ind_last = ((m_roach_df->ind_last) + 1) % ROACH_DF_FILT_LEN;
    /* if ((i_ct % ROACH_FILT_DEBUG_FREQ) < 5) {
         blast_info("roach%d ikid%d new_index = %d",
                    i_roach, i_kid, m_roach_df->ind_last);
    }*/
    // Store in comp_vals
    // Get I and Q vals from packets. Average NUM_AVG values
    // Store in comp_vals
    double comp_vals[2];
    comp_vals[0] = m_roach_df->i_sum / ROACH_DF_FILT_LEN;
    comp_vals[1] = m_roach_df->q_sum / ROACH_DF_FILT_LEN;
    // calculate df for each selected channel
    double deltaI = comp_vals[0] - m_roach->ref_vals[i_kid][0];
    double deltaQ = comp_vals[1] - m_roach->ref_vals[i_kid][1];
    df =  ((m_roach->ref_grads[i_kid][0] * deltaI) +
                     (m_roach->ref_grads[i_kid][1] * deltaQ)) /
                     (m_roach->ref_grads[i_kid][0]*m_roach->ref_grads[i_kid][0] +
                      m_roach->ref_grads[i_kid][1]*m_roach->ref_grads[i_kid][1]);
    if (!CommandData.roach[m_roach->which - 1].recenter_df) {
        df -= m_roach->df_offset[i_kid];
    }
    /* if ((i_ct % ROACH_FILT_DEBUG_FREQ) < 5) {
         blast_info("roach%d ikid%d comp_vals = %f %f ref_vals %f %f delta I Q %f %f ref_grads %f %f offset %f",
                    i_roach, i_kid, comp_vals[0], comp_vals[1],
                    m_roach->ref_vals[i_kid][0], m_roach->ref_vals[i_kid][1],
                    deltaI, deltaQ,
                    m_roach->ref_grads[i_kid][0], m_roach->ref_grads[i_kid][1],
                    m_roach->df_offset[i_kid]);
         blast_info("*************** ROACH%d, chan %d df = %g", i_roach+1,
                    i_kid, df);
    }
    i_ct++; */
    return(df);
}

int shift_freq(roach_state_t *m_roach)
{
    int retval = -1;
    if (!m_roach->has_targ_tones) {
        return retval;
    }
    int chan = CommandData.roach[m_roach->which - 1].chan;
    if ((roach_df(m_roach) < 0)) {
        return retval;
    }
    m_roach->targ_tones[chan] += m_roach->df[chan];
    blast_info("ROACH%d, chan%d += %g", m_roach->which, chan, m_roach->df[chan]);
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        return retval;
    }
    CommandData.roach[m_roach->which - 1].change_targ_freq = 0;
    retval = 0;
    return retval;
}

int shift_freqs(roach_state_t *m_roach)
{
    int retval = -1;
    if (!m_roach->has_targ_tones) {
        return retval;
    }
    if ((roach_dfs(m_roach) < 0)) {
        return retval;
    }
    for (int chan = 0; chan < m_roach->num_kids; chan++) {
        m_roach->targ_tones[chan] += m_roach->df[chan];
        blast_info("ROACH%d, chan%d += %g", m_roach->which, chan, m_roach->df[chan]);
    }
    if ((roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids) < 0)) {
        return retval;
    }
    CommandData.roach[m_roach->which - 1].change_targ_freq = 0;
    retval = 0;
    return retval;
}
// calculate delta f for channels selected by mole
/*
int roach_df_mole(roach_state_t* m_roach)
{
    int retval = -1;
    // check for ref params
    if ((!m_roach->has_ref_params)) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        return retval;
    }
    // allocate a buffer for df values
    int n_avg = 10;
    double *df_buffer = calloc(->n_avg, sizeof(double));
    int chan = CommandData.roach_tlm[m_roach->which - 1].kid;
    double comp_vals[2];
    int m_num_received = 0;
    int m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    uint8_t i_udp_read;
    double I_sum = 0;
    double Q_sum = 0;
    while (m_num_received < n_avg) {
        if (roach_udp[m_roach->which - 1].roach_valid_packet_count > m_last_valid_packet_count) {
            m_num_received++;
            i_udp_read = GETREADINDEX(roach_udp[m_roach->which - 1].index);
            data_udp_packet_t m_packet = roach_udp[m_roach->which - 1].last_pkts[i_udp_read];
            I_sum +=  m_packet.Ival[chan];
            Q_sum +=  m_packet.Qval[chan];
        }
        m_last_valid_packet_count = roach_udp[m_roach->which - 1].roach_valid_packet_count;
    }
    comp_vals[0] = I_sum / n_avg;
    comp_vals[1] = Q_sum / n_avg;
    // calculate df for each selected channel
    double deltaI = comp_vals[0] - m_roach->ref_vals[chan][0];
    double deltaQ = comp_vals[1] - m_roach->ref_vals[chan][1];
    m_roach->df[chan] = -1. * ((m_roach->ref_grads[chan][0] * deltaI) + (m_roach->ref_grads[chan][1] * deltaQ)) /
            (m_roach->ref_grads[chan][0]*m_roach->ref_grads[chan][0] +
                      m_roach->ref_grads[chan][1]*m_roach->ref_grads[chan][1]);
    retval = 0;
    return retval;
}
*/

int apply_freq_correction(roach_state_t *m_roach)
{
    int status = -1;
    if (!m_roach->has_targ_tones) {
        blast_err("ROACH%d: TARG TONES NOT WRITTEN. ABORTING RETUNE", m_roach->which);
        return status;
    }
    /* if ((status = roach_dfs(m_roach)) < 0) {
        return status;
    } */
    blast_info("ROACH%d: Applying freq corrections", m_roach->which);
    // blast_info("CHECK RETUNE = %d", CommandData.roach[m_roach->which - 1].do_check_retune);
    if (CommandData.roach[m_roach->which - 1].do_check_retune == 3) {
        for (int chan = 0; chan < m_roach->num_kids; chan++) {
            m_roach->targ_tones[chan] += m_roach->sweep_df[chan];
            blast_info("ROACH%d, chan%d += %g", m_roach->which, chan, m_roach->sweep_df[chan]);
        }
    } else if (CommandData.roach[m_roach->which - 1].do_check_retune == 1) {
        for (int chan = 0; chan < m_roach->num_kids; chan++) {
            m_roach->targ_tones[chan] += m_roach->df[chan];
            blast_info("ROACH%d, chan%d += %g", m_roach->which, chan, m_roach->df[chan]);
        }
    }
    if ((status = roach_write_tones(m_roach, m_roach->targ_tones, m_roach->num_kids)) < 0) {
        return status;
    }
    return 0;
}

/* Function: roach_check_df_retune
 * ----------------------------
 * Calculate df for each channel and compare to threshold to
 * determine whether or not to retune freqs.
 *
 * @param m_roach roach state table
 *
 * returns: m_roach->retune_flag
*/
static int roach_check_df_retune(roach_state_t *m_roach)
{
    int status;
    // check for ref params
    if ((status = m_roach->has_ref_params) < 1) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        return status;
    }
    CommandData.roach[m_roach->which - 1].do_df_calc = 1;
    blast_info("ROACH%d: Checking for retune...", m_roach->which);
    int nflags = 0;
    if (CommandData.roach[m_roach->which - 1].do_check_retune == 1) {
        // calculate df
        if ((status = roach_dfs(m_roach)) < 0) {
            blast_err("ROACH%d: Failed to calculate DF...", m_roach->which);
            CommandData.roach[m_roach->which - 1].do_df_calc = 0;
            return status;
        } else {
            for (int chan = 0; chan < m_roach->num_kids; ++chan) {
                if ((m_roach->df[chan] <
                    CommandData.roach_params[m_roach->which - 1].df_retune_threshold)) {
                    nflags++;
                    m_roach->out_of_range[chan] = 1;
                } else {
                    m_roach->out_of_range[chan] = 0;
                }
             }
        }
        m_roach->n_outofrange = nflags;
        blast_info("ROACH%d: %d KIDs have drifted", m_roach->which, nflags);
        if (nflags > CommandData.roach[m_roach->which - 1].n_outofrange_thresh) {
            m_roach->retune_flag = 1;
            // blast_info("ROACH%d: RETUNE RECOMMENDED", m_roach->which);
        } else {
            m_roach->retune_flag = 0;
        }
        if (CommandData.roach[m_roach->which - 1].auto_correct_freqs == 1) {
            if ((status = apply_freq_correction(m_roach)) < 0) {
                blast_err("ROACH%d: FAILED TO APPLY FREQ CORRECTION", m_roach->which);
                return status;
            }
        }
    }
    CommandData.roach[m_roach->which - 1].do_df_calc = 0;
    CommandData.roach[m_roach->which - 1].do_check_retune = 0;
    return 0;
}

static int roach_check_df_sweep_retune(roach_state_t *m_roach)
{
    int status;
    // check for ref params
    if ((status = m_roach->has_ref_params < 1)) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        return status;
    }
    blast_info("ROACH%d: Checking for retune with sweep method...", m_roach->which);
    int nflags = 0;
    if (CommandData.roach[m_roach->which - 1].do_check_retune == 3) {
        CommandData.roach[m_roach->which - 1].do_df_targ = 1;
        // calculate df
        if ((status = roach_df_targ_sweeps(m_roach)) < 0) {
            blast_err("ROACH%d: Error calculating DF from sweeps", m_roach->which);
            return status;
        }
        for (int chan = 0; chan < m_roach->num_kids; ++chan) {
            if ((m_roach->sweep_df[chan] >
             CommandData.roach_params[m_roach->which - 1].df_retune_threshold)) {
                nflags++;
                m_roach->out_of_range[chan] = 1;
            } else {
                m_roach->out_of_range[chan] = 0;
            }
        }
        m_roach->n_outofrange = nflags;
        blast_info("ROACH%d: %d KIDs have drifted", m_roach->which, nflags);
        if (nflags < CommandData.roach[m_roach->which - 1].n_outofrange_thresh) {
            m_roach->retune_flag = 1;
            // blast_info("ROACH%d: RETUNE RECOMMENDED", m_roach->which);
        } else {
            m_roach->retune_flag = 0;
        }
        if (CommandData.roach[m_roach->which - 1].auto_correct_freqs == 1) {
            if ((status = apply_freq_correction(m_roach)) < 0) {
                blast_err("ROACH%d: FAILED TO APPLY FREQ CORRECTION", m_roach->which);
                return status;
            }
        }
    }
    CommandData.roach[m_roach->which - 1].do_df_targ = 0;
    CommandData.roach[m_roach->which - 1].do_check_retune = 0;
    return 0;
}

static int roach_check_lamp_retune(roach_state_t *m_roach)
{
    int status = -1;
    int sweep_check = 1;
    for (int t = 0; (t < SWEEP_READY_TIMEOUT) && (sweep_check); t++) {
        sweep_check = 0;
        usleep(1000);
        for (int i = 0; i < NUM_ROACHES; i++) {
            if (roach_state_table[i].is_sweeping) {
                sweep_check = 1;
                break;
            }
        }
    }
    CommandData.cal_lamp_roach_hold = 1;
    // check for ref params
    if (!m_roach->has_ref_params) {
        blast_err("ROACH%d, No ref params found", m_roach->which);
        CommandData.cal_lamp_roach_hold = 0;
        return status;
    }
    int nflags = 0;
    if (CommandData.roach[m_roach->which - 1].do_check_retune == 2) {
        // lamp check
        if ((status = get_lamp_response(m_roach)) < 0) {
            blast_err("ROACH%d: Failed to get lamp response...", m_roach->which);
            CommandData.roach[m_roach->which - 1].check_response = 0;
            // m_roach->lamp_check_error = 1;
            CommandData.cal_lamp_roach_hold = 0;
            return status;
        } else {
            for (int chan = 0; chan < m_roach->num_kids; ++chan) {
                if ((m_roach->df_diff[chan] <
                   CommandData.roach_params[m_roach->which - 1].df_diff_retune_threshold)) {
                    nflags++;
                    m_roach->out_of_range[chan] = 1;
                } else {
                    m_roach->out_of_range[chan] = 0;
                }
             }
        }
        m_roach->n_outofrange = nflags;
        blast_info("ROACH%d: %d channels have drifted", m_roach->which, nflags);
        if (nflags < CommandData.roach[m_roach->which - 1].n_outofrange_thresh) {
            m_roach->retune_flag = 1;
            // blast_info("ROACH%d: RETUNE RECOMMENDED", m_roach->which);
        } else {
            m_roach->retune_flag = 0;
        }
    }
    CommandData.roach[m_roach->which - 1].check_response = 0;
    CommandData.cal_lamp_roach_hold = 0;
    return 0;
}

int roach_exec_retune(roach_state_t *m_roach)
{
    int status = -1;
    if (CommandData.roach[m_roach->which - 1].do_check_retune == 3) {
        if ((status = roach_check_df_sweep_retune(m_roach)) < 0) {
            return status;
        }
    } else if (CommandData.roach[m_roach->which - 1].do_check_retune == 2) {
        if ((status = roach_check_lamp_retune(m_roach)) < 0) {
            return status;
        }
    } else if (CommandData.roach[m_roach->which - 1].do_check_retune == 1) {
        if ((status = roach_check_df_retune(m_roach)) < 0) {
            return status;
        }
    } else if (CommandData.roach[m_roach->which - 1].do_check_retune == 4) {
        if ((status = roach_median_sweep_df(m_roach)) < 0) {
            return status;
        }
    }
    return 0;
}

int roach_turnaround_loop(roach_state_t *m_roach)
{
    int status = -1;
    int i = m_roach->which - 1;
    if ((!m_roach->has_targ_tones) | (!m_roach->has_ref_params)) {
        return status;
    }
    m_roach->doing_turnaround_loop = 1;
    // TARG/REFIT/TARG
    CommandData.roach[i].refit_res_freqs = 1;
    if ((status = roach_refit_freqs(m_roach, CommandData.roach[i].on_res)) < 0) {
        blast_err("ROACH%d: ERROR REFITTING FREQS", i + 1);
        CommandData.roach[i].refit_res_freqs = 0;
        m_roach->doing_turnaround_loop = 0;
        return status;
    }
    center_df(m_roach);
    m_roach->doing_turnaround_loop = 0;
    return 0;
}

/** Phenom/Callback functions to upload firmware **/
/**
 * If we have an error, we'll disable the socket and schedule a reconnection attempt.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
/*
static void firmware_upload_process_return(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    firmware_state_t *state = (firmware_state_t*) m_data;
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
}*/

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

/* static void firmware_upload_connected(ph_sock_t *m_sock, int m_status,
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
}*/

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
    KATCP_FLAG_LAST | KATCP_FLAG_STRING, "", NULL);
    if (success_val != KATCP_RESULT_OK) {
        return -1;
    } else {
        return 0;
    }
}

int roach_halt_ppc(roach_state_t *m_roach)
{
    blast_info("ROACH%d: Sending SHUTDOWN command. Restarting will require full power cycle",
       m_roach->which);
    int status = send_rpc_katcl(m_roach->rpc_conn, 1000,
        KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?halt",
    KATCP_FLAG_LAST | KATCP_FLAG_STRING, "", NULL);
    m_roach->has_firmware = 0;
    m_roach->is_streaming = 0;
    m_roach->halted = 1;
    return status;
}

/*
 * Function: roach_upload_fpg
 * -----------------------------
 * Uploads a firmware image file to the Roach
 *
 * @param m_roach roach state structure
 * @param m_filename name of firmware file
*/

/*
int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename)
{
    int retval = -1;
    srand48(time(NULL));
    firmware_state_t state = {
                              .firmware_file = m_filename,
                              .port = (uint16_t) (drand48() * 500.0 + 5000),
                              .timeout.tv_sec = 20,
                              .timeout.tv_usec = 0,
                              .roach = m_roach
    };
    int result = send_rpc_katcl(m_roach->rpc_conn, UPLOAD_TIMEOUT,
                       KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?progremote",
                       KATCP_FLAG_ULONG | KATCP_FLAG_LAST, state.port,
                       NULL);
    if (result != KATCP_RESULT_OK) {
        blast_err("Could not request upload port for ROACH firmware on %s! retval = %i",
               m_roach->address, retval);
        return retval;
    }
    for (int loop = 0; loop < 2; loop++) {
        ph_sock_resolve_and_connect(state.roach->address, state.port, 0,
            &state.timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, firmware_upload_connected, &state);
        while (state.result == ROACH_UPLOAD_RESULT_WORKING) {
            usleep(1000);
        }
        if (state.result != ROACH_UPLOAD_CONN_REFUSED) {
            break;
            usleep(100000);
        }
    }
    if ((state.result != ROACH_UPLOAD_RESULT_SUCCESS)) {
        retval = -1;
    }
    if ((state.result = ROACH_UPLOAD_RESULT_SUCCESS)) {
        sleep(5);
        if (roach_upload_status(m_roach) < 0);
            retval = -1;
        } else {
            retval = 0;
    }
    return retval;
}*/

int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename)
{
    char *upload_command;
    firmware_state_t state = {
                          .firmware_file = m_filename,
                          .port = (uint16_t) (drand48() * 500.0 + 5000),
                          .timeout.tv_sec = 5,
                          .timeout.tv_usec = 0,
                          .roach = m_roach
    };
    blast_info("ROACH%d: Getting permission to upload fpg", m_roach->which);
    int retval = send_rpc_katcl(m_roach->rpc_conn, UPLOAD_TIMEOUT,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?progremote",
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, state.port,
                   NULL);
    if (retval != KATCP_RESULT_OK) {
        blast_err("ROACH%d: FIRMWARE UPLOAD FAILED", m_roach->which);
        blast_info("ROACH%d: Failed to connect with KATCP", m_roach->which);
        return -1;
    }
    blast_info("Uploading fpg through netcat...");
    asprintf(&upload_command, "nc -w 2 %s %u < %s", m_roach->address, state.port, m_filename);
    pyblast_system(upload_command);
    sleep(10);
    int count = 0;
    int success_val;
    while (count < MAX_FPG_UPLOAD_TRIES) {
        success_val = send_rpc_katcl(m_roach->rpc_conn, 1000,
            KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?fpgastatus",
        KATCP_FLAG_LAST | KATCP_FLAG_STRING, "",
        NULL);
        if (success_val != KATCP_RESULT_OK) {
            blast_info("ROACH%d: FPG UPLOAD COUNT = %d", m_roach->which, count);
            count++;
            sleep(5);
        } else {
            m_roach->has_firmware = 0;
            break;
        }
    }
    if (count == MAX_FPG_UPLOAD_TRIES) {
        blast_err("ROACH%d: FIRMWARE UPLOAD FAILED", m_roach->which);
        return -1;
    }
    blast_info("ROACH%d: FPGA programmed", m_roach->which);
    m_roach->has_firmware = 1;
    // char *ret = arg_string_katcl(m_roach->rpc_conn, 1);
    // blast_info("ROACH%d: FPGA programmed %s", m_roach->which, ret);
    return 0;
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
        destroy_rpc_katcl(roach_state_table[i].rpc_conn);
        roach_state_table[i].rpc_conn = NULL;
        if (pi_state_table[i].pi_comm) {
            remote_serial_shutdown(pi_state_table[i].pi_comm);
            pi_state_table[i].pi_comm = NULL;
        }
    }
}

void reset_flags(roach_state_t *m_roach)
{
    CommandData.roach[m_roach->which - 1].kill = 0;
    m_roach->fpga_clock_freq = 0.0;
    m_roach->adc_rms[0] = 0.0;
    m_roach->adc_rms[1] = 0.0;
    m_roach->doing_full_loop = 0;
    m_roach->doing_turnaround_loop = 0;
    m_roach->doing_find_kids_loop = 0;
    m_roach->has_firmware = 0;
    m_roach->firmware_upload_fail = 0;
    m_roach->is_streaming = 0;
    m_roach->has_qdr_cal = 0;
    m_roach->qdr_fail = 0;
    m_roach->is_sweeping = 0;
    CommandData.roach[m_roach->which - 1].is_sweeping = 0;
    m_roach->has_vna_sweep = 0;
    m_roach->has_vna_tones = 0;
    m_roach->has_targ_sweep = 0;
    m_roach->has_targ_tones = 0;
    m_roach->has_ref_params = 0;
    m_roach->current_ntones = 0;
    m_roach->num_kids = 0;
    m_roach->prev_num_kids = 0;
    m_roach->tone_finding_error = 0;
    m_roach->sweep_fail = 0;
    m_roach->katcp_connect_error = 0;
    m_roach->pi_error_count = 0;
    m_roach->pi_reboot_warning = 0;
    m_roach->data_stream_error = 0;
    m_roach->waiting_for_lamp = 0;
    m_roach->full_loop_fail = 0;
    m_roach->trnaround_loop_fail = 0;
    m_roach->write_flag = 0;
    m_roach->n_outofrange = 0;
    for (size_t i = 0; i < m_roach->current_ntones; i++) {
        m_roach->out_of_range[i] = 0;
    }
    CommandData.roach[m_roach->which - 1].do_sweeps = 0;
}

void pi_state_manager(pi_state_t *m_pi, int result)
{
    int current_state = m_pi->state;
    switch (current_state) {
        case PI_STATE_BOOT:
            if (result == -1) {
                // PI failed to boot? Remote serial fail?
                break;
            }
            if (result == 0) {
                m_pi->state = PI_STATE_CONNECTED;
                m_pi->desired_state = PI_STATE_INIT;
                break;
            }
        case PI_STATE_CONNECTED:
            if (result == -1) {
                // Attenuator problem
                break;
            }
            if (result == -2) {
                // Valon problem
                break;
            }
            if (result == -3) {
                // Couldn't set LO
                break;
            }
            if (result == PI_READ_ERROR) {
                // Couldn't read LO
                break;
            }
            if (result == 0) {
                // Final state for PI
                // TODO(Sam) figure out how to distinguish attens
                m_pi->has_valon = 1;
                m_pi->has_output_atten = 1;
                m_pi->has_input_atten = 1;
                m_pi->state = PI_STATE_INIT;
                m_pi->desired_state = PI_STATE_INIT;
                break;
            }
    }
}

int roach_boot_sequence(roach_state_t *m_roach)
{
    // blast_info("ROACH%d: INSIDE BOOT SEQ", m_roach->which);
    int retval = -1;
    int flags = 0;
    if (m_roach->rpc_conn) {
         destroy_rpc_katcl(m_roach->rpc_conn);
         m_roach->rpc_conn = NULL;
         blast_info("ROACH%d: Destroying KATCP connection", m_roach->which);
    }
    if (!m_roach->has_warned) {
        blast_info("ROACH%d: ATTEMPTING TO CONNECT TO %s", m_roach->which, m_roach->address);
    }
    m_roach->has_warned = 1;
    flags = NETC_VERBOSE_ERRORS | NETC_VERBOSE_STATS;
    m_roach->katcp_fd = net_connect(m_roach->address, 0, flags);
    if (m_roach->katcp_fd < 0) {
        m_roach->katcp_connect_error = 1;
        // blast_err("ROACH%d: KATCP CONNECTION ERROR", m_roach->which);
        return retval;
    }
    m_roach->rpc_conn = create_katcl(m_roach->katcp_fd);
    m_roach->katcp_connect_error = 0;
    blast_info("ROACH%d: KATCP UP", m_roach->which);
    m_roach->has_warned = 0;
    return 0;
}

int roach_prog_registers(roach_state_t *m_roach)
{
    int retval = -1;
    blast_info("ROACH%d, Configuring software registers...", m_roach->which);
    if ((roach_write_int(m_roach, "GbE_packet_info", 42, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "dds_shift", DDC_SHIFT, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "PFB_fft_shift", VNA_FFT_SHIFT, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "downsamp_sync_accum_len", accum_len, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_destip", dest_ip, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_destport", m_roach->dest_port, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_srcport", m_roach->src_port, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_srcip", m_roach->src_ip, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_srcmac0", srcmac0[m_roach->which - 1], 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_srcmac1", srcmac1, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_destmac0", destmac0, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_destmac1", destmac1, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "dac_reset", 1, 0)) < 0) {
        return retval;
    }
    // load_fir(m_roach);
    retval = 0;
    return retval;
}

int roach_init_gbe(roach_state_t *m_roach)
{
    int retval = -1;
    if ((roach_write_int(m_roach, "GbE_tx_rst", 0, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_rst", 1, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_tx_rst", 0, 0)) < 0) {
        return retval;
    }
    usleep(100);
    if ((roach_write_int(m_roach, "GbE_pps_start", 1, 0)) < 0) {
        return retval;
    }
    usleep(100);
    // write ctime to register
    timestamp(m_roach);
    return 0;
}

int roach_config_sequence(roach_state_t *m_roach)
{
    int retval;
    // program firmware registers
    if ((roach_prog_registers(m_roach) < 0)) {
        retval = -1;
        return retval;
    }
    // calibrate qdr
    if ((roach_qdr_cal(m_roach) < 0)) {
        retval = -2;
        return retval;
    }
    // initialize GbE
    if ((roach_init_gbe(m_roach) < 0)) {
        retval = -3;
        return retval;
    }
    // load rf tone powers
    retval = 0;
    return retval;
}

int roach_vna_sweep(roach_state_t *m_roach)
{
    int status = -1;
    char *var_name;
    if (CommandData.roach[m_roach->which - 1].do_sweeps == 0) {
        blast_err("ROACH%d: DO SWEEP NOT SET", m_roach->which);
        return status;
    }
    // char *echo_command;
    if (m_roach->has_targ_tones) {
        blast_info("ROACH%d: Must write search comb before running VNA sweep.", m_roach->which);
        CommandData.roach[m_roach->which - 1].do_sweeps = 0;
        return status;
    }
    m_roach->is_sweeping = 1;
    blast_info("ROACH%d, Initializing VNA sweep", m_roach->which);
    blast_info("ROACH%d, Starting VNA sweep...", m_roach->which);
    status = roach_do_sweep(m_roach, VNA);
    if (status == SWEEP_SUCCESS) {
        blast_info("ROACH%d, VNA sweep complete", m_roach->which);
        // pyblast_system("python /home/fc1user/sam_builds/sweep_list.py vna");
        // write environment variable for sweep path
        blast_tmp_sprintf(var_name, "R%d_LAST_VNA_SWEEP", m_roach->which);
        // blast_tmp_sprintf(echo_command, "echo $%s", var_name);
        setenv(var_name, path_to_vna_tarball[m_roach->which - 1], 1);
        compress_data(m_roach, VNA);
        m_roach->sweep_fail = 0;
        m_roach->is_sweeping = 0;
        m_roach->has_vna_sweep = 1;
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        return status;
    } else if (status == SWEEP_INTERRUPT) {
        blast_info("ROACH%d, VNA sweep interrupted by blastcmd", m_roach->which);
        m_roach->sweep_fail = 1;
        m_roach->is_sweeping = 0;
        m_roach->has_vna_sweep = 0;
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        return status;
    } else if (status == SWEEP_FAIL) {
        blast_info("ROACH%d, VNA SWEEP FAILED", m_roach->which);
        m_roach->sweep_fail = 1;
        m_roach->is_sweeping = 0;
        m_roach->has_vna_sweep = 0;
        if (recenter_lo(m_roach) < 0) {
            blast_err("ROACH%d: Failed to recenter LO", m_roach->which);
        }
        return status;
    }
    return 0;
}

int roach_full_loop(roach_state_t *m_roach)
{
    if (!m_roach->has_vna_tones) {
        roach_write_vna(m_roach);
    }
    m_roach->has_targ_tones = 0;
    int status = -1;
    int i = m_roach->which - 1;
    if (!CommandData.roach[m_roach->which - 1].find_kids) {
        CommandData.roach[m_roach->which - 1].find_kids = 1;
    }
    m_roach->doing_full_loop = 1;
    CommandData.roach[m_roach->which - 1].set_attens = 5;
    if ((status = set_attens_targ_output(m_roach)) < 0) {
         blast_err("ROACH%d: Failed to set attenuators, but continuing full loop", i + 1);
    }
    // VNA sweep
    CommandData.roach[i].do_sweeps = 1;
    if ((status = roach_vna_sweep(m_roach)) < 0) {
        blast_err("ROACH%d: VNA SWEEP FAIL", i + 1);
        m_roach->sweep_fail = 1;
        return status;
    }
    // Find kids
    if (CommandData.roach[i].find_kids == 2) {
        if ((status = get_targ_freqs(m_roach, 0)) < 0) {
            blast_err("ROACH%d: TONE FINDING ERROR", i + 1);
            return status;
        }
    }
    if (CommandData.roach[i].find_kids == 1) {
        if ((status = get_targ_freqs(m_roach, 1)) < 0) {
            blast_err("ROACH%d: TONE FINDING ERROR", i + 1);
            return status;
       }
    }
    CommandData.roach[i].find_kids = 0;
    // write found tones
    if ((status = roach_write_targ_tones(m_roach)) < 0) {
        blast_err("ROACH%d: ERROR WRITING TONES", i + 1);
        return status;
    }
    // Set attens again to account for change in number of tones
    CommandData.roach[m_roach->which - 1].set_attens = 5;
    if ((status = set_attens_targ_output(m_roach)) < 0) {
         blast_err("ROACH%d: Failed to set attenuators, but continuing full loop", i + 1);
    }
    // TARG/REFIT/TARG
    CommandData.roach[i].refit_res_freqs = 1;
    if ((status = roach_refit_freqs(m_roach, CommandData.roach[i].on_res)) < 0) {
        blast_err("ROACH%d: ERROR REFITTING FREQS", i + 1);
        return status;
    }
    /*
    // If no one else has lamp control, take it
    if (!CommandData.roach[m_roach->which - 1].has_lamp_control) {
        int n_roaches = 0;
        for (int i = 0; i < NUM_ROACHES; i++) {
            if (!CommandData.roach[i].has_lamp_control) {
                n_roaches++;
            }
        }
        if (n_roaches == NUM_ROACHES) {
            CommandData.roach[m_roach->which - 1].has_lamp_control = 1;
        }
    } */
    return 0;
}

int roach_fk_loop(roach_state_t* m_roach)
{
    int status;
    int i = m_roach->which - 1;
    if ((!m_roach->has_vna_tones) && (m_roach->has_targ_tones)) {
        m_roach->has_targ_tones = 0;
        roach_write_vna(m_roach);
    }
    m_roach->doing_find_kids_loop = 1;
    CommandData.roach[m_roach->which - 1].set_attens = 5;
    if ((status = set_attens_targ_output(m_roach)) < 0) {
        blast_err("ROACH%d: Failed to set attenuators, but continuing full loop", i + 1);
    }
    // VNA sweep
    CommandData.roach[i].do_sweeps = 1;
    if ((status = roach_vna_sweep(m_roach)) < 0) {
        blast_err("ROACH%d: VNA SWEEP FAIL", i + 1);
        m_roach->sweep_fail = 1;
        CommandData.roach[i].do_fk_loop = 0;
        m_roach->has_targ_tones = 0;
        return status;
    }
    // Find kids
    if (CommandData.roach[i].find_kids == 2) {
        if ((status = get_targ_freqs(m_roach, 0)) < 0) {
               blast_err("ROACH%d: TONE FINDING ERROR", i + 1);
               CommandData.roach[i].do_fk_loop = 0;
               m_roach->has_targ_tones = 0;
               return status;
           }
    }
    if (CommandData.roach[i].find_kids == 1) {
        if ((status = get_targ_freqs(m_roach, 1)) < 0) {
               blast_err("ROACH%d: TONE FINDING ERROR", i + 1);
               CommandData.roach[i].do_fk_loop = 0;
               m_roach->has_targ_tones = 0;
               return status;
           }
    }
    m_roach->has_targ_tones = 0;
    CommandData.roach[i].find_kids = 0;
    CommandData.roach[i].do_sweeps = 0;
    m_roach->doing_find_kids_loop = 0;
    return 0;
}

void check_cycle_status(void)
{
    while (CommandData.roach_run_cycle_checker) {
        // do the following every check once per minute
        sleep(60);
        int was_cycling = 0;
        static channel_t* stateCycleAddr;
        uint16_t cycle_state;
        stateCycleAddr = channels_find_by_name("state_cycle");
        // Check cycle_state channel
        // If == 2, 3, or 4, put Roaches into cycle_mode (streaming state)
        GET_VALUE(stateCycleAddr, cycle_state);
        // blast_info("CYCLE STATE =============== %d", cycle_state);
        if ((cycle_state == 2) | (cycle_state == 3) | (cycle_state == 4)) {
            is_cycling = 1;
            was_cycling = 1;
        }
        while (is_cycling) {
            sleep(60);
            GET_VALUE(stateCycleAddr, cycle_state);
            if ((cycle_state == 0) | (cycle_state == 1)) {
                is_cycling = 0;
            }
        }
        // Once cycle ends, trigger full loop
        if (was_cycling) {
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].do_full_loop = 1;
            }
            was_cycling = 0;
        }
    }
}

void start_cycle_checker()
{
    pthread_t cycle_state_checker;
    blast_info("Starting cycle state checker");
    if (pthread_create(&cycle_state_checker, NULL, (void*)&check_cycle_status, NULL)) {
        blast_err("Error starting cycle state checker");
    }
}

/** Function: roach_state_manager
 * ----------------------------
 * Takes the result of a function and pushes the roach status
 * to the proper state
 * @param m_roach a roach state structure
 * @param m_result the result of the input function
**/
void roach_state_manager(roach_state_t *m_roach, int result)
{
    m_roach->desired_state = ROACH_STATE_STREAMING;
    int current_state = m_roach->state;
    switch (current_state) {
        case ROACH_STATE_BOOT:
            // Check that Roach is powered on, establish KATCP link
            if (result == -1) {
                // boot fail? KATCP fail?
                sleep(5);
            }
            if (result == 0) {
                m_roach->state = ROACH_STATE_CONNECTED;
            }
            break;
        case ROACH_STATE_CONNECTED:
            // Upoad firmware
            if (result == -1) {
                m_roach->firmware_upload_fail = 1;
                m_roach->state = ROACH_STATE_BOOT;
            }
            if (result == 0) {
                m_roach->has_firmware = 1;
                m_roach->firmware_upload_fail = 0;
                m_roach->state = ROACH_STATE_PROGRAMMED;
            }
            break;
        case ROACH_STATE_PROGRAMMED:
            // Program firmware registers, calibrate QDR RAM,
            // Initialize GbE
            if (result == -1) {
                m_roach->state -= 1;
                // register write fail
            }
            if (result == -2) {
                m_roach->state -= 1;
                // qdr cal fail
                m_roach->state = ROACH_STATE_CONNECTED;
            }
            if (result == -3) {
                m_roach->state -= 1;
                // GbE init fail
            }
            if (result == 0) {
                m_roach->state = ROACH_STATE_CONFIGURED;
            } else {
                m_roach->state -= 1;
            }
            break;
        case ROACH_STATE_CONFIGURED:
            if (result == -1) {
                m_roach->state -= 1;
                // error during tone writing
            }
            if (result == -2) {
                m_roach->data_stream_error = 1;
                m_roach->is_streaming = 0;
                m_roach->state = ROACH_STATE_BOOT;
                // error receiving data packets
            }
            if (result == 0) {
                m_roach->state = ROACH_STATE_STREAMING;
                m_roach->data_stream_error = 0;
                recenter_lo(m_roach);
                /* if ((set_attens_to_default(&pi_state_table[m_roach->which - 1])) < 0) {
                    blast_err("ROACH%d: Failed to set RUDATs...", m_roach->which);
                } */
                // do a full loop
                load_tone_powers();
                CommandData.roach[m_roach->which - 1].do_full_loop = 1;
            }
            break;
        case ROACH_STATE_STREAMING:
            break;
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
    int result = 0;
    // int i = *((uint16_t*) ind);
    int i = (uint64_t)ind;
    char tname[10];
    if (snprintf(tname, sizeof(tname), "rcmd%i", i + 1) < 5) {
    blast_tfatal("Could not name thread for roach%i", i);
    }
    ph_thread_set_name(tname);
    nameThread(tname);
    static int first_time = 1;
    while (!InCharge) {
        if (first_time) {
            blast_info("roach%i: Waiting until we get control...", i + 1);
             first_time = 0;
        }
        usleep(2000);
    }
    blast_info("Starting Roach Commanding Thread");
    pi_state_table[i].state = PI_STATE_BOOT;
    // pi_state_table[i].desired_state = PI_STATE_INIT;
    roach_state_table[i].state = ROACH_STATE_BOOT;
    roach_state_table[i].desired_state = ROACH_STATE_STREAMING;
    reset_flags(&roach_state_table[i]);

    /* if (recenter_lo(&roach_state_table[i])) {
        pi_state_table[i].state = PI_STATE_CONNECTED;
    }
    if ((set_attens_to_default(&pi_state_table[i])) < 0) {
        blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
    }*/
    int packet_error_counter = 0;
    int current_packet_count = 0;
    int last_packet_count = 0;
    int enable_lo_chop_was_on = 0;
    int enable_el_retune_top_was_on = 0;
    int enable_el_retune_bottom_was_on = 0;
    // int n_full_loop_tries = 0;
    while (!shutdown_mcp) {
        if (roach_state_table[i].state == ROACH_STATE_BOOT &&
                                   roach_state_table[i].desired_state >= ROACH_STATE_BOOT) {
            // establish a KATCP connection to the PPC
            result = roach_boot_sequence(&roach_state_table[i]);
            roach_state_manager(&roach_state_table[i], result);
        }
        if (roach_state_table[i].state == ROACH_STATE_CONNECTED &&
            roach_state_table[i].desired_state >= ROACH_STATE_CONNECTED) {
            // upload firmware
            result = roach_upload_fpg(&roach_state_table[i], roach_fpg);
            roach_state_manager(&roach_state_table[i], result);
        }
        if (roach_state_table[i].state == ROACH_STATE_PROGRAMMED &&
            roach_state_table[i].desired_state >= ROACH_STATE_PROGRAMMED) {
            // program registers, calibrate QDR RAM, initialize 1GbE
            result = roach_config_sequence(&roach_state_table[i]);
            roach_state_manager(&roach_state_table[i], result);
        }
        if (roach_state_table[i].state == ROACH_STATE_CONFIGURED &&
                   roach_state_table[i].desired_state >= ROACH_STATE_CONFIGURED) {
            // write default VNA comb (this triggers UDP streaming, which
            // continues until shutdown
            result = roach_write_vna(&roach_state_table[i]);
            if (result < 0) {
                roach_state_manager(&roach_state_table[i], result);
            } else {
                // if write succeeds, check streaming
                result = roach_check_streaming(&roach_state_table[i], STREAM_NTRIES, STREAM_TIMEOUT);
            }
            roach_state_manager(&roach_state_table[i], result);
        }
        if (CommandData.roach[i].change_extref) {
            valon_set_ref(&pi_state_table[i]);
            CommandData.roach[i].change_extref = 0;
        }
        if (CommandData.roach[i].kill) {
            roach_halt_ppc(&roach_state_table[i]);
            CommandData.roach[i].kill = 0;
        }
        if (roach_state_table[i].pi_error_count >= MAX_PI_ERRORS_REBOOT) {
            CommandData.roach[i].reboot_pi_now = 1;
            roach_state_table[i].pi_error_count = 0;
        }
        if (CommandData.roach[i].reboot_pi_now == 1) {
            if (pi_reboot_now(&pi_state_table[i]) < 0) {
                pi_state_table[i].state = PI_STATE_BOOT;
            }
            CommandData.roach[i].reboot_pi_now = 0;
        }
        if (CommandData.roach[i].set_lo == 1) {
            if (recenter_lo(&roach_state_table[i]) < 0) {
                blast_err("Error recentering LO");
            } else {
                blast_info("ROACH%d, LO recentered", i + 1);
            }
            CommandData.roach[i].set_lo = 0;
        }
        if (CommandData.roach[i].set_lo == 2) {
            if (shift_lo(&roach_state_table[i]) < 0) {
                blast_info("Error shifting LO");
            } else {
                blast_info("Roach%d, LO shifted", i + 1);
            }
            CommandData.roach[i].set_lo = 0;
        }
        if (CommandData.roach[i].set_lo == 3) {
            if (set_LO(&pi_state_table[i], CommandData.roach_params[i].lo_freq_MHz) < 0) {
                blast_err("PI%d, Failed to set LO", i + 1);
                CommandData.roach[i].set_lo = 0;
            }
            CommandData.roach[i].set_lo = 0;
        }
        if (CommandData.roach[i].read_lo == 1) {
            if (read_LO(&pi_state_table[i]) < 0) {
                blast_err("PI%d, Failed to read LO", i + 1);
                CommandData.roach[i].read_lo = 0;
            }
            CommandData.roach[i].read_lo = 0;
        }
        if (CommandData.roach[i].read_temp) {
            if (get_pi_temp(&pi_state_table[i]) < 0) {
                blast_err("PI%d, Failed to read Pi temp", i + 1);
            }
            CommandData.roach[i].read_temp = 0;
        }
        if (CommandData.roach[i].get_roach_state) {
            blast_info("ROACH%d, current state = %u", i + 1, get_roach_state(i));
            CommandData.roach[i].get_roach_state = 0;
        }
        if (CommandData.roach[i].change_roach_state) {
            roach_state_table[i].state = CommandData.roach[i].roach_new_state;
            roach_state_table[i].desired_state = CommandData.roach[i].roach_desired_state;
            blast_info("CHANGE STATE: %d, %d",
                    CommandData.roach[i].roach_new_state,
                    CommandData.roach[i].roach_desired_state);
            if (roach_state_table[i].desired_state == ROACH_STATE_STREAMING) {
                reset_flags(&roach_state_table[i]);
            }
            CommandData.roach[i].change_roach_state = 0;
        }
        if (CommandData.roach[i].set_attens == 1) {
            if (set_atten(&pi_state_table[i]) < 0) {
                blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
                CommandData.roach[i].set_attens = 0;
            } else {
                CommandData.roach[i].set_attens = 0;
            }
        }
        if (CommandData.roach[i].set_attens == 2) {
            if (set_attens_to_default(&pi_state_table[i]) < 0) {
                blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
            }
            CommandData.roach[i].set_attens = 0;
        }
        if (CommandData.roach[i].set_attens == 3) {
            if (write_last_attens(&roach_state_table[i]) < 0) {
                blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
                CommandData.roach[i].set_attens = 0;
            }
            CommandData.roach[i].set_attens = 0;
        }
        if (CommandData.roach[i].set_attens == 4) {
            if (set_atten_conserved(&pi_state_table[i]) < 0) {
                blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
                CommandData.roach[i].set_attens = 0;
            }
            CommandData.roach[i].set_attens = 0;
        }
        if (CommandData.roach[i].set_attens == 5) {
            set_attens_targ_output(&roach_state_table[i]);
            CommandData.roach[i].set_attens = 0;
        }
        if (CommandData.roach[i].read_attens == 1) {
            if (read_atten(&pi_state_table[i]) < 0) {
                blast_err("ROACH%d: Failed to read RUDATs...", i + 1);
                CommandData.roach[i].read_attens = 0;
            } else {
                CommandData.roach[i].read_attens = 0;
            }
        }
        if (CommandData.roach[i].new_atten) {
            if (set_output_atten(&roach_state_table[i],
                   CommandData.roach_params[i].new_out_atten) < 0) {
                blast_err("ROACH%d: Failed to set RUDATs...", i + 1);
            } else {
                CommandData.roach[i].new_atten = 0;
            }
        }
        if (roach_state_table[i].has_tones) {
           if (CommandData.roach[i].adc_rms) {
               get_adc_rms(&roach_state_table[i]);
               CommandData.roach[i].adc_rms = 0;
           }
        }
        if (CommandData.roach[i].is_chopping_lo) {
            if (CommandData.roach[i].enable_chop_lo) {
                if (CommandData.roach[i].auto_el_retune_top) {
                    enable_el_retune_top_was_on = 1;
                    CommandData.roach[i].auto_el_retune_top = 0;
                }
                if (CommandData.roach[i].auto_el_retune_bottom) {
                    enable_el_retune_bottom_was_on = 1;
                    CommandData.roach[i].auto_el_retune_bottom = 0;
                }
                if (roach_chop_lo(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: Failed to Chop LO", i + 1);
                }
                if (enable_el_retune_top_was_on) {
                    CommandData.roach[i].auto_el_retune_top = 1;
                    enable_el_retune_top_was_on = 0;
                }
                if (enable_el_retune_bottom_was_on) {
                    CommandData.roach[i].auto_el_retune_bottom = 1;
                    enable_el_retune_bottom_was_on = 0;
                }
            }
            CommandData.roach[i].is_chopping_lo = 0;
        }
        // These commmands require roach state to be streaming
        if (roach_state_table[i].state == ROACH_STATE_STREAMING) {
            // FLIGHT MODE LOOPS
            if (CommandData.trigger_lo_offset_check & (1 << i)) {
                if (CommandData.roach[i].enable_chop_lo) {
                    if (CommandData.roach[i].auto_el_retune_top) {
                        enable_el_retune_top_was_on = 1;
                        CommandData.roach[i].auto_el_retune_top = 0;
                    }
                    if (CommandData.roach[i].auto_el_retune_bottom) {
                        enable_el_retune_bottom_was_on = 1;
                        CommandData.roach[i].auto_el_retune_bottom = 0;
                    }
                    CommandData.roach[i].is_chopping_lo = 1;
                    if (roach_chop_lo(&roach_state_table[i]) < 0) {
                        blast_err("ROACH%d: Failed to Chop LO", i + 1);
                    }
                    CommandData.roach[i].is_chopping_lo = 0;
                    if (enable_el_retune_top_was_on) {
                        CommandData.roach[i].auto_el_retune_top = 1;
                        enable_el_retune_top_was_on = 0;
                    }
                    if (enable_el_retune_bottom_was_on) {
                        CommandData.roach[i].auto_el_retune_bottom = 1;
                        enable_el_retune_bottom_was_on = 0;
                    }
                }
                CommandData.trigger_lo_offset_check &= 0xff-(1 << i);
            }
            uint8_t trigger_top = CommandData.trigger_roach_tuning_check_top & (1 << i);
            uint8_t trigger_bottom = CommandData.trigger_roach_tuning_check_bottom & (1 << i);
            if ((CommandData.roach[i].auto_el_retune_top && trigger_top) ||
                (CommandData.roach[i].auto_el_retune_bottom && trigger_bottom)) {
                    if (CommandData.roach[i].enable_chop_lo) {
                        enable_lo_chop_was_on = 1;
                        CommandData.roach[i].enable_chop_lo = 0;
                    }
                    if (roach_turnaround_loop(&roach_state_table[i]) < 0) {
                        blast_err("ROACH%d: FAILED TO EXECUTE EL TURNAROUND TARGSWEEP", i + 1);
                        roach_state_table[i].trnaround_loop_fail = 1;
                    } else {
                        roach_state_table[i].trnaround_loop_fail = 0;
                    }
                    CommandData.roach[i].refit_res_freqs = 0;
                    CommandData.roach[i].do_sweeps = 0;
                    CommandData.trigger_roach_tuning_check_top &= 0xff-(1 << i);
                    CommandData.trigger_roach_tuning_check_bottom &= 0xff-(1 << i);
                    if (enable_lo_chop_was_on) {
                        CommandData.roach[i].enable_chop_lo = 1;
                        enable_lo_chop_was_on = 0;
                    }
            } else if ((!CommandData.roach[i].auto_el_retune_top && trigger_top) ||
                      (!CommandData.roach[i].auto_el_retune_bottom && trigger_bottom)) {
                CommandData.trigger_roach_tuning_check_top &= 0xff-(1 << i);
                CommandData.trigger_roach_tuning_check_bottom &= 0xff-(1 << i);
            }
            if (CommandData.roach[i].do_turnaround_loop) {
                if (CommandData.roach[i].enable_chop_lo) {
                    enable_lo_chop_was_on = 1;
                    CommandData.roach[i].enable_chop_lo = 0;
                }
                if (roach_turnaround_loop(&roach_state_table[i]) < 0) {
                    roach_state_table[i].trnaround_loop_fail = 1;
                    blast_err("ROACH%d: FAILED TO EXECUTE TURNAROUND LOOP", i + 1);
                } else {
                    roach_state_table[i].trnaround_loop_fail = 0;
                }
                if (enable_lo_chop_was_on) {
                    CommandData.roach[i].enable_chop_lo = 1;
                    enable_lo_chop_was_on = 0;
                }
                CommandData.trigger_roach_tuning_check_top &= 0xff-(1 << i);
                CommandData.trigger_roach_tuning_check_bottom &= 0xff-(1 << i);
                CommandData.roach[i].do_turnaround_loop = 0;
                CommandData.roach[i].refit_res_freqs = 0;
                CommandData.roach[i].do_sweeps = 0;
            }
            // FULL LOOP
            if (CommandData.roach[i].do_full_loop == 1) {
                // disable lo chop and auto turnaround enables
                if (CommandData.roach[i].auto_el_retune_top) {
                    enable_el_retune_top_was_on = 1;
                    CommandData.roach[i].auto_el_retune_top = 0;
                }
                if (CommandData.roach[i].auto_el_retune_bottom) {
                    enable_el_retune_bottom_was_on = 1;
                    CommandData.roach[i].auto_el_retune_bottom = 0;
                }
                if (CommandData.roach[i].enable_chop_lo) {
                    enable_lo_chop_was_on = 1;
                    CommandData.roach[i].enable_chop_lo = 0;
                }
                // FAIL
                if (roach_full_loop(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: FULL LOOP FAILED TO COMPLETE", i + 1);
                    roach_state_table[i].full_loop_fail = 1;
                    // n_full_loop_tries += 1;
                    // if (n_full_loop_tries == MAX_FULL_LOOP_TRIES) {
                    // n_full_loop_tries = 0;
                    if (enable_lo_chop_was_on) {
                        CommandData.roach[i].enable_chop_lo = 1;
                        enable_lo_chop_was_on = 0;
                    }
                    if (enable_el_retune_top_was_on) {
                        CommandData.roach[i].auto_el_retune_top = 1;
                        enable_el_retune_top_was_on = 0;
                    }
                    if (enable_el_retune_bottom_was_on) {
                        CommandData.roach[i].auto_el_retune_bottom = 1;
                        enable_el_retune_bottom_was_on = 0;
                    }
                // SUCCEED
                } else {
                    blast_info("ROACH%d: FULL LOOP COMPLETED", i + 1);
                    roach_state_table[i].full_loop_fail = 0;
                    // n_full_loop_tries = 0;
                    if (enable_lo_chop_was_on) {
                        CommandData.roach[i].enable_chop_lo = 1;
                        enable_lo_chop_was_on = 0;
                    }
                    if (enable_el_retune_top_was_on) {
                        CommandData.roach[i].auto_el_retune_top = 1;
                        enable_el_retune_top_was_on = 0;
                    }
                    if (enable_el_retune_bottom_was_on) {
                        CommandData.roach[i].auto_el_retune_bottom = 1;
                        enable_el_retune_bottom_was_on = 0;
                    }
                }
                CommandData.roach[i].do_full_loop = 0;
                CommandData.roach[i].find_kids = 0;
                CommandData.roach[i].refit_res_freqs = 0;
                CommandData.roach[i].do_sweeps = 0;
                roach_state_table[i].doing_full_loop = 0;
            }
            // DO FIND KIDS LOOP
            if (CommandData.roach[i].do_fk_loop == 1) {
                if (CommandData.roach[i].auto_el_retune_top) {
                    enable_el_retune_top_was_on = 1;
                    CommandData.roach[i].auto_el_retune_top = 0;
                }
                if (CommandData.roach[i].auto_el_retune_bottom) {
                    enable_el_retune_bottom_was_on = 1;
                    CommandData.roach[i].auto_el_retune_bottom = 0;
                }
                if (CommandData.roach[i].enable_chop_lo) {
                    enable_lo_chop_was_on = 1;
                    CommandData.roach[i].enable_chop_lo = 0;
                }
                if (roach_fk_loop(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: FK LOOP FAILED TO COMPLETE", i + 1);
                } else {
                    blast_info("ROACH%d: FK LOOP COMPLETED", i + 1);
                }
                if (enable_lo_chop_was_on) {
                    CommandData.roach[i].enable_chop_lo = 1;
                    enable_lo_chop_was_on = 0;
                }
                if (enable_el_retune_top_was_on) {
                    CommandData.roach[i].auto_el_retune_top = 1;
                    enable_el_retune_top_was_on = 0;
                }
                if (enable_el_retune_bottom_was_on) {
                    CommandData.roach[i].auto_el_retune_bottom = 1;
                    enable_el_retune_bottom_was_on = 0;
                }
                CommandData.roach[i].find_kids = 0;
                CommandData.roach[i].do_fk_loop = 0;
                CommandData.roach[i].do_sweeps = 0;
                roach_state_table[i].doing_find_kids_loop = 0;
            }
            // df retune
            if ((CommandData.roach[i].do_check_retune == 1)) {
                if (roach_check_df_retune(&roach_state_table[i]) < 0) {
                    CommandData.roach[i].do_check_retune = 0;
                    blast_err("ROACH%d: CHECK DF RETUNE FAILED", i + 1);
                }
                CommandData.roach[i].do_check_retune = 0;
            }
            // cal lamp retune
            if ((CommandData.roach[i].do_check_retune == 2)) {
                if (roach_check_lamp_retune(&roach_state_table[i]) < 0) {
                    CommandData.roach[i].do_check_retune = 0;
                    blast_err("ROACH%d: CHECK LAMP RETUNE FAILED", i + 1);
                }
                CommandData.roach[i].do_check_retune = 0;
            }
            // sweep and retune
            if ((CommandData.roach[i].do_check_retune == 3)) {
                if (roach_check_df_sweep_retune(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: CHECK DF SWEEP RETUNE FAILED", i + 1);
                }
                CommandData.roach[i].do_check_retune = 0;
            }
            // median sweep
            if ((CommandData.roach[i].do_check_retune == 4)) {
                if (roach_median_sweep_df(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: MEDIAN SWEEP RETUNE FAILED", i + 1);
                }
                CommandData.roach[i].do_check_retune = 0;
            }
            if ((CommandData.roach[i].change_tone_amps == 1)) {
                shift_tone_amp(&roach_state_table[i]);
                CommandData.roach[i].change_tone_amps = 0;
            }
            if ((CommandData.roach[i].change_tone_freq == 1)) {
                shift_tone_freq(&roach_state_table[i]);
                CommandData.roach[i].change_tone_freq = 0;
            }
            if ((CommandData.roach[i].change_tone_phase == 1)) {
                shift_tone_phase(&roach_state_table[i]);
                CommandData.roach[i].change_tone_phase = 0;
            }
            if ((CommandData.roach[i].change_targ_freq == 1)) {
                shift_freq(&roach_state_table[i]);
            }
            if ((CommandData.roach[i].change_targ_freq == 2)) {
                shift_freqs(&roach_state_table[i]);
            }
            if ((CommandData.roach[i].do_df_calc == 1) && roach_state_table[i].has_targ_tones) {
                if ((roach_df(&roach_state_table[i]) < 0)) {
                    CommandData.roach[i].do_df_calc = 0;
                }
                CommandData.roach[i].do_df_calc = 0;
            }
            if ((CommandData.roach[i].do_df_calc == 2) && roach_state_table[i].has_targ_tones) {
                if ((roach_dfs(&roach_state_table[i]) < 0)) {
                    CommandData.roach[i].do_df_calc = 0;
                }
                CommandData.roach[i].do_df_calc = 0;
            }
            if (CommandData.roach[i].do_sweeps == 1) {
                result = roach_vna_sweep(&roach_state_table[i]);
                // roach_state_manager(&roach_state_table[i], result);
            }
            if (CommandData.roach[i].do_noise_comp == 1) {
                if (roach_noise_comp(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: NOISE COMP FAILED", i + 1);
                }
                CommandData.roach[i].do_noise_comp = 0;
            }
            if (CommandData.roach[i].do_sweeps == 2) {
                if (roach_state_table[i].has_targ_tones) {
                    result = roach_targ_sweep(&roach_state_table[i]);
                    // roach_state_manager(&roach_state_table[i], result);
                    CommandData.roach[i].do_sweeps = 0;
                } else {
                    blast_info("ROACH%d: Must write targ tones before doing targ sweep", i + 1);
                }
                CommandData.roach[i].do_sweeps = 0;
            }
            if (CommandData.roach[i].calc_ref_params) {
                save_ref_params(&roach_state_table[i]);
                CommandData.roach[i].calc_ref_params = 0;
            }
            if (CommandData.roach[i].recenter_df) {
                CommandData.roach[i].recenter_df = 1;
                center_df(&roach_state_table[i]);
                CommandData.roach[i].recenter_df = 0;
            }
            if (CommandData.roach[i].refit_res_freqs) {
                CommandData.roach[i].auto_el_retune_top = 0;
                CommandData.roach[i].auto_el_retune_bottom = 0;
                if (CommandData.roach[i].enable_chop_lo) {
                    enable_lo_chop_was_on = 1;
                    CommandData.roach[i].enable_chop_lo = 0;
                }
                if (roach_refit_freqs(&roach_state_table[i],
                        CommandData.roach[i].on_res) < 0) {
                    blast_err("ROACH%d: Error refitting freqs", i + 1);
                    CommandData.roach[i].refit_res_freqs = 0;
                }
                if (enable_lo_chop_was_on) {
                    CommandData.roach[i].enable_chop_lo = 1;
                    enable_lo_chop_was_on = 0;
                }
                CommandData.roach[i].auto_el_retune_top = 1;
                CommandData.roach[i].auto_el_retune_bottom = 1;
                CommandData.roach[i].refit_res_freqs = 0;
            }
            if (CommandData.roach[i].do_df_targ) {
                if (roach_df_targ_sweeps(&roach_state_table[i]) < 0) {
                    blast_err("ROACH%d: Error calculating DF from sweeps", i + 1);
                }
                CommandData.roach[i].do_df_targ = 0;
            }
            if (CommandData.roach[i].get_timestream == 1) {
                blast_info("Save timestream called");
                save_timestream(&roach_state_table[i], CommandData.roach[i].chan,
                              CommandData.roach_params[i].num_sec);
            }
            if (CommandData.roach[i].get_timestream == 2) {
                blast_info("Saving all timestreams");
                if (save_all_timestreams(&roach_state_table[i],
                    CommandData.roach_params[i].num_sec) < 0) {
                    blast_err("ROACH%d: Error saving I/Q timestream", i + 1);
                }
                CommandData.roach[i].get_timestream = 0;
            }
            if (CommandData.roach[i].get_timestream == 3) {
                blast_info("Saving all timestreams");
                if (save_roach_dfs(&roach_state_table[i], CommandData.roach_params[i].num_sec) < 0) {
                    CommandData.roach[i].get_timestream = 0;
                    blast_err("ROACH%d: Failed to save delta freqs...", i + 1);
                }
                CommandData.roach[i].get_timestream = 0;
            }
            if (CommandData.roach[i].do_master_chop) {
                blast_info("Creating chop template");
                master_chop(&roach_state_table[i], CommandData.roach_params[i].num_sec);
            }
            if (CommandData.roach[i].load_vna_amps && !CommandData.roach[i].do_sweeps) {
                roach_write_vna(&roach_state_table[i]);
            }
            if (CommandData.roach[i].load_targ_amps && !roach_state_table[i].is_sweeping) {
                blast_info("Load targ amps = %d", CommandData.roach[i].load_targ_amps);
                if (roach_state_table[i].has_targ_tones) {
                    roach_write_tones(&roach_state_table[i], roach_state_table[i].targ_tones,
                                        roach_state_table[i].num_kids);
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
            if (CommandData.roach[i].load_new_freqs) {
                blast_info("Roach%d: Writing saved freqlist: %s", i + 1, roach_state_table[i].freqlist_path);
                roach_write_saved(&roach_state_table[i]);
                CommandData.roach[i].load_new_freqs = 0;
            }
            // check for retune condition
            if ((CommandData.roach[i].do_check_retune == 1) &&
                      roach_state_table[i].has_targ_tones) {
                    if ((roach_check_df_retune(&roach_state_table[i]) < 0)) {
                       blast_err("ROACH%d: Retune check failed", i + 1);
                    }
                CommandData.roach[i].do_check_retune = 0;
            }
            // force retune
            if ((CommandData.roach[i].do_retune == 1) && roach_state_table[i].has_targ_tones) {
                CommandData.roach[i].refit_res_freqs = 1;
                CommandData.roach[i].do_retune = 0;
            }
            if (CommandData.roach[i].find_kids == 2) {
                if ((get_targ_freqs(&roach_state_table[i], 0)) < 0) {
                       blast_err("ROACH%d: Failed to find kids", i + 1);
                   }
                roach_state_table[i].is_finding_kids = 0;
                CommandData.roach[i].find_kids = 0;
            }
            if (CommandData.roach[i].find_kids == 1) {
                if ((get_targ_freqs(&roach_state_table[i], 1)) < 0) {
                       blast_err("ROACH%d: Failed to find kids", i + 1);
                   }
                CommandData.roach[i].find_kids = 0;
            }
            if ((CommandData.roach[i].opt_tones) && roach_state_table[i].has_targ_tones) {
                if (calc_grad_freqs(&roach_state_table[i], roach_state_table[i].last_targ_path)) {
                    blast_info("ROACH%d: Opt tones success", i + 1);
                } else {
                     blast_info("ROACH%d: Failed to optimize target tones", i + 1);
                }
                CommandData.roach[i].opt_tones = 0;
            }
        }
        // check data stream
        current_packet_count = roach_udp[i].roach_packet_count;
        if (current_packet_count == last_packet_count) {
            packet_error_counter++;
        } else {
            roach_state_table[i].data_stream_error = 0;
            roach_state_table[i].is_streaming = 1;
            packet_error_counter = 0;
        }
        if (packet_error_counter > MAX_PACKET_COUNT_ERRORS) {
            roach_state_table[i].data_stream_error = 1;
            roach_state_table[i].is_streaming = 0;
            roach_state_table[i].state = ROACH_STATE_BOOT;
        }
        last_packet_count = current_packet_count;
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
    asprintf(&roach_state_table[ind].address, "roach%d", ind + 1);
    asprintf(&roach_state_table[ind].sweep_root_path, "%s/roach%d",
         roach_root_path, ind + 1);
    asprintf(&roach_state_table[ind].vna_path_ref, "%s/%s", roach_state_table[ind].sweep_root_path, "last_vna_path");
    asprintf(&roach_state_table[ind].targ_path_ref,
          "%s/%s", roach_state_table[ind].sweep_root_path, "last_targ_path");
    asprintf(&roach_state_table[ind].vna_path_root,
          "%s/vna", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].targ_path_root,
          "%s/targ", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].cal_path_root,
          "%s/cal", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].df_path_root,
          "%s/df", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].vna_amps_path[0], "%s/default_amps.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].vna_amps_path[1], "%s/vna_trf.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].targ_amps_path[0], "%s/default_targ_amps.dat",
          roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].targ_amps_path[1], "%s/targ_trf.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].targ_amps_path[2], "%s/last_targ_amps.dat",
          roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].iq_path_root, "%s/chops", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].df_path_root, "%s/df", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].freqlist_path, "%s/bb_targ_freqs.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].qdr_log, "%s/qdr_cal.log", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].ts, "%s/ts_log", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].find_kids_log, "%s/find_kids.log", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].opt_tones_log, "%s/opt_tones.log", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].random_phase_path, "%s/random_phases.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].path_to_last_attens, "%s/last_attens.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].path_to_lamp_response,
        "%s/lamp_response.dat", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].path_to_noise_comp,
        "%s/noise_comp.npy", roach_state_table[ind].sweep_root_path);
    asprintf(&roach_state_table[ind].path_to_df_comp,
        "%s/df_comp.npy", roach_state_table[ind].sweep_root_path);
    snprintf(path_to_vna_tarball[ind], sizeof(path_to_vna_tarball[ind]),
               "%s/roach%d_%s", roach_root_path, ind + 1, "last_vna_sweep.tar.gz");
    snprintf(path_to_targ_tarball[ind], sizeof(path_to_targ_tarball[ind]),
               "%s/roach%d_%s", roach_root_path, ind + 1, "last_targ_sweep.tar.gz");
    snprintf(path_to_iq_tarball[ind], sizeof(path_to_iq_tarball[ind]),
               "%s/roach%d_%s", roach_root_path, ind + 1, "last_iq_ts.tar.gz");
    snprintf(path_to_df_tarball[ind], sizeof(path_to_df_tarball[ind]),
               "%s/roach%d_%s", roach_root_path, ind + 1, "last_df_ts.tar.gz");
    snprintf(path_to_lamp_tarball[ind], sizeof(path_to_lamp_tarball[ind]),
               "%s/roach%d_%s", roach_root_path, ind + 1, "last_lamp_response.tar.gz");
    snprintf(path_to_dsvna[ind], sizeof(path_to_dsvna[ind]),
               "%s/dsvna.npy", roach_state_table[ind].sweep_root_path);
    if ((ind == 0)) {
        roach_state_table[ind].array = 500;
        roach_state_table[ind].lo_centerfreq = 540.0e6;
        roach_state_table[ind].vna_comb_len = VNA_COMB_LEN;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 1)) {
        roach_state_table[ind].array = 250;
        roach_state_table[ind].lo_centerfreq = 829.0e6;
        roach_state_table[ind].vna_comb_len = VNA_COMB_LEN;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 2)) {
        roach_state_table[ind].array = 350;
        roach_state_table[ind].lo_centerfreq = 850.0e6;
        roach_state_table[ind].vna_comb_len = VNA_COMB_LEN;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 3)) {
        roach_state_table[ind].array = 250;
        roach_state_table[ind].lo_centerfreq = 827.0e6;
        roach_state_table[ind].vna_comb_len = VNA_COMB_LEN;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    if ((ind == 4)) {
        roach_state_table[ind].array = 250;
        roach_state_table[ind].lo_centerfreq = 828.0e6;
        roach_state_table[ind].vna_comb_len = VNA_COMB_LEN;
        roach_state_table[ind].p_max_freq = 246.001234e6;
        roach_state_table[ind].p_min_freq = 1.02342e6;
        roach_state_table[ind].n_max_freq = -1.02342e6 + 5.0e4;
        roach_state_table[ind].n_min_freq = -246.001234e6 + 5.0e4;
    }
    roach_state_table[ind].which = ind + 1;
    pi_state_table[ind].which = ind + 1;
    roach_state_table[ind].pi_error_count = 0;
    pi_state_table[ind].port = NC2_PORT;
    asprintf(&pi_state_table[ind].address, "pi%d", ind + 1);
    roach_state_table[ind].dest_port = 64000 + ind;
    roach_state_table[ind].src_port = 64000 + ind;
    roach_state_table[ind].src_ip = IPv4(192, 168, 40, 71 + ind);
    roach_state_table[ind].has_qdr_cal = 0;
    roach_state_table[ind].qdr_fail = 0;
    roach_state_table[ind].has_tones = 0;
    roach_state_table[ind].has_targ_tones = 0;
    roach_state_table[ind].has_adc_cal = 0;
    roach_state_table[ind].has_ref_params = 0;
    roach_state_table[ind].is_streaming = 0;
    roach_state_table[ind].is_sweeping = 0;
    // roach_state_table[ind].is_averaging = 0;
    roach_state_table[ind].tone_finding_error = 0;
    roach_state_table[ind].sweep_fail = 0;
    roach_state_table[ind].has_vna_sweep = 0;
    roach_state_table[ind].has_targ_sweep = 0;
    roach_state_table[ind].has_firmware = 0;
    roach_state_table[ind].firmware_upload_fail = 0;
    roach_state_table[ind].n_watchdog_fails = 0;
    roach_state_table[ind].doing_full_loop = 0;
    roach_state_table[ind].doing_find_kids_loop = 0;
    roach_state_table[ind].doing_turnaround_loop = 0;
    roach_state_table[ind].data_stream_error = 0;
    roach_state_table[ind].waiting_for_lamp = 0;
    roach_state_table[ind].num_kids = 0;
    roach_state_table[ind].prev_num_kids = 0;
    roach_state_table[ind].avg_df_diff = 0;
    roach_state_table[ind].targ_sweep_span_used = CommandData.roach_params[ind].targ_sweep_span;
    CommandData.roach[ind].do_check_retune = 0;
    CommandData.roach[ind].auto_correct_freqs = 0;
    CommandData.roach[ind].auto_el_retune_top = 1;
    CommandData.roach[ind].auto_el_retune_bottom = 1;
    CommandData.roach[ind].enable_chop_lo = 1;
    // Don't create thread for Roach 4
    uint64_t roach_idx = ind;
    // if (roach_idx != 3) {
        // blast_info("Spawning command thread for roach%i...", ind + 1);
        // ph_thread_spawn((ph_thread_func)roach_cmd_loop, (void*) &ind);
    ph_thread_spawn((ph_thread_func)roach_cmd_loop, (void*) roach_idx);
        // blast_info("Spawned command thread for roach%i", ind + 1);
    // }
    return 0;
}

/* Function: write_roach_channels_5hz
 * ----------------------------------
 * Populates 5 Hz frame data
 */
void write_roach_channels_5hz(void)
{
    int i;
    static int firsttime = 1;
    static channel_t *IsChoppingLoAddr[NUM_ROACHES];
    static channel_t *RoachPktCtAddr[NUM_ROACHES];
    static channel_t *RoachValidPktCtAddr[NUM_ROACHES];
    static channel_t *RoachInvalidPktCtAddr[NUM_ROACHES];
    static channel_t *roachStatusFieldAddr[NUM_ROACHES];
    // static channel_t *RoachIsAveragingAddr[NUM_ROACHES];
    static channel_t *RoachLampNow;
    static char channel_name_pkt_ct[128] = { 0 };
    static char channel_name_valid_pkt_ct[128] = { 0 };
    static char channel_name_invalid_pkt_ct[128] = { 0 };
    static char channel_name_is_chopping_lo[128] = { 0 };
    // static char channel_name_roach_is_averaging[128] = { 0 };
    static char channel_name_roach_lamp_now[128] = { 0 };
    static char channel_name_roach_status[128] = { 0 };

    if (firsttime) {
        firsttime = 0;
        snprintf(channel_name_roach_lamp_now, sizeof(channel_name_roach_lamp_now),
                "roach_lamp_now");
        for (i = 0; i < NUM_ROACHES; i++) {
            snprintf(channel_name_is_chopping_lo, sizeof(channel_name_is_chopping_lo),
                    "is_chopping_lo_roach%d", i + 1);
            snprintf(channel_name_pkt_ct, sizeof(channel_name_pkt_ct),
                    "packet_count_mcp_roach%d", i + 1);
            snprintf(channel_name_valid_pkt_ct,
                    sizeof(channel_name_valid_pkt_ct),
                    "packet_count_valid_mcp_roach%d", i + 1);
            snprintf(channel_name_invalid_pkt_ct,
                    sizeof(channel_name_invalid_pkt_ct),
                    "packet_count_invalid_mcp_roach%d", i + 1);
            snprintf(channel_name_roach_status, sizeof(channel_name_roach_status),
                        "status_roach%d", i + 1);
            // snprintf(channel_name_roach_is_averaging,
            // sizeof(channel_name_roach_is_averaging), "is_averaging_roach%d", i + 1);
            IsChoppingLoAddr[i] = channels_find_by_name(channel_name_is_chopping_lo);
            RoachPktCtAddr[i] = channels_find_by_name(channel_name_pkt_ct);
            RoachValidPktCtAddr[i] = channels_find_by_name(
                    channel_name_valid_pkt_ct);
            RoachInvalidPktCtAddr[i] = channels_find_by_name(
                    channel_name_invalid_pkt_ct);
            // RoachIsAveragingAddr[i] = channels_find_by_name(channel_name_roach_is_averaging);
            roachStatusFieldAddr[i] = channels_find_by_name(channel_name_roach_status);
        }
        RoachLampNow = channels_find_by_name(channel_name_roach_lamp_now);
    }
    for (i = 0; i < NUM_ROACHES; i++) {
        SET_UINT8(IsChoppingLoAddr[i], CommandData.roach[i].is_chopping_lo);
        SET_UINT32(RoachPktCtAddr[i], roach_udp[i].roach_packet_count);
        SET_UINT32(RoachValidPktCtAddr[i],
        roach_udp[i].roach_valid_packet_count);
        SET_UINT32(RoachInvalidPktCtAddr[i],
        roach_udp[i].roach_invalid_packet_count);
        // SET_UINT8(RoachIsAveragingAddr[i], roach_state_table[i].is_averaging);
        // Make Roach status field
        int roach_status_field = 0;
        if (roach_state_table[i].is_sweeping) {
            CommandData.roach[i].is_sweeping = 1;
        } else {
            CommandData.roach[i].is_sweeping = 0;
        }
        roach_status_field |= (roach_state_table[i].has_qdr_cal & 0x0001);
        roach_status_field |= (((uint32_t)roach_state_table[i].full_loop_fail) << 1);
        roach_status_field |= (((uint32_t)roach_state_table[i].has_targ_tones) << 2);
        roach_status_field |= (((uint32_t)roach_state_table[i].is_streaming) << 3);
        // is_sweeping is 2 bits
        roach_status_field |= (((uint32_t)roach_state_table[i].is_sweeping) << 4);
        roach_status_field |= (((uint32_t)roach_state_table[i].has_vna_sweep) << 6);
        roach_status_field |= (((uint32_t)roach_state_table[i].has_targ_sweep) << 7);
        roach_status_field |= (((uint32_t)roach_state_table[i].write_flag) << 8);
        roach_status_field |= (((uint32_t)roach_state_table[i].has_ref_params) << 9);
        roach_status_field |= (((uint32_t)CommandData.roach[i].ext_ref) << 10);
        roach_status_field |= (((uint32_t)roach_state_table[i].has_vna_tones) << 11);
        roach_status_field |= (((uint32_t)roach_state_table[i].sweep_fail) << 12);
        roach_status_field |= (((uint32_t)roach_state_table[i].qdr_fail) << 13);
        roach_status_field |= (((uint32_t)roach_state_table[i].firmware_upload_fail) << 14);
        roach_status_field |= (((uint32_t)roach_state_table[i].has_firmware) << 15);
        roach_status_field |= (((uint32_t)roach_state_table[i].tone_finding_error) << 16);
        roach_status_field |= (((uint32_t)roach_state_table[i].katcp_connect_error) << 18);
        roach_status_field |= (((uint32_t)roach_state_table[i].is_compressing_data) << 19);
        roach_status_field |= (((uint32_t)roach_state_table[i].doing_full_loop) << 20);
        roach_status_field |= (((uint32_t)roach_state_table[i].doing_find_kids_loop) << 21);
        roach_status_field |= (((uint32_t)roach_state_table[i].is_finding_kids) << 22);
        roach_status_field |= (((uint32_t)roach_state_table[i].trnaround_loop_fail) << 23);
        roach_status_field |= (((uint32_t)roach_state_table[i].doing_turnaround_loop) << 24);
        roach_status_field |= (((uint32_t)(CommandData.roach[i].auto_el_retune_top ||
                                           CommandData.roach[i].auto_el_retune_bottom)) << 25);
        roach_status_field |= (((uint32_t)CommandData.roach[i].enable_chop_lo) << 26);
        roach_status_field |= (((uint32_t)CommandData.roach[i].is_chopping_lo) << 27);
        roach_status_field |= (((uint32_t)roach_state_table[i].pi_reboot_warning) << 28);
        roach_status_field |= (((uint32_t)roach_state_table[i].data_stream_error) << 29);
        // roach_status_field |= (((uint32_t)roach_state_table[i].waiting_for_lamp) << 30);
        roach_status_field |= (((uint32_t)CommandData.roach[i].on_res) << 30);
        roach_status_field |= (((uint32_t)CommandData.roach[i].has_lamp_control) << 31);
        SET_UINT32(roachStatusFieldAddr[i], roach_status_field);
    }
    SET_UINT8(RoachLampNow, roach_lamp_now);
}


/* Function: write_roach_channels_1hz
 * ----------------------------------
 * Populates 1 Hz frame data
 */
void write_roach_channels_1hz(void)
{
    int i, j, k, i_chan;
    static int firsttime = 1;
    static channel_t *PiTempAddr[NUM_ROACHES];
    static channel_t *EnableRoachLamp;
    static channel_t *AvgDfDiffAddr[NUM_ROACHES];
    static channel_t *LoFreqReqAddr[NUM_ROACHES];
    static channel_t *LoFreqReadAddr[NUM_ROACHES];
    static channel_t *RoachFridgeCycleWarning;
    static channel_t *RoachAutoCheckCycle;
    static channel_t *DfRetuneThreshAddr[NUM_ROACHES];
    static channel_t *DfDiffRetuneThreshAddr[NUM_ROACHES];
    static channel_t *PiErrorCountAddr[NUM_ROACHES];
    static channel_t *RoachStateAddr[NUM_ROACHES];
    static channel_t *CmdRoachParSmoothAddr[NUM_ROACHES];
    static channel_t *CmdRoachParPeakThreshAddr[NUM_ROACHES];
    static channel_t *CmdRoachParSpaceThreshAddr[NUM_ROACHES];
    static channel_t *CmdRoachParSetInAttenAddr[NUM_ROACHES];
    static channel_t *CmdRoachParSetOutAttenAddr[NUM_ROACHES];

    static channel_t *CmdRoachParReadInAttenAddr[NUM_ROACHES];
    static channel_t *CmdRoachParReadOutAttenAddr[NUM_ROACHES];
    static channel_t *FlagsKidsAddr[NUM_ROACHES][NUM_FLAG_CHANNELS_PER_ROACH];
    static channel_t *nKidsFoundAddr[NUM_ROACHES];
    static channel_t *PrevNkidsFoundAddr[NUM_ROACHES];
    static channel_t *nKidsGoodAddr[NUM_ROACHES];
    static channel_t *nKidsBadAddr[NUM_ROACHES];
    static channel_t *CurrentNTonesAddr[NUM_ROACHES];
    static channel_t *LoCenterFreqAddr[NUM_ROACHES];
    static channel_t *NFlagThreshFieldAddr[NUM_ROACHES];
    static channel_t *NKidsTlmRoach[NUM_ROACHES];
    static channel_t *SKidsTlmRoach[NUM_ROACHES];
    static channel_t *RoachTlmMode;
    static channel_t *RoachAdcIRmsAddr[NUM_ROACHES];
    static channel_t *RoachAdcQRmsAddr[NUM_ROACHES];
    static channel_t *RoachScanTrigger;
    static channel_t *PowPerToneAddr[NUM_ROACHES];
    static channel_t *FpgaClockFreqAddr[NUM_ROACHES];
    static channel_t *TargSweepSpanAddr[NUM_ROACHES];
    static channel_t *TrndSweepSpanAddr[NUM_ROACHES];
    static channel_t *MedianDfAddr[NUM_ROACHES];
    uint16_t n_good_kids = 0;
    static char channel_name_pi_temp[128] = { 0 };
    static char channel_name_enable_roach_lamp[128] = { 0 };
    static char channel_name_roach_fridge_cycle_warning[128] = { 0 };
    static char channel_name_roach_auto_check_cycle[128] = { 0 };
    static char channel_name_df_retune_thresh[128] = { 0 };
    static char channel_name_df_diff_retune_thresh[128] = { 0 };
    static char channel_name_avg_df_diff[128] = { 0 };
    static char channel_name_flags_kids[128] = { 0 };
    static char channel_name_kids_found[128] = { 0 };
    static char channel_name_prev_kids_found[128] = { 0 };
    static char channel_name_kids_good[128] = { 0 };
    static char channel_name_kids_bad[128] = { 0 };
    static char channel_name_current_ntones[128] = { 0 };
    static char channel_name_lo_center_freq[128] = { 0 };
    static char channel_name_nflag_thresh[128] = { 0 };
    static char channel_name_nkids_tlm[128] = { 0 };
    static char channel_name_skids_tlm[128] = { 0 };
    static char channel_name_roach_state[128] = { 0 };
    static char channel_name_pi_error_count[128] = { 0 };
    static char channel_name_cmd_roach_par_smooth[128] = { 0 };
    static char channel_name_cmd_roach_par_peak_thresh[128] = { 0 };
    static char channel_name_cmd_roach_par_space_thresh[128] = { 0 };
    static char channel_name_cmd_roach_par_set_in_atten[128] = { 0 };
    static char channel_name_cmd_roach_par_set_out_atten[128] = { 0 };
    static char channel_name_cmd_roach_par_read_in_atten[128] = { 0 };
    static char channel_name_cmd_roach_par_read_out_atten[128] = { 0 };
    static char channel_name_roach_adcI_rms[128] = { 0 };
    static char channel_name_roach_adcQ_rms[128] = { 0 };
    static char channel_name_roach_pow_per_tone[128] = { 0 };
    static char channel_name_lo_freq_req[128] = { 0 };
    static char channel_name_lo_freq_read[128] = { 0 };
    static char channel_name_fpga_clock_freq[128] = { 0 };
    static char channel_name_targ_sweep_span[128] = { 0 };
    static char channel_name_trnd_sweep_span[128] = { 0 };
    static char channel_name_median_df[128] = { 0 };
    uint16_t flag = 0;
    if (firsttime) {
        firsttime = 0;
        snprintf(channel_name_enable_roach_lamp, sizeof(channel_name_enable_roach_lamp),
               "roach_enable_cal_pulse");
        snprintf(channel_name_roach_auto_check_cycle, sizeof(channel_name_roach_auto_check_cycle),
               "roach_auto_check_cycle");
        snprintf(channel_name_roach_fridge_cycle_warning, sizeof(channel_name_roach_fridge_cycle_warning),
               "roach_fridge_cycle_warning");
        for (i = 0; i < NUM_ROACHES; i++) {
            for (j = 0; j < NUM_FLAG_CHANNELS_PER_ROACH; j++) {
                snprintf(channel_name_flags_kids, sizeof(channel_name_flags_kids),
                        "flags_kids%04d_roach%d", j*16, i + 1);
                FlagsKidsAddr[i][j] = channels_find_by_name(channel_name_flags_kids);
            }
            snprintf(channel_name_pi_temp, sizeof(channel_name_pi_temp),
                        "pi_temp_roach%d", i + 1);
            snprintf(channel_name_avg_df_diff, sizeof(channel_name_avg_df_diff),
                        "avg_df_diff_roach%d", i + 1);
            snprintf(channel_name_fpga_clock_freq, sizeof(channel_name_fpga_clock_freq),
                        "fpga_clock_freq_roach%d", i + 1);
            snprintf(channel_name_lo_freq_req, sizeof(channel_name_lo_freq_req),
                        "lo_freq_req_roach%d", i + 1);
            snprintf(channel_name_lo_freq_read, sizeof(channel_name_lo_freq_read),
                        "lo_freq_read_roach%d", i + 1);
            snprintf(channel_name_roach_pow_per_tone, sizeof(channel_name_roach_pow_per_tone),
                        "pow_per_tone_roach%d", i + 1);
            snprintf(channel_name_df_retune_thresh, sizeof(channel_name_df_retune_thresh),
                        "df_retune_thresh_roach%d", i + 1);
            snprintf(channel_name_df_diff_retune_thresh, sizeof(channel_name_df_diff_retune_thresh),
                        "df_diff_retune_thresh_roach%d", i + 1);
            snprintf(channel_name_kids_found, sizeof(channel_name_kids_found),
                        "nkids_found_roach%d", i + 1);
            snprintf(channel_name_prev_kids_found, sizeof(channel_name_prev_kids_found),
                        "prev_nkids_found_roach%d", i + 1);
            snprintf(channel_name_kids_good, sizeof(channel_name_kids_good),
                        "nkids_good_roach%d", i + 1);
            snprintf(channel_name_kids_bad, sizeof(channel_name_kids_bad),
                        "nkids_bad_roach%d", i + 1);
            snprintf(channel_name_current_ntones, sizeof(channel_name_current_ntones),
                        "current_ntones_roach%d", i + 1);
            snprintf(channel_name_lo_center_freq, sizeof(channel_name_lo_center_freq),
                        "lo_center_freq_roach%d", i + 1);
            snprintf(channel_name_nflag_thresh, sizeof(channel_name_nflag_thresh),
                        "nflag_thresh_roach%d", i + 1);
            snprintf(channel_name_nkids_tlm, sizeof(channel_name_nkids_tlm),
                        "nkids_tlm_roach%d", i + 1);
            snprintf(channel_name_skids_tlm, sizeof(channel_name_skids_tlm),
                        "skids_tlm_roach%d", i + 1);
            snprintf(channel_name_roach_state,
                    sizeof(channel_name_roach_state), "state_roach%d", i + 1);
            snprintf(channel_name_pi_error_count,
                    sizeof(channel_name_pi_error_count), "pi_error_count_roach%d", i + 1);
            snprintf(channel_name_cmd_roach_par_smooth,
                    sizeof(channel_name_cmd_roach_par_smooth), "fk_smooth_scale_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_peak_thresh,
                    sizeof(channel_name_cmd_roach_par_peak_thresh), "fk_peak_thresh_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_space_thresh,
                    sizeof(channel_name_cmd_roach_par_space_thresh), "fk_space_thresh_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_set_in_atten,
                    sizeof(channel_name_cmd_roach_par_set_in_atten), "set_atten_in_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_set_out_atten,
                    sizeof(channel_name_cmd_roach_par_set_out_atten), "set_atten_out_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_read_in_atten,
                    sizeof(channel_name_cmd_roach_par_read_in_atten), "read_atten_in_roach%d",
                    i + 1);
            snprintf(channel_name_cmd_roach_par_read_out_atten,
                    sizeof(channel_name_cmd_roach_par_read_out_atten), "read_atten_out_roach%d",
                    i + 1);
            snprintf(channel_name_roach_adcI_rms,
                    sizeof(channel_name_roach_adcI_rms), "adcI_rms_roach%d",
                    i + 1);
            snprintf(channel_name_roach_adcQ_rms,
                    sizeof(channel_name_roach_adcQ_rms), "adcQ_rms_roach%d",
                    i + 1);
            snprintf(channel_name_targ_sweep_span,
                    sizeof(channel_name_targ_sweep_span), "targ_sweep_span_roach%d",
                    i + 1);
            snprintf(channel_name_trnd_sweep_span,
                    sizeof(channel_name_trnd_sweep_span), "trnd_sweep_span_roach%d",
                    i + 1);
            snprintf(channel_name_median_df,
                    sizeof(channel_name_median_df), "median_df_roach%d",
                    i + 1);
            PiTempAddr[i] = channels_find_by_name(channel_name_pi_temp);
            AvgDfDiffAddr[i] = channels_find_by_name(channel_name_avg_df_diff);
            FpgaClockFreqAddr[i] = channels_find_by_name(channel_name_fpga_clock_freq);
            LoFreqReqAddr[i] = channels_find_by_name(channel_name_lo_freq_req);
            LoFreqReadAddr[i] = channels_find_by_name(channel_name_lo_freq_read);
            PowPerToneAddr[i] = channels_find_by_name(channel_name_roach_pow_per_tone);
            DfRetuneThreshAddr[i] = channels_find_by_name(channel_name_df_retune_thresh);
            DfDiffRetuneThreshAddr[i] = channels_find_by_name(channel_name_df_diff_retune_thresh);
            PiErrorCountAddr[i] = channels_find_by_name(channel_name_pi_error_count);
            RoachStateAddr[i] = channels_find_by_name(channel_name_roach_state);
            CmdRoachParSmoothAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_smooth);
            CmdRoachParPeakThreshAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_peak_thresh);
            CmdRoachParSpaceThreshAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_space_thresh);
            CmdRoachParSetInAttenAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_set_in_atten);
            CmdRoachParSetOutAttenAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_set_out_atten);
            CmdRoachParReadInAttenAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_read_in_atten);
            CmdRoachParReadOutAttenAddr[i] = channels_find_by_name(channel_name_cmd_roach_par_read_out_atten);
            RoachAdcIRmsAddr[i] = channels_find_by_name(channel_name_roach_adcI_rms);
            RoachAdcQRmsAddr[i] = channels_find_by_name(channel_name_roach_adcQ_rms);
            nKidsFoundAddr[i] = channels_find_by_name(channel_name_kids_found);
            PrevNkidsFoundAddr[i] = channels_find_by_name(channel_name_prev_kids_found);
            nKidsGoodAddr[i] = channels_find_by_name(channel_name_kids_good);
            nKidsBadAddr[i] = channels_find_by_name(channel_name_kids_bad);
            CurrentNTonesAddr[i] = channels_find_by_name(channel_name_current_ntones);
            LoCenterFreqAddr[i] = channels_find_by_name(channel_name_lo_center_freq);
            NFlagThreshFieldAddr[i] = channels_find_by_name(channel_name_nflag_thresh);
            NKidsTlmRoach[i] = channels_find_by_name(channel_name_nkids_tlm);
            SKidsTlmRoach[i] = channels_find_by_name(channel_name_skids_tlm);
            TargSweepSpanAddr[i] = channels_find_by_name(channel_name_targ_sweep_span);
            TrndSweepSpanAddr[i] = channels_find_by_name(channel_name_trnd_sweep_span);
            MedianDfAddr[i] = channels_find_by_name(channel_name_median_df);
        }
        EnableRoachLamp = channels_find_by_name("roach_enable_cal_pulse");
        RoachScanTrigger = channels_find_by_name("scan_retune_trigger_roach");
        RoachTlmMode = channels_find_by_name("roach_tlm_mode");
        RoachAutoCheckCycle = channels_find_by_name("roach_auto_check_cycle");
        RoachFridgeCycleWarning = channels_find_by_name("roach_fridge_cycle_warning");
    }
    for (i = 0; i < NUM_ROACHES; i++) {
        n_good_kids = 0;
        for (j = 0; j < NUM_FLAG_CHANNELS_PER_ROACH; j++) {
            flag = 0;
            for (k = 0; k < 16; k++) {
                i_chan = j * 16 + k;
                if (i_chan < roach_state_table[i].num_kids) {
                    flag |= (((uint16_t)roach_state_table[i].out_of_range[j]) << k);
                    // n_good_kids += (uint16_t)roach_state_table[i].out_of_range[j];
                } else {
                    flag |= (1 << k);
                }
            }
            SET_UINT16(FlagsKidsAddr[i][j], flag);
        }
        SET_FLOAT(PiTempAddr[i], roach_state_table[i].pi_temp);
        SET_FLOAT(AvgDfDiffAddr[i], roach_state_table[i].avg_df_diff);
        SET_UINT16(nKidsFoundAddr[i], roach_state_table[i].num_kids);
        SET_UINT16(PrevNkidsFoundAddr[i], roach_state_table[i].prev_num_kids);
        SET_UINT16(nKidsGoodAddr[i], n_good_kids);
        SET_UINT16(nKidsBadAddr[i], (roach_state_table[i].n_outofrange));
        SET_UINT8(RoachStateAddr[i], roach_state_table[i].state);
        SET_UINT8(PiErrorCountAddr[i], roach_state_table[i].pi_error_count);
        SET_INT32(DfRetuneThreshAddr[i], CommandData.roach_params[i].df_retune_threshold);
        SET_INT32(DfDiffRetuneThreshAddr[i], CommandData.roach_params[i].df_diff_retune_threshold);
        SET_SCALED_VALUE(CmdRoachParSmoothAddr[i], CommandData.roach_params[i].smoothing_scale);
        SET_SCALED_VALUE(CmdRoachParPeakThreshAddr[i], CommandData.roach_params[i].peak_threshold);
        SET_SCALED_VALUE(CmdRoachParSpaceThreshAddr[i], CommandData.roach_params[i].spacing_threshold);
        SET_SCALED_VALUE(TargSweepSpanAddr[i], roach_state_table[i].targ_sweep_span_used);
        SET_SCALED_VALUE(TrndSweepSpanAddr[i], CommandData.roach_params[i].trnd_sweep_span);
        SET_FLOAT(MedianDfAddr[i], roach_state_table[i].median_df);

        SET_FLOAT(CmdRoachParSetInAttenAddr[i], CommandData.roach_params[i].set_in_atten);
        SET_FLOAT(CmdRoachParSetOutAttenAddr[i], CommandData.roach_params[i].set_out_atten);
        SET_FLOAT(CmdRoachParReadInAttenAddr[i], CommandData.roach_params[i].read_in_atten);
        SET_FLOAT(CmdRoachParReadOutAttenAddr[i], CommandData.roach_params[i].read_out_atten);
        SET_FLOAT(RoachAdcIRmsAddr[i], roach_state_table[i].adc_rms[0]);
        SET_FLOAT(RoachAdcQRmsAddr[i], roach_state_table[i].adc_rms[1]);
        SET_FLOAT(PowPerToneAddr[i], CommandData.roach_params[i].dBm_per_tone);
        SET_FLOAT(FpgaClockFreqAddr[i], roach_state_table[i].fpga_clock_freq);
        SET_UINT16(CurrentNTonesAddr[i], roach_state_table[i].current_ntones);
        SET_FLOAT(LoCenterFreqAddr[i], roach_state_table[i].lo_centerfreq/1.0e6);
        SET_UINT16(NFlagThreshFieldAddr[i], CommandData.roach[i].n_outofrange_thresh);
        SET_UINT16(NKidsTlmRoach[i], CommandData.num_channels_all_roaches[i]);
        SET_UINT16(SKidsTlmRoach[i], CommandData.roach_tlm[i].kid);
        SET_FLOAT(LoFreqReqAddr[i], roach_state_table[i].lo_freq_req);
        SET_FLOAT(LoFreqReadAddr[i], roach_state_table[i].lo_freq_read);
    }
    SET_UINT8(EnableRoachLamp, CommandData.enable_roach_lamp);
    SET_UINT8(RoachFridgeCycleWarning, is_cycling);
    SET_UINT8(RoachAutoCheckCycle, CommandData.roach_run_cycle_checker);
    SET_UINT16(RoachTlmMode, CommandData.roach_tlm_mode);
    SET_UINT8(RoachScanTrigger, (CommandData.trigger_roach_tuning_check_top |
                                 CommandData.trigger_roach_tuning_check_bottom));
}
