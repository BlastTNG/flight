/* tx_struct.c: contains the channel specificiation lists
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp licensed under the GNU General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <limits.h>
#include "channels.h"
#include "calibrate.h"
#include "bbc_pci.h"

/* card name to (node number, bus number) mapping */
#define ADC1_C	  0, 0
#define ADC1_D	  1, 0
#define ADC1_A1	  2, 0
#define ADC1_A2	  3, 0
#define ADC2_C	  4, 0
#define ADC2_D	  5, 0
#define ADC2_A1	  6, 0
#define LOOP1	 20, 0
#define DECOM	 50, 0

#define CAL16(m,b) ((m)*M_16PRE), ((b) + B_16PRE*(m)*M_16PRE)
#define CAL32(m,b) ((m)*M_32PRE), ((b) + B_32PRE*(m)*M_32PRE)
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T - 273.15)

#define U_NONE	  "","" 
#define U_T_C	  "Temperature","^oC"
#define U_V_DPS	  "Rate","^o/s"
#define U_V_MPS	  "Speed","m/s"
#define U_V_KPH	  "Speed","km/hr"
#define U_ALT_M	  "Altitude","m"
#define U_P_DEG	  "Position","^o"
#define U_PH_DEG  "Phase","^o"
#define U_LA_DEG  "Latitude","^o"
#define U_LO_DEG  "Longitude","^o"
#define U_D_DEG	  "Direction","^o"
#define U_V_V	  "Voltage","V"
#define U_I_A	  "Current","A"
#define U_T_MS	  "Time","ms"
#define U_R_O	  "Resistance","Ohms"
#define U_RATE	  "Rate", "bps"

struct ChannelStruct WideSlowChannels[] = {
  {"time",         'w',  LOOP1,  28,            1.0,          0.0, 'U', U_NONE}, 
  {"time_usec",    'w',  LOOP1,  30,            1.0,          0.0, 'U', U_NONE},
  END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  //status and sync channels for handshaking with bbus nodes
  {"status00",     'r', ADC1_C,  63,		1.0,          0.0, 'u', U_NONE},
  {"status01",     'r', ADC1_D,  63,		1.0,          0.0, 'u', U_NONE},
  {"status02",     'r', ADC1_A1, 63,		1.0,          0.0, 'u', U_NONE},
  {"status03",     'r', ADC1_A2, 63,		1.0,          0.0, 'u', U_NONE},
  {"status04",     'r', ADC2_C,  63,		1.0,          0.0, 'u', U_NONE},
  {"status05",     'r', ADC2_D,  63,		1.0,          0.0, 'u', U_NONE},
  {"status06",     'r', ADC2_A1, 63,		1.0,          0.0, 'u', U_NONE},
  {"sync00",       'w', ADC1_C,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync01",       'w', ADC1_D,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync02",       'w', ADC1_A1, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync03",       'w', ADC1_A2, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync04",       'w', ADC2_C,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync05",       'w', ADC2_D,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync06",       'w', ADC2_A1, 63,            1.0,          0.0, 'u', U_NONE},

  //phase of AC DAC signals (NB: overflowed to analog daughter)
  {"dac1_00_phase",'w', ADC1_A1,  0,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_01_phase",'w', ADC1_A1,  1,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_02_phase",'w', ADC1_A1,  2,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_03_phase",'w', ADC1_A1,  3,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_04_phase",'w', ADC1_A1,  4,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_05_phase",'w', ADC1_A1,  5,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_06_phase",'w', ADC1_A1,  6,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_07_phase",'w', ADC1_A1,  7,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_08_phase",'w', ADC1_A1,  8,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_09_phase",'w', ADC1_A1,  9,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_10_phase",'w', ADC1_A1, 10,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_11_phase",'w', ADC1_A1, 11,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_12_phase",'w', ADC1_A1, 12,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_13_phase",'w', ADC1_A1, 13,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_14_phase",'w', ADC1_A1, 14,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_15_phase",'w', ADC1_A1, 15,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_16_phase",'w', ADC1_A1, 16,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_17_phase",'w', ADC1_A1, 17,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_18_phase",'w', ADC1_A1, 18,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_19_phase",'w', ADC1_A1, 19,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_20_phase",'w', ADC1_A1, 20,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_21_phase",'w', ADC1_A1, 21,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_22_phase",'w', ADC1_A1, 22,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_23_phase",'w', ADC1_A1, 23,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_24_phase",'w', ADC1_A1, 24,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_25_phase",'w', ADC1_A1, 25,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_26_phase",'w', ADC1_A1, 26,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_27_phase",'w', ADC1_A1, 27,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_28_phase",'w', ADC1_A1, 28,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_29_phase",'w', ADC1_A1, 29,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_30_phase",'w', ADC1_A1, 30,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac1_31_phase",'w', ADC1_A1, 31,  360.0/65536.0,          0.0,'u',U_PH_DEG},

  {"dac2_00_phase",'w', ADC2_A1,  0,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_01_phase",'w', ADC2_A1,  1,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_02_phase",'w', ADC2_A1,  2,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_03_phase",'w', ADC2_A1,  3,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_04_phase",'w', ADC2_A1,  4,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_05_phase",'w', ADC2_A1,  5,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_06_phase",'w', ADC2_A1,  6,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_07_phase",'w', ADC2_A1,  7,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_08_phase",'w', ADC2_A1,  8,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_09_phase",'w', ADC2_A1,  9,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_10_phase",'w', ADC2_A1, 10,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_11_phase",'w', ADC2_A1, 11,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_12_phase",'w', ADC2_A1, 12,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_13_phase",'w', ADC2_A1, 13,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_14_phase",'w', ADC2_A1, 14,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_15_phase",'w', ADC2_A1, 15,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_16_phase",'w', ADC2_A1, 16,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_17_phase",'w', ADC2_A1, 17,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_18_phase",'w', ADC2_A1, 18,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_19_phase",'w', ADC2_A1, 19,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_20_phase",'w', ADC2_A1, 20,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_21_phase",'w', ADC2_A1, 21,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_22_phase",'w', ADC2_A1, 22,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_23_phase",'w', ADC2_A1, 23,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_24_phase",'w', ADC2_A1, 24,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_25_phase",'w', ADC2_A1, 25,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_26_phase",'w', ADC2_A1, 26,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_27_phase",'w', ADC2_A1, 27,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_28_phase",'w', ADC2_A1, 28,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_29_phase",'w', ADC2_A1, 29,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_30_phase",'w', ADC2_A1, 30,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac2_31_phase",'w', ADC2_A1, 31,  360.0/65536.0,          0.0,'u',U_PH_DEG},

  {"dac1_00_ampl", 'w', ADC1_D,   0,            1.0,          0.0, 'u', U_NONE},
  {"dac1_01_ampl", 'w', ADC1_D,   1,            1.0,          0.0, 'u', U_NONE},
  {"dac1_02_ampl", 'w', ADC1_D,   2,            1.0,          0.0, 'u', U_NONE},
  {"dac1_03_ampl", 'w', ADC1_D,   3,            1.0,          0.0, 'u', U_NONE},
  {"dac1_04_ampl", 'w', ADC1_D,   4,            1.0,          0.0, 'u', U_NONE},
  {"dac1_05_ampl", 'w', ADC1_D,   5,            1.0,          0.0, 'u', U_NONE},
  {"dac1_06_ampl", 'w', ADC1_D,   6,            1.0,          0.0, 'u', U_NONE},
  {"dac1_07_ampl", 'w', ADC1_D,   7,            1.0,          0.0, 'u', U_NONE},
  {"dac1_08_ampl", 'w', ADC1_D,   8,            1.0,          0.0, 'u', U_NONE},
  {"dac1_09_ampl", 'w', ADC1_D,   9,            1.0,          0.0, 'u', U_NONE},
  {"dac1_10_ampl", 'w', ADC1_D,  10,            1.0,          0.0, 'u', U_NONE},
  {"dac1_11_ampl", 'w', ADC1_D,  11,            1.0,          0.0, 'u', U_NONE},
  {"dac1_12_ampl", 'w', ADC1_D,  12,            1.0,          0.0, 'u', U_NONE},
  {"dac1_13_ampl", 'w', ADC1_D,  13,            1.0,          0.0, 'u', U_NONE},
  {"dac1_14_ampl", 'w', ADC1_D,  14,            1.0,          0.0, 'u', U_NONE},
  {"dac1_15_ampl", 'w', ADC1_D,  15,            1.0,          0.0, 'u', U_NONE},
  {"dac1_16_ampl", 'w', ADC1_D,  16,            1.0,          0.0, 'u', U_NONE},
  {"dac1_17_ampl", 'w', ADC1_D,  17,            1.0,          0.0, 'u', U_NONE},
  {"dac1_18_ampl", 'w', ADC1_D,  18,            1.0,          0.0, 'u', U_NONE},
  {"dac1_19_ampl", 'w', ADC1_D,  19,            1.0,          0.0, 'u', U_NONE},
  {"dac1_20_ampl", 'w', ADC1_D,  20,            1.0,          0.0, 'u', U_NONE},
  {"dac1_21_ampl", 'w', ADC1_D,  21,            1.0,          0.0, 'u', U_NONE},
  {"dac1_22_ampl", 'w', ADC1_D,  22,            1.0,          0.0, 'u', U_NONE},
  {"dac1_23_ampl", 'w', ADC1_D,  23,            1.0,          0.0, 'u', U_NONE},
  {"dac1_24_ampl", 'w', ADC1_D,  24,            1.0,          0.0, 'u', U_NONE},
  {"dac1_25_ampl", 'w', ADC1_D,  25,            1.0,          0.0, 'u', U_NONE},
  {"dac1_26_ampl", 'w', ADC1_D,  26,            1.0,          0.0, 'u', U_NONE},
  {"dac1_27_ampl", 'w', ADC1_D,  27,            1.0,          0.0, 'u', U_NONE},
  {"dac1_28_ampl", 'w', ADC1_D,  28,            1.0,          0.0, 'u', U_NONE},
  {"dac1_29_ampl", 'w', ADC1_D,  29,            1.0,          0.0, 'u', U_NONE},
  {"dac1_30_ampl", 'w', ADC1_D,  30,            1.0,          0.0, 'u', U_NONE},
  {"dac1_31_ampl", 'w', ADC1_D,  31,            1.0,          0.0, 'u', U_NONE},

  {"dac2_00_ampl", 'w', ADC2_D,   0,            1.0,          0.0, 'u', U_NONE},
  {"dac2_01_ampl", 'w', ADC2_D,   1,            1.0,          0.0, 'u', U_NONE},
  {"dac2_02_ampl", 'w', ADC2_D,   2,            1.0,          0.0, 'u', U_NONE},
  {"dac2_03_ampl", 'w', ADC2_D,   3,            1.0,          0.0, 'u', U_NONE},
  {"dac2_04_ampl", 'w', ADC2_D,   4,            1.0,          0.0, 'u', U_NONE},
  {"dac2_05_ampl", 'w', ADC2_D,   5,            1.0,          0.0, 'u', U_NONE},
  {"dac2_06_ampl", 'w', ADC2_D,   6,            1.0,          0.0, 'u', U_NONE},
  {"dac2_07_ampl", 'w', ADC2_D,   7,            1.0,          0.0, 'u', U_NONE},
  {"dac2_08_ampl", 'w', ADC2_D,   8,            1.0,          0.0, 'u', U_NONE},
  {"dac2_09_ampl", 'w', ADC2_D,   9,            1.0,          0.0, 'u', U_NONE},
  {"dac2_10_ampl", 'w', ADC2_D,  10,            1.0,          0.0, 'u', U_NONE},
  {"dac2_11_ampl", 'w', ADC2_D,  11,            1.0,          0.0, 'u', U_NONE},
  {"dac2_12_ampl", 'w', ADC2_D,  12,            1.0,          0.0, 'u', U_NONE},
  {"dac2_13_ampl", 'w', ADC2_D,  13,            1.0,          0.0, 'u', U_NONE},
  {"dac2_14_ampl", 'w', ADC2_D,  14,            1.0,          0.0, 'u', U_NONE},
  {"dac2_15_ampl", 'w', ADC2_D,  15,            1.0,          0.0, 'u', U_NONE},
  {"dac2_16_ampl", 'w', ADC2_D,  16,            1.0,          0.0, 'u', U_NONE},
  {"dac2_17_ampl", 'w', ADC2_D,  17,            1.0,          0.0, 'u', U_NONE},
  {"dac2_18_ampl", 'w', ADC2_D,  18,            1.0,          0.0, 'u', U_NONE},
  {"dac2_19_ampl", 'w', ADC2_D,  19,            1.0,          0.0, 'u', U_NONE},
  {"dac2_20_ampl", 'w', ADC2_D,  20,            1.0,          0.0, 'u', U_NONE},
  {"dac2_21_ampl", 'w', ADC2_D,  21,            1.0,          0.0, 'u', U_NONE},
  {"dac2_22_ampl", 'w', ADC2_D,  22,            1.0,          0.0, 'u', U_NONE},
  {"dac2_23_ampl", 'w', ADC2_D,  23,            1.0,          0.0, 'u', U_NONE},
  {"dac2_24_ampl", 'w', ADC2_D,  24,            1.0,          0.0, 'u', U_NONE},
  {"dac2_25_ampl", 'w', ADC2_D,  25,            1.0,          0.0, 'u', U_NONE},
  {"dac2_26_ampl", 'w', ADC2_D,  26,            1.0,          0.0, 'u', U_NONE},
  {"dac2_27_ampl", 'w', ADC2_D,  27,            1.0,          0.0, 'u', U_NONE},
  {"dac2_28_ampl", 'w', ADC2_D,  28,            1.0,          0.0, 'u', U_NONE},
  {"dac2_29_ampl", 'w', ADC2_D,  29,            1.0,          0.0, 'u', U_NONE},
  {"dac2_30_ampl", 'w', ADC2_D,  30,            1.0,          0.0, 'u', U_NONE},
  {"dac2_31_ampl", 'w', ADC2_D,  31,            1.0,          0.0, 'u', U_NONE},

  {"width_az",     'w', LOOP1,    0,       180.0/65535.0,   0.0, 'u', U_P_DEG},
  {"v_az",         'w', LOOP1,    1,         5.0/65535.0,   0.0, 'u', U_V_DPS},
  {"v_el",         'w', LOOP1,    2,         5.0/65535.0,   0.0, 'u', U_V_DPS},
  {"a_az",         'w', LOOP1,    3, 5.0/65535.0,   0.0, 'u',"accel","^o/s^2"},
  {"a_el",         'w', LOOP1,    4, 5.0/65535.0,   0.0, 'u',"accel","^o/s^2"},
  {"n_el",         'w', LOOP1,    5,                0.0,    0.0, 'u', U_NONE},
  {"height_el",    'w', LOOP1,    6,        90.0/65535.0,   0.0, 'u', U_P_DEG},
  {"az",           'w', LOOP1,    7,      360.0/65535.0,   0.0, 's', U_P_DEG},
  {"el",           'w', LOOP1,    8,       90.0/65535.0,  -5.0, 'u', U_P_DEG},
  {"az_ref",       'w', LOOP1,    9,      360.0/65535.0,   0.0, 's', U_P_DEG},
  {"el_ref",       'w', LOOP1,   10,       96.0/65535.0,  -10.0, 'u', U_P_DEG},
  // LOOP1 channels 12-15 are wide fast channels
  {"step1_ena_phase",  'w', LOOP1,16,           1.0,    0.0, 'u',  U_NONE},
  {"step1_start_phase",'w', LOOP1,17, 360.0/65536.0,    0.0, 'u',U_PH_DEG},
  {"step1_end_phase",  'w', LOOP1,18, 360.0/65536.0,    0.0, 'u',U_PH_DEG},
  {"step1_nsteps_phase",'w',LOOP1,19,           1.0,    0.0, 'u',  U_NONE},
  {"step1_time_phase", 'w', LOOP1,20,           1.0,    0.0, 'u',  U_T_MS},
  {"step1_ena_bias",   'w', LOOP1,21,           1.0,    0.0, 'u',  U_NONE},
  //TODO calibrate bias in volts? for commanding too?
  {"step1_start_bias", 'w', LOOP1,22,           1.0,    0.0, 'u',  U_NONE},
  {"step1_end_bias",   'w', LOOP1,24,           1.0,    0.0, 'u',  U_NONE},
  {"step1_n_bias",     'w', LOOP1,25,           1.0,    0.0, 'u',  U_NONE},
  {"step1_time_bias",  'w', LOOP1,26,           1.0,    0.0, 'u',  U_T_MS},
  {"step1_which_bias", 'w', LOOP1,27,           1.0,    0.0, 'u',  U_NONE},
  // LOOP1 channels 28-31 are wide slow channels
  {"disk_free",        'w', LOOP1,32,        1./250,    0.0, 'u',  U_NONE},
  {"bbc_fifo_size",    'w', LOOP1,33,        1./624,    0.0, 'u',  U_NONE},
  {"plover",           'w', LOOP1,34,           1.0,    0.0, 'u',  U_NONE},

  {"step2_ena_phase",  'w', LOOP1,35,           1.0,    0.0, 'u',  U_NONE},
  {"step2_start_phase",'w', LOOP1,36, 360.0/65536.0,    0.0, 'u',U_PH_DEG},
  {"step2_end_phase",  'w', LOOP1,37, 360.0/65536.0,    0.0, 'u',U_PH_DEG},
  {"step2_nsteps_phase",'w',LOOP1,38,           1.0,    0.0, 'u',  U_NONE},
  {"step2_time_phase", 'w', LOOP1,39,           1.0,    0.0, 'u',  U_T_MS},
  {"step2_ena_bias",   'w', LOOP1,40,           1.0,    0.0, 'u',  U_NONE},
  //TODO calibrate bias in volts? for commanding too?
  {"step2_start_bias", 'w', LOOP1,41,           1.0,    0.0, 'u',  U_NONE},
  {"step2_end_bias",   'w', LOOP1,42,           1.0,    0.0, 'u',  U_NONE},
  {"step2_n_bias",     'w', LOOP1,43,           1.0,    0.0, 'u',  U_NONE},
  {"step2_time_bias",  'w', LOOP1,44,           1.0,    0.0, 'u',  U_T_MS},
  {"step2_which_bias", 'w', LOOP1,45,           1.0,    0.0, 'u',  U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
  {"gyro0",   'r', ADC1_D,   0, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"gyro1",   'r', ADC1_D,   2, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"gyro2",   'r', ADC1_D,   4, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"gyro3",   'r', ADC1_D,   6, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"gyro4",   'r', ADC1_D,   8, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"gyro5",   'r', ADC1_D,  10, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"enc_table",    'r', ADC1_D,  12, 360.0/144000.0,           0.0,'U',U_P_DEG},
  {"enc_el",       'r', ADC1_D,  14,  360.0/72000.0,           0.0,'U',U_P_DEG},
  {"enc_az",       'r', ADC1_D,  16,  360.0/72000.0,           0.0,'U',U_P_DEG},

  {"a1_ch00",      'r', ADC1_A1,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch01",      'r', ADC1_A1,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch02",      'r', ADC1_A1,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch03",      'r', ADC1_A1,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch04",      'r', ADC1_A1,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch05",      'r', ADC1_A1, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch06",      'r', ADC1_A1, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch07",      'r', ADC1_A1, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch08",      'r', ADC1_A1, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch09",      'r', ADC1_A1, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch10",      'r', ADC1_A1, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch11",      'r', ADC1_A1, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch12",      'r', ADC1_A1, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch13",      'r', ADC1_A1, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch14",      'r', ADC1_A1, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch15",      'r', ADC1_A1, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch16",      'r', ADC1_A1, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch17",      'r', ADC1_A1, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch18",      'r', ADC1_A1, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch19",      'r', ADC1_A1, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch20",      'r', ADC1_A1, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch21",      'r', ADC1_A1, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch22",      'r', ADC1_A1, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch23",      'r', ADC1_A1, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a1_ch24",      'r', ADC1_A1, 48,      CAL32(1.0,          0.0),'U', U_V_V},

  {"a2_ch00",      'r', ADC1_A2,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch01",      'r', ADC1_A2,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch02",      'r', ADC1_A2,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch03",      'r', ADC1_A2,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch04",      'r', ADC1_A2,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch05",      'r', ADC1_A2, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch06",      'r', ADC1_A2, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch07",      'r', ADC1_A2, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch08",      'r', ADC1_A2, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch09",      'r', ADC1_A2, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch10",      'r', ADC1_A2, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch11",      'r', ADC1_A2, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch12",      'r', ADC1_A2, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch13",      'r', ADC1_A2, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch14",      'r', ADC1_A2, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch15",      'r', ADC1_A2, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch16",      'r', ADC1_A2, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch17",      'r', ADC1_A2, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch18",      'r', ADC1_A2, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch19",      'r', ADC1_A2, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch20",      'r', ADC1_A2, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch21",      'r', ADC1_A2, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch22",      'r', ADC1_A2, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch23",      'r', ADC1_A2, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a2_ch24",      'r', ADC1_A2, 48,      CAL32(1.0,          0.0),'U', U_V_V},

  {"a3_ch00",      'r', ADC2_A1,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch01",      'r', ADC2_A1,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch02",      'r', ADC2_A1,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch03",      'r', ADC2_A1,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch04",      'r', ADC2_A1,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch05",      'r', ADC2_A1, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch06",      'r', ADC2_A1, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch07",      'r', ADC2_A1, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch08",      'r', ADC2_A1, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch09",      'r', ADC2_A1, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch10",      'r', ADC2_A1, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch11",      'r', ADC2_A1, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch12",      'r', ADC2_A1, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch13",      'r', ADC2_A1, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch14",      'r', ADC2_A1, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch15",      'r', ADC2_A1, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch16",      'r', ADC2_A1, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch17",      'r', ADC2_A1, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch18",      'r', ADC2_A1, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch19",      'r', ADC2_A1, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch20",      'r', ADC2_A1, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch21",      'r', ADC2_A1, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch22",      'r', ADC2_A1, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch23",      'r', ADC2_A1, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"a3_ch24",      'r', ADC2_A1, 48,      CAL32(1.0,          0.0),'U', U_V_V},

  {"az_now",       'w', LOOP1,   12,    360.0/4294967295.0, 0.0, 'S', U_P_DEG},
  {"el_now",       'w', LOOP1,   14,     96.0/4294967295.0,-10.0, 'U', U_P_DEG},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
  {"in1_grp21_dig", 'r',ADC1_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"in1_grp65_dig", 'r',ADC1_D,  52,            1.0,          0.0, 'u', U_NONE},
  {"in1_grp43_dig", 'r',ADC1_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"out1_grp21_dig",'w',ADC1_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"out1_grp43_dig",'w',ADC1_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"out1_grp65_dig",'w',ADC1_D,  52,            1.0,          0.0, 'u', U_NONE},

  {"in2_grp21_dig", 'r',ADC2_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"in2_grp65_dig", 'r',ADC2_D,  52,            1.0,          0.0, 'u', U_NONE},
  {"in2_grp43_dig", 'r',ADC2_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"out2_grp21_dig",'w',ADC2_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"out2_grp43_dig",'w',ADC2_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"out2_grp65_dig",'w',ADC2_D,  52,            1.0,          0.0, 'u', U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",       'w', DECOM,   1,             1.0,          0.0, 'u', U_NONE},
  {"polarity",     'w', DECOM,   2,             1.0,          0.0, 'u', U_NONE},
  {"decom_unlock", 'w', DECOM,   3,             1.0,          0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
