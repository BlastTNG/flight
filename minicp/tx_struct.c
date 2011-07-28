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
#include "share/channels.h"
#include "calibrate.h"
#include "share/bbc_pci.h"

/* card name to (node number, bus number) mapping */
#define ADC1_C	  0, 0
#define ADC1_D	  1, 0
#define ADC1_A1	  2, 0
#define ADC1_A2	  3, 0
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
  END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  //status and sync channels for handshaking with bbus nodes
  {"status00",     'r', ADC1_C,  63,		1.0,          0.0, 'u', U_NONE},
  {"status01",     'r', ADC1_D,  63,		1.0,          0.0, 'u', U_NONE},
  {"status02",     'r', ADC1_A1, 63,		1.0,          0.0, 'u', U_NONE},
  {"status03",     'r', ADC1_A2, 63,		1.0,          0.0, 'u', U_NONE},
  {"sync00",       'w', ADC1_C,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync01",       'w', ADC1_D,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync02",       'w', ADC1_A1, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync03",       'w', ADC1_A2, 63,            1.0,          0.0, 'u', U_NONE},

  //phase of AC DAC signals (NB: overflowed to analog daughter)
  {"dac00_phase",  'w', ADC1_A1,  0,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac01_phase",  'w', ADC1_A1,  1,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac02_phase",  'w', ADC1_A1,  2,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac03_phase",  'w', ADC1_A1,  3,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac04_phase",  'w', ADC1_A1,  4,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac05_phase",  'w', ADC1_A1,  5,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac06_phase",  'w', ADC1_A1,  6,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac07_phase",  'w', ADC1_A1,  7,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac08_phase",  'w', ADC1_A1,  8,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac09_phase",  'w', ADC1_A1,  9,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac10_phase",  'w', ADC1_A1, 10,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac11_phase",  'w', ADC1_A1, 11,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac12_phase",  'w', ADC1_A1, 12,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac13_phase",  'w', ADC1_A1, 13,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac14_phase",  'w', ADC1_A1, 14,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac15_phase",  'w', ADC1_A1, 15,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac16_phase",  'w', ADC1_A1, 16,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac17_phase",  'w', ADC1_A1, 17,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac18_phase",  'w', ADC1_A1, 18,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac19_phase",  'w', ADC1_A1, 19,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac20_phase",  'w', ADC1_A1, 20,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac21_phase",  'w', ADC1_A1, 21,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac22_phase",  'w', ADC1_A1, 22,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac23_phase",  'w', ADC1_A1, 23,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac24_phase",  'w', ADC1_A1, 24,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac25_phase",  'w', ADC1_A1, 25,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac26_phase",  'w', ADC1_A1, 26,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac27_phase",  'w', ADC1_A1, 27,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac28_phase",  'w', ADC1_A1, 28,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac29_phase",  'w', ADC1_A1, 29,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac30_phase",  'w', ADC1_A1, 30,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"dac31_phase",  'w', ADC1_A1, 31,  360.0/65536.0,          0.0,'u',U_PH_DEG},

  {"dac00_ampl",   'w', ADC1_D,   0,            1.0,          0.0, 'u', U_NONE},
  {"dac01_ampl",   'w', ADC1_D,   1,            1.0,          0.0, 'u', U_NONE},
  {"dac02_ampl",   'w', ADC1_D,   2,            1.0,          0.0, 'u', U_NONE},
  {"dac03_ampl",   'w', ADC1_D,   3,            1.0,          0.0, 'u', U_NONE},
  {"dac04_ampl",   'w', ADC1_D,   4,            1.0,          0.0, 'u', U_NONE},
  {"dac05_ampl",   'w', ADC1_D,   5,            1.0,          0.0, 'u', U_NONE},
  {"dac06_ampl",   'w', ADC1_D,   6,            1.0,          0.0, 'u', U_NONE},
  {"dac07_ampl",   'w', ADC1_D,   7,            1.0,          0.0, 'u', U_NONE},
  {"dac08_ampl",   'w', ADC1_D,   8,            1.0,          0.0, 'u', U_NONE},
  {"dac09_ampl",   'w', ADC1_D,   9,            1.0,          0.0, 'u', U_NONE},
  {"dac10_ampl",   'w', ADC1_D,  10,            1.0,          0.0, 'u', U_NONE},
  {"dac11_ampl",   'w', ADC1_D,  11,            1.0,          0.0, 'u', U_NONE},
  {"dac12_ampl",   'w', ADC1_D,  12,            1.0,          0.0, 'u', U_NONE},
  {"dac13_ampl",   'w', ADC1_D,  13,            1.0,          0.0, 'u', U_NONE},
  {"dac14_ampl",   'w', ADC1_D,  14,            1.0,          0.0, 'u', U_NONE},
  {"dac15_ampl",   'w', ADC1_D,  15,            1.0,          0.0, 'u', U_NONE},
  {"dac16_ampl",   'w', ADC1_D,  16,            1.0,          0.0, 'u', U_NONE},
  {"dac17_ampl",   'w', ADC1_D,  17,            1.0,          0.0, 'u', U_NONE},
  {"dac18_ampl",   'w', ADC1_D,  18,            1.0,          0.0, 'u', U_NONE},
  {"dac19_ampl",   'w', ADC1_D,  19,            1.0,          0.0, 'u', U_NONE},
  {"dac20_ampl",   'w', ADC1_D,  20,            1.0,          0.0, 'u', U_NONE},
  {"dac21_ampl",   'w', ADC1_D,  21,            1.0,          0.0, 'u', U_NONE},
  {"dac22_ampl",   'w', ADC1_D,  22,            1.0,          0.0, 'u', U_NONE},
  {"dac23_ampl",   'w', ADC1_D,  23,            1.0,          0.0, 'u', U_NONE},
  {"dac24_ampl",   'w', ADC1_D,  24,            1.0,          0.0, 'u', U_NONE},
  {"dac25_ampl",   'w', ADC1_D,  25,            1.0,          0.0, 'u', U_NONE},
  {"dac26_ampl",   'w', ADC1_D,  26,            1.0,          0.0, 'u', U_NONE},
  {"dac27_ampl",   'w', ADC1_D,  27,            1.0,          0.0, 'u', U_NONE},
  {"dac28_ampl",   'w', ADC1_D,  28,            1.0,          0.0, 'u', U_NONE},
  {"dac29_ampl",   'w', ADC1_D,  29,            1.0,          0.0, 'u', U_NONE},
  {"dac30_ampl",   'w', ADC1_D,  30,            1.0,          0.0, 'u', U_NONE},
  {"dac31_ampl",   'w', ADC1_D,  31,            1.0,          0.0, 'u', U_NONE},

  {"width_az",     'w', LOOP1,    0,       180.0/65535.0,   0.0, 'u', U_P_DEG},
  {"v_az",         'w', LOOP1,    1,         5.0/65535.0,   0.0, 'u', U_V_DPS},
  {"v_el",         'w', LOOP1,    2,         5.0/65535.0,   0.0, 'u', U_V_DPS},
  {"a_az",         'w', LOOP1,    3, 5.0/65535.0,   0.0, 'u',"accel","^o/s^2"},
  {"a_el",         'w', LOOP1,    4, 5.0/65535.0,   0.0, 'u',"accel","^o/s^2"},
  {"n_el",         'w', LOOP1,    5,                0.0,    0.0, 'u', U_NONE},
  {"height_el",    'w', LOOP1,    6,        99.0/65535.0,   0.0, 'u', U_P_DEG},
  {"az",           'w', LOOP1,    7,      360.0/65535.0,   0.0, 's', U_P_DEG},
  {"el",           'w', LOOP1,    8,       99.0/65535.0,  -10.0, 'u', U_P_DEG},
  {"az_ref",       'w', LOOP1,    9,      360.0/65535.0,   0.0, 's', U_P_DEG},
  {"el_ref",       'w', LOOP1,   10,       99.0/65535.0,  -10.0, 'u', U_P_DEG},
  // LOOP1 channels 12-15 are wide fast channels
  {"step_ena_phase",   'w', LOOP1,16,           1.0,    0.0, 'u',  U_NONE},
  {"step_start_phase", 'w', LOOP1,17, 360.0/65536.0,    0.0, 'u',U_PH_DEG},
  {"step_end_phase",   'w', LOOP1,18, 360.0/65536.0,    0.0, 'u',U_PH_DEG},
  {"step_nsteps_phase",'w', LOOP1,19,           1.0,    0.0, 'u',  U_NONE},
  {"step_time_phase",  'w', LOOP1,20,           1.0,    0.0, 'u',  U_T_MS},
  {"step_ena_bias",    'w', LOOP1,21,           1.0,    0.0, 'u',  U_NONE},
  //TODO calibrate bias in volts? for commanding too?
  {"step_start_bias",  'w', LOOP1,22,           1.0,    0.0, 'u',  U_NONE},
  {"step_end_bias",    'w', LOOP1,24,           1.0,    0.0, 'u',  U_NONE},
  {"step_n_bias",      'w', LOOP1,25,           1.0,    0.0, 'u',  U_NONE},
  {"step_time_bias",   'w', LOOP1,26,           1.0,    0.0, 'u',  U_T_MS},
  {"step_which_bias",  'w', LOOP1,27,           1.0,    0.0, 'u',  U_NONE},

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

  {"az_now",       'w', LOOP1,   12,    360.0/4294967295.0, 0.0, 'S', U_P_DEG},
  {"el_now",       'w', LOOP1,   14,     99.0/4294967295.0,-10.0, 'U', U_P_DEG},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
  {"in_grp21_dig", 'r', ADC1_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"in_grp43_dig", 'r', ADC1_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"in_grp65_dig", 'r', ADC1_D,  52,            1.0,          0.0, 'u', U_NONE},
  {"out_grp21_dig",'w', ADC1_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"out_grp43_dig",'w', ADC1_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"out_grp65_dig",'w', ADC1_D,  52,            1.0,          0.0, 'u', U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",       'w', DECOM,   1,             1.0,          0.0, 'u', U_NONE},
  {"polarity",     'w', DECOM,   2,             1.0,          0.0, 'u', U_NONE},
  {"decom_unlock", 'w', DECOM,   3,             1.0,          0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
