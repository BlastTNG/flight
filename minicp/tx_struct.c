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
#define ADC2_C	  4, 0
#define ADC2_D	  5, 0
#define ADC2_A1	  6, 0
#define ADC2_A2	  7, 0
#define ADC3_C	  8, 0
//#define ADC3_D	  9, 0
#define ADC3_A0	  9, 0
#define ADC3_A1	 10, 0
#define ADC3_A2	 11, 0
#define LOOP1    20, 0
#define DECOM    50, 0
//still here to prevent crashes
#define ADC3_D	  9, 0

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
  {"status04",     'r', ADC2_C,  63,		1.0,          0.0, 'u', U_NONE},
  {"status05",     'r', ADC2_D,  63,		1.0,          0.0, 'u', U_NONE},
  {"status06",     'r', ADC2_A1, 63,		1.0,          0.0, 'u', U_NONE},
  {"status07",     'r', ADC2_A2, 63,		1.0,          0.0, 'u', U_NONE},
  {"status08",     'r', ADC3_C,  63,		1.0,          0.0, 'u', U_NONE},
  //{"status09",     'r', ADC3_D,  63,		1.0,          0.0, 'u', U_NONE},
  {"status09",     'r', ADC3_A0, 63,		1.0,          0.0, 'u', U_NONE},
  {"status10",     'r', ADC3_A1, 63,		1.0,          0.0, 'u', U_NONE},
  {"status11",     'r', ADC3_A2, 63,		1.0,          0.0, 'u', U_NONE},
  {"sync00",       'w', ADC1_C,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync01",       'w', ADC1_D,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync02",       'w', ADC1_A1, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync03",       'w', ADC1_A2, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync04",       'w', ADC2_C,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync05",       'w', ADC2_D,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync06",       'w', ADC2_A1, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync07",       'w', ADC2_A2, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync08",       'w', ADC3_C,  63,            1.0,          0.0, 'u', U_NONE},
  //{"sync09",       'w', ADC3_D,  63,            1.0,          0.0, 'u', U_NONE},
  {"sync09",       'w', ADC3_A0, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync10",       'w', ADC3_A1, 63,            1.0,          0.0, 'u', U_NONE},
  {"sync11",       'w', ADC3_A2, 63,            1.0,          0.0, 'u', U_NONE},

  //phase of lock-in multiplier wave (in 2000ths of a period)
  {"n02_phase",    'w', ADC1_A1,  8,            1.0,          0.0, 'u', U_NONE},
  {"n03_phase",    'w', ADC1_A2,  8,            1.0,          0.0, 'u', U_NONE},
  {"n06_phase",    'w', ADC2_A1,  8,            1.0,          0.0, 'u', U_NONE},
  {"n07_phase",    'w', ADC2_A2,  8,            1.0,          0.0, 'u', U_NONE},
  {"n10_phase",    'w', ADC3_A1,  8,            1.0,          0.0, 'u', U_NONE},
  {"n11_phase",    'w', ADC3_A2,  8,            1.0,          0.0, 'u', U_NONE},

  {"adc1_dac00",   'w', ADC1_D,   0,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac01",   'w', ADC1_D,   1,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac02",   'w', ADC1_D,   2,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac03",   'w', ADC1_D,   3,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac04",   'w', ADC1_D,   4,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac05",   'w', ADC1_D,   5,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac06",   'w', ADC1_D,   6,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac07",   'w', ADC1_D,   7,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac08",   'w', ADC1_D,   8,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac09",   'w', ADC1_D,   9,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac10",   'w', ADC1_D,  10,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac11",   'w', ADC1_D,  11,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac12",   'w', ADC1_D,  12,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac13",   'w', ADC1_D,  13,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac14",   'w', ADC1_D,  14,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac15",   'w', ADC1_D,  15,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac16",   'w', ADC1_D,  16,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac17",   'w', ADC1_D,  17,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac18",   'w', ADC1_D,  18,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac19",   'w', ADC1_D,  19,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac20",   'w', ADC1_D,  20,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac21",   'w', ADC1_D,  21,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac22",   'w', ADC1_D,  22,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac23",   'w', ADC1_D,  23,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac24",   'w', ADC1_D,  24,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac25",   'w', ADC1_D,  25,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac26",   'w', ADC1_D,  26,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac27",   'w', ADC1_D,  27,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac28",   'w', ADC1_D,  28,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac29",   'w', ADC1_D,  29,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac30",   'w', ADC1_D,  30,            1.0,          0.0, 'u', U_NONE},
  {"adc1_dac31",   'w', ADC1_D,  31,            1.0,          0.0, 'u', U_NONE},

#if 0
  {"adc2_dac00",   'w', ADC2_D,   0,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac01",   'w', ADC2_D,   1,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac02",   'w', ADC2_D,   2,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac03",   'w', ADC2_D,   3,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac04",   'w', ADC2_D,   4,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac05",   'w', ADC2_D,   5,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac06",   'w', ADC2_D,   6,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac07",   'w', ADC2_D,   7,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac08",   'w', ADC2_D,   8,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac09",   'w', ADC2_D,   9,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac10",   'w', ADC2_D,  10,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac11",   'w', ADC2_D,  11,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac12",   'w', ADC2_D,  12,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac13",   'w', ADC2_D,  13,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac14",   'w', ADC2_D,  14,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac15",   'w', ADC2_D,  15,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac16",   'w', ADC2_D,  16,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac17",   'w', ADC2_D,  17,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac18",   'w', ADC2_D,  18,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac19",   'w', ADC2_D,  19,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac20",   'w', ADC2_D,  20,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac21",   'w', ADC2_D,  21,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac22",   'w', ADC2_D,  22,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac23",   'w', ADC2_D,  23,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac24",   'w', ADC2_D,  24,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac25",   'w', ADC2_D,  25,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac26",   'w', ADC2_D,  26,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac27",   'w', ADC2_D,  27,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac28",   'w', ADC2_D,  28,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac29",   'w', ADC2_D,  29,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac30",   'w', ADC2_D,  30,            1.0,          0.0, 'u', U_NONE},
  {"adc2_dac31",   'w', ADC2_D,  31,            1.0,          0.0, 'u', U_NONE},

  {"adc3_dac00",   'w', ADC3_D,   0,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac01",   'w', ADC3_D,   1,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac02",   'w', ADC3_D,   2,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac03",   'w', ADC3_D,   3,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac04",   'w', ADC3_D,   4,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac05",   'w', ADC3_D,   5,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac06",   'w', ADC3_D,   6,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac07",   'w', ADC3_D,   7,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac08",   'w', ADC3_D,   8,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac09",   'w', ADC3_D,   9,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac10",   'w', ADC3_D,  10,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac11",   'w', ADC3_D,  11,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac12",   'w', ADC3_D,  12,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac13",   'w', ADC3_D,  13,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac14",   'w', ADC3_D,  14,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac15",   'w', ADC3_D,  15,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac16",   'w', ADC3_D,  16,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac17",   'w', ADC3_D,  17,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac18",   'w', ADC3_D,  18,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac19",   'w', ADC3_D,  19,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac20",   'w', ADC3_D,  20,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac21",   'w', ADC3_D,  21,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac22",   'w', ADC3_D,  22,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac23",   'w', ADC3_D,  23,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac24",   'w', ADC3_D,  24,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac25",   'w', ADC3_D,  25,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac26",   'w', ADC3_D,  26,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac27",   'w', ADC3_D,  27,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac28",   'w', ADC3_D,  28,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac29",   'w', ADC3_D,  29,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac30",   'w', ADC3_D,  30,            1.0,          0.0, 'u', U_NONE},
  {"adc3_dac31",   'w', ADC3_D,  31,            1.0,          0.0, 'u', U_NONE},

#endif 
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
  // LOOP1 channels 11-12 are fast channels
  
  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
  {"adc1_gyro0",   'r', ADC1_D,   0, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"adc1_gyro1",   'r', ADC1_D,   2, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"adc1_gyro2",   'r', ADC1_D,   4, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"adc1_gyro3",   'r', ADC1_D,   6, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"adc1_gyro4",   'r', ADC1_D,   8, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"adc1_gyro5",   'r', ADC1_D,  10, DGY32_TO_DPS, 
                                      -1*DGY32_OFFSET*DGY32_TO_DPS,'U',U_V_DPS},
  {"adc1_table",   'r', ADC1_D,  12, 360.0/144000.0,           0.0,'U',U_P_DEG},
  {"adc1_enc_el",  'r', ADC1_D,  14,  360.0/72000.0,           0.0,'U',U_P_DEG},
  {"adc1_enc_az",  'r', ADC1_D,  16,  360.0/72000.0,           0.0,'U',U_P_DEG},
  {"adc1_a1_00",   'r', ADC1_A1,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_01",   'r', ADC1_A1,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_02",   'r', ADC1_A1,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_03",   'r', ADC1_A1,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_04",   'r', ADC1_A1,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_05",   'r', ADC1_A1, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_06",   'r', ADC1_A1, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_07",   'r', ADC1_A1, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_08",   'r', ADC1_A1, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_09",   'r', ADC1_A1, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_10",   'r', ADC1_A1, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_11",   'r', ADC1_A1, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_12",   'r', ADC1_A1, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_13",   'r', ADC1_A1, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_14",   'r', ADC1_A1, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_15",   'r', ADC1_A1, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_16",   'r', ADC1_A1, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_17",   'r', ADC1_A1, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_18",   'r', ADC1_A1, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_19",   'r', ADC1_A1, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_20",   'r', ADC1_A1, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_21",   'r', ADC1_A1, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_22",   'r', ADC1_A1, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_23",   'r', ADC1_A1, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a1_24",   'r', ADC1_A1, 48,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_00",   'r', ADC1_A2,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_01",   'r', ADC1_A2,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_02",   'r', ADC1_A2,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_03",   'r', ADC1_A2,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_04",   'r', ADC1_A2,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_05",   'r', ADC1_A2, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_06",   'r', ADC1_A2, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_07",   'r', ADC1_A2, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_08",   'r', ADC1_A2, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_09",   'r', ADC1_A2, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_10",   'r', ADC1_A2, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_11",   'r', ADC1_A2, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_12",   'r', ADC1_A2, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_13",   'r', ADC1_A2, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_14",   'r', ADC1_A2, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_15",   'r', ADC1_A2, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_16",   'r', ADC1_A2, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_17",   'r', ADC1_A2, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_18",   'r', ADC1_A2, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_19",   'r', ADC1_A2, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_20",   'r', ADC1_A2, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_21",   'r', ADC1_A2, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_22",   'r', ADC1_A2, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_23",   'r', ADC1_A2, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc1_a2_24",   'r', ADC1_A2, 48,      CAL32(1.0,          0.0),'U', U_V_V},
  {"az_now",       'w', LOOP1,   11,    360.0/4294967295.0, 0.0, 'S', U_P_DEG},
  {"el_now",       'w', LOOP1,   12,     99.0/4294967295.0,-10.0, 'U', U_P_DEG},

#if 0
  {"adc2_a1_00",   'r', ADC2_A1,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_01",   'r', ADC2_A1,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_02",   'r', ADC2_A1,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_03",   'r', ADC2_A1,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_04",   'r', ADC2_A1,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_05",   'r', ADC2_A1, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_06",   'r', ADC2_A1, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_07",   'r', ADC2_A1, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_08",   'r', ADC2_A1, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_09",   'r', ADC2_A1, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_10",   'r', ADC2_A1, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_11",   'r', ADC2_A1, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_12",   'r', ADC2_A1, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_13",   'r', ADC2_A1, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_14",   'r', ADC2_A1, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_15",   'r', ADC2_A1, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_16",   'r', ADC2_A1, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_17",   'r', ADC2_A1, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_18",   'r', ADC2_A1, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_19",   'r', ADC2_A1, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_20",   'r', ADC2_A1, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_21",   'r', ADC2_A1, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_22",   'r', ADC2_A1, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_23",   'r', ADC2_A1, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a1_24",   'r', ADC2_A1, 48,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_00",   'r', ADC2_A2,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_01",   'r', ADC2_A2,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_02",   'r', ADC2_A2,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_03",   'r', ADC2_A2,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_04",   'r', ADC2_A2,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_05",   'r', ADC2_A2, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_06",   'r', ADC2_A2, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_07",   'r', ADC2_A2, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_08",   'r', ADC2_A2, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_09",   'r', ADC2_A2, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_10",   'r', ADC2_A2, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_11",   'r', ADC2_A2, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_12",   'r', ADC2_A2, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_13",   'r', ADC2_A2, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_14",   'r', ADC2_A2, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_15",   'r', ADC2_A2, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_16",   'r', ADC2_A2, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_17",   'r', ADC2_A2, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_18",   'r', ADC2_A2, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_19",   'r', ADC2_A2, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_20",   'r', ADC2_A2, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_21",   'r', ADC2_A2, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_22",   'r', ADC2_A2, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_23",   'r', ADC2_A2, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc2_a2_24",   'r', ADC2_A2, 48,      CAL32(1.0,          0.0),'U', U_V_V},

  {"adc3_a0_00",   'r', ADC3_A0,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_01",   'r', ADC3_A0,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_02",   'r', ADC3_A0,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_03",   'r', ADC3_A0,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_04",   'r', ADC3_A0,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_05",   'r', ADC3_A0, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_06",   'r', ADC3_A0, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_07",   'r', ADC3_A0, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_08",   'r', ADC3_A0, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_09",   'r', ADC3_A0, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_10",   'r', ADC3_A0, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_11",   'r', ADC3_A0, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_12",   'r', ADC3_A0, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_13",   'r', ADC3_A0, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_14",   'r', ADC3_A0, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_15",   'r', ADC3_A0, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_16",   'r', ADC3_A0, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_17",   'r', ADC3_A0, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_18",   'r', ADC3_A0, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_19",   'r', ADC3_A0, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_20",   'r', ADC3_A0, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_21",   'r', ADC3_A0, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_22",   'r', ADC3_A0, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_23",   'r', ADC3_A0, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a0_24",   'r', ADC3_A0, 48,      CAL32(1.0,          0.0),'U', U_V_V},

  {"adc3_a1_00",   'r', ADC3_A1,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_01",   'r', ADC3_A1,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_02",   'r', ADC3_A1,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_03",   'r', ADC3_A1,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_04",   'r', ADC3_A1,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_05",   'r', ADC3_A1, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_06",   'r', ADC3_A1, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_07",   'r', ADC3_A1, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_08",   'r', ADC3_A1, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_09",   'r', ADC3_A1, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_10",   'r', ADC3_A1, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_11",   'r', ADC3_A1, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_12",   'r', ADC3_A1, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_13",   'r', ADC3_A1, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_14",   'r', ADC3_A1, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_15",   'r', ADC3_A1, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_16",   'r', ADC3_A1, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_17",   'r', ADC3_A1, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_18",   'r', ADC3_A1, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_19",   'r', ADC3_A1, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_20",   'r', ADC3_A1, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_21",   'r', ADC3_A1, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_22",   'r', ADC3_A1, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_23",   'r', ADC3_A1, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a1_24",   'r', ADC3_A1, 48,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_00",   'r', ADC3_A2,  0,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_01",   'r', ADC3_A2,  2,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_02",   'r', ADC3_A2,  4,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_03",   'r', ADC3_A2,  6,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_04",   'r', ADC3_A2,  8,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_05",   'r', ADC3_A2, 10,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_06",   'r', ADC3_A2, 12,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_07",   'r', ADC3_A2, 14,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_08",   'r', ADC3_A2, 16,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_09",   'r', ADC3_A2, 18,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_10",   'r', ADC3_A2, 20,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_11",   'r', ADC3_A2, 22,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_12",   'r', ADC3_A2, 24,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_13",   'r', ADC3_A2, 26,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_14",   'r', ADC3_A2, 28,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_15",   'r', ADC3_A2, 30,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_16",   'r', ADC3_A2, 32,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_17",   'r', ADC3_A2, 34,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_18",   'r', ADC3_A2, 36,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_19",   'r', ADC3_A2, 38,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_20",   'r', ADC3_A2, 40,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_21",   'r', ADC3_A2, 42,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_22",   'r', ADC3_A2, 44,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_23",   'r', ADC3_A2, 46,      CAL32(1.0,          0.0),'U', U_V_V},
  {"adc3_a2_24",   'r', ADC3_A2, 48,      CAL32(1.0,          0.0),'U', U_V_V},
#endif


  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
  {"adc1_d1",	   'r', ADC1_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"adc1_d2",	   'r', ADC1_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"adc1_d3",	   'r', ADC1_D,  52,            1.0,          0.0, 'u', U_NONE},
  {"adc1_wd1",     'w', ADC1_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"adc1_wd2",     'w', ADC1_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"adc1_wd3",     'w', ADC1_D,  52,            1.0,          0.0, 'u', U_NONE},

#if 0
  {"adc2_d1",	   'r', ADC2_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"adc2_d2",	   'r', ADC2_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"adc2_d3",	   'r', ADC2_D,  52,            1.0,          0.0, 'u', U_NONE},
  {"adc2_wd1",     'w', ADC2_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"adc2_wd2",     'w', ADC2_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"adc2_wd3",     'w', ADC2_D,  52,            1.0,          0.0, 'u', U_NONE},

  {"adc3_d1",	   'r', ADC3_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"adc3_d2",	   'r', ADC3_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"adc3_d3",	   'r', ADC3_D,  52,            1.0,          0.0, 'u', U_NONE},
  {"adc3_wd1",     'w', ADC3_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"adc3_wd2",     'w', ADC3_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"adc3_wd3",     'w', ADC3_D,  52,            1.0,          0.0, 'u', U_NONE},
#endif

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",       'w', DECOM,   1,             1.0,          0.0, 'u', U_NONE},
  {"polarity",     'w', DECOM,   2,             1.0,          0.0, 'u', U_NONE},
  {"decom_unlock", 'w', DECOM,   3,             1.0,          0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
