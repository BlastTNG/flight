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

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MOVE, OR DELETE *ANY* CHANNELS IN THIS FILE YOU *MUST*
 * RECOMPILE AND RESTART THE DECOM DAEMON (DECOMD) ON widow!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include <limits.h>
#include "channels.h"
#include "calibrate.h"
#include "mpc_proto.h"
#ifdef __MCP__
#include "camstruct.h"
#endif
/* card name to (node number, bus number) mapping */
#define ACS1_C    0, 0  /* C denotes a common motherboard node */
#define ACS1_D    1, 0  /* D denotes a digital daughter card */
#define ACS1_A1   2, 0  /* A deontes a node for analog daughter card */
#define ACS1_T1   3, 0  /* T denotes an AD590 thermometry daughter card */
#define ACS2_C    4, 0
#define ACS2_D    5, 0
#define ACS2_A1   6, 0
#define ACS2_A2   7, 0
#define RTD_C     8, 0
#define RTD_D     9, 0
#define RTD_A1   10, 0
#define RTD_A2   11, 0
#define DIOD_C   12, 0
#define DIOD_A1  13, 0
#define DIOD_A2  14, 0
#define DIOD_A3  15, 0 /* swapped with theo, originally 15 */
#define HWP_C    16, 0
#define HWP_D    17, 0
#define HWP_A1   19, 0 /* nodes swapped to change Theo/HWP, originally 18 */
#define HWP_A2   18, 0 /* nodes swapped to change Theo/HWP, originally 19 */
//TODO could make it possible for LOOPbacks to not need explicit channels
#define LOOP1    32, 0
#define LOOP2    33, 0
#define LOOP3    34, 0
#define LOOP4    35, 0
#define LOOP5    36, 0
#define LOOP6    37, 0
#define LOOP7    38, 0
#define LOOP8    39, 0
#define LOOP9    40, 0
#define LOOP0    41, 0
#define LOOP11   42, 0
#define DECOM    43, 0

/* Analog channel calibrations */
/* 16/32-bit channels with analog preamps. To Volts */
#define CAL16(m,b) ((m)*M_16PRE), ((b) + B_16PRE*(m)*M_16PRE)
#define CAL32(m,b) ((m)*M_32PRE), ((b) + B_32PRE*(m)*M_32PRE)
/* 16-bit bare analog. To Volts */
#define CAL16B(m,b) ((m)*M_16B), ((b) + B_16B*(m)*M_16B)
/* bare thermomtstor. To Volts. Use LUT for temperature conversion */
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T)
/* AD590 conversion. To Celsius */
#define CAL_AD590(m,b) ((m)*M_16_AD590),((b)+B_16_AD590*(m)*M_16_AD590-273.15)
/* DAC outputs. To Volts */
#define CALDAC(m,b) ((m)*M_DAC), ((b) + B_DAC*(m)*M_DAC)

/* Housekeeping calibrations (D)iode, (C)ernox, (N)TD*/
/* factor 0.994 is rough ADC gain calibration for all these channels */
#define CAL32D(m,b) CAL32((m) / 0.994, (b))
#define CAL32C(m,b) CAL32((m) / 0.994 / 991.0, (b))
#define CAL32N(m,b) CAL32((m) / 0.994 / 726.4, (b))

/* units definitions */
#define U_NONE  "","" 
#define U_T_C   "Temperature","^oC"
#define U_V_DPS "Rate","^o/s"
#define U_V_MPS "Speed","m/s"
#define U_V_KPH "Speed","km/hr"
#define U_ALT_M "Altitude","m"
#define U_P_DEG "Position","^o"
#define U_P_HR  "Position", "h"
#define U_PH_DEG  "Phase","^o"
#define U_LA_DEG "Latitude","^o"
#define U_LO_DEG "Longitude","^o"
#define U_D_DEG "Direction","^o"
#define U_V_V   "Voltage","V"
#define U_I_A   "Current","A"
#define U_T_MS  "Time","ms"
#define U_R_O   "Resistance","Ohms"
#define U_RATE "Rate", "bps"
#define U_F_HZ "Frequency", "Hz"
#define U_TRIM_DEG "Trim","^o"
#define U_TRIM_MM "Trim","mm"

struct ChannelStruct WideSlowChannels[] = {
  {"count_1_el",   'r',ACS2_D, 58,                1.0,             0.0, 'U', U_NONE}, 
  {"count_2_el",   'r',ACS2_D, 60,                1.0,             0.0, 'U', U_NONE}, 
  {"time",         'w', LOOP1,  0,                1.0,             0.0, 'U', U_NONE}, 
  {"time_usec",     'w', LOOP4, 58,                1.0,             0.0, 'U', U_NONE},
  {"time_sip",     'w', LOOP1,  2,                1.0,             0.0, 'U', U_NONE},
  {"time_dgps",    'w', LOOP1,  4,                1.0,             0.0, 'U', U_NONE},
  {"lst",          'w', LOOP1,  6,         1.0/3600.0,             0.0, 'U', U_NONE},
  {"parts_sched",   'w', LOOP1, 12,                1.0,             0.0, 'U', U_NONE},
  {"frame_thegood",'w', LOOP1, 32,                1.0,             0.0, 'U', U_NONE},
  {"lat",          'w', LOOP1, 38,             LI2DEG,             0.0, 'S', U_NONE},
  {"lon",          'w', LOOP1, 40,             LI2DEG,             0.0, 'S', U_NONE},
  {"sec_thegood",   'w', LOOP2, 20,                1.0,             0.0, 'U', U_NONE},
  {"usec_thegood",  'w', LOOP2, 60,                1.0,             0.0, 'U', U_NONE},
  {"ra",           'w', LOOP3,  4,               LI2H,             0.0, 'U', U_P_HR},
  {"frame_thebad", 'w', LOOP3, 31,                1.0,             0.0, 'U', U_NONE},
  {"sec_thebad",   'w', LOOP3, 44,                1.0,             0.0, 'U', U_NONE},
  {"usec_thebad",  'w', LOOP3, 58,                1.0,             0.0, 'U', U_NONE},
  {"dec",          'w', LOOP5,  6,             LI2DEG,             0.0, 'S', U_P_DEG},
  {"lst_sched",    'w', LOOP6, 56,                1.0,             0.0, 'U', U_NONE},  // ls day
  {"frame_theugly",    'w', LOOP9, 50,                1.0,             0.0, 'U', U_NONE},
  {"sec_theugly",      'w', LOOP9, 52,                1.0,             0.0, 'U', U_NONE},
  {"usec_theugly",     'w', LOOP9, 54,                1.0,             0.0, 'U', U_NONE},
  {"time_b_flc",       'w', LOOP0, 48,                1.0,             0.0, 'U', U_NONE},
  {"time_i_flc",       'w', LOOP0, 50,                1.0,             0.0, 'U', U_NONE},
  //derived channel time_theugly adds these together
  {"start_1_cycle",  'w', LOOP0,  12,                1.0,            0.0, 'U', U_NONE},
  {"start_2_cycle",  'w', LOOP0,  14,                1.0,            0.0, 'U', U_NONE},
  {"start_3_cycle",  'w', LOOP0,  16,                1.0,            0.0, 'U', U_NONE},
  {"start_4_cycle",  'w', LOOP0,  18,                1.0,            0.0, 'U', U_NONE},
  {"start_5_cycle",  'w', LOOP0,  20,                1.0,            0.0, 'U', U_NONE},
  {"start_6_cycle",  'w', LOOP0,  22,                1.0,            0.0, 'U', U_NONE},
  //LOOP0 24-29 are narrow
  {"start_state_1_cycle",  'w', LOOP0,  30,                1.0,            0.0, 'U', U_NONE},
  {"start_state_2_cycle",  'w', LOOP0,  32,                1.0,            0.0, 'U', U_NONE},
  {"start_state_3_cycle",  'w', LOOP0,  34,                1.0,            0.0, 'U', U_NONE},
  {"start_state_4_cycle",  'w', LOOP0,  36,                1.0,            0.0, 'U', U_NONE},
  {"start_state_5_cycle",  'w', LOOP0,  38,                1.0,            0.0, 'U', U_NONE},
  {"start_state_6_cycle",  'w', LOOP0,  40,                1.0,            0.0, 'U', U_NONE},

  /* housekeeping channels */  /* TODO many can probably be not-wide */
  {"vr_still_2_hk",  'r', RTD_A1,  0, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_2_hk",    'r', RTD_A1,  2, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_still_3_hk",  'r', RTD_A1, 16, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_3_hk",    'r', RTD_A1, 18, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_still_1_hk",  'r', RTD_A1, 32, CAL32C(       1.053,           0.0), 'U', U_V_V},
  {"vr_mux_1_hk",    'r', RTD_A1, 34, CAL32C(       1.065,           0.0), 'U', U_V_V},
  {"vr_still_5_hk",  'r', RTD_A2,  0, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_5_hk",    'r', RTD_A2,  2, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_still_6_hk",  'r', RTD_A2, 16, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_6_hk",    'r', RTD_A2, 18, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_still_4_hk",  'r', RTD_A2, 32, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_4_hk",    'r', RTD_A2, 34, CAL32C(       1.055,           0.0), 'U', U_V_V},

  //TODO name the diode voltages. Must change derived.c to match
  {"vd_4k_2_hk",       'r', DIOD_A1,  0, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_cp_2_hk",       'r', DIOD_A1,  2, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_pump_2_hk",     'r', DIOD_A1,  4, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_hsw_2_hk",      'r', DIOD_A1,  6, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_plate_2_hk",    'r', DIOD_A1,  8, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_snout_2_hk",    'r', DIOD_A1, 10, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_eyepiece_2_hk", 'r', DIOD_A1, 12, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_objective_2_hk",'r', DIOD_A1, 14, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_spittoon_2_hk",'r',  DIOD_A1, 16, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_aux_post_2_hk", 'r', DIOD_A1, 18, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_stop_2_hk",     'r', DIOD_A1, 20, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_ssa_2_hk",      'r', DIOD_A1, 22, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_4k_1_hk",       'r', DIOD_A1, 24, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_cp_1_hk",       'r', DIOD_A1, 26, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_pump_1_hk",     'r', DIOD_A1, 28, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_hsw_1_hk",      'r', DIOD_A1, 30, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_plate_1_hk",    'r', DIOD_A1, 32, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_snout_1_hk",    'r', DIOD_A1, 34, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_eyepiece_1_hk", 'r', DIOD_A1, 36, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_objective_1_hk",'r', DIOD_A1, 38, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_spittoon_1_hk",'r',  DIOD_A1, 40, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_aux_post_1_hk", 'r', DIOD_A1, 42, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_stop_1_hk",     'r', DIOD_A1, 44, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_ssa_1_hk",      'r', DIOD_A1, 46, CAL32D(     1.0,     0.0), 'U', U_V_V},

  {"vd_4k_4_hk",       'r', DIOD_A2,  0, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_cp_4_hk",       'r', DIOD_A2,  2, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_pump_4_hk",     'r', DIOD_A2,  4, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_hsw_4_hk",      'r', DIOD_A2,  6, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_plate_4_hk",    'r', DIOD_A2,  8, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_snout_4_hk",    'r', DIOD_A2, 10, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_eyepiece_4_hk", 'r', DIOD_A2, 12, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_objective_4_hk",'r', DIOD_A2, 14, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_spittoon_4_hk",'r',  DIOD_A2, 16, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_aux_post_4_hk", 'r', DIOD_A2, 18, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_stop_4_hk",     'r', DIOD_A2, 20, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_ssa_4_hk",      'r', DIOD_A2, 22, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_4k_3_hk",       'r', DIOD_A2, 24, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_cp_3_hk",       'r', DIOD_A2, 26, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_pump_3_hk",     'r', DIOD_A2, 28, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_hsw_3_hk",      'r', DIOD_A2, 30, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_plate_3_hk",    'r', DIOD_A2, 32, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_snout_3_hk",    'r', DIOD_A2, 34, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_eyepiece_3_hk", 'r', DIOD_A2, 36, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_objective_3_hk",'r', DIOD_A2, 38, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_spittoon_3_hk",'r',  DIOD_A2, 40, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_aux_post_3_hk", 'r', DIOD_A2, 42, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_stop_3_hk",     'r', DIOD_A2, 44, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_ssa_3_hk",      'r', DIOD_A2, 46, CAL32D(     1.0,     0.0), 'U', U_V_V},

  {"vd_4k_6_hk",       'r', DIOD_A3,  0, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_cp_6_hk",       'r', DIOD_A3,  2, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_pump_6_hk",     'r', DIOD_A3,  4, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_hsw_6_hk",      'r', DIOD_A3,  6, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_plate_6_hk",    'r', DIOD_A3,  8, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_snout_6_hk",    'r', DIOD_A3, 10, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_eyepiece_6_hk", 'r', DIOD_A3, 12, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_objective_6_hk",'r', DIOD_A3, 14, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_spittoon_6_hk",'r',  DIOD_A3, 16, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_aux_post_6_hk", 'r', DIOD_A3, 18, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_stop_6_hk",     'r', DIOD_A3, 20, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_ssa_6_hk",      'r', DIOD_A3, 22, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_4k_5_hk",       'r', DIOD_A3, 24, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_cp_5_hk",       'r', DIOD_A3, 26, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_pump_5_hk",     'r', DIOD_A3, 28, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_hsw_5_hk",      'r', DIOD_A3, 30, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_plate_5_hk",    'r', DIOD_A3, 32, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_snout_5_hk",    'r', DIOD_A3, 34, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_eyepiece_5_hk", 'r', DIOD_A3, 36, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_objective_5_hk",'r', DIOD_A3, 38, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_spittoon_5_hk",'r',  DIOD_A3, 40, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_aux_post_5_hk", 'r', DIOD_A3, 42, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_stop_5_hk",     'r', DIOD_A3, 44, CAL32D(     1.0,     0.0), 'U', U_V_V},
  {"vd_ssa_5_hk",      'r', DIOD_A3, 46, CAL32D(     1.0,     0.0), 'U', U_V_V},

  {"vd_00_hk",     'r',  HWP_A2,  0, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_01_hk",     'r',  HWP_A2,  2, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_hk",     'r',  HWP_A2,  4, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_hk",     'r',  HWP_A2,  6, CAL32D(         1.0,           0.0), 'U', U_V_V},
  //{"vd_04_hk",     'r',  HWP_A2,  8, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_hk",     'r',  HWP_A2, 10, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_hk",     'r',  HWP_A2, 12, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_hk",     'r',  HWP_A2, 14, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_hk",     'r',  HWP_A2, 16, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_hk",     'r',  HWP_A2, 18, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_hk",     'r',  HWP_A2, 20, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_hk",     'r',  HWP_A2, 22, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_12_hk",     'r',  HWP_A2, 24, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_13_hk",     'r',  HWP_A2, 26, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_14_hk",     'r',  HWP_A2, 28, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_15_hk",     'r',  HWP_A2, 30, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_16_hk",     'r',  HWP_A2, 32, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_17_hk",     'r',  HWP_A2, 34, CAL32D(         1.0,           0.0), 'U', U_V_V},

  {"mic_time",     'w',  LOOP11, 2, 0.01, MPC_EPOCH, 'U', U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  //status and sync channels for handshaking with bbus nodes
  {"status00",     'r',  ACS1_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status01",     'r',  ACS1_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"status02",     'r', ACS1_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status03",     'r', ACS1_T1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status04",     'r',  ACS2_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status05",     'r',  ACS2_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"status06",     'r', ACS2_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status07",     'r', ACS2_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status08",     'r',   RTD_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status09",     'r',   RTD_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"status10",     'r',  RTD_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status11",     'r',  RTD_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status12",     'r',  DIOD_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status13",     'r', DIOD_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status14",     'r', DIOD_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status15",     'r', DIOD_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"status16",     'r',   HWP_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status17",     'r',   HWP_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"status18",     'r',  HWP_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status19",     'r',  HWP_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync00",       'w',  ACS1_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync01",       'w',  ACS1_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync02",       'w', ACS1_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync03",       'w', ACS1_T1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync04",       'w',  ACS2_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync05",       'w',  ACS2_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync06",       'w', ACS2_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync07",       'w', ACS2_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync08",       'w',   RTD_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync09",       'w',   RTD_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync10",       'w',  RTD_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync11",       'w',  RTD_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync12",       'w',  DIOD_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync13",       'w', DIOD_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync14",       'w', DIOD_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync15",       'w', DIOD_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync16",       'w',   HWP_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync17",       'w',   HWP_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync18",       'w',  HWP_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync19",       'w',  HWP_A1, 63,                1.0,             0.0, 'u', U_NONE},

  //phase of AC DAC signals (NB: overflowed from digital to analog daughter)
  {"ph_cnx_2_hk",   'w', RTD_A1,  0,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_2_hk",   'w', RTD_A1,  1,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_1_hk",   'w', RTD_A1,  4,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_1_hk",   'w', RTD_A1,  5,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_3_hk",   'w', RTD_A1,  8,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_3_hk",   'w', RTD_A1,  9,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_4_hk",   'w', RTD_A1, 12,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_4_hk",   'w', RTD_A1, 13,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_6_hk",   'w', RTD_A1, 16,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_6_hk",   'w', RTD_A1, 17,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_5_hk",   'w', RTD_A1, 20,          I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_5_hk",   'w', RTD_A1, 21,          I2DEG,          0.0,'u',U_PH_DEG},

  //commanded bias. use f_bias_hk to see what this really corresponds to
  {"f_bias_cmd_hk", 'w', RTD_C,   0,            1.0,          0.0, 'u', U_F_HZ},

  {"v_cnx_2_hk",    'w', RTD_D,   0, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_2_hk",    'w', RTD_D,   1, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_strap_2_hk", 'w', RTD_D,   2, CALDAC( 0.9970,     -0.0013), 'u',  U_V_V},
  {"heat_fplo_2_hk",'w', RTD_D,   3, CALDAC( 0.9973,     -0.0010), 'u',  U_V_V},
  {"v_cnx_1_hk",    'w', RTD_D,   4, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_1_hk",    'w', RTD_D,   5, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_strap_1_hk", 'w', RTD_D,   6, CALDAC( 0.9974,     -0.0015), 'u',  U_V_V},
  {"heat_fplo_1_hk",'w', RTD_D,   7, CALDAC( 0.9989,     -0.0015), 'u',  U_V_V},
  {"v_cnx_3_hk",    'w', RTD_D,   8, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_3_hk",    'w', RTD_D,   9, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_strap_3_hk", 'w', RTD_D,  10, CALDAC( 1.0006,     -0.0013), 'u',  U_V_V},
  {"heat_fplo_3_hk",'w', RTD_D,  11, CALDAC( 1.0012,     -0.0012), 'u',  U_V_V},
  {"v_cnx_4_hk",    'w', RTD_D,  12, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_4_hk",    'w', RTD_D,  13, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_strap_4_hk", 'w', RTD_D,  14, CALDAC( 1.0035,     -0.0010), 'u',  U_V_V},
  {"heat_fplo_4_hk",'w', RTD_D,  15, CALDAC( 0.9983,     -0.0013), 'u',  U_V_V},
  {"v_cnx_6_hk",    'w', RTD_D,  16, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_6_hk",    'w', RTD_D,  17, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_strap_6_hk", 'w', RTD_D,  18, CALDAC( 0.9981,     -0.0015), 'u',  U_V_V},
  {"heat_fplo_6_hk",'w', RTD_D,  19, CALDAC( 1.0040,     -0.0015), 'u',  U_V_V},
  {"v_cnx_5_hk",    'w', RTD_D,  20, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_5_hk",    'w', RTD_D,  21, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_strap_5_hk", 'w', RTD_D,  22, CALDAC( 0.9997,     -0.0012), 'u',  U_V_V},
  {"heat_fplo_5_hk",'w', RTD_D,  23, CALDAC( 0.9988,     -0.0014), 'u',  U_V_V},

  {"ifpwr",         'w', HWP_D,  50,                1.0,             0.0, 'u', U_NONE},
  {"hwp_bias",      'w', HWP_D,  54,                1.0,             0.0, 'u', U_NONE},
  //TODO replace these with one write
  {"phase_00_hwp",  'w', HWP_A1,  0,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_01_hwp",  'w', HWP_A1,  1,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_02_hwp",  'w', HWP_A1,  2,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_03_hwp",  'w', HWP_A1,  3,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_04_hwp",  'w', HWP_A1,  4,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_05_hwp",  'w', HWP_A1,  5,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_06_hwp",  'w', HWP_A1,  6,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_07_hwp",  'w', HWP_A1,  7,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_08_hwp",  'w', HWP_A1,  8,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_09_hwp",  'w', HWP_A1,  9,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_10_hwp",  'w', HWP_A1, 10,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_11_hwp",  'w', HWP_A1, 11,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_12_hwp",  'w', HWP_A1, 12,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_13_hwp",  'w', HWP_A1, 13,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_14_hwp",  'w', HWP_A1, 14,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_15_hwp",  'w', HWP_A1, 15,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_16_hwp",  'w', HWP_A1, 16,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_17_hwp",  'w', HWP_A1, 17,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_18_hwp",  'w', HWP_A1, 18,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_19_hwp",  'w', HWP_A1, 19,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_20_hwp",  'w', HWP_A1, 20,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_21_hwp",  'w', HWP_A1, 21,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_22_hwp",  'w', HWP_A1, 22,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_23_hwp",  'w', HWP_A1, 23,               I2DEG,            0.0, 'u',U_PH_DEG},
  {"phase_24_hwp",  'w', HWP_A1, 24,               I2DEG,            0.0, 'u',U_PH_DEG},

  // TODO: Give these real names.
  {"vt_1_if",      'r',  HWP_A2, 37, CAL16B(         1.0,           0.0), 'u', U_V_V},
  {"vt_2_if",      'r',  HWP_A2, 39, CAL16B(         1.0,           0.0), 'u', U_V_V},
  {"vt_3_if",      'r',  HWP_A2, 41, CAL16B(         1.0,           0.0), 'u', U_V_V},
  {"vt_4_if",      'r',  HWP_A2, 43, CAL16B(         1.0,           0.0), 'u', U_V_V},
  {"vt_5_if",      'r',  HWP_A2, 45, CAL16B(         1.0,           0.0), 'u', U_V_V},
  {"vt_6_if",      'r',  HWP_A2, 47, CAL16B(         1.0,           0.0), 'u', U_V_V},
  {"vt_7_if",      'r',  HWP_A2, 49, CAL16B(         1.0,           0.0), 'u', U_V_V},

  
/* default current cal is 12.5, 0.0 */
  {"i_mce",        'r',  HWP_A2,   9,          CAL16(12.5, 0.0),      'u', U_I_A},
  {"i_hk_misc",    'r',  RTD_A2,  49,          CAL16(12.5, 0.0),      'u', U_I_A},
 
/* spare IF analog inputs */ 
  // DIOD_A2 and DIOD_A3 49 are wide
  {"v_sft_valve_hk", 'r', DIOD_A1, 49,         CAL16(1.0, 0.0),       'u', U_V_V},
 
  /* LOOP1 0-7 are wide */
  /* LOOP1 9, 12-13 are wide */
  {"g_com_el",     'w', LOOP1,  8,                3.0/65536.0,     0.0, 'u', U_NONE},
  {"channelset_oth", 'w', LOOP1,  9,                1,             0.0, 'u', U_NONE},
  {"foc_res_thegood", 'w', LOOP1, 17,             1.0,             0.0, 'u', U_NONE},
  {"status_eth",   'w', LOOP1, 19,                1.0,             0.0, 'u', U_NONE}, // Star camera net status
  {"timeout_b",      'w', LOOP1, 20,                1.0,             0.0, 'u', U_NONE},
  {"az_sun",       'w', LOOP1, 21,              I2DEG,             0.0, 'u', U_D_DEG},
  {"status_flc",   'w', LOOP1, 23,                1.0,             0.0, 'u', U_NONE}, //bitsy_i_am, at_float, schedule, slot_sched
  {"upslot_sched", 'w', LOOP1, 25,                1.0,             0.0, 'u', U_NONE},  
  {"t_chip_flc",   'w', LOOP1, 26,               0.01,             0.0, 'u', U_NONE},
  {"declination_mag",'w', LOOP1, 27,              I2DEG,             0.0, 'u', U_D_DEG}, // magnetic declination
  {"veto_sensor",  'w', LOOP1, 28,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 32-33 are wide */
  /* LOOP1 34 is fast */
  {"timeout_i",      'w', LOOP1, 35,                1.0,             0.0, 'u', U_NONE},
  {"df_b_flc",    'w', LOOP1, 36,             1./250,             0.0, 'u', U_NONE},
  {"alt_sip",      'w', LOOP1, 37,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 38-41 are wide */
  {"mapmean_thegood",'w', LOOP1, 42,                1.0,             0.0, 'u', U_NONE},
  {"lat_sip",      'w', LOOP1, 45,              I2DEG,             0.0, 's', U_LA_DEG},
  {"lon_sip",      'w', LOOP1, 46,              I2DEG,             0.0, 's', U_LO_DEG},
  {"lat_dgps",     'w', LOOP1, 47,              I2DEG,             0.0, 's', U_LA_DEG},
  {"lon_dgps",     'w', LOOP1, 48,              I2DEG,             0.0, 's', U_LO_DEG},
  {"alt_dgps",     'w', LOOP1, 49,                1.0,             0.0, 'u', U_ALT_M},
  {"speed_dgps",   'w', LOOP1, 50,             1./100,             0.0, 's', U_V_KPH},
  {"dir_dgps",     'w', LOOP1, 51,              I2DEG,             0.0, 'u', U_D_DEG},
  {"climb_dgps",   'w', LOOP1, 52,             1./100,             0.0, 's', U_V_MPS},
  {"att_ok_dgps",  'w', LOOP1, 53,                1.0,             0.0, 'u', U_NONE},
  {"g_p_table",    'w', LOOP1, 54,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"g_i_table",    'w', LOOP1, 55,        1.0/10000.0,             0.0, 'u', U_NONE},
  {"g_d_table",    'w', LOOP1, 56,          1.0/100.0,             0.0, 'u', U_NONE},
  {"n_sat_dgps",   'w', LOOP1, 57,                1.0,             0.0, 'u', U_NONE},
  {"df_i_flc",    'w', LOOP1, 58,             1./250,             0.0, 'u', U_NONE},
  {"mode_p",       'w', LOOP1, 59,                  1,             0.0, 'u', U_NONE},
  {"x_p",          'w', LOOP1, 60,              I2DEG,             0.0, 'u', U_NONE},
  {"y_p",          'w', LOOP1, 61,              I2DEG,             0.0, 's', U_NONE},
  {"vel_az_p",     'w', LOOP1, 62,              I2VEL,             0.0, 's', U_NONE},
  {"del_p",        'w', LOOP1, 63,              I2VEL,             0.0, 'u', U_NONE},

  {"nblobs_thegood",  'w', LOOP2,  0,                1.0,             0.0, 'u', U_NONE},
  // TODO: add appropriate derived types for these
  {"blob00_x_thegood",'w', LOOP2,  1, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob00_y_thegood",'w', LOOP2,  2, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob00_f_thegood",'w', LOOP2,  3,                1.0,             0.0, 'u', U_NONE},
  {"blob00_s_thegood",'w', LOOP2,  4,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob01_x_thegood",'w', LOOP2,  5, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob01_y_thegood",'w', LOOP2,  6, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob01_f_thegood",'w', LOOP2,  7,                1.0,             0.0, 'u', U_NONE},
  {"blob01_s_thegood",'w', LOOP2,  8,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob02_x_thegood",'w', LOOP2,  9, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob02_y_thegood",'w', LOOP2, 10, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob02_f_thegood",'w', LOOP2, 11,                1.0,             0.0, 'u', U_NONE},
  {"blob02_s_thegood",'w', LOOP2, 12,          1.0/100.0,             0.0, 'u', U_NONE},
  {"w_p",             'w', LOOP2, 13,              I2DEG,             0.0, 'u', U_NONE}, // pointing scan width
  {"move_tol_thegood",'w', LOOP2, 14,                1.0,             0.0, 'u', U_NONE},
  {"exp_int_thegood", 'w', LOOP2, 15,                1.0,             0.0, 'u', U_NONE},
  {"exp_time_thegood",'w', LOOP2, 16,                1.0,             0.0, 'u', U_NONE},
  {"force_thegood",   'w', LOOP2, 17,                1.0,             0.0, 'u', U_NONE},
  {"blob_idx_thegood",'w', LOOP2, 18,                1.0,             0.0, 'u', U_NONE},
  {"blob_idx_thebad", 'w', LOOP2, 19,                1.0,             0.0, 'u', U_NONE},
  /*P2 20-21 is wide */
  {"blob_idx_theugly",'w', LOOP2, 22,                1.0,             0.0, 'u', U_NONE},
  {"thresh_thegood",  'w', LOOP2, 23,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"grid_thegood",    'w', LOOP2, 24,                1.0,             0.0, 'u', U_NONE},
  {"cal_off_pss1",    'w', LOOP2, 25,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"cal_off_pss2",    'w', LOOP2, 26,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"mdist_thegood",   'w', LOOP2, 27,                1.0,             0.0, 'u', U_NONE},
  {"cal_off_pss3",    'w', LOOP2, 28,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"cal_off_pss4",    'w', LOOP2, 29,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"cal_off_pss5",    'w', LOOP2, 30,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"accel_max_az",    'w', LOOP2, 31,      100.0/65535.0,             0.0, 'u', U_NONE},
  {"cal_off_pss6",    'w', LOOP2, 32,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"cal_d_pss1",      'w', LOOP2, 33,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  /* LOOP2 34 is fast */
  {"cal_d_pss2",      'w', LOOP2, 35,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"offset_ofpch_gy",  'w',LOOP2, 36,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ofroll_gy",'w',LOOP2, 37,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ofyaw_gy", 'w',LOOP2, 38,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"az_raw_mag",      'w',LOOP2, 39,            I2DEG,             0.0, 'u', U_D_DEG},
  {"sigma_mag",       'w',LOOP2, 40,            I2DEG,             0.0, 'u', U_NONE},
  {"az_dgps",         'w',LOOP2, 41,            I2DEG,             0.0, 'u', U_D_DEG},
  {"sigma_dgps",      'w',LOOP2, 42,            I2DEG,             0.0, 'u', U_NONE},
  {"cal_d_pss3",      'w', LOOP2, 44,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"cal_d_pss4",      'w', LOOP2, 45,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"mapsigma_thegood",'w',LOOP2, 46,           1.0/10.0,             0.0, 'u', U_NONE},
  // LOOP2 47-48 are fast
  {"cal_d_pss5",      'w', LOOP2, 50,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  /* LOOP2 51-54 are wide fast */
  {"az_mag",       'w', LOOP2, 56,              I2DEG,             0.0, 'u', U_D_DEG},
  {"cal_d_pss6",      'w', LOOP2, 57,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  {"cal_imin_pss",    'w', LOOP2, 58,       40.0/65536.0,             0.0, 's', U_TRIM_DEG},
  /* LOOP2 59 is unusued */
  /* LOOP2 60-61 are wide */
  /* LOOP2 62-63 are unusued */

  /* LOOP3 0-3 are unusued */
  /* LOOP3 4-5 are wide */
  {"t_cpu_i_flc",    'w', LOOP3,  6,               0.01,             0.0, 'u', U_NONE},
  {"bbc_fifo_size",'w', LOOP3,  7,             1./624,             0.0, 'u', U_NONE},
  {"t_cpu_b_flc",    'w', LOOP3,  8,               0.01,             0.0, 'u', U_NONE},
  {"t_mb_flc",      'w', LOOP3,  9,               0.01,             0.0, 'u', U_NONE},
  {"mks_hi_sip",   'w', LOOP3, 10,           0.003256,       -0.226858, 'u', U_NONE},
  {"mks_med_sip",  'w', LOOP3, 11,           0.032614,       -0.072580, 'u', U_NONE},
  {"nblobs_thebad",  'w', LOOP3, 12,                1.0,             0.0, 'u', U_NONE},
  {"blob00_x_thebad",'w', LOOP3, 13, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob00_y_thebad",'w', LOOP3, 14, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob00_f_thebad",'w', LOOP3, 15,                1.0,             0.0, 'u', U_NONE},
  {"blob00_s_thebad",'w', LOOP3, 16,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob01_x_thebad",'w', LOOP3, 17, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob01_y_thebad",'w', LOOP3, 18, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob01_f_thebad",'w', LOOP3, 19,                1.0,             0.0, 'u', U_NONE},
  {"blob01_s_thebad",'w', LOOP3, 20,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob02_x_thebad",'w', LOOP3, 21, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob02_y_thebad",'w', LOOP3, 22, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob02_f_thebad",'w', LOOP3, 23,                1.0,             0.0, 'u', U_NONE},
  {"blob02_s_thebad",'w', LOOP3, 24,          1.0/100.0,             0.0, 'u', U_NONE},
  {"mapmean_thebad", 'w', LOOP3, 25,                1.0,             0.0, 'u', U_NONE},
  {"ra_thegood",     'w', LOOP3, 26,          1.0/100.0,             0.0, 's', U_NONE},
  {"dec_thegood",    'w', LOOP3, 27,          1.0/100.0,             0.0, 's', U_NONE},
  {"roll_thegood",   'w', LOOP3, 28,          1.0/100.0,             0.0, 's', U_NONE},
  {"ra_thebad",      'w', LOOP3, 29,          1.0/100.0,             0.0, 's', U_NONE},
  {"foc_res_thebad", 'w', LOOP3, 30,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 31-32 are wide */
  {"dec_thebad",     'w', LOOP3, 33,          1.0/100.0,             0.0, 's', U_NONE},
  {"roll_thebad",    'w', LOOP3, 34,          1.0/100.0,             0.0, 's', U_NONE},
  {"mapsigma_thebad",'w', LOOP3, 35,           1.0/10.0,             0.0, 'u', U_NONE},
  {"move_tol_thebad",'w', LOOP3, 36,                1.0,             0.0, 'u', U_NONE},
  {"exp_int_thebad", 'w', LOOP3, 37,                1.0,             0.0, 'u', U_NONE},
  {"exp_time_thebad",'w', LOOP3, 38,                1.0,             0.0, 'u', U_NONE},
  {"force_thebad",   'w', LOOP3, 39,                1.0,             0.0, 'u', U_NONE},
  {"ra_theugly",     'w', LOOP3, 40,          1.0/100.0,             0.0, 's', U_NONE},
  {"dec_theugly",    'w', LOOP3, 41,          1.0/100.0,             0.0, 's', U_NONE},
  {"roll_theugly",   'w', LOOP3, 42,          1.0/100.0,             0.0, 's', U_NONE},
  /* LOOP3 43 is unusued */
  /* LOOP3 44-45 are wide */
  {"thresh_thebad",'w',LOOP3, 46,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"grid_thebad",  'w', LOOP3, 47,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 48-49 are unusued */
  {"mdist_thebad",   'w', LOOP3, 50,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 51-57 are unusued */
  /* LOOP3 58-59 are wide */
  /* LOOP3 60-63 are unusued */
  /* LOOP4 0-2 are unusued */
  {"focpos_thegood",'w',LOOP4,  3,           1.0/10.0,             0.0, 's', U_NONE},
  {"focpos_thebad",'w', LOOP4,  4,           1.0/10.0,             0.0, 's', U_NONE},
  {"focpos_theugly",'w',LOOP4,  5,           1.0/10.0,             0.0, 's', U_NONE},
  {"maxblob_thegood",'w',LOOP4, 6,                1.0,             0.0, 'u', U_NONE},
  {"maxblob_thebad",'w',LOOP4,  7,                1.0,             0.0, 'u', U_NONE},
  {"bi0_fifo_size",'w', LOOP4,  8,             1./624,             0.0, 'u', U_NONE},
  {"plover",       'w', LOOP4,  9,                1.0,             0.0, 'u', U_NONE},
  {"ccd_t_thegood",'w', LOOP4, 10,          1.0/100.0,             0.0, 's', U_NONE},
  /* LOOP4 11-14 are unusued */
  {"ccd_t_thebad", 'w', LOOP4, 15,          1.0/100.0,             0.0, 's', U_NONE},
  /* LOOP4 16-20 are unusued */
  /* LOOP4 23 is unusued */
  /* LOOP4 24-25 are wide */
  //{"cycle_state",  'w', LOOP4, 26,                1.0,             0.0, 'u', U_NONE},
  /* LOOP4 27-32 is unusued */
  {"g_p_heat_gy",   'w', LOOP4, 33,                1.0,             0.0, 'u', U_NONE},
  {"g_i_heat_gy",   'w', LOOP4, 34,                1.0,             0.0, 'u', U_NONE},
  {"g_d_heat_gy",   'w', LOOP4, 35,                1.0,             0.0, 'u', U_NONE},
  {"t_set_gy",     'w', LOOP4, 36,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"trim_pss",     'w', LOOP4, 39,              I2DEG,             0.0, 's', U_NONE},
  {"az_pss",       'w', LOOP4, 40,              I2DEG,             0.0, 'u', U_P_DEG},
  /* LOOP4 41 is unusued */
  /* LOOP4 42-43 appear unused */
  {"ra_1_p",       'w', LOOP4, 44,                I2H,             0.0, 'u', U_NONE}, // pointing mode coordinates
  {"dec_1_p",      'w', LOOP4, 45,              I2DEG,             0.0, 's', U_NONE},
  {"ra_2_p",       'w', LOOP4, 46,                I2H,             0.0, 'u', U_NONE},
  {"dec_2_p",      'w', LOOP4, 47,              I2DEG,             0.0, 's', U_NONE},
  {"ra_3_p",       'w', LOOP4, 48,                I2H,             0.0, 'u', U_NONE},
  {"dec_3_p",      'w', LOOP4, 49,              I2DEG,             0.0, 's', U_NONE},
  {"ra_4_p",       'w', LOOP4, 50,                I2H,             0.0, 'u', U_NONE},
  {"dec_4_p",      'w', LOOP4, 51,              I2DEG,             0.0, 's', U_NONE},
  {"trim_null",    'w', LOOP4, 54,              I2DEG,             0.0, 's', U_NONE},
  {"trim_mag",     'w', LOOP4, 55,              I2DEG,             0.0, 's', U_NONE},
  {"trim_dgps",    'w', LOOP4, 56,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP4 57 is unused */
  /* LOOP4 58-59 are wide */
  {"az_raw_dgps",  'w', LOOP4, 60,              I2DEG,             0.0, 'u', U_D_DEG},
  {"h_p",          'w', LOOP4, 63,              I2DEG,             0.0, 'u', U_NONE}, // scan height

  /* LOOP5 0 is unusued */
  /* LOOP5 2 is unusued */
  {"mks_lo_sip",   'w', LOOP5,  3,           0.327045,       -5.944902, 'u', U_NONE},
  /* LOOP5 4-5 are unused */
  /* LOOP5 6-7 are wide */
  {"alt",          'w', LOOP5,  8,                1.0,             0.0, 'u', U_NONE},
  {"mode_az_mc",   'w', LOOP5,  9,                1.0,             0.0, 'u', U_NONE},
  {"mode_el_mc",   'w', LOOP5, 10,                1.0,             0.0, 'u', U_NONE},
  {"dest_az_mc",   'w', LOOP5, 11,              I2DEG,             0.0, 'u', U_NONE},
  {"dest_el_mc",   'w', LOOP5, 12,              I2DEG,             0.0, 'u', U_NONE},
  {"vel_az_mc",    'w', LOOP5, 13,            1./6000,             0.0, 's', U_NONE},
  {"vel_el_mc",    'w', LOOP5, 14,            1./6000,             0.0, 's', U_NONE},
  {"dir_az_mc",    'w', LOOP5, 15,                1.0,             0.0, 's', U_NONE},
  {"dir_el_mc",    'w', LOOP5, 16,                1.0,             0.0, 's', U_NONE},
  {"slew_veto",    'w', LOOP5, 17,           1200.0/65535.0,             0.0, 'u', U_NONE},
  /* LOOP5 18-19 are wide */
  {"sveto_len",    'w', LOOP5, 20,           1200.0/65535.0,             0.0, 'u', U_NONE},
  {"dith_el",      'w', LOOP5, 23,        0.5/32768.0,             0.0, 's', U_D_DEG},
  {"state_lock",   'w', LOOP5, 25,                1.0,             0.0, 'u', U_NONE},
  /* LOOP5 28 is fast */
  /* LOOP5 34 is fast */
  /* LOOP5 35 is unused */
  /* LOOP5 41 is unusued */
  /* LOOP5 42-53 are wide */
  /* LOOP5 54 is unusued */
  {"pitch_mag",        'w',LOOP5, 55,           I2DEG,             0.0, 'u', U_NONE},
  /* LOOP5 56-63 are unusued */

  /* LOOP6 0-19 is unusued */
  /* LOOP6 23 is unusued */
  /* LOOP6 30-33 are wide */
  /* LOOP6 56-57 are wide */
  {"el_sun",       'w', LOOP6, 58,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP6 59 is unusued */
  /* LOOP6 61-62 are unusued */
  {"vel_ser_rw",   'w', LOOP7,  0,     2400.0/65536.0,  0.0, 's',      U_V_DPS},
  /* LOOP7 3 is fast narrow */
  {"res_rw",       'w', LOOP7,  3,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"stat_dr_rw",   'w', LOOP7,  4,             1.0,           0.0, 'u', U_NONE},
  {"stat_s1_rw",   'w', LOOP7,  5,             1.0,           0.0, 'u', U_NONE},
  
  {"stat_dr_piv",  'w', LOOP7, 10,                1.0,             0.0, 'u', U_NONE},  
  {"stat_s1_piv",  'w', LOOP7, 11,                1.0,             0.0, 'u', U_NONE},  
  {"i_ser_rw",     'w', LOOP7, 13,    60.0/32768.0,           0.0, 's', U_I_A},
  {"res_piv",      'w', LOOP7, 16,              I2DEG,             0.0, 'u', U_P_DEG},
  {"i_ser_piv",    'w', LOOP7, 17,       20.0/32768.0,             0.0, 's', U_I_A},
  {"g_v_rw_piv",   'w', LOOP7, 18,                1.0,             0.0, 'u', U_NONE},
  {"g_i_rw_piv",   'w', LOOP7, 19,                1.0,             0.0, 'u', U_NONE},
  {"g_v_az_piv",   'w', LOOP7, 20,                1.0,             0.0, 'u', U_NONE},
  {"g_t_rw_piv",   'w', LOOP7, 21,                1.0,             0.0, 'u', U_NONE},
  {"set_rw",       'w', LOOP7, 22,      500.0/32768.0,             0.0, 's', U_V_DPS},
  {"vel_dps_az",   'w', LOOP7, 23,       20.0/32768.0,             0.0, 's', U_V_DPS},
  {"i_v_rw_term_piv",  'w', LOOP7, 24,             1.0,        -32768.0, 'u', U_NONE},
  {"p_t_rw_term_piv",  'w', LOOP7, 25,             1.0,        -32768.0, 'u', U_NONE},
  {"p_v_req_az_term_piv", 'w', LOOP7, 26,          1.0,        -32768.0, 'u', U_NONE},
  {"vel_ser_piv",  'w', LOOP7, 27,   2000.0/65536.0,         0.0, 's', U_V_DPS},
  {"g_v_req_az_piv",'w',LOOP7, 29,                 1.0,             0.0, 'u', U_NONE},
  {"az_raw_pss1",   'w',   LOOP7, 30,              I2DEG,             0.0, 'u', U_P_DEG},
  {"az_raw_pss2",   'w',   LOOP7, 31,              I2DEG,             0.0, 'u', U_P_DEG},
  {"drive_info_rw",'w', LOOP7, 32,                 1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_rw",'w', LOOP7, 33,              1.0,             0.0, 'u', U_NONE},
  {"drive_info_piv",'w', LOOP7, 36,                1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_piv",'w', LOOP7, 37,             1.0,             0.0, 'u', U_NONE},
  /* LOOP7 38 is fast narrow */
  {"vel_hwp",          'w', LOOP7, 39,    10.0/65535.0,             0.0, 'u', U_V_DPS},
  {"i_move_hwp",       'w', LOOP7, 40,     1.2/65535.0,             0.0, 'u', U_I_A},
  {"status_hwp",       'w', LOOP7, 41,             1.0,             0.0, 'u', U_NONE},
  {"pos_1_hwp",        'w', LOOP7, 42,           I2DEG,             0.0, 'u', U_P_DEG},
  {"pos_2_hwp",        'w', LOOP7, 43,           I2DEG,             0.0, 'u', U_P_DEG},
  {"pos_3_hwp",        'w', LOOP7, 44,           I2DEG,             0.0, 'u', U_P_DEG},
  
  {"pitch_raw_dgps",   'w', LOOP7, 46,           I2DEG,             0.0, 'u', U_P_DEG},
  {"roll_raw_dgps",    'w', LOOP7, 47,           I2DEG,             0.0, 'u', U_P_DEG},
  {"verbose_rw",       'w', LOOP7, 48,             1.0,             0.0, 'u', U_NONE},
  {"verbose_piv",      'w', LOOP7, 52,             1.0,             0.0, 'u', U_NONE},
  {"p_v_rw_term_piv",  'w', LOOP7, 53,             1.0,        -32768.0, 'u', U_NONE},
  {"p_v_az_term_piv",  'w', LOOP7, 54,             1.0,        -32768.0, 'u', U_NONE},
  
  /* charge controller related channels */
  
  {"v_batt_cc1",  'w',  LOOP8,  3,   1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"v_arr_cc1",   'w',  LOOP8,  4,   1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"i_batt_cc1",  'w',  LOOP8,  5,   1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  {"i_arr_cc1",   'w',  LOOP8,  6,   1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  { "t_hs_cc1",   'w',  LOOP8,  7,   1.0,      0.0,            's',  U_T_C},
  {"fault_cc1",   'w',  LOOP8,  8,   1.0,      0.0,            'u',  U_NONE},  // fault bitfield
  {"alarm_hi_cc1",'w',  LOOP8,  9,   1.0,      0.0,            'u',  U_NONE},  // alarm high bitfield
  {"alarm_lo_cc1",'w',  LOOP8,  10,  1.0,      0.0,            'u',  U_NONE},  // alarm low bitfield
  {"v_targ_cc1",  'w',  LOOP8,  11,  1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"state_cc1",   'w',  LOOP8,  12,  1.0,      0.0,            'u',  U_NONE},   
  {"led_cc1",     'w',  LOOP8,  40,  1.0,      0.0,            'u', U_NONE}, // charge controller LED state

  {"v_batt_cc2",  'w',  LOOP9,  39,   1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"v_arr_cc2",   'w',  LOOP9,  40,   1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"i_batt_cc2",  'w',  LOOP9,  41,   1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  {"i_arr_cc2",   'w',  LOOP9,  42,   1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  { "t_hs_cc2",   'w',  LOOP9,  43,   1.0,      0.0,            's',  U_T_C},
  {"fault_cc2",   'w',  LOOP9,  44,   1.0,      0.0,            'u',  U_NONE},  // fault bitfield
  {"alarm_hi_cc2",'w',  LOOP9,  45,   1.0,      0.0,            'u',  U_NONE},  // alarm high bitfield
  {"alarm_lo_cc2",'w',  LOOP9,  46,  1.0,      0.0,            'u',  U_NONE},  // alarm low bitfield
  {"v_targ_cc2",  'w',  LOOP9,  47,  1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"state_cc2",   'w',  LOOP9,  48,  1.0,      0.0,            'u',  U_NONE},   
  {"led_cc2",     'w',  LOOP9,  49,  1.0,      0.0,            'u', U_NONE}, // charge controller LED state

  {"azraw_pss",   'w',   LOOP8, 26,              I2DEG,             0.0, 'u', U_P_DEG},
  {"elraw_pss",   'w',   LOOP8, 27,              I2DEG,             0.0, 'u', U_P_DEG},
  {"snr_pss1",     'w',   LOOP8, 28,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss2",     'w',   LOOP8, 29,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss3",     'w',   LOOP8, 30,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss4",     'w',   LOOP8, 31,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss5",     'w',   LOOP8, 32,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss6",     'w',   LOOP8, 33,            1/1000.,             0.0, 'u', U_NONE},
  {"accel_az",     'w',   LOOP8, 34,          2.0/65536,             0.0, 'u', U_NONE},
  {"az_raw_pss3",   'w',   LOOP8, 35,              I2DEG,             0.0, 'u', U_P_DEG},
  {"az_raw_pss4",   'w',   LOOP8, 36,              I2DEG,             0.0, 'u', U_P_DEG},

  {"az_cov_dgps",   'w', LOOP8, 37,              I2DEG,             0.0, 'u', U_NONE},
  {"pitch_cov_dgps",'w', LOOP8, 38,              I2DEG,             0.0, 'u', U_NONE},
  {"roll_cov_dgps", 'w', LOOP8, 39,              I2DEG,             0.0, 'u', U_NONE},

  {"force_theugly",     'w', LOOP8, 41,                1.0,             0.0, 'u', U_NONE},
  {"exp_int_theugly",   'w', LOOP8, 42,                1.0,             0.0, 'u', U_NONE},
  {"exp_time_theugly",  'w', LOOP8, 43,                1.0,             0.0, 'u', U_NONE},
  {"foc_res_theugly",   'w', LOOP8, 44,                1.0,             0.0, 'u', U_NONE},
  {"move_tol_theugly",  'w', LOOP8, 45,                1.0,             0.0, 'u', U_NONE},
  {"maxblob_theugly",   'w', LOOP8, 46,                1.0,             0.0, 'u', U_NONE},
  {"grid_theugly",      'w', LOOP8, 47,                1.0,             0.0, 'u', U_NONE},
  {"thresh_theugly",    'w', LOOP8, 48,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"mdist_theugly",     'w', LOOP8, 49,                1.0,             0.0, 'u', U_NONE},
  {"mapmean_theugly",   'w', LOOP8, 50,                1.0,             0.0, 'u', U_NONE},
  {"mapsigma_theugly",  'w', LOOP8, 51,           1.0/10.0,             0.0, 'u', U_NONE},
  {"ccd_t_theugly",     'w', LOOP8, 52,          1.0/100.0,             0.0, 's', U_NONE},
  {"nblobs_theugly",  'w', LOOP8, 53,                1.0,             0.0, 'u', U_NONE},
  {"blob00_x_theugly",  'w', LOOP8, 54, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob00_y_theugly",  'w', LOOP8, 55, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob00_f_theugly",  'w', LOOP8, 56,                1.0,             0.0, 'u', U_NONE},
  {"blob00_s_theugly",  'w', LOOP8, 57,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob01_x_theugly",  'w', LOOP8, 58, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob01_y_theugly",  'w', LOOP8, 59, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob01_f_theugly",  'w', LOOP8, 60,                1.0,             0.0, 'u', U_NONE},
  {"blob01_s_theugly",  'w', LOOP8, 61,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob02_x_theugly",  'w', LOOP8, 62, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob02_y_theugly",  'w', LOOP8, 63, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob02_f_theugly",  'w', LOOP9,  0,                1.0,             0.0, 'u', U_NONE},
  {"blob02_s_theugly",  'w', LOOP9,  1,          1.0/100.0,             0.0, 'u', U_NONE},
  {"g_pt_az",        'w', LOOP9,  2,                1.0,             0.0, 'u', U_NONE},
  {"pos_4_hwp",      'w', LOOP9,  4,              I2DEG,             0.0, 'u', U_P_DEG},
  {"pos_5_hwp",      'w', LOOP9,  5,              I2DEG,             0.0, 'u', U_P_DEG},
  {"pos_6_hwp",      'w', LOOP9,  6,              I2DEG,             0.0, 'u', U_P_DEG},
  {"rate_tdrss",     'w', LOOP9,  9,                1.0,             0.0, 'u', U_RATE},
  {"rate_iridium",   'w', LOOP9,  10,                1.0,             0.0, 'u', U_RATE},
  {"az_raw_pss5",   'w',   LOOP9, 26,              I2DEG,             0.0, 'u', U_P_DEG},
  {"az_raw_pss6",   'w',   LOOP9, 27,              I2DEG,             0.0, 'u', U_P_DEG},
  {"el_raw_pss1",   'w',   LOOP9, 28,              I2DEG,             0.0, 'u', U_P_DEG},
  {"el_raw_pss2",   'w',   LOOP9, 29,              I2DEG,             0.0, 'u', U_P_DEG},
  {"el_raw_pss3",   'w',   LOOP9, 30,              I2DEG,             0.0, 'u', U_P_DEG},
  {"el_raw_pss4",   'w',   LOOP9, 31,              I2DEG,             0.0, 'u', U_P_DEG},
  {"el_raw_pss5",   'w',   LOOP9, 32,              I2DEG,             0.0, 'u', U_P_DEG},
  {"el_raw_pss6",   'w',   LOOP9, 33,              I2DEG,             0.0, 'u', U_P_DEG},
  {"insert_last_hk", 'w', LOOP9,  34,                1.0,             0.0, 'u', U_NONE},
  {"f_bias_hk",      'w', LOOP9,  35,      400.0/65535.0,             0.0, 'u', U_F_HZ},
  {"v_heat_last_hk", 'w', LOOP9,  37,         CALDAC(1.0,            0.0), 'u', U_V_V},
  // {"pulse_last_hk",  'w', LOOP9,  38,                1.0,             0.0, 'u', U_NONE},
  /* LOOP9 50-55 are wide */
  {"i_tot",         'w', LOOP9, 56,              1.0e-3,            0.0, 'u', U_I_A}, // sum of currents read through ACS1 A1
  {"cov_lim_dgps",   'w', LOOP9, 58,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"ant_e_dgps",     'w', LOOP9, 60,          1.0/100.0,         0.0, 's',U_NONE},
  {"ant_n_dgps",     'w', LOOP9, 61,          1.0/100.0,         0.0, 's',U_NONE},
  {"ant_u_dgps",     'w', LOOP9, 62,          1.0/100.0,         0.0, 's',U_NONE},
  {"ants_lim_dgps",   'w', LOOP9, 63,    (100.0/32768.0),            0.0, 'u', U_NONE},
  {"frame_int_bbc",  'w', LOOP0,   0,                1.0,            0.0, 'u', U_NONE},
  {"rate_ext_bbc",   'w', LOOP0,   1,      400.0/65535.0,            0.0, 'u', U_F_HZ},
  {"frame_ext_bbc",  'w', LOOP0,   2,                1.0,            0.0, 'u', U_NONE},
  {"rate_frame_bbc", 'w', LOOP0,   3,      400.0/65535.0,            0.0, 'u', U_F_HZ},
  {"rate_samp_adc",  'w', LOOP0,  42,      20000.0/65535.0,          0.0, 'u', U_F_HZ},

  /* LOOP0 4-5 are wide fast TODO (change when hwp counts moved) */
  {"enc_cnt_2_hwp",'w', LOOP0,  6,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"enc_cnt_3_hwp",'w', LOOP0,  7,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"enc_cnt_4_hwp",'w', LOOP0,  8,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"enc_cnt_5_hwp",'w', LOOP0,  9,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"enc_cnt_6_hwp",'w', LOOP0, 10,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"fset",         'w', LOOP0, 11,         1,        0,      'u',      U_NONE},
  /* LOOP0 12-23 and 30-41 are wide slow */
  {"state_1_cycle",  'w', LOOP0,  24,                1.0,            0.0, 'u', U_NONE},
  {"state_2_cycle",  'w', LOOP0,  25,                1.0,            0.0, 'u', U_NONE},
  {"state_3_cycle",  'w', LOOP0,  26,                1.0,            0.0, 'u', U_NONE},
  {"state_4_cycle",  'w', LOOP0,  27,                1.0,            0.0, 'u', U_NONE},
  {"state_5_cycle",  'w', LOOP0,  28,                1.0,            0.0, 'u', U_NONE},
  {"state_6_cycle",  'w', LOOP0,  29,                1.0,            0.0, 'u', U_NONE},
  {"band_az",        'w', LOOP0,  43,                30.0/65535.0,   0.0, 'u', U_P_DEG},
  // TODO: GET RID OF THIS AFTER TURN AROUND FLAG HAS BEEN TESTED AND WORKS
  {"is_turn_around", 'w', LOOP0,  44,                1.0,            0.0, 'u', U_NONE},
  {"row_len_sync",   'w', LOOP0,  45,                1.0,            0.0, 'u', U_NONE},
  {"num_rows_sync",  'w', LOOP0,  46,                1.0,            0.0, 'u', U_NONE},
  {"free_run_sync",  'w', LOOP0,  47,                1.0,            0.0, 'u', U_NONE},
  // LOOP0 48-51 are wide slow
  {"last_b_cmd",     'w', LOOP0,  52,                1.0,             0.0, 'u', U_NONE},
  {"last_i_cmd",     'w', LOOP0,  53,                1.0,             0.0, 'u', U_NONE},
  {"count_b_cmd",    'w', LOOP0,  54,                1.0,             0.0, 'u', U_NONE},
  {"count_i_cmd",    'w', LOOP0,  55,                1.0,             0.0, 'u', U_NONE},
  {"delay_az",       'w', LOOP0,  56,                (10.0/32768.0),  0.0, 'u', U_NONE},
  
#ifndef BOLOTEST
/* ACS1 Digital I/O card */
  {"latch0",       'w',  ACS1_D,  0,                1.0,             0.0, 'u', U_NONE},
  {"latch1",       'w',  ACS1_D,  1,                1.0,             0.0, 'u', U_NONE},
  {"switch_gy",    'w',  ACS1_D,  2,                1.0,             0.0, 'u', U_NONE},
  //this may be unused because of the new program
  {"switch_grp2",  'w',  ACS1_D,  3,                1.0,             0.0, 'u', U_NONE},
  {"switch_misc",  'w',  ACS1_D,  4,                1.0,             0.0, 'u', U_NONE},
  {"heat_gy",      'w',  ACS1_D,  6,                1.0,             0.0, 'u', U_NONE},


/* ACS1 Analog card 1 */
/* default current cal is 12.5, 0.0 */
  {"i_trans",      'r',  ACS1_A1, 1,          CAL16(11.32, -0.0566),      'u', U_I_A},
  {"i_das",        'r',  ACS1_A1, 3,          CAL16(11.03, -0.041),      'u', U_I_A},
  {"i_acs",        'r',  ACS1_A1, 5,          CAL16(12.5,  0.0),        'u', U_I_A},
  {"i_mcc",        'r',  ACS1_A1, 7,          CAL16(10.75, -0.05),      'u', U_I_A},
  {"i_sc",         'r',  ACS1_A1, 9,          CAL16(10.89, -0.058),      'u', U_I_A},
  {"i_dgps",       'r',  ACS1_A1, 11,         CAL16(11.03, -0.066),      'u', U_I_A},
  {"i_piv",        'r',  ACS1_A1, 13,         CAL16(12.5,  -0.03125),      'u', U_I_A},
  {"i_el",         'r',  ACS1_A1, 15,         CAL16(12.5, -0.11),       'u', U_I_A},
  {"i_flc",        'r',  ACS1_A1, 17,         CAL16(12.5, -0.09),       'u', U_I_A},
  {"i_rw",         'r',  ACS1_A1, 19,         CAL16(12.5, -0.0742),       'u', U_I_A},
  {"i_step",       'r',  ACS1_A1, 21,         CAL16(11.0, -0.099),       'u', U_I_A},
  {"i_gy",         'r',  ACS1_A1, 23,         CAL16(12.5,  -0.04),        'u', U_I_A},
/* ACS_A1 24-41 are unused. */

/* ACS1 Temperature card */
  {"t_gy",         'r',  ACS1_T1, 1,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_serial",     'r',  ACS1_T1, 3,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_mc_piv",     'r',  ACS1_T1, 5,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_piv",        'r',  ACS1_T1, 7,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_wd_flc",        'r',  ACS1_T1, 9,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_port_hexc",    'r',  ACS1_T1, 11,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_1_bat",      'r',  ACS1_T1, 13,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_2_bat",      'r',  ACS1_T1, 15,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_port_back",      'r',  ACS1_T1, 17,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_dgps",      'r',  ACS1_T1, 19,         CAL16T(1.0, 0.0),         'u', U_T_C}, /* Not used can be re-assigned*/
  {"t_star_front",      'r',  ACS1_T1, 21,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_acs",        'r',  ACS1_T1, 23,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_dcdc_acs",   'r',  ACS1_T1, 25,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_mc_lock",    'r',  ACS1_T1, 27,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_lock",       'r',  ACS1_T1, 29,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_bsc",        'r',  ACS1_T1, 31,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_box_bal",    'r',  ACS1_T1, 33,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_pump_bal",   'r',  ACS1_T1, 35,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_el",         'r',  ACS1_T1, 37,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_array",      'r',  ACS1_T1, 39,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_sun",        'r',  ACS1_T1, 41,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_rw",        'r',  ACS1_T1, 43,         CAL16T(1.0, 0.0),         'u', U_V_V},
  {"t_earth",      'r',  ACS1_T1, 45,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_chin",       'r',  ACS1_T1, 47,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_port_pyr",       'r',  ACS1_T1, 49,         CAL16T(1.0, 0.0),         'u', U_T_C},

/* ACS2 Digital I/O card */
  // ACS2_D 20-22 are unused
  {"dac2_ampl",    'w',  ACS2_D,  1,                 1.0,             0.0, 'u', U_NONE},
  {"dac_piv",      'w',  ACS2_D,  2,                 1.0,             0.0, 'u', U_NONE},
  {"mask_gy",      'w',  ACS2_D, 13,                1.0,             0.0, 'u', U_NONE},
  {"control_lock", 'w',  ACS2_D, 14,                1.0,             0.0, 'u', U_NONE},
  {"g_p_az",       'w',  ACS2_D, 23,                1.0,             0.0, 'u', U_NONE},
  {"g_i_az",       'w',  ACS2_D, 24,                1.0,             0.0, 'u', U_NONE},
  {"enc1_offset",  'w',  ACS2_D, 25,            I2DEG,                0.0,  'u', U_P_DEG},
  {"enc2_offset",  'w',  ACS2_D, 26,            I2DEG,                0.0,  'u', U_P_DEG}, 
  {"fault_gy",     'r',  ACS2_D, 15,                1.0,             0.0, 'u', U_NONE},
  {"dac_rw",       'r',   ACS2_D, 24,                 1.0,            0.0, 'u', U_NONE},
  {"p_term_az",    'r',   ACS2_D, 25,                1.0,        -32768.0, 'u', U_NONE},
  {"i_term_az",    'r',   ACS2_D, 26,                1.0,        -32768.0, 'u', U_NONE},
  {"error_az",     'r',   ACS2_D, 27,        614.4e-6, 614.4*(-32768.0e-6), 'u', U_NONE},
  {"limit_lock",   'r',   ACS2_D, 30,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Analog card */
  {"pitch_piv_clin",'r', ACS2_A1,  1,           0.001343,          -43.18, 'u', U_NONE},
  {"roll_piv_clin",'r',  ACS2_A1,  3,           0.001413,         -44.788, 'u', U_NONE},
  {"t_piv_clin",   'r',  ACS2_A1,  5, 100.0*10.0/32768.0,     -100.0*10.0, 'u', U_NONE},
  {"pitch_of_clin",'r',  ACS2_A1,  7,            -0.00143,         47.555, 'u', U_NONE},
  {"roll_of_clin", 'r',  ACS2_A1,  9,              -0.001,           31.6, 'u', U_NONE},
  {"t_of_clin",    'r',  ACS2_A1, 11, 100.0*10.0/32768.0,     -100.0*10.0, 'u', U_NONE},
  {"x_mag",        'r',  ACS2_A1, 13,              MAGX_M,         MAGX_B, 'u', U_NONE},
  {"y_mag",        'r',  ACS2_A1, 15,              MAGY_M,         MAGY_B, 'u', U_NONE},
  {"z_mag",        'r',  ACS2_A1, 17,              MAGZ_M,         MAGZ_B, 'u', U_NONE},
  {"trig_thegood", 'r',  ACS2_A1, 23,        CAL16(1.0,          0.0), 'u',  U_V_V}, 
  {"trig_thebad",  'r',  ACS2_A1, 25,        CAL16(1.0,          0.0), 'u',  U_V_V}, 
  {"trig_theugly", 'r',  ACS2_A1, 27,        CAL16(1.0,          0.0), 'u',  U_V_V}, 
  {"t_ofpch_gy",    'r',  ACS2_A1, 47,        CAL16(50.0, 0.0),     'u',  U_T_C},
  {"trig_s_thegood",'r', ACS2_A1, 51,                1.0,             0.0, 'u', U_NONE},
  {"trig_l_thegood",'r', ACS2_A1, 52,                1.0,             0.0, 'u', U_NONE},
  {"trig_s_thebad", 'r', ACS2_A1, 53,                1.0,             0.0, 'u', U_NONE},
  {"trig_l_thebad", 'r', ACS2_A1, 54,                1.0,             0.0, 'u', U_NONE},
  {"trig_s_theugly",'r', ACS2_A1, 55,                1.0,             0.0, 'u', U_NONE},
  {"trig_l_theugly",'r', ACS2_A1, 56,                1.0,             0.0, 'u', U_NONE},
  {"v1_1_pss",     'r',  ACS2_A2,  1,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_1_pss",     'r',  ACS2_A2,  3,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_1_pss",     'r',  ACS2_A2,  5,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_1_pss",     'r',  ACS2_A2,  7,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_2_pss",     'r',  ACS2_A2, 11,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_2_pss",     'r',  ACS2_A2, 13,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_2_pss",     'r',  ACS2_A2, 15,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_2_pss",     'r',  ACS2_A2, 17,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_3_pss",     'r',  ACS2_A2, 19,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_3_pss",     'r',  ACS2_A2, 21,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_3_pss",     'r',  ACS2_A2, 23,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_3_pss",     'r',  ACS2_A2, 25,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_4_pss",     'r',  ACS2_A2, 27,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_4_pss",     'r',  ACS2_A2, 29,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_4_pss",     'r',  ACS2_A2, 31,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_4_pss",     'r',  ACS2_A2, 47,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_5_pss",     'r',  ACS2_A2, 39,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_5_pss",     'r',  ACS2_A2, 41,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_5_pss",     'r',  ACS2_A2, 43,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_5_pss",     'r',  ACS2_A2, 45,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_6_pss",     'r',  ACS2_A2, 49,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_6_pss",     'r',  ACS2_A2, 33,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_6_pss",     'r',  ACS2_A2, 35,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_6_pss",     'r',  ACS2_A2, 37,           CAL16(-1.,            0.),  'u', U_V_V},

#endif

  {"heat_t_hk",    'w',  HWP_D,  53,            1.0,          0.0, 'u', U_NONE},
  {"heat_13_hk",   'w',  RTD_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"heat_45_hk",   'w',  RTD_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"heat_26_hk",   'w',  RTD_D,  52,            1.0,          0.0, 'u', U_NONE},

  /* slow MCE data -- all these are 6x mulitplexed using the mce_txmux channel
   */
  {"mce_txmux",   'w', LOOP11, 0, 1, 0, 'u', U_NONE},
  {"data_mode",   'w', LOOP11, 1, 1, 0, 'u', U_NONE},
  /* LOOP11 2-3 are wide */
  {"mic_free",    'w', LOOP11, 4, (1<<24), 0, 'u', U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
#ifndef BOLOTEST
  {"ofyaw_1_gy",  'r',  ACS2_D,  0, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofyaw_2_gy",  'r',  ACS2_D,  2, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofroll_1_gy", 'r',  ACS2_D,  4, DGY32_TO_DPS,   -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofroll_2_gy", 'r',  ACS2_D,  6, DGY32_TO_DPS,   -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofpch_2_gy",   'r',  ACS2_D,  8, DGY32_TO_DPS,   -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofpch_1_gy",   'r',  ACS2_D, 10, DGY32_TO_DPS,   -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"enc_table",   'r',  ACS2_D, 54,     360.0/144000.0,             0.0, 'U', U_P_DEG},
#endif

  {"az",          'w', LOOP2,   51,             LI2DEG,             0.0, 'U', U_P_DEG},
  {"el",          'w', LOOP2,   53,             LI2DEG,             0.0, 'U', U_P_DEG},

/* housekeeping channels */  /* TODO many can probably be slow */
  {"vr_ntd1_2_hk",   'r', RTD_A1,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_2_hk",     'r', RTD_A1,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_2_hk",    'r', RTD_A1,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_2_hk",   'r', RTD_A1, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_2_hk",   'r', RTD_A1, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_2_hk",   'r', RTD_A1, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd1_3_hk",   'r', RTD_A1, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_3_hk",     'r', RTD_A1, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_3_hk",    'r', RTD_A1, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_3_hk",   'r', RTD_A1, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_3_hk",   'r', RTD_A1, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_3_hk",   'r', RTD_A1, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  //NB: (2011-09-25) 50Hz cals measured for board #1. other vr_ cals use mean
  {"vr_ntd1_1_hk",   'r', RTD_A1, 36, CAL32N(       1.134,           0.0), 'U', U_V_V},
  {"vr_fp_1_hk",     'r', RTD_A1, 38, CAL32C(       1.052,           0.0), 'U', U_V_V},
  {"vr_strap_1_hk",    'r', RTD_A1, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_1_hk",   'r', RTD_A1, 42, CAL32N(       1.126,           0.0), 'U', U_V_V},
  {"vr_ntd3_1_hk",   'r', RTD_A1, 44, CAL32N(       1.122,           0.0), 'U', U_V_V},
  {"vr_ntd2_1_hk",   'r', RTD_A1, 46, CAL32N(       1.132,           0.0), 'U', U_V_V},

  {"vr_ntd1_5_hk",   'r', RTD_A2,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_5_hk",     'r', RTD_A2,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_5_hk",    'r', RTD_A2,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_5_hk",   'r', RTD_A2, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_5_hk",   'r', RTD_A2, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_5_hk",   'r', RTD_A2, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd1_6_hk",   'r', RTD_A2, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_6_hk",     'r', RTD_A2, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_6_hk",    'r', RTD_A2, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_6_hk",   'r', RTD_A2, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_6_hk",   'r', RTD_A2, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_6_hk",   'r', RTD_A2, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd1_4_hk",   'r', RTD_A2, 36, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_4_hk",     'r', RTD_A2, 38, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_4_hk",    'r', RTD_A2, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_4_hk",   'r', RTD_A2, 42, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_4_hk",   'r', RTD_A2, 44, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_4_hk",   'r', RTD_A2, 46, CAL32N(       1.129,           0.0), 'U', U_V_V},
  
  {"vp_01_hk",       'r', DIOD_A2, 48, CAL32(1.0, 0.0),                    'U', U_V_V},
  {"vp_02_hk",       'r', DIOD_A3, 48, CAL32(1.0, 0.0),                    'U', U_V_V},
  
  {"enc_a_1_hwp",  'r',HWP_A1,  0,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_1_hwp",  'r',HWP_A1,  2,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_1_hwp",  'r',HWP_A1,  4,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_1_hwp",  'r',HWP_A1,  6,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_2_hwp",  'r',HWP_A1,  8,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_2_hwp",  'r',HWP_A1, 10,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_2_hwp",  'r',HWP_A1, 12,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_2_hwp",  'r',HWP_A1, 14,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_3_hwp",  'r',HWP_A1, 16,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_3_hwp",  'r',HWP_A1, 18,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_3_hwp",  'r',HWP_A1, 20,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_3_hwp",  'r',HWP_A1, 22,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_4_hwp",  'r',HWP_A1, 24,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_4_hwp",  'r',HWP_A1, 26,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_4_hwp",  'r',HWP_A1, 28,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_4_hwp",  'r',HWP_A1, 30,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_5_hwp",  'r',HWP_A1, 32,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_5_hwp",  'r',HWP_A1, 34,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_5_hwp",  'r',HWP_A1, 36,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_5_hwp",  'r',HWP_A1, 38,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_6_hwp",  'r',HWP_A1, 40,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_6_hwp",  'r',HWP_A1, 42,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_6_hwp",  'r',HWP_A1, 44,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_6_hwp",  'r',HWP_A1, 46,CAL32(1.0,      0.0),      'U',      U_V_V},

  //TODO this counts can be made narrow slow once debugging is complete
  {"enc_cnt_1_hwp",'w', LOOP0,  4,    LI2DEG,      0.0,      'U',      U_P_DEG},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
#ifndef BOLOTEST
/* ACS2 Common Node */
  {"framenum",     'r',  ACS2_C,  1,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Digital Card */
  {"ofpch_gy",      'r',   ACS2_D, 12, GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ofroll_gy",    'r',   ACS2_D, 13,GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ofyaw_gy",     'r',   ACS2_D, 14,-GY16_TO_DPS, GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ofaz_gy",     'r',   ACS2_D, 16  ,-GY16_TO_DPS, GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS}, 
  {"vel_req_az",   'w',   ACS2_D, 27, GY16_TO_DPS,-32768.0*GY16_TO_DPS, 'u', U_V_DPS},
  {"step_1_el",    'w',   ACS2_D, 29,     10000.0/32767.0, -10000.30518509, 'u', U_F_HZ},
  {"step_2_el",    'w',   ACS2_D, 30,     10000.0/32767.0, -10000.30518509, 'u', U_F_HZ},
  {"el_raw_1_enc", 'r',   ACS2_D, 56,               I2DEG,            ENC1_OFFSET, 'u', U_P_DEG},
  {"el_raw_2_enc", 'r',   ACS2_D, 57,               -I2DEG,           ENC2_OFFSET, 'u', U_P_DEG},
  {"vel_ofpch_gy",  'r', ACS2_A1, 45,    CAL16(14.9925037, 0.0), 'u', U_V_DPS},  
  {"pulse_sc",     'r',  ACS2_A1, 50,                 1.0,            0.0, 'u', U_NONE},
  {"dps_table",    'w',    LOOP1, 34,           70.0/32767.0,            0.0, 's', U_V_DPS},

#endif
  {"chatter",      'w', LOOP7, 38,     1.0,        0.0,      'u',      U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,             1.0,                    0.0, 'u', U_NONE},
  {"polarity",    'w', DECOM,  2,             1.0,                    0.0, 'u', U_NONE},
  {"decom_unlock",'w', DECOM,  3,             1.0,                    0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
