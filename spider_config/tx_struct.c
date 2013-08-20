/* tx_struct.c: contains the channel specificiation lists
 *
 * This software is copyright (C) 2002-2013 University of Toronto
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
#include <stdlib.h>

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
#define LOOP0    32, 0
#define LOOP1    33, 0
#define LOOP2    34, 0
#define LOOP3    35, 0
#define LOOP4    36, 0
#define LOOP5    37, 0
#define LOOP6    38, 0
#define LOOP7    39, 0
#define LOOP8    40, 0
#define LOOP9    41, 0
#define LOOP10   42, 0
#define LOOP11   43, 0
#define DECOM    44, 0
#define LOOPMCE0 45, 0
#define LOOPMCE1 46, 0
#define LOOPMCE2 47, 0
#define LOOPMCE3 48, 0

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
#define U_T_H   "Time","hours"
#define U_R_O   "Resistance","Ohms"
#define U_RATE "Rate", "bps"
#define U_F_HZ "Frequency", "Hz"
#define U_TRIM_DEG "Trim","^o"
#define U_TRIM_MM "Trim","mm"

struct ChannelStruct WideSlowChannels[] = {
  {"count_1_el", 'r',ACS2_D, 58,            1.0,             0.0, 'U', U_NONE},
  {"count_2_el", 'r',ACS2_D, 60,            1.0,             0.0, 'U', U_NONE},
  {"time",       'w', LOOP0,  0,            1.0,             0.0, 'U', U_NONE},
  {"time_usec",  'w', LOOP0,  2,            1.0,             0.0, 'U', U_NONE},
  {"time_sip",   'w', LOOP0,  4,            1.0,             0.0, 'U', U_NONE},
  {"time_dgps",  'w', LOOP0,  6,            1.0,             0.0, 'U', U_NONE},
  {"lst",        'w', LOOP0,  8,     1.0/3600.0,             0.0, 'U', U_NONE},
  {"parts_sched",'w', LOOP0, 10,            1.0,             0.0, 'U', U_NONE},
  {"frame_g",    'w', LOOP0, 12,            1.0,             0.0, 'U', U_NONE},
  {"lat",        'w', LOOP0, 14,         LI2DEG,             0.0, 'S', U_NONE},
  {"lon",        'w', LOOP0, 16,         LI2DEG,             0.0, 'S', U_NONE},
  {"sec_g",      'w', LOOP0, 18,            1.0,             0.0, 'U', U_NONE},
  {"usec_g",     'w', LOOP0, 20,            1.0,             0.0, 'U', U_NONE},
  {"ra",         'w', LOOP0, 22,           LI2H,             0.0, 'U', U_P_HR},
  {"frame_b",    'w', LOOP0, 24,            1.0,             0.0, 'U', U_NONE},
  {"sec_b",      'w', LOOP0, 26,            1.0,             0.0, 'U', U_NONE},
  {"usec_b",     'w', LOOP0, 28,            1.0,             0.0, 'U', U_NONE},
  {"dec",        'w', LOOP0, 30,         LI2DEG,             0.0, 'S', U_P_DEG},
  {"lst_sched",  'w', LOOP0, 32,            1.0,             0.0, 'U', U_NONE},
  {"frame_u",    'w', LOOP0, 34,            1.0,             0.0, 'U', U_NONE},
  {"sec_u",      'w', LOOP0, 36,            1.0,             0.0, 'U', U_NONE},
  {"usec_u",     'w', LOOP0, 38,            1.0,             0.0, 'U', U_NONE},
  {"time_b_flc", 'w', LOOP0, 40,            1.0,             0.0, 'U', U_NONE},
  {"time_i_flc", 'w', LOOP0, 42,            1.0,             0.0, 'U', U_NONE},
  //derived channel time_u adds these together
  {"time_start_x1_cycle",  'w', LOOP0, 44,    1.0,            0.0, 'U', U_NONE},
  {"time_start_x2_cycle",  'w', LOOP0, 46,    1.0,            0.0, 'U', U_NONE},
  {"time_start_x3_cycle",  'w', LOOP0, 48,    1.0,            0.0, 'U', U_NONE},
  {"time_start_x4_cycle",  'w', LOOP0, 50,    1.0,            0.0, 'U', U_NONE},
  {"time_start_x5_cycle",  'w', LOOP0, 52,    1.0,            0.0, 'U', U_NONE},
  {"time_start_x6_cycle",  'w', LOOP0, 54,    1.0,            0.0, 'U', U_NONE},
  {"time_state_x1_cycle",  'w', LOOP0, 56,    1.0,            0.0, 'U', U_NONE},
  {"time_state_x2_cycle",  'w', LOOP0, 58,    1.0,            0.0, 'U', U_NONE},
  {"time_state_x3_cycle",  'w', LOOP0, 60,    1.0,            0.0, 'U', U_NONE},
  {"time_state_x4_cycle",  'w', LOOP0, 62,    1.0,            0.0, 'U', U_NONE},
  {"time_state_x5_cycle",  'w', LOOP1,  0,    1.0,            0.0, 'U', U_NONE},
  {"time_state_x6_cycle",  'w', LOOP1,  2,    1.0,            0.0, 'U', U_NONE},
  {"mce_cmplex",           'w', LOOP1,  4,    1.0,            0.0, 'U', U_NONE},

  {"time_mcc1",     'w',  LOOP1,  6, 0.01, MPC_EPOCH, 'U', U_NONE},
  {"time_mcc2",     'w',  LOOP1,  8, 0.01, MPC_EPOCH, 'U', U_NONE},
  {"time_mcc3",     'w',  LOOP1, 10, 0.01, MPC_EPOCH, 'U', U_NONE},
  {"time_mcc4",     'w',  LOOP1, 12, 0.01, MPC_EPOCH, 'U', U_NONE},
  {"time_mcc5",     'w',  LOOP1, 14, 0.01, MPC_EPOCH, 'U', U_NONE},
  {"time_mcc6",     'w',  LOOP1, 16, 0.01, MPC_EPOCH, 'U', U_NONE},

  /* housekeeping channels */  /* TODO many can probably be not-wide */
  {"vr_still_x2_hk",  'r', RTD_A1,  0, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_mux_x2_hk",    'r', RTD_A1,  2, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_still_x3_hk",  'r', RTD_A1, 16, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_mux_x3_hk",    'r', RTD_A1, 18, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_still_x1_hk",  'r', RTD_A1, 32, CAL32C(1.053,          0.0), 'U', U_V_V},
  {"vr_mux_x1_hk",    'r', RTD_A1, 34, CAL32C(1.065,          0.0), 'U', U_V_V},
  {"vr_still_x5_hk",  'r', RTD_A2,  0, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_mux_x5_hk",    'r', RTD_A2,  2, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_still_x6_hk",  'r', RTD_A2, 16, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_mux_x6_hk",    'r', RTD_A2, 18, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_still_x4_hk",  'r', RTD_A2, 32, CAL32C(1.055,          0.0), 'U', U_V_V},
  {"vr_mux_x4_hk",    'r', RTD_A2, 34, CAL32C(1.055,          0.0), 'U', U_V_V},

  //TODO name the diode voltages. Must change derived.c to match
  {"vd_truss_x2_hk",    'r', DIOD_A1,  0, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_cp_x2_hk",       'r', DIOD_A1,  2, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_pump_x2_hk",     'r', DIOD_A1,  4, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_hsw_x2_hk",      'r', DIOD_A1,  6, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_plate_x2_hk",    'r', DIOD_A1,  8, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_snout_x2_hk",    'r', DIOD_A1, 10, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_eyepiece_x2_hk", 'r', DIOD_A1, 12, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_objective_x2_hk",'r', DIOD_A1, 14, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_spittoon_x2_hk", 'r', DIOD_A1, 16, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_aux_post_x2_hk", 'r', DIOD_A1, 18, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_stop_x2_hk",     'r', DIOD_A1, 20, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_ssa_x2_hk",      'r', DIOD_A1, 22, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_truss_x1_hk",    'r', DIOD_A1, 24, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_cp_x1_hk",       'r', DIOD_A1, 26, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_pump_x1_hk",     'r', DIOD_A1, 28, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_hsw_x1_hk",      'r', DIOD_A1, 30, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_plate_x1_hk",    'r', DIOD_A1, 32, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_snout_x1_hk",    'r', DIOD_A1, 34, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_eyepiece_x1_hk", 'r', DIOD_A1, 36, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_objective_x1_hk",'r', DIOD_A1, 38, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_spittoon_x1_hk", 'r', DIOD_A1, 40, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_aux_post_x1_hk", 'r', DIOD_A1, 42, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_stop_x1_hk",     'r', DIOD_A1, 44, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_ssa_x1_hk",      'r', DIOD_A1, 46, CAL32D(     1.0,    0.0), 'U', U_V_V},

  {"vd_truss_x4_hk",    'r', DIOD_A2,  0, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_cp_x4_hk",       'r', DIOD_A2,  2, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_pump_x4_hk",     'r', DIOD_A2,  4, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_hsw_x4_hk",      'r', DIOD_A2,  6, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_plate_x4_hk",    'r', DIOD_A2,  8, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_snout_x4_hk",    'r', DIOD_A2, 10, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_eyepiece_x4_hk", 'r', DIOD_A2, 12, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_objective_x4_hk",'r', DIOD_A2, 14, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_spittoon_x4_hk", 'r', DIOD_A2, 16, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_aux_post_x4_hk", 'r', DIOD_A2, 18, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_stop_x4_hk",     'r', DIOD_A2, 20, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_ssa_x4_hk",      'r', DIOD_A2, 22, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_truss_x3_hk",    'r', DIOD_A2, 24, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_cp_x3_hk",       'r', DIOD_A2, 26, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_pump_x3_hk",     'r', DIOD_A2, 28, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_hsw_x3_hk",      'r', DIOD_A2, 30, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_plate_x3_hk",    'r', DIOD_A2, 32, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_snout_x3_hk",    'r', DIOD_A2, 34, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_eyepiece_x3_hk", 'r', DIOD_A2, 36, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_objective_x3_hk",'r', DIOD_A2, 38, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_spittoon_x3_hk", 'r', DIOD_A2, 40, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_aux_post_x3_hk", 'r', DIOD_A2, 42, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_stop_x3_hk",     'r', DIOD_A2, 44, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_ssa_x3_hk",      'r', DIOD_A2, 46, CAL32D(     1.0,    0.0), 'U', U_V_V},

  {"vd_truss_x6_hk",    'r', DIOD_A3,  0, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_cp_x6_hk",       'r', DIOD_A3,  2, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_pump_x6_hk",     'r', DIOD_A3,  4, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_hsw_x6_hk",      'r', DIOD_A3,  6, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_plate_x6_hk",    'r', DIOD_A3,  8, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_snout_x6_hk",    'r', DIOD_A3, 10, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_eyepiece_x6_hk", 'r', DIOD_A3, 12, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_objective_x6_hk",'r', DIOD_A3, 14, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_spittoon_x6_hk", 'r', DIOD_A3, 16, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_aux_post_x6_hk", 'r', DIOD_A3, 18, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_stop_x6_hk",     'r', DIOD_A3, 20, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_ssa_x6_hk",      'r', DIOD_A3, 22, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_truss_x5_hk",    'r', DIOD_A3, 24, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_cp_x5_hk",       'r', DIOD_A3, 26, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_pump_x5_hk",     'r', DIOD_A3, 28, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_hsw_x5_hk",      'r', DIOD_A3, 30, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_plate_x5_hk",    'r', DIOD_A3, 32, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_snout_x5_hk",    'r', DIOD_A3, 34, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_eyepiece_x5_hk", 'r', DIOD_A3, 36, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_objective_x5_hk",'r', DIOD_A3, 38, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_spittoon_x5_hk", 'r', DIOD_A3, 40, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_aux_post_x5_hk", 'r', DIOD_A3, 42, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_stop_x5_hk",     'r', DIOD_A3, 44, CAL32D(     1.0,    0.0), 'U', U_V_V},
  {"vd_ssa_x5_hk",      'r', DIOD_A3, 46, CAL32D(     1.0,    0.0), 'U', U_V_V},

  {"vd_mt_tophi_t_hk",    'r', HWP_A2,  0, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs1_bottom_t_hk", 'r', HWP_A2,  2, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_mt_botlo2_t_hk",   'r', HWP_A2,  4, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs2_filter_t_hk", 'r', HWP_A2,  6, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_sft_nose_t_hk",    'r', HWP_A2,  8, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs2_bottom_t_hk", 'r', HWP_A2, 10, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs1_filter_t_hk", 'r', HWP_A2, 12, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_capillary_t_hk",   'r', HWP_A2, 14, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs2_top_t_hk",    'r', HWP_A2, 16, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_mt_botlo_t_hk",    'r', HWP_A2, 18, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_mt_bothi_t_hk",    'r', HWP_A2, 20, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_sft_bottom_t_hk",  'r', HWP_A2, 22, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs1_top_t_hk",    'r', HWP_A2, 24, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_mt_toplo_t_hk",    'r', HWP_A2, 26, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_capillary2_t_hk",  'r', HWP_A2, 28, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs2_flex_t_hk",   'r', HWP_A2, 30, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_sft_ring_t_hk",    'r', HWP_A2, 32, CAL32D(1.0,  0.0), 'U', U_V_V},
  {"vd_vcs1_apert_t_hk",  'r', HWP_A2, 34, CAL32D(1.0,  0.0), 'U', U_V_V},

  END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  //status and sync channels for handshaking with bbus nodes
  {"status00",     'r',  ACS1_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"status01",     'r',  ACS1_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"status02",     'r', ACS1_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"status03",     'r', ACS1_T1, 63,         1.0,             0.0, 'u', U_NONE},
  {"status04",     'r',  ACS2_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"status05",     'r',  ACS2_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"status06",     'r', ACS2_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"status07",     'r', ACS2_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"status08",     'r',   RTD_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"status09",     'r',   RTD_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"status10",     'r',  RTD_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"status11",     'r',  RTD_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"status12",     'r',  DIOD_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"status13",     'r', DIOD_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"status14",     'r', DIOD_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"status15",     'r', DIOD_A3, 63,         1.0,             0.0, 'u', U_NONE},
  {"status16",     'r',   HWP_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"status17",     'r',   HWP_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"status18",     'r',  HWP_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"status19",     'r',  HWP_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync00",       'w',  ACS1_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync01",       'w',  ACS1_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync02",       'w', ACS1_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync03",       'w', ACS1_T1, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync04",       'w',  ACS2_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync05",       'w',  ACS2_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync06",       'w', ACS2_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync07",       'w', ACS2_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync08",       'w',   RTD_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync09",       'w',   RTD_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync10",       'w',  RTD_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync11",       'w',  RTD_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync12",       'w',  DIOD_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync13",       'w', DIOD_A1, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync14",       'w', DIOD_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync15",       'w', DIOD_A3, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync16",       'w',   HWP_C, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync17",       'w',   HWP_D, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync18",       'w',  HWP_A2, 63,         1.0,             0.0, 'u', U_NONE},
  {"sync19",       'w',  HWP_A1, 63,         1.0,             0.0, 'u', U_NONE},

  //phase of AC DAC signals (NB: overflowed from digital to analog daughter)
  {"ph_cnx_x2_hk",   'w', RTD_A1,  0,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_x2_hk",   'w', RTD_A1,  1,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_x1_hk",   'w', RTD_A1,  4,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_x1_hk",   'w', RTD_A1,  5,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_x3_hk",   'w', RTD_A1,  8,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_x3_hk",   'w', RTD_A1,  9,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_x4_hk",   'w', RTD_A1, 12,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_x4_hk",   'w', RTD_A1, 13,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_x6_hk",   'w', RTD_A1, 16,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_x6_hk",   'w', RTD_A1, 17,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_cnx_x5_hk",   'w', RTD_A1, 20,         I2DEG,          0.0,'u',U_PH_DEG},
  {"ph_ntd_x5_hk",   'w', RTD_A1, 21,         I2DEG,          0.0,'u',U_PH_DEG},

  //commanded bias. use f_bias_hk to see what this really corresponds to
  {"f_bias_cmd_hk", 'w', RTD_C,   0,            1.0,          0.0, 'u', U_F_HZ},

  {"v_cnx_x2_hk",    'w', RTD_D,  0, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_x2_hk",    'w', RTD_D,  1, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ring_x2_hk",'w', RTD_D,  2, CALDAC( 0.9970,     -0.0013), 'u',  U_V_V},
  {"heat_fplo_x2_hk",'w', RTD_D,  3, CALDAC( 0.9973,     -0.0010), 'u',  U_V_V},
  {"v_cnx_x1_hk",    'w', RTD_D,  4, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_x1_hk",    'w', RTD_D,  5, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ring_x1_hk",'w', RTD_D,  6, CALDAC( 0.9974,     -0.0015), 'u',  U_V_V},
  {"heat_fplo_x1_hk",'w', RTD_D,  7, CALDAC( 0.9989,     -0.0015), 'u',  U_V_V},
  {"v_cnx_x3_hk",    'w', RTD_D,  8, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_x3_hk",    'w', RTD_D,  9, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ring_x3_hk",'w', RTD_D, 10, CALDAC( 1.0006,     -0.0013), 'u',  U_V_V},
  {"heat_fplo_x3_hk",'w', RTD_D, 11, CALDAC( 1.0012,     -0.0012), 'u',  U_V_V},
  {"v_cnx_x4_hk",    'w', RTD_D, 12, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_x4_hk",    'w', RTD_D, 13, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ring_x4_hk",'w', RTD_D, 14, CALDAC( 1.0035,     -0.0010), 'u',  U_V_V},
  {"heat_fplo_x4_hk",'w', RTD_D, 15, CALDAC( 0.9983,     -0.0013), 'u',  U_V_V},
  {"v_cnx_x6_hk",    'w', RTD_D, 16, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_x6_hk",    'w', RTD_D, 17, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ring_x6_hk",'w', RTD_D, 18, CALDAC( 0.9981,     -0.0015), 'u',  U_V_V},
  {"heat_fplo_x6_hk",'w', RTD_D, 19, CALDAC( 1.0040,     -0.0015), 'u',  U_V_V},
  {"v_cnx_x5_hk",    'w', RTD_D, 20, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_x5_hk",    'w', RTD_D, 21, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ring_x5_hk",'w', RTD_D, 22, CALDAC( 0.9997,     -0.0012), 'u',  U_V_V},
  {"heat_fplo_x5_hk",'w', RTD_D, 23, CALDAC( 0.9988,     -0.0014), 'u',  U_V_V},

  {"ifpwr",         'w', HWP_D,  50,         1.0,           0.0, 'u', U_NONE},
  {"ctl_sftv",      'w', HWP_D,  51,         1.0,           0.0, 'u', U_NONE},
  {"lim_sftv",      'r', HWP_D,  51,         1.0,           0.0, 'u', U_NONE},
  {"heat_if",       'w', HWP_D,  52,         1.0,           0.0, 'u', U_NONE},
  {"hwp_bias",      'w', HWP_D,  54,         1.0,           0.0, 'u', U_NONE},
  //with quadrature code, phase_hwp is deprecated
  {"phase_hwp",     'w', HWP_A1,  0,       I2DEG,           0.0, 'u', U_PH_DEG},

  // TODO: Give these real names.
  {"vt_mt_tavco",  'r', HWP_A2, 37, CAL16B(    1.0,           0.0), 'u', U_V_V},
  {"vt_sftv",      'r', HWP_A2, 39, CAL16B(    1.0,           0.0), 'u', U_V_V},
  {"vt_3_if",      'r', HWP_A2, 41, CAL16B(    1.0,           0.0), 'u', U_V_V},
  {"vt_4_if",      'r', HWP_A2, 43, CAL16B(    1.0,           0.0), 'u', U_V_V},
  {"vt_5_if",      'r', HWP_A2, 45, CAL16B(    1.0,           0.0), 'u', U_V_V},
  {"vt_6_if",      'r', HWP_A2, 47, CAL16B(    1.0,           0.0), 'u', U_V_V},
  {"vt_7_if",      'r', HWP_A2, 49, CAL16B(    1.0,           0.0), 'u', U_V_V},

  /* default current cal is 12.5, 0.0 */
  {"i_mce",        'r',  DIOD_A1,   49, CAL16(12.5, 0.0),      'u', U_I_A},
  {"i_hk_misc",    'r',  RTD_A2,    49, CAL16(12.5, 0.0),      'u', U_I_A},

  /* spare IF analog inputs */
  /* all of  LOOP0, and LOOP1 0-17 are wide */
  {"g_com_el",         'w', LOOP1, 18,      3.0/65536.0,  0.0, 'u',     U_NONE},
  {"channelset_oth",   'w', LOOP1, 19,                1,  0.0, 'u',     U_NONE},
  {"cal_xmax_mag",     'w', LOOP1, 20,       1,           0.0, 's',     U_NONE},
  {"cal_xmin_mag",     'w', LOOP1, 21,       1,           0.0, 's',     U_NONE},
  {"cal_ymax_mag",     'w', LOOP1, 22,       1,           0.0, 's',     U_NONE},
  {"cal_ymin_mag",     'w', LOOP1, 23,       1,           0.0, 's',     U_NONE},
  {"foc_res_g",        'w', LOOP1, 24,              1.0,  0.0, 'u',     U_NONE},
  /* status_eth =  star camera net status */
  {"status_eth",       'w', LOOP1, 25,              1.0,  0.0, 'u',     U_NONE},
  {"timeout_b",        'w', LOOP1, 26,              1.0,  0.0, 'u',     U_NONE},
  {"az_sun",           'w', LOOP1, 27,            I2DEG,  0.0, 'u',    U_D_DEG},
  /* status_flc = {bitsy_i_am, at_float, schedule, slot_sched} */
  {"status_flc",       'w', LOOP1, 28,              1.0,  0.0, 'u',     U_NONE},
  {"upslot_sched",     'w', LOOP1, 29,              1.0,  0.0, 'u',     U_NONE},
  {"t_chip_flc",       'w', LOOP1, 30,             0.01,  0.0, 'u',      U_T_C},
  /* declination_mag = magnetic declination */
  {"declination_mag",  'w', LOOP1, 31,            I2DEG,  0.0, 'u',    U_D_DEG},
  {"veto_sensor",      'w', LOOP1, 32,              1.0,  0.0, 'u',     U_NONE},
  {"timeout_i",        'w', LOOP1, 33,              1.0,  0.0, 'u',     U_NONE},
  {"df_b_flc",         'w', LOOP1, 34,           1./250,  0.0, 'u',     U_NONE},
  {"alt_sip",          'w', LOOP1, 35,              1.0,  0.0, 'u',     U_NONE},
  {"mapmean_g",        'w', LOOP1, 36,              1.0,  0.0, 'u',     U_NONE},
  {"lat_sip",          'w', LOOP1, 37,            I2DEG,  0.0, 's',   U_LA_DEG},
  {"lon_sip",          'w', LOOP1, 38,            I2DEG,  0.0, 's',   U_LO_DEG},
  {"lat_dgps",         'w', LOOP1, 39,            I2DEG,  0.0, 's',   U_LA_DEG},
  {"lon_dgps",         'w', LOOP1, 40,            I2DEG,  0.0, 's',   U_LO_DEG},
  {"alt_dgps",         'w', LOOP1, 41,              1.0,  0.0, 'u',    U_ALT_M},
  {"speed_dgps",       'w', LOOP1, 42,           1./100,  0.0, 's',    U_V_KPH},
  {"dir_dgps",         'w', LOOP1, 43,            I2DEG,  0.0, 'u',    U_D_DEG},
  {"climb_dgps",       'w', LOOP1, 44,           1./100,  0.0, 's',    U_V_MPS},
  {"att_ok",           'w', LOOP1, 45,              1.0,  0.0, 'u',     U_NONE},
  {"g_p_table",        'w', LOOP1, 46,       1.0/1000.0,  0.0, 'u',     U_NONE},
  {"g_i_table",        'w', LOOP1, 47,      1.0/10000.0,  0.0, 'u',     U_NONE},
  {"g_d_table",        'w', LOOP1, 48,        1.0/100.0,  0.0, 'u',     U_NONE},
  {"n_sat_dgps",       'w', LOOP1, 49,              1.0,  0.0, 'u',     U_NONE},
  {"df_i_flc",         'w', LOOP1, 50,           1./250,  0.0, 'u',     U_NONE},
  {"mode_p",           'w', LOOP1, 51,                1,  0.0, 'u',     U_NONE},
  {"x_p",              'w', LOOP1, 52,            I2DEG,  0.0, 'u',     U_NONE},
  {"y_p",              'w', LOOP1, 53,            I2DEG,  0.0, 's',     U_NONE},
  {"vel_az_p",         'w', LOOP1, 54,            I2VEL,  0.0, 's',     U_NONE},
  {"del_p",            'w', LOOP1, 55,    (2.0/65536.0), -1.0, 'u',     U_NONE},
  {"nblobs_g",         'w', LOOP1, 56,              1.0,  0.0, 'u',     U_NONE},
  // TODO: add appropriate derived types for these
  {"blob00_x_g",       'w', LOOP1, 57, CAM_WIDTH/SHRT_MAX,0.0, 'u',     U_NONE},
  {"blob00_y_g",       'w', LOOP1, 58, CAM_WIDTH/SHRT_MAX,0.0, 'u',     U_NONE},
  {"blob00_f_g",       'w', LOOP1, 59,              1.0,  0.0, 'u',     U_NONE},
  {"blob00_s_g",       'w', LOOP1, 60,        1.0/100.0,  0.0, 'u',     U_NONE},
  {"blob01_x_g",       'w', LOOP1, 61, CAM_WIDTH/SHRT_MAX,0.0, 'u',     U_NONE},
  {"blob01_y_g",       'w', LOOP1, 62, CAM_WIDTH/SHRT_MAX,0.0, 'u',     U_NONE},
  {"blob01_f_g",       'w', LOOP1, 63,              1.0,  0.0, 'u',     U_NONE},

  {"blob01_s_g",       'w', LOOP2,  0,        1.0/100.0,  0.0, 'u',     U_NONE},
  {"blob02_x_g",       'w', LOOP2,  1, CAM_WIDTH/SHRT_MAX,0.0, 'u',     U_NONE},
  {"blob02_y_g",       'w', LOOP2,  2, CAM_WIDTH/SHRT_MAX,0.0, 'u',     U_NONE},
  {"blob02_f_g",       'w', LOOP2,  3,              1.0,  0.0, 'u',     U_NONE},
  {"blob02_s_g",       'w', LOOP2,  4,        1.0/100.0,  0.0, 'u',     U_NONE},
  /* w_p = pointing scan width */
  {"w_p",              'w', LOOP2,  5,            I2DEG,  0.0, 'u',     U_NONE},
  {"move_tol_g",       'w', LOOP2,  6,              1.0,  0.0, 'u',     U_NONE},
  {"exp_int_g",        'w', LOOP2,  7,              1.0,  0.0, 'u',     U_NONE},
  {"exp_time_g",       'w', LOOP2,  8,              1.0,  0.0, 'u',     U_NONE},
  {"force_g",          'w', LOOP2,  9,              1.0,  0.0, 'u',     U_NONE},
  {"blob_idx_g",       'w', LOOP2, 10,              1.0,  0.0, 'u',     U_NONE},
  {"blob_idx_b",       'w', LOOP2, 11,              1.0,  0.0, 'u',     U_NONE},
  {"blob_idx_u",       'w', LOOP2, 12,              1.0,  0.0, 'u',     U_NONE},
  {"thresh_g",         'w', LOOP2, 13,       1.0/1000.0,  0.0, 'u',     U_NONE},
  {"grid_g",           'w', LOOP2, 14,              1.0,  0.0, 'u',     U_NONE},
  {"cal_off_pss1",     'w', LOOP2, 15,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"cal_off_pss2",     'w', LOOP2, 16,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"mdist_g",          'w', LOOP2, 17,              1.0,  0.0, 'u',     U_NONE},
  {"cal_off_pss3",     'w', LOOP2, 18,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"cal_off_pss4",     'w', LOOP2, 19,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"cal_off_pss5",     'w', LOOP2, 20,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"accel_max_az",     'w', LOOP2, 21,    100.0/65535.0,  0.0, 'u',     U_NONE},
  {"cal_off_pss6",     'w', LOOP2, 22,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"cal_d_pss1",       'w', LOOP2, 23,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"sigma_pss",        'w', LOOP2, 24,            I2DEG,  0.0, 'u',     U_NONE},
  {"cal_d_pss2",       'w', LOOP2, 25,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"offset_ofpch_gy",  'w', LOOP2, 26,      1.0/32768.0,  0.0, 's',    U_V_DPS},
  {"offset_ofroll_gy", 'w', LOOP2, 27,      1.0/32768.0,  0.0, 's',    U_V_DPS},
  {"offset_ofyaw_gy",  'w', LOOP2, 28,      1.0/32768.0,  0.0, 's',    U_V_DPS},
  {"az_raw_mag",       'w', LOOP2, 29,            I2DEG,  0.0, 'u',    U_D_DEG},
  {"sigma_mag",        'w', LOOP2, 30,            I2DEG,  0.0, 'u',     U_NONE},
  {"az_dgps",          'w', LOOP2, 31,            I2DEG,  0.0, 'u',    U_D_DEG},
  {"sigma_dgps",       'w', LOOP2, 32,            I2DEG,  0.0, 'u',     U_NONE},
  {"n_scan_per_step",  'w', LOOP2, 33,              1.0,  0.0, 'u',     U_NONE},
  {"cal_d_pss3",       'w', LOOP2, 34,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"cal_d_pss4",       'w', LOOP2, 35,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"mapsigma_g",       'w', LOOP2, 36,         1.0/10.0,  0.0, 'u',     U_NONE},
  {"size_el_step",     'w', LOOP2, 37,      2.0/65536.0, -1.0, 'u',    U_P_DEG},
  {"n_el_steps",       'w', LOOP2, 38,    500.0/65536.0,  0.0, 'u',     U_NONE},
  {"foc_rng_g",        'w', LOOP2, 39,              1.0,  0.0, 'u',     U_NONE},
  {"cal_d_pss5",       'w', LOOP2, 40,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"bbc_sync_auto",    'w', LOOP2, 41,              1.0,  0.0, 'u',     U_NONE},
  {"az_mag",           'w', LOOP2, 42,            I2DEG,  0.0, 'u',    U_D_DEG},
  {"cal_d_pss6",       'w', LOOP2, 43,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"cal_imin_pss",     'w', LOOP2, 44,     40.0/65536.0,  0.0, 's', U_TRIM_DEG},
  {"mce_power",        'w', LOOP2, 45,              1.0,  0.0, 'u',     U_NONE},
  {"del_relmove_port", 'w', LOOP2, 46,      1.0/32768.0, -1.0, 'u',    U_P_DEG},
  {"del_relmove_stbd", 'w', LOOP2, 47,      1.0/32768.0, -1.0, 'u',    U_P_DEG},
  {"v_relmove_port",   'w', LOOP2, 48,      0.2/65536.0,  0.0, 'u',    U_V_DPS},
  {"v_relmove_stbd",   'w', LOOP2, 49,      0.2/65536.0,  0.0, 'u',    U_V_DPS},
  {"foc_rng_b",        'w', LOOP2, 50,              1.0,  0.0, 'u',     U_NONE},
  {"foc_rng_u",        'w', LOOP2, 51,              1.0,  0.0, 'u',     U_NONE},
  {"t_cpu_i_flc",      'w', LOOP2, 52,             0.01,  0.0, 'u',      U_T_C},
  {"bbc_fifo_size",    'w', LOOP2, 53,           1./624,  0.0, 'u',     U_NONE},
  {"t_cpu_b_flc",      'w', LOOP2, 54,             0.01,  0.0, 'u',      U_T_C},
  {"t_mb_flc",         'w', LOOP2, 55,             0.01,  0.0, 'u',      U_T_C},
  {"mks_hi_sip",       'w', LOOP2, 56,         0.003256,-0.226858, 'u', U_NONE},
  {"mks_med_sip",      'w', LOOP2, 57,         0.032614,-0.072580, 'u', U_NONE},
  {"nblobs_b",         'w', LOOP2, 58,              1.0,  0.0, 'u',     U_NONE},
  {"blob00_x_b",       'w', LOOP2, 59,CAM_WIDTH/SHRT_MAX, 0.0, 'u',     U_NONE},
  {"blob00_y_b",       'w', LOOP2, 60,CAM_WIDTH/SHRT_MAX, 0.0, 'u',     U_NONE},
  {"blob00_f_b",       'w', LOOP2, 61,              1.0,  0.0, 'u',     U_NONE},
  {"blob00_s_b",       'w', LOOP2, 62,        1.0/100.0,  0.0, 'u',     U_NONE},
  {"blob01_x_b",       'w', LOOP2, 63,CAM_WIDTH/SHRT_MAX, 0.0, 'u',     U_NONE},

  {"blob01_y_b",       'w', LOOP3,  0,CAM_WIDTH/SHRT_MAX, 0.0, 'u',     U_NONE},
  {"blob01_f_b",       'w', LOOP3,  1,              1.0,  0.0, 'u',     U_NONE},
  {"blob01_s_b",       'w', LOOP3,  2,        1.0/100.0,  0.0, 'u',     U_NONE},
  {"blob02_x_b",       'w', LOOP3,  3,CAM_WIDTH/SHRT_MAX, 0.0, 'u',     U_NONE},
  {"blob02_y_b",       'w', LOOP3,  4,CAM_WIDTH/SHRT_MAX, 0.0, 'u',     U_NONE},
  {"blob02_f_b",       'w', LOOP3,  5,              1.0,  0.0, 'u',     U_NONE},
  {"blob02_s_b",       'w', LOOP3,  6,        1.0/100.0,  0.0, 'u',     U_NONE},
  {"mapmean_b",        'w', LOOP3,  7,              1.0,  0.0, 'u',     U_NONE},
  {"ra_g",             'w', LOOP3,  8,              I2H,  0.0, 'u',     U_P_HR},
  {"dec_g",            'w', LOOP3,  9,            I2DEG,  0.0, 's',    U_P_DEG},
  {"roll_g",           'w', LOOP3, 10,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"ra_b",             'w', LOOP3, 11,              I2H,  0.0, 'u',     U_P_HR},
  {"foc_res_b",        'w', LOOP3, 12,              1.0,  0.0, 'u',     U_NONE},
  {"dec_b",            'w', LOOP3, 13,            I2DEG,  0.0, 's',    U_P_DEG},
  {"roll_b",           'w', LOOP3, 14,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"mapsigma_b",       'w', LOOP3, 15,         1.0/10.0,  0.0, 'u',     U_NONE},
  {"move_tol_b",       'w', LOOP3, 16,              1.0,  0.0, 'u',     U_NONE},
  {"exp_int_b",        'w', LOOP3, 17,              1.0,  0.0, 'u',     U_NONE},
  {"exp_time_b",       'w', LOOP3, 18,              1.0,  0.0, 'u',     U_NONE},
  {"force_b",          'w', LOOP3, 19,              1.0,  0.0, 'u',     U_NONE},
  {"ra_u",             'w', LOOP3, 20,              I2H,  0.0, 'u',     U_P_HR},
  {"dec_u",            'w', LOOP3, 21,            I2DEG,  0.0, 's',    U_P_DEG},
  {"roll_u",           'w', LOOP3, 22,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"trig_delay",       'w', LOOP3, 23,       1.0/1000.0,  0.0, 'u',     U_NONE},
  {"thresh_b",         'w', LOOP3, 24,       1.0/1000.0,  0.0, 'u',     U_NONE},
  {"grid_b",           'w', LOOP3, 25,              1.0,  0.0, 'u',     U_NONE},
  {"mdist_b",          'w', LOOP3, 26,              1.0,  0.0, 'u',     U_NONE},
  {"az_g",             'w', LOOP3, 27,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"el_g",             'w', LOOP3, 28,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"az_b",             'w', LOOP3, 29,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"el_b",             'w', LOOP3, 30,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"az_u",             'w', LOOP3, 31,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"el_u",             'w', LOOP3, 32,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"focpos_g",         'w', LOOP3, 33,         1.0/10.0,  0.0, 's',     U_NONE},
  {"focpos_b",         'w', LOOP3, 34,         1.0/10.0,  0.0, 's',     U_NONE},
  {"focpos_u",         'w', LOOP3, 35,         1.0/10.0,  0.0, 's',     U_NONE},
  {"maxblob_g",        'w', LOOP3, 36,              1.0,  0.0, 'u',     U_NONE},
  {"maxblob_b",        'w', LOOP3, 37,              1.0,  0.0, 'u',     U_NONE},
  {"bi0_fifo_size",    'w', LOOP3, 38,           1./624,  0.0, 'u',     U_NONE},
  {"plover",           'w', LOOP3, 39,              1.0,  0.0, 'u',     U_NONE},
  {"t_ccd_g",          'w', LOOP3, 40,        1.0/100.0,  0.0, 's',      U_T_C},
  {"t_ccd_b",          'w', LOOP3, 41,        1.0/100.0,  0.0, 's',      U_T_C},
  {"t_set_pivot",      'w', LOOP3, 42,  (100.0/32768.0),  0.0, 's',      U_T_C},
  {"t_set_elmot_p",    'w', LOOP3, 43,  (100.0/32768.0),  0.0, 's',      U_T_C},
  {"t_set_mttavco",    'w', LOOP3, 44,  (100.0/32768.0),  0.0, 's',      U_T_C},
  {"t_set_gybox",      'w', LOOP3, 45,  (100.0/32768.0),  0.0, 's',      U_T_C},
  {"trim_pss",         'w', LOOP3, 46,            I2DEG,  0.0, 's',     U_NONE},
  {"az_pss",           'w', LOOP3, 47,            I2DEG,  0.0, 'u',    U_P_DEG},

  /* pointing mode coordinates */
  {"ra_1_p",           'w', LOOP3, 48,              I2H,  0.0, 'u',     U_NONE},
  {"dec_1_p",          'w', LOOP3, 49,            I2DEG,  0.0, 's',     U_NONE},
  {"ra_2_p",           'w', LOOP3, 50,              I2H,  0.0, 'u',     U_NONE},
  {"dec_2_p",          'w', LOOP3, 51,            I2DEG,  0.0, 's',     U_NONE},
  {"ra_3_p",           'w', LOOP3, 52,              I2H,  0.0, 'u',     U_NONE},
  {"dec_3_p",          'w', LOOP3, 53,            I2DEG,  0.0, 's',     U_NONE},
  {"ra_4_p",           'w', LOOP3, 54,              I2H,  0.0, 'u',     U_NONE},
  {"dec_4_p",          'w', LOOP3, 55,            I2DEG,  0.0, 's',     U_NONE},
  {"trim_null",        'w', LOOP3, 56,            I2DEG,  0.0, 's',     U_NONE},
  {"trim_mag",         'w', LOOP3, 57,            I2DEG,  0.0, 's',     U_NONE},
  {"trim_dgps",        'w', LOOP3, 58,            I2DEG,  0.0, 's',     U_NONE},
  {"az_raw_dgps",      'w', LOOP3, 59,            I2DEG,  0.0, 'u',    U_D_DEG},
  /* h_p = scan height */
  {"h_p",              'w', LOOP3, 60,            I2DEG,  0.0, 'u',     U_NONE},
  {"mks_lo_sip",       'w', LOOP3, 61,         0.327045,-5.944902, 'u', U_NONE},
  {"alt",              'w', LOOP3, 62,              1.0,  0.0, 'u',     U_NONE},
  {"mode_az_mc",       'w', LOOP3, 63,              1.0,  0.0, 'u',     U_NONE},

  {"mode_el_mc",       'w', LOOP4,  0,              1.0,  0.0, 'u',     U_NONE},
  {"dest_az_mc",       'w', LOOP4,  1,            I2DEG,  0.0, 'u',     U_NONE},
  {"dest_el_mc",       'w', LOOP4,  2,            I2DEG,  0.0, 'u',     U_NONE},
  {"vel_az_mc",        'w', LOOP4,  3,          1./6000,  0.0, 's',     U_NONE},
  {"vel_el_mc",        'w', LOOP4,  4,          1./6000,  0.0, 's',     U_NONE},
  {"dir_az_mc",        'w', LOOP4,  5,              1.0,  0.0, 's',     U_NONE},
  {"dir_el_mc",        'w', LOOP4,  6,              1.0,  0.0, 's',     U_NONE},
  {"slew_veto",        'w', LOOP4,  7,   1200.0/65535.0,  0.0, 'u',     U_NONE},
  {"sveto_len",        'w', LOOP4,  8,   1200.0/65535.0,  0.0, 'u',     U_NONE},
  {"dith_el",          'w', LOOP4,  9,      0.5/32768.0,  0.0, 's',    U_D_DEG},
  {"state_lock",       'w', LOOP4, 10,              1.0,  0.0, 'u',     U_NONE},
  {"x_mag",            'w', LOOP4, 11,              1.0,  0.0, 's',     U_NONE},
  {"y_mag",            'w', LOOP4, 12,              1.0,  0.0, 's',     U_NONE},
  {"z_mag",            'w', LOOP4, 13,              1.0,  0.0, 's',     U_NONE},
  {"pitch_mag",        'w', LOOP4, 14,            I2DEG,  0.0, 'u',     U_NONE},
  {"el_sun",           'w', LOOP4, 15,            I2DEG,  0.0, 's',     U_NONE},
  {"vel_ser_rw",       'w', LOOP4, 16,   2400.0/65536.0,  0.0, 's',    U_V_DPS},
  {"res_rw",           'w', LOOP4, 17,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"stat_dr_rw",       'w', LOOP4, 18,              1.0,  0.0, 'u',     U_NONE},
  {"stat_s1_rw",       'w', LOOP4, 19,              1.0,  0.0, 'u',     U_NONE},
  {"stat_dr_piv",      'w', LOOP4, 20,              1.0,  0.0, 'u',     U_NONE},
  {"stat_s1_piv",      'w', LOOP4, 21,              1.0,  0.0, 'u',     U_NONE},
  {"i_ser_rw",         'w', LOOP4, 22,     60.0/32768.0,  0.0, 's',      U_I_A},
  {"res_piv",          'w', LOOP4, 23,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"i_ser_piv",        'w', LOOP4, 24,     20.0/32768.0,  0.0, 's',      U_I_A},
  {"g_v_rw_piv",       'w', LOOP4, 25,              1.0,  0.0, 'u',     U_NONE},
  {"g_pe_piv",         'w', LOOP4, 26,              1.0,  0.0, 'u',     U_NONE},
  {"g_pv_piv",         'w', LOOP4, 27,              1.0,  0.0, 'u',     U_NONE},
  {"g_t_rw_piv",       'w', LOOP4, 28,              1.0,  0.0, 'u',     U_NONE},
  {"set_rw",           'w', LOOP4, 29,    500.0/32768.0,  0.0, 's',    U_V_DPS},
  {"vel_dps_az",       'w', LOOP4, 30,     20.0/32768.0,  0.0, 's',    U_V_DPS},
  {"frict_off_piv",    'w', LOOP4, 31,      2.0/65535.0,  0.0, 'u',     U_NONE},
  {"term_p_t_rw_piv",  'w', LOOP4, 32,              1.0,-32768.0, 'u',  U_NONE},
  {"term_p_v_req_az_piv",'w',LOOP4,33,              1.0,-32768.0, 'u',  U_NONE},
  {"vel_ser_piv",      'w', LOOP4, 34,   2000.0/65536.0,  0.0, 's',    U_V_DPS},
  {"g_v_req_az_piv",   'w', LOOP4, 35,              1.0,  0.0, 'u',     U_NONE},
  {"az_raw_pss1",      'w', LOOP4, 36,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"az_raw_pss2",      'w', LOOP4, 37,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"drive_info_rw",    'w', LOOP4, 38,              1.0,  0.0, 'u',     U_NONE},
  {"drive_err_cts_rw", 'w', LOOP4, 39,              1.0,  0.0, 'u',     U_NONE},
  {"drive_info_piv",   'w', LOOP4, 40,              1.0,  0.0, 'u',     U_NONE},
  {"drive_err_cts_piv",'w', LOOP4, 41,              1.0,  0.0, 'u',     U_NONE},
  /* LOOP7 38 is fast narrow */
  {"vel_hwp",          'w', LOOP4, 42,     10.0/65535.0,  0.0, 'u',    U_V_DPS},
  {"i_move_hwp",       'w', LOOP4, 43,      1.2/65535.0,  0.0, 'u',      U_I_A},
  {"status_hwp",       'w', LOOP4, 44,              1.0,  0.0, 'u',     U_NONE},
  {"pos_x1_hwp",       'w', LOOP4, 45,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"pos_x2_hwp",       'w', LOOP4, 46,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"pos_x3_hwp",       'w', LOOP4, 47,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"pitch_raw_dgps",   'w', LOOP4, 48,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"roll_raw_dgps",    'w', LOOP4, 49,            I2DEG,  0.0, 'u',    U_P_DEG},
  {"verbose_rw",       'w', LOOP4, 50,              1.0,  0.0, 'u',     U_NONE},
  {"verbose_piv",      'w', LOOP4, 51,              1.0,  0.0, 'u',     U_NONE},
  {"term_p_v_rw_piv",  'w', LOOP4, 52,              1.0,-32768.0, 'u',  U_NONE},
  {"term_frict_piv",   'w', LOOP4, 53,      2.0/32767.0,  0.0, 's',     U_NONE},
  {"term_p_rw_piv",    'w', LOOP4, 54,              1.0,  0.0, 's',     U_NONE},
  {"term_p_err_piv",   'w', LOOP4, 55,              1.0,  0.0, 's',     U_NONE},
  {"mode_piv",         'w', LOOP4, 56,              1.0,  0.0, 'u',     U_NONE},
  {"mode_el",          'w', LOOP4, 57,              1.0,  0.0, 'u',     U_NONE},

  /* charge controller related channels */
  {"v_batt_of_cc",  'w',  LOOP4,  58,  1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"v_arr_of_cc",   'w',  LOOP4,  59,  1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"i_batt_of_cc",  'w',  LOOP4,  60,  1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  {"i_arr_of_cc",   'w',  LOOP4,  61,  1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  { "t_hs_of_cc",   'w',  LOOP4,  62,  1.0,      0.0,            's',  U_T_C},
  // Fault bitfield
  {"fault_of_cc",   'w',  LOOP4,  63,  1.0,      0.0,            'u',  U_NONE},

  // alarm high bitfield
  {"alarm_hi_of_cc",'w',  LOOP5,   0,  1.0,      0.0,            'u',  U_NONE},
  // alarm low bitfield
  {"alarm_lo_of_cc",'w',  LOOP5,   1,  1.0,      0.0,            'u',  U_NONE},

  {"v_targ_of_cc",  'w',  LOOP5,   2,  1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"state_of_cc",   'w',  LOOP5,   3,  1.0,      0.0,            'u',  U_NONE},
  // charge controller LED state
  {"led_of_cc",     'w',  LOOP5,   4,       1.0,             0.0, 'u', U_NONE},
  {"v_batt_if_cc",  'w',  LOOP5,   5,   1/180.0,  -32400.0/180.0, 'u', U_V_V},
  {"v_arr_if_cc",   'w',  LOOP5,   6,   1/180.0,  -32400.0/180.0, 'u', U_V_V},
  {"i_batt_if_cc",  'w',  LOOP5,   7,   1/400.0,  -32000.0/400.0, 'u', U_I_A},
  {"i_arr_if_cc",   'w',  LOOP5,   8,   1/400.0,  -32000.0/400.0, 'u', U_I_A},
  { "t_hs_if_cc",   'w',  LOOP5,   9,       1.0,             0.0, 's', U_T_C},
  // fault bitfield
  {"fault_if_cc",   'w',  LOOP5,  10,       1.0,             0.0, 'u', U_NONE},
  // alarm high bitfield
  {"alarm_hi_if_cc",'w',  LOOP5,  11,       1.0,             0.0, 'u', U_NONE},
  // alarm low bitfield
  {"alarm_lo_if_cc",'w',  LOOP5,  12,       1.0,             0.0, 'u', U_NONE},
  {"v_targ_if_cc",  'w',  LOOP5,  13,   1/180.0,  -32400.0/180.0, 'u', U_V_V},
  {"state_if_cc",   'w',  LOOP5,  14,       1.0,             0.0, 'u', U_NONE},
  // charge controller LED state
  {"led_if_cc",     'w',  LOOP5,  15,       1.0,             0.0, 'u', U_NONE},

  {"azraw_pss",     'w', LOOP5, 16,         I2DEG,           0.0, 'u', U_P_DEG},
  {"elraw_pss",     'w', LOOP5, 17,         I2DEG,           0.0, 'u', U_P_DEG},
  {"snr_pss1",      'w', LOOP5, 18,       1/1000.,           0.0, 'u', U_NONE},
  {"snr_pss2",      'w', LOOP5, 19,       1/1000.,           0.0, 'u', U_NONE},
  {"snr_pss3",      'w', LOOP5, 20,       1/1000.,           0.0, 'u', U_NONE},
  {"snr_pss4",      'w', LOOP5, 21,       1/1000.,           0.0, 'u', U_NONE},
  {"snr_pss5",      'w', LOOP5, 22,       1/1000.,           0.0, 'u', U_NONE},
  {"snr_pss6",      'w', LOOP5, 23,       1/1000.,           0.0, 'u', U_NONE},
  {"accel_az",      'w', LOOP5, 24,     2.0/65536,           0.0, 'u', U_NONE},
  {"az_raw_pss3",   'w', LOOP5, 25,         I2DEG,           0.0, 'u', U_P_DEG},
  {"az_raw_pss4",   'w', LOOP5, 26,         I2DEG,           0.0, 'u', U_P_DEG},
  {"az_cov_dgps",   'w', LOOP5, 27,         I2DEG,           0.0, 'u', U_NONE},
  {"pitch_cov_dgps",'w', LOOP5, 28,         I2DEG,           0.0, 'u', U_NONE},
  {"roll_cov_dgps", 'w', LOOP5, 29,         I2DEG,           0.0, 'u', U_NONE},
  {"force_u",       'w', LOOP5, 30,           1.0,           0.0, 'u', U_NONE},
  {"exp_int_u",     'w', LOOP5, 31,           1.0,           0.0, 'u', U_NONE},
  {"exp_time_u",    'w', LOOP5, 32,           1.0,           0.0, 'u', U_NONE},
  {"foc_res_u",     'w', LOOP5, 33,           1.0,           0.0, 'u', U_NONE},
  {"move_tol_u",    'w', LOOP5, 34,           1.0,           0.0, 'u', U_NONE},
  {"maxblob_u",     'w', LOOP5, 35,           1.0,           0.0, 'u', U_NONE},
  {"grid_u",        'w', LOOP5, 36,           1.0,           0.0, 'u', U_NONE},
  {"thresh_u",      'w', LOOP5, 37,    1.0/1000.0,           0.0, 'u', U_NONE},
  {"mdist_u",       'w', LOOP5, 38,           1.0,           0.0, 'u', U_NONE},
  {"mapmean_u",     'w', LOOP5, 39,           1.0,           0.0, 'u', U_NONE},
  {"mapsigma_u",    'w', LOOP5, 40,      1.0/10.0,           0.0, 'u', U_NONE},
  {"t_ccd_u",       'w', LOOP5, 41,     1.0/100.0,           0.0, 's', U_T_C},
  {"nblobs_u",      'w', LOOP5, 42,           1.0,           0.0, 'u', U_NONE},
  {"blob00_x_u",    'w', LOOP5, 43, CAM_WIDTH/SHRT_MAX,      0.0, 'u', U_NONE},
  {"blob00_y_u",    'w', LOOP5, 44, CAM_WIDTH/SHRT_MAX,      0.0, 'u', U_NONE},
  {"blob00_f_u",    'w', LOOP5, 45,           1.0,           0.0, 'u', U_NONE},
  {"blob00_s_u",    'w', LOOP5, 46,     1.0/100.0,           0.0, 'u', U_NONE},
  {"blob01_x_u",    'w', LOOP5, 47, CAM_WIDTH/SHRT_MAX,      0.0, 'u', U_NONE},
  {"blob01_y_u",    'w', LOOP5, 48, CAM_WIDTH/SHRT_MAX,      0.0, 'u', U_NONE},
  {"blob01_f_u",    'w', LOOP5, 49,           1.0,           0.0, 'u', U_NONE},
  {"blob01_s_u",    'w', LOOP5, 50,     1.0/100.0,           0.0, 'u', U_NONE},
  {"blob02_x_u",    'w', LOOP5, 51, CAM_WIDTH/SHRT_MAX,      0.0, 'u', U_NONE},
  {"blob02_y_u",    'w', LOOP5, 52, CAM_WIDTH/SHRT_MAX,      0.0, 'u', U_NONE},
  {"blob02_f_u",    'w', LOOP5, 53,           1.0,           0.0, 'u', U_NONE},
  {"blob02_s_u",    'w', LOOP5, 54,     1.0/100.0,           0.0, 'u', U_NONE},
  {"g_pt_az",       'w', LOOP5, 55,           1.0,           0.0, 'u', U_NONE},
  {"pos_x4_hwp",    'w', LOOP5, 56,         I2DEG,           0.0, 'u', U_P_DEG},
  {"pos_x5_hwp",    'w', LOOP5, 57,         I2DEG,           0.0, 'u', U_P_DEG},
  {"pos_x6_hwp",    'w', LOOP5, 58,         I2DEG,           0.0, 'u', U_P_DEG},
  {"rate_tdrss",    'w', LOOP5, 59,           1.0,           0.0, 'u', U_RATE},
  {"rate_iridium",  'w', LOOP5, 60,           1.0,           0.0, 'u', U_RATE},
  {"rate_pilot",    'w', LOOP5, 61,           1.0,           0.0, 'u', U_RATE},
  {"az_raw_pss5",   'w', LOOP5, 62,         I2DEG,           0.0, 'u', U_P_DEG},
  {"az_raw_pss6",   'w', LOOP5, 63,         I2DEG,           0.0, 'u', U_P_DEG},

  {"el_raw_pss1",   'w', LOOP6,  0,         I2DEG,           0.0, 'u', U_P_DEG},
  {"el_raw_pss2",   'w', LOOP6,  1,         I2DEG,           0.0, 'u', U_P_DEG},
  {"el_raw_pss3",   'w', LOOP6,  2,         I2DEG,           0.0, 'u', U_P_DEG},
  {"el_raw_pss4",   'w', LOOP6,  3,         I2DEG,           0.0, 'u', U_P_DEG},
  {"el_raw_pss5",   'w', LOOP6,  4,         I2DEG,           0.0, 'u', U_P_DEG},
  {"el_raw_pss6",   'w', LOOP6,  5,         I2DEG,           0.0, 'u', U_P_DEG},
  {"insert_last_hk",'w', LOOP6,  6,           1.0,           0.0, 'u', U_NONE},
  {"f_bias_hk",     'w', LOOP6,  7, 400.0/65535.0,           0.0, 'u', U_F_HZ},
  {"v_heat_last_hk",'w', LOOP6,  8,    CALDAC(1.0,           0.0), 'u', U_V_V},
  //{"pulse_last_hk", 'w', LOOP6,  9,           1.0,           0.0, 'u',U_NONE},
  // sum of currents read through ACS1 A1
  {"bsc_trigger",   'w', LOOP6,  9,           1.0,           0.0, 'u', U_NONE},
  {"i_of_tot",      'w', LOOP6, 10,        1.0e-3,           0.0, 'u', U_I_A},
  {"cov_lim_dgps",  'w', LOOP6, 11,(100.0/32768.0),          0.0, 'u', U_NONE},
  {"ant_e_dgps",    'w', LOOP6, 12,     1.0/100.0,           0.0, 's',U_NONE},
  {"ant_n_dgps",    'w', LOOP6, 13,     1.0/100.0,           0.0, 's',U_NONE},
  {"ant_u_dgps",    'w', LOOP6, 14,     1.0/100.0,           0.0, 's',U_NONE},
  {"ants_lim_dgps", 'w', LOOP6, 15,(100.0/32768.0),          0.0, 'u', U_NONE},
  {"frame_int_bbc", 'w', LOOP6, 16,           1.0,           0.0, 'u', U_NONE},
  {"rate_ext_bbc",  'w', LOOP6, 17, 400.0/65535.0,           0.0, 'u', U_F_HZ},
  {"frame_ext_bbc", 'w', LOOP6, 18,           1.0,           0.0, 'u', U_NONE},
  {"rate_frame_bbc",'w', LOOP6, 19, 400.0/65535.0,           0.0, 'u', U_F_HZ},
  {"rate_samp_adc", 'w', LOOP6, 20,20000.0/65535.0,          0.0, 'u', U_F_HZ},
  {"enc_cnt_x1_hwp",'w', LOOP6, 21,         I2DEG,           0.0, 'u', U_P_DEG},
  {"enc_cnt_x2_hwp",'w', LOOP6, 22,         I2DEG,           0.0, 'u', U_P_DEG},
  {"enc_cnt_x3_hwp",'w', LOOP6, 23,         I2DEG,           0.0, 'u', U_P_DEG},
  {"enc_cnt_x4_hwp",'w', LOOP6, 24,         I2DEG,           0.0, 'u', U_P_DEG},
  {"enc_cnt_x5_hwp",'w', LOOP6, 25,         I2DEG,           0.0, 'u', U_P_DEG},
  {"enc_cnt_x6_hwp",'w', LOOP6, 26,         I2DEG,           0.0, 'u', U_P_DEG},
  {"bset",          'w', LOOP6, 27,             1,             0, 'u', U_NONE},
  {"state_x1_cycle",'w', LOOP6, 28,           1.0,           0.0, 'u', U_NONE},
  {"state_x2_cycle",'w', LOOP6, 29,           1.0,           0.0, 'u', U_NONE},
  {"state_x3_cycle",'w', LOOP6, 30,           1.0,           0.0, 'u', U_NONE},
  {"state_x4_cycle",'w', LOOP6, 31,           1.0,           0.0, 'u', U_NONE},
  {"state_x5_cycle",'w', LOOP6, 32,           1.0,           0.0, 'u', U_NONE},
  {"state_x6_cycle",'w', LOOP6, 33,           1.0,           0.0, 'u', U_NONE},
  {"band_az",       'w', LOOP6, 34,  30.0/65535.0,           0.0, 'u', U_P_DEG},
  
  /* LOOP6, 35 is fast */
  {"row_len_sync",   'w', LOOP6, 36,            1.0,         0.0, 'u', U_NONE},
  {"num_rows_sync",  'w', LOOP6, 37,            1.0,         0.0, 'u', U_NONE},
  {"data_rate_sync", 'w', LOOP6, 38,            1.0,         0.0, 'u', U_NONE},
  {"last_b_cmd",     'w', LOOP6, 39,            1.0,         0.0, 'u', U_NONE},
  {"last_i_cmd",     'w', LOOP6, 40,            1.0,         0.0, 'u', U_NONE},
  {"count_b_cmd",    'w', LOOP6, 41,            1.0,         0.0, 'u', U_NONE},
  {"count_i_cmd",    'w', LOOP6, 42,            1.0,         0.0, 'u', U_NONE},
  {"delay_az",       'w', LOOP6, 43, (10.0/32768.0),         0.0, 'u', U_NONE},
  {"mce_cindex",     'w', LOOP6, 44,            1.0,         0.0, 'u', U_NONE},
  {"state_sftv",     'w', LOOP6, 45,            1.0,         0.0, 'u', U_NONE},

#ifndef BOLOTEST
/* ACS1 Digital I/O card */
  {"latch0",       'w',  ACS1_D,  0,         1.0,             0.0, 'u', U_NONE},
  {"latch1",       'w',  ACS1_D,  1,         1.0,             0.0, 'u', U_NONE},
  {"switch_gy",    'w',  ACS1_D,  2,         1.0,             0.0, 'u', U_NONE},
  //this may be unused because of the new program
  {"switch_grp2",  'w',  ACS1_D,  3,         1.0,             0.0, 'u', U_NONE},
  {"switch_misc",  'w',  ACS1_D,  4,         1.0,             0.0, 'u', U_NONE},
  {"switch_pv",    'w',  ACS1_D,  5,         1.0,             0.0, 'u', U_NONE},
  {"heat_gy",      'w',  ACS1_D,  6,         1.0,             0.0, 'u', U_NONE},

/* ACS1 Analog card 1 */
/* default current cal is 12.5, 0.0 */
  {"i_trans",      'r',  ACS1_A1,  1, CAL16(11.32, -0.0566),      'u', U_I_A},
  {"i_das",        'r',  ACS1_A1,  3, CAL16(11.03, -0.041),      'u', U_I_A},
  {"i_acs",        'r',  ACS1_A1,  5, CAL16(12.5,  0.0),        'u', U_I_A},
  {"i_mcc",        'r',  ACS1_A1,  7, CAL16(10.75, -0.05),      'u', U_I_A},
  {"i_sc",         'r',  ACS1_A1,  9, CAL16(10.89, -0.058),      'u', U_I_A},
  {"i_dgps",       'r',  ACS1_A1, 11, CAL16(11.03, -0.066),      'u', U_I_A},
  {"i_piv",        'r',  ACS1_A1, 13, CAL16(12.5,  -0.03125),      'u', U_I_A},
  {"i_el",         'r',  ACS1_A1, 15, CAL16(12.5, -0.11),       'u', U_I_A},
  {"i_flc",        'r',  ACS1_A1, 17, CAL16(12.5, -0.09),       'u', U_I_A},
  {"i_rw",         'r',  ACS1_A1, 19, CAL16(12.5, -0.0742),       'u', U_I_A},
  {"i_step",       'r',  ACS1_A1, 21, CAL16(11.0, -0.099),       'u', U_I_A},
  //{"i_gy",         'r',  ACS1_A1, 23, CAL16(12.5,  -0.04),        'u', U_I_A},
/* ACS_A1 24-41 are unused. */

/* ACS1 Temperature card */
  {"vt_gy",           'r',  ACS1_T1, 1, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_rsc",          'r',  ACS1_T1, 3, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_mcc_power",    'r',  ACS1_T1, 5, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_bsc",          'r',  ACS1_T1, 7, CAL16T(1.0, 0.0),         'u', U_T_C},
  //{"vt_wd_flc",       'r',  ACS1_T1, 9, CAL16T(1.0, 0.0),         'u', U_T_C}, // free
  {"vt_sc_mot",       'r',  ACS1_T1, 11, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_rw1",          'r',  ACS1_T1, 13, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_rw2",          'r',  ACS1_T1, 15, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_lockport_mc",  'r',  ACS1_T1, 17, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_lockport_mot", 'r',  ACS1_T1, 19, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_lockstar_mc",  'r',  ACS1_T1, 21, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_lockstar_mot", 'r',  ACS1_T1, 23, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_dgps",         'r',  ACS1_T1, 25, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_elport_mot",   'r',  ACS1_T1, 27, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_elstar_mot",   'r',  ACS1_T1, 29, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_elport_mc",    'r',  ACS1_T1, 31, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_ambient1",     'r',  ACS1_T1, 33, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_if1",          'r',  ACS1_T1, 35, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_if2",          'r',  ACS1_T1, 37, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_pss",          'r',  ACS1_T1, 39, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_floor",        'r',  ACS1_T1, 41, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_mylar_sun",    'r',  ACS1_T1, 43, CAL16T(1.0, 0.0),         'u', U_V_V},
  {"vt_mylar_in",     'r',  ACS1_T1, 45, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_piv",          'r',  ACS1_T1, 47, CAL16T(1.0, 0.0),         'u', U_T_C},
  {"vt_serial",       'r',  ACS1_T1, 49, CAL16T(1.0, 0.0),         'u', U_T_C},

/* ACS2 Digital I/O card */
  // ACS2_D 20-22 are unused
  {"dac2_ampl",    'w',  ACS2_D,  1,         1.0,            0.0, 'u', U_NONE},
  {"dac_piv",      'w',  ACS2_D,  2,         1.0,            0.0, 'u', U_NONE},
  {"mask_gy",      'w',  ACS2_D, 13,         1.0,            0.0, 'u', U_NONE},
  {"control_lock", 'w',  ACS2_D, 14,         1.0,            0.0, 'u', U_NONE},
  {"g_p_az",       'w',  ACS2_D, 23,         1.0,            0.0, 'u', U_NONE},
  {"g_i_az",       'w',  ACS2_D, 24,         1.0,            0.0, 'u', U_NONE},
  {"enc1_offset",  'w',  ACS2_D, 25,       I2DEG,            0.0, 'u', U_P_DEG},
  {"enc2_offset",  'w',  ACS2_D, 26,       I2DEG,            0.0, 'u', U_P_DEG},
  {"fault_gy",     'r',  ACS2_D, 15,         1.0,            0.0, 'u', U_NONE},
  {"dac_rw",       'r',  ACS2_D, 24,         1.0,            0.0, 'u', U_NONE},
  {"term_p_az",    'r',  ACS2_D, 25,         1.0,       -32768.0, 'u', U_NONE},
  {"term_i_az",    'r',  ACS2_D, 26,         1.0,       -32768.0, 'u', U_NONE},
  {"error_az",     'r',  ACS2_D, 27, 614.4e-6, 614.4*(-32768.0e-6),'u',U_NONE},
  {"limit_lock",   'r',  ACS2_D, 30,         1.0,            0.0, 'u', U_NONE},

/* ACS2 Analog card */
  {"pitch_piv_clin",'r',ACS2_A1,  1,   0.001343,          -42.28, 'u', U_NONE},
  {"roll_piv_clin",'r', ACS2_A1,  3,   0.001413,         -44.640, 'u', U_NONE},
  {"t_piv_clin",   'r', ACS2_A1,  5, 100.0*10.0/32768.0, -100.0*10.0,'u',U_T_C},
  {"pitch_of_clin",'r', ACS2_A1,  7,    0.00147,         -48.768, 'u', U_NONE},
  {"roll_of_clin", 'r', ACS2_A1,  9,    0.00144,        -44.78, 'u', U_NONE},
  {"t_of_clin",    'r', ACS2_A1, 11, 100.0*10.0/32768.0, -100.0*10.0,'u',U_T_C},
  {"trig_g",       'r', ACS2_A1, 23, CAL16(1.0,          0.0), 'u',  U_V_V},
  {"trig_b",       'r', ACS2_A1, 25, CAL16(1.0,          0.0), 'u',  U_V_V},
  {"trig_u",       'r', ACS2_A1, 27, CAL16(1.0,          0.0), 'u',  U_V_V},
  // not used - free
  //{"t_ofpch_gy", 'r', ACS2_A1, 47, CAL16(50.0, 0.0),   'u',  U_T_C},
  {"trig_s_g", 'r', ACS2_A1, 51,             1.0,             0.0, 'u', U_NONE},
  {"trig_l_g", 'r', ACS2_A1, 52,             1.0,             0.0, 'u', U_NONE},
  {"trig_s_b", 'r', ACS2_A1, 53,             1.0,             0.0, 'u', U_NONE},
  {"trig_l_b", 'r', ACS2_A1, 54,             1.0,             0.0, 'u', U_NONE},
  {"trig_s_u", 'r', ACS2_A1, 55,             1.0,             0.0, 'u', U_NONE},
  {"trig_l_u", 'r', ACS2_A1, 56,             1.0,             0.0, 'u', U_NONE},
  {"v1_1_pss", 'r', ACS2_A2,  1,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_1_pss", 'r', ACS2_A2,  3,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_1_pss", 'r', ACS2_A2,  5,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_1_pss", 'r', ACS2_A2,  7,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_2_pss", 'r', ACS2_A2, 11,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_2_pss", 'r', ACS2_A2, 13,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_2_pss", 'r', ACS2_A2, 15,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_2_pss", 'r', ACS2_A2, 17,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_3_pss", 'r', ACS2_A2, 19,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_3_pss", 'r', ACS2_A2, 21,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_3_pss", 'r', ACS2_A2, 23,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_3_pss", 'r', ACS2_A2, 25,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_4_pss", 'r', ACS2_A2, 27,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_4_pss", 'r', ACS2_A2, 29,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_4_pss", 'r', ACS2_A2, 31,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_4_pss", 'r', ACS2_A2, 47,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_5_pss", 'r', ACS2_A2, 39,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_5_pss", 'r', ACS2_A2, 41,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_5_pss", 'r', ACS2_A2, 43,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_5_pss", 'r', ACS2_A2, 45,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_6_pss", 'r', ACS2_A2, 49,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_6_pss", 'r', ACS2_A2, 33,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_6_pss", 'r', ACS2_A2, 35,        CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_6_pss", 'r', ACS2_A2, 37,        CAL16(-1.,            0.),  'u', U_V_V},
#endif

  {"heat_t_hk",    'w',  HWP_D,  53,            1.0,          0.0, 'u', U_NONE},
  {"heat_13_hk",   'w',  RTD_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"heat_45_hk",   'w',  RTD_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"heat_26_hk",   'w',  RTD_D,  52,            1.0,          0.0, 'u', U_NONE},

  /* slow MCE data */
  {"drive_map_mpc1",  'w',  LOOP6, 46, 1, 0, 'u', U_NONE},
  {"drive_map_mpc2",  'w',  LOOP6, 47, 1, 0, 'u', U_NONE},
  {"drive_map_mpc3",  'w',  LOOP6, 48, 1, 0, 'u', U_NONE},
  {"drive_map_mpc4",  'w',  LOOP6, 49, 1, 0, 'u', U_NONE},
  {"drive_map_mpc5",  'w',  LOOP6, 50, 1, 0, 'u', U_NONE},
  {"drive_map_mpc6",  'w',  LOOP6, 51, 1, 0, 'u', U_NONE},

  {"df_0_mcc1",       'w',  LOOP6, 52, (1<<24), 0, 'u', U_NONE},
  {"df_0_mcc2",       'w',  LOOP6, 53, (1<<24), 0, 'u', U_NONE},
  {"df_0_mcc3",       'w',  LOOP6, 54, (1<<24), 0, 'u', U_NONE},
  {"df_0_mcc4",       'w',  LOOP6, 55, (1<<24), 0, 'u', U_NONE},
  {"df_0_mcc5",       'w',  LOOP6, 56, (1<<24), 0, 'u', U_NONE},
  {"df_0_mcc6",       'w',  LOOP6, 57, (1<<24), 0, 'u', U_NONE},
 
  {"df_1_mcc1",       'w',  LOOP6, 58, (1<<24), 0, 'u', U_NONE},
  {"df_1_mcc2",       'w',  LOOP6, 59, (1<<24), 0, 'u', U_NONE},
  {"df_1_mcc3",       'w',  LOOP6, 60, (1<<24), 0, 'u', U_NONE},
  {"df_1_mcc4",       'w',  LOOP6, 61, (1<<24), 0, 'u', U_NONE},
  {"df_1_mcc5",       'w',  LOOP6, 62, (1<<24), 0, 'u', U_NONE},
  {"df_1_mcc6",       'w',  LOOP6, 63, (1<<24), 0, 'u', U_NONE},

  {"df_2_mcc1",       'w',  LOOP7,  0, (1<<24), 0, 'u', U_NONE},
  {"df_2_mcc2",       'w',  LOOP7,  1, (1<<24), 0, 'u', U_NONE},
  {"df_2_mcc3",       'w',  LOOP7,  2, (1<<24), 0, 'u', U_NONE},
  {"df_2_mcc4",       'w',  LOOP7,  3, (1<<24), 0, 'u', U_NONE},
  {"df_2_mcc5",       'w',  LOOP7,  4, (1<<24), 0, 'u', U_NONE},
  {"df_2_mcc6",       'w',  LOOP7,  5, (1<<24), 0, 'u', U_NONE},

  {"df_3_mcc1",       'w',  LOOP7,  6, (1<<24), 0, 'u', U_NONE},
  {"df_3_mcc2",       'w',  LOOP7,  7, (1<<24), 0, 'u', U_NONE},
  {"df_3_mcc3",       'w',  LOOP7,  8, (1<<24), 0, 'u', U_NONE},
  {"df_3_mcc4",       'w',  LOOP7,  9, (1<<24), 0, 'u', U_NONE},
  {"df_3_mcc5",       'w',  LOOP7, 10, (1<<24), 0, 'u', U_NONE},
  {"df_3_mcc6",       'w',  LOOP7, 11, (1<<24), 0, 'u', U_NONE},

  {"state_mpc1",      'w',  LOOP7, 12, 1.0,   0.0, 'u', U_NONE},
  {"state_mpc2",      'w',  LOOP7, 13, 1.0,   0.0, 'u', U_NONE},
  {"state_mpc3",      'w',  LOOP7, 14, 1.0,   0.0, 'u', U_NONE},
  {"state_mpc4",      'w',  LOOP7, 15, 1.0,   0.0, 'u', U_NONE},
  {"state_mpc5",      'w',  LOOP7, 16, 1.0,   0.0, 'u', U_NONE},
  {"state_mpc6",      'w',  LOOP7, 17, 1.0,   0.0, 'u', U_NONE},

  {"dtg_mpc1",        'w',  LOOP7, 18, 1.0,   0.0, 'u', U_NONE},
  {"dtg_mpc2",        'w',  LOOP7, 19, 1.0,   0.0, 'u', U_NONE},
  {"dtg_mpc3",        'w',  LOOP7, 20, 1.0,   0.0, 'u', U_NONE},
  {"dtg_mpc4",        'w',  LOOP7, 21, 1.0,   0.0, 'u', U_NONE},
  {"dtg_mpc5",        'w',  LOOP7, 22, 1.0,   0.0, 'u', U_NONE},
  {"dtg_mpc6",        'w',  LOOP7, 23, 1.0,   0.0, 'u', U_NONE},

  {"task_mpc1",       'w',  LOOP7, 24, 1.0,   0.0, 'u', U_NONE},
  {"task_mpc2",       'w',  LOOP7, 25, 1.0,   0.0, 'u', U_NONE},
  {"task_mpc3",       'w',  LOOP7, 26, 1.0,   0.0, 'u', U_NONE},
  {"task_mpc4",       'w',  LOOP7, 27, 1.0,   0.0, 'u', U_NONE},
  {"task_mpc5",       'w',  LOOP7, 28, 1.0,   0.0, 'u', U_NONE},
  {"task_mpc6",       'w',  LOOP7, 29, 1.0,   0.0, 'u', U_NONE},

  {"last_iv_mpc1",    'w',  LOOP7, 30, 1.0,   0.0, 'u', U_NONE},
  {"last_iv_mpc2",    'w',  LOOP7, 31, 1.0,   0.0, 'u', U_NONE},
  {"last_iv_mpc3",    'w',  LOOP7, 32, 1.0,   0.0, 'u', U_NONE},
  {"last_iv_mpc4",    'w',  LOOP7, 33, 1.0,   0.0, 'u', U_NONE},
  {"last_iv_mpc5",    'w',  LOOP7, 34, 1.0,   0.0, 'u', U_NONE},
  {"last_iv_mpc6",    'w',  LOOP7, 35, 1.0,   0.0, 'u', U_NONE},

  {"t_mcc1",          'w',  LOOP7, 36, 0.01,  0.0, 's', U_T_C},
  {"t_mcc2",          'w',  LOOP7, 37, 0.01,  0.0, 's', U_T_C},
  {"t_mcc3",          'w',  LOOP7, 38, 0.01,  0.0, 's', U_T_C},
  {"t_mcc4",          'w',  LOOP7, 39, 0.01,  0.0, 's', U_T_C},
  {"t_mcc5",          'w',  LOOP7, 40, 0.01,  0.0, 's', U_T_C},
  {"t_mcc6",          'w',  LOOP7, 41, 0.01,  0.0, 's', U_T_C},

  {"t_mce1",          'w',  LOOP7, 42, 0.5,   0.0, 's', U_T_C},
  {"t_mce2",          'w',  LOOP7, 43, 0.5,   0.0, 's', U_T_C},
  {"t_mce3",          'w',  LOOP7, 44, 0.5,   0.0, 's', U_T_C},
  {"t_mce4",          'w',  LOOP7, 45, 0.5,   0.0, 's', U_T_C},
  {"t_mce5",          'w',  LOOP7, 46, 0.5,   0.0, 's', U_T_C},
  {"t_mce6",          'w',  LOOP7, 47, 0.5,   0.0, 's', U_T_C},

  {"ramp_count_mce1", 'w',  LOOP7, 48, 0.5,   0.0, 'u', U_NONE},
  {"ramp_count_mce2", 'w',  LOOP7, 49, 0.5,   0.0, 'u', U_NONE},
  {"ramp_count_mce3", 'w',  LOOP7, 50, 0.5,   0.0, 'u', U_NONE},
  {"ramp_count_mce4", 'w',  LOOP7, 51, 0.5,   0.0, 'u', U_NONE},
  {"ramp_count_mce5", 'w',  LOOP7, 52, 0.5,   0.0, 'u', U_NONE},
  {"ramp_count_mce6", 'w',  LOOP7, 53, 0.5,   0.0, 'u', U_NONE},

  /* LOOP7, 54-59 are fast or wide-fast */
  {"blob_num_mpc",    'w',  LOOP7, 60,  1,      0, 'u', U_NONE},
  {"squid_veto_mpc",  'w',  LOOP7, 61,  1,      0, 'u', U_NONE},
  {"reporting_mpcs",  'w',  LOOP7, 62,  1,      0, 'u', U_NONE},
  {"alive_mpcs",      'w',  LOOP7, 63,  1,      0, 'u', U_NONE},

  {"tile_heater_mce1",  'w',  LOOP8,  0, 5./32767, 0.0, 'u', U_V_V},
  {"tile_heater_mce2",  'w',  LOOP8,  1, 5./32767, 0.0, 'u', U_V_V},
  {"tile_heater_mce3",  'w',  LOOP8,  2, 5./32767, 0.0, 'u', U_V_V},
  {"tile_heater_mce4",  'w',  LOOP8,  3, 5./32767, 0.0, 'u', U_V_V},
  {"tile_heater_mce5",  'w',  LOOP8,  4, 5./32767, 0.0, 'u', U_V_V},
  {"tile_heater_mce6",  'w',  LOOP8,  5, 5./32767, 0.0, 'u', U_V_V},

  {"last_tune_mpc1",  'w',  LOOP8,  6, 0.5,   0.0, 'u', U_NONE},
  {"last_tune_mpc2",  'w',  LOOP8,  7, 0.5,   0.0, 'u', U_NONE},
  {"last_tune_mpc3",  'w',  LOOP8,  8, 0.5,   0.0, 'u', U_NONE},
  {"last_tune_mpc4",  'w',  LOOP8,  9, 0.5,   0.0, 'u', U_NONE},
  {"last_tune_mpc5",  'w',  LOOP8, 10, 0.5,   0.0, 'u', U_NONE},
  {"last_tune_mpc6",  'w',  LOOP8, 11, 0.5,   0.0, 'u', U_NONE},

  {"used_tune_mpc1",  'w',  LOOP8, 12, 0.5,   0.0, 'u', U_NONE},
  {"used_tune_mpc2",  'w',  LOOP8, 13, 0.5,   0.0, 'u', U_NONE},
  {"used_tune_mpc3",  'w',  LOOP8, 14, 0.5,   0.0, 'u', U_NONE},
  {"used_tune_mpc4",  'w',  LOOP8, 15, 0.5,   0.0, 'u', U_NONE},
  {"used_tune_mpc5",  'w',  LOOP8, 16, 0.5,   0.0, 'u', U_NONE},
  {"used_tune_mpc6",  'w',  LOOP8, 17, 0.5,   0.0, 'u', U_NONE},
  
  /* END of MCE slow channels */

  /* LOOP8, 18 is fast */
  {"data_mode_mce",   'w',  LOOP8, 19,   1,     0, 'u', U_NONE},
  {"data_mode_bits",  'w',  LOOP8, 20,   1,     0, 'u', U_NONE},
  {"sync_veto_mpc",   'w',  LOOP8, 21,   1,     0, 'u', U_NONE},

  {"plate_g",         'w', LOOP8, 22, 1.0/1000.0, 0.0, 'u',U_NONE},
  {"plate_b",         'w', LOOP8, 23, 1.0/1000.0, 0.0, 'u',U_NONE},
  {"plate_u",         'w', LOOP8, 24, 1.0/1000.0, 0.0, 'u',U_NONE},
  {"pyramid",         'w', LOOP8, 25,    1,         0, 'u',U_NONE},
  {"vel_table",       'w', LOOP8, 26, 1.0/1000.0, 0.0, 's',U_V_DPS},
  {"pos_table",       'w', LOOP8, 27, 1.0/1000.0, 0.0, 'u',U_P_DEG},
  {"mode_table",      'w', LOOP8, 28,    1,         0, 'u',U_NONE},

  {"clamp_count_mce1", 'w', LOOP8, 29, 1, 0, 'u', U_NONE},
  {"clamp_count_mce2", 'w', LOOP8, 30, 1, 0, 'u', U_NONE},
  {"clamp_count_mce3", 'w', LOOP8, 31, 1, 0, 'u', U_NONE},
  {"clamp_count_mce4", 'w', LOOP8, 32, 1, 0, 'u', U_NONE},
  {"clamp_count_mce5", 'w', LOOP8, 33, 1, 0, 'u', U_NONE},
  {"clamp_count_mce6", 'w', LOOP8, 34, 1, 0, 'u', U_NONE},
  
  {"bolo_filt_freq",  'w',  LOOP8, 35, 120./32767., 0.0, 'u', U_NONE},
  {"bolo_filt_bw",    'w',  LOOP8, 36,  10./32767., 0.0, 'u', U_NONE},
  {"bolo_filt_len",   'w',  LOOP8, 37,           1,   0, 'u', U_NONE},

  {"therm_veto_mpc",  'w',  LOOP8, 38,           1,   0, 'u', U_NONE},

  {"ref_tune_mpc1",  'w',   LOOP8, 39, 0.5,   0.0, 'u', U_NONE},
  {"ref_tune_mpc2",  'w',   LOOP8, 40, 0.5,   0.0, 'u', U_NONE},
  {"ref_tune_mpc3",  'w',   LOOP8, 41, 0.5,   0.0, 'u', U_NONE},
  {"ref_tune_mpc4",  'w',   LOOP8, 42, 0.5,   0.0, 'u', U_NONE},
  {"ref_tune_mpc5",  'w',   LOOP8, 43, 0.5,   0.0, 'u', U_NONE},
  {"ref_tune_mpc6",  'w',   LOOP8, 44, 0.5,   0.0, 'u', U_NONE},

  {"tune_stat_mpc1",  'w',  LOOP8, 45, 0.5,   0.0, 'u', U_NONE},
  {"tune_stat_mpc2",  'w',  LOOP8, 46, 0.5,   0.0, 'u', U_NONE},
  {"tune_stat_mpc3",  'w',  LOOP8, 47, 0.5,   0.0, 'u', U_NONE},
  {"tune_stat_mpc4",  'w',  LOOP8, 48, 0.5,   0.0, 'u', U_NONE},
  {"tune_stat_mpc5",  'w',  LOOP8, 49, 0.5,   0.0, 'u', U_NONE},
  {"tune_stat_mpc6",  'w',  LOOP8, 50, 0.5,   0.0, 'u', U_NONE},

  {"t_cpu_g",         'w',  LOOP8, 51, 0.01,  0.0, 'u', U_T_C},
  {"t_cpu_b",         'w',  LOOP8, 52, 0.01,  0.0, 'u', U_T_C},
  {"t_cpu_u",         'w',  LOOP8, 53, 0.01,  0.0, 'u', U_T_C},

  {"uptime_mpc1",     'w',  LOOP8, 54, 40./3600,   0.0, 'u', U_T_H},
  {"uptime_mpc2",     'w',  LOOP8, 55, 40./3600,   0.0, 'u', U_T_H},
  {"uptime_mpc3",     'w',  LOOP8, 56, 40./3600,   0.0, 'u', U_T_H},
  {"uptime_mpc4",     'w',  LOOP8, 57, 40./3600,   0.0, 'u', U_T_H},
  {"uptime_mpc5",     'w',  LOOP8, 58, 40./3600,   0.0, 'u', U_T_H},
  {"uptime_mpc6",     'w',  LOOP8, 59, 40./3600,   0.0, 'u', U_T_H},
  {"t_set_motvalve",  'w',  LOOP8, 60,  (100.0/32768.0),  0.0, 's', U_T_C},
  {"twist_default",   'w',  LOOP8, 61,  I2DEG,     0.0, 's', U_P_DEG},
  {"twist_limit",     'w',  LOOP8, 62,  I2DEG,     0.0, 's', U_P_DEG},
  {"t_set_elmot_s",    'w', LOOP8, 63,  (100.0/32768.0),  0.0, 's',      U_T_C},
  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
#ifndef BOLOTEST
  {"ofyaw_1_gy",  'r', ACS2_D,  0, -DGY32_TO_DPS, DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofyaw_2_gy",  'r', ACS2_D,  2, -DGY32_TO_DPS, DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofroll_1_gy", 'r', ACS2_D,  4, DGY32_TO_DPS, -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofroll_2_gy", 'r', ACS2_D,  6, DGY32_TO_DPS, -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofpch_2_gy",  'r', ACS2_D,  8, DGY32_TO_DPS, -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ofpch_1_gy",  'r', ACS2_D, 10, DGY32_TO_DPS, -DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"enc_table",   'r', ACS2_D, 54, 360.0/144000.0,           0.0, 'U', U_P_DEG},
#endif

/* housekeeping channels */  /* TODO many can probably be slow */
  {"vr_ntd1_x2_hk",   'r', RTD_A1,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_x2_hk",     'r', RTD_A1,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ring_x2_hk",    'r', RTD_A1,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_x2_hk",   'r', RTD_A1, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_x2_hk",   'r', RTD_A1, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_x2_hk",   'r', RTD_A1, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd1_x3_hk",   'r', RTD_A1, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_x3_hk",     'r', RTD_A1, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ring_x3_hk",    'r', RTD_A1, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_x3_hk",   'r', RTD_A1, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_x3_hk",   'r', RTD_A1, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_x3_hk",   'r', RTD_A1, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  //NB: (2011-09-25) 50Hz cals measured for board #1. other vr_ cals use mean
  {"vr_ntd1_x1_hk",   'r', RTD_A1, 36, CAL32N(       1.134,           0.0), 'U', U_V_V},
  {"vr_fp_x1_hk",     'r', RTD_A1, 38, CAL32C(       1.052,           0.0), 'U', U_V_V},
  {"vr_ring_x1_hk",    'r', RTD_A1, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_x1_hk",   'r', RTD_A1, 42, CAL32N(       1.126,           0.0), 'U', U_V_V},
  {"vr_ntd3_x1_hk",   'r', RTD_A1, 44, CAL32N(       1.122,           0.0), 'U', U_V_V},
  {"vr_ntd2_x1_hk",   'r', RTD_A1, 46, CAL32N(       1.132,           0.0), 'U', U_V_V},

  {"vr_ntd1_x5_hk",   'r', RTD_A2,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_x5_hk",     'r', RTD_A2,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ring_x5_hk",    'r', RTD_A2,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_x5_hk",   'r', RTD_A2, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_x5_hk",   'r', RTD_A2, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_x5_hk",   'r', RTD_A2, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd1_x6_hk",   'r', RTD_A2, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_x6_hk",     'r', RTD_A2, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ring_x6_hk",    'r', RTD_A2, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_x6_hk",   'r', RTD_A2, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_x6_hk",   'r', RTD_A2, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_x6_hk",   'r', RTD_A2, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd1_x4_hk",   'r', RTD_A2, 36, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_x4_hk",     'r', RTD_A2, 38, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ring_x4_hk",    'r', RTD_A2, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_x4_hk",   'r', RTD_A2, 42, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_x4_hk",   'r', RTD_A2, 44, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_x4_hk",   'r', RTD_A2, 46, CAL32N(       1.129,           0.0), 'U', U_V_V},

  {"vp_01_hk",       'r',  RTD_A1, 48, CAL32(1.0, 0.0),                    'U', U_V_V},
  {"vp_02_hk",       'r', DIOD_A3, 48, CAL32(1.0, 0.0),                    'U', U_V_V},

  {"enc_a_x1_hwp",  'r', HWP_A1,  0,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_x1_hwp",  'r', HWP_A1,  2,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_x1_hwp",  'r', HWP_A1,  4,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_x1_hwp",  'r', HWP_A1,  6,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_x2_hwp",  'r', HWP_A1,  8,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_x2_hwp",  'r', HWP_A1, 10,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_x2_hwp",  'r', HWP_A1, 12,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_x2_hwp",  'r', HWP_A1, 14,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_x3_hwp",  'r', HWP_A1, 16,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_x3_hwp",  'r', HWP_A1, 18,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_x3_hwp",  'r', HWP_A1, 20,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_x3_hwp",  'r', HWP_A1, 22,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_x4_hwp",  'r', HWP_A1, 24,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_x4_hwp",  'r', HWP_A1, 26,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_x4_hwp",  'r', HWP_A1, 28,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_x4_hwp",  'r', HWP_A1, 30,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_x5_hwp",  'r', HWP_A1, 32,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_x5_hwp",  'r', HWP_A1, 34,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_x5_hwp",  'r', HWP_A1, 36,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_x5_hwp",  'r', HWP_A1, 38,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_a_x6_hwp",  'r', HWP_A1, 40,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_i_x6_hwp",  'r', HWP_A1, 42,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_b_x6_hwp",  'r', HWP_A1, 44,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"enc_s_x6_hwp",  'r', HWP_A1, 46,CAL32(1.0,      0.0),      'U',      U_V_V},

  {"az",          'w', LOOP7,   54,      LI2DEG,             0.0, 'U', U_P_DEG},
  {"el",          'w', LOOP7,   56,      LI2DEG,             0.0, 'U', U_P_DEG},
  /* NB: This is LOOP0, 0 on the READ side -- ie. a completely different channel
   * than the LOOP0, 0 on the WRITE side (which is used for the "time" field)
   *
   * Like all the MCE fast data, values for this channel are inserted
   * directly into the RxFrame and never appear on the BBus */
  {"mce_frameno",   'r', LOOP0,   0, 1.0,           0.0,       'U',     U_NONE},
  {"mce_mplex",     'r', LOOP0,   2, 1.0,           0.0,       'U',     U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
#ifndef BOLOTEST
  /* ACS2 Common Node */
  {"framenum",     'r',  ACS2_C,  1,                1.0,      0.0, 'u', U_NONE},

  /* ACS2 Digital Card */
  {"ofpch_gy",  'r', ACS2_D, 12, GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ofroll_gy", 'r', ACS2_D, 13, GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ofyaw_gy",  'r', ACS2_D, 14,-GY16_TO_DPS, GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ofaz_gy",   'r', ACS2_D, 16,-GY16_TO_DPS, GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"vel_req_az",'w', ACS2_D, 27, GY16_TO_DPS,-32768.0*GY16_TO_DPS, 'u',U_V_DPS},
  {"step_1_el", 'w', ACS2_D, 29, 10000.0/32767.0, -10000.30518509, 'u', U_F_HZ},
  {"step_2_el", 'w', ACS2_D, 30, 10000.0/32767.0, -10000.30518509, 'u', U_F_HZ},

  {"el_raw_1_enc", 'r', ACS2_D, 56,  I2DEG,        ENC1_OFFSET, 'u', U_P_DEG},
  {"el_raw_2_enc", 'r', ACS2_D, 57, -I2DEG,        ENC2_OFFSET, 'u', U_P_DEG},
  // free
  //{"vel_ofpch_gy",  'r', ACS2_A1, 45,   CAL16(14.9925037, 0.0), 'u', U_V_DPS},
  {"pulse_sc",     'r',  ACS2_A1, 50,                 1.0,            0.0, 'u', U_NONE},

  {"chopper",         'r',DIOD_A2, 49, CAL16(1.00, 0.0),       'u', U_V_V},

  {"mce_index",        'r', LOOP0,  4, 1.0,         0.0,       'u',     U_NONE},
  {"bolo_stats_index", 'r', LOOP0,  5, 1.0,         0.0,       'u',     U_NONE},
  {"bolo_stats_mplex", 'r', LOOP0,  6, 1.0,         0.0,       'u',     U_NONE},
  {"mce_blob",         'r', LOOP0,  7, 1.0,         0.0,       'u',     U_NONE},
  {"is_turn_around",   'w', LOOP6, 35, 1.0,         0.0,       'u',     U_NONE},

  /* Create the correct number of mce channels in the tx struct.  They
   * will be called "mce###" for ### from zero to the value defined in tes.h
   * They are placed on the READ side of LOOPMCE# for # from 0 to something
   * large enough to contain them all.  The defnitions are of the form:
   *
   * {"mce###", 'r', LOOPMCE#, ##, 1.0, 0.0, 'u', U_NONE}
   *
   * Use spider_config/make_tx_struct_mce to build this file (see pcm/Makefile
   * for an example).  */
#include "tx_struct_mce.c"

#endif

  {"dps_table",    'w', LOOP7, 58,          70.0/32767.0,   0.0, 's', U_V_DPS},
  {"chatter",      'w', LOOP7, 59,                   1.0,   0.0, 'u', U_NONE},

  /* TES debugging */
  {"tes_nfifo",    'w', LOOP8, 18,                     1,     0, 'u', U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,   1.0,                    0.0, 'u', U_NONE},
  {"polarity",    'w', DECOM,  2,   1.0,                    0.0, 'u', U_NONE},
  {"decom_unlock",'w', DECOM,  3,   1.0,                    0.0, 'u', U_NONE},
  END_OF_CHANNELS
};


char *GetArrayFieldName(int i_field) {
  static char **names = 0;
  int i;

  if (names == 0) {
    int type, tel, row, col;
    char types[N_STAT_TYPES][10] = {"mean", "sigma", "noise", "bstep"};
    names = (char **) malloc(NUM_ARRAY_STAT * sizeof(char *));
    for (i=0; i < NUM_ARRAY_STAT; i++) {
      names[i] = (char *) malloc(21*sizeof(char));
    }
    i=0;
    for (tel = 0; tel < NUM_MCE; tel++) {
      for (type = 0; type < 3; type++) {
        for (row = 0; row<NUM_ROW; row++) {
          for (col = 0; col<NUM_COL; col++) {
            sprintf(names[i], "%s_x%1dr%02dc%02d", types[type], tel+1, row,
                col);
            i++;
          }
        }
      }
    }
  }
  return names[i_field];
}

