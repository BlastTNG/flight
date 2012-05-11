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
#ifdef __MCP__
#include "camstruct.h"
#endif

/* card name to (node number, bus number) mapping */
#define ACS1_C	 0, 0  /* C denotes a common motherboard node */
#define ACS1_D	 1, 0  /* D denotes a digital daughter card */
#define ACS1_A1	 2, 0  /* A deontes a node for analog daughter card */
#define ACS1_T1	 3, 0  /* T denotes an AD590 thermometry daughter card */
#define ACS2_C	 4, 0
#define ACS2_D	 5, 0
#define ACS2_A1	 6, 0
#define ACS2_A2	 7, 0
#define RTD_C	 8, 0
#define RTD_D	 9, 0
#define RTD_A1	10, 0
#define RTD_A2	11, 0
#define DIOD_C	12, 0
#define DIOD_A1	13, 0
#define DIOD_A2	14, 0
#define DIOD_A3	15, 0
#define HWP_C	16, 0
#define HWP_D	17, 0
#define HWP_A1	18, 0
#define HWP_A2	19, 0
//TODO could make it possible for LOOPbacks to not need explicit channels
#define LOOP1	32, 0
#define LOOP2	33, 0
#define LOOP3	34, 0
#define LOOP4	35, 0
#define LOOP5	36, 0
#define LOOP6	37, 0
#define LOOP7	38, 0
#define LOOP8	39, 0
#define LOOP9	40, 0
#define LOOP0	41, 0
#define DECOM	42, 0

/* Analog channel calibrations */
#define CAL16(m,b) ((m)*M_16PRE), ((b) + B_16PRE*(m)*M_16PRE)
#define CAL32(m,b) ((m)*M_32PRE), ((b) + B_32PRE*(m)*M_32PRE)
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T)
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
#define U_PH_DEG  "Phase","^o"
#define U_LA_DEG "Latitude","^o"
#define U_LO_DEG "Longitude","^o"
#define U_D_DEG "Direction","^o"
#define U_V_V	"Voltage","V"
#define U_I_A   "Current","A"
#define U_T_MS  "Time","ms"
#define U_R_O   "Resistance","Ohms"
#define U_RATE "Rate", "bps"
#define U_F_HZ "Frequency", "Hz"

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
  {"ra",           'w', LOOP3,  4,               LI2H,             0.0, 'U', U_NONE},
  {"frame_thebad", 'w', LOOP3, 31,                1.0,             0.0, 'U', U_NONE},
  {"sec_thebad",   'w', LOOP3, 44,                1.0,             0.0, 'U', U_NONE},
  {"usec_thebad",  'w', LOOP3, 58,                1.0,             0.0, 'U', U_NONE},
  //{"cycle_start",  'w', LOOP4, 24,                1.0,             0.0, 'U', U_NONE},
  {"dec",          'w', LOOP5,  6,             LI2DEG,             0.0, 'S', U_NONE},
  {"lst_sched",    'w', LOOP6, 56,                1.0,             0.0, 'U', U_NONE},  // ls day
  {"frame_theugly",    'w', LOOP9, 50,                1.0,             0.0, 'U', U_NONE},
  {"sec_theugly",      'w', LOOP9, 52,                1.0,             0.0, 'U', U_NONE},
  {"usec_theugly",     'w', LOOP9, 54,                1.0,             0.0, 'U', U_NONE},
  //derived channel time_theugly adds these together

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
  {"status18",     'r',  HWP_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status19",     'r',  HWP_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync00",       'w',  ACS1_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync01",       'w',  ACS1_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync02",       'w', ACS1_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync03",       'w', ACS1_T1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync04",       'w',  ACS2_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync05",       'w',  ACS2_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync06",       'w', ACS2_A1, 63,                1.0,             0.0, 'u', U_NONE},
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
  {"sync18",       'w',  HWP_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync19",       'w',  HWP_A2, 63,                1.0,             0.0, 'u', U_NONE},

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
  {"heat_ssa_2_hk", 'w', RTD_D,   2, CALDAC( 0.9970,     -0.0013), 'u',  U_V_V},
  {"heat_fplo_2_hk",'w', RTD_D,   3, CALDAC( 0.9973,     -0.0010), 'u',  U_V_V},
  {"v_cnx_1_hk",    'w', RTD_D,   4, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_1_hk",    'w', RTD_D,   5, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ssa_1_hk", 'w', RTD_D,   6, CALDAC( 0.9974,     -0.0015), 'u',  U_V_V},
  {"heat_fplo_1_hk",'w', RTD_D,   7, CALDAC( 0.9989,     -0.0015), 'u',  U_V_V},
  {"v_cnx_3_hk",    'w', RTD_D,   8, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_3_hk",    'w', RTD_D,   9, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ssa_3_hk", 'w', RTD_D,  10, CALDAC( 1.0006,     -0.0013), 'u',  U_V_V},
  {"heat_fplo_3_hk",'w', RTD_D,  11, CALDAC( 1.0012,     -0.0012), 'u',  U_V_V},
  {"v_cnx_4_hk",    'w', RTD_D,  12, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_4_hk",    'w', RTD_D,  13, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ssa_4_hk", 'w', RTD_D,  14, CALDAC( 1.0035,     -0.0010), 'u',  U_V_V},
  {"heat_fplo_4_hk",'w', RTD_D,  15, CALDAC( 0.9983,     -0.0013), 'u',  U_V_V},
  {"v_cnx_6_hk",    'w', RTD_D,  16, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_6_hk",    'w', RTD_D,  17, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ssa_6_hk", 'w', RTD_D,  18, CALDAC( 0.9981,     -0.0015), 'u',  U_V_V},
  {"heat_fplo_6_hk",'w', RTD_D,  19, CALDAC( 1.0040,     -0.0015), 'u',  U_V_V},
  {"v_cnx_5_hk",    'w', RTD_D,  20, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"v_ntd_5_hk",    'w', RTD_D,  21, CALDAC(    1.0,         0.0), 'u',  U_V_V},
  {"heat_ssa_5_hk", 'w', RTD_D,  22, CALDAC( 0.9997,     -0.0012), 'u',  U_V_V},
  {"heat_fplo_5_hk",'w', RTD_D,  23, CALDAC( 0.9988,     -0.0014), 'u',  U_V_V},

#if 0	  //BLAST-Pol thermometers
  /* this is all the inerframe ad590's.*/
  {"t_padcdc_rec",  'r',  RTD_A1,  1, CAL16T(1.0,                    0.0), 'u', U_T_C},
  {"t_pauram_rec",  'r',  RTD_A1,  3, CAL16T(1.0,                    0.0), 'u', U_T_C},
  {"t_hkdcdc_rec",  'r',  RTD_A1,  5, CAL16T(1.0,                    0.0), 'u', U_T_C},
  {"t_stbd_das",    'r',  RTD_A1,  7, CAL16T(1.0, 0.0),                    'u', U_T_C}, 
  {"t_stbd_rec",    'r',  RTD_A1,  9, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_port_das",    'r',  RTD_A1, 11, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_port_rec",    'r',  RTD_A1, 13, CAL16T(1.0, 0.0),                    'u', U_T_C},
//{"t_8_das",       'r',  RTD_A1, 15, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_mot_pump_val",'r',  RTD_A1, 17,  CAL16T(1.0,	    0.0), 'u', U_T_C},
  {"t_1_prime",     'r',  RTD_A1, 19, CAL16T(1.0, AD590_CALIB_PRIMARY_1),  'u', U_T_C},
  {"t_if_top_back", 'r',  RTD_A1, 21, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_hwpr_mot",    'r',  RTD_A1, 23,  CAL16T(1.0,	    0.0), 'u', U_T_C},
  {"t_if_top_frnt", 'r',  RTD_A1, 25, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_hwpr_feed",   'r',  RTD_A1, 27,  CAL16T(1.0,	    0.0), 'u', U_T_C},
  {"t_if_bot_frnt", 'r',  RTD_A1, 29, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_strut_bot",   'r',  RTD_A1, 31, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_strut_side",  'r',  RTD_A1, 33, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_2_prime",     'r',  RTD_A1, 35, CAL16T(1.0, AD590_CALIB_PRIMARY_2),  'u', U_T_C},
  {"t_if_bot_back", 'r',  RTD_A1, 37, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_dac_box",     'r',  RTD_A1, 39,  CAL16T(1.0,      0.0), 'u', U_T_C},
  {"t_mot_act",     'r',  RTD_A1, 41, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_push_plate",  'r',  RTD_A1, 43, CAL16T(1.0, 0.0),                    'u', U_T_C},
  {"t_1_second",    'r',  RTD_A1, 45, CAL16T(1.0, AD590_CALIB_SECONDARY_1),'u', U_T_C},
  {"t_2_second",    'r',  RTD_A1, 47, CAL16T(1.0, AD590_CALIB_SECONDARY_1),'u', U_T_C},
//{"t_24_das",      'r',  RTD_A1, 49,         CAL16T(1.0,            0.0), 'u', U_NONE},
//{"t_25_das",      'r',  RTD_A1, 51, CAL16T(1.0, 0.0),                    'u', U_T_C},
//{"t_rec",         'r',  RTD_A1, 53, CAL16T(1.0, 0.0),                    'u', U_T_C},

  /* narrow DIOD_A2 channels */
  {"he4_lev",       'r', DIOD_A2,  1,         CAL16(1.0,            0.0), 'u', U_V_V},
  {"i_charcoal",    'r', DIOD_A2,  3,         CAL16(1.0,            0.0), 'u', U_V_V},
  {"i_cal_lamp",    'r', DIOD_A2,  5,         CAL16(1.0,            0.0), 'u', U_V_V},
  {"i_hs_char",     'r', DIOD_A2,  7,         CAL16(1.0,            0.0), 'u', U_V_V},
  {"i_hs_pot",      'r', DIOD_A2,  9,         CAL16(1.0,            0.0), 'u', U_V_V},
  {"i_300mk",       'r', DIOD_A2, 13,         CAL16(1.0,            0.0), 'u', U_V_V},
  {"i_jfet",        'r', DIOD_A2, 21,         CAL16(1.0,            0.0), 'u', U_V_V},
#endif

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

  {"vt_1_if",      'r',  HWP_A2, 37, CAL16T(         1.0,           0.0), 'u', U_V_V},
  {"vt_2_if",      'r',  HWP_A2, 39, CAL16T(         1.0,           0.0), 'u', U_V_V},
  {"vt_3_if",      'r',  HWP_A2, 41, CAL16T(         1.0,           0.0), 'u', U_V_V},
  {"vt_4_if",      'r',  HWP_A2, 43, CAL16T(         1.0,           0.0), 'u', U_V_V},
  {"vt_5_if",      'r',  HWP_A2, 45, CAL16T(         1.0,           0.0), 'u', U_V_V},
  {"vt_6_if",      'r',  HWP_A2, 47, CAL16T(         1.0,           0.0), 'u', U_V_V},
  {"vt_7_if",      'r',  HWP_A2, 49, CAL16T(         1.0,           0.0), 'u', U_V_V},

  /* LOOP1 0-7 are wide */
  /* LOOP1 10-11 are unused */
  /* LOOP1 12-13 are wide */
  {"g_com_el",     'w', LOOP1,  8,                3.0/65536.0,     0.0, 'u', U_NONE},
  {"g_diff_el",    'w', LOOP1,  9,                1.0/65536.0,     0.0, 'u', U_NONE},
  {"foc_res_thegood", 'w', LOOP1, 17,                1.0,             0.0, 'u', U_NONE},
  {"period_cal",   'w', LOOP1, 18,                .20,             0.0, 'u', U_NONE},
  {"status_eth",   'w', LOOP1, 19,                1.0,             0.0, 'u', U_NONE}, //Sun, ISC, OSC net status
  {"timeout",      'w', LOOP1, 20,                1.0,             0.0, 'u', U_NONE},
  {"az_sun",       'w', LOOP1, 21,              I2DEG,             0.0, 'u', U_D_DEG},
  {"status_mcc",   'w', LOOP1, 23,                1.0,             0.0, 'u', U_NONE}, //south_i_am, at_float, schedule, slot_sched
  //{"cryostate",    'w', LOOP1, 24,                1.0,             0.0, 'u', U_NONE},
  {"upslot_sched", 'w', LOOP1, 25,                1.0,             0.0, 'u', U_NONE},  
  {"t_chip_flc",   'w', LOOP1, 26,               0.01,             0.0, 'u', U_NONE},
  {"declination_mag",'w', LOOP1, 27,              I2DEG,             0.0, 'u', U_D_DEG}, // magnetic declination
  {"veto_sensor",  'w', LOOP1, 28,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 32-33 are wide */
  /* LOOP1 34 is fast */
  {"blob07_x_thegood",'w', LOOP1, 35, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob07_y_thegood",'w', LOOP1, 36, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"alt_sip",      'w', LOOP1, 37,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 38-41 are wide */
  {"mapmean_thegood",'w', LOOP1, 42,                1.0,             0.0, 'u', U_NONE},
  {"g_table_move", 'w', LOOP1, 43,     100.0/SHRT_MAX,             0.0, 'u', U_NONE},
  {"table_move",   'w', LOOP1, 44,           1.0/10.0,             0.0, 's', U_NONE}, 
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
  {"disk_free",    'w', LOOP1, 58,             1./250,             0.0, 'u', U_NONE},
  {"mode_p",       'w', LOOP1, 59,                  1,             0.0, 'u', U_NONE},
  {"x_p",          'w', LOOP1, 60,              I2DEG,             0.0, 'u', U_NONE},
  {"y_p",          'w', LOOP1, 61,              I2DEG,             0.0, 's', U_NONE},
  {"vel_az_p",     'w', LOOP1, 62,              I2VEL,             0.0, 's', U_NONE},
  {"del_p",        'w', LOOP1, 63,              I2VEL,             0.0, 'u', U_NONE},

  {"nblobs_thegood",  'w', LOOP2,  0,                1.0,             0.0, 'u', U_NONE},
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
  {"w_p",          'w', LOOP2, 13,              I2DEG,             0.0, 'u', U_NONE}, // pointing scan width
  {"move_tol_thegood",'w',LOOP2,14,                1.0,             0.0, 'u', U_NONE},
  {"exp_int_thegood",'w',LOOP2,15,                1.0,             0.0, 'u', U_NONE},
  {"exp_time_thegood",'w', LOOP2, 16,                1.0,             0.0, 'u', U_NONE},
  {"force_thegood",'w', LOOP2, 17,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 18-19 are unusued */
  /*P2 20-21 is wide */
  /* LOOP2 22 is unusued */
  {"thresh_thegood",'w', LOOP2, 23,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"grid_thegood", 'w', LOOP2, 24,                1.0,             0.0, 'u', U_NONE},
  {"blob07_x_thebad",'w', LOOP2,  25, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  /* LOOP2 26 is unusued */
  {"mdist_thegood",'w', LOOP2, 27,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 28 is unusued */
  {"blob07_f_thegood",'w', LOOP2, 29,                1.0,             0.0, 'u', U_NONE},
  {"blob07_s_thegood",'w', LOOP2, 30,          1.0/100.0,             0.0, 'u', U_NONE},
  {"accel_max_az",'w', LOOP2, 31,         100.0/65535.0,             0.0, 'u', U_NONE},
  {"blob07_y_thebad",'w', LOOP2,  32, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  /* LOOP2 33 is unusued */
  /* LOOP2 34 is fast */
  {"blob07_f_thebad",'w', LOOP2, 35,                1.0,             0.0, 'u', U_NONE},
  {"offset_ifel_gy",  'w',LOOP2, 36,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ifroll_gy",'w',LOOP2, 37,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ifyaw_gy", 'w',LOOP2, 38,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"az_raw_mag",      'w',LOOP2, 39,            I2DEG,             0.0, 'u', U_D_DEG},
  {"sigma_mag",       'w',LOOP2, 40,            I2DEG,             0.0, 'u', U_NONE},
  {"az_dgps",         'w',LOOP2, 41,            I2DEG,             0.0, 'u', U_D_DEG},
  {"sigma_dgps",      'w',LOOP2, 42,            I2DEG,             0.0, 'u', U_NONE},
  /* LOOP2 44-45 are unusued */
  {"mapsigma_thegood",'w',LOOP2, 46,           1.0/10.0,             0.0, 'u', U_NONE},
  // LOOP2 47-48 are fast
  {"pulse_cal",    'w', LOOP2, 49,               10.0,              0., 'u', U_NONE},
  {"blob07_s_thebad",'w', LOOP2, 50,          1.0/100.0,             0.0, 'u', U_NONE},
  /* LOOP2 51-54 are wide fast */
  {"sigma_clin",   'w', LOOP2, 55,              I2DEG,             0.0, 'u', U_NONE},
  {"az_mag",       'w', LOOP2, 56,              I2DEG,             0.0, 'u', U_D_DEG},
  {"blob03_x_thegood",'w', LOOP2,  57, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob03_y_thegood",'w', LOOP2,  58, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob03_f_thegood",'w', LOOP2,  59,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 60-61 are wide */
  {"blob03_s_thegood",'w', LOOP2,  62,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob04_x_thegood",'w', LOOP2,  63, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},

  /* LOOP3 0 is unusued */
  {"blob04_y_thegood",'w', LOOP3,   1, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob04_f_thegood",'w', LOOP3,   2,                1.0,             0.0, 'u', U_NONE},
  {"blob04_s_thegood",'w', LOOP3,   3,          1.0/100.0,             0.0, 'u', U_NONE},
  /* LOOP3 4-5 are wide */
  /* LOOP3 6 is unusued */
  {"bbc_fifo_size",'w', LOOP3,  7,             1./624,             0.0, 'u', U_NONE},
  {"t_cpu_flc",    'w', LOOP3,  8,               0.01,             0.0, 'u', U_NONE},
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
  {"ra_thegood",     'w', LOOP3, 26,	      1.0/100.0,             0.0, 's', U_NONE},
  {"dec_thegood",    'w', LOOP3, 27,	      1.0/100.0,             0.0, 's', U_NONE},
  {"roll_thegood",   'w', LOOP3, 28,	      1.0/100.0,             0.0, 's', U_NONE},
  {"ra_thebad",      'w', LOOP3, 29,	      1.0/100.0,             0.0, 's', U_NONE},
  {"foc_res_thebad", 'w', LOOP3, 30,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 31-32 are wide */
  {"dec_thebad",     'w', LOOP3, 33,	      1.0/100.0,             0.0, 's', U_NONE},
  {"roll_thebad",    'w', LOOP3, 34,	      1.0/100.0,             0.0, 's', U_NONE},
  {"mapsigma_thebad",'w', LOOP3, 35,           1.0/10.0,             0.0, 'u', U_NONE},
  {"move_tol_thebad",'w', LOOP3, 36,                1.0,             0.0, 'u', U_NONE},
  {"exp_int_thebad", 'w', LOOP3, 37,                1.0,             0.0, 'u', U_NONE},
  {"exp_time_thebad",'w', LOOP3, 38,                1.0,             0.0, 'u', U_NONE},
  {"force_thebad",   'w', LOOP3, 39,                1.0,             0.0, 'u', U_NONE},
  {"ra_theugly",     'w', LOOP3, 40,	      1.0/100.0,             0.0, 's', U_NONE},
  {"dec_theugly",    'w', LOOP3, 41,          1.0/100.0,             0.0, 's', U_NONE},
  {"roll_theugly",   'w', LOOP3, 42,	      1.0/100.0,             0.0, 's', U_NONE},
  /* LOOP3 43 is unusued */
  /* LOOP3 44-45 are wide */
  {"thresh_thebad",'w',LOOP3, 46,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"grid_thebad",  'w', LOOP3, 47,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 48-49 are unusued */
  {"mdist_thebad",   'w', LOOP3, 50,                1.0,             0.0, 'u', U_NONE},
  {"blob03_x_thebad",  'w', LOOP3,  51, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob03_y_thebad",  'w', LOOP3,  52, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob03_f_thebad",  'w', LOOP3,  53,                1.0,             0.0, 'u', U_NONE},
  {"blob03_s_thebad",  'w', LOOP3,  54,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob04_x_thebad",  'w', LOOP3,  55, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob04_y_thebad",  'w', LOOP3,  56, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob04_f_thebad",  'w', LOOP3,  57,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 58-59 are wide */
  {"blob04_s_thebad",  'w', LOOP3,  60,          1.0/100.0,             0.0, 'u', U_NONE},
  /* LOOP3 61-63 are unusued */
  {"blob07_x_theugly",  'w', LOOP4,  0, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob07_y_theugly",  'w', LOOP4,  2, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
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
  {"blob07_f_theugly",  'w', LOOP4,  23,                1.0,             0.0, 'u', U_NONE},
  /* LOOP4 24-25 are wide */
  //{"cycle_state",  'w', LOOP4, 26,                1.0,             0.0, 'u', U_NONE},
  /* LOOP4 27-32 is unusued */
  {"g_p_heat_gy",   'w', LOOP4, 33,                1.0,             0.0, 'u', U_NONE},
  {"g_i_heat_gy",   'w', LOOP4, 34,                1.0,             0.0, 'u', U_NONE},
  {"g_d_heat_gy",   'w', LOOP4, 35,                1.0,             0.0, 'u', U_NONE},
  {"t_set_gy",     'w', LOOP4, 36,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"h_age_gy",     'w', LOOP4, 37,                1.0,             0.0, 'u', U_NONE},
  {"h_hist_gy",    'w', LOOP4, 38,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"trim_pss",     'w', LOOP4, 39,              I2DEG,             0.0, 's', U_NONE},
  {"az_pss",       'w', LOOP4, 40,              I2DEG,             0.0, 'u', U_P_DEG},
  {"blob07_s_theugly",  'w', LOOP4,  41,          1.0/100.0,             0.0, 'u', U_NONE},
  /* LOOP4 42-43 appear unused */
  {"ra_1_p",       'w', LOOP4, 44,                I2H,             0.0, 'u', U_NONE}, // pointing mode coordinates
  {"dec_1_p",      'w', LOOP4, 45,              I2DEG,             0.0, 's', U_NONE},
  {"ra_2_p",       'w', LOOP4, 46,                I2H,             0.0, 'u', U_NONE},
  {"dec_2_p",      'w', LOOP4, 47,              I2DEG,             0.0, 's', U_NONE},
  {"ra_3_p",       'w', LOOP4, 48,                I2H,             0.0, 'u', U_NONE},
  {"dec_3_p",      'w', LOOP4, 49,              I2DEG,             0.0, 's', U_NONE},
  {"ra_4_p",       'w', LOOP4, 50,                I2H,             0.0, 'u', U_NONE},
  {"dec_4_p",      'w', LOOP4, 51,              I2DEG,             0.0, 's', U_NONE},
  {"trim_clin",    'w', LOOP4, 52,              I2DEG,             0.0, 's', U_NONE},
  {"trim_enc",     'w', LOOP4, 53,              I2DEG,             0.0, 's', U_NONE},
  {"trim_null",    'w', LOOP4, 54,              I2DEG,             0.0, 's', U_NONE},
  {"trim_mag",     'w', LOOP4, 55,              I2DEG,             0.0, 's', U_NONE},
  {"trim_dgps",    'w', LOOP4, 56,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP4 57 is unused */
  /* LOOP4 58-59 are wide */
  {"az_raw_dgps",  'w', LOOP4, 60,              I2DEG,             0.0, 'u', U_D_DEG},
  {"el_clin",      'w', LOOP4, 62,              I2DEG,             0.0, 'u', U_NONE},
  {"h_p",          'w', LOOP4, 63,              I2DEG,             0.0, 'u', U_NONE}, // scan height

  /* LOOP5 0 is unusued */
  {"el_lut_clin",  'w', LOOP5,  1,              I2DEG,             0.0, 'u', U_NONE},
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
  {"slew_veto",    'w', LOOP5, 17,           4.0 / SR,             0.0, 'u', U_NONE},
  /* LOOP5 18-19 are wide */
  {"sveto_len",    'w', LOOP5, 20,           4.0 / SR,             0.0, 'u', U_NONE},
  {"dith_el",      'w', LOOP5, 23,        0.5/32768.0,             0.0, 's', U_D_DEG},
  {"state_lock",   'w', LOOP5, 25,                1.0,             0.0, 'u', U_NONE},
  /* LOOP5 28 is fast */
  {"dith_step_p",  'w', LOOP5, 29,        0.1/32768.0,             0.0, 's', U_D_DEG},
  /* LOOP5 34 is fast */
  /* LOOP5 35 is unused */
  //{"he4_lev_old",  'w', LOOP5, 40,          CAL16(1.0,            0.0), 'u', U_NONE},
  /* LOOP5 41 is unusued */
  /* LOOP5 42-53 are wide */
  /* LOOP5 54 is unusued */
  {"pitch_mag",        'w',LOOP5, 55,           I2DEG,             0.0, 'u', U_NONE},
  /* LOOP5 56-63 are unusued */

  {"blob05_x_thegood",  'w', LOOP6,  0, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob05_y_thegood",  'w', LOOP6,  1, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob05_f_thegood",  'w', LOOP6,  2,                1.0,             0.0, 'u', U_NONE},
  {"blob05_s_thegood",  'w', LOOP6,  3,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob05_x_thebad",   'w', LOOP6,  4, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob05_y_thebad",   'w', LOOP6,  5, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob05_f_thebad",   'w', LOOP6,  6,                1.0,             0.0, 'u', U_NONE},
  {"blob05_s_thebad",   'w', LOOP6,  7,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob05_x_theugly",  'w', LOOP6,  8, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob05_y_theugly",  'w', LOOP6,  9, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob05_f_theugly",  'w', LOOP6, 10,                1.0,             0.0, 'u', U_NONE},
  {"blob05_s_theugly",  'w', LOOP6, 11,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob06_x_thegood",  'w', LOOP6, 12, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob06_y_thegood",  'w', LOOP6, 13, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob06_f_thegood",  'w', LOOP6, 14,                1.0,             0.0, 'u', U_NONE},
  {"blob06_s_thegood",  'w', LOOP6, 15,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob06_x_thebad",   'w', LOOP6, 16, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob06_y_thebad",   'w', LOOP6, 17, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob06_f_thebad",   'w', LOOP6, 18,                1.0,             0.0, 'u', U_NONE},
  {"blob06_s_thebad",   'w', LOOP6, 19,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob06_x_theugly",  'w', LOOP6, 23, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  /* LOOP6 30-33 are wide */
  /* LOOP6 56-57 are wide */
  {"el_sun",       'w', LOOP6, 58,              I2DEG,             0.0, 's', U_NONE},
  {"blob06_y_theugly",  'w', LOOP6, 59, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"mode_cal",     'w', LOOP6, 60,                1.0,             0.0, 'u', U_NONE},
  /* LOOP6 61-62 are unusued */
  /* LOOP7 0-3 are fast narrow */
//{"stat_1_rw",    'w', LOOP7,  4,             1.0,           0.0, 'u', U_NONE},
//{"stat_2_rw",    'w', LOOP7,  5,             1.0,           0.0, 'u', U_NONE},
//{"fault_rw",     'w', LOOP7,  6,             1.0,           0.0, 'u', U_NONE},
  {"stat_dr_rw",   'w', LOOP7,  4,             1.0,           0.0, 'u', U_NONE},
  {"stat_s1_rw",   'w', LOOP7,  5,             1.0,           0.0, 'u', U_NONE},
  
  {"stat_1_el",    'w', LOOP7,  7,                1.0,             0.0, 'u', U_NONE},  
  {"stat_2_el",    'w', LOOP7,  8,                1.0,             0.0, 'u', U_NONE},  
  {"fault_el",     'w', LOOP7,  9,                1.0,             0.0, 'u', U_NONE},  
  {"stat_dr_piv",  'w', LOOP7, 10,                1.0,             0.0, 'u', U_NONE},  
  {"stat_s1_piv",  'w', LOOP7, 11,                1.0,             0.0, 'u', U_NONE},  
//{"t_mc_rw",      'w', LOOP7, 12,             1.0,           0.0, 's', U_T_C},
  {"i_ser_rw",     'w', LOOP7, 13,    60.0/32768.0,           0.0, 's', U_I_A},
  {"t_mc_el",      'w', LOOP7, 14,                1.0,             0.0, 's', U_T_C},
  {"i_ser_el",     'w', LOOP7, 15,       30.0/32768.0,             0.0, 's', U_I_A},
  {"res_piv",      'w', LOOP7, 16,              I2DEG,             0.0, 'u', U_P_DEG},
  {"i_ser_piv",    'w', LOOP7, 17,       20.0/32768.0,             0.0, 's', U_I_A},
  /* LOOP7 18-19 are unusued */
  {"g_pe_piv",     'w', LOOP7, 20,                1.0,             0.0, 'u', U_NONE},
  {"g_pv_piv",     'w', LOOP7, 21,                1.0,             0.0, 'u', U_NONE},
  {"set_rw",       'w', LOOP7, 22,      500.0/32768.0,             0.0, 's', U_V_DPS},
  {"vel_dps_az",   'w', LOOP7, 23,       20.0/32768.0,             0.0, 's', U_V_DPS},
  {"blob08_s_theugly",  'w', LOOP7, 24,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob06_f_theugly",  'w', LOOP7, 25,                1.0,             0.0, 'u', U_NONE},
  {"blob06_s_theugly",  'w', LOOP7, 26,          1.0/100.0,             0.0, 'u', U_NONE},
  {"vel_ser_piv",  'w', LOOP7, 27,   2000.0/65536.0,         0.0, 's', U_V_DPS},
  {"vel_calc_piv", 'w', LOOP7, 28,        20.0/32768.0,             0.0, 's', U_V_DPS},
  /* LOOP7 29-31 are unusued */
  {"drive_info_rw",'w', LOOP7, 32,                 1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_rw",'w', LOOP7, 33,              1.0,             0.0, 'u', U_NONE},
  {"drive_info_el",'w', LOOP7, 34,                 1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_el",'w', LOOP7, 35,              1.0,             0.0, 'u', U_NONE},
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
  {"verbose_el",       'w', LOOP7, 49,             1.0,             0.0, 'u', U_NONE},
  {"verbose_piv",      'w', LOOP7, 52,             1.0,             0.0, 'u', U_NONE},
  {"p_rw_term_piv",    'w', LOOP7, 53,             1.0,             0.0, 's', U_NONE},
  {"p_err_term_piv",   'w', LOOP7, 54,             1.0,             0.0, 's', U_NONE},
  //{"step_start_bias",  'w', LOOP7, 55,             0.5,             0.0, 'u', U_NONE},
  //{"step_end_bias",    'w', LOOP7, 56,             0.5,             0.0, 'u', U_NONE},
  //{"step_n_bias",      'w', LOOP7, 57,             1.0,             0.0, 'u', U_NONE},
  //{"step_time_bias",   'w', LOOP7, 58,             1.0,             0.0, 'u', U_NONE},
  //{"step_which_bias",  'w', LOOP7, 60,             1.0,             0.0, 'u', U_NONE},
  //{"step_start_phase", 'w', LOOP7, 61,             0.5,             0.0, 'u', U_NONE},
  //{"step_end_phase",   'w', LOOP7, 62,             0.5,             0.0, 'u', U_NONE},
  //{"step_nsteps_phase",'w', LOOP7, 63,             1.0,             0.0, 'u', U_NONE},
  {"step_time_phase",  'w', LOOP8,  0,             1.0,             0.0, 'u', U_NONE},
  {"step_ena_bias",    'w', LOOP8,  1,             1.0,             0.0, 'u', U_NONE},
  {"step_ena_phase",   'w', LOOP8,  2,             1.0,             0.0, 'u', U_NONE},
  
  /* charge controller related channels */
  
  {"v_batt_cc",  'w',  LOOP8,  3,   1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"v_arr_cc",   'w',  LOOP8,  4,   1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"i_batt_cc",  'w',  LOOP8,  5,   1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  {"i_arr_cc",   'w',  LOOP8,  6,   1/400.0,  -32000.0/400.0,  'u',  U_I_A},
  { "t_hs_cc",   'w',  LOOP8,  7,   1.0,      0.0,            's',  U_T_C},
  {"fault_cc",   'w',  LOOP8,  8,   1.0,      0.0,            'u',  U_NONE},  // fault bitfield
  {"alarm_hi_cc",'w',  LOOP8,  9,   1.0,      0.0,            'u',  U_NONE},  // alarm high bitfield
  {"alarm_lo_cc",'w',  LOOP8,  10,  1.0,      0.0,            'u',  U_NONE},  // alarm low bitfield
  {"v_targ_cc",  'w',  LOOP8,  11,  1/180.0,  -32400.0/180.0,  'u',  U_V_V},
  {"state_cc",   'w',  LOOP8,  12,  1.0,      0.0,            'u',  U_NONE},   
  
  {"frict_off_piv",'w', LOOP8,  16,      2.0/65535.0,              0.0,   'u', U_NONE},
  {"frict_term_piv",'w',LOOP8,  17,      2.0/32767.0,              0.0,   's', U_NONE},
  {"frict_term_uf_piv",'w',LOOP8,18,     2.0/32767.0,              0.0,   's', U_NONE}, // For debugging remove later

  {"az_gy",        'w',   LOOP8, 19,       20.0/32768.0,             0.0, 's', U_V_DPS},

  {"azraw_pss",   'w',   LOOP8, 26,              I2DEG,             0.0, 'u', U_P_DEG},
  {"elraw_pss",   'w',   LOOP8, 27,              I2DEG,             0.0, 'u', U_P_DEG},
  {"snr_pss1",     'w',   LOOP8, 28,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss2",     'w',   LOOP8, 29,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss3",     'w',   LOOP8, 30,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss4",     'w',   LOOP8, 31,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss5",     'w',   LOOP8, 32,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss6",     'w',   LOOP8, 33,            1/1000.,             0.0, 'u', U_NONE},
  {"accel_az",     'w',   LOOP8, 34,          2.0/65536,             0.0, 'u', U_NONE},
  /* LOOP8 35-36 are unusued */

  {"az_cov_dgps",   'w', LOOP8, 37,              I2DEG,             0.0, 'u', U_NONE},
  {"pitch_cov_dgps",'w', LOOP8, 38,              I2DEG,             0.0, 'u', U_NONE},
  {"roll_cov_dgps", 'w', LOOP8, 39,              I2DEG,             0.0, 'u', U_NONE},

  {"led_cc",        'w', LOOP8, 40,              1.0,               0.0, 'u', U_NONE}, // charge controller LED state
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
  {"g_pt_el",        'w', LOOP9,  3,                1.0,             0.0, 'u', U_NONE},
  {"pos_4_hwp",      'w', LOOP9,  4,              I2DEG,             0.0, 'u', U_P_DEG},
  {"pos_5_hwp",      'w', LOOP9,  5,              I2DEG,             0.0, 'u', U_P_DEG},
  {"pos_6_hwp",      'w', LOOP9,  6,              I2DEG,             0.0, 'u', U_P_DEG},
  {"rate_tdrss",     'w', LOOP9,  9,                1.0,             0.0, 'u', U_RATE},
  {"rate_iridium",   'w', LOOP9,  10,                1.0,             0.0, 'u', U_RATE},
  {"blob03_x_theugly",  'w', LOOP9,  26, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob03_y_theugly",  'w', LOOP9,  27, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob03_f_theugly",  'w', LOOP9,  28,                1.0,             0.0, 'u', U_NONE},
  {"blob03_s_theugly",  'w', LOOP9,  29,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob04_x_theugly",  'w', LOOP9,  30, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob04_y_theugly",  'w', LOOP9,  31, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob04_f_theugly",  'w', LOOP9,  32,                1.0,             0.0, 'u', U_NONE},
  {"blob04_s_theugly",  'w', LOOP9,  33,          1.0/100.0,             0.0, 'u', U_NONE},
  {"insert_last_hk", 'w', LOOP9,  34,                1.0,             0.0, 'u', U_NONE},
  {"f_bias_hk",      'w', LOOP9,  35,      400.0/65535.0,             0.0, 'u', U_F_HZ},
  {"tile_last_hk",   'w', LOOP9,  36,                1.0,             0.0, 'u', U_NONE},
  {"v_heat_last_hk", 'w', LOOP9,  37,         CALDAC(1.0,            0.0), 'u', U_V_V},
  {"pulse_last_hk",  'w', LOOP9,  38,                1.0,             0.0, 'u', U_NONE},
  {"blob08_x_thegood",  'w', LOOP9,  39, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob08_y_thegood",  'w', LOOP9,  40, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob08_f_thegood",  'w', LOOP9,  41,                1.0,             0.0, 'u', U_NONE},
  {"blob08_s_thegood",  'w', LOOP9,  42,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob08_x_thebad",   'w', LOOP9,  43, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob08_y_thebad",   'w', LOOP9,  44, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob08_f_thebad",   'w', LOOP9,  45,                1.0,             0.0, 'u', U_NONE},
  {"blob08_s_thebad",   'w', LOOP9,  46,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob08_x_theugly",  'w', LOOP9,  47, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob08_y_theugly",  'w', LOOP9,  48, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob08_f_theugly",  'w', LOOP9,  49,                1.0,             0.0, 'u', U_NONE},
  /* LOOP9 50-55 are wide */
  {"i_tot",         'w', LOOP9, 56,              1.0e-3,            0.0, 'u', U_I_A}, // sum of currents read through ACS1 A1
  {"t_set_theugly",     'w', LOOP9, 57,    (100.0/32768.0),             0.0, 'u', U_NONE},  
  {"cov_lim_dgps",   'w', LOOP9, 58,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"ant_e_dgps",     'w', LOOP9, 60,	      1.0/100.0,	     0.0, 's',U_NONE},
  {"ant_n_dgps",     'w', LOOP9, 61,	      1.0/100.0,	     0.0, 's',U_NONE},
  {"ant_u_dgps",     'w', LOOP9, 62,	      1.0/100.0,	     0.0, 's',U_NONE},
  {"ants_lim_dgps",   'w', LOOP9, 63,    (100.0/32768.0),            0.0, 'u', U_NONE},
  {"frame_int_bbc",  'w', LOOP0,   0,                1.0,            0.0, 'u', U_NONE},
  {"rate_ext_bbc",   'w', LOOP0,   1,      400.0/65535.0,            0.0, 'u', U_F_HZ},
  {"frame_ext_bbc",  'w', LOOP0,   2,                1.0,            0.0, 'u', U_NONE},
  /* LOOP0 4-15 are wide fast TODO (change when hwp counts moved) */

#ifndef BOLOTEST
/* ACS1 Digital I/O card */
  {"latch0",       'w',  ACS1_D,  0,                1.0,             0.0, 'u', U_NONE},
  {"latch1",       'w',  ACS1_D,  1,                1.0,             0.0, 'u', U_NONE},
  {"switch_gy",    'w',  ACS1_D,  2,                1.0,             0.0, 'u', U_NONE},
  //this may be unused because of the new program
  //{"switch_charge",'w',  ACS1_D,  3,                1.0,             0.0, 'u', U_NONE},
  {"switch_misc",  'w',  ACS1_D,  4,                1.0,             0.0, 'u', U_NONE},

/* ACS1 Analog card 1 */
/* default current cal is 12.5, 0.0 */
  {"i_trans",      'r',  ACS1_A1, 1,          CAL16(11.32, -0.02),      'u', U_I_A},
  {"i_das",        'r',  ACS1_A1, 3,          CAL16(11.03, -0.04),      'u', U_I_A},
  {"i_acs",        'r',  ACS1_A1, 5,          CAL16(12.5,  0.0),        'u', U_I_A},
  {"i_rec",        'r',  ACS1_A1, 7,          CAL16(10.75, -0.07),      'u', U_I_A},
  {"i_sc",         'r',  ACS1_A1, 9,          CAL16(10.89, -0.09),      'u', U_I_A},
  {"i_dgps",       'r',  ACS1_A1, 11,         CAL16(11.03, -0.19),      'u', U_I_A},
  {"i_el",         'r',  ACS1_A1, 13,         CAL16(12.5,  -0.02),      'u', U_I_A},
  {"i_piv",        'r',  ACS1_A1, 15,         CAL16(12.5, -0.05),       'u', U_I_A},
  {"i_rw",         'r',  ACS1_A1, 17,         CAL16(12.5, -0.09),       'u', U_I_A},
  {"i_step",       'r',  ACS1_A1, 19,         CAL16(12.5, -0.25),       'u', U_I_A},
  {"i_gy",         'r',  ACS1_A1, 21,         CAL16(11.0, -0.01),       'u', U_I_A},
  {"i_flc",        'r',  ACS1_A1, 23,         CAL16(12.5,  0.0),        'u', U_I_A},
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
  // ACS2_D 20-21 are unused
  {"dac2_ampl",    'w',  ACS2_D,  1,                 1.0,             0.0, 'u', U_NONE},
  {"dac_piv",      'w',  ACS2_D,  2,                 1.0,             0.0, 'u', U_NONE},
  //  {"dac3_ampl",    'w',  ACS2_D,  2,                1.0,             0.0, 'u', U_NONE},
  //  {"dac4_ampl",    'w',  ACS2_D,  3,                1.0,             0.0, 'u', U_NONE},
  //  {"dac5_ampl",    'w',  ACS2_D,  4,                1.0,             0.0, 'u', U_NONE},
  {"mask_gy",      'w',  ACS2_D, 13,                1.0,             0.0, 'u', U_NONE},
  {"control_lock", 'w',  ACS2_D, 14,                1.0,             0.0, 'u', U_NONE},
 // {"g_p_el",       'w',  ACS2_D, 20,                1.0,             0.0, 'u', U_NONE},
 // {"g_i_el",       'w',  ACS2_D, 21,                1.0,             0.0, 'u', U_NONE},
  {"g_p_az",       'w',  ACS2_D, 23,                1.0,             0.0, 'u', U_NONE},
  {"g_i_az",       'w',  ACS2_D, 24,                1.0,             0.0, 'u', U_NONE},
  {"fault_gy",     'r',  ACS2_D, 15,                1.0,             0.0, 'u', U_NONE},
  {"dac_el",       'r',   ACS2_D, 20,                 1.0,            0.0, 'u', U_NONE},
  {"p_term_el",    'r',   ACS2_D, 21,                1.0,        -32768.0, 'u', U_NONE},
  {"i_term_el",    'r',   ACS2_D, 22,                1.0,        -32768.0, 'u', U_NONE},
  {"error_el",     'r',   ACS2_D, 23,	    614.4e-6, 614.4*(-32768.0e-6), 'u', U_NONE},
  {"dac_rw",       'r',   ACS2_D, 24,                 1.0,            0.0, 'u', U_NONE},
  {"p_term_az",    'r',   ACS2_D, 25,                1.0,        -32768.0, 'u', U_NONE},
  {"i_term_az",    'r',   ACS2_D, 26,                1.0,        -32768.0, 'u', U_NONE},
  {"error_az",     'r',   ACS2_D, 27,	    614.4e-6, 614.4*(-32768.0e-6), 'u', U_NONE},
  {"limit_lock",   'r',   ACS2_D, 30,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Analog card */
  {"pch_piv_clin", 'r',  ACS2_A1,  1,           0.001343,          -43.54, 'u', U_NONE},
  {"roll_piv_clin",'r',  ACS2_A1,  3,           0.001413,          -44.69, 'u', U_NONE},
  {"t_piv_clin",   'r',  ACS2_A1,  5, 100.0*10.0/32768.0,     -100.0*10.0, 'u', U_NONE},
  {"pitch_of_clin",'r',  ACS2_A1,  7,            0.00143,           -47.0, 'u', U_NONE},
  {"roll_of_clin", 'r',  ACS2_A1,  9,              0.001,           -32.0, 'u', U_NONE},
  {"t_of_clin",    'r',  ACS2_A1, 11, 100.0*10.0/32768.0,     -100.0*10.0, 'u', U_NONE},
  {"x_mag",        'r',  ACS2_A1, 13,              MAGX_M,         MAGX_B, 'u', U_NONE},
  {"y_mag",        'r',  ACS2_A1, 15,              MAGY_M,         MAGY_B, 'u', U_NONE},
  {"z_mag",        'r',  ACS2_A1, 17,              MAGZ_M,         MAGZ_B, 'u', U_NONE},
  {"ifpm_hall",    'r',  ACS2_A1, 19,                1.0,             0.0, 'u', U_NONE},
  {"trig_thegood", 'r',  ACS2_A1, 23,		CAL16(1.0, 	     0.0), 'u',  U_V_V}, 
  {"trig_thebad",  'r',  ACS2_A1, 25,		CAL16(1.0, 	     0.0), 'u',  U_V_V}, 
  {"trig_theugly", 'r',  ACS2_A1, 27,		CAL16(1.0, 	     0.0), 'u',  U_V_V}, 
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

  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
#ifndef BOLOTEST
  {"ifyaw_1_gy",  'r',  ACS2_D,  0, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifyaw_2_gy",  'r',  ACS2_D,  2, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifroll_1_gy", 'r',  ACS2_D,  4, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifroll_2_gy", 'r',  ACS2_D,  6, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifel_2_gy",   'r',  ACS2_D,  8, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifel_1_gy",   'r',  ACS2_D, 10, -DGY32_TO_DPS,   DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"enc_table",   'r',  ACS2_D, 54,     360.0/144000.0,             0.0, 'U', U_P_DEG},
#endif

  {"az",          'w', LOOP2,   51,             LI2DEG,             0.0, 'U', U_P_DEG},
  {"el",          'w', LOOP2,   53,             LI2DEG,             0.0, 'U', U_P_DEG},

/* housekeeping channels */  /* TODO many can probably be slow */
  {"vr_still_2_hk",  'r', RTD_A1,  0, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_2_hk",    'r', RTD_A1,  2, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_2_hk",   'r', RTD_A1,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_2_hk",     'r', RTD_A1,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_2_hk",    'r', RTD_A1,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_2_hk",   'r', RTD_A1, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_2_hk",   'r', RTD_A1, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_2_hk",   'r', RTD_A1, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_still_3_hk",  'r', RTD_A1, 16, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_3_hk",    'r', RTD_A1, 18, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_3_hk",   'r', RTD_A1, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_3_hk",     'r', RTD_A1, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_3_hk",    'r', RTD_A1, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_3_hk",   'r', RTD_A1, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_3_hk",   'r', RTD_A1, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_3_hk",   'r', RTD_A1, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  //NB: (2011-09-25) 50Hz cals measured for board #1. other vr_ cals use mean
  {"vr_still_1_hk",  'r', RTD_A1, 32, CAL32C(       1.053,           0.0), 'U', U_V_V},
  {"vr_mux_1_hk",    'r', RTD_A1, 34, CAL32C(       1.065,           0.0), 'U', U_V_V},
  {"vr_ntd1_1_hk",   'r', RTD_A1, 36, CAL32N(       1.134,           0.0), 'U', U_V_V},
  {"vr_fp_1_hk",     'r', RTD_A1, 38, CAL32C(       1.052,           0.0), 'U', U_V_V},
  {"vr_strap_1_hk",    'r', RTD_A1, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_1_hk",   'r', RTD_A1, 42, CAL32N(       1.126,           0.0), 'U', U_V_V},
  {"vr_ntd3_1_hk",   'r', RTD_A1, 44, CAL32N(       1.122,           0.0), 'U', U_V_V},
  {"vr_ntd2_1_hk",   'r', RTD_A1, 46, CAL32N(       1.132,           0.0), 'U', U_V_V},

  {"vr_still_5_hk",  'r', RTD_A2,  0, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_5_hk",    'r', RTD_A2,  2, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_5_hk",   'r', RTD_A2,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_5_hk",     'r', RTD_A2,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_5_hk",    'r', RTD_A2,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_5_hk",   'r', RTD_A2, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_5_hk",   'r', RTD_A2, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_5_hk",   'r', RTD_A2, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_still_6_hk",  'r', RTD_A2, 16, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_6_hk",    'r', RTD_A2, 18, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_6_hk",   'r', RTD_A2, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_6_hk",     'r', RTD_A2, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_6_hk",    'r', RTD_A2, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_6_hk",   'r', RTD_A2, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_6_hk",   'r', RTD_A2, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_6_hk",   'r', RTD_A2, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_still_4_hk",  'r', RTD_A2, 32, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_4_hk",    'r', RTD_A2, 34, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_4_hk",   'r', RTD_A2, 36, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_4_hk",     'r', RTD_A2, 38, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_strap_4_hk",    'r', RTD_A2, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_4_hk",   'r', RTD_A2, 42, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_4_hk",   'r', RTD_A2, 44, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_4_hk",   'r', RTD_A2, 46, CAL32N(       1.129,           0.0), 'U', U_V_V},

  //TODO name the diode voltages. Must change derived.c to match
  {"vd_00_2_hk",   'r', DIOD_A1,  0, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_01_2_hk",   'r', DIOD_A1,  2, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_2_hk",   'r', DIOD_A1,  4, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_2_hk",   'r', DIOD_A1,  6, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_2_hk",   'r', DIOD_A1,  8, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_2_hk",   'r', DIOD_A1, 10, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_2_hk",   'r', DIOD_A1, 12, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_2_hk",   'r', DIOD_A1, 14, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_2_hk",   'r', DIOD_A1, 16, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_2_hk",   'r', DIOD_A1, 18, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_2_hk",   'r', DIOD_A1, 20, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_2_hk",   'r', DIOD_A1, 22, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_00_1_hk",   'r', DIOD_A1, 24, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_01_1_hk",   'r', DIOD_A1, 26, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_1_hk",   'r', DIOD_A1, 28, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_1_hk",   'r', DIOD_A1, 30, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_1_hk",   'r', DIOD_A1, 32, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_1_hk",   'r', DIOD_A1, 34, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_1_hk",   'r', DIOD_A1, 36, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_1_hk",   'r', DIOD_A1, 38, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_1_hk",   'r', DIOD_A1, 40, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_1_hk",   'r', DIOD_A1, 42, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_1_hk",   'r', DIOD_A1, 44, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_1_hk",   'r', DIOD_A1, 46, CAL32D(         1.0,           0.0), 'U', U_V_V},

  {"vd_00_4_hk",   'r', DIOD_A2,  0, CAL32D(         1.0,           0.0), 'U', U_V_V},
  //NB: (board #3 diode 00) and (board #4 diode 01) are swapped, inverted
  {"vd_00_3_hk",   'r', DIOD_A2,  2, CAL32D(        -1.0,           0.0), 'U', U_V_V},
  {"vd_02_4_hk",   'r', DIOD_A2,  4, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_4_hk",   'r', DIOD_A2,  6, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_4_hk",   'r', DIOD_A2,  8, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_4_hk",   'r', DIOD_A2, 10, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_4_hk",   'r', DIOD_A2, 12, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_4_hk",   'r', DIOD_A2, 14, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_4_hk",   'r', DIOD_A2, 16, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_4_hk",   'r', DIOD_A2, 18, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_4_hk",   'r', DIOD_A2, 20, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_4_hk",   'r', DIOD_A2, 22, CAL32D(         1.0,           0.0), 'U', U_V_V},
  //NB: (board #3 diode 00) and (board #4 diode 01) are swapped, inverted
  {"vd_01_4_hk",   'r', DIOD_A2, 24, CAL32D(        -1.0,           0.0), 'U', U_V_V},
  {"vd_01_3_hk",   'r', DIOD_A2, 26, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_3_hk",   'r', DIOD_A2, 28, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_3_hk",   'r', DIOD_A2, 30, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_3_hk",   'r', DIOD_A2, 32, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_3_hk",   'r', DIOD_A2, 34, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_3_hk",   'r', DIOD_A2, 36, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_3_hk",   'r', DIOD_A2, 38, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_3_hk",   'r', DIOD_A2, 40, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_3_hk",   'r', DIOD_A2, 42, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_3_hk",   'r', DIOD_A2, 44, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_3_hk",   'r', DIOD_A2, 46, CAL32D(         1.0,           0.0), 'U', U_V_V},

  {"vd_00_6_hk",   'r', DIOD_A3,  0, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_01_6_hk",   'r', DIOD_A3,  2, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_6_hk",   'r', DIOD_A3,  4, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_6_hk",   'r', DIOD_A3,  6, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_6_hk",   'r', DIOD_A3,  8, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_6_hk",   'r', DIOD_A3, 10, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_6_hk",   'r', DIOD_A3, 12, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_6_hk",   'r', DIOD_A3, 14, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_6_hk",   'r', DIOD_A3, 16, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_6_hk",   'r', DIOD_A3, 18, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_6_hk",   'r', DIOD_A3, 20, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_6_hk",   'r', DIOD_A3, 22, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_00_5_hk",   'r', DIOD_A3, 24, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_01_5_hk",   'r', DIOD_A3, 26, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_5_hk",   'r', DIOD_A3, 28, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_5_hk",   'r', DIOD_A3, 30, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_5_hk",   'r', DIOD_A3, 32, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_05_5_hk",   'r', DIOD_A3, 34, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_06_5_hk",   'r', DIOD_A3, 36, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_07_5_hk",   'r', DIOD_A3, 38, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_08_5_hk",   'r', DIOD_A3, 40, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_09_5_hk",   'r', DIOD_A3, 42, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_10_5_hk",   'r', DIOD_A3, 44, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_11_5_hk",   'r', DIOD_A3, 46, CAL32D(         1.0,           0.0), 'U', U_V_V},

  {"vd_00_hk",     'r',  HWP_A2,  0, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_01_hk",     'r',  HWP_A2,  2, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_02_hk",     'r',  HWP_A2,  4, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_03_hk",     'r',  HWP_A2,  6, CAL32D(         1.0,           0.0), 'U', U_V_V},
  {"vd_04_hk",     'r',  HWP_A2,  8, CAL32D(         1.0,           0.0), 'U', U_V_V},
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

  //TODO fix naming of HWP encoder channels
  {"hwp_00",       'r',HWP_A1,  0,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_01",       'r',HWP_A1,  2,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_02",       'r',HWP_A1,  4,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_03",       'r',HWP_A1,  6,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_04",       'r',HWP_A1,  8,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_05",       'r',HWP_A1, 10,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_06",       'r',HWP_A1, 12,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_07",       'r',HWP_A1, 14,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_08",       'r',HWP_A1, 16,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_09",       'r',HWP_A1, 18,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_10",       'r',HWP_A1, 20,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_11",       'r',HWP_A1, 22,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_12",       'r',HWP_A1, 24,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_13",       'r',HWP_A1, 26,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_14",       'r',HWP_A1, 28,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_15",       'r',HWP_A1, 30,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_16",       'r',HWP_A1, 32,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_17",       'r',HWP_A1, 34,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_18",       'r',HWP_A1, 36,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_19",       'r',HWP_A1, 38,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_20",       'r',HWP_A1, 40,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_21",       'r',HWP_A1, 42,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_22",       'r',HWP_A1, 44,CAL32(1.0,      0.0),      'U',      U_V_V},
  {"hwp_23",       'r',HWP_A1, 46,CAL32(1.0,      0.0),      'U',      U_V_V},

  //TODO the counts can be made narrow slow once debugging is complete
  {"enc_cnt_1_hwp",'w', LOOP0,  4,    LI2DEG,      0.0,      'U',      U_P_DEG},
  {"enc_cnt_2_hwp",'w', LOOP0,  6,    LI2DEG,      0.0,      'U',      U_P_DEG},
  {"enc_cnt_3_hwp",'w', LOOP0,  8,    LI2DEG,      0.0,      'U',      U_P_DEG},
  {"enc_cnt_4_hwp",'w', LOOP0, 10,    LI2DEG,      0.0,      'U',      U_P_DEG},
  {"enc_cnt_5_hwp",'w', LOOP0, 12,    LI2DEG,      0.0,      'U',      U_P_DEG},
  {"enc_cnt_6_hwp",'w', LOOP0, 14,    LI2DEG,      0.0,      'U',      U_P_DEG},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
#ifndef BOLOTEST
/* ACS1 Digital Card */
  {"heat_gy",      'w',  ACS1_D,  6,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Common Node */
  {"framenum",     'r',  ACS2_C,  1,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Digital Card */
  {"ifel_gy",      'r',   ACS2_D, 12,-GY16_TO_DPS,GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ifroll_gy",    'r',   ACS2_D, 13,-GY16_TO_DPS,GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ifyaw_gy",     'r',   ACS2_D, 14,-GY16_TO_DPS,GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
//  {"trigger_isc",  'w',   ACS2_D, 11,                 1.0,            0.0, 'u', U_NONE},
//  {"trigger_osc",  'w',   ACS2_D, 12,                 1.0,            0.0, 'u', U_NONE},
  //{"vel_req_el",   'w',   ACS2_D, 22, GY16_TO_DPS*0.1,-3276.8*GY16_TO_DPS, 'u', U_V_DPS},
  {"cos_el", 	   'w',    ACS2_D, 25, 1/32768.0, -1.0, 'u', U_NONE},
  {"sin_el", 	   'w',    ACS2_D, 26, 1/32768.0, -1.0, 'u', U_NONE}, 
//{"vel_req_az",   'w',   ACS2_D, 27, GY16_TO_DPS*0.1,-3276.8*GY16_TO_DPS, 'u', U_V_DPS},
  {"vel_req_az",   'w',   ACS2_D, 27, GY16_TO_DPS,-32768.0*GY16_TO_DPS, 'u', U_V_DPS},
  {"step_1_el",    'w',   ACS2_D, 29,     10000.0/32767.0, -10000.30518509, 'u', U_F_HZ},
  {"step_2_el",    'w',   ACS2_D, 30,     10000.0/32767.0, -10000.30518509, 'u', U_F_HZ},
  {"el_raw_1_enc", 'r',   ACS2_D, 56,               I2DEG,            -245.54, 'u', U_P_DEG},
  {"el_raw_2_enc", 'r',   ACS2_D, 57,               -I2DEG,            214.87, 'u', U_P_DEG},
  {"pulse_sc",     'r',  ACS2_A1, 50,                 1.0,            0.0, 'u', U_NONE},
  {"dps_table",    'w',    LOOP1, 34,  	     70.0/32767.0,            0.0, 's', U_V_DPS},

#endif
  {"heat_t_hk",    'w',  HWP_D,  53,            1.0,          0.0, 'u', U_NONE},
  {"heat_13_hk",   'w',  RTD_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"heat_45_hk",   'w',  RTD_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"heat_26_hk",   'w',  RTD_D,  52,            1.0,          0.0, 'u', U_NONE},

  {"vel_ser_rw",   'w', LOOP7,  0,     2400.0/65536.0,  0.0, 's',      U_V_DPS},
  {"vel_rw",       'w', LOOP7,  1,     2400.0/65536.0, -1200.0,  'u',  U_V_DPS},
 // {"el_raw_enc",   'w', LOOP7,  2,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"res_rw",       'w', LOOP7,  3,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"el_enc",       'w', LOOP2, 47,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"sigma_enc",    'w', LOOP2, 48,     I2DEG,      0.0,      'u',      U_NONE},
  {"chatter",      'w', LOOP7, 38,     1.0,        0.0,      'u',      U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,             1.0,                    0.0, 'u', U_NONE},
  {"polarity",    'w', DECOM,  2,             1.0,                    0.0, 'u', U_NONE},
  {"decom_unlock",'w', DECOM,  3,             1.0,                    0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
