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
#include "share/channels.h"
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
#define LOOP1	32, 0
#define LOOP2	33, 0
#define LOOP3	34, 0
#define LOOP4	35, 0
#define LOOP5	36, 0
#define LOOP6	37, 0
#define LOOP7	38, 0
#define LOOP8	39, 0
#define LOOP9	40, 0
#define DECOM	41, 0

/* Analog channel calibrations */
#define CAL16(m,b) ((m)*M_16PRE), ((b) + B_16PRE*(m)*M_16PRE)
#define CAL32(m,b) ((m)*M_32PRE), ((b) + B_32PRE*(m)*M_32PRE)
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T - 273.15)
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
  {"time",         'w', LOOP1,  0,                1.0,             0.0, 'U', U_NONE}, 
  {"time_usec",     'w', LOOP4, 58,                1.0,             0.0, 'U', U_NONE},
  {"time_sip",     'w', LOOP1,  2,                1.0,             0.0, 'U', U_NONE},
  {"time_dgps",    'w', LOOP1,  4,                1.0,             0.0, 'U', U_NONE},
  {"lst",          'w', LOOP1,  6,         1.0/3600.0,             0.0, 'U', U_NONE},
  {"ra_isc",       'w', LOOP1,  8,              LI2H,              0.0, 'U', U_NONE},
  {"dec_isc",      'w', LOOP1, 10,          LI2DEG/2.,            -90., 'U', U_NONE},
  {"parts_sched",   'w', LOOP1, 12,                1.0,             0.0, 'U', U_NONE},
  {"frame_thegood",'w', LOOP1, 32,                1.0,             0.0, 'U', U_NONE},
  {"lat",          'w', LOOP1, 38,             LI2DEG,             0.0, 'S', U_NONE},
  {"lon",          'w', LOOP1, 40,             LI2DEG,             0.0, 'S', U_NONE},
  {"sec_thegood",   'w', LOOP2, 20,                1.0,             0.0, 'U', U_NONE},
  {"usec_thegood",  'w', LOOP2, 60,                1.0,             0.0, 'U', U_NONE},
  {"ra",           'w', LOOP3,  4,               LI2H,             0.0, 'U', U_NONE},
  {"ra_osc",       'w', LOOP3, 26,               LI2H,             0.0, 'U', U_NONE},
  {"dec_osc",      'w', LOOP3, 28,          LI2DEG/2.,            -90., 'U', U_NONE},
  {"frame_thebad", 'w', LOOP3, 31,                1.0,             0.0, 'U', U_NONE},
  {"sec_thebad",   'w', LOOP3, 44,                1.0,             0.0, 'U', U_NONE},
  {"usec_thebad",  'w', LOOP3, 58,                1.0,             0.0, 'U', U_NONE},
  {"cycle_start",  'w', LOOP4, 24,                1.0,             0.0, 'U', U_NONE},
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
  {"ph_cnx_2_hk",   'w', RTD_A1,  0,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_ntd_2_hk",   'w', RTD_A1,  1,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_cnx_1_hk",   'w', RTD_A1,  4,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_ntd_1_hk",   'w', RTD_A1,  5,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_cnx_3_hk",   'w', RTD_A1,  8,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_ntd_3_hk",   'w', RTD_A1,  9,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_cnx_4_hk",   'w', RTD_A1, 12,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_ntd_4_hk",   'w', RTD_A1, 13,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_cnx_6_hk",   'w', RTD_A1, 16,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_ntd_6_hk",   'w', RTD_A1, 17,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_cnx_5_hk",   'w', RTD_A1, 20,  360.0/65536.0,          0.0,'u',U_PH_DEG},
  {"ph_ntd_5_hk",   'w', RTD_A1, 21,  360.0/65536.0,          0.0,'u',U_PH_DEG},

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

  /* LOOP1 0-13 are wide */
  {"pin_in_lock",  'w', LOOP1, 16,                1.0,             0.0, 'u', U_NONE},
  {"foc_res_thegood", 'w', LOOP1, 17,                1.0,             0.0, 'u', U_NONE},
  {"period_cal",   'w', LOOP1, 18,                .20,             0.0, 'u', U_NONE},
  {"status_eth",   'w', LOOP1, 19,                1.0,             0.0, 'u', U_NONE}, //Sun, ISC, OSC net status
  {"timeout",      'w', LOOP1, 20,                1.0,             0.0, 'u', U_NONE},
  {"az_sun",       'w', LOOP1, 21,              I2DEG,             0.0, 'u', U_D_DEG},
  {"lvdt_low_act", 'w', LOOP1, 22,                1.0,         -5000.0, 'u', U_NONE},
  {"status_mcc",   'w', LOOP1, 23,                1.0,             0.0, 'u', U_NONE}, //south_i_am, at_float, schedule, slot_sched
  {"cryostate",    'w', LOOP1, 24,                1.0,             0.0, 'u', U_NONE},
  {"upslot_sched", 'w', LOOP1, 25,                1.0,             0.0, 'u', U_NONE},  
  {"t_chip_flc",   'w', LOOP1, 26,               0.01,             0.0, 'u', U_NONE},
  {"declination_mag",'w', LOOP1, 27,              I2DEG,             0.0, 'u', U_D_DEG}, // magnetic declination
  {"veto_sensor",  'w', LOOP1, 28,                1.0,             0.0, 'u', U_NONE},
  {"level_on_bal", 'w', LOOP1, 29,           1./1990.13,             0.0, 'u', U_I_A},
  {"level_off_bal",'w', LOOP1, 30,           1./1990.13,             0.0, 'u', U_I_A},
  {"level_target_bal",'w', LOOP1, 31,           1./1990.13,            -5.0, 'u', U_I_A},
  /* LOOP1 32-33 are wide */
  /* LOOP1 34 is fast */
  /* LOOP1 35-36 are unused */
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
  {"mtol_isc",     'w', LOOP2, 18,        100./65536.,             0.0, 'u', U_NONE},
  {"qtol_isc",     'w', LOOP2, 19,        100./65536.,             0.0, 'u', U_NONE},
  /*P2 20-21 is wide */
  {"lrad_isc",     'w', LOOP2, 22,              I2DEG,             0.0, 'u', U_NONE},
  {"thresh_thegood",'w', LOOP2, 23,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"grid_thegood", 'w', LOOP2, 24,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 25 is unused */
  {"real_trig_osc",'w', LOOP2, 26,                1.0,             0.0, 's', U_NONE},
  {"mdist_thegood",'w', LOOP2, 27,                1.0,             0.0, 'u', U_NONE},
  {"nblobs_isc",   'w', LOOP2, 28,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 29-30 are unused */
  {"foc_off_osc",  'w', LOOP2, 31,                1.0,             0.0, 's', U_NONE},
  /* LOOP2 32 is unused */
  {"tol_isc",      'w', LOOP2, 33,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 34 is fast */
  /* LOOP2 35 is unused */
  {"offset_ifel_gy",  'w',LOOP2, 36,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ifroll_gy",'w',LOOP2, 37,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ifyaw_gy", 'w',LOOP2, 38,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"az_raw_mag",      'w',LOOP2, 39,            I2DEG,             0.0, 'u', U_D_DEG},
  /* LOOP2 39 is unused */
  {"sigma_mag",       'w',LOOP2, 40,            I2DEG,             0.0, 'u', U_NONE},
  {"az_dgps",         'w',LOOP2, 41,            I2DEG,             0.0, 'u', U_D_DEG},
  {"sigma_dgps",      'w',LOOP2, 42,            I2DEG,             0.0, 'u', U_NONE},
  {"lvdt_high_act",   'w',LOOP2, 43,              1.0,         -5000.0, 'u', U_NONE},
  {"az_isc",          'w',LOOP2, 44,            I2DEG,             0.0, 'u', U_NONE},
  {"el_isc",          'w',LOOP2, 45,            I2DEG,             0.0, 'u', U_NONE},
  {"mapsigma_thegood",'w',LOOP2, 46,           1.0/10.0,             0.0, 'u', U_NONE},
  // LOOP2 47-48 are fast
  {"pulse_cal",    'w', LOOP2, 49,               10.0,              0., 'u', U_NONE},
  /* LOOP2 50 is unused */
  /* LOOP2 51-54 are wide fast */
  {"sigma_clin",   'w', LOOP2, 55,              I2DEG,             0.0, 'u', U_NONE},
  {"az_mag",       'w', LOOP2, 56,              I2DEG,             0.0, 'u', U_D_DEG},
  {"blob03_x_thegood",'w', LOOP2,  57, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},
  {"blob03_y_thegood",'w', LOOP2,  58, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob03_f_thegood",'w', LOOP2,  59,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 60-61 are wide */
  {"blob03_s_thegood",'w', LOOP2,  62,          1.0/100.0,             0.0, 'u', U_NONE},
  {"blob04_x_thegood",'w', LOOP2,  63, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE},

  {"gain_osc",     'w', LOOP3,  0,        100./65536.,             0.0, 'u', U_NONE},
  {"blob04_y_thegood",'w', LOOP3,   1, CAM_WIDTH/SHRT_MAX,             0.0, 'u', U_NONE}, 
  {"blob04_f_thegood",'w', LOOP3,   2,                1.0,             0.0, 'u', U_NONE},
  {"blob04_s_thegood",'w', LOOP3,   3,          1.0/100.0,             0.0, 'u', U_NONE},
  /* LOOP3 4-5 are wide */
  {"offset_isc",   'w', LOOP3,  6,                1.0,             0.0, 's', U_NONE},
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
  /* LOOP3 26-29 are wide */
  {"foc_res_thebad",'w',LOOP3, 30,                1.0,             0.0, 'u', U_NONE},
  /* LOOP3 31-32 are wide */
  {"az_osc",       'w', LOOP3, 33,              I2DEG,             0.0, 'u', U_NONE},
  {"el_osc",       'w', LOOP3, 34,              I2DEG,             0.0, 'u', U_NONE},
  {"mapsigma_thebad",'w',LOOP3,35,           1.0/10.0,             0.0, 'u', U_NONE},
  {"move_tol_thebad",'w',LOOP3,36,                1.0,             0.0, 'u', U_NONE},
  {"exp_int_thebad",'w',LOOP3, 37,                1.0,             0.0, 'u', U_NONE},
  {"exp_time_thebad",'w',LOOP3,38,                1.0,             0.0, 'u', U_NONE},
  {"force_thebad", 'w', LOOP3, 39,                1.0,             0.0, 'u', U_NONE},
  {"mtol_osc",     'w', LOOP3, 40,        100./65536.,             0.0, 'u', U_NONE},
  {"qtol_osc",     'w', LOOP3, 41,        100./65536.,             0.0, 'u', U_NONE},
  {"offset_osc",   'w', LOOP3, 42,                1.0,             0.0, 's', U_NONE},
  {"lrad_osc",     'w', LOOP3, 43,              I2DEG,             0.0, 'u', U_NONE},
  /* LOOP3 44-45 are wide */
  {"thresh_thebad",'w',LOOP3, 46,         1.0/1000.0,             0.0, 'u', U_NONE},
  {"grid_thebad",  'w', LOOP3, 47,                1.0,             0.0, 'u', U_NONE},
  {"real_trig_isc",'w', LOOP3, 48,                1.0,             0.0, 's', U_NONE},
  {"foc_off_isc",  'w', LOOP3, 49,                1.0,             0.0, 's', U_NONE},
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
  {"i_hold_osc",   'w', LOOP3, 61,                1.0,             0.0, 'u', U_NONE},
  {"save_prd_osc", 'w', LOOP3, 62,               0.01,             0.0, 'u', U_NONE},
  {"y_off_osc",    'w', LOOP3, 63,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP4 0 is unused */
  {"pref_tp_sf",   'w', LOOP4,  1,                1.0,             0.0, 'u', U_NONE},
  /* LOOP4 3-5 are unused */
  {"maxblob_thegood",'w',LOOP4, 6,                1.0,             0.0, 'u', U_NONE},
  {"maxblob_thebad",'w',LOOP4,  7,                1.0,             0.0, 'u', U_NONE},
  {"bi0_fifo_size",'w', LOOP4,  8,             1./624,             0.0, 'u', U_NONE},
  {"plover",       'w', LOOP4,  9,                1.0,             0.0, 'u', U_NONE},
  {"ccd_t_thegood",'w', LOOP4, 10,          1.0/100.0,             0.0, 's', U_NONE},
  {"t_lens_isc",   'w', LOOP4, 11,            1./100.,         -273.15, 'u', U_NONE},
  {"t_heat_isc",   'w', LOOP4, 12,            1./100.,         -273.15, 'u', U_NONE},
  {"t_comp_isc",   'w', LOOP4, 13,            1./100.,         -273.15, 'u', U_NONE},
  {"pressure1_isc",'w', LOOP4, 14,           1./2000.,             0.0, 'u', U_NONE},
  {"ccd_t_thebad", 'w', LOOP4, 15,          1.0/100.0,             0.0, 's', U_NONE},
  {"t_lens_osc",   'w', LOOP4, 16,            1./100.,         -273.15, 'u', U_NONE},
  {"t_heat_osc",   'w', LOOP4, 17,            1./100.,         -273.15, 'u', U_NONE},
  {"t_comp_osc",   'w', LOOP4, 18,            1./100.,         -273.15, 'u', U_NONE},
  {"pressure1_osc",'w', LOOP4, 19,           1./2000.,             0.0, 'u', U_NONE},
  {"gain_isc",     'w', LOOP4, 20,        100./65536.,             0.0, 'u', U_NONE},
  {"jfet_set_on",  'w', LOOP4, 21,             1/100.,             0.0, 'u', U_NONE},
  {"jfet_set_off", 'w', LOOP4, 22,             1/100.,             0.0, 'u', U_NONE},
  /* LOOP4 23 is unused */
  /* LOOP4 24-25 are wide */
  {"cycle_state",  'w', LOOP4, 26,                1.0,             0.0, 'u', U_NONE},
  {"trig_type_isc",'w', LOOP4, 27,                1.0,             0.0, 'u', U_NONE},
  {"exposure_isc", 'w', LOOP4, 28,               100.,             0.0, 'u', U_NONE},
  {"trig_type_osc",'w', LOOP4, 29,                1.0,             0.0, 'u', U_NONE},
  {"exposure_osc", 'w', LOOP4, 30,               100.,             0.0, 'u', U_NONE},
  {"fieldrot_isc", 'w', LOOP4, 31,              I2DEG,             0.0, 's', U_NONE},
  {"fieldrot_osc", 'w', LOOP4, 32,              I2DEG,             0.0, 's', U_NONE},
  {"g_p_heat_gy",   'w', LOOP4, 33,                1.0,             0.0, 'u', U_NONE},
  {"g_i_heat_gy",   'w', LOOP4, 34,                1.0,             0.0, 'u', U_NONE},
  {"g_d_heat_gy",   'w', LOOP4, 35,                1.0,             0.0, 'u', U_NONE},
  {"t_set_gy",     'w', LOOP4, 36,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"h_age_gy",     'w', LOOP4, 37,                1.0,             0.0, 'u', U_NONE},
  {"h_hist_gy",    'w', LOOP4, 38,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"trim_pss",     'w', LOOP4, 39,              I2DEG,             0.0, 's', U_NONE},
  {"az_pss",       'w', LOOP4, 40,              I2DEG,             0.0, 'u', U_P_DEG},
  /* LOOP4 41-43 appear unused */
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
  {"gain_bal",     'w', LOOP4, 61,            1/1000.,             0.0, 'u', U_NONE},
  {"el_clin",      'w', LOOP4, 62,              I2DEG,             0.0, 'u', U_NONE},
  {"h_p",          'w', LOOP4, 63,              I2DEG,             0.0, 'u', U_NONE}, // scan height

  {"error_isc",    'w', LOOP5,  0,                 1.,             0.0, 'u', U_NONE},
  {"el_lut_clin",  'w', LOOP5,  1,              I2DEG,             0.0, 'u', U_NONE},
  {"rd_sigma_isc", 'w', LOOP5,  2,                1.0,             0.0, 'u', U_NONE},
  {"mks_lo_sip",   'w', LOOP5,  3,           0.327045,       -5.944902, 'u', U_NONE},
  /* LOOP5 4 is unused */
  {"error_osc",    'w', LOOP5,  5,                 1.,             0.0, 'u', U_NONE},
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
  //POT_LOCK = -100/(LOCK_MAX_POT-LOCK_MIN_POT)*pot_lock 
  //	+ 100*LOCK_MAX_POT/(LOCK_MAX_POT-LOCK_MIN_POT)
  {"pot_lock",     'w', LOOP5, 22,    -100.0/13368.0,1636800.0/13368.0, 'u', U_NONE},
  {"dith_el",      'w', LOOP5, 23,        0.5/32768.0,             0.0, 's', U_D_DEG},
  {"state_lock",   'w', LOOP5, 25,                1.0,             0.0, 'u', U_NONE},
  {"goal_lock",    'w', LOOP5, 26,                1.0,             0.0, 'u', U_NONE},
  {"seized_act",   'w', LOOP5, 27,                1.0,             0.0, 's', U_NONE},
  /* LOOP5 28 is fast */
  {"dith_step_p",  'w', LOOP5, 29,        0.1/32768.0,             0.0, 's', U_D_DEG},
  {"x_vel_stage",  'w', LOOP5, 30,                1.0,             0.0, 'u', U_NONE}, // not used in flight...
  {"x_stp_stage",  'w', LOOP5, 31,                1.0,             0.0, 'u', U_NONE},
  {"x_str_stage",  'w', LOOP5, 32,                1.0,             0.0, 'u', U_NONE},
  {"y_lim_stage",  'w', LOOP5, 33,                1.0,             0.0, 'u', U_NONE},
  /* LOOP5 34 is fast */
  /* LOOP5 35 is unused */
  {"y_stp_stage",  'w', LOOP5, 36,                1.0,             0.0, 'u', U_NONE},
  {"y_str_stage",  'w', LOOP5, 37,                1.0,             0.0, 'u', U_NONE},
  {"x_lim_stage",  'w', LOOP5, 38,                1.0,             0.0, 'u', U_NONE},
  {"y_vel_stage",  'w', LOOP5, 39,                1.0,             0.0, 'u', U_NONE},
  {"he4_lev_old",  'w', LOOP5, 40,          CAL16(1.0,            0.0), 'u', U_NONE},
  {"focus_isc",    'w', LOOP5, 41,                1.0,             0.0, 's', U_NONE},
  /* LOOP5 42-53 are wide */
  {"focus_osc",        'w',LOOP5, 54,             1.0,             0.0, 's', U_NONE},
  {"pitch_mag",        'w',LOOP5, 55,           I2DEG,             0.0, 'u', U_NONE},
  {"diskfree_isc",     'w',LOOP5, 56,             5.0,             0.0, 'u', U_NONE},
  {"diskfree_osc",     'w',LOOP5, 57,             5.0,             0.0, 'u', U_NONE},
  {"off_ifel_gy_isc",  'w',LOOP5, 58,     1.0/32768.0,             0.0, 's', U_NONE},
  {"off_ifel_gy_osc",  'w',LOOP5, 59,     1.0/32768.0,             0.0, 's', U_NONE},
  {"off_ifroll_gy_isc",'w',LOOP5, 60,     1.0/32768.0,             0.0, 's', U_NONE},
  {"off_ifroll_gy_osc",'w',LOOP5, 61,     1.0/32768.0,             0.0, 's', U_NONE},
  {"off_ifyaw_gy_isc", 'w',LOOP5, 62,     1.0/32768.0,             0.0, 's', U_NONE},
  {"off_ifyaw_gy_osc", 'w',LOOP5, 63,     1.0/32768.0,             0.0, 's', U_NONE},

  //this set of actuator channels used to be wide
  {"pos_lock",     'w', LOOP5, 18,                1.0,             0.0, 's', U_NONE},
  {"pos_0_act",    'w', LOOP5, 42,                1.0,		   0.0, 's', U_NONE},
  {"pos_1_act",    'w', LOOP5, 44,                1.0,             0.0, 's', U_NONE},
  {"pos_2_act",    'w', LOOP5, 46,                1.0,             0.0, 's', U_NONE},
  {"enc_0_act",    'w', LOOP5, 48,                1.0,		   0.0, 's', U_NONE},
  {"enc_1_act",    'w', LOOP5, 50,                1.0,             0.0, 's', U_NONE},
  {"enc_2_act",    'w', LOOP5, 52,                1.0,             0.0, 's', U_NONE},
  {"goal_sf",      'w', LOOP6, 30,                1.0,             0.0, 'u', U_NONE},
  {"focus_sf",     'w', LOOP6, 32,                1.0,             0.0, 's', U_NONE},

  {"maxslew_isc",  'w', LOOP6,  0,              I2DEG,             0.0, 'u', U_NONE},
  {"maxslew_osc",  'w', LOOP6,  1,              I2DEG,             0.0, 'u', U_NONE},
  /* LOOP6 2-19 are unused */
  {"pref_ts_sf",   'w', LOOP6, 20,                1.0,             0.0, 'u', U_NONE},
  {"spread_sf",    'w', LOOP6, 21,             1/500.,             0.0, 'u', U_NONE},
  {"acc_lock",     'w', LOOP6, 22,                1.0,             0.0, 'u', U_NONE},
  /* LOOP6 23 is unused */
  {"i_move_act",   'w', LOOP6, 25,                1.0,             0.0, 'u', U_NONE},
  {"i_hold_act",   'w', LOOP6, 26,                1.0,             0.0, 'u', U_NONE},
  {"vel_act",      'w', LOOP6, 27,                1.0,             0.0, 'u', U_NONE},
  {"acc_act",      'w', LOOP6, 28,                1.0,             0.0, 'u', U_NONE},
  {"i_move_lock",  'w', LOOP6, 29,                1.0,             0.0, 'u', U_NONE},
  /* LOOP6 30-33 are wide */
  {"i_hold_lock",  'w', LOOP6, 34,                1.0,             0.0, 'u', U_NONE},
  {"vel_lock",     'w', LOOP6, 35,               100.,             0.0, 'u', U_NONE},
  {"g_prime_sf",   'w', LOOP6, 36,               0.01,             0.0, 's', U_NONE},
  {"g_second_sf",  'w', LOOP6, 37,               0.01,             0.0, 's', U_NONE},
  {"step_sf",      'w', LOOP6, 38,                1.0,             0.0, 'u', U_NONE},
  {"wait_sf",      'w', LOOP6, 39,              1/30.,             0.0, 'u', U_NONE},
  {"mode_sf",      'w', LOOP6, 40,                1.0,             0.0, 'u', U_NONE},
  {"correction_sf",'w', LOOP6, 41,                1.0,             0.0, 's', U_NONE},
  {"age_sf",       'w', LOOP6, 42,              1/30.,             0.0, 'u', U_NONE},
  {"offset_sf",    'w', LOOP6, 43,                1.0,             0.0, 's', U_NONE},
  {"t_prime_sf",   'w', LOOP6, 44,         CAL16T(1.0,            0.0), 'u', U_T_C},
  {"t_second_sf",  'w', LOOP6, 45,         CAL16T(1.0,            0.0), 'u', U_T_C},
  {"flags_act",    'w', LOOP6, 50,                1.0,             0.0, 'u', U_NONE},
  {"lvdt_spread_act",'w', LOOP6, 55,              1.0,             0.0, 's', U_NONE},
  /* LOOP6 56-57 are wide */
  {"el_sun",       'w', LOOP6, 58,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP6 59 unused */
  {"mode_cal",     'w', LOOP6, 60,                1.0,             0.0, 'u', U_NONE},
  {"minblobs_isc", 'w', LOOP6, 61,                1.0,             0.0, 'u', U_NONE},
  {"minblobs_osc", 'w', LOOP6, 62,                1.0,             0.0, 'u', U_NONE},
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
  {"max_age_isc",  'w', LOOP7, 18,                1.0,             0.0, 'u', U_T_MS},
  {"max_age_osc",  'w', LOOP7, 19,                1.0,             0.0, 'u', U_T_MS},
  {"g_pe_piv",     'w', LOOP7, 20,                1.0,             0.0, 'u', U_NONE},
  {"g_pv_piv",     'w', LOOP7, 21,                1.0,             0.0, 'u', U_NONE},
  {"set_rw",       'w', LOOP7, 22,      200.0/32768.0,             0.0, 's', U_V_DPS},
  {"vel_dps_az",   'w', LOOP7, 23,       20.0/32768.0,             0.0, 's', U_V_DPS},
  /* LOOP7 24 unassigned */
  /* LOOP7 25-26 are unused */
  {"vel_ser_piv",  'w', LOOP7, 27,   2000.0/65536.0,         0.0, 's', U_V_DPS},
  {"vel_calc_piv", 'w', LOOP7, 28,        20.0/32768.0,             0.0, 's', U_V_DPS},
  {"age_isc",      'w', LOOP7, 29,                 1.0,             0.0, 'u', U_T_MS},
  {"age_osc",      'w', LOOP7, 30,                 1.0,             0.0, 'u', U_T_MS},
  {"overshoot_hwpr",'w', LOOP7, 31,                1.0,             0.0, 'u', U_NONE},
  {"drive_info_rw",'w', LOOP7, 32,                 1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_rw",'w', LOOP7, 33,              1.0,             0.0, 'u', U_NONE},
  {"drive_info_el",'w', LOOP7, 34,                 1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_el",'w', LOOP7, 35,              1.0,             0.0, 'u', U_NONE},
  {"drive_info_piv",'w', LOOP7, 36,                1.0,             0.0, 'u', U_NONE},
  {"drive_err_cts_piv",'w', LOOP7, 37,             1.0,             0.0, 'u', U_NONE},
  /* LOOP7 38 is fast narrow */
  {"vel_hwpr",         'w', LOOP7, 39,             1.0,             0.0, 'u', U_NONE},
  {"acc_hwpr",         'w', LOOP7, 40,             1.0,             0.0, 'u', U_NONE},
  {"i_move_hwpr",      'w', LOOP7, 41,             1.0,             0.0, 'u', U_NONE},
  {"i_hold_hwpr",      'w', LOOP7, 42,             1.0,             0.0, 'u', U_NONE},
  {"pos_hwpr",         'w', LOOP7, 43,             1.0,             0.0, 'u', U_NONE},
  {"enc_hwpr",         'w', LOOP7, 44,             1.0,             0.0, 'u', U_NONE},
  {"mode_bal",         'w', LOOP7, 45,             1.0,             0.0, 'u', U_NONE},
  {"pitch_raw_dgps",   'w', LOOP7, 46,           I2DEG,             0.0, 'u', U_P_DEG},
  {"roll_raw_dgps",    'w', LOOP7, 47,           I2DEG,             0.0, 'u', U_P_DEG},
  {"verbose_rw",       'w', LOOP7, 48,             1.0,             0.0, 'u', U_NONE},
  {"verbose_el",       'w', LOOP7, 49,             1.0,             0.0, 'u', U_NONE},
  {"verbose_piv",      'w', LOOP7, 52,             1.0,             0.0, 'u', U_NONE},
  {"p_rw_term_piv",    'w', LOOP7, 53,             1.0,             0.0, 's', U_NONE},
  {"p_err_term_piv",   'w', LOOP7, 54,             1.0,             0.0, 's', U_NONE},
  {"step_start_bias",  'w', LOOP7, 55,             0.5,             0.0, 'u', U_NONE},
  {"step_end_bias",    'w', LOOP7, 56,             0.5,             0.0, 'u', U_NONE},
  {"step_n_bias",      'w', LOOP7, 57,             1.0,             0.0, 'u', U_NONE},
  {"step_time_bias",   'w', LOOP7, 58,             1.0,             0.0, 'u', U_NONE},
  {"step_which_bias",  'w', LOOP7, 60,             1.0,             0.0, 'u', U_NONE},
  {"step_start_phase", 'w', LOOP7, 61,             0.5,             0.0, 'u', U_NONE},
  {"step_end_phase",   'w', LOOP7, 62,             0.5,             0.0, 'u', U_NONE},
  {"step_nsteps_phase",'w', LOOP7, 63,             1.0,             0.0, 'u', U_NONE},
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
  
  /* filtered LVDTs, rotated to motor positions */
  {"lvdt_0_act",  'w',  LOOP8, 13,   1.0,     0.0,   's', U_NONE},
  {"lvdt_1_act",  'w',  LOOP8, 14,   1.0,     0.0,   's', U_NONE},
  {"lvdt_2_act",  'w',  LOOP8, 15,   1.0,     0.0,   's', U_NONE},

  {"frict_off_piv",'w', LOOP8,  16,      2.0/65535.0,              0.0,   'u', U_NONE},
  {"frict_term_piv",'w',LOOP8,  17,      2.0/32767.0,              0.0,   's', U_NONE},
  {"frict_term_uf_piv",'w',LOOP8,18,     2.0/32767.0,              0.0,   's', U_NONE}, // For debugging remove later

  {"az_gy",        'w',   LOOP8, 19,       20.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_0_act", 'w',   LOOP8, 20,                1.0,             0.0, 'u', U_NONE},
  {"offset_1_act", 'w',   LOOP8, 21,                1.0,             0.0, 'u', U_NONE},
  {"offset_2_act", 'w',   LOOP8, 22,                1.0,             0.0, 'u', U_NONE},
  {"goal_0_act",   'w',   LOOP8, 23,                1.0,             0.0, 's', U_NONE},
  {"goal_1_act",   'w',   LOOP8, 24,                1.0,             0.0, 's', U_NONE},
  {"goal_2_act",   'w',   LOOP8, 25,                1.0,             0.0, 's', U_NONE},

  {"azraw_pss",   'w',   LOOP8, 26,              I2DEG,             0.0, 'u', U_P_DEG},
  {"elraw_pss",   'w',   LOOP8, 27,              I2DEG,             0.0, 'u', U_P_DEG},
  {"snr_pss1",     'w',   LOOP8, 28,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss2",     'w',   LOOP8, 29,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss3",     'w',   LOOP8, 30,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss4",     'w',   LOOP8, 31,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss5",     'w',   LOOP8, 32,            1/1000.,             0.0, 'u', U_NONE},
  {"snr_pss6",     'w',   LOOP8, 33,            1/1000.,             0.0, 'u', U_NONE},
  {"accel_az",     'w',   LOOP8, 34,          2.0/65536,             0.0, 'u', U_NONE},
  {"pos_focus_isc",'w',   LOOP8, 35,                1.0,             0.0, 'u', U_NONE},
  {"pos_focus_osc",'w',   LOOP8, 36,                1.0,             0.0, 'u', U_NONE},

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
  {"pos0_hwpr",      'w', LOOP9,  4,        1.0/65535.0,             0.0, 'u', U_NONE},
  {"pos1_hwpr",      'w', LOOP9,  5,        1.0/65535.0,             0.0, 'u', U_NONE},
  {"pos2_hwpr",      'w', LOOP9,  6,        1.0/65535.0,             0.0, 'u', U_NONE},
  {"pos3_hwpr",      'w', LOOP9,  7,        1.0/65535.0,             0.0, 'u', U_NONE},
  {"i_pos_rq_hwpr",  'w', LOOP9,  8,                1.0,             0.0, 'u', U_NONE},
  {"rate_tdrss",     'w', LOOP9,  9,                1.0,             0.0, 'u', U_RATE},
  {"rate_iridium",   'w', LOOP9,  10,                1.0,             0.0, 'u', U_RATE},
  {"read_wait_hwpr", 'w', LOOP9,  11,                1.0,             0.0, 'u', U_NONE},
  {"i_pos_hwpr",     'w', LOOP9,  12,                1.0,             0.0, 'u', U_NONE},
  {"stop_cnt_hwpr",  'w', LOOP9,  13,                1.0,             0.0, 'u', U_NONE},
  {"rel_move_hwpr",  'w', LOOP9,  14,                2.0,             0.0, 's', U_NONE},
  {"stat_control_hwpr",'w', LOOP9,15,                1.0,             0.0, 's', U_NONE},
  {"pot_targ_hwpr",  'w', LOOP9,  16,        1.0/65535.0,             0.0, 'u', U_NONE},
  {"mode_act",       'w', LOOP9,  17,                1.0,             0.0, 's', U_NONE},
  {"enc_targ_hwpr",  'w', LOOP9,  18,                1.0,             0.0, 's', U_NONE},
  {"enc_err_hwpr",   'w', LOOP9,  19,                1.0,             0.0, 's', U_NONE},
  {"dr_0_act",       'w', LOOP9,  20,                1.0,             0.0, 's', U_NONE},
  {"dr_1_act",       'w', LOOP9,  21,                1.0,             0.0, 's', U_NONE},
  {"dr_2_act",       'w', LOOP9,  22,                1.0,             0.0, 's', U_NONE},
  {"tol_act",        'w', LOOP9,  23,                1.0,             0.0, 'u', U_NONE},
  {"status_actbus",  'w', LOOP9,  24,                1.0,             0.0, 'u', U_NONE},
  {"pot_err_hwpr",   'w', LOOP9,  25,        1.0/32767.0,             0.0, 's', U_NONE},
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
  /* LOOP9 39-49 are unused */
  /* LOOP9 50-55 are wide */
  {"i_tot",         'w', LOOP9, 56,              1.0e-3,            0.0, 'u', U_I_A}, // sum of currents read through ACS1 A1
  {"t_set_theugly",     'w', LOOP9, 57,    (100.0/32768.0),             0.0, 'u', U_NONE},  
  {"cov_lim_dgps",   'w', LOOP9, 58,    (100.0/32768.0),             0.0, 'u', U_NONE},
  {"pot_hwpr",       'w', LOOP9, 59,        1.0/65535.0,             0.0, 'u', U_NONE},
  {"ant_e_dgps",     'w', LOOP9, 60,	      1.0/100.0,	     0.0, 's',U_NONE},
  {"ant_n_dgps",     'w', LOOP9, 61,	      1.0/100.0,	     0.0, 's',U_NONE},
  {"ant_u_dgps",     'w', LOOP9, 62,	      1.0/100.0,	     0.0, 's',U_NONE},
  {"ants_lim_dgps",   'w', LOOP9, 63,    (100.0/32768.0),            0.0, 'u', U_NONE},

#ifndef BOLOTEST
/* ACS1 Digital I/O card */
  {"latch0",       'w',  ACS1_D,  0,                1.0,             0.0, 'u', U_NONE},
  {"latch1",       'w',  ACS1_D,  1,                1.0,             0.0, 'u', U_NONE},
  {"switch_gy",    'w',  ACS1_D,  2,                1.0,             0.0, 'u', U_NONE},
  //this may be unused because of the new program
  //{"switch_charge",'w',  ACS1_D,  3,                1.0,             0.0, 'u', U_NONE},
  {"switch_misc",  'w',  ACS1_D,  4,                1.0,             0.0, 'u', U_NONE},
  {"bus_reset_act",'w',  ACS1_D,  5,                1.0,             0.0, 'u', U_NONE},

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
  {"t_bsc ",       'r',  ACS1_T1, 31,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_box_bal",    'r',  ACS1_T1, 33,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_pump_bal",   'r',  ACS1_T1, 35,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_el",         'r',  ACS1_T1, 37,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_array",      'r',  ACS1_T1, 39,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_sun",        'r',  ACS1_T1, 41,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_rw",         'r',  ACS1_T1, 43,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_earth",      'r',  ACS1_T1, 45,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_chin",       'r',  ACS1_T1, 47,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_port_pyr",       'r',  ACS1_T1, 49,         CAL16T(1.0, 0.0),         'u', U_T_C},

/* ACS2 Digital I/O card */
  {"v_pump_bal",    'w',  ACS2_D,  0,       3.91/13107.0,          -9.775, 'u', U_NONE},
  {"dac2_ampl",    'w',  ACS2_D,  1,                 1.0,             0.0, 'u', U_NONE},
  {"dac_piv",      'w',  ACS2_D,  2,                 1.0,             0.0, 'u', U_NONE},
  //  {"dac3_ampl",    'w',  ACS2_D,  2,                1.0,             0.0, 'u', U_NONE},
  //  {"dac4_ampl",    'w',  ACS2_D,  3,                1.0,             0.0, 'u', U_NONE},
  //  {"dac5_ampl",    'w',  ACS2_D,  4,                1.0,             0.0, 'u', U_NONE},
  {"mask_gy",      'w',  ACS2_D, 13,                1.0,             0.0, 'u', U_NONE},
  {"bits_vtx",     'w',  ACS2_D, 14,                1.0,             0.0, 'u', U_NONE},
  {"g_p_el",       'w',  ACS2_D, 20,                1.0,             0.0, 'u', U_NONE},
  {"g_i_el",       'w',  ACS2_D, 21,                1.0,             0.0, 'u', U_NONE},
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
  {"bits_bal",     'w',   ACS2_D, 28,                1.0,             0.0, 'u', U_NONE},
  {"trig_s_theugly",  'r',   ACS2_D, 51,                1.0,             0.0, 'u', U_NONE},
  {"trig_l_theugly",  'r',   ACS2_D, 52,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Analog card */
  {"pch_pyr_clin", 'r',  ACS2_A1,  1,           0.001343,          -47.09, 'u', U_NONE},
  {"roll_pyr_clin",'r',  ACS2_A1,  3,           0.001413,          -45.86, 'u', U_NONE},
  {"t_pyr_clin",   'r',  ACS2_A1,  5,          100.0*10.0/32768.0,    -100.0*10.0, 'u', U_NONE},
  {"xel_if_clin",  'r',  ACS2_A1,  7,         0.00546739,      -25.*6.144, 'u', U_NONE},
  {"el_raw_if_clin",'r', ACS2_A1,  9,                1.0,             0.0, 'u', U_NONE},
  //  {"el_raw_if_clin",'r', ACS2_A1,  9,         0.00546739,         -133.78, 'u', U_NONE},
  {"t_if_clin",    'r',  ACS2_A1, 11,            100.0*10.0/32768.0,    -100.0*10.0, 'u', U_NONE},
  {"x_mag",        'r',  ACS2_A1, 13,              MAGX_M,         MAGX_B, 'u', U_NONE},
  {"y_mag",        'r',  ACS2_A1, 15,              MAGY_M,         MAGY_B, 'u', U_NONE},
  {"z_mag",        'r',  ACS2_A1, 17,              MAGZ_M,         MAGZ_B, 'u', U_NONE},
  {"ifpm_hall",    'r',  ACS2_A1, 19,                1.0,             0.0, 'u', U_NONE},
  {"lvdt_65_act",  'r',  ACS2_A1, 21,   LVDT65_ADC_TO_ENC,     LVDT65_ZERO,   'u', U_NONE},
  {"lvdt_63_act",  'r',  ACS2_A1, 23,   LVDT63_ADC_TO_ENC,     LVDT63_ZERO,   'u', U_NONE},
  {"lvdt_64_act",  'r',  ACS2_A1, 25,   LVDT64_ADC_TO_ENC,     LVDT64_ZERO,   'u', U_NONE},
  //{"bsc_trig",	   'r',	 ACS2_A1, 27,		      1.0,	      0.0, 'u', U_NONE},
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
  {"ifroll_1_gy", 'r',  ACS2_D,  4, DGY32_TO_DPS, -1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifroll_2_gy", 'r',  ACS2_D,  6, DGY32_TO_DPS, -1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifel_2_gy",   'r',  ACS2_D,  8, DGY32_TO_DPS, -1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifel_1_gy",   'r',  ACS2_D, 10, DGY32_TO_DPS, -1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"enc_table",   'r',  ACS2_D, 54,     360.0/144000.0,             0.0, 'U', U_P_DEG},
#endif

  {"az",          'w', LOOP2,   51,             LI2DEG,             0.0, 'U', U_P_DEG},
  {"el",          'w', LOOP2,   53,             LI2DEG,             0.0, 'U', U_P_DEG},

/* housekeeping channels */  /* TODO many can probably be slow */
  {"vr_still_2_hk",  'r', RTD_A1,  0, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_2_hk",    'r', RTD_A1,  2, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_2_hk",   'r', RTD_A1,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_2_hk",     'r', RTD_A1,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ssa_2_hk",    'r', RTD_A1,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_2_hk",   'r', RTD_A1, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_2_hk",   'r', RTD_A1, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_2_hk",   'r', RTD_A1, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_still_3_hk",  'r', RTD_A1, 16, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_3_hk",    'r', RTD_A1, 18, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_3_hk",   'r', RTD_A1, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_3_hk",     'r', RTD_A1, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ssa_3_hk",    'r', RTD_A1, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_3_hk",   'r', RTD_A1, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_3_hk",   'r', RTD_A1, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_3_hk",   'r', RTD_A1, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  //NB: (2011-09-25) 50Hz cals measured for board #1. other vr_ cals use mean
  {"vr_still_1_hk",  'r', RTD_A1, 32, CAL32C(       1.053,           0.0), 'U', U_V_V},
  {"vr_mux_1_hk",    'r', RTD_A1, 34, CAL32C(       1.065,           0.0), 'U', U_V_V},
  {"vr_ntd1_1_hk",   'r', RTD_A1, 36, CAL32N(       1.134,           0.0), 'U', U_V_V},
  {"vr_fp_1_hk",     'r', RTD_A1, 38, CAL32C(       1.052,           0.0), 'U', U_V_V},
  {"vr_ssa_1_hk",    'r', RTD_A1, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_1_hk",   'r', RTD_A1, 42, CAL32N(       1.126,           0.0), 'U', U_V_V},
  {"vr_ntd3_1_hk",   'r', RTD_A1, 44, CAL32N(       1.122,           0.0), 'U', U_V_V},
  {"vr_ntd2_1_hk",   'r', RTD_A1, 46, CAL32N(       1.132,           0.0), 'U', U_V_V},

  {"vr_still_5_hk",  'r', RTD_A2,  0, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_5_hk",    'r', RTD_A2,  2, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_5_hk",   'r', RTD_A2,  4, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_5_hk",     'r', RTD_A2,  6, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ssa_5_hk",    'r', RTD_A2,  8, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_5_hk",   'r', RTD_A2, 10, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_5_hk",   'r', RTD_A2, 12, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_5_hk",   'r', RTD_A2, 14, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_still_6_hk",  'r', RTD_A2, 16, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_6_hk",    'r', RTD_A2, 18, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_6_hk",   'r', RTD_A2, 20, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_6_hk",     'r', RTD_A2, 22, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ssa_6_hk",    'r', RTD_A2, 24, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd4_6_hk",   'r', RTD_A2, 26, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd3_6_hk",   'r', RTD_A2, 28, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_ntd2_6_hk",   'r', RTD_A2, 30, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_still_4_hk",  'r', RTD_A2, 32, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_mux_4_hk",    'r', RTD_A2, 34, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ntd1_4_hk",   'r', RTD_A2, 36, CAL32N(       1.129,           0.0), 'U', U_V_V},
  {"vr_fp_4_hk",     'r', RTD_A2, 38, CAL32C(       1.055,           0.0), 'U', U_V_V},
  {"vr_ssa_4_hk",    'r', RTD_A2, 40, CAL32C(       1.055,           0.0), 'U', U_V_V},
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
  {"trigger_isc",  'w',   ACS2_D, 11,                 1.0,            0.0, 'u', U_NONE},
  {"trigger_osc",  'w',   ACS2_D, 12,                 1.0,            0.0, 'u', U_NONE},
  {"vel_req_el",   'w',   ACS2_D, 22, GY16_TO_DPS*0.1,-3276.8*GY16_TO_DPS, 'u', U_V_DPS},
  {"cos_el",       'w',   ACS2_D, 25,         1.0/32768.0,           -1.0, 'u', U_NONE},
  {"sin_el",       'w',   ACS2_D, 26,         1.0/32768.0,           -1.0, 'u', U_NONE},
//{"vel_req_az",   'w',   ACS2_D, 27, GY16_TO_DPS*0.1,-3276.8*GY16_TO_DPS, 'u', U_V_DPS},
  {"vel_req_az",   'w',   ACS2_D, 27, GY16_TO_DPS,-32768.0*GY16_TO_DPS, 'u', U_V_DPS},
  {"pulse_sc",     'r',   ACS2_D, 50,                 1.0,            0.0, 'u', U_NONE},
  //{"trig_s_theugly",  'r',   ACS2_D, 51,                1.0,             0.0, 'u', U_NONE},
  //{"trig_l_theugly",  'r',   ACS2_D, 52,                1.0,             0.0, 'u', U_NONE},
  {"dps_table",    'w',    LOOP1, 34,  	     70.0/32767.0,            0.0, 's', U_V_DPS},

#endif
  {"heat_13_hk",   'w',  RTD_D,  50,            1.0,          0.0, 'u', U_NONE},
  {"heat_45_hk",   'w',  RTD_D,  51,            1.0,          0.0, 'u', U_NONE},
  {"heat_26_hk",   'w',  RTD_D,  52,            1.0,          0.0, 'u', U_NONE},

  {"x_stage",      'w', LOOP5, 28,     2.0,        0.0,      'u',      U_NONE},
  {"y_stage",      'w', LOOP5, 34,     2.0,        0.0,      'u',      U_NONE},
  {"vel_ser_rw",   'w', LOOP7,  0,     2400.0/65536.0,  0.0, 's',      U_V_DPS},
  {"vel_rw",       'w', LOOP7,  1,     2400.0/65536.0, -1200.0,  'u',  U_V_DPS},
  {"el_raw_enc",   'w', LOOP7,  2,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"res_rw",       'w', LOOP7,  3,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"el_enc",       'w', LOOP2, 47,     I2DEG,      0.0,      'u',      U_P_DEG},
  {"sigma_enc",    'w', LOOP2, 48,     I2DEG,      0.0,      'u',      U_NONE},
  {"chatter",      'w', LOOP7, 38,     1.0,        0.0,      'u',      U_NONE},
  {"mcp_frame",    'w', LOOP2, 34,     1.0,        0.0,      'u',      U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,             1.0,                    0.0, 'u', U_NONE},
  {"polarity",    'w', DECOM,  2,             1.0,                    0.0, 'u', U_NONE},
  {"decom_unlock",'w', DECOM,  3,             1.0,                    0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
