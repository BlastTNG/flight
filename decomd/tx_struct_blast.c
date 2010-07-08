/* tx_struct.c: contains the channel specificiation lists
 *
 * This software is copyright (C) 2002-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MOVE, OR DELETE *ANY* CHANNELS IN THIS FILE YOU *MUST*
 * RECOMPILE AND RESTART THE DECOM DAEMON (DECOMD) ON ARWEN!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include <limits.h>
#include "channels.h"
#include "bbc_pci.h"
#ifdef __MCP__
#include "sbsc_protocol.h"
#endif

/* card name to (node number, bus number) mapping */
#define ACS1_C	 0, 0  /* C denotes a common motherboard node */
#define ACS1_D	 1, 0  /* D denotes a digital daughter card */
#define ACS1_A1	 2, 0  /* A deontes a node for analog daughter card */
#define ACS1_T1	 3, 0  /* T denotes an AD590 thermometry daughter card */
#define ACS2_C	 4, 0
#define ACS2_D	 5, 0
#define ACS2_A1	 6, 0
//ACS2_unused	 7, 0
#define BIAS_C	 8, 0
#define BIAS_D	 9, 0
#define BIAS_T1	10, 0
//BIAS_unused	11, 0
#define CRYO_C	12, 0
#define CRYO_A1	13, 0
#define CRYO_A2	14, 0
//CRYO_unused	15, 0
#define DAS1_C	16, 0
#define DAS1_A1	17, 0
#define DAS1_A2	18, 0
#define DAS1_A3	19, 0
#define DAS2_C	20, 0
#define DAS2_A1	21, 0
#define DAS2_A2	22, 0
#define DAS2_A3	23, 0
#define DAS3_C	24, 0
#define DAS3_A1	25, 0
#define DAS3_A2	26, 0
#define DAS3_A3	27, 0
#define DAS4_C	28, 0
#define DAS4_A1	29, 0
#define DAS4_A2	30, 0
#define DAS4_A3	31, 0
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

#define CAL16(m,b) ((m)*M_16PRE), ((b) + B_16PRE*(m)*M_16PRE)
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T - 273.15)

#define U_NONE  "","" 
#define U_T_C   "Temperature","^oC"
#define U_V_DPS "Rate","^o/s"
#define U_V_MPS "Speed","m/s"
#define U_V_KPH "Speed","km/hr"
#define U_ALT_M "Altitude","m"
#define U_P_DEG "Position","^o"
#define U_LA_DEG "Latitude","^o"
#define U_LO_DEG "Longitude","^o"
#define U_D_DEG "Direction","^o"
#define U_V_V	"Voltage","V"
#define U_I_A   "Current","A"
#define U_T_MS  "Time","ms"
#define U_R_O   "Resistance","Ohms"

/*******************************************************************************
 * TODO The channel list needs updating! Some of this is still from BLAST06
 *
 * Anything allocated to one of the following TMP nodes must be reassigned
 * to the correct new node above, or it must be deleted
 *
 * LOOP nodes might also need cleanup. Their channels have all been carried
 * forward, but some may no longer be needed
 *
 * TMP nodes are only so mcp doesn't barf during development.
 * 
 * ALSO! If you delete anything from the TMP nodes be sure that there isn't a 
 * corresponding entry in slow_dl.c
 * !!!!!!!!!!!!!!!!!!!!!!!!!! DO NOT FLY WITH TMP NODES !!!!!!!!!!!!!!!!!!!!!!!!
 ******************************************************************************/
//#define	TMP1	50, 0	//all channels formerly on ACS0
//#define	TMP2	51, 0	//all channels formerly on ACS1
//#define TMP3	52, 0	//all channels formerly on ACS2
//#define TMP4	53, 0	//all channels formerly on ACS3
//#define TMP5    54, 0   //all channels formerly on CRYO
//#define TMP6	55, 0	//all channels formerly on BIAS

/* read and write channel 56 on all boards reserved for ADC Sync */

struct ChannelStruct WideSlowChannels[] = {
  {"tr_he3_fridge", 'r', CRYO_A1,  0,  CRYO_HE3_FRIDGE_M,CRYO_HE3_FRIDGE_B,'U', U_R_O},
  {"tr_m5",         'r', CRYO_A1,  2,          CRYO_M5_M,       CRYO_M5_B, 'U', U_R_O},
//{"cryo_a1_02",    'r', CRYO_A1,  4,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_03",    'r', CRYO_A1,  6,                1.0,             0.0, 'U', U_NONE},
  {"tr_m4",         'r', CRYO_A1,  8,          CRYO_M4_M,       CRYO_M4_B, 'U', U_R_O},
//{"cryo_a1_05",    'r', CRYO_A1, 10,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_06",    'r', CRYO_A1, 12,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_07",    'r', CRYO_A1, 14,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_08",    'r', CRYO_A1, 16,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_09",    'r', CRYO_A1, 18,                1.0,             0.0, 'U', U_NONE},
  {"tr_hwpr",       'r', CRYO_A1, 20,        CRYO_HWPR_M,     CRYO_HWPR_B, 'U', U_R_O},
  {"tr_horn_500",   'r', CRYO_A1, 22,    CRYO_HORN_500_M, CRYO_HORN_500_B, 'U', U_R_O},
//{"cryo_a1_12",    'r', CRYO_A1, 24,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_13",    'r', CRYO_A1, 26,                1.0,             0.0, 'U', U_NONE},
  {"tr_horn_350",   'r', CRYO_A1, 28,    CRYO_HORN_350_M, CRYO_HORN_350_B, 'U', U_R_O},
//{"cryo_a1_15",    'r', CRYO_A1, 30,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_16",    'r', CRYO_A1, 32,                1.0,             0.0, 'U', U_NONE},
  {"tr_horn_250",   'r', CRYO_A1, 34,    CRYO_HORN_250_M, CRYO_HORN_250_B, 'U', U_R_O},
//{"cryo_a1_18",    'r', CRYO_A1, 36,                1.0,             0.0, 'U', U_NONE},
//{"cryo_a1_19",    'r', CRYO_A1, 38,                1.0,             0.0, 'U', U_NONE},
  {"tr_300mk_strap",'r', CRYO_A1, 40, CRYO_300MK_STRAP_M,CRYO_300MK_STRAP_B,'U',U_R_O},
//{"cryo_a1_21",    'r', CRYO_A1, 42,                1.0,             0.0, 'U', U_NONE},
  {"tr_he4_pot",    'r', CRYO_A1, 44,     CRYO_HE4_POT_M,  CRYO_HE4_POT_B, 'U', U_R_O},
  {"tr_optbox_filt",'r', CRYO_A1, 46, CRYO_OPTBOX_FILT_M,CRYO_OPTBOX_FILT_B,'U',U_R_O},
//{"cryo_a1_24",    'r', CRYO_A1, 48,                1.0,             0.0, 'U', U_NONE},
  {"he4_lev",       'r', CRYO_A2,  0,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"i_charcoal",    'r', CRYO_A2,  2,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"i_cal_lamp",    'r', CRYO_A2,  4,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"i_hs_pot",      'r', CRYO_A2,  6,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"i_hs_char",     'r', CRYO_A2,  8,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"td_ln",         'r', CRYO_A2, 10,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"i_300mk",       'r', CRYO_A2, 12,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
//{"cryo_a2_07",    'r', CRYO_A2, 14,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"td_vcs_filt",   'r', CRYO_A2, 16,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"hwpr_enc_cryo", 'r', CRYO_A2, 18,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"i_jfet",        'r', CRYO_A2, 20,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
//{"cryo_a2_11",    'r', CRYO_A2, 22,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"td_hs_charcoal",'r', CRYO_A2, 24,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"td_lhe_filt",   'r', CRYO_A2, 26,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"td_lhe",        'r', CRYO_A2, 28,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
//{"cryo_a2_15",    'r', CRYO_A2, 30,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  //CRYO_A2 CH16 is narrow fast
//{"cryo_a2_17",    'r', CRYO_A2, 34,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
//{"cryo_a2_18",    'r', CRYO_A2, 36,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
//{"cryo_a2_19",    'r', CRYO_A2, 38,          CRYO_A2_M,       CRYO_A2_B, 'U', U_V_V},
  {"td_vcs_jfet",   'r', CRYO_A2, 40,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"td_jfet",       'r', CRYO_A2, 42,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"td_hs_pot",     'r', CRYO_A2, 44,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"td_charcoal",   'r', CRYO_A2, 46,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},
  {"td_ln_filt",    'r', CRYO_A2, 48,           CRYO_D_M,        CRYO_D_B, 'U', U_V_V},

  {"time",         'w', LOOP1,  0,                1.0,             0.0, 'U', U_NONE}, // could delete and rename cpu_usec to time_usec,
  {"time_usec",     'w', LOOP4, 58,                1.0,             0.0, 'U', U_NONE}, // but look in derived.c at Time...
  {"time_sip",     'w', LOOP1,  2,                1.0,             0.0, 'U', U_NONE},
  {"time_dgps",    'w', LOOP1,  4,                1.0,             0.0, 'U', U_NONE},
  {"lst",          'w', LOOP1,  6,         1.0/3600.0,             0.0, 'U', U_NONE},
  {"ra_isc",       'w', LOOP1,  8,              LI2H,              0.0, 'U', U_NONE},
  {"dec_isc",      'w', LOOP1, 10,          LI2DEG/2.,            -90., 'U', U_NONE},
  //{"FREEFIELD",    'w', LOOP1, 12,                1.0,             0.0, 'U', U_NONE}, // free field... use me
  {"framenum_isc", 'w', LOOP1, 32,                1.0,             0.0, 'U', U_NONE},
  {"lat",          'w', LOOP1, 38,             LI2DEG,             0.0, 'S', U_NONE},
  {"lon",          'w', LOOP1, 40,             LI2DEG,             0.0, 'S', U_NONE},
  {"state_isc",    'w', LOOP2, 20,                1.0,             0.0, 'U', U_NONE},
  {"mcpnum_isc",   'w', LOOP2, 60,                1.0,             0.0, 'U', U_NONE},
  {"ra",           'w', LOOP3,  4,               LI2H,             0.0, 'U', U_NONE},
  {"ra_osc",       'w', LOOP3, 26,               LI2H,             0.0, 'U', U_NONE},
  {"dec_osc",      'w', LOOP3, 28,          LI2DEG/2.,            -90., 'U', U_NONE},
  {"framenum_osc", 'w', LOOP3, 31,                1.0,             0.0, 'U', U_NONE},
  {"state_osc",    'w', LOOP3, 44,                1.0,             0.0, 'U', U_NONE},
  {"mcpnum_osc",   'w', LOOP3, 58,                1.0,             0.0, 'U', U_NONE},
  {"cycle_start",  'w', LOOP4, 24,                1.0,             0.0, 'U', U_NONE},
  {"dec",          'w', LOOP5,  6,             LI2DEG,             0.0, 'S', U_NONE},
  {"lst_sched",    'w', LOOP6, 56,                1.0,             0.0, 'U', U_NONE},  // ls day
  {"sbsc_frame",    'w', LOOP9, 50,                1.0,             0.0, 'U', U_NONE},
  {"sbsc_sec",      'w', LOOP9, 52,                1.0,             0.0, 'U', U_NONE},
  {"sbsc_usec",     'w', LOOP9, 54,                1.0,             0.0, 'U', U_NONE},
  //derived channel sbsc_time adds these together

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
  {"status08",     'r',  BIAS_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status09",     'r',  BIAS_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"status10",     'r', BIAS_T1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status12",     'r',  CRYO_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status13",     'r', CRYO_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status14",     'r', CRYO_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status16",     'r',  DAS1_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status17",     'r', DAS1_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status18",     'r', DAS1_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status19",     'r', DAS1_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"status20",     'r',  DAS2_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status21",     'r', DAS2_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status22",     'r', DAS2_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status23",     'r', DAS2_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"status24",     'r',  DAS3_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status25",     'r', DAS3_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status26",     'r', DAS3_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status27",     'r', DAS3_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"status28",     'r',  DAS4_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"status29",     'r', DAS4_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"status30",     'r', DAS4_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"status31",     'r', DAS4_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync00",       'w',  ACS1_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync01",       'w',  ACS1_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync02",       'w', ACS1_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync03",       'w', ACS1_T1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync04",       'w',  ACS2_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync05",       'w',  ACS2_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync06",       'w', ACS2_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync08",       'w',  BIAS_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync09",       'w',  BIAS_D, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync10",       'w', BIAS_T1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync12",       'w',  CRYO_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync13",       'w', CRYO_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync14",       'w', CRYO_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync16",       'w',  DAS1_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync17",       'w', DAS1_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync18",       'w', DAS1_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync19",       'w', DAS1_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync20",       'w',  DAS2_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync21",       'w', DAS2_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync22",       'w', DAS2_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync23",       'w', DAS2_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync24",       'w',  DAS3_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync25",       'w', DAS3_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync26",       'w', DAS3_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync27",       'w', DAS3_A3, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync28",       'w',  DAS4_C, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync29",       'w', DAS4_A1, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync30",       'w', DAS4_A2, 63,                1.0,             0.0, 'u', U_NONE},
  {"sync31",       'w', DAS4_A3, 63,                1.0,             0.0, 'u', U_NONE},

  //set lock-in phase for DAS channels
  {"n13_phase",      'w', CRYO_A1,  8,                0.5,             0.0, 'u', U_NONE},
  {"n17_phase",      'w', DAS1_A1,  8,                0.5,             0.0, 'u', U_NONE},
  {"n18_phase",      'w', DAS1_A2,  8,                0.5,             0.0, 'u', U_NONE},
  {"n19_phase",      'w', DAS1_A3,  8,                0.5,             0.0, 'u', U_NONE},
  {"n21_phase",      'w', DAS2_A1,  8,                0.5,             0.0, 'u', U_NONE},
  {"n22_phase",      'w', DAS2_A2,  8,                0.5,             0.0, 'u', U_NONE},
  {"n23_phase",      'w', DAS2_A3,  8,                0.5,             0.0, 'u', U_NONE},
  {"n25_phase",      'w', DAS3_A1,  8,                0.5,             0.0, 'u', U_NONE},
  {"n26_phase",      'w', DAS3_A2,  8,                0.5,             0.0, 'u', U_NONE},
  {"n27_phase",      'w', DAS3_A3,  8,                0.5,             0.0, 'u', U_NONE},
  {"n29_phase",      'w', DAS4_A1,  8,                0.5,             0.0, 'u', U_NONE},
  {"n30_phase",      'w', DAS4_A2,  8,                0.5,             0.0, 'u', U_NONE},
  {"n31_phase",      'w', DAS4_A3,  8,                0.5,             0.0, 'u', U_NONE},
  //generic names should be updated. Also bitfields could be useful for dig outs
  {"ampl_500_bias",'w',  BIAS_D,  0,                0.5,             0.0, 'u', U_NONE},
  {"ampl_350_bias",'w',  BIAS_D,  1,                0.5,             0.0, 'u', U_NONE},
  {"ampl_250_bias",'w',  BIAS_D,  2,                0.5,             0.0, 'u', U_NONE},
  {"ampl_rox_bias",'w',  BIAS_D,  3,                0.5,             0.0, 'u', U_NONE},
  {"ampl_x_bias",  'w',  BIAS_D,  4,                1.0,             0.0, 'u', U_NONE},
  {"dig21_das",    'w',  BIAS_D,  5,                1.0,             0.0, 'u', U_NONE},
  //dig43_das is fast, for calibration pulse
  {"dig65_das",    'w',  BIAS_D,  7,                1.0,             0.0, 'u', U_NONE},
  {"ramp_ena_bias",'w',  BIAS_D,  8,                1.0,             0.0, 'u', U_NONE},
  {"ramp_ampl_bias",'r',  BIAS_D,  0,                1.0,             0.0, 'u', U_NONE},

  /* the following 9 AD590s, except t_das all had calibrated offsets */
  /* these are all carry dover from the previous BIAS card (TMP6) */
  {"t_padcdc_rec", 'r', BIAS_T1,  1, CAL16T(1.0,                    0.0), 'u', U_T_C},
  {"t_pauram_rec", 'r', BIAS_T1,  3, CAL16T(1.0,                    0.0), 'u', U_T_C},
  {"t_hkdcdc_rec", 'r', BIAS_T1,  5, CAL16T(1.0,                    0.0), 'u', U_T_C},
  {"t_2_prime",    'r', BIAS_T1,  7, CAL16T(1.0, AD590_CALIB_PRIMARY_2),  'u', U_NONE},
  {"t_strut_bot",  'r', BIAS_T1,  9, CAL16T(1.0, AD590_CALIB_STRUT_1),    'u', U_NONE},
  {"t_1_prime",    'r', BIAS_T1, 11, CAL16T(1.0, AD590_CALIB_PRIMARY_1),  'u', U_NONE},
  {"t_1_second",   'r', BIAS_T1, 13, CAL16T(1.0, AD590_CALIB_SECONDARY_1),'u', U_NONE},
  {"t_2_second",   'r', BIAS_T1, 15, CAL16T(1.0, AD590_CALIB_SECONDARY_2),'u', U_NONE},
  {"t_strut_side", 'r', BIAS_T1, 17, CAL16T(1.0, AD590_CALIB_STRUT_2),    'u', U_NONE},
  {"t_push_plate", 'r', BIAS_T1, 19, CAL16T(1.0, AD590_CALIB_PUSH_PLATE), 'u', U_NONE},
  {"t_mot_act",    'r', BIAS_T1, 21, CAL16T(1.0, AD590_CALIB_ACT_MOTOR),  'u', U_NONE},
  {"t_rec",        'r', BIAS_T1, 23, CAL16T(1.0, AD590_CALIB_REC),        'u', U_NONE},
  {"t_das",        'r', BIAS_T1, 25, CAL16T(1.0, 0.0),                    'u', U_NONE},
  /* AD590 calibrations per Marco 2006-11 */
  {"t_if_top_frnt",'r', BIAS_T1, 27, CAL16T(1.0, AD590_CALIB_INFRAME_1),  'u', U_NONE},
  {"t_if_top_back",'r', BIAS_T1, 29, CAL16T(1.0, AD590_CALIB_INFRAME_2),  'u', U_NONE},
  {"t_if_bot_frnt",'r', BIAS_T1, 31, CAL16T(1.0, AD590_CALIB_INFRAME_3),  'u', U_NONE},
  {"t_if_bot_back",'r', BIAS_T1, 33, CAL16T(1.0, AD590_CALIB_INFRAME_4),  'u', U_NONE},
  /* generic names for remaining analog channels. BE MORE SPECIFIC */
  {"t17_das",      'r', BIAS_T1, 35,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t18_das",      'r', BIAS_T1, 37,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t19_das",      'r', BIAS_T1, 39,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t20_das",      'r', BIAS_T1, 41,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t21_das",      'r', BIAS_T1, 43,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t22_das",      'r', BIAS_T1, 45,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t23_das",      'r', BIAS_T1, 47,         CAL16T(1.0,            0.0), 'u', U_NONE},
  {"t24_das",      'r', BIAS_T1, 49,         CAL16T(1.0,            0.0), 'u', U_NONE},

  /* LOOP1 0-13 are wide */
  {"pin_in_lock",  'w', LOOP1, 16,                1.0,             0.0, 'u', U_NONE},
  {"fpulse_isc",   'w', LOOP1, 17,                10.,             0.0, 'u', U_NONE},
  {"period_cal",   'w', LOOP1, 18,                .20,             0.0, 'u', U_NONE},
  {"status_net",   'w', LOOP1, 19,                1.0,             0.0, 'u', U_NONE}, //Sun, ISC, OSC net status
  {"timeout",      'w', LOOP1, 20,                1.0,             0.0, 'u', U_NONE},
  {"az_sun",       'w', LOOP1, 21,              I2DEG,             0.0, 'u', U_D_DEG},
  {"lvdt_low_act", 'w', LOOP1, 22,                1.0,         -5000.0, 'u', U_NONE},
  {"status_mcc",   'w', LOOP1, 23,                1.0,             0.0, 'u', U_NONE}, //south_i_am, at_float, schedule, alice_file
  {"cryostate",    'w', LOOP1, 24,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 25 is unused */
  {"t_chip_flc",   'w', LOOP1, 26,               0.01,             0.0, 'u', U_NONE},
  {"declination_mag",'w', LOOP1, 27,              I2DEG,             0.0, 'u', U_D_DEG}, // magnetic declination
  {"veto_sensor",  'w', LOOP1, 28,                1.0,             0.0, 'u', U_NONE},
  {"level_on_bal", 'w', LOOP1, 29,           1./1990.13,             0.0, 'u', U_I_A},
  {"level_off_bal",'w', LOOP1, 30,           1./1990.13,             0.0, 'u', U_I_A},
  {"level_target_bal",'w', LOOP1, 31,           1./1990.13,            -5.0, 'u', U_I_A},
  /* LOOP1 32-33 are wide */
  /* LOOP1 34-36 are unused */
  {"alt_sip",      'w', LOOP1, 37,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 38-41 are wide */
  {"mapmean_isc",  'w', LOOP1, 42,                 1.,             0.0, 'u', U_NONE},
  {"pitch_dgps",   'w', LOOP1, 43,              I2DEG,             0.0, 's', U_P_DEG},
  {"roll_dgps",    'w', LOOP1, 44,              I2DEG,             0.0, 's', U_P_DEG},
  {"lat_sip",      'w', LOOP1, 45,              I2DEG,             0.0, 's', U_LA_DEG},
  {"lon_sip",      'w', LOOP1, 46,              I2DEG,             0.0, 's', U_LO_DEG},
  {"lat_dgps",     'w', LOOP1, 47,              I2DEG,             0.0, 's', U_LA_DEG},
  {"lon_dgps",     'w', LOOP1, 48,              I2DEG,             0.0, 's', U_LO_DEG},
  {"alt_dgps",     'w', LOOP1, 49,                1.0,             0.0, 'u', U_ALT_M},
  {"speed_dgps",   'w', LOOP1, 50,                1.0,             0.0, 'u', U_V_KPH},
  {"dir_dgps",     'w', LOOP1, 51,              I2DEG,             0.0, 'u', U_D_DEG},
  {"climb_dgps",   'w', LOOP1, 52,                1.0,             0.0, 's', U_V_MPS},
  {"att_ok_dgps",  'w', LOOP1, 53,                1.0,             0.0, 'u', U_NONE},
  /* LOOP1 54-56 are unused */
  {"n_sat_dgps",   'w', LOOP1, 57,                1.0,             0.0, 'u', U_NONE},
  {"disk_free",    'w', LOOP1, 58,             1./250,             0.0, 'u', U_NONE},
  {"mode_p",       'w', LOOP1, 59,                  1,             0.0, 'u', U_NONE},
  {"x_p",          'w', LOOP1, 60,              I2DEG,             0.0, 'u', U_NONE},
  {"y_p",          'w', LOOP1, 61,              I2DEG,             0.0, 's', U_NONE},
  {"vel_az_p",     'w', LOOP1, 62,              I2VEL,             0.0, 'u', U_NONE},
  {"del_p",        'w', LOOP1, 63,              I2VEL,             0.0, 'u', U_NONE},

  {"blob_idx_isc", 'w', LOOP2,  0,                1.0,             0.0, 'u', U_NONE},
  {"blob00_x_isc", 'w', LOOP2,  1,             1./40.,             0.0, 'u', U_NONE},
  {"blob00_y_isc", 'w', LOOP2,  2,             1./40.,             0.0, 'u', U_NONE},
  {"blob00_f_isc", 'w', LOOP2,  3,                1.0,             0.0, 'u', U_NONE},
  {"blob00_s_isc", 'w', LOOP2,  4,       1000./65536.,             0.0, 'u', U_NONE},
  {"blob01_x_isc", 'w', LOOP2,  5,             1./40.,             0.0, 'u', U_NONE},
  {"blob01_y_isc", 'w', LOOP2,  6,             1./40.,             0.0, 'u', U_NONE},
  {"blob01_f_isc", 'w', LOOP2,  7,                1.0,             0.0, 'u', U_NONE},
  {"blob01_s_isc", 'w', LOOP2,  8,       1000./65536.,             0.0, 'u', U_NONE},
  {"blob02_x_isc", 'w', LOOP2,  9,             1./40.,             0.0, 'u', U_NONE},
  {"blob02_y_isc", 'w', LOOP2, 10,             1./40.,             0.0, 'u', U_NONE},
  {"blob02_f_isc", 'w', LOOP2, 11,                1.0,             0.0, 'u', U_NONE},
  {"blob02_s_isc", 'w', LOOP2, 12,       1000./65536.,             0.0, 'u', U_NONE},
  {"w_p",          'w', LOOP2, 13,              I2DEG,             0.0, 'u', U_NONE}, // pointing scan width
  {"rtol_isc",     'w', LOOP2, 14,              I2DEG,             0.0, 'u', U_NONE},
  {"apert_isc",    'w', LOOP2, 15,                1.0,             0.0, 'u', U_NONE},
  {"maglimit_isc", 'w', LOOP2, 16,           1./1000.,             0.0, 'u', U_NONE},
  {"nrad_isc",     'w', LOOP2, 17,              I2DEG,             0.0, 'u', U_NONE},
  {"mtol_isc",     'w', LOOP2, 18,        100./65536.,             0.0, 'u', U_NONE},
  {"qtol_isc",     'w', LOOP2, 19,        100./65536.,             0.0, 'u', U_NONE},
  /*P2 20-21 is wide */
  {"lrad_isc",     'w', LOOP2, 22,              I2DEG,             0.0, 'u', U_NONE},
  {"thresh_isc",   'w', LOOP2, 23,             1./10.,             0.0, 'u', U_NONE},
  {"grid_isc",     'w', LOOP2, 24,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 25 is unused */
  {"real_trig_osc",'w', LOOP2, 26,                1.0,             0.0, 's', U_NONE},
  {"mdist_isc",    'w', LOOP2, 27,                1.0,             0.0, 'u', U_NONE},
  {"nblobs_isc",   'w', LOOP2, 28,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 29-30 are unused */
  {"foc_off_osc",  'w', LOOP2, 31,                1.0,             0.0, 's', U_NONE},
  /* LOOP2 32 is unused */
  {"tol_isc",      'w', LOOP2, 33,                1.0,             0.0, 'u', U_NONE},
  /* LOOP2 34 is fast */
  {"az_ss",           'w',LOOP2, 35,            I2DEG,             0.0, 'u', U_NONE},
  {"offset_ifel_gy",  'w',LOOP2, 36,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ifroll_gy",'w',LOOP2, 37,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_ifyaw_gy", 'w',LOOP2, 38,      1.0/32768.0,             0.0, 's', U_V_DPS},
  {"az_raw_mag",      'w',LOOP2, 39,            I2DEG,             0.0, 's', U_D_DEG},
  /* LOOP2 39 is unused */
  {"sigma_mag",       'w',LOOP2, 40,            I2DEG,             0.0, 'u', U_NONE},
  {"az_dgps",         'w',LOOP2, 41,            I2DEG,             0.0, 'u', U_D_DEG},
  {"sigma_dgps",      'w',LOOP2, 42,            I2DEG,             0.0, 'u', U_NONE},
  {"lvdt_high_act",   'w',LOOP2, 43,              1.0,         -5000.0, 'u', U_NONE},
  {"az_isc",          'w',LOOP2, 44,            I2DEG,             0.0, 'u', U_NONE},
  {"el_isc",          'w',LOOP2, 45,            I2DEG,             0.0, 'u', U_NONE},
  {"sigma_isc",       'w',LOOP2, 46,            I2DEG,             0.0, 'u', U_NONE},
  // LOOP2 47-48 are fast
  {"pulse_cal",    'w', LOOP2, 49,               10.0,              0., 'u', U_NONE},
  {"sigma_ss",     'w', LOOP2, 50,              I2DEG,             0.0, 'u', U_NONE},
  /* LOOP2 51-54 are wide fast */
  {"sigma_clin",   'w', LOOP2, 55,              I2DEG,             0.0, 'u', U_NONE},
  {"az_mag",       'w', LOOP2, 56,              I2DEG,             0.0, 'u', U_D_DEG},
  {"spulse_isc",   'w', LOOP2, 57,               10.0,             0.0, 'u', U_NONE},
  {"hx_flag_isc",  'w', LOOP2, 58,                1.0,             0.0, 'u', U_NONE},
  {"brra_isc",     'w', LOOP2, 59,              I2DEG,             0.0, 'u', U_NONE},
  /* LOOP2 60-61 are wide */
  {"brdec_isc",    'w', LOOP2, 62,              I2DEG,             0.0, 'u', U_NONE},
  {"x_off_isc",    'w', LOOP2, 63,              I2DEG,             0.0, 's', U_NONE},

  {"gain_osc",     'w', LOOP3,  0,        100./65536.,             0.0, 'u', U_NONE},
  {"i_hold_isc",   'w', LOOP3,  1,                1.0,             0.0, 'u', U_NONE},
  {"save_prd_isc", 'w', LOOP3,  2,               0.01,             0.0, 'u', U_NONE},
  {"y_off_isc",    'w', LOOP3,  3,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP3 4-5 are wide */
  {"offset_isc",   'w', LOOP3,  6,                1.0,             0.0, 's', U_NONE},
  {"bbc_fifo_size",'w', LOOP3,  7,             1./624,             0.0, 'u', U_NONE},
  {"t_cpu_flc",    'w', LOOP3,  8,               0.01,             0.0, 'u', U_NONE},
  {"t_x_flc",      'w', LOOP3,  9,               0.01,             0.0, 'u', U_NONE},
  {"mks_hi_sip",   'w', LOOP3, 10,           0.003256,       -0.226858, 'u', U_NONE},
  {"mks_med_sip",  'w', LOOP3, 11,           0.032614,       -0.072580, 'u', U_NONE},
  {"blob_idx_osc", 'w', LOOP3, 12,                1.0,             0.0, 'u', U_NONE},
  {"blob00_x_osc", 'w', LOOP3, 13,             1./40.,             0.0, 'u', U_NONE},
  {"blob00_y_osc", 'w', LOOP3, 14,             1./40.,             0.0, 'u', U_NONE},
  {"blob00_f_osc", 'w', LOOP3, 15,                1.0,             0.0, 'u', U_NONE},
  {"blob00_s_osc", 'w', LOOP3, 16,             1./40.,             0.0, 'u', U_NONE},
  {"blob01_x_osc", 'w', LOOP3, 17,             1./40.,             0.0, 'u', U_NONE},
  {"blob01_y_osc", 'w', LOOP3, 18,             1./40.,             0.0, 'u', U_NONE},
  {"blob01_f_osc", 'w', LOOP3, 19,                1.0,             0.0, 'u', U_NONE},
  {"blob01_s_osc", 'w', LOOP3, 20,             1./40.,             0.0, 'u', U_NONE},
  {"blob02_x_osc", 'w', LOOP3, 21,             1./40.,             0.0, 'u', U_NONE},
  {"blob02_y_osc", 'w', LOOP3, 22,             1./40.,             0.0, 'u', U_NONE},
  {"blob02_f_osc", 'w', LOOP3, 23,                1.0,             0.0, 'u', U_NONE},
  {"blob02_s_osc", 'w', LOOP3, 24,             1./40.,             0.0, 'u', U_NONE},
  {"mapmean_osc",  'w', LOOP3, 25,                 1.,             0.0, 'u', U_NONE},
  /* LOOP3 26-29 are wide */
  {"fpulse_osc",   'w', LOOP3, 30,                10.,             0.0, 'u', U_NONE}, //apparently not used
  /* LOOP3 31-32 are wide */
  {"az_osc",       'w', LOOP3, 33,              I2DEG,             0.0, 'u', U_NONE},
  {"el_osc",       'w', LOOP3, 34,              I2DEG,             0.0, 'u', U_NONE},
  {"sigma_osc",    'w', LOOP3, 35,              I2DEG,             0.0, 'u', U_NONE},
  {"tol_osc",      'w', LOOP3, 36,                1.0,             0.0, 'u', U_NONE},
  {"apert_osc",    'w', LOOP3, 37,                1.0,             0.0, 'u', U_NONE},
  {"maglimit_osc", 'w', LOOP3, 38,           1./1000.,             0.0, 'u', U_NONE},
  {"nrad_osc",     'w', LOOP3, 39,              I2DEG,             0.0, 'u', U_NONE},
  {"mtol_osc",     'w', LOOP3, 40,        100./65536.,             0.0, 'u', U_NONE},
  {"qtol_osc",     'w', LOOP3, 41,        100./65536.,             0.0, 'u', U_NONE},
  {"offset_osc",   'w', LOOP3, 42,                1.0,             0.0, 's', U_NONE},
  {"lrad_osc",     'w', LOOP3, 43,              I2DEG,             0.0, 'u', U_NONE},
  /* LOOP3 44-45 are wide */
  {"thresh_osc",   'w', LOOP3, 46,             1./10.,             0.0, 'u', U_NONE},
  {"grid_osc",     'w', LOOP3, 47,                1.0,             0.0, 'u', U_NONE},
  {"real_trig_isc",'w', LOOP3, 48,                1.0,             0.0, 's', U_NONE},
  {"foc_off_isc",  'w', LOOP3, 49,                1.0,             0.0, 's', U_NONE},
  {"mdist_osc",    'w', LOOP3, 50,                1.0,             0.0, 'u', U_NONE},
  {"nblobs_osc",   'w', LOOP3, 51,                1.0,             0.0, 'u', U_NONE},
  {"rtol_osc",     'w', LOOP3, 52,              I2DEG,             0.0, 'u', U_NONE},
  {"rd_sigma_osc", 'w', LOOP3, 53,                1.0,             0.0, 'u', U_NONE},
  {"spulse_osc",   'w', LOOP3, 54,               10.0,             0.0, 'u', U_NONE},
  {"hx_flag_osc",  'w', LOOP3, 55,                1.0,             0.0, 'u', U_NONE},
  {"brra_osc",     'w', LOOP3, 56,              I2DEG,             0.0, 'u', U_NONE},
  {"brdec_osc",    'w', LOOP3, 57,              I2DEG,             0.0, 'u', U_NONE},
  /* LOOP3 58-59 are wide */
  {"x_off_osc",    'w', LOOP3, 60,              I2DEG,             0.0, 's', U_NONE},
  {"i_hold_osc",   'w', LOOP3, 61,                1.0,             0.0, 'u', U_NONE},
  {"save_prd_osc", 'w', LOOP3, 62,               0.01,             0.0, 'u', U_NONE},
  {"y_off_osc",    'w', LOOP3, 63,              I2DEG,             0.0, 's', U_NONE},

  {"phase_ss",     'w', LOOP4,  0,              I2DEG,             0.0, 's', U_NONE},
  {"pref_tp_sf",   'w', LOOP4,  1,                1.0,             0.0, 'u', U_NONE},
  {"sun_time_ss",  'w', LOOP4,  3,                1.0,             0.0, 'u', U_NONE},
  {"t_cpu_ss",     'w', LOOP4,  4,             1/100.,         -273.15, 'u', U_NONE},
  {"t_hdd_ss",     'w', LOOP4,  5,             1/100.,         -273.15, 'u', U_NONE},
  {"maxblobs_isc", 'w', LOOP4,  6,                1.0,             0.0, 'u', U_NONE},
  {"maxblobs_osc", 'w', LOOP4,  7,                1.0,             0.0, 'u', U_NONE},
  {"bi0_fifo_size",'w', LOOP4,  8,             1./624,             0.0, 'u', U_NONE},
  {"plover",       'w', LOOP4,  9,                1.0,             0.0, 'u', U_NONE},
  {"t_flange_isc", 'w', LOOP4, 10,            1./200.,         -273.15, 'u', U_NONE},
  {"t_lens_isc",   'w', LOOP4, 11,            1./200.,         -273.15, 'u', U_NONE},
  {"t_heat_isc",   'w', LOOP4, 12,            1./200.,         -273.15, 'u', U_NONE},
  {"t_comp_isc",   'w', LOOP4, 13,            1./200.,         -273.15, 'u', U_NONE},
  {"pressure1_isc",'w', LOOP4, 14,           1./2000.,             0.0, 'u', U_NONE},
  {"t_flange_osc", 'w', LOOP4, 15,            1./200.,         -273.15, 'u', U_NONE},
  {"t_lens_osc",   'w', LOOP4, 16,            1./200.,         -273.15, 'u', U_NONE},
  {"t_heat_osc",   'w', LOOP4, 17,            1./200.,         -273.15, 'u', U_NONE},
  {"t_comp_osc",   'w', LOOP4, 18,            1./200.,         -273.15, 'u', U_NONE},
  {"pressure1_osc",'w', LOOP4, 19,           1./2000.,             0.0, 'u', U_NONE},
  {"gain_isc",     'w', LOOP4, 20,        100./65536.,             0.0, 'u', U_NONE},
  {"jfet_set_on",  'w', LOOP4, 21,             1/100.,             0.0, 'u', U_NONE},
  {"jfet_set_off", 'w', LOOP4, 22,             1/100.,             0.0, 'u', U_NONE},
  {"t_case_ss",    'w', LOOP4, 23,             1/100.,         -273.15, 'u', U_NONE},
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
  /* LOOP4 39 is unused */
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
  {"trim_ss",      'w', LOOP4, 57,              I2DEG,             0.0, 's', U_NONE},
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
  {"dith_el",      'w', LOOP5, 23,          2.0*I2DEG,             0.0, 's', U_D_DEG},
  {"state_lock",   'w', LOOP5, 25,                1.0,             0.0, 'u', U_NONE},
  {"goal_lock",    'w', LOOP5, 26,                1.0,             0.0, 'u', U_NONE},
  {"seized_act",   'w', LOOP5, 27,                1.0,             0.0, 's', U_NONE},
  /* LOOP5 28 is fast */
  /* LOOP5 29 is unused */
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
  {"he4_lev_old",  'w', LOOP5, 40,          CRYO_A2_M,       CRYO_A2_B, 'u', U_NONE},
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
  {"goal_sf",      'w', LOOP6, 30,                1.0,             0.0, 's', U_NONE},
  {"focus_sf",     'w', LOOP6, 32,                1.0,             0.0, 's', U_NONE},

  {"maxslew_isc",  'w', LOOP6,  0,              I2DEG,             0.0, 'u', U_NONE},
  {"maxslew_osc",  'w', LOOP6,  1,              I2DEG,             0.0, 'u', U_NONE},
  {"t_port_ss",    'w', LOOP6,  2,             1/100.,         -273.15, 'u', U_NONE},
  {"t_star_ss",    'w', LOOP6,  3,             1/100.,         -273.15, 'u', U_NONE},
  {"v_5_ss",       'w', LOOP6,  4,             1/100.,             0.0, 'u', U_NONE},
  {"v_12_ss",      'w', LOOP6,  5,             1/100.,             0.0, 'u', U_NONE},
  {"v_batt_ss",    'w', LOOP6,  6,             1/100.,             0.0, 'u', U_NONE},
  {"raw_01_ss",    'w', LOOP6,  7,                1.0,             0.0, 'u', U_NONE},
  {"raw_02_ss",    'w', LOOP6,  8,                1.0,             0.0, 'u', U_NONE},
  {"raw_03_ss",    'w', LOOP6,  9,                1.0,             0.0, 'u', U_NONE},
  {"raw_04_ss",    'w', LOOP6, 10,                1.0,             0.0, 'u', U_NONE},
  {"raw_05_ss",    'w', LOOP6, 11,                1.0,             0.0, 'u', U_NONE},
  {"raw_06_ss",    'w', LOOP6, 12,                1.0,             0.0, 'u', U_NONE},
  {"raw_07_ss",    'w', LOOP6, 13,                1.0,             0.0, 'u', U_NONE},
  {"raw_08_ss",    'w', LOOP6, 14,                1.0,             0.0, 'u', U_NONE},
  {"raw_09_ss",    'w', LOOP6, 15,                1.0,             0.0, 'u', U_NONE},
  {"raw_10_ss",    'w', LOOP6, 16,                1.0,             0.0, 'u', U_NONE},
  {"raw_11_ss",    'w', LOOP6, 17,                1.0,             0.0, 'u', U_NONE},
  {"raw_12_ss",    'w', LOOP6, 18,                1.0,             0.0, 'u', U_NONE},
  {"az_rel_sun_ss",'w', LOOP6, 19,              I2DEG,             0.0, 'u', U_NONE},
  {"pref_ts_sf",   'w', LOOP6, 20,                1.0,             0.0, 'u', U_NONE},
  {"spread_sf",    'w', LOOP6, 21,             1/500.,             0.0, 'u', U_NONE},
  {"acc_lock",     'w', LOOP6, 22,                1.0,             0.0, 'u', U_NONE},
  {"snr_ss",       'w', LOOP6, 23,            1/1000.,             0.0, 'u', U_NONE},
  //TODO focus veto appears unused, delete if it truly is, even when secondary code finished
  //{"focus_veto",   'w', LOOP6, 24,                1.0,             0.0, 'u', U_NONE},
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
  {"t_prime_sf",   'w', LOOP6, 44,             1/500.,             0.0, 's', U_NONE},
  {"t_second_sf",  'w', LOOP6, 45,             1/500.,             0.0, 's', U_NONE},
  {"flags_act",    'w', LOOP6, 50,                1.0,             0.0, 'u', U_NONE},
  //TODO focus mode appears unused, delete if it truly is, even when secondary code finished
  //{"focus_mode",   'w', LOOP6, 51,                1.0,             0.0, 'u', U_NONE},
  {"lvdt_spread_act",'w', LOOP6, 55,              1.0,             0.0, 's', U_NONE},
  /* LOOP6 56-57 are wide */
  {"el_sun",       'w', LOOP6, 58,              I2DEG,             0.0, 's', U_NONE},
  /* LOOP6 59 unused */
  {"mode_cal",     'w', LOOP6, 60,                1.0,             0.0, 'u', U_NONE},
  {"minblobs_isc", 'w', LOOP6, 61,                1.0,             0.0, 'u', U_NONE},
  {"minblobs_osc", 'w', LOOP6, 62,                1.0,             0.0, 'u', U_NONE},
  /* LOOP7 0-1 are fast narrow */
  /* LOOP7 2-3 are unused */
  {"stat_1_rw",    'w', LOOP7,  4,                1.0,             0.0, 'u', U_NONE},  
  {"stat_2_rw",    'w', LOOP7,  5,                1.0,             0.0, 'u', U_NONE},  
  {"fault_rw",     'w', LOOP7,  6,                1.0,             0.0, 'u', U_NONE},  
  /* LOOP7 0-3 are slow wide0*/
  {"stat_1_el",    'w', LOOP7,  7,                1.0,             0.0, 'u', U_NONE},  
  {"stat_2_el",    'w', LOOP7,  8,                1.0,             0.0, 'u', U_NONE},  
  {"fault_el",     'w', LOOP7,  9,                1.0,             0.0, 'u', U_NONE},  
  {"stat_dr_piv",  'w', LOOP7, 10,                1.0,             0.0, 'u', U_NONE},  
  {"stat_s1_piv",  'w', LOOP7, 11,                1.0,             0.0, 'u', U_NONE},  
  {"t_mc_rw",      'w', LOOP7, 12,                1.0,             0.0, 's', U_T_C},
  {"i_ser_rw",     'w', LOOP7, 13,       30.0/32768.0,             0.0, 's', U_I_A},
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
  {"vel_ser_piv",  'w', LOOP7, 27,           1.0/0.144,             0.0, 's', U_NONE},
  {"vel_calc_piv", 'w', LOOP7, 28,        20.0/32768.0,             0.0, 's', U_V_DPS},
  {"age_isc",      'w', LOOP7, 29,                 1.0,             0.0, 'u', U_T_MS},
  {"age_osc",      'w', LOOP7, 30,                 1.0,             0.0, 'u', U_T_MS},
  /* LOOP7 31 is unused */
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
  {"step_pul_len_bias",'w', LOOP7, 59,             1.0,             0.0, 'u', U_NONE},
  {"step_array_bias",  'w', LOOP7, 60,             1.0,             0.0, 'u', U_NONE},
  {"step_start_phase", 'w', LOOP7, 61,             0.5,             0.0, 'u', U_NONE},
  {"step_end_phase",   'w', LOOP7, 62,             0.5,             0.0, 'u', U_NONE},
  {"step_nsteps_phase",'w', LOOP7, 63,             1.0,             0.0, 'u', U_NONE},
  {"step_time_phase",  'w', LOOP8,  0,             1.0,             0.0, 'u', U_NONE},
  {"step_ena_bias",    'w', LOOP8,  1,             1.0,             0.0, 'u', U_NONE},
  {"step_ena_phase",   'w', LOOP8,  2,             1.0,             0.0, 'u', U_NONE},
  
  /* charge controller related channels */
  
  {"v_batt_cc",  'w',  LOOP8,  3,   1.0e-3,  0.0,  'u',  U_V_V},
  {"v_arr_cc",   'w',  LOOP8,  4,   1.0e-3,  0.0,  'u',  U_V_V},
  {"i_batt_cc",  'w',  LOOP8,  5,   1.0e-3,  0.0,  'u',  U_I_A},
  {"i_arr_cc",   'w',  LOOP8,  6,   1.0e-3,  0.0,  'u',  U_I_A},
  { "t_hs_cc",   'w',  LOOP8,  7,   1.0,     0.0,  's',  U_T_C},
  {"fault_cc",   'w',  LOOP8,  8,   1.0,     0.0,  'u',  U_NONE},   // fault bitfield
  {"alarm_hi_cc",'w',  LOOP8,  9,   1.0,     0.0,  'u',  U_NONE},   // alarm high bitfield
  {"alarm_lo_cc",'w',  LOOP8,  10,  1.0,     0.0,  'u',  U_NONE},  // alarm low bitfield
  {"v_targ_cc",  'w',  LOOP8,  11,  1.0e-3,  0.0,  'u',  U_V_V},
  {"state_cc",   'w',  LOOP8,  12,  1.0,     0.0,  'u',  U_NONE},   
  
  /* filtered LVDTs, rotated to motor positions */
  {"lvdt_0_act",  'w',  LOOP8, 13,   1.0,     0.0,   's', U_NONE},
  {"lvdt_1_act",  'w',  LOOP8, 14,   1.0,     0.0,   's', U_NONE},
  {"lvdt_2_act",  'w',  LOOP8, 15,   1.0,     0.0,   's', U_NONE},

  {"frict_off_piv",'w', LOOP8,  16,      2.0/65535.0,              0.0,   'u', U_NONE},
  {"frict_term_piv",'w',LOOP8,  17,      2.0/32767.0,              0.0,   's', U_NONE},
  {"frict_term_uf_piv",'w',LOOP8,18,     2.0/32767.0,              0.0,   's', U_NONE}, // For debugging remove later

  {"az_gy",        'w',   LOOP8, 19,       20.0/32768.0,             0.0, 's', U_V_DPS},
  {"offset_0_act", 'w',   LOOP8, 20,                1.0,             0.0, 's', U_NONE},
  {"offset_1_act", 'w',   LOOP8, 21,                1.0,             0.0, 's', U_NONE},
  {"offset_2_act", 'w',   LOOP8, 22,                1.0,             0.0, 's', U_NONE},
  {"goal_0_act",   'w',   LOOP8, 23,                1.0,             0.0, 's', U_NONE},
  {"goal_1_act",   'w',   LOOP8, 24,                1.0,             0.0, 's', U_NONE},
  {"goal_2_act",   'w',   LOOP8, 25,                1.0,             0.0, 's', U_NONE},

  {"azraw_pss1",   'w',   LOOP8, 26,              I2DEG,             0.0, 'u', U_P_DEG},
  {"elraw_pss1",   'w',   LOOP8, 27,              I2DEG,             0.0, 'u', U_P_DEG},
  {"snr_pss1",     'w',   LOOP8, 28,            1/1000.,             0.0, 'u', U_NONE},
  {"az_pss1",      'w',   LOOP8, 29,              I2DEG,             0.0, 'u', U_P_DEG},
  {"azraw_pss2",   'w',   LOOP8, 30,              I2DEG,             0.0, 'u', U_P_DEG},
  {"elraw_pss2",   'w',   LOOP8, 31,              I2DEG,             0.0, 'u', U_P_DEG},
  {"snr_pss2",     'w',   LOOP8, 32,            1/1000.,             0.0, 'u', U_NONE},
  {"az_pss2",      'w',   LOOP8, 33,              I2DEG,             0.0, 'u', U_P_DEG},
  {"accel_az",     'w',   LOOP8, 34,          2.0/65536,             0.0, 'u', U_NONE},
  {"pos_focus_isc",'w',   LOOP8, 35,                1.0,             0.0, 'u', U_NONE},
  {"pos_focus_osc",'w',   LOOP8, 36,                1.0,             0.0, 'u', U_NONE},

  {"az_cov_dgps",   'w', LOOP8, 37,              I2DEG,             0.0, 'u', U_NONE},
  {"pitch_cov_dgps",'w', LOOP8, 38,              I2DEG,             0.0, 'u', U_NONE},
  {"roll_cov_dgps", 'w', LOOP8, 39,              I2DEG,             0.0, 'u', U_NONE},

  {"led_cc",        'w', LOOP8, 40,              1.0,               0.0, 'u', U_NONE}, // charge controller LED state
  {"sbsc_force",     'w', LOOP8, 41,                1.0,             0.0, 'u'},
  {"sbsc_exp_int",   'w', LOOP8, 42,                1.0,             0.0, 'u'},
  {"sbsc_exp_time",  'w', LOOP8, 43,                1.0,             0.0, 'u'},
  {"sbsc_foc_res",   'w', LOOP8, 44,                1.0,             0.0, 'u'},
  {"sbsc_move_tol",  'w', LOOP8, 45,                1.0,             0.0, 'u'},
  {"sbsc_maxblob",   'w', LOOP8, 46,                1.0,             0.0, 'u'},
  {"sbsc_grid",      'w', LOOP8, 47,                1.0,             0.0, 'u'},
  {"sbsc_thresh",    'w', LOOP8, 48,         1.0/1000.0,             0.0, 'u'},
  {"sbsc_mdist",     'w', LOOP8, 49,                1.0,             0.0, 'u'},
  {"sbsc_mapmean",   'w', LOOP8, 50,                1.0,             0.0, 'u'},
  {"sbsc_mapsigma",  'w', LOOP8, 51,           1.0/10.0,             0.0, 'u'},
  {"sbsc_ccd_t",     'w', LOOP8, 52,          1.0/100.0,             0.0, 's'},
  {"sbsc_numblobs",  'w', LOOP8, 53,                1.0,             0.0, 'u'},
  {"sbsc_blob00_x",  'w', LOOP8, 54, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob00_y",  'w', LOOP8, 55, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob00_f",  'w', LOOP8, 56,                1.0,             0.0, 'u'},
  {"sbsc_blob00_s",  'w', LOOP8, 57,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob01_x",  'w', LOOP8, 58, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob01_y",  'w', LOOP8, 59, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob01_f",  'w', LOOP8, 60,                1.0,             0.0, 'u'},
  {"sbsc_blob01_s",  'w', LOOP8, 61,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob02_x",  'w', LOOP8, 62, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob02_y",  'w', LOOP8, 63, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob02_f",  'w', LOOP9,  0,                1.0,             0.0, 'u'},
  {"sbsc_blob02_s",  'w', LOOP9,  1,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob03_x",  'w', LOOP9,  2, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob03_y",  'w', LOOP9,  3, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob03_f",  'w', LOOP9,  4,                1.0,             0.0, 'u'},
  {"sbsc_blob03_s",  'w', LOOP9,  5,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob04_x",  'w', LOOP9,  6, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob04_y",  'w', LOOP9,  7, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob04_f",  'w', LOOP9,  8,                1.0,             0.0, 'u'},
  {"sbsc_blob04_s",  'w', LOOP9,  9,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob05_x",  'w', LOOP9, 10, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob05_y",  'w', LOOP9, 11, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob05_f",  'w', LOOP9, 12,                1.0,             0.0, 'u'},
  {"sbsc_blob05_s",  'w', LOOP9, 13,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob06_x",  'w', LOOP9, 14, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob06_y",  'w', LOOP9, 15, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob06_f",  'w', LOOP9, 16,                1.0,             0.0, 'u'},
  {"sbsc_blob06_s",  'w', LOOP9, 17,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob07_x",  'w', LOOP9, 18, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob07_y",  'w', LOOP9, 19, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob07_f",  'w', LOOP9, 20,                1.0,             0.0, 'u'},
  {"sbsc_blob07_s",  'w', LOOP9, 21,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob08_x",  'w', LOOP9, 22, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob08_y",  'w', LOOP9, 23, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob08_f",  'w', LOOP9, 24,                1.0,             0.0, 'u'},
  {"sbsc_blob08_s",  'w', LOOP9, 25,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob09_x",  'w', LOOP9, 26, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob09_y",  'w', LOOP9, 27, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob09_f",  'w', LOOP9, 28,                1.0,             0.0, 'u'},
  {"sbsc_blob09_s",  'w', LOOP9, 29,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob10_x",  'w', LOOP9, 30, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob10_y",  'w', LOOP9, 31, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob10_f",  'w', LOOP9, 32,                1.0,             0.0, 'u'},
  {"sbsc_blob10_s",  'w', LOOP9, 33,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob11_x",  'w', LOOP9, 34, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob11_y",  'w', LOOP9, 35, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob11_f",  'w', LOOP9, 36,                1.0,             0.0, 'u'},
  {"sbsc_blob11_s",  'w', LOOP9, 37,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob12_x",  'w', LOOP9, 38, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob12_y",  'w', LOOP9, 39, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob12_f",  'w', LOOP9, 40,                1.0,             0.0, 'u'},
  {"sbsc_blob12_s",  'w', LOOP9, 41,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob13_x",  'w', LOOP9, 42, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob13_y",  'w', LOOP9, 43, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob13_f",  'w', LOOP9, 44,                1.0,             0.0, 'u'},
  {"sbsc_blob13_s",  'w', LOOP9, 45,          1.0/100.0,             0.0, 'u'},
  {"sbsc_blob14_x",  'w', LOOP9, 46, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sbsc_blob14_y",  'w', LOOP9, 47, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sbsc_blob14_f",  'w', LOOP9, 48,                1.0,             0.0, 'u'},
  {"sbsc_blob14_s",  'w', LOOP9, 49,          1.0/100.0,             0.0, 'u'}, 
  /* LOOP9 50-55 are wide */
  {"i_tot",         'w', LOOP9, 56,              1.0e-3,            0.0, 'u', U_I_A}, // sum of currents read through ACS1 A1
  {"t_set_sbsc",     'w', LOOP9, 57,    (100.0/32768.0),             0.0, 'u', U_NONE},  
  /* LOOP9 58-63 are unused */

#ifndef BOLOTEST
/* ACS1 Digital I/O card */
  {"latch0",       'w',  ACS1_D,  0,                1.0,             0.0, 'u', U_NONE},
  {"latch1",       'w',  ACS1_D,  1,                1.0,             0.0, 'u', U_NONE},
  {"switch_gy",    'w',  ACS1_D,  2,                1.0,             0.0, 'u', U_NONE},
  //TODO this may be unused because of the new program
  //{"switch_charge",'w',  ACS1_D,  3,                1.0,             0.0, 'u', U_NONE},
  {"switch_misc",  'w',  ACS1_D,  4,                1.0,             0.0, 'u', U_NONE},
  {"bus_reset_act",'w',  ACS1_D,  5,                1.0,             0.0, 'u', U_NONE},

/* ACS1 Analog card 1 */
/* default current cal is 12.5, 0.0 */
  {"i_trans",      'r',  ACS1_A1, 1,          CAL16(11.32, -0.02),      'u', U_I_A},
  {"i_das",        'r',  ACS1_A1, 3,          CAL16(11.03, -0.09),      'u', U_I_A},
  {"i_acs",        'r',  ACS1_A1, 5,          CAL16(12.5,  0.0),        'u', U_I_A},
  {"i_rec",        'r',  ACS1_A1, 7,          CAL16(10.75, -0.09),      'u', U_I_A},
  {"i_sc",         'r',  ACS1_A1, 9,          CAL16(10.89, -0.17),      'u', U_I_A},
  {"i_dgps",       'r',  ACS1_A1, 11,         CAL16(11.03, -0.22),      'u', U_I_A},
  {"i_el",         'r',  ACS1_A1, 13,         CAL16(12.5,  -0.02),      'u', U_I_A},
  {"i_piv",        'r',  ACS1_A1, 15,         CAL16(12.5, -0.05),       'u', U_I_A},
  {"i_rw",         'r',  ACS1_A1, 17,         CAL16(12.5, -0.09),       'u', U_I_A},
  {"i_step",       'r',  ACS1_A1, 19,         CAL16(12.5, -0.25),       'u', U_I_A},
  {"i_gy",         'r',  ACS1_A1, 21,         CAL16(11.0, -0.01),       'u', U_I_A},
  {"i_mcc",        'r',  ACS1_A1, 23,         CAL16(12.5,  0.0),        'u', U_I_A},
  {"v_1_sol",      'r',  ACS1_A1, 25,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"v_2_sol",      'r',  ACS1_A1, 27,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"v_3_sol",      'r',  ACS1_A1, 29,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"v_4_sol",      'r',  ACS1_A1, 31,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"v_5_sol",      'r',  ACS1_A1, 33,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"v_6_sol",      'r',  ACS1_A1, 35,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"v_batt",       'r',  ACS1_A1, 37,         CAL16(1.0, 0.0), 		'u', U_V_V},
  {"i_sol",        'r',  ACS1_A1, 39,         CAL16(25.0, 0.0),         'u', U_I_A},
  {"i_batt",       'r',  ACS1_A1, 41,         CAL16(25.0, 0.0),         'u', U_I_A},

/* ACS1 Temperature card */
  {"t_gy",         'r',  ACS1_T1, 1,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_serial",     'r',  ACS1_T1, 3,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_mc_piv",     'r',  ACS1_T1, 5,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_piv",        'r',  ACS1_T1, 7,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_mcc",        'r',  ACS1_T1, 9,          CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_charger",    'r',  ACS1_T1, 11,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_1_bat",      'r',  ACS1_T1, 13,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_2_bat",      'r',  ACS1_T1, 15,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_3_bat",      'r',  ACS1_T1, 17,         CAL16T(1.0, 0.0),         'u', U_T_C}, /* Not used can be re-assigned*/
  {"t_4_bat",      'r',  ACS1_T1, 19,         CAL16T(1.0, 0.0),         'u', U_T_C}, /* Not used can be re-assigned*/
  {"t_array",      'r',  ACS1_T1, 21,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_acs",        'r',  ACS1_T1, 23,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_dcdc_acs",   'r',  ACS1_T1, 25,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_mc_lock",    'r',  ACS1_T1, 27,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_lock",       'r',  ACS1_T1, 29,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_sbsc",       'r',  ACS1_T1, 31,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_box_bal",    'r',  ACS1_T1, 33,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_pump_bal",   'r',  ACS1_T1, 35,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_el",         'r',  ACS1_T1, 37,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_shade",      'r',  ACS1_T1, 39,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_sun",        'r',  ACS1_T1, 41,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_rw",         'r',  ACS1_T1, 43,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_earth",      'r',  ACS1_T1, 45,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_chin",       'r',  ACS1_T1, 47,         CAL16T(1.0, 0.0),         'u', U_T_C},
  {"t_dgps",       'r',  ACS1_T1, 49,         CAL16T(1.0, 0.0),         'u', U_T_C},

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
  {"trig_s_sbsc",  'r',   ACS2_D, 51,                1.0,             0.0, 'u', U_NONE},
  {"trig_l_sbsc",  'r',   ACS2_D, 52,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Analog card */
  {"roll_pyr_clin",'r',  ACS2_A1,  1,     -4.0/5333.3333,        4.*6.144, 'u', U_NONE},
  {"pch_pyr_clin", 'r',  ACS2_A1,  3,      4.0/5333.3333,       -4.*6.144, 'u', U_NONE},
  {"t_pyr_clin",   'r',  ACS2_A1,  5,           -0.01875,           614.4, 'u', U_NONE},
  {"xel_if_clin",  'r',  ACS2_A1,  7,         0.00546739,      -25.*6.144, 'u', U_NONE},
  {"el_raw_if_clin",'r', ACS2_A1,  9,                1.0,             0.0, 'u', U_NONE},
  //  {"el_raw_if_clin",'r', ACS2_A1,  9,         0.00546739,         -133.78, 'u', U_NONE},
  {"t_if_clin",    'r',  ACS2_A1, 11,           -0.01875,           614.4, 'u', U_NONE},
  {"x_mag",        'r',  ACS2_A1, 13,              MAGX_M,         MAGX_B, 'u', U_NONE},
  {"y_mag",        'r',  ACS2_A1, 15,              MAGY_M,         MAGY_B, 'u', U_NONE},
  {"z_mag",        'r',  ACS2_A1, 17,              MAGZ_M,         MAGZ_B, 'u', U_NONE},
  {"ifpm_hall",    'r',  ACS2_A1, 19,                1.0,             0.0, 'u', U_NONE},
  {"lvdt_65_act",  'r',  ACS2_A1, 21,   LVDT65_ADC_TO_ENC,     LVDT65_ZERO,   'u', U_NONE},
  {"lvdt_63_act",  'r',  ACS2_A1, 23,   LVDT63_ADC_TO_ENC,     LVDT63_ZERO,   'u', U_NONE},
  {"lvdt_64_act",  'r',  ACS2_A1, 25,   LVDT64_ADC_TO_ENC,     LVDT64_ZERO,   'u', U_NONE},
  //{"sbsc_trig",	   'r',	 ACS2_A1, 27,		      1.0,	      0.0, 'u', U_NONE},
  {"v1_1_pss",     'r',  ACS2_A1, 29,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_1_pss",     'r',  ACS2_A1, 31,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_1_pss",     'r',  ACS2_A1, 33,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_1_pss",     'r',  ACS2_A1, 35,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v1_2_pss",     'r',  ACS2_A1, 37,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v2_2_pss",     'r',  ACS2_A1, 39,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v3_2_pss",     'r',  ACS2_A1, 41,           CAL16(-1.,            0.),  'u', U_V_V},
  {"v4_2_pss",     'r',  ACS2_A1, 43,           CAL16(-1.,            0.),  'u', U_V_V},

#endif

/* TODO These TMP channels need to be added to correct new nodes, or deleted */
//  {"t_el_mc",      'r',  TMP1,  3,                    CAL16T(1.0, 0.0), 'u', U_NONE},
//  {"t_el_mot",     'r',  TMP1,  5,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //{"apcu_reg",     'w',  TMP1,  4,             0.0382,           27.25, 'u', U_NONE}, // to be deprecated
  //{"dpcu_reg",     'w',  TMP1,  5,             0.0382,           27.25, 'u', U_NONE}, // to be deprecated

//  {"t_reac",       'r',  TMP2, 29,                    CAL16T(1.0, 0.0), 'u', U_NONE},
//  {"t_reac_mc",    'r',  TMP2, 31,                    CAL16T(1.0, 0.0), 'u', U_NONE},

// TODO: See if these are legacy and can be deleted.

  /* ACS2 0-1 is wide fast */
  //  {"t_ss_back_mid",'r',  TMP3,  3,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //  {"i_sun",        'r',  TMP3, 13,           0.000625,          -20.48, 'u', U_NONE},


  //  {"t_chin_mid",   'r',  TMP3, 31,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //  {"sensor_reset", 'w',  TMP3,  1,                1.0,             0.0, 'u', U_NONE},

  //  {"v_batt_acs",   'r',  TMP4,  1,    -0.000525352612,      34.4796641, 'u', U_NONE},
  //  {"v_batt_das",   'r',  TMP4,  3,  -0.00052776250894,  34.62735213282, 'u', U_NONE},
  //  {"t_apcu",       'r',  TMP4,  5,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //  {"t_dpcu",       'r',  TMP4,  7,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //  {"t_sol_port",   'r',  TMP4,  9,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //  {"t_sol_stbd",   'r',  TMP4, 11,                    CAL16T(1.0, 0.0), 'u', U_NONE},
  //{"t_batt_acs",   'r',  TMP4, 13,                    CAL16T(1.0, 0.0), 'u', U_NONE}, // to be changed/deprecated
  //{"t_batt_das",   'r',  TMP4, 15,                    CAL16T(1.0, 0.0), 'u', U_NONE}, // to be changed/deprecated
  //  {"i_gond_acs",   'r',  TMP4, 17,          -1.875E-3,           61.44, 'u', U_NONE},
  //  {"i_gond_das",   'r',  TMP4, 19,          -1.875E-3,           61.44, 'u', U_NONE},
  //{"lev_pump_bal",  'w',  TMP4,  3,    -0.048851978505,           100.0, 'u', U_NONE}, 
  //{"sprpump_lev",  'w',  TMP4,  4,    -0.048851978505,           100.0, 'u', U_NONE}, // to be deprecated
  //{"inpump_lev",   'w',  TMP4,  5,    -0.048851978505,           100.0, 'u', U_NONE}, // to be deprecated
  //{"outpump_lev",  'w',  TMP4,  6,    -0.048851978505,           100.0, 'u', U_NONE}, // to be deprecated


  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
#ifndef BOLOTEST
  {"ifyaw_1_gy",  'r',  ACS2_D,  0, DGY32_TO_DPS, -1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifroll_1_gy", 'r',  ACS2_D,  2, -DGY32_TO_DPS, 1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifyaw_2_gy",  'r',  ACS2_D,  4, DGY32_TO_DPS, -1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifel_1_gy",   'r',  ACS2_D,  6, -DGY32_TO_DPS, 1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifel_2_gy",   'r',  ACS2_D,  8, -DGY32_TO_DPS, 1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
  {"ifroll_2_gy", 'r',  ACS2_D, 10, -DGY32_TO_DPS, 1*DGY32_OFFSET*DGY32_TO_DPS, 'U', U_V_DPS},
#endif
  {"az",          'w', LOOP2,   51,             LI2DEG,             0.0, 'U', U_P_DEG},
  {"el",          'w', LOOP2,   53,             LI2DEG,             0.0, 'U', U_P_DEG},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
#ifndef BOLOTEST
/* ACS1 Digital Card */
  //TODO can heat_gy be made slow? Probably.
  {"heat_gy",      'w',  ACS1_D,  6,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Common Node */
  {"framenum",     'r',  ACS2_C,  1,                1.0,             0.0, 'u', U_NONE},

/* ACS2 Digital Card */
  {"ifel_gy",      'r',   ACS2_D, 12,GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ifroll_gy",    'r',   ACS2_D, 13,GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"ifyaw_gy",     'r',   ACS2_D, 14,GY16_TO_DPS,-GY16_OFFSET*GY16_TO_DPS, 'u', U_V_DPS},
  {"trigger_isc",  'w',   ACS2_D, 11,                 1.0,            0.0, 'u', U_NONE},
  {"trigger_osc",  'w',   ACS2_D, 12,                 1.0,            0.0, 'u', U_NONE},
  {"vel_req_el",   'w',   ACS2_D, 22, GY16_TO_DPS*0.1,-3276.8*GY16_TO_DPS, 'u', U_V_DPS},
  {"cos_el",       'w',   ACS2_D, 25,         1.0/32768.0,           -1.0, 'u', U_NONE},
  {"sin_el",       'w',   ACS2_D, 26,         1.0/32768.0,           -1.0, 'u', U_NONE},
  {"vel_req_az",   'w',   ACS2_D, 27, GY16_TO_DPS*0.1,-3276.8*GY16_TO_DPS, 'u', U_V_DPS},
  {"pulse_sc",     'r',   ACS2_D, 50,                 1.0,            0.0, 'u', U_NONE},

#endif
  {"dig43_das",    'w',   BIAS_D,  6,                1.0,             0.0, 'u', U_NONE},
  {"chopper",      'r',  CRYO_A2, 33,          CAL16(1.0,            0.0), 'u',  U_V_V},

  {"x_stage",      'w', LOOP5, 28,                2.0,             0.0, 'u', U_NONE},
  {"y_stage",      'w', LOOP5, 34,                2.0,             0.0, 'u', U_NONE},
  {"vel_rw",       'w', LOOP7,  0,         I2DEG*4.0,             0.0, 's', U_V_DPS},
  {"el_raw_enc",   'w', LOOP7,  2,             I2DEG,             0.0, 'u', U_P_DEG},
  {"el_enc",       'w', LOOP2, 47,              I2DEG,             0.0, 'u', U_P_DEG},
  {"sigma_enc",    'w', LOOP2, 48,              I2DEG,             0.0, 'u', U_NONE},
  {"chatter",      'w', LOOP7, 38,              1.0,               0.0, 'u', U_NONE},
  {"mcp_frame",    'w', LOOP2, 34,              1.0,               0.0, 'u', U_NONE},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,             1.0,                    0.0, 'u', U_NONE},
  {"polarity",    'w', DECOM,  2,             1.0,                    0.0, 'u', U_NONE},
  {"decom_unlock",'w', DECOM,  3,             1.0,                    0.0, 'u', U_NONE},
  END_OF_CHANNELS
};
