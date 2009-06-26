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

#include "channels.h"
#include "bbc_pci.h"

/* card name to (node number, bus number) mapping */
#define ACS1_C	 0, 0  /* C denotes a common motherboard node */
#define ACS1_A1	 1, 0  /* A deontes a node for analog daughter card */
#define ACS1_T1	 2, 0  /* T denotes an AD590 thermometry daughter card */
#define ACS1_D	 3, 0  /* D denotes a digital daughter card */
#define ACS2_C	 4, 0
#define ACS2_A1	 5, 0
//ACS2_unused	 6, 0
#define ACS2_D	 7, 0
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
#define DECOM	40, 0

#define CAL16(m,b) ((m)*M_16PRE), ((b) + B_16PRE*(m)*M_16PRE)
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T - 273.15)
/*******************************************************************************
 * TODO The channel list needs updating! Most of this is still from BLAST06
 *
 * Anything allocated to one of the following TMP nodes must be reassigned
 * to the correct new node above, or it must be deleted
 *
 * LOOP nodes might also need cleanup. Their channels have all been carried
 * forward, but some may no longer be needed
 *
 * TMP nodes are only so mcpol doesn't barf during development.
 * !!!!!!!!!!!!!!!!!!!!!!!!!! DO NOT FLY WITH TMP NODES !!!!!!!!!!!!!!!!!!!!!!!!
 ******************************************************************************/
#define	TMP1	50, 0	//all channels formerly on ACS0
#define	TMP2	51, 0	//all channels formerly on ACS1
#define TMP3	52, 0	//all channels formerly on ACS2
#define TMP4	53, 0	//all channels formerly on ACS3
#define TMP5	54, 0	//all channels formerly on CRYO
#define TMP6	55, 0	//all channels formerly on BIAS

/* read and write channel 56 on all boards reserved for ADC Sync */
struct ChannelStruct WideSlowChannels[] = {
  /* generic names for analog channels. BE MORE SPECIFIC 
   * If possible, make it 16 bit. Make it fast if necessary
   * Also, can maybe get rid on a TMP Channel when you do */
  {"cryo_a1_00",    'r', CRYO_A1,  0,                1.0,             0.0, 'U'},
  {"cryo_a1_01",    'r', CRYO_A1,  2,                1.0,             0.0, 'U'},
  {"cryo_a1_02",    'r', CRYO_A1,  4,                1.0,             0.0, 'U'},
  {"cryo_a1_03",    'r', CRYO_A1,  6,                1.0,             0.0, 'U'},
  {"cryo_a1_04",    'r', CRYO_A1,  8,                1.0,             0.0, 'U'},
  {"cryo_a1_05",    'r', CRYO_A1, 10,                1.0,             0.0, 'U'},
  {"cryo_a1_06",    'r', CRYO_A1, 12,                1.0,             0.0, 'U'},
  {"cryo_a1_07",    'r', CRYO_A1, 14,                1.0,             0.0, 'U'},
  {"cryo_a1_08",    'r', CRYO_A1, 16,                1.0,             0.0, 'U'},
  {"cryo_a1_09",    'r', CRYO_A1, 18,                1.0,             0.0, 'U'},
  {"cryo_a1_10",    'r', CRYO_A1, 20,                1.0,             0.0, 'U'},
  {"cryo_a1_11",    'r', CRYO_A1, 22,                1.0,             0.0, 'U'},
  {"cryo_a1_12",    'r', CRYO_A1, 24,                1.0,             0.0, 'U'},
  {"cryo_a1_13",    'r', CRYO_A1, 26,                1.0,             0.0, 'U'},
  {"cryo_a1_14",    'r', CRYO_A1, 28,                1.0,             0.0, 'U'},
  {"cryo_a1_15",    'r', CRYO_A1, 30,                1.0,             0.0, 'U'},
  {"cryo_a1_16",    'r', CRYO_A1, 32,                1.0,             0.0, 'U'},
  {"cryo_a1_17",    'r', CRYO_A1, 34,                1.0,             0.0, 'U'},
  {"cryo_a1_18",    'r', CRYO_A1, 36,                1.0,             0.0, 'U'},
  {"cryo_a1_19",    'r', CRYO_A1, 38,                1.0,             0.0, 'U'},
  {"cryo_a1_20",    'r', CRYO_A1, 40,                1.0,             0.0, 'U'},
  {"cryo_a1_21",    'r', CRYO_A1, 42,                1.0,             0.0, 'U'},
  {"cryo_a1_22",    'r', CRYO_A1, 44,                1.0,             0.0, 'U'},
  {"cryo_a1_23",    'r', CRYO_A1, 46,                1.0,             0.0, 'U'},
  {"cryo_a1_24",    'r', CRYO_A1, 48,                1.0,             0.0, 'U'},
  {"cryo_a2_00",    'r', CRYO_A2,  0,                1.0,             0.0, 'U'},
  {"cryo_a2_01",    'r', CRYO_A2,  2,                1.0,             0.0, 'U'},
  {"cryo_a2_02",    'r', CRYO_A2,  4,                1.0,             0.0, 'U'},
  {"cryo_a2_03",    'r', CRYO_A2,  6,                1.0,             0.0, 'U'},
  {"cryo_a2_04",    'r', CRYO_A2,  8,                1.0,             0.0, 'U'},
  {"cryo_a2_05",    'r', CRYO_A2, 10,                1.0,             0.0, 'U'},
  {"cryo_a2_06",    'r', CRYO_A2, 12,                1.0,             0.0, 'U'},
  {"cryo_a2_07",    'r', CRYO_A2, 14,                1.0,             0.0, 'U'},
  {"cryo_a2_08",    'r', CRYO_A2, 16,                1.0,             0.0, 'U'},
  {"cryo_a2_09",    'r', CRYO_A2, 18,                1.0,             0.0, 'U'},
  {"cryo_a2_10",    'r', CRYO_A2, 20,                1.0,             0.0, 'U'},
  {"cryo_a2_11",    'r', CRYO_A2, 22,                1.0,             0.0, 'U'},
  {"cryo_a2_12",    'r', CRYO_A2, 24,                1.0,             0.0, 'U'},
  {"cryo_a2_13",    'r', CRYO_A2, 26,                1.0,             0.0, 'U'},
  {"cryo_a2_14",    'r', CRYO_A2, 28,                1.0,             0.0, 'U'},
  {"cryo_a2_15",    'r', CRYO_A2, 30,                1.0,             0.0, 'U'},
  {"cryo_a2_16",    'r', CRYO_A2, 32,                1.0,             0.0, 'U'},
  {"cryo_a2_17",    'r', CRYO_A2, 34,                1.0,             0.0, 'U'},
  {"cryo_a2_18",    'r', CRYO_A2, 36,                1.0,             0.0, 'U'},
  {"cryo_a2_19",    'r', CRYO_A2, 38,                1.0,             0.0, 'U'},
  {"cryo_a2_20",    'r', CRYO_A2, 40,                1.0,             0.0, 'U'},
  {"cryo_a2_21",    'r', CRYO_A2, 42,                1.0,             0.0, 'U'},
  {"cryo_a2_22",    'r', CRYO_A2, 44,                1.0,             0.0, 'U'},
  {"cryo_a2_23",    'r', CRYO_A2, 46,                1.0,             0.0, 'U'},
  {"cryo_a2_24",    'r', CRYO_A2, 48,                1.0,             0.0, 'U'},

  {"cpu_time",     'w', LOOP1,  0,                1.0,             0.0, 'U'},
  {"cpu_usec",     'w', LOOP4, 58,                1.0,             0.0, 'U'},
  {"sip_time",     'w', LOOP1,  2,                1.0,             0.0, 'U'},
  {"dgps_time",    'w', LOOP1,  4,                1.0,             0.0, 'U'},
  {"lst",          'w', LOOP1,  6,         1.0/3600.0,             0.0, 'U'},
  {"isc_ra",       'w', LOOP1,  8,              LI2H,              0.0, 'U'},
  {"isc_dec",      'w', LOOP1, 10,          LI2DEG/2.,            -90., 'U'},
  {"time",         'w', LOOP1, 12,                1.0,             0.0, 'U'},
  {"isc_framenum", 'w', LOOP1, 32,                1.0,             0.0, 'U'},
  {"lat",          'w', LOOP1, 38,             LI2DEG,             0.0, 'S'},
  {"lon",          'w', LOOP1, 40,             LI2DEG,             0.0, 'S'},
  {"isc_state",    'w', LOOP2, 20,                1.0,             0.0, 'U'},
  {"isc_mcpnum",   'w', LOOP2, 60,                1.0,             0.0, 'U'},
  {"ra",           'w', LOOP3,  4,               LI2H,             0.0, 'U'},
  {"osc_ra",       'w', LOOP3, 26,               LI2H,             0.0, 'U'},
  {"osc_dec",      'w', LOOP3, 28,          LI2DEG/2.,            -90., 'U'},
  {"osc_framenum", 'w', LOOP3, 31,                1.0,             0.0, 'U'},
  {"osc_state",    'w', LOOP3, 44,                1.0,             0.0, 'U'},
  {"osc_mcpnum",   'w', LOOP3, 58,                1.0,             0.0, 'U'},
  {"cycle_start",  'w', LOOP4, 24,                1.0,             0.0, 'U'},
  {"dec",          'w', LOOP5,  6,             LI2DEG,             0.0, 'S'},
  {"lock_pos",     'w', LOOP5, 18,                1.0,             0.0, 'S'},
  {"act0_pos",     'w', LOOP5, 42,                1.0,  -ACTENC_OFFSET, 'S'},
  {"act1_pos",     'w', LOOP5, 44,                1.0,  -ACTENC_OFFSET, 'S'},
  {"act2_pos",     'w', LOOP5, 46,                1.0,  -ACTENC_OFFSET, 'S'},
  {"act0_enc",     'w', LOOP5, 48,                1.0,  -ACTENC_OFFSET, 'S'},
  {"act1_enc",     'w', LOOP5, 50,                1.0,  -ACTENC_OFFSET, 'S'},
  {"act2_enc",     'w', LOOP5, 52,                1.0,  -ACTENC_OFFSET, 'S'},
  {"sec_goal",     'w', LOOP6, 30,                1.0,             0.0, 'S'},
  {"abs_focus",    'w', LOOP6, 32,                1.0,             0.0, 'S'},
  {"sched_lst",    'w', LOOP6, 56,                1.0,             0.0, 'U'},


/* Rox a la Jeff */
  {"t_he3fridge",  'r',  TMP5,  6,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_m4",         'r',  TMP5,  8,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_m5",         'r',  TMP5, 10,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_horn_250",   'r',  TMP5, 12,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_m3",         'r',  TMP5, 14,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_horn_350",   'r',  TMP5, 38,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_300mk_strap",'r',  TMP5, 40,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_horn_500",   'r',  TMP5, 42,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_he4pot",     'r',  TMP5, 44,    ROX_C2V,   ROX_OFFSET, 'U'},
  {"t_optbox_filt",'r',  TMP5, 46,    ROX_C2V,   ROX_OFFSET, 'U'},

  END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  //status and sync channels for handshaking with bbus nodes
  {"status00",     'r',  ACS1_C, 63,                1.0,             0.0, 'u'},
  {"status01",     'r', ACS1_A1, 63,                1.0,             0.0, 'u'},
  {"status02",     'r', ACS1_T1, 63,                1.0,             0.0, 'u'},
  {"status03",     'r',  ACS1_D, 63,                1.0,             0.0, 'u'},
  {"status04",     'r',  ACS2_C, 63,                1.0,             0.0, 'u'},
  {"status05",     'r', ACS2_A1, 63,                1.0,             0.0, 'u'},
  {"status07",     'r',  ACS2_D, 63,                1.0,             0.0, 'u'},
  {"status08",     'r',  BIAS_C, 63,                1.0,             0.0, 'u'},
  {"status09",     'r',  BIAS_D, 63,                1.0,             0.0, 'u'},
  {"status10",     'r', BIAS_T1, 63,                1.0,             0.0, 'u'},
  {"status12",     'r',  CRYO_C, 63,                1.0,             0.0, 'u'},
  {"status13",     'r', CRYO_A1, 63,                1.0,             0.0, 'u'},
  {"status14",     'r', CRYO_A2, 63,                1.0,             0.0, 'u'},
  {"status16",     'r',  DAS1_C, 63,                1.0,             0.0, 'u'},
  {"status17",     'r', DAS1_A1, 63,                1.0,             0.0, 'u'},
  {"status18",     'r', DAS1_A2, 63,                1.0,             0.0, 'u'},
  {"status19",     'r', DAS1_A3, 63,                1.0,             0.0, 'u'},
  {"status20",     'r',  DAS2_C, 63,                1.0,             0.0, 'u'},
  {"status21",     'r', DAS2_A1, 63,                1.0,             0.0, 'u'},
  {"status22",     'r', DAS2_A2, 63,                1.0,             0.0, 'u'},
  {"status23",     'r', DAS2_A3, 63,                1.0,             0.0, 'u'},
  {"status24",     'r',  DAS3_C, 63,                1.0,             0.0, 'u'},
  {"status25",     'r', DAS3_A1, 63,                1.0,             0.0, 'u'},
  {"status26",     'r', DAS3_A2, 63,                1.0,             0.0, 'u'},
  {"status27",     'r', DAS3_A3, 63,                1.0,             0.0, 'u'},
  {"status28",     'r',  DAS4_C, 63,                1.0,             0.0, 'u'},
  {"status29",     'r', DAS4_A1, 63,                1.0,             0.0, 'u'},
  {"status30",     'r', DAS4_A2, 63,                1.0,             0.0, 'u'},
  {"status31",     'r', DAS4_A3, 63,                1.0,             0.0, 'u'},
  {"sync00",       'w',  ACS1_C, 63,                1.0,             0.0, 'u'},
  {"sync01",       'w', ACS1_A1, 63,                1.0,             0.0, 'u'},
  {"sync02",       'w', ACS1_T1, 63,                1.0,             0.0, 'u'},
  {"sync03",       'w',  ACS1_D, 63,                1.0,             0.0, 'u'},
  {"sync04",       'w',  ACS2_C, 63,                1.0,             0.0, 'u'},
  {"sync05",       'w', ACS2_A1, 63,                1.0,             0.0, 'u'},
  {"sync07",       'w',  ACS2_D, 63,                1.0,             0.0, 'u'},
  {"sync08",       'w',  BIAS_C, 63,                1.0,             0.0, 'u'},
  {"sync09",       'w',  BIAS_D, 63,                1.0,             0.0, 'u'},
  {"sync10",       'w', BIAS_T1, 63,                1.0,             0.0, 'u'},
  {"sync12",       'w',  CRYO_C, 63,                1.0,             0.0, 'u'},
  {"sync13",       'w', CRYO_A1, 63,                1.0,             0.0, 'u'},
  {"sync14",       'w', CRYO_A2, 63,                1.0,             0.0, 'u'},
  {"sync16",       'w',  DAS1_C, 63,                1.0,             0.0, 'u'},
  {"sync17",       'w', DAS1_A1, 63,                1.0,             0.0, 'u'},
  {"sync18",       'w', DAS1_A2, 63,                1.0,             0.0, 'u'},
  {"sync19",       'w', DAS1_A3, 63,                1.0,             0.0, 'u'},
  {"sync20",       'w',  DAS2_C, 63,                1.0,             0.0, 'u'},
  {"sync21",       'w', DAS2_A1, 63,                1.0,             0.0, 'u'},
  {"sync22",       'w', DAS2_A2, 63,                1.0,             0.0, 'u'},
  {"sync23",       'w', DAS2_A3, 63,                1.0,             0.0, 'u'},
  {"sync24",       'w',  DAS3_C, 63,                1.0,             0.0, 'u'},
  {"sync25",       'w', DAS3_A1, 63,                1.0,             0.0, 'u'},
  {"sync26",       'w', DAS3_A2, 63,                1.0,             0.0, 'u'},
  {"sync27",       'w', DAS3_A3, 63,                1.0,             0.0, 'u'},
  {"sync28",       'w',  DAS4_C, 63,                1.0,             0.0, 'u'},
  {"sync29",       'w', DAS4_A1, 63,                1.0,             0.0, 'u'},
  {"sync30",       'w', DAS4_A2, 63,                1.0,             0.0, 'u'},
  {"sync31",       'w', DAS4_A3, 63,                1.0,             0.0, 'u'},

  {"phase17",      'w', DAS1_A1,  8,                1.0,             0.0, 'u'},
  {"phase18",      'w', DAS1_A2,  8,                1.0,             0.0, 'u'},
  {"phase19",      'w', DAS1_A3,  8,                1.0,             0.0, 'u'},
  {"phase21",      'w', DAS2_A1,  8,                1.0,             0.0, 'u'},
  {"phase22",      'w', DAS2_A2,  8,                1.0,             0.0, 'u'},
  {"phase23",      'w', DAS2_A3,  8,                1.0,             0.0, 'u'},
  {"phase25",      'w', DAS3_A1,  8,                1.0,             0.0, 'u'},
  {"phase26",      'w', DAS3_A2,  8,                1.0,             0.0, 'u'},
  {"phase27",      'w', DAS3_A3,  8,                1.0,             0.0, 'u'},
  {"phase29",      'w', DAS4_A1,  8,                1.0,             0.0, 'u'},
  {"phase30",      'w', DAS4_A2,  8,                1.0,             0.0, 'u'},
  {"phase31",      'w', DAS4_A3,  8,                1.0,             0.0, 'u'},

  {"bias1_ampl",   'w',  BIAS_D,  0,                1.0,             0.0, 'u'},
  {"bias2_ampl",   'w',  BIAS_D,  1,                1.0,             0.0, 'u'},
  {"bias3_ampl",   'w',  BIAS_D,  2,                1.0,             0.0, 'u'},
  {"bias4_ampl",   'w',  BIAS_D,  3,                1.0,             0.0, 'u'},
  {"bias5_ampl",   'w',  BIAS_D,  4,                1.0,             0.0, 'u'},
  {"das_dig21",    'w',  BIAS_D,  5,                1.0,             0.0, 'u'},
  {"das_dig43",    'w',  BIAS_D,  6,                1.0,             0.0, 'u'},
  {"das_dig65",    'w',  BIAS_D,  7,                1.0,             0.0, 'u'},
  {"bias_ramp_ena",'w',  BIAS_D,  8,                1.0,             0.0, 'u'},
  {"ramp_ampl",    'r',  BIAS_D,  0,                1.0,             0.0, 'u'},

  /* generic names for analog channels. BE MORE SPECIFIC 
   * Also, can maybe get rid on a TMP Channel when you do */
  {"das_t00",      'r', BIAS_T1,  1,                1.0,             0.0, 'u'},
  {"das_t01",      'r', BIAS_T1,  3,                1.0,             0.0, 'u'},
  {"das_t02",      'r', BIAS_T1,  5,                1.0,             0.0, 'u'},
  {"das_t03",      'r', BIAS_T1,  7,                1.0,             0.0, 'u'},
  {"das_t04",      'r', BIAS_T1,  9,                1.0,             0.0, 'u'},
  {"das_t05",      'r', BIAS_T1, 11,                1.0,             0.0, 'u'},
  {"das_t06",      'r', BIAS_T1, 13,                1.0,             0.0, 'u'},
  {"das_t07",      'r', BIAS_T1, 15,                1.0,             0.0, 'u'},
  {"das_t08",      'r', BIAS_T1, 17,                1.0,             0.0, 'u'},
  {"das_t09",      'r', BIAS_T1, 19,                1.0,             0.0, 'u'},
  {"das_t10",      'r', BIAS_T1, 21,                1.0,             0.0, 'u'},
  {"das_t11",      'r', BIAS_T1, 23,                1.0,             0.0, 'u'},
  {"das_t12",      'r', BIAS_T1, 25,                1.0,             0.0, 'u'},
  {"das_t13",      'r', BIAS_T1, 27,                1.0,             0.0, 'u'},
  {"das_t14",      'r', BIAS_T1, 29,                1.0,             0.0, 'u'},
  {"das_t15",      'r', BIAS_T1, 31,                1.0,             0.0, 'u'},
  {"das_t16",      'r', BIAS_T1, 33,                1.0,             0.0, 'u'},
  {"das_t17",      'r', BIAS_T1, 35,                1.0,             0.0, 'u'},
  {"das_t18",      'r', BIAS_T1, 37,                1.0,             0.0, 'u'},
  {"das_t19",      'r', BIAS_T1, 39,                1.0,             0.0, 'u'},
  {"das_t20",      'r', BIAS_T1, 41,                1.0,             0.0, 'u'},
  {"das_t21",      'r', BIAS_T1, 43,                1.0,             0.0, 'u'},
  {"das_t22",      'r', BIAS_T1, 45,                1.0,             0.0, 'u'},
  {"das_t23",      'r', BIAS_T1, 47,                1.0,             0.0, 'u'},
  {"das_t24",      'r', BIAS_T1, 49,                1.0,             0.0, 'u'},

  /* LOOP1 0-13 are wide */
  {"g_i_gyheat1",  'w', LOOP1, 14,                1.0,             0.0, 'u'},
  {"g_d_gyheat1",  'w', LOOP1, 15,                1.0,             0.0, 'u'},
  {"lokmot_pin",   'w', LOOP1, 16,                1.0,             0.0, 'u'},
  {"isc_fpulse",   'w', LOOP1, 17,                10.,             0.0, 'u'},
  {"cal_repeat",   'w', LOOP1, 18,                .20,             0.0, 'u'},
  {"alice_file",   'w', LOOP1, 19,                1.0,             0.0, 'u'},
  {"timeout",      'w', LOOP1, 20,                1.0,             0.0, 'u'},
  {"sun_az",       'w', LOOP1, 21,              I2DEG,             0.0, 'u'},
  {"lvdt_low",     'w', LOOP1, 22,                1.0,             0.0, 's'},
  {"sam_i_am",     'w', LOOP1, 23,                1.0,             0.0, 'u'},
  {"cryostate",    'w', LOOP1, 24,                1.0,             0.0, 'u'},
  {"cpu_temp1",    'w', LOOP1, 26,               0.01,             0.0, 'u'},
  {"mag_model",    'w', LOOP1, 27,              I2DEG,             0.0, 'u'},
  {"sensor_veto",  'w', LOOP1, 28,                1.0,             0.0, 'u'},
  {"bal_on",       'w', LOOP1, 29,           1./1648.,             0.0, 'u'},
  {"bal_off",      'w', LOOP1, 30,           1./1648.,             0.0, 'u'},
  {"bal_target",   'w', LOOP1, 31,           1./1648.,            -5.0, 'u'},
  /* LOOP1 32-33 are wide */
  {"bal_veto",     'w', LOOP1, 34,                1.0,             0.0, 's'},
  {"sip_alt",      'w', LOOP1, 37,                1.0,             0.0, 'u'},
  /* LOOP1 38-41 are wide */
  {"isc_mapmean",  'w', LOOP1, 42,                 1.,             0.0, 'u'},
  {"dgps_pitch",   'w', LOOP1, 43,              I2DEG,             0.0, 's'},
  {"dgps_roll",    'w', LOOP1, 44,              I2DEG,             0.0, 's'},
  {"sip_lat",      'w', LOOP1, 45,              I2DEG,             0.0, 's'},
  {"sip_lon",      'w', LOOP1, 46,              I2DEG,             0.0, 's'},
  {"dgps_lat",     'w', LOOP1, 47,              I2DEG,             0.0, 's'},
  {"dgps_lon",     'w', LOOP1, 48,              I2DEG,             0.0, 's'},
  {"dgps_alt",     'w', LOOP1, 49,                1.0,             0.0, 'u'},
  {"dgps_speed",   'w', LOOP1, 50,              I2DEG,             0.0, 'u'},
  {"dgps_dir",     'w', LOOP1, 51,              I2DEG,             0.0, 'u'},
  {"dgps_climb",   'w', LOOP1, 52,              I2DEG,             0.0, 's'},
  {"dgps_att_ok",  'w', LOOP1, 53,                1.0,             0.0, 'u'},
  {"dgps_att_index",'w',LOOP1, 54,                1.0,             0.0, 'u'},
  {"dgps_pos_index",'w',LOOP1, 55,                1.0,             0.0, 'u'},
  //{"outcool_state",'w', LOOP1, 56,                1.0,             0.0, 's'},
  {"dgps_n_sat",   'w', LOOP1, 57,                1.0,             0.0, 'u'},
  {"disk_free",    'w', LOOP1, 58,             1./250,             0.0, 'u'},
  {"p_mode",       'w', LOOP1, 59,                  1,             0.0, 'u'},
  {"p_x_deg",      'w', LOOP1, 60,              I2DEG,             0.0, 'u'},
  {"p_y",          'w', LOOP1, 61,              I2DEG,             0.0, 's'},
  {"p_vaz",        'w', LOOP1, 62,              I2VEL,             0.0, 'u'},
  {"p_del",        'w', LOOP1, 63,              I2VEL,             0.0, 'u'},

  {"isc_blob_idx", 'w', LOOP2,  0,                1.0,             0.0, 'u'},
  {"isc_blob00_x", 'w', LOOP2,  1,             1./40.,             0.0, 'u'},
  {"isc_blob00_y", 'w', LOOP2,  2,             1./40.,             0.0, 'u'},
  {"isc_blob00_f", 'w', LOOP2,  3,                1.0,             0.0, 'u'},
  {"isc_blob00_s", 'w', LOOP2,  4,       1000./65536.,             0.0, 'u'},
  {"isc_blob01_x", 'w', LOOP2,  5,             1./40.,             0.0, 'u'},
  {"isc_blob01_y", 'w', LOOP2,  6,             1./40.,             0.0, 'u'},
  {"isc_blob01_f", 'w', LOOP2,  7,                1.0,             0.0, 'u'},
  {"isc_blob01_s", 'w', LOOP2,  8,       1000./65536.,             0.0, 'u'},
  {"isc_blob02_x", 'w', LOOP2,  9,             1./40.,             0.0, 'u'},
  {"isc_blob02_y", 'w', LOOP2, 10,             1./40.,             0.0, 'u'},
  {"isc_blob02_f", 'w', LOOP2, 11,                1.0,             0.0, 'u'},
  {"isc_blob02_s", 'w', LOOP2, 12,       1000./65536.,             0.0, 'u'},
  {"p_w",          'w', LOOP2, 13,              I2DEG,             0.0, 'u'},
  {"isc_rtol",     'w', LOOP2, 14,              I2DEG,             0.0, 'u'},
  {"isc_apert",    'w', LOOP2, 15,                1.0,             0.0, 'u'},
  {"isc_maglimit", 'w', LOOP2, 16,           1./1000.,             0.0, 'u'},
  {"isc_nrad",     'w', LOOP2, 17,              I2DEG,             0.0, 'u'},
  {"isc_mtol",     'w', LOOP2, 18,        100./65536.,             0.0, 'u'},
  {"isc_qtol",     'w', LOOP2, 19,        100./65536.,             0.0, 'u'},
  /* LOOP2 20-21 is wide */
  {"isc_lrad",     'w', LOOP2, 22,              I2DEG,             0.0, 'u'},
  {"isc_thresh",   'w', LOOP2, 23,             1./10.,             0.0, 'u'},
  {"isc_grid",     'w', LOOP2, 24,                1.0,             0.0, 'u'},
  {"gy1_h_age",    'w', LOOP2, 25,                1.0,             0.0, 'u'},
  {"osc_real_trig",'w', LOOP2, 26,                1.0,             0.0, 's'},
  {"isc_mdist",    'w', LOOP2, 27,                1.0,             0.0, 'u'},
  {"isc_nblobs",   'w', LOOP2, 28,                1.0,             0.0, 'u'},
  {"t_gy1_set",    'w', LOOP2, 29,    (100.0/32768.0),             0.0, 'u'},
  {"g_p_gyheat1",  'w', LOOP2, 30,                1.0,             0.0, 'u'},
  {"osc_foc_off",  'w', LOOP2, 31,                1.0,             0.0, 's'},
  {"gy1_h_hist",   'w', LOOP2, 32,    (100.0/32768.0),             0.0, 'u'},
  {"isc_tol",      'w', LOOP2, 33,                1.0,             0.0, 'u'},
  /* LOOP2 34 is fast */
  {"ss_az",        'w', LOOP2, 35,              I2DEG,             0.0, 'u'},
  {"gy1_offset",   'w', LOOP2, 36,        1.0/32768.0,             0.0, 's'},
  {"gy2_offset",   'w', LOOP2, 37,        1.0/32768.0,             0.0, 's'},
  {"gy3_offset",   'w', LOOP2, 38,        1.0/32768.0,             0.0, 's'},
  {"mag_sigma",    'w', LOOP2, 40,              I2DEG,             0.0, 'u'},
  {"dgps_az",      'w', LOOP2, 41,              I2DEG,             0.0, 'u'},
  {"dgps_sigma",   'w', LOOP2, 42,              I2DEG,             0.0, 'u'},
  {"lvdt_high",    'w', LOOP2, 43,                1.0,             0.0, 's'},
  {"isc_az",       'w', LOOP2, 44,              I2DEG,             0.0, 'u'},
  {"isc_el",       'w', LOOP2, 45,              I2DEG,             0.0, 'u'},
  {"isc_sigma",    'w', LOOP2, 46,              I2DEG,             0.0, 'u'},
  {"enc_el",       'w', LOOP2, 47,              I2DEG,             0.0, 'u'},
  {"enc_sigma",    'w', LOOP2, 48,              I2DEG,             0.0, 'u'},
  {"cal_pulse",    'w', LOOP2, 49,               10.0,              0., 'u'},
  {"ss_sigma",     'w', LOOP2, 50,              I2DEG,             0.0, 'u'},
  /* LOOP2 51-54 are wide fast */
  {"clin_sigma",   'w', LOOP2, 55,              I2DEG,             0.0, 'u'},
  {"mag_az",       'w', LOOP2, 56,              I2DEG,             0.0, 'u'},
  {"isc_spulse",   'w', LOOP2, 57,               10.0,             0.0, 'u'},
  {"isc_hx_flag",  'w', LOOP2, 58,                1.0,             0.0, 'u'},
  {"isc_brra",     'w', LOOP2, 59,              I2DEG,             0.0, 'u'},
  /* LOOP2 60-61 are wide */
  {"isc_brdec",    'w', LOOP2, 62,              I2DEG,             0.0, 'u'},
  {"isc_x_off",    'w', LOOP2, 63,              I2DEG,             0.0, 's'},

  {"osc_gain",     'w', LOOP3,  0,        100./65536.,             0.0, 'u'},
  {"isc_hold_i",   'w', LOOP3,  1,                1.0,             0.0, 'u'},
  {"isc_save_prd", 'w', LOOP3,  2,               0.01,             0.0, 'u'},
  {"isc_y_off",    'w', LOOP3,  3,              I2DEG,             0.0, 's'},
  /* LOOP2 4-5 are wide */
  {"isc_offset",   'w', LOOP3,  6,                1.0,             0.0, 's'},
  {"bbc_fifo_size",'w', LOOP3,  7,             1./624,             0.0, 'u'},
  {"cpu_temp2",    'w', LOOP3,  8,               0.01,             0.0, 'u'},
  {"cpu_temp3",    'w', LOOP3,  9,               0.01,             0.0, 'u'},
  {"sip_mks_hi",   'w', LOOP3, 10,           0.003256,       -0.226858, 'u'},
  {"sip_mks_med",  'w', LOOP3, 11,           0.032614,       -0.072580, 'u'},
  {"osc_blob_idx", 'w', LOOP3, 12,                1.0,             0.0, 'u'},
  {"osc_blob00_x", 'w', LOOP3, 13,             1./40.,             0.0, 'u'},
  {"osc_blob00_y", 'w', LOOP3, 14,             1./40.,             0.0, 'u'},
  {"osc_blob00_f", 'w', LOOP3, 15,                1.0,             0.0, 'u'},
  {"osc_blob00_s", 'w', LOOP3, 16,             1./40.,             0.0, 'u'},
  {"osc_blob01_x", 'w', LOOP3, 17,             1./40.,             0.0, 'u'},
  {"osc_blob01_y", 'w', LOOP3, 18,             1./40.,             0.0, 'u'},
  {"osc_blob01_f", 'w', LOOP3, 19,                1.0,             0.0, 'u'},
  {"osc_blob01_s", 'w', LOOP3, 20,             1./40.,             0.0, 'u'},
  {"osc_blob02_x", 'w', LOOP3, 21,             1./40.,             0.0, 'u'},
  {"osc_blob02_y", 'w', LOOP3, 22,             1./40.,             0.0, 'u'},
  {"osc_blob02_f", 'w', LOOP3, 23,                1.0,             0.0, 'u'},
  {"osc_blob02_s", 'w', LOOP3, 24,             1./40.,             0.0, 'u'},
  {"osc_mapmean",  'w', LOOP3, 25,                 1.,             0.0, 'u'},
  /* LOOP3 26-29 are wide */
  {"osc_fpulse",   'w', LOOP3, 30,                10.,             0.0, 'u'},
  /* LOOP3 31-32 are wide */
  {"osc_az",       'w', LOOP3, 33,              I2DEG,             0.0, 'u'},
  {"osc_el",       'w', LOOP3, 34,              I2DEG,             0.0, 'u'},
  {"osc_sigma",    'w', LOOP3, 35,              I2DEG,             0.0, 'u'},
  {"osc_tol",      'w', LOOP3, 36,                1.0,             0.0, 'u'},
  {"osc_apert",    'w', LOOP3, 37,                1.0,             0.0, 'u'},
  {"osc_maglimit", 'w', LOOP3, 38,           1./1000.,             0.0, 'u'},
  {"osc_nrad",     'w', LOOP3, 39,              I2DEG,             0.0, 'u'},
  {"osc_mtol",     'w', LOOP3, 40,        100./65536.,             0.0, 'u'},
  {"osc_qtol",     'w', LOOP3, 41,        100./65536.,             0.0, 'u'},
  {"osc_offset",   'w', LOOP3, 42,                1.0,             0.0, 's'},
  {"osc_lrad",     'w', LOOP3, 43,              I2DEG,             0.0, 'u'},
  /* LOOP3 44-45 are wide */
  {"osc_thresh",   'w', LOOP3, 46,             1./10.,             0.0, 'u'},
  {"osc_grid",     'w', LOOP3, 47,                1.0,             0.0, 'u'},
  {"isc_real_trig",'w', LOOP3, 48,                1.0,             0.0, 's'},
  {"isc_foc_off",  'w', LOOP3, 49,                1.0,             0.0, 's'},
  {"osc_mdist",    'w', LOOP3, 50,                1.0,             0.0, 'u'},
  {"osc_nblobs",   'w', LOOP3, 51,                1.0,             0.0, 'u'},
  {"osc_rtol",     'w', LOOP3, 52,              I2DEG,             0.0, 'u'},
  {"osc_rd_sigma", 'w', LOOP3, 53,                1.0,             0.0, 'u'},
  {"osc_spulse",   'w', LOOP3, 54,               10.0,             0.0, 'u'},
  {"osc_hx_flag",  'w', LOOP3, 55,                1.0,             0.0, 'u'},
  {"osc_brra",     'w', LOOP3, 56,              I2DEG,             0.0, 'u'},
  {"osc_brdec",    'w', LOOP3, 57,              I2DEG,             0.0, 'u'},
  /* LOOP3 58-59 are wide */
  {"osc_x_off",    'w', LOOP3, 60,              I2DEG,             0.0, 's'},
  {"osc_hold_i",   'w', LOOP3, 61,                1.0,             0.0, 'u'},
  {"osc_save_prd", 'w', LOOP3, 62,               0.01,             0.0, 'u'},
  {"osc_y_off",    'w', LOOP3, 63,              I2DEG,             0.0, 's'},

  {"ss_phase",     'w', LOOP4,  0,              I2DEG,             0.0, 's'},
  {"tc_pref_tp",   'w', LOOP4,  1,                1.0,             0.0, 'u'},
  {"tc_filter",    'w', LOOP4,  2,                0.2,             0.0, 'u'},
  {"ss_sun_time",  'w', LOOP4,  3,                1.0,             0.0, 'u'},
  {"ss_cpu_temp",  'w', LOOP4,  4,             1/100.,         -273.15, 'u'},
  {"ss_hdd_temp",  'w', LOOP4,  5,             1/100.,         -273.15, 'u'},
  {"isc_maxblobs", 'w', LOOP4,  6,                1.0,             0.0, 'u'},
  {"osc_maxblobs", 'w', LOOP4,  7,                1.0,             0.0, 'u'},
  {"bi0_fifo_size",'w', LOOP4,  8,             1./624,             0.0, 'u'},
  {"plover",       'w', LOOP4,  9,                1.0,             0.0, 'u'},
  {"t_isc_flange", 'w', LOOP4, 10,            1./200.,         -273.15, 'u'},
  {"t_isc_lens",   'w', LOOP4, 11,            1./200.,         -273.15, 'u'},
  {"t_isc_heat",   'w', LOOP4, 12,            1./200.,         -273.15, 'u'},
  {"t_isc_comp",   'w', LOOP4, 13,            1./200.,         -273.15, 'u'},
  {"isc_pressure1",'w', LOOP4, 14,           1./2000.,             0.0, 'u'},
  {"t_osc_flange", 'w', LOOP4, 15,            1./200.,         -273.15, 'u'},
  {"t_osc_lens",   'w', LOOP4, 16,            1./200.,         -273.15, 'u'},
  {"t_osc_heat",   'w', LOOP4, 17,            1./200.,         -273.15, 'u'},
  {"t_osc_comp",   'w', LOOP4, 18,            1./200.,         -273.15, 'u'},
  {"osc_pressure1",'w', LOOP4, 19,           1./2000.,             0.0, 'u'},
  {"isc_gain",     'w', LOOP4, 20,        100./65536.,             0.0, 'u'},
  {"jfet_set_on",  'w', LOOP4, 21,             1/100.,             0.0, 'u'},
  {"jfet_set_off", 'w', LOOP4, 22,             1/100.,             0.0, 'u'},
  {"ss_case_temp", 'w', LOOP4, 23,             1/100.,         -273.15, 'u'},
  /* LOOP4 24-25 are wide */
  {"cycle_state",  'w', LOOP4, 26,                1.0,             0.0, 'u'},
  {"isc_trig_type",'w', LOOP4, 27,                1.0,             0.0, 'u'},
  {"isc_exposure", 'w', LOOP4, 28,               100.,             0.0, 'u'},
  {"osc_trig_type",'w', LOOP4, 29,                1.0,             0.0, 'u'},
  {"osc_exposure", 'w', LOOP4, 30,               100.,             0.0, 'u'},
  {"isc_fieldrot", 'w', LOOP4, 31,              I2DEG,             0.0, 's'},
  {"osc_fieldrot", 'w', LOOP4, 32,              I2DEG,             0.0, 's'},
  {"g_p_gyheat2",  'w', LOOP4, 33,                1.0,             0.0, 'u'},
  {"g_i_gyheat2",  'w', LOOP4, 34,                1.0,             0.0, 'u'},
  {"g_d_gyheat2",  'w', LOOP4, 35,                1.0,             0.0, 'u'},
  {"t_gy2_set",    'w', LOOP4, 36,    (100.0/32768.0),             0.0, 'u'},
  {"gy2_h_age",    'w', LOOP4, 37,                1.0,             0.0, 'u'},
  {"gy2_h_hist",   'w', LOOP4, 38,    (100.0/32768.0),             0.0, 'u'},
  {"incool_state", 'w', LOOP4, 39,                1.0,             0.0, 's'},
  {"apcu_trim",    'w', LOOP4, 40,                0.01,            0.0, 's'},
  {"dpcu_trim",    'w', LOOP4, 41,                0.01,             0.0, 's'},
  {"apcu_auto",    'w', LOOP4, 42,                1.0,             0.0, 'u'},
  {"dpcu_auto",    'w', LOOP4, 43,                1.0,             0.0, 'u'},
  {"p_ra_1",       'w', LOOP4, 44,                I2H,             0.0, 'u'},
  {"p_dec_1",      'w', LOOP4, 45,              I2DEG,             0.0, 's'},
  {"p_ra_2",       'w', LOOP4, 46,                I2H,             0.0, 'u'},
  {"p_dec_2",      'w', LOOP4, 47,              I2DEG,             0.0, 's'},
  {"p_ra_3",       'w', LOOP4, 48,                I2H,             0.0, 'u'},
  {"p_dec_3",      'w', LOOP4, 49,              I2DEG,             0.0, 's'},
  {"p_ra_4",       'w', LOOP4, 50,                I2H,             0.0, 'u'},
  {"p_dec_4",      'w', LOOP4, 51,              I2DEG,             0.0, 's'},
  {"clin_trim",    'w', LOOP4, 52,              I2DEG,             0.0, 's'},
  {"enc_trim",     'w', LOOP4, 53,              I2DEG,             0.0, 's'},
  {"null_trim",    'w', LOOP4, 54,              I2DEG,             0.0, 's'},
  {"mag_trim",     'w', LOOP4, 55,              I2DEG,             0.0, 's'},
  {"dgps_trim",    'w', LOOP4, 56,              I2DEG,             0.0, 's'},
  {"ss_trim",      'w', LOOP4, 57,              I2DEG,             0.0, 's'},
  /* LOOP4 58-59 are wide */
  {"dgps_az_raw",  'w', LOOP4, 60,              I2DEG,             0.0, 'u'},
  {"bal_gain",     'w', LOOP4, 61,            1/1000.,             0.0, 'u'},
  {"clin_el",      'w', LOOP4, 62,              I2DEG,             0.0, 'u'},
  {"p_h",          'w', LOOP4, 63,              I2DEG,             0.0, 'u'},

  {"isc_error",    'w', LOOP5,  0,                 1.,             0.0, 'u'},
  {"schedule",     'w', LOOP5,  1,                 1.,              0., 'u'},
  {"isc_rd_sigma", 'w', LOOP5,  2,                1.0,             0.0, 'u'},
  {"sip_mks_lo",   'w', LOOP5,  3,           0.327045,       -5.944902, 'u'},
  {"osc_error",    'w', LOOP5,  5,                 1.,             0.0, 'u'},
  /* LOOP5 6-7 are wide */
  {"alt",          'w', LOOP5,  8,                1.0,             0.0, 'u'},
  {"az_mode",      'w', LOOP5,  9,                1.0,             0.0, 'u'},
  {"el_mode",      'w', LOOP5, 10,                1.0,             0.0, 'u'},
  {"az_dest",      'w', LOOP5, 11,              I2DEG,             0.0, 'u'},
  {"el_dest",      'w', LOOP5, 12,              I2DEG,             0.0, 'u'},
  {"az_vel",       'w', LOOP5, 13,            1./6000,             0.0, 's'},
  {"el_vel",       'w', LOOP5, 14,            1./6000,             0.0, 's'},
  {"az_dir",       'w', LOOP5, 15,                1.0,             0.0, 's'},
  {"el_dir",       'w', LOOP5, 16,                1.0,             0.0, 's'},
  {"slew_veto",    'w', LOOP5, 17,           4.0 / SR,             0.0, 'u'},
  /* LOOP5 18-19 are wide */
  {"sveto_len",    'w', LOOP5, 20,           4.0 / SR,             0.0, 'u'},
  {"act1_dead_rec",'w', LOOP5, 21,                1.0,             0.0, 's'},
  {"lock_pot",     'w', LOOP5, 22,          -0.007425,     121.5325215, 'u'},
  {"lock_lim_sw",  'w', LOOP5, 23,                1.0,             0.0, 'u'},
  {"act2_dead_rec",'w', LOOP5, 24,                1.0,             0.0, 's'},
  {"lock_state",   'w', LOOP5, 25,                1.0,             0.0, 'u'},
  {"lock_goal",    'w', LOOP5, 26,                1.0,             0.0, 'u'},
  {"seized_bus",   'w', LOOP5, 27,                1.0,             0.0, 's'},
  /* LOOP5 29-30 are wide */
  {"stage_x_vel",  'w', LOOP5, 30,                1.0,             0.0, 'u'},
  {"stage_x_stp",  'w', LOOP5, 31,                1.0,             0.0, 'u'},
  {"stage_x_str",  'w', LOOP5, 32,                1.0,             0.0, 'u'},
  {"stage_y_lim",  'w', LOOP5, 33,                1.0,             0.0, 'u'},
  /* LOOP5 34-35 are wide */
  {"stage_y_stp",  'w', LOOP5, 36,                1.0,             0.0, 'u'},
  {"stage_y_str",  'w', LOOP5, 37,                1.0,             0.0, 'u'},
  {"stage_x_lim",  'w', LOOP5, 38,                1.0,             0.0, 'u'},
  {"stage_y_vel",  'w', LOOP5, 39,                1.0,             0.0, 'u'},
  {"he4_lev_old",  'w', LOOP5, 40, -2.87477e-09*65536,      12.3273561, 'u'},
  {"isc_focus",    'w', LOOP5, 41,                1.0,             0.0, 's'},
  /* LOOP5 42-53 are wide */
  {"osc_focus",    'w', LOOP5, 54,                1.0,             0.0, 's'},
  {"mag_pitch",    'w', LOOP5, 55,              I2DEG,             0.0, 'u'},
  {"isc_diskfree", 'w', LOOP5, 56,                5.0,             0.0, 'u'},
  {"osc_diskfree", 'w', LOOP5, 57,                5.0,             0.0, 'u'},
  {"isc_gy1_off",  'w', LOOP5, 58,        1.0/32768.0,             0.0, 's'},
  {"osc_gy1_off",  'w', LOOP5, 59,        1.0/32768.0,             0.0, 's'},
  {"isc_gy2_off",  'w', LOOP5, 60,        1.0/32768.0,             0.0, 's'},
  {"osc_gy2_off",  'w', LOOP5, 61,        1.0/32768.0,             0.0, 's'},
  {"isc_gy3_off",  'w', LOOP5, 62,        1.0/32768.0,             0.0, 's'},
  {"osc_gy3_off",  'w', LOOP5, 63,        1.0/32768.0,             0.0, 's'},

  {"isc_maxslew",  'w', LOOP6,  0,              I2DEG,             0.0, 'u'},
  {"osc_maxslew",  'w', LOOP6,  1,              I2DEG,             0.0, 'u'},
  {"ss_port_temp", 'w', LOOP6,  2,             1/100.,         -273.15, 'u'},
  {"ss_star_temp", 'w', LOOP6,  3,             1/100.,         -273.15, 'u'},
  {"ss_v_5",       'w', LOOP6,  4,             1/100.,             0.0, 'u'},
  {"ss_v_12",      'w', LOOP6,  5,             1/100.,             0.0, 'u'},
  {"ss_v_batt",    'w', LOOP6,  6,             1/100.,             0.0, 'u'},
  {"ss_raw_01",    'w', LOOP6,  7,                1.0,             0.0, 'u'},
  {"ss_raw_02",    'w', LOOP6,  8,                1.0,             0.0, 'u'},
  {"ss_raw_03",    'w', LOOP6,  9,                1.0,             0.0, 'u'},
  {"ss_raw_04",    'w', LOOP6, 10,                1.0,             0.0, 'u'},
  {"ss_raw_05",    'w', LOOP6, 11,                1.0,             0.0, 'u'},
  {"ss_raw_06",    'w', LOOP6, 12,                1.0,             0.0, 'u'},
  {"ss_raw_07",    'w', LOOP6, 13,                1.0,             0.0, 'u'},
  {"ss_raw_08",    'w', LOOP6, 14,                1.0,             0.0, 'u'},
  {"ss_raw_09",    'w', LOOP6, 15,                1.0,             0.0, 'u'},
  {"ss_raw_10",    'w', LOOP6, 16,                1.0,             0.0, 'u'},
  {"ss_raw_11",    'w', LOOP6, 17,                1.0,             0.0, 'u'},
  {"ss_raw_12",    'w', LOOP6, 18,                1.0,             0.0, 'u'},
  {"ss_az_rel_sun",'w', LOOP6, 19,              I2DEG,             0.0, 'u'},
  {"tc_pref_ts",   'w', LOOP6, 20,                1.0,             0.0, 'u'},
  {"tc_spread",    'w', LOOP6, 21,             1/500.,             0.0, 'u'},
  {"lock_acc",     'w', LOOP6, 22,                1.0,             0.0, 'u'},
  {"ss_snr",       'w', LOOP6, 23,            1/1000.,             0.0, 'u'},
  {"focus_veto",   'w', LOOP6, 24,                1.0,             0.0, 'u'},
  {"act_move_i",   'w', LOOP6, 25,                1.0,             0.0, 'u'},
  {"act_hold_i",   'w', LOOP6, 26,                1.0,             0.0, 'u'},
  {"act_vel",      'w', LOOP6, 27,                1.0,             0.0, 'u'},
  {"act_acc",      'w', LOOP6, 28,                1.0,             0.0, 'u'},
  {"lock_move_i",  'w', LOOP6, 29,                1.0,             0.0, 'u'},
  /* LOOP6 30-33 are wide */
  {"lock_hold_i",  'w', LOOP6, 34,                1.0,             0.0, 'u'},
  {"lock_vel",     'w', LOOP6, 35,               100.,             0.0, 'u'},
  {"tc_g_prim",    'w', LOOP6, 36,               0.01,             0.0, 's'},
  {"tc_g_sec",     'w', LOOP6, 37,               0.01,             0.0, 's'},
  {"tc_step",      'w', LOOP6, 38,                1.0,             0.0, 'u'},
  {"tc_wait",      'w', LOOP6, 39,              1/30.,             0.0, 'u'},
  {"tc_mode",      'w', LOOP6, 40,                1.0,             0.0, 'u'},
  {"sf_correction",'w', LOOP6, 41,                1.0,             0.0, 's'},
  {"sf_age",       'w', LOOP6, 42,              1/30.,             0.0, 'u'},
  {"sf_offset",    'w', LOOP6, 43,                1.0,             0.0, 's'},
  {"t_prime_fid",  'w', LOOP6, 44,             1/500.,             0.0, 's'},
  {"t_second_fid", 'w', LOOP6, 45,             1/500.,             0.0, 's'},
  {"act0_l_good",  'w', LOOP6, 46,                1.0,             0.0, 's'},
  {"act1_l_good",  'w', LOOP6, 47,                1.0,             0.0, 's'},
  {"act2_l_good",  'w', LOOP6, 48,                1.0,             0.0, 's'},
  {"act0_dead_rec",'w', LOOP6, 49,                1.0,             0.0, 's'},
  {"act_flags",    'w', LOOP6, 50,                1.0,             0.0, 'u'},
  {"focus_mode",   'w', LOOP6, 51,                1.0,             0.0, 'u'},
  {"act0_postrim", 'w', LOOP6, 52,                1.0,             0.0, 'u'},
  {"act1_postrim", 'w', LOOP6, 53,                1.0,             0.0, 'u'},
  {"act2_postrim", 'w', LOOP6, 54,                1.0,             0.0, 'u'},
  {"lvdt_spread",  'w', LOOP6, 55,                1.0,             0.0, 's'},
  /* LOOP6 56-57 are wide */
  {"sun_el",       'w', LOOP6, 58,              I2DEG,             0.0, 's'},
  {"at_float",     'w', LOOP6, 59,                1.0,             0.0, 'u'},
  {"cal_mode",     'w', LOOP6, 60,                1.0,             0.0, 'u'},
  {"isc_minblobs", 'w', LOOP6, 61,                1.0,             0.0, 'u'},
  {"osc_minblobs", 'w', LOOP6, 62,                1.0,             0.0, 'u'},

#ifndef BOLOTEST
/* ACS1 Analog card 1 */
  {"i_trans",      'r',  ACS1_A1, 1,          CAL16(1.0, 0.0), 		'u'},
  {"i_das",        'r',  ACS1_A1, 3,          CAL16(1.0, 0.0), 		'u'},
  {"i_acs",        'r',  ACS1_A1, 5,          CAL16(1.0, 0.0), 		'u'},
  {"i_rec",        'r',  ACS1_A1, 7,          CAL16(1.0, 0.0),		'u'},
  {"i_starcam",    'r',  ACS1_A1, 9,          CAL16(1.0, 0.0), 		'u'},
  {"i_dgps",       'r',  ACS1_A1, 11,         CAL16(1.0, 0.0),		'u'},
  {"i_el",         'r',  ACS1_A1, 13,         CAL16(1.0, 0.0), 		'u'},
  {"i_piv",        'r',  ACS1_A1, 15,         CAL16(1.0, 0.0), 		'u'},
  {"i_reac",       'r',  ACS1_A1, 17,         CAL16(1.0, 0.0), 		'u'},
  {"i_step",       'r',  ACS1_A1, 19,         CAL16(1.0, 0.0),		'u'},
  {"i_gybox",      'r',  ACS1_A1, 21,         CAL16(1.0, 0.0), 		'u'},
  {"i_aux",        'r',  ACS1_A1, 23,         CAL16(1.0, 0.0),		'u'},
  {"v_sol1",       'r',  ACS1_A1, 25,         CAL16(1.0, 0.0), 		'u'},
  {"v_sol2",       'r',  ACS1_A1, 27,         CAL16(1.0, 0.0), 		'u'},
  {"v_sol3",       'r',  ACS1_A1, 29,         CAL16(1.0, 0.0), 		'u'},
  {"v_sol4",       'r',  ACS1_A1, 31,         CAL16(1.0, 0.0), 		'u'},
  {"v_sol5",       'r',  ACS1_A1, 33,         CAL16(1.0, 0.0), 		'u'},
  {"v_sol6",       'r',  ACS1_A1, 35,         CAL16(1.0, 0.0), 		'u'},
  {"v_batt",       'r',  ACS1_A1, 37,         CAL16(1.0, 0.0), 		'u'},
  {"i_sol",        'r',  ACS1_A1, 39,         CAL16(1.0, 0.0), 		'u'},
  {"i_batt",       'r',  ACS1_A1, 41,         CAL16(1.0, 0.0), 		'u'},

/* ACS1 Temperature card */
  {"t_gybox",      'r',  ACS1_T1, 1,          CAL16T(1.0, 0.0),         'u'},
  {"t_npv",        'r',  ACS1_T1, 9,          CAL16T(1.0, 0.0),         'u'},
  {"t_charger",    'r',  ACS1_T1, 11,         CAL16T(1.0, 0.0),         'u'},
  //TODO why two of these? should other be deleted or changed?
  //{"t_charger",    'r',  ACS1_T1, 11,         CAL16T(1.0, 0.0),         'u'},
  {"t_bat1",       'r',  ACS1_T1, 13,         CAL16T(1.0, 0.0),         'u'},
  {"t_bat2",       'r',  ACS1_T1, 15,         CAL16T(1.0, 0.0),         'u'},
  {"t_bat3",       'r',  ACS1_T1, 17,         CAL16T(1.0, 0.0),         'u'},
  {"t_bat4",       'r',  ACS1_T1, 19,         CAL16T(1.0, 0.0),         'u'},
  {"t_array",      'r',  ACS1_T1, 21,         CAL16T(1.0, 0.0),         'u'},

#endif

/* TODO These TMP channels need to be added to correct new nodes, or deleted */
  {"t_el_mc",      'r',  TMP1,  3,              I2T_M,           I2T_B, 'u'},
  {"t_el_mot",     'r',  TMP1,  5,              I2T_M,           I2T_B, 'u'},
  {"roll_clin_pyr",'r',  TMP1, 33,     -4.0/5333.3333,        4.*6.144, 'u'},
  {"pch_clin_pyr", 'r',  TMP1, 35,      4.0/5333.3333,       -4.*6.144, 'u'},
  {"t_clin_pyr",   'r',  TMP1, 23,           -0.01875,           614.4, 'u'},
  {"t_clin_sip",   'r',  TMP1, 31,           -0.01875,           614.4, 'u'},
  {"t_clin_if",    'r',  TMP1, 41,           -0.01875,           614.4, 'u'},
  {"apcu_reg",     'w',  TMP1,  4,             0.0382,           27.25, 'u'},
  {"dpcu_reg",     'w',  TMP1,  5,             0.0382,           27.25, 'u'},
  {"actbus_reset", 'w',  TMP1,  6,                1.0,             0.0, 'u'},

  {"t_reac",       'r',  TMP2, 29,              I2T_M,           I2T_B, 'u'},
  {"t_reac_mc",    'r',  TMP2, 31,              I2T_M,           I2T_B, 'u'},
  {"t_piv_mc",     'r',  TMP2, 45,              I2T_M,           I2T_B, 'u'},

  {"g_p_el",       'w',  TMP2,  2,                1.0,             0.0, 'u'},
  {"g_i_el",       'w',  TMP2,  3,                1.0,             0.0, 'u'},
  {"g_p_az",       'w',  TMP2,  7,                1.0,             0.0, 'u'},
  {"g_i_az",       'w',  TMP2,  8,                1.0,             0.0, 'u'},
  {"g_p_pivot",    'w',  TMP2, 15,                1.0,             0.0, 'u'},
  {"set_reac",     'w',  TMP2, 16,    7.9498291016e-5,          -2.605, 'u'},

  /* ACS2 0-1 is wide fast */
  {"t_ss_back_mid",'r',  TMP3,  3,              I2T_M,           I2T_B, 'u'},
  {"pch_clin_piv", 'r',  TMP3,  7,      4.0/5333.3333,       -4.*6.144, 'u'},
  {"roll_clin_piv",'r',  TMP3,  9,      4.0/5333.3333,       -4.*6.144, 'u'},
  {"t_clin_piv",   'r',  TMP3, 11,            0.01875,          -614.4, 'u'},
  {"i_sun",        'r',  TMP3, 13,           0.000625,          -20.48, 'u'},

  /* AD590 calibrations per Marco 2006-11 */
  {"t_if_top_frnt",'r',  TMP3, 17,              I2T_M,  I2T_B +
                                                 AD590_CALIB_INFRAME_1, 'u'},
  {"t_if_top_back",'r',  TMP3, 19,              I2T_M,  I2T_B +
                                                 AD590_CALIB_INFRAME_2, 'u'},
  {"t_if_bot_frnt",'r',  TMP3, 21,              I2T_M,  I2T_B + 
                                                 AD590_CALIB_INFRAME_3, 'u'},
  {"t_if_bot_back",'r',  TMP3, 23,              I2T_M,  I2T_B + 
                                                 AD590_CALIB_INFRAME_4, 'u'},

  {"t_lock_motor", 'r',  TMP3, 25,              I2T_M,           I2T_B, 'u'},
  {"t_acs",        'r',  TMP3, 29,              I2T_M,           I2T_B, 'u'},
  {"t_chin_mid",   'r',  TMP3, 31,              I2T_M,           I2T_B, 'u'},
  {"sensor_reset", 'w',  TMP3,  1,                1.0,             0.0, 'u'},

  {"v_batt_acs",   'r',  TMP4,  1,    -0.000525352612,      34.4796641, 'u'},
  {"v_batt_das",   'r',  TMP4,  3,  -0.00052776250894,  34.62735213282, 'u'},
  {"t_apcu",       'r',  TMP4,  5,              I2T_M,           I2T_B, 'u'},
  {"t_dpcu",       'r',  TMP4,  7,              I2T_M,           I2T_B, 'u'},
  {"t_sol_port",   'r',  TMP4,  9,              I2T_M,           I2T_B, 'u'},
  {"t_sol_stbd",   'r',  TMP4, 11,              I2T_M,           I2T_B, 'u'},
  {"t_batt_acs",   'r',  TMP4, 13,              I2T_M,           I2T_B, 'u'},
  {"t_batt_das",   'r',  TMP4, 15,              I2T_M,           I2T_B, 'u'},
  {"i_gond_acs",   'r',  TMP4, 17,          -1.875E-3,           61.44, 'u'},
  {"i_gond_das",   'r',  TMP4, 19,          -1.875E-3,           61.44, 'u'},
  {"ifpm_bits",    'w',  TMP4,  1,                1.0,             0.0, 'u'},
  {"balpump_lev",  'w',  TMP4,  3,    -0.048851978505,           100.0, 'u'},
  {"sprpump_lev",  'w',  TMP4,  4,    -0.048851978505,           100.0, 'u'},
  {"inpump_lev",   'w',  TMP4,  5,    -0.048851978505,           100.0, 'u'},
  {"outpump_lev",  'w',  TMP4,  6,    -0.048851978505,           100.0, 'u'},

  {"he4_lev",      'r',  TMP5,  1,  -2.87477e-09*65536,      12.3273561, 'u'},
  {"i_charcoal",   'r',  TMP5,  3,     -2.639826420E-6,     0.157988332, 'u'},
  {"i_coldplate",  'r',  TMP5,  5,      -2.32217573E-5,     1.390309833, 'u'},
  {"t_lhe",        'r',  TMP5, 17,             T_LHE_M,         T_LHE_B, 'u'},
  {"t_lhe_filt",   'r',  TMP5, 19, -2.856350e-09*65536,    1.231143e+01, 'u'},
  {"t_he4pot_d",   'r',  TMP5, 21, -2.869274e-09*65536,    1.236054e+01, 'u'},
  {"t_vcs_filt",   'r',  TMP5, 23, -2.871969e-09*65536,    1.236866e+01, 'u'},
  {"t_ln2",        'r',  TMP5, 25, -2.871958e-09*65536,    1.236808e+01, 'u'},
  {"t_ln2_filt",   'r',  TMP5, 27, -2.873729e-09*65536,    1.238262e+01, 'u'},
  {"t_charcoal",   'r',  TMP5, 29,        T_CHARCOAL_M,    T_CHARCOAL_B, 'u'},
  {"t_heatswitch", 'r',  TMP5, 31, -2.864185e-09*65536,    1.233900e+01, 'u'},
  {"t_jfet",       'r',  TMP5, 33,            T_JFET_M,        T_JFET_B, 'u'},
  {"t_vcs_fet",    'r',  TMP5, 35, -2.865493e-09*65536,    1.234227e+01, 'u'},
  {"t_opt_box_ext",'r',  TMP5, 37, -2.863415e-09*65536,    1.232882e+01, 'u'},
  {"cryoin",       'r',  TMP5, 60,                 1.0,             0.0, 'u'},
  {"cryoout2",     'w',  TMP5,  1,                 1.0,             0.0, 'u'},
  {"cryoout3",     'w',  TMP5,  2,             1.0,                 0.0, 'u'},
  {"bdapwm",       'w',  TMP5,  3,          100./2047.,              0., 'u'},
  {"hspwm",        'w',  TMP5,  4,          100./2047.,              0., 'u'},
  {"cryopwm",      'w',  TMP5,  5,          100./2047.,              0., 'u'},
  {"jfetpwm",      'w',  TMP5,  6,          100./2047.,              0., 'u'},
  {"cryoctrl",     'w',  TMP5, 31,                 1.0,              0., 'u'},
  {"set_bdaheat",  'w',  TMP5, 32,                 1.0,              0., 'u'},
  {"g_fl_bdaheat", 'w',  TMP5, 33,                 1.0,              0., 'u'},
  {"g_d_bdaheat",  'w',  TMP5, 34,                 1.0,              0., 'u'},
  {"g_i_bdaheat",  'w',  TMP5, 35,                 1.0,              0., 'u'},
  {"g_p_bdaheat",  'w',  TMP5, 36,                 1.0,              0., 'u'},

  /* BIAS 0-4 are wide fast */
  {"t_primary_2",  'r',  TMP6,  5,              I2T_M,  I2T_B +
                                                 AD590_CALIB_PRIMARY_2, 'u'},
  {"t_strut_bot",  'r',  TMP6,  7,              I2T_M,  I2T_B +
                                                   AD590_CALIB_STRUT_1, 'u'},
  {"t_primary_1",  'r',  TMP6,  9,              I2T_M,  I2T_B +
                                                 AD590_CALIB_PRIMARY_1, 'u'},
  {"t_secondary_1",'r',  TMP6, 11,              I2T_M,  I2T_B +
                                               AD590_CALIB_SECONDARY_1, 'u'},
  {"t_secondary_2",'r',  TMP6, 13,              I2T_M,  I2T_B +
                                               AD590_CALIB_SECONDARY_2, 'u'},
  {"t_strut_side", 'r',  TMP6, 15,              I2T_M,  I2T_B +
                                                   AD590_CALIB_STRUT_2, 'u'},
  {"t_push_plate", 'r',  TMP6, 17,              I2T_M,  I2T_B +
                                                AD590_CALIB_PUSH_PLATE, 'u'},
  {"t_act_motor",  'r',  TMP6, 19,              I2T_M,  I2T_B +
                                                 AD590_CALIB_ACT_MOTOR, 'u'},
  {"lvdt_10",      'r',  TMP6, 21,  LVDT10_ADC_TO_ENC,     LVDT10_ZERO, 'u'},
  {"lvdt_11",      'r',  TMP6, 33,  LVDT11_ADC_TO_ENC,     LVDT11_ZERO, 'u'},
  {"t_rec",        'r',  TMP6, 35,              I2T_M, I2T_B +
                                                       AD590_CALIB_REC, 'u'},
  {"lvdt_13",      'r',  TMP6, 37,  LVDT13_ADC_TO_ENC,     LVDT13_ZERO, 'u'},
  {"t_das",        'r',  TMP6, 47,              I2T_M,           I2T_B, 'u'},

  /* TODO these TMP channels used to be wide-fast */
#ifndef BOLOTEST
  {"raw_gy1",     'r',  TMP2, 26,     -AGY32_TO_DPS,
                                     AGY32_OFFSET * AGY32_TO_DPS + 0.1925, 'u'},
  {"raw_gy2",     'r',  TMP2, 22,      AGY32_TO_DPS,
                                     -AGY32_OFFSET * AGY32_TO_DPS - 0.138, 'u'},
  {"raw_gy3",     'r',  TMP2, 24,      AGY32_TO_DPS,
                                     -AGY32_OFFSET * AGY32_TO_DPS - 0.145, 'u'},
  {"raw_gy4",     'r',  TMP2,  6,      DGY32_TO_DPS,
                                     -DGY32_OFFSET * DGY32_TO_DPS + 0.004, 'u'},
  {"raw_gy5",     'r',  TMP2,  2,      DGY32_TO_DPS,
                                     -DGY32_OFFSET * DGY32_TO_DPS + 0.010, 'u'},
  {"raw_gy6",     'r',  TMP2, 36,     -DGY32_TO_DPS,
                                      DGY32_OFFSET * DGY32_TO_DPS - 0.005, 'u'},

  {"az",          'w', LOOP2, 51,          LI2DEG,                    0.0, 'u'},
  {"el",          'w', LOOP2, 53,          LI2DEG,                    0.0, 'u'},

#endif
  {"stage_x",      'w', LOOP5, 28,                1.0,             0.0, 'u'},
  {"stage_y",      'w', LOOP5, 34,                1.0,             0.0, 'u'},

  /* BIAS Stuff */
  {"b_amp2",      'r',  TMP6,  0,        B_AMP2_M,               B_AMP2_B, 'u'},
  {"b_amp1",      'r',  TMP6,  2,        B_AMP1_M,               B_AMP1_B, 'u'},
  {"b_amp3",      'r',  TMP6, 38,        B_AMP3_M,               B_AMP3_B, 'u'},

  /* TODO these TMP channels used to be narrow-fast */
#ifndef BOLOTEST
  /* read channels from ACS0 */
  {"pch_clin_sip",'r',  TMP1, 27,   4.0/5333.3333,              -25.91, 'u'},
  {"roll_clin_sip",'r', TMP1, 29,  -4.0/5333.3333,               24.778, 'u'},

  {"clin_elev",   'r',  TMP1, 37,      0.00546739,                -133.78, 'u'},
  {"xel_clin_if", 'r',  TMP1, 39,      0.00546739,             -25.*6.144, 'u'},

  {"mag_x",       'r',  TMP1, 45,          MAGX_M,                 MAGX_B, 'u'},
  {"mag_y",       'r',  TMP1, 47,          MAGY_M,                 MAGY_B, 'u'},
  {"mag_z",       'r',  TMP1, 43,          MAGZ_M,                 MAGZ_B, 'u'},
  {"isc_pulse",   'r',  TMP1, 53,             1.0,                    0.0, 'u'},
  {"osc_pulse",   'r',  TMP1, 54,             1.0,                    0.0, 'u'},
  {"piv_enc",     'r',  TMP1, 59,    360.0/8192.0,                    0.0, 'u'},

  /* send data to ACS0 */
  {"isc_trigger", 'w',  TMP1,  1,             1.0,                    0.0, 'u'},
  {"osc_trigger", 'w',  TMP1,  2,             1.0,                    0.0, 'u'},
  {"gy2_heat",    'w',  TMP1,  3,             1.0,                    0.0, 'u'},

  /* read channels from ACS1 */
  {"gyro2",       'r',  TMP2, 50,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro3",       'r',  TMP2, 56,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro1",       'r',  TMP2, 59,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},

  {"reac_enc",    'r',  TMP2, 60,    360.0/4000.0,                    0.0, 'u'},
  {"pwm_el",      'r',  TMP2, 51,             1.0,                -4000.0, 'u'},

  {"pwm_reac",    'r',  TMP2, 54,             1.0,                -4000.0, 'u'},
  {"rps_reac",    'r',  TMP2, 55,7.9498291016e-05,                 -2.605, 'u'},
  {"pwm_piv",     'r',  TMP2, 61,             1.0,                -4000.0, 'u'},

  /* send data to ACS1 */
  {"gy1_heat",    'w',  TMP2,  1,             1.0,                    0.0, 'u'},
  {"el_vreq",     'w',  TMP2,  4,  GY16_TO_DPS/6.,  -32768*GY16_TO_DPS/6., 'u'},
  {"az_vreq",     'w',  TMP2, 14,  GY16_TO_DPS/6.,  -32768*GY16_TO_DPS/6., 'u'},
  {"cos_el",      'w',  TMP2,  9,             1.0,                    0.0, 'u'},
  {"sin_el",      'w',  TMP2, 10,             1.0,                    0.0, 'u'},
  {"gy4_errs",    'r',  TMP2, 58,             1.0,                    0.0, 'u'},
  {"gy5_errs",    'r',  TMP2, 62,             1.0,                    0.0, 'u'},
/* channel 63 has been usurped. presumably this will have to change anyway
  {"gy6_errs",    'r',  TMP2, 63,             1.0,                    0.0, 'u'},
*/

  /* read from board ACS2 */
  {"enc_elev",    'r',  TMP3, 50, -360.0/65536.0,ENC_ELEV_OFFSET,ENC_ELEV_TYPE},

  {"mcp_frame",   'w', LOOP2, 34,             1.0,                    0.0, 'u'},
#endif

  /* Read from DAS3 -- cryo commanding */
  {"calstat",      'r',  TMP5, 61,                 1.0,             0.0, 'u'},


  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
  /* 25th channels of all the DAS cards, not currently used */
  /*
  {"n17c25",      'r', DAS1_A1, 36,                1.0,             0.0, 'U'},
  {"n18c25",      'r', DAS1_A2, 36,                1.0,             0.0, 'U'},
  {"n19c25",      'r', DAS1_A3, 36,                1.0,             0.0, 'U'},
  {"n21c25",      'r', DAS2_A1, 36,                1.0,             0.0, 'U'},
  {"n22c25",      'r', DAS2_A2, 36,                1.0,             0.0, 'U'},
  {"n23c25",      'r', DAS2_A3, 36,                1.0,             0.0, 'U'},
  {"n25c25",      'r', DAS3_A1, 36,                1.0,             0.0, 'U'},
  {"n26c25",      'r', DAS3_A2, 36,                1.0,             0.0, 'U'},
  {"n27c25",      'r', DAS3_A3, 36,                1.0,             0.0, 'U'},
  {"n29c25",      'r', DAS4_A1, 36,                1.0,             0.0, 'U'},
  {"n30c25",      'r', DAS4_A2, 36,                1.0,             0.0, 'U'},
  {"n31c25",      'r', DAS4_A3, 36,                1.0,             0.0, 'U'},
  */

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
  //interrupt counters, for debugging
  /*
  {"bias_10k",     'r',  BIAS_C,  0,                1.0,             0.0, 'u'},
  {"bias_100",     'r',  BIAS_C,  1,                1.0,             0.0, 'u'},
  {"bias_licnt",   'r',  BIAS_C,  2,                1.0,             0.0, 'u'},
  {"das1_10k",     'r',  DAS1_C,  0,                1.0,             0.0, 'u'},
  {"das1_100",     'r',  DAS1_C,  1,                1.0,             0.0, 'u'},
  {"das1_licnt",   'r',  DAS1_C,  2,                1.0,             0.0, 'u'},
  */

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,             1.0,                    0.0, 'u'},
  {"polarity",    'w', DECOM,  2,             1.0,                    0.0, 'u'},
  {"decom_unlock",'w', DECOM,  3,             1.0,                    0.0, 'u'},
  END_OF_CHANNELS
};
