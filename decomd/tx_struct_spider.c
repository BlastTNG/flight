/* tx_struct.c: contains the channel specificiation lists
 *
 * This software is copyright (C) 2002-2007 University of Toronto
 *
 * This file is part of the Spider flight code licensed under the GNU
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
#include "camstruct.h"  //for defs
#include "motordefs.h"  // for scaling factors

/* card name to (node number, bus number) mapping */
#define ACS1   1, 0
#define TEST1  2, 0
#define TEST2  3, 0
#define TEST3  4, 0
#define DECOM 22, 0
#define LOOP1 23, 0
#define LOOP2 24, 0


/* read and write channel 56 on all boards reserved for ADC Sync */
struct ChannelStruct WideSlowChannels[] = {
  {"sc1_frame",    'w', LOOP1, 13,                1.0,             0.0, 'U'},
  {"sc1_sec",      'w', LOOP1, 15,                1.0,             0.0, 'U'},
  {"sc1_usec",     'w', LOOP1, 17,                1.0,             0.0, 'U'},
  //derived channel sc1_time adds these together

 END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  //sync and status channels (1 per card), sync must be on 56
  {"status00",     'r',  ACS1, 60,                1.0,             0.0, 'u'},
  {"sync00",	   'w',  ACS1, 56,                1.0,             0.0, 'u'},
  {"status01",     'r', TEST1, 60,                1.0,             0.0, 'u'},
  {"sync01",	   'w', TEST1, 56,                1.0,             0.0, 'u'},
  {"status02",     'r', TEST2, 60,                1.0,             0.0, 'u'},
  {"sync02",	   'w', TEST2, 56,                1.0,             0.0, 'u'},
  {"status03",     'r', TEST3, 60,                1.0,             0.0, 'u'},
  {"sync03",	   'w', TEST3, 56,                1.0,             0.0, 'u'},

  //padding to help the 2nd bus work without many slow channels
  {"pad01",        'w', TEST1,  0,                1.0,             0.0, 'u'},
  {"pad02",        'w', TEST1,  1,                1.0,             0.0, 'u'},
  {"pad03",        'w', TEST1,  2,                1.0,             0.0, 'u'},
  {"pad04",        'w', TEST1,  3,                1.0,             0.0, 'u'},
  {"pad05",        'w', TEST1,  4,                1.0,             0.0, 'u'},
 
  //ACS1 analog channels with upper word stolen (or not needed)
  //making these wide requires a change to DSP code!
  {"acs1_a00",     'r',  ACS1,  0,                1.0,             0.0, 'u'},
  //analog channels 1, 2, 3 are gyros -- wide-fast
  {"acs1_a06",     'r',  ACS1, 12,                1.0,             0.0, 'u'},
  {"acs1_a07",     'r',  ACS1, 14,                1.0,             0.0, 'u'},
  {"acs1_a08",     'r',  ACS1, 16,                1.0,             0.0, 'u'},
  {"acs1_a09",     'r',  ACS1, 18,                1.0,             0.0, 'u'},
  {"acs1_a10",     'r',  ACS1, 20,                1.0,             0.0, 'u'},
  {"acs1_a11",     'r',  ACS1, 22,                1.0,             0.0, 'u'},
#if 0
  //TODO these have been made long for testing, change back (DSP too)
  {"acs1_a12",     'r',  ACS1, 24,                1.0,             0.0, 'u'},
  {"acs1_a13",     'r',  ACS1, 26,                1.0,             0.0, 'u'},
  {"acs1_a14",     'r',  ACS1, 28,                1.0,             0.0, 'u'},
  {"acs1_a15",     'r',  ACS1, 30,                1.0,             0.0, 'u'},
  {"acs1_a16",     'r',  ACS1, 32,                1.0,             0.0, 'u'},
  //analog channels 17, 18 are gyros -- wide-fast
  {"acs1_a19",     'r',  ACS1, 38,                1.0,             0.0, 'u'},
  //analog channel 20 is gyro -- wide fast
  {"acs1_a21",     'r',  ACS1, 42,                1.0,             0.0, 'u'},
  {"acs1_a22",     'r',  ACS1, 44,                1.0,             0.0, 'u'},
  {"acs1_a23",     'r',  ACS1, 46,                1.0,             0.0, 'u'},
  {"acs1_a24",     'r',  ACS1, 48,                1.0,             0.0, 'u'},
#endif

  {"gy1_bad",      'r',  ACS1,  9,                1.0,             0.0, 'u'},
  {"gy2_bad",      'r',  ACS1, 11,                1.0,             0.0, 'u'},
  {"gy3_bad",      'r',  ACS1, 13,                1.0,             0.0, 'u'},
  {"gy4_bad",      'r',  ACS1, 15,                1.0,             0.0, 'u'},
  {"gy5_bad",      'r',  ACS1, 17,                1.0,             0.0, 'u'},
  {"gy6_bad",      'r',  ACS1, 19,                1.0,             0.0, 'u'},

  {"g_p_table",    'w', LOOP1,  1,         1.0/1000.0,             0.0, 'u'},
  {"g_i_table",    'w', LOOP1,  2,        1.0/10000.0,             0.0, 'u'},
  {"g_d_table",    'w', LOOP1,  3,          1.0/100.0,             0.0, 'u'},
  {"sc_force",     'w', LOOP1,  4,                1.0,             0.0, 'u'},
  {"sc_exp_int",   'w', LOOP1,  5,                1.0,             0.0, 'u'},
  {"sc_exp_time",  'w', LOOP1,  6,                1.0,             0.0, 'u'},
  {"sc_foc_res",   'w', LOOP1,  7,                1.0,             0.0, 'u'},
  {"sc_move_tol",  'w', LOOP1,  8,                1.0,             0.0, 'u'},
  {"sc_maxblob",   'w', LOOP1,  9,                1.0,             0.0, 'u'},
  {"sc_grid",      'w', LOOP1, 10,                1.0,             0.0, 'u'},
  {"sc_thresh",    'w', LOOP1, 11,         1.0/1000.0,             0.0, 'u'},
  {"sc_mdist",     'w', LOOP1, 12,                1.0,             0.0, 'u'},
  //LOOP1 13-18 are wide
  {"sc1_mapmean",  'w', LOOP1, 19,                1.0,             0.0, 'u'},
  {"sc1_mapsigma", 'w', LOOP1, 20,           1.0/10.0,             0.0, 'u'},
  {"sc1_ccd_t",    'w', LOOP1, 21,          1.0/100.0,             0.0, 's'},
  {"sc1_numblobs", 'w', LOOP1, 22,                1.0,             0.0, 'u'},
  {"sc1_blob00_x", 'w', LOOP1, 23, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob00_y", 'w', LOOP1, 24, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob00_f", 'w', LOOP1, 25,                1.0,             0.0, 'u'},
  {"sc1_blob00_s", 'w', LOOP1, 26,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob01_x", 'w', LOOP1, 27, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob01_y", 'w', LOOP1, 28, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob01_f", 'w', LOOP1, 29,                1.0,             0.0, 'u'},
  {"sc1_blob01_s", 'w', LOOP1, 30,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob02_x", 'w', LOOP1, 31, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob02_y", 'w', LOOP1, 32, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob02_f", 'w', LOOP1, 33,                1.0,             0.0, 'u'},
  {"sc1_blob02_s", 'w', LOOP1, 34,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob03_x", 'w', LOOP1, 35, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob03_y", 'w', LOOP1, 36, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob03_f", 'w', LOOP1, 37,                1.0,             0.0, 'u'},
  {"sc1_blob03_s", 'w', LOOP1, 38,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob04_x", 'w', LOOP1, 39, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob04_y", 'w', LOOP1, 40, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob04_f", 'w', LOOP1, 41,                1.0,             0.0, 'u'},
  {"sc1_blob04_s", 'w', LOOP1, 42,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob05_x", 'w', LOOP1, 43, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob05_y", 'w', LOOP1, 44, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob05_f", 'w', LOOP1, 45,                1.0,             0.0, 'u'},
  {"sc1_blob05_s", 'w', LOOP1, 46,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob06_x", 'w', LOOP1, 47, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob06_y", 'w', LOOP1, 48, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob06_f", 'w', LOOP1, 49,                1.0,             0.0, 'u'},
  {"sc1_blob06_s", 'w', LOOP1, 50,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob07_x", 'w', LOOP1, 51, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob07_y", 'w', LOOP1, 52, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob07_f", 'w', LOOP1, 53,                1.0,             0.0, 'u'},
  {"sc1_blob07_s", 'w', LOOP1, 54,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob08_x", 'w', LOOP1, 55, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob08_y", 'w', LOOP1, 56, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob08_f", 'w', LOOP1, 57,                1.0,             0.0, 'u'},
  {"sc1_blob08_s", 'w', LOOP1, 58,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob09_x", 'w', LOOP1, 59, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob09_y", 'w', LOOP1, 60, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob09_f", 'w', LOOP1, 61,                1.0,             0.0, 'u'},
  {"sc1_blob09_s", 'w', LOOP1, 62,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob10_x", 'w', LOOP1, 63, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob10_y", 'w', LOOP2,  0, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob10_f", 'w', LOOP2,  1,                1.0,             0.0, 'u'},
  {"sc1_blob10_s", 'w', LOOP2,  2,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob11_x", 'w', LOOP2,  3, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob11_y", 'w', LOOP2,  4, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob11_f", 'w', LOOP2,  5,                1.0,             0.0, 'u'},
  {"sc1_blob11_s", 'w', LOOP2,  6,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob12_x", 'w', LOOP2,  7, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob12_y", 'w', LOOP2,  8, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob12_f", 'w', LOOP2,  9,                1.0,             0.0, 'u'},
  {"sc1_blob12_s", 'w', LOOP2, 10,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob13_x", 'w', LOOP2, 11, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob13_y", 'w', LOOP2, 12, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob13_f", 'w', LOOP2, 13,                1.0,             0.0, 'u'},
  {"sc1_blob13_s", 'w', LOOP2, 14,          1.0/100.0,             0.0, 'u'},
  {"sc1_blob14_x", 'w', LOOP2, 15, CAM_WIDTH/SHRT_MAX,             0.0, 'u'},
  {"sc1_blob14_y", 'w', LOOP2, 16, CAM_WIDTH/SHRT_MAX,             0.0, 'u'}, 
  {"sc1_blob14_f", 'w', LOOP2, 17,                1.0,             0.0, 'u'},
  {"sc1_blob14_s", 'w', LOOP2, 18,          1.0/100.0,             0.0, 'u'},
  {"g_table_move", 'w', LOOP2, 19,     100.0/SHRT_MAX,             0.0, 'u'},
  {"table_move",   'w', LOOP2, 20,           1.0/10.0,             0.0, 's'},
  {"dps_gond_req", 'w', LOOP2, 21,       60.0/32767.0,             0.0, 's'},
  {"gond_az",      'w', LOOP2, 22,      360.0/65535.0,             0.0, 'u'},
  {"gond_theta",   'w', LOOP2, 23,      360.0/65535.0,             0.0, 'u'},
  {"dpsps_gond_req",'w',LOOP2, 24,        2.0/32767.0,             0.0, 's'},
  {"is_gond_accel",'w', LOOP2, 25,                1.0,             0.0, 'u'},
  {"dps_piv_req",  'w', LOOP2, 26,       60.0/32767.0,             0.0, 's'},
  {"i_reac_req",   'w', LOOP2, 27,       20.0/32767.0,             0.0, 's'},
  {"dps_piv",      'w', LOOP2, 28,       70.0/32767.0,             0.0, 's'},
  {"dps_rw_filt",  'w', LOOP2, 29,     3000.0/32767.0,             0.0, 's'},
  // Next batch is all gains for spin and scan mode
  {"spin_gain_r1", 'w', LOOP2, 30,    SPR1_LIM/32767.0,             0.0, 's'},
  {"spin_gain_r2", 'w', LOOP2, 31,    SPR2_LIM/32767.0,             0.0, 's'},
  {"spin_gain_p1", 'w', LOOP2, 32,    SPP1_LIM/32767.0,             0.0, 's'},
  {"spin_gain_p2", 'w', LOOP2, 33,    SPP2_LIM/32767.0,             0.0, 's'},
  {"scan_gain_r1", 'w', LOOP2, 34,    SCR1_LIM/32767.0,             0.0, 's'},
  {"scan_gain_r2", 'w', LOOP2, 35,    SCR2_LIM/32767.0,             0.0, 's'},
  {"scan_gain_p1", 'w', LOOP2, 36,    SCP1_LIM/32767.0,             0.0, 's'},
  {"scan_gain_p2", 'w', LOOP2, 37,    SCP2_LIM/32767.0,             0.0, 's'},
  // Parameters from the command structure for scan and point mode
  {"scan_az_centre",'w',LOOP2, 38,    360/65535.0,                  0.0, 'u'},
  {"scan_period",   'w',LOOP2, 39,    60/65535.0,                   0.0, 'u'},
  {"scan_az_width",'w', LOOP2, 40,    120/65535.0,                  0.0, 'u'},
  {"scan_az_wcrit",'w', LOOP2, 41,    120/65535.0,                  0.0, 'u'},
  {"point_az"     ,'w', LOOP2, 42,    360/65535.0,                  0.0, 'u'},
  {"point_tol"    ,'w', LOOP2, 43,    360/65535.0,                  0.0, 'u'},
  // LOOP2, 44-46 defined in the fast channels. 
  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
  {"raw_gy1",      'r',  ACS1,  4,      DGY32_TO_DPS,
                                   -DGY32_OFFSET * DGY32_TO_DPS + 0.010, 'U'},
  {"raw_gy2",      'r',  ACS1, 34,      DGY32_TO_DPS,
                                   -DGY32_OFFSET * DGY32_TO_DPS + 0.004, 'U'},
  {"raw_gy3",      'r',  ACS1, 40,     -DGY32_TO_DPS,
                                    DGY32_OFFSET * DGY32_TO_DPS - 0.005, 'U'},
  {"raw_gy4",      'r',  ACS1,  2,      DGY32_TO_DPS,
                                   -DGY32_OFFSET * DGY32_TO_DPS + 0.010, 'U'},
  {"raw_gy5",      'r',  ACS1,  6,      DGY32_TO_DPS,
                                   -DGY32_OFFSET * DGY32_TO_DPS + 0.004, 'U'},
  {"raw_gy6",      'r',  ACS1, 36,     -DGY32_TO_DPS,
                                    DGY32_OFFSET * DGY32_TO_DPS - 0.005, 'U'},
  {"enc_table",    'r',  ACS1, 56,     360.0/144000.0,              0.0, 'U'},

  //TODO these don't belong here (they've been moved for testing)
  //  {"acs1_a10",     'r',  ACS1, 20,                1.0,             0.0, 'U'},
  //  {"acs1_a11",     'r',  ACS1, 22,                1.0,             0.0, 'U'},
  {"acs1_a12",     'r',  ACS1, 24,                1.0,             0.0, 'U'},
  {"acs1_a13",     'r',  ACS1, 26,                1.0,             0.0, 'U'},
  {"acs1_a14",     'r',  ACS1, 28,                1.0,             0.0, 'U'},
  {"acs1_a15",     'r',  ACS1, 30,                1.0,             0.0, 'U'},
  {"acs1_a16",     'r',  ACS1, 32,                1.0,             0.0, 'U'},
  //analog channels 17, 18 are gyros -- wide-fast
  {"acs1_a19",     'r',  ACS1, 38,                1.0,             0.0, 'U'},
  //analog channel 20 is gyro -- wide fast
  {"acs1_a21",     'r',  ACS1, 42,                1.0,             0.0, 'U'},
  {"acs1_a22",     'r',  ACS1, 44,                1.0,             0.0, 'U'},
  {"acs1_a23",     'r',  ACS1, 46,                1.0,             0.0, 'U'},
  {"acs1_a24",     'r',  ACS1, 48,                1.0,             0.0, 'U'},

  {"test1_a00",    'r', TEST1,  0,                1.0,             0.0, 'U'},
  {"test1_a01",    'r', TEST1,  2,                1.0,             0.0, 'U'},
  {"test1_a02",    'r', TEST1,  4,                1.0,             0.0, 'U'},
  {"test1_a03",    'r', TEST1,  6,                1.0,             0.0, 'U'},
  {"test1_a04",    'r', TEST1,  8,                1.0,             0.0, 'U'},
  {"test1_a05",    'r', TEST1, 10,                1.0,             0.0, 'U'},
  {"test1_a06",    'r', TEST1, 12,                1.0,             0.0, 'U'},
  {"test1_a07",    'r', TEST1, 14,                1.0,             0.0, 'U'},
  {"test1_a08",    'r', TEST1, 16,                1.0,             0.0, 'U'},
  {"test1_a09",    'r', TEST1, 18,                1.0,             0.0, 'U'},
  {"test1_a10",    'r', TEST1, 20,                1.0,             0.0, 'U'},
  {"test1_a11",    'r', TEST1, 22,                1.0,             0.0, 'U'},
  {"test1_a12",    'r', TEST1, 24,                1.0,             0.0, 'U'},
  {"test1_a13",    'r', TEST1, 26,                1.0,             0.0, 'U'},
  {"test1_a14",    'r', TEST1, 28,                1.0,             0.0, 'U'},
  {"test1_a15",    'r', TEST1, 30,                1.0,             0.0, 'U'},
  {"test1_a16",    'r', TEST1, 32,                1.0,             0.0, 'U'},
  {"test1_a17",    'r', TEST1, 34,                1.0,             0.0, 'U'},
  {"test1_a18",    'r', TEST1, 36,                1.0,             0.0, 'U'},
  {"test1_a19",    'r', TEST1, 38,                1.0,             0.0, 'U'},
  {"test1_a20",    'r', TEST1, 40,                1.0,             0.0, 'U'},
  {"test1_a21",    'r', TEST1, 42,                1.0,             0.0, 'U'},
  {"test1_a22",    'r', TEST1, 44,                1.0,             0.0, 'U'},
  {"test1_a23",    'r', TEST1, 46,                1.0,             0.0, 'U'},
  {"test1_a24",    'r', TEST1, 48,                1.0,             0.0, 'U'},
  //
  //TODO for testing only, unfiltered, stage1 ch24
  {"test1_uf24",   'r', TEST1, 54,                1.0,             0.0, 'U'},
  {"test1_s124",   'r', TEST1, 56,                1.0,             0.0, 'U'},

  {"test2_a00",    'r', TEST2,  0,                1.0,             0.0, 'U'},
  {"test2_a01",    'r', TEST2,  2,                1.0,             0.0, 'U'},
  {"test2_a02",    'r', TEST2,  4,                1.0,             0.0, 'U'},
  {"test2_a03",    'r', TEST2,  6,                1.0,             0.0, 'U'},
  {"test2_a04",    'r', TEST2,  8,                1.0,             0.0, 'U'},
  {"test2_a05",    'r', TEST2, 10,                1.0,             0.0, 'U'},
  {"test2_a06",    'r', TEST2, 12,                1.0,             0.0, 'U'},
  {"test2_a07",    'r', TEST2, 14,                1.0,             0.0, 'U'},
  {"test2_a08",    'r', TEST2, 16,                1.0,             0.0, 'U'},
  {"test2_a09",    'r', TEST2, 18,                1.0,             0.0, 'U'},
  {"test2_a10",    'r', TEST2, 20,                1.0,             0.0, 'U'},
  {"test2_a11",    'r', TEST2, 22,                1.0,             0.0, 'U'},
  {"test2_a12",    'r', TEST2, 24,                1.0,             0.0, 'U'},
  {"test2_a13",    'r', TEST2, 26,                1.0,             0.0, 'U'},
  {"test2_a14",    'r', TEST2, 28,                1.0,             0.0, 'U'},
  {"test2_a15",    'r', TEST2, 30,                1.0,             0.0, 'U'},
  {"test2_a16",    'r', TEST2, 32,                1.0,             0.0, 'U'},
  {"test2_a17",    'r', TEST2, 34,                1.0,             0.0, 'U'},
  {"test2_a18",    'r', TEST2, 36,                1.0,             0.0, 'U'},
  {"test2_a19",    'r', TEST2, 38,                1.0,             0.0, 'U'},
  {"test2_a20",    'r', TEST2, 40,                1.0,             0.0, 'U'},
  {"test2_a21",    'r', TEST2, 42,                1.0,             0.0, 'U'},
  {"test2_a22",    'r', TEST2, 44,                1.0,             0.0, 'U'},
  {"test2_a23",    'r', TEST2, 46,                1.0,             0.0, 'U'},
  {"test2_a24",    'r', TEST2, 48,                1.0,             0.0, 'U'},
  {"test3_a00",    'r', TEST3,  0,                1.0,             0.0, 'U'},
  {"test3_a01",    'r', TEST3,  2,                1.0,             0.0, 'U'},
  {"test3_a02",    'r', TEST3,  4,                1.0,             0.0, 'U'},
  {"test3_a03",    'r', TEST3,  6,                1.0,             0.0, 'U'},
  {"test3_a04",    'r', TEST3,  8,                1.0,             0.0, 'U'},
  {"test3_a05",    'r', TEST3, 10,                1.0,             0.0, 'U'},
  {"test3_a06",    'r', TEST3, 12,                1.0,             0.0, 'U'},
  {"test3_a07",    'r', TEST3, 14,                1.0,             0.0, 'U'},
  {"test3_a08",    'r', TEST3, 16,                1.0,             0.0, 'U'},
  {"test3_a09",    'r', TEST3, 18,                1.0,             0.0, 'U'},
  {"test3_a10",    'r', TEST3, 20,                1.0,             0.0, 'U'},
  {"test3_a11",    'r', TEST3, 22,                1.0,             0.0, 'U'},
  {"test3_a12",    'r', TEST3, 24,                1.0,             0.0, 'U'},
  {"test3_a13",    'r', TEST3, 26,                1.0,             0.0, 'U'},
  {"test3_a14",    'r', TEST3, 28,                1.0,             0.0, 'U'},
  {"test3_a15",    'r', TEST3, 30,                1.0,             0.0, 'U'},
  {"test3_a16",    'r', TEST3, 32,                1.0,             0.0, 'U'},
  {"test3_a17",    'r', TEST3, 34,                1.0,             0.0, 'U'},
  {"test3_a18",    'r', TEST3, 36,                1.0,             0.0, 'U'},
  {"test3_a19",    'r', TEST3, 38,                1.0,             0.0, 'U'},
  {"test3_a20",    'r', TEST3, 40,                1.0,             0.0, 'U'},
  {"test3_a21",    'r', TEST3, 42,                1.0,             0.0, 'U'},
  {"test3_a22",    'r', TEST3, 44,                1.0,             0.0, 'U'},
  {"test3_a23",    'r', TEST3, 46,                1.0,             0.0, 'U'},
  {"test3_a24",    'r', TEST3, 48,                1.0,             0.0, 'U'},
  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
  /* read channels from ACS1 */
  {"gyro1",        'r',  ACS1, 50,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro2",        'r',  ACS1, 51,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro3",        'r',  ACS1, 52,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro4",        'r',  ACS1, 53,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro5",        'r',  ACS1, 54,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro6",        'r',  ACS1, 55,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"acs_toggle",   'r',  ACS1, 58,  1.0,                              0.0, 'u'},
  {"temporary",    'r',  ACS1, 59,  1.0,                              0.0, 'u'},
  {"sc1_trig_s",   'r',  ACS1, 61,  1.0,                              0.0, 'u'},
  {"sc1_trig_l",   'r',  ACS1, 62,  1.0,                              0.0, 'u'},
  {"sc2_trig_s",   'r',  ACS1, 63,  1.0,                              0.0, 'u'},
  {"sc2_trig_l",   'r',  ACS1,  1,  1.0,                              0.0, 'u'},
  {"dps_table",    'w', LOOP1,  0,  70.0/32767.0,                     0.0, 's'},

//  {"rwheel_vel",   'r',  ACS1,  8,  RWVEL_TO_DPS,            RWVEL_OFFSET, 'u'},
  {"rwheel_phase",   'r',  ACS1,  8,  RWCTS_TO_PHASE,          RWPHASE_OFFSET, 'u\
'},
  {"rwheel_cur",   'r',  ACS1, 10,  RWCUR_TO_DPS,            RWCUR_OFFSET, 'u'},
  // RW Controller outputs calculated on the DSP.
  {"rwheel_angle",      'r',  ACS1, 21,                1.0,             0.0, 'u'},
  {"rwheel_vel",        'r',  ACS1, 23,       RWCTS_TO_VEL,         0.0, 's'},

  {"test1_d1",	   'r', TEST1, 50,                1.0,             0.0, 'u'},
  {"test1_d2",	   'r', TEST1, 51,                1.0,             0.0, 'u'},
  {"test1_d3",	   'r', TEST1, 52,                1.0,             0.0, 'u'},
  {"test1_cnt",    'r', TEST1, 53,                1.0,             0.0, 'u'},
  {"test2_d1",	   'r', TEST2, 50,                1.0,             0.0, 'u'},
  {"test2_d2",	   'r', TEST2, 51,                1.0,             0.0, 'u'},
  {"test2_d3",	   'r', TEST2, 52,                1.0,             0.0, 'u'},
  {"test2_cnt",    'r', TEST2, 53,                1.0,             0.0, 'u'},
  {"test3_d1",	   'r', TEST3, 50,                1.0,             0.0, 'u'},
  {"test3_d2",	   'r', TEST3, 51,                1.0,             0.0, 'u'},
  {"test3_d3",	   'r', TEST3, 52,                1.0,             0.0, 'u'},
  {"test3_cnt",    'r', TEST3, 53,                1.0,             0.0, 'u'},
  {"dpsps_gond"   ,'w', LOOP2, 44,        100/32767.0,             0.0, 's'},
  {"dpsps_gond_rough",'w', LOOP2, 45,        100/32767.0,             0.0, 's'},
  {"dps_err"      ,'w', LOOP2, 46,        72.0/32767.0,             0.0, 's'},
  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  {"crc_ok",      'w', DECOM,  1,             1.0,                    0.0, 'u'},
  {"polarity",    'w', DECOM,  2,             1.0,                    0.0, 'u'},
  {"decom_unlock",'w', DECOM,  3,             1.0,                    0.0, 'u'},
  END_OF_CHANNELS
};
