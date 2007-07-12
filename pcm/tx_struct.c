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

#include "channels.h"
#include "bbc_pci.h"

/* card name to (node number, bus number) mapping */
#define ACS1   1, 0
#define LOOP1  2, 0

/* read and write channel 56 on all boards reserved for ADC Sync */
struct ChannelStruct WideSlowChannels[] = {
  {"sc1_frame",    'w', LOOP1, 13,                1.0,             0.0, 'u'},
  {"sc1_time",     'w', LOOP1, 15,                1.0,             0.0, 'u'},
  {"sc1_usec",     'w', LOOP1, 17,                1.0,             0.0, 'u'},
  //TODO add a derived field that adds the two times together
 END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  {"status00",     'r',  ACS1, 60,                1.0,             0.0, 'u'},
  {"sync00",	   'w',  ACS1, 56,                1.0,             0.0, 'u'},
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
  //TODO may want to rescale sigma, will have to for temperature
  {"sc1_mapsigma", 'w', LOOP1, 20,                1.0,             0.0, 'u'},
  {"sc1_ccd_t",    'w', LOOP1, 21,                1.0,             0.0, 'u'},
  {"sc1_numblobs", 'w', LOOP1, 22,                1.0,             0.0, 'u'},
  //TODO add fields for blob x, y, flux, snr for blobs
  END_OF_CHANNELS
};

struct ChannelStruct WideFastChannels[] = {
  {"t_gybox1",    'r',  ACS1, 14,          TGYBOX_M,             TGYBOX_B, 'U'},
  {"raw_gy1",     'r',  ACS1, 26,     -AGY32_TO_DPS,
                                     AGY32_OFFSET * AGY32_TO_DPS + 0.1925, 'U'},
  {"raw_gy2",     'r',  ACS1, 22,      AGY32_TO_DPS,
                                     -AGY32_OFFSET * AGY32_TO_DPS - 0.138, 'U'},
  {"raw_gy3",     'r',  ACS1, 24,      AGY32_TO_DPS,
                                     -AGY32_OFFSET * AGY32_TO_DPS - 0.145, 'U'},
  {"raw_gy4",     'r',  ACS1,  6,      DGY32_TO_DPS,
                                     -DGY32_OFFSET * DGY32_TO_DPS + 0.004, 'U'},
  {"raw_gy5",     'r',  ACS1,  2,      DGY32_TO_DPS,
                                     -DGY32_OFFSET * DGY32_TO_DPS + 0.010, 'U'},
  {"raw_gy6",     'r',  ACS1, 36,     -DGY32_TO_DPS,
                                      DGY32_OFFSET * DGY32_TO_DPS - 0.005, 'U'},
  {"enc_table",   'r',  ACS1, 56,     360.0/144000.0,                 0.0, 'U'},

  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
  /* read channels from ACS1 */
  {"gyro1",       'r',  ACS1, 50,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro2",       'r',  ACS1, 51,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro3",       'r',  ACS1, 52,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro4",       'r',  ACS1, 53,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro5",       'r',  ACS1, 54,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"gyro6",       'r',  ACS1, 55,  GY16_TO_DPS, -GY16_OFFSET*GY16_TO_DPS, 'u'},
  {"enc_table_z", 'r',  ACS1, 58,  1.0,                              0.0, 'u'},
  {"gy4_bad",     'r',  ACS1, 61,  1.0,                              0.0, 'u'},
  {"gy5_bad",     'r',  ACS1, 62,  1.0,                              0.0, 'u'},
  {"gy6_bad",     'r',  ACS1, 63,  1.0,                              0.0, 'u'},
  {"dps_table",   'w', LOOP1,  0,  70.0/32767.0,                     0.0, 's'},

  END_OF_CHANNELS
};

struct ChannelStruct DecomChannels[] = {
  END_OF_CHANNELS
};
