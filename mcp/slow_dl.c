/* slow_dl.c: slow downlink packet specification
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "slow_dl.h"

/* slope and intercept only used for SLOWDL_FORCE_INT */
/* "src", type, numbits, value, wide, mindex, chnum, min, max*/
struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {
  /* Does the pv work? == 0.125/0.125 */
  {"sam_i_am",     SLOWDL_U_MASK,    1},
  /* Does the das work? == 3/3.125 bytes */
  {"t_dpm_5v",     SLOWDL_FORCE_INT, 8,-10, 50},  // 0.23 deg resolution
  {"i_dpm_5v",     SLOWDL_FORCE_INT, 8,  0, 5},   // 19mA resolution
  {"i_rec",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mA resolution
  /* Does the acs work? == 3/6.125 bytes */
  {"t_apm_5v",     SLOWDL_FORCE_INT, 8,-10, 50},  // 0.23 deg resolution
  {"t_el_bearing", SLOWDL_FORCE_INT, 8,-60, 60},  // 0.47 deg resolution
  {"i_gybox",      SLOWDL_FORCE_INT, 8,  0, 5},   // 19mA resolution
  /* Does the isc work? == 4.625/10.75 bytes */
  {"isc_framenum", SLOWDL_U_MASK,    8},          // 0--255
  {"isc_ra",       SLOWDL_FORCE_INT, 8,  0, 24},  // 5.6' resolution
  {"isc_dec",      SLOWDL_FORCE_INT, 8, 90, -90}, // 0.70 deg resolution
  {"t_isc_lens",   SLOWDL_FORCE_INT, 8,-10, 30},  // 0.15 deg resolution
  {"isc_nblobs",   SLOWDL_U_MASK,    5},          // 0--31
  /* Does the osc work? == 4.625/15.375 bytes */
  {"osc_framenum", SLOWDL_U_MASK,    8},          // 0--255
  {"osc_ra",       SLOWDL_FORCE_INT, 8,  0, 24},  // 5.6' resolution
  {"osc_dec",      SLOWDL_FORCE_INT, 8, 90, -90}, // 0.70 deg resolution
  {"t_osc_lens",   SLOWDL_FORCE_INT, 8,-10, 30},  // 0.15 deg resolution
  {"osc_nblobs",   SLOWDL_U_MASK,    5},          // 0--31
  /* Do the gyros work? == 4.125/19.5 bytes */
  {"gyro1",        SLOWDL_FORCE_INT, 8, -3, 3},   // 0.023 deg/s resolution
  {"raw_gy2",      SLOWDL_FORCE_INT, 8,-10, 10},  // 0.078 deg/s resolution
  {"raw_gy5",      SLOWDL_FORCE_INT, 8,-10, 10},  // 0.078 deg/s resolution
  {"gyro3",        SLOWDL_FORCE_INT, 8, -3, 3},   // 0.023 deg/s resolution
  {"use_analogue", SLOWDL_TAKE_BIT,  1}, 
  /* Does the bias work? == 3.5/23 bytes */
  {"biasin",       SLOWDL_U_MASK,    4},          // 0--15
  {"b_amp1",       SLOWDL_FORCE_INT, 8,  0, 127}, // 0.5 step resolution
  {"b_amp2",       SLOWDL_FORCE_INT, 8,  0, 127}, // 0.5 step resolution
  {"b_amp3",       SLOWDL_FORCE_INT, 8,  0, 127}, // 0.5 step resolution
  /* Do the bolometers work? == 12/35 bytes */
  {"n5c5",         SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n6c5",         SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n7c5",         SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n8c5",         SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n9c5",         SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n10c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n11c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n12c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n13c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n14c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n15c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  {"n16c5",        SLOWDL_FORCE_INT, 8,  0, 5},   // 19mV resolution
  /* Do the motors work? == 4/39 bytes */
  {"i_el",         SLOWDL_FORCE_INT, 8,-10, 10},  // 78mV resolution
  {"i_reac",       SLOWDL_FORCE_INT, 8,-10, 10},  // 78mV resolution
  {"i_piv",        SLOWDL_FORCE_INT, 8,-10, 10},  // 78mV resolution
  {"i_roll",       SLOWDL_FORCE_INT, 8,-10, 10},  // 78mV resolution
  /* Does the pointing solution work? == 5/44 bytes */
  {"az",           SLOWDL_FORCE_INT, 8,  0, 360}, // 1.4 deg resolution
  {"el",           SLOWDL_FORCE_INT, 8,-90, 90},  // 0.7 deg resolution
  {"lst",          SLOWDL_FORCE_INT, 8,  0, 24},  // 5.6' resolution
  {"sensor_vetos", SLOWDL_U_MASK,    8},          // 0--255
  {"sensor_reset", SLOWDL_U_MASK,    8},          // 0--255
  /* Does the cryostat work? == 6.375/50.625 bytes */
  {"t_m4",         SLOWDL_FORCE_INT, 8,  1 /* 4K */,    2 /* 1K */},
  {"t_horn_350",   SLOWDL_FORCE_INT, 8,  0 /* 175mK */, 1.25 /* 3K */},
  {"t_lhe",        SLOWDL_FORCE_INT, 8,  9.25 /* 5K */, 9.65 /* 3K */},
  {"t_ln_filt",    SLOWDL_FORCE_INT, 8,  6 /* 85K */, 6.17 /* 75K */},
  {"t_charcoal",   SLOWDL_FORCE_INT, 8,  6.62 /* 30K */, 9.65 /* 3K */},
  {"cryo_state",   SLOWDL_U_MASK,    8},    // 0-255
  {"cycle_state",  SLOWDL_U_MASK,    3},    // 0-255
  /* Does the power system work? == 6/56.625 bytes */
  {"v_batt_acs",   SLOWDL_FORCE_INT, 8,  19, 32},  // 51mV resolution
  {"v_batt_das",   SLOWDL_FORCE_INT, 8,  19, 32},  // 51mV resolution
  {"i_batt_acs",   SLOWDL_FORCE_INT, 8, -15, 20},  // 136mA resolution
  {"i_batt_das",   SLOWDL_FORCE_INT, 8, -15, 20},  // 136mA resolution
  {"i_gond_acs",   SLOWDL_FORCE_INT, 8, -5, 20},   // 98mA resolution
  {"i_gond_das",   SLOWDL_FORCE_INT, 8, -5, 20},   // 98mA resolution
  /* ADC card statuses == 9/65.625 bytes */
  {"status00",     SLOWDL_U_MASK,    4},
  {"status01",     SLOWDL_U_MASK,    4},
  {"status02",     SLOWDL_U_MASK,    4},
  {"status03",     SLOWDL_U_MASK,    4},
  {"status04",     SLOWDL_U_MASK,    4},
  {"status05",     SLOWDL_U_MASK,    4},
  {"status06",     SLOWDL_U_MASK,    4},
  {"status07",     SLOWDL_U_MASK,    4},
  {"status08",     SLOWDL_U_MASK,    4},
  {"status09",     SLOWDL_U_MASK,    4},
  {"status10",     SLOWDL_U_MASK,    4},
  {"status11",     SLOWDL_U_MASK,    4},
  {"status12",     SLOWDL_U_MASK,    4},
  {"status13",     SLOWDL_U_MASK,    4},
  {"status14",     SLOWDL_U_MASK,    4},
  {"status15",     SLOWDL_U_MASK,    4},
  {"status16",     SLOWDL_U_MASK,    4},
  {"status17",     SLOWDL_U_MASK,    4},
};
