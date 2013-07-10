/* MPC: MCE-PCM communicator
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <expcfg_list.h>

/* serialisation data */
const struct cfg_serial expcfg_list[] = {
  {"sa_ramp_bias", CFGSER_BIT, 0, 0},
  {"sq1_servo_bias_ramp", CFGSER_BIT, 0, 1},
  {"sq2_servo_bias_ramp", CFGSER_BIT, 0, 2},
  {"sq1_ramp_tes_bias", CFGSER_BIT, 0, 3},
  {"tuning_check_bias", CFGSER_BIT, 0, 4},
  {"tuning_do_plots", CFGSER_BIT, 0, 5},
  {"flux_jumping", CFGSER_BIT, 0, 6},                  /* total:  1 */

  {"columns_off", CFGSER_BITARR, 1, 1},                /* total:  2  07 */
  {"sa_offset_bias_ratio", CFGSER_FLOAT, 2, 1, -1, 1}, /* total:  3  08 */
  {"sa_ramp_flux_start", CFGSER_INT, 3, 1},            /* total:  4  09 */
  {"sa_ramp_flux_step",  CFGSER_INT, 4, 1},            /* total:  5  0A */
  {"sa_ramp_flux_count", CFGSER_INT, 5, 1},            /* total:  6  0B */
  {"sa_ramp_bias_start", CFGSER_INT, 6, 1},            /* total:  7  0C */
  {"sa_ramp_bias_step",  CFGSER_INT, 7, 1},            /* total:  8  0D */
  {"sa_ramp_bias_count", CFGSER_INT, 8, 1},            /* total:  9  0E */
  {"sq2_rows", CFGSER_INT, 9, 16},                     /* total: 25  0F */
  {"sq2_servo_gain", CFGSER_INT, 25, 16},              /* total: 41  10 */
  {"sq1_servo_gain", CFGSER_INT, 41, 16},              /* total: 57  11 */
  {"sq1_servo_flux_start", CFGSER_INT, 57, 1},         /* total: 58  12 */
  {"sq1_servo_flux_step",  CFGSER_INT, 58, 1},         /* total: 59  13 */
  {"sq1_servo_flux_count", CFGSER_INT, 59, 1},         /* total: 60  14 */
  {"sq1_servo_bias_start", CFGSER_INT, 60, 1},         /* total: 61  15 */
  {"sq1_servo_bias_step",  CFGSER_INT, 61, 1},         /* total: 62  16 */
  {"sq1_servo_bias_count", CFGSER_INT, 62, 1},         /* total: 63  17 */
  {"sq2_servo_flux_start", CFGSER_INT, 63, 1},         /* total: 64  18 */
  {"sq2_servo_flux_step",  CFGSER_INT, 64, 1},         /* total: 65  19 */
  {"sq2_servo_flux_count", CFGSER_INT, 65, 1},         /* total: 66  1A */
  {"sq2_servo_bias_start", CFGSER_INT, 66, 1},         /* total: 67  1B */
  {"sq2_servo_bias_step",  CFGSER_INT, 67, 1},         /* total: 68  1C */
  {"sq2_servo_bias_count", CFGSER_INT, 68, 1},         /* total: 69  1D */
  {"locktest_pass_amplitude", CFGSER_INT, 69, 1},      /* total: 70  1E */
  {"sq1_ramp_tes_bias_start", CFGSER_INT, 70, 1},      /* total: 71  1F */
  {"sq1_ramp_tes_bias_step",  CFGSER_INT, 71, 1},      /* total: 72  20 */
  {"sq1_ramp_tes_bias_count", CFGSER_INT, 72, 1},      /* total: 73  21 */
  {"tes_bias_idle", CFGSER_INT, 73, 16},               /* total: 89  22 */
  {"tes_bias_normal", CFGSER_INT, 89, 16},             /* total: 105 23 */
  {"tes_bias_normal_time", CFGSER_FLOAT, 105, 1, 0, 1},/* total: 106 24 */
  {"tuning_therm_time", CFGSER_INT, 106, 1},           /* total: 107 25 */
  {"sq2servo_safb_init", CFGSER_INT, 107, 16},         /* total: 123 26 */
  {"sq1servo_sq2fb_init", CFGSER_INT, 123, 16},        /* total: 139 27 */
  {"ramp_tes_start", CFGSER_INT, 139, 1},              /* total: 140 28 */
  {"ramp_tes_step",  CFGSER_INT, 140, 1},              /* total: 141 29 */
  {"ramp_tes_count", CFGSER_INT, 141, 1},              /* total: 142 2A */
  {"ramp_tes_final_bias", CFGSER_INT, 142, 1},         /* total: 143 2B */
  {"ramp_tes_initial_pause", CFGSER_INT, 143, 1},      /* total: 144 2C */
  {"num_rows_reported", CFGSER_INT, 144, 1},           /* total: 145 2D */
  {"readout_row_index", CFGSER_INT, 145, 1},           /* total: 146 2E */
  {"sample_dly", CFGSER_INT, 146, 1},                  /* total: 147 2F */
  {"sample_num", CFGSER_INT, 147, 1},                  /* total: 148 30 */
  {"fb_dly", CFGSER_INT, 148, 1},                      /* total: 149 31 */
  {"row_dly", CFGSER_INT, 149, 1},                     /* total: 150 32 */
  {"servo_mode", CFGSER_INT, 150, 1},                  /* total: 151 33 */
  {"tes_bias", CFGSER_INT, 151, 16},                   /* total: 167 34 */
  {"sa_flux_quanta", CFGSER_INT, 167, 16},             /* total: 183 35 */
  {"sq2_flux_quanta", CFGSER_INT, 183, 16},            /* total: 199 36 */
  {"flux_quanta_all", CFGSER_INT, 199, 528},           /* total: 727 37 */
  {"sq1_bias", CFGSER_INT, 727, 33},                   /* total: 760 38 */
  {"sq1_bias_off", CFGSER_INT, 760, 33},               /* total: 793 39 */
  {"sq2_bias", CFGSER_INT, 793, 16},                   /* total: 809 3A */
  {"sq2_fb", CFGSER_INT, 809, 528},                    /* total: 1337 3B */
  {"sa_bias", CFGSER_INT, 1337, 16},                   /* total: 1353 3C */
  {"sa_fb", CFGSER_INT, 1353, 16},                     /* total: 1369 3D */
  {"sa_offset", CFGSER_INT, 1369, 16},                 /* total: 1385 3E */
  {"adc_offset_cr", CFGSER_INT, 1385, 528},            /* total: 1913 3F */
  {"servo_p", CFGSER_INT, 1913, 16},                   /* total: 1929 40 */
  {"servo_i", CFGSER_INT, 1929, 16},                   /* total: 1945 41 */
  {"servo_d", CFGSER_INT, 1945, 16},                   /* total: 1961 42 */
  {"frail_servo_p", CFGSER_INT, 1961, 1},              /* total: 1962 43 */
  {"frail_servo_i", CFGSER_INT, 1962, 1},              /* total: 1963 44 */
  {"frail_servo_d", CFGSER_INT, 1963, 1},              /* total: 1964 45 */
  {"dead_detectors", CFGSER_BITARR, 1964, 33},         /* total: 1997 46 */
  {"frail_detectors", CFGSER_BITARR, 1997, 33},        /* total: 2030 47 */
  {0}
};

