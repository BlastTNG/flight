#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "tx_struct.h"
//#include "sunsensor.h"

unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

/* read and write channel 56 on all boards reserved for ADC Sync */
struct ChannelStruct SlowChList[N_SLOW][FAST_PER_SLOW] = {
  {
    {"cpu_time", 'w',  17,  8, 1.0,         0.0,    'U'},      //OK
    {"t_roll",   'r',  1,  9, -0.00625,     136.45,  'u'},     // OK
    {"i_roll",   'r',  1, 11, 0.00048828125,   -16.09,   'u'},
    {"t_race",   'r',  1, 13, -0.00625,     136.45,  'u'},
    {"t_gyro2",  'r',  1, 17, -0.003125,   79.8,   'u'},     // OK
    {"t_gyro3",  'r',  1, 19, -0.003125,   79.8,   'u'},     // OK
    {"t_gyro1",  'r',  1, 21, -0.003125,   79.8,   'u'},     // OK
    {"t_reac",   'r',  1, 29, -0.00625,     136.45,  'u'},    // OK
    {"t_reac_mc", 'r',  1, 31, -0.00625,     136.45,  'u'}, //  OK
    {"i_reac",   'r',  1, 33, 1.0/1648.0, -32768.0/1648.0, 'u'},
    {"t_el_mot",  'r',  1, 35, -0.00625,     136.45,  'u'},    // OK
    {"t_el_mc",   'r',  1, 37, -0.00625,     136.45,  'u'},    // OK
    {"i_el",   'r',  1, 39, 1.0/1648, -32768.0/1648,   'u'},
    {"i_gybox",  'r',  2,  5, -0.000625, 20.48,   'u'},       // OK
    {"t_pv",     'r',  2, 33, -0.00625,     136.45,  'u'},     // OK
    {"i_pv",     'r',  2, 35, -0.000625, 20.48,   'u'},     // OK
    {"t_apm_3v",  'r',  2, 37, -0.00625,   136.45, 'u'},     // OK
    {"t_apm_5v",  'r',  2, 39, -0.00625,    136.45, 'u'},     // OK
    {"t_apm_10v", 'r',  2, 41, -0.00625,    136.45, 'u'},     // OK
    {"i_apm_5v",  'r',  2, 45, -0.000625, 20.48,   'u'}     // OK
  },
  {
    {"",         'w',  17,  9, 1.0,         0.0,    'u'}, // OK
    {"i_apm_10v", 'r',  2, 47, -0.000625,     20.48,   'u'},     // OK
    {"t_if1",     'r',  4,  5, -0.00625,     136.45,  'u'},
    {"t_if2",     'r',  4,  7, -0.00625,     136.45,  'u'},
    {"t_if3",     'r',  4,  9, -0.00625,     136.45,  'u'},
    {"t_if4",     'r',  4, 11, -0.00625,     136.45,  'u'},
    {"t_if5",     'r',  4, 13, -0.00625,     136.45,  'u'},
    {"t_if6",     'r',  4, 15, -0.00625,     136.45,  'u'},
    {"t_if7",     'r',  4, 17, -0.00625,     136.45,  'u'},
    {"t_if8",     'r',  4, 19, -0.00625,     136.45,  'u'},
    {"t_if9",     'r',  4, 21, -0.00625,     136.45,  'u'},
    {"i_dpm_28v",    'r',  4, 23, -0.000625,     20.48,   'u'},
    {"i_dpm_3v",     'r',  4, 25, -0.000625,     20.48,   'u'},
    {"i_dpm_5v",     'r',  4, 27, -0.000625,     20.48,   'u'},
    {"i_dpm_10v",    'r',  4, 29, -0.000625,     20.48,   'u'},
    {"i_rec",        'r',  4, 31, -0.000625,     20.48,   'u'},
    {"t_dpm_7.5v",   'r',  4, 41, -0.00625,     136.45,  'u'},
    {"t_dpm_10v",    'r',  4, 43, -0.00625,     136.45,  'u'},
    {"t_dpm_5v",     'r',  4, 45, -0.00625,     136.45,  'u'},
    {"t_dpm_3v",     'r',  4, 47, -0.00625,     136.45,  'u'},
  },
  {
    {"i_charcoal",   'r', 3,   3, -2.639826420E-6,    0.157988332,   'u'},  //OK Enzo June 13 2003
    {"he4_lev",      'r', 3,   1, -7.58182245E-5,     4.960560932,   'u'},  //OK Enzo June 13 2003
    //{"i_coldplate",  'r', 3,   5, -6.2723597E-5,    4.114858676,   'u'},
    {"i_coldplate",  'r', 3,   5, -2.32217573E-5,     1.390309833,   'u'}, //OK Enzo June 13 2003
    {"i_heatswitch", 'r', 3,   7, -2.87477e-09*65536,  12.3273561,   'u'},
    {"i_he3",        'r', 3,   9, -2.87539e-09*65536,  12.3309023,   'u'},
    {"t_vcs1",       'r', 3,  11, -2.87784e-09*65536,  12.3420847,   'u'},
    {"t_vcs_fet",    'r', 3,  13, -2.86354e-09*65536,  12.2829004,   'u'},
    {"t_he4",        'r', 3,  15, -2.8768e-09*65536,   12.3399714,   'u'},
    {"t_ln2",        'r', 3,  17, -2.86817e-09*65536,  12.2988073,   'u'},
    {"t_jfet",       'r', 3,  19, -2.8648e-09*65536,   12.2876163,   'u'},
    {"t_mirror3",    'r', 3,  21, -2.87765e-09*65536,  12.3369079,   'u'},
    {"t_mirror5",    'r', 3,  23, -2.88013e-09*65536,  12.3447907,   'u'},
    {"t_lyot_stim",  'r', 3,  25, -2.88055e-09*65536,  12.3452373,   'u'},
    {"t_heatswitch", 'r', 3,  27, -2.88295e-09*65536,  12.3571711,   'u'},
    {"t_charcoal",   'r', 3,  29, -2.87356e-09*65536,  12.3273120,   'u'},
    {"t_he4pot",     'r', 3,  31, -2.87221e-09*65536,  12.3169612,   'u'},
    {"t_mirror4",    'r', 3,  33, -2.86827e-09*65536,  12.2987976,   'u'},
    {"t_ln_filt",    'r', 3,  35,   1.0,                0.0,         'u'},
    {"t_vcs_filt",   'r', 3,  37,   1.0,                0.0,         'u'},
    {"t_he_filt",    'r', 3,  39, -2.87516e-09*65536,  12.3290947,   'u'}
  },
  {
    {"t_he3",        'r',  3,  41, -2.87482e-09*65536, 12.4223147,   'u'},
    {"t_bda1",       'r',  3,  43, -2.8681e-09*65536,  12.3164528,   'u'},
    {"t_bda2",       'r',  3,  45, -2.87714e-09*65536, 12.3240610,   'u'},
    {"t_bda3",       'r',  3,  47, -2.8779e-09*65536,  12.3446613,   'u'},
    //{"he4_level",    'r',  3,  49, -2.90117e-09*65536, 12.4604317,   'u'},
    {"",             'r',  3,  49, -2.90117e-09*65536, 12.4604317,   'u'},
    {"cryoout1",     'w',  3,   0, 1.0,             0.0,   'u'},
    {"cryoout2",     'w',  3,   1, 1.0,             0.0,   'u'},
    {"cryoout3",     'w',  3,   2, 1.0,             0.0,   'u'},
    {"i_race",       'r',  1,  41, 1.0/1648,-32768.0/1648,  'u'},
    {"t_piv_mc",     'r',  1,  45, -0.00625,     136.45,   'u'},
    {"i_piv",        'r',  1,  47, 1.0/1648,-32768.0/1648,  'u'},
    {"t_clin_if",    'r', 21,  41, -0.00625,     136.45,   'u'},
    {"t_clin_sip",   'r', 21,  31, -0.00625,     136.45,   'u'},
    {"t_gy_set",     'w', 31,   4, (100.0/32768.0), 0.0,   'u'},
    {"g_p_gyheat",   'w', 17,   5, 1.0,             0.0,   'u'},
    {"g_i_gyheat",   'w', 17,   6, 1.0,             0.0,   'u'},
    {"g_d_gyheat",   'w', 17,   7, 1.0,             0.0,   'u'},
    {"g_p_el",       'w',  1,   2, 1.0,             0.0,   'u'},
    {"g_i_el",       'w',  1,   3, 1.0,             0.0,   'u'},
    {"lokmot_pin",    'w', 17, 22, 1.0,     0.0,   'u'},
  },
  {
    {"biasout1",      'w',  4,  0, 1.0,     0.0,   'u'},
    {"biasout2",      'w',  4,  1, 1.0,     0.0,   'u'},
    {"status00",      'r', 21, 56, 1.0,     0.0,   'u'},
    {"status01",      'r',  1, 57, 1.0,     0.0,   'u'},
    {"status02",      'r',  2, 56, 1.0,     0.0,   'u'},
    {"status03",      'r',  3, 57, 1.0,     0.0,   'u'},
    {"status04",      'r',  4, 57, 1.0,     0.0,   'u'},
    {"status05",      'r',  5, 57, 1.0,     0.0,   'u'},
    {"status06",      'r',  6, 57, 1.0,     0.0,   'u'},
    {"status07",      'r',  7, 57, 1.0,     0.0,   'u'},
    {"status08",      'r',  8, 57, 1.0,     0.0,   'u'},
    {"status09",      'r',  9, 57, 1.0,     0.0,   'u'},
    {"status10",      'r', 10, 57, 1.0,     0.0,   'u'},
    {"status11",      'r', 11, 57, 1.0,     0.0,   'u'},
    {"status12",      'r', 12, 57, 1.0,     0.0,   'u'},
    {"status13",      'r', 13, 57, 1.0,     0.0,   'u'},
    {"status14",      'r', 14, 57, 1.0,     0.0,   'u'},
    {"status15",      'r', 15, 57, 1.0,     0.0,   'u'},
    {"status16",      'r', 16, 57, 1.0,     0.0,   'u'},
    {"i_isc",         'r',  2, 15, -0.000625, 20.48,'u'}
  },
  {
    {"g_p_roll",      'w',  1,   5, 1.0,     0.0,   'u'},
    {"sync",          'w', 17, 56, 1.0,     0.0,   'u'},
    {"vsc_col",       'w', 18,  0, 0.01,     0.0,   'u'},
    {"vsc_row",       'w', 18,  1, 0.01,     0.0,   'u'},
    {"vsc_mag",       'w', 18,  2, 1.0,     0.0,   'u'},
    {"vsc_fra",       'w', 18,  3, 1.0,     0.0,   'u'},
    {"az_rel_sun",    'w', 18,  4, (140./65536.), 0.0,   's'},
    {"el_rel_sun",    'w', 18,  5, (140./65536.), 0.0,   's'},
    {"ss_prin",       'w', 18,  6, 1.0,     0.0,   'u'},
    {"sam_i_am",      'w', 18,  7, 1.0,     0.0,   'u'},
    {"spare45",       'r', 18, 45, -0.00625,       136.45, 'u'},
    {"cpu_fan",       'w', 17, 13, 1.0,     0.0,   'u'},
    {"t_cpu",         'w', 17, 14, 1.0,     0.0,   'u'},
    {"i_apm_3v",      'r',  2, 43, -0.000625,     20.48,   'u'},     // OK
    {"t_isc_set",     'w', 31,  5, (100.0/32768.0), 0.0,   'u'},
    {"g_p_ischeat",   'w', 17, 16, 1.0,     0.0,   'u'},
    {"g_i_ischeat",   'w', 17, 17, 1.0,     0.0,   'u'},
    {"set_reac",      'w',  1, 16, 7.9498291016e-5,-2.605,   'u'},
    {"pump_bits",     'w', 21,  2, 1.0,     0.0,   'u'},
    {"g_d_ischeat",   'w', 17, 18, 1.0,     0.0,   'u'},  
  },
  {
    {"balpump_lev",   'w', 21,  3, -0.048851978505, 100.0,   'u'},
    {"sprpump_lev",   'w', 21,  4, -0.048851978505, 100.0,   'u'},
    {"inpump_lev",    'w', 21,  5, -0.048851978505, 100.0,   'u'},
    {"outpump_lev",   'w', 21,  6, -0.048851978505, 100.0,   'u'},
    {"g_p_az",        'w',  1,  7, 1.0,     0.0,   'u'},
    {"g_i_az",        'w',  1,  8, 1.0,     0.0,   'u'},
    {"g_p_pivot",     'w',  1, 15, 1.0,     0.0,   'u'},
    {"pch_clin_pyr",'r', 21, 33, 4.0/5333.3333,   -4.*6.144,   'u'},
    {"rll_clin_pyr", 'r', 21, 35, 4.0/5333.3333,   -4.*6.144,   'u'},
    {"t_clin_pyr",    'r', 21, 23, -0.00625,     136.45,   'u'},
    {"bal_on",        'w', 17, 27, 1./1648.,0.0,   'u'},
    {"bal_off",       'w', 17, 28, 1./1648.,0.0,   'u'},
    {"bal_target",    'w', 17, 29, 1./1648.,0.0,   'u'},
    {"bal_veto",      'w', 17, 30, 1.0,     0.0,   'u'},
    {"pch_clin_piv",'r',  2,  7, 4.0/5333.3333,   -4.*6.144,   'u'},
    {"rll_clin_piv", 'r',  2,  9, 4.0/5333.3333,   -4.*6.144,   'u'},
    {"t_clin_piv",    'r',  2, 11, -0.00625,     136.45,   'u'},
    {"he3pwm",   'w',  3,  3,  1.0,             0.00, 'u'}, // OK
    {"jfetpwm",  'w',  3,  4, -0.000625,       20.48, 'u'},     // OK
    {"hspwm",    'w',  3,  5, -0.00625,       136.45, 'u'},
  },
  {
    {"pch_clin_pyr",'r', 21, 33, 4.0/5333.3333,   -4.*6.144,   'u'},
    {"rll_clin_pyr", 'r', 21, 35, 4.0/5333.3333,   -4.*6.144,   'u'},
    {"t_clin_pyr",    'r', 21, 23, -0.00625,     136.45,   'u'},
    {"bias_lev1",       'w', 17, 43, 1.0,     0.0,   'u'},
    {"bias_lev2",       'w', 17, 44, 1.0,     0.0,   'u'},
    {"bias_lev3",       'w', 17, 45, 1.0,     0.0,   'u'},
    {"spare46",       'r', 18, 46, 1.0,     0.0,   'u'},
    {"spare47",       'r', 18, 47, 1.0,     0.0,   'u'},
    {"phase5",     'w',    5, 10, 1.0,       0.0,    'u'},
    {"phase6",     'w',    6, 10, 1.0,       0.0,    'u'},
    {"phase7",     'w',    7, 10, 1.0,       0.0,    'u'},
    {"phase8",     'w',    8, 10, 1.0,       0.0,    'u'},
    {"phase9",     'w',    9, 10, 1.0,       0.0,    'u'},
    {"phase10",    'w',   10, 10, 1.0,       0.0,    'u'},
    {"phase11",    'w',   11, 10, 1.0,       0.0,    'u'},
    {"phase12",    'w',   12, 10, 1.0,       0.0,    'u'},
    {"phase13",    'w',   13, 10, 1.0,       0.0,    'u'},
    {"phase14",    'w',   14, 10, 1.0,       0.0,    'u'},  /* delta_az */
    {"phase15",    'w',   15, 10, 1.0,       0.0,    'u'},  /* delta_el */
    {"phase16",    'w',   16, 10, 1.0,       0.0,    'u'},  
  }
};

struct ChannelStruct FastChList[N_FASTCHLIST] = {
#ifndef BOLOTEST
    /* read channels from ACS0 */
    {"f_i_el",         'r',  1, 39, 1.0/(0.01*10.0)*4.096/65536.0,
     -32768.0*1.0/(0.01*10.0)*4.096/65536.0,   'u'},

    {"t_isc",          'r',  21, 24, -9.5367431641e-08,  136.45,  'U'},
    {"",               'r',  21, 25, 1.0,     0.0,   'u'},
    {"roll_clin_sip",  'r',  21, 27, 4.0/5333.3333, -4.*6.144,   'u'},
    {"pch_clin_sip", 'r',  21, 29, 4.0/5333.3333, -4.*6.144,   'u'},

    {"xel_clin_if",    'r',  21, 37, 0.00546739, -25.*6.144,   'u'},
    {"el_clin_if",     'r',  21, 39, 0.00546739, -135.5,   'u'},
    {"mag_x",          'r',  21, 45, 1.0,     0.0,   'u'},
    {"mag_y",          'r',  21, 47, 1.0,     0.0,   'u'},
    {"acs0bits",       'r',  21, 59, 1.0,     0.0,   'u'},

    {"az",             'w',  17, 20, 0.005493164,-180.0,'u'},
    {"el",             'w',  17, 21, 360.0/65536.0, 0.0,  'u'},

    /* send data to ACS0 */
    {"isc_bits",  'w', 21,  1, 1.0, 0.0, 'u'},

    /* read channels from ACS1 */
    {"t_gybox",  'r',  1, 14, -9.5367431641e-08,  136.45,  'U'},
    {"",         'r',  1, 15, 1.0,     0.0,   'u'},
    {"gyro2",    'r',  1, 22, 0.00091506980885, -25535.0*0.00091506980, 'u'},
    {"raw_gy2",  'r',  1, 23, 0.00091506980885, -25747.0*0.00091506980, 'u'},
    {"gyro3",    'r',  1, 24, 0.00091506980885, -25600.0*0.00091506980, 'u'},
    {"raw_gy3",  'r',  1, 25, 0.00091506980885, -25804.0*0.00091506980, 'u'},
    {"gyro1",    'r',  1, 26, 0.00091506980885, -25794.0*0.00091506980, 'u'},
    {"raw_gy1",  'r',  1, 27, 0.00091506980885, -25800.0*0.00091506980, 'u'},

    {"temp1",   'r',  1, 62, 1.0,     0.0,   'u'},
    {"temp2",   'r',  1, 63, 1.0,     0.0,   'u'},

    {"piv_enc",  'r',  1, 59, 360.0/8192.0,   0.0,   'u'},
    {"reac_enc", 'r',  1, 60, 360.0/4000.0,   0.0,   'u'},
    {"pwm_el",   'r',  1, 51, 1.0,     -4000.0,'u'},
    {"pwm_roll", 'r',  1, 52, 1.0,     -4000.0,'u'},
    {"r_dt",     'r',  1, 53, 1.0,     -4000.0,'u'},
    {"pwm_reac",   'r',  1, 54, 1.0,     -4000.0,    'u'},
    {"rps_reac",  'r',  1, 55, 7.9498291016e-05, -2.605, 'u'},
    {"pwm_piv",  'r',  1, 61, 1.0,     -4000.0,    'u'},

  /* send data to ACS1 */
    {"gy_heat",  'w',  1,  1, 1.0, 0.0, 'u'},
    {"el_vreq",  'w',  1,  4, 1.0, -32768, 'u'}, /* req speed for el drive */
    {"az_vreq",  'w',  1,  14, 1.0, -32768, 'u'}, /* req speed for az drive */
    {"cos_el",   'w',  1,  9, 1.0, 0.0, 'u'},
    {"sin_el",   'w',  1, 10, 1.0,0.0, 'u'},

  /* read from board ACS2 */
    {"enc_elev", 'r', 2, 50, -360.0/65536.0,   ENC_ELEV_OFFSET,   'u'},
#endif

    // Read Requests: for signalling the das to copy data to the BB memory
    /* !!! WARNING!  MCP assumes these appear sequentially in the frame !!! */
    {"readd3",         'w', 3,  40, 1.0,     0.0,   'u'},
    {"readd4",         'w', 4,  40, 1.0,     0.0,   'u'},
    {"readd5",         'w', 5,  40, 1.0,     0.0,   'u'},
    {"readd6",         'w', 6,  40, 1.0,     0.0,   'u'},
    {"readd7",         'w', 7,  40, 1.0,     0.0,   'u'},
    {"readd8",         'w', 8,  40, 1.0,     0.0,   'u'},
    {"readd9",         'w', 9,  40, 1.0,     0.0,   'u'},
    {"readd10",         'w', 10,  40, 1.0,     0.0,   'u'},
    {"readd11",         'w', 11,  40, 1.0,     0.0,   'u'},
    {"readd12",         'w', 12,  40, 1.0,     0.0,   'u'},
    {"readd13",         'w', 13,  40, 1.0,     0.0,   'u'},
    {"readd14",         'w', 14,  40, 1.0,     0.0,   'u'},
    {"readd15",         'w', 15,  40, 1.0,     0.0,   'u'},
    {"readd16",         'w', 16,  40, 1.0,     0.0,   'u'},
    
    {"n13c40",         'r', 13,  40, 1.0,     0.0,   'u'},
    {"n13c41",         'r', 13,  41, 1.0,     0.0,   'u'},

  /* Read from DAS4 -- bias controller and DPM/inner frame monitoring */
    {"biasin",       'r',   4,  50, 1.0,     0.0,   'u'},
    /* BIAS Amplitude */
//    {"b_amp2",         'r', 4,   0, 1.0,     0.0,   'U'},
//    {"",               'r', 4,   1, 1.0,     0.0,   'u'},
//    {"b_amp1",         'r', 4,   2, 1.0,     0.0,   'U'},
//    {"",               'r', 4,   3, 1.0,     0.0,   'u'},
//    {"b_amp3",         'r', 4,  38, 1.0,     0.0,   'U'},
//    {"",               'r', 4,  39, 1.0,     0.0,   'u'},
//    {"n3ref",    'r', 4, 48, 0.00390625,     -8388608.0,   'U'},
//    {"",         'r', 4, 49, 1.0,     0.0,   'u'},

    {"n5ref",    'r',  5, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r',  5, 37, 1.19209e-7,     0.0,   'u'},
    {"n6ref",    'r',  6, 36, 1.19209e-7,    0.0,   'U'},
    {"",         'r',  6, 37, 1.19209e-7,     0.0,   'u'},
    {"n7ref",    'r',  7, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r',  7, 37, 1.19209e-7,     0.0,   'u'},
    {"n8ref",    'r',  8, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r',  8, 37, 1.19209e-7,     0.0,   'u'},
    {"n9ref",    'r',  9, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r',  9, 37, 1.19209e-7,     0.0,   'u'},
    {"n10ref",   'r', 10, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r', 10, 37, 1.19209e-7,     0.0,   'u'},
    {"n11ref",   'r', 11, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r', 11, 37, 1.19209e-7,     0.0,   'u'},
    {"n12ref",   'r', 12, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r', 12, 37, 1.19209e-7,     0.0,   'u'},
    {"n13ref",   'r', 13, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r', 13, 37, 1.19209e-7,     0.0,   'u'},
    {"n14ref",   'r', 14, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r', 14, 37, 1.19209e-7,     0.0,   'u'},
    {"n15ref",   'r', 15, 36, 1.23301e-7,    0.0,   'U'},
    {"",         'r', 15, 37, 1.23301e-7,    0.0,   'u'},
    {"n16ref",   'r', 16, 36, 1.19209e-7,     0.0,   'U'},
    {"",         'r', 16, 37, 1.19209e-7,     0.0,   'u'},

/*     {"testbolo", 'r', 5, 38, 1.0,     0.0,   'U'}, */
/*     {"",         'r', 5, 39, 1.0,     0.0,   'u'}, */

    {"ENDMARKER",'x', 0,  0, 0,     0.0,   'x'}
};

/************************************************************************\
|*                                                                      *|
|*    MakeTxFrame: insert bolometer channels into the Frame             *|
|*                                                                      *|
\************************************************************************/
void MakeTxFrame(void) {
  int i, j, last_fastchlist = N_FASTCHLIST_INIT;
  struct ChannelStruct channel = {"", 'r', 3, 0, 1.19209e-7, -2097152.0, 'u'};

  if (strncmp(FastChList[last_fastchlist].field, "ENDMARKER", 9)!=0) {
    printf("error: N_FASTCHLIST_INIT not correct\n");
    printf("%d field: %s\n", last_fastchlist,
    FastChList[last_fastchlist].field);
    exit(1);
  }
  
  for (i = 0; i < DAS_CARDS; ++i) {
    channel.node = i + 5;
    channel.rw = 'r';
    for (j = 0; j < DAS_CHS; j += 2) {
      /* lsw channel at j */
      channel.adr = j;
      sprintf(channel.field, "n%ic%ilo", channel.node, j); /* ignored */
      boloIndex[i][j][0] = last_fastchlist + FAST_OFFSET;
      memcpy(&FastChList[last_fastchlist++], &channel, sizeof(channel));
      /* msw at j and j+1 */
      channel.adr = DAS_CHS + (j >> 1);
      sprintf(channel.field, "n%ic%ihi", channel.node, j); /* ignored */
      boloIndex[i][j + 1][1] = boloIndex[i][j][1] =
          last_fastchlist + FAST_OFFSET;
      memcpy(&FastChList[last_fastchlist++], &channel, sizeof(channel));
      /* lsw channel at j+1 */
      channel.adr = j+1;
      sprintf(channel.field, "n%ic%ilo", channel.node, j+1); /* ignored */
      boloIndex[i][j+1][0] = last_fastchlist + FAST_OFFSET;
      memcpy(&FastChList[last_fastchlist++], &channel, sizeof(channel));
    }
  } 
}

/************************************************************************\
|*                                                                      *|
|*    FastChIndex                                                       *|
|*                                                                      *|
\************************************************************************/
void FastChIndex(char* field, int* index) {
  int i, t = -1;

  for (i = 0; i < N_FASTCHLIST && t == -1; ++i)
    if (strcmp(FastChList[i].field, field) == 0)
      t = i + FAST_OFFSET;

  if (t == -1) {
    fprintf(stderr, "Can't find fast channel: %s\n", field);
    exit(1);
  }

  *index = t;
}

/************************************************************************\
|*                                                                      *|
|*    SlowChIndex                                                       *|
|*                                                                      *|
\************************************************************************/
void SlowChIndex(char* field, int* channel, int* index) {
  int i, j, t = -1, c = -1;

  for (i = 0; i < N_SLOW && t == -1; ++i)
    for (j = 0; j < FAST_PER_SLOW; ++j)
      if (strcmp(SlowChList[i][j].field, field) == 0) {
        c = i;
        t = j;
      }

  if (t == -1) {
    fprintf(stderr, "Can't find slow channel: %s\n", field);
    exit(1);
  }

  *index = t;
  *channel = c;
}
