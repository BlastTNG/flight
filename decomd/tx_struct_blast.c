#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "tx_struct.h"
#include "bbc_pci.h"

/* if compiling MCP load the real mprintf function prototypes, otherwise, just
 * make up a fake one */
#ifdef __MCP__
#  include "mcp.h"
#else
#  define mprintf(x, ...) \
     do {  /* encase in a do {} while(0) loop to properly swallow the ; */ \
       printf(__VA_ARGS__); \
       if (strcmp(#x, "MCP_FATAL") == 0) \
         exit(1); \
     } while (0)
#endif

unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

unsigned short ccWideFast;
unsigned short ccNarrowFast;
unsigned short ccWideSlow;
unsigned short ccNarrowSlow;
unsigned short ccSlow;
unsigned short ccFast;
unsigned short ccNoBolos;
unsigned short ccTotal;

unsigned short BiPhaseFrameWords;
unsigned short BiPhaseFrameSize;
unsigned short TxFrameWords[2];
unsigned short TxFrameSize[2];
unsigned short slowsPerBi0Frame;
unsigned short slowCount[2] = {0, 0};
unsigned short slowsPerBusFrame[2];
unsigned short fastsPerBusFrame[2] = {FAST_OFFSET, 1};

struct NiosStruct* NiosLookup;
unsigned int NiosSpares[FAST_PER_SLOW * 2];
unsigned int BBCSpares[FAST_PER_SLOW * 2];
struct BiPhaseStruct *BiPhaseLookup;

/* bus on which the bolometers are */
#define BOLO_BUS  1
struct ChannelStruct BoloChannels[N_FAST_BOLOS];

/* card name to (node number, bus number) mapping */
#define ACS1   1, 0
#define ACS2   2, 0
#define CRYO   3, 1
#define BIAS   4, 1
#define DAS5   5, 1
#define DAS6   6, 1
#define DAS7   7, 1
#define DAS8   8, 1
#define DAS9   9, 1
#define DAS10 10, 1
#define DAS11 11, 1
#define DAS12 12, 1
#define DAS13 13, 1
#define DAS14 14, 1
#define DAS15 15, 1
#define DAS16 16, 1
#define LOOP1 17, 0
#define LOOP2 18, 0
#define LOOP3 19, 0
#define LOOP4 20, 0
#define ACS0  21, 0
#define ACSN  22, 0

/* other reserved node numbers */
#define SPARE      62
#define SPECIAL    63
#define EOC_MARKER -1
#define END_OF_CHANNELS {"", 'x', EOC_MARKER, -1, 0, 0}

/* read and write channel 56 on all boards reserved for ADC Sync */
struct ChannelStruct WideSlowChannels[] = {
  {"cpu_time",     'w', LOOP1,  0,                1.0,             0.0, 'U'},
  {"sip_time",     'w', LOOP1,  6,                1.0,             0.0, 'U'},
  {"lst",          'w', LOOP1,  2,         1.0/3600.0,             0.0, 'U'},
  {"dgps_time",    'w', LOOP1,  8,                1.0,             0.0, 'U'},
  {"isc_ra",       'w', LOOP1,  4,             LI2DEG,             0.0, 'U'},
  {"isc_dec",      'w', LOOP1, 10,          LI2DEG/2.,            -90., 'U'},
  {"time",         'w', LOOP2, 28,                1.0,             0.0, 'U'},
  END_OF_CHANNELS
};

struct ChannelStruct SlowChannels[] = {
  {"t_el_mc",      'r',  ACS0,  3,           -0.00625,          136.45, 'u'},
  {"t_el_mot",     'r',  ACS0,  5,           -0.00625,          136.45, 'u'},
  {"t_osc_heat",   'r',  ACS0,  7,                1.0,             0.0, 'u'},
  {"t_osc_flange", 'r',  ACS0,  9,                1.0,             0.0, 'u'},
  {"t_osc_comp",   'r',  ACS0, 11,                1.0,             0.0, 'u'},
  {"t_osc_lens",   'r',  ACS0, 13,                1.0,             0.0, 'u'},
  {"t_isc_heat",   'r',  ACS0, 15,                1.0,             0.0, 'u'},
  {"t_isc_flange", 'r',  ACS0, 17,                1.0,             0.0, 'u'},
  {"t_isc_comp",   'r',  ACS0, 19,                1.0,             0.0, 'u'},
  {"i_starcam",    'r',  ACS0, 21,                1.0,             0.0, 'u'},
  {"t_isc_lens",   'r',  ACS0, 25,           -0.00625,          136.45, 'u'},
  {"roll_clin_pyr",'r',  ACS0, 33,     -4.0/5333.3333,        4.*6.144, 'u'},
  {"pch_clin_pyr", 'r',  ACS0, 35,      4.0/5333.3333,       -4.*6.144, 'u'},
  {"t_clin_pyr",   'r',  ACS0, 23,           -0.01875,           614.4, 'u'},
  {"t_clin_sip",   'r',  ACS0, 31,           -0.01875,           614.4, 'u'},
  {"t_clin_if",    'r',  ACS0, 41,           -0.01875,           614.4, 'u'},
  {"status00",     'r',  ACS0, 56,                1.0,             0.0, 'u'},
  {"pump_bits",    'w',  ACS0,  2,                1.0,             0.0, 'u'},
  {"balpump_lev",  'w',  ACS0,  3,    -0.048851978505,           100.0, 'u'},
  {"sprpump_lev",  'w',  ACS0,  4,    -0.048851978505,           100.0, 'u'},
  {"inpump_lev",   'w',  ACS0,  5,    -0.048851978505,           100.0, 'u'},
  {"outpump_lev",  'w',  ACS0,  6,    -0.048851978505,           100.0, 'u'},

  {"t_roll",       'r',  ACS1,  9,           -0.00625,          136.45, 'u'},
  {"i_roll",       'r',  ACS1, 11,      0.00048828125,          -16.09, 'u'},
  {"t_gyro2",      'r',  ACS1, 17,          -0.003125,            79.8, 'u'},
  {"t_gyro3",      'r',  ACS1, 19,          -0.003125,            79.8, 'u'},
  {"t_gyro1",      'r',  ACS1, 21,          -0.003125,            79.8, 'u'},
  {"t_reac",       'r',  ACS1, 29,           -0.00625,          136.45, 'u'},
  {"t_reac_mc",    'r',  ACS1, 31,           -0.00625,          136.45, 'u'},
  {"i_reac",       'r',  ACS1, 33,         1.0/1648.0, -32768.0/1648.0, 'u'},
  {"t_gyro5",      'r',  ACS1, 35,           -0.00625,          136.45, 'u'},
  {"t_gyro6",      'r',  ACS1, 37,                1.0,             0.0, 'u'},
  {"i_el",         'r',  ACS1, 39,           1.0/1648,   -32768.0/1648, 'u'},
  {"t_gyro4",      'r',  ACS1, 41,           1.0/1648,   -32768.0/1648, 'u'},
  {"t_piv_mc",     'r',  ACS1, 45,           -0.00625,          136.45, 'u'},
  {"i_piv",        'r',  ACS1, 47,           1.0/1648,   -32768.0/1648, 'u'},
  {"status01",     'r',  ACS1, 57,                1.0,             0.0, 'u'},
  {"g_p_el",       'w',  ACS1,  2,                1.0,             0.0, 'u'},
  {"g_i_el",       'w',  ACS1,  3,                1.0,             0.0, 'u'},
  {"g_p_roll",     'w',  ACS1,  5,                1.0,             0.0, 'u'},
  {"g_p_az",       'w',  ACS1,  7,                1.0,             0.0, 'u'},
  {"g_i_az",       'w',  ACS1,  8,                1.0,             0.0, 'u'},
  {"emf_gain",     'w',  ACS1, 11,                1.0,             0.0, 'u'},
  {"emf_offset",   'w',  ACS1, 12,                1.0,             0.0, 'u'},
  {"g_p_pivot",    'w',  ACS1, 15,                1.0,             0.0, 'u'},
  {"set_reac",     'w',  ACS1, 16,    7.9498291016e-5,          -2.605, 'u'}, 

  {"t_battery",    'r',  ACS2,  1,           -0.00625,          136.45, 'u'},
  {"t_out_heatx",  'r',  ACS2,  3,           -0.00625,          136.45, 'u'},
  {"i_gybox",      'r',  ACS2,  5,          -0.000625,           20.48, 'u'},
  {"pch_clin_piv", 'r',  ACS2,  7,      4.0/5333.3333,       -4.*6.144, 'u'},
  {"rll_clin_piv", 'r',  ACS2,  9,      4.0/5333.3333,       -4.*6.144, 'u'},
  {"t_clin_piv",   'r',  ACS2, 11,            0.01875,          -614.4, 'u'},
  {"i_sun",        'r',  ACS2, 13,          -0.000625,           20.48, 'u'},
  {"p_pv",         'r',  ACS2, 15,                1.0,             0.0, 'u'},
  {"t_scoop_tip",  'r',  ACS2, 17,           -0.00625,          136.45, 'u'},
  {"t_fl_sshield", 'r',  ACS2, 19,           -0.00625,          136.45, 'u'},
  {"t_fr_sshield", 'r',  ACS2, 21,           -0.00625,          136.45, 'u'},
  {"t_bl_sshield", 'r',  ACS2, 23,           -0.00625,          136.45, 'u'},
  {"t_br_sshield", 'r',  ACS2, 25,           -0.00625,          136.45, 'u'},
  {"t_el_bearing", 'r',  ACS2, 27,           -0.00625,          136.45, 'u'},
  {"t_lock_motor", 'r',  ACS2, 29,           -0.00625,          136.45, 'u'},
  {"t_sun_sensor", 'r',  ACS2, 31,           -0.00625,          136.45, 'u'},
  {"t_pv",         'r',  ACS2, 33,           -0.00625,          136.45, 'u'},
  {"i_pv",         'r',  ACS2, 35,       -0.002083333,          68.267, 'u'},
  {"t_apm_3v",     'r',  ACS2, 37,           -0.00625,          136.45, 'u'},
  {"t_apm_5v",     'r',  ACS2, 39,           -0.00625,          136.45, 'u'},
  {"t_apm_10v",    'r',  ACS2, 41,           -0.00625,          136.45, 'u'},
  {"i_apm_3v",     'r',  ACS2, 43,           0.000625,          -20.48, 'u'},
  {"i_apm_5v",     'r',  ACS2, 45,         -0.0020833,          68.267, 'u'}, 
  {"i_apm_10v",    'r',  ACS2, 47,          -0.000625,           20.48, 'u'},
  {"status02",     'r',  ACS2, 56,                1.0,             0.0, 'u'},

  {"v_p_batt",     'r',  ACSN,  1,                1.0,             0.0, 'u'},
  {"v_s_batt",     'r',  ACSN,  3,                1.0,             0.0, 'u'},
  {"t_dpcu",       'r',  ACSN,  5,                1.0,             0.0, 'u'},
  {"t_apcu",       'r',  ACSN,  7,                1.0,             0.0, 'u'},
  {"t_p_sol",      'r',  ACSN,  9,                1.0,             0.0, 'u'},
  {"t_s_sol",      'r',  ACSN, 11,                1.0,             0.0, 'u'},
  {"t_p_batt",     'r',  ACSN, 13,                1.0,             0.0, 'u'},
  {"t_s_batt",     'r',  ACSN, 15,                1.0,             0.0, 'u'},
  {"i_p_gond",     'r',  ACSN, 17,                1.0,             0.0, 'u'},
  {"i_s_gond",     'r',  ACSN, 19,                1.0,             0.0, 'u'},
  {"i_p_chrg",     'r',  ACSN, 21,                1.0,             0.0, 'u'},
  {"i_s_chrg",     'r',  ACSN, 23,                1.0,             0.0, 'u'},
  {"i_p_sol",      'r',  ACSN, 25,                1.0,             0.0, 'u'},
  {"i_s_sol",      'r',  ACSN, 27,                1.0,             0.0, 'u'},
  {"v_p5",         'r',  ACSN, 29,                1.0,             0.0, 'u'},
  {"v_s5",         'r',  ACSN, 31,                1.0,             0.0, 'u'},
  {"v_p4",         'r',  ACSN, 33,                1.0,             0.0, 'u'},
  {"v_s4",         'r',  ACSN, 35,                1.0,             0.0, 'u'},
  {"v_p3",         'r',  ACSN, 37,                1.0,             0.0, 'u'},
  {"v_s3",         'r',  ACSN, 39,                1.0,             0.0, 'u'},
  {"v_p2",         'r',  ACSN, 41,                1.0,             0.0, 'u'},
  {"v_s2",         'r',  ACSN, 43,                1.0,             0.0, 'u'},
  {"v_p1",         'r',  ACSN, 45,                1.0,             0.0, 'u'},
  {"v_s1",         'r',  ACSN, 47,                1.0,             0.0, 'u'},

  {"i_charcoal",   'r',  CRYO,  3,     -2.639826420E-6,     0.157988332, 'u'},
  {"i_coldplate",  'r',  CRYO,  5,      -2.32217573E-5,     1.390309833, 'u'},
  {"t_he3fridge",  'r',  CRYO,  7,  -2.87477e-09*65536,      12.3273561, 'u'},
  {"t_he4pot",     'r',  CRYO,  9,  -2.87539e-09*65536,      12.3309023, 'u'},
  {"t_horn_500",   'r',  CRYO, 11,  -2.87784e-09*65536,      12.3420847, 'u'},
  {"t_base_500",   'r',  CRYO, 13,  -2.86354e-09*65536,      12.2829004, 'u'},
  {"t_base_250",   'r',  CRYO, 15,      -1.50793738E-3,     12.49804401, 'u'},
  {"t_opt_box_w",  'r',  CRYO, 17, -2.859373e-09*65536,    1.232225e+01, 'u'},
  {"t_lhe_filt",   'r',  CRYO, 19, -2.856350e-09*65536,    1.231143e+01, 'u'},
  {"t_vcs",        'r',  CRYO, 21, -2.869274e-09*65536,    1.236054e+01, 'u'},
  {"t_vcs_filt",   'r',  CRYO, 23, -2.871969e-09*65536,    1.236866e+01, 'u'},
  {"t_ln2",        'r',  CRYO, 25, -2.871958e-09*65536,    1.236808e+01, 'u'},
  {"t_ln2_filt",   'r',  CRYO, 27, -2.873729e-09*65536,    1.238262e+01, 'u'},
  {"t_charcoal",   'r',  CRYO, 29, -2.865098e-09*65536,    1.235145e+01, 'u'},
  {"t_optics_box", 'r',  CRYO, 31, -2.864185e-09*65536,    1.233900e+01, 'u'},
  {"t_jfet",       'r',  CRYO, 33, -2.860308e-09*65536,    1.232735e+01, 'u'},
  {"t_he4pot_d",   'r',  CRYO, 35, -2.865493e-09*65536,    1.234227e+01, 'u'},
  {"t_cold_plate", 'r',  CRYO, 37, -2.863415e-09*65536,    1.232882e+01, 'u'},
  {"t_opt_box_rox",'r',  CRYO, 39,  -2.87516e-09*65536,      12.3290947, 'u'},
  {"t_xtherm_2",   'r',  CRYO, 41,  -2.87482e-09*65536,      12.4223147, 'u'},
  {"t_bda1",       'r',  CRYO, 43,   -2.8681e-09*65536,      12.3164528, 'u'},
  {"t_bda2",       'r',  CRYO, 45,  -2.87714e-09*65536,      12.3240610, 'u'},
  {"t_bda3",       'r',  CRYO, 47,   -2.8779e-09*65536,      12.3446613, 'u'},
  {"he4_lev",      'r',  CRYO, 51,  -2.87477e-09*65536,      12.3273561, 'u'},
  {"cryoin",       'r',  CRYO, 60,                 1.0,             0.0, 'u'},
  {"status03",     'r',  CRYO, 57,                1.0,             0.0, 'u'},
  {"cryoout2",     'w',  CRYO,  1,                 1.0,             0.0, 'u'},
  {"he3pwm",       'w',  CRYO,  3,          100./2047.,              0., 'u'},
  {"hspwm",        'w',  CRYO,  4,          100./2047.,              0., 'u'},
  {"cryopwm",      'w',  CRYO,  5,          100./2047.,              0., 'u'},
  {"jfetpwm",      'w',  CRYO,  6,          100./2047.,              0., 'u'},

  {"t_rstrt_mid",  'r',  BIAS,  5,            -0.00625,          136.45, 'u'},
  {"t_lstrt_mid",  'r',  BIAS,  7,           -0.00625,          136.45, 'u'},
  {"t_tstrt_pri",  'r',  BIAS,  9,           -0.00625,          136.45, 'u'},
  {"t_tstrt_sec",  'r',  BIAS, 11,           -0.00625,          136.45, 'u'},
  {"t_bstrt_pri",  'r',  BIAS, 13,           -0.00625,          136.45, 'u'},
  {"t_bstrt_sec",  'r',  BIAS, 15,           -0.00625,          136.45, 'u'},
  {"t_secondary",  'r',  BIAS, 17,           -0.00625,          136.45, 'u'},
  {"t_primary_r",  'r',  BIAS, 19,           -0.00625,          136.45, 'u'},
  {"t_primary_c",  'r',  BIAS, 21,           -0.00625,          136.45, 'u'},
  {"i_dpm_28v",    'r',  BIAS, 23,          -0.000625,           20.48, 'u'},
  {"i_dpm_3v",     'r',  BIAS, 25,          -0.000625,           20.48, 'u'},
  {"i_dpm_5v",     'r',  BIAS, 27,          -0.000625,           20.48, 'u'},
  {"i_dpm_10v",    'r',  BIAS, 29,          -0.000625,           20.48, 'u'},
  {"i_rec",        'r',  BIAS, 31,           0.000625,          -20.48, 'u'},
  {"t_baffle",     'r',  BIAS, 33,           -0.00625,          136.45, 'u'},
  {"t_rec",        'r',  BIAS, 35,           -0.00625,          136.45, 'u'},
  {"t_if12",       'r',  BIAS, 37,           -0.00625,          136.45, 'u'},
  {"t_dpm_7.5v",   'r',  BIAS, 41,           -0.00625,          136.45, 'u'},
  {"t_dpm_10v",    'r',  BIAS, 43,           -0.00625,          136.45, 'u'},
  {"t_dpm_5v",     'r',  BIAS, 45,           -0.00625,          136.45, 'u'},
  {"t_dpm_3v",     'r',  BIAS, 47,           -0.00625,          136.45, 'u'},
  {"status04",     'r',  BIAS, 57,                1.0,             0.0, 'u'},
  {"biasout1",     'w',  BIAS,  0,                1.0,             0.0, 'u'},
  {"biasout2",     'w',  BIAS,  1,                1.0,             0.0, 'u'},

  {"status05",     'r',  DAS5, 57,                1.0,             0.0, 'u'},
  {"status06",     'r',  DAS6, 57,                1.0,             0.0, 'u'},
  {"status07",     'r',  DAS7, 57,                1.0,             0.0, 'u'},
  {"status08",     'r',  DAS8, 57,                1.0,             0.0, 'u'},
  {"status09",     'r',  DAS9, 57,                1.0,             0.0, 'u'},
  {"status10",     'r', DAS10, 57,                1.0,             0.0, 'u'},
  {"status11",     'r', DAS11, 57,                1.0,             0.0, 'u'},
  {"status12",     'r', DAS12, 57,                1.0,             0.0, 'u'},
  {"status13",     'r', DAS13, 57,                1.0,             0.0, 'u'},
  {"status14",     'r', DAS14, 57,                1.0,             0.0, 'u'},
  {"status15",     'r', DAS15, 57,                1.0,             0.0, 'u'},
  {"status16",     'r', DAS16, 57,                1.0,             0.0, 'u'},
  {"phase5",       'w',  DAS5, 10,                1.0,             0.0, 'u'},
  {"phase6",       'w',  DAS6, 10,                1.0,             0.0, 'u'},
  {"phase7",       'w',  DAS7, 10,                1.0,             0.0, 'u'},
  {"phase8",       'w',  DAS8, 10,                1.0,             0.0, 'u'},
  {"phase9",       'w',  DAS9, 10,                1.0,             0.0, 'u'},
  {"phase10",      'w', DAS10, 10,                1.0,             0.0, 'u'},
  {"phase11",      'w', DAS11, 10,                1.0,             0.0, 'u'},
  {"phase12",      'w', DAS12, 10,                1.0,             0.0, 'u'},
  {"phase13",      'w', DAS13, 10,                1.0,             0.0, 'u'},
  {"phase14",      'w', DAS14, 10,                1.0,             0.0, 'u'},
  {"phase15",      'w', DAS15, 10,                1.0,             0.0, 'u'},
  {"phase16",      'w', DAS16, 10,                1.0,             0.0, 'u'},

  {"t_gy_set",     'w', LOOP1, 12,    (100.0/32768.0),             0.0, 'u'},
  {"g_p_gyheat",   'w', LOOP1, 13,                1.0,             0.0, 'u'},
  {"g_i_gyheat",   'w', LOOP1, 14,                1.0,             0.0, 'u'},
  {"g_d_gyheat",   'w', LOOP1, 15,                1.0,             0.0, 'u'},
  {"lokmot_pin",   'w', LOOP1, 16,                1.0,             0.0, 'u'},
  {"isc_fpulse",   'w', LOOP1, 17,                10.,             0.0, 'u'},
  {"cal_repeat",   'w', LOOP1, 18,                1.0,             0.0, 'u'},
  {"alice_file",   'w', LOOP1, 19,                1.0,             0.0, 'u'},
  {"timeout",      'w', LOOP1, 20,                1.0,             0.0, 'u'},
  {"sun_az",       'w', LOOP1, 21,              I2DEG,             0.0, 's'},
  {"ss_prin",      'w', LOOP1, 22,                1.0,             0.0, 'u'},
  {"sam_i_am",     'w', LOOP1, 23,                1.0,             0.0, 'u'},
  {"cryostate",    'w', LOOP1, 24,                1.0,             0.0, 'u'},
  {"cpu_fan",      'w', LOOP1, 25,                1.0,             0.0, 'u'},
  {"cpu_temp1",    'w', LOOP1, 26,               0.01,             0.0, 'u'},
  {"mag_model",    'w', LOOP1, 27,              I2DEG,             0.0, 'u'},
  {"sensor_veto",  'w', LOOP1, 28,                1.0,             0.0, 'u'},
  {"bal_on",       'w', LOOP1, 29,           1./1648.,             0.0, 'u'},
  {"bal_off",      'w', LOOP1, 30,           1./1648.,             0.0, 'u'},
  {"bal_target",   'w', LOOP1, 31,           1./1648.,            -5.0, 'u'},
  {"bal_veto",     'w', LOOP1, 32,                1.0,             0.0, 'u'},
  {"isc_framenum", 'w', LOOP1, 33,                1.0,             0.0, 'u'},
  {"bias_lev1",    'w', LOOP1, 34,                1.0,             0.0, 'u'},
  {"bias_lev2",    'w', LOOP1, 35,                1.0,             0.0, 'u'},
  {"bias_lev3",    'w', LOOP1, 36,                1.0,             0.0, 'u'},
  {"sip_alt",      'w', LOOP1, 37,                4.0,             0.0, 'u'},
  {"lat",          'w', LOOP1, 38,              I2DEG,             0.0, 'u'},
  {"lon",          'w', LOOP1, 39,              I2DEG,             0.0, 'u'},
  {"p_h",          'w', LOOP1, 40,              I2DEG,             0.0, 'u'},
  {"isc_error",    'w', LOOP1, 41,                 1.,             0.0, 'u'},
  {"isc_mapmean",  'w', LOOP1, 42,                 1.,             0.0, 'u'},
  {"dgps_pitch",   'w', LOOP1, 43,              I2DEG,             0.0, 'u'},
  {"dgps_roll",    'w', LOOP1, 44,              I2DEG,             0.0, 'u'},
  {"sip_lat",      'w', LOOP1, 45,              I2DEG,             0.0, 'u'},
  {"sip_lon",      'w', LOOP1, 46,              I2DEG,             0.0, 'u'},
  {"dgps_lat",     'w', LOOP1, 47,              I2DEG,             0.0, 'u'},
  {"dgps_lon",     'w', LOOP1, 48,              I2DEG,             0.0, 'u'},
  {"dgps_alt",     'w', LOOP1, 49,                1.0,             0.0, 'u'},
  {"dgps_speed",   'w', LOOP1, 50,              I2DEG,             0.0, 'u'},
  {"dgps_dir",     'w', LOOP1, 51,              I2DEG,             0.0, 'u'},
  {"dgps_climb",   'w', LOOP1, 52,              I2DEG,             0.0, 's'},
  {"dgps_att_ok",  'w', LOOP1, 53,                1.0,             0.0, 'u'},
  {"dgps_att_index",'w',LOOP1, 54,                1.0,             0.0, 'u'},
  {"dgps_pos_index",'w',LOOP1, 55,                1.0,             0.0, 'u'},
  {"sync",         'w', LOOP1, 56,                1.0,             0.0, 'u'},
  {"dgps_n_sat",   'w', LOOP1, 57,                1.0,             0.0, 'u'},
  {"disk_free",    'w', LOOP1, 58,             1./250,             0.0, 'u'},
  {"p_mode",       'w', LOOP1, 59,                  1,             0.0, 'u'},
  {"p_x_deg",      'w', LOOP1, 60,              I2DEG,             0.0, 'u'},
  {"p_y",          'w', LOOP1, 61,              I2DEG,             0.0, 's'},
  {"p_vaz",        'w', LOOP1, 62,              I2VEL,             0.0, 'u'},
  {"p_del",        'w', LOOP1, 63,              I2VEL,             0.0, 'u'},

  {"isc_rd_sigma", 'w', LOOP2,  0,              I2DEG,             0.0, 'u'},
  {"bal_gain",     'w', LOOP2,  1,            1/1000.,             0.0, 'u'},
  {"blob0_sn",     'w', LOOP2,  2,       1000./65536.,             0.0, 'u'},
  {"blob1_sn",     'w', LOOP2,  3,       1000./65536.,             0.0, 'u'},
  {"blob2_sn",     'w', LOOP2,  4,       1000./65536.,             0.0, 'u'},
  {"blob0_x",      'w', LOOP2,  5,             1./40.,             0.0, 'u'},
  {"blob1_x",      'w', LOOP2,  6,             1./40.,             0.0, 'u'},
  {"blob2_x",      'w', LOOP2,  7,             1./40.,             0.0, 'u'},
  {"p_w",          'w', LOOP2,  8,              I2DEG,             0.0, 'u'},
  {"isc_rtol",     'w', LOOP2,  9,              I2DEG,             0.0, 'u'},
  {"blob0_y",      'w', LOOP2, 10,             1./40.,             0.0, 'u'},
  {"blob1_y",      'w', LOOP2, 11,             1./40.,             0.0, 'u'},
  {"blob2_y",      'w', LOOP2, 12,             1./40.,             0.0, 'u'},
  {"isc_apert",    'w', LOOP2, 13,                1.0,             0.0, 'u'},
  {"isc_maglimit", 'w', LOOP2, 14,           1./1000.,             0.0, 'u'},
  {"isc_nrad",     'w', LOOP2, 15,              I2DEG,             0.0, 'u'},
  {"isc_mtol",     'w', LOOP2, 16,        100./65536.,             0.0, 'u'},
  {"isc_qtol",     'w', LOOP2, 17,        100./65536.,             0.0, 'u'},
  {"blob2_flux",   'w', LOOP2, 18,               32.0,             0.0, 'u'},
  {"isc_focus",    'w', LOOP2, 19,                1.0,             0.0, 'u'},
  {"isc_state",    'w', LOOP2, 20,                1.0,             0.0, 'u'},
  {"isc_lrad",     'w', LOOP2, 21,              I2DEG,             0.0, 'u'},
  {"isc_thresh",   'w', LOOP2, 22,             1./10.,             0.0, 'u'},
  {"isc_grid",     'w', LOOP2, 23,                1.0,             0.0, 'u'},
  {"isc_cenbox",   'w', LOOP2, 24,                1.0,             0.0, 'u'},
  {"isc_apbox",    'w', LOOP2, 25,                1.0,             0.0, 'u'},
  {"isc_mdist",    'w', LOOP2, 26,                1.0,             0.0, 'u'},
  {"isc_nblobs",   'w', LOOP2, 27,                1.0,             0.0, 'u'},
  {"blob0_flux",   'w', LOOP2, 53,               32.0,             0.0, 'u'},
  {"blob1_flux",   'w', LOOP2, 30,               32.0,             0.0, 'u'},
  {"bal_min",      'w', LOOP2, 31,        -100./2047.,            100., 'u'},
  {"bal_max",      'w', LOOP2, 32,        -100./2047.,            100., 'u'},
  {"isc_tol",      'w', LOOP2, 33,                1.0,             0.0, 'u'},
  {"ss_az",        'w', LOOP2, 35,              I2DEG,             0.0, 's'},
  {"gy1_offset",   'w', LOOP2, 36,        1.0/32768.0,             0.0, 's'},
  {"gy2_offset",   'w', LOOP2, 37,        1.0/32768.0,             0.0, 's'},
  {"gy3_offset",   'w', LOOP2, 38,        1.0/32768.0,             0.0, 's'},
  {"gy_roll_amp",  'w', LOOP2, 39,          1./65536.,             0.0, 'u'},
  {"mag_sigma",    'w', LOOP2, 40,              I2DEG,             0.0, 'u'},
  {"dgps_az",      'w', LOOP2, 41,              I2DEG,             0.0, 'u'},
  {"dgps_sigma",   'w', LOOP2, 42,              I2DEG,             0.0, 'u'},
  {"ss_sigma",     'w', LOOP2, 43,              I2DEG,             0.0, 'u'},
  {"isc_az",       'w', LOOP2, 44,              I2DEG,             0.0, 'u'},
  {"isc_el",       'w', LOOP2, 45,              I2DEG,             0.0, 'u'},
  {"isc_sigma",    'w', LOOP2, 46,              I2DEG,             0.0, 'u'},
  {"enc_el",       'w', LOOP2, 47,              I2DEG,             0.0, 'u'},
  {"enc_sigma",    'w', LOOP2, 48,              I2DEG,             0.0, 'u'},
  {"clin_el",      'w', LOOP2, 54,              I2DEG,             0.0, 'u'},
  {"clin_sigma",   'w', LOOP2, 55,              I2DEG,             0.0, 'u'},
  {"mag_az",       'w', LOOP2, 56,              I2DEG,             0.0, 'u'},
  {"isc_spulse",   'w', LOOP2, 57,               10.0,             0.0, 'u'},
  {"isc_afocus",   'w', LOOP2, 58,                1.0,             0.0, 'u'},
  {"isc_brra",     'w', LOOP2, 59,              I2DEG,             0.0, 'u'},
  {"isc_brdec",    'w', LOOP2, 60,              I2DEG,             0.0, 'u'},
  {"isc_mcpnum",   'w', LOOP2, 61,                1.0,             0.0, 'u'},
  {"isc_maxblobs", 'w', LOOP2, 62,                1.0,             0.0, 'u'},
  {"isc_x_off",    'w', LOOP2, 63,              I2DEG,             0.0, 'u'},

  {"pulse_len",    'w', LOOP3,  0,                10.,             0.0, 'u'},
  {"isc_hold_i",   'w', LOOP3,  1,                1.0,             0.0, 'u'},
  {"isc_save_prd", 'w', LOOP3,  2,               0.01,             0.0, 'u'},
  {"isc_y_off",    'w', LOOP3,  3,              I2DEG,             0.0, 'u'},
  {"ra",           'w', LOOP3,  4,       24.0/65536.0,             0.0, 'u'},
  {"dec",          'w', LOOP3,  5,              I2DEG,             0.0, 's'},
  {"cal_pulse",    'w', LOOP3,  6,                1.0,             0.0, 'u'},
  {"l_override",   'w', LOOP3,  7,                1.0,             0.0, 'u'},
  {"cpu_temp2",    'w', LOOP3,  8,               0.01,             0.0, 'u'},
  {"cpu_temp3",    'w', LOOP3,  9,               0.01,             0.0, 'u'},
  {"sip_mks_hi",   'w', LOOP3, 10,                4.0,             0.0, 'u'},
  {"sip_mks_med",  'w', LOOP3, 11,                4.0,             0.0, 'u'},
  {"sip_mks_lo",   'w', LOOP3, 12,                4.0,             0.0, 'u'},
  END_OF_CHANNELS
}; 

struct ChannelStruct WideFastChannels[] = {
#ifndef BOLOTEST
  {"t_gybox2",    'r',  ACS1, 12, -9.5367431641e-08,               136.45, 'U'},
  {"t_gybox",     'r',  ACS1, 14, -9.5367431641e-08,               136.45, 'U'},
#endif

  /* BIAS Amplitude */
  {"b_amp2",      'r',  BIAS,  0,             1.0,                    0.0, 'U'},
  {"b_amp1",      'r',  BIAS,  2,             1.0,                    0.0, 'U'},
  {"b_amp3",      'r',  BIAS, 38,             1.0,                    0.0, 'U'},
  {"n3ref",       'r',  CRYO, 48,      0.00390625,             -8388608.0, 'U'},
  {"n5ref",       'r',  DAS5, 36,      1.19209e-7,                    0.0, 'U'},
  {"n6ref",       'r',  DAS6, 36,      1.19209e-7,                    0.0, 'U'},
  {"n7ref",       'r',  DAS7, 36,      1.19209e-7,                    0.0, 'U'},
  {"n8ref",       'r',  DAS8, 36,      1.19209e-7,                    0.0, 'U'},
  {"n9ref",       'r',  DAS9, 36,      1.19209e-7,                    0.0, 'U'},
  {"n10ref",      'r', DAS10, 36,      1.19209e-7,                    0.0, 'U'},
  {"n11ref",      'r', DAS11, 36,      1.19209e-7,                    0.0, 'U'},
  {"n12ref",      'r', DAS12, 36,      1.19209e-7,                    0.0, 'U'},
  {"n13ref",      'r', DAS13, 36,      1.19209e-7,                    0.0, 'U'},
  {"n14ref",      'r', DAS14, 36,      1.19209e-7,                    0.0, 'U'},
  {"n15ref",      'r', DAS15, 36,      1.23301e-7,                    0.0, 'U'},
  {"n16ref",      'r', DAS16, 36,      1.19209e-7,                    0.0, 'U'},
  END_OF_CHANNELS
};

struct ChannelStruct FastChannels[] = {
#ifndef BOLOTEST
  /* read channels from ACS0 */
  {"pch_clin_sip",'r', ACS0, 27,   4.0/5333.3333,              -4.*6.144, 'u'},
  {"roll_clin_sip",'r',ACS0, 29,  -4.0/5333.3333,               4.*6.144, 'u'},

  {"clin_elev",   'r',  ACS0, 37,      0.00546739,                -133.78, 'u'},
  {"xel_clin_if", 'r',  ACS0, 39,      0.00546739,             -25.*6.144, 'u'},

  {"mag_bias",    'r',  ACS0, 43,             1.0,                    0.0, 'u'},
  {"mag_x",       'r',  ACS0, 45,             1.0,                    0.0, 'u'},
  {"mag_y",       'r',  ACS0, 47,             1.0,                    0.0, 'u'},
  {"acs0bits",    'r',  ACS0, 59,             1.0,                    0.0, 'u'},

  {"az",          'w', LOOP2, 51,       I2DEG,                    0.0, 'u'},
  {"el",          'w', LOOP2, 52,       I2DEG,                    0.0, 'u'},

  {"mcp_frame",   'w', LOOP2, 34,         1.0,                    0.0, 'u'},
  {"ss_x_ccd",    'w', LOOP2, 50,         1.0,                    0.0, 's'},

  /* send data to ACS0 */
  {"isc_bits",    'w',  ACS0,  1,             1.0,                    0.0, 'u'},

  /* read channels from ACS1 */
  {"gyro5",       'r',  ACS1,  2,  ADU2_TO_DPS, -GYRO2_OFFSET*ADU2_TO_DPS, 'u'},
  {"raw_gy5",     'r',  ACS1,  3,  ADU2_TO_DPS, -GYRO2_OFFSET*ADU2_TO_DPS, 'u'},
  {"gyro6",       'r',  ACS1,  4,  ADU3_TO_DPS, -GYRO3_OFFSET*ADU3_TO_DPS, 'u'},
  {"raw_gy6",     'r',  ACS1,  5,  ADU3_TO_DPS, -GYRO3_OFFSET*ADU3_TO_DPS, 'u'},
  {"gyro4",       'r',  ACS1,  6,  ADU1_TO_DPS, -GYRO1_OFFSET*ADU1_TO_DPS, 'u'},
  {"raw_gy4",     'r',  ACS1,  7,  ADU1_TO_DPS, -GYRO1_OFFSET*ADU1_TO_DPS, 'u'},
  {"gyro2",       'r',  ACS1, 22,  ADU2_TO_DPS, -GYRO2_OFFSET*ADU2_TO_DPS, 'u'},
  {"raw_gy2",     'r',  ACS1, 23,  ADU2_TO_DPS, -GYRO2_OFFSET*ADU2_TO_DPS, 'u'},
  {"gyro3",       'r',  ACS1, 24,  ADU3_TO_DPS, -GYRO3_OFFSET*ADU3_TO_DPS, 'u'},
  {"raw_gy3",     'r',  ACS1, 25,  ADU3_TO_DPS, -GYRO3_OFFSET*ADU3_TO_DPS, 'u'},
  {"gyro1",       'r',  ACS1, 26,  ADU1_TO_DPS, -GYRO1_OFFSET*ADU1_TO_DPS, 'u'},
  {"raw_gy1",     'r',  ACS1, 27,  ADU1_TO_DPS, -GYRO1_OFFSET*ADU1_TO_DPS, 'u'},

  {"piv_enc",     'r',  ACS1, 59,    360.0/8192.0,                    0.0, 'u'},
  {"reac_enc",    'r',  ACS1, 60,    360.0/4000.0,                    0.0, 'u'},
  {"pwm_el",      'r',  ACS1, 51,             1.0,                -4000.0, 'u'},
  {"pwm_roll",    'r',  ACS1, 52,             1.0,                -4000.0, 'u'},
  //{"r_dt", 'r',  ACS1, 53,  1.0,     -4000.0, 'u'}, // see acs1.c

  {"pwm_reac",    'r',  ACS1, 54,             1.0,                -4000.0, 'u'},
  {"rps_reac",    'r',  ACS1, 55,7.9498291016e-05,                 -2.605, 'u'},
  {"pwm_piv",     'r',  ACS1, 61,             1.0,                -4000.0, 'u'},

  /* send data to ACS1 */
  {"gy_heat",     'w',  ACS1,  1,             1.0,                    0.0, 'u'},
  {"el_vreq",     'w',  ACS1,  4,             1.0,                 -32768, 'u'},
  {"az_vreq",     'w',  ACS1, 14,             1.0,                 -32768, 'u'},
  {"cos_el",      'w',  ACS1,  9,             1.0,                    0.0, 'u'},
  {"sin_el",      'w',  ACS1, 10,             1.0,                    0.0, 'u'},

  /* read from board ACS2 */
  {"enc_elev",    'r',  ACS2, 50,  -360.0/65536.0,        ENC_ELEV_OFFSET, 'u'},
#endif

  /* Write to DAS3 -- cryo controller */
  {"cryoout3",    'w',  CRYO,  2,             1.0,                    0.0, 'u'},

  /* Read from DAS4 -- bias controller and DPM/inner frame monitoring */
  {"biasin",      'r',  BIAS, 50,             1.0,                    0.0, 'u'},

  // Read Requests: for signalling the das to copy data to the BB memory
  // Used for low/high word read synchronisation
  /* !!! WARNING!  MCP assumes these appear sequentially in the frame !!! */
  {"readd3",      'w',  CRYO, 40,             1.0,                    0.0, 'u'},
  {"readd4",      'w',  BIAS, 40,             1.0,                    0.0, 'u'},
  {"readd5",      'w',  DAS5, 40,             1.0,                    0.0, 'u'},
  {"readd6",      'w',  DAS6, 40,             1.0,                    0.0, 'u'},
  {"readd7",      'w',  DAS7, 40,             1.0,                    0.0, 'u'},
  {"readd8",      'w',  DAS8, 40,             1.0,                    0.0, 'u'},
  {"readd9",      'w',  DAS9, 40,             1.0,                    0.0, 'u'},
  {"readd10",     'w', DAS10, 40,             1.0,                    0.0, 'u'},
  {"readd11",     'w', DAS11, 40,             1.0,                    0.0, 'u'},
  {"readd12",     'w', DAS12, 40,             1.0,                    0.0, 'u'},
  {"readd13",     'w', DAS13, 40,             1.0,                    0.0, 'u'},
  {"readd14",     'w', DAS14, 40,             1.0,                    0.0, 'u'},
  {"readd15",     'w', DAS15, 40,             1.0,                    0.0, 'u'},
  {"readd16",     'w', DAS16, 40,             1.0,                    0.0, 'u'},
  END_OF_CHANNELS
};

/************************************************************************/
/*                                                                      */
/*    MakeBoloTable: create the bolometer channel table                 */
/*                                                                      */
/************************************************************************/
void MakeBoloTable(void) {
  int i, j, index = 0;;
  struct ChannelStruct channel = {
    "", 'r', 3, BOLO_BUS, 0, 1.19209e-7, -2097152.0, 'u'
  };

  mprintf(MCP_INFO, "Generating Bolometer Channel Table.\n");

  for (i = 0; i < DAS_CARDS; ++i) {
    channel.node = i + 5;
    channel.rw = 'r';
    for (j = 0; j < DAS_CHS; j += 2) {
      /* lsw channel at j */
      channel.addr = j;
      sprintf(channel.field, "n%ic%ilo", channel.node, j); /* ignored */
      boloIndex[i][j][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* msw at j and j+1 */
      channel.addr = DAS_CHS + (j >> 1);
      sprintf(channel.field, "n%ic%ihi", channel.node, j); /* ignored */
      boloIndex[i][j + 1][1] = boloIndex[i][j][1] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* lsw channel at j+1 */
      channel.addr = j+1;
      sprintf(channel.field, "n%ic%ilo", channel.node, j+1); /* ignored */
      boloIndex[i][j + 1][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
    }
  }
}

struct NiosStruct SetNiosData(const struct ChannelStruct *channel, int addr,
    int fast, int wide)
{
  struct NiosStruct NiosData;
  NiosData.field = channel->field;
  NiosData.fast = fast;
  NiosData.wide = wide;
  NiosData.bus = channel->bus;
  NiosData.bbcAddr = (channel->rw =='r' ? BBC_READ : BBC_WRITE)
    | BBC_NODE(channel->node) | BBC_CH(channel->addr);
  NiosData.niosAddr = channel->bus ? BBCPCI_WFRAME2_ADD(addr)
    : BBCPCI_WFRAME1_ADD(addr);

  return NiosData;
}

/* DumpNiosFrame - writes the constructed nios frame to the map file */
void DumpNiosFrame(void)
{
  int bus, m, i, j, n, addr, m0addr;
  FILE* map;

  if ((map = fopen("/data/etc/Nios.map", "w")) == NULL)
    return;

  for (bus = 0; bus < 2; ++bus) {
    fprintf(map, "Bus %i Map:\n", bus);
    for (m = 0; m < FAST_PER_SLOW; ++m) {
      for (i = 0; i < TxFrameWords[bus]; ++i) {
        addr = i + bus * BBCPCI_MAX_FRAME_SIZE + m * TxFrameWords[bus];
        m0addr = i + bus * BBCPCI_MAX_FRAME_SIZE;
        n = 0;
        fprintf(map, "%06x", addr);
        if (i == 0) {
          if (bus) {
            n++;
            addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(4);
            fprintf(map, " %08x [%2i %3i (%04x)] Y Frame Sync", addr,
                BiPhaseLookup[BI0_MAGIC(addr)].index,
                BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
          } else {
            n++;
            addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(0);
            fprintf(map, " %08x [%2i %3i (%04x)] Y Frame Sync", addr,
                BiPhaseLookup[BI0_MAGIC(addr)].index,
                BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
          }
        } else if (i == 1 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(1);
          fprintf(map, " %08x [%2i %3i (%04x)] F FASTSAMP (lsb)", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
        } else if (i == 2 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(2);
          fprintf(map, " %08x [%2i %3i (%04x)] F FASTSAMP (msb)", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
        } else if (i == 3 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(3);
          fprintf(map, " %08x [%2i %3i (%04x)] M Multiplex Index = %i", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr), m);
        } else {
          for (j = 0; j < ccTotal; ++j)
            if (NiosLookup[j].niosAddr == addr) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s",
                  NiosLookup[j].bbcAddr,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].index,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].channel,
                  BI0_MAGIC(NiosLookup[j].bbcAddr),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
              if (NiosLookup[j].wide)
                fprintf(map, " (lsb)");
            } else if (NiosLookup[j].niosAddr == addr - 1 &&
                NiosLookup[j].wide) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s (msb)",
                  BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].index,
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].channel,
                  BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr)),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
            } else if (NiosLookup[j].fast && NiosLookup[j].niosAddr == m0addr) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s",
                  NiosLookup[j].bbcAddr,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].index,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].channel,
                  BI0_MAGIC(NiosLookup[j].bbcAddr),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
              if (NiosLookup[j].wide)
                fprintf(map, " (lsb)");
            } else if (NiosLookup[j].fast && NiosLookup[j].niosAddr
                == m0addr - 1 && NiosLookup[j].wide) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s (msb)",
                  BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].index,
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].channel,
                  BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr)),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
            }
        }
        for (j = 0; j < 2 * FAST_PER_SLOW; ++j) {
          if (NiosSpares[j] == addr) {
            n++;
            fprintf(map, " %08x [%2i %3i (%04x)] X Spare%i", BBCSpares[j],
                BiPhaseLookup[BI0_MAGIC(BBCSpares[j])].index,
                BiPhaseLookup[BI0_MAGIC(BBCSpares[j])].channel,
                BI0_MAGIC(BBCSpares[j]), j);
          }
        }
        fprintf(map, "\n");
        if (n == 0) {
          fclose(map);
          mprintf(MCP_FATAL, "FATAL: unassigned address in Nios Address Table."
              " Consult Nios Map.\n");
        } else if (n != 1) {
          fclose(map);
          mprintf(MCP_FATAL, "FATAL: collision in Nios Address Table "
              "assignment. Consult Nios Map.\n");
        }
      }
    }
    fprintf(map, "\n");
  }

  mprintf(MCP_INFO, "Wrote /data/etc/Nios.map.\n");
  fclose(map);
}

/* Checks BBC Addresses to see if multiple fields are occupying the same
 * place */
void BBCAddressCheck(char* fields[64][64], char* name, int node, int addr)
{
  if (fields[node][addr])
    mprintf(MCP_FATAL, "FATAL: Conflicting BBC address found for %s and %s"
        " (node %i channel %i)\n", fields[node][addr], name, node, addr);

  fields[node][addr] = name;
}

/* DoSanityChecks - run various sanity checks on the channel tables.  Also
 * compute useful parameters */
void DoSanityChecks(void)
{
  int i, j;
  char* fields[2][64][64];

  mprintf(MCP_INFO, "Running Sanity Checks on Channel Lists.\n");

  for (i = 0; i < 64; ++i)
    for (j = 0; j < 64; ++j)
      fields[0][i][j] = fields[1][i][j] = NULL;

  for (i = 0; WideSlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[WideSlowChannels[i].bus] += 2;
    if (WideSlowChannels[i].type != 'U' && WideSlowChannels[i].type != 'S'
        && WideSlowChannels[i].type != 'i')
      mprintf(MCP_FATAL, "FATAL: Error in Wide Slow Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideSlowChannels[i].field, WideSlowChannels[i].type);

    BBCAddressCheck(fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr);
    BBCAddressCheck(fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr + 1);
  }
  ccWideSlow = i;

  for (i = 0; SlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[SlowChannels[i].bus]++;
    if (SlowChannels[i].type != 'u' && SlowChannels[i].type != 's')
      mprintf(MCP_FATAL, "Error in Slow Channel List:\n"
          "    %s does not have a valid slow type (%c)\n",
          SlowChannels[i].field, SlowChannels[i].type);

    BBCAddressCheck(fields[SlowChannels[i].rw == 'r'], SlowChannels[i].field,
        SlowChannels[i].node, SlowChannels[i].addr);
  }
  ccNarrowSlow = i;
  ccSlow = ccNarrowSlow + ccWideSlow;

  for (i = 0; WideFastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[WideFastChannels[i].bus] += 2;
    if (WideFastChannels[i].type != 'U' && WideFastChannels[i].type != 'S'
        && WideFastChannels[i].type != 'i')
      mprintf(MCP_FATAL, "FATAL: Error in Wide Fast Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideFastChannels[i].field, WideFastChannels[i].type);

    BBCAddressCheck(fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr);
    BBCAddressCheck(fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr + 1);
  }
  ccWideFast = i;

  for (i = 0; FastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[FastChannels[i].bus]++;
    if (FastChannels[i].type != 'u' && FastChannels[i].type != 's')
      mprintf(MCP_FATAL, "Error in Fast Channel List:\n"
          "    %s does not have a valid slow type (%c)\n",
          FastChannels[i].field, FastChannels[i].type);

    BBCAddressCheck(fields[FastChannels[i].rw == 'r'], FastChannels[i].field,
        FastChannels[i].node, FastChannels[i].addr);
  }
  ccNarrowFast = i;
  ccFast = ccWideFast + ccNarrowFast + N_FAST_BOLOS;
  ccNoBolos = ccSlow + ccWideFast + ccNarrowFast;
  ccTotal = ccFast + ccSlow;

  for (i = 0; i < N_FAST_BOLOS; ++i) {
    fastsPerBusFrame[BOLO_BUS]++;
    BBCAddressCheck(fields[1], FastChannels[i].field, BoloChannels[i].node,
        BoloChannels[i].addr);
  }

  /* Calculate slowsPerBi0Frame */
  for (i = 0; i < 2; ++ i) {
    slowsPerBusFrame[i] = 1 + (slowCount[i] - 1) / FAST_PER_SLOW;
    TxFrameWords[i] = fastsPerBusFrame[i] + slowsPerBusFrame[i];
    TxFrameSize[i] = TxFrameWords[i] * 4;
  }

  slowsPerBi0Frame = slowsPerBusFrame[0] + slowsPerBusFrame[1];


  mprintf(MCP_INFO, "All Checks Passed.\n");
  mprintf(MCP_INFO, "Slow Channels Per Biphase Frame: %i\n", slowsPerBi0Frame);
  mprintf(MCP_INFO, "Slow Channels Per Tx Frame: %i / %i\n",
      slowsPerBusFrame[0],  slowsPerBusFrame[1]);
  mprintf(MCP_INFO, "Fast Channels Per Biphase Frame: %i\n", ccFast
      + FAST_OFFSET);
  mprintf(MCP_INFO, "Fast Channels Per Tx Frame: %i / %i\n",
      fastsPerBusFrame[0],  fastsPerBusFrame[1]);

  BiPhaseFrameWords = FAST_OFFSET + ccFast + slowsPerBi0Frame + 1;
  BiPhaseFrameSize = 2 * BiPhaseFrameWords;
  for (i = 0; i < 2; ++i) {
    mprintf(MCP_INFO,
        "BBC Bus %i: Frame Bytes: %4i  Allowed: %4i (%.2f%% full)\n", i,
        4 * TxFrameWords[i], 4 * BBC_FRAME_SIZE,
        100. * TxFrameWords[i] / BBC_FRAME_SIZE);
    if (TxFrameWords[i] > BBC_FRAME_SIZE)
      mprintf(MCP_FATAL, "FATAL: BBC Bus %i frame too big.\n", i);
  }

  mprintf(MCP_INFO,
      " BiPhase : Frame Bytes: %4i  Allowed: %4i (%.2f%% full)\n",
      2 * BiPhaseFrameWords, 2 * BI0_FRAME_SIZE,
      100. * BiPhaseFrameWords / BI0_FRAME_SIZE);
  if (BiPhaseFrameWords > BI0_FRAME_SIZE)
    mprintf(MCP_FATAL, "FATAL: Biphase frame too big.\n");
}

/* MakeAddressLookups - fills the nios and biphase address lookup tables */
void MakeAddressLookups(void)
{
  int i, mplex, bus, spare_count;
  int slowIndex[2][FAST_PER_SLOW];

  MakeBoloTable();

  DoSanityChecks();

  mprintf(MCP_INFO, "Generating Address Lookup Tables\n");

  unsigned int BiPhaseAddr = FAST_OFFSET + slowsPerBi0Frame;
  unsigned int addr[2] = {
    FAST_OFFSET + slowsPerBusFrame[0],
    1 + slowsPerBusFrame[1]
  };

  int slowTop[2] = {
    FAST_OFFSET + slowsPerBusFrame[0],
    1 + slowsPerBusFrame[1]
  };

  /* allocate the Nios address table */
  if ((NiosLookup = malloc(ccTotal * sizeof(struct NiosStruct)))
      == NULL)
    mprintf(MCP_TFATAL, "Unable to malloc Nios Address Lookup Table.\n");

  /* allocate the BiPhase address table */
  if ((BiPhaseLookup = malloc(BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct)))
      == NULL)
    mprintf(MCP_TFATAL, "Unable to malloc Biphase Address Lookup Table.\n");

  /* fill BiPhase Lookup with invalid data */
  memset(BiPhaseLookup, 0xff, BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct));

  /* initialise slow channels */
  for (i = 0; i < FAST_PER_SLOW; ++i) {
    slowIndex[0][i] = FAST_OFFSET;
    slowIndex[1][i] = 1;
  }
  for (i = 0; i < FAST_PER_SLOW * 2; ++i)
    NiosSpares[i] = -1;

  /* populate slow channels */
  for (i = 0; i < ccWideSlow; ++i) {
    bus = WideSlowChannels[i].bus;
    mplex = 0;
    while (slowIndex[bus][mplex] + 1 >= slowTop[bus]) {
      if (++mplex >= FAST_PER_SLOW)
        mprintf(MCP_FATAL, "FATAL: Ran out of subframes while trying to "
            "insert wide slow channel %s\n", WideSlowChannels[i].field);
    }

    NiosLookup[i] = SetNiosData(&WideSlowChannels[i], mplex * TxFrameWords[bus]
        + slowIndex[bus][mplex], 0, 1);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i].bbcAddr)].index = mplex;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i].bbcAddr)].channel
      = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0] + FAST_OFFSET - 1);
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i].bbcAddr))].index
      = mplex;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i].bbcAddr))].channel
      = slowIndex[bus][mplex] + 1 + bus * (slowsPerBusFrame[0] + FAST_OFFSET
          - 1);

    slowIndex[bus][mplex] += 2;
  }

  for (i = 0; i < ccNarrowSlow; ++i) {
    bus = SlowChannels[i].bus;
    mplex = 0;
    while (slowIndex[bus][mplex] >= slowTop[bus]) {
      if (++mplex >= FAST_PER_SLOW) {
        mprintf(MCP_FATAL, "FATAL: Ran out of subframes while trying to "
            "insert slow channel %s\n", SlowChannels[i].field);
      }
    }

    NiosLookup[i + ccWideSlow] = SetNiosData(&SlowChannels[i],
        mplex * TxFrameWords[bus] + slowIndex[bus][mplex], 0, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccWideSlow].bbcAddr)].index = mplex;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccWideSlow].bbcAddr)].channel
      = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0] + FAST_OFFSET - 1);

    slowIndex[bus][mplex]++;
  }

  /* Fill Up remaining Slow Channels with spares */
  spare_count = 0;
  for (bus = 0; bus < 2; ++bus)
    for (mplex = 0; mplex < FAST_PER_SLOW; ++mplex)
      while (slowIndex[bus][mplex] < slowTop[bus]) {
        BBCSpares[spare_count] = BBC_WRITE | BBC_NODE(SPARE) |
          BBC_CH(spare_count);
        BiPhaseLookup[BI0_MAGIC(BBCSpares[spare_count])].index = mplex;
        BiPhaseLookup[BI0_MAGIC(BBCSpares[spare_count])].channel
          = slowIndex[bus][mplex] + bus * slowsPerBusFrame[0];
        i = mplex * TxFrameWords[bus] + slowIndex[bus][mplex];
        NiosSpares[spare_count++] = bus ? BBCPCI_WFRAME2_ADD(i)
          : BBCPCI_WFRAME1_ADD(i);
        slowIndex[bus][mplex]++;
      }

  mprintf(MCP_INFO, "Added %i spare slow channels.\n", spare_count);

  for (i = 0; i < ccNarrowFast; ++i) {
    NiosLookup[i + ccSlow + ccWideFast] = SetNiosData(&FastChannels[i],
        addr[FastChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow + ccWideFast].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow
        + ccWideFast].bbcAddr)].channel = BiPhaseAddr++;

    addr[FastChannels[i].bus]++;
  }

  for (i = 0; i < ccWideFast; ++i) {
    NiosLookup[i + ccSlow] = SetNiosData(&WideFastChannels[i],
        addr[WideFastChannels[i].bus], 1, 1);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow].bbcAddr)].channel
      = BiPhaseAddr++;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i
          + ccSlow].bbcAddr))].index = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i
          + ccSlow].bbcAddr))].channel = BiPhaseAddr++;

    addr[WideFastChannels[i].bus] += 2;
  }

  for (i = 0; i < N_FAST_BOLOS; ++i) {
    NiosLookup[i + ccNoBolos] = SetNiosData(&BoloChannels[i],
        addr[BoloChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos].bbcAddr)].channel
      = addr[BoloChannels[i].bus];

    addr[BoloChannels[i].bus]++;
  }

  /* Add the channels that aren't in the channel list to the Biphase lookup */
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(0)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(0)) >> 16].channel = 0;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(1)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(1)) >> 16].channel = 1;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(2)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(2)) >> 16].channel = 2;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(3)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(3)) >> 16].channel = 3;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(4)) >> 16].index = DISCARD_WORD;

  DumpNiosFrame();
}

inline struct BiPhaseStruct* ExtractBiPhaseAddr(struct NiosStruct* niosAddr)
{
  return &BiPhaseLookup[BI0_MAGIC(niosAddr->bbcAddr)];
}

inline struct BiPhaseStruct* GetBiPhaseAddr(const char* field)
{
  return &BiPhaseLookup[BI0_MAGIC((GetNiosAddr(field))->bbcAddr)];
}

/************************************************************************/
/*                                                                      */
/*    GetNiosAddr                                                       */
/*                                                                      */
/************************************************************************/
struct NiosStruct* GetNiosAddr(const char* field) {
  int i;

  for (i = 0; i < ccTotal; ++i)
    if (strcmp(NiosLookup[i].field, field) == 0)
      return &NiosLookup[i];

  mprintf(MCP_FATAL, "Nios Lookup for channel %s failed.\n", field);

  return NULL;
}

void FPrintDerived(FILE *fp) {
  fprintf(fp,
      "P_X_H         LINCOM 1 p_x_deg 0.0003662109375 0\n"
      "### Sensor Veto ###\n"
      "SUN_VETO         BIT sensor_veto 0\n"
      "ISC_VETO         BIT sensor_veto 1\n"
      "ELENC_VETO       BIT sensor_veto 2\n"
      "MAG_VETO         BIT sensor_veto 3\n"
      "GPS_VETO         BIT sensor_veto 4\n"
      "ELCLIN_VETO      BIT sensor_veto 5\n"
      "IS_SCHED         BIT sensor_veto 6\n"
      "### ISC State Field ###\n"
      "ISC_SAVE_IMAGES  BIT isc_state 0\n"
      "ISC_PAUSE        BIT isc_state 1\n"
      "ISC_ABORT        BIT isc_state 2\n"
      "ISC_AUTOFOCUS    BIT isc_state 3\n"
      "ISC_BRIGHT_STAR  BIT isc_state 4\n"
      "ISC_SHUTDOWN     BIT isc_state 5\n"
      "ISC_PULSE        BIT isc_bits  1\n"
      "### Bias Generator Bitfield ###\n"
      "BIAS_IS_DC       BIT biasin 1\n"
      "BIAS_CLK_IS_INT  BIT biasin 2\n"
      "BIAS_IS_INT      BIT biasin 3\n"
    "### Cryo State Bitfield ###\n"
    "HE_LEV_SENS      BIT cryostate 0\n"
    "CHARC_HEATER     BIT cryostate 1\n"
    "COLDP_HEATER     BIT cryostate 2\n"
    "CALIBRATOR       BIT cryostate 3\n"
    "POT_VALVE        BIT cryostate 4\n"
    "POT_DIREC        BIT cryostate 5\n"
    "LHE_VALVE        BIT cryostate 6\n"
    "LHE_DIREC        BIT cryostate 7\n"
    "### Cryo Valve Limit Switches ###\n"
    "POT_IS_CLOSED    BIT cryoin 0\n"
    "POT_IS_OPEN      BIT cryoin 1\n"
    "POT_STATE        LINCOM 2 POT_IS_CLOSED 1 0 POT_IS_OPEN 1 0\n"
    "LHE_IS_CLOSED    BIT cryoin 2\n"
    "LHE_IS_OPEN      BIT cryoin 3\n"
    "LHE_STATE        LINCOM 2 LHE_IS_CLOSED 1 0 LHE_IS_OPEN 1 0\n"
    "### Cryo Table Lookups ###\n"
    "# Diodes\n"
    "T_opt_box_w	LINTERP	T_OPT_BOX_W	/data/etc/dt600.txt\n"
    "T_lhe_filt   LINTERP T_LHE_FILT    /data/etc/dt600.txt\n"
    "T_vcs	LINTERP	T_VCS	/data/etc/dt600.txt\n"
    "T_vcs_filt   LINTERP T_VCS_FILT    /data/etc/dt600.txt\n"
    "T_ln2   LINTERP T_LN2    /data/etc/dt600.txt\n"
    "T_ln2_filt   LINTERP T_LN2_FILT    /data/etc/dt600.txt\n"
    "T_charcoal   LINTERP T_CHARCOAL    /data/etc/dt600.txt\n"
    "T_optics_box   LINTERP T_OPTICS_BOX    /data/etc/dt600.txt\n"
    "T_jfet   LINTERP T_JFET    /data/etc/dt600.txt\n"
    "T_he4pot_d   LINTERP T_HE4POT_D    /data/etc/dt600.txt\n"
    "T_cold_plate   LINTERP T_COLD_PLATE   /data/etc/dt600.txt\n"
    "# GRTs (ROX)\n"
    "#T_he3fridge	LINTERP	T_HE3FRIDGE	/data/etc/rox102a.txt\n"
    "#T_he4pot    LINTERP T_HE4POT    /data/etc/rox102a.txt\n"
    "#T_horn_500    LINTERP T_HORN_500   /data/etc/rox102a.txt\n"
    "#T_base_500   LINTERP T_BASE_500   /data/etc/rox102a.txt\n"
    "#T_base_250    LINTERP T_BASE_250   /data/etc/rox102a.txt\n"
    "# Level Sensor\n"
    "he4_litre    LINTERP HE4_LEVEL   /data/etc/he4_level.txt\n"
    "# Control Bits\n"
    "# To Be Filled In At A Later Date\n"
    "# TEMPORARY GRT Calibration\n"
    "Res_He3	       LINCOM  1       N10C0   1.9416E04       -8.4574\n"
    "Res_horn_500   LINCOM  1       N10C1   1.9416E04       -8.4574\n"
    "Res_pot        LINCOM  1       N10C2   1.9416E04       -8.4574\n"
    "Res_base_500   LINCOM  1       N10C4   1.9416E04       -8.4574\n"
    "Res_ring_250   LINCOM  1       N10C7   1.9416E04       -8.4574\n"
    "T_he3fridge LINTERP N10C3        /data/etc/rox102a.txt\n"
    "T_horn_500  LINTERP N10C7       /data/etc/rox102a.txt\n"
    "T_he4pot    LINTERP N10C22       /data/etc/rox102a.txt\n"
    "T_base_500  LINTERP N10C4       /data/etc/rox102a.txt \n"
    "T_base_250  LINTERP N10C5       /data/etc/rox102a.txt\n"
    "#\n"
    "Clin_Elev LINTERP clin_elev /data/etc/clin_elev.lut\n"
    "# Nice CPU Values\n"
    "CPU_SEC LINCOM  1       cpu_time        1       -%lu\n"
    "CPU_MIN LINCOM  1       CPU_SEC 0.016666666 0\n"
    "CPU_HOUR LINCOM 1       CPU_SEC 0.000277777 0\n"
    "CPU_DAY LINCOM  1       CPU_SEC 1.15741E-5  0\n"
    "CPU_WEEK LINCOM 1       CPU_SEC 1.65344E-6  0\n"
    "CPU_MONTH LINCOM 1      CPU_SEC 3.85803E-7  0\n"
    "CPU_YEAR LINCOM 1       CPU_SEC 3.17099E-8  0\n", time(NULL)
    );
}
