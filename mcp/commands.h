#include "isc_protocol.h"  /* required for constants */

#define N_NM_SCOMMANDS 71      /* total number of named single word cmds */
#define N_SCOMMANDS N_NM_SCOMMANDS  /* total number of single word commands */
#define N_MCOMMANDS 44         /* total number of multiword commands */
#define MAX_N_PARAMS 6
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#if N_SCOMMANDS < N_NM_SCOMMANDS
#error N_SCOMMANDS < N_NM_SCOMMANDS
#endif

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 80
#define SIZE_ABOUT 80
#define SIZE_PARNAME 80

#define N_GROUPS 14

#define GR_POINT        0x0001
#define GR_BAL          0x0002
#define GR_BIAS         0x0004
#define GR_TRIM         0x0008
#define GR_COOL         0x0010
#define GR_CALLAMP      0x0020
#define GR_VETO         0x0040
#define GR_EHEAT        0x0080
#define GR_CRYO_HEAT    0x0100
#define GR_GAIN         0x0200
#define GR_MISC         0x0400
#define GR_CRYO_CONTROL 0x0800
#define GR_LOCK         0x1000
#define GR_ISC          0x2000

#ifdef INCLUDE_VARS

const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Balance System",    "Bias",
  "Pointing Sensor Trims", "Cooling System",    "Cal Lamp",
  "Pointing Sensor Vetos", "Electronics Heat",  "Cryo Heat",
  "Pointing Motor Gains",  "Miscellaneous",     "Cryo Control",
  "Inner Frame Lock",      "ISC Commanding"
};

struct scom {
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  int group;
};

struct scom scommands[N_NM_SCOMMANDS] = {
  {"stop", "servo off of gyros to zero speed now", GR_POINT},

  {"az_off", "disable az motors", GR_GAIN},
  {"az_on",  "enable az motors",  GR_GAIN},
  {"el_off", "disable el motors", GR_GAIN},
  {"el_on",  "enable el motors",  GR_GAIN},

  {"elclin_veto", "veto elevation clinometer", GR_VETO},
  {"elclin_allow", "un-veto elevation clinometer", GR_VETO},
  {"elenc_veto", "veto elevation encoder", GR_VETO},
  {"elenc_allow", "un-veto elevation encoder", GR_VETO},
  {"sun_veto", "veto sun sensor", GR_VETO},
  {"sun_allow", "un-veto sun sensor", GR_VETO},
  {"isc_veto", "veto integrating star-cam", GR_VETO},
  {"isc_allow", "un-veto integrating star-cam", GR_VETO},
  {"mag_veto", "veto magnotometer", GR_VETO},
  {"mag_allow", "un-veto magnetometer", GR_VETO},
  {"gps_veto", "veto differntial gps", GR_VETO},
  {"gps_allow", "un-veto differential gps", GR_VETO},

  {"reset", "reset coarse pointing trims to zero", GR_TRIM},
  {"trim_to_isc", "trim coarse sensors to isc", GR_TRIM},

  {"clock_int", "bias clock internal", GR_BIAS},
  {"clock_ext", "bias clock external", GR_BIAS},
  {"bias_ac", "bias AC", GR_BIAS},
  {"bias_dc", "bias DC", GR_BIAS},
  {"ramp", "bias: external, ramp", GR_BIAS},
  {"fixed", "bias: internal, fixed", GR_BIAS},

  {"charcoal_on", "charcoal heater on", GR_CRYO_HEAT},
  {"charcoal_off", "charcoal heater off", GR_CRYO_HEAT},
  {"coldplate_on", "cold plate heater on", GR_CRYO_HEAT},
  {"coldplate_off", "cold plate heater off", GR_CRYO_HEAT},

  {"cal_on", "calibrator on", GR_CALLAMP},
  {"cal_off", "calibrator off", GR_CALLAMP},
  {"cal_stop", "stop calibrator pulses", GR_CALLAMP},

  {"level_on", "helium level sensor on", GR_CRYO_CONTROL},
  {"level_off", "helium level sensor off", GR_CRYO_CONTROL},
  {"ln_valve_on",  "LN valve on", GR_CRYO_CONTROL},
  {"ln_valve_off", "LN valve off", GR_CRYO_CONTROL},
  {"ln_valve_open", "set LN valve direction open", GR_CRYO_CONTROL},
  {"ln_valve_close", "set LN valve direction close", GR_CRYO_CONTROL},
  {"he_valve_on",  "helium valve on", GR_CRYO_CONTROL},
  {"he_valve_off", "helium valve off", GR_CRYO_CONTROL},
  {"he_valve_open", "set helium valve direction open", GR_CRYO_CONTROL},
  {"he_valve_close", "set helium valve direction close", GR_CRYO_CONTROL},

  {"balance_veto", "veto balance system", GR_BAL},
  {"balance_allow", "unveto balance system", GR_BAL},
  {"pump1_on", "balance pump 1 on", GR_BAL},
  {"pump1_off", "balance pump 1 off", GR_BAL},
  {"pump1_fwd", "balance pump 1 forward", GR_BAL},
  {"pump1_rev", "balance pump 1 reverse", GR_BAL},
  {"pump2_on", "balance pump 2 on", GR_BAL},
  {"pump2_off", "balance pump 2 off", GR_BAL},
  {"pump2_fwd", "balance pump 2 forward", GR_BAL},
  {"pump2_rev", "balance pump 2 reverse", GR_BAL},

  {"inner_cool_on", "inner frame cooling pump 1 on", GR_COOL},
  {"inner_cool_off", "inner frame cooling pump 1 off", GR_COOL},
  {"outer_cool_on", "outer frame colling pump 1 on", GR_COOL},
  {"outer_cool_off", "outer frame colling pump 1 off", GR_COOL},
  {"outer_spare_on", "outer frame colling pump 2 on", GR_COOL},
  {"outer_spare_off", "outer frame colling pump 2 off", GR_COOL},

  {"pin_in", "close lock pin without checking encoder (dangerous)", GR_LOCK},
  {"unlock", "unlock the lock", GR_LOCK},
  {"use_limitswitch", "reset pin position overrides and use limit switches", GR_LOCK},
  {"pin_in_override", "override limit switch readout and set state to pin in", GR_LOCK},
  {"pin_out_override", "override limit switch readout and set state to pin out", GR_LOCK},

  {"isc_run", "start automatic image capture (normal mode)", GR_ISC},
  {"pause", "pause image capture", GR_ISC},
  {"isc_abort", "abort current solution attempt", GR_ISC},
  {"no_bright_star", "cancel bright star mode", GR_ISC},
  {"save_images", "turn on saving of images", GR_ISC},
  {"discard_images", "turn off saving of images", GR_ISC},
  {"full_screen", "show full screen", GR_ISC},
  {"auto_focus", "autofocus camera", GR_ISC}
};

struct par {
  char name[SIZE_PARNAME];
  double min;
  double max;
  char type;
  char field[20];
};

struct mcom {
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  int group;
  char numparams;
  struct par params[MAX_N_PARAMS];
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
struct mcom mcommands[N_MCOMMANDS] = {

  /***************************************/
  /********** Pointing Mode **************/
  {"vcap", "scan a circle centred on RA/Dec with el drift", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El drift Speed (deg el/s)", 0,  2, 'f', "NONE"}
    }
  },

  {"cap", "scan a circle centred on RA/Dec with el steps", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"}
    }
  },
      
  {"box", "scan an az/el box centred on RA/Dec with el steps", GR_POINT, 6,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"}
    }
  },
      
  {"drift", "move at constant speed in az and el", GR_POINT, 2,
    {
      {"Az Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"},
      {"El Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"}
    }
  },

  {"az_scan", "scan in azimuth", GR_POINT, 4,
    {
      {"Az centre (deg)",       -180, 360, 'f', "NONE"},
      {"El centre (deg)",         15,  65, 'f', "NONE"},
      {"Width (deg on sky)",       0, 360, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)", 0,   2, 'f', "NONE"}
    }
  },

  {"az_el_goto", "goto point in azimuth and elevation", GR_POINT, 2,
    {
      {"Azimuth (deg)", -360, 360, 'f', "NONE"},
      {"Elevation (deg)", 15,  65, 'f', "NONE"}
    }
  },

  {"ra_dec_goto", "track a location RA/Dec", GR_POINT, 2,
    {
      {"RA of Centre (h)",      0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)", -90, 90, 'f', "NONE"}
    }
  },

  {"ra_dec_set", "define RA/Dec of current position", GR_TRIM, 2,
    {
      {"Current RA (h)",      0, 24, 'f', "NONE"},
      {"Current Dec (deg)", -90, 90, 'f', "NONE"}
    }
  },

  /***************************************/
  /********* Pointing Sensor Trims *******/
  {"isc_offset", "set offset of star camera from primary beam", GR_TRIM | GR_ISC, 2,
    {
      {"Offset Az (deg)", -5., 5, 'f', "ADD"},
      {"Offset El (deg)", -5., 5, 'f', "ADD"}
    }
  },

  /***************************************/
  /********** Pointing Motor Gains *******/
  {"roll_gain", "roll reaction wheel gain", GR_GAIN, 1,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_roll"}
    }
  },

  {"el_gain", "elevation motor gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_el"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_el"}
    }
  },

  {"az_gain", "az reaction wheel gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_az"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_az"}
    }
  },

  {"pivot_gain", "pivot gains", GR_GAIN, 2,
    {
      {"Set Point (rpm)",   0, MAX_15BIT, 'f', "set_reac"},
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_pivot"}
    }
  },

  /***************************************/
  /********** Inner Frame Lock  **********/
  {"lock", "lock inner frame", GR_LOCK | GR_POINT, 1,
    {
      {"Lock Elevation (deg)", 5, 90, 'f', "ENC_ELEV"}
    }
  },

  /***************************************/
  /********** Balance System  ************/
  {"setpoints", "balance system setpoints", GR_BAL, 3,
    {
      {"Pump On Point (A)",  0, 2, 'f', "BAL_ON"},
      {"Pump Off Point (A)", 0, 2, 'f', "BAL_OFF"},
      {"Target (A)",        -2, 2, 'f', "BAL_TARGET"}
    }
  },
  
  {"bal_gain", "balance system gain and extrema", GR_BAL, 3,
    {
      {"Gain",           0.01,   1, 'f', "BAL_GAIN"},
      {"Maximum Speed (%)", 0, 100, 'f', "BAL_MAX"},
      {"Minimum Speed (%)", 0, 100, 'f', "BAL_MIN"}
    }
  },

  {"bal_level", "balance pump pwm level", GR_BAL, 1,
    {
      {"Level (%)", 0, 100, 'f', "BALPUMP_LEV"}
    }
  },

  /***************************************/
  /********** Cooling System  ************/
  {"spare_pwm", "spare pump pwm level", GR_COOL | GR_BAL, 1,
    {
      {"Level", 0, 2047, 'i', "sprpump_lev"}
    }
  },

  {"inner_pwm", "inner frame cooling pump speed", GR_COOL, 1,
    {
      {"Level", 0, 2047, 'i', "inpump_lev"}
    }
  },

  {"outer_pwm", "outer frame cooling pump speed", GR_COOL, 1,
    {
      {"Level", 0, 2047, 'i', "outpump_lev"}
    }
  },

  /***************************************/
  /******** Electronics Heaters  *********/
  {"t_gyrobox", "gyro box temperature set point", GR_EHEAT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "t_gy_set"}
    }
  },

  {"t_gyro_gain", "gyro box heater gains", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_gyheat"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_gyheat"},
      {"Derrivative Gain",  0, MAX_15BIT, 'i', "g_d_gyheat"}
    }
  },

  /***************************************/
  /*************** Misc  *****************/
  {"timeout", "time until schedule mode", GR_MISC, 1,
    {
      {"Timeout (s)", 15, 14400, 'i', "ADD"}
    }
  },

  {"xml_file", "set XML file for compressed downlink", GR_MISC, 1,
    {
      {"File #", 0, 15, 'i', "ADD"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {"bias1_level", "bias 1 level", GR_BIAS, 1,
    {
      {"Level", 0, 15, 'i', "bias_lev1"}
    }
  },

  {"bias2_level", "bias 2 level", GR_BIAS, 1,
    {
      {"Level", 0, 15, 'i', "bias_lev2"}
    }
  },

  {"bias3_level", "bias 3 level", GR_BIAS, 1,
    {
      {"Level", 0, 15, 'i', "bias_lev3"}
    }
  },

  {"phase", "set phase shift", GR_BIAS, 2,
    {
      {"DAS Card", 5,   16, 'i', "ADD"},
      {"Phase",    0, 2000, 'i', "ADD"}
    }
  },

  /***************************************/
  /*********** Cal Lamp  *****************/
  {"cal_pulse", "calibrator single pulse", GR_CALLAMP, 1,
    {
      {"Pulse Length (ms)", 0, 8000, 'i', "ADD"}
    }
  },

  {"cal_repeat", "pulse calibrator repeatedly", GR_CALLAMP, 2,
    {
      {"Pulse Length (ms)", 1,  8000, 'i', "ADD"},
      {"Repeat Delay (s)",  1, 86400, 'f', "ADD"}
    }
  },

  /***************************************/
  /********* Cryo heat   *****************/
  {"jfet_heat", "jfet heater pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "JFETPWM"}
    }
  },

  {"heatsw_heat", "heat switch pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "HSPWM"}
    }
  },

  {"he3_heat", "helium 3 pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "HE3PWM"}
    }
  },

  {"spare_heat", "spare cryo pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "CRYOPWM"}
    }
  },

  /***************************************/
  /********* ISC Commanding **************/
  {"set_focus", "set focus position", GR_ISC, 1,
    {
      {"Focus Position", 0, FOCUS_RANGE, 'i', "ISC_FOCUS"}
    }
  },

  {"set_aperture", "set the f-stop", GR_ISC, 1,
    {
      {"Aperture Position", 0, AP_RANGE, 'i', "ISC_APERT"}
    }
  },

  {"pixel_centre", "centre display on pixel", GR_ISC, 2,
    {
      {"Pixel X", 0, CCD_X_PIXELS - 1, 'i', "ADD"},
      {"Pixel Y", 0, CCD_Y_PIXELS - 1, 'i', "ADD"}
    }
  },

  {"blob_centre", "centre display on blob", GR_ISC, 1,
    {
      {"Blob #", 0, MAX_ISC_BLOBS, 'i', "ADD"}
    }
  },

  {"bright_star", "set RA/Dec of bright source", GR_ISC, 2,
    {
      {"RA (deg)",     0, 360, 'f', "ISC_BRRA"},
      {"Dec (deg)", -180, 180, 'f', "ISC_BRDEC"}
    }
  },

  {"integration", "set camera integration time", GR_ISC, 1,
    {
      {"integration time (ms)", 0, 5000, 'f', "ISC_PULSE"}
    }
  },

  {"det_set", "set detection parameters", GR_ISC, 5,
    {
      {"Search Grid (px/side)",     0, CCD_Y_PIXELS, 'i', "ISC_GRID"},
      {"S/N Threshold",           0.1,       3276.7, 'f', "ISC_THRESH"},
      {"Centroiding Box (px/side)", 0, CCD_Y_PIXELS, 'i', "ISC_CENBOX"},
      {"Photometry Box (px/side)",  0, CCD_Y_PIXELS, 'i', "ISC_APBOX"},
      {"Exclusion Distance (px)",   0, CCD_Y_PIXELS, 'i', "ISC_MDIST"}
    }
  },

  {"max_blobs", "max number of blobs used in solution", GR_ISC, 1,
    {
      {"# of Blobs", 0, MAX_ISC_BLOBS, 'i', "ISC_MAXBLOBS"}
    }
  },

  {"catalogue", "set catalogue retreival parameters", GR_ISC, 3,
    {
      {"Magnitude Limit",            0, 12, 'f', "ISC_MAG"},
      {"Normal Search Radius (deg)", 0, 50, 'f', "ISC_NRAD"},
      {"Lost Search Radius (deg)",   0, 50, 'f', "ISC_LRAD"}
    }
  },

  {"tolerances", "set pointing solution tolerances", GR_ISC, 4,
    {
      {"Assoc. Tolerance (arcsec)", 0, 1000, 'f', "ISC_TOL"},
      {"Match Tolerance (%)",       0,  100, 'f', "ISC_MTOL"},
      {"Quit Tolerance (%)",        0,  100, 'f', "ISC_QTOL"},
      {"Rot. Tolerance (deg)",      0,   90, 'f', "ISC_RTOL"}
    }
  }
};

#endif
