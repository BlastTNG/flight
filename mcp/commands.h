#include "isc_protocol.h"  /* required for constants */

#define N_SCOMMANDS 99         /* total number of single word commands */
#define N_NM_SCOMMANDS 64      /* total number of named single word cmds */
#define N_MCOMMANDS 43         /* total number of multiword commands */
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

#define N_GROUPS 13

#define GR_POINT 0
#define GR_BAL 1
#define GR_BIAS 2
#define GR_SENSOR 3
#define GR_COOL 4
#define GR_CALLAMP 5
#define GR_GAIN 6
#define GR_EHEAT 7
#define GR_CRYO_HEAT 8
#define GR_LOCK 9
#define GR_MISC 10
#define GR_CRYO_CONTROL 11
#define GR_ISC 12

#ifdef INCLUDE_VARS

const char *GroupNames[N_GROUPS] = {
  "Pointing Mode",        "Balance System",    "Bias",
  "Pointing Sensors",     "Cooling System",    "Cal Lamp",
  "Pointing Motor Gains", "Electronics Heat",  "Cryo Heat",
  "Inner Frame Lock",     "Miscellaneous",     "Cryo Control",
  "ISC Commanding"
};

struct scom {
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  char group;
};

struct scom scommands[N_NM_SCOMMANDS] = {
  {"stop", "servo off of gyros to zero speed now", GR_POINT},

  {"az_off", "disable az motors", GR_GAIN},
  {"az_on",  "enable az motors",  GR_GAIN},
  {"el_off", "disable el motors", GR_GAIN},
  {"el_on",  "enable el motors",  GR_GAIN},

  {"elclin_veto", "veto elevation clinometer", GR_SENSOR},
  {"elclin_allow", "un-veto elevation clinometer", GR_SENSOR},
  {"elenc_veto", "veto elevation encoder", GR_SENSOR},
  {"elenc_allow", "un-veto elevation encoder", GR_SENSOR},
  {"sun_veto", "veto sun sensor", GR_SENSOR},
  {"sun_allow", "un-veto sun sensor", GR_SENSOR},
  {"isc_veto", "veto integrating star-cam", GR_SENSOR},
  {"isc_allow", "un-veto integrating star-cam", GR_SENSOR},
  {"mag_veto", "veto magnotometer", GR_SENSOR},
  {"mag_allow", "un-veto magnetometer", GR_SENSOR},
  {"gps_veto", "veto differntial gps", GR_SENSOR},
  {"gps_allow", "un-veto differential gps", GR_SENSOR},

  {"clock_int", "bias clock internal", GR_BIAS},
  {"clock_ext", "bias clock external", GR_BIAS},
  {"bias_ac", "bias AC", GR_BIAS},
  {"bias_dc", "bias DC", GR_BIAS},
  {"ramp", "bias: external, ramp", GR_BIAS},
  {"fixed", "bias: internal, fixed", GR_BIAS},

  {"level_on", "helium level sensor on", GR_CRYO_CONTROL},
  {"level_off", "helium level sensor off", GR_CRYO_CONTROL},
  {"charcoal_on", "charcoal heater on", GR_CRYO_HEAT},
  {"charcoal_off", "charcoal heater off", GR_CRYO_HEAT},
  {"coldplate_on", "cold plate heater on", GR_CRYO_HEAT},
  {"coldplate_off", "cold plate heater off", GR_CRYO_HEAT},
  {"cal_on", "calibrator on", GR_CALLAMP},
  {"cal_off", "calibrator off", GR_CALLAMP},
  {"cal_stop", "stop calibrator pulses", GR_CALLAMP},
  {"ln_valve_on",  "LN valve on", GR_CRYO_CONTROL},
  {"ln_valve_off", "LN valve off", GR_CRYO_CONTROL},
  {"ln_valve_open", "set LN valve direction open", GR_CRYO_CONTROL},
  {"ln_valve_close", "set LN valve direction close", GR_CRYO_CONTROL},
  {"he_valve_on",  "Helium valve on", GR_CRYO_CONTROL},
  {"he_valve_off", "Helium valve off", GR_CRYO_CONTROL},
  {"he_valve_open", "set Helium valve direction open", GR_CRYO_CONTROL},
  {"he_valve_close", "set Helium valve direction close", GR_CRYO_CONTROL},

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

  {"isc_run", "start automatic image capture (normal mode)", GR_ISC},
  {"pause", "pause image capture", GR_ISC},
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
  char precision;
  char field[12];
};

struct mcom {
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  char group;
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
  {"ra_dec_raster", "raster scan a circle in RA/DEC", GR_POINT, 5,
    {
      {"RA of Center (h)", 0.0, 24.0, 'f', 4, "0.0"}, //FIXME: field
      {"DEC of Center (deg)", -90.0, 90.0, 'f', 4, "0.0"}, //FIXME: field
      {"Radius (deg on sky)", 0.0, 90.0, 'f', 4, "0.0"}, //FIXME: field
      {"Az Scan Speed (deg az/s)", 0.0, 2.0, 'f', 4, "0.1"},
      {"El drift Speed (deg el/s)", 0.0, 2.0, 'f', 4, "0.005"}
    }
  },
  {"ra_dec_goto", "Track a location RA/DEC", GR_POINT, 2,
    {
      {"RA of Center (h)", 0.0, 24.0, 'f', 4, "0.0"}, //FIXME: field
      {"DEC of Center (deg)", -90.0, 90.0, 'f', 4, "0.0"}, //FIXME: field
    }
  },
  {"ra_dec_set", "Define RA/DEC of current position", GR_POINT, 2,
    {
      {"Current RA (h)", 0.0, 24.0, 'f', 4, "0.0"}, //FIXME: field
      {"Current DEC (deg)", -90.0, 90.0, 'f', 4, "0.0"}, //FIXME: field
    }
  },
  {"az_scan", "scan in azimuth", GR_POINT, 3,
    {
      {"centre (deg)", -180.0, 360.0, 'f', 4, "AZ"}, //FIXME: field
      {"peak to peak (deg azimuth)", 0.0, 360, 'f', 4, "1.0"}, //FIXME: field
      {"scan speed (deg az/s", 0.01, 2, 'f', 4, "0.1"} //FIXME: field
    }
  },

  {"az_goto", "goto azimuth", GR_POINT, 1,
    {
      {"azimuth (deg)", -360.0, 360.0, 'f', 4, "AZ"}
    }
  },

  {"az_vel", "azimuth velocity", GR_POINT, 1,
    {
      {"velocity (deg/s)", -2.0, 2.0, 'f', 4, "0.0"}
    }
  },

  {"el_goto", "goto elevation", GR_POINT, 1,
    {
      {"elevation (deg)", 5.2, 90, 'f', 4, "EL"}
    }
  },

  {"el_vel", "elevation velocity", GR_POINT, 1,
    {
      {"velocity (deg/s)", -2.0, 2.0, 'f', 4, "0.0"}
    }
  },

  /***************************************/
  /********** Pointing Motor Gains *******/
  {"roll_gain", "roll reaction wheel gain", GR_GAIN, 1,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_roll"}
    }
  },

  {"el_gain", "elevation motor gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_el"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_el"}
    }
  },

  {"az_gain", "az reaction wheel gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_az"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_az"}
    }
  },

  {"pivot_gain", "pivot gains", GR_GAIN, 2,
    {
      {"Set Point (rpm)", 0, MAX_15BIT, 'f', 0, "set_reac"},
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_pivot"}
    }
  },

  /***************************************/
  /********** Inner Frame Lock  **********/
  {"lock", "Lock Inner Frame", GR_LOCK, 1,
    {
      {"Lock Elevation (deg)", 5, 90, 'f', 4, "ENC_ELEV"}
    }
  },

  /***************************************/
  /********** Balance System  ************/
  {"setpoints", "balance system setpoints", GR_BAL, 3,
    {
      {"pump on point (A)", 0, 2, 'f', 5, "BAL_ON"},
      {"pump off point (A)", 0, 2, 'f', 5, "BAL_OFF"},
      {"target (A)", -2, 2, 'f', 5, "BAL_TARGET"}
    }
  },
  
  {"bal_gain", "balance system gain and extrema", GR_BAL, 3,
    {
      {"gain", 0.01, 1.00, 'f', 2, "BAL_GAIN"},
      {"maximum speed (%)", 0, 100, 'f', 2, "BAL_MAX"},
      {"minimum speed (%)", 0, 100, 'f', 2, "BAL_MIN"}
    }
  },

  {"bal_level", "balance pump pwm level", GR_BAL, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "BALPUMP_LEV"}
    }
  },

  /***************************************/
  /********** Cooling System  ************/
  {"spare_pwm", "spare pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "sprpump_lev"}
    }
  },

  {"inner_pwm", "inner frame cooling pump speed", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "inpump_lev"}
    }
  },

  {"outer_pwm", "outer frame cooling pump speed", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "outpump_lev"}
    }
  },

  /***************************************/
  /******** Electronics Heaters  *********/
  {"t_gyrobox", "gyro box T", GR_EHEAT, 1,
    {
      {"deg C", 0, 60, 'f', 2, "t_gy_set"}
    }
  },

  {"t_gyro_gain", "gyro box heater gains", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_gyheat"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_gyheat"},
      {"Derrivative Gain", 0, MAX_15BIT, 'i', 0, "g_d_gyheat"}
    }
  },

  /***************************************/
  /*************** Misc  *****************/
  {"timeout", "time until schedule mode", GR_MISC, 1,
    {
      {"timeout (s)", 15, 14400, 'i', 0, "ADD"}
    }
  },

  {"xml_file", "set XML file for compressed downlink", GR_MISC, 1,
    {
      {"file#", 0, 15, 'i', 0, "ADD"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {"bias1_level", "bias 1 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "bias_lev1"}
    }
  },

  {"bias2_level", "bias 2 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "bias_lev2"}
    }
  },

  {"bias3_level", "bias 3 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "bias_lev3"}
    }
  },

  {"phase", "set phase shift", GR_BIAS, 2,
    {
      {"DAS card", 5, 16, 'i', 0, "ADD"},
      {"Phase", 0, 2000, 'i', 0, "ADD"}
    }
  },

  /***************************************/
  /*********** Cal Lamp  *****************/
  {"cal_pulse", "calibrator single pulse", GR_CALLAMP, 1,
    {
      {"pulse length (ms)", 0, 8000, 'i', 0, "ADD"}
    }
  },

  {"cal_repeat", "pulse calibrator repeatedly", GR_CALLAMP, 2,
    {
      {"pulse length (ms)", 1, 8000, 'i', 0, "ADD"},
      {"repeat delay (s)", 1, 86400, 'f', 0, "ADD"}
    }
  },

  /***************************************/
  /********* Cryo heat   *****************/
  {"jfet_heat", "JFET heater pwm", GR_CRYO_HEAT, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "JFETPWM"}
    }
  },

  {"heatsw_heat", "Heat Switch pwm", GR_CRYO_HEAT, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "HSPWM"}
    }
  },

  {"he3_heat", "Helium 3 pwm", GR_CRYO_HEAT, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "HE3PWM"}
    }
  },

  {"spare_heat", "Spare cryo pwm", GR_CRYO_HEAT, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "CRYOPWM"}
    }
  },

  /***************************************/
  /********* ISC Commanding **************/
  {"set_focus", "Set the focus position", GR_ISC, 1,
    {
      {"focus position", 0, FOCUS_RANGE, 'i', 0, "ISC_FOCUS"}
    }
  },

  {"set_aperture", "Set the F-stop", GR_ISC, 1,
    {
      {"aperture position", 0, AP_RANGE, 'i', 0, "ISC_APERT"}
    }
  },

  {"pixel_centre", "Centre display on pixel", GR_ISC, 2,
    {
      {"pixel x", 0, CCD_X_PIXELS - 1, 'i', 0, "ADD"},
      {"pixel y", 0, CCD_Y_PIXELS - 1, 'i', 0, "ADD"}
    }
  },

  {"blob_centre", "Centre display on blob", GR_ISC, 1,
    {
      {"blob #", 0, MAX_ISC_BLOBS, 'i', 0, "ADD"}
    }
  },

  {"integration", "Set Integration Time", GR_ISC, 1,
    {
      {"integration time (ms)", 0, 5000, 'f', 0, "ISC_PULSE"}
    }
  },

  {"det_set", "Set Detection Parameters", GR_ISC, 5,
    {
      {"search grid (px/side)", 0, CCD_Y_PIXELS, 'i', 0, "ISC_GRID"},
      {"S/N threshold", 0.1, 3276.7, 'f', 1, "ISC_THRESH"},
      {"centroiding box (px/side)", 0, CCD_Y_PIXELS, 'i', 0, "ISC_CENBOX"},
      {"photometry box (px/side)", 0, CCD_Y_PIXELS, 'i', 0, "ISC_APBOX"},
      {"exclusion distance (px)", 0, CCD_Y_PIXELS, 'i', 0, "ISC_MDIST"}
    }
  },

  {"catalogue", "Set Catalogue Retreival Parameters", GR_ISC, 3,
    {
      {"Magnitude Limit", 0, 12, 'f', 1, "ISC_MAG"},
      {"Normal Search Radius (deg)", 0, 50, 'f', 2, "ISC_NRAD"},
      {"Lost Search Radius (deg)", 0, 50, 'f', 2, "ISC_LRAD"}
    }
  },

  {"tolerances", "Set Pointing Solution Tolerances", GR_ISC, 4,
    {
      {"Assoc. Tolerance (arcsec)", 0, 1000, 'f', 0, "ISC_TOL"},
      {"Match Tolerance (%)", 0, 100, 'f', 2, "ISC_MTOL"},
      {"Quit Tolerance (%)", 0, 100, 'f', 2, "ISC_QTOL"},
      {"Rot. Tolerance (deg)", 0, 90, 'f', 1, "ISC_RTOL"}
    }
  }
};

#endif
