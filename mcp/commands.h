#include "isc_protocol.h"  /* required for constants */

#define N_SCOMMANDS 75      /* total number of named single word cmds */
#define N_MCOMMANDS 46         /* total number of multiword commands */
#define MAX_N_PARAMS 6
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

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

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions below */
enum singleCommand {
  auto_focus,       az_off,           az_on,            balance_allow,
  balance_veto,     bias_ac,          bias_dc,          cal_off,
  cal_on,           cal_stop,         charcoal_off,     charcoal_on,
  clock_ext,        clock_int,        coldplate_off,    coldplate_on,
  discard_images,   el_off,           el_on,            elclin_allow,
  elclin_veto,      elenc_allow,      elenc_veto,       fixed,
  full_screen,      gps_allow,        gps_veto,         he_valve_close,
  he_valve_on,      he_valve_off,     he_valve_open,    inner_cool_off,
  inner_cool_on,    isc_abort,        isc_allow,        isc_pause,
  isc_reconnect,    isc_run,          isc_shutdown,     isc_veto,
  level_off,        level_on,         mag_allow,        mag_veto,
  no_bright_star,   outer_cool_off,   outer_cool_on,    outer_spare_off,
  outer_spare_on,   pin_in,           pin_in_override,  pin_out_override,
  pot_valve_close,  pot_valve_off,    pot_valve_on,     pot_valve_open,
  pump1_fwd,        pump1_off,        pump1_on,         pump1_rev, 
  pump2_fwd,        pump2_off,        pump2_on,         pump2_rev,
  ramp,             reset_trims,      save_images,      stop,
  sun_veto,         sun_allow,        sync_adc,         trim_to_isc,
  unlock,           use_limitswitch,  xyzzy
};

struct scom {
  enum singleCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  int group;
};

#define COMMAND(x) x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},

  {COMMAND(az_off), "disable az motors", GR_GAIN},
  {COMMAND(az_on),  "enable az motors",  GR_GAIN},
  {COMMAND(el_off), "disable el motors", GR_GAIN},
  {COMMAND(el_on),  "enable el motors",  GR_GAIN},

  {COMMAND(elclin_veto), "veto elevation clinometer", GR_VETO},
  {COMMAND(elclin_allow), "un-veto elevation clinometer", GR_VETO},
  {COMMAND(elenc_veto), "veto elevation encoder", GR_VETO},
  {COMMAND(elenc_allow), "un-veto elevation encoder", GR_VETO},
  {COMMAND(sun_veto), "veto sun sensor", GR_VETO},
  {COMMAND(sun_allow), "un-veto sun sensor", GR_VETO},
  {COMMAND(isc_veto), "veto integrating star-cam", GR_VETO},
  {COMMAND(isc_allow), "un-veto integrating star-cam", GR_VETO},
  {COMMAND(mag_veto), "veto magnotometer", GR_VETO},
  {COMMAND(mag_allow), "un-veto magnetometer", GR_VETO},
  {COMMAND(gps_veto), "veto differntial gps", GR_VETO},
  {COMMAND(gps_allow), "un-veto differential gps", GR_VETO},

  {COMMAND(reset_trims), "reset coarse pointing trims to zero", GR_TRIM},
  {COMMAND(trim_to_isc), "trim coarse sensors to isc", GR_TRIM},

  {COMMAND(clock_int), "bias clock internal", GR_BIAS},
  {COMMAND(clock_ext), "bias clock external", GR_BIAS},
  {COMMAND(bias_ac), "bias AC", GR_BIAS},
  {COMMAND(bias_dc), "bias DC", GR_BIAS},
  {COMMAND(ramp), "bias: external, ramp", GR_BIAS},
  {COMMAND(fixed), "bias: internal, fixed", GR_BIAS},

  {COMMAND(charcoal_on), "charcoal heater on", GR_CRYO_HEAT},
  {COMMAND(charcoal_off), "charcoal heater off", GR_CRYO_HEAT},
  {COMMAND(coldplate_on), "cold plate heater on", GR_CRYO_HEAT},
  {COMMAND(coldplate_off), "cold plate heater off", GR_CRYO_HEAT},

  {COMMAND(cal_on), "calibrator on", GR_CALLAMP},
  {COMMAND(cal_off), "calibrator off", GR_CALLAMP},
  {COMMAND(cal_stop), "stop calibrator pulses", GR_CALLAMP},

  {COMMAND(level_on), "helium level sensor on", GR_CRYO_CONTROL},
  {COMMAND(level_off), "helium level sensor off", GR_CRYO_CONTROL},
  {COMMAND(pot_valve_on),  "He4 pot valve on", GR_CRYO_CONTROL},
  {COMMAND(pot_valve_off), "He4 pot valve off", GR_CRYO_CONTROL},
  {COMMAND(pot_valve_open), "set He4 pot valve direction open",
    GR_CRYO_CONTROL},
  {COMMAND(pot_valve_close), "set He4 pot valve direction close",
    GR_CRYO_CONTROL},
  {COMMAND(he_valve_on),  "he4 tank valve on", GR_CRYO_CONTROL},
  {COMMAND(he_valve_off), "he4 tank valve off", GR_CRYO_CONTROL},
  {COMMAND(he_valve_open), "set he4 tank valve direction open",
    GR_CRYO_CONTROL},
  {COMMAND(he_valve_close), "set he4 tank valve direction close",
    GR_CRYO_CONTROL},

  {COMMAND(xyzzy), "nothing happens here", GR_MISC},
  {COMMAND(sync_adc), "resync ADC Boards that require it", GR_MISC},

  {COMMAND(balance_veto), "veto balance system", GR_BAL},
  {COMMAND(balance_allow), "unveto balance system", GR_BAL},
  {COMMAND(pump1_on), "balance pump 1 on", GR_BAL},
  {COMMAND(pump1_off), "balance pump 1 off", GR_BAL},
  {COMMAND(pump1_fwd), "balance pump 1 forward", GR_BAL},
  {COMMAND(pump1_rev), "balance pump 1 reverse", GR_BAL},
  {COMMAND(pump2_on), "balance pump 2 on", GR_BAL},
  {COMMAND(pump2_off), "balance pump 2 off", GR_BAL},
  {COMMAND(pump2_fwd), "balance pump 2 forward", GR_BAL},
  {COMMAND(pump2_rev), "balance pump 2 reverse", GR_BAL},

  {COMMAND(inner_cool_on), "inner frame cooling pump 1 on", GR_COOL},
  {COMMAND(inner_cool_off), "inner frame cooling pump 1 off", GR_COOL},
  {COMMAND(outer_cool_on), "outer frame colling pump 1 on", GR_COOL},
  {COMMAND(outer_cool_off), "outer frame colling pump 1 off", GR_COOL},
  {COMMAND(outer_spare_on), "outer frame colling pump 2 on", GR_COOL},
  {COMMAND(outer_spare_off), "outer frame colling pump 2 off", GR_COOL},

  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK},
  {COMMAND(unlock), "unlock the lock", GR_LOCK},
  {COMMAND(use_limitswitch),
    "reset pin position overrides and use limit switches", GR_LOCK},
  {COMMAND(pin_in_override),
    "override limit switch readout and set state to pin in", GR_LOCK},
  {COMMAND(pin_out_override),
    "override limit switch readout and set state to pin out", GR_LOCK},

  {COMMAND(isc_run), "start automatic image capture (normal mode)", GR_ISC},
  {COMMAND(isc_pause), "pause image capture", GR_ISC},
  {COMMAND(isc_shutdown),
    "shutdown ISC computer in prepratation for power cycle", GR_ISC},
  {COMMAND(isc_abort), "abort current solution attempt", GR_ISC},
  {COMMAND(isc_reconnect),
    "tell mcp to try and establish a new connection with ISC", GR_ISC},
  {COMMAND(save_images), "turn on saving of images", GR_ISC},
  {COMMAND(discard_images), "turn off saving of images", GR_ISC},
  {COMMAND(no_bright_star), "cancel bright star mode", GR_ISC},
  {COMMAND(full_screen), "show full screen", GR_ISC},
  {COMMAND(auto_focus), "autofocus camera", GR_ISC}
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions below */
enum multiCommand {
  az_el_goto,   az_gain,      az_scan,          bal_gain,     bal_level,
  bias1_level,  bias2_level,  bias3_level,      blob_centre,  box,
  bright_star,  cal_pulse,    cal_repeat,       cap,          catalogue,
  az_el_trim,   det_set,      drift,            el_gain,      fast_integration,
  he3_heat,     heatsw_heat,  inner_pwm,        isc_offset,   jfet_heat,
  lock,         max_blobs,    outer_pwm,        phase,        pivot_gain,
  pixel_centre, ra_dec_goto,  ra_dec_set,       roll_gain,    set_aperture,
  set_focus,    setpoints,    slow_integration, spare_heat,   spare_pwm,
  t_gyrobox,    t_gyro_gain,  timeout,          tolerances,   vcap,
  xml_file
};

struct par {
  char name[SIZE_PARNAME];
  double min;
  double max;
  char type;
  char field[20];
};

struct mcom {
  enum multiCommand command;
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
  {COMMAND(vcap), "scan a circle centred on RA/Dec with el drift", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El drift Speed (deg el/s)", 0,  2, 'f', "NONE"}
    }
  },

  {COMMAND(cap), "scan a circle centred on RA/Dec with el steps", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"}
    }
  },
      
  {COMMAND(box), "scan an az/el box centred on RA/Dec with el steps",
    GR_POINT, 6,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"}
    }
  },
      
  {COMMAND(drift), "move at constant speed in az and el", GR_POINT, 2,
    {
      {"Az Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"},
      {"El Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"}
    }
  },

  {COMMAND(az_scan), "scan in azimuth", GR_POINT, 4,
    {
      {"Az centre (deg)",       -180, 360, 'f', "NONE"},
      {"El centre (deg)",         15,  65, 'f', "NONE"},
      {"Width (deg on sky)",       0, 360, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)", 0,   2, 'f', "NONE"}
    }
  },

  {COMMAND(az_el_goto), "goto point in azimuth and elevation", GR_POINT, 2,
    {
      {"Azimuth (deg)", -360, 360, 'f', "NONE"},
      {"Elevation (deg)", 15,  65, 'f', "NONE"}
    }
  },

  {COMMAND(ra_dec_goto), "track a location RA/Dec", GR_POINT, 2,
    {
      {"RA of Centre (h)",      0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)", -90, 90, 'f', "NONE"}
    }
  },

  {COMMAND(ra_dec_set), "define RA/Dec of current position", GR_TRIM, 2,
    {
      {"Current RA (h)",      0, 24, 'f', "NONE"},
      {"Current Dec (deg)", -90, 90, 'f', "NONE"}
    }
  },

  /***************************************/
  /********* Pointing Sensor Trims *******/
  {COMMAND(isc_offset), "set offset of star camera from primary beam",
    GR_TRIM | GR_ISC, 2,
    {
      {"X Offset (deg)", -5., 5, 'f', "ISC_X_OFF"},
      {"Y Offset (deg)", -5., 5, 'f', "ISC_Y_OFF"}
    }
  },

  {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },

  /***************************************/
  /********** Pointing Motor Gains *******/
  {COMMAND(roll_gain), "roll reaction wheel gain", GR_GAIN, 1,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_roll"}
    }
  },

  {COMMAND(el_gain), "elevation motor gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_el"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_el"}
    }
  },

  {COMMAND(az_gain), "az reaction wheel gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_az"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_az"}
    }
  },

  {COMMAND(pivot_gain), "pivot gains", GR_GAIN, 2,
    {
      {"Set Point (rpm)",   0, MAX_15BIT, 'f', "set_reac"},
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_pivot"}
    }
  },

  /***************************************/
  /********** Inner Frame Lock  **********/
  {COMMAND(lock), "lock inner frame", GR_LOCK | GR_POINT, 1,
    {
      {"Lock Elevation (deg)", 5, 90, 'f', "ENC_ELEV"}
    }
  },

  /***************************************/
  /********** Balance System  ************/
  {COMMAND(setpoints), "balance system setpoints", GR_BAL, 3,
    {
      {"Pump On Point (A)",  0, 2, 'f', "BAL_ON"},
      {"Pump Off Point (A)", 0, 2, 'f', "BAL_OFF"},
      {"Target (A)",        -2, 2, 'f', "BAL_TARGET"}
    }
  },
  
  {COMMAND(bal_gain), "balance system gain and extrema", GR_BAL, 3,
    {
      {"Gain",           0.01,   1, 'f', "BAL_GAIN"},
      {"Maximum Speed (%)", 0, 100, 'f', "BAL_MAX"},
      {"Minimum Speed (%)", 0, 100, 'f', "BAL_MIN"}
    }
  },

  {COMMAND(bal_level), "balance pump pwm level", GR_BAL, 1,
    {
      {"Level (%)", 0, 100, 'f', "BALPUMP_LEV"}
    }
  },

  /***************************************/
  /********** Cooling System  ************/
  {COMMAND(spare_pwm), "spare pump pwm level", GR_COOL | GR_BAL, 1,
    {
      {"Level", 0, 2047, 'i', "sprpump_lev"}
    }
  },

  {COMMAND(inner_pwm), "inner frame cooling pump speed", GR_COOL, 1,
    {
      {"Level", 0, 2047, 'i', "inpump_lev"}
    }
  },

  {COMMAND(outer_pwm), "outer frame cooling pump speed", GR_COOL, 1,
    {
      {"Level", 0, 2047, 'i', "outpump_lev"}
    }
  },

  /***************************************/
  /******** Electronics Heaters  *********/
  {COMMAND(t_gyrobox), "gyro box temperature set point", GR_EHEAT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "t_gy_set"}
    }
  },

  {COMMAND(t_gyro_gain), "gyro box heater gains", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_gyheat"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_gyheat"},
      {"Derrivative Gain",  0, MAX_15BIT, 'i', "g_d_gyheat"}
    }
  },

  /***************************************/
  /*************** Misc  *****************/
  {COMMAND(timeout), "time until schedule mode", GR_MISC, 1,
    {
      {"Timeout (s)", 1, 14400, 'i', "ADD"}
    }
  },

  {COMMAND(xml_file), "set XML file for compressed downlink", GR_MISC, 1,
    {
      {"File #", 0, 15, 'i', "ADD"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {COMMAND(bias1_level), "bias 1 level", GR_BIAS, 1,
    {
      {"Level", 0, 15, 'i', "bias_lev1"}
    }
  },

  {COMMAND(bias2_level), "bias 2 level", GR_BIAS, 1,
    {
      {"Level", 0, 15, 'i', "bias_lev2"}
    }
  },

  {COMMAND(bias3_level), "bias 3 level", GR_BIAS, 1,
    {
      {"Level", 0, 15, 'i', "bias_lev3"}
    }
  },

  {COMMAND(phase), "set phase shift", GR_BIAS, 2,
    {
      {"DAS Card", 5,   16, 'i', "ADD"},
      {"Phase",    0, 2000, 'i', "ADD"}
    }
  },

  /***************************************/
  /*********** Cal Lamp  *****************/
  {COMMAND(cal_pulse), "calibrator single pulse", GR_CALLAMP, 1,
    {
      {"Pulse Length (ms)", 0, 8000, 'i', "ADD"}
    }
  },

  {COMMAND(cal_repeat), "pulse calibrator repeatedly", GR_CALLAMP, 2,
    {
      {"Pulse Length (ms)", 1,  8000, 'i', "ADD"},
      {"Repeat Delay (s)",  1, 86400, 'f', "ADD"}
    }
  },

  /***************************************/
  /********* Cryo heat   *****************/
  {COMMAND(jfet_heat), "jfet heater pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "JFETPWM"}
    }
  },

  {COMMAND(heatsw_heat), "heat switch pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "HSPWM"}
    }
  },

  {COMMAND(he3_heat), "helium 3 pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "HE3PWM"}
    }
  },

  {COMMAND(spare_heat), "spare cryo pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "CRYOPWM"}
    }
  },

  /***************************************/
  /********* ISC Commanding **************/
  {COMMAND(set_focus), "set focus position", GR_ISC, 1,
    {
      {"Focus Position", 0, FOCUS_RANGE, 'i', "ISC_FOCUS"}
    }
  },

  {COMMAND(set_aperture), "set the f-stop", GR_ISC, 1,
    {
      {"Aperture Position", 0, AP_RANGE, 'i', "ISC_APERT"}
    }
  },

  {COMMAND(pixel_centre), "centre display on pixel", GR_ISC, 2,
    {
      {"Pixel X", 0, CCD_X_PIXELS - 1, 'i', "ADD"},
      {"Pixel Y", 0, CCD_Y_PIXELS - 1, 'i', "ADD"}
    }
  },

  {COMMAND(blob_centre), "centre display on blob", GR_ISC, 1,
    {
      {"Blob #", 0, MAX_ISC_BLOBS, 'i', "ADD"}
    }
  },

  {COMMAND(bright_star), "set RA/Dec of bright source", GR_ISC, 2,
    {
      {"RA (deg)",     0, 360, 'f', "ISC_BRRA"},
      {"Dec (deg)", -180, 180, 'f', "ISC_BRDEC"}
    }
  },

  {COMMAND(fast_integration), "set camera short integration time", GR_ISC, 1,
    {
      {"integration time (ms)", 0, 5000, 'i', "ISC_FAST_PULSE"}
    }
  },

  {COMMAND(slow_integration), "set camera long integration time", GR_ISC, 1,
    {
      {"integration time (ms)", 0, 5000, 'i', "ISC_PULSE"}
    }
  },

  {COMMAND(det_set), "set detection parameters", GR_ISC, 5,
    {
      {"Search Grid (px/side)",     0, CCD_Y_PIXELS, 'i', "ISC_GRID"},
      {"S/N Threshold",           0.1,       3276.7, 'f', "ISC_THRESH"},
      {"Centroiding Box (px/side)", 0, CCD_Y_PIXELS, 'i', "ISC_CENBOX"},
      {"Photometry Box (px/side)",  0, CCD_Y_PIXELS, 'i', "ISC_APBOX"},
      {"Exclusion Distance (px)",   0, CCD_Y_PIXELS, 'i', "ISC_MDIST"}
    }
  },

  {COMMAND(max_blobs), "max number of blobs used in solution", GR_ISC, 1,
    {
      {"# of Blobs", 0, MAX_ISC_BLOBS, 'i', "ISC_MAXBLOBS"}
    }
  },

  {COMMAND(catalogue), "set catalogue retreival parameters", GR_ISC, 3,
    {
      {"Magnitude Limit",            0, 12, 'f', "ISC_MAG"},
      {"Normal Search Radius (deg)", 0, 50, 'f', "ISC_NRAD"},
      {"Lost Search Radius (deg)",   0, 50, 'f', "ISC_LRAD"}
    }
  },

  {COMMAND(tolerances), "set pointing solution tolerances", GR_ISC, 4,
    {
      {"Assoc. Tolerance (arcsec)", 0, 1000, 'f', "ISC_TOL"},
      {"Match Tolerance (%)",       0,  100, 'f', "ISC_MTOL"},
      {"Quit Tolerance (%)",        0,  100, 'f', "ISC_QTOL"},
      {"Rot. Tolerance (deg)",      0,   90, 'f', "ISC_RTOL"}
    }
  }
};

#endif
