#define N_SCOMMANDS 63         /* total number of single word commands */
#define N_NM_SCOMMANDS 54      /* total number of named single word cmds */
#define N_MCOMMANDS 30         /* total number of multiword commands */
#define MAX_N_PARAMS 12
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#if N_SCOMMANDS < N_NM_SCOMMANDS
#error N_SCOMMANDS < N_NM_SCOMMANDS
#endif

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 16
#define SIZE_ABOUT 80
#define SIZE_PARNAME 25

#define N_GROUPS 13

#define GR_POINT 0
#define GR_EHEAT 1
#define GR_BIAS 2
#define GR_MOTORS 3
#define GR_BAL 4
#define GR_CRYO 5
#define GR_SENSOR 6
#define GR_COOL 7
#define GR_VALVE 8
#define GR_LOCK 9
#define GR_MISC 10
#define GR_CALIBRATOR 11
#define GR_TEST 12

#ifdef INCLUDE_VARS

const char *GroupNames[N_GROUPS] = {
  "Pointing",
  "Electronics Heat",
  "Bias and DAS",
  "Motor Control",
  "Balance System",
  "Cryo Control",
  "Sensor Systems",
  "Cooling System",
  "Motorized Valves",
  "Inner Frame Lock",
  "Miscellaneous",
  "Calibrator",
  "Test"
};

struct scom {
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  char group;
};

struct scom scommands[N_NM_SCOMMANDS] = {
  {"all_stop", "all stop: set az vel and el vel to zero", GR_POINT},

  {"fss_default", "set sun sensor as default", GR_SENSOR},
  {"isc_default", "set integrating star-cam as default", GR_SENSOR},
  {"vsc_default", "set video star-cam as default", GR_SENSOR},
  {"mag_default", "set magnetometer as default", GR_SENSOR},

  {"fss_veto", "veto sun sensor", GR_SENSOR},
  {"isc_veto", "veto integrating star-cam", GR_SENSOR},
  {"vsc_veto", "veto video star-cam", GR_SENSOR},
  {"mag_veto", "veto magnotometer", GR_SENSOR},

  {"fss_unveto", "un-veto sun sensor", GR_SENSOR},
  {"isc_unveto", "un-veto integrating star-cam", GR_SENSOR},
  {"vsc_unveto", "un-veto video star-cam", GR_SENSOR},
  {"mag_unveto", "un-veto magnetometer", GR_SENSOR},

  {"clock_internal", "bias clock internal", GR_BIAS},
  {"clock_external", "bias clock external", GR_BIAS},
  {"bias_ac", "bias AC", GR_BIAS},
  {"bias_dc", "bias DC", GR_BIAS},
  {"bias_ramp", "bias: external, ramp", GR_BIAS},
  {"bias_internal", "bias: internal, fixed", GR_BIAS},

  {"levl_sensor_on", "helium level sensor on", GR_CRYO},
  {"levl_sensor_off", "helium level sensor off", GR_CRYO},
  {"charc_heat_on", "charcoal heater on", GR_CRYO},
  {"charc_heat_off", "charcoal heater off", GR_CRYO},
  {"cplate_heat_on", "cold plate heater on", GR_CRYO},
  {"cplate_heat_off", "cold plate heater off", GR_CRYO},

  {"ln2_valve_on",  "LN valve on", GR_VALVE},
  {"ln2_valve_off", "LN valve off", GR_VALVE},
  {"ln2_valve_open", "set LN valve direction open", GR_VALVE},
  {"ln2_valve_close", "set LN valve direction close", GR_VALVE},
  {"lhe_valve_on",  "LHe valve on", GR_VALVE},
  {"lhe_valve_off", "LHe valve off", GR_VALVE},
  {"lhe_valve_open", "set LHe valve direction open", GR_VALVE},
  {"lhe_valve_close", "set LHe valve direction close", GR_VALVE},

  {"calibrator_on", "calibrator on", GR_CALIBRATOR},
  {"calibrator_off", "calibrator off", GR_CALIBRATOR},
  {"calibrator_stop", "stop calibrator pulsing", GR_CALIBRATOR},

  {"bal_veto", "veto balance system", GR_BAL},
  {"bal_unveto", "unveto balance system", GR_BAL},

  {"bal_pump1_on", "balance pump 1 on", GR_BAL},
  {"bal_pump1_off", "balance pump 1 off", GR_BAL},
  {"bal_pump1_frwrd", "balance pump 1 forward", GR_BAL},
  {"bal_pump1_rvrse", "balance pump 1 reverse", GR_BAL},
  {"bal_pump2_on", "balance pump 2 on", GR_BAL},
  {"bal_pump2_off", "balance pump 2 off", GR_BAL},
  {"bal_pump2_frwrd", "balance pump 2 forward", GR_BAL},
  {"bal_pump2_rvrse", "balance pump 2 reverse", GR_BAL},

  {"if_pump1_on", "inner frame cooling pump 1 on", GR_COOL},
  {"if_pump1_off", "inner frame cooling pump 1 off", GR_COOL},

  {"of_pump1_on", "outer frame colling pump 1 on", GR_COOL},
  {"of_pump1_off", "outer frame colling pump 1 off", GR_COOL},
  {"of_pump2_on", "outer frame colling pump 2 on", GR_COOL},
  {"of_pump2_off", "outer frame colling pump 2 off", GR_COOL},

  {"lock_mot_lock", "close the lock motor now (dangerous)", GR_LOCK},
  {"lock_mot_unlock", "unlock the lock now", GR_LOCK},
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
  {"lock_mot_el", "Lock Inner Frame", GR_LOCK, 1,
    {
      {"Lock Elevation (deg)", 5, 90, 'f', 4, "ENC_ELEV"}
    }
  },

  {"goto_el", "goto elevation", GR_POINT, 1,
    {
      {"elevation (deg)", 10, 90, 'f', 4, "ENC_ELEV"}
    }
  },

  {"goto_az", "goto azimuth", GR_POINT, 1,
    {
      {"azimuth (deg)", 0, 360, 'f', 4, "MAG_AZ"}
    }
  },

  {"el_vel", "elevation velocity", GR_POINT, 1,
    {
      {"velocity (deg/s)", -2.0, 2.0, 'f', 1, "0.0"}
    }
  },

  {"az_vel", "azimuth velocity", GR_POINT, 1,
    {
      {"velocity (deg/s)", -2.0, 2.0, 'f', 1, "0.0"}
    }
  },

  {"time_sched", "time until schedule mode", GR_MISC, 1,
    {
      {"timeout (s)", 15, 14400, 'i', 0, "ADD"}
    }
  },

  {"gyro_box_temp", "gyro box temp", GR_EHEAT, 1,
    {
      {"deg C", 0, 60, 'f', 2, "t_gy_set"}
    }
  },

  {"isc_pv_temp", "ISC pressure vessel temp", GR_EHEAT, 1,
    {
      {"deg C", 0, 60, 'f', 2, "t_isc_set"}
    }
  },


  {"roll_gain", "roll gain", GR_MOTORS, 1,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_roll"}
    }
  },

  {"elev_gain", "elevation gains", GR_MOTORS, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_el"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_el"}
    }
  },

  {"azim_gain", "azimuth gains", GR_MOTORS, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_az"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_az"}
    }
  },

  {"pivot_gain", "pivot gains", GR_MOTORS, 2,
    {
      {"Set Point(rpm)", 0, MAX_15BIT, 'f', 0, "set_reac"},
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_pivot"}
    }
  },

  {"gyro_heat_gain", "gyro heat gain", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_gyheat"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_gyheat"},
      {"Derrivative Gain", 0, MAX_15BIT, 'i', 0, "g_d_gyheat"}
    }
  },

  {"isc_heat_gain", "isc heat gain", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_pisch"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_iisch"},
      {"Derrivative Gain", 0, MAX_15BIT, 'i', 0, "g_disch"}
    }
  },

  {"bias_1_level", "bias 1 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"bias_2_level", "bias 2 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"bias_3_level", "bias 3 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"phase", "set phase shift", GR_BIAS, 2,
    {
      {"DAS card", 5, 16, 'i', 0, "ADD"},
      {"Phase", 0, 2000, 'i', 0, "ADD"}
    }
  },

  {"jfet_heat", "JFET heater level", GR_CRYO, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "JFETPWM"}
    }
  },

  {"heatswitch_heat", "Heat Switch heater level", GR_CRYO, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "HSPWM"}
    }
  },

  {"he3_heat", "Helium 3 heater level", GR_CRYO, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "HE3PWM"}
    }
  },

  {"spare_cryo_pwm", "Spare cryo pwm level", GR_CRYO, 1,
    {
      {"level (%)", 0, 100, 'f', 2, "CRYOPWM"}
    }
  },

  {"calib_pulse", "calibrator single pulse", GR_CALIBRATOR, 1,
    {
      {"pulse length (ms)", 0, 8000, 'i', 0, "ADD"}
    }
  },

  {"calib_pulse_rpt", "pulse calibrator repeatedly", GR_CALIBRATOR, 2,
    {
      {"pulse length (ms)", 1, 8000, 'i', 0, "ADD"},
      {"repeat delay (s)", 1, 86400, 'f', 0, "ADD"}
    }
  },

  {"xml_file", "set XML file for compressed downlink", GR_MISC, 1,
    {
      {"file#", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"balance_pwm", "balance pump pwm level", GR_BAL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  },

  {"balance_goal", "balance system goals", GR_BAL, 3,
    {
      {"pump on point (A)", 0, 2, 'f', 5, "BAL_ON"},
      {"pump off point (A)", 0, 2, 'f', 5, "BAL_OFF"},
      {"target (A)", -2, 2, 'f', 5, "BAL_TARGET"}
    }
  },

  {"spare_pump_pwm", "spare pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  },

  {"if_pump_pwm", "inner frame cooling pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  },

  {"of_pump_pwm", "outer frame cooling pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  }
};

#endif
