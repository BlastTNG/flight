#define N_SCOMMANDS 63         /* total number of single word commands */
#define N_NM_SCOMMANDS 53      /* total number of named single word cmds */
#define N_MCOMMANDS 27         /* total number of multiword commands */
#define MAX_N_PARAMS 12
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#if N_SCOMMANDS < N_NM_SCOMMANDS
#error N_SCOMMANDS < N_NM_SCOMMANDS
#endif

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 8
#define SIZE_ABOUT 80
#define SIZE_PARNAME 25

#define N_GROUPS 11

#define GR_POINT 0
#define GR_GAIN 1
#define GR_BIAS 2
#define GR_SENSOR 3
#define GR_CRYO 4
#define GR_BAL 5
#define GR_COOL 6
#define GR_LOCK 7
#define GR_EHEAT 8
#define GR_MISC 9
#define GR_TEST 10

#ifdef INCLUDE_VARS

const char *GroupNames[N_GROUPS] = {
  "Pointing",
  "Motor Control",
  "Bias Control",
  "Sensor Systems",
  "Cryo Control",
  "Balance System",
  "Cooling System",
  "Inner Frame Lock",
  "Electronics Heat",
  "Miscellaneous",
  "Test"
};

struct scom {
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  char group;
};

struct scom scommands[N_NM_SCOMMANDS] = {
  {"all_stp", "all stop: set az vel and el vel to zero", GR_POINT},

  {"sns_def", "set sun sensor as default", GR_SENSOR},
  {"isc_def", "set integrating star-cam as default", GR_SENSOR},
  {"vsc_def", "set video star-cam as default", GR_SENSOR},
  {"mag_def", "set magnetometer as default", GR_SENSOR},

  {"sns_vet", "veto sun sensor", GR_SENSOR},
  {"isc_vet", "veto integrating star-cam", GR_SENSOR},
  {"vsc_vet", "veto video star-cam", GR_SENSOR},
  {"mag_vet", "veto magnotometer", GR_SENSOR},

  {"sns_uvt", "un-veto sun sensor", GR_SENSOR},
  {"isc_uvt", "un-veto integrating star-cam", GR_SENSOR},
  {"vsc_uvt", "un-veto video star-cam", GR_SENSOR},
  {"mag_uvt", "un-veto magnetometer", GR_SENSOR},

  {"b_clint", "bias clock internal", GR_BIAS},
  {"b_clext", "bias clock external", GR_BIAS},
  {"bias_ac", "bias AC", GR_BIAS},
  {"bias_dc", "bias DC", GR_BIAS},
  {"b_extrn", "bias: external, ramp", GR_BIAS},
  {"b_intrn", "bias: internal, fixed", GR_BIAS},

  {"he_lv_1", "helium level sensor on", GR_CRYO},
  {"he_lv_0", "helium level sensor off", GR_CRYO},
  {"charc_1", "charcoal heater on", GR_CRYO},
  {"charc_0", "charcoal heater off", GR_CRYO},
  {"cplat_1", "cold plate heater on", GR_CRYO},
  {"cplat_0", "cold plate heater off", GR_CRYO},
  {"calib_1", "calibrator on", GR_CRYO},
  {"calib_0", "calibrator off", GR_CRYO},
  {"lnv_opn", "set LN valve direction open", GR_CRYO},
  {"lnv_cls", "set LN valve direction close", GR_CRYO},
  {"ln_val1", "LN valve on", GR_CRYO},
  {"ln_val0", "LN valve off", GR_CRYO},
  {"lhe_opn", "set LHe valve direction open", GR_CRYO},
  {"lhe_cls", "set LHe valve direction close", GR_CRYO},
  {"lhe_vl1", "LHe valve on", GR_CRYO},
  {"lhe_vl0", "LHe valve off", GR_CRYO},

  {"bal_vet", "veto balance system", GR_BAL},
  {"bal_uvt", "unveto balance system", GR_BAL},

  {"bl_p1_1", "balance pump 1 on", GR_BAL},
  {"bl_p1_0", "balance pump 1 off", GR_BAL},
  {"bl_p1_f", "balance pump 1 forward", GR_BAL},
  {"bl_p1_r", "balance pump 1 reverse", GR_BAL},
  {"bl_p2_1", "balance pump 2 on", GR_BAL},
  {"bl_p2_0", "balance pump 2 off", GR_BAL},
  {"bl_p2_f", "balance pump 2 forward", GR_BAL},
  {"bl_p2_r", "balance pump 2 reverse", GR_BAL},

  {"if_p1_1", "inner frame cooling pump 1 on", GR_COOL},
  {"if_p1_0", "inner frame cooling pump 1 off", GR_COOL},

  {"of_p1_1", "outer frame colling pump 1 on", GR_COOL},
  {"of_p1_0", "outer frame colling pump 1 off", GR_COOL},
  {"of_p2_1", "outer frame colling pump 2 on", GR_COOL},
  {"of_p2_0", "outer frame colling pump 2 off", GR_COOL},

  {"lkmot_l", "close the lock motor now (dangerous)", GR_LOCK},
  {"lkmot_u", "unlock the lock now", GR_LOCK},
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
  {"lock_el", "Lock Inner Frame", GR_LOCK, 1,
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

  {"t_sched", "time until schedule mode", GR_MISC, 1,
    {
      {"timeout (s)", 15, 14400, 'i', 0, "ADD"}
    }
  },

  {"gyrob_t", "gyro box T", GR_EHEAT, 1,
    {
      {"deg C", 0, 60, 'f', 2, "t_gy_set"}
    }
  },

  {"iscbx_t", "isc box T", GR_EHEAT, 1,
    {
      {"deg C", 0, 60, 'f', 2, "t_isc_set"}
    }
  },


  {"gn_roll", "roll gain", GR_GAIN, 1,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_roll"}
    }
  },

  {"gn_elev", "elevation gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_el"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_el"}
    }
  },

  {"gn_azim", "azimuth gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_az"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_az"}
    }
  },

  {"gn_pivo", "pivot gains", GR_GAIN, 2,
    {
      {"Set Point(rpm)", 0, MAX_15BIT, 'r', 0, "set_reac"},
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_pivot"}
    }
  },

  {"gn_gyht", "gyro heat gain", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_p_gyheat"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_i_gyheat"},
      {"Derrivative Gain", 0, MAX_15BIT, 'i', 0, "g_d_gyheat"}
    }
  },

  {"gn_isc", "isc heat gain", GR_EHEAT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', 0, "g_pisch"},
      {"Integral Gain", 0, MAX_15BIT, 'i', 0, "g_iisch"},
      {"Derrivative Gain", 0, MAX_15BIT, 'i', 0, "g_disch"}
    }
  },

  {"b_levl1", "bias 1 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"b_levl2", "bias 2 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"b_levl3", "bias 3 level", GR_BIAS, 1,
    {
      {"level", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"jfet_ht", "JFET heater level", GR_CRYO, 1,
    {
      {"level", 0, 100, 'i', 0, "ADD"}
    }
  },

  {"hs_heat", "Heat Switch heater level", GR_CRYO, 1,
    {
      {"level", 0, 100, 'i', 0, "ADD"}
    }
  },

  {"he3_ht", "Helium 3 heater level", GR_CRYO, 1,
    {
      {"level", 0, 100, 'i', 0, "ADD"}
    }
  },

  {"cryopwm", "Spare cryo pwm level", GR_CRYO, 1,
    {
      {"level", 0, 100, 'i', 0, "ADD"}
    }
  },

  {"xml_fil", "set XML file for compressed downlink", GR_MISC, 1,
    {
      {"file#", 0, 15, 'i', 0, "ADD"}
    }
  },

  {"balpwm", "balance pump pwm level", GR_BAL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  },

  {"balgoal", "balance system goals", GR_BAL, 3,
    {
      {"pump on point (A)", 0, 2, 'f', 5, "BAL_ON"},
      {"pump off point (A)", 0, 2, 'f', 5, "BAL_OFF"},
      {"target (A)", -2, 2, 'f', 5, "BAL_TARGET"}
    }
  },

  {"sprpwm", "spare pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  },

  {"inpwm", "inner frame cooling pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  },

  {"outpwm", "outer frame cooling pump pwm level", GR_COOL, 1,
    {
      {"level", 0, 2047, 'i', 0, "ADD"}
    }
  }
};

#endif
