/* command_list.c: BLAST command specification file
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "command_list.h"
#include "isc_protocol.h"  /* required for constants */

const char command_list_serial[] = "$Revision: 2.35 $";

const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Balance System",    "Bias",
  "Pointing Sensor Trims", "Cooling System",    "Cal Lamp",
  "Pointing Sensor Vetos", "Electronics Heat",  "Cryo Heat",
  "Pointing Sensor Power", "Inner Frame Lock",  "Cryo Control",
  "Pointing Motor Gains",  "ISC Parameters",    "ISC Housekeeping",
  "Miscellaneous",         "OSC Parameters",    "OSC Housekeeping"
};

#define COMMAND(x) x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},

  {COMMAND(gps_off), "turn off the dGPS at the ACS", GR_POWER | CONFIRM},
  {COMMAND(gps_on), "turn on the dGPS at the ACS", GR_POWER},
  {COMMAND(gyro_off), "turn off the digital gyros at the ACS", GR_POWER
    | CONFIRM},
  {COMMAND(gyro_on), "turn on the digital gyros at the ACS", GR_POWER},
  {COMMAND(isc_off), "turn off the ISC at the ACS", GR_POWER | CONFIRM},
  {COMMAND(isc_on), "turn on the ISC at the ACS", GR_POWER},
  {COMMAND(osc_off), "turn off the OSC at the ACS", GR_POWER | CONFIRM},
  {COMMAND(osc_on), "turn on the OSC at the ACS", GR_POWER},
  {COMMAND(ss_off), "turn off the Sun Sensor at the ACS", GR_POWER
    | CONFIRM},
  {COMMAND(ss_on), "turn on the Sun Sensor at the ACS", GR_POWER},

  {COMMAND(az_off), "disable az motors", GR_GAIN},
  {COMMAND(az_on),  "enable az motors",  GR_GAIN},
  {COMMAND(el_off), "disable el motors", GR_GAIN},
  {COMMAND(el_on),  "enable el motors",  GR_GAIN},
  {COMMAND(force_el_on),  "force enable el motors despite the pin being in",
    CONFIRM | GR_GAIN},

  {COMMAND(analogue_gyros), "use the analogue gyros for in-flight pointing",
    GR_VETO},
  {COMMAND(digital_gyros), "use the digital gyros for in-flight pointing",
    GR_VETO},
  {COMMAND(elclin_veto), "veto elevation clinometer", GR_VETO},
  {COMMAND(elclin_allow), "un-veto elevation clinometer", GR_VETO},
  {COMMAND(elenc_veto), "veto elevation encoder", GR_VETO},
  {COMMAND(elenc_allow), "un-veto elevation encoder", GR_VETO},
  {COMMAND(sun_veto), "veto sun sensor", GR_VETO},
  {COMMAND(sun_allow), "un-veto sun sensor", GR_VETO},
  {COMMAND(isc_veto), "veto integrating star camera", GR_VETO},
  {COMMAND(isc_allow), "un-veto integrating star camera", GR_VETO},
  {COMMAND(osc_veto), "veto other star camera", GR_VETO},
  {COMMAND(osc_allow), "un-veto other star-cam", GR_VETO},
  {COMMAND(mag_veto), "veto magnotometer", GR_VETO},
  {COMMAND(mag_allow), "un-veto magnetometer", GR_VETO},
  {COMMAND(gps_veto), "veto differntial gps", GR_VETO},
  {COMMAND(gps_allow), "un-veto differential gps", GR_VETO},

  {COMMAND(reset_trims), "reset coarse pointing trims to zero", GR_TRIM},
  {COMMAND(trim_to_isc), "trim coarse sensors to isc", GR_TRIM},
  {COMMAND(auto_gyro), "automatically calculate gyro offsets", GR_TRIM},

  {COMMAND(clock_int), "bias clock internal", GR_BIAS},
  {COMMAND(clock_ext), "bias clock external", GR_BIAS},
  {COMMAND(bias_ac), "bias AC", GR_BIAS},
  {COMMAND(bias_dc), "bias DC", GR_BIAS},
  {COMMAND(ramp), "bias: external, ramp", GR_BIAS},
  {COMMAND(fixed), "bias: internal, fixed", GR_BIAS},

  {COMMAND(charcoal_on), "charcoal heater on", GR_CRYO_HEAT},
  {COMMAND(charcoal_off), "charcoal heater off", GR_CRYO_HEAT},
  {COMMAND(fridge_cycle), "start cycling the helium fridge", GR_CRYO_HEAT},
  {COMMAND(coldplate_on), "cold plate heater on", GR_CRYO_HEAT},
  {COMMAND(coldplate_off), "cold plate heater off", GR_CRYO_HEAT},
  {COMMAND(auto_jfetheat), "automatically reguate jfet heater level",
    GR_CRYO_HEAT},
  {COMMAND(auto_bdaheat), "automatically reguate bda heater level",
    GR_CRYO_HEAT},

  {COMMAND(cal_on), "calibrator on", GR_CALLAMP},
  {COMMAND(cal_off), "calibrator off", GR_CALLAMP},

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
  {COMMAND(l_valve_open), "set he4 AND ln tank valve direction open",
    GR_CRYO_CONTROL},
  {COMMAND(l_valve_close), "set he4 AND ln tank valve direction close",
    GR_CRYO_CONTROL},
  {COMMAND(ln_valve_on), "ln tank valve on", GR_CRYO_CONTROL},
  {COMMAND(ln_valve_off), "ln tank valve off", GR_CRYO_CONTROL},

  {COMMAND(xyzzy), "nothing happens here", GR_MISC},
  {COMMAND(sync_adc), "resync ADC Boards that require it", GR_MISC},
  {COMMAND(mcc_halt), "ask MCP to halt the MCC", GR_MISC | CONFIRM},
  {COMMAND(reap), "ask MCP to reap the watchdog tickle", GR_MISC | CONFIRM},

  {COMMAND(balance_veto), "veto balance system", GR_BAL},
  {COMMAND(balance_allow), "unveto balance system", GR_BAL},
  {COMMAND(balpump_on), "balance pump 1 on", GR_BAL},
  {COMMAND(balpump_off), "balance pump 1 off", GR_BAL},
  {COMMAND(balpump_up), "balance pump 1 reverse", GR_BAL},
  {COMMAND(balpump_down), "balance pump 1 forward", GR_BAL},
  {COMMAND(sprpump_on), "balance pump 2 on", GR_BAL},
  {COMMAND(sprpump_off), "balance pump 2 off", GR_BAL},
  {COMMAND(sprpump_fwd), "balance pump 2 forward", GR_BAL},
  {COMMAND(sprpump_rev), "balance pump 2 reverse", GR_BAL},

  {COMMAND(inner_cool_on), "inner frame cooling pump 1 on", GR_COOL},
  {COMMAND(inner_cool_off), "inner frame cooling pump 1 off", GR_COOL},
  {COMMAND(outer_cool_on), "outer frame colling pump 1 on", GR_COOL},
  {COMMAND(outer_cool_off), "outer frame colling pump 1 off", GR_COOL},
  {COMMAND(outer_spare_on), "outer frame colling pump 2 on", GR_COOL},
  {COMMAND(outer_spare_off), "outer frame colling pump 2 off", GR_COOL},

  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK | CONFIRM},
  {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
  {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},

  {COMMAND(isc_run), "start automatic image capture (normal mode)",
    GR_ISC_HOUSE},
  {COMMAND(isc_pause), "pause image capture", GR_ISC_HOUSE},
  {COMMAND(isc_reboot), "ask for software reboot of ISC computer",
    GR_ISC_HOUSE | CONFIRM},
  {COMMAND(isc_shutdown), "ask for shutdown of ISC computer", GR_ISC_HOUSE |
    CONFIRM},
  {COMMAND(isc_cam_cycle), "cycle star camera CCD power", GR_ISC_HOUSE},
  {COMMAND(isc_abort), "abort current solution attempt", GR_ISC_HOUSE},
  {COMMAND(isc_reconnect),
    "tell mcp to try and establish a new connection with ISC", GR_ISC_HOUSE},
  {COMMAND(isc_save_images), "turn on saving of images", GR_ISC_HOUSE},
  {COMMAND(isc_discard_images), "turn off saving of images", GR_ISC_HOUSE},
  {COMMAND(isc_full_screen), "show full screen", GR_ISC_HOUSE},

  {COMMAND(isc_trig_int), "tell ISC to use internal (software) triggers",
    GR_ISC_HOUSE},
  {COMMAND(isc_trig_ext), "tell ISC to use external negative pulse triggers",
    GR_ISC_HOUSE},

  {COMMAND(isc_auto_focus), "autofocus camera", GR_ISC_PARAM},

  {COMMAND(osc_run), "start automatic image capture (normal mode)",
    GR_OSC_HOUSE},
  {COMMAND(osc_pause), "pause image capture", GR_OSC_HOUSE},
  {COMMAND(osc_reboot), "ask for software reboot of OSC computer",
    GR_OSC_HOUSE | CONFIRM},
  {COMMAND(osc_shutdown), "ask for shutdown of OSC computer", GR_OSC_HOUSE |
    CONFIRM},
  {COMMAND(osc_cam_cycle), "cycle star camera CCD power", GR_OSC_HOUSE},
  {COMMAND(osc_abort), "abort current solution attempt", GR_OSC_HOUSE},
  {COMMAND(osc_reconnect),
    "tell mcp to try and establish a new connection with OSC", GR_OSC_HOUSE},
  {COMMAND(osc_save_images), "turn on saving of images", GR_OSC_HOUSE},
  {COMMAND(osc_discard_images), "turn off saving of images", GR_OSC_HOUSE},
  {COMMAND(osc_full_screen), "show full screen", GR_OSC_HOUSE},

  {COMMAND(osc_trig_int), "tell OSC to use internal (software) triggers",
    GR_OSC_HOUSE},
  {COMMAND(osc_trig_ext), "tell OSC to use external negative pulse triggers",
    GR_OSC_HOUSE},

  {COMMAND(osc_auto_focus), "autofocus camera", GR_OSC_PARAM}
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
struct mcom mcommands[N_MCOMMANDS] = {

  /***************************************/
  /********** Pointing Mode **************/
  {COMMAND(cap), "scan a circle centred on RA/Dec with el steps", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"}
    }
  },

  {COMMAND(vcap), "scan a circle centred on RA/Dec with el drift", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Drift Speed (deg el/s)", 0,  2, 'f', "NONE"}
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

  {COMMAND(vbox), "scan an az/el box centred on RA/Dec with el drift",
    GR_POINT, 6,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Drift Speed (deg el/s)", 0,  2, 'f', "NONE"}
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
  {COMMAND(isc_offset), "set offset of ISC from primary beam",
    GR_TRIM | GR_ISC_PARAM, 2,
    {
      {"X Offset (deg)", -5., 5, 'f', "ISC_X_OFF"},
      {"Y Offset (deg)", -5., 5, 'f', "ISC_Y_OFF"}
    }
  },

  {COMMAND(osc_offset), "set offset of OSC from primary beam",
    GR_TRIM | GR_OSC_PARAM, 2,
    {
      {"X Offset (deg)", -5., 5, 'f', "OSC_X_OFF"},
      {"Y Offset (deg)", -5., 5, 'f', "OSC_Y_OFF"}
    }
  },

  {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },

  {COMMAND(gyro_override), "manually set gyro offsets", GR_TRIM, 2,
    {
      {"Gyro 2 offset (deg/s)", -0.5, 0.5, 'f', "GY2_OFFSET"},
      {"Gyro 3 offset (deg/s)", -0.5, 0.5, 'f', "GY3_OFFSET"}
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
      {"Set Point (rpm)",   0, 200.0, 'f', "set_reac"},
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_pivot"}
    }
  },

  {COMMAND(back_emf), "tweak reaction wheel back-EMF compensation", GR_GAIN, 2,
    {
      {"Gain Correction", 0.1, 10, 'f', "emf_gain"},
      {"Offset Correction", -100, 100, 'f', "emf_offset"}
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
  {COMMAND(spare_level), "spare pump pwm level", GR_COOL | GR_BAL, 1,
    {
      {"Level (%)", 0, 100, 'f', "SPRPUMP_LEV"}
    }
  },

  {COMMAND(inner_level), "inner frame cooling pump speed", GR_COOL, 1,
    {
      {"Level (%)", 0, 100, 'f', "INPUMP_LEV"}
    }
  },

  {COMMAND(outer_level), "outer frame cooling pump speed", GR_COOL, 1,
    {
      {"Level (%)", 0, 100, 'f', "OUTPUMP_LEV"}
    }
  },

  /***************************************/
  /******** Electronics Heaters  *********/
  {COMMAND(t_gyro_set), "gyro box temperature set points", GR_EHEAT, 3,
    {
      {"Min Set Point (deg C)", 0, 60, 'f', "T_GY_MIN"},
      {"Max Set Point (deg C)", 0, 60, 'f', "T_GY_MAX"},
      {"Cur Set Point (deg C)", 0, 60, 'f', "T_GY_SET"}
    }
  },

  {COMMAND(t_gyro_heat), "gyro box heater levels", GR_EHEAT, 2,
    {
      {"Min Heater Level (%)", 0, 60, 'f', "GY_H_MIN"},
      {"Max Heater Level (%)", 0, 60, 'f', "GY_H_MAX"},
    }
  },

  {COMMAND(t_gyro_param), "gyro box thermostat parameters", GR_EHEAT, 2,
    {
      {"Integral Length (min)", 0.5, 60, 'f', "GY_H_TC"},
      {"Setpoint step size (deg C)", 0.1, 5, 'f', "T_GY_STEP"},
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
      {"Timeout (s)", 1, 14400, 'i', "TIMEOUT"}
    }
  },

  {COMMAND(alice_file), "set XML file for compressed (6kbit) downlink", GR_MISC,
    1,
    {
      {"File #", 0, 15, 'i', "ALICE_FILE"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {COMMAND(bias1_level), "bias 1 level", GR_BIAS, 1,
    {
      {"Level", 0, 127, 'i', "bias_lev1"}
    }
  },

  {COMMAND(bias2_level), "bias 2 level", GR_BIAS, 1,
    {
      {"Level", 0, 127, 'i', "bias_lev2"}
    }
  },

  {COMMAND(bias3_level), "bias 3 level", GR_BIAS, 1,
    {
      {"Level", 0, 127, 'i', "bias_lev3"}
    }
  },

  {COMMAND(phase), "set phase shift", GR_BIAS, 2,
    {
      {"DAS Card", 5,   16, 'i', "NONE"},
      {"Phase",    0, 2000, 'i', "NONE"}
    }
  },

  /***************************************/
  /*********** Cal Lamp  *****************/
  {COMMAND(cal_pulse), "calibrator single pulse", GR_CALLAMP, 1,
    {
      {"Pulse Length (ms)", 0, 8000, 'i', "CAL_PULSE"}
    }
  },

  {COMMAND(cal_repeat), "pulse calibrator repeatedly", GR_CALLAMP, 2,
    {
      {"Pulse Length (ms)", 10, 8000, 'i', "CAL_PULSE"},
      {"Repeat Delay (s)",  1, 32767, 'i', "CAL_REPEAT"}
    }
  },

  /***************************************/
  /********* Cryo heat   *****************/
  {COMMAND(jfet_heat), "manually set jfet heater pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "JFETPWM"}
    }
  },

  {COMMAND(jfet_set), "jfet heater setpoints", GR_CRYO_HEAT, 2,
    {
      {"On Point (K)", 0, 400., 'f', "JFET_SET_ON"},
      {"Off Point (K)", 0, 400., 'f', "JFET_SET_OFF"}
    }
  },

  {COMMAND(heatsw_heat), "heat switch pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "HSPWM"}
    }
  },

  {COMMAND(cryo_heat), "spare cryo pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "CRYOPWM"}
    }
  },

  {COMMAND(bda_heat), "manually set bda heater pwm", GR_CRYO_HEAT, 1,
    {
      {"Level (%)", 0, 100, 'f', "BDAPWM"}
    }
  },

  {COMMAND(bda_gain), "set bda heater gains", GR_CRYO_HEAT, 4,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "G_P_BDAHEAT"},
      {"Integral Gain", 0, MAX_15BIT, 'i', "G_I_BDAHEAT"},
      {"Derivative Gain", 0, MAX_15BIT, 'i', "G_D_BDAHEAT"},
      {"Integral Length", 0, MAX_15BIT, 'i', "G_FL_BDAHEAT"},
    }
  },

  {COMMAND(bda_set), "set bda heater setpoint", GR_CRYO_HEAT, 1,
    {
      {"Set Point (Counts)", 0, MAX_15BIT, 'i', "SET_BDAHEAT"}
    }
  },

  /***************************************/
  /********* ISC Commanding **************/
  {COMMAND(isc_set_focus), "set focus position", GR_ISC_PARAM, 1,
    {
      {"Focus Position", -1000, 1000, 'i', "ISC_FOCUS"}
    }
  },

  {COMMAND(isc_foc_off), "set focus offset relative to the home position",
    GR_ISC_PARAM, 1,
    {
      {"Focus Offset", -500, 2500, 'i', "ISC_FOC_OFF"}
    }
  },

  {COMMAND(isc_set_aperture), "set the f-stop", GR_ISC_PARAM, 1,
    {
      {"Aperture Position", 0, AP_RANGE, 'i', "ISC_APERT"}
    }
  },

  {COMMAND(isc_save_period), "set the time between automatically saved images",
    GR_ISC_HOUSE, 1,
    {
      {"Period (s):", 0, 1000, 'i', "ISC_SAVE_PRD"}
    }
  },

  {COMMAND(isc_pixel_centre), "centre display on pixel", GR_ISC_HOUSE, 2,
    {
      {"Pixel X", 0, ISC_CCD_X_PIXELS - 1, 'i', "NONE"},
      {"Pixel Y", 0, ISC_CCD_Y_PIXELS - 1, 'i', "NONE"}
    }
  },

  {COMMAND(isc_blob_centre), "centre display on blob", GR_ISC_HOUSE, 1,
    {
      {"Blob #", 0, MAX_ISC_BLOBS, 'i', "NONE"}
    }
  },

  {COMMAND(isc_integrate), "set camera integration times", GR_ISC_PARAM, 2,
    {
      {"fast integration time (ms)", 0, 1572.864, 'f', "ISC_FPULSE"},
      {"slow integration time (ms)", 0, 1572.864, 'f', "ISC_SPULSE"}
    }
  },

  {COMMAND(isc_det_set), "set detection parameters", GR_ISC_PARAM, 5,
    {
      {"Search Grid (px/side)",     0, ISC_CCD_Y_PIXELS, 'i', "ISC_GRID"},
      {"S/N Threshold",           0.1,       3276.7, 'f', "ISC_THRESH"},
      {"Centroiding Box (px/side)", 0, ISC_CCD_Y_PIXELS, 'i', "ISC_CENBOX"},
      {"Photometry Box (px/side)",  0, ISC_CCD_Y_PIXELS, 'i', "ISC_APBOX"},
      {"Exclusion Distance (px)",   0, ISC_CCD_Y_PIXELS, 'i', "ISC_MDIST"}
    }
  },

  {COMMAND(isc_max_blobs), "max number of blobs used in solution",
    GR_ISC_PARAM, 1,
    {
      {"# of Blobs", 0, MAX_ISC_BLOBS, 'i', "ISC_MAXBLOBS"}
    }
  },

  {COMMAND(isc_catalogue), "set catalogue retreival parameters",
    GR_ISC_PARAM, 3,
    {
      {"Magnitude Limit",            0, 12, 'f', "ISC_MAGLIMIT"},
      {"Normal Search Radius (deg)", 0, 50, 'f', "ISC_NRAD"},
      {"Lost Search Radius (deg)",   0, 50, 'f', "ISC_LRAD"}
    }
  },

  {COMMAND(isc_tolerances), "set pointing solution tolerances", GR_ISC_PARAM, 4,
    {
      {"Assoc. Tolerance (arcsec)", 0, 1000, 'f', "ISC_TOL"},
      {"Match Tolerance (%)",       0,  100, 'f', "ISC_MTOL"},
      {"Quit Tolerance (%)",        0,  100, 'f', "ISC_QTOL"},
      {"Rot. Tolerance (deg)",      0,   90, 'f', "ISC_RTOL"}
    }
  },

  {COMMAND(isc_hold_current), "set ISC stepper motor hold current",
    GR_ISC_HOUSE, 1,
    {
      {"Level (%)", 0, 50, 'i', "ISC_HOLD_I"}
    }
  },

  {COMMAND(isc_gain), "set CCD preamp gain and offset", GR_ISC_PARAM, 2,
    {
      {"Gain", 0.1, 100, 'f', "ISC_CCD_GAIN"},
      {"Offset", -4096, 4096, 'i', "ISC_CCD_OFFSET"}
    }
  },

  /***************************************/
  /********* OSC Commanding **************/
  {COMMAND(osc_set_focus), "set focus position", GR_OSC_PARAM, 1,
    {
      {"Focus Position", -1000, 1000, 'i', "OSC_FOCUS"}
    }
  },

  {COMMAND(osc_foc_off), "set focus offset relative to the home position",
    GR_OSC_PARAM, 1,
    {
      {"Focus Offset", -500, 2500, 'i', "OSC_FOC_OFF"}
    }
  },

  {COMMAND(osc_set_aperture), "set the f-stop", GR_OSC_PARAM, 1,
    {
      {"Aperture Position", 0, AP_RANGE, 'i', "OSC_APERT"}
    }
  },

  {COMMAND(osc_save_period), "set the time between automatically saved images",
    GR_OSC_HOUSE, 1,
    {
      {"Period (s):", 0, 1000, 'i', "OSC_SAVE_PRD"}
    }
  },

  {COMMAND(osc_pixel_centre), "centre display on pixel", GR_OSC_HOUSE, 2,
    {
      {"Pixel X", 0, OSC_CCD_X_PIXELS - 1, 'i', "NONE"},
      {"Pixel Y", 0, OSC_CCD_Y_PIXELS - 1, 'i', "NONE"}
    }
  },

  {COMMAND(osc_blob_centre), "centre display on blob", GR_OSC_HOUSE, 1,
    {
      {"Blob #", 0, MAX_ISC_BLOBS, 'i', "NONE"}
    }
  },

  {COMMAND(osc_integrate), "set camera integration times", GR_OSC_PARAM, 2,
    {
      {"fast integration time (ms)", 0, 1572.864, 'f', "OSC_FPULSE"},
      {"slow integration time (ms)", 0, 1572.864, 'f', "OSC_SPULSE"}
    }
  },

  {COMMAND(osc_det_set), "set detection parameters", GR_OSC_PARAM, 5,
    {
      {"Search Grid (px/side)",     0, OSC_CCD_Y_PIXELS, 'i', "OSC_GRID"},
      {"S/N Threshold",           0.1,       3276.7, 'f', "OSC_THRESH"},
      {"Centroiding Box (px/side)", 0, OSC_CCD_Y_PIXELS, 'i', "OSC_CENBOX"},
      {"Photometry Box (px/side)",  0, OSC_CCD_Y_PIXELS, 'i', "OSC_APBOX"},
      {"Exclusion Distance (px)",   0, OSC_CCD_Y_PIXELS, 'i', "OSC_MDIST"}
    }
  },

  {COMMAND(osc_max_blobs), "max number of blobs used in solution",
    GR_OSC_PARAM, 1,
    {
      {"# of Blobs", 0, MAX_ISC_BLOBS, 'i', "OSC_MAXBLOBS"}
    }
  },

  {COMMAND(osc_catalogue), "set catalogue retreival parameters",
    GR_OSC_PARAM, 3,
    {
      {"Magnitude Limit",            0, 12, 'f', "OSC_MAGLIMIT"},
      {"Normal Search Radius (deg)", 0, 50, 'f', "OSC_NRAD"},
      {"Lost Search Radius (deg)",   0, 50, 'f', "OSC_LRAD"}
    }
  },

  {COMMAND(osc_tolerances), "set pointing solution tolerances", GR_OSC_PARAM, 4,
    {
      {"Assoc. Tolerance (arcsec)", 0, 1000, 'f', "OSC_TOL"},
      {"Match Tolerance (%)",       0,  100, 'f', "OSC_MTOL"},
      {"Quit Tolerance (%)",        0,  100, 'f', "OSC_QTOL"},
      {"Rot. Tolerance (deg)",      0,   90, 'f', "OSC_RTOL"}
    }
  },

  {COMMAND(osc_hold_current), "set OSC stepper motor hold current",
    GR_OSC_HOUSE, 1,
    {
      {"Level (%)", 0, 50, 'i', "OSC_HOLD_I"}
    }
  },

  {COMMAND(osc_gain), "set CCD preamp gain and offset", GR_OSC_PARAM, 2,
    {
      {"Gain", 0.1, 100, 'f', "OSC_CCD_GAIN"},
      {"Offset", -4096, 4096, 'i', "OSC_CCD_OFFSET"}
    }
  },

  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, MAX_15BIT, 'i', "PLOVER"}
    }
  }
};
