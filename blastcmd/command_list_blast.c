/* command_list.c: BLAST command specification file
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MODIFY, OR DELETE *ANY* COMMANDS IN THIS FILE YOU *MUST*
 * RECOMPILE AND REINSTALL BLASTCMD ON ARWEN/WIDOW/!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "command_list.h"
#include "isc_protocol.h"  /* required for constants */

const char *command_list_serial = "$Revision: 4.45 $";

const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Balance",          "Waveplate Rotator",
  "Pointing Sensor Trims", "Aux. Electronics", "Bias",
  "Pointing Sensor Vetos", "Actuators",        "Unused (Cal Lamp)",
  "Pointing Motor Gains",  "Secondary Focus",  "Cryo Heat",
  "Subsystem Power",       "Lock Motor",       "Cryo Control",
  "Telemetry",             "ISC Housekeeping", "OSC Housekeeping",
  "X-Y Stage",             "ISC Modes",        "OSC Modes",
  "Miscellaneous",         "ISC Parameters",   "OSC Parameters"
};

#define COMMAND(x) x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},
  {COMMAND(antisun), "turn antisolar now", GR_POINT},

  {COMMAND(gps_off), "turn off the dGPS", GR_POWER | CONFIRM},
  {COMMAND(gps_on), "turn on the dGPS", GR_POWER},
  {COMMAND(gps_cycle), "power cycle the dGPS", GR_POWER | CONFIRM},
  {COMMAND(isc_off), "turn off the ISC", GR_POWER | CONFIRM},
  {COMMAND(isc_on), "turn on the ISC", GR_POWER},
  {COMMAND(isc_cycle), "power cycle the ISC", GR_POWER | CONFIRM},
  {COMMAND(osc_off), "turn off the OSC", GR_POWER | CONFIRM},
  {COMMAND(osc_on), "turn on the OSC", GR_POWER},
  {COMMAND(osc_cycle), "power cycle the OSC", GR_POWER | CONFIRM},
  {COMMAND(ss_off), "turn off the Sun Sensor", GR_POWER},
  {COMMAND(ss_on), "turn on the Sun Sensor", GR_POWER},
  {COMMAND(ss_cycle), "power cycle the Sun Sensor", GR_POWER},
  {COMMAND(gybox_off), "turn off the digital gyros' box", GR_POWER},
  {COMMAND(gybox_on), "turn on the digital gyros' box", GR_POWER},
  {COMMAND(gybox_cycle), "power cycle the digital gyros' box", GR_POWER},
  {COMMAND(ifroll_1_gy_off), "turn off ifroll_1_gy", GR_POWER},
  {COMMAND(ifroll_1_gy_on), "turn on ifroll_1_gy", GR_POWER},
  {COMMAND(ifroll_1_gy_cycle), "power cycle ifroll_1_gy", GR_POWER},
  {COMMAND(ifroll_2_gy_off), "turn off ifroll_2_gy", GR_POWER},
  {COMMAND(ifroll_2_gy_on), "turn on ifroll_2_gy", GR_POWER},
  {COMMAND(ifroll_2_gy_cycle), "power cycle ifroll_2_gy", GR_POWER},
  {COMMAND(ifyaw_1_gy_off), "turn off ifyaw_1_gy", GR_POWER},
  {COMMAND(ifyaw_1_gy_on), "turn on ifyaw_1_gy", GR_POWER},
  {COMMAND(ifyaw_1_gy_cycle), "power cycle ifyaw_1_gy", GR_POWER},
  {COMMAND(ifyaw_2_gy_off), "turn off ifyaw_2_gy", GR_POWER},
  {COMMAND(ifyaw_2_gy_on), "turn on ifyaw_2_gy", GR_POWER},
  {COMMAND(ifyaw_2_gy_cycle), "power cycle ifyaw_2_gy", GR_POWER},
  {COMMAND(ifel_1_gy_off), "turn off ifel_1_gy", GR_POWER},
  {COMMAND(ifel_1_gy_on), "turn on ifel_1_gy", GR_POWER},
  {COMMAND(ifel_1_gy_cycle), "power cycle ifel_1_gy", GR_POWER},
  {COMMAND(ifel_2_gy_off), "turn off ifel_2_gy", GR_POWER},
  {COMMAND(ifel_2_gy_on), "turn on ifel_2_gy", GR_POWER},
  {COMMAND(ifel_2_gy_cycle), "power cycle ifel_2_gy", GR_POWER},
  {COMMAND(actbus_off), "turn off the Actuators, Lock, and HWPR", GR_POWER 
    | GR_LOCK | GR_ACT | GR_HWPR | CONFIRM},
  {COMMAND(actbus_on), "turn on the Actuators, Lock, and HWPR", GR_POWER 
    | GR_LOCK | GR_ACT | GR_HWPR},
  {COMMAND(actbus_cycle), "power cycle the Actuators, Lock, and HWPR", GR_POWER 
    | GR_LOCK | GR_ACT | GR_HWPR | CONFIRM},
  {COMMAND(rw_off), "turn off the reaction wheel motor", GR_POWER},
  {COMMAND(rw_on), "turn on the reaction wheel motor", GR_POWER},
  {COMMAND(rw_cycle), "power cycle the reaction wheel motor", GR_POWER},
  {COMMAND(piv_off), "turn off the pivot motor", GR_POWER},
  {COMMAND(piv_on), "turn on the pivot motor", GR_POWER},
  {COMMAND(piv_cycle), "power cycle the pivot motor", GR_POWER},
  {COMMAND(elmot_off), "turn off the elevation motor", GR_POWER},
  {COMMAND(elmot_on), "turn on the elevation motor", GR_POWER},
  {COMMAND(elmot_cycle), "power cycle the elevation motor", GR_POWER},
  {COMMAND(sc_tx_off), "turn off the starcam transmitters", GR_POWER},
  {COMMAND(sc_tx_on), "turn on the starcam transmitters", GR_POWER},
  {COMMAND(bi0_off), "turn off the biphase transmitters", GR_POWER},
  {COMMAND(bi0_on), "turn on the biphase transmitters", GR_POWER},
  {COMMAND(hub232_off), "turn off the RS-232 (serial) hub", GR_POWER},
  {COMMAND(hub232_on), "turn on the RS-232 (serial) hub", GR_POWER},
  {COMMAND(hub232_cycle), "power cycle the RS-232 (serial) hub", GR_POWER},
  {COMMAND(das_off), "turn off the DAS", GR_POWER},
  {COMMAND(das_on), "turn on the DAS", GR_POWER},
  {COMMAND(das_cycle), "power cycle the DAS", GR_POWER},
  //TODO do any of these merit requiring CONFIRM ?
  {COMMAND(preamp_off), "turn off the preamp crate", GR_POWER},
  {COMMAND(preamp_on), "turn on the preamp crate", GR_POWER},
  {COMMAND(bias_off), "turn off the bias", GR_POWER},
  {COMMAND(bias_on), "turn on the bias", GR_POWER},
  {COMMAND(heat_off), "turn off the cryostat heaters", GR_POWER},
  {COMMAND(heat_on), "turn on the cryostat heaters", GR_POWER},
  {COMMAND(hk_off), "turn off the cryostat housekeepng", GR_POWER},
  {COMMAND(hk_on), "turn on the cryostat housekeepng", GR_POWER},
  {COMMAND(um250_off), "turn off the 250um array", GR_POWER},
  {COMMAND(um250_on), "turn on the 250um array", GR_POWER},
  {COMMAND(um350_off), "turn off the 350um array", GR_POWER},
  {COMMAND(um350_on), "turn on the 350um array", GR_POWER},
  {COMMAND(um500_off), "turn off the 500um array", GR_POWER},
  {COMMAND(um500_on), "turn on the 500um array", GR_POWER},

  {COMMAND(reset_rw), "reset the serial connection to the RW controller", GR_GAIN},
  {COMMAND(reset_piv), "reset the serial connection to the pivot controller", GR_GAIN},
  {COMMAND(reset_elev), "reset the serial connection to the elev controller", GR_GAIN},
  {COMMAND(restore_piv), "restore the serial settings for the pivot controller", GR_GAIN},
  {COMMAND(az_off), "disable az motors", GR_GAIN},
  {COMMAND(az_on), "enable az motors", GR_GAIN},
  {COMMAND(el_off), "disable el motors", GR_GAIN},
  {COMMAND(el_on), "enable el motors", GR_GAIN},
  {COMMAND(force_el_on), "force enable el motors despite the pin being in",
    CONFIRM | GR_GAIN},

  {COMMAND(elclin_veto), "veto elevation clinometer", GR_VETO},
  {COMMAND(elclin_allow), "un-veto elevation clinometer", GR_VETO},
  {COMMAND(elenc_veto), "veto elevation encoder", GR_VETO},
  {COMMAND(elenc_allow), "un-veto elevation encoder", GR_VETO},
  {COMMAND(gps_veto), "veto differntial gps", GR_VETO},
  {COMMAND(gps_allow), "un-veto differential gps", GR_VETO},
  {COMMAND(isc_veto), "veto integrating star camera", GR_VETO},
  {COMMAND(isc_allow), "un-veto integrating star camera", GR_VETO},
  {COMMAND(osc_veto), "veto other star camera", GR_VETO},
  {COMMAND(osc_allow), "un-veto other star-cam", GR_VETO},
  {COMMAND(mag_veto), "veto magnotometer", GR_VETO},
  {COMMAND(mag_allow), "un-veto magnetometer", GR_VETO},
  {COMMAND(sun_veto), "veto sun sensor", GR_VETO},
  {COMMAND(sun_allow), "un-veto sun sensor", GR_VETO},
  {COMMAND(ifroll_1_gy_allow), "enable ifroll_1_gy", GR_VETO},
  {COMMAND(ifroll_1_gy_veto), "disable ifroll_1_gy", GR_VETO},
  {COMMAND(ifroll_2_gy_allow), "enable ifroll_2_gy", GR_VETO},
  {COMMAND(ifroll_2_gy_veto), "disable ifroll_2_gy", GR_VETO},
  {COMMAND(ifyaw_1_gy_allow), "enable ifyaw_1_gy", GR_VETO},
  {COMMAND(ifyaw_1_gy_veto), "disable ifyaw_1_gy", GR_VETO},
  {COMMAND(ifyaw_2_gy_allow), "enable ifyaw_2_gy", GR_VETO},
  {COMMAND(ifyaw_2_gy_veto), "disable ifyaw_2_gy", GR_VETO},
  {COMMAND(ifel_1_gy_allow), "enable ifel_1_gy", GR_VETO},
  {COMMAND(ifel_1_gy_veto), "disable ifel_1_gy", GR_VETO},
  {COMMAND(ifel_2_gy_allow), "enable ifel_2_gy", GR_VETO},
  {COMMAND(ifel_2_gy_veto), "disable ifel_2_gy", GR_VETO},

  {COMMAND(az_auto_gyro), "automatically calculate az gyro offsets", GR_TRIM},
  {COMMAND(el_auto_gyro), "automatically calculate el gyro offset", GR_TRIM},
  {COMMAND(reset_trims), "reset coarse pointing trims to zero", GR_TRIM},
  {COMMAND(trim_to_isc), "trim coarse sensors to ISC", GR_TRIM},
  {COMMAND(trim_to_osc), "trim coarse sensors to OSC", GR_TRIM},
  {COMMAND(fixed), "fixed level bias", GR_BIAS},
  {COMMAND(ramp), "ramp bias with triangular waveform", GR_BIAS},

  {COMMAND(auto_jfetheat), "automatically reguate jfet heater level",
    GR_CRYO_HEAT},
  {COMMAND(charcoal_on), "charcoal heater on, helium fridge autocycle off",
    GR_CRYO_HEAT},
  {COMMAND(charcoal_off), "charcoal heater off, helium fridge autocycle off",
    GR_CRYO_HEAT},
  {COMMAND(hs_charcoal_on), "charcoal heat switch on, fridge autocycle off",
    GR_CRYO_HEAT},
  {COMMAND(hs_charcoal_off), "charcoal heat switch off, fridge autocycle off",
    GR_CRYO_HEAT},
  {COMMAND(auto_cycle), "activate helium fridge autocycle system",
    GR_CRYO_HEAT},
  {COMMAND(fridge_cycle),
    "manually cycle helium fridge now, fridge autocycle on", GR_CRYO_HEAT},
  {COMMAND(jfet_on), "manually turn JFET heater on, auto control off",
    GR_CRYO_HEAT},
  {COMMAND(jfet_off), "manually turn JFET heater off, auto control off",
    GR_CRYO_HEAT},
  {COMMAND(bda_on), "manually turn 300mK BDA heater on", GR_CRYO_HEAT},
  {COMMAND(bda_off), "manually turn 300mK BDA heater off", GR_CRYO_HEAT},
  {COMMAND(hs_pot_on), "pot heat switch on", GR_CRYO_HEAT},
  {COMMAND(hs_pot_off), "pot heat switch off", GR_CRYO_HEAT},

  {COMMAND(cal_on), "calibrator on", GR_CRYO_HEAT},
  {COMMAND(cal_off), "calibrator off", GR_CRYO_HEAT},

  {COMMAND(level_on), "helium level sensor on", GR_CRYO_CONTROL},
  {COMMAND(level_off), "helium level sensor off", GR_CRYO_CONTROL},
  {COMMAND(level_pulse), "helium level sensor pulse", GR_CRYO_CONTROL},
  {COMMAND(hwpr_enc_on), "HWP rotation sensor on", GR_CRYO_CONTROL},
  {COMMAND(hwpr_enc_off), "HWP rotation sensor off", GR_CRYO_CONTROL},
  {COMMAND(hwpr_enc_pulse), "HWP rotation sensor pulse", GR_CRYO_CONTROL},
  {COMMAND(he_valve_on), "he4 tank valve on", GR_CRYO_CONTROL},
  {COMMAND(he_valve_off), "he4 tank valve off", GR_CRYO_CONTROL},
  {COMMAND(l_valve_open), "set he4 AND ln tank valve direction open",
    GR_CRYO_CONTROL},
  {COMMAND(l_valve_close), "set he4 AND ln tank valve direction close",
    GR_CRYO_CONTROL},
  {COMMAND(ln_valve_on), "ln tank valve on", GR_CRYO_CONTROL},
  {COMMAND(ln_valve_off), "ln tank valve off", GR_CRYO_CONTROL},
  {COMMAND(pot_valve_on), "He4 pot valve on", GR_CRYO_CONTROL | CONFIRM},
  {COMMAND(pot_valve_off), "He4 pot valve off", GR_CRYO_CONTROL},
  {COMMAND(pot_valve_open), "set He4 pot valve direction open",
    GR_CRYO_CONTROL},
  {COMMAND(pot_valve_close), "set He4 pot valve direction close",
    GR_CRYO_CONTROL},

  {COMMAND(blast_rocks), "the receiver rocks, use the happy schedule file",
    GR_TELEM},
  {COMMAND(blast_sucks), "the receiver sucks, use the sad schedule file",
    GR_TELEM},
  {COMMAND(at_float),
    "tell the scheduler that we're at float (don't run initial float controls)",
    GR_TELEM},
  {COMMAND(not_at_float), "tell the scheduler that we're not at float",
    GR_TELEM},

  {COMMAND(north_halt), "ask MCP to halt north MCC", GR_MISC | CONFIRM},
  {COMMAND(south_halt), "ask MCP to halt south MCC", GR_MISC | CONFIRM},
  {COMMAND(reap_north), "ask MCP to reap the north watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(reap_south), "ask MCP to reap the south watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(xyzzy), "nothing happens here", GR_MISC},
  {COMMAND(xy_panic), "stop XY stage motors immediately", GR_STAGE},

  {COMMAND(balance_auto), "Put balance system into auto mode", GR_BAL},
  {COMMAND(balance_off),  "Turn off the balance pumps", GR_BAL},
  {COMMAND(balance_heat_on),  "Turn on the balance pump heating card", GR_BAL},
  {COMMAND(balance_heat_off), "Turn off the balance pump heating card", GR_BAL},

  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK | CONFIRM},
  {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
  {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},
  {COMMAND(repoll), "force repoll of the stepper busses (act, lock, HWPR, XY)",
    GR_STAGE | GR_LOCK | GR_ACT | GR_HWPR},
  {COMMAND(autofocus_veto), "veto the secondary actuator system temperature"
    " correction mode", GR_FOCUS},
  {COMMAND(autofocus_allow), "allow the secondary actuator system temperature"
    " correction mode", GR_FOCUS},
  {COMMAND(actuator_stop), "stop all secondary actuators immediately", GR_ACT},
  {COMMAND(reset_dr), "reset the actuator dead reckoning", GR_ACT},
  {COMMAND(actpos_trim), "trim the actuator positions to the encoders", GR_ACT},
  {COMMAND(hwpr_panic), "stop the HWPR rotator immediately", GR_HWPR},

  {COMMAND(isc_abort), "abort current solution attempt", GR_ISC_MODE},
  {COMMAND(isc_auto_focus), "autofocus camera", GR_ISC_MODE},
  {COMMAND(isc_cam_cycle), "cycle star camera CCD power", GR_ISC_MODE},
  {COMMAND(isc_discard_images), "turn off saving of images", GR_ISC_MODE},
  {COMMAND(isc_pause), "pause image capture", GR_ISC_MODE},
  {COMMAND(isc_reboot), "ask for software reboot of ISC computer",
    GR_ISC_MODE | CONFIRM},
  {COMMAND(isc_run), "start automatic image capture (normal mode)",
    GR_ISC_MODE},
  {COMMAND(isc_save_images), "turn on saving of images", GR_ISC_MODE},
  {COMMAND(isc_shutdown), "ask for shutdown of ISC computer", GR_ISC_MODE |
    CONFIRM},

  {COMMAND(isc_eye_on), "turn on ISC eViL eYe", GR_ISC_HOUSE},
  {COMMAND(isc_eye_off), "turn off ISC eViL eYe", GR_ISC_HOUSE},
  {COMMAND(isc_full_screen), "show full screen", GR_ISC_HOUSE},
  {COMMAND(isc_reconnect),
    "tell mcp to try and establish a new connection with ISC", GR_ISC_HOUSE},
  {COMMAND(isc_trig_ext), "tell ISC to use external negative pulse triggers",
    GR_ISC_HOUSE},
  {COMMAND(isc_trig_int), "tell ISC to use internal (software) triggers",
    GR_ISC_HOUSE},
  {COMMAND(isc_use_pyramid), "tell ISC to use the pyramid solution finder",
    GR_ISC_PARAM},
  {COMMAND(isc_no_pyramid), "tell ISC not to use the pyramid solution finder",
    GR_ISC_PARAM},


  {COMMAND(osc_abort), "abort current solution attempt", GR_OSC_MODE},
  {COMMAND(osc_auto_focus), "autofocus camera", GR_OSC_MODE},
  {COMMAND(osc_cam_cycle), "cycle star camera CCD power", GR_OSC_MODE},
  {COMMAND(osc_discard_images), "turn off saving of images", GR_OSC_MODE},
  {COMMAND(osc_pause), "pause image capture", GR_OSC_MODE},
  {COMMAND(osc_reboot), "ask for software reboot of OSC computer",
    GR_OSC_MODE | CONFIRM},
  {COMMAND(osc_run), "start automatic image capture (normal mode)",
    GR_OSC_MODE},
  {COMMAND(osc_save_images), "turn on saving of images", GR_OSC_MODE},
  {COMMAND(osc_shutdown), "ask for shutdown of OSC computer", GR_OSC_MODE |
    CONFIRM},

  {COMMAND(osc_eye_on), "turn on OSC eViL eYe", GR_OSC_HOUSE},
  {COMMAND(osc_eye_off), "turn off OSC eViL eYe", GR_OSC_HOUSE},
  {COMMAND(osc_full_screen), "show full screen", GR_OSC_HOUSE},
  {COMMAND(osc_reconnect),
    "tell mcp to try and establish a new connection with OSC", GR_OSC_HOUSE},
  {COMMAND(osc_trig_ext), "tell OSC to use external negative pulse triggers",
    GR_OSC_HOUSE},
  {COMMAND(osc_trig_int), "tell OSC to use internal (software) triggers",
    GR_OSC_HOUSE},
  {COMMAND(osc_use_pyramid), "tell OSC to use the pyramid solution finder",
    GR_OSC_PARAM},
  {COMMAND(osc_no_pyramid), "tell OSC not to use the pyramid solution finder",
    GR_OSC_PARAM},
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * l :  parameter is 30 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * d :  parameter is 30 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
struct mcom mcommands[N_MCOMMANDS] = {
  /* TODO delete when done with these. Temporary commands for dac testing */
  /*
  {COMMAND(dac1_level), "DAC1 output level. Temporary", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "ifpm_ampl"}
    }
  },
  */
  {COMMAND(dac2_level), "DAC2 output level. Temporary", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "dac2_ampl"}
    }
  },
  {COMMAND(dac3_level), "DAC3 output level. Temporary", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "dac3_ampl"}
    }
  },
  {COMMAND(dac4_level), "DAC4 output level. Temporary", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "dac4_ampl"}
    }
  },
  {COMMAND(dac5_level), "DAC5 output level. Temporary", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "dac5_ampl"}
    }
  },

  {COMMAND(alice_file), "set XML file for compressed (6kbit) downlink",
    GR_TELEM, 1,
    {
      {"File #", 0, 15, 'i', "ALICE_FILE"}
    }
  },

  /* pointing modes */
  {COMMAND(az_el_goto), "goto point in azimuth and elevation", GR_POINT, 2,
    {
      {"Azimuth (deg)", -360, 360, 'f', "NONE"},
      {"Elevation (deg)", 4.95,  65, 'f', "NONE"}
    }
  },
  {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },
  {COMMAND(az_gain), "az reaction wheel gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_az"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_az"}
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
  {COMMAND(balance_manual), "Manually set balance pump rate", GR_BAL, 1,
    {
      {"level",           -1.0, 1.0, 'f', "NONE"},
    }
  },
  {COMMAND(balance_gain), "Set balance system setpoints", GR_BAL, 4,
    {
      {"Pump On Point (A)",  0, 2, 'f', "LEVEL_ON_BAL"},
      {"Pump Off Point (A)", 0, 2, 'f', "LEVEL_OFF_BAL"},
      {"Target (A)",        -2, 2, 'f', "LEVEL_TARGET_BAL"},
      {"Gain",            0.01, 10, 'f', "GAIN_BAL"},
    }
  },
  {COMMAND(balance_veto), "Set veto for the balance system", GR_BAL, 1,
    {
      {"Balance veto (s)",  0, 60000, 'f', "VETO_BAL"},
    }
  },
  {COMMAND(balance_tset), "Set balance pump minumum temperature", GR_BAL, 1,
     {
       {"Temperature (C)",  -273.4, 40, 'f', "T_BOX_BAL"},
     }
  },
  {COMMAND(box), "scan an az/el box centred on RA/Dec with el steps",
    GR_POINT, 6,
    {
      {"RA of Centre (h)",          0, 24, 'd', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "NONE"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"}
    }
  },
  {COMMAND(cap), "scan a circle centred on RA/Dec with el steps", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'd', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
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
  {COMMAND(quad), "scan a quadrilateral region in RA/Dec (corners must be "
    "ordered)", GR_POINT, 10,
    {
      {"RA of Corner 1 (h)",        0, 24, 'f', "NONE"},
      {"Dec of Corner 1 (deg)",   -90, 90, 'f', "NONE"},
      {"RA of Corner 2 (h)",        0, 24, 'f', "NONE"},
      {"Dec of Corner 2 (deg)",   -90, 90, 'f', "NONE"},
      {"RA of Corner 3 (h)",        0, 24, 'f', "NONE"},
      {"Dec of Corner 3 (deg)",   -90, 90, 'f', "NONE"},
      {"RA of Corner 4 (h)",        0, 24, 'f', "NONE"},
      {"Dec of Corner 4 (deg)",   -90, 90, 'f', "NONE"},
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
  {COMMAND(vcap), "scan a circle centred on RA/Dec with el drift", GR_POINT, 5,
    {
      {"RA of Centre (h)",          0, 24, 'f', "NONE"},
      {"Dec of Centre (deg)",     -90, 90, 'f', "NONE"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Drift Speed (deg el/s)", 0,  2, 'f', "NONE"}
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
  {COMMAND(pivot_gain), "pivot gains", GR_GAIN, 3,
    {
      {"Set Point (dps)",   -200, 200, 'f', "SET_RW"},
      {"V_err Gain (prop)", 0, MAX_15BIT, 'i', "G_PE_PIVOT"},
      {"V_RW Gain (prop)", 0, MAX_15BIT, 'i', "G_PV_PIVOT"},
      {"Static Friction offset",   0, 2, 'f', "FRICT_OFF_PIV"},
    }
  },
  {COMMAND(el_gain), "elevation motor gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_el"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_el"}
    }
  },
  {COMMAND(az_gyro_offset), "manually set az gyro offsets", GR_TRIM, 2,
    {
      {"Gyro 2 offset (deg/s)", -0.5, 0.5, 'f', "GY2_OFFSET"},
      {"Gyro 3 offset (deg/s)", -0.5, 0.5, 'f', "GY3_OFFSET"}
    }
  },
  {COMMAND(el_gyro_offset), "manually set el gyro offset", GR_TRIM, 1,
    {
      {"Gyro 1 offset (deg/s)", -0.5, 0.5, 'f', "GY1_OFFSET"},
    }
  },
  {COMMAND(slew_veto), "set the length of the gyro offset slew veto", GR_TRIM,
    1,
    {
      {"Slew Veto (s)", 0., 1200., 'f', "SVETO_LEN"},
    }
  },

  /* actuator bus commands */
  {COMMAND(lock), "lock inner frame", GR_LOCK | GR_POINT, 1,
    {
      {"Lock Elevation (deg)", 5, 90, 'f', "ENC_ELEV"}
    }
  },
  {COMMAND(general), "send a general command string to the lock or actuators",
    GR_STAGE | GR_ACT | GR_LOCK | GR_HWPR, 2,
    {
      {"Address (1-3,5,33)", 1, 0x2F, 'i', "1.0"},
      {"Command", 0, 32, 's', ""},
    }
  },
  {COMMAND(lock_vel), "set the lock motor velocity and acceleration", GR_LOCK,
    2,
    {
      {"Velocity", 5, 500000, 'l', "LOCK_VEL"},
      {"Acceleration", 1, 1000, 'i', "LOCK_ACC"},
    }
  },
  {COMMAND(lock_i), "set the lock motor currents", GR_LOCK, 2,
    {
      {"Move current (%)", 0, 100, 'i', "LOCK_MOVE_I"},
      {"Hold current (%)", 0,  50, 'i', "LOCK_HOLD_I"},
    }
  },
  {COMMAND(set_secondary), "servo the secondary mirror to absolute position",
    GR_FOCUS, 1,
    {
      {"Position (counts)", -15000, 15000, 'i', "ABS_FOCUS"},
    }
  },
  {COMMAND(delta_secondary), "servo the secondary mirror by a relative amount",
    GR_FOCUS, 1,
    {
      {"Position (counts)", -1000, 1000, 'i', "0"},
    }
  },
  {COMMAND(thermo_gain), "set the secondary actuator system gains", GR_FOCUS, 5,
    {
      {"T. Primary Gain",   1, 1000, 'f', "TC_G_PRIM"},
      {"T. Secondary Gain", 1, 1000, 'f', "TC_G_SEC"},
      {"Step Size (um)",   10, 1000, 'i', "TC_STEP"},
      {"Step Wait (min)"  , 0, 1500, 'i', "TC_WAIT"},
      {"Filter Length (s)", 0, 6000, 'i', "TC_FILTER"}
    }
  },
  {COMMAND(actuator_servo), "servo the actuators to absolute positions",
    GR_ACT, 3,
    {
      {"Actuator Alpha", -15000, 15000, 'i', "ACT0_ENC"},
      {"Actuator Beta",  -15000, 15000, 'i', "ACT1_ENC"},
      {"Actuator Gamma", -15000, 15000, 'i', "ACT2_ENC"}
    }
  },
  {COMMAND(focus_offset), "set the in focus position offset relative the "
    "nominal focus", GR_FOCUS, 1,
    {
      {"Offset", -5000, 25000, 'i', "SF_OFFSET"}
    }
  },
  {COMMAND(actuator_delta), "offset the actuators to from current position",
    GR_ACT, 3,
    {
      {"Actuator Alpha", -1000, 1000, 'i', "0"},
      {"Actuator Beta",  -1000, 1000, 'i', "0"},
      {"Actuator Gamma", -1000, 1000, 'i', "0"}
    }
  },
  {COMMAND(encoder_offset), "set the current actuator encoder offset",
    GR_ACT, 3,
    {
      {"Actuator Alpha", 0, 30000, 'i', "ACT0_ENC"},
      {"Actuator Beta",  0, 30000, 'i', "ACT1_ENC"},
      {"Actuator Gamma", 0, 30000, 'i', "ACT2_ENC"}
    }
  },
  {COMMAND(actuator_vel), "set the actuator velocity and acceleration", GR_ACT,
    2,
    {
      {"Velocity", 5, 20000, 'i', "ACT_VEL"},
      {"Acceleration", 1, 20, 'i', "ACT_ACC"},
    }
  },
  {COMMAND(actuator_i), "set the actuator motor currents", GR_ACT, 2,
    {
      {"Move current (%)", 0, 100, 'i', "ACT_MOVE_I"},
      {"Hold current (%)", 0,  50, 'i', "ACT_HOLD_I"},
    }
  },
  {COMMAND(lvdt_limit), "set the hard LVDT limits on actuator moves", GR_ACT, 3,
    {
      {"Spread limit", 0, 3000, 'f', "LVDT_SPREAD"},
      {"Lower limit", -6000, 26000, 'f', "LVDT_LOW"},
      {"Upper limit", -6000, 26000, 'f', "LVDT_HIGH"}
    }
  },
  {COMMAND(thermo_param), "set the thermal compensation parameters", GR_FOCUS,
    3,
    {
      {"Temp. Spread", 0, 100, 'f', "TC_SPREAD"},
      {"Preferred T Prime", 0, 2, 'i', "TC_PREF_TP"},
      {"Preferred T Second", 0, 2, 'i', "TC_PREF_TS"}
    }
  },
  {COMMAND(hwpr_vel), "set the wavepalte rotator velocity and acceleration", 
    GR_HWPR, 2,
    {
      {"Velocity", 5, 500000, 'l', "HWPR_VEL"},
      {"Acceleration", 1, 1000, 'i', "HWPR_ACC"},
    }
  },
  {COMMAND(hwpr_i), "set the wavepalte rotator currents", GR_HWPR, 2,
    {
      {"Move current (%)", 0, 100, 'i', "HWPR_MOVE_I"},
      {"Hold current (%)", 0,  50, 'i', "HWPR_HOLD_I"},
    }
  },
  {COMMAND(hwpr_goto), "move the waveplate rotator to absolute position",
    GR_HWPR, 1,
    {
      //TODO calibrate hwpr move units
      {"destination", 0, 80000, 'l', "STAGE_X"}
    }
  },
  {COMMAND(hwpr_jump), "move the waveplate rotator to relative position",
    GR_HWPR, 1,
    {
      {"delta", -80000, 80000, 'l', "0"}
    }
  },

  /* XY Stage */
  {COMMAND(xy_goto), "move the X-Y translation stage to absolute position",
    GR_STAGE, 4,
    {
      {"X destination", 0, 80000, 'l', "STAGE_X"},
      {"Y destination", 0, 80000, 'l', "STAGE_Y"},
      {"X speed", 0, 16000, 'i', "STAGE_X_VEL"},
      {"Y speed", 0, 16000, 'i', "STAGE_Y_VEL"}
    }
  },
  {COMMAND(xy_jump), "move the X-Y translation stage to relative position",
    GR_STAGE, 4,
    {
      {"X delta", -80000, 80000, 'l', "0"},
      {"Y delta", -80000, 80000, 'l', "0"},
      {"X speed", 0, 16000, 'i', "STAGE_X_VEL"},
      {"Y speed", 0, 16000, 'i', "STAGE_Y_VEL"}
    }
  },
  {COMMAND(xy_xscan), "scan the X-Y translation stage in X", GR_STAGE, 3,
    {
      {"X center", 0, 80000, 'l', "STAGE_X"},
      {"delta X", 0, 80000, 'l', "NONE"},
      {"X speed", 0, 16000, 'i', "STAGE_X_VEL"},
    }
  },
  {COMMAND(xy_yscan), "scan the X-Y translation stage in Y", GR_STAGE, 3,
    {
      {"Y center", 0, 80000, 'l', "STAGE_Y"},
      {"delta Y", 0, 80000, 'l', "NONE"},
      {"Y speed", 0, 16000, 'i', "STAGE_Y_VEL"},
    }
  },
  {COMMAND(xy_raster), "raster the X-Y translation stage", GR_STAGE, 5,
    {
      {"X center", 0, 80000, 'l', "STAGE_X"},
      {"Y center", 0, 80000, 'l', "STAGE_Y"},
      {"Map Size", 0, 30000, 'i', "NONE"},
      {"Step Size", 0, 30000, 'i', "NONE"},
      {"Velocity", 0, 16000, 'i', "STAGE_X_VEL"},
    }
  },

  /*******************************************************/
  /*************** Telemetry/Scheduling  *****************/
  {COMMAND(timeout), "time until schedule mode", GR_TELEM, 1,
    {
      {"Timeout (s)", 2, 65535, 'f', "TIMEOUT"}
    }
  },

  /****************************************/
  /*************** Misc.  *****************/
  {COMMAND(reset_adc), "Reset an ADC motherboard", GR_POWER, 1,
    {
      {"Node number",  0, 64, 'i', ""}
    }
  },
  {COMMAND(t_gyro_gain), "gyro box heater gains", GR_ELECT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_gyheat"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_gyheat"},
      {"Derrivative Gain",  0, MAX_15BIT, 'i', "g_d_gyheat"}
    }
  },
  {COMMAND(t_gyro_set), "gyro box temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_GY_SET"}
    }
  },
  {COMMAND(auto_apcu), "Automatically control the ACS PCU charge voltage",
    GR_ELECT, 1,
    {
      {"Trim (V)", -1, 1, 'f', "APCU_TRIM"}
    }
  },
  {COMMAND(apcu_charge), "Set the ACS PCU battery charge level", GR_ELECT, 1,
    {
      {"Level (V)", 27.25, 31.06, 'f', "APCU_REG"}
    }
  },
  {COMMAND(auto_dpcu), "Automatically control the DAS PCU charge voltage",
    GR_ELECT, 1,
    {
      {"Trim (V)", -1, 1, 'f', "DPCU_TRIM"}
    }
  },
  {COMMAND(dpcu_charge), "Set the DAS PCU battery charge level", GR_ELECT, 1,
    {
      {"Level (V)", 27.25, 31.09, 'f', "DPCU_REG"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {COMMAND(bias_level_250), "bias level 250 micron", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "bias_ampl_250"}
    }
  },
  {COMMAND(bias_level_350), "bias level 350 micron", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "bias_ampl_350"}
    }
  },
  {COMMAND(bias_level_500), "bias level 500 micron", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "bias_ampl_500"}
    }
  },
  {COMMAND(bias_step), "step through different bias levels", GR_BIAS, 6,
    {
      {"Start", 0, 32767, 'i', "bias_step_start"},
      {"End", 0, 32767, 'i', "bias_step_end"},
      {"N steps", 1, 32767, 'i', "bias_step_nsteps"},
      {"Time per step (ms)", 10, 32767, 'i', "bias_step_time"},
      {"Cal pulse length (ms)", 0, 32767, 'i', "bias_step_pul_len"},
      {"Array (250,350,500,0=all)", 0, 32767, 'i', "bias_step_array"},
    }
  },
  {COMMAND(phase_step), "step through different phases", GR_BIAS, 4,
    {
      {"Start", 0, 32767, 'i', "phase_step_start"},
      {"End", 0, 32767, 'i', "phase_step_end"},
      {"N steps", 1, 32767, 'i', "phase_step_nsteps"},
      {"Time per step (ms)", 1, 32767, 'i', "phase_step_time"},
    }
  },
  {COMMAND(bias_level_rox), "bias level ROX", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "bias_ampl_rox"}
    }
  },
  {COMMAND(bias_level_x), "bias level X (unused)", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "bias_ampl_x"}
    }
  },
  {COMMAND(phase), "set phase shift", GR_BIAS, 2,
    {
      {"DAS Card (0=all)", 0, 63, 'i', "NONE"},
      {"Phase",            0, 32767, 'i', "NONE"}
    }
  },

  /***************************************/
  /*********** Cal Lamp  *****************/
  {COMMAND(cal_pulse), "calibrator single pulse", GR_CRYO_HEAT, 1,
    {
      {"Pulse Length (ms)", 0, 8000, 'i', "CAL_PULSE"}
    }
  },
  {COMMAND(cal_repeat), "pulse calibrator repeatedly", GR_CRYO_HEAT, 2,
    {
      {"Pulse Length (ms)", 10, 8000, 'i', "CAL_PULSE"},
      {"Repeat Delay (s)",  1, 32767, 'i', "CAL_REPEAT"}
    }
  },

  /***************************************/
  /********* Cryo heat   *****************/
  {COMMAND(jfet_set), "jfet heater setpoints", GR_CRYO_HEAT, 2,
    {
      {"On Point (K)", 0, 400., 'f', "JFET_SET_ON"},
      {"Off Point (K)", 0, 400., 'f', "JFET_SET_OFF"}
    }
  },

  /***************************************/
  /********* ISC Commanding **************/
  {COMMAND(isc_offset), "set offset of ISC to primary beam",
    GR_TRIM | GR_ISC_PARAM, 2,
    {
      {"X Offset (deg)", -5., 5, 'f', "ISC_X_OFF"},
      {"Y Offset (deg)", -5., 5, 'f', "ISC_Y_OFF"}
    }
  },
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
  {COMMAND(isc_det_set), "set detection parameters", GR_ISC_PARAM, 3,
    {
      {"Search Grid (px/side)",     0, ISC_CCD_Y_PIXELS, 'i', "ISC_GRID"},
      {"S/N Threshold",           0.1,       3276.7, 'f', "ISC_THRESH"},
      {"Exclusion Distance (px)",   0, ISC_CCD_Y_PIXELS, 'i', "ISC_MDIST"}
    }
  },
  {COMMAND(isc_blobs), "number of blobs used in solution",
    GR_ISC_PARAM, 2,
    {
      {"Min Blobs", 0, MAX_ISC_BLOBS, 'i', "ISC_MINBLOBS"},
      {"Max Blobs", 0, MAX_ISC_BLOBS, 'i', "ISC_MAXBLOBS"}
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

  {COMMAND(isc_max_age), "set maximum delay between trigger and solution (ms)", GR_ISC_PARAM, 1,
    {
      {"Max Age", 0, MAX_15BIT, 'i', "ISC_MAX_AGE"},
    }
  },

  /***************************************/
  /********* OSC Commanding **************/
  {COMMAND(osc_offset), "set offset of OSC to primary beam",
    GR_TRIM | GR_OSC_PARAM, 2,
    {
      {"X Offset (deg)", -5., 5, 'f', "OSC_X_OFF"},
      {"Y Offset (deg)", -5., 5, 'f', "OSC_Y_OFF"}
    }
  },
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
  {COMMAND(osc_det_set), "set detection parameters", GR_OSC_PARAM, 3,
    {
      {"Search Grid (px/side)",     0, OSC_CCD_Y_PIXELS, 'i', "OSC_GRID"},
      {"S/N Threshold",           0.1,       3276.7, 'f', "OSC_THRESH"},
      {"Exclusion Distance (px)",   0, OSC_CCD_Y_PIXELS, 'i', "OSC_MDIST"}
    }
  },
  {COMMAND(osc_blobs), "number of blobs used in solution",
    GR_OSC_PARAM, 2,
    {
      {"Min Blobs", 0, MAX_ISC_BLOBS, 'i', "OSC_MINBLOBS"},
      {"Max Blobs", 0, MAX_ISC_BLOBS, 'i', "OSC_MAXBLOBS"}
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
  {COMMAND(osc_max_age), "set maximum delay between trigger and solution (ms)", GR_OSC_PARAM, 1,
    {
      {"Max Age", 0, MAX_15BIT, 'i', "OSC_MAX_AGE"},
    }
  },
  {COMMAND(motors_verbose), "Set verbosity of motor serial threads (0=norm, 1=verbose, 2= superverbose )", GR_MISC, 3,
   {
     {"Reaction Wheel", 0, 5, 'i', "VERBOSE_RW"},
     {"Elevation", 0, 5, 'i', "VERBOSE_EL"},
     {"Pivot", 0, 5, 'i', "VERBOSE_PIV"}
   }
  },

  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, MAX_15BIT, 'i', "PLOVER"}
    }
  }
};
