
/* command_list.c: BLAST command specification file
 *
 * This software is copyright (C) 2002-20010 University of Toronto
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

const char *command_list_serial = "$Revision: 5.2 $";

const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Balance",          "Waveplate Rotator",
  "Pointing Sensor Trims", "Aux. Electronics", "Bias",
  "Pointing Sensor Vetos", "Actuators",        "SBSC",
  "Pointing Motor Gains",  "Secondary Focus",  "Cryo Heat",
  "Subsystem Power",       "Lock Motor",       "Cryo Control",
  "Telemetry",             "ISC Housekeeping", "OSC Housekeeping",
  "X-Y Stage",             "ISC Modes",        "OSC Modes",
  "Miscellaneous",         "ISC Parameters",   "OSC Parameters",
  "Shutter"
  };

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},
  {COMMAND(antisun), "turn antisolar now", GR_POINT},

  {COMMAND(xsc0_off), "turn off XSC0", GR_ISC_MODE | GR_POWER | CONFIRM},
  {COMMAND(xsc0_on), "turn on XSC0", GR_ISC_MODE | GR_POWER},
  {COMMAND(xsc0_cycle), "power cycle XSC0", GR_ISC_MODE | GR_POWER | CONFIRM},
  {COMMAND(xsc1_off), "turn off XSC1", GR_OSC_MODE | GR_POWER | CONFIRM},
  {COMMAND(xsc1_on), "turn on XSC1", GR_OSC_MODE | GR_POWER},
  {COMMAND(xsc1_cycle), "power cycle XSC1", GR_OSC_MODE | GR_POWER | CONFIRM},
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
  {COMMAND(vtx_off), "turn off the video transmitters", GR_TELEM | GR_POWER},
  {COMMAND(vtx_on), "turn on the video transmitters", GR_TELEM | GR_POWER},
  {COMMAND(bi0_off), "turn off the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(bi0_on), "turn on the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(hub232_off), "turn off the RS-232 (serial) hub", GR_POWER},
  {COMMAND(hub232_on), "turn on the RS-232 (serial) hub", GR_POWER},
  {COMMAND(hub232_cycle), "power cycle the RS-232 (serial) hub", GR_POWER},
  {COMMAND(das_off), "turn off the DAS", GR_POWER},
  {COMMAND(das_on), "turn on the DAS", GR_POWER},
  {COMMAND(das_cycle), "power cycle the DAS", GR_POWER},
  {COMMAND(rx_off), "receiver/preamp crate Make it Not-So!", GR_POWER},
  {COMMAND(rx_on), "receiver/preamp crate Make it So!", GR_POWER},
  {COMMAND(rx_hk_off), "cryostat housekeepng Make it Not-So!", GR_POWER},
  {COMMAND(rx_hk_on), "cryostat housekeepng Make it So!", GR_POWER},
  {COMMAND(rx_amps_off), "receiver amplifiers Make it Not-So!", GR_POWER},
  {COMMAND(rx_amps_on), "receiver amplifiers Make it So!", GR_POWER},
  {COMMAND(charge_off), "turn off the charge controller", GR_POWER | CONFIRM},
  {COMMAND(charge_on), "turn on the charge controller", GR_POWER},
  {COMMAND(charge_cycle), "power cycle the charge controller", 
    GR_POWER | CONFIRM},

  {COMMAND(reset_rw), "reset the serial connection to the RW controller", GR_GAIN},
  {COMMAND(reset_piv), "reset the serial connection to the pivot controller", GR_GAIN},
  {COMMAND(reset_elev), "reset the serial connection to the elev controller", GR_GAIN},
  {COMMAND(restore_piv), "restore the serial settings for the pivot controller", GR_GAIN},
  {COMMAND(az_off), "disable az motors' gains", GR_GAIN},
  {COMMAND(az_on), "enable az motors' gains", GR_GAIN},
  {COMMAND(el_off), "disable el motor gains", GR_GAIN},
  {COMMAND(el_on), "enable el motor gains", GR_GAIN},
  {COMMAND(force_el_on), "force enable el motors despite the pin being in",
    CONFIRM | GR_GAIN},

  {COMMAND(elclin_veto), "veto elevation clinometer", GR_VETO},
  {COMMAND(elclin_allow), "un-veto elevation clinometer", GR_VETO},
  {COMMAND(elenc_veto), "veto elevation encoder", GR_VETO},
  {COMMAND(elenc_allow), "un-veto elevation encoder", GR_VETO},
  {COMMAND(elmotenc_veto), "veto elevation motor encoder", GR_VETO},
  {COMMAND(elmotenc_allow), "un-veto elevation motor encoder", GR_VETO},
  {COMMAND(xsc0_veto), "veto star camera 0", GR_VETO},
  {COMMAND(xsc0_allow), "un-veto star camera 0", GR_VETO},
  {COMMAND(xsc1_veto), "veto star camera 1", GR_VETO},
  {COMMAND(xsc1_allow), "un-veto star camera 1", GR_VETO},
  {COMMAND(mag_veto), "veto magnotometer", GR_VETO},
  {COMMAND(mag_allow), "un-veto magnetometer", GR_VETO},
  {COMMAND(pss_veto), "veto pss sensor", GR_VETO},
  {COMMAND(pss_allow), "un-veto pss sensor", GR_VETO},
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
  {COMMAND(trim_osc_to_isc), "trim OSC to ISC", GR_TRIM},
  {COMMAND(trim_isc_to_osc), "trim ISC to OSC", GR_TRIM},
  {COMMAND(autotrim_off), "disable auto-trim to ISC/OSC", GR_TRIM},
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
  {COMMAND(hwpr_enc_on), "HWP rotation sensor on", GR_CRYO_CONTROL | GR_HWPR},
  {COMMAND(hwpr_enc_off), "HWP rotation sensor off", GR_CRYO_CONTROL | GR_HWPR},
  {COMMAND(hwpr_enc_pulse), "HWP rotation sensor pulse", GR_CRYO_CONTROL | GR_HWPR},
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
  {COMMAND(vtx1_isc), "put ISC video on transmitter #1", GR_TELEM},
  {COMMAND(vtx1_osc), "put OSC video on transmitter #1", GR_TELEM},
  {COMMAND(vtx2_isc), "put ISC video on transmitter #2", GR_TELEM},
  {COMMAND(vtx2_osc), "put OSC video on transmitter #2", GR_TELEM},

  {COMMAND(north_halt), "ask MCP to halt north MCC", GR_MISC | CONFIRM},
  {COMMAND(south_halt), "ask MCP to halt south MCC", GR_MISC | CONFIRM},
  {COMMAND(reap_north), "ask MCP to reap the north watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(reap_south), "ask MCP to reap the south watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(xy_panic), "stop XY stage motors immediately", GR_STAGE},

  {COMMAND(balance_auto), "Put balance system into auto mode", GR_BAL},
  {COMMAND(balance_off),  "Turn off the balance pumps", GR_BAL},
  {COMMAND(balance_heat_on),  "Turn on the balance pump heating card", GR_BAL},
  {COMMAND(balance_heat_off), "Turn off the balance pump heating card", GR_BAL},

  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK | CONFIRM},
  {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
  {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},
  {COMMAND(lock45), "Lock the inner frame at 45 degrees", GR_LOCK},
  {COMMAND(repoll), "force repoll of the stepper busses (act, lock, HWPR, XY)",
    GR_STAGE | GR_LOCK | GR_ACT | GR_HWPR},
  {COMMAND(autofocus_veto), "veto the secondary actuator system temperature"
    " correction mode", GR_FOCUS},
  {COMMAND(autofocus_allow), "allow the secondary actuator system temperature"
    " correction mode", GR_FOCUS},
  {COMMAND(actuator_stop), "stop all secondary actuators immediately", GR_ACT},
  {COMMAND(hwpr_panic), "stop the HWPR rotator immediately", GR_HWPR},
  {COMMAND(hwpr_step), "step the hwpr", GR_HWPR},
  {COMMAND(hwpr_step_off), "Disable half wave plate stepping", GR_HWPR},
  {COMMAND(hwpr_step_on), "Enable half wave plate stepping", GR_HWPR},
  {COMMAND(hwpr_pot_is_dead), "don't use the potentiometer when stepping the hwpr", GR_HWPR},
  {COMMAND(hwpr_pot_is_alive), "use the potentiometer when stepping the hwpr", GR_HWPR},

  //Shutter commands
  {COMMAND(shutter_init), "Initialize shutter move parameters", GR_SHUTTER},
  {COMMAND(shutter_close), "Close shutter and keep it closed", GR_SHUTTER},
  {COMMAND(shutter_reset), "Reset shutter; shutter will open", GR_SHUTTER},
  {COMMAND(shutter_open), "Open shutter", GR_SHUTTER},
  {COMMAND(shutter_open_close), "If shutter is open, then open completely and then close", GR_SHUTTER},
  {COMMAND(shutter_off), "Turn off shutter; shutter will fall open", GR_SHUTTER},
  {COMMAND(shutter_close_slow), "Close shutter using opto feedback and keep it closed", GR_SHUTTER},
  {COMMAND(xyzzy), "nothing happens here", GR_MISC}

};

/* parameter type:
 * i :  parameter is 16 bit unnormalised integer. Max is CMD_I_MAX
 * l :  parameter is 32 bit unnormalised integer. Max is CMD_L_MAX
 * f :  parameter is 16 bit renormalised floating point
 * d :  parameter is 32 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
struct mcom mcommands[N_MCOMMANDS] = {

  {COMMAND(slot_sched), "set uplinked slot to use for schedule file",
    GR_TELEM, 1,
    {
      {"Slot #", 0, 250, 'i', "SLOT_SCHED"}
    }
  },

  /* pointing modes */
  {COMMAND(az_el_goto), "goto point in azimuth and elevation", GR_POINT, 2,
    {
      {"Azimuth (deg)", -360, 360, 'f', "AZ"},
      {"Elevation (deg)", 4.95,  65, 'f', "EL"}
    }
  },
  {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },
  {COMMAND(mag_cal), "set magnetometer calibration", GR_TRIM, 4,
    {
      {"Max X", 0, 65535, 'i', "cal_xmax_mag"},
      {"Min X", 0, 65535, 'i', "cal_xmin_mag"},
      {"Max Y", 0, 65535, 'i', "cal_ymax_mag"},
      {"Min Y", 0, 65535, 'i', "cal_ymin_mag"}
    }
  }, // 10 10 10.5 10.34
  {COMMAND(pss_cal), "set pss calibration", GR_TRIM, 9,
    {
      {"Offset 1", -20.0, 20.0, 'f', "CAL_OFF_PSS1"},
      {"Distance 1", -2.0, 2.0, 'f', "CAL_D_PSS1"},
      {"Offset 2", -20.0, 20.0, 'f', "CAL_OFF_PSS2"},
      {"Distance 2", -2.0, 2.0, 'f', "CAL_D_PSS2"},
      {"Offset 3", -20.0, 20.0, 'f', "CAL_OFF_PSS3"},
      {"Distance 3", -2.0, 2.0, 'f', "CAL_D_PSS3"},
      {"Offset 4", -20.0, 20.0, 'f', "CAL_OFF_PSS4"},
      {"Distance 4", -2.0, 2.0, 'f', "CAL_D_PSS4"},
      {"I Min", 0.0, 20.0, 'f', "CAL_IMIN_PSS"}
    }
  },
  {COMMAND(autotrim_to_sc), "enable auto-trim to ISC/OSC", GR_TRIM, 3,
    {
      {"Threshold (sigma)", 0, 10, 'f', "THRESH_ATRIM"},
      {"Good time (s)", 0, CMD_I_MAX, 'i', "TIME_ATRIM"},
      {"Set rate (deg/s)", 0, 30, 'f', "RATE_ATRIM"}
    }
  },
  {COMMAND(az_gain), "az reaction wheel gains", GR_GAIN, 4,
    {
      {"Proportional Gain", 0, CMD_I_MAX, 'f', "g_p_az"},
      {"Integral Time",     0, 200, 'f', "g_i_az"},
      {"Derivative Time",     0, 200, 'f', "g_d_az"},
      {"Pointing Gain", 0, CMD_I_MAX, 'f', "g_pt_az"},
    }
  },
  {COMMAND(az_scan_accel), "set azimuth scan turnaround acceleration", GR_GAIN, 1,
    {
      {"Az Acceleration", 0.1, 2.0, 'f', "accel_az"}
    }
  },
  {COMMAND(set_scan_params), "set pos hwpr and dither index for next scan", GR_POINT, 2,
    {
      {"Next HWPR pos (0-3, -1: no change)", -1, 3, 'i', "next_i_hwpr"},
      {"Next dither index ", 0, 200, 'i', "next_i_dith"}
    }
  },
  {COMMAND(az_scan), "scan in azimuth", GR_POINT, 4,
    {
      {"Az centre (deg)",       -180, 360, 'f', "AZ"},
      {"El centre (deg)",         15,  65, 'f', "EL"},
      {"Width (deg on sky)",       0, 360, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)", 0,   2, 'f', "NONE"}
    }
  },
  {COMMAND(el_scan), "scan in azimuth", GR_POINT, 4,
    {
      {"Az centre (deg)",       -180, 360, 'f', "AZ"},
      {"El centre (deg)",         15,  65, 'f', "EL"},
      {"Height (deg on sky)",       0, 360, 'f', "NONE"},
      {"El Scan Speed (deg az/s)", 0,   2, 'f', "NONE"}
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
  {COMMAND(balance_tset), "Set balance pump minumum temperature", GR_BAL, 1,
     {
       {"Temperature (C)",  -273.4, 40, 'f', "T_BOX_BAL"},
     }
  },
  {COMMAND(box), "scan an az/el box centred on RA/Dec with el steps",
    GR_POINT, 7,
    {
      {"RA of Centre (h)",          0, 24, 'd', "RA"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0,200, 'i', "n_dith"}
    }
  },
  {COMMAND(el_box), "scan an az/el box centred on RA/Dec with az steps",
    GR_POINT, 7,
    {
      {"RA of Centre (h)",          0, 24, 'd', "RA"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"El Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"Az Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0,200, 'i', "n_dith"}
    }
  },
  {COMMAND(cap), "scan a circle centred on RA/Dec with el steps", GR_POINT, 6,
    {
      {"RA of Centre (h)",          0, 24, 'd', "RA"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0,200, 'i', "n_dith"}
    }
  },
  {COMMAND(drift), "move at constant speed in az and el", GR_POINT, 2,
    {
      {"Az Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"},
      {"El Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"}
    }
  },
  {COMMAND(quad), "scan a quadrilateral region in RA/Dec (corners must be "
    "ordered)", GR_POINT, 11,
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
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0,200, 'i', "n_dith"}
    }
  },
  {COMMAND(vbox), "DEPRECATED - scan an az/el box centred on RA/Dec with el drift",
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
  {COMMAND(vcap), "DEPRECATED - scan a circle centred on RA/Dec with el drift", GR_POINT, 5,
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
      {"RA of Centre (h)",      0, 24, 'f', "RA"},
      {"Dec of Centre (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(ra_dec_set), "define RA/Dec of current position", GR_TRIM, 2,
    {
      {"Current RA (h)",      0, 24, 'f', "RA"},
      {"Current Dec (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(pos_set), "define Latitude/Longitude of current position", GR_TRIM, 2,
    {
      {"Current Latitude (deg)",      -90, 90, 'f', "LAT"},
      {"Current Longitude (deg)", 0, 360, 'f', "LON"}
    }
  },
  {COMMAND(pivot_gain), "pivot gains", GR_GAIN, 5,
    {
      {"Set Point (dps)",   -200, 200, 'f', "SET_RW"},
      {"V_err Gain (prop)", 0, CMD_L_MAX, 'f', "G_PE_PIV"},
      {"V_RW Gain (prop)", 0, CMD_L_MAX, 'f', "G_PV_PIV"},
      {"V_RW Integral time", 0, 200, 'f', "G_IV_PIV"},
      {"Static Friction offset",   0, 100, 'f', "FRICT_OFF_PIV"},
    }
  },
  {COMMAND(el_gain), "elevation motor gains", GR_GAIN, 6,
    {
      {"Proportional Gain", 0, CMD_L_MAX, 'f', "G_P_EL"},
      {"Integral Time",     0, 200, 'f', "G_I_EL"},
      {"Derivative Time",   0, 200, 'f', "G_D_EL"},
      {"Pointing Gain",     0, CMD_L_MAX, 'f', "G_PT_EL"},
      {"Integral Term Deadband  (mA)",     0, 500, 'f', "G_DB_EL"},
      {"Static Friction offset",   0, 100, 'f', "FRICT_OFF_EL"},
    }
  },
  {COMMAND(az_gyro_offset), "manually set az gyro offsets", GR_TRIM, 2,
    {
      {"IF Roll Gyro offset (deg/s)", -0.5, 0.5, 'f', "OFFSET_IFROLL_GY"},
      {"IF Yaw Gyro offset (deg/s)", -0.5, 0.5, 'f', "OFFSET_IFYAW_GY"}
    }
  },
  {COMMAND(el_gyro_offset), "manually set el gyro offset", GR_TRIM, 1,
    {
      {"IF Elev Gyro offset (deg/s)", -0.5, 0.5, 'f', "OFFSET_IFEL_GY"},
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
      {"Lock Elevation (deg)", 5, 90, 'f', "EL_ENC"}
    }
  },
  {COMMAND(general), "send a general command string to the lock or actuators",
    GR_STAGE | GR_ACT | GR_LOCK | GR_HWPR, 2,
    {
      {"Address (1-3,5,8,13,33)", 1, 0x2F, 'i', "1.0"},
      {"Command", 0, 32, 's', "NONE"},
    }
  },
  {COMMAND(lock_vel), "set the lock motor velocity and acceleration", GR_LOCK,
    2,
    {
      {"Velocity", 5, 500000, 'l', "VEL_LOCK"},
      {"Acceleration", 1, 1000, 'i', "ACC_LOCK"},
    }
  },
  {COMMAND(lock_i), "set the lock motor currents", GR_LOCK, 2,
    {
      {"Move current (%)", 0, 100, 'i', "I_MOVE_LOCK"},
      {"Hold current (%)", 0,  50, 'i', "I_HOLD_LOCK"},
    }
  },
  {COMMAND(set_secondary), "servo the secondary mirror to absolute position",
    GR_FOCUS, 1,
    {
      {"Position (per FOCUS_SF counts)", -15000, 15000, 'i', "FOCUS_SF"},
    }
  },
  {COMMAND(delta_secondary), "servo the secondary mirror by a relative amount",
    GR_FOCUS, 1,
    {
      {"Position (counts)", -1000, 1000, 'i', "0"},
    }
  },
  {COMMAND(thermo_gain), "set the secondary actuator system gains", GR_FOCUS, 4,
    {
      {"T. Primary Gain",   1, 1000, 'f', "G_PRIME_SF"},
      {"T. Secondary Gain", 1, 1000, 'f', "G_SECOND_SF"},
      {"Step Size (um)",   10, 1000, 'i', "STEP_SF"},
      {"Step Wait (min)"  , 0, 1500, 'i', "WAIT_SF"},
    }
  },
  {COMMAND(actuator_servo), "servo the actuators to absolute positions",
    GR_ACT, 3,
    {
      {"Actuator Alpha (ENC units)", -15000, 15000, 'i', "ENC_0_ACT"},
      {"Actuator Beta (ENC units)",  -15000, 15000, 'i', "ENC_1_ACT"},
      {"Actuator Gamma (ENC units)", -15000, 15000, 'i', "ENC_2_ACT"}
    }
  },
  {COMMAND(focus_offset), "set the in focus position offset relative the "
    "nominal focus", GR_FOCUS, 1,
    {
      {"Offset", -5000, 25000, 'i', "OFFSET_SF"}
    }
  },
  {COMMAND(actuator_delta), "offset the actuators to from current position",
    GR_ACT, 3,
    {
      {"Actuator Alpha", -5000, 5000, 'i', "0"},
      {"Actuator Beta",  -5000, 5000, 'i', "0"},
      {"Actuator Gamma", -5000, 5000, 'i', "0"}
    }
  },
  {COMMAND(act_offset), "set the actuator encoder/lvdt offsets", GR_ACT, 3,
    {
      {"Actuator Alpha (Enc units)", 0, 65536, 'f', "Enc_0_act"},
      {"Actuator Beta (Enc units)",  0, 65536, 'f', "Enc_1_act"},
      {"Actuator Gamma (Enc units)", 0, 65536, 'f', "Enc_2_act"}
    }
  },
  {COMMAND(act_enc_trim), "manually set encoder and dead reckoning", GR_ACT, 3,
    {
      {"Actuator Alpha (Enc units)", 0, 65536, 'f', "Dr_0_act"},
      {"Actuator Beta (Enc units)",  0, 65536, 'f', "Dr_1_act"},
      {"Actuator Gamma (Enc units)", 0, 65536, 'f', "Dr_2_act"}
    }
  },
  {COMMAND(actuator_vel), "set the actuator velocity and acceleration", GR_ACT,
    2,
    {
      {"Velocity", 5, 20000, 'i', "VEL_ACT"},
      {"Acceleration", 1, 20, 'i', "ACC_ACT"}
    }
  },
  {COMMAND(actuator_i), "set the actuator motor currents", GR_ACT, 2,
    {
      {"Move current (%)", 0, 100, 'i', "I_MOVE_ACT"},
      {"Hold current (%)", 0,  50, 'i', "I_HOLD_ACT"}
    }
  },
  {COMMAND(actuator_tol), "set the tolerance for servo moves", GR_ACT, 1,
    {
      {"Move tolerance (~um)", 0, 1000, 'i', "TOL_ACT"}
    }
  },
  {COMMAND(lvdt_limit), "set the hard LVDT limits on actuator moves", GR_ACT, 3,
    {
      {"Spread limit", 0, 5000, 'f', "LVDT_SPREAD_ACT"},
      {"Lower limit", -5000, 60000, 'f', "LVDT_LOW_ACT"},
      {"Upper limit", -5000, 60000, 'f', "LVDT_HIGH_ACT"}
    }
  },
  {COMMAND(thermo_param), "set the thermal compensation parameters", GR_FOCUS,
    3,
    {
      {"Temp. Spread", 0, 100, 'f', "SPREAD_SF"},
      {"Preferred T Prime", 0, 2, 'i', "PREF_TP_SF"},
      {"Preferred T Second", 0, 2, 'i', "PREF_TS_SF"}
    }
  },
  {COMMAND(hwpr_vel), "set the waveplate rotator velocity and acceleration", 
    GR_HWPR, 2,
    {
      {"Velocity", 5, 500000, 'l', "VEL_HWPR"},
      {"Acceleration", 1, 1000, 'i', "ACC_HWPR"},
    }
  },
  {COMMAND(hwpr_i), "set the waveplate rotator currents", GR_HWPR, 2,
    {
      {"Move current (%)", 0, 100, 'i', "I_MOVE_HWPR"},
      {"Hold current (%)", 0,  50, 'i', "I_HOLD_HWPR"},
    }
  },
  {COMMAND(hwpr_goto), "move the waveplate rotator to absolute position",
    GR_HWPR, 1,
    {
      {"destination", 0, 80000, 'l', "ENC_HWPR"}
    }
  },
  {COMMAND(hwpr_jump), "move the waveplate rotator to relative position",
    GR_HWPR, 1,
    {
      {"delta", -80000, 80000, 'l', "0"}
    }
  },
  {COMMAND(hwpr_repeat), 
    "DEPRECATED - repeatedly cycle the hwpr through a number of positions",
    GR_HWPR, 4,
    {
      {"Number of step positions", 0, MAX_15BIT, 'i', "NONE"},
      {"Number of times to step", 0, MAX_15BIT, 'i', "NONE"},
      {"Time between steps (s)", 0, MAX_15BIT, 'i', "NONE"},
      {"Step size (encoder ticks)", -MAX_15BIT/2, MAX_15BIT/2, 'i', "NONE"},
    }
  },
  {COMMAND(hwpr_define_pos), 
    "define the four hwpr potentiometer positions to be used for scans",
    GR_HWPR, 4,
    {
      {"Position 1", 0.1, 0.9, 'f', "POS0_HWPR"},
      {"Position 2", 0.1, 0.9, 'f', "POS1_HWPR"},
      {"Position 3", 0.1, 0.9, 'f', "POS2_HWPR"},
      {"Position 4", 0.1, 0.9, 'f', "POS3_HWPR"}
    }
  },
  {COMMAND(hwpr_goto_pot), 
    "Move wave plate rotator to commanded potentiometer value",
    GR_HWPR, 1,
    {
      {"Pot Value ", 0.1, 0.9, 'f', "POT_HWPR"},
    }
  },
  {COMMAND(hwpr_set_overshoot), 
    "set the overshoot in encoder counts for backwards hwpr moves",
    GR_HWPR, 1,
    {
      {"overshoot", 0, MAX_15BIT, 'i', "OVERSHOOT_HWPR"},
    }
  },
  {COMMAND(hwpr_goto_i), 
    "goto hwpr position (0-3)",
    GR_HWPR, 1,
    {
      {"hwpr position", 0, 3, 'i', "I_POS_RQ_HWPR"},
    }
  },

  /* XY Stage */
  {COMMAND(xy_goto), "move the X-Y translation stage to absolute position",
    GR_STAGE, 4,
    {
      {"X destination", 0, 80000, 'l', "X_STAGE"},
      {"Y destination", 0, 80000, 'l', "Y_STAGE"},
      {"X speed", 0, 16000, 'i', "X_VEL_STAGE"},
      {"Y speed", 0, 16000, 'i', "Y_VEL_STAGE"}
    }
  },
  {COMMAND(xy_jump), "move the X-Y translation stage to relative position",
    GR_STAGE, 4,
    {
      {"X delta", -80000, 80000, 'l', "0"},
      {"Y delta", -80000, 80000, 'l', "0"},
      {"X speed", 0, 16000, 'i', "X_VEL_STAGE"},
      {"Y speed", 0, 16000, 'i', "Y_VEL_STAGE"}
    }
  },
  {COMMAND(xy_xscan), "scan the X-Y translation stage in X", GR_STAGE, 3,
    {
      {"X center", 0, 80000, 'l', "X_STAGE"},
      {"delta X", 0, 80000, 'l', "NONE"},
      {"X speed", 0, 16000, 'i', "X_VEL_STAGE"},
    }
  },
  {COMMAND(xy_yscan), "scan the X-Y translation stage in Y", GR_STAGE, 3,
    {
      {"Y center", 0, 80000, 'l', "Y_STAGE"},
      {"delta Y", 0, 80000, 'l', "NONE"},
      {"Y speed", 0, 16000, 'i', "Y_VEL_STAGE"},
    }
  },
  {COMMAND(xy_raster), "raster the X-Y translation stage", GR_STAGE, 7,
    {
      {"X center", 0, 80000, 'l', "X_STAGE"},
      {"X Width", 0, 40000, 'i', "NONE"},
      {"Y center", 0, 80000, 'l', "Y_STAGE"},
      {"Y Width", 0, 40000, 'i', "NONE"},
      {"X Velocity", 0, 16000, 'i', "X_VEL_STAGE"},
      {"Y Velocity", 0, 16000, 'i', "Y_VEL_STAGE"},
      {"Step Size", 0, 40000, 'i', "NONE"},
    }
  },

  /*******************************************************/
  /*************** Telemetry/Scheduling  *****************/
  {COMMAND(timeout), "time until schedule mode", GR_TELEM, 1,
    {
      {"Timeout (s)", 2, 65535, 'f', "TIMEOUT"}
    }
  },

  {COMMAND(tdrss_bw), "tdrss omni bandwith", GR_TELEM, 1,
    {
      {"Bandwidth (bps)", 100, 75000, 'f', "rate_tdrss"}
    }
  },

  {COMMAND(iridium_bw), "iridium dialup bandwith", GR_TELEM, 1,
    {
      {"Bandwidth (bps)", 100, 75000, 'f', "rate_iridium"}
    }
  },

  /****************************************/
  /*************** Misc.  *****************/
  {COMMAND(t_gyro_gain), "gyro box heater gains", GR_ELECT, 3,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_heat_gy"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_heat_gy"},
      {"Derivative Gain",  0, MAX_15BIT, 'i', "g_d_heat_gy"}
    }
  },
  {COMMAND(t_gyro_set), "gyro box temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_GY"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {COMMAND(bias_level_250), "bias level 250 micron", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "AMPL_250_BIAS"}
    }
  },
  {COMMAND(bias_level_350), "bias level 350 micron", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "AMPL_350_BIAS"}
    }
  },
  {COMMAND(bias_level_500), "bias level 500 micron", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "AMPL_500_BIAS"}
    }
  },
  {COMMAND(bias_step), "step through different bias levels", GR_BIAS, 6,
    {
      {"Start", 0, 32767, 'i', "STEP_START_BIAS"},
      {"End", 0, 32767, 'i', "STEP_END_BIAS"},
      {"N steps", 1, 32767, 'i', "step_nsteps_bias"},
      {"Time per step (ms)", 10, 32767, 'i', "step_time_bias"},
      {"Cal pulse length (ms)", 0, 32767, 'i', "step_pul_len_bias"},
      {"Array (250,350,500,0=all)", 0, 32767, 'i', "step_array_bias"},
    }
  },
  {COMMAND(phase_step), "step through different phases", GR_BIAS, 4,
    {
      {"Start", 0, 32767, 'i', "STEP_START_PHASE"},
      {"End", 0, 32767, 'i', "STEP_END_PHASE"},
      {"N steps", 1, 32767, 'i', "step_nsteps_phase"},
      {"Time per step (ms)", 1, 32767, 'i', "step_time_phase"},
    }
  },
  {COMMAND(bias_level_rox), "bias level ROX", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "AMPL_ROX_BIAS"}
    }
  },
  {COMMAND(bias_level_x), "bias level X (unused)", GR_BIAS, 1,
    {
      {"Level", 0, 32767, 'i', "ampl_x_bias"}
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
      {"Pulse Length (ms)", 0, 8000, 'i', "PULSE_CAL"}
    }
  },
  {COMMAND(cal_repeat), "set calibrator to automatic repeated pulse mode", GR_CRYO_HEAT, 3,
    {
      {"Pulse Length (ms)", 10, 8000, 'i', "PULSE_CAL"},
      {"Max Pulse Delay (0=never pulse) (s)",  0, 32767, 'i', "PERIOD_CAL"},
      {"Always Pulse before HWP move (0=no,1=yes)",  0, 1, 'i', "NONE"}
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

  {COMMAND(fridge_cycle_params), "Fridge cycle parameters", GR_CRYO_HEAT, 6,
    {
      {"300mK_strap Start Temp (K)", 0, 4., 'f', "T_START_CYCLE"},
      {"Pot Max Temp (K)", 0, 10., 'f', "T_POT_MAX_CYCLE"},
      {"Charcoal Max Temp (K)", 0, 70., 'f', "T_CHAR_MAX_CYCLE"},
      {"Charcoal Timeout (min)", 0, 120., 'f', "TIME_CHAR_CYCLE"},
      {"Charcoal Settled Temp (K)", 0, 70., 'f', "T_CHAR_SET_CYCLE"},
      {"Charcoal Settle Time (min)", 0, 120., 'f', "TIME_SET_CYCLE"}
    }
  },

//  <!-- XSC general -->





////  <!-- XSC exposure -->

    {COMMAND(xsc_exposure_timing), "xsc exposure time", GR_ISC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"exposure time_cs (cs) [default: 12]", 1, 100, 'l', "NONE"},
            {"grace period (s) (disregards which) [default: 10.0]", 1.0, 100.0, 'f', "NONE"},
            {"post trigger counter_mcp share delay (cs) (disregards which) [default: 200]", 1, 1000, 'l', "NONE"},
        },
    },


    {COMMAND(xsc_multi_trigger), "xsc trigger timing", GR_ISC_PARAM, 3,
        {
            {"num triggers [default: 1]", 0, 16, 'i', "NONE"},
            {"time between triggers (cs) [default: 19]", 1, 100, 'l', "NONE"},
            {"stars readout delay (s) [default: 1.0]", 0.1, 10.0, 'f', "NONE"},
        },
    },

////  <!-- XSC main -->


    {COMMAND(xsc_main_settings), "xsc main settings", GR_ISC_PARAM, 6,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"display_frequency [default: 20.0]", 1.0, 30.0, 'f', "NONE"},
            {"display_fullscreen [default: 1]", 0, 1, 'i', "NONE"},
            {"display_image_only [default: 0]", 0, 1, 'i', "NONE"},
            {"display_solving_filters [default: 0]]", 0, 1, 'i', "NONE"},
            {"display_image_brightness [default: 1.0]", 0.5, 4.0, 'f', "NONE"},
        },
    },


    {COMMAND(xsc_display_zoom), "xsc display zoom", GR_ISC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"center x display pixel [default: 320]", 0, 1000, 'i', "NONE"},
            {"center y display pixel [default: 240]", 0, 1000, 'i', "NONE"},
            {"zoom [default: 1.0]", 1.0, 4.0, 'f', "NONE"},
        },
    },


////  <!-- XSC imaging (lens, camera, fake sky, masking) -->



    {COMMAND(xsc_run_autofocus), "xsc run autofocus", GR_ISC_PARAM, 1,
        {
            {"which", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_set_autofocus_range), "xsc set autofocus range", GR_ISC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"focus_search_min", 1, 10000, 'l', "NONE"},
            {"focus_search_max", 2, 10000, 'l', "NONE"},
            {"focus_search_step", 2, 1000, 'l', "NONE"},
        },
    },

    {COMMAND(xsc_abort_autofocus), "xsc abort autofocus", GR_ISC_PARAM, 2,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"still_use_solution", 0, 1, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_autofocus_display_mode), "xsc autofocus display mode", GR_ISC_PARAM, 2,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"mode (0=auto, 1=on, 2=off)", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_get_gain), "xsc get gain", GR_ISC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_set_gain), "xsc set gain", GR_ISC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"position", 0.5, 10.0, 'f', "NONE"},
        },
    },


    {COMMAND(xsc_fake_sky_brightness), "xsc fake sky brightness", GR_ISC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"enabled", 0, 1, 'i', "NONE"},
            {"level_kepsa",0.0, 10000.0, 'f', "NONE"},
            {"gain_db",-50, 10, 'f', "NONE"},
            {"actual exposure time (s)", 0.01, 10.0, 'f', "NONE"},
            {"simulated exposure time (s)", 0.01, 10.0, 'f', "NONE"},
        },
    },



  {COMMAND(motors_verbose), "Set verbosity of motor serial threads (0=norm, 1=verbose, 2= superverbose )", GR_MISC, 3,
   {
     {"Reaction Wheel", 0, 5, 'i', "VERBOSE_RW"},
     {"Elevation", 0, 5, 'i', "VERBOSE_EL"},
     {"Pivot", 0, 5, 'i', "VERBOSE_PIV"}
   }
  },
  {COMMAND(shutter_step), "set number of shutter steps to close (default 4224)", 
    GR_SHUTTER, 1,
    {
      {"Steps", 1, 5000, 'i', "STEPS_SHUTTER"},
    }
  },
  {COMMAND(shutter_step_slow), "set number of incremental shutter steps to close (default 300)", GR_SHUTTER, 1,
    {
      {"Steps slow", 1, 5000, 'i', "STEPS_SLOW_SHUTTER"},
    }
  },
  {COMMAND(params_test), "Do nothing, with all paramter types", GR_MISC, 5,
    {
      {"i", 0, CMD_I_MAX, 'i', "NONE"},
      {"l", 0, CMD_L_MAX, 'l', "NONE"},
      {"f (-100 to +100)", -100, 100, 'f', "NONE"},
      {"d (-100 to +100)", -100, 100, 'd', "NONE"},
      {"s", 0, 32, 's', "NONE"}
    }
  },
  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, CMD_I_MAX, 'i', "PLOVER"}
    }
  },
  {COMMAND(xsc_is_new_window_period), "Set the time over which commands are valid (in centi-seconds)", GR_XSC, 2,
      {
              {"Which", 0, 2, 'i', "NONE"},
              {"Window period", 0, 2000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_offset), "Trim the star camera", GR_XSC, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Cross-El trim", -180, 180, 'd', "NONE"},
              {"El trim", -180, 180, 'd', "NONE"},
      }
  },

  ////  <!-- XSC heaters -->

  {COMMAND(xsc_heaters_off), "Turn off the XSC heater", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_heaters_on), "Turn on the XSC heater", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_heaters_auto), "Allow XSC to control its heater", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_trigger_threshold), "Allow XSC to trigger based on predicted px streaking", GR_XSC, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Enabled", 0, 1, 'i', "NONE"},
              {"Blob streaking limit (pixels", 0, 100, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_scan_force_trigger), "Force XSC to trigger on turnaround (ignore speed)", GR_XSC, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Force the trigger on scan limits", 0, 1, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_quit), "Quit XSC", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_shutdown), "Shutdown the XSC computer", GR_XSC, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Restart?", 0, 1, 'i', "NONE"}
      }
  },

  {COMMAND(xsc_image_client), "Enable or disable sending images to mcp", GR_XSC, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Image client enabled", 0, 1, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_init_focus), "Initialize the focus motor", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_set_focus), "Set the absolute focus position", GR_XSC, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Absolute focus position", 0, 10000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_set_focus_incremental), "Command an incremental step to the focus motor", GR_XSC, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Incremental focus steps", -10000, 10000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_init_aperture), "Initialize the aperture motor", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_set_aperture), "Set the absolute aperture position", GR_XSC, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Absolute aperture position", 0, 10000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_solver_general), "Solver parameter settings", GR_XSC, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Enable the solver", 0, 1, 'i', "NONE"},
              {"Set the solver timeout period (s)", 5, 600, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_solver_abort), "Abort the solving the current image", GR_XSC, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"}
      }
  },
  {COMMAND(xsc_selective_mask), "Set the XSC selective mask", GR_XSC, 6,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Enable selective masking", 0, 1, 'i', "NONE"},
              {"Mask field 1", 0, CMD_L_MAX, 'i', "NONE"},
              {"Mask field 2", 0, CMD_L_MAX, 'i', "NONE"},
              {"Mask field 3", 0, CMD_L_MAX, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_blob_finding), "XSC blob finder settings", GR_XSC, 6,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"SNR Threshhold", 0.01, 10, 'f', "NONE"},
              {"Max num blobs", 3, 100, 'i', "NONE"},
              {"Enable Robust mode", 0, 1, 'i', "NONE"},
              {"Fitting method (0= none, 1=gaussian, 2=double gaussian)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_blob_cells), "XSC blob cell settings", GR_XSC, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Cell size (pixels, power of 2)", 4, 512, 'i', "NONE"},
              {"Max num blobs per cell (default 2)", 1, 10, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_motion_psf), "XSC Motion PSF Settings", GR_XSC, 4,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Motion PSF Enabled", 0, 1, 'i', "NONE"},
              {"Horizontal Roll (degrees)", -90.0, 90.0, 'f', "NONE"},
              {"Platescale (\"/px)", 6.0, 7.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_pattern_matching), "XSC pattern matching settings", GR_XSC, 8,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Pattern match enable", 0, 1, 'i', "NONE"},
              {"Display Star names", 0, 1, 'i', "NONE"},
              {"Match Tolerance (pixels)", 0.01, 10, 'f', "NONE"},
              {"Platescale min (\"/px)", 6.0, 7.0, 'f', "NONE"},
              {"Platescale max (\"/px)", 6.0, 7.0, 'f', "NONE"},
              {"Use fixed Platescale", 0, 1, 'i', "NONE"},
              {"Fixed Platescale (\"/px)", 6.0, 7.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_hor_location), "XSC Horizontal Location Filter", GR_XSC, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Horizontal Limit Enabled", 0, 1, 'i', "NONE"},
              {"Horizontal Radius (degrees)", 0.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_eq_location), "XSC Equatorial Location Filter", GR_XSC, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Equatorial Limit Enabled", 0, 1, 'i', "NONE"},
              {"Equatorial Radius (degrees)", 0.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_hor_roll), "XSC Horizontal Roll Limit", GR_XSC, 4,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Horizontal Roll Limit", 0, 1, 'i', "NONE"},
              {"Minimum Horizontal Roll (degrees)", -90.0, 90.0, 'f', "NONE"},
              {"Maximum Horizontal Roll (degrees)", -90.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_el), "XSC Elevation Limit", GR_XSC, 4,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Elevation Limit", 0, 1, 'i', "NONE"},
              {"Minimum Elevation (degrees)", -90.0, 90.0, 'f', "NONE"},
              {"Maximum Elevation (degrees)", -90.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_matching), "XSC Matching Filter", GR_XSC, 4,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Pointing Error threshold (arc seconds)", 0.01, 120, 'f', "NONE"},
              {"Pixel Error threshold (pixels)", 0.01, 20, 'f', "NONE"},
              {"Minimum stars matched", 4, 30, 'i', "NONE"},
      }
  }
};

/* validate parameters of an mcom -- called by spidercmd before tranmitting a
 * command and by pcm after decoding one.  Inputs:
 *
 * cmd:         command number
 * [irs]values: mcp-style parsed parameters
 * buflen       size of the err_buffer
 * err_buffer   a place to write the error string
 *
 * Return value:
 *
 *  0:  if parameters are okay; err_buffer ignored.
 *  !0: if parameters are not okay.  In this case a descriptive error message
 *      should be written to err_buffer.
 */
int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer)
{
  return 0; /* no checks -- everything passes */
}
