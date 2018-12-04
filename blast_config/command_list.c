
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

#include "include/command_list.h"

const char *command_list_serial = "$Revision: 5.2 $";

const char *GroupNames[N_GROUPS] = {
                                    [GRPOS_POINT] = "Pointing Modes",
                                    [GRPOS_BAL] = "Balance",
                                    [GRPOS_HWPR] =  "Waveplate Rotator",
                                    [GRPOS_TRIM] = "Pointing Sensor Trims",
                                    [GRPOS_ELECT] = "Aux. Electronics",
                                    [GRPOS_BIAS] = "Bias",
                                    [GRPOS_ROACH] = "ROACH Commands",
                                    [GRPOS_VETO] = "Pointing Sensor Vetos",
                                    [GRPOS_ACT] = "Actuators",
                                    [GRPOS_XSC_HOUSE] = "XSC Housekeeping",
                                    [GRPOS_XSC_MODE] = "XSC Mode Settings",
                                    [GRPOS_XSC_PARAM] = "XSC Solving Parameters",
                                    [GRPOS_MOTOR] =  "Pointing Motors",
                                    [GRPOS_CRYO] = "Cryo Control",
                                    [GRPOS_POWER] = "Subsystem Power",
                                    [GRPOS_LOCK] = "Lock Motor",
                                    [GRPOS_TELEM] =  "Telemetry",
                                    [GRPOS_MISC] = "Miscellaneous",
                                    [GRPOS_FOCUS] = "Focus",
  };

#define LINKLIST_SELECT "Linklist", 0, 64, 'i', "NONE", {linklist_names}

const char *downlink_names[] = {"Pilot", "Bi0", "Highrate", "SBD", 0};
const char *linklist_names[] = {0};


// echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

struct scom scommands[xyzzy + 1] = {
  {COMMAND(load_curve), "starting load curve", GR_CRYO},
  {COMMAND(reboot_ljcryo1), "rebooting labjack cryo 1", GR_POWER},
  {COMMAND(vtx_xsc0), "Setting video transmitter to XSC0", GR_XSC_MODE | GR_TELEM},
  {COMMAND(vtx_xsc1), "Setting video transmitter to XSC1", GR_XSC_MODE | GR_TELEM},
  {COMMAND(pilot_oth_on), "Pilot set to OTH", GR_TELEM},
  {COMMAND(pilot_oth_off), "Pilot set to not OTH (GND)", GR_TELEM},
  {COMMAND(heater_300mk_on), "turning on 300mK heater", GR_CRYO},
  {COMMAND(heater_300mk_off), "turning off 300mK heater", GR_CRYO},
  {COMMAND(charcoal_hs_on), "turning on charcoal hs", GR_CRYO},
  {COMMAND(charcoal_hs_off), "turning off charcoal hs", GR_CRYO},
  {COMMAND(single_cal_pulse), "pulsing the cal lamp", GR_CRYO},
  {COMMAND(lna250_on), "turning on 250 lna", GR_CRYO},
  {COMMAND(lna250_off), "turning off 250 lna", GR_CRYO},
  {COMMAND(lna350_on), "turning on 350 lna", GR_CRYO},
  {COMMAND(lna350_off), "turning off 350 lna", GR_CRYO},
  {COMMAND(lna500_on), "turning on 500 lna", GR_CRYO},
  {COMMAND(lna500_off), "turning off 500 lna", GR_CRYO},
  {COMMAND(allow_cycle), "autocycle on", GR_CRYO},
  {COMMAND(disallow_cycle), "autocycle_off", GR_CRYO},
  {COMMAND(force_cycle), "forcing a cycle", GR_CRYO},
  // {COMMAND(level_sensor_on), "turning on level sensor", GR_CRYO},
  // {COMMAND(level_sensor_off), "turning off level sensor", GR_CRYO},
  {COMMAND(level_sensor_pulse), "pulsing the level sensor", GR_CRYO},
  {COMMAND(charcoal_on), "turning on charcoal heater", GR_CRYO},
  {COMMAND(charcoal_off), "turning off charcoal heater", GR_CRYO},
  {COMMAND(heaters_off), "turning off the heater card channels", GR_CRYO},
  {COMMAND(heater_1k_on), "turning on 1K heater", GR_CRYO},
  {COMMAND(heater_1k_off), "turning off 1K heater", GR_CRYO},
  {COMMAND(power_box_on), "turning on the power box", GR_CRYO},
  {COMMAND(power_box_off), "turning off the power box", GR_CRYO},
  {COMMAND(amp_supply_on), "turning on +5V, +/-15V", GR_CRYO},
  {COMMAND(amp_supply_off), "turning off +5V, +/-15V", GR_CRYO},
  {COMMAND(therm_readout_on), "turning on 12V channels", GR_CRYO},
  {COMMAND(therm_readout_off), "turning off 12V channels", GR_CRYO},
  {COMMAND(heater_supply_on), "turning on 40V channels", GR_CRYO},
  {COMMAND(heater_supply_off), "turning off 40V channels", GR_CRYO},
  {COMMAND(heater_sync), "syncing heater command channel to input", GR_CRYO},
  {COMMAND(bias_reset_rox), "Attempt to restart the ALSA sound card ROX bias generation.", GR_CRYO},
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},
  {COMMAND(antisun), "turn antisolar now", GR_POINT},
// power box OF and IF relay controls
  {COMMAND(hd_pv_cycle), "powercycling HD PV", GR_POWER},
  {COMMAND(eth_switch_cycle), "powercycling Eth Switch", GR_POWER},
  {COMMAND(fc1_cycle), "powercycling FC1", GR_POWER},
  {COMMAND(xsc1_cycle), "powercycling XSC1", GR_POWER},
  {COMMAND(fc2_cycle), "powercycling FC2", GR_POWER},
  {COMMAND(xsc0_cycle), "powercycling XSC0", GR_POWER},
  {COMMAND(gyros_cycle), "powercycling gyros", GR_POWER},
  {COMMAND(data_transmit_cycle), "powercycling Data Transmit", GR_POWER},
  {COMMAND(elmot_cycle), "powercycling El Motor", GR_POWER},
  {COMMAND(pivot_cycle), "powercycling pivot", GR_POWER},
  {COMMAND(mag_cycle), "powercycling magnetometer", GR_POWER},
  {COMMAND(rw_cycle), "powercycling RW Motor", GR_POWER},
  {COMMAND(steppers_cycle), "powercycling steppers", GR_POWER},
  {COMMAND(clino_cycle), "powercycling clinometers", GR_POWER},
  {COMMAND(of_15_cycle), "powercycling OF relay 15", GR_POWER},
  {COMMAND(gps_timing_cycle), "powercycling gps timing", GR_POWER},
  {COMMAND(hd_pv_on), "turning on HD PV", GR_POWER},
  {COMMAND(eth_switch_on), "turning on Eth Switch", GR_POWER},
  {COMMAND(fc1_on), "turning on FC1", GR_POWER},
  {COMMAND(xsc1_on), "turning on XSC1", GR_POWER},
  {COMMAND(fc2_on), "turning on FC2", GR_POWER},
  {COMMAND(xsc0_on), "turning on XSC0", GR_POWER},
  {COMMAND(gyros_on), "turning on OF gyros", GR_POWER},
  {COMMAND(data_transmit_on), "turning on Data Transmit", GR_POWER},
  {COMMAND(elmot_on), "turning on El Motor", GR_POWER},
  {COMMAND(pivot_on), "turning on pivot", GR_POWER},
  {COMMAND(mag_on), "turning on magnetometer", GR_POWER},
  {COMMAND(rw_on), "turning on RW Motor", GR_POWER},
  {COMMAND(steppers_on), "turning on steppers", GR_POWER},
  {COMMAND(clino_on), "turning on clinometers", GR_POWER},
  {COMMAND(of_relay_15_on), "turning on OF relay 15", GR_POWER},
  {COMMAND(gps_timing_on), "turning on gps timing", GR_POWER},
  {COMMAND(hd_pv_off), "turning off HD PV", GR_POWER},
  {COMMAND(eth_switch_off), "turning off Eth Switch", GR_POWER},
  {COMMAND(fc1_off), "turning off FC1", GR_POWER},
  {COMMAND(xsc1_off), "turning off XSC1", GR_POWER},
  {COMMAND(fc2_off), "turning off FC2", GR_POWER},
  {COMMAND(xsc0_off), "turning off XSC0", GR_POWER},
  {COMMAND(gyros_off), "turning off OF gyros", GR_POWER},
  {COMMAND(data_transmit_off), "turning off Data Transmit", GR_POWER},
  {COMMAND(elmot_off), "turning off El Motor", GR_POWER},
  {COMMAND(pivot_off), "turning off pivot", GR_POWER},
  {COMMAND(mag_off), "turning off magnetometer", GR_POWER},
  {COMMAND(rw_off), "turning off RW Motor", GR_POWER},
  {COMMAND(steppers_off), "turning off steppers", GR_POWER},
  {COMMAND(clino_off), "turning off clinometers", GR_POWER},
  {COMMAND(of_relay_15_off), "turning off OF relay 15", GR_POWER},
  {COMMAND(gps_timing_off), "turning off gps timing", GR_POWER},
  {COMMAND(if_1_cycle), "powercycling if relay 1", GR_POWER},
  {COMMAND(if_2_cycle), "powercycling if relay 2", GR_POWER},
  {COMMAND(if_3_cycle), "powercycling if relay 3", GR_POWER},
  {COMMAND(if_4_cycle), "powercycling if relay 4", GR_POWER},
  {COMMAND(if_5_cycle), "powercycling if relay 5", GR_POWER},
  {COMMAND(if_6_cycle), "powercycling if relay 6", GR_POWER},
  {COMMAND(if_7_cycle), "powercycling if relay 7", GR_POWER},
  {COMMAND(if_8_cycle), "powercycling if relay 8", GR_POWER},
  {COMMAND(if_9_cycle), "powercycling if relay 9", GR_POWER},
  {COMMAND(if_10_cycle), "powercycling if relay 10", GR_POWER},
  {COMMAND(if_relay_1_on), "turning on IF relay 1", GR_POWER},
  {COMMAND(if_relay_2_on), "turning on IF relay 2", GR_POWER},
  {COMMAND(if_relay_3_on), "turning on IF relay 3", GR_POWER},
  {COMMAND(if_relay_4_on), "turning on IF relay 4", GR_POWER},
  {COMMAND(if_relay_5_on), "turning on IF relay 5", GR_POWER},
  {COMMAND(if_relay_6_on), "turning on IF relay 6", GR_POWER},
  {COMMAND(if_relay_7_on), "turning on IF relay 7", GR_POWER},
  {COMMAND(if_relay_8_on), "turning on IF relay 8", GR_POWER},
  {COMMAND(if_relay_9_on), "turning on IF relay 9", GR_POWER},
  {COMMAND(if_relay_10_on), "turning on IF relay 10", GR_POWER},
  {COMMAND(if_relay_1_off), "turning off IF relay 1", GR_POWER},
  {COMMAND(if_relay_2_off), "turning off IF relay 2", GR_POWER},
  {COMMAND(if_relay_3_off), "turning off IF relay 3", GR_POWER},
  {COMMAND(if_relay_4_off), "turning off IF relay 4", GR_POWER},
  {COMMAND(if_relay_5_off), "turning off IF relay 5", GR_POWER},
  {COMMAND(if_relay_6_off), "turning off IF relay 6", GR_POWER},
  {COMMAND(if_relay_7_off), "turning off IF relay 7", GR_POWER},
  {COMMAND(if_relay_8_off), "turning off IF relay 8", GR_POWER},
  {COMMAND(if_relay_9_off), "turning off IF relay 9", GR_POWER},
  {COMMAND(if_relay_10_off), "turning off IF relay 10", GR_POWER},

  {COMMAND(actbus_off), "turn off the Actuators, Lock, and HWPR", GR_POWER | GR_LOCK | GR_ACT | GR_HWPR | CONFIRM},
  {COMMAND(actbus_on), "turn on the Actuators, Lock, and HWPR", GR_POWER | GR_LOCK | GR_ACT | GR_HWPR},
  {COMMAND(actbus_cycle), "power cycle the Actuators, Lock, and HWPR", GR_POWER | GR_LOCK | GR_ACT | GR_HWPR | CONFIRM},
  {COMMAND(vtx_off), "Turn off the video transmitter", GR_TELEM | GR_POWER},
  {COMMAND(vtx_on), "Turn on the video transmitter", GR_TELEM | GR_POWER},
  {COMMAND(bi0_off), "turn off the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(bi0_on), "turn on the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(charge_off), "turn off the charge controller", GR_POWER | CONFIRM},
  {COMMAND(charge_on), "turn on the charge controller", GR_POWER},
  {COMMAND(charge_cycle), "power cycle the charge controller", GR_POWER | CONFIRM},

  {COMMAND(mag_reset), "command a reset of the magnetometer", GRPOS_VETO | GRPOS_TRIM},
  {COMMAND(reset_rw), "reset the serial connection to the RW controller", GR_MOTOR},
  {COMMAND(reset_piv), "reset the serial connection to the pivot controller", GR_MOTOR},
  {COMMAND(reset_elev), "reset the serial connection to the elev controller", GR_MOTOR},
  {COMMAND(reset_ethercat), "reset communications with all EtherCat devices", GR_MOTOR},
  {COMMAND(restore_piv), "restore the serial settings for the pivot controller", GR_MOTOR},
  {COMMAND(az_off), "disable az motors' gains", GR_MOTOR},
  {COMMAND(az_on), "enable az motors' gains", GR_MOTOR},
  {COMMAND(el_off), "disable el motor gains", GR_MOTOR},
  {COMMAND(el_on), "enable el motor gains", GR_MOTOR},
  {COMMAND(force_el_on), "force enable el motors despite the pin being in", CONFIRM | GR_MOTOR},

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
  {COMMAND(mag_veto_fc1), "veto magnotometer attached to fc1", GR_VETO},
  {COMMAND(mag_allow_fc1), "un-veto magnetometer attached to fc1", GR_VETO},
  {COMMAND(mag_veto_fc2), "veto magnotometer attached to fc2", GR_VETO},
  {COMMAND(mag_allow_fc2), "un-veto magnetometer attached to fc2", GR_VETO},
  {COMMAND(pss_veto), "veto pss sensor", GR_VETO},
  {COMMAND(pss_allow), "un-veto pss sensor", GR_VETO},
  {COMMAND(dgps_veto), "veto CSBF DGPS sensor", GR_VETO},
  {COMMAND(dgps_allow), "un-veto CSBF DGPS sensor", GR_VETO},
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
  {COMMAND(trim_to_xsc0), "trim coarse sensors to XSC0 (disables autotrim)", GR_TRIM},
  {COMMAND(trim_to_xsc1), "trim coarse sensors to XSC1 (disables autotrim)", GR_TRIM},
  {COMMAND(trim_xsc1_to_xsc0), "trim XSC1 to XSC0", GR_TRIM},
  {COMMAND(trim_xsc0_to_xsc1), "trim XSC0 to XSC1", GR_TRIM},
  {COMMAND(autotrim_off), "disable auto-trim to XSC0/XSC1", GR_TRIM},
  {COMMAND(fixed), "fixed level bias", GR_BIAS},
  {COMMAND(ramp), "ramp bias with triangular waveform", GR_BIAS},

  // {COMMAND(bda_on), "manually turn 300mK BDA heater on", GR_CRYO},
  // {COMMAND(bda_off), "manually turn 300mK BDA heater off", GR_CRYO},
  // {COMMAND(hs_pot_on), "pot heat switch on", GR_CRYO},
  // {COMMAND(hs_pot_off), "pot heat switch off", GR_CRYO},

  // {COMMAND(cal_on), "calibrator on", GR_CRYO},
  // {COMMAND(cal_off), "calibrator off", GR_CRYO},
  {COMMAND(hwpr_enc_on), "HWP rotation sensor on", GR_CRYO | GR_HWPR},
  {COMMAND(hwpr_enc_off), "HWP rotation sensor off", GR_CRYO | GR_HWPR},
  {COMMAND(hwpr_enc_pulse), "HWP rotation sensor pulse", GR_CRYO | GR_HWPR},

  // Old commands, should we delete? PAW 2018/03/27
  // {COMMAND(ln_valve_on), "ln tank valve on", GR_CRYO},
  // {COMMAND(ln_valve_off), "ln tank valve off", GR_CRYO},
  // {COMMAND(he_valve_on), "he4 tank valve on", GR_CRYO},
  // {COMMAND(he_valve_off), "he4 tank valve off", GR_CRYO},
  // {COMMAND(l_valve_open), "set he4 AND ln tank valve direction open", GR_CRYO},
  // {COMMAND(l_valve_close), "set he4 AND ln tank valve direction close", GR_CRYO},

  {COMMAND(potvalve_on), "Turn He4 pot valve on (will accept move commands)", GR_CRYO | CONFIRM},
  {COMMAND(potvalve_off), "Turn He4 pot valve off (stops the motor, will not accept move commands", GR_CRYO},
  {COMMAND(potvalve_open), "set He4 pot valve direction open", GR_CRYO},
  {COMMAND(potvalve_close), "set He4 pot valve direction close", GR_CRYO},
  {COMMAND(pump_valve_open), "open pump valve", GR_CRYO},
  {COMMAND(pump_valve_close), "close pump valve", GR_CRYO},
  {COMMAND(pump_valve_off), "stop pump valve, reset goal to 0", GR_CRYO},
  {COMMAND(pump_valve_on), "re-enable pump valve", GR_CRYO},
  {COMMAND(fill_valve_open), "open fill valve", GR_CRYO},
  {COMMAND(fill_valve_close), "close fill valve", GR_CRYO},
  {COMMAND(fill_valve_off), "stop fill valve, reset goal to 0", GR_CRYO},
  {COMMAND(fill_valve_on), "re-enable pump valve", GR_CRYO},

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
  {COMMAND(reap_north), "ask MCP to reap the north watchdog tickle", GR_MISC | CONFIRM},
  {COMMAND(reap_south), "ask MCP to reap the south watchdog tickle", GR_MISC | CONFIRM},
  {COMMAND(xy_panic), "stop XY stage motors immediately", GR_MISC},

  {COMMAND(balance_auto), "Put balance system into auto mode", GR_BAL},
  {COMMAND(balance_off),  "Turn off the balance motor", GR_BAL},
  {COMMAND(balance_terminate),  "Drive balance system to lower limit, after sending lock45,"
	  "before termination", GR_BAL},

  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK | CONFIRM},
  {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
  {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},
  {COMMAND(lock45), "Lock the inner frame at 45 degrees", GR_LOCK},
  {COMMAND(repoll), "force repoll of the stepper busses (act, lock, HWPR, XY)",
    GR_LOCK | GR_ACT | GR_HWPR},
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

  // Shutter commands
  {COMMAND(shutter_init), "Initialize shutter move parameters", GR_MISC},
  {COMMAND(shutter_close), "Close shutter and keep it closed", GR_MISC},
  {COMMAND(shutter_reset), "Reset shutter; shutter will open", GR_MISC},
  {COMMAND(shutter_open), "Open shutter", GR_MISC},
  {COMMAND(shutter_open_close), "If shutter is open, then open completely and then close", GR_MISC},
  {COMMAND(shutter_off), "Turn off shutter; shutter will fall open", GR_MISC},
  {COMMAND(shutter_close_slow), "Close shutter using opto feedback and keep it closed", GR_MISC},
  {COMMAND(vna_sweep_all), "(All Roaches) Do VNA sweeps", CONFIRM | GR_ROACH},
  {COMMAND(targ_sweep_all), "(All Roaches) Do TARG sweeps", GR_ROACH},
  {COMMAND(find_kids_default_all), "(All Roaches) Find frequencies using VNA sweeps", GR_ROACH},
  {COMMAND(center_lo_all), "(All Roaches) recenter LOs", GR_ROACH},
  {COMMAND(calc_dfs), "(All Roaches) Calculate df for all channels", GR_ROACH},
  {COMMAND(change_amps), "Writes the tone amplitudes contained in roach->last_amps", GR_ROACH},
  {COMMAND(load_freqs_all), "(All Roaches) Write all saved targ freqs", GR_ROACH},
  {COMMAND(reload_vna_all), "(All Roaches) Reload vna freqs and vna trf", GR_ROACH},
  {COMMAND(end_sweeps_all), "(All Roaches) End all sweeps", GR_ROACH},
  {COMMAND(new_ref_params_all), "(All Roaches) Calculates and saves ref params from last target sweep", GR_ROACH},
  {COMMAND(set_attens_default), "(All Roaches) Set all attens to default values", GR_ROACH},
  {COMMAND(auto_find_kids_all), "(All Roaches) on startup, do VNA sweep, find kids and write tones", GR_ROACH},
  {COMMAND(zero_df_all), "(All Roaches) zero the delta fs", GR_ROACH},
  {COMMAND(reset_roach_all), "(All Roaches) reinitialize all Roaches from BOOT state", GR_ROACH},
  {COMMAND(flight_mode), "(All Roaches) resets all state/status fields, goes full auto", GR_ROACH},
  {COMMAND(debug_mode), "(All Roaches) Undoes flight mode, put in manual mode", GR_ROACH},
  {COMMAND(change_freqs_all), "(All Roaches) Apply delta f to targ tones, rewrite comb", GR_ROACH},
  {COMMAND(set_attens_last_all),
     "(All Roaches) Set all attens to previous settings (e.g., after hard reset)", GR_ROACH},
  {COMMAND(xyzzy), "nothing happens here", GR_MISC}
};

/* parameter type:
 * i :  parameter is 16 bit unnormalised integer. Max is CMD_I_MAX
 * l :  parameter is 32 bit unnormalised integer. Max is CMD_L_MAX
 * f :  parameter is 16 bit renormalised floating point
 * d :  parameter is 32 bit renormalised floating point
 * s :  parameter is 7-bit character string JOY: actually 32 char long
 */
struct mcom mcommands[plugh + 2] = {
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
  {COMMAND(mag_cal_fc1), "set fc1 magnetometer calibration", GR_TRIM, 5,
    {
      {"Max X", -20, 20, 'd', "CAL_XMAX_MAG1"},
      {"Min X", -20, 20, 'd', "CAL_XMIN_MAG1"},
      {"Max Y", -20, 20, 'd', "CAL_YMAX_MAG1"},
      {"Min Y", -20, 20, 'd', "CAL_YMIN_MAG1"},
      {"Mag Angle Offset", -180.0, 180.0, 'f', "CAL_ALIGNMENT_MAG1"}
    }
  }, // 10 10 10.5 10.34
  {COMMAND(mag_cal_fc2), "set fc2 magnetometer calibration", GR_TRIM, 5,
    {
      {"Max X", -20, 20, 'd', "CAL_XMAX_MAG2"},
      {"Min X", -20, 20, 'd', "CAL_XMIN_MAG2"},
      {"Max Y", -20, 20, 'd', "CAL_YMAX_MAG2"},
      {"Min Y", -20, 20, 'd', "CAL_YMIN_MAG2"},
      {"Mag Angle Offset", -180.0, 180.0, 'f', "CAL_ALIGNMENT_MAG2"}
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
  {COMMAND(az_gain), "az reaction wheel gains", GR_MOTOR, 4,
    {
      {"Proportional Gain", 0, CMD_I_MAX, 'f', "g_p_az"},
      {"Integral Time",     0, 200, 'f', "g_i_az"},
      {"Derivative Time",     0, 200, 'f', "g_d_az"},
      {"Pointing Gain", 0, CMD_I_MAX, 'f', "g_pt_az"},
    }
  },
  {COMMAND(az_scan_accel), "set azimuth scan turnaround acceleration", GR_MOTOR, 1,
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
  {COMMAND(balance_manual), "Manually set balance on", GR_BAL, 1,
    {
      {"dir (pos=1, neg=-1, off=0)",           -1, 1, 'i', "NONE"},
    }
  },
  {COMMAND(balance_vel), "set the balance system velocity and acceleration",
    GR_BAL, 2,
    {
      {"Velocity", 5, 500000, 'l', "VEL_BAL"},
      {"Acceleration", 1, 1000, 'i', "ACC_BAL"},
    }
  },
  {COMMAND(balance_i), "set the balance system currents", GR_BAL, 2,
    {
      {"Move current (%)", 0, 100, 'i', "I_MOVE_BAL"},
      {"Hold current (%)", 0,  50, 'i', "I_HOLD_BAL"},
    }
  },
  {COMMAND(balance_gain), "Set balance system setpoints", GR_BAL, 2,
    {
      {"I_El Balance On (A)",  0, 5, 'f', "I_LEVEL_ON_BAL"},
      {"I_El Balance Off  (A)", 0, 5, 'f', "I_LEVEL_OFF_BAL"},
    }
  },
  {COMMAND(box), "scan an az/el box centred on RA/Dec with el steps", GR_POINT, 7,
    {
      {"RA of Centre (h)",          0, 24, 'd', "RA"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0, 200, 'i', "n_dith"}
    }
  },
  {COMMAND(el_box), "scan an az/el box centred on RA/Dec with az steps", GR_POINT, 7,
    {
      {"RA of Centre (h)",          0, 24, 'd', "RA"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
      {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
      {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
      {"El Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"Az Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0, 200, 'i', "n_dith"}
    }
  },
  {COMMAND(cap), "scan a circle centred on RA/Dec with el steps", GR_POINT, 6,
    {
      {"RA of Centre (h)",          0, 24, 'd', "RA"},
      {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
      {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
      {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
      {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
      {"No. of dither steps",       0, 200, 'i', "n_dith"}
    }
  },
  {COMMAND(drift), "move at constant speed in az and el", GR_POINT, 2,
    {
      {"Az Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"},
      {"El Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"}
    }
  },
  {COMMAND(cur_mode), "drive motors at constant current", GR_POINT, 3,
    {
      {"Pivot Current (Amps)", -20.0, 20.0, 'f', "0.0"},
      {"RW Current (Amps)", -20.0, 20.0, 'f', "0.0"},
      {"Elevation Current (Amps)", -20.0, 20.0, 'f', "0.0"}
    }
  },
  {COMMAND(quad), "scan a quadrilateral region in RA/Dec (corners must be ordered)", GR_POINT, 11,
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
      {"No. of dither steps",       0, 200, 'i', "n_dith"}
    }
  },
  {COMMAND(vbox), "DEPRECATED - scan an az/el box centred on RA/Dec with el drift", GR_POINT, 6,
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
      {"Current Longitude (deg)", -360, 360, 'f', "LON"}
    }
  },
  {COMMAND(pivot_gain), "pivot gains", GR_MOTOR, 5,
    {
      {"Set Point (dps)",   -200, 200, 'f', "SET_RW"},
      {"V_err Gain (prop)", 0, CMD_L_MAX, 'f', "G_PE_PIV"},
      {"V_RW Gain (prop)", 0, CMD_L_MAX, 'f', "G_PV_PIV"},
      {"V_RW Integral time", 0, 200, 'f', "G_IV_PIV"},
      {"Static Friction offset",   0, 100, 'f', "FRICT_OFF_PIV"},
    }
  },
  {COMMAND(el_gain), "elevation motor gains", GR_MOTOR, 6,
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
{COMMAND(fix_ethercat), "Attempt to fix EC device? (1=yes, 0=no)", GR_MOTOR, 4,
    {
      {"RW", 0, 1, 'i', "NONE"},
      {"El", 0, 1, 'i', "NONE"},
      {"Pivot", 0, 1, 'i', "NONE"},
      {"HWP", 0, 1, 'i', "NONE"},
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
    GR_ACT | GR_LOCK | GR_HWPR | GR_BAL, 2,
    {
      {"Address (1-10)", 1, 0x2F, 'i', "1.0"},
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
  {COMMAND(hwpr_goto_rel), "move the waveplate rotator to relative position",
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
      {"Position 1", 0.0, 360.0, 'f', "POS0_HWPR"},
      {"Position 2", 0.0, 360.0, 'f', "POS1_HWPR"},
      {"Position 3", 0.0, 360.0, 'f', "POS2_HWPR"},
      {"Position 4", 0.0, 360.0, 'f', "POS3_HWPR"}
    }
  },
  {COMMAND(hwpr_goto_pot),
    "Move wave plate rotator to commanded encoder value",
    GR_HWPR, 1,
    {
      {"Encoder Value ", 0.0, 360.0, 'f', "POT_HWPR"},
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
  {COMMAND(hwpr_set_margin),
    "Set HWPR margin for determinting which indexed position we are at",
    GR_HWPR, 1,
    {
      {"hwpr margin", 0, 64000, 'i', "NONE"},
    }
  },
  /* XY Stage */
  {COMMAND(xy_goto), "move the X-Y translation stage to absolute position",
    GR_MISC, 4,
    {
      {"X destination", 0, 80000, 'l', "X_STAGE"},
      {"Y destination", 0, 80000, 'l', "Y_STAGE"},
      {"X speed", 0, 16000, 'i', "X_VEL_STAGE"},
      {"Y speed", 0, 16000, 'i', "Y_VEL_STAGE"}
    }
  },
  {COMMAND(xy_jump), "move the X-Y translation stage to relative position",
    GR_MISC, 4,
    {
      {"X delta", -80000, 80000, 'l', "0"},
      {"Y delta", -80000, 80000, 'l', "0"},
      {"X speed", 0, 16000, 'i', "X_VEL_STAGE"},
      {"Y speed", 0, 16000, 'i', "Y_VEL_STAGE"}
    }
  },
  {COMMAND(xy_xscan), "scan the X-Y translation stage in X", GR_MISC, 3,
    {
      {"X center", 0, 80000, 'l', "X_STAGE"},
      {"delta X", 0, 80000, 'l', "NONE"},
      {"X speed", 0, 16000, 'i', "X_VEL_STAGE"},
    }
  },
  {COMMAND(xy_yscan), "scan the X-Y translation stage in Y", GR_MISC, 3,
    {
      {"Y center", 0, 80000, 'l', "Y_STAGE"},
      {"delta Y", 0, 80000, 'l', "NONE"},
      {"Y speed", 0, 16000, 'i', "Y_VEL_STAGE"},
    }
  },
  {COMMAND(xy_raster), "raster the X-Y translation stage", GR_MISC, 7,
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

  {COMMAND(set_linklists), "change linklists for downlink", GR_TELEM, 2,
    {
      {"Downlink", 0, 3, 'i', "NONE", {downlink_names}},
      {LINKLIST_SELECT}
    }
  },

  {COMMAND(request_file), "send a specified file to a linklist", GR_TELEM, 2,
    {
      {LINKLIST_SELECT},
      {"Absolute file path", 0, 64, 's', ""}
    }
  },

  {COMMAND(biphase_clk_speed), "mpsse clock speed", GR_TELEM, 1,
    {
      {"Clock speed (kbps)", 100, 2000, 'i', "mpsse_clock_speed"}
    }
  },

  {COMMAND(highrate_through_tdrss), "Highrate downlink", GR_TELEM, 1,
    {
      {"TDRSS(1) or Iridium(0)", 0, 1, 'i', "NONE"}
    }
  },

  {COMMAND(highrate_bw), "Highrate bandwidth", GR_TELEM, 2,
    {
      {"Bandwidth (kbps)", 0, 500, 'f', "rate_highrate"},
      {"Allframe fraction", 0, 1, 'f', "aff_highrate"}
    }
  },

  {COMMAND(biphase_bw), "biphase bandwidth", GR_TELEM, 2,
    {
      {"Bandwidth (kbps)", 1, 2000, 'f', "rate_biphase"},
      {"Allframe fraction", 0, 1, 'f', "aff_biphase"}
    }
  },

  {COMMAND(pilot_bw), "pilot bandwidth", GR_TELEM, 2,
    {
      {"Bandwidth (kbps)", 0, 80000, 'f', "rate_pilot"},
      {"Allframe fraction", 0, 1, 'f', "aff_pilot"}
    }
  },
  {COMMAND(set_roach_mode), "0=normal, 1=delta", GR_TELEM, 1,
    {
      {"Roach DL mode", 0, 1, 'i', "NONE"},
    }
  },
  {COMMAND(set_roach_all_chan), "Send lots of kids for a given roach", GR_TELEM, 2,
    {
      {"Roach", 1, 5, 'i', "NONE"},
      {"Number of kids", 0, 1024, 'i', "NONE"},
    }
  },
  {COMMAND(set_roach_chan), "Select 5 I/Q/df channel triplets", GR_TELEM, 10,
    {
      {"Kid A-C", 0, 1023, 'i', "NONE"},
      {"Roach A-C", 1, 5, 'i', "NONE"},
      {"Kid D-F", 0, 1023, 'i', "NONE"},
      {"Roach D-F", 1, 5, 'i', "NONE"},
      {"Kid G-I", 0, 1023, 'i', "NONE"},
      {"Roach G-I", 1, 5, 'i', "NONE"},
      {"Kid J-L", 0, 1023, 'i', "NONE"},
      {"Roach J-L", 1, 5, 'i', "NONE"},
      {"Kid M-O", 0, 1023, 'i', "NONE"},
      {"Roach M-O", 1, 5, 'i', "NONE"}
    }
  },

  /****************************************/
  /*************** Misc.  *****************/
  // {COMMAND(t_gyro_gain), "gyro box heater gains", GR_ELECT, 3,
  //   {
  //     {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_heat_gy"},
  //     {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_heat_gy"},
  //     {"Derivative Gain",  0, MAX_15BIT, 'i', "g_d_heat_gy"}
  //   }
  // },
  // {COMMAND(t_gyro_set), "gyro box temperature set point", GR_ELECT, 1,
  //   {
  //     {"Set Point (deg C)", 0, 60, 'f', "T_SET_GY"}
  //   }
  // },

// *****************************************
// ROACH Commands
// *****************************************
  {COMMAND(load_new_vna_amps), "loads new VNA amplitudes from file", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"APPLY TRF FILE[1 = default (all 1), 2 = apply trf]", 1, 2, 'i', "NONE"},
    }
  },
  {COMMAND(load_new_targ_amps), "loads new TARG amplitudes from file", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"APPLY TRF FILE[1 = default, 2 = apply trf, 3 = apply last]", 1, 3, 'i', "NONE"},
    }
  },
  {COMMAND(cal_adc), "Calibrate ADC RMS voltage using input atten", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(end_sweep), "exit sweep", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(vna_sweep), "perform a new VNA sweep", CONFIRM | GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
    }
  },
  {COMMAND(cal_sweeps), "perform a new set of cal sweeps", GR_ROACH, 4,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Atten step (dB)", 0.5, 6.0, 'f', "NONE"},
      {"Number of sweep points", 5, 101, 'f', "NONE"},
      {"Number of cycles (sweeps)", 2, 20, 'i', "NONE"},
    }
  },
  {COMMAND(targ_sweep), "perform a new TARG sweep", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(reset_roach), "re-upload roach firmware & recalibrate", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(calc_df), "Calculate df for each channel", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Channel number", 0, 1015, 'i', "NONE"},
    }
  },
  {COMMAND(auto_retune), "Set mcp to retune the kid freqs based on settings in roach_check_retune()", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"0 = Manually retune, 1 = Auto retune"}
    }
  },
  {COMMAND(opt_tones), "Attempt to fine tune targ tones found by get_targ_freqs()", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"0 = Default, off, 1 = Run", 0, 1, 'i', "NONE"}
    }
  },
  {COMMAND(find_kids), "Set the parameters for the kid finding algorithm", GR_ROACH, 4,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"smoothing scale (kHz)", 1000.0, 100000.0, 'f', "NONE"},
      {"peak threshold (dB)", 0.1, 100.0, 'f', "NONE"},
      {"spacing threshold (kHz)", 100.0, 10000.0, 'f', "NONE"},
    }
  },
  {COMMAND(set_attens), "Set attenuators", GR_ROACH, 3,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"rf_out_level", 0.0, 30.0, 'f', "NONE"},
      {"rf_in_level", 0.0, 30.0, 'f', "NONE"},
    }
  },
  {COMMAND(read_attens), "Read attenuators", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
    }
  },
  {COMMAND(reboot_pi), "Attempt to reboot unresponsive Pi", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
    }
  },
  {COMMAND(set_attens_all), "Set all attenuators to same values (input/output)", GR_ROACH, 2,
    {
      {"rf_out_level", 0.0, 30.0, 'f', "NONE"},
      {"rf_in_level", 0.0, 30.0, 'f', "NONE"},
    }
  },
  {COMMAND(new_output_atten), "Set only output atten", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"new_out_atten", 0.0, 30.0, 'f', "NONE"}
    }
  },
  {COMMAND(show_adc_rms), "Print the ADC rms voltages to the log", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(test_tone), "Writes a single test tone to the DAC comb", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Test tone in Hz, between 1 - 250 MHz", 1.0e6, 250.0e6, 'f', "NONE"},
    }
  },
  {COMMAND(change_state), "Change Roach state", GR_ROACH, 3,
  {
    {"ROACH no", 1, 5, 'i', "NONE"},
    {"ROACH new state", 0, 11, 'i', "NONE"},
    {"ROACH desired state", 0, 11, 'i', "NONE"},
  }
  },
  {COMMAND(get_state), "Print current Roach state to MCP log", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(calc_phase_centers), "Calculate channel phase centers from TARG sweep", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(timestream), "Save a short IQ timestream", GR_ROACH, 3,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Channel no", 0, 1000, 'i', "NONE"},
      {"Number of sec to stream", 0, 300, 'f', "NONE"},
    }
  },
  {COMMAND(all_roach_ts), "Save IQ timestreams for all Roaches", GR_ROACH, 1,
    {
      {"Number of sec to stream", 0, 300, 'f', "NONE"},
    }
  },
  {COMMAND(cal_amps), "Tune channel responsivity with optical chop", GR_ROACH, 5,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Number of seconds to avg data", 0, 10.0, 'f', "NONE"},
      {"Number of cycles for cal", 0, 10.0, 'i', "NONE"},
      {"Delta amp", 0, 10.0, 'f', "NONE"},
      {"Desired response threshold", 0, 30000, 'i', "NONE"},
    }
  },
  {COMMAND(refit_freqs), "Performs a short sweep, fits res freqs and rewrites comb", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Find on res, or find max IQ grad", 0, 1, 'i', "NONE"},
    }
  },
  {COMMAND(refit_freqs_all), "Refit freqs on all Roaches", GR_ROACH, 1,
    {
      {"Find on res, or find max IQ grad", 0, 1, 'i', "NONE"},
    }
  },
  {COMMAND(chop_template), "Saves timestreams for all channel and calculates avg chop", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(load_freqs), "loads a list of tone freqs from file", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(new_ref_params), "calculates and saves ref params from last (or reference) target sweep", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(retune), "Calculate df for each channel", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
    }
  },
  {COMMAND(check_retune), "Calculate df for each channel", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
    }
  },
  {COMMAND(offset_lo), "shift LO by specified amount in Hz", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Amount to shift LO", -1000000., 1000000., 'f', "NONE"},
    }
  },
  {COMMAND(offset_lo_all), "shift all LOs by specified amount in Hz", GR_ROACH, 1,
    {
      {"Amount to shift LO", -1000000.0, 1000000.0, 'f', "NONE"},
    }
  },
  {COMMAND(center_lo), "recenter the LO", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(find_kids_default), "Find res freqs from VNA sweeps using default params", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(change_amp), "Shifts the amplitude of specified channel bias by delta amp", GR_ROACH, 3,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Channel number", 0, 1015, 'i', "NONE"},
      {"delta_amp", -5, 5, 'f', "NONE"},
    }
  },
  {COMMAND(change_freq), "Shifts freq of single chan by m_roach->df", GR_ROACH, 2,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Channel number", 0, 1015, 'i', "NONE"},
    }
  },
  {COMMAND(change_phase), "Shifts the phase of specified channel by delta phase", GR_ROACH, 3,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Channel number", 0, 1015, 'i', "NONE"},
      {"delta_phase", -3.14159, 3.14159, 'f', "NONE"},
    }
  },
  {COMMAND(offset_freq), "Shifts the freq of specified channel by freq offset", GR_ROACH, 3,
    {
      {"ROACH no", 1, 5, 'i', "NONE"},
      {"Channel number", 0, 1015, 'i', "NONE"},
      {"delta_freq", -1000000.0, 1000000.0, 'f', "NONE"},
    }
  },
  {COMMAND(auto_find_kids), "Automatically do a VNA sweep, find kids and write tonest", GR_ROACH, 1,
    {
      {"ROACH no", 1, 5, 'i', "NONE"}
    }
  },
  {COMMAND(lamp_check_all), "(All Roaches) Checks response to cal lamp (I,Q,mag(I,Q)) and saves to disk", GR_ROACH, 1,
    {
      {"Number of sec to stream", 0, 300, 'f', "NONE"},
    }
  },
  /***************************************/
  /*************** ROX Bias  *************/
  {COMMAND(set_rox_bias_amp), "Set the ROX bias amplitude", GR_CRYO, 1,
    {
      {"ROX bias amplitude (0-64)", 0, 64, 'i', "NONE"}
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
      {"Array (250, 350, 500, 0=all)", 0, 32767, 'i', "step_array_bias"},
    }
  },
  // {COMMAND(phase_step), "step through different phases", GR_BIAS, 4,
  //   {
  //     {"Start", 0, 32767, 'i', "STEP_START_PHASE"},
  //     {"End", 0, 32767, 'i', "STEP_END_PHASE"},
  //     {"N steps", 1, 32767, 'i', "step_nsteps_phase"},
  //     {"Time per step (ms)", 1, 32767, 'i', "step_time_phase"},
  //   }
  // },
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
  // {COMMAND(cal_repeat), "set calibrator to automatic repeated pulse mode", GR_CRYO, 3,
  //   {
  //     {"Pulse Length (ms)", 10, 8000, 'i', "PULSE_CAL"},
  //     {"Max Pulse Delay (0=never pulse) (s)",  0, 32767, 'i', "PERIOD_CAL"},
  //     {"Always Pulse before HWP move (0=no, 1=yes)",  0, 1, 'i', "NONE"}
  //   }
  // },
  {COMMAND(cal_length), "set length of calibration pulse", GR_CRYO, 1,
      {
          {"Pulse Length (ms)", 5, 5000, 'i', "PULSE_CAL"}
      }
  },
  {COMMAND(level_length), "set length of level sensor pulse", GR_CRYO, 1,
      {
          {"Pulse Length (s)", 5, 5000, 'i', "PULSE_LEVEL"}
      }
  },
  {COMMAND(periodic_cal), "periodic cal pulses sent", GR_CRYO, 3,
      {
          {"Number of Pulses", 1, 1000, 'i', "NUM_PULSE"},
          {"Separation (in 5ms steps)", 2, 30000, 'i', "SEPARATION"},
          {"Length of Pulse (in 5ms steps)", 2, 30000, 'i', "LENGTH_PULSE"},
      }
  },
  {COMMAND(set_queue_execute), "command queue changed", GR_CRYO, 1,
      {
          {"Labjack to execute queue", 0, 4, 'i', "LJ"},
      }
  },
  {COMMAND(reconnect_lj), "rebooting labjack cryo 1", GR_CRYO, 1,
      {
          {"Labjack to reconnect", 1, 5, 'i', "NONE"},
      }
  },
  /***************************************/
  /********* Cryo heat   *****************/
  {COMMAND(send_dac), "turning on dac0 to specified voltage on specified labjack",
      GR_CRYO, 2, {
      {"Voltage", 0., 5., 'f', "VOLTS_TO_DAC"},
      {"Labjack - not 1", 0, 4, 'i', "LABJACK"}
      }
  },
  // {COMMAND(jfet_set), "jfet heater setpoints", GR_CRYO, 2,
  //   {
  //     {"On Point (K)", 0, 400., 'f', "JFET_SET_ON"},
  //     {"Off Point (K)", 0, 400., 'f', "JFET_SET_OFF"}
  //   }
  // },

  // {COMMAND(fridge_cycle_params), "Fridge cycle parameters", GR_CRYO, 6,
  //   {
  //     {"300mK_strap Start Temp (K)", 0, 4., 'f', "T_START_CYCLE"},
  //     {"Pot Max Temp (K)", 0, 10., 'f', "T_POT_MAX_CYCLE"},
  //     {"Charcoal Max Temp (K)", 0, 70., 'f', "T_CHAR_MAX_CYCLE"},
  //     {"Charcoal Timeout (min)", 0, 120., 'f', "TIME_CHAR_CYCLE"},
  //     {"Charcoal Settled Temp (K)", 0, 70., 'f', "T_CHAR_SET_CYCLE"},
  //     {"Charcoal Settle Time (min)", 0, 120., 'f', "TIME_SET_CYCLE"}
  //   }
  // },
  {COMMAND(actuators_set_used), "Set each stepper as used (1) or not used (0)", GR_CRYO | GR_HWPR | GR_BAL | GR_ACT, 10,
    {
      {"Actuator #0", 0, 1, 'i', "NONE"},
      {"Actuator #1", 0, 1, 'i', "NONE"},
      {"Actuator #2", 0, 1, 'i', "NONE"},
      {"Balance", 0, 1, 'i', "NONE"},
      {"Lockpin", 0, 1, 'i', "NONE"},
      {"HWPR", 0, 1, 'i', "NONE"},
      {"Shutter", 0, 1, 'i', "NONE"},
      {"Pumped Pot Valve", 0, 1, 'i', "NONE"},
      {"Pump Valve", 0, 1, 'i', "NONE"},
      {"Fill Valve", 0, 1, 'i', "NONE"},
    }
  },

  {COMMAND(potvalve_set_vel), "Set pot valve motor velocity", GR_CRYO, 1,
    {
      {"Velocity (microsteps/sec)", 0, 100000, 'i', "POTVALVE_VEL"}
    }
  },

  {COMMAND(potvalve_set_current), "Set pot valve open and close currents", GR_CRYO, 2,
    {
      {"Pot valve open current (% max)", 0, 100, 'i', "POTVALVE_I_OPEN"},
      {"Pot valve close current (% max)", 0, 100, 'i', "POTVALVE_I_CLOSE"}
    }
  },

  {COMMAND(potvalve_set_thresholds), "Set pumped pot valve thresholds", GR_CRYO, 3,
    {
      {"Closed threshold (1000-8000)", 1000, 8000, 'i', "THRESH_CLOS_POTVALVE"},
      {"Loose close threshold (8100-10000)", 8200, 10000, 'i', "THRESHLCLOS_POTVALVE"},
      {"Open threshold (10100-16000)", 10200, 16000, 'i', "THRESH_OPEN_POTVALVE"},
    }
  },

  {COMMAND(valves_set_vel), "Set cryostat valves velocity", GR_CRYO, 1,
    {
      {"Cryostat valves velocity (microsteps/sec)", 0, 100000, 'i', "VEL_VALVES"}
    }
  },

  {COMMAND(valves_set_move_i), "Set cryostat valves move current", GR_CRYO, 1,
    {
      {"Cryostat valves move current (% max)", 0, 100, 'i', "CURRENT_VALVES"}
    }
  },

  {COMMAND(valves_set_hold_i), "Set cryostat valves hold current", GR_CRYO, 1,
    {
      {"Cryostat valves hold current (up to 50%)", 0, 50, 'i', "CURRENT_VALVES"}
    }
  },

  {COMMAND(valves_set_acc), "Set cryostat valves acceleration", GR_CRYO, 1,
    {
      {"Cryostat valves acceleration", 0, 6000, 'i', "ACC_VALVES"}
    }
  },

  //  <!-- XSC general -->





////  <!-- XSC exposure -->

    {COMMAND(xsc_exposure_timing), "xsc exposure time", GR_XSC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"exposure time_cs (cs) [default: 12]", 1, 100, 'l', "NONE"},
            {"grace period (s) (disregards which) [default: 45.0]", 1.0, 100.0, 'f', "NONE"},
            {"post trigger counter_mcp share delay (cs) (disregards which) [default: 200]", 1, 1000, 'l', "NONE"},
        },
    },


    {COMMAND(xsc_multi_trigger), "xsc trigger timing", GR_XSC_PARAM, 3,
        {
            {"num triggers [default: 1]", 0, 16, 'i', "NONE"},
            {"time between triggers (cs) [default: 19]", 1, 100, 'l', "NONE"},
            {"stars readout delay (s) [default: 1.0]", 0.1, 10.0, 'f', "NONE"},
        },
    },

////  <!-- XSC main -->


    {COMMAND(xsc_main_settings), "xsc main settings", GR_XSC_PARAM, 6,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"display_frequency [default: 20.0]", 1.0, 30.0, 'f', "NONE"},
            {"display_fullscreen [default: 1]", 0, 1, 'i', "NONE"},
            {"display_image_only [default: 0]", 0, 1, 'i', "NONE"},
            {"display_solving_filters [default: 0]]", 0, 1, 'i', "NONE"},
            {"display_image_brightness [default: 1.0]", 0.5, 4.0, 'f', "NONE"},
        },
    },


    {COMMAND(xsc_display_zoom), "xsc display zoom", GR_XSC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"center x display pixel [default: 320]", 0, 1000, 'i', "NONE"},
            {"center y display pixel [default: 240]", 0, 1000, 'i', "NONE"},
            {"zoom [default: 1.0]", 1.0, 4.0, 'f', "NONE"},
        },
    },

//    {COMMAND(xsc_network_reset), "Reset the xsc network", GR_XSC_PARAM, 4,
//        {
//            {"which", 0, 2, 'i', "NONE"},
//            {"reset now?", 0, 1, 'i', "NONE"},
//            {"enable lull?", 0, 1, 'i', "NONE"},
//            {"lull delay", 0.0, 30.0, 'f', "NONE"},
//        },
//    },



////  <!-- XSC imaging (lens, camera, fake sky, masking) -->



    {COMMAND(xsc_run_autofocus), "xsc run autofocus", GR_XSC_PARAM, 1,
        {
            {"which", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_set_autofocus_range), "xsc set autofocus range", GR_XSC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"focus_search_min", 0, 5000, 'l', "NONE"},
            {"focus_search_max", 0, 5000, 'l', "NONE"},
            {"focus_search_step", 1, 1000, 'l', "NONE"},
        },
    },

    {COMMAND(xsc_abort_autofocus), "xsc abort autofocus", GR_XSC_PARAM, 2,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"still_use_solution", 0, 1, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_autofocus_display_mode), "xsc autofocus display mode", GR_XSC_PARAM, 2,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"mode (0=auto, 1=on, 2=off)", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_get_gain), "xsc get gain", GR_XSC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_set_gain), "xsc set gain", GR_XSC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"position", 0.5, 10.0, 'f', "NONE"},
        },
    },


    {COMMAND(xsc_fake_sky_brightness), "xsc fake sky brightness", GR_XSC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"enabled", 0, 1, 'i', "NONE"},
            {"level_kepsa", 0.0, 10000.0, 'f', "NONE"},
            {"gain_db", -50, 10, 'f', "NONE"},
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
    {COMMAND(shutter_step), "set number of shutter steps to close (default 4224)", GR_MISC, 1,
        {
          {"Steps", 1, 5000, 'i', "STEPS_SHUTTER"},
        }
    },
    {COMMAND(shutter_step_slow), "set number of incremental shutter steps to close (default 300)", GR_MISC, 1,
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
    {COMMAND(xsc_is_new_window_period), "Set the time over which commands are valid (in centi-seconds)",
     GR_XSC_PARAM, 2,
        {
              {"Which", 0, 2, 'i', "NONE"},
              {"Window period", 0, 2000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_offset), "Trim the star camera", GR_XSC_PARAM|GR_TRIM, 3,
        {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Cross-El trim", -180, 180, 'd', "NONE"},
              {"El trim", -180, 180, 'd', "NONE"},
        }
    },

  ////  <!-- XSC heaters -->

  {COMMAND(xsc_heaters_off), "Turn off the XSC heater", GR_XSC_HOUSE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_heaters_on), "Turn on the XSC heater", GR_XSC_HOUSE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_heaters_auto), "Allow XSC to control its heater", GR_XSC_HOUSE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_trigger_threshold), "Allow XSC to trigger based on predicted px streaking", GR_XSC_PARAM, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Enabled", 0, 1, 'i', "NONE"},
              {"Blob streaking limit (pixels", 0, 100, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_scan_force_trigger), "Force XSC to trigger on turnaround (ignore speed)", GR_XSC_PARAM, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Force the trigger on scan limits", 0, 1, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_quit), "Quit XSC", GR_XSC_HOUSE|CONFIRM, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_shutdown), "Shutdown the XSC computer", GR_XSC_HOUSE|CONFIRM, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Restart?", 0, 1, 'i', "NONE"}
      }
  },

  {COMMAND(xsc_image_client), "Enable or disable sending images to mcp", GR_XSC_MODE, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Image client enabled", 0, 1, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_init_focus), "Initialize the focus motor", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_get_focus), "Get the absolute focus position", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_set_focus), "Set the absolute focus position", GR_XSC_MODE, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Absolute focus position", 0, 5000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_stop_focus), "Stop all motion on the focus actuator", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_define_focus), "Define the value of the focus at the current position", GR_XSC_MODE, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Focus value", 0, 5000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_set_focus_incremental), "Command an incremental step to the focus motor", GR_XSC_MODE, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Incremental focus steps", -5000, 5000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_init_aperture), "Initialize the aperture motor", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_get_aperture), "Get Aperture", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_set_aperture), "Set the absolute aperture position", GR_XSC_MODE, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Absolute aperture position", 0, 1000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_stop_aperture), "Stop all motion on the aperture actuator", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_define_aperture), "Define the value of the aperture at the current position", GR_XSC_MODE, 2,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Aperture value", 0, 1000, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_solver_general), "Solver parameter settings", GR_XSC_PARAM, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Enable the solver", 0, 1, 'i', "NONE"},
              {"Set the solver timeout period (s)", 5, 600, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_solver_abort), "Abort the solving the current image", GR_XSC_MODE, 1,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"}
      }
  },
  {COMMAND(xsc_selective_mask), "Set the XSC selective mask", GR_XSC_PARAM, 6,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Enable selective masking", 0, 1, 'i', "NONE"},
              {"Mask field 1", 0, CMD_L_MAX, 'i', "NONE"},
              {"Mask field 2", 0, CMD_L_MAX, 'i', "NONE"},
              {"Mask field 3", 0, CMD_L_MAX, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_blob_finding), "XSC blob finder settings", GR_XSC_PARAM, 5,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"SNR Threshhold", 0.01, 10, 'f', "NONE"},
              {"Max num blobs", 3, 100, 'i', "NONE"},
              {"Enable Robust mode", 0, 1, 'i', "NONE"},
              {"Fitting method (0= none, 1=gaussian, 2=double gaussian)", 0, 2, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_blob_cells), "XSC blob cell settings", GR_XSC_PARAM, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Cell size (pixels, power of 2)", 4, 512, 'i', "NONE"},
              {"Max num blobs per cell (default 2)", 1, 10, 'i', "NONE"},
      }
  },
  {COMMAND(xsc_pattern_matching), "XSC pattern matching settings", GR_XSC_PARAM, 8,
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
  {COMMAND(xsc_filter_hor_location), "XSC Horizontal Location Filter", GR_XSC_PARAM, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Horizontal Limit Enabled", 0, 1, 'i', "NONE"},
              {"Horizontal Radius (degrees)", 0.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_eq_location), "XSC Equatorial Location Filter", GR_XSC_PARAM, 3,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Equatorial Limit Enabled", 0, 1, 'i', "NONE"},
              {"Equatorial Radius (degrees)", 0.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_hor_roll), "XSC Horizontal Roll Limit", GR_XSC_PARAM, 4,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Horizontal Roll Limit", 0, 1, 'i', "NONE"},
              {"Minimum Horizontal Roll (degrees)", -180.0, 180.0, 'f', "NONE"},
              {"Maximum Horizontal Roll (degrees)", -180.0, 180.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_el), "XSC Elevation Limit", GR_XSC_PARAM, 4,
      {
              {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
              {"Elevation Limit", 0, 1, 'i', "NONE"},
              {"Minimum Elevation (degrees)", -90.0, 90.0, 'f', "NONE"},
              {"Maximum Elevation (degrees)", -90.0, 90.0, 'f', "NONE"},
      }
  },
  {COMMAND(xsc_filter_matching), "XSC Matching Filter", GR_XSC_PARAM, 4,
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
