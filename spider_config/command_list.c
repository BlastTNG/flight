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

#include <limits.h>

#include "command_list.h"
#ifdef __MCP__
#include "camstruct.h"
#endif


const char *command_list_serial = "$Rev$";

const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "CMB grenades",     "Waveplate Rotator",
  "Pointing Sensor Trims", "Aux. Electronics", "HK Bias",
  "Pointing Sensor Vetos", "Actuators",        "Long Range Missiles",
  "Pointing Motor Gains",  "Lock Motor",       "HK Insert Heat",
  "Subsystem Power",       "SC Table",         "HK Theo Heat",
  "Telemetry",             "The Good SC",      "Semi-Automatic Weapons",
  "X-Y Stage",             "The Bad SC",       "Knives and Swords",
  "Miscellaneous",         "The Ugly SC",      "Handguns"
  };

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},
  {COMMAND(antisun), "turn antisolar now", GR_POINT},

  {COMMAND(gps_off), "turn off the dGPS", GR_POWER | CONFIRM},
  {COMMAND(gps_on), "turn on the dGPS", GR_POWER},
  {COMMAND(gps_cycle), "power cycle the dGPS", GR_POWER | CONFIRM},
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
    | GR_ACT | CONFIRM},
  {COMMAND(actbus_on), "turn on the Actuators, Lock, and HWPR", GR_POWER 
    | GR_ACT},
  {COMMAND(actbus_cycle), "power cycle the Actuators, Lock, and HWPR", GR_POWER 
    | GR_ACT | CONFIRM},
  {COMMAND(rw_off), "turn off the reaction wheel motor", GR_POWER},
  {COMMAND(rw_on), "turn on the reaction wheel motor", GR_POWER},
  {COMMAND(rw_cycle), "power cycle the reaction wheel motor", GR_POWER},
  {COMMAND(piv_off), "turn off the pivot motor", GR_POWER},
  {COMMAND(piv_on), "turn on the pivot motor", GR_POWER},
  {COMMAND(piv_cycle), "power cycle the pivot motor", GR_POWER},
  {COMMAND(elmot_off), "turn off the elevation motor", GR_POWER},
  {COMMAND(elmot_on), "turn on the elevation motor", GR_POWER},
  {COMMAND(elmot_cycle), "power cycle the elevation motor", GR_POWER},
  {COMMAND(elmot_auto), "el motors automatically power on for a move, and then power off again", GR_POWER},
  {COMMAND(elmot_manual), "el motors must be powered on/off manually", GR_POWER},
  {COMMAND(vtx_off), "turn off the video transmitters", GR_TELEM | GR_POWER},
  {COMMAND(vtx_on), "turn on the video transmitters", GR_TELEM | GR_POWER},
  {COMMAND(bi0_off), "turn off the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(bi0_on), "turn on the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(thegood_off), "turn off The Good", GR_POWER},
  {COMMAND(thegood_on), "turn on The Good", GR_POWER},
  {COMMAND(thegood_cam_cycle), "power cycle The Good (camera,lens,heater)", GR_POWER},
  {COMMAND(thegood_cpu_cycle), "power cycle The Good (computer)", GR_POWER},
  {COMMAND(thebad_off), "turn off The Bad", GR_POWER},
  {COMMAND(thebad_on), "turn on The Bad", GR_POWER},
  {COMMAND(thebad_cam_cycle), "power cycle The Bad (camera,lens,heater)", GR_POWER},
  {COMMAND(thebad_cpu_cycle), "power cycle The Bad (computer)", GR_POWER},
  {COMMAND(theugly_off), "turn off the The Ugly", GR_POWER},
  {COMMAND(theugly_on), "turn on the The Ugly", GR_POWER},
  {COMMAND(theugly_cam_cycle), "power cycle The Ugly (camera,lens,heater)", GR_POWER},
  {COMMAND(theugly_cpu_cycle), "power cycle The Ugly (computer)", GR_POWER},
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
  {COMMAND(elenc1_veto), "veto elevation encoder 1", GR_VETO},
  {COMMAND(elenc2_veto), "veto elevation encoder 2", GR_VETO},
  {COMMAND(elenc1_allow), "un-veto elevation encoder 1", GR_VETO},
  {COMMAND(elenc2_allow), "un-veto elevation encoder 2", GR_VETO},
  {COMMAND(gps_veto), "veto differntial gps", GR_VETO},
  {COMMAND(gps_allow), "un-veto differential gps", GR_VETO},
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
  {COMMAND(vtx1_bsc), "put BSC video on transmitter #1", GR_TELEM},
  {COMMAND(vtx2_isc), "put ISC video on transmitter #2", GR_TELEM},
  {COMMAND(vtx2_osc), "put OSC video on transmitter #2", GR_TELEM},
  {COMMAND(vtx2_bsc), "put BSC video on transmitter #2", GR_TELEM},

  {COMMAND(halt_itsy), "ask MCP to halt *ITSY* MCC", GR_MISC | CONFIRM},
  {COMMAND(halt_bitsy), "ask MCP to halt *BITSY* MCC", GR_MISC | CONFIRM},
  {COMMAND(reap_itsy), "ask MCP to reap the *ITSY* watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(reap_bitsy), "ask MCP to reap the *BITSY* watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(bbc_sync_ext), "Set BBC to external (sync box) mode", 
    GR_MISC | CONFIRM},
  {COMMAND(bbc_sync_int), "Set BBC to internal sync mode", GR_MISC | CONFIRM},
  {COMMAND(bbc_sync_auto),
    "Auto-set BBC to external (sync box) mode if possible", GR_MISC | CONFIRM},
  {COMMAND(xy_panic), "stop XY stage motors immediately", GR_STAGE},

  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK | CONFIRM},
  {COMMAND(lock), "lock inner frame", GR_LOCK | GR_POINT},
  {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
  {COMMAND(lock_on), "turn on the lock motor", GR_LOCK},
  {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},
  {COMMAND(repoll), "force repoll of the stepper busses (act, lock, XY)",
    GR_STAGE | GR_ACT},
  {COMMAND(actuator_stop), "stop all secondary actuators immediately", GR_ACT},
  {COMMAND(hwp_repoll), "repoll HWP bus for stepper controllers", GR_HWPR},
  {COMMAND(hwp_panic), "stop all HWP rotators immediately", GR_HWPR},
  {COMMAND(hwp_step), "step the HWPs to their next position", GR_HWPR},


  //The Good commands
  {COMMAND(thegood_expose), "Start The Good exposure (in triggered mode)", GR_SCGOOD},
  {COMMAND(thegood_autofocus), "The Good autofocus mode", GR_SCGOOD},
  {COMMAND(thegood_settrig_ext), "Set external The Good trigger mode", GR_SCGOOD},
  {COMMAND(thegood_pause), "Stop automatic image capture", GR_SCGOOD},
  {COMMAND(thegood_run), "Start automatic image capture", GR_SCGOOD},
  //The Bad commands
  {COMMAND(thebad_expose), "Start The Bad exposure (in triggered mode)", GR_SCBAD},
  {COMMAND(thebad_autofocus), "The Bad autofocus mode", GR_SCBAD},
  {COMMAND(thebad_settrig_ext), "Set external The Bad trigger mode", GR_SCBAD},
  {COMMAND(thebad_pause), "Stop automatic image capture", GR_SCBAD},
  {COMMAND(thebad_run), "Start automatic image capture", GR_SCBAD},
  //The Ugly commands
  {COMMAND(theugly_expose), "Start The Ugly exposure (in triggered mode)", GR_SCUGLY},
  {COMMAND(theugly_autofocus), "The Ugly autofocus mode", GR_SCUGLY},
  {COMMAND(theugly_settrig_ext), "Set external The Ugly trigger mode", GR_SCUGLY},
  {COMMAND(theugly_pause), "Stop automatic image capture", GR_SCUGLY},
  {COMMAND(theugly_run), "Start automatic image capture", GR_SCUGLY},
  //Star Camera table
  {COMMAND(table_track), "Put the table in track mode", GR_SCTAB},
  //Theo heater housekeeping commands
  {COMMAND(hk_t0_heat_on), "Turn on Theo's Heater #0", GR_THEO_HEAT},
  {COMMAND(hk_t0_heat_off), "Turn off Theo's Heater #0", GR_THEO_HEAT},
  {COMMAND(hk_t1_heat_on), "Turn on Theo's Heater #1", GR_THEO_HEAT},
  {COMMAND(hk_t1_heat_off), "Turn off Theo's Heater #1", GR_THEO_HEAT},
  {COMMAND(hk_t2_heat_on), "Turn on Theo's Heater #2", GR_THEO_HEAT},
  {COMMAND(hk_t2_heat_off), "Turn off Theo's Heater #2", GR_THEO_HEAT},
  {COMMAND(hk_t3_heat_on), "Turn on Theo's Heater #3", GR_THEO_HEAT},
  {COMMAND(hk_t3_heat_off), "Turn off Theo's Heater #3", GR_THEO_HEAT},
  {COMMAND(hk_t4_heat_on), "Turn on Theo's Heater #4", GR_THEO_HEAT},
  {COMMAND(hk_t4_heat_off), "Turn off Theo's Heater #4", GR_THEO_HEAT},
  {COMMAND(hk_t5_heat_on), "Turn on Theo's Heater #5", GR_THEO_HEAT},
  {COMMAND(hk_t5_heat_off), "Turn off Theo's Heater #5", GR_THEO_HEAT},
  {COMMAND(hk_t6_heat_on), "Turn on Theo's Heater #6", GR_THEO_HEAT},
  {COMMAND(hk_t6_heat_off), "Turn off Theo's Heater #6", GR_THEO_HEAT},
  {COMMAND(hk_t7_heat_on), "Turn on Theo's Heater #7", GR_THEO_HEAT},
  {COMMAND(hk_t7_heat_off), "Turn off Theo's Heater #7", GR_THEO_HEAT},

  //make better use of unused groups
  {COMMAND(pull_cmb_pin), "????", 0x00000002},
  {COMMAND(global_thermonuclear_war), "????", 0x00000100},
  {COMMAND(get_some), "????", 0x00020000},
  {COMMAND(stab), "????", 0x00100000},
  {COMMAND(lock_and_load), "????", 0x00800000},


  {COMMAND(xyzzy), "nothing happens here", GR_MISC}
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * l :  parameter is 30 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * d :  parameter is 30 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
struct mcom mcommands[N_MCOMMANDS] = {
  {COMMAND(dac2_level), "*UNUSED* DAC2 output level. Does nothing", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "dac2_ampl"}
    }
  },

  {COMMAND(slot_sched), "set uplinked slot to use for schedule file",
    GR_TELEM, 1,
    {
      {"Slot #", 0, 250, 'i', "SLOT_SCHED"}
    }
  },

  /* pointing modes */
  {COMMAND(az_el_goto), "goto point in azimuth and elevation", GR_POINT, 2,
    {
      {"Azimuth (deg)",     -360, 360, 'f', "AZ"},
      {"Elevation (deg)", 10.0,  60.0, 'f', "EL"}
    }
  },
  {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },
  {COMMAND(az_gain), "az reaction wheel gains", GR_GAIN, 3,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_az"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_az"},
      {"Pointing Gain", 0, USHRT_MAX, 'i', "g_pt_az"}
    }
  },
  {COMMAND(az_scan), "scan in azimuth", GR_POINT, 4,
    {
      {"Az centre (deg)",       -180, 360, 'f', "AZ"},
      {"El centre (deg)",         15,  65, 'f', "EL"},
      {"Width (deg on sky)",       0, 360, 'f', "W_P"},
      {"Az Scan Speed (deg az/s)", 0,  10, 'f', "VEL_AZ_P"}
    }
  },
  {COMMAND(drift), "move at constant speed in az and el", GR_POINT, 2,
    {
      {"Az Speed (deg/s on sky)", -10.0, 10.0, 'f', "0.0"},
      {"El Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"}
    }
  },
  {COMMAND(ra_dec_goto), "track a location RA/Dec", GR_POINT, 2,
    {
      {"RA of Centre (h)",      0, 24, 'f', "RA"},
      {"Dec of Centre (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(set_scan_params), "set common scan parameters for flight", 
   GR_POINT, 5,
    {
      {"Az Scan Accel (deg/s^2)",   0,  2, 'f', "ACCEL_AZ"},
      {"Az MAX Acceleration (deg/s^2)",  0.0, 100.0, 'f', "ACCEL_MAX_AZ"},
      {"Number of half-scans per el microstep", 1, 100, 'i', "NONE"},
      {"El microstep size (deg)", 0.0, 1.0, 'f', "NONE"},
      {"Total number of el microsteps", 1, 100, 'i', "NONE"}
    }
  },
  {COMMAND(spider_scan), "scan in azimuth within a quad region in RA/Dec", 
   GR_POINT, 10,
    {
      {"RA of Corner 1 (h)",        0, 24, 'f', "RA_1_P"},
      {"Dec of Corner 1 (deg)",   -90, 90, 'f', "DEC_1_P"},
      {"RA of Corner 2 (h)",        0, 24, 'f', "RA_2_P"},
      {"Dec of Corner 2 (deg)",   -90, 90, 'f', "DEC_2_P"},
      {"RA of Corner 3 (h)",        0, 24, 'f', "RA_3_P"},
      {"Dec of Corner 3 (deg)",   -90, 90, 'f', "DEC_3_P"},
      {"RA of Corner 4 (h)",        0, 24, 'f', "RA_4_P"},
      {"Dec of Corner 4 (deg)",   -90, 90, 'f', "DEC_4_P"},
      {"Scan Starting RA (h)",      0, 24, 'f', "RA"},
      {"Scan Starting Dec (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(sine_scan), "scan sinusoidally in azimuth with a specific amplitude",
   GR_POINT, 3,
    {
      {"scan amplitude (deg)",        0, 90, 'f', "AMPL_P"},
      {"scan az centre (deg)",        0, 360,'f', "AZ"},
      {"scan elevation (deg)",        20, 50, 'f', "EL"},
    }
  },   
  {COMMAND(ra_dec_set), "define RA/Dec of current position", GR_TRIM, 2,
    {
      {"Current RA (h)",      0, 24, 'f', "RA"},
      {"Current Dec (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(pivot_gain), "pivot gains", GR_GAIN, 4,
    {
      {"Set Point (dps)",   -500, 500, 'f', "SET_RW"},
      {"V_err Gain (prop)", 0, USHRT_MAX, 'i', "G_PE_PIVOT"},
      {"V_RW Gain (prop)", 0, USHRT_MAX, 'i', "G_PV_PIVOT"},
      {"Static Friction offset",   0, 2, 'f', "FRICT_OFF_PIV"},
    }
  },
/*  {COMMAND(el_gain), "elevation motor gains", GR_GAIN, 3,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_el"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_el"},
      {"Pointing Gain",     0, USHRT_MAX, 'i', "g_pt_el"}
    }
  },*/
  {COMMAND(el_gain), "elevation motor gains", GR_GAIN, 3,
    {
      {"Common-Mode Gain (sqrt(accel))", 0.0,  3.0, 'f', "G_COM_EL"},
      {"Differential Gain (Hz)",         0.0, 1.0, 'f', "G_DIFF_EL"},
      {"Elevation Encoder Twist (deg)",     -0.5, 0.5, 'f', "NONE"}
    }
  },
  {COMMAND(el_pulse), "manually set el motor pulse rates", GR_GAIN, 2,
    {
      {"port motor pulse rate (Hz)", -10000.0, 10000.0, 'f', "STEP_1_EL"},
      {"starboard motor pulse rate (Hz)", -10000.0, 10000.0, 'f', "STEP_2_EL"}
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
  {COMMAND(cov_gps), "set the threshhold for allowable DGPS covariance", GR_TRIM,
    1,
    {
      {"Covariance (deg^2)", 0, 5.0, 'f', "COV_LIM_DGPS"},
    }
  },
  {COMMAND(ants_gps), "set the threshhold for allowable DGPS antenna separation error", GR_TRIM,
    1,
    {
      {"Antenna Separation Error (m)", 0, 10.0, 'f', "ANT_E_DGPS"},
    }
  },

  /* actuator bus commands */
  {COMMAND(general), "send a general command string to the lock or actuators",
    GR_STAGE | GR_ACT, 2,
    {
      {"Address (1-3,5,33)", 1, 0x2F, 'i', "1.0"},
      {"Command", 0, 32, 's', ""},
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
  {COMMAND(hwp_general), "send a general phytron command string", GR_HWPR, 2,
    {
      {"Stepper (1-6)", 1, 6, 'i', ""},
      {"Command", 0, 32, 's', ""},
    }
  },
  {COMMAND(hwp_vel), "set the HWP rotator velocity", GR_HWPR, 1,
    {
      {"Velocity (dps)", 0, 10, 'f', "VEL_HWP"},
    }
  },
  {COMMAND(hwp_i), "set the HWP rotator current", GR_HWPR, 1,
    {
      {"Move current (A)", 0, 10, 'f', "I_MOVE_HWP"},
    }
  },
  {COMMAND(hwp_halt), "halt HWP rotator motion", GR_HWPR, 1,
    {
      {"Stepper (1-6,0=all)", 0, 6, 'i', "0"},
    }
  },
  {COMMAND(hwp_move), "move the HWP rotator to relative position", GR_HWPR, 2,
    {
      {"Stepper (1-6,0=all)", 0, 6, 'i', "0"},
      {"delta (degrees)", -360, 360, 'f', "0"}
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
  {COMMAND(reset_adc), "Reset an ADC motherboard", GR_POWER, 1,
    {
      {"Node number",  0, 64, 'i', ""}
    }
  },
  {COMMAND(t_gyro_gain), "gyro box heater gains", GR_ELECT, 3,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_heat_gy"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_heat_gy"},
      {"Derrivative Gain",  0, USHRT_MAX, 'i', "g_d_heat_gy"}
    }
  },
  {COMMAND(t_gyro_set), "gyro box temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_GY"}
    }
  },
  {COMMAND(t_rsc_set), "RSC temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_RSC"}
    }
  },
  {COMMAND(t_bsc_set), "BSC temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_BSC"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {COMMAND(hk_ampl_cernox), "Set cernox bias amplitude", GR_BIAS, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Amplitude (V)", 0.0, 5.0, 'f', ""}
    }
  },
  {COMMAND(hk_ampl_ntd), "Set NTD bias amplitude", GR_BIAS, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Amplitude (V)", 0.0, 5.0, 'f', ""}
    }
  },
  {COMMAND(hk_phase_cernox), "Set cernox bias phase", GR_BIAS, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Phase (degrees)", 0.0, 360.0, 'f', ""}
    }
  },
  {COMMAND(hk_phase_ntd), "Set NTD bias phase", GR_BIAS, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Phase (degrees)", 0.0, 360.0, 'f', ""}
    }
  },
  {COMMAND(hk_bias_freq), "Set NTD & cernox bias frequency", GR_BIAS, 1,
    {
      {"Frequency (Hz)", 10, 400, 'i', "F_BIAS_CMD_HK"}
    }
  },

  /***************************************/
  /*************** Heat  *****************/
  {COMMAND(hk_pump_heat_on), "Turn on the pump (charcoal) heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
    }
  },
  {COMMAND(hk_pump_heat_off), "Turn off the pump (charcoal) heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
    }
  },
  {COMMAND(hk_heat_switch_on), "Turn on the heat switch heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
    }
  },
  {COMMAND(hk_heat_switch_off), "Turn off the heat switch heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
    }
  },
  {COMMAND(hk_fphi_heat_on), "Turn on the high-current focal plane heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
    }
  },
  {COMMAND(hk_fphi_heat_off), "Turn off the high-current focal plane heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
    }
  },
  {COMMAND(hk_tile_heat_on), "Turn on a detector tile heater",
      GR_CRYO_HEAT, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Tile (1-4,0=all)", 0, 4, 'i', "TILE_LAST_HK"},
    }
  },
  {COMMAND(hk_tile_heat_off), "Turn off a detector tile heater",
      GR_CRYO_HEAT, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Tile (1-4,0=all)", 0, 4, 'i', "TILE_LAST_HK"},
    }
  },
  {COMMAND(hk_tile_heat_pulse), "Pulse on a detector tile heater",
      GR_CRYO_HEAT, 3,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Tile (1-4,0=all)", 0, 4, 'i', "TILE_LAST_HK"},
      {"On Time (# of 0.2s frames)", 0, USHRT_MAX, 'i', "PULSE_LAST_HK"},
    }
  },
  {COMMAND(hk_ssa_heat_set), "Set SSA heater voltage", GR_CRYO_HEAT, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Level (V)", -5.0, 5.0, 'f', "V_HEAT_LAST_HK"},
    }
  },
  {COMMAND(hk_fplo_heat_set), "Set low-current focal plane heater voltage",
      GR_CRYO_HEAT, 2,
    {
      {"Insert (1-6,0=all)", 0, HK_MAX, 'i', "INSERT_LAST_HK"},
      {"Level (V)", -5.0, 5.0, 'f', "V_HEAT_LAST_HK"},
    }
  },

  /***************************************/
  /*************** The Good  *****************/
  {COMMAND(thegood_any), "Execute arbitrary The Good command", GR_SCGOOD, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(thegood_settrig_timed), "Use timed exposure mode on The Good", GR_SCGOOD, 1,
    {
      {"Exposure Interval (ms)", 0, USHRT_MAX, 'i', "exp_int_thegood"}
    }
  },
  {COMMAND(thegood_exp_params), "set The Good exposure commands", GR_SCGOOD, 1,
    {
      {"Exposure duration (ms)", 40, USHRT_MAX, 'i', "exp_time_thegood"}
    }
  },
  {COMMAND(thegood_focus_params), "set The Good autofocus params", GR_SCGOOD, 2,
    {
      {"Resolution (number total positions)", 0, USHRT_MAX, 'i', "foc_res_thegood"},
      {"Range (inverse fraction of total range)", 0, USHRT_MAX, 'i', "NONE"} 
    }
  },
  {COMMAND(thegood_bad_pix), "Indicate pixel to ignore on The Good", GR_SCGOOD, 3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      //1530 = CAM_WIDTH, 1020 = CAM_HEIGHT (camstruct.h)
      {"x (0=left)", 0, 1530, 'i', ""},
      {"y (0=top)", 0, 1020, 'i', ""}
    }
  },
  {COMMAND(thegood_blob_params), "set blob finder params on The Good", GR_SCGOOD, 4,
    {
      {"Max number of blobs", 1, USHRT_MAX, 'i', "maxblob_thegood"},
      {"Search grid size (pix)", 1, 1530 , 'i', "grid_thegood"},
      {"Threshold (# sigma)", 0, 100, 'f', "thresh_thegood"},
      {"Min blob separation ^2 (pix^2)", 1, 1530 , 'i', "mdist_thegood"}
    }
  },
  {COMMAND(thegood_lens_any), "execute The Good lens command directly", GR_SCGOOD, 1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(thegood_lens_move), "move The Good lens", GR_SCGOOD, 1,
    {
      //total range on Sigma EX 120-300mm is about 3270
      {"New position (ticks)", -10000, 10000, 'i', ""}
    }
  },
  {COMMAND(thegood_lens_params), "set The Good lens params", GR_SCGOOD, 1,
    {
      {"Allowed move error (ticks)", 0, USHRT_MAX, 'i', "move_tol_thegood"}
    }
  },
  /***************************************/
  /*************** The Bad  *****************/
  {COMMAND(thebad_any), "Execute arbitrary The Bad command", GR_SCBAD, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(thebad_settrig_timed), "Use timed exposure mode on The Bad", GR_SCBAD, 1,
    {
      {"Exposure Interval (ms)", 0, USHRT_MAX, 'i', "exp_int_thebad"}
    }
  },
  {COMMAND(thebad_exp_params), "set The Bad exposure commands", GR_SCBAD, 1,
    {
      {"Exposure duration (ms)", 40, USHRT_MAX, 'i', "exp_time_thebad"}
    }
  },
  {COMMAND(thebad_focus_params), "set The Bad autofocus params", GR_SCBAD, 2,
    {
      {"Resolution (number total positions)", 0, USHRT_MAX, 'i', "foc_res_thebad"},
      {"Range (inverse fraction of total range)", 0, USHRT_MAX, 'i', "NONE"} 
    }
  },
  {COMMAND(thebad_bad_pix), "Indicate pixel to ignore on The Bad", GR_SCBAD, 3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      //1530 = CAM_WIDTH, 1020 = CAM_HEIGHT (camstruct.h)
      {"x (0=left)", 0, 1530, 'i', ""},
      {"y (0=top)", 0, 1020, 'i', ""}
    }
  },
  {COMMAND(thebad_blob_params), "set blob finder params on The Bad", GR_SCBAD, 4,
    {
      {"Max number of blobs", 1, USHRT_MAX, 'i', "maxblob_thebad"},
      {"Search grid size (pix)", 1, 1530 , 'i', "grid_thebad"},
      {"Threshold (# sigma)", 0, 100, 'f', "thresh_thebad"},
      {"Min blob separation ^2 (pix^2)", 1, 1530 , 'i', "mdist_thebad"}
    }
  },
  {COMMAND(thebad_lens_any), "execute The Bad lens command directly", GR_SCBAD, 1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(thebad_lens_move), "move The Bad lens", GR_SCBAD, 1,
    {
      //total range on Sigma EX 120-300mm is about 3270
      {"New position (ticks)", -10000, 10000, 'i', ""}
    }
  },
  {COMMAND(thebad_lens_params), "set The Bad lens params", GR_SCBAD, 1,
    {
      {"Allowed move error (ticks)", 0, USHRT_MAX, 'i', "move_tol_thebad"}
    }
  },
  /***************************************/
  /*************** The Ugly  *****************/
  {COMMAND(theugly_any), "Execute arbitrary The Ugly command", GR_SCUGLY, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(theugly_settrig_timed), "Use timed exposure mode on The Ugly", GR_SCUGLY, 1,
    {
      {"Exposure Interval (ms)", 0, USHRT_MAX, 'i', "exp_int_theugly"}
    }
  },
  {COMMAND(theugly_exp_params), "set The Ugly exposure commands", GR_SCUGLY, 1,
    {
      {"Exposure duration (ms)", 40, USHRT_MAX, 'i', "exp_time_theugly"}
    }
  },
  {COMMAND(theugly_focus_params), "set The Ugly autofocus params", GR_SCUGLY, 2,
    {
      {"Resolution (number total positions)", 0, USHRT_MAX, 'i', "foc_res_theugly"},
      {"Range (inverse fraction of total range)", 0, USHRT_MAX, 'i', "NONE"} 
    }
  },
  {COMMAND(theugly_bad_pix), "Indicate pixel to ignore on The Ugly", GR_SCUGLY, 3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      //1530 = CAM_WIDTH, 1020 = CAM_HEIGHT (camstruct.h)
      {"x (0=left)", 0, 1530, 'i', ""},
      {"y (0=top)", 0, 1020, 'i', ""}
    }
  },
  {COMMAND(theugly_blob_params), "set blob finder params on The Ugly", GR_SCUGLY, 4,
    {
      {"Max number of blobs", 1, USHRT_MAX, 'i', "maxblob_theugly"},
      {"Search grid size (pix)", 1, 1530 , 'i', "grid_theugly"},
      {"Threshold (# sigma)", 0, 100, 'f', "thresh_theugly"},
      {"Min blob separation ^2 (pix^2)", 1, 1530 , 'i', "mdist_theugly"}
    }
  },
  {COMMAND(theugly_lens_any), "execute The Ugly lens command directly", GR_SCUGLY, 1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(theugly_lens_move), "move The Ugly lens", GR_SCUGLY, 1,
    {
      //total range on Sigma EX 120-300mm is about 3270
      {"New position (ticks)", -10000, 10000, 'i', ""}
    }
  },
  {COMMAND(theugly_lens_params), "set The Ugly lens params", GR_SCUGLY, 1,
    {
      {"Allowed move error (ticks)", 0, USHRT_MAX, 'i', "move_tol_theugly"}
    }
  },
  //STAR CAMERA TABLE
  {COMMAND(table_gain), "RSC rotary table gains", GR_SCTAB, 2,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_table"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_table"},
      {"Derivative Gain",   0, USHRT_MAX, 'i', "g_d_table"}
    }
  },
  {COMMAND(table_goto), "move RSC to specific encoder position", GR_SCTAB, 1,
    {
      {"Goto position (deg)", 0, 360, 'd', "table_goto"}
    }
  },
  {COMMAND(table_relmove), "move RSC by relative angle", GR_SCTAB, 1,
    {
      {"Relative angle (deg)", -360, 360, 'd', "table_move_"}
    }
  },
  {COMMAND(table_speed), "change RSC table speed", GR_SCTAB, 1,
    {
      {"Speed (deg/s)", 0, 45, 'd', "table_speed"}
    }
  },

  {COMMAND(motors_verbose), "Set verbosity of motor serial threads (0=norm, 1=verbose, 2= superverbose )", GR_MISC, 3,
   {
     {"Reaction Wheel", 0, 5, 'i', "VERBOSE_RW"},
     {"Elevation", 0, 5, 'i', "VERBOSE_EL"},
     {"Pivot", 0, 5, 'i', "VERBOSE_PIV"}
   }
  },

  {COMMAND(bbc_rate_ext), "Set BBC external (sync box) sample rate",
    GR_MISC | CONFIRM, 1,
    {
      {"Rate (Sync frames per BBC)", 1, USHRT_MAX, 'i', "FRAME_EXT_BBC"}
    }
  },
  {COMMAND(bbc_rate_int), "Set BBC internal sample rate",
    GR_MISC | CONFIRM, 1,
    {
      {"Rate (# ADC samples (~10kHz))", 1, USHRT_MAX, 'i', "FRAME_INT_BBC"}
    }
  },


  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, USHRT_MAX, 'i', "PLOVER"}
    }
  }
};
