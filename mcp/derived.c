/* derived.c: a list of derived channels
 *
 * This software is copyright (C) 2002-2005 University of Toronto
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
 * IF YOU ADD, MODIFY, OR DELETE *ANY* CHANNELS IN THIS FILE YOU *MUST*
 * RECOMPILE AND RESTART THE DECOM DAEMON (DECOMD) ON ARWEN!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "derived.h"

/* Don's Handy Guide to adding derived channels:
 *
 * There are seven types of derived channels which can be added to the format
 * file.  Channel names are, of course, case sensitive.  On start-up, both
 * mcp and decomd run sanity checks on the channel lists, including the derived
 * channel list.  The checks will fail for the derived channel list if either
 * a source channel doesn't exist or a derived channel has a name already in
 * use.  Listed below are the derived channel types and their arguments.  If
 * more derived channel types are desired, please ask me to add them.  --dvw
 *
 * o LINCOM: Single field calibration.  Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Calibration Slope (double)
 *   4.  Calibration Intercept (double)
 * o LINCOM2: Two field linear combination
 *   1.  Derived Channel Name (string)
 *   2.  Source 1 Channel Name (string)
 *   3.  Source 1 Calibration Slope (double)
 *   4.  Source 1 Calibration Intercept (double)
 *   5.  Source 2 Channel Name (string)
 *   6.  Source 2 Calibration Slope (double)
 *   7.  Source 2 Calibration Intercept (double)
 * o LINTERP: Linearly interpolated look up table
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Full Path to Look Up Table File (string)
 * o BITFIELD: Single bit channels derived from a common source channel
 *   1.  Source Channel Name (string)
 *   2.  Bit 0 Derived Channel Name (string)
 *   3.  Bit 1 Derived Channel Name (string)
 *   ...
 *   17. Bit 15 Derived Channel Name (string)
 *
 *      NB: Bits for which no channel is defined should be given an empty string
 *      "" as a channel name.  Additionally, unused trailing high-bit channels
 *      can be omitted.
 * o BITWORD: A multi-bit channel extracted from a larger source channel
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  First bit of the bitword, numbered from zero (integer)
 *   4.  Length of the bitword (integer)
 * o COMMENT: A litteral comment to be inserted into the format file
 *   1.  Comment Text (string) -- there is no need to include the comment
 *       delimiter (#).
 * o UNITS: meta data defining units and quantity for existing fields
 *   1.  Source Channel Name (string)
 *   2.  String for the quantity (eg "Temperature")
 *   3.  String for the Units (eg, "^oC")
 *
 *
 * In addition to the derived channels derived below, defile will add the
 * "Nice CPU Values" (to wit: CPU_SEC, CPU_HOUR, etc.), properly offset to the
 * start of the file, to the end of the format file.
 */

union DerivedUnion DerivedChannels[] = {
  /* Pointing */
  COMMENT("Microsecond Resolution Time"),
  LINCOM2("Time", "cpu_time", 1, 0, "cpu_usec",  1.0E-6, 0),
  UNITS("Time", "Time", "s"),
#ifndef BOLOTEST
  COMMENT("Pointing Stuff"),
  LINCOM("P_X_H", "p_x_deg", 0.0003662109375, 0),
  LINTERP("Clin_Elev", "clin_elev", "/data/etc/clin_elev.lut"),
  UNITS("Clin_Elev", "Elevation", "^o"),
  LINTERP("SS_AzRelSun", "SS_AZ", "/data/etc/ss.lut"),
  BITFIELD("sensor_veto",
      "SUN_VETO",
      "ISC_VETO",
      "ELENC_VETO",
      "MAG_VETO",
      "GPS_VETO",
      "ELCLIN_VETO",
      "OSC_VETO",
      "IS_SCHED",
      "AZ_AUTO_GYRO",
      "EL_AUTO_GYRO",
      "DISABLE_EL",
      "DISABLE_AZ",
      "FORCE_EL"
      ),

#if 0
  BITFIELD("sensor_reset",
      "",
      "",
      "",
      "DGPS_RESET",
      "ISC_RESET",
      "GYRO_RESET",
      "SS_RESET",
      "OSC_RESET"
      ),
#endif

  /* ISC and OSC */
  COMMENT("Star Camera State"),
  BITFIELD("isc_state",
      "ISC_SAVE_IMAGES",
      "ISC_PAUSE",
      "ISC_ABORT",
      "ISC_AUTOFOCUS",
      "ISC_SHUTDOWN",
      "ISC_REBOOT",
      "ISC_EYE_ON",
      "ISC_HEATER_ON",
      "ISC_USE_LOST"
      ),
  BITWORD("ISC_SENT_TRIG", "isc_trigger", 0, 14),

  BITFIELD("osc_state",
      "OSC_SAVE_IMAGES",
      "OSC_PAUSE",
      "OSC_ABORT",
      "OSC_AUTOFOCUS",
      "OSC_SHUTDOWN",
      "OSC_REBOOT",
      "OSC_EYE_ON",
      "OSC_HEATER_ON",
      "OSC_USE_LOST"
      ),
  BITWORD("OSC_SENT_TRIG", "osc_trigger", 0, 14),

  COMMENT("ACS Digital Signals"),

#if 0 
  BITFIELD("ifpm_bits",
      "",
      "",
      "BAL_ON_BIT",
      "BAL_REV",
      "BAL2_ON",
      "BAL2_REV"
      ),
#endif
#endif

  COMMENT("Lock Motor/Actuators"),
  BITFIELD("lock_state",
      "LS_OPEN",
      "LS_CLOSED",
      "LS_DRIVE_OFF",
      "LS_POT_RAIL",
      "LS_DRIVE_EXT",
      "LS_DRIVE_RET",
      "LS_DRIVE_STP",
      "LS_DRIVE_JIG",
      "LS_DRIVE_UNK",
      "LS_EL_OK",
      "LS_IGNORE_EL",
      "LS_DRIVE_FORCE"
      ),

  /* Secondary Focus */
  COMMENT("Secondary Focus"),
  LINCOM2("REL_FOCUS", "SF_CORRECTION", 1, 0, "SF_OFFSET", 1, 0),
  LINCOM2("TC_VETO", "TC_WAIT", 1, 0, "SF_AGE", -1, 0),

  BITFIELD("act_flags",
      "ACT_LOST",
      "ACT_DR_POS_BAD",
      "ACT_DR_ENC_BAD",
      "ACT_ENC_POS_BAD",
      "ACT_DR_LVDT_BAD",
      "ACT_ENC_LVDT_BAD",
      "ACT_POS_LVDT_BAD",
      "ACT_BAD_MOVE",
      "ACT0_FAULT",
      "ACT1_FAULT",
      "ACT2_FAULT"
      ),

  BITFIELD("el_stat_1",
	   "EL_SHORT_CIRCUIT",
	   "EL_AMP_OVER_TEMP",
	   "EL_OVER_VOLT",
	   "EL_UNDER_VOLT",
	   "",
	   "EL_FEEDBACK_ERR",
	   "EL_MOT_PHASE_ERR",
	   "EL_I_LIMITED",
	   "EL_VOLT_LIM",
	   "",
           "",
	   "EL_AMP_DISAB_HWARE",
	   "EL_AMP_DISAB_SWARE",	
	   "EL_ATTEMPT_STOP",
	   "EL_MOT_BREAK_ACTIVE",
	   "EL_PWM_OUT_DISABLED"
	   ),
  BITFIELD("el_stat_2",
	   "EL_POS_SOFT_LIM",
	   "EL_NEG_SOFT_LIM",
	   "EL_FOLLOW_ERR",
	   "EL_FOLLOW_WARN",
	   "EL_AMP_HAS_RESET",
	   "EL_ENCODER_WRAP",
	   "EL_AMP_FAULT",
	   "EL_VEL_LIMITED",
	   "EL_ACCEL_LIMITED",
	   "",
	   "",
	   "EL_IN_MOTION",
	   "EL_VEL_OUT_TRACK_W",
	   "EL_PHASE_NOT_INIT",
	   "",
	   ""
   ),
  BITFIELD("rw_stat_1",
	   "RW_SHORT_CIRCUIT",
	   "RW_AMP_OVER_TEMP",
	   "RW_OVER_VOLT",
	   "RW_UNDER_VOLT",
	   "",
	   "RW_FEEDBACK_ERR",
	   "RW_MOT_PHASE_ERR",
	   "RW_I_LIMITED",
	   "RW_VOLT_LIM",
	   "",
           "",
	   "RW_AMP_DISAB_HWARE",
	   "RW_AMP_DISAB_SWARE",	
	   "RW_ATTEMPT_STOP",
	   "RW_MOT_BREAK_ACTIVE",
	   "RW_PWM_OUT_DISABLED"
   ),
  BITFIELD("rw_stat_2",
	   "RW_POS_SOFT_LIM",
	   "RW_NEG_SOFT_LIM",
	   "RW_FOLLOW_ERR",
	   "RW_FOLLOW_WARN",
	   "RW_AMP_HAS_RESET",
	   "RW_ENCODER_WRAP",
	   "RW_AMP_FAULT",
	   "RW_VEL_LIMITED",
	   "RW_ACCEL_LIMITED",
	   "",
	   "",
	   "RW_IN_MOTION",
	   "RW_VEL_OUT_TRACK_W",
	   "RW_PHASE_NOT_INIT",
	   "",
	   ""
   ),
  BITFIELD("el_fault",
	   "EL_F_CRC",
	   "EL_F_AD_OFF_RANGE",
	   "EL_F_SHORT_CIRC",
	   "EL_F_AMP_OVER_TEMP",
	   "",
	   "EL_F_OVER_VOLT",
	   "EL_F_UNDER_VOLT",
	   "EL_F_FEEDBACK_ERR",
	   "EL_F_MOT_PHASE_ERR",
	   "EL_F_FOLL_ERR",
	   "EL_F_OVER_CURR"
	   ),
  BITFIELD("rw_fault",
	   "RW_F_CRC",
	   "RW_F_AD_OFF_RANGE",
	   "RW_F_SHORT_CIRC",
	   "RW_F_AMP_OVER_TEMP",
	   "",
	   "RW_F_OVER_VOLT",
	   "RW_F_UNDER_VOLT",
	   "RW_F_FEEDBACK_ERR",
	   "RW_F_MOT_PHASE_ERR",
	   "RW_F_FOLL_ERR",
	   "RW_F_OVER_CURR"
	   ),
  BITFIELD("piv_d_stat",
	   "PIV_BRIDGE_ENAB",
	   "PIV_DYN_BRAKE_ENAB",
	   "PIV_SHUNT_EN",
	   "PIV_POS_STOP_ENAB",
	   "PIV_NEG_STOP_ENAB",
	   "PIV_POS_TORQ_INHIB",
	   "PIV_NEG_TORQ_INHIB",
	   "PIV_EXT_BRAKE",
	   "PIV_DR_RESET",
	   "PIV_DR_INTER_ERR",
	   "PIV_DR_SHORT_CIRC",
	   "PIV_DR_I_OVERSHOOT",
	   "PIV_DR_UNDER_V",
	   "PIV_DR_OVER_V",
	   "PIV_DR_OVER_TEMP",
	   ""
	   ),
  /* CRYO */

#if 0
  COMMENT("Internal (to mcp) reference of the cryo stat"),
  BITFIELD("cryostate",
      "HE_LEV_SENS",
      "CHARC_HEATER",
      "COLDP_HEATER",
      "FRIDGE_CYCLING",
      "POT_VALVE",
      "POT_DIREC",
      "LHE_VALVE",
      "CRYO_DIREC",
      "LN_VALVE",
      "AUTO_JFET_HEAT"
      ),

  COMMENT("Control bits to the Cryo control card"),
  BITFIELD("cryoctrl",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "CAL_LAMP_ON_CMD",
      "AUTO_BDA_HEAT_ON"
      ),

  COMMENT("Cryo Valve Limit Switches"),

  BITFIELD("cryoin",
      "POT_IS_CLOSED",
      "POT_IS_OPEN",
      "LHE_IS_CLOSED",
      "LHE_IS_OPEN",
      "LN_IS_CLOSED",
      "LN_IS_OPEN"
      ),

  LINCOM2("POT_STATE", "POT_IS_CLOSED", 2, 0, "POT_IS_OPEN",  1, 0),
  LINCOM2("LHE_STATE", "LHE_IS_CLOSED", 2, 0, "LHE_IS_OPEN",  1, 0),
  LINCOM2("LN_STATE",  "LN_IS_CLOSED", 2, 0, "LN_IS_OPEN", 1, 0),

  COMMENT("Limit Switch Niceties"),

  LINCOM2("LS_EL_STATUS", "LS_EL_OK", 2, 0, "LS_IGNORE_EL", 1, 0),
  LINCOM2("LS_OPENCLOSE", "LS_OPEN", 2, 0, "LS_CLOSED", 1, 0),
  LINCOM2("LS_MOTION", "LS_DRIVE_RET", 2, 0, "LS_DRIVE_EXT", 1, 0),
  LINCOM2("LS_NICE_STAT", "LS_MOTION", 3, 0, "LS_OPENCLOSE", 1, 0),
#endif

  COMMENT("Cryo Table Lookups"),
  COMMENT("Diodes"),

  LINTERP("Td_charcoal",   "TD_CHARCOAL"   , "/data/etc/dt600.txt"),
  LINTERP("Td_lhe",        "TD_LHE"        , "/data/etc/dt600.txt"),
  LINTERP("Td_ln",         "TD_LN"         , "/data/etc/dt600.txt"),
  LINTERP("Td_hs_pot",     "TD_HS_POT"     , "/data/etc/dt600.txt"),
  LINTERP("Td_jfet",       "TD_JFET"       , "/data/etc/dt600.txt"),
  LINTERP("Td_vcs_filt",   "TD_VCS_FILT"   , "/data/etc/dt600.txt"),
  LINTERP("Td_ln_filt",    "TD_LN_FILT"    , "/data/etc/dt600.txt"),
  LINTERP("Td_lhe_filt",   "TD_LHE_FILT"   , "/data/etc/dt600.txt"),
  LINTERP("Td_vcs_jfet",   "TD_VCS_JFET"   , "/data/etc/dt600.txt"),
  LINTERP("Td_hs_charcoal","TD_HS_CHARCOAL", "/data/etc/dt600.txt"),

//  COMMENT("Level Sensor"),
//  LINTERP("HE4_LITRE", "HE4_LEV", "/data/etc/he4_litre.txt"),
//  LINTERP("HE4_PERCENT", "HE4_LEV", "/data/etc/he4_percent.txt"),

  END_OF_DERIVED_CHANNELS
};
