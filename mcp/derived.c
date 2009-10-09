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
#ifndef BOLOTEST
  COMMENT("Pointing Stuff"),
  LINCOM("P_X_H", "p_x_deg", 0.0003662109375, 0),
  LINTERP("Clin_Elev", "clin_elev", "/data/etc/clin_elev.lut"),
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

  /* bits to acs 0 */
  COMMENT("Auxiliary heaters"),
  BITFIELD("gy2_heat",
      "ISC_HEAT_ENABLE",
      "",
      "OSC_HEAT_ENABLE",
      "",
      "GYBOX2_HEAT"
      ),

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

  BITFIELD("ifpm_bits",
      "",
      "",
      "BAL_ON_BIT",
      "BAL_REV",
      "BAL2_ON",
      "BAL2_REV"
      ),
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


  COMMENT("ROX TEMPORARY - Via DAS NODE 17"),

  COMMENT("Z_ are in 'signed' counts where zero is zero"),
  LINCOM("Z_N17C00", "n17c00",  1.0, -8388608),
  LINCOM("Z_N17C01", "n17c01",  1.0, -8388573),
  LINCOM("Z_N17C02", "n17c02",  1.0, -8388625),
  LINCOM("Z_N17C03", "n17c03",  1.0, -8388586),
  LINCOM("Z_N17C04", "n17c04",  1.0, -8388604),
  LINCOM("Z_N17C05", "n17c05",  1.0, -8388690),
  LINCOM("Z_N17C06", "n17c06",  1.0, -8388617),
  LINCOM("Z_N17C07", "n17c07",  1.0, -8374070),
  LINCOM("Z_N17C08", "n17c08",  1.0, -8388591),
  LINCOM("Z_N17C09", "n17c09", -1.0,  8388621),
  LINCOM("Z_N17C10", "n17c10", -1.0,  8388602),
  LINCOM("Z_N17C11", "n17c11",  1.0, -8388628),
  LINCOM("Z_N17C12", "n17c12",  1.0, -8388599),
  LINCOM("Z_N17C13", "n17c13",  1.0, -8388585),
  LINCOM("Z_N17C14", "n17c14",  1.0, -8388618),
  LINCOM("Z_N17C15", "n17c15", -1.0,  8388625),
  LINCOM("Z_N17C16", "n17c16",  1.0, -8388628),
  LINCOM("Z_N17C17", "n17c17",  1.0, -8388606),
  LINCOM("Z_N17C18", "n17c18",  1.0, -8388644),
  LINCOM("Z_N17C19", "n17c19", -1.0,  8388606),
  LINCOM("Z_N17C20", "n17c20",  1.0, -8388548),
  LINCOM("Z_N17C21", "n17c21",  1.0, -8388550),
  LINCOM("Z_N17C22", "n17c22",  1.0, -8358435),
  LINCOM("Z_N17C23", "n17c23",  1.0, -8388579),

  COMMENT("R_ are in ohms"),
  LINTERP("RN17C00", "Z_N17C00", "/data/etc/R_N17.txt"),
  LINTERP("RN17C01", "Z_N17C01", "/data/etc/R_N17.txt"),
  LINTERP("RN17C02", "Z_N17C02", "/data/etc/R_N17.txt"),
  LINTERP("RN17C03", "Z_N17C03", "/data/etc/R_N17.txt"),
  LINTERP("RN17C04", "Z_N17C04", "/data/etc/R_N17.txt"),
  LINTERP("RN17C05", "Z_N17C05", "/data/etc/R_N17.txt"),
  LINTERP("RN17C06", "Z_N17C06", "/data/etc/R_N17.txt"),
  LINTERP("RN17C07", "Z_N17C07", "/data/etc/R_N17.txt"),
  LINTERP("RN17C08", "Z_N17C08", "/data/etc/R_N17.txt"),
  LINTERP("RN17C09", "Z_N17C09", "/data/etc/R_N17.txt"),
  LINTERP("RN17C10", "Z_N17C10", "/data/etc/R_N17.txt"),
  LINTERP("RN17C11", "Z_N17C11", "/data/etc/R_N17.txt"),
  LINTERP("RN17C12", "Z_N17C12", "/data/etc/R_N17.txt"),
  LINTERP("RN17C13", "Z_N17C13", "/data/etc/R_N17.txt"),
  LINTERP("RN17C14", "Z_N17C14", "/data/etc/R_N17.txt"),
  LINTERP("RN17C15", "Z_N17C15", "/data/etc/R_N17.txt"),
  LINTERP("RN17C16", "Z_N17C16", "/data/etc/R_N17.txt"),
  LINTERP("RN17C17", "Z_N17C17", "/data/etc/R_N17.txt"),
  LINTERP("RN17C18", "Z_N17C18", "/data/etc/R_N17.txt"),
  LINTERP("RN17C19", "Z_N17C19", "/data/etc/R_N17.txt"),
  LINTERP("RN17C20", "Z_N17C20", "/data/etc/R_N17.txt"),
  LINTERP("RN17C21", "Z_N17C21", "/data/etc/R_N17.txt"),
  LINTERP("RN17C22", "Z_N17C22", "/data/etc/R_N17.txt"),
  LINTERP("RN17C23", "Z_N17C23", "/data/etc/R_N17.txt"),

  COMMENT("Tr_ are in Kelvin"),
  LINTERP("Tr_3he_fridge",  "RN17C00", "/data/etc/rox102a.txt"),
  LINTERP("Tr_m5",          "RN17C01", "/data/etc/rox102a.txt"),
  UNITS("Tr_m5", "Temperature", "K"),
  LINTERP("Tr_m4",          "RN17C04", "/data/etc/rox102a.txt"),
  LINTERP("Tr_m3_BROKEN",   "RN17C07", "/data/etc/rox102a.txt"),
  LINTERP("Tr_bda3",        "RN17C11", "/data/etc/rox102a.txt"),
  LINTERP("Tr_bda2",        "RN17C14", "/data/etc/rox102a.txt"),
  LINTERP("Tr_bda1",        "RN17C17", "/data/etc/rox102a.txt"),
  LINTERP("Tr_300mK_strap", "RN17C20", "/data/etc/rox102a.txt"),
  LINTERP("Tr_pot_diode",   "RN17C22", "/data/etc/rox102a.txt"),
  LINTERP("Tr_optics_filt", "RN17C23", "/data/etc/rox102a.txt"),

//  COMMENT("Level Sensor"),
//  LINTERP("HE4_LITRE", "HE4_LEV", "/data/etc/he4_litre.txt"),
//  LINTERP("HE4_PERCENT", "HE4_LEV", "/data/etc/he4_percent.txt"),

  END_OF_DERIVED_CHANNELS
};
