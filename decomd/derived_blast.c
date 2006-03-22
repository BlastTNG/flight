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
 * There are six types of derived channels which can be added to the format
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
  LINTERP("SS_AzRelSun", "SS_AZ_CENTER", "/data/etc/ss.lut"),
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
      "EL_AUTO_GYRO"
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
      "ISC_HEATER_ON"
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
      "OSC_HEATER_ON"
      ),
  BITWORD("OSC_SENT_TRIG", "osc_trigger", 0, 14),

  COMMENT("ACS Digital Signals"),

  BITFIELD("ifpm_bits",
      "",
      "",
      "BAL_ON_BIT",
      "BAL_REV",
      "IF_COOL1_OFF",
      "IF_COOL1_ON",
      "BAL2_ON",
      "BAL2_REV"
      ),
  BITFIELD("ofpm_bits",
      "OF_COOL2_ON",
      "OF_COOL2_OFF",
      "OF_COOL1_ON",
      "OF_COOL1_OFF",
      ),
#endif

  /* BIAS */
  COMMENT("Bias Control"),
  BITFIELD("biasin",
      "",
      "BIAS_IS_DC",
      "BIAS_CLK_IS_INT",
      "BIAS_IS_INT"
      ),

  /* CRYO */
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
      "AUTO_CYCLE_ON"
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

  COMMENT("Cryo Table Lookups"),
  COMMENT("Diodes"),

  LINTERP("T_charcoal",   "T_CHARCOAL"   , "/data/etc/dt600.txt"),
  LINTERP("T_lhe",        "T_LHE"        , "/data/etc/dt600.txt"),
  LINTERP("T_ln2",        "T_LN2"        , "/data/etc/dt600.txt"),
  LINTERP("T_heatswitch", "T_HEATSWITCH" , "/data/etc/dt600.txt"),
  LINTERP("T_jfet",       "T_JFET"       , "/data/etc/dt600.txt"),
  LINTERP("T_vcs_filt",   "T_VCS_FILT"   , "/data/etc/dt600.txt"),
  LINTERP("T_ln2_filt",   "T_LN2_FILT"   , "/data/etc/dt600.txt"),
  LINTERP("T_lhe_filt",   "T_LHE_FILT"   , "/data/etc/dt600.txt"),
  LINTERP("T_he4pot_d",   "T_HE4POT_D"   , "/data/etc/dt600.txt"),
  LINTERP("T_vcs_fet",    "T_VCS_FET"    , "/data/etc/dt600.txt"),

  COMMENT("GRTs (ROX)"),
  LINTERP("T_he3fridge",   "T_HE3FRIDGE",   "/data/etc/rox102a3.txt"),
  LINTERP("T_he4pot",      "T_HE4POT",      "/data/etc/rox102a22.txt"),
  LINTERP("T_m3",          "T_M3",          "/data/etc/rox102a7.txt"),
  LINTERP("T_m4",          "T_M4",          "/data/etc/rox102a4.txt"),
  LINTERP("T_m5",          "T_M5",          "/data/etc/rox102a5.txt"),
  LINTERP("T_optbox_filt", "T_OPTBOX_FILT", "/data/etc/rox102a23.txt"),
  LINTERP("T_300mk_strap", "T_300MK_STRAP", "/data/etc/rox102a20.txt"),
  LINTERP("T_horn_250",    "T_HORN_250",    "/data/etc/rox102a6.txt"),
  LINTERP("T_horn_350",    "T_HORN_350",    "/data/etc/rox102a19.txt"),
  LINTERP("T_horn_500",    "T_HORN_500",    "/data/etc/rox102a21.txt"),

  COMMENT("Level Sensor"),
  LINTERP("HE4_LITRE", "HE4_LEV", "/data/etc/he4_litre.txt"),
  LINTERP("HE4_PERCENT", "HE4_LEV", "/data/etc/he4_percent.txt"),

  END_OF_DERIVED_CHANNELS
};
