/* derived.c: a list of derived channels
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

#include "derived.h"

/* Don's Handy Guide to adding derived channels:
 *
 * There are five types of derived channels which can be added to the format
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
#ifndef BOLOTEST
  COMMENT("Pointing Stuff"), 
  LINCOM("P_X_H", "p_x_deg", 0.0003662109375, 0),
  LINTERP("Clin_Elev", "clin_elev", "/data/etc/clin_elev.lut"),
  BITFIELD("sensor_veto",
      "SUN_VETO",
      "ISC_VETO",
      "ELENC_VETO",
      "MAG_VETO",
      "GPS_VETO",
      "ELCLIN_VETO",
      "IS_SCHED"
      ),

  /* ISC and OSC */
  COMMENT("Star Camera State"), 
  BITFIELD("isc_state",
      "ISC_SAVE_IMAGES",
      "ISC_PAUSE",
      "ISC_ABORT",
      "ISC_AUTOFOCUS",
      "ISC_BRIGHT_STAR",
      "ISC_SHUTDOWN"
      ),

  BITFIELD("osc_state",
      "OSC_SAVE_IMAGES",
      "OSC_PAUSE",
      "OSC_ABORT",
      "OSC_AUTOFOCUS",
      "OSC_BRIGHT_STAR",
      "OSC_SHUTDOWN"
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
  BITFIELD("cryostate",
      "HE_LEV_SENS",
      "CHARC_HEATER",
      "COLDP_HEATER",
      "",
      "POT_VALVE",
      "POT_DIREC",
      "LHE_VALVE",
      "LHE_DIREC",
      "",
      "AUTO_JFET_HEAT"
      ),

  COMMENT("Cryo Valve Limit Switches"),

  BITFIELD("cryoin",
      "POT_IS_CLOSED",
      "POT_IS_OPEN",
      "LHE_IS_CLOSED",
      "LHE_IS_OPEN"
      ),
  
  LINCOM2("POT_STATE", "POT_IS_CLOSED", 1, 0, "POT_IS_OPEN",  1, 0),
  LINCOM2("LHE_STATE", "LHE_IS_CLOSED", 1, 0, "LHE_IS_OPEN",  1, 0),

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
