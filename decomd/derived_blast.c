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

union DerivedUnion DerivedChannels[] = {
  /* Pointing */
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

  COMMENT("Cryo Vale Limit Switches"),

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

  LINTERP("T_charcoal",   "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_lhe",        "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_ln2",        "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_heatswitch", "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_jfet",       "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_vcs_filt",   "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_ln2_filt",   "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_lhe_filt",   "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_he4pot_d",   "T_CHARCOAL", "/data/etc/dt600.txt"),
  LINTERP("T_vcs_fet",    "T_CHARCOAL", "/data/etc/dt600.txt"),

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
