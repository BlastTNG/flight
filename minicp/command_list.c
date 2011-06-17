/* command_list.c: Spider command specification file
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

const char *command_list_serial = "$Revision: 1.5 $";

//these must correspond to #defines in header
const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Balance",          "Cooling", 
  "Pointing Sensor Trims", "Aux. Electronics", "Bias",
  "Pointing Sensor Vetos", "Actuators",        "Cal Lamp",
  "Pointing Motor Gains",  "Secondary Focus",  "Cryo Heat",
  "Subsystem Power",       "Lock Motor",       "Cryo Control",
  "Telemetry",             "SC Miscellaneous", "Free for use 1",
  "X-Y Stage",             "SC Modes",         "Free for use 2",
  "Miscellaneous",         "SC Parameters",    "Free for use 3"
};

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(az_el_disable), "Az-El mount emergency shutdown!", GR_POINT},
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
  {COMMAND(bias_ampl), "Set bias signal amplitude", GR_BIAS, 1,
    {
      {"Amplitude (full=32767)", 1, 32767, 'i', "bias_ampl"}
    }
  },
  {COMMAND(lockin_phase1), "Set lock-in phase on card 1", GR_BIAS, 1,
    {
      {"Phase (in 2000ths of a cycle)", 0, 2000, 'i', "adc1_phase"}
    }
  },
  {COMMAND(lockin_phase2), "Set lock-in phase on card 2", GR_BIAS, 1,
    {
      {"Phase (in 2000ths of a cycle)", 0, 2000, 'i', "adc2_phase"}
    }
  },
  {COMMAND(lockin_phase3), "Set lock-in phase on card 3", GR_BIAS, 1,
    {
      {"Phase (in 2000ths of a cycle)", 0, 2000, 'i', "adc3_phase"}
    }
  },
  {COMMAND(lockin_phase4), "Set lock-in phase on card 4", GR_BIAS, 1,
    {
      {"Phase (in 2000ths of a cycle)", 0, 2000, 'i', "adc4_phase"}
    }
  },
  {COMMAND(reset_adc), "Reset an ADC motherboard", GR_POWER, 1,
    {
      {"Node number",  0, 64, 'i', ""}
    }
  },
  {COMMAND(az_el_raster), "Az-El mount raster scan", GR_POINT, 9,
    { 
      {"az raster centre (deg)",  0.0, 360.0, 'f', "AZ"}, 
      {"el raster centre (deg)", -10.0, 89.0, 'f', "EL"},  
      {"az raster scan width (deg)", 0.0, 180.0, 'f', "WIDTH_AZ"}, 
      {"az raster scan speed (deg/s)", 0.0, 5.0, 'f', "V_AZ"},  
      {"el raster scan speed (deg/s)", 0.0, 5.0, 'f', "V_EL"},
      {"az raster acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_AZ"},
      {"el raster acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_EL"},
      {"el raster step size (deg)", 0.0, 5.0, 'f', "STEP_EL"},
      {"el raster height (deg)", 0.0, 99.0, 'f', "HEIGHT_EL"},

    }
  },
  {COMMAND(az_el_goto), "Az-El mount goto mode", GR_POINT, 6,
    {
      {"az goto acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_AZ"},
      {"el goto acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_EL;"},
      {"az goto speed (deg/s)", 0.0, 5.0, 'f', "V_AZ"},
      {"el goto speed (deg/s)", 0.0, 5.0, 'f', "V_EL"},
      {"az goto position (deg)",  0.0, 360.0, 'f', "AZ"}, 
      {"el goto position (deg)", -10.0, 89.0, 'f', "EL"}  
    }
  },
  {COMMAND(az_el_set), "Input Az-El mount starting angles", GR_POINT, 2,
    {
      {"starting azimuth (deg)", 0.0, 360.0, 'f', "AZ_REF"},
      {"starting elevation (deg)", -10.0, 89.0, 'f', "EL_REF"},
    }
  }, 
  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, MAX_15BIT, 'i', "PLOVER"}
    }
  }
};
