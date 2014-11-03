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

const char *command_list_serial = "$Revision: 1.12 $";

//these must correspond to #defines in header
/*const char *GroupNames[N_GROUPS] = {
  "AzEl Gadget",           "Balance",          "Cooling", 
  "Pointing Sensor Trims", "Aux. Electronics", "Bias",
  "Pointing Sensor Vetos", "Actuators",        "Cal Lamp",
  "Pointing Motor Gains",  "Secondary Focus",  "Cryo Heat",
  "Subsystem Power",       "Lock Motor",       "Cryo Control",
  "Telemetry",             "SC Miscellaneous", "Free for use 1",
  "X-Y Stage",             "SC Modes",         "Free for use 2",
  "Miscellaneous",         "SC Parameters",    "Free for use 3"
};*/

const char *GroupNames[N_GROUPS] = {
  "Az-El Gadget",          "Bias",
  "Subsystem Power",       "Miscellaneous"         
};

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(az_el_disable), "Az-El mount emergency shutdown!", GR_AZEL},
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
  {COMMAND(dac1_ampl), "Set bias signal amplitude on MB #1", GR_BIAS, 2,
    {
      {"Which (0-31,32=all)", 0, 32, 'i', ""},
      {"Amplitude (full=32767)", 1, MAX_15BIT, 'i', ""}
    }
  },
  {COMMAND(dac1_phase), "Set lock-in phase  on MB #1", GR_BIAS, 2,
    {
      {"Which (0-31,32=all)", 0, 32, 'i', ""},
      {"Phase (degrees)", 0, 360, 'f', ""}
    }
  },
  {COMMAND(bias1_step), "step through different bias levels on MB #1",
    GR_BIAS, 5,
    {
      {"Start (full=32767)", 0, MAX_15BIT, 'i', "STEP_START_BIAS"},
      {"End (full=32767)", 0, MAX_15BIT, 'i', "STEP_END_BIAS"},
      {"N steps", 1, MAX_15BIT, 'i', "STEP_NSTEPS_BIAS"},
      {"Time per step (ms)", 100, MAX_15BIT, 'i', "STEP_TIME_BIAS"},
      {"Which (0-31,32=all)", 0, 32, 'i', "STEP_WHICH_BIAS"},
    }
  },
  {COMMAND(phase1_step), "step through different phases on MB #1",
    GR_BIAS, 4,
    {
      {"Start (degrees)", 0, 360, 'f', "STEP_START_PHASE"},
      {"End (degrees)", 0, 360, 'f', "STEP_END_PHASE"},
      {"N steps", 1, MAX_15BIT, 'i', "STEP_NSTEPS_PHASE"},
      {"Time per step (ms)", 100, MAX_15BIT, 'i', "STEP_TIME_PHASE"},
    }
  },

  {COMMAND(dac2_ampl), "Set bias signal amplitude on MB #2", GR_BIAS, 2,
    {
      {"Which (0-31,32=all)", 0, 32, 'i', ""},
      {"Amplitude (full=32767)", 1, MAX_15BIT, 'i', ""}
    }
  },
  {COMMAND(dac2_phase), "Set lock-in phase  on MB #2", GR_BIAS, 2,
    {
      {"Which (0-31,32=all)", 0, 32, 'i', ""},
      {"Phase (degrees)", 0, 360, 'f', ""}
    }
  },
  {COMMAND(bias2_step), "step through different bias levels on MB #2",
    GR_BIAS, 5,
    {
      {"Start (full=32767)", 0, MAX_15BIT, 'i', "STEP_START_BIAS"},
      {"End (full=32767)", 0, MAX_15BIT, 'i', "STEP_END_BIAS"},
      {"N steps", 1, MAX_15BIT, 'i', "STEP_NSTEPS_BIAS"},
      {"Time per step (ms)", 100, MAX_15BIT, 'i', "STEP_TIME_BIAS"},
      {"Which (0-31,32=all)", 0, 32, 'i', "STEP_WHICH_BIAS"},
    }
  },
  {COMMAND(phase2_step), "step through different phases on MB #2",
    GR_BIAS, 4,
    {
      {"Start (degrees)", 0, 360, 'f', "STEP_START_PHASE"},
      {"End (degrees)", 0, 360, 'f', "STEP_END_PHASE"},
      {"N steps", 1, MAX_15BIT, 'i', "STEP_NSTEPS_PHASE"},
      {"Time per step (ms)", 100, MAX_15BIT, 'i', "STEP_TIME_PHASE"},
    }
  },

  {COMMAND(reset_adc), "Reset an ADC motherboard", GR_POWER, 1,
    {
      {"Node number",  0, 64, 'i', ""}
    }
  },
  {COMMAND(az_el_raster), "Az-El mount raster scan", GR_AZEL, 9,
    { 
      {"az raster centre (deg)",  -180.0, 180.0, 'f', "AZ"}, 
      {"el raster centre (deg)", -5.0, 85.0, 'f', "EL"},  
      {"az raster scan width (deg)", 0.0, 180.0, 'f', "WIDTH_AZ"}, 
      {"el raster height (deg)", 0.0, 90.0, 'f', "HEIGHT_EL"},
      {"az raster scan speed (deg/s)", 0.0, 5.0, 'f', "V_AZ"},  
      {"el raster scan speed (deg/s)", 0.0, 5.0, 'f', "V_EL"},
      {"az raster acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_AZ"},
      {"el raster acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_EL"},
      {"el raster no. of steps", 0, MAX_15BIT, 'i', "N_EL"},

    }
  },
  {COMMAND(az_el_goto), "Az-El mount goto mode", GR_AZEL, 6,
    {
      {"az goto acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_AZ"},
      {"el goto acceleration (deg/s^2)", 0.0, 5.0, 'f', "A_EL;"},
      {"az goto speed (deg/s)", 0.0, 5.0, 'f', "V_AZ"},
      {"el goto speed (deg/s)", 0.0, 5.0, 'f', "V_EL"},
      {"az goto position (deg)", -180, 180.0, 'f', "AZ"}, 
      {"el goto position (deg)", -5.0, 85.0, 'f', "EL"}  
    }
  },
  {COMMAND(az_el_set), "Input Az-El mount starting angles", GR_AZEL, 2,
    {
      {"starting azimuth (deg)", -180.0, 180.0, 'f', "AZ_NOW"},
      {"starting elevation (deg)", -10.0, 86.0, 'f', "EL_NOW"},
    }
  }, 
  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, MAX_15BIT, 'i', "PLOVER"}
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
