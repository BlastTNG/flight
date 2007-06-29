/* command_list.c: Spider command specification file
 *
 * This software is copyright (C) 2002-2007 University of Toronto
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
 * RECOMPILE AND REINSTALL BLASTCMD ON ARWEN!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "command_list.h"

const char *command_list_serial = "$Revision: 1.1.1.1 $";

//TODO revise this in parallel with defines in the header file
const char *GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Balance",          "Cooling", 
  "Pointing Sensor Trims", "Aux. Electronics", "Bias",
  "Pointing Sensor Vetos", "Actuators",        "Cal Lamp",
  "Pointing Motor Gains",  "Secondary Focus",  "Cryo Heat",
  "Subsystem Power",       "Lock Motor",       "Cryo Control",
  "Telemetry",             "ISC Housekeeping", "OSC Housekeeping",
  "X-Y Stage",             "ISC Modes",        "OSC Modes",
  "Miscellaneous",         "ISC Parameters",   "OSC Parameters"
};

//echoes as string; makes enum name the command name string
#define COMMAND(x) x, #x

struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(test), "just a test of command system", GR_MISC} 
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * l :  parameter is 30 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * d :  parameter is 30 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
struct mcom mcommands[N_MCOMMANDS] = {
  {COMMAND(table_gain), "starcam rotary table gains", GR_GAIN, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_table"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_table"}
    }
  }
};
