/* command_list.c: BIT command specification file
 *
 * This software is copyright (C) 2013 University of Toronto
 *
 * This file is part of the BIT flight code licensed under the GNU
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "command_list.h"

const char *const command_list_serial = "$Rev: 7043 $";

const char *const GroupNames[N_GROUPS] = {
  "Rate Gyros",			"Reaction Wheel",			"Pivot Motor",
  "Coarse Motors",		"Fine Motors",				"Star Cameras",
  "Magnetometer",		"Miscellaneous"
};

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

/* parameter value lists */


const struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(killprog), "Closes the main BIT program!", GR_MISC},
  {COMMAND(gyro_on), "Initializes gyros.", GR_GYRO},
  {COMMAND(gyro_off), "Closes gyros.", GR_GYRO},
  {COMMAND(rw_on), "Initializes reaction wheel.", GR_RW},
  {COMMAND(rw_off), "Closes reaction wheel.", GR_RW},
  {COMMAND(pivot_on), "Initializes pivot.", GR_PIVOT},
  {COMMAND(pivot_off), "Closes pivot.", GR_PIVOT},
  {COMMAND(p_stp_on), "Initializes ptich stepper motor.s", GR_STPMTR},
  {COMMAND(p_stp_off), "Closes pitch stepper motors.", GR_STPMTR},
  {COMMAND(p_frm_on), "Initializes pitch frameless motors.", GR_FRMMTR},
  {COMMAND(p_frm_off), "Closes pitch frameless motors.", GR_FRMMTR},
  {COMMAND(r_frm_on), "Initializes roll frameless motors.", GR_FRMMTR},
  {COMMAND(r_frm_off), "Closes roll frameless motors.", GR_FRMMTR},
  {COMMAND(mag_on), "Initializes magnetometer.", GR_MAG},
  {COMMAND(mag_off), "Closes magnetometer", GR_MAG},

  /* DON'T PUT ANYTHING BELOW THIS */
  {COMMAND(xyzzy), "nothing happens here", GR_MISC}
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * l :  parameter is 30 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * d :  parameter is 30 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
const struct mcom mcommands[N_MCOMMANDS] = {
  
  {COMMAND(rw_torque), "Open-loop RW torque command.", GR_RW, 1,
    {
      {"RW torque (Nm):",     -15, 15, 'd', "rw_torque"},
    }
  },
  {COMMAND(rw_gains), "Reaction wheel gains.", GR_RW, 4,
    {
      {"RW Speed Kp:",     -100, 0, 'd', "rw_kp_wrw"},
      {"RW Speed Ki:",     -100, 0, 'd', "rw_ki_wrw"},
      {"BIT Az. Speed Kp:",     -100, 0, 'd', "rw_kp_waz"},
      {"BIT Az. Speed Kp:",     -100, 0, 'd', "rw_ki_waz"},
    }
  },
  {COMMAND(rw_nspeed), "Set RW nominal speed", GR_RW, 1,
    {
      {"RW Nomial Speed (rad/s):",     -3, 3, 'd', "rw_nspeed"},
    }
  },
  {COMMAND(pivot_speed), "Open-loop pivot speed command", GR_PIVOT, 1,
    {
      {"Pivot speed (rad/s):",     -3, 3, 'd', "pivot_speed"},
    }
  },
  {COMMAND(pivot_gains), "Pivot gains.", GR_PIVOT, 4,
    {
						{"RW Speed Kp:",     -100, 0, 'd', "pivot_kp_wrw"},
      {"RW Speed Ki:",     -100, 0, 'd', "pivot_ki_wrw"},
      {"BIT Az. Speed Kp:",     -100, 0, 'd', "pivot_kp_waz"},
      {"BIT Az. Speed Kp:",     -100, 0, 'd', "pivot_ki_waz"},
    }
  },
  {COMMAND(pivot_ustp), "Set the pivot motor micro-resolution", GR_PIVOT, 1,
    {
      {"Pivot micro-resolution (ustp/stp):",     2, 256, 'd', "pivot_ustp"},
    }
  },
  {COMMAND(p_frm_torque), "Open-loop fine pitch torque command.", GR_FRMMTR, 1,
    {
      {"Fine pitch torque (Nm):",     -15, 15, 'd', "frm_ptorque"},
    }
  },
  {COMMAND(r_frm_torque), "Open-loop fine roll torque command.", GR_FRMMTR, 1,
    {
      {"Fine roll torque (Nm):",     -15, 15, 'd', "frm_rtorque"},
    }
  },
  {COMMAND(p_stp_torque), "Open-loop coarse pitch torque command.", GR_STPMTR, 1,
    {
      {"Coarse pitch torque (Nm):",     -15, 15, 'd', "stp_ptorque"},
    }
  },

  /* DON'T PUT ANYTHING BELOW THIS */
  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, USHRT_MAX, 'i', "PLOVER"}
    }
  }
};

// validates the parameter list for a given command with n parameters
// returns the index of the mcommand given
int mcom_validate(char *cmd, float *params, int n)
{
	int j, i;
	for (j=0;j<N_MCOMMANDS;j++) if (strcmp(mcommands[j].name,cmd)==0) break; // get index
	if (j==N_MCOMMANDS) return -1; // no command found
	if (mcommands[j].numparams != n) return -1; // incorrect number of parameters
	for (i=0;i<n;i++) // check all parameter limits
		if ((params[i] < mcommands[j].params[i].min) || (params[i] > mcommands[j].params[i].max)) return -1;

	return j; // no issues with the command; return index
}
