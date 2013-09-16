/* command_list.c: Spider command specification file
 *
 * This software is copyright (C) 2002-20010 University of Toronto
 *
 * This file is part of the Spider flight code licensed under the GNU
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

#include "command_list.h"

const char *const command_list_serial = "$Rev: 7043 $";

/* parse the above; returns -1 if command_list_serial can't be parsed */
const int command_list_serial_as_int(void)
{
  int cmd_rev = -1;
  sscanf(command_list_serial, "$R" /* prevent SVN from munging this string */
      "ev: %i $", &cmd_rev);

  return cmd_rev;
}

#define CHOOSE_INSERT_PARAM "Insert", 0, 6, 'i', "NONE", {mce_names}, 1
#define CHOOSE_INSERT_NO_ALL "Insert", 1, 6, 'i',"NONE", {mce_names + 1}, 2
#define MCE_ACTION_PARAM(n,w) "Action", 0, n, 'i', "NONE", {w}
#define NO_YES_PARAM(n) n "?", 0, 1, 'i', "NONE", {noyes_names}

#define MCECMD1(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 1, { \
      {CHOOSE_INSERT_PARAM}, \
    }

#define MCECMD1P(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 1, { \
      {CHOOSE_INSERT_NO_ALL}, \
    }

#define MCECMD1AD(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 2, { \
      {CHOOSE_INSERT_PARAM}, \
      {MCE_ACTION_PARAM(4,daction_names)}, \
    }

#define MCECMD2(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 2, { \
      {CHOOSE_INSERT_PARAM}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMD2P(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 2, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMD2A(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 3, { \
      {CHOOSE_INSERT_PARAM}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMD2AP(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 3, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDC(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 3, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMDCA(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDCAD(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(4,daction_names)}, \
    }

#define MCECMDRAD(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(4,daction_names)}, \
    }

#define MCECMDCR(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMDCRA(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 5, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDCR1A(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDSCS(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_PARAM}, \
      {"Start", -65536, 65535, 'l', "NONE"}, \
      {"Count", 0, 65535, 'i', "NONE"}, \
      {"Step", -32768, 32767, 'i', "NONE"}, \
    }

const char *const GroupNames[N_GROUPS] = {
  "Group #1",        "Group #2",      "Group #3",
  "Miscellaneous"
};

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

/* parameter value lists */
const char *noyes_names[] = {"No", "Yes", NULL};
const char *nyd_names[] = {"No", "Yes", "On darks", NULL};
const char *mce_names[] = {"all", "X1", "X2", "X3", "X4", "X5", "X6", NULL};
const char *just_mce_names[] = {"X1", "X2", "X3", "X4", "X5", "X6", NULL};
const char *wb_cards[] = {"CC", "RC1", "RC2", "BC1", "BC2", "AC", NULL};
const char *action_names[] = {"Apply & Record", "Apply only",
  "Record & Reconfig", "Record only", NULL};
const char *daction_names[] = {"Apply & Record", "Apply only",
  "Record & Reconfig", "Record only", "Record default only", NULL};
const char *tunedata_names[] = {"expt.cfg", "SA Ramp sqtune",
  "SQ2 Servo sqtune", "SQ1 Servo sqtune", "SQ1 Ramp sqtune", NULL};
const char *zbias_names[] = {"TES", "squids", "all", NULL};

const struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(testcmd1), "This is test command #1.", GR_1 | GR_2},
  {COMMAND(testcmd2), "This is test command #2.", GR_2 | GR_3},
  {COMMAND(testcmd3), "This is test command #3.", GR_2 | GR_3},
  {COMMAND(testcmd4), "This is test command #4.", GR_1 | GR_2 | GR_3},

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
  
  {COMMAND(testcmd5), "This is test command #5", GR_1, 2,
    {
      {"Azimuth (deg)",     -360, 360, 'd', "AZ"},
      {"Elevation (deg)", 15.0,  45.0, 'f', "EL"}
    }
  },
  {COMMAND(testcmd6), "This is test command #6", GR_2, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },

  {COMMAND(testcmd7), "This is test command #7", GR_3, 4,
    {
      {"Max X", -65535, 65535, 'f', "cal_xmax_mag"},
      {"Min X", -65535, 65535, 'f', "cal_xmin_mag"},
      {"Max Y", -65535, 65535, 'f', "cal_ymax_mag"},
      {"Min Y", -65535, 65535, 'f', "cal_ymin_mag"}
    }
  },
  
  {COMMAND(testcmd8), "This is test command #8", GR_1 | GR_2 | GR_3, 4,
    {
      {"Max X", -65535, 65535, 'f', "cal_xmax_mag"},
      {"Min X", -65535, 65535, 'f', "cal_xmin_mag"},
      {"Max Y", -65535, 65535, 'f', "cal_ymax_mag"},
      {"Min Y", -65535, 65535, 'f', "cal_ymin_mag"}
    }
  },

  /* DON'T PUT ANYTHING BELOW THIS */
  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, USHRT_MAX, 'i', "PLOVER"}
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
  switch (cmd) {
    default:
      break; /* default: assume everything's fine */
  }

  /* if we got here parameter checks passed, I guess */
  return 0;
}
