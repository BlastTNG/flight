/* command_list.h: Spider command specification file definitions
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

#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#include "netcmd.h"  /* common parts of command defintions moved here */

/* N_SCOMMANDS and N_MCOMMANDS are now automatically calculated at compile time
 */

#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

//maximum insert number for hk commands.
#define HK_MAX  6

#define N_GROUPS 4

#define GR_1        0x00000001
#define GR_2        0x00000002
#define GR_3         0x00000004
#define GR_MISC         0x04000000

#define MCECMD          0x40000000 /* MCE command flag */
//reserved for CONFIRM  0x80000000

extern const char *const command_list_serial;
extern const int command_list_serial_as_int(void);

extern const char *const GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  testcmd1,         testcmd2,           testcmd3,           testcmd4,

  /* DON'T PUT ANYTHING BELOW THIS */
  xyzzy, N_SCOMMANDS /* SENTINAL: this must be the last thing in this list */
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  testcmd5,         testcmd6,           testcmd7,           testcmd8,

  /* DON'T PUT ANYTHING BELOW THIS */
  plugh,   N_MCOMMANDS, /* SENTINAL: this must be the last thing in the list */
  sched_packet = 0xff   //not really a command, more of a placeholder
};

extern const struct scom scommands[N_SCOMMANDS];

/* parameter type:
 * i :  parameter is 16 bit unnormalised integer
 * f :  parameter is 16 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern const struct mcom mcommands[N_MCOMMANDS];

/* validator function for mcommands */
extern int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer);

#endif /* COMMAND_LIST_H */
