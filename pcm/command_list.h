/* command_list.h: Spider command specification file definitions
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

#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#define N_SCOMMANDS 1        /* total number of single word cmds */
#define N_MCOMMANDS 1         /* total number of multiword commands */
#define MAX_N_PARAMS 10
#define CMD_STRING_LEN 32      /* maximum allowable lenght of command string */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 80
#define SIZE_ABOUT 80
#define SIZE_PARNAME 80

#define N_GROUPS 24

//TODO revise groups as necessary
#define GR_POINT        0x00000001
#define GR_BAL          0x00000002
#define GR_COOL         0x00000004
#define GR_TRIM         0x00000008
#define GR_ELECT        0x00000010
#define GR_BIAS         0x00000020
#define GR_VETO         0x00000040
#define GR_ACT          0x00000080
#define GR_CALLAMP      0x00000100
#define GR_GAIN         0x00000200
#define GR_FOCUS        0x00000400
#define GR_CRYO_HEAT    0x00000800
#define GR_POWER        0x00001000
#define GR_LOCK         0x00002000
#define GR_CRYO_CONTROL 0x00004000
#define GR_TELEM        0x00008000
#define GR_ISC_HOUSE    0x00010000
#define GR_OSC_HOUSE    0x00020000
#define GR_STAGE        0x00040000
#define GR_ISC_MODE     0x00080000
#define GR_OSC_MODE     0x00100000
#define GR_MISC         0x00200000
#define GR_ISC_PARAM    0x00400000
#define GR_OSC_PARAM    0x00800000

#define CONFIRM         0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  test
};

struct scom {
  enum singleCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
};

extern struct scom scommands[N_SCOMMANDS];

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  table_gain
};

struct par {
  char name[SIZE_PARNAME];
  double min;
  double max;
  char type;
  char field[20];
};

struct mcom {
  enum multiCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
  char numparams;
  struct par params[MAX_N_PARAMS];
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern struct mcom mcommands[N_MCOMMANDS];

#endif /* COMMAND_LIST_H */
