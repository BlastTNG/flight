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

#define N_SCOMMANDS 6        /* total number of single word cmds */
#define N_MCOMMANDS 12       /* total number of multiword commands */
#define MAX_N_PARAMS 10
#define CMD_STRING_LEN 32      /* maximum allowable lenght of command string */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 80
#define SIZE_ABOUT 80
#define SIZE_PARNAME 80

#define N_GROUPS 24

//"not yet used" groups leftover from BLAST, can be changed
#define GR_POINT        0x00000001  //not yet used
#define GR_BAL          0x00000002  //not yet used
#define GR_COOL         0x00000004  //not yet used
#define GR_TRIM         0x00000008  //not yet used
#define GR_ELECT        0x00000010  //not yet used
#define GR_BIAS         0x00000020  //not yet used
#define GR_VETO         0x00000040  //not yet used
#define GR_ACT          0x00000080  //not yet used
#define GR_CALLAMP      0x00000100  //not yet used
#define GR_GAIN         0x00000200
#define GR_FOCUS        0x00000400  //not yet used
#define GR_CRYO_HEAT    0x00000800  //not yet used
#define GR_POWER        0x00001000  //not yet used
#define GR_LOCK         0x00002000  //not yet used
#define GR_CRYO_CONTROL 0x00004000  //not yet used
#define GR_TELEM        0x00008000  //not yet used
#define GR_SC_MISC      0x00010000
#define GR_FREE1        0x00020000  //not yet used
#define GR_STAGE        0x00040000  //not yet used
#define GR_SC_MODE      0x00080000
#define GR_FREE2        0x00100000  //not yet used
#define GR_MISC         0x00200000
#define GR_SC_PARAM     0x00400000
#define GR_FREE3        0x00800000  //not yet used

#define CONFIRM         0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  cam_expose, cam_autofocus, cam_settrig_ext, cam_force_lens, 
  cam_unforce_lens, test
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
  cam_any, cam_settrig_timed, cam_exp_params, cam_focus_params,
  cam_bad_pix, cam_blob_params, cam_lens_any, cam_lens_move, 
  cam_lens_params, table_move, table_move_g, table_gain
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
