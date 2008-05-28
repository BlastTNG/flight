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
#define N_MCOMMANDS 15       /* total number of multiword commands */
#define MAX_N_PARAMS 10
#define CMD_STRING_LEN 32      /* maximum allowable lenght of command string */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 80
#define SIZE_ABOUT 80
#define SIZE_PARNAME 80

#define N_GROUPS 9

//"not yet used" groups leftover from BLAST, can be changed
#define GR_TABLE        0x00000001
#define GR_MISC         0x00000002
#define GR_MOTORS       0x00000004
#define GR_SC_MISC      0x00000008
#define GR_SC_MODE      0x00000010
#define GR_SC_PARAM     0x00000020
#define GR_PT_MODE      0x00000040
#define GR_PT_GAIN      0x00000080
#define GR_PT_PARAM     0x00000100

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
  cam_lens_params, table_move, table_move_g, table_gain,
  pt_spin_vel
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
