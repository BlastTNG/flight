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

#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#include "netcmd.h"  /* common parts of command definitions moved here */

/* N_SCOMMANDS and N_MCOMMANDS are now automatically calculated at compile time
 */

#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

//maximum insert number for hk commands.
#define HK_MAX  6

#define N_GROUPS 8

#define GR_GYRO        0x00000001
#define GR_RW        0x00000002
#define GR_PIVOT         0x00000004
#define GR_STPMTR         0x00000008
#define GR_FRMMTR         0x00000010
#define GR_SC         0x00000020
#define GR_MAG         0x00000040
#define GR_MISC			0x00000080

//reserved for CONFIRM  0x80000000

extern const char *const command_list_serial;

extern const char *const GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  killprog,			gyro_on,			gyro_off,			rw_on,
  rw_off,			pivot_on,			pivot_off,			p_stp_on,
  p_stp_off,		p_frm_on,			p_frm_off,			r_frm_on,
  r_frm_off,		mag_on,				mag_off,
  
  
  

  /* DON'T PUT ANYTHING BELOW THIS */
  xyzzy, N_SCOMMANDS /* SENTINAL: this must be the last thing in this list */
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  rw_torque,			rw_gains,			rw_nspeed,			pivot_speed,
  pivot_gains,			pivot_ustp,			p_frm_torque,		r_frm_torque,
  p_stp_torque, 					
  
  
  

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
int mcom_validate(char *cmd, float *params, int n);

#endif /* COMMAND_LIST_H */
