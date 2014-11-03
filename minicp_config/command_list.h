/* command_list.h: Spider command specification file definitions
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

#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#include "netcmd.h"    /* common parts of command defintions moved here */
#include <sys/types.h>

#define N_SCOMMANDS 2          /* total number of single word cmds */
#define N_MCOMMANDS 13         /* total number of multiword commands */
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

//#define N_GROUPS 24
#define N_GROUPS 4

//"not yet used" groups leftover from BLAST, can be changed
#define GR_AZEL         0x00000001 
//#define GR_BAL          0x00000002  //not yet used
//#define GR_COOL         0x00000004  //not yet used
//#define GR_TRIM         0x00000008  //not yet used
//#define GR_ELECT        0x00000010  //not yet used
//#define GR_BIAS         0x00000020
#define GR_BIAS         0x00000002
//#define GR_VETO         0x00000040  //not yet used
//#define GR_ACT          0x00000080  //not yet used
//#define GR_CALLAMP      0x00000100  //not yet used
//#define GR_GAIN         0x00000200  //not yet used
//#define GR_FOCUS        0x00000400  //not yet used
//#define GR_CRYO_HEAT    0x00000800  //not yet used
//#define GR_POWER        0x00001000  //not yet used
#define GR_POWER        0x00000004 
//#define GR_LOCK         0x00002000  //not yet used
//#define GR_CRYO_CONTROL 0x00004000  //not yet used
//#define GR_TELEM        0x00008000  //not yet used
//#define GR_SC_MISC      0x00010000  //not yet used
//#define GR_FREE1        0x00020000  //not yet used
//#define GR_STAGE        0x00040000  //not yet used
//#define GR_SC_MODE      0x00080000  //not yet used
//#define GR_FREE2        0x00100000  //not yet used
#define GR_MISC         0x00000008
//#define GR_SC_PARAM     0x00400000  //not yet used
//#define GR_FREE3        0x00800000  //not yet used
//reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  az_el_disable,
  xyzzy
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  dac1_ampl,         dac1_phase,        bias1_step,        phase1_step,
  dac2_ampl,         dac2_phase,        bias2_step,        phase2_step,
  az_el_raster,      az_el_set,		az_el_goto,        reset_adc,         
  plugh
};

extern struct scom scommands[N_SCOMMANDS];

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern struct mcom mcommands[N_MCOMMANDS];

/* validator function for mcommands */
extern int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer);

#endif /* COMMAND_LIST_H */
