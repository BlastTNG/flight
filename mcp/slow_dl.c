/* slow_dl.c: slow downlink packet specification
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "slow_dl.h"

struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {
  {"t_dpm_5v", SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"cpu_time", SLOWDL_U_MASK,    16, 0.0, -1, -1, -1},
  {"t_gybox1", SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"gyro1",    SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"t_reac",   SLOWDL_FORCE_INT, 7,  0.0, -1, -1, -1},
  {"g_p_el",   SLOWDL_FORCE_INT, 8,  0.0, -1, -1, -1},
  {"i_el",     SLOWDL_TAKE_BIT,  3,  0.0, -1, -1, -1}
};
