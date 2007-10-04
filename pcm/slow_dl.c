/* slow_dl.c: slow downlink packet specification
 *
 * This software is copyright (C) 2004-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "channels.h"
#include "slow_dl.h"
#include "blast.h"

/* SLOWDL_FORCE_INT force an integer on numbits between min and max */
/* SLOWDL_U_MASK    masks off all but the lowest numbits and sends those */
/* SLOWDL_TAKE_BIT  takes bit numbits, counting from 1 */
/* "src", type, numbits, min, max */
struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {
  /* TODO need to add stuff back here, also check implementation of
   * SlowDL things (especially in mcp.c)
   */
};

void InitSlowDL(void) {
  int i;
  double dtmp;
  struct NiosStruct *address;
  
  for (i = 0; i < SLOWDL_NUM_DATA; i++) {
    address = GetNiosAddr(SlowDLInfo[i].src);
    SlowDLInfo[i].wide = address->wide;
    SlowDLInfo[i].mindex = ExtractBiPhaseAddr(address)->index;
    SlowDLInfo[i].chnum = ExtractBiPhaseAddr(address)->channel;
  
    SlowDLInfo[i].max = (SlowDLInfo[i].calib_max - address->b) / address->m;
    SlowDLInfo[i].min = (SlowDLInfo[i].calib_min - address->b) / address->m;
    
    if( SlowDLInfo[i].max <  SlowDLInfo[i].min ) {
      dtmp = SlowDLInfo[i].max;
      SlowDLInfo[i].max = SlowDLInfo[i].min;
      SlowDLInfo[i].min = dtmp;
    }
  }
}
