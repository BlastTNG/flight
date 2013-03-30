/* slowdl_struct.c: contains the slow channel lists
 *
 * This software is copyright (C) 2013 University of Toronto
 *
 * This file is part of mcp/pcm licensed under the GNU General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include "slowdl.h"

// If encoding is SDL_RAW, then type can be c, s, u, S, or U.
// if encoding is SDL_SCALE, then type can be s, u, or U.
struct SlowDlStruct slowDLList[] = {
  {"time", 'U', SDL_RAW},
  {"t_hs_cc1", 'c', SDL_SCALE, 70.0, -45.0},
  {""}
};

