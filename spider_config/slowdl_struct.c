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
  {"time_i_flc", 'U', SDL_RAW},
  {"time_b_flc", 'U', SDL_RAW},
  {"frame_g", 'u', SDL_RAW},
  {"frame_b", 'u', SDL_RAW},  
  {"frame_u", 'u', SDL_RAW},
  {"nblobs_g", 'c', SDL_RAW},
  {"nblobs_b", 'c', SDL_RAW},
  {"nblobs_u", 'c', SDL_RAW},
  {"mapmean_g", 'c', SDL_SCALE, 0.0, 65535.0},
  {"mapmean_b", 'c', SDL_SCALE, 0.0, 65535.0},
  {"mapmean_u", 'c', SDL_SCALE, 0.0, 65535.0},
  {"az_pss", 'c', SDL_SCALE, 0.0, 360.0},
  {"az_dgps", 'c', SDL_SCALE, 0.0, 360.0},
  {"az_mag", 'c', SDL_SCALE, 0.0, 360.0},
  {"el", 'c', SDL_SCALE, 10.0, 60.0},
  {"plover", 'c', SDL_RAW},
  {"count_b_cmd", 'u', SDL_RAW},
  {"count_i_cmd", 'u', SDL_RAW},
  {"last_b_cmd", 'u', SDL_RAW},
  {"last_i_cmd", 'u', SDL_RAW},
  {"i_tot", 'c', SDL_SCALE, 0.0, 60.0},
  {"v_batt_cc1", 'c', SDL_SCALE, 18.0, 36.0},
  {"v_batt_cc2", 'c', SDL_SCALE, 18.0, 36.0},
  {"t_cpu_i_flc", 'c', SDL_SCALE, -10.0, 90.0},
  {"t_cpu_b_flc", 'c', SDL_SCALE, -10.0, 90.0},
  {"vt_gy", 'c', SDL_SCALE, -45.0, 70.0},
  {"t_hs_cc1", 'c', SDL_SCALE, -45.0, 70.0},
  {"t_hs_cc2", 'c', SDL_SCALE, -45.0, 70.0},
  {""}
};

