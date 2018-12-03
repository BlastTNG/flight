/* slowdl.h: contains the slow channel defines
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
#ifndef INCLUDE_SLOWDL_H
#define INCLUDE_SLOWDL_H

#include "channels_tng.h"

#define SLOWDLSYNCWORD 0xeb90a174

#define SDL_RAW 0
#define SDL_SCALE 1
#define SDL_LOG 2
struct SlowDlStruct {
  char name[256];
  char type; // c s S u U (c is signed char)
  char encode; // RAW or Scale
  double min;
  double max;
  // 'private' variables: set by updateSlowDL
  channel_t *bi0s;
  double X[3]; // data as read by ReadCalData
  unsigned iX[3]; // data as read by ReadData
  int i_read; // for buffer
};

void updateSlowDL();

#endif /* INCLUDE_SLOWDL_H */
