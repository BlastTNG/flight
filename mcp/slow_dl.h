/* slow_dl.h: slow downlink definitions and prorotypes for BLAST
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

#define SLOWDL_NUM_DATA     209

#define SLOWDL_LEN          255

#define SLOWDL_DLE          0x10
#define SLOWDL_SYNC         0x53
#define SLOWDL_ETX          0x03

#define SLOWDL_TAKE_BIT     0
#define SLOWDL_FORCE_INT    1
#define SLOWDL_U_MASK       2

struct SlowDLStruct {
  char src[20];
  char type;
  int numbits;
  double calib_min;
  double calib_max;
  double value;
  int wide;
  int mindex;
  int chnum;
  double min;
  double max;
};

void InitSlowDL(void);
