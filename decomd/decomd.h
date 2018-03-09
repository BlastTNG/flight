
/* decomd.h: contains definitions for decomd
 *
 * This software is copyright (C) 2017 University of Southern California
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdint.h>

#define BI0_FRAME_BUFBITS (4)
#define NUM_FRAMES (1 << BI0_FRAME_BUFBITS)
#define BI0_FRAME_BUFMASK (NUM_FRAMES-1)

typedef struct
{
    int i_in;
    int i_out;
    uint8_t *framelist[NUM_FRAMES];
    size_t framesize[NUM_FRAMES];
} frames_list_t;

