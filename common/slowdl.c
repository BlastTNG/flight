/* slowdl.c: contains code for handling th 255 byte packets
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
#include <math.h>

#include "blast.h"

#include "slowdl.h"
#include "tx.h"


extern struct SlowDlStruct slowDLList[];

int slow_dl_read_index = 0;

// read and store the data for use by the writer thread.
// this runs in the main loop.
void updateSlowDL() {
  int i_ch;
  static int first_time = 1;
  int i_w;
  
  if (first_time) {
    int size = 0;
    for (i_ch = 0; slowDLList[i_ch].name[0]!='\0'; i_ch++) {
      slowDLList[i_ch].bi0s = GetBiPhaseAddr(slowDLList[i_ch].name);
      switch (slowDLList[i_ch].type) {
        case 'c':
          size++;
          break;
        case 's':
        case 'u':
          size += 2;
          break;
        case 'S':
        case 'U':
          size += 4;
          break;
        default:
          bprintf(fatal, "%c: unknown var type in slow dl field %s", 
                  slowDLList[i_ch].type, slowDLList[i_ch].name);
          break;
      }
      if (size>255) {
        bprintf(fatal, "slow downlink structure too large (%d > 255)", size);
      }
    }
    bprintf(info, "slow downlink packet %d/255", size);
    first_time = 0;
  }

  i_w = slow_dl_read_index+1;
  if (i_w==3) i_w = 0;
  
  for (i_ch = 0; slowDLList[i_ch].name[0]!='\0'; i_ch++) {
    if (slowDLList[i_ch].encode == SDL_RAW) {
      slowDLList[i_ch].iX[i_w] = ReadData(slowDLList[i_ch].bi0s);
    } else if (slowDLList[i_ch].encode == SDL_SCALE) {
      slowDLList[i_ch].X[i_w] = ReadCalData(slowDLList[i_ch].bi0s);
    } else if (slowDLList[i_ch].encode == SDL_LOG) {
      slowDLList[i_ch].X[i_w] = ReadCalData(slowDLList[i_ch].bi0s);
    }
  }
  slow_dl_read_index = i_w;
}

// convert and copy the data from the slowDLList into a char buffer
// runs in the sip thread - called on demand
void fillDLData(unsigned char *b, int len) {
  int i_r;
  int i_ch;
  unsigned short *u;
  unsigned *U;
  double x;
  unsigned char *b_end;
  
  b_end = b+len-4; // FIXME: wasted space if last field is not 32bit.
    
  i_r = slow_dl_read_index;

  U = (unsigned  *)b;
  *U = SLOWDLSYNCWORD;
  b+=4;
  
  for (i_ch = 0; (slowDLList[i_ch].name[0]!='\0') && (b<b_end); i_ch++) {
    if ((slowDLList[i_ch].encode == SDL_SCALE) || (slowDLList[i_ch].encode == SDL_LOG)) {
      double min, max;
      
      min = slowDLList[i_ch].min;
      max = slowDLList[i_ch].max;
      x = slowDLList[i_ch].X[i_r];
      
      if (slowDLList[i_ch].encode == SDL_LOG) {
        min = log(min);
        max = log(max);
        x = log(x);
      }
      
      /* scale between 0 and 1 */
      x = (x - min)/(max - min);      
      if (x>1.0) x = 1.0;
      if (x<0) x = 0;
      
      /* scale to fit in unsigned type.  s vs u is ignored. */
      switch (slowDLList[i_ch].type) {
        case 'c':
          slowDLList[i_ch].iX[i_r] = ((double)0xff)*x;
          break;
        case 's':
        case 'u':
          slowDLList[i_ch].iX[i_r] = ((double)0xffff)*x;
          break;
        case 'S':
        case 'U':
          slowDLList[i_ch].iX[i_r] = ((double)0xffffffff)*x;
          break;
        default:
          // should be impossible
          break;
      }
    }
    
    switch (slowDLList[i_ch].type) {
      case 'c':
        *b = (unsigned char)(0xff & slowDLList[i_ch].iX[i_r]);
        b++;
        break;
      case 's':
      case 'u':
        u = (unsigned short *)b;
        *u = (unsigned short)(0xffff & slowDLList[i_ch].iX[i_r]);
        b+=2;
        break;
      case 'S':
      case 'U':
        U = (unsigned  *)b;
        *U = slowDLList[i_ch].iX[i_r];
        b+=4;
        break;
      default:
        // 'impossible' error.
        break;
    }
  }
  while (b<b_end) {
    *b = i_ch++;
    b++;
  }
  
}
