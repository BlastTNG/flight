/* test_decom: sample program for the decom driver
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * test_decom is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * test_decom is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with test_decom; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 
#include <string.h>
#include <errno.h>

#include "decom_pci.h"

#define SYNC_LEN 4
const unsigned short syncwords[SYNC_LEN] = {0xeb90, 0xc5c5, 0x3a3a, 0x146f};

int main(int argc, char *argv[]) {
  int i,j;
  int fp;
  unsigned short receiveword;
  int syncstate = 0, sync_pol;
  int polarity = 0;

  fp = open("/dev/decom_pci", O_RDWR);
  if (fp < 0) {
    perror("Error opening DECOM_PCI.\n");
    exit(0);
  }

  printf("NIOS program version is %08x.\n", ioctl(fp, DECOM_IOC_VERSION));
  printf("Reseting board . . . \n");
  ioctl(fp, DECOM_IOC_RESET);
  printf("Press any key to continue\n");

  getchar();		//wait

  for (i = 0; ; i++) {
    if (read(fp, (void *)(&receiveword), sizeof(unsigned short))) {
      if (polarity) receiveword = ~receiveword; //flip if inverse polarity

      //don't print zeros
      //NB: real decom program will be more sophisticated about word rejection
      if (receiveword != 0x0000) {
        printf("0x%04x\n", receiveword);
      }

      //detect sync words
      if (syncstate == 0) {
        if (receiveword == syncwords[0]) {
          syncstate = 1;
          sync_pol = 0;
        } else if (receiveword == (unsigned short)~syncwords[0]) {
          syncstate = 1;
          sync_pol = 1;
        }
      } else if (syncstate > 0 && receiveword == (unsigned short)
          (sync_pol ? ~syncwords[syncstate] : syncwords[syncstate])) {
        syncstate++;
      } else syncstate = 0;
      if (syncstate == SYNC_LEN) {  //full sync word detected
        printf("** %s%s **\n", sync_pol ? "Inverted " : "", "Sync Word");
        if (sync_pol) polarity = !polarity;
      }

    }
  }
   
  return 0;
}
