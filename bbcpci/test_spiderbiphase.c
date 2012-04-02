/* test_spiderbiphase: sample program for the bbc_pci driver
 *
 * This software is copyright (C) 2012 C. Barth Netterfield
 * 
 * test_biphase is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * test_biphase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with test_biphase; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

// frame length in bytes
#define FRAMELEN 64
#define DELAY (1000000/4)

int main() {
  unsigned short *frame;
  int i;
  int bi0_fp;
  int nw;

  bi0_fp = open("/dev/bbc_bi0", O_RDWR);
  if (bi0_fp < 0) {
    printf("Error opening bi0_pci.\n");
    exit(0);
  }

  frame = (unsigned short *)malloc(FRAMELEN);

  frame[0] = 0xeb90;
  frame[1] = 0xc5c5;
  frame[2] = 0x3a3a;
  frame[3] = 0x146f;
  frame[4] = 0;
  for (i=5; i<20; i++) {
    frame[i] = i;
  }
  
  while (1) {
    frame[4]++;
    for (i=20; i<FRAMELEN; i++) {
      frame[i] = rand();
    }
    
    nw = write(bi0_fp, (void *)frame, FRAMELEN);
    printf("wrote %d bytes. Buffer has %d.\n", nw, 
	   ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD));
    usleep(DELAY);
    printf("after delay, buffer has %d.\n", ioctl(bi0_fp, 	BBCPCI_IOC_BI0_FIONREAD));
  }
}
