/* test_bbcpci: sample program for the bbc_pci driver
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * test_bbcpci is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * test_bbcpci is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with test_bbcpci; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

#define FRAMELEN  0x0

int main(int argc, char *argv[]) {
  int fp;
  unsigned int i[0x1000 * 2];
  unsigned int j, k, numerrs, secret[2], oldsecret; 
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    fprintf(stderr, "Error opening BBCPCI\n");
    exit(0);
  }
  system("clear");
  fprintf(stderr, "NIOS program version %8x.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  
  /* BBC 2. */
  i[0] = BBCPCI_WFRAME2_ADD(0); 
  i[1] = BBC_FSYNC | 2;
  for(k = 1; k < FRAMELEN+1; k++) {
      i[2*k] = BBCPCI_WFRAME2_ADD(k);
      i[2*k+1] = BBC_DATA((0xa000 | k)) | BBC_NODE(8) | BBC_CH(0) | BBC_WRITE;
  }
  i[2*k] = BBCPCI_WFRAME2_ADD(FRAMELEN+1);
  i[2*k+1] = BBC_ENDWORD;
  k++;
  
  for (j = 0; j < k; j++)  {
    fprintf(stderr, "1: %02x ", write(fp, (void *)(i + 2 * j), 2 * BBCPCI_SIZE_UINT));
    fprintf(stderr, "%08x %08x\n", i[j * 2], i[j * 2 + 1]);
  }

  i[0] = BBCPCI_WFRAME1_ADD(0); 
  i[1] = BBC_FSYNC | 1;
  i[2] = BBCPCI_WFRAME1_ADD(1); 
  i[3] = BBC_DATA((0xb000 | 1)) | BBC_NODE(1) | BBC_CH(0) | BBC_WRITE;
  i[4] = BBCPCI_WFRAME1_ADD(2); 
  i[5] = BBC_DATA((0xb000 | 2)) | BBC_NODE(1) | BBC_CH(1) | BBC_WRITE;
  i[6] = BBCPCI_WFRAME1_ADD(3);
  i[7] = BBC_ENDWORD;
  for (k = 0; k < 4; k++) {
    fprintf(stderr, "2: %02x  ", write(fp, (void *)(i + k * 2), 2 * BBCPCI_SIZE_UINT));
    fprintf(stderr, "%08x %08x\n", i[k * 2], i[k * 2 + 1]);
  }
  
  //usleep(6000000); 
  //for (oldsecret = 0xabcdabcd, k = 0; k < 100000; k++) {
  //  ioctl(fp, BBCPCI_IOC_SECRET, &secret);
  //  if (oldsecret != secret[0])
  //    printf("===> %08x %08x\n", secret[0], secret[1]);
  //  oldsecret = secret[0];
 // }
  getchar();
  
  usleep(100000);
  ioctl(fp, BBCPCI_IOC_SYNC);
  usleep(100000);
  
  numerrs = 0;
  
  while (1) {
    for (k = 0; read(fp, (void *)(&j), sizeof(unsigned int)) == 4; k++) {
      printf("%03x %08x\n", k, j);  
      write(fp, (void *)&i[2], 2 * BBCPCI_SIZE_UINT);
      usleep(100);
    }
  }
  
  return 0;
}
