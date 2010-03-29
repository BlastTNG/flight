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
#include <time.h>
#include <errno.h>

#include "bbc_pci.h"

//define to use external serial numbers and interrupt generation
//#define USE_EXT_SERIAL

#define FRAMELEN  64

int main(int argc, char *argv[]) {
  int fp;
  unsigned int i[0x1000 * 2];
  unsigned int j, k, numerrs, secret[2], oldsecret; 
  unsigned int serial;
  unsigned int num;
  struct timeval tv_frame;
  double frametime, old_frametime = 0.0;
  char frate_str[11] = "";
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    perror("Error opening BBCPCI\n");
    exit(0);
  }
  system("clear");
  fprintf(stderr, "NIOS program version %8x.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  
  i[0] = BBCPCI_WFRAME_ADD(0); 
  i[1] = BBC_FSYNC | 1;
  i[2] = BBCPCI_WFRAME_ADD(1);
  i[3] = 1;
  i[4] = BBCPCI_WFRAME_ADD(2);
  i[5] = 1;
  for(k = 3; k < FRAMELEN+3; k++) {
      i[2*k] = BBCPCI_WFRAME_ADD(k);
      num = k - 3;
      //num = (num*2) % 64;
      i[2*k+1] = BBC_DATA((0xc000+num)) | BBC_NODE(num) | BBC_CH(0) | BBC_READ;
  }
  for(k = FRAMELEN+3; k < 2*FRAMELEN+3; k++) {
      i[2*k] = BBCPCI_WFRAME_ADD(k);
      num = k - 3 - FRAMELEN;
      //num = (num*2) % 64;
      i[2*k+1] = BBC_DATA((0x8000+num)) | BBC_NODE(num) | BBC_CH(0) | BBC_WRITE;
  }
  i[2*k] = BBCPCI_WFRAME_ADD(2*FRAMELEN+3);
  i[2*k+1] = BBC_ENDWORD;
  k++;
  
  for (j = 0; j < k; j++)  {
    fprintf(stderr, "1: %02x ", write(fp, (void *)(i + 2 * j), 2 * BBCPCI_SIZE_UINT));
    fprintf(stderr, "%08x %08x\n", i[j * 2], i[j * 2 + 1]);
  }

  //usleep(6000000); 
  //for (oldsecret = 0xabcdabcd, k = 0; k < 100000; k++) {
  //  ioctl(fp, BBCPCI_IOC_SECRET, &secret);
  //  if (oldsecret != secret[0])
  //    printf("===> %08x %08x\n", secret[0], secret[1]);
  //  oldsecret = secret[0];
 // }
#ifdef USE_EXT_SERIAL
  printf("\nUsing external serial number generation\n");
#else
  printf("\nUsing internal serial number generation\n");
#endif

  getchar();
  
  usleep(100000);
  ioctl(fp, BBCPCI_IOC_ON_IRQ);
  ioctl(fp, BBCPCI_IOC_SYNC);
  usleep(100000);
  
#ifdef USE_EXT_SERIAL
  ioctl(fp, BBCPCI_IOC_EXT_SER_ON);
  ioctl(fp, BBCPCI_IOC_IRQ_RATE, 1);
  ioctl(fp, BBCPCI_IOC_FRAME_RATE, 1);
#else
  ioctl(fp, BBCPCI_IOC_EXT_SER_OFF);
  ioctl(fp, BBCPCI_IOC_IRQ_RATE, 320000);
#endif
  
  numerrs = 0;
  
  while (1) {
    for (k = 0; read(fp, (void *)(&j), sizeof(unsigned int)) == 4; k++) {
      serial = ioctl(fp, BBCPCI_IOC_GET_SERIAL);
      if (j & BBC_FSYNC) {
	gettimeofday(&tv_frame, NULL);
	frametime = tv_frame.tv_sec + (double)(tv_frame.tv_usec)/1.0e6;
	snprintf(frate_str, 10, " %8g", 1.0/(frametime-old_frametime));
	old_frametime = frametime;
      }
      else frate_str[0] = '\0';
      printf("%03x %08x %08x%s\n", k, j, serial, frate_str);  
      //printf("%03x %08x\n", k, j);  
      write(fp, (void *)&i[2], 2 * BBCPCI_SIZE_UINT);
    }
    usleep(100);
  }
  
  return 0;
}
