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
  int fp, l, serial_signed;
  unsigned int i[0x1000 * 2], serial, frame_serial;
  unsigned int j, k, numerrs, secret[2], oldsecret; 
  unsigned long long cputime;
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    fprintf(stderr, "Error opening BBCPCI (code %d).\n", fp);
    exit(0);
  }
  system("clear");
  fprintf(stderr, "NIOS program version %8x.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  ioctl(fp, BBCPCI_IOC_RESET);
  
  i[0] = BBCPCI_WFRAME_ADD(0); 
  i[1] = BBC_FSYNC | 1;
  i[2] = BBCPCI_WFRAME_ADD(1); 
  i[3] = BBC_DATA((0xb000 | 1)) | BBC_NODE(63) | BBC_CH(0) | BBC_WRITE;
  i[4] = BBCPCI_WFRAME_ADD(2); 
  i[5] = BBC_DATA((0xb000 | 2)) | BBC_NODE(63) | BBC_CH(1) | BBC_WRITE;
  i[6] = BBCPCI_WFRAME_ADD(3);
  i[7] = BBC_DATA((0xb000 | 2)) | BBC_NODE(0) | BBC_CH(1) | BBC_WRITE;
  i[8] = BBCPCI_WFRAME_ADD(3);
  i[9] = BBC_ENDWORD;
  for (k = 0; k < 5; k++) {
    fprintf(stderr, "2: %02x  ", write(fp, (void *)(i + k * 2), 
                                 2 * BBCPCI_SIZE_UINT));
    fprintf(stderr, "%08x %08x\n", i[k * 2], i[k * 2 + 1]);
  }
  getchar();
  
  usleep(100000);
  ioctl(fp, BBCPCI_IOC_SYNC);
  usleep(100000);

  ioctl(fp, BBCPCI_IOC_ON_IRQ);
//  ioctl(fp, BBCPCI_IOC_EXT_SER_ON);
//  ioctl(fp, BBCPCI_IOC_IRQ_RATE, 1);
//  ioctl(fp, BBCPCI_IOC_FRAME_RATE, 1);
  ioctl(fp, BBCPCI_IOC_EXT_SER_OFF);
  ioctl(fp, BBCPCI_IOC_IRQ_RATE, 320000);
  
  numerrs = 0;
  
  while (1) {
    for (k = 0; read(fp, (void *)(&j), sizeof(unsigned int)) == 4; k++) {
      if (GET_STORE(j)) {
        char serial_string[40]="xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx";
        int ibit, ipos;
        serial = ioctl(fp, BBCPCI_IOC_GET_SERIAL);
        ioctl(fp, BBCPCI_IOC_IRQT_READ, &cputime);
        if (GET_NODE(j) == 63) {
          if (GET_CH(j) == 0)
            frame_serial = (frame_serial & 0xffff0000) | (j & 0xffff);
          if (GET_CH(j) == 1)
            frame_serial |= (j & 0xffff) << 16;
        }
      
        for (ibit=31; ibit >= 0; ibit--) {
          ipos = 31-ibit + 7-(ibit/4);
          serial_string[ipos] = (serial & (1<<ibit)) ? '*' : '.';
        }
        printf("%03x %08x %01x %08x %s %08x\n", k, j,
               ioctl(fp, BBCPCI_IOC_SERIAL_RDY), serial, serial_string,
               frame_serial);
        write(fp, (void *)&i[2], 2 * BBCPCI_SIZE_UINT);
        usleep(100);
      }
    }
  }
  
  return 0;
}
