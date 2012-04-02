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
//NB: now defined in Makefile, for separate targets
//#define USE_EXT_SERIAL
//in external mode, number of serial numbers per frame
#define SERIAL_PER_FRAME 2
//internal frame rate, in units of 4MHz cycles
#define FRAME_RATE_INT 104*384

#define FRAMELEN  8

int main(int argc, char *argv[]) {
  int fp;
  unsigned int i[0x1000 * 2];
  unsigned int j, k, numerrs, secret[2], oldsecret; 
  unsigned int serial;
  unsigned int frame_count;
  int frame_stopped = 0;
  unsigned int num;
  struct timeval tv_frame;
  double frametime, old_frametime = 0.0;
  char frate_str[11] = "";

  fp = open("/dev/bbcpci", O_RDWR | O_NONBLOCK);
  if (fp < 0) {
    perror("Error opening BBCPCI\n");
    exit(0);
  }
  if (system("clear") < 0) perror("Failed to clear");
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
    i[2*k+1] = BBC_DATA((0xc000+num)) | BBC_NODE(num) | BBC_CH(0) | BBC_READ;
  }
  for(k = FRAMELEN+3; k < 2*FRAMELEN+3; k++) {
    i[2*k] = BBCPCI_WFRAME_ADD(k);
    num = k - 3 - FRAMELEN;
    i[2*k+1] = BBC_DATA((0x8000+num)) | BBC_NODE(num) | BBC_CH(0) | BBC_WRITE;
  }
  i[2*k] = BBCPCI_WFRAME_ADD(2*FRAMELEN+3);
  i[2*k+1] = BBC_ENDWORD;
  k++;

  for (j = 0; j < k; j++)  {
    fprintf(stderr, "1: %02x ", write(fp, (void *)(i + 2 * j), 2 * BBCPCI_SIZE_UINT));
    fprintf(stderr, "%08x %08x\n", i[j * 2], i[j * 2 + 1]);
  }

#ifdef USE_EXT_SERIAL
  fprintf(stderr,"\nUsing external serial number generation\n");
#else
  fprintf(stderr,"\nUsing internal serial number generation\n");
#endif

  getchar();

  usleep(100000);
  ioctl(fp, BBCPCI_IOC_OFF_IRQ);
  usleep(100000);
  ioctl(fp, BBCPCI_IOC_SYNC);
  //"reset" the rates so that older firmwares will work still
  ioctl(fp, BBCPCI_IOC_IRQ_RATE, 1);
  ioctl(fp, BBCPCI_IOC_FRAME_RATE, 1);
  usleep(100000);

#ifdef USE_EXT_SERIAL
  ioctl(fp, BBCPCI_IOC_EXT_SER_ON);
  ioctl(fp, BBCPCI_IOC_IRQ_RATE_EXT, 1);
  ioctl(fp, BBCPCI_IOC_FRAME_RATE_EXT, SERIAL_PER_FRAME);
  //BICEP2 firmwares with settable internal rate needs this:
  //ioctl(fp, BBCPCI_IOC_IRQ_RATE_INT, FRAME_RATE_INT);
  //ioctl(fp, BBCPCI_IOC_FRAME_RATE_INT, FRAME_RATE_INT);
#else
  ioctl(fp, BBCPCI_IOC_EXT_SER_OFF);
  ioctl(fp, BBCPCI_IOC_IRQ_RATE_INT, FRAME_RATE_INT);
  ioctl(fp, BBCPCI_IOC_FRAME_RATE_INT, FRAME_RATE_INT);
#endif

  numerrs = 0;

  while (1) {
    for (k = 0; read(fp, (void *)(&j), sizeof(unsigned int)) == 4; k++) {
      serial = ioctl(fp, BBCPCI_IOC_GET_SERIAL);
      frame_count = ioctl(fp, BBCPCI_IOC_FRAME_COUNT);
      if (j & BBC_FSYNC) {
	gettimeofday(&tv_frame, NULL);
	frametime = tv_frame.tv_sec + (double)(tv_frame.tv_usec)/1.0e6;
	snprintf(frate_str, 10, " %8g", 1.0/(frametime-old_frametime));
	old_frametime = frametime;
      }
      else frate_str[0] = '\0';
      printf("%05x %08x %08x %08x %s\n", k, j, serial, frame_count, frate_str);  
      //printf("%03x %08x\n", k, j);  
      if (write(fp, (void *)&i[2], 2 * BBCPCI_SIZE_UINT) < 0)
	perror("Write failed");

      frame_stopped = 0;
    }

    usleep(100);

    //check for stopped frames (dead sync box)
    frame_count = ioctl(fp, BBCPCI_IOC_FRAME_COUNT);
    if (!frame_stopped && frame_count > 4000000) {
      printf("%s %08x\n", "frames stopped at count", frame_count, "");  
      frame_stopped = 1;
    }
  }

  return 0;
}
