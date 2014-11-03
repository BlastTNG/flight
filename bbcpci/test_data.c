/* test_data: test data coming from partner DSP program
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
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h> 
#include <time.h>
#include <errno.h>

#include "bbc_pci.h"

//define to use external serial numbers and interrupt generation
//#define USE_EXT_SERIAL

//define to print traffic a la test_bbcpci
//#define SHOW_TRAFFIC

#define NUM_NODES   4
#define	DATA_WORDS  50				//per node
#define FRAME_LEN   (NUM_NODES*DATA_WORDS + 4)	//start, dummy*2, end
const unsigned short node_list[NUM_NODES] = {0,1,2,3};

#define MAGIC_WORD    0xeb90
#define INVERSE_MAGIC ((~MAGIC_WORD)&0xffff)

ssize_t readFromBus(int bus_fd, void* data)
{
  int fd;
  struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(bus_fd, &rfds);
  fd = select(bus_fd+1, &rfds, NULL, NULL, &timeout);

  if (fd == -1) {
    perror("Problem reading from bus\n");
    return 0;
  } else if (!fd) { //timeout
    perror("Timeout reading from bus\n");
    return 0;
  }

  return read(bus_fd, data, sizeof(unsigned int));
}

int main(int argc, char *argv[]) {
  int fp;
  unsigned int f[FRAME_LEN * 2];
  unsigned int j, k, numerrs;
  unsigned int i_frame = 0;
#ifdef SHOW_TRAFFIC
  unsigned int serial;
  struct timeval tv_frame;
  double frametime, old_frametime = 0.0;
  char frate_str[11] = "";
#endif
  int old_inversion = 0;
  unsigned int data;
  int in_sync = 0;
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    perror("Error opening BBCPCI\n");
    exit(0);
  }
  if (system("clear") < 0) perror("can't clear\n");
  fprintf(stderr, "NIOS program version %8x.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  
  f[i_frame++] = BBCPCI_WFRAME_ADD(i_frame/2); 
  f[i_frame++] = BBC_FSYNC | 1;
  f[i_frame++] = BBCPCI_WFRAME_ADD(i_frame/2);
  f[i_frame++] = 1;
  f[i_frame++] = BBCPCI_WFRAME_ADD(i_frame/2);
  f[i_frame++] = 1;
  for(j=0; j<NUM_NODES; ++j) {
    for (k=0; k<DATA_WORDS; ++k) {
      f[i_frame++] = BBCPCI_WFRAME_ADD(i_frame/2);
      f[i_frame++] = BBC_DATA(0x1000*j+k) | BBC_NODE(node_list[j]) 
		      | BBC_CH(k) | BBC_READ;
    }
  }
  f[i_frame++] = BBCPCI_WFRAME_ADD(i_frame/2);
  f[i_frame++] = BBC_ENDWORD;
  
  for (j = 0; j < i_frame/2; ++j)  {
    fprintf(stderr, "1: %04x ", 
	(unsigned)write(fp, (void *)(f + 2 * j), 2 * BBCPCI_SIZE_UINT));
    fprintf(stderr, "%08x %08x\n", f[j * 2], f[j * 2 + 1]);
  }

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
    for (k = 0; readFromBus(fp, (void *)(&j)) == 4; ++k) {
#ifdef SHOW_TRAFFIC
      //this section prints traffic as per test_bbcpci
      serial = ioctl(fp, BBCPCI_IOC_GET_SERIAL);
      frate_str[0] = '\0';
      if (j & BBC_FSYNC) {
	gettimeofday(&tv_frame, NULL);
	frametime = tv_frame.tv_sec + (double)(tv_frame.tv_usec)/1.0e6;
	snprintf(frate_str, 10, " %8g", 1.0/(frametime-old_frametime));
	old_frametime = frametime;
      }
      printf("%03x %08x %08x%s\n", k, j, serial, frate_str);  
#endif
#if 0
      if (j & BBC_FSYNC) start_of_frame = 2;	  	//frame sync word
      else if (start_of_frame > 0) start_of_frame--;	//dummy words
      else {						//data word
#endif
      if (j & BBC_FSYNC) in_sync = 1;
      if (GET_STORE(j && in_sync)) {
        data = j&0xffff;
	if (data == MAGIC_WORD) {
	  if (old_inversion) 
	    printf("Err %d: word %x out of order\n", ++numerrs, data);
	  old_inversion = 1;
	} else if (data == INVERSE_MAGIC) {
	  if (!old_inversion)
	    printf("Err %d: word %x out of order\n", ++numerrs, data);
	  old_inversion = 0;
	} else {
	  printf("Err %d: word %x doesn't match magic words (%x %x)\n", 
	      ++numerrs, data, MAGIC_WORD, INVERSE_MAGIC);
	}
      }
    }
  }
  
  return 0;
}
