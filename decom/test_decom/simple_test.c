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
#include <stdbool.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h> 
#include <string.h>
#include <errno.h>

#include "decom_pci.h"

#define BBC_MCLKS_PER_BBC_CLK  8
#define BBC_ADC_MULTIPLIER     384            /* set by the ADC hardware */
#define BBC_ADCS_PER_SAMPLE    104            /* this sets the frame size */
#define BI0_MCLKS_PER_BI0_CLK  32
#define BI0_WORD_SIZE          16

#define BI0_FRAME_SIZE        ((BBC_MCLKS_PER_BBC_CLK * BBC_ADC_MULTIPLIER * BBC_ADCS_PER_SAMPLE) \
                              / (BI0_MCLKS_PER_BI0_CLK * BI0_WORD_SIZE))
 
#define SYNC_LEN 1

const unsigned short syncwords[SYNC_LEN] = {0xeb90};

int main(int argc, char *argv[]) {
  int i, j, k = 0;
  int fp;
  unsigned short receiveword;
  int syncstate = 0, sync_pol;
  int polarity = 0;
  ssize_t bytes_read;
  ssize_t bytes_read_since_last_sync = 0;
  bool first_time_error = true;

  fp = open("/dev/decom_pci", O_RDWR);
  if (fp < 0) {
    perror("Error opening DECOM_PCI.\n");
    exit(0);
  }

  printf("NIOS program version is %08x.\n", ioctl(fp, DECOM_IOC_VERSION));
  printf("Reseting board . . . \n");
  ioctl(fp, DECOM_IOC_RESET);
  usleep(100);
  printf("Setting num words between sync words (DECOM_IOC_FRAMELEN) to BI0_FRAME_SIZE-1 = %d\n", 2*(BI0_FRAME_SIZE)-1);
  ioctl(fp, DECOM_IOC_FRAMELEN, 2*(BI0_FRAME_SIZE)-1);
  usleep(100);
  printf("Force unlock:\n");
  ioctl(fp, DECOM_IOC_FORCE_UNLOCK);
  usleep(100);
  printf("Some READ information:\n");
  printf("\tCounter = %d\n", ioctl(fp, DECOM_IOC_COUNTER));
  printf("\tLocked ? = %d\n", ioctl(fp, DECOM_IOC_LOCKED));
  printf("\tNum unlocked = %d\n", ioctl(fp, DECOM_IOC_NUM_UNLOCKED));
  printf("\tFIONREAD = %d\n", ioctl(fp, DECOM_IOC_FIONREAD));
  printf("Reseting board . . . \n");
  ioctl(fp, DECOM_IOC_RESET);
  usleep(100);


  for (i = 0; ; i++) {
    bytes_read = read(fp, (void *)(&receiveword), sizeof(unsigned short));
    if (bytes_read > 0) {
	  k = 0;
	  j += 1;
	  bytes_read_since_last_sync += bytes_read;
	  if ((receiveword == 0xeb90) || (receiveword == 0x146f)) {
        // printf("\n== j=%d ==\n", j);
        printf("\n");
	    j = 0;
	    bytes_read_since_last_sync = 0;
	  }
      printf("%04x ", receiveword);
 	  //printf("\nWord Received: 0x%04x\n", receiveword);
	  //printf("Read %zd bytes\n", bytes_read);
	  //printf("Number of reads since last sync: %d\n", j);
	  //printf("Number of bytes received since last sync: %zd\n", bytes_read_since_last_sync);
	  // printf("\tcounter = %d\n", ioctl(fp, DECOM_IOC_COUNTER));
	  // printf("\tlocked? = %d\n", ioctl(fp, DECOM_IOC_LOCKED));
	  // printf("\tnum unlocked = %d\n", ioctl(fp, DECOM_IOC_NUM_UNLOCKED));
	  // printf("\tFIONREAD = %d\n", ioctl(fp, DECOM_IOC_FIONREAD));

    } else {
	    k += 1;
	    // printf("\nNo Word: bytes_read is %zd, error is %s.\n", bytes_read, strerror(errno));
	    // printf("Number of reads without any bytes: %d\n", k);
	    // printf("counter = %d\n", ioctl(fp, DECOM_IOC_COUNTER));
	    // printf("locked? = %d\n", ioctl(fp, DECOM_IOC_LOCKED));
	    // printf("num unlocked = %d\n", ioctl(fp, DECOM_IOC_NUM_UNLOCKED));
	    // printf("FIONREAD = %d\n", ioctl(fp, DECOM_IOC_FIONREAD));
    }
  }
  return 0;
}
