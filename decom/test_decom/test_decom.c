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
#include <sys/ioctl.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include "decom_pci.h"
#define DEFAULT_FRAME_FILE  "../bbcpci/frame.dat"
#define WAITFRAMES          4

int main(int argc, char *argv[]) {
  int i, j, k, fp, framelen, numerrs, numunlocked;
  time_t now, lasterror;
  FILE *f;
  unsigned short *receivebuf, receiveword;
  char syncfound;
  char waiter[WAITFRAMES] = {'-', '\\', '|', '/'};
  char framefile[80];

  if (argc >= 2)
    strcpy(framefile, argv[1]);
  else
    strcpy(framefile, DEFAULT_FRAME_FILE);
  
  fp = open("/dev/decom_pci", O_RDWR);
  if (fp < 0) {
    printf("Error opening DECOM_PCI.\n");
    exit(0);
  }

  if ((f = fopen(framefile, "r")) == NULL) {
    printf("Could not open %s for reading.  Exiting.\n", framefile);
    return 0;
  }

  if (fscanf(f, "%d", &framelen) == EOF) {
    printf("Bad frame file (%s).  Couldn't read frame length.  Exiting.\n",
           framefile);
    return 0;
  }

  receivebuf = (unsigned short *)malloc(framelen * sizeof(unsigned short));
  
  receivebuf[0] = DECOM_SYNC;
  for (i = 1; i < framelen; i++) {
    if (fscanf(f, "%hx", receivebuf + i) == EOF) {
      printf("Bad frame file (%s).  Reached EOF at line %d; was expecting a "
             "frame size of %d.  Exiting.\n", framefile, i, framelen);
      return 0;
    }
  }

  fclose(f);

  printf("NIOS program version is %d.\n", ioctl(fp, DECOM_IOC_VERSION));
  printf("Using frame in file %s.\n", framefile);
  printf("Reseting board . . . \n");
  ioctl(fp, DECOM_IOC_RESET);
  printf("Set frame length to %d.\n", ioctl(fp, DECOM_IOC_FRAMELEN,
        framelen));

  numerrs = 0;
  numunlocked = 0;
  j = 0;
  k = 0;
  syncfound = 0;
  lasterror = time(NULL);
  
  for (i = 0; ; i++) {
    if (read(fp, (void *)(&receiveword), sizeof(unsigned short))) {
      if (syncfound) {
        if (!k)
          receivebuf[0] = ~receivebuf[0];
        if (receivebuf[k++] != receiveword) {
          numerrs++;
          lasterror = time(NULL);
          syncfound = 0;
        }
        if (k >= framelen)
          k = 0;
      }
      else {
        if (receiveword == DECOM_SYNC) {
          k = 1;
          syncfound = 1;
          receivebuf[0] = receiveword;
        }
      }
    }
    
    if (i % 100000 == 0) {
      now = time(NULL);
      printf("# errors = %3d | last error %5zd second%s ago %s| "
             "# unlocks = %3d %c\r",
             numerrs, now - lasterror, (now - lasterror) == 1 ? "" : "s", 
             (now - lasterror) == 1 ? " " : "", 
             ioctl(fp, DECOM_IOC_NUM_UNLOCKED), waiter[j++]);
      fflush(stdout);
      if (j >= WAITFRAMES)
        j = 0;
    }
  }
   
  return 0;
}
