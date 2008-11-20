/* test_irq: sample program for the bbc_pci driver
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * test_irq is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * test_irq is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with test_irq; if not, write to the Free Software Foundation, Inc.,
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

#define TO_SEC(x)      ((double)(x) / 930552000)
#define TO_MSEC(x)     (TO_SEC(x) * 1e3)

int main(int argc, char *argv[]) {
  int fp, numbads;
  unsigned int serial, old_serial;
  struct tinfo_t {
    unsigned long long t;
    unsigned char bad;
  } tinfo;
  unsigned long long deltat, last_t, lastskip;
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    fprintf(stderr, "Error opening BBCPCI (code %d).\n", fp);
    exit(0);
  }
 
  numbads = 0;
  ioctl(fp, BBCPCI_IOC_ON_IRQ);
  ioctl(fp, BBCPCI_IOC_EXT_SER_ON);
  ioctl(fp, BBCPCI_IOC_IRQ_RATE, 1);
  while (1 == 1) {
    if (ioctl(fp, BBCPCI_IOC_IRQT_READ, &tinfo)) {
      if (!tinfo.bad) {
        serial = ioctl(fp, BBCPCI_IOC_GET_SERIAL);
        if (serial != old_serial + 1)
          printf("******* ");
        deltat = tinfo.t - last_t;
        if (TO_MSEC(deltat) > 24.61 || TO_MSEC(deltat) < 24.58)
          printf("------- ");
        printf("%d %-12.8gms %x\n", tinfo.bad, TO_MSEC(deltat),
               ioctl(fp, BBCPCI_IOC_GET_SERIAL));
        last_t = tinfo.t;
        old_serial = serial;
      }

/*      if (tinfo.bad)
        numbads++;
      else {
        deltat = tinfo.t - last_t;
        //if (deltat > 2450000 || deltat < 2250000) {
          printf("Last overlong: %12.8gs ago | Overlength: %12.8gms | "
                 "Falses: %u\n", TO_SEC(tinfo.t - lastskip), 
                 TO_MSEC(deltat), numbads);
          fflush(stdout);
          lastskip = tinfo.t;
        //}
        numbads = 0;
        last_t = tinfo.t;
      }
*/
    }
  }
  
  return 0;
}
