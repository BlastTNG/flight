#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

#define  FRAMELEN   0x000f

int main(int argc, char *argv[]) {
  int fp, nr;
  unsigned int i[0x100 * 2];
  unsigned int j, k, l, m, numerrs, mycounter; 
  unsigned int buf[4];
  
  fp = open("/dev/bi0_pci", O_RDWR);
  if (fp < 0) {
    printf("Error opening bi0_pci.\n");
    exit(0);
  }
  
  system("clear");
  printf("NIOS program version is %d.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  printf("Resetting card ");
  fflush(stdout);
  ioctl(fp, BBCPCI_IOC_RESET);
  for (k = 0; ioctl(fp, BBCPCI_IOC_COMREG); k++) {
    if (!(k % 10000)) {
      printf(".");
      fflush(stdout);
    }
  }
  printf("\n");

  /* Biphase. */ 
  i[0] = (BBC_BI0_SYNC << 16) | 0xffff;
  for (k = 0; k < FRAMELEN; k++)
    i[k + 1] = ((k * 2) << 16) | (k * 2 + 1) | 0xa000a000;
  for (k = 0; k <= FRAMELEN; k++)
    printf("%3d %10u %8x\n", k, i[k], i[k]);
  printf("Press enter.\n");
  getchar();
  
  while (1) {
    for (k = 0; k <= FRAMELEN; k++) {
      write(fp, (void *)&(i[k]), sizeof(unsigned int));
      printf("%3d %4x %4x\n", k, i[k] & 0x0000ffff, i[k] >> 16);
    }
  }
  
  return 0;
}
