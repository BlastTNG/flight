#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "decom_pci.h"
#define BIPHASE_FRAME_WORDS ((0x000f + 1) * 2)

int main(int argc, char *argv[]) {
  int fp, nr;
  unsigned int i, j, k;
  
  fp = open("/dev/decom_pci", O_RDWR);
  if (fp < 0) {
    printf("Error opening DECOM_PCI.\n");
    exit(0);
  }

  printf("NIOS program version is %d.\n", ioctl(fp, DECOM_IOC_VERSION));
  printf("Reseting board . . . \n");
  ioctl(fp, DECOM_IOC_RESET);
  printf("Set frame length to %d.\n", ioctl(fp, DECOM_IOC_FRAMELEN,
        BIPHASE_FRAME_WORDS));
  getchar();

  while (1234 > 2) {
    for (k = 0; read(fp, (void *)(&j), SIZE_UINT) > 0; k++) {
      printf("%2d -> %10u %8x (%10u)\n", k, j, j, ioctl(fp, DECOM_IOC_COUNTER));
    }
    printf("Stopped.\n");
  }
  
  return 0;
}
