#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

#define FRAMELEN    0x555
#define DIVIDER     0x002

int main(int argc, char *argv[]) {
  int fp, nr;
  unsigned short i[FRAMELEN];
  unsigned int j, k, l, m, numerrs, mycounter; 
  unsigned int buf[4];
  
  fp = open("/dev/bi0_pci", O_RDWR);
  if (fp < 0) {
    printf("Error opening bi0_pci.\n");
    exit(0);
  }
  
  system("clear");
  printf("NIOS program version is %d.\n", ioctl(fp, BBCPCI_IOC_VERSION));

  /* Biphase. */ 
  i[0] = BBC_BI0_SYNC;
  for (k = 1; k < FRAMELEN; k++)
    i[k] = 0xa000 | k;
  for (k = 0; k < FRAMELEN; k++)
    printf("%3d %8hx\n", k, i[k]);
  printf("Press <ENTER>.\n");
  getchar();
  
    write(fp, (void *)i, (FRAMELEN - DIVIDER) * sizeof(unsigned short));
    write(fp, (void *)(i + FRAMELEN - DIVIDER), DIVIDER * 
	  sizeof(unsigned short));    
    while(1);
  
  return 0;
}
