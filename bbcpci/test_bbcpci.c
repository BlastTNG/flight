#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

#define RS          23
#define ULEN        1000

int main(int argc, char *argv[]) {
  int fp, nr;
  unsigned int i[0x100 * 2];
  unsigned int j, k, l, m, numerrs, mycounter; 
  unsigned int buf[4];
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    printf("Error opening BBCPCI\n");
    exit(0);
  }

  system("clear");
  printf("NIOS program version is %d.\n", ioctl(fp, BBCPCI_IOC_VERSION));
  printf("Resetting card (takes 2.25 seconds) . . . \n");
  ioctl(fp, BBCPCI_IOC_RESET);
  usleep(2250000);

  /* BBC 1. */
  i[0] = WFRAME1_ADD(0); 
  i[1] = BBC_FSYNC;
  for (k = 1; k <= 16; k++) {
    i[k * 2] = WFRAME1_ADD(k);
    i[k * 2 + 1] = BBC_DATA(0xa000 + k - 1) | BBC_NODE(1) | BBC_CH(k - 1) | 
                   BBC_WRITE;
  }
  for (k = 1; k <= 16; k++) {
    i[k * 2 + 32] = WFRAME1_ADD(k + 16);
    i[k * 2 + 33] = BBC_NODE(1) | BBC_CH(k - 1) | BBC_READ;
  }
  i[66] = WFRAME1_ADD(33);
  i[67] = BBC_ENDWORD;
  
  /* BBC 2. */
  i[68] = WFRAME2_ADD(0); 
  i[69] = BBC_FSYNC;
  for (k = 1; k <= 16; k++) {
    i[k * 2 + 68] = WFRAME2_ADD(k);
    i[k * 2 + 69] = BBC_DATA(0xb000 + k - 1) | BBC_NODE(1) | BBC_CH(k + 15) | 
                    BBC_WRITE;
  }
  for (k = 1; k <= 16; k++) {
    i[k * 2 + 100] = WFRAME2_ADD(k + 16);
    i[k * 2 + 101] = BBC_NODE(1) | BBC_CH(k + 15) | BBC_READ;
  }
  i[134] = WFRAME2_ADD(33);
  i[135] = BBC_ENDWORD;

  /* Biphase. */
  i[136] = BI0_ADD(0);
  i[137] = BBC_BI0_SYNC;
  for (k = 1; k <= 16; k++) {
    i[k * 2 + 136] = BI0_ADD(k);
    i[k * 2 + 137] = 0x0aa0 + k - 1;
  }
  i[170] = BI0_ADD(17);
  i[171] = BBC_BI0_SYNC;
  for (k = 1; k <= 16; k++) {
    i[k * 2 + 170] = BI0_ADD(k + 17);
    i[k * 2 + 171] = 0x0bb0 + k - 1;
  }
  i[204] = BI0_ADD(34);
  i[205] = BBC_BI0_ENDWORD;
  
  printf("%d words written: \n\n", write(fp, (void *)i, 206 * SIZE_UINT));
  for (k = 0; k < 103; k++) 
    printf("%10u %10u %8x\n", i[k * 2], i[k * 2 + 1], i[k * 2 + 1]);
  printf("\nPress enter . . .\n");
  getchar();
  usleep(100000);
  ioctl(fp, BBCPCI_IOC_SYNC);
  usleep(100000);
  
  numerrs = 0;
  
  while (1) {
    for (k = 0; read(fp, (void *)(&j), sizeof(unsigned int)) == 4; k++) {
      if (GET_STORE(j) && ((j & 0x00f00000) > 0x00a00000)) {
        l = j & 0xff;
        if (++m > 15)
          m = 0;
        if (l != m)
          numerrs++;
        printf("%2u %10u %8x (%d, %d)\n", k, j, j, numerrs, 
               ioctl(fp, BBCPCI_IOC_COUNTER));
        m = l;
      }
    }
  }
  
  return 0;
}
