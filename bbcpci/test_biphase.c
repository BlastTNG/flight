#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

#include "bbc_pci.h"

#define MAX_FRAME_LEN   0x1000
#define WAITFRAMES      4

int main(int argc, char *argv[]) {
  int fp, framelen;
  unsigned short i[MAX_FRAME_LEN];
  unsigned int j, k; 
  char waiter[4] = {'-', '\\', '|', '/'};

  if (argc != 2) {
    printf("USAGE: test_biphase <frame length>\n");
    return 0;
  }

  framelen = atoi(argv[1]);
  if (framelen <= 0 || framelen >= MAX_FRAME_LEN) {
    printf("Frame length must be > 0 and <= %d.\n", MAX_FRAME_LEN);
    return 0;
  }
  
  fp = open("/dev/bi0_pci", O_RDWR);
  if (fp < 0) {
    printf("Error opening bi0_pci.\n");
    exit(0);
  }
  
  system("clear");
  printf("NIOS program version is %d.\n", ioctl(fp, BBCPCI_IOC_VERSION));

  /* Biphase. */ 
  i[0] = BBC_BI0_SYNC;
  for (k = 1; k < framelen; k++)
    i[k] = 0xa000 | k;
  for (k = 0; k < framelen; k++)
    printf("%3d %8hx\n", k, i[k]);
  printf("Press <ENTER>.\n");
  getchar();
  
  for (j = 0, k = 0; ; k++) {
    i[0] = ~i[0];
    write(fp, (void *)i, framelen * sizeof(unsigned short));
    if (k % 50 == 0) {
      printf("%c\r", waiter[j++]);
      if (j >= WAITFRAMES)
        j = 0;
      fflush(stdout);
    }
//    usleep(5);
  }
   
  return 1;
}
