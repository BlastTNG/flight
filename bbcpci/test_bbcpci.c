#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 
#include <time.h>

#include "bbc_pci.h"

#define F1LEN 10
#define F2LEN 5

#define WBUF write(fp, (void *)buf, 2*sizeof(unsigned int))

int main(int argc, char *argv[]) {
  int fp;
  unsigned buf[2], inbuf[1];
  int ret,i,frame = 0;
  
  fp = open("/dev/bbcpci", O_RDWR);
  if (fp < 0) {
    printf("Error opening BBCPCI\n");
    exit(0);
  }

  //reset nios
  //ioctl(fp, BBCPCI_IOC_RESET);
  while (ioctl(fp, BBCPCI_IOC_COMREG) !=0) {
     printf("%x %d\n", ioctl(fp, BBCPCI_IOC_COMREG), ioctl(fp, BBCPCI_IOC_READBUF_WP)); 
  }

  printf("NIOS program version is %d.\n", ioctl(fp, BBCPCI_IOC_VERSION));
 
  /* fill frames */
  buf[0] = WFRAME1_ADD(0);
  buf[1] = BBC_FSYNC;
  WBUF;
  
  buf[0] = WFRAME2_ADD(0);
  buf[1] = BBC_FSYNC;
  WBUF;
  
  for (i=1; i<F1LEN; i++) {
    buf[0] = WFRAME1_ADD(i);
    buf[1] = i;
    WBUF;
  }
  buf[0] = WFRAME1_ADD(i);
  buf[1] = BBC_ENDWORD;
  WBUF;

  for (i=1; i<F2LEN; i++) {
    buf[0] = WFRAME2_ADD(i);
    buf[1] = 0x2000 + i;
    WBUF;
  }
  buf[0] = WFRAME2_ADD(i);
  buf[1] = BBC_ENDWORD;
  WBUF;

  /* for (i=0; i<1000; i++) { */
/*     ret = ioctl(fp, BBCPCI_IOC_WRITEBUF_WP); */
/*     printf("writebuf: %x %d %d \n", ret, ioctl(fp, BBCPCI_IOC_WRITEBUF), ioctl(fp, BBCPCI_IOC_CBCOUNTER)); */
/*     usleep(1000); */
/*   } */
  
  ioctl(fp, BBCPCI_IOC_SYNC);
  //getchar();

  while (1) {
    while (read(fp, inbuf, SIZE_UINT)>0) {
      if (inbuf[0] == BBC_FSYNC) {
        buf[0] = WFRAME1_ADD(1);
        buf[1] = frame++;
        WBUF;
        //printf("."); fflush(stdout);
        printf("------------------------------\n");
      printf("%x %x %x %x %x\n", ioctl(fp, BBCPCI_IOC_READBUF_RP),
	     ioctl(fp, BBCPCI_IOC_CBCOUNTER), ioctl(fp, BBCPCI_IOC_JIFFIES),
	     ioctl(fp, BBCPCI_IOC_COUNTER), inbuf[0]);
      }
/*       printf("%x %x %x %x %x\n", ioctl(fp, BBCPCI_IOC_READBUF_RP), */
/* 	     ioctl(fp, BBCPCI_IOC_CBCOUNTER), ioctl(fp, BBCPCI_IOC_JIFFIES), */
/* 	     ioctl(fp, BBCPCI_IOC_COUNTER), inbuf[0]); */
    }
    //printf("%d %d\n", time(NULL), ioctl(fp, BBCPCI_IOC_COUNTER));
    usleep(10000);
  }
  
  return 0;
}
