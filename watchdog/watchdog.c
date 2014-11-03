#include <stdio.h>
#include <asm/io.h>
#include <unistd.h>

/* A watchdog program to run at boot time
 */

int main(int argc, char *argv[]) {
  int in,i;
  
  /* Set up IO permissions: 16 words starting at 0x378 */
  if (ioperm(0x378,0x0F, 1)!=0) {
    printf("Error setting permissionss\n");
    exit(1);
  }
  ioperm(0x80,1,1); /* used for timing */


 
  while (1) {
    outb(0xAA,0x378);
    usleep(4000);
    outb(0x55,0x378);
    usleep(4000);
  }

  return 1;
}
