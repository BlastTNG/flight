#include <stdio.h>
#include <asm/io.h>
#include <unistd.h>


/* A watchdog program to run at boot time
 */

int main(int argc, char *argv[]) {
  int in,i;
  
  /* Set up IO permissions: 16 words starting at 0x378 */
  if (ioperm(0x378,0x0F, 1)!=0) {
    printf("Error setting permissions\n");
    exit(1);
  }
  ioperm(0x80,1,1); /* used for timing */


 
  while (1) {
    outb_p(0xFF,0x378);
    usleep(100000);
    outb_p(0x00,0x378);
    usleep (100000);
  }

  return 1;
}
