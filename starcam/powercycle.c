#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/io.h>

int main(void)
{
  if (ioperm(0x378, 0x0F, 1) != 0) {	
    perror("Unable to power cycle");
  }

  printf("================================================\n");
  printf("==       Power Cycling the Camera             ==\n");
  printf("================================================\n");
  outb(0xFF, 0x378);
  usleep(2000000);
  outb(0x00, 0x378);

  return 0;
}
