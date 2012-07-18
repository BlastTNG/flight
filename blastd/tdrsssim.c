#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define OUTPUT_TTY "/dev/ttyS5"
#define DAEMON "tdrsssim"
unsigned short simbuf[1024];

//-------------------------------------------------------------
//
// OpenSerial: open serial port
//
//-------------------------------------------------------------

int OpenSerial(char* device) {
  int fd;
  struct termios term;

  if ((fd = open(device, O_RDWR)) < 0) {
    printf("Unable to open serial port.\n");
    exit(1);
  }
  if (tcgetattr(fd, &term)) {
    printf("Unable to get serial device attributes.");
    return -1;
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if (cfsetospeed(&term, B19200)) {       // Baud
    printf("Error setting serial output speed.");
    return -1;
  }
  if (cfsetispeed(&term, B19200)) {       // Baud
    printf("Error setting serial output speed.");
    return -1;
  }
  if(tcsetattr(fd, TCSANOW, &term)) {
    printf("Unable to set serial attributes.");
    return -1;
  }

  printf("%s: %s : %d open\n", DAEMON, device, fd);
  return fd;
}


int main(void)
{
  int idx;
  int fd;
  int written;

  if ( (fd = OpenSerial(OUTPUT_TTY)) < 0 ) return 0;  
  
  simbuf[0] = 0xeb90;
  for(idx = 2; idx < 1024; idx++) {
    simbuf[idx] = idx-2;
  }

  for(idx = 0;;) {
    simbuf[1] = idx++;
    written = write(fd, simbuf, 1024*sizeof(short)); 
    printf("writing data %d - written %d\n", idx, written);
    sleep(3);
  }

  return 0;
}
