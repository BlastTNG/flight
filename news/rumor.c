#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <time.h>

#define RAWDIR "/data/rawdir"

#define LNKFILE "/data/etc/rumor.lnk"

int tty_fd;
char hostname[255];

void Usage() {
  printf("rumor <hostname>\n  Connects to an lnc server at hostname to\n"
  "download and append to a dirfile.\n");
  exit(0);
}

int main(int argc, char *argv[]) {
  char filedirname[1024];

  if (argc!=2) Usage();
  if (argv[1][0]=='-') Usage();
      
  strncpy(hostname, argv[1], 250);
  
  sprintf(filedirname, "%s/%lu.l", RAWDIR, time(NULL));
  if (mkdir(filedirname, 0777)<0) {
    fprintf(stderr, "could not create dirfile %s\n", filedirname);
    exit(0);
  }

  unlink(LNKFILE);
  if (symlink(filedirname, LNKFILE)<0) {
    fprintf(stderr, "could not create link from `%s' to `%s'",
            filedirname, LNKFILE);
            exit(0);
  }

  strncpy(hostname, argv[1], 250);
  
  return 0;
}
