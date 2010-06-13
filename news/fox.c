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

int cli_conn(const char *hostname)
{
  int s;
  struct sockaddr_in sn;
  struct hostent *hostinfo;
  struct servent *sp;
  struct servent sp_real;
  int on =1 ;
  /* Get service */
  if ( (sp = getservbyname("blastd", NULL)) == NULL ) {
    fprintf(stderr,
        "Service blastd not found; using 44144 (default) instead\n");
    sp = &sp_real;
    sp->s_port = htons(44144);
  }

  sn.sin_family = AF_INET;
  sn.sin_port = sp->s_port;

  if(!inet_aton(hostname, &sn.sin_addr)) {
    hostinfo = gethostbyname(hostname);
    if (hostinfo == NULL) {
      herror(hostname);
      exit(1);
    }
    sn.sin_addr = *(struct in_addr *) hostinfo->h_addr;
  }

  /* Create the socket. */
  if ( (s = socket (AF_INET, SOCK_STREAM, 0)) < 0 ) {
    perror("socket");
    exit(1);
  }
  /* set socket options */
  (void) setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  if (connect(s, (struct sockaddr *)&sn, sizeof (sn)) < 0) {
    perror("socket");
  }

  return s;
}

void Usage() {
  fprintf(stderr,"fox <hostname>\n"
    "File Output eXtractor: \n"
    "Connects to an rnc server and downloads and\n"
    "produce defiles from the high gain tdrss link.\n");
    exit(0);
}

void main(int argc, char *argv[]) {
  int tty_fd;
  
  if (argc!=2) Usage();
  if (argv[1][0]=='-') Usage();
  
  tty_fd = cli_conn(argv[1]);

  /*
  while (1) {
    ioctl(tty_fd, FIONREAD, &numin);
    if (numin) {
      insize += read(tty_fd, indata+insize, numin);
    } else {
      usleep(10000);
    }
    */
  
  /*
sleeptime.tv_sec = 0;
  sleeptime.tv_nsec = SLEEPTIME;

      */
}
