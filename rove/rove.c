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

#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define LINKFILE "/data/etc/highgain.lnk"
#define RAWDIR "/data/rawdir"

#define RNC_PORT 41114
#define TIMEOUT 30


#define BUFSIZE 8194

void Usage() {
  fprintf(stderr, "rove: collect data from a remote rnc server, and make it availible for a local rnc server.\n"
                  "   usage: rove <remoteserver1> [<remoteserver1> [...]]\n");
  exit(0);
}

//*********************************************************
// connect to the political party server
//*********************************************************
int party_connect(const char *hostname, int port) {
  int s;
  struct sockaddr_in sn;
  struct hostent *hostinfo;
  struct servent *sp;
  struct servent sp_real;
  int on =1 ;

  sp = &sp_real;
  sp->s_port = htons(port);

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


int main(int argc, char *argv[]) {
  char remote_server[2][256];
  int port[2];
  int n_servers;
  char out_filename[256];
  char inbuf[BUFSIZE];
  
  int tty_fd;
  int out_fd;
  int numin, numread, totalread = 0;
  time_t t, last_t = 0;
  int blockread = 0;
  int i_server = 0;
  int i_ch;

  time_t t_r, t_lr;
  
  t_lr = time(NULL);

  n_servers = argc-1;
  
  if (argc<2) Usage();
  n_servers = argc-1;
  
  for (i_server=0; i_server<n_servers; i_server++) {
    if (argv[i_server+1][0]=='-') Usage();
    for (i_ch = 0; (argv[i_server+1][i_ch] != '\0') && (argv[i_server+1][i_ch] != ':'); i_ch++) {
      remote_server[i_server][i_ch] = argv[i_server+1][i_ch];
    }
    remote_server[i_server][i_ch] = '\0';
    if (argv[i_server+1][i_ch] == ':') {
      port[i_server] = atoi(argv[i_server+1]+i_ch+1);
    } else {
      port[i_server] = RNC_PORT;
    }
    //strncpy(remote_server[i_server], argv[i_server+1], 254);
  }

  for (i_server=0; i_server<n_servers; i_server++) {
    printf("Using host: %s port: %d\n", remote_server[i_server], port[i_server]);
  }

  i_server = 0;
  
  sprintf(out_filename, "%s/%lu.highgain", RAWDIR, time(NULL));
  if( (out_fd = open(out_filename, O_WRONLY | O_CREAT, 00644)) < 0 ) {
    syslog(LOG_ERR | LOG_DAEMON, "Error opening data file\n");
    fprintf(stderr, "Error opening data file %s\n", out_filename);
    exit(0);
  }
  
  unlink(LINKFILE);
  if (symlink(out_filename, LINKFILE)<0) {
    fprintf(stderr, "rove: could not create link from `%s' to `%s'",
            out_filename, LINKFILE);
    exit(0);
  }

  tty_fd = party_connect(remote_server[i_server], port[i_server]);
  
  while (1) {
    ioctl(tty_fd, FIONREAD, &numin);
    if (numin>BUFSIZE-2) numin = BUFSIZE-2;
    
    if (numin>64) {
      numread = read(tty_fd, inbuf, numin);
      write(out_fd, inbuf, numread);
      totalread += numread;
      blockread += numread;
      t_lr = time(NULL);
    } else {
      t_r = time(NULL);
      if ((t_r -t_lr) > TIMEOUT) {
        printf("No data for %us.  Resetting connection.\n", t_r-t_lr);
        t_lr = t_r;
        shutdown(tty_fd, SHUT_RDWR);
        i_server++;
        if (i_server>=n_servers) i_server = 0;
        tty_fd = party_connect(remote_server[i_server], port[i_server]);
      } 
      usleep(30000);
    }
    t = time(NULL);
    
    if (t-last_t > 10) {
      printf("Host %s:%d %7d bytes at %.0f bps at %s", remote_server[i_server],port[i_server], totalread, (double)blockread*8.0/(t-last_t), ctime(&t));
      last_t = t;
      blockread = 0;
    }
  }
  
  exit(0);
}
