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

#include "news.h"

#define TIMEOUT 100

//*********************************************************
// "out" needs to be allocated before we come here.
//*********************************************************
void convertToUpper(char *in, char *out) {
  int i;
  for (i=0; in[i] != '\0'; i++) {
    out[i] = toupper(in[i]);
  }
  out[i] = '\0';
}
 
//*********************************************************
// "out" needs to be allocated before we come here.
//*********************************************************
void convertToLower(char *in, char *out) {
  int i;
  for (i=0; in[i] != '\0'; i++) {
    out[i] = tolower(in[i]);
  }
  out[i] = '\0';
}

//*********************************************************
// insert data into the fifo
//*********************************************************
void push(struct fifoStruct *fs, char x[], int size) {
  int i;
  for (i=0; i<size; i++) {
    fs->d[fs->i_in] = x[i];
    fs->i_in++;
    if (fs->i_in>=FIFODEPTH) {
      fs->i_in = 0;
    }
  }
}

//*********************************************************
// return data w/out removing it
//*********************************************************
void peek(struct fifoStruct *fs, char x[], int size) {
  // warning: no error checking.  Use nFifo first to make
  // sure you don't wrap the fifo.
  int i;
  int i_out = fs->i_out;
  
  for (i=0; i< size; i++) {
    x[i] = fs->d[i_out];
    i_out++;
    if (i_out >= FIFODEPTH) {
      i_out = 0;
    }
  }
}

//*********************************************************
// advance the fifo pointer (removes data)
//*********************************************************
void advance(struct fifoStruct *fs, int size) {
  fs->i_out += size;
  if (fs->i_out >= FIFODEPTH) {
    fs->i_out -= FIFODEPTH;
  }
}

//*********************************************************
// remove data from the fifo
//*********************************************************
void pop(struct fifoStruct *fs, char x[], int size) {
  peek(fs, x, size);
  advance(fs,size);
}

//*********************************************************
// how many bytes are availible in the fifo
//*********************************************************
int nFifo(struct fifoStruct *fs) {
  int n;

  n = fs->i_in - fs->i_out;
  if (n < 0) n+= FIFODEPTH;

  return n;
}


//*********************************************************
// connect to the political party server
//*********************************************************
int party_connect(char *hostname, int port) {
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


//*********************************************************
// read data, leaving at least minRead in the fifo, but without overloading the Fifo
//*********************************************************
int BlockingRead(int minRead, struct fifoStruct *fs, int tty_fd, char *hostname, int port) {
  int numin;
  char inbuf[FIFODEPTH];
  int numread=0;
  
  time_t t_r, t_lr;
  
  t_lr = time(NULL);
  do {
    ioctl(tty_fd, FIONREAD, &numin);

    // Read data from the port into the FIFO
    // don't over-fill the fifo.
    if (numin>=FIFODEPTH - nFifo(fs) - 2) {
      numin = FIFODEPTH - nFifo(fs) - 2;
    }
    if (numin) {
      if (numin>0) {
        numread = read(tty_fd, inbuf, numin);
        push(fs, inbuf, numread);
      }
      t_lr = time(NULL);
    } else {
      t_r = time(NULL);
      if ((t_r -t_lr) > TIMEOUT) {
        printf("No data for %us.  Resetting connection.\n", (unsigned int)(t_r-t_lr));
        t_lr = t_r;
        shutdown(tty_fd, SHUT_RDWR);
        tty_fd = party_connect(hostname, port);
      } 
      usleep(10000);
    }
  } while (nFifo(fs)<minRead);
  
  return (numread);
}

