#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "decom.h"

#define DEV "/dev/decom_pci"
#define SOCK_PORT 11411

#define FB_K 0.99977

void InitialiseFrameFile(char type);
void FrameFileWriter(void);
void pushDiskFrame(unsigned short *RxFrame);

extern struct file_info {
  int fd;            /* current file descriptor */
  int chunk;         /* current chunk number */
  int frames;        /* number of frames writen so far to current chunk */
  char type;         /* run type */
  time_t time;       /* file timestamp */
  char name[200];    /* filename to write to */
  void* buffer;      /* frame buffer */
  void* b_write_to;  /* buffer write-to pointer */
  void* b_read_from; /* buffer read-from pointer */
  void* buffer_end;  /* end of frame buffer */
} framefile;

unsigned short FrameBuf[BI0_MAX_FRAME_SIZE];
int status = 0;
double f_bad = 0;


void ReadDecom (int decom)
{
  unsigned short buf;
  static int i_word = 0;

  while ((read(decom, &buf, sizeof(unsigned short))) == 2) {
    FrameBuf[i_word] = buf;
    if (i_word % BI0_MAX_FRAME_SIZE == 0) { /* begining of frame */
      if (buf != FRAME_SYNC_WORD) {
        status = 0;
        i_word = 0;
      } else {
        if (status < 2)
          status++;
        else
          pushDiskFrame(FrameBuf);

        i_word++;
      }
    } else {
      i_word++;
      if (i_word >= BI0_MAX_FRAME_SIZE)
        i_word = 0;
    }
  }

  printf("done %i\n", i_word);

  if (status == 2)
    f_bad *= FB_K;
  else
    f_bad = f_bad * FB_K + (1.0 - FB_K);
}

int MakeSock(void) {
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    perror("decomd: unable to create tcp socket");
    exit(1);
  }

  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    perror("decomd: unable to set socket options");
    exit(1);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    perror("decomd: unable to bind to socket");
    exit(1);
  }

  if (listen(sock, 10) == -1) {
    perror("decomd: unable to listen on port");
    exit(1);
  }

  printf("decomd: listening on port %i with socket %i.\n", SOCK_PORT, sock);

  return sock;
}

int main(void) {
  int sock, csock;
  int n, z, lastsock;
  fd_set fdlist, fdread, fdwrite;
  struct sockaddr_in addr;
  int addrlen;
  char buf[209];
  int decom;
  struct timeval no_time = {0, 0};
  pthread_t framefile_thread;

  if ((decom = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
    perror("decomd: fatal error opening " DEV "\n");
    exit(1);
  }

  lastsock = sock = MakeSock();

  InitialiseFrameFile('d');

  pthread_create(&framefile_thread, NULL, (void*)&FrameFileWriter, NULL);

  FD_ZERO(&fdlist);
  FD_SET(sock, &fdlist);

  /* main loop */
  for (;;) {
    ReadDecom(decom);
    sprintf(buf, "%1i %4.2f %-200s\n", status, f_bad, framefile.name);

    fdwrite = fdread = fdlist;
    FD_CLR(sock, &fdwrite);
    n = select(lastsock + 1, &fdread, &fdwrite, NULL, &no_time);

    if (n == -1 && errno == EINTR)
      continue;
    else if (n == -1) {
      perror("decomd: error on select");
      exit(1);
    }

    /* loop through all socket numbers, looking for ones that have been
     * returned by select */
    for (n = 0; n <= lastsock; ++n) {
      if (FD_ISSET(n, &fdread)) {     /* connextion n is waiting for read */
        if (n == sock) {
          /* listener has a new connexion */
          addrlen = sizeof(addr);
          if ((csock  = accept(sock, (struct sockaddr*)&addr, &addrlen))
              == -1) {
            perror("decomd: error on accept");
            exit(1);
          }
          FD_SET(csock, &fdlist);
          if (csock > lastsock)
            lastsock = csock;
          printf("decomd: connexion from %s accepted on socket %i\n",
              inet_ntoa(addr.sin_addr), csock);
        } else {
          /* read from socket */
#if 0
          if ((z = recv(n, buf, sizeof(buf), MSG_NOSIGNAL | MSG_DONTWAIT))
              == -1) {
            if (errno == EPIPE) {  /* connexion dropped */
              printf("decomd: connextion dropped on socket %i\n", n);
              close(n);
              FD_CLR(n, &fdlist);
              continue;
            } else if (errno != EAGAIN) {  /* ignore socket buffer overflows */
              perror("decomd: send");
              exit(1);
            }
          } else if (z == 0) { /* connexion closed */
            printf("decomd: close on socket %i\n", n);
            close(n);
            FD_CLR(n, &fdlist);
            continue;
          } else {
            printf("decomd: socket %i is ready (read %i bytes)\n", n, z);
          }
#endif
        }
      }

      if (FD_ISSET(n, &fdwrite)) {     /* connexion n is waiting for write */
        if (n != sock) {   /* don't write to the listener */
          if ((z = send(n, buf, 208, MSG_NOSIGNAL | MSG_DONTWAIT))
              == -1) {
            if (errno == EPIPE) {  /* connexion dropped */
              printf("decomd: connextion dropped on socket %i\n", n);
              close(n);
              FD_CLR(n, &fdlist);
            } else if (errno != EAGAIN) {  /* ignore socket buffer overflows */
              perror("decomd: send");
              exit(1);
            }
          }
        }
      }
    }

    usleep(20000);
  }

  return 1;
}
