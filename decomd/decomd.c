#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "decom_pci.h"
#include "bbc_pci.h"
#include "tx_struct.h"
#include "crc.h"

#define DEV "/dev/decom_pci"
#define SOCK_PORT 11411

#define FS_FILTER 0.999977
#define DQ_FILTER 0.9977

void InitialiseFrameFile(char type);
void FrameFileWriter(void);
void pushDiskFrame(unsigned short *RxFrame);
int decom;

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

#define FRAME_SYNC_WORD 0xEB90

unsigned short FrameBuf[BI0_FRAME_SIZE];
unsigned short AntiFrameBuf[BI0_FRAME_SIZE];
int status = 0;
double fs_bad = 0;
double dq_bad = 0;
unsigned short polarity = 1;
int du = 0;

void ReadDecom (void)
{
  unsigned short crc_ok = 1;
  unsigned short buf;
  int i_word = 0;
  int read_data = 0;

  for (;;) {
    while ((read(decom, &buf, sizeof(unsigned short))) > 0) {
      read_data = 1;
      FrameBuf[i_word] = buf;
      AntiFrameBuf[i_word] = ~buf;
      if (i_word % BI0_FRAME_SIZE == 0) { /* begining of frame */
        du = ioctl(decom, DECOM_IOC_NUM_UNLOCKED);
        if ((buf != FRAME_SYNC_WORD) && ((~buf & 0xffff) != FRAME_SYNC_WORD)) {
          status = 0;
          i_word = 0;
          fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
        } else {
          if (status < 2)
            status++;
          else 
            if (polarity) {
              FrameBuf[BiPhaseFrameWords] = crc_ok;
              FrameBuf[BiPhaseFrameWords + 1] = polarity;
              FrameBuf[BiPhaseFrameWords + 2] = du;
              pushDiskFrame(FrameBuf);
              fs_bad *= FS_FILTER;
            } else {
              AntiFrameBuf[BiPhaseFrameWords] = crc_ok;
              AntiFrameBuf[BiPhaseFrameWords + 1] = polarity;
              AntiFrameBuf[BiPhaseFrameWords + 2] = du;
              pushDiskFrame(AntiFrameBuf);
              fs_bad *= FS_FILTER;
            }
          if (crc_ok)
            dq_bad *= DQ_FILTER;
          else
            dq_bad = dq_bad * DQ_FILTER + (1.0 - DQ_FILTER);

          i_word++;
        }
      } else {
        if (++i_word >= BI0_FRAME_SIZE)
          i_word = 0;

        if (i_word - 1 == BiPhaseFrameWords) {
          FrameBuf[0] = AntiFrameBuf[0] = 0xEB90;

          if (buf == CalculateCRC(CRC_SEED, FrameBuf, BiPhaseFrameWords)) {
            crc_ok = 1;
            polarity = 1;
          } else if ((unsigned short)~buf == CalculateCRC(CRC_SEED,
                AntiFrameBuf, BiPhaseFrameWords)) {
            polarity = 0;
            crc_ok = 1;
          } else
            crc_ok = 0;
        }
      }
    }

    if (!read_data) {
      fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
      fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
      fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
      fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
    } else
      read_data = 0;

    usleep(1000);
  }
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
  struct timeval no_time = {0, 0};
  pthread_t framefile_thread;
  pthread_t decom_thread;

  if ((decom = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
    perror("decomd: fatal error opening " DEV "\n");
    exit(1);
  }

  MakeAddressLookups();

  printf("Reseting board . . . \n");
  ioctl(decom, DECOM_IOC_RESET);
  printf("Set frame length to %d.\n",
      ioctl(decom, DECOM_IOC_FRAMELEN, BI0_FRAME_SIZE));

  lastsock = sock = MakeSock();

  InitialiseFrameFile('y');

  pthread_create(&framefile_thread, NULL, (void*)&FrameFileWriter, NULL);
  pthread_create(&decom_thread, NULL, (void*)&ReadDecom, NULL);

  FD_ZERO(&fdlist);
  FD_SET(sock, &fdlist);

  /* main loop */
  for (;;) {
    fdwrite = fdread = fdlist;
    FD_CLR(sock, &fdwrite);
    n = select(lastsock + 1, &fdread, &fdwrite, NULL, &no_time);
    sprintf(buf, "%1i %1i %3i %4.2f %4.2f %-200s\r\n", status, polarity, du,
        fs_bad, dq_bad, framefile.name);

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

    usleep(1000000);
  }

  return 1;
}
