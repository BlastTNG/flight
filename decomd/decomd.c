/* decomd: reads the decom stream, performs integrity checking, and writes it
 * to disk
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of decomd.
 * 
 * decomd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * decomd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with decomd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <pthread.h>

#include "decom_pci.h"
#include "bbc_pci.h"
#include "channels.h"
#include "crc.h"

#define DEV "/dev/decom_pci"
#define SOCK_PORT 11411
#define FILE_SUFFIX 'y'

#define FS_FILTER 0.999977
#define DQ_FILTER 0.9977

void InitialiseFrameFile(char type);
void FrameFileWriter(void);
void ShutdownFrameFile(void);
void pushDiskFrame(unsigned short *RxFrame);
int decom;
int system_idled = 0;

sigset_t signals;
pthread_t framefile_thread;
pthread_t decom_thread;

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

inline void syserror(int priority, char* preamble) {
  syslog(priority, "%s: %s", preamble, strerror(errno));
}

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
          fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
          status = 0;
          i_word = 0;
        } else {
          if (status < 2)
            status++;
          else {
            fs_bad *= FS_FILTER;
            if (polarity) {
              FrameBuf[BiPhaseFrameWords] = crc_ok;
              FrameBuf[BiPhaseFrameWords + 1] = polarity;
              FrameBuf[BiPhaseFrameWords + 2] = du;
              if (!system_idled)
                pushDiskFrame(FrameBuf);
            } else {
              AntiFrameBuf[BiPhaseFrameWords] = crc_ok;
              AntiFrameBuf[BiPhaseFrameWords + 1] = polarity;
              AntiFrameBuf[BiPhaseFrameWords + 2] = du;
              if (!system_idled)
                pushDiskFrame(AntiFrameBuf);
            }
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

        if (i_word % 5 == 0)
          fs_bad *= FS_FILTER;

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
      status = 0;
      fs_bad = fs_bad * FS_FILTER + (1.0 - FS_FILTER);
    } else
      read_data = 0;

    usleep(1000);
  }
}

int MakeSock(void)
{
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    syserror(LOG_CRIT, "socket");
    exit(1);
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
    syserror(LOG_CRIT, "setsockopt");
    exit(1);
  }

  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    syserror(LOG_CRIT, "setsockopt");
    exit(1);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    syserror(LOG_CRIT, "bind");
    exit(1);
  }

  if (listen(sock, 10) == -1) {
    syserror(LOG_CRIT, "listen");
    exit(1);
  }

  syslog(LOG_INFO, "listening on port %i with socket %i.\n", SOCK_PORT, sock);

  return sock;
}

void CleanUp(void) {
  unlink("/var/run/decomd.pid");
  closelog();
}

void SigAction(int signo) {
  if (system_idled) {
    syslog(LOG_INFO, "caught signal %i while system idle", signo);
  } else {
    syslog(LOG_INFO, "caught signal %i; going down to idle", signo);

    ShutdownFrameFile();
    system_idled = 1;
    pthread_join(framefile_thread, NULL);
  }

  if (signo == SIGINT) { /* idle */
    return;
  } else if (signo == SIGHUP) { /* cycle */
    syslog(LOG_INFO, "system idle, bringing system back up");

    InitialiseFrameFile(FILE_SUFFIX);

    /* block signals */
    pthread_sigmask(SIG_BLOCK, &signals, NULL);

    pthread_create(&framefile_thread, NULL, (void*)&FrameFileWriter, NULL);

    /* unblock signals */
    pthread_sigmask(SIG_UNBLOCK, &signals, NULL);

    system_idled = 0;
  } else { /* terminate */
    syslog(LOG_INFO, "system idle, terminating");
    CleanUp();
    signal(signo, SIG_DFL);
    raise(signo);
  }
}

int main(void) {
  int pid;
  FILE* stream;
  int sock, csock;
  int n, z, lastsock;
  fd_set fdlist, fdread, fdwrite;
  struct sockaddr_in addr;
  int addrlen;
  struct statvfs vfsbuf;
  char buf[209];
  struct timeval no_time = {0, 0};
  unsigned long long int disk_free = 0;
  int i, reset_lastsock = 0;

  struct sigaction action;

  openlog("decomd", LOG_PID, LOG_DAEMON);

  /* Fork to background */
  if ((pid = fork()) != 0) {
    if (pid == -1) {
      syserror(LOG_CRIT, "unable to fork to background");
      exit(1);
    }

    if ((stream = fopen("/var/run/decomd.pid", "w")) == NULL) 
      syserror(LOG_ERR, "unable to write PID to disk");
    else {
      fprintf(stream, "%i\n", pid);
      fflush(stream);
      fclose(stream);
    }
    closelog();
    printf("PID = %i\n", pid);
    exit(0);
  }
  atexit(CleanUp);

  /* Daemonise */
  chdir("/");
  freopen("/dev/null", "r", stdin);
  freopen("/dev/null", "w", stdout);
  freopen("/dev/null", "w", stderr);
  setsid();

  /* Open Decom */
  if ((decom = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
    syserror(LOG_CRIT, "fatal error opening " DEV);
    exit(1);
  }

  /* Initialise Channel Lists */
  MakeAddressLookups();

  /* Initialise Decom */
  ioctl(decom, DECOM_IOC_RESET);
  ioctl(decom, DECOM_IOC_FRAMELEN, BI0_FRAME_SIZE);

  /* Initialise Frame File Writer */
  InitialiseFrameFile(FILE_SUFFIX);

  /* set up signal masks */
  sigemptyset(&signals);
  sigaddset(&signals, SIGHUP);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);

  /* set up signal handlers */
  action.sa_handler = SigAction;
  action.sa_mask = signals;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGHUP, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  /* block signals */
  pthread_sigmask(SIG_BLOCK, &signals, NULL);

  pthread_create(&framefile_thread, NULL, (void*)&FrameFileWriter, NULL);
  pthread_create(&decom_thread, NULL, (void*)&ReadDecom, NULL);

  /* unblock signals */
  pthread_sigmask(SIG_UNBLOCK, &signals, NULL);

  /* initialise network sockets */
  lastsock = sock = MakeSock();
  FD_ZERO(&fdlist);
  FD_SET(sock, &fdlist);

  /* main loop */
  for (;;) {
    fdwrite = fdread = fdlist;
    FD_CLR(sock, &fdwrite);
    if (reset_lastsock) {
      reset_lastsock = 0;
      for (i = 0; i < sizeof(fd_set) * 8; ++i)
        if (__FDS_BITS(&fdlist)[__FDELT(i)] & __FDMASK(i))
          lastsock = i;
    }
    n = select(lastsock + 1, &fdread, &fdwrite, NULL, &no_time);

    if (statvfs("/data", &vfsbuf))
      syserror(LOG_ERR, "statvfs");
    else
      disk_free = (unsigned long long int)vfsbuf.f_bavail * vfsbuf.f_bsize;

    memset(buf, 0, 209);
    sprintf(buf, "%1i %1i %3i %5.3f %5.3f %Lu ", status + system_idled
        * 0x4, polarity, du, fs_bad, dq_bad, disk_free);
    strcat(buf, framefile.name);

    if (n == -1 && errno == EINTR)
      continue;
    else if (n == -1)
      syserror(LOG_ERR, "error on select");
    else

      /* loop through all socket numbers, looking for ones that have been
       * returned by select */
      for (n = 0; n <= lastsock; ++n) {
        if (FD_ISSET(n, &fdread))       /* connextion n is waiting for read */
          if (n == sock) {              /* only read from the listener */
            /* listener has a new connexion */
            addrlen = sizeof(addr);
            if ((csock = accept(sock, (struct sockaddr*)&addr, &addrlen)) == -1)
              syserror(LOG_ERR, "accept");
            FD_SET(csock, &fdlist);
            if (csock > lastsock)
              lastsock = csock;
            syslog(LOG_INFO, "connect from %s accepted on socket %i\n",
                inet_ntoa(addr.sin_addr), csock);
          }

        if (FD_ISSET(n, &fdwrite))     /* connexion n is waiting for write */
          if (n != sock)               /* don't write to the listener */
            if ((z = send(n, buf, 1 + strlen(buf), MSG_NOSIGNAL | MSG_DONTWAIT))
                == -1) {
              if (errno == EPIPE) {  /* connexion dropped */
                syslog(LOG_INFO, "connexion dropped on socket %i\n", n);
                shutdown(n, SHUT_RDWR);
                close(n);
                FD_CLR(n, &fdlist);
                reset_lastsock = 1;
              } else if (errno != EAGAIN)  /* ignore socket buffer overflows */
                syserror(LOG_ERR, "send");
            }
      }
    usleep(1000000);
  }

  return 1;
}
