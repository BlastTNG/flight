/* decompoller: decomd TCP client code
 *
 * This software is copyright (C) 2004-2005 University of Toronto
 * 
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

#include <netdb.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "decompoll.h"

/* Decom Data */
DecomData::DecomData()
{
  status = -1;
  fs_bad = 0;
  dq_bad = 0;
  df = 0;
  frame_counter = 0;
  filename[0] = 0;
}

int DecomData::Status(void)
{
  return status;
}

double DecomData::FrameLoss(void)
{
  return fs_bad * 100.;
}

double DecomData::DataQuality(void)
{
  return 100. * (1. - dq_bad);
}

double DecomData::DiskFree(void)
{
  return (double)df / 1073741824.;
}

unsigned long DecomData::FrameCounter(void)
{
  return frame_counter;
}

char* DecomData::DecomFile(void)
{
  return filename;
}

void DecomData::setData(char* buf)
{
  sscanf(buf, "%i %i %i %lf %lf %Lu %lu %s", &status, &polarity, &decomUnlocks,
      &fs_bad, &dq_bad, &df, &frame_counter, filename);
  filename[12] = 0;
}

/* Polls the decom */
DecomPoll::DecomPoll() : QThread()
{
  connectState = 0;
  theDecom = new DecomData();
  pollDecomd = false;
}

void DecomPoll::start(const char* h, int p)
{
  strncpy(decomdHost, h, MAXPATHLENGTH);
  decomdPort = p;
printf("poller started with host: %s\n", decomdHost);
  QThread::start();
  pollDecomd = true;
}

void DecomPoll::run()
{
  struct hostent *theHost;
  char buf[256];
  int i;
  int sock;
  struct sockaddr_in addr;
  fd_set fdr;

  connectState = 1;

  theHost = gethostbyname(decomdHost);
  // FIXME: better error handling 
  if(!theHost) return;
  printf("tmpHost name: %s tmpHost len: %d\n", theHost->h_name, theHost->h_length);

  connectState = 2;

  for (;;) {
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
      connectState = 4;
      return;
    }

    i = 1;
    if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &i, sizeof(i)) != 0) {
      connectState = 4;
      return;
    }

    addr.sin_addr.s_addr=*((unsigned long*)theHost->h_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(decomdPort);

    if (connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) < 0) {
      connectState = 4;
      sleep(5);
      continue;
    }
    connectState = 5;

    for (;;) {
      usleep(100000);

      if (!pollDecomd) {
        connectState = 0;
        return;
      }

      FD_ZERO(&fdr);
      FD_SET(sock, &fdr);

      i = select(sock + 1, &fdr, NULL, NULL, NULL);

      if (i == -1)
        continue;

      if (FD_ISSET(sock, &fdr)) {
        do {
          i = recv(sock, &buf, 256, MSG_DONTWAIT);
          if (i == -1 && errno != EAGAIN)
            break;
          else if (i == 0) {
            shutdown(sock, SHUT_RDWR);
            connectState = 6;
            close(sock);
            break;
          } else if (i > 0)
            theDecom->setData(buf);
        } while (i > 0);
      }

      if (connectState == 6)
        break;
    }
    sleep(5);
  }
}
