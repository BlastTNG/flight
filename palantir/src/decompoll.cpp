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

char decomdHost[MAXPATHLENGTH];
int decomdPort;
bool pollDecomd;
int connectState = 0;
class DecomData *theDecom;

/* Decom Data */
DecomData::DecomData()
{
  status = -1;
  fs_bad = 0;
  dq_bad = 0;
  df = 0;
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

char* DecomData::DecomFile(void)
{
  return filename;
}

void DecomData::setData(char* buf) {
  sscanf(buf, "%i %i %i %lf %lf %Lu %s", &status, &polarity, &decomUnlocks,
      &fs_bad, &dq_bad, &df, filename);
}

/* Polls the decom */
void DecomPoll::run()
{
  struct hostent* result;
  struct hostent theHost;
  char buf[256];
  int i;
  int sock;
  struct sockaddr_in addr;
  fd_set fdr;

  connectState = 1;

  gethostbyname_r(decomdHost, &theHost, buf, 256, &result, &i);

  if (!result) {
    fprintf(stderr, "gethostbyname failed: ");
    if (i == HOST_NOT_FOUND)
      fprintf(stderr, "host not found\n");
    else if (i == NO_ADDRESS || i == NO_DATA)
      fprintf(stderr, "host not bound to an IP address\n");
    else if (i == NO_RECOVERY)
      fprintf(stderr, "non-recoverable error in domain resolution\n");
    else if (i ==  TRY_AGAIN)
      fprintf(stderr, "temporary failure in domain resolution\n");
    else
      fprintf(stderr, "unspecified error in gethostbyname_r\n");

    connectState = 3;
    return;
  }
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

    addr.sin_addr.s_addr=*((unsigned long*)theHost.h_addr);
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

