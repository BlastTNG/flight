/* interloquendi: copies a mcp frame file to a TCP port
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of interloquendi.
 * 
 * interloquendi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * interloquendi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with interloquendi; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>

#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <tcpd.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "quendi.h"

#define VERSION   "0.9.0"
#define SOCK_PORT 44144
#define PID_FILE "/var/run/interloquendi.pid"

#define DEBUG

/* for libwrap */
int allow_severity = LOG_INFO;
int deny_severity = LOG_WARNING;

void Connection(int csock) {
  struct sockaddr_in addr;
  int addrlen = sizeof(addr);
  struct hostent* thishost;
  struct request_info req;

  /* tcp wrapper check */
  request_init(&req, RQ_DAEMON, "interloquendi", RQ_FILE, csock, 0);
  fromhost(&req);

  if (!hosts_access(&req)) {
    refuse(&req);
    exit(1); /* can't get here */
  }

  getsockname(csock, (struct sockaddr*)&addr, &addrlen);
  thishost = gethostbyaddr((const char*)&addr.sin_addr, sizeof(addr.sin_addr),
      AF_INET);
  if (thishost == NULL && h_errno) {
    syslog(LOG_WARNING, "gethostbyaddr: %s", hstrerror(h_errno));
    thishost = NULL;
  }

  quendi_server_start(csock, VERSION, "Interloquendi", thishost ?
      thishost->h_name : inet_ntoa(addr.sin_addr));

  exit(0);
}

int MakeSock(void)
{
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    syslog(LOG_CRIT, "socket: %m");
    exit(1);
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
    syslog(LOG_CRIT, "setsockopt: %m");
    exit(1);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    syslog(LOG_CRIT, "bind: %m");
    exit(1);
  }

  if (listen(sock, 10) == -1) {
    syslog(LOG_CRIT, "listen: %m");
    exit(1);
  }

  syslog(LOG_INFO, "listening on port %i.", SOCK_PORT);

  return sock;
}

void CleanUp(void) {
  unlink(PID_FILE);
  //  closelog();
}

int main(void)
{
  int pid;
  FILE* stream;
  int sock, addrlen, csock;
  struct sockaddr_in addr;

  openlog("interloquendi", LOG_PID, LOG_DAEMON);

#ifndef DEBUG
  /* Fork to background */
  if ((pid = fork()) != 0) {
    if (pid == -1) {
      syslog(LOG_CRIT, "unable to fork to background: %m");
      exit(1);
    }

    if ((stream = fopen(PID_FILE, "w")) == NULL)
      syslog(LOG_ERR, "unable to write PID to disk: %m");
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
#endif

  /* initialise listener socket */
  sock = MakeSock();

  /* accept loop */
  for (;;) {
    addrlen = sizeof(addr);
    csock = accept(sock, (struct sockaddr*)&addr, &addrlen);

    if (csock == -1 && errno == EINTR)
      continue;
    else if (csock == -1)
      syslog(LOG_ERR, "accept: %m");
    else {
      /* fork child */
      if ((pid = fork()) == 0) {
        close(sock);
        Connection(csock);
      }

      syslog(LOG_INFO, "spawned %i to handle connect from %s", pid,
          inet_ntoa(addr.sin_addr));
      close(csock);
    }
  }
}
