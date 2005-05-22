/* daemon: groundstation BLAST command daemon
 *
 * This software is copyright (C) 2005 University of Toronto
 * 
 * This file is part of blastcmd.
 * 
 * blastcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * blastcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <fcntl.h>
#include <stdlib.h>

#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <tcpd.h>
#include <time.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>
#include <sys/time.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "netcmd.h"
#include "daemon.h"
#include "command_list.h"

int SIPRoute(int sock, int fd, int t_link, int t_route, char* buffer)
{
  int i_cmd, i_ack;
  int count = 0;
  char* token[1024];
  char* ptr = buffer;

  token[count++] = buffer;

  for (ptr = buffer + 1; *ptr != '\0'; ++ptr) {
    if (*ptr == ' ')
      *ptr = 0;
    else if (*(ptr - 1) == '\0')
      token[count++] = ptr;
  }
  
  /* Look for single packet commands */
  for (i_cmd = 0; i_cmd < N_SCOMMANDS; i_cmd++) {
    if (strncmp(token[0], scommands[i_cmd].name, SIZE_NAME) == 0) {
      SendScommand(sock, i_cmd, t_link, t_route, &i_ack);
      token[0] -= 3;

      if (i_ack == 0x0a)
        i_ack = 4;
      else if (i_ack == 0x0b)
        i_ack = 5;
      else if (i_ack == 0x0c)
        i_ack = 6;
      else if (i_ack == 0x0d)
        i_ack = 7;
      else if (i_ack == 0x0e)
        i_ack = 8;
      else if (i_ack == 0x0f)
        i_ack = 9;
      else if (i_ack == 0x10)
        i_ack = 13;
      else if (i_ack == 0x112)
        i_ack = 12;
      else
        i_ack = 0;

      WriteLogFile(count, token, i_ack);

      return i_ack;
    }
  }

  /* Look for multi packet commands */
  for (i_cmd = 0; i_cmd < N_MCOMMANDS; i_cmd++) {
    if (strncmp(token[0], mcommands[i_cmd].name, SIZE_NAME) == 0) {
      SendMcommand(sock, i_cmd, t_link, t_route, token + 1 , count - 1, &i_ack);
      token[0] -= 3;

      if (i_ack == 0x0a)
        i_ack = 4;
      else if (i_ack == 0x0b)
        i_ack =  5;
      else if (i_ack == 0x0c)
        i_ack =  6;
      else if (i_ack == 0x0d)
        i_ack =  7;
      else if (i_ack == 0x0e)
        i_ack =  8;
      else if (i_ack == 0x0f)
        i_ack =  9;
      else if (i_ack == 0x10)
        i_ack =  13;
      else if (i_ack == 0x102)
        i_ack =  2;
      else if (i_ack == 0x103)
        i_ack =  3;
      else if (i_ack == 0x110)
        i_ack =  10;
      else if (i_ack == 0x111)
        i_ack =  11;
      else if (i_ack == 0x112)
        i_ack =  12;
      else
        i_ack = 0;

      WriteLogFile(count, token, i_ack);

      return i_ack;
    }
  }

  printf("Unknown command (%s) from socket %i\n", token[0], sock);
  return 1;
}

int SimpleRoute(int sock, int fd, char* buffer)
{
  write(fd, buffer, strlen(buffer));
  return 0;
}

void ExecuteCommand(int sock, int fd, int route, char* buffer)
{
  int t_link = LINK_DEFAULT;
  int t_route = ROUTING_DEFAULT;
  char output[100];

  int result = 0;

  printf("EXE %s\n", buffer);

  switch (buffer[0]) {
    case 'L':
      t_link = 0x00;
      break;
    case 'T':
      t_link = 0x01;
      break;
    case 'I':
      t_link = 0x02;
      break;
    default:
      result = 11;
  }

  switch (buffer[1]) {
    case '1':
      t_route = 0x09;
      break;
    case '2':
      t_route = 0x0C;
      break;
    default:
      result = 11;
  }

  if (result == 0) {
    if (route)
      result = SimpleRoute(sock, fd, &buffer[3]);
    else
      result = SIPRoute(sock, fd, t_link, t_route, &buffer[3]);
  }

  sprintf(output, ":::ack:::%i\r\n", result);
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);
}

void SendCommandList(int sock)
{
  unsigned short i;
  char output[1024];

  sprintf(output, ":::rev:::%s\r\n", command_list_serial);
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);

  i = N_SCOMMANDS;
  if (send(sock, &i, sizeof(i), MSG_NOSIGNAL) < 1)
    return;
  i = N_MCOMMANDS;
  if (send(sock, &i, sizeof(i), MSG_NOSIGNAL) < 1)
    return;
  if (send(sock, &scommands, sizeof(struct scom) * N_SCOMMANDS, MSG_NOSIGNAL)
      < 1)
    return;
  send(sock, &mcommands, sizeof(struct mcom) * N_MCOMMANDS, MSG_NOSIGNAL);
}

int MakeSock(void)
{
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    perror("Unable to create socket");
    exit(15);
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
    perror("Unable to set socket options");
    exit(15);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    perror("Unable to bind to port");
    exit(15);
  }

  if (listen(sock, 100) == -1) {
    perror("Unable to listen on port");
    exit(15);
  }

  printf("Listening on port %i.\n", SOCK_PORT);

  return sock;
}

void Daemonise(int route, int no_fork)
{
  struct {
    int state;
    int report;
    char user[1024];
  } conn[1024];

  int owner = 0;

  char buffer[1025];

  int fd, n, i, size, pid, reset_lastsock = 0;
  int report = 0;
  struct sockaddr_in addr;
  int addrlen, sock, csock, lastsock;
  fd_set fdlist, fdread, fdwrite;

  /* open our output before daemonising just in case it fails. */
  if (route == 1) /* fifo */
    fd = open("/tmp/SIPSS.FIFO", O_RDONLY);
  else if (route == 2) /* null */
    fd = open("/dev/null", O_RDONLY);
  else
    fd = bc_setserial();

  if (fd < 0) {
    perror("Unable to open output device");
    exit(2);
  }

  /* start the listener. */
  lastsock = sock = MakeSock();

  if (!no_fork) {
    /* Fork to background */
    if ((pid = fork()) != 0) {
      if (pid == -1) {
        perror("Unable to fork to background");
        exit(-1);
      }

      printf("PID = %i\n", pid);
      exit(0);
    }

    /* Daemonise */
    chdir("/");
    freopen("/dev/null", "r", stdin);
    freopen("/dev/null", "w", stdout);
    freopen("/dev/null", "w", stderr);
    setsid();
  }

  /* select */
  FD_ZERO(&fdlist);
  FD_SET(sock, &fdlist);

  for (;;) {
    fdwrite = fdread = fdlist;
    FD_CLR(sock, &fdwrite);

    if (reset_lastsock) {
      reset_lastsock = 0;
      for (i = 0; i < sizeof(fd_set) * 8; ++i)
        if (__FDS_BITS(&fdlist)[__FDELT(i)] & __FDMASK(i))
          lastsock = i;
    }

    if (report) {
      reset_lastsock = 0;
      for (i = 0; i < sizeof(fd_set) * 8; ++i)
        if (i != sock && (__FDS_BITS(&fdlist)[__FDELT(i)] & __FDMASK(i)))
          conn[i].report = 1;

      report = 0;
    }

    n = select(lastsock + 1, &fdread, &fdwrite, NULL, NULL);

    if (n == -1 && errno == EINTR)
      continue; /* timeout on select */
    else if (n == -1) 
      perror("select");
    else

      /* loop through all sockets, looking for ones that have been returned by
       * select */
      for (n = 0; n <= lastsock; ++n) {
        if (FD_ISSET(n, &fdread)) { /* socket n is waiting for read */
          if (n == sock) { /* read from the listener */
            addrlen = sizeof(addr);
            if ((csock = accept(sock, (struct sockaddr*)&addr, &addrlen)) == -1)
              perror("accept");
            else {
              FD_SET(csock, &fdlist);
              if (csock > lastsock)
                lastsock = csock;
              printf("connect from %s accepted on socket %i\n",
                  inet_ntoa(addr.sin_addr), csock);
              conn[csock].state = 0;
            }
          } else { /* read from client */
            if ((size = recv(n, &buffer, 1024, 0)) == -1)
              perror("recv");
            else if (size == 0) { /* connection closed */
              printf("connexion dropped on socket %i\n", n);
              shutdown(n, SHUT_RDWR);
              close(n);
              FD_CLR(n, &fdlist);
              FD_CLR(n, &fdwrite);
              reset_lastsock = 1;
              if (owner == n) {
                printf("Conn dropped!\n");
                report = 1;
                owner = 0;
              }
              continue;
            } else {
              buffer[1023] = 0;
              for (i = 0; i < 1023; ++i)
                if (buffer[i] == '\n' || buffer[i] == '\r')
                  buffer[i] = 0;
            }

            printf("%i-->%s\n", n, buffer);

            if (conn[n].state == 0) { /* authentication */
              if (strncmp(buffer, "::user::", 8) == 0) {
                conn[n].state = 1;
                strcpy(conn[n].user, &buffer[8]);
                printf("Authentication on socket %i by %s.\n", n, conn[n].user);
              } else { /* failed authentication */
                printf("Failed authentication on socket %i.  Closing "
                    "connection.\n", n);
                shutdown(n, SHUT_RDWR);
                close(n);
                FD_CLR(n, &fdlist);
                reset_lastsock = 1;
                if (owner == n) {
                  printf("Conn dropped!\n");
                  report = 1;
                  owner = 0;
                }
              }
            } else if (owner != n) { /* no conn */
              if (strncmp(buffer, "::take::", 8) == 0) {
                printf("Socket %i has taken the conn.\n", n);
                report = 1;
                owner = n;
                conn[n].state = 2;
              } else if (strncmp(buffer, "::list::", 8) == 0)
                conn[n].state = 4;
              else
                conn[n].state = 5;
            } else if (owner == n) { /* has conn */
              if (strncmp(buffer, "::give::", 8) == 0) {
                printf("Socket %i has given up the conn.\n", n);
                report = 1;
                owner = 0;
                conn[n].state = 2;
              } else if (strncmp(buffer, "::list::", 8) == 0)
                conn[n].state = 4;
              else if (buffer[0] != ':')
                ExecuteCommand(n, fd, route, buffer);
            }
          }
        } /* read */

        if (FD_ISSET(n, &fdwrite))    /* connection n available for write */
          if (n != sock) {            /* don't write to the listener */
            buffer[0] = 0;

            /* compose message */
            if (conn[n].state == 1) { /* need to know who has the conn */
              if (owner == 0)
                strcpy(buffer, ":::free:::\r\n");
              else {
                strcpy(buffer, ":::conn:::");
                strcat(buffer, conn[owner].user);
                strcat(buffer, "\r\n");
              }
              conn[n].state = 2;
              conn[n].report = 0;
            } else if (conn[n].state == 4) { /* list */
              SendCommandList(n);
              conn[n].state = 2;
            } else if (conn[n].state == 5) { /* request with no conn */
              strcpy(buffer, ":::noconn:::\r\n");
              conn[n].state = 1;
            }

            /* send */
            if (buffer[0]) {
              printf("%i<--%s", n, buffer);
              if ((size = send(n, buffer, 1 + strlen(buffer), MSG_NOSIGNAL))
                  == -1) {
                if (errno == EPIPE) {
                  printf("connexion dropped on socket %i\n", n);
                  shutdown(n, SHUT_RDWR);
                  close(n);
                  FD_CLR(n, &fdlist);
                  reset_lastsock = 1;
                  if (owner == n) {
                    printf("Conn dropped!\n");
                    report = 1;
                    owner = 0;
                  }
                } else if (errno != EAGAIN) /* ignore socket buffer overflows */
                  perror("send");
              }
            }

            if (conn[n].state == 2 && conn[n].report) {
              conn[n].report = 0;
              conn[n].state = 1;
            }
          } /* n != sock */
      } /* socket loop */

    usleep(50);
  } /* main loop */
}
