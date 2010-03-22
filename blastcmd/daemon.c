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
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "netcmd.h"
#include "daemon.h"
#include "command_list.h"

#ifdef USE_AUTHENTICATION
# define GOOD_ADDR1 "157.132.95.145"
# define GOOD_ADDR2 "192.168.20.10"
# define GOOD_ADDR3 "24.219.66.100"
# define GOOD_ADDR4 "128.148.60.67"
#endif

int SIPRoute(int sock, int t_link, int t_route, char* buffer)
{
  int i_cmd;
  unsigned int i_ack;
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
  write(fd, "\n", 1);
  return 0;
}

int ExecuteCommand(int sock, int fd, int route, char* buffer)
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
      result = SIPRoute(sock, t_link, t_route, &buffer[3]);
  }

  sprintf(output, ":::ack:::%i\r\n", result);
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);
  return result;
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
    int lurk; /* 0 = off, 1 = on, 2 = request, 3 = cmd_announce */
    int spy;  /* 0 = off, 1 = on, 2 = request, 3 = client_announce */
    char user[1024];
  } conn[1024];
  /* state list
   *  0 = client just connected
   *  1 = client user known
   *  2 = client has taken/given/knows conn
   *  4 = client requested command list
   *  5 = client request denied
   *  8 = client unauthorized
   * 0x100 bit set = ping
   */ 

  int owner = 0;

  char *buffer;
  char tmp_buf[512];
  char cmd[512];
  int cmd_from = 0;
  int ack = 0;

  int fd, n, i, size, pid, reset_lastsock = 0;
  int report = 0;
  struct sockaddr_in addr;
  int sock, csock, lastsock;
  socklen_t addrlen;
  fd_set fdlist, fdread, fdwrite;

  struct timespec sleep_time;
  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 10000000; /* 10ms */

#ifdef USE_AUTHENTICATION
  /* the addresses we allow connections from */
  struct in_addr good_addr1;
  struct in_addr good_addr2;
  struct in_addr good_addr3;
  struct in_addr good_addr4;
  struct in_addr local_addr;
  inet_aton(GOOD_ADDR1, &good_addr1);
  inet_aton(GOOD_ADDR2, &good_addr2);
  inet_aton(GOOD_ADDR3, &good_addr3);
  inet_aton(GOOD_ADDR4, &good_addr4);
  inet_aton("127.0.0.1", &local_addr);
#endif

  /* open our output before daemonising just in case it fails. */
  if (route == 1) /* fifo */
    fd = open("/tmp/SIPSS.FIFO", O_RDWR); 
  else if (route == 2) /* null */
    fd = open("/dev/null", O_WRONLY);
  else
    fd = bc_setserial();

  if (fd < 0) {
    perror("Unable to open output device");
    exit(2);
  }

  buffer = malloc(30 * 1024 * sizeof(char));
  if (buffer == NULL) {
    perror("Unable to malloc send buffer");
    exit(3);
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

  /* Zero everything */
  for (i = 0; i < 1024; i++)
  {
    conn[i].state = conn[i].report = conn[i].lurk = conn[i].spy = 0;
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

    n = pselect(lastsock + 1, &fdread, &fdwrite, NULL, &sleep_time, NULL);

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
#ifdef USE_AUTHENTICATION
              if (!memcmp(&addr.sin_addr, &good_addr1, sizeof(struct in_addr)))
              {
                printf("Autentication 1 OK from client.\n");
                conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
              } else if (!memcmp(&addr.sin_addr, &good_addr2,
                    sizeof(struct in_addr)))
              {
                printf("Autentication 2 OK from client.\n");
                conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
              } else if (!memcmp(&addr.sin_addr, &good_addr3,
                    sizeof(struct in_addr)))
              {
                printf("Autentication 3 OK from client.\n");
                conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
              } else if (!memcmp(&addr.sin_addr, &good_addr4,
                    sizeof(struct in_addr)))
              {
                printf("Autentication 4 OK from client.\n");
                conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
              } else if (!memcmp(&addr.sin_addr, &local_addr, sizeof(struct
                      in_addr))) {
                printf("Autentication OK from local client.\n");
                conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
              } else {
                printf("Failed authentication from client.\n");
                conn[csock].state = 8;
              }
#else
              conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
#endif
            }
          } else if (conn[n].state != 8) { /* read from authorised client */
            if ((size = recv(n, buffer, 1024, 0)) == -1)
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
              conn[n].state = 0; /* Note that the connection is closed */
              for (i = 0; i < 1024; i++)
                if (conn[i].spy == 1)
                  conn[i].spy = 3;
              continue;
            } else {
              buffer[1023] = '\0';
              for (i = 0; i < 1023; ++i)
                if (buffer[i] == '\n' || buffer[i] == '\r')
                  buffer[i] = '\0';
            }

            printf("%i-->%s\n", n, buffer);

            if (conn[n].state == 0) { /* authentication */
              if (strncmp(buffer, "::user::", 8) == 0) {
                conn[n].state = 1;
                strcpy(conn[n].user, buffer + 8);
                printf("Authentication on socket %i by %s.\n", n, conn[n].user);
                for (i = 0; i < 1024; i++) /* Remember to tell moles about new user */
                  if (conn[i].spy == 1)
                    conn[i].spy = 3;
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
            } else if (strncmp(buffer, "::ping::", 8) == 0) {
              conn[n].state |= 0x100;
            } else if (strncmp(buffer, "::lurk::", 8) == 0) {
              conn[n].lurk = 2;
            } else if (strncmp(buffer, "::spy::", 7) == 0) {
              conn[n].spy = 2;
            } else if (strncmp(buffer, "::list::", 8) == 0) {
              conn[n].state = 4;
            } else if (owner != n) { /* no conn */
              if (strncmp(buffer, "::take::", 8) == 0) {
                printf("Socket %i has taken the conn.\n", n);
                report = 1;
                owner = n;
                conn[n].state = 2;
              } else
                conn[n].state = 5;
            } else if (owner == n) { /* has conn */
              if (strncmp(buffer, "::give::", 8) == 0) {
                printf("Socket %i has given up the conn.\n", n);
                report = 1;
                owner = 0;
                conn[n].state = 2;
              } else if (buffer[0] != ':') {
                ack = ExecuteCommand(n, fd, route, buffer);
                cmd_from = n;
                strncpy(cmd, buffer, 512);
                cmd[511] = 0;
                for (i = 0; i < 1024; i++)
                  if (conn[i].lurk)
                    conn[i].lurk = 3;
              }
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
            } else if (conn[n].lurk == 2) { /* request for lurking */
              strcpy(buffer, ":::slink:::\r\n");
              conn[n].lurk = 1;
            } else if (conn[n].spy == 2) { /* request for spying */
              strcpy(buffer, ":::mole:::\r\n");
              conn[n].spy = 1;
            } else if (conn[n].state & 0x100) { /* ping pong */
              strcpy(buffer, ":::pong:::\r\n");
              conn[n].state &= ~0x100;
            } else if (conn[n].lurk == 3) { /* report command */
              snprintf(buffer, 1024, ":::sender:::%s\r\n:::cmd:::%s\r\n:::rep:::%d\r\n", 
                       conn[cmd_from].user, cmd, ack);
              conn[n].lurk = 1;
              buffer[1023] = 0;
            } else if (conn[n].spy == 3) { /* report clients */
              for (i = 0; i < 1024; i++) {
                if (conn[i].state) {
                  snprintf(tmp_buf, 100, ":::here:::%s\r\n", conn[i].user);
                  strncat(buffer + strlen(buffer), tmp_buf, 30*1024 - strlen(buffer));
                }
              }
              conn[n].spy = 1;
            } else if (conn[n].state == 8) /* authentication failed */
              strcpy(buffer, ":::nope:::\r\n");

            /* send */
            if (buffer[0]) {
              printf("%i<--%s", n, buffer);
              if ((size = send(n, buffer, strlen(buffer), MSG_NOSIGNAL))
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

            /* close unauthorised connexions */
            if (conn[n].state == 8) {
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
            }

            if (conn[n].state == 2 && conn[n].report) {
              conn[n].report = 0;
              conn[n].state = 1;
            }
          } /* n != sock */
      } /* socket loop */

    nanosleep(&sleep_time, NULL);
  } /* main loop */
}
