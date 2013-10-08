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
#include "bitdaemon.h"
//#include "command_list.h"

void setDefault(char *cmd, double D); // in cmdlist.c
double getDefault(char *cmd_in); // in cmdlist.c
struct sockaddr_in serv_addr, my_addr; // UDP sockets
socklen_t slen = sizeof(serv_addr);

unsigned short N_SCOMMANDS;
unsigned short N_MCOMMANDS;
struct scom *scommands;
struct mcom *mcommands;
int N_GROUPS;
char **GroupNames;
char command_list_serial[1024];

// error generator
void err(char *s)
{
	perror(s);
	exit(1);
}

void SendCmdDefault(int sock, double d)
{
	char output[512];

	sprintf(output, ":::cmddef:::%10g\r\n", d);
	printf("%i<--%s", sock, output);
	send(sock, output, strlen(output), MSG_NOSIGNAL);
}

void RelayCommandList(int sock)
{
	uint16_t u16;
  int i, j, k;
  char output[4096];

  sprintf(output, ":::rev:::%s\r\n", command_list_serial);
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);

  u16 = N_SCOMMANDS;
  if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
    return;
  u16 = N_MCOMMANDS;
  if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
    return;
    
   
  if (send(sock, scommands, sizeof(struct scom) * N_SCOMMANDS, MSG_NOSIGNAL)< 0)
  {
				printf("RelayCommandList scommand error: %s\n",strerror(errno));
    return;
  }
  if (send(sock, mcommands, sizeof(struct mcom) * N_MCOMMANDS, MSG_NOSIGNAL)< 0)
  {
			printf("RelayCommandList mcommand error: %s\n",strerror(errno));
    return;
  }

  for (i = 0; i < N_MCOMMANDS; ++i) {
    for (j = 0; j < mcommands[i].numparams; ++j)
      if (mcommands[i].params[j].nt) {
        u16 = i * 256 + j;
        if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
        {
									 printf("RelayCommandList u16 error: %s\n",strerror(errno));
          return;
								}

        /* count */
        for (u16 = 0; mcommands[i].params[j].nt[u16]; ++u16);
        if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
        {
										printf("RelayCommandList u16 error: %s\n",strerror(errno));
          return;
								}

        output[0] = 0;
        for (k = 0; mcommands[i].params[j].nt[k]; ++k) {
          strncat(output, mcommands[i].params[j].nt[k], 79);
          strcat(output, "\n");
        }
        if (send(sock, output, strlen(output), MSG_NOSIGNAL) < 1)
        {
									 printf("RelayCommandList output error: %s\n",strerror(errno));
          return;
								}
      } 
  }
  u16 = 0xFFFF;
  if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
  {
				printf("RelayCommandList u16 error: %s\n",strerror(errno));
    return;
		}
}

void RelayGroupNames(int sock)
{
  unsigned short i;
  char output[4096];
		
  i = N_GROUPS;
  if (send(sock, &i, sizeof(i), MSG_NOSIGNAL) < 1)
    return;

  output[0] = '\0';
  for (i=0; i<N_GROUPS; ++i) {
    strncat(output, GroupNames[i], 127);
    strcat(output, "\n");
  }
  send(sock, output, strlen(output), MSG_NOSIGNAL);
}

/* makes a TCP socket to talk with COW */
int MakeSock(void)
{
	int sock, n;
	struct sockaddr_in addr;

	if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) 
	{
		perror("Unable to create socket");
		exit(15);
	}

	n = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) 
	{
		perror("Unable to set socket options");
		exit(15);
	}

	addr.sin_family = AF_INET;
	addr.sin_port = htons(SOCK_PORT);
	addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) 
	{
		perror("Unable to bind to port");
		exit(15);
	}

	if (listen(sock, 100) == -1) 
	{
		perror("Unable to listen on port");
		exit(15);
	}

	printf("Listening on port %i.\n", SOCK_PORT);

	return sock;
}

int CreateUDPSock()
{
	int sockfd; // socket file descripter
	struct hostent* the_target;

	if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		err("socket");

	int size = 65536;
	if (setsockopt(sockfd,SOL_SOCKET,SO_RCVBUF,&size,sizeof(size)) < 0)
		printf("Unable to set socket options.\n");

	// setup host UDP socket
	bzero(&my_addr, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(TARGET_PORT);
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(sockfd, (struct sockaddr *) &my_addr, sizeof(my_addr)) < 0) err("bind");

	// setup target UDP socket
	the_target = gethostbyname(target); /* get remote host IP */

	if (the_target == NULL) 
	{
		fprintf(stderr, "host lookup failed for `%s': %s\n", target,
		hstrerror(h_errno));
		return -14;
	}

	bzero(&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(TARGET_PORT);
	memcpy(&(serv_addr.sin_addr.s_addr), the_target->h_addr, the_target->h_length);

	//printf("UDP link to %s estabalished.\n",target);

	return sockfd;
}

int UDPRoute(int fd, char* buffer)
{
	if (sendto(fd, buffer, strlen(buffer), 0, (struct sockaddr*)&serv_addr, slen) < 0)
		perror("UDPRoute has failed\n");
	return 0;
}

int ExecuteCommand(int sock, int fd, int route, char* buffer)
{
	int result = 0;
	char output[100];

	printf("EXE %s\n", buffer);

	result = UDPRoute(fd,&buffer[3]);

	if (err_message[0]) { /* parameter validation failed */
		sprintf(output, ":::ack:::%i:::%s\r\n", result, err_message);
	} else {
		sprintf(output, ":::ack:::%i\r\n", result);
	}
	printf("%i<--%s", sock, output);
	send(sock, output, strlen(output), MSG_NOSIGNAL);

	return result;
}

//Blocks on reading until list comes through.
int ReceiveGroupNamesUDP(int fd)
{
	int i;
	char buffer[128] = "::group::";

	sendto(fd, buffer, strlen(buffer), MSG_NOSIGNAL, (struct sockaddr*)&serv_addr, slen);

	if (recvfrom(fd, &N_GROUPS, sizeof(N_GROUPS), 0, (struct sockaddr*)&serv_addr, &slen) < 0)
		printf("Warning: ReceiveGroupNames failed to read n_groups\n");
	if (N_GROUPS > 31) 
	{
		fprintf(stderr, "Protocol error from daemon.\n");
		exit(14);
	}
	GroupNames = (char**)malloc(N_GROUPS * sizeof(char*));
	
	char *buf;
	int len = N_GROUPS * 128;
	buf = (char*)malloc(len);
	
	if (buf == NULL) 
				printf("Warning: ReceiveGroupNames failed to allocate buffer memory");
	
	if (recvfrom(fd, buf, len, 0, (struct sockaddr*)&serv_addr, &slen) < 0)
		printf("Warning: ReceiveGroupNames failed to read group_names\n");

	char *value;
	i = 0;
	do
	{
		GroupNames[i] = NULL;
		if (i==0) value = strtok(buf,"\n");
		else value = strtok(NULL,"\n");
		if (value != NULL)
		{
			GroupNames[i] = value;
			//printf("%s\n",GroupNames[i]);
			i++;
		}
		
	}
	while (value != NULL);
	;
	if (i != N_GROUPS) 
		printf("Warning: ReceiveGroupNames expected %d groups, but received %d\n",N_GROUPS,i);

	return 0;
}

int ReceiveCmdListUDP(int fd)
{
	uint16_t u16;
	int n, c = 0;
	size_t i, len;
	char buffer[1024] = "::list::";
	char *buf;

	sendto(fd, buffer, strlen(buffer), MSG_NOSIGNAL, (struct sockaddr*)&serv_addr, slen);
	if ((n = recvfrom(fd, command_list_serial, sizeof(command_list_serial), 0, (struct sockaddr*)&serv_addr, &slen)) <= 0) 
		printf("Warning: ReceiveCmdListUDP unable to read command_list_serial\n");
		
	for (i=0;i<strlen(command_list_serial);i++) // erase header and terminator
	{
			if ((command_list_serial[i] == '\n') || (command_list_serial[i] == '\r'))
			{
				command_list_serial[i] = '\0';
				break;
			}
	}
	if (recvfrom(fd, &N_SCOMMANDS, sizeof(N_SCOMMANDS), 0, (struct sockaddr*)&serv_addr, &slen) < 0)
		printf("Warning: ReceiveCmdListUDP failed to read n_scommands\n");
	if (recvfrom(fd, &N_MCOMMANDS, sizeof(N_MCOMMANDS), 0, (struct sockaddr*)&serv_addr, &slen) < 0)
		printf("Warning: ReceiveCmdListUDP failed to read n_mcommands\n");

	if (N_SCOMMANDS > 255 || N_MCOMMANDS > 255 || N_SCOMMANDS * N_MCOMMANDS == 0)
	{
		printf("Error: ReceiveCmdListUDP received invalid N_SCOMMANDS=%d and/or N_MCOMMANDS=%d\n",N_SCOMMANDS,N_MCOMMANDS);
		exit(14);
	}

	scommands = (struct scom*)malloc(N_SCOMMANDS * sizeof(struct scom));
	mcommands = (struct mcom*)malloc(N_MCOMMANDS * sizeof(struct mcom));

	recvfrom(fd, scommands, N_SCOMMANDS * sizeof(struct scom), MSG_WAITALL, (struct sockaddr*)&serv_addr, &slen);
	recvfrom(fd, mcommands, N_MCOMMANDS * sizeof(struct mcom), MSG_WAITALL, (struct sockaddr*)&serv_addr, &slen);

	/* read parameter value tables */
	for (;;) 
	{
		int cmd, prm;
		if (recvfrom(fd, &u16, sizeof(uint16_t), 0, (struct sockaddr*)&serv_addr, &slen) < 1)
			goto CMDLIST_READ_ERROR;

		if (u16 == 0xFFFF) /* end */
			break;
		cmd = u16 >> 8;
		prm = u16 & 0xFF;

		if (cmd >= N_MCOMMANDS || prm >= mcommands[cmd].numparams)
			goto CMDLIST_READ_ERROR;

		if (recvfrom(fd, &u16, sizeof(uint16_t), 0, (struct sockaddr*)&serv_addr, &slen) < 1)
			goto CMDLIST_READ_ERROR;

		mcommands[cmd].params[prm].nt = malloc((u16 + 1) * sizeof(char*));
		
		len = 80*u16;
		buf = (char*)malloc(len);
		if (recvfrom(fd, buf, len, 0, (struct sockaddr*)&serv_addr, &slen) < 1)
			goto CMDLIST_READ_ERROR;
			
		char *value;
		i = 0;
		do
		{
			mcommands[cmd].params[prm].nt[i] = NULL;
			if (i==0) value = strtok(buf,"\n");
			else value = strtok(NULL,"\n");
			if (value != NULL)
			{
				mcommands[cmd].params[prm].nt[i] = value;
				printf("%s\n",mcommands[cmd].params[prm].nt[i]);
				i++;
			}
		}
		while (value != NULL);
		mcommands[cmd].params[prm].nt[u16] = NULL;
		if (i != u16) 
			printf("Warning: ReceiveCmdListUDP expected %d nt params but received %d\n",u16,i);
	}
	return 0;

	CMDLIST_READ_ERROR:
	fprintf(stderr, "Protocol error from daemon.\n");
	exit(14);
		
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
   *  3 = client requested group names
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
  double d;
  char dcmd[SIZE_CMDPARNAME];

  int fd, n, i, size, pid, reset_lastsock = 0;
  int report = 0;
  struct sockaddr_in addr;
  int sock, csock, lastsock;
  socklen_t addrlen;
  fd_set fdlist, fdread, fdwrite;

  struct timespec sleep_time;
  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 10000000; /* 10ms */

  /* open our output before daemonising just in case it fails. */
  if (route == 3) /* UDP socket */
    fd = CreateUDPSock();
  else
    fd = -1;

  if (fd < 0) {
    perror("Unable to open output device");
    exit(2);
  }
  
  /* request and receive the command list and group names from target */
  ReceiveGroupNamesUDP(fd);
  ReceiveCmdListUDP(fd);
  
  printf("Command list and group names received from target.\n");

  buffer = malloc(30 * 1024 * sizeof(char));
  if (buffer == NULL) {
    perror("Unable to malloc send buffer");
    exit(3);
  }

  /* start the listener. */
  lastsock = sock = MakeSock();

  signal(SIGCHLD, SIG_IGN); /* We don't wait for children so don't require
                               the kernel to keep zombies.                  */

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
    if (chdir("/") < 0) perror("chdir failed");
    if (!freopen("/dev/null", "r", stdin)) perror("freopen stdin failed");
    if (!freopen("/dev/null", "w", stdout)) perror("freopen stdout failed");
    if (!freopen("/dev/null", "w", stderr)) perror("freopen stderr failed");
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
        if (FD_ISSET(i, &fdlist))
          lastsock = i;
    }

    if (report) {
      reset_lastsock = 0;
      for (i = 0; i < sizeof(fd_set) * 8; ++i)
        if (i != sock && FD_ISSET(i, &fdlist))
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
              conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
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
            } else if (strncmp(buffer, "::group::", 9) == 0) {
              conn[n].state = 3;
            } else if (strncmp(buffer, "::list::", 8) == 0) {
              conn[n].state = 4;
            } else if (strncmp(buffer, "::cmddef::", 10) == 0) {
              conn[n].state = 6;
              strncpy(dcmd, buffer+10, SIZE_CMDPARNAME);
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
                  if (i != n && conn[i].lurk)
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
            } else if (conn[n].state == 3) { /* list */
              RelayGroupNames(n);
              conn[n].state = 2;
            } else if (conn[n].state == 4) { /* list */
              RelayCommandList(n);
              conn[n].state = 2;
            } else if (conn[n].state == 5) { /* request with no conn */
              strcpy(buffer, ":::noconn:::\r\n");
              conn[n].state = 1;
            } else if (conn[n].state == 6) { /* get command default */
              d = getDefault(dcmd);
              SendCmdDefault(n, d);
              conn[n].state = 2;
            } else if (conn[n].lurk == 2) { /* request for lurking */
              strcpy(buffer, ":::slink:::\r\n");
              conn[n].lurk = 1;
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
            } else if (conn[n].spy == 2) { /* request for spying */
              strcpy(buffer, ":::mole:::\r\n");
              conn[n].spy = 3;
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
