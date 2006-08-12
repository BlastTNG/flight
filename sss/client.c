/* client to demonstrate how to connect to sun sensor
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * Copyright (C) 2005 Matthew Truch
 * 
 * Released under the GPL
 *
 */

#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "client.h"

void printfData (sss_packet_data *dat) {

  printf("%.3f", dat->sun_time);
  printf(" %u %u %u %u %u %u %u %u %u %u %u %u",
      dat->m01, dat->m02, dat->m03, dat->m04, dat->m05, dat->m06,
      dat->m07, dat->m08, dat->m09, dat->m10, dat->m11, dat->m12);
  printf(" %.3f %.3f %.3f", dat->v5, dat->v12, dat->vbatt);
  printf(" %.2f %.2f %.2f %.2f %.2f",
      dat->t_cpu, dat->t_hdd, dat->t_case, dat->t_port, dat->t_starboard);
  printf(" %.3f %.3f %.3f %.3f %.3e %u", dat->az_rel_sun, dat->amp,
      dat->dc_off, dat->phase, dat->chi, dat->iter);
  printf("\n");
  fflush(stdout);
}

int main (int argv, char *argc[]) {
  int sock = -1, n;


  fd_set fdr;
  struct timeval timeout;

  struct sockaddr_in addr;

  sss_packet_data dat;

  while (1) {
    if (sock != -1)
      if (close(sock) == -1)
      {
        fprintf(stderr, "Error on close()\n");
        return 2;
      }
    /* create an empty socket connection */
    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == -1)
    {
      fprintf(stderr, "Error on socket()\n");
      return 3;
    }

    /* set options */
    n = 1;
    if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) == -1)
    {
      fprintf(stderr, "Error on setsockopt()\n");
      return 4;
    }

    /* Connect to Sun Sensor */
    if (argv >= 2) //you can put the address on the command line
    {
      if (!inet_aton(argc[1], &addr.sin_addr))  //if it's valid, we'll use it
      {
        fprintf(stderr, "Address '%s' is invalid, using default (%s)\n",
          argc[1], SSS_IP);
        inet_aton(SSS_IP, &addr.sin_addr); //otherwise use the default
      }
    }
    else
    {
      fprintf(stderr, "No address on command line, using default (%s)\n",
        SSS_IP);
      inet_aton(SSS_IP, &addr.sin_addr);
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SSS_PORT);
    while ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
        < 0) {
      fprintf(stderr, "Error on connect()\nSleeping for 10 seconds\n");
      sleep(10);
    };

    fprintf(stderr, "Connected to the Sun Sensor\n");
    n = 0;

    while (n != -1) {
      usleep(10000);

      FD_ZERO(&fdr);
      FD_SET(sock, &fdr);

      timeout.tv_sec = 10;
      timeout.tv_usec = 0;

      n = select(sock + 1, &fdr, NULL, NULL, &timeout);

      if (n == -1 && errno == EINTR) {
        fprintf(stderr, "Sun Sensor: Timeout\n");
        continue;
      }
      if (n == -1) {
        fprintf(stderr, "Error on select()\n");
        continue;
      }

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, &dat, sizeof(dat), MSG_DONTWAIT);
        if (n == sizeof(dat)) {
          printfData(&dat);
        } else if (n == -1) {
          fprintf(stderr, "Error on recv()\n");
        } else if (n == 0) {
          fprintf(stderr, "Connection to Sun Sensor closed\n");
          n = -1;
        } else {
          fprintf(stderr, "Didn't receive all data from Sun Sensor.\n");
        }
      } else {
        fprintf(stderr, "Connection to Sun Sensor timed out.\n");
        n = -1;
      }
    }
  }
}
