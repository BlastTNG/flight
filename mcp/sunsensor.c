/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <sys/types.h>
#include <time.h>
#include <sys/socket.h>
#include <signal.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "sss_struct.h"
#include "pointing_struct.h"
#include "mcp.h"

#define ARIEN "192.168.1.7"
#define ARIEN_PORT 54321

void nameThread(const char*);	/* mcp.c */

sss_packet_data SunSensorData[3];
int ss_index = 0;

void SunSensor(void) {
  int sock = -1, n;


  fd_set fdr;
  struct timeval timeout;

  struct sockaddr_in addr;

  sss_packet_data Rx_Data;

  nameThread("Sun");
  bputs(startup, "Startup\n");

  while (1) {
    if (sock != -1)
      if (close(sock) == -1)
        berror(err, "close()");

    /* create an empty socket connection */
    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == -1)
      berror(tfatal, "socket()");

    /* set options */
    n = 1;
    if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) == -1)
      berror(tfatal, "setsockopt()");

    /* Connect to Arien */
    inet_aton(ARIEN, &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(ARIEN_PORT);
    while ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
        < 0) {
      berror(err, "connect()");
      sleep(10);
    };

    bputs(info, "Connected to Arien\n");
    n = 0;

    while (n != -1) {
      usleep(10000);

      FD_ZERO(&fdr);
      FD_SET(sock, &fdr);

      timeout.tv_sec = 10;
      timeout.tv_usec = 0;

      n = select(sock + 1, &fdr, NULL, NULL, &timeout);

      if (n == -1 && errno == EINTR) {
        bputs(warning, "Timeout\n");
        continue;
      }
      if (n == -1) {
        berror(err, "select()");
        continue;
      }

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, &Rx_Data, sizeof(Rx_Data), MSG_DONTWAIT);
        if (n == sizeof(Rx_Data)) {
          SunSensorData[ss_index] = Rx_Data;
          ss_index = INC_INDEX(ss_index);
        } else if (n == -1) {
          berror(err, "recv()");
        } else if (n == 0) {
          bprintf(err, "Connection to Arien closed");
          n = -1;
        } else {
          bprintf(err, "Short read: %i of %i bytes.\n", n,
              sizeof(Rx_Data));
          n = -1;
        }
      } else {
        bputs(warning, "Connection to Arien timed out.\n");
        n = -1;
      }
    }
  }
}
