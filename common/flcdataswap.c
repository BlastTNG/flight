/* mcp: the master control program
 *
 * flcdataswap - swap data between FLCs using UDP packets
 *
 * This software is copyright (C) 2012 University of Toronto
 *
 * This file is part of mcp and pcm
 *
 * mcp and pcm are free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp and pcm are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <string.h>
#include <netdb.h>
#include <pthread.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "flcdataswap.h"
#include "blast.h"
#include "mcp.h"

//in and out buffers
static int i_flc_out = 0;
static int i_flc_in = 0;
static struct flc_data flc_out_data[3];
static struct flc_data flc_in_data[3];

static void *data_in_thread(void *dummy)
{
  struct addrinfo hints, *ai_other, *p;
  int ret;
  int sockfd = -1;
  ssize_t num, received;

  //nameThread("SwapRx");
  //bputs(startup, "Startup\n");

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET; //use IPv4, AF_UNSPEC allows IPv4 or IPv6
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV;

	if ((ret = getaddrinfo(NULL, FLC_DATA_PORT, &hints, &ai_other)) != 0) {
		bprintf(tfatal, "getaddrinfo: %s\n", gai_strerror(ret));
	}

  // loop through getaddrinfo results
  for(p = ai_other; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      continue;
    }
    if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      continue;
    }
    break;
  }
  if (p == NULL) berror(tfatal, "failed to bind socket\n");
	freeaddrinfo(ai_other);

  while (1) {
    //receive data from other
    received = 0;
    while (received < sizeof(struct flc_data)) {
      if ((num = recvfrom(sockfd, &flc_in_data[i_flc_in] + received,
              sizeof(struct flc_data) - received, 0, NULL, NULL)) == -1) {
        berror(err, "recvfrom");
      }
      received += num;
    }
    i_flc_in = INC_INDEX(i_flc_in);
  }

  return NULL;
}

static void *data_out_thread(void *v_other)
{
  struct addrinfo hints, *ai_other, *p;
  int ret;
  int sockfd = -1;
  int last_i_flc_out = 0;
  ssize_t num;
  char *other = (char*)v_other;

  //nameThread("SwapTx");
  //bputs(startup, "Startup\n");

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET; //use IPv4, AF_UNSPEC allows IPv4 or IPv6
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;//numbers, don't do lookup

	if ((ret = getaddrinfo(other, FLC_DATA_PORT, &hints, &ai_other)) != 0) {
		bprintf(tfatal, "getaddrinfo: %s\n", gai_strerror(ret));
	}

  // loop through getaddrinfo results
  for(p = ai_other; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      continue;
    }
    break;
  }
  if (p == NULL) berror(tfatal, "failed to make socket\n");

  while (1) {
    //send data to other
    while (i_flc_out == last_i_flc_out) usleep(1000);
    if ((num = sendto(sockfd, &flc_out_data[GETREADINDEX(i_flc_out)],
            sizeof(struct flc_data), 0, p->ai_addr, p->ai_addrlen)) < 0) {
      berror(err, "sendto");
    }
    last_i_flc_out = i_flc_out;
  }

  return NULL;
}

void start_flc_data_swapper(const char *other)
{
  pthread_t in_id;
  pthread_t out_id;
  pthread_create(&in_id, NULL, &data_in_thread, NULL);
  pthread_create(&out_id, NULL, &data_out_thread, (void*)other);
}

struct flc_data *get_flc_out_data()
{
  return &flc_out_data[i_flc_out];
}

struct flc_data *swap_flc_data(struct flc_data *d)
{
  i_flc_out = INC_INDEX(i_flc_out);
  memcpy(d, &flc_in_data[GETREADINDEX(i_flc_in)], sizeof(struct flc_data));
  return d;
}

