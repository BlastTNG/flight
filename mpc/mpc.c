/* MPC: MCE-PCM communicator
 *
 * Copyright (c) 2013, D. V. Wiebe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * This software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. In no event shall the copyright holder or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */
#include "blast.h"
#include "mpc_proto.h"
#include "udp.h"
#include "tes.h"

#include <sys/statvfs.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#define MAS_DATA_ROOT "/data0/mce/"

/* some timing constants; in the main loop, timing is done using the udp poll
 * timeout.  As a result, all timings are approximate (typically lower bounds)
 * and the granularity of timing is the UDP_TIMEOUT
 */
#define UDP_TIMEOUT    100 /* milliseconds */

/* wait time betewwn sending the ping to an unresponsive PCM? */
#define INIT_TIMEOUT 60000 /* milliseconds */

/* sets the rate at which slow data are sent to PCM.  Since PCM multiplexes
 * these over the MCE count into a slow channel, this can happen "rarely".
 * If we assume the PCM frame rate is 100 Hz, it's once every 1.2 seconds;
 * this is slightly faster than that, probably.
 */
#define SLOW_TIMEOUT   800 /* milliseconds */

/* UDP socket */
int sock;

/* The number of the attached MCE */
int nmce = 0;

/* Initialisation veto */
int init = 1;

/* Data return veto */
int veto = 0;

/* The slow data struct */
struct mpc_slow_data slow_dat;

/* the list of bolometers to send to PCM */
uint16_t bset_num = 0xFFFF;
int ntes = 0;
int16_t tes[NUM_ROW * NUM_COL];

/* make and send fake data -- we only make data mode 11 data, which is static */
//#define FAKE_DATA_RATE   2500 /* microseconds -- this is approximate */
#define FAKE_DATA_RATE   1000000 /* microseconds -- this is approximate */
static void *fake_data(void *dummy)
{
  uint32_t framenum = 0x37183332;
  int i, j;
  uint32_t mode11[NUM_COL * NUM_ROW];

  char data[UDP_MAXSIZE];

  /* generate the mode 11 data -- this is simply:
   *
   *  0000 0000 0000 0000 0000 000r rrrr rccc
   *
   *  where :
   *
   *  r = the row number
   *  c = the column number
   */
  for (i = 0; i < NUM_COL; ++i)
    for (j = 0; j < NUM_ROW; ++j)
      mode11[i + j * NUM_COL] = (i & 0x7) | ((j & 0x3F) << 3);

  /* wait for initialisation from PCM */
  while (init)
    sleep(1);

  /* send data packets at half the "frame rate" */
  for (;;) {
    /* we just don't bother sending anything if nothing is requested */
    if (!veto && ntes > 0) {
      size_t len = mpc_compose_tes(mode11, framenum, bset_num, nmce, ntes, tes,
          data);
      udp_bcast(sock, MCESERV_PORT, len, data);
    }
    framenum += 2;
    usleep(FAKE_DATA_RATE * 2);
  }

  return NULL;
}

/* send slow data to PCM */
static void send_slow_data(char *data)
{
  struct timeval tv;
  struct statvfs buf;
  size_t len;

  /* data mode is always 11 for now */
  slow_dat.data_mode = 11;

  /* disk free -- units are 2**24 bytes = 16 MB */
  if (statvfs(MAS_DATA_ROOT, &buf) == 0)
    slow_dat.df =
      (uint16_t)(((unsigned long long)buf.f_bfree * buf.f_bsize) >> 24);

  /* time -- this wraps around ~16 months after the epoch */
  gettimeofday(&tv, NULL);
  slow_dat.time = (tv.tv_sec - MPC_EPOCH) * 100 + tv.tv_usec / 10000;

  /* make packet and send */
  len = mpc_compose_slow(&slow_dat, nmce, data);
  udp_bcast(sock, MCESERV_PORT, len, data);
}

int main(void)
{
  int port, type;
  ssize_t n;
  size_t len;

  pthread_t data_thread;

  int init_timer = 0;
  int slow_timer = 0;

  char peer[UDP_MAXHOST];
  char data[UDP_MAXSIZE];

  printf("This is MPC.\n");
  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC subsystem.");

  /* bind to the UDP port */
  sock = udp_bind_port(MPC_PORT, 1);

  /* create the data thread */
  pthread_create(&data_thread, NULL, fake_data, NULL);

  /* main loop */
  for (;;) {
    struct ScheduleEvent ev;

    /* check inbound packets */
    n = udp_recv(sock, UDP_TIMEOUT, peer, &port, UDP_MAXSIZE, data);

    /* n == 0 on timeout */
    type = (n > 0) ? mpc_check_packet(n, data, peer, port) : -1;

    if (init) {
      if (init_timer <= 0) {
        /* send init packet */
        len = mpc_compose_init(nmce, data);

        /* Broadcast this to everyone */
        if (udp_bcast(sock, MCESERV_PORT, len, data) == 0)
          bputs(info, "Broadcast awake ping.\n");

        init_timer = INIT_TIMEOUT;
      } else if (n == 0)
        init_timer -= UDP_TIMEOUT;
    } else {
      /* finished the init; slow data */
      if (slow_timer <= 0) {
//        send_slow_data(data);
        slow_timer = SLOW_TIMEOUT;
      } else if (n == 0)
        slow_timer -= UDP_TIMEOUT;
    }

    /* skip bad packets */
    if (type < 0)
      continue;

    /* do something based on packet type */
    switch (type) {
      case 'C': /* command packet */
        if (mpc_decompose_command(&ev, n, data)) {
          /* command decomposition failed */
          break;
        }
        /* run the command here */
        break;
      case 'F': /* field set packet */
        veto = 1; /* veto transmission */
        if ((n = mpc_decompose_bset(&bset_num, tes, nmce, n, data)) >= 0)
          ntes = n;
        veto = 0; /* unveto transmission */
        init = 0;
        break;
      default:
        bprintf(err, "Unintentionally dropping unhandled packet of type 0x%X\n",
            type);
    }
  }

  close(sock);
  return 0;
}
