/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2006 University of Toronto
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "isc_protocol.h"
#include "mcp.h"

#define BASE_PORT 2000

//#define USE_ISC_LOG

static struct {
  char who[4];
  char where[16];
} isc_which[2] = {
  {"ISC", "192.168.1.8"},
  {"OSC", "192.168.1.9"}
};

extern short int SouthIAm;   /* mcp.c */
void nameThread(const char*);
extern short int InCharge; /* tx.c */
extern int EthernetIsc; /* tx.c */
extern int EthernetOsc; /* tx.c */

/*---- ISC semaphores ----*/

/* write_ISC_pointing -- send a new solution packet to the star camera:
 *   raised by auxiliary.c
 *   lowered by isc.c
 */
short int write_ISC_pointing[2] = {0, 0};
/* write_ISC_trigger -- send the timing pulse request to the ACS:
 *   raised by isc.c
 *   lowered by auxiliary.c
 */
short int write_ISC_trigger[2] = {0, 0};
/* ISC_link_ok -- flag the link as bad: don't wait for handshaking from SC:
 *   raised by auxiliary.c
 *   lowered by isc.c
 */
short int ISC_link_ok[2] = {0, 0};
/* start_ISC_cycle -- start a new isc plse cycle:
 *   raised by isc.c
 *   lowered by auxiliary.c
 */
short int start_ISC_cycle[2] = {0, 0};

struct ISCStatusStruct ISCSentState[2];

struct ISCSolutionStruct ISCSolution[2][5];
int iscread_index[2] = {0, 0};
int iscwrite_index[2] = {0, 0};
int iscpoint_index[2] = {0, 0};

#ifdef USE_ISC_LOG
static FILE* isc_log[2] = {NULL, NULL};
#endif

static int ISCInit(int which)
{
  int sock;
  struct sockaddr_in addr;
  static int firsttime = 1;

  int n;

#ifdef USE_ISC_LOG
  if (which) {
    if (isc_log[which] == NULL)
      if ((isc_log[which] = fopen("/tmp/isc.1.log", "a")) == NULL)
        berror(err, "log fopen()");
  } else {
    if (isc_log[which] == NULL)
      if ((isc_log[which] = fopen("/tmp/isc.0.log", "a")) == NULL)
        berror(err, "log fopen()");
  }
  fprintf(isc_log[which], "This is %s.\n", isc_which[which].who);
#endif

  sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == -1) {
    berror(err, "socket()");
    return -1;
  }

  n = 1;
  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    berror(err, "setsockopt()");
    if (sock != -1)
      if (close(sock) < 0)
        berror(err, "close()");
    return -1;
  }

  inet_aton(isc_which[which].where, &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(BASE_PORT + SouthIAm);

  if ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
      < 0) {
    if (firsttime) {
      berror(err, "connect()");
      firsttime = 0;
    }
    if (errno == ENETUNREACH || errno == EHOSTUNREACH || errno == EHOSTDOWN)
      which ? (EthernetOsc = 1) : (EthernetIsc = 1);
    else if (errno == ECONNREFUSED)
      which ? (EthernetOsc = 2) : (EthernetIsc = 2);
    else
      which ? (EthernetOsc = 3) : (EthernetIsc = 3);
    if (sock != -1)
      if (close(sock) < 0)
        berror(err, "close()");
    return -1;
  }

  bprintf(info, "Connected in %s mode\n",
      (InCharge) ? "active" : "passive");
  CommandData.ISCState[which].shutdown = 0;

  which ? (EthernetOsc = 0) : (EthernetIsc = 0);

  if (WHICH)
    bprintf(info, "%iSC (i): Lowered write_ISC_pointing semaphore on connect\n",
        which);
  write_ISC_pointing[which] = 0;
  if (WHICH)
    bprintf(info, "%iSC (i): Raised ISC_link_ok\n", which);
  ISC_link_ok[which] = 1;

  ISCSentState[which] = CommandData.ISCState[which];

  firsttime = 1; /* reset firsttime as we made a good connection */
  return sock;
}

static double GetNominalVel(struct PointingDataStruct MyPointData)
{
  if (CommandData.pointing_mode.nw) { /* doing slew */
    return 1.9 * DEG2RAD;
  } else if (CommandData.pointing_mode.mode == P_AZEL_GOTO ||
      CommandData.pointing_mode.mode == P_RADEC_GOTO ||
      CommandData.pointing_mode.mode == P_LOCK) {
    return 0;
  } else if (CommandData.pointing_mode.mode == P_DRIFT) { /* drift mode */
    return sqrt(CommandData.pointing_mode.vaz * CommandData.pointing_mode.vaz
        + CommandData.pointing_mode.del * CommandData.pointing_mode.del)
      * DEG2RAD;
  } else {
    return CommandData.pointing_mode.vaz * DEG2RAD;
  }
}

void IntegratingStarCamera(void* parameter)
{
  fd_set fdr, fdw;
  struct PointingDataStruct MyPointData;
  int which = (intptr_t)parameter;
  int waiting_for_ACK = 0;

  int sock = -1;

#ifdef USE_ISC_LOG
  time_t t;
#endif

  int n, save_image_state = 0;

  nameThread(isc_which[which].who);
  bprintf(startup, "Startup\n");

  for (;;) {
    which ? (EthernetOsc = 3) : (EthernetIsc = 3);
    do {
      if (sock != -1)
        if (close(sock) < 0)
          berror(err, "close()");

      sock = ISCInit(which);
      if (sock == -1) {
        sleep(10);
      }
    } while (sock == -1);

    /* select loop */
    for (;;) {
      /* fill the file descriptor lists */
      FD_ZERO(&fdr);
      FD_ZERO(&fdw);

      FD_SET(sock, &fdw);
      FD_SET(sock, &fdr);

      /* request received to reconnect to ISC */
      if (CommandData.ISCControl[which].reconnect) {
        CommandData.ISCControl[which].reconnect = 0;
        break;
      }

      usleep(10000);
      /* select poll on fds; this is a blocking call */
      n = select(sock + 1, &fdr, &fdw, NULL, NULL);

      if (n == -1 && errno == EINTR)
        continue;
      if (n == -1) {
        berror(err, "select()");
        continue;
      }

      /* ------------ read ---------- */

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, &ISCSolution[which][iscwrite_index[which]],
            sizeof(struct ISCSolutionStruct), 0);
        if (n == -1) {
          berror(err, "recv()");
          break;
        } else if (n < sizeof(struct ISCSolutionStruct)) {
          bprintf(err, "Expected %i but received %i bytes.\n",
              (int) sizeof(struct ISCSolutionStruct), n);
          break;
        }
#ifdef USE_ISC_LOG
        if (isc_log[which] != NULL) {
          t = mcp_systime(NULL);
          fprintf(isc_log[which], "%s: %i %i - %.4lf %.4lf %.4lf\n", ctime(&t),
              ISCSolution[which][iscdata_index[which]].framenum,
              ISCSolution[which][iscdata_index[which]].n_blobs,
              ISCSolution[which][iscdata_index[which]].ra * RAD2DEG,
              ISCSolution[which][iscdata_index[which]].dec * RAD2DEG,
              ISCSolution[which][iscdata_index[which]].sigma * RAD2DEG * 3600.);
          fflush(isc_log[which]);
        }
#endif
        /* Flag link as good, if necesary */
        if (!ISC_link_ok[which]) {
          bprintf(info, "Network link OK.\n");
          ISC_link_ok[which] = 1;
        }

        /* Wait for acknowledgement from camera before sening trigger */
        if (waiting_for_ACK) {
          if (WHICH)
            bprintf(info, "%iSC (i): Was waiting for ACK, flag was: %i (%i)\n",
                which, ISCSolution[which][iscwrite_index[which]].flag,
                ISCSolution[which][iscwrite_index[which]].framenum);
          if (ISCSolution[which][iscwrite_index[which]].flag == 0) {
            if (WHICH)
              bprintf(info, "%iSC (i): Raise write_ISC_trigger semaphore\n",
                  which);
            if (WHICH)
              bprintf(info, "%iSC (i): Stopped waiting for ACK\n", which);
            write_ISC_trigger[which] = 1;
            waiting_for_ACK = 0;
          }
        } else {
          if (WHICH)
            bprintf(info,
                "%iSC (i): Wasn't waiting for ACK, flag was: %i (%i)\n",
                which, ISCSolution[which][iscwrite_index[which]].flag,
                ISCSolution[which][iscwrite_index[which]].framenum);
          if (WHICH)
            bprintf(info, "%iSC (i): Raising start_ISC_cycle semaphore\n",
                which);
          start_ISC_cycle[which] = 1;
        }

        if (CommandData.ISCState[which].autofocus)
          CommandData.ISCState[which].autofocus = 0;

        /* increment pointers -- tx.c is responsible for incrementing
         * iscread_index, so we don't touch it here */
        if (ISCSolution[which][iscwrite_index[which]].flag)
          iscpoint_index[which] = iscwrite_index[which];
        iscwrite_index[which] = (iscwrite_index[which] + 1) % 5;
      }

      /* ------------ write ---------- */

      if (FD_ISSET(sock, &fdw) && (write_ISC_pointing[which])) {
        /* Retreive the derived pointing information */
        MyPointData = PointingData[GETREADINDEX(point_index)];

        if (CommandData.ISCControl[which].autofocus > 0) {
          CommandData.ISCState[which].autofocus = 1;
          CommandData.ISCControl[which].autofocus--;
        } else
          CommandData.ISCState[which].autofocus = 0;
        CommandData.ISCState[which].lat = MyPointData.lat * DEG2RAD;
        CommandData.ISCState[which].az = MyPointData.az * DEG2RAD;
        CommandData.ISCState[which].el = MyPointData.el * DEG2RAD;
        CommandData.ISCState[which].lst = MyPointData.lst * SEC2RAD;
        CommandData.ISCState[which].maxSlew = GetNominalVel(MyPointData);


        /* request for one automaticly saved image */
        if (CommandData.ISCControl[which].auto_save) {
          save_image_state = CommandData.ISCState[which].save;
          CommandData.ISCState[which].save = 1;
        }

        /* Write to ISC */
        if (InCharge) {
          n = send(sock, &CommandData.ISCState[which],
              sizeof(CommandData.ISCState[which]), 0);
          if (n == -1) {
            berror(err, "send()");
            break;
          } else if (n < sizeof(struct ISCStatusStruct)) {
            bprintf(err, "Expected %i but sent %i bytes.\n",
                (int) sizeof(struct ISCStatusStruct), n);
            break;
          }
          if (WHICH)
            bprintf(info,
                "%iSC (i): Lower write_ISC_pointing semaphore --------------\n",
                which);
          write_ISC_pointing[which] = 0;
#ifdef USE_ISC_LOG
          if (isc_log[which] != NULL) {
            t = mcp_systime(NULL);
            fprintf(isc_log[which],
                "%s: %i %i %i %i - %i %i %i %i - %.4lf %.4lf %.4lf %.4lf\n"
                "%f %.1lf %i %i %i %i - %.1f %.6f %.4f "
                "- %.4f %.4f %.4f %.4f\n\n",
                ctime(&t),
                CommandData.ISCState[which].pause,
                CommandData.ISCState[which].save,
                CommandData.ISCState[which].focus_pos,
                CommandData.ISCState[which].ap_pos,

                CommandData.ISCState[which].display_mode,
                CommandData.ISCState[which].roi_x,
                CommandData.ISCState[which].roi_y,
                CommandData.ISCState[which].blob_num,

                CommandData.ISCState[which].az,
                CommandData.ISCState[which].el,
                CommandData.ISCState[which].lst,
                CommandData.ISCState[which].lat,

                CommandData.ISCState[which].maxSlew,
                CommandData.ISCState[which].sn_threshold,
                CommandData.ISCState[which].grid,
                CommandData.ISCState[which].cenbox,
                CommandData.ISCState[which].apbox,
                CommandData.ISCState[which].mult_dist,

                CommandData.ISCState[which].mag_limit,
                CommandData.ISCState[which].norm_radius,
                CommandData.ISCState[which].lost_radius,

                CommandData.ISCState[which].tolerance,
                CommandData.ISCState[which].match_tol,
                CommandData.ISCState[which].quit_tol,
                CommandData.ISCState[which].rot_tol);
            fflush(isc_log[which]);
          }
#endif
        }

        ISCSentState[which] = CommandData.ISCState[which];

        /* Deassert abort, shutdown and set_focus after (perhaps) sending it */
        CommandData.ISCState[which].abort = 0;
        CommandData.ISCState[which].shutdown = 0;
        CommandData.ISCState[which].focus_pos = 0;

        /* Return to default save_image state after autosaving image */
        if (CommandData.ISCControl[which].auto_save) {
          CommandData.ISCState[which].save = save_image_state;
          CommandData.ISCControl[which].auto_save = 0;
        }

        waiting_for_ACK = 1;
      }
    }
  }
}
