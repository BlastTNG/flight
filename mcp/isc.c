/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2004 University of Toronto
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
#include <sys/time.h>

#include "command_struct.h"
#include "pointing_struct.h"
#include "isc_protocol.h"
#include "mcp.h"

#define BASE_PORT 2000

//#define USE_ISC_LOG

struct {
  char who[4];
  char where[16];
  char what[5];
} isc_which[2] = {
  {"Isc", "192.168.62.5", "isc "},
  {"Osc", "192.168.62.6", "osc "}
};

extern short int SamIAm;   /* mcp.c */
extern short int InCharge; /* tx.c */
extern int frame_num;      /* tx.c */

short int write_ISC_pointing[2] = {0, 0}; // isc.c

struct ISCStatusStruct ISCSentState[2];

struct ISCSolutionStruct ISCSolution[2][3];
int iscdata_index[2] = {0, 0};

#ifdef USE_ISC_LOG
FILE* isc_log[2] = {NULL, NULL};
#endif

int ISCInit(int which)
{
  int sock;
  struct sockaddr_in addr;

  int n;

#ifdef USE_ISC_LOG
  if (which) {
    if (isc_log[which] == NULL)
      if ((isc_log[which] = fopen("/tmp/isc.1.log", "a")) == NULL)
        berror(err, "%s log fopen()", isc_which[which].who);
  } else {
    if (isc_log[which] == NULL)
      if ((isc_log[which] = fopen("/tmp/isc.0.log", "a")) == NULL)
        berror(err, "%s log fopen()", isc_which[which].who);
  }
  fprintf(isc_log[which], "This is %s.\n", isc_which[which].who);
#endif

  sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == -1) {
    berror(err, "%s socket()", isc_which[which].who);
    return -1;
  }

  n = 1;
  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    berror(err, "%s setsockopt()", isc_which[which].who);
    if (sock != -1)
      if (close(sock) < 0)
        berror(err, "%s close()", isc_which[which].who);
    return -1;
  }

  inet_aton(isc_which[which].where, &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(BASE_PORT + SamIAm);

  if ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
      < 0) {
    berror(err, "%s connect()", isc_which[which].who);
    if (sock != -1)
      if (close(sock) < 0)
        berror(err, "%s close()", isc_which[which].who);
    return -1;
  }

  bprintf(info, "Connected to %s\n", isc_which[which].who);
  CommandData.ISCState[which].shutdown = 0;

  ISCSentState[which] = CommandData.ISCState[which];

  return sock;
}

void IntegratingStarCamera(void* parameter)
{
  fd_set fdr, fdw;
  struct PointingDataStruct MyPointData;
  int which = (int)parameter;

  int sock = -1, ISCReadIndex;
  static int come_down_to = -1;

#ifdef USE_ISC_LOG
  time_t t;
#endif

  int n, save_image_state = 0;

  pthread_setspecific(identity, isc_which[which].what);
  bprintf(startup, "%s startup\n", isc_which[which].who);

  for (;;) {
    do {
      if (sock != -1)
        if (close(sock) < 0)
          berror(err, "%s close()", isc_which[which].who);

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
        berror(err, "%s select()", isc_which[which].who);
        continue;
      }

      /* ------------ read ---------- */ 

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, &ISCSolution[which][iscdata_index[which]],
            sizeof(struct ISCSolutionStruct), 0);
        if (n == -1) {
          berror(err, "%s recv()", isc_which[which].who);
          break;
        } else if (n < sizeof(struct ISCSolutionStruct)) {
          bprintf(err, "ISC: Expected %i but received %i bytes.\n",
              sizeof(struct ISCSolutionStruct), n);
          break;
        }
#ifdef USE_ISC_LOG
        if (isc_log[which] != NULL) {
          t = time(NULL);
          fprintf(isc_log[which], "%s: %i %i - %.4lf %.4lf %.4lf\n", ctime(&t),
              ISCSolution[which][iscdata_index[which]].framenum,
              ISCSolution[which][iscdata_index[which]].n_blobs,
              ISCSolution[which][iscdata_index[which]].ra * RAD2DEG,
              ISCSolution[which][iscdata_index[which]].dec * RAD2DEG,
              ISCSolution[which][iscdata_index[which]].sigma * RAD2DEG * 3600.);
          fflush(isc_log[which]);
        }
#endif

        if (CommandData.ISCState[which].autofocus) {
          CommandData.ISCState[which].autofocus = 0;
          CommandData.ISCState[which].focus_pos
            = CommandData.ISCControl[which].old_focus;
        }

        /* Process results of autofocus */
        if (come_down_to >= 0) {
          CommandData.ISCState[which].focus_pos = come_down_to;
          come_down_to = -1;
        } else if (ISCSolution[which][iscdata_index[which]].autoFocusPosition
            > 2000 && ISCSolution[which][iscdata_index[which]].autoFocusPosition
            < FOCUS_RANGE && CommandData.ISCState[which].focus_pos !=
            ISCSolution[which][iscdata_index[which]].autoFocusPosition) {
          come_down_to
            = ISCSolution[which][iscdata_index[which]].autoFocusPosition;
          CommandData.ISCState[which].focus_pos = FOCUS_RANGE;
        }

        iscdata_index[which] = INC_INDEX(iscdata_index[which]);
      }

      /* ------------ write ---------- */ 

      if (FD_ISSET(sock, &fdw) && (write_ISC_pointing[which])) {
        /* Retreive the derived pointing information */
        MyPointData = PointingData[GETREADINDEX(point_index)];
        ISCReadIndex = GETREADINDEX(iscdata_index[which]);

        if (CommandData.ISCControl[which].autofocus > 0) {
          CommandData.ISCState[which].autofocus = 1;
          CommandData.ISCControl[which].autofocus--;
        } else
          CommandData.ISCState[which].autofocus = 0;
        CommandData.ISCState[which].lat = MyPointData.lat * DEG2RAD;
        CommandData.ISCState[which].az = MyPointData.az * DEG2RAD;
        CommandData.ISCState[which].el = MyPointData.el * DEG2RAD;
        CommandData.ISCState[which].lst = MyPointData.lst * SEC2RAD;
        CommandData.ISCState[which].MCPFrameNum = frame_num;

        /* az kludge XXX */
        CommandData.ISCState[which].az = 180 * DEG2RAD;

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
            berror(err, "%s send()", isc_which[which].who);
            break;
          } else if (n < sizeof(struct ISCStatusStruct)) {
            bprintf(err, "ISC: Expected %i but sent %i bytes.\n",
                sizeof(struct ISCStatusStruct), n);
            break;
          }
          write_ISC_pointing[which] = 0;
#ifdef USE_ISC_LOG
          if (isc_log[which] != NULL) {
            t = time(NULL);
            fprintf(isc_log[which],
                "%s: %i %i %i %i - %i %i %i %i - %.4lf %.4lf %.4lf %.4lf\n"
                "%.1lf %i %i %i %i - %.1f %.6f %.4f - %.4f %.4f %.4f %.4f\n\n",
                ctime(&t),
                CommandData.ISCState[which].pause,
                CommandData.ISCState[which].save,
                CommandData.ISCState[which].focus_pos,
                CommandData.ISCState[which].ap_pos,
                CommandData.ISCState[which].display_mode,
                CommandData.ISCState[which].roi_x,
                CommandData.ISCState[which].roi_y,
                CommandData.ISCState[which].blob_num,
                CommandData.ISCState[which].az, CommandData.ISCState[which].el,
                CommandData.ISCState[which].lst,
                CommandData.ISCState[which].lat,
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
      }
    }
  }
}
