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

#define ELBERETH "192.168.1.98"
#define BASE_PORT 2000

extern short int SamIAm;   /* mcp.c */
extern short int InCharge; /* tx.c */
extern int frame_num;      /* tx.c */

short int write_ISC_pointing = 0; // isc.c

struct ISCStatusStruct SentState;

struct ISCSolutionStruct ISCSolution[3];
int iscdata_index = 0;

#ifdef USE_ISC_LOG
FILE* isc_log = NULL;
#endif

int ISCInit(void)
{
  int sock;
  struct sockaddr_in addr;

  int n;

#ifdef USE_ISC_LOG
  if (isc_log == NULL) {
    if ((isc_log = fopen("/tmp/mcp.isc.log", "a")) == NULL) {
      perror("ISC log fopen()");
    }
  }
#endif

  sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == -1) {
    perror("ISC socket()");
    return -1;
  }

  n = 1;
  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    perror("ISC setsockopt()");
    if (sock != -1)
      if (close(sock) < 0)
        perror("ISC close()");
    return -1;
  }

  inet_aton(ELBERETH, &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(BASE_PORT + SamIAm);

  if ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
      < 0) {
    perror("ISC connect()");
    if (sock != -1)
      if (close(sock) < 0)
        perror("ISC close()");
    return -1;
  }

  fprintf(stderr, "Connected to Elbereth\n");
  CommandData.ISCState.shutdown = 0;

  SentState = CommandData.ISCState;

  return sock;
}

void IntegratingStarCamera(void)
{
  fd_set fdr, fdw;
  struct PointingDataStruct MyPointData;

  int sock = -1, ISCReadIndex;
  static int come_down_to = -1;

#ifdef USE_ISC_LOG
  time_t t;
#endif

  int n, save_image_state = 0;

//  struct timeval t1, t2;
//  int delta;

  int pid = getpid();
  fprintf(stderr, ">> ISC startup on pid %i\n", pid);

  for (;;) {
    do {
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");

      sock = ISCInit();
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
      if (CommandData.ISC_reconnect) {
        CommandData.ISC_reconnect = 0;
        break;
      }

      usleep(10000);
      /* select poll on fds; this is a blocking call */
      n = select(sock + 1, &fdr, &fdw, NULL, NULL);

      if (n == -1 && errno == EINTR)
        continue;
      if (n == -1) {
        perror("ISC select()");
        continue;
      }

      /* ------------ read ---------- */ 

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, &ISCSolution[iscdata_index], sizeof(struct ISCSolutionStruct), 0);
        if (n == -1) {
          perror("ISC recv()");
          break;
        } else if (n < sizeof(struct ISCSolutionStruct)) {
          fprintf(stderr, "ISC: Expected %i but received %i bytes.\n",
              sizeof(struct ISCSolutionStruct), n);
          break;
        }
#ifdef USE_ISC_LOG
        if (isc_log != NULL) {
          t = time(NULL);
          fprintf(isc_log, "%s: %i %i - %.4lf %.4lf %.4lf\n", ctime(&t),
              ISCSolution[iscdata_index].framenum,
              ISCSolution[iscdata_index].n_blobs,
              ISCSolution[iscdata_index].ra * RAD2DEG,
              ISCSolution[iscdata_index].dec * RAD2DEG,
              ISCSolution[iscdata_index].sigma * RAD2DEG * 3600.);
          fflush(isc_log);
        }
#endif

        //t2 = t1;
        //gettimeofday(&t1, NULL);
        //delta = (t1.tv_sec - t2.tv_sec) * 1000000 + (t1.tv_usec - t2.tv_usec);
        //fprintf(stderr, "ISC: Received %i bytes after %f milliseconds.\n", n, (double)delta / 1000.);
        
        if (CommandData.ISCState.autofocus) {
          CommandData.ISCState.autofocus = 0;
          CommandData.ISCState.focus_pos = CommandData.old_ISC_focus;
        }

        /* Process results of autofocus */
        if (come_down_to >= 0) {
          CommandData.ISCState.focus_pos = come_down_to;
          come_down_to = -1;
        } else if (ISCSolution[iscdata_index].autoFocusPosition > 2000 &&
            ISCSolution[iscdata_index].autoFocusPosition < FOCUS_RANGE &&
            CommandData.ISCState.focus_pos !=
            ISCSolution[iscdata_index].autoFocusPosition) {
          come_down_to = ISCSolution[iscdata_index].autoFocusPosition;
          CommandData.ISCState.focus_pos = FOCUS_RANGE;
      }

      iscdata_index = INC_INDEX(iscdata_index);
    }

    /* ------------ write ---------- */ 

    if (FD_ISSET(sock, &fdw) && (write_ISC_pointing)) {
      /* Retreive the derived pointing information */
      MyPointData = PointingData[GETREADINDEX(point_index)];
      ISCReadIndex = GETREADINDEX(iscdata_index);

      if (CommandData.ISC_autofocus > 0) {
        CommandData.ISCState.autofocus = 1;
        CommandData.ISC_autofocus--;
      } else
        CommandData.ISCState.autofocus = 0;
      CommandData.ISCState.lat = MyPointData.lat * DEG2RAD;
      CommandData.ISCState.az = MyPointData.az * DEG2RAD;
      CommandData.ISCState.el = MyPointData.el * DEG2RAD;
      CommandData.ISCState.lst = MyPointData.lst * SEC2RAD;
      CommandData.ISCState.MCPFrameNum = frame_num;

      /* request for one automaticly saved image */
      if (CommandData.ISC_auto_save) {
        save_image_state = CommandData.ISCState.save;
        CommandData.ISCState.save = 1;
      }

      /* Write to ISC */
      if (InCharge) {
        n = send(sock, &CommandData.ISCState, sizeof(CommandData.ISCState),
            0);
        if (n == -1) {
          perror("ISC send()");
          break;
        } else if (n < sizeof(struct ISCStatusStruct)) {
          fprintf(stderr, "ISC: Expected %i but sent %i bytes.\n",
              sizeof(struct ISCStatusStruct), n);
          break;
        }
        write_ISC_pointing = 0;
#ifdef USE_ISC_LOG
        if (isc_log != NULL) {
          t = time(NULL);
          fprintf(isc_log,
              "%s: %i %i %i %i - %i %i %i %i - %.4lf %.4lf %.4lf %.4lf\n"
              "%.1lf %i %i %i %i - %.1f %.6f %.4f - %.4f %.4f %.4f %.4f\n\n",
              ctime(&t),
              CommandData.ISCState.pause, CommandData.ISCState.save,
              CommandData.ISCState.focus_pos, CommandData.ISCState.ap_pos,
              CommandData.ISCState.display_mode, CommandData.ISCState.roi_x,
              CommandData.ISCState.roi_y, CommandData.ISCState.blob_num,
              CommandData.ISCState.az, CommandData.ISCState.el,
              CommandData.ISCState.lst, CommandData.ISCState.lat,
              CommandData.ISCState.sn_threshold,
              CommandData.ISCState.grid, CommandData.ISCState.cenbox,
              CommandData.ISCState.apbox, CommandData.ISCState.mult_dist,
              CommandData.ISCState.mag_limit,
              CommandData.ISCState.norm_radius,
              CommandData.ISCState.lost_radius,
              CommandData.ISCState.tolerance, CommandData.ISCState.match_tol,
              CommandData.ISCState.quit_tol, CommandData.ISCState.rot_tol);
          fflush(isc_log);
        }
#endif
      }

      SentState = CommandData.ISCState;

      /* Deassert abort and shutdown after (perhaps) sending it */
      CommandData.ISCState.abort = 0;
      CommandData.ISCState.shutdown = 0;

      /* Return to default save_image state after autosaving image */
      if (CommandData.ISC_auto_save) {
        CommandData.ISCState.save = save_image_state;
        CommandData.ISC_auto_save = 0;
      }
    }
  }
}
}
