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

short int write_ISC_pointing = 0; // isc.c

struct ISCSolutionStruct ISCSolution[3];
int iscdata_index = 0;
FILE* isc_log = NULL;

int ISCInit(void)
{
  int sock;
  struct sockaddr_in addr;

  int n;

  if (isc_log == NULL) {
    if ((isc_log = fopen("/tmp/mcp.isc.log", "a")) == NULL) {
      perror("ISC log fopen()");
    }
  }

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

  return sock;
}

void IntegratingStarCamera(void)
{
  fd_set fdr, fdw;
  struct PointingDataStruct MyPointData;

  int sock = -1, ISCReadIndex;
  time_t t;

  int n;

  struct timeval t1, t2;
  int delta;

  int pid = getpid();
  fprintf(stderr, ">> ISC startup on pid %i\n", pid);

  for (;;) {
    do {
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");

      sock = ISCInit();
      if (sock == -1) {
        //        fprintf(stderr, "ISC: connect failed.\n");
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

        t2 = t1;
        gettimeofday(&t1, NULL);
        delta = (t1.tv_sec - t2.tv_sec) * 1000000 + (t1.tv_usec - t2.tv_usec);
        //fprintf(stderr, "ISC: Received %i bytes after %f milliseconds.\n", n, (double)delta / 1000.);

        iscdata_index = INC_INDEX(iscdata_index);
      }

      /* ------------ write ---------- */ 

      if (FD_ISSET(sock, &fdw) && (write_ISC_pointing)) {
        /* Retreive the derived pointing information */
        MyPointData = PointingData[GETREADINDEX(point_index)];
        ISCReadIndex = GETREADINDEX(iscdata_index);

        CommandData.ISCState.lat = MyPointData.lat * DEG2RAD;
        CommandData.ISCState.az = MyPointData.az * DEG2RAD;
        CommandData.ISCState.el = MyPointData.el * DEG2RAD;
        CommandData.ISCState.lst = MyPointData.lst * SEC2RAD;

        /* Write to ISC */
        if (InCharge) {
          n = send(sock, &CommandData.ISCState, sizeof(CommandData.ISCState), 0);
          if (n == -1) {
            perror("ISC send()");
            break;
          } else if (n < sizeof(struct ISCStatusStruct)) {
            fprintf(stderr, "ISC: Expected %i but sent %i bytes.\n",
                sizeof(struct ISCStatusStruct), n);
            break;
          }
          write_ISC_pointing = 0;
          if (isc_log != NULL) {
            t = time(NULL);
            fprintf(isc_log, "%s: %i %i %i %i - %i %i %i %i - %.4lf %.4lf %.4lf %.4lf\n"
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
        }
      }
    }
  }
}
