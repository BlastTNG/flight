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

server_frame ISCData[3];
int iscdata_index = 0;

int ISCInit(client_frame* client_data)
{
  fd_set fds;
  struct timeval timeout;

  int sock;
  struct sockaddr_in addr;

  int n;

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

  if (InCharge) {
    FD_ZERO(&fds);
    FD_SET(sock, &fds);

    timeout.tv_sec = 10;
    timeout.tv_usec = 0;

    n = select(sock + 1, NULL, &fds, NULL, &timeout);

    if (n == -1 && errno == EINTR) {
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");
      return -1;
    }
    if (n == -1) {
      perror("ISC select()");
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");
      return -1;
    }

    if (FD_ISSET(sock, &fds)) {
      /* Ask for defaults and start free run */
      client_data->command = freerun;

      n = send(sock, client_data, sizeof(client_frame), 0);
      if (n == -1) {
        perror("ISC send()");
        if (sock != -1)
          if (close(sock) < 0)
            perror("ISC close()");
        return -1;
      } else if (n < sizeof(client_frame)) {
        fprintf(stderr, "ISC: Expected %i bytes, but sent %i bytes.\n",
            sizeof(client_frame), n);
        if (sock != -1)
          if (close(sock) < 0)
            perror("ISC close()");
        return -1;
      }
    } else {
      fprintf(stderr, "ISC: Time out waiting for CTS\n");
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");
      return -1;
    }
  } else {
    fprintf(stderr, "ISC: Not in charge\n");
  }

  /* Read defaults */
  FD_ZERO(&fds);
  FD_SET(sock, &fds);

  timeout.tv_sec = 10;
  timeout.tv_usec = 0;

  n = select(sock + 1, &fds, NULL, NULL, &timeout);

  if (n == -1 && errno == EINTR) {
    if (sock != -1)
      if (close(sock) < 0)
        perror("ISC close()");
    return -1;
  }
  if (n == -1) {
    perror("ISC select()");
    if (sock != -1)
      if (close(sock) < 0)
        perror("ISC close()");
    return -1;
  }

  if (FD_ISSET(sock, &fds)) {
    n = recv(sock, &ISCData[iscdata_index], sizeof(server_frame), 0);
    if (n == -1) {
      perror("ISC recv()");
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");
      return -1;
    } else if (n < sizeof(server_frame)) {
      fprintf(stderr, "ISC: Expected %i but received %i bytes.\n",
          sizeof(server_frame), n);
      if (sock != -1)
        if (close(sock) < 0)
          perror("ISC close()");
      return -1;
    }

    /* Fill client frames with defaults */
    client_data->gain = ISCData[iscdata_index].gain;
    client_data->exposure = ISCData[iscdata_index].exposure;
    client_data->platescale = ISCData[iscdata_index].platescale;
    client_data->gain = ISCData[iscdata_index].gain;
    client_data->offset = ISCData[iscdata_index].offset;
    client_data->saturation = ISCData[iscdata_index].saturation;
    client_data->threshold = ISCData[iscdata_index].threshold;
    client_data->grid = ISCData[iscdata_index].grid;
    client_data->cenbox = ISCData[iscdata_index].cenbox;
    client_data->apbox = ISCData[iscdata_index].apbox;
    client_data->multiple_dist = ISCData[iscdata_index].multiple_dist;

    CommandData.ISCCommand = *client_data;

    /* Since we just trounced whatever data was queued to send (it would
     * have had uninitialised data in any case), deassert write semaphores. */
    write_ISC_pointing = CommandData.write_ISC_command = 0;

    /* Increment index, finally */
    iscdata_index = INC_INDEX(iscdata_index);

    fprintf(stderr, "ISC connect.\n");
  } else {
    fprintf(stderr, "ISC: Time out waiting for RTS\n");
    if (sock != -1)
      if (close(sock) < 0)
        perror("ISC close()");
    return -1;
  }


  return sock;
}

void IntegratingStarCamera(void)
{
  client_frame client_data;
  fd_set fdr, fdw;
  struct PointingDataStruct MyPointData;

  int sock = -1, ISCReadIndex;

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

      sock = ISCInit(&client_data);
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

      /* If there is a command frame to write, we forget about reading for a
       * while.  This is to prevent us from clobbering possible command
       * parameters when we get the updated parameters from the ISC.
       * We're banking on the expectation that the time it takes to poll, write
       * and poll again is significantly faster than we're getting frames. */
      if (FD_ISSET(sock, &fdr) && !CommandData.write_ISC_command) {
        n = recv(sock, &ISCData[iscdata_index], sizeof(server_frame), 0);
        if (n == -1) {
          perror("ISC recv()");
          break;
        } else if (n < sizeof(server_frame)) {
          fprintf(stderr, "ISC: Expected %i but received %i bytes.\n",
              sizeof(server_frame), n);
          break;
        }

        t2 = t1;
        gettimeofday(&t1, NULL);
        delta = (t1.tv_sec - t2.tv_sec) * 1000000 + (t1.tv_usec - t2.tv_usec);
        //fprintf(stderr, "ISC: Received %i bytes after %f milliseconds.\n", n, (double)delta / 1000.);

        /* If we've received a command in the interrim, don't clobber it. */
        if (!CommandData.write_ISC_command) {
          CommandData.ISCCommand.gain = ISCData[iscdata_index].gain;
          CommandData.ISCCommand.exposure = ISCData[iscdata_index].exposure;
          CommandData.ISCCommand.platescale = ISCData[iscdata_index].platescale;
          CommandData.ISCCommand.gain = ISCData[iscdata_index].gain;
          CommandData.ISCCommand.offset = ISCData[iscdata_index].offset;
          CommandData.ISCCommand.saturation = ISCData[iscdata_index].saturation;
          CommandData.ISCCommand.threshold = ISCData[iscdata_index].threshold;
          CommandData.ISCCommand.grid = ISCData[iscdata_index].grid;
          CommandData.ISCCommand.cenbox = ISCData[iscdata_index].cenbox;
          CommandData.ISCCommand.apbox = ISCData[iscdata_index].apbox;
          CommandData.ISCCommand.multiple_dist =
            ISCData[iscdata_index].multiple_dist;
        }

        iscdata_index = INC_INDEX(iscdata_index);
      }

      /* ------------ write ---------- */ 

      if (FD_ISSET(sock, &fdw) &&
          (write_ISC_pointing || CommandData.write_ISC_command)) {
        /* Retreive the derived pointing information */
        MyPointData = PointingData[GETREADINDEX(point_index)];
        ISCReadIndex = GETREADINDEX(iscdata_index);

        /* Fill client_frame */
        if (CommandData.write_ISC_command) {
          fprintf(stderr, "ISC: Getting client frame from CommandData\n");
          client_data = CommandData.ISCCommand;
          CommandData.write_ISC_command = 0;
        } else if (write_ISC_pointing) {
          client_data.command = nocmd;
          write_ISC_pointing = 0;
        }

        client_data.az = MyPointData.az * DEG2RAD;
        client_data.el = MyPointData.el * DEG2RAD;
        client_data.lat = MyPointData.lat * DEG2RAD;
        client_data.lst = MyPointData.lst * SEC2RAD;

        /* Write to ISC */
        if (InCharge) {
          n = send(sock, &client_data, sizeof(client_data), 0);
          if (n == -1) {
            perror("ISC send()");
            break;
          } else if (n < sizeof(client_frame)) {
            fprintf(stderr, "ISC: Expected %i but sent %i bytes.\n",
                sizeof(client_frame), n);
            break;
          }
          //        fprintf(stderr, "ISC: Sent %i bytes.\n", n);
        }
      }
    }
  }
}
