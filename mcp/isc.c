#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include "pointing_struct.h"
#include "isc_protocol.h"

#define ELBERETH "192.168.1.98"
#define BASE_PORT 2001

extern short int SamIAm;  /* mcp.c */

short int write_ISC = 0;

server_frame ISCData[3];
int iscdata_index = 0;

void IntegratingStarCamera(void)
{
  client_frame client_data;
  fd_set fdr, fdw;
  struct PointingDataStruct MyPointingData;

  int sock, PointingIndex, ISCReadIndex;

  int n;
  struct sockaddr_in addr;

  struct timeval t1, t2;
  int delta;

  fprintf(stderr, "ISC startup.\n");

  sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == -1) {
    fprintf(stderr, "ISC: socket creation failed.\n");
    return;
  }

  n = 1;
  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    fprintf(stderr, "ISC: setsockopt failed.\n");
    return;
  }

  inet_aton(ELBERETH, &addr.sin_addr);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(BASE_PORT + SamIAm);

  while ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
      < 0) {
    perror("ISC connect()");

    sleep(1);
  }

  /* Start continuous scan */
  client_data.command = freerun;
  n = send(sock, &client_data, sizeof(client_data), 0);
  if (n < sizeof(client_data)) {
    fprintf(stderr, "ISC: Expected %i bytes, but sent %i bytes.\n",
        sizeof(client_data), n);
    return;
  }

  fprintf(stderr, "ISC connect.\n");

  for (;;) {
    /* fill the file descriptor lists */
    FD_ZERO(&fdr);
    FD_ZERO(&fdw);
    FD_SET(sock, &fdr);

    if (write_ISC) {
      FD_SET(sock, &fdw);
    }

    n = select(sock + 1, &fdr, &fdw, NULL, NULL);
    printf("Select: %i %i %i\n", n, FD_ISSET(sock, &fdr), FD_ISSET(sock, &fdw));

    if (n == -1 && errno == EINTR)
      continue;
    if (n == -1) {
      perror("ISC select()");
      continue;
    }

    if (FD_ISSET(sock, &fdr)) {
      printf("Read\n");
      n = recv(sock, &ISCData[iscdata_index], sizeof(server_frame), 0);
      if (n == -1) {
        perror("ISC recv()");
      } else if (n < sizeof(server_frame)) {
        fprintf(stderr, "ISC: Expected %i but received %i bytes.\n",
            sizeof(server_frame), n);
        return;
      }
      t2 = t1;
      gettimeofday(&t1, NULL);
      delta = (t1.tv_sec - t2.tv_sec) * 1000000 + (t1.tv_usec - t2.tv_usec);
      fprintf(stderr, "ISC: Received %i bytes after %f milliseconds.\n", n,
          (double)delta / 1000.);
      iscdata_index = INC_INDEX(iscdata_index);
    }
    if (FD_ISSET(sock, &fdw)) {
      /* Retreive the derived pointing information */
      PointingIndex = GETREADINDEX(point_index);
      MyPointingData = PointingData[PointingIndex];
      ISCReadIndex = GETREADINDEX(iscdata_index);

      /* Fill client_frame */
      client_data.command = freerun;
      client_data.az = MyPointingData.az;
      client_data.el = MyPointingData.el;
      client_data.lat = MyPointingData.lat;
      client_data.lst = MyPointingData.lst;
      client_data.exposure = ISCData[ISCReadIndex].exposure;  /* ??? */
      client_data.gyro_speed = 0;
      client_data.platescale = ISCData[ISCReadIndex].platescale;
      client_data.gain = ISCData[ISCReadIndex].gain;
      client_data.offset = ISCData[ISCReadIndex].offset;
      client_data.saturation = ISCData[ISCReadIndex].saturation;
      client_data.threshold = ISCData[ISCReadIndex].threshold;
      client_data.grid = ISCData[ISCReadIndex].grid;
      client_data.cenbox = ISCData[ISCReadIndex].cenbox;
      client_data.apbox = ISCData[ISCReadIndex].apbox;
      client_data.multiple_dist = ISCData[ISCReadIndex].multiple_dist;
      client_data.par1 = client_data.par2 = 
        client_data.par3 = client_data.par4 = 0;

      /* Write to ISC */

      /* Clear write list */
      write_ISC = 0;
      FD_ZERO(&fdw);
    }
  }
}
