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
#define BASE_PORT 2001

extern short int SamIAm;  /* mcp.c */

short int write_ISC_pointing = 0;

server_frame ISCData[3];
client_frame ISCInput;
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

  /* Ask for defaults and start free run */
  client_data.command = freerun;
  n = send(sock, &client_data, sizeof(client_data), 0);
  if (n < sizeof(client_data)) {
    fprintf(stderr, "ISC: Expected %i bytes, but sent %i bytes.\n",
        sizeof(client_data), n);
    return;
  }

  /* Read defaults */
  n = recv(sock, &ISCData[iscdata_index], sizeof(server_frame), 0);
  if (n == -1) {
    perror("ISC recv()");
  } else if (n < sizeof(server_frame)) {
    fprintf(stderr, "ISC: Expected %i but received %i bytes.\n",
        sizeof(server_frame), n);
    return;
  }

  /* Fill client frames with defaults */
  client_data.gain = ISCData[iscdata_index].gain;
  client_data.exposure = ISCData[iscdata_index].exposure;
  client_data.platescale = ISCData[iscdata_index].platescale;
  client_data.gain = ISCData[iscdata_index].gain;
  client_data.offset = ISCData[iscdata_index].offset;
  client_data.saturation = ISCData[iscdata_index].saturation;
  client_data.threshold = ISCData[iscdata_index].threshold;
  client_data.grid = ISCData[iscdata_index].grid;
  client_data.cenbox = ISCData[iscdata_index].cenbox;
  client_data.apbox = ISCData[iscdata_index].apbox;
  client_data.multiple_dist = ISCData[iscdata_index].multiple_dist;

  ISCInput = CommandData.ISCCommand = client_data;

  /* Since we just trounced whatever data was queued to send (it would
   * have had uninitialised data in any case), deassert write semaphores. */
  write_ISC_pointing = CommandData.write_ISC_command = 0;
  
  /* Increment index, finally */
  iscdata_index = INC_INDEX(iscdata_index);

  fprintf(stderr, "ISC connect.\n");

  /* select loop */
  for (;;) {
    /* fill the file descriptor lists */
    FD_ZERO(&fdr);
    FD_ZERO(&fdw);

    /* ask select to poll for a non-blocking write if we have data to send */
    if (write_ISC_pointing || CommandData.write_ISC_command) {
      FD_SET(sock, &fdw);
    }

    /* If there is a command frame to write, we forget about reading for a
     * while.  This is to prevent us from clobbering possible command parameters
     * when we get the updated parameters from the ISC.
     * We're banking on the expectation that the time it takes to poll, write
     * and poll again is significantly faster than we're getting frames. */
    if (!CommandData.write_ISC_command) {
      FD_SET(sock, &fdr);
    }

    /* select poll on fds; this is a blocking call */
    n = select(sock + 1, &fdr, &fdw, NULL, NULL);
    printf("Select: %i %i %i\n", n, FD_ISSET(sock, &fdr), FD_ISSET(sock, &fdw));

    if (n == -1 && errno == EINTR)
      continue;
    if (n == -1) {
      perror("ISC select()");
      continue;
    }

    /* read */
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

      ISCInput.gain = ISCData[iscdata_index].gain;
      ISCInput.exposure = ISCData[iscdata_index].exposure;
      ISCInput.platescale = ISCData[iscdata_index].platescale;
      ISCInput.gain = ISCData[iscdata_index].gain;
      ISCInput.offset = ISCData[iscdata_index].offset;
      ISCInput.saturation = ISCData[iscdata_index].saturation;
      ISCInput.threshold = ISCData[iscdata_index].threshold;
      ISCInput.grid = ISCData[iscdata_index].grid;
      ISCInput.cenbox = ISCData[iscdata_index].cenbox;
      ISCInput.apbox = ISCData[iscdata_index].apbox;
      ISCInput.multiple_dist = ISCData[iscdata_index].multiple_dist;
    
      /* If we've received a command in the interrim, don't clobber it. */
      if (!CommandData.write_ISC_Command) {
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

    /* write */
    if (FD_ISSET(sock, &fdw)) {
      /* Retreive the derived pointing information */
      PointingIndex = GETREADINDEX(point_index);
      MyPointingData = PointingData[PointingIndex];
      ISCReadIndex = GETREADINDEX(iscdata_index);

      /* Fill client_frame */
      if (CommandData.write_ISC_command) {
        client_data = CommandData.ISCCommand;
        CommandData.write_ISC_command = 0;
      } else if (write_ISC_pointing) {
        client_data = ISCInput;
        write_ISC_pointing = 0;
      }

      /* Write to ISC */
      n = send(sock, &client_data, sizeof(client_data), 0);
      if (n == -1) {
        perror("ISC recv()");
      } else if (n < sizeof(client_frame)) {
        fprintf(stderr, "ISC: Expected %i but sent %i bytes.\n",
            sizeof(client_frame), n);
        return;
      }
    }
  }
}
