#include <sys/types.h>
#include <time.h>
#include <sys/socket.h>
#include <signal.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "ss_struct.h"
#include "pointing_struct.h"
#include "mcp.h"

#define ARIEN "192.168.62.7"
#define ARIEN_PORT 11235

ss_packet_data SunSensorData[3];
int ss_index = 0;

void SunSensor(void) {
  int sock = -1, n;


  fd_set fdr;
  struct timeval timeout;

  struct sockaddr_in addr;

  ss_packet_data Rx_Data;


  mputs(MCP_STARTUP, "SunSensor startup\n");

  while (1) {
    if (sock != -1)
      if (close(sock) == -1)
        merror(MCP_ERROR, "SunSensor close()");

    /* create an empty socket connection */
    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == -1)
      merror(MCP_TFATAL, "SunSensor socket()");

    /* set options */
    n = 1;
    if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) == -1)
      merror(MCP_TFATAL, "SunSensor setsockopt()");

    /* Connect to Arien */
    inet_aton(ARIEN, &addr.sin_addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(ARIEN_PORT);
    while ((n = connect(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)))
        < 0) {
      merror(MCP_ERROR, "SunSensor connect()");
      sleep(10);
    };

    mputs(MCP_INFO, "Connected to Arien\n");
    n = 0;

    while (n != -1) {
      usleep(10000);

      FD_ZERO(&fdr);
      FD_SET(sock, &fdr);

      timeout.tv_sec = 10;
      timeout.tv_usec = 0;

      n = select(sock + 1, &fdr, NULL, NULL, &timeout);

      if (n == -1 && errno == EINTR) {
        mputs(MCP_WARNING, "timeout on Sun Sensor\n");
        continue;
      }
      if (n == -1) {
        merror(MCP_ERROR, "SunSensor select()");
        continue;
      }

      if (FD_ISSET(sock, &fdr)) {
        n = recv(sock, &Rx_Data, sizeof(Rx_Data), MSG_DONTWAIT);
        printf("Rx: %f\n", Rx_Data.az_center);
        if (n == sizeof(Rx_Data)) {
          SunSensorData[ss_index] = Rx_Data;
          ss_index = INC_INDEX(ss_index);
        } else if (n == -1) {
          merror(MCP_ERROR, "SunSensor recv()");
        } else if (n == 0) {
          mprintf(MCP_ERROR, "Connection to Arien closed");
          n = -1;
        } else {
          mputs(MCP_ERROR, "Didn't receive all data from Sun Sensor.\n");
        }
      } else {
        mputs(MCP_WARNING, "Connection to Arien timed out.\n");
        n = -1;
      }
    }
    }
  }

