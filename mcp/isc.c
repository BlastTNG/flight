#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include "pointing_struct.h"
#include "isc_protocol.h"

#define ELBERETH "192.168.1.42"
#define BASE_PORT 2000

extern short int SamIAm;  /* mcp.c */

server_frame ISCData[3];
int iscdata_index = 0;

void IntegratingStarCamera(void)
{
  client_frame client_data;

  int sock;

  int n;
  struct sockaddr_in addr;

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
//    fprintf(stderr, "ISC: connect failed.\n");
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

  for (;;) {
    n = recv(sock, &ISCData[iscdata_index], sizeof(server_frame), 0);
    if (n < sizeof(server_frame)) {
      fprintf(stderr, "ISC: Expected %i but received %i bytes.\n",
          sizeof(server_frame), n);
      return;
    }
    fprintf(stderr, "ISC: Received %i bytes.\n", n);
    iscdata_index = INC_INDEX(iscdata_index);

    usleep(500000);
  }
}
