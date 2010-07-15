/**************************************************************
 * sss source code
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/

#include "net.h"

int MakeSock(void) {
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    fprintf(stderr, "unable to create tcp socket\n");
    exit(1);
  }

  if (setsockopt(sock, SOL_TCP, TCP_NODELAY, &n, sizeof(n)) != 0) {
    fprintf(stderr, "unable to set socket options\n");
    exit(1);
  }

  //  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
  //    fprintf(stderr, "unable to set other socket options\n");
  //    exit(1);
  //  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    fprintf(stderr, "unable to bind to socket\n");
    exit(1);
  }

  if (listen(sock, 10) == -1) {
    fprintf(stderr, "unable to listen on port\n");
    exit(1);
  }

  fprintf(stderr, "Listening on port %i with socket %i.\n", SOCK_PORT, sock);

  return sock;
}

int SendData(sss_packet_data * data) {

  static int firsttime = 1;
  static int sock;
  static int csock;
  int n;
  int z;
  static int lastsock;
  static fd_set fdlist;
  fd_set fdread;
  fd_set fdwrite;
  struct sockaddr_in addr;
  socklen_t addrlen;
  struct timeval no_time = {0, 0};
  int num_connect = 0;

  if (firsttime)
  {
    lastsock = sock = MakeSock();
    FD_ZERO(&fdlist);
    firsttime = 0;
  }

  fdwrite = fdlist;
  FD_ZERO(&fdread);
  FD_SET(sock, &fdread);

  n = select(lastsock + 1, &fdread, &fdwrite, NULL, &no_time);

  if (n == -1 && errno == EINTR) {
    return num_connect;
  } else if (n == -1) {
    fprintf(stderr, "error on select\n");
    exit(1);
  }
  /* loop through all socket numbers, looking for ones that have been
   * returned by select */
  for (n = 0; n <= lastsock; ++n) {
    if (FD_ISSET(n, &fdread)) {     /* connextion n is waiting for read */
      if (n == sock) {
        /* listener has a new connexion */
        addrlen = sizeof(addr);
        if ((csock  = accept(sock, (struct sockaddr*)&addr, &addrlen))
            == -1) {
          fprintf(stderr, "error on accept\n");
          exit(1);
        }
        FD_SET(csock, &fdlist);
        if (csock > lastsock)
          lastsock = csock;
        fprintf(stderr, "connexion from %s accepted on socket %i\n",
            inet_ntoa(addr.sin_addr), csock);
      }
    }



    if (FD_ISSET(n, &fdwrite)) {     /* connexion n is waiting for write */
      if (n != sock) {   /* don't write to the listener */
        if ((z = send(n, data, sizeof(sss_packet_data), MSG_NOSIGNAL | MSG_DONTWAIT))
            == -1) {
          if (errno == EPIPE) {  /* connexion dropped */
            fprintf(stderr, "connexion dropped on socket %i\n", n);
            close(n);
            FD_CLR(n, &fdlist);
          } else if (errno != EAGAIN) {  /* ignore socket buffer overflows */
            fprintf(stderr, "send error on socket %i; closing connexion\n", n);
            close(n);
            FD_CLR(n, &fdlist);
          }
        } else {
          num_connect++;
#ifdef DEBUG
          fprintf(stderr, "wrote %i/%i bytes to socket %i\n", z, (int)sizeof(sss_packet_data), n);
#endif
        }
      }
    }
  }
  return num_connect;
}
