/* mceserv: the MCE flight computer network server
 *
 * lorem ipsum dolor sit GPL!
 *
 */
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <string.h>

#include "mcp.h"
#include "mceserv.h"

#define SALUTATION "MCEserv: $Rev$\r\n"

/* initialise the network -- shouldn't there be a generic function for this
 * somewhere in the flight code? */
static int start(void)
{
  struct sockaddr_in addr;
  int sock, n;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    berror(warning, "Unable to create socket.");
    return -1;
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
    berror(warning, "Unable set socket options.");
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(MCESERV_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    berror(warning, "Unable to bind to port.");
    return -1;
  }

  if (listen(sock, 20) == -1) {
    berror(warning, "Unable to listen.");
    return -1;
  }

  bprintf(info, "Listening on port %i.", MCESERV_PORT);

  return sock;
}

/* main routine */
void *mceserv(void *unused)
{
  socklen_t addr_len; /* stoopid POSIX */
  struct sockaddr_in addr;
  int sock;
  int client[4] = {-1, -1, -1, -1};
  nfds_t nfds = 1;
  struct pollfd fds[4];

  nameThread("MCE");
  bputs(startup, SALUTATION);

  /* wait here until serving works */
  while ((sock = start()) < 0)
    sleep(1);

  fds[0].fd = sock;

  /* poll loop */
  for (;;) {
RESET:
    fds[0].events = POLLIN;
    fds[0].revents = 0;

    /* this blocks */
    int n = poll(fds, nfds, -1);
    if (n < 0) { /* poll() error? */
      berror(err, "poll");
      continue;
    } else if (n == 0) {
      /* nothing to do */
      continue;
    }

    /* handle incomming connections */
    if (fds[0].revents & POLLIN) {
      char buffer[1024] = SALUTATION;
      const size_t len = strlen(SALUTATION);
      size_t done = 0;
      struct pollfd new_fd;

      new_fd.fd = accept(fds[0].fd, (struct sockaddr *)&addr, &addr_len);
      new_fd.events = POLLOUT;
      new_fd.revents = 0;

      /* wait for the client to become ready */
      n = poll(&new_fd, 1, 300000);
      if (n <= 0) {
        if (n < 0)
          berror(err, "poll on accept");
        bprintf(warning, "Dropping lame client %s before handshake",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      }

      /* handshake */
      while (done < len) {
        n = write(new_fd.fd, buffer + done, len - done);
        if (n < 0) {
          bprintf(warning, "Dropping deaf client %s during handshake",
              inet_ntoa(addr.sin_addr));
          close(new_fd.fd);
          goto RESET;
        }
        done += n;
      }

      /* wait for the client to respond */
      new_fd.events = POLLIN;
      new_fd.revents = 0;
      n = poll(&new_fd, 1, 300000);
      if (n <= 0) {
        if (n < 0)
          berror(err, "poll on handshake");
        bprintf(warning, "Dropping mute client %s during handshake",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      }

      /* validate the client */
      n = read(new_fd.fd, buffer, 1024);
      if (n < 0) {
        bprintf(warning, "Dropping incognito client %s during handshake",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      } else if (n > 1023) {
        bprintf(warning, "Dropping co-dependent client %s during handshake",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      }

      /* parse the client response.  This should be:
       *    <client_spec> <hostname> <protovers>
       * where client_spec is one of:
       *   MPC - an MCC (they're indistinguishable to PCM)
       *   MAC - the MAC
       *   MON - for any other client that wants to kibitz
       *
       * A client is also allows to reply BAD PROTO here, if the server protocol
       * version isn't liked.  Not much we can do about that, but we'll report
       * it.
       */
      if (strncmp(buffer, "BAD PROTO", 9) == 0) {
        bprintf(err, "Client reset.  BAD PROTO from %s",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      } else if (buffer[0] != 'M') {
HANDSHAKE_FAIL:
        bprintf(err, "Unrecognised response from client %s.  Dropped.",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      }
    }
  }

  return NULL;
};
