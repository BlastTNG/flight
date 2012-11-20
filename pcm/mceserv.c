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
#include <stdlib.h>

#include "mcp.h"
#include "command_struct.h"
#include "mceserv.h"

/* desecrate the C preprocessor to extract this file's SVN revision */

/* Fun fact: GCC's CPP allows $ in identifier names */
#define $Rev (0?0 /* eat a : here */
#define $    )

const static inline int ProtoRev(void) { return $Rev$; };

#undef $Rev
#undef $
/* end preprocessor desecration */

#define CLI_NONE -1
#define CLI_MPC1 1
#define CLI_MPC2 2
#define CLI_MPC3 3
#define CLI_MAC  4
#define CLI_MON  5
static const char *cli_name[] = {NULL, "MPC1", "MPC2", "MPC3", "MAC", "MON"};

/* reverse lookup on an integer array */
static inline int FindInt(int v, const int *a, size_t l)
{
  size_t i;
  for (i = 0; i < l; ++i)
    if (v == a[i])
      return v;
  return -1;
}

/* check a client revision number.  Also isolate the hostname */
static int GetRev(char *buffer)
{
  char *end = NULL;
  int rev;
  char *ptr = buffer + 5;

  /* find first space */
  while (*ptr && *ptr != ' ')
    ptr++;

  /* no space */
  if (!*ptr)
    return -1;

  *(ptr++) = 0;

  /* skip all the spaces */
  while (*ptr && *ptr == ' ')
    ptr++;

  if (!*ptr)
    return -1;

  /* convert revision number */
  rev = (int)strtol(ptr, &end, 10);
  if (rev <= 0)
    return -1;

  /* revision number should be ended by a \r\n */
  if (*end != '\r')
    return -1;

  return rev;
}

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

/* send a command if one is pending */
static int ForwardCommand(const struct pollfd *fds, int *ready,
    const int *client)
{
  int i;
  int32_t i32;
  size_t len, sent[5] = {0, 0, 0, 0, 0};
  char *ptr, buffer[1024];
  const int cmd_idx = GETREADINDEX(CommandData.mcecmd_index);
  struct ScheduleEvent ev;

  /* all this saves us is potentially a useless memcpy */
  if (CommandData.mcecmd[cmd_idx].done || CommandData.mcecmd[cmd_idx].t == -1)
    return 0;

  memcpy(&ev, CommandData.mcecmd + cmd_idx, sizeof(ev));

  /* mitigate race conditions */
  if (ev.done || ev.t == -1)
    return 0;

  /* compose the command for transfer. */
  if (ev.is_multi) {
    sprintf(buffer, "CMD m %3i ", ev.command);
    ptr = buffer + 10;
    for (i = 0; i < mcommands[ev.t].numparams; ++i) {
      switch (mcommands[ev.t].params[i].type) {
        case 'i':
        case 'l':
          *(ptr++) = 'N';
          i32 = (int32_t)ev.ivalues + i;
          memcpy(ptr, &i32, sizeof(i32));
          ptr += sizeof(i32);
          break;
        case 'f':
        case 'd':
          *(ptr++) = 'R';
          memcpy(ptr, ev.rvalues + i, sizeof(double));
          ptr += sizeof(double);
          break;
        case 's':
          *(ptr++) = 'T';
          memcpy(ptr, ev.svalues + i, 32);
          ptr += 32;
          break;
      }
      *(ptr++) = '\r';
      *(ptr++) = '\n';
      *ptr = 0;
    }
  } else {
    sprintf(buffer, "CMD s %3i\r\n", ev.command);
  }

  len = strlen(buffer);
  /* "Broadcast" this to everyone */
  for (i = 1; i < 5; ++i) {
    /* theoretically there should be an iteration counter here */
    while (ready[i] && sent[i] < len) {
      ssize_t n = write(fds[i].fd, buffer + sent[i], len - sent[i]);
      if (n <= 0) {
        ready[i] = 0;
        if (n < 0)
          berror(warning, "error writing to client %s", cli_name[client[i]]);
      }
      if ((sent[i] += n) == len) {
        bprintf(info, "Transmitted %s command #%i to %s.\n",
            ev.is_multi ? "multi" : "single", ev.command, cli_name[client[i]]);
      }
    }
  }

  /* mark as written */
  CommandData.mcecmd[cmd_idx].done = 1;

  /* indicate something has been sent */
  return 1;
}

/* main routine */
void *mceserv(void *unused)
{
  socklen_t addr_len; /* stoopid POSIX */
  struct sockaddr_in addr;
  char buffer[1024];
  int client[5] = {0, -1, -1, -1, -1};
  int ready[5];
  int i;
  nfds_t nfds = 1;
  struct pollfd fds[5];
  const int proto_rev = ProtoRev();

  nameThread("MCE");
  bprintf(startup, "Startup");
  bprintf(info, "Protocol revision: %i", proto_rev);

  /* wait here until serving works */
  while ((fds[0].fd = start()) < 0)
    sleep(1);

  /* poll loop */
  for (;;) {
RESET:
    usleep(10000);
    fds[0].events = POLLIN;
    memset(ready, 0, sizeof(int) * 5);
    for (i = 1; i < nfds; ++i)
      fds[i].events = POLLIN | POLLOUT;

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
      sprintf(buffer, "MCEserv %i\r\n", proto_rev);
      const size_t len = strlen(buffer);
      size_t done = 0;
      struct pollfd new_fd;

      new_fd.fd = accept(fds[0].fd, (struct sockaddr *)&addr, &addr_len);
      new_fd.events = POLLOUT;
      new_fd.revents = 0;

      /* wait for the client to become ready */
      n = poll(&new_fd, 1, 10000);
      if (n <= 0) {
        if (n < 0)
          berror(err, "poll on accept");
        bprintf(warning, "Dropping lame client %s before handshake",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      }

      /* handshake -- iteration counter here? */
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
      n = poll(&new_fd, 1, 10000);
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
       *   MP1 - an MCC (they're indistinguishable to PCM)
       *   MP2 - an MCC (they're indistinguishable to PCM)
       *   MP3 - an MCC (they're indistinguishable to PCM)
       *   MAC - the MAC
       *   MON - for any other client that wants to kibitz
       *
       * A client is also allows to reply BAD PROTO here, if the server protocol
       * version isn't liked.  Not much we can do about that, but we'll report
       * it.
       */
      if (strncmp(buffer, "BAD PROTO", 9) == 0) {
        bprintf(warning, "Client reset and dropped.  BAD PROTO from %s",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      } else if (buffer[0] != 'M') {
HANDSHAKE_FAIL:
        bprintf(warning, "Unrecognised response from client %s.  Dropped.",
            inet_ntoa(addr.sin_addr));
        close(new_fd.fd);
        goto RESET;
      }

      if (buffer[1] == 'P' && (buffer[2] == '1' /* a MCC */
            || buffer[2] == '2' || buffer[2] == '3'))
      {
        int rev = GetRev(buffer);
        if (rev < 0)
          goto HANDSHAKE_FAIL;
        n = buffer[2] - '0';

        if (rev != proto_rev) {
          bprintf(warning, "Dropping client %s with bad proto revision: %i",
              inet_ntoa(addr.sin_addr), rev);
          close(new_fd.fd);
          goto RESET;
        }
      } else if (buffer[1] == 'A' && buffer[2] == 'C') { /* the MAC */
        n = CLI_MAC;
        int rev = GetRev(buffer);
        if (rev < 0)
          goto HANDSHAKE_FAIL;
      } else if (buffer[1] == 'O' && buffer[2] == 'N') { /* monitor */
        n = CLI_MON;
      } else
        goto HANDSHAKE_FAIL;

      /* close the existing connection to this client, if any */
      i = FindInt(n, client + 1, 4);
      if (i > 0) {
        bprintf(info, "Closing old connection to %s", cli_name[n]);
        close(fds[i].fd);
      } else  {
        i = nfds++; /* new client */
        client[i] = n;
      }
      /* update record */
      fds[i].fd = new_fd.fd;

      bprintf(info, "Registered new client %s on %s", cli_name[n],
          inet_ntoa(addr.sin_addr));

      /* now RESET to get back to the SOP */
      goto RESET;
    }

    /* run through active clients and do, you know, things */
    for (i = 1; i < nfds; ++i) {
      /* handle weirdness / drops */
      if (fds[i].revents & (POLLERR | POLLNVAL | POLLHUP)) {
        if (fds[i].revents & POLLHUP)
          bprintf(warning, "Client %s disconnected.\n", cli_name[client[i]]);
        else
          bprintf(warning, "Dropping client %s on error.\n",
              cli_name[client[i]]);
        close(fds[i].fd);

        /* move the last client over top of this one and back up a step */
        client[i] = client[nfds];
        memcpy(fds + i, fds + nfds, sizeof(fds[i]));
        nfds--;
        i--;
      } else if (fds[i].revents & POLLIN) {
        if (client[i] == CLI_MON) {
          /* we ignore data from the monitor */
          while (read(fds[i].fd, buffer, 1024) > 0);
        } else {
          /* client has something to say */
        }
      } else if (fds[i].revents & POLLOUT) {
        /* client is ready to listen */
        ready[0] = 1;
        ready[i] = 1;
      }
    }

    /* "Broadcast" stuff to the clients, assuming someone's ready */
    if (ready[0]) {
      if (ForwardCommand(fds, ready, client)) {
        ; /* done */
      }
    }
  }

  return NULL;
};
