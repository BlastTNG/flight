/* This software is copyright (C) 2004 University of Toronto
 * and futher copyright 2009-2010 Matthew Truch
 * 
 * This file is part of the BLASTpol software, and is heavily borrowed from BLAST
 * and interloquendi.
 * 
 * You should have received a copy of the GNU General Public License
 * along with interloquendi; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
extern int h_errno;
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

#define RNC_SOCK_PORT 41114
#define DNC_SOCK_PORT 14441
#define TEA_SOCK_PORT 14141
#define LNC_SOCK_PORT 11111

#define RNC_LNKFILE  "/data/etc/highgain.lnk"
#define DNC_LNKFILE  "/data/etc/tdomni.lnk"
#define TEA_LNKFILE  "/data/etc/iromni.lnk"
#define LNC_LNKFILE  "/data/etc/irslow.lnk"

#define LEN 500
#define BUF_LEN 5000
#define PAST_SEND 100
#define SLEEP_PERIOD 100000000
#define NO_DATA_LIMIT 2000

char LNKFILE[4096];
int SOCK_PORT;

enum PartyType{RNC=0, DNC, TEA, LNC};

enum PartyType party = RNC;

void Connection(int csock)
{
  struct sockaddr_in addr;
  socklen_t addrlen = sizeof(addr);
  struct hostent* thishost;
  char data_file[LEN];
  int fd;
  char data[BUF_LEN];
  off_t offset;
  size_t n_read;
  struct timespec ts;
  int no_data = 0;

  ts.tv_sec = 0;
  ts.tv_nsec = SLEEP_PERIOD;

  getsockname(csock, (struct sockaddr*)&addr, &addrlen);
  thishost = gethostbyaddr((const char*)&addr.sin_addr, sizeof(addr.sin_addr),
      AF_INET);
  if (thishost == NULL && h_errno) {
    fprintf(stderr, "Warning: gethostbyaddr\n");
    thishost = NULL;
  }

  fd = open(LNKFILE, O_RDONLY);
  if (fd == -1) {
    fprintf(stderr, "Couldn't open %s!\n", LNKFILE);
    shutdown(csock, SHUT_RDWR);
    close(csock);
    return;
  }

  offset = lseek(fd, 0, SEEK_END);
  if (offset > PAST_SEND)
    offset = PAST_SEND;
  lseek(fd, -offset, SEEK_END);
  fprintf(stderr, "  %s: Rewind %ld bytes\n", inet_ntoa(addr.sin_addr), (long)offset);

  /* Service Loop */
  for (;;) {
    n_read = read(fd, data, BUF_LEN);

    if (n_read == 0) {
      if (no_data > NO_DATA_LIMIT) {
        fprintf(stderr, "  %s: No data in %s for too long\n", inet_ntoa(addr.sin_addr), data_file);
        no_data = 0;
        shutdown(csock, SHUT_RDWR);
        close(csock);
        return;
      }
      no_data++;
      nanosleep(&ts, NULL);
      continue;
    }

    if (n_read == -1){
      fprintf(stderr, "  %s: Error reading from %s\n", inet_ntoa(addr.sin_addr), data_file);
      shutdown(csock, SHUT_RDWR);
      close(csock);
      return;
    }

    no_data = 0; /* We have data, so reset no_data counter */

    if (write(csock, data, n_read) != n_read) {
      fprintf(stderr, "  %s: Error writing to socket with %zd data.\n", inet_ntoa(addr.sin_addr), n_read);
      shutdown(csock, SHUT_RDWR);
      close(csock);
      return;
    }
  }
}

int MakeSock(void)
{
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
  {
    fprintf(stderr, "Fatal: Socket\n");
    exit(-2);
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0)
  {
    fprintf(stderr, "Fatal: setsockopt\n");
    exit(-2);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(SOCK_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1)
  {
    fprintf(stderr, "fatal: bind\n");
    exit(-2);
  }

  if (listen(sock, 10) == -1)
  {
    fprintf(stderr, "fatal: listen\n");
    exit(-2);
  }

  fprintf(stderr, "listening on port %i.\n", SOCK_PORT);

  return sock;
}

int main(int argc, char *argv[])
{
  int pid;
  char *name;
  socklen_t sock, addrlen, csock;
  struct sockaddr_in addr;

  name = argv[0] + strlen(argv[0])-3;
  printf("program: %s\n", name);
  if (strcmp(name, "rnc")==0) {
    party = RNC;
    strcpy(LNKFILE, RNC_LNKFILE);
    SOCK_PORT = RNC_SOCK_PORT;
  } else if (strcmp(name, "dnc")==0) {
    party = DNC;
    strcpy(LNKFILE, DNC_LNKFILE);
    SOCK_PORT = DNC_SOCK_PORT;
  } else if (strcmp(name, "tea")==0) {
    party = TEA;
    strcpy(LNKFILE, TEA_LNKFILE);
    SOCK_PORT = TEA_SOCK_PORT;
  } else if (strcmp(name, "lnc")==0) {
    party = LNC;
    strcpy(LNKFILE, LNC_LNKFILE);
    SOCK_PORT = LNC_SOCK_PORT;
  } else {
    fprintf(stderr, "unknown program: %s\n", name);
    exit(0);
  }
  
  /* Autoreap children */
  signal(SIGCHLD, SIG_IGN);

  /* initialise listener socket */
  sock = MakeSock();

  /* accept loop */
  for (;;) {
    addrlen = sizeof(addr);
    csock = accept(sock, (struct sockaddr*)&addr, &addrlen);

    if (csock == -1 && errno == EINTR)
      continue;
    else if (csock == -1)
      fprintf(stderr, "Error: accept\n");
    else {
      /* fork child */
      if ((pid = fork()) == 0) {
        close(sock);
        Connection(csock);
        exit(0);
      }

      fprintf(stderr, "Spawned %i to handle connect from %s\n", pid,
          inet_ntoa(addr.sin_addr));
      close(csock);
    }
  }
}
