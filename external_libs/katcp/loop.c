/* (c) 2010,2011 SKA SA */
/* Released under the GNU GPLv3 - see COPYING */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include <sys/time.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "katpriv.h"
#include "katcl.h"
#include "katcp.h"
#include "netc.h"

int run_katcp(struct katcp_dispatch *d, int server, char *host, int port)
{
  if(server == 0){
    return run_client_katcp(d, host, port);
  } else{
    return run_server_katcp(d, host, port);
  }
}

int run_server_katcp(struct katcp_dispatch *d, char *host, int port)
{
  return run_config_server_katcp(d, NULL, 1, host, port);
}

int run_multi_server_katcp(struct katcp_dispatch *d, int count, char *host, int port)
{
  return run_config_server_katcp(d, NULL, count, host, port);
}
