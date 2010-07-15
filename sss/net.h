/**************************************************************
 * sss source code
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include "sss_struct.h"

#define SOCK_PORT 54321

/* Create the socket for sending out the data */
int MakeSock(void);

/* Keep track of all connected clients 
   as well as send the data to said clients */
int SendData(sss_packet_data *);
