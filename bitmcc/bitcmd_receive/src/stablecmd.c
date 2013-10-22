/* ---------------------------------------------------------------------
 * ------------------ STABLECMD UDP Receive Program --------------------
 * ---------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a UDP server developed to interface with the STABLE client
 * running on the STABLE computer.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: October 15, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h> // socket stuff
#include <sys/stat.h>
#include <sys/socket.h> // socket stuff
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h> // address mapping for I/O
#include <hw/inout.h> // raw PC104 I/O routines
#include <sys/neutrino.h> // Neutrino kernel/thread control

#include "stablecmd.h"

#define COMMAND(x) (uint16_t)x, #x // echoes enum as string

// STABLE commands data structure
const struct STABLECMD stablecmd[NSTABLECMDS] = {
		{COMMAND(PMM_SET_MODE_CONTROL),DATA,NONE,1,1,1,1},
		{COMMAND(PMM_SET_MODE_READY),DATA,NONE,1,1,1,1},
		{COMMAND(PMM_FSW_RESET),DATA,NONE,1,1,1,1}
};

//initialized UDP server for STABLE hardware
int init_stableserver(struct STABLESERVER *server)
{
	server->slen = sizeof(server->cli_addr);

	if ((server->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) printf("STABLESERVER : Socket() unsuccessful\n");
	//else
	//  printf("Server : Socket() successful\n");

	bzero(&(server->my_addr), sizeof(server->my_addr));
	server->my_addr.sin_family = AF_INET;
	server->my_addr.sin_port = htons(STABLE_PORT);
	server->my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(server->sockfd, (struct sockaddr* ) &(server->my_addr), sizeof(server->my_addr))==-1) perror("bind");

	return 0;
}

// closes UDP server for STABLE hardware
int close_stableserver(struct STABLESERVER *server)
{
	close(server->sockfd);
	return 0;
}

// reads command from STABLE via UDP and stores it to stableserver structure
int read_stableserver(struct STABLESERVER *server)
{
	int num_bytes;

	memset(server->recv_buf,0x0,RECVBUFLEN);
	memset(&(server->stablecmd),0x0,sizeof(struct STABLECMD));

	num_bytes = recvfrom(server->sockfd, server->recv_buf, RECVBUFLEN, 0, (struct sockaddr*)&(server->cli_addr), &(server->slen));
	if (num_bytes<=0)
	{
		printf("Server : RecvFrom() failed\n");
		return -1;
	}

	// parse the message
	memcpy(&(server->stablecmd.type),server->recv_buf+0,4);
	memcpy(&(server->stablecmd.id),server->recv_buf+4,2);
	server->stablecmd.id = server->stablecmd.id-STABLE_TELEM_ID_OFFSET; // offset ID
	memcpy(&(server->stablecmd.rt_update_interval),server->recv_buf+6,2);
	memcpy(&(server->stablecmd.rec_update_interval),server->recv_buf+8,2);
	memcpy(&(server->stablecmd.rt_reported_insts),server->recv_buf+10,2);
	memcpy(&(server->stablecmd.rec_reported_insts),server->recv_buf+12,2);
	memcpy(&(server->ts),server->recv_buf+14,8);
	memcpy(&(server->stablecmd.dataType),server->recv_buf+22,4);
	if (server->stablecmd.dataType == U32) memcpy(&(server->tlm_U32_lsv),server->recv_buf+26,4);
	else if (server->stablecmd.dataType == I32) memcpy(&(server->tlm_I32_lsv),server->recv_buf+26,4);
	else if (server->stablecmd.dataType == F32) memcpy(&(server->tlm_F32_lsv),server->recv_buf+26,4);
	else
	{
		printf("Unknown data type received.\n");
		return -1;
	}

	return 0;
}

// sends a specified command (and data if applicable) to STABLE via UDP
int send_stableserver(struct STABLESERVER *server, enum StableCmds cmd, uint32_t * data)
{
	int i;

	// find the command in STABLE command data structure
	for (i=0;i<NSTABLECMDS;i++)
	{
		if (stablecmd[i].id == (int)cmd) break;
	}
	if (i==NSTABLECMDS)
	{
		printf("Unable to find specified command.\n");
		return -1;
	}
	uint16_t trueid = stablecmd[i].id+STABLE_CMD_ID_OFFSET;

	// assemble the message
	memset(server->recv_buf,0x0,RECVBUFLEN);

	memcpy(server->recv_buf+0,&(stablecmd[i].type),4);
	memcpy(server->recv_buf+4,&trueid,2);
	memcpy(server->recv_buf+6,&(stablecmd[i].rt_update_interval),2);
	memcpy(server->recv_buf+8,&(stablecmd[i].rec_update_interval),2);
	memcpy(server->recv_buf+10,&(stablecmd[i].rt_reported_insts),2);
	memcpy(server->recv_buf+12,&(stablecmd[i].rec_reported_insts),2);

	memcpy(server->recv_buf+22,&(stablecmd[i].dataType),4);
	if (stablecmd[i].dataType != NONE) memcpy(server->recv_buf+26,data,4);

	// send the message
	if (sendto(server->sockfd, server->recv_buf, 30, MSG_NOSIGNAL,(struct sockaddr*)&(server->cli_addr), server->slen) < 1) return -1;

	return 0;
}

