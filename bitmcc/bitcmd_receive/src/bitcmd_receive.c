/* ---------------------------------------------------------------------
 * -------------------- BITCMD UDP Receive Program ---------------------
 * ---------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a UDP client developed to interface with the BITCMD daemon
 * running on the ground computer. It uses the same command structure
 * constructed for COW clients on the ground and interprets received
 * commands accordingly.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: September 16, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
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

#include "command_list.h"
#include "netcmd.h"
#include "bitcmd_receive.h"


// initializes the UDP connection with bitcmd on the ground
int init_bitcmd(struct BITCMD *cmd)
{
	cmd->slen = sizeof(cmd->cli_addr);

	if ((cmd->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) printf("BITCMD : Socket() unsuccessful\n");
	//else
	//  printf("Server : Socket() successful\n");

	int size = 65536;
	if (setsockopt(cmd->sockfd,SOL_SOCKET,SO_SNDBUF,&size,sizeof(size)) < 0)
		printf("Unable to set socket options.\n");

	bzero(&(cmd->my_addr), sizeof(cmd->my_addr));
	cmd->my_addr.sin_family = AF_INET;
	cmd->my_addr.sin_port = htons(GROUND_PORT);
	cmd->my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(cmd->sockfd, (struct sockaddr* ) &(cmd->my_addr), sizeof(cmd->my_addr))==-1) perror("bind");

	return 0;
}

// reads command from UDP and stores it to bitcmd structure
int read_bitcmd(struct BITCMD *cmd)
{
	int i, num_bytes;

	num_bytes = recvfrom(cmd->sockfd, cmd->recv_buf, RECVBUFLEN, 0, (struct sockaddr*)&(cmd->cli_addr), &(cmd->slen));
	if (num_bytes<=0)
	{
		printf("Server : RecvFrom() failed\n");
		return -1;
	}
	else
	{
		cmd->recv_buf[num_bytes] = '\0'; // only read new bytes
		//printf("Message from %s (%d bytes): %s\n",inet_ntoa(cmd->cli_addr.sin_addr),num_bytes,cmd->recv_buf);
	}

	// parse the command by spaces ( )
	char *value;
	i = 0;

	do
	{
		cmd->inputs[i] = NULL;
		if (i==0) // first entry is the command name/id
		{
			value = strtok(cmd->recv_buf," ");
			if (value != NULL)
			{
				strcpy(cmd->id,value);
				i++;
			}
		}
		else // all other entries are command parameters
		{
			value = strtok(NULL," ");
			if (value != NULL)
			{
				cmd->inputs[i-1] = atof(value);
				i++;
			}
		}

	}
	while (value != NULL);
	cmd->n_in = i-1;

	/*
	printf("Sender: %s\nCommand: %s\nParameters: ",inet_ntoa(cmd->cli_addr.sin_addr),cmd->id);
	for (i=0;i<cmd->n_in;i++) printf("%f ",cmd->inputs[i]);
	printf("\n\n");
	*/

	return 0;
}

// closes the UDP connection with bitcmd on the ground
int close_bitcmd(struct BITCMD *cmd)
{
	close(cmd->sockfd);
	return 0;
}

// send the command list down to bitcmd
int send_cmdlist(struct BITCMD *cmd)
{
	uint16_t u16;
	int i, j, k;
	char output[4096];

	sprintf(output, "%s\n", command_list_serial);
	printf("Sending command list to bitcmd...");
	sendto(cmd->sockfd, output, strlen(output), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen);

	u16 = N_SCOMMANDS;
	if (sendto(cmd->sockfd, &u16, sizeof(u16), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
		return -1;

	u16 = N_MCOMMANDS;
	if (sendto(cmd->sockfd, &u16, sizeof(u16), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
		return -1;

	if (sendto(cmd->sockfd, scommands, sizeof(struct scom) * N_SCOMMANDS, MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen)< 1)
	{
		printf("Failed to send scommands: %s\n",strerror(errno));
		return -1;
	}
	if (sendto(cmd->sockfd, mcommands, sizeof(struct mcom) * N_MCOMMANDS, MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen)< 1)
	{
		printf("Failed to send mcommands: %s\n",strerror(errno));
		return -1;
	}

	for (i = 0; i < N_MCOMMANDS; ++i)
	{
		for (j = 0; j < mcommands[i].numparams; ++j)
			if (mcommands[i].params[j].nt)
			{
				u16 = i * 256 + j;
				printf("u16 %d\n",u16);
				if (sendto(cmd->sockfd, &u16, sizeof(u16), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
				{
					printf("Error u16: %s\n",strerror(errno));
					return -1;
				}
				/* count */
				for (u16 = 0; mcommands[i].params[j].nt[u16]; ++u16);
				if (sendto(cmd->sockfd, &u16, sizeof(u16), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
				{
					printf("Error u16: %s\n",strerror(errno));
				}

				output[0] = '\0';
				for (k = 0; mcommands[i].params[j].nt[k]; ++k)
				{
				  strncat(output, mcommands[i].params[j].nt[k], 79);
				  strcat(output, "\n");
				}
				if (sendto(cmd->sockfd, output, strlen(output), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
				{
					printf("Error output: %s\n",strerror(errno));
					return -1;
				}
				printf("output %s\n",output);
			}
	}
	u16 = 0xFFFF;
	if (sendto(cmd->sockfd, &u16, sizeof(u16), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
	return -1;

	printf("done!\n");
	return 0;
}

// send the command group names down to bitcmd
int send_groupnames(struct BITCMD *cmd)
{
	unsigned short i;
	char output[4096];

	printf("Sending command group names to bitcmd...");
	i = N_GROUPS;
	if (sendto(cmd->sockfd, &i, sizeof(i), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1)
	return -1;

	output[0] = '\0';
	for (i=0; i<N_GROUPS; ++i)
	{
		strncat(output, GroupNames[i], 127);
		strcat(output, "\n");
	}
	sendto(cmd->sockfd, output, strlen(output), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen);

	printf("done!\n");

	return 0;
}

// sends a "pong" in response to a "ping" from bitcmd
int send_pong(struct BITCMD *cmd)
{
	char buf[8] = "::pong::";
	if (sendto(cmd->sockfd, buf, sizeof(buf), MSG_NOSIGNAL,(struct sockaddr*)&(cmd->cli_addr), cmd->slen) < 1) return -1;
	else return 0;
}


