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

#ifndef BITCMD_INCLUDE_H_
#define BITCMD_INCLUDE_H_

// some defines
#define RECVBUFLEN 1024
#define GROUND_PORT 21212	// port over which ground commands are received

struct BITCMD
{
	struct sockaddr_in cli_addr, my_addr;
	int sockfd, i;
	int num_bytes;
	socklen_t slen;

	char id[1024];
	float inputs[HK_MAX];
	int n_in;

	uint8_t recv_buf[RECVBUFLEN];
};

// some function prototypes
int init_bitcmd(struct BITCMD * );
int read_bitcmd(struct BITCMD * );
int execute_bitcmd(struct BITCMD * );
int close_bitcmd(struct BITCMD * );
int send_cmdlist(struct BITCMD * );
int send_groupnames(struct BITCMD * );
int send_pong(struct BITCMD * );

#endif /* BITCMD_INCLUDE_H_ */
