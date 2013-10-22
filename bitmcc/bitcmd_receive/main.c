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
#include <sys/mman.h> // address mapping for I/O
#include <hw/inout.h> // raw PC104 I/O routines
#include <sys/neutrino.h> // Neutrino kernel/thread control

#include "command_list.h"
#include "netcmd.h"
#include "bitcmd_receive.h"
#include "stablecmd.h"

struct BITCMD bitcmd; // the BIT command structure
struct STABLESERVER stableserver; // the STABLE command/telemetry structure

int main()
{
	init_bitcmd(&bitcmd);
	init_stableserver(&stableserver);

	printf("Waiting for commands from bitcmd...\n");

	while (1)
	{
		read_bitcmd(&bitcmd);
		if (strncmp(bitcmd.id,"::group::",9) == 0) send_groupnames(&bitcmd);
		else if (strncmp(bitcmd.id,"::list::",8) == 0) send_cmdlist(&bitcmd);
		else if (strncmp(bitcmd.id,"::ping::",8) == 0) send_pong(&bitcmd);

	}

	close_bitcmd(&bitcmd);
	close_stableserver(&stableserver);

	return 0;
}
