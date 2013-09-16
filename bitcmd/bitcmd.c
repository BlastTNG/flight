/* bitcmd: groundstation BIT command software
 *
 * This software is copyright (C) 2002-2013 D. V. Wiebe and others
 * 
 * This file is part of bitcmd.
 * 
 * bitcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * bitcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <limits.h>

#include "bitdaemon.h" /* contains all daemonising functions */
#include "netcmd.h" /* contains comm functions with COW */

#define ACK_COUNT 1

char err_message[ERR_MESSAGE_LEN];

int silent = 0;

char host[1024] = "localhost"; /* BITCMD host */
char target[1024] = "marvin.bit"; /* BIT flight computer */

char *ack[ACK_COUNT] = {
  "Command transmitted.", /* 0 */
};

void USAGE(int flag) {
  int i;
  
  printf("bitcmd [@target] [-l] [-lg] [-lc] [-d] [-nf]\n"
      "         command [param00 [param01 [param02 [ ... ]]]]\n"
      "Options:\n"
      "    @target   Connect to the BIT flight computer at target "
      "(default marvin.bit).\n"
      "       -l   List valid commands and parameters and exit.\n"
      "      -lg   List all command groups in order.\n"
      "      -lc   List all commands in group number given.\n"
      "       -d   Start the bittcmd daemon.\n"
      "      -nf   Don't fork into the background when daemonizing.\n"
      );

    printf("Exit codes:\n"
        "    -1  Unexpected internal error.\n");

    for (i = 0; i < ACK_COUNT; ++i)
      if (ack[i][0])
        printf("    %2i  %s\n", i, ack[i]);

    printf("\nFor a list of valid commands use `bitcmd -l'\n");
    exit(11);
}
   

int main(int argc, char *argv[]) {
	int i;
	int daemon_route, daemon_fork;
	int daemonise = 0;
	char* command[200];
	int nc = 0;
	int group = -1;

	daemon_route = 3; /* route to BIT MCC */
	daemon_fork = 0;  /* fork into background process */
	
	/* Parse switches */
	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-l") == 0) {
		  command[0] = "-l";
		  nc = 1;
		} 
		else if (strcmp(argv[i], "-lg") == 0) {
		  command[0] = "-lg";
		  nc = 1;
		} 
		else if (strcmp(argv[i], "-lc") == 0) {
		  command[0] = "-lc";
		  if (i < (argc - 1)) {
			group = strtol (argv[i+1], NULL, 0);
			i++;
			nc = 1;
		  }
		} 
		else if (strcmp(argv[i], "-d") == 0)
		  daemonise = 1;
		else if (strcmp(argv[i], "-nf") == 0)
		  daemon_fork = 1;
		else if (argv[i][0] == '@')
		  strcpy(target, &argv[i][1]);
		else if (argv[i][0] == '-' && (argv[i][1] < '0' || argv[i][1] > '9')
			&& argv[i][1] != '.')
		  USAGE(0);
		else
		  command[nc++] = argv[i];
	}
	
	if (daemonise) Daemonise(daemon_route, daemon_fork);
	if (daemon_route || daemon_fork) USAGE(0);

	fprintf(stderr, "Unexpected trap in main. Stop.\n");
	return -1;




}
