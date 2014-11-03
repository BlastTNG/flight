/* -----------------------------------------------------------------------
 * --------------------- HMR2300 Magnetometer Driver-- -------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * --------------------------- Description ------------------------------
 * This is simple driver interface for the HMR2300 magnetometer via
 * RS-232 serial interface. This script makes use of the QNX driver for
 * the EMM-8P-XT RevC RS-422/485 PC/104 serial card for serial port
 * configuration setup.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: July 18, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <termios.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <pthread.h>

#include "serial_board.h"
#include "HMR2300_mag.h"

struct serial_board serial;
struct magnetometer mag;

void readOut(void)
{
	int n;
	printf("Loop started...\n");
	while (1)
	{
		n = read_HMR2300(&mag);
		//printf("%s\n",mag.buffer);
		printf("X: %4.6f, Y: %4.6f, Z: %4.6f\n",mag.reading[0],mag.reading[1],mag.reading[2]);
	}
	printf("Loop ended.\n");
}

int main()
{

	pthread_t thr;

	printf("Initializing hardware...");
	init_serial(&serial,SERIAL_ADDR_DEFAULT);
	init_HMR2300(&mag,&serial,5);
	printf("done!\n");

	char input[1024];
	char *p;

	input[0] = '\0';

	pthread_create(&thr, NULL, (void*)&readOut, NULL);

	printf("Enter command to magnetometer:\n");
	while (1)
	{
		p = fgets (input, 100, stdin);
		if (strncmp(input,"exit",4) == 0) break;
		if (p == NULL) printf("Input error.\n");
		write_HMR2300(&mag,input,strlen(input));

		/*
		n = read_HMR2300(&mag);
		printf("%s %d\n",mag.buffer,n);
		for (i=0;i<n;i++) printf("0x%x ",mag.buffer[i]);
		printf("\n\n");
		*/
	}

	pthread_cancel(thr);

	printf("Closing hardware...");
	close_HMR2300(&mag);
	close_serial(&serial); // close the serial port
	printf("done!\n");

	return 0;
}


