/* -----------------------------------------------------------------------
 * ------------------------- RW Serial Interface--------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a simple script to send serial commands to the DPRALTE
 * reaction wheel controller via RS-232.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: August 2, 2013
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
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "dscud.h"
#include "serial_board.h"
#include "IO_board.h"
#include "DPRALTE_RW.h"

int main()
{
	struct serial_board serial;
	static struct rw_controller rw;
	static struct IO_board io;

	rw.verbosity = 0; // be verbose with sent and received messages

	printf("Initializing hardware...\n");
	init_serial(&serial,SERIAL_ADDR_DEFAULT); // initialize serial board
	init_DPRALTE(&rw,&serial,&io,4);
	check_drive_errors_DPRALTE(&rw);
	printf("done!\n\n");
	//printf("0x%x 0x%x\n",serial.config[0],serial.config[1]); // check serial configuration

	/*
	// disable heartbeat pulses
	data = 0x0000;
	memcpy(temp,&data,2);
	write_message_DPRALTE(&rw,0x04,0x01,1,temp);
	read_message_DPRALTE(&rw);


	// set gain for the velocity loop controller
	printf("Setting gains...");
	set_vel_gains_DPRALTE(&rw,0,20,20,20);
	printf("done!\n");
	*/

	// enter command interface for user
	unsigned int i;
	int inputs[4];
	char *p;
	char line[100];
	char *value;
	uint8_t data[255];

	printf("Enter index, offset, size, and data separated by commas. Enter 0 to terminated program\n");
	printf("The data field is optional depending on whether or not a parameter requires data.\n");
	printf("*Note that the size is the number of 2 byte words.\n");
	printf("------------------------------------------------------------------------------------\n");
	while (1)
	{
		// get the input line
		printf("Enter command: ");
		p = fgets (line, 100, stdin);
		if (p == NULL) printf("Input error.\n");
		else
		{
			// parse the line by commas
			i = 0;
			do
			{
				inputs[i] = NULL;
				if (i==0) value = strtok(line,",");
				else value = strtok(NULL,",");
				if (value != NULL)
				{
					inputs[i] = atoh(value);
					//printf("0x%x\n",inputs[i]);
					i++;
				}
			}
			while ((value != NULL) && (i<4));
			if (inputs[0] == 0) break; // break condition

			// check for invalid inputs
			if ((inputs[0] > 255) || (inputs[1] > 255) || (inputs[2] > 255))
			{
				printf("Invalid input in one of the data fields.");
			}
			else
			{
				if (i == 4)
				{
					memcpy(data,&inputs[3],inputs[2]*2);
					write_message_DPRALTE(&rw,(uint8_t) inputs[0],(uint8_t) inputs[1],(uint8_t) inputs[2],data);
				}
				else write_message_DPRALTE(&rw,(uint8_t) inputs[0],(uint8_t) inputs[1],(uint8_t) inputs[2],NULL);;
				read_message_DPRALTE(&rw);
			}
		}
	}
	printf("------------------------------------------------------------------------------------\n");
	printf("\nClosing hardware...\n");
	close_DPRALTE(&rw);
	close_serial(&serial);
	printf("done!\n");

	return 0;
}
