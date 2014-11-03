/* ---------------------------------------------------------------------
 * ----------- DMM-32DX-AT Digital I/O PC/104 Module Driver ------------
 * ---------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver for the DMM-32DX-AT Digital I/O PC/104 module
 * for QNX. Based on the Diamond Systems Universal Driver (DSUD), this
 * driver allows for easier initialization of the I/O board and handling
 * of board properties.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 24, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "dscud.h"
#include "IO_board.h"

int main()
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	/*
	uint8_t msb, lsb;

	// raw A/D conversion
	out8(IO_ADDR_DEFAULT+2,0x0);
	out8(IO_ADDR_DEFAULT+3,0x0);
	out8(IO_ADDR_DEFAULT+11,0x8);
	while (in8(IO_ADDR_DEFAULT+11) & 0x80);

	printf("Hellow\n");

	while (1)
	{
		out8(IO_ADDR_DEFAULT+0,0x0);
		usleep(1000);
		lsb = in8(IO_ADDR_DEFAULT+0);
		msb = in8(IO_ADDR_DEFAULT+1);
		printf("0x%x%x\n",msb,lsb);
	}
	*/


	static struct IO_board io;
	init_IO(&io,IO_ADDR_DEFAULT,IO_IRQ_DEFAULT);
	printf("I/O board initialized...\n");

	printf("State: 0x%x\n",in8(IO_ADDR_DEFAULT+11));

	int result;

	/*
	DSCSAMPLE dscsample;

	while (1)
	{
		result = dscADSample(io.dscb, &dscsample);
		if (result == DE_NONE) printf("0x%x\n",dscsample);
		usleep(1000);
	}
	*/

	// set default pin states for power board
	DFLOAT voltage;
	unsigned int i;
	for (i=1;i<20;i++) dscDIOSetPin(io.dscb,i);

	// enter digital I/O commands
	unsigned int j;
	char inputs[24][3];
	char *p;
	unsigned int thePin;
	char line[100];
	char *value;

	printf("Enter digital I/O commands for the I/O board.\n"
			"Commands: p# to pulse to ground, s# to set pin, c# to clear pin\n"
			"Separate commands by commas (max. 24 commands). Enter 0 to close.\n");
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
				inputs[i][0] = NULL;
				if (i==0) value = strtok(line," ,");
				else value = strtok(NULL," ,");
				if (value != NULL)
				{
					memcpy(inputs[i],value,3);
					//printf("0x%x\n",inputs[i]);
					i++;
				}
			}
			while ((value != NULL) && (i<24));
			if (inputs[0][0] == '0') break; // break condition

			if (strncmp(inputs[0],"adc",3) == 0) // A/D scan
			{
				if ((result = dscADScan(io.dscb, &(io.dscadscan), io.dscadscan.sample_values)) != DE_NONE) printf("DSCADSCAN failed\n");
				else
				{
					for (i=0;i<16;i++)
					{
						//dscADCodeToVoltage(io.dscb,io.dscadsettings,io.dscadscan.sample_values[i],&voltage);
						voltage = (io.dscadscan.sample_values[i]-2048.0)*(10.0/2048.0);
						printf("Sample %u: 0x%x = %g V\n",i,io.dscadscan.sample_values[i],voltage);
					}
					usleep(500000);
				}
			}
			else // commands
			{
				for (j=0;j<i;j++)
				{
					thePin = atoi(inputs[j]+1);
					if (inputs[j][0] == 's') // set
					{
						if (dscDIOSetPin(io.dscb,thePin)) printf("Invalid pin #%u\n",thePin);
						else printf("Set pin #%u\n",thePin);
					}
					else if (inputs[j][0] == 'c') // clear
					{
						if (dscDIOClearPin(io.dscb,thePin)) printf("Invalid pin #%u\n",thePin);
						else printf("Clear pin #%u\n",thePin);
					}
					else if (inputs[j][0] == 'p') // pulse
					{
						dscDIOClearPin(io.dscb,thePin);
						usleep(200000); // 200 ms delay
						dscDIOSetPin(io.dscb,thePin);
						printf("Pulse pin #%u\n",thePin);
					}
					else
					{
						printf("Unrecognized '%c' command.\n",inputs[j][0]);
					}
				}
			}
		}
	}

	close_IO(&io);
	printf("I/O board closed.\n");
	return 0;
}


