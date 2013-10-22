/* -----------------------------------------------------------------------
 * --------------- EMM-8P-XT RevC RS-422/485 PC/104 Driver ---------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver for the EMM-8P-XT RS-422/485 PC/104 module for
 * QNX. This file contains a test main function that initializes the board
 * enables a port, changes the port address, changes the port IRQ, reads
 * some data from the port, disables the port, and closes the board.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 10, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 * 18/6/13 - added some functionality with DSP 1750 rate gyroscopes;
 * included bit reversal algorithm to read data from DSP 1750
 *
 * 19/6/13 - created setup functions for the DSP 1750 for easy
 * initialization, reading, and closing of multiple devices
 *
 * 11/7/13 - finished fine tuning all setup functions for DSP 1750 on all
 * gyros; this script now reads three gyros and writes the data to a file
 *
 * 19/7/13 - added functionality for digital I/O pins on the serial ports;
 * included "set_serial_DIO", "clear_serial_DIO", and "read_serial_DIO"
 * functions
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "serial_board.h"
#include "DSP1750_gyro.h"

unsigned int num_samples;
struct serial_board serial;
struct rategyro gyro1, gyro2, gyro3;


int main()
{
	unsigned int i;

	init_serial(&serial,SERIAL_ADDR_DEFAULT);

	printf("Serial board initialized. Displaying current state:\n");

	for (i=0; i<8; i++)
	{
		out8(serial.base_addr,0x80+i);
		printf("Port %u: 0x%x 0x%x\n",i,in8(serial.base_addr+1)*8,(unsigned int) serial.port_IRQ[i]);
	}

	printf("\n");
	printf("0x%x 0x%x\n",serial.config[0],serial.config[1]);


	/*
	set_port_IRQ(&serial,0,0x3); // change port 0 IRQ
	set_port_IRQ(&serial,1,0x3); // change port 1 IRQ
	set_port_IRQ(&serial,2,0x3); // change port 0 IRQ
	set_port_IRQ(&serial,3,0x3); // change port 1 IRQ
	set_port_IRQ(&serial,4,0x3); // change port 0 IRQ
	set_port_IRQ(&serial,5,0x3); // change port 1 IRQ
	set_port_IRQ(&serial,6,0x3); // change port 0 IRQ
	set_port_IRQ(&serial,7,0x3);
	 // change port 1 IRQ

	set_port_addr(&serial,0,0x208); // change port 0 address
	set_port_addr(&serial,1,0x210); // change port 1 address
	set_port_addr(&serial,2,0x218); // change port 2 address
	set_port_addr(&serial,3,0x220); // change port 3 address
	set_port_addr(&serial,4,0x228); // change port 4 address
	set_port_addr(&serial,5,0x230); // change port 5 address
	set_port_addr(&serial,6,0x238); // change port 6 address
	set_port_addr(&serial,7,0x240); // change port 7 address

	printf("\nSaving to EEPROM...");
	save_to_EEPROM(&serial); // save state to the EEPROM
	printf("done!\n");
	*/

	/*
	set_port_addr(&serial,0,0x248); // change port 0 address
	set_port_addr(&serial,1,0x250); // change port 1 address
	set_port_IRQ(&serial,0,0xa); // change port 0 IRQ
	set_port_IRQ(&serial,1,0xa); // change port 1 IRQ
	set_port_config(&serial,0,RS232_CONFIG); // change port 0 configuration
	set_port_config(&serial,1,RS232_CONFIG); // change port 1 configuration


	printf("Ports 0 and 1 changed. Displaying current state:\n");
	for (i=0; i<8; i++) printf("Port %u: 0x%x 0x%x\n",i,(unsigned int) serial.port_addr[i],(unsigned int) serial.port_IRQ[i]);

	printf("\nSaving to EEPROM...");
	save_to_EEPROM(&serial); // save state to the EEPROM
	printf("done!\n");	{


	set_port_addr(&serial,0,0x208); // change port 0 address
	set_port_addr(&serial,1,0x210); // change port 1 address
	set_port_IRQ(&serial,0,0x3); // change port 0 IRQ
	set_port_IRQ(&serial,1,0x3); // change port 1 IRQ
	set_port_config(&serial,0,RS422_CONFIG); // change port 0 configuration
	set_port_config(&serial,1,RS422_CONFIG); // change port 1 configuration

	printf("\nPorts 0 and 1 reverted. Displaying current state:\n");
	for (i=0; i<8; i++) printf("Port %u: 0x%x 0x%x\n",i,(unsigned int) serial.port_addr[i],(unsigned int) serial.port_IRQ[i]);

	printf("\nSaving to EEPROM...");
	save_to_EEPROM(&serial); // save state to the EEPROM
	printf("done!\n");
	*/

	// set up DSP 1750 on serial port 0 ("/dev/ser1" as set in /etc/rc.d/rc.local)
	// ** ONLY READ ONE GYRO AT A TIME IF NOT USING PTHREAD!! **
	init_DSP1750(&gyro1,&serial,0);

	FILE* myfile;
	int count = 0;
	myfile = fopen("test_gyro_data.txt","w");
	while (count < 10000)
	{
		read_DSP1750(&gyro1);

		fprintf(myfile,"%4.6f\n",gyro1.reading);
		if ((count%1000) == 0) printf("%4.6f\n",gyro1.reading);
		count++;
	}
	fclose(myfile);

	close_DSP1750(&gyro1);
	close_serial(&serial);
	printf("Serial board closed.\n");
	return 0;
}
