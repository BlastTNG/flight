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
 * QNX.
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
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "serial_board.h"

// initializes serial board located at addr in memory (all ports disabled by default)
// default addresses and IRQ levels are (re)loaded from EEPROM
int init_serial(struct serial_board* board, uint64_t addr)
{

	// request I/O privileges for the thread
	if (ThreadCtl(_NTO_TCTL_IO, 0) == -1)
	{
		printf("Error: ThreadCtl() failed to set I/O privileges.");
		return -1;
	}

	// create pointer to serial_board address
	board->base_addr = mmap_device_io(SERIAL_REG_SIZE,addr);
	if (board->base_addr == MAP_DEVICE_FAILED)
	{
		printf("Error: mmap_device_io() failed to map device addresses.");
		return -1;
	}

	// reload configuration from EEPROM
	reload_from_EEPROM(board);

	// set all DIO ports to output by default and clear all bits
	out8(board->base_addr+2,0xff);
	board->DIO_config = 0xff;
	out8(board->base_addr+3,0x00);
	board->DIO_output = 0xff;

	/*
	// set to RS-422 configuration on all ports by default
	out8(board->base_addr,0x10); // select ports 0-3
	out8(board->base_addr+1,0x55); // set RS-422 configuration
	board->config[0] = 0x55;

	out8(board->base_addr,0x11); // select ports 4-7
	out8(board->base_addr+1,0x55); // set RS-422 configuration
	board->config[1] = 0x55;
	*/

	return 0;
}

// un-initializes the serial board by disabling all ports
int close_serial(struct serial_board* board)
{
	unsigned int i;

	// disable all ports
	for (i=0;i<8;i++)
	{
		out8(board->base_addr,0x00+i); // set port as disabled
		board->port_enable[i] = 0;
	}

	// un-mapping device memory
	if (munmap_device_io(board->base_addr,SERIAL_REG_SIZE) == -1)
	{
		printf("Error: munmap_device_io() failed to un-map device addresses.");
		return -1;
	}

	return 0;
}

// reads a single byte value from the serial board at given port
int read_serial_byte(struct serial_board* serial, uint8_t port, uint8_t *value)
{
	uint8_t lsr, isr;
	do
	{
		isr = in8(serial->port_addr[port]+0x2) & 0x7; // get interrupt status
		lsr = in8(serial->port_addr[port]+0x5); // get TX/RX line status
		//printf("LSR: 0x%x\nISR: 0x%x\n",lsr,isr);
	}
	while (!(lsr & 0x1)); // loop until next byte ready
	if (isr & 0x6) lsr = in8(serial->port_addr[port]+0x5); // clear lsr if need be
	//printf("ISR: 0x%x\n",isr);

	*value = in8(serial->port_addr[port]+0x0); // get the data
	return 0;
}

// sends a single byte value to the serial board at given port
int write_serial_byte(struct serial_board* serial, uint8_t port, uint8_t* value)
{
	uint8_t lsr, isr;

	do
	{
		isr = in8(serial->port_addr[port]+0x2) & 0x7; // get interrupt status
		lsr = in8(serial->port_addr[port]+0x5); // get TX/RX line status
		//printf("LSR: 0x%x\n",lsr);
	}
	while (!(lsr & 0x20)); // loop until ready
	if (isr & 0x6) lsr = in8(serial->port_addr[port]+0x5); // clear lsr if need be

	out8(serial->port_addr[port]+0x0,*value); // send the data

	//printf("Sent: 0x%x\n",*value);

	return 0;
}

// enables port on serial board
int enable_serial_port(struct serial_board* board, uint8_t port)
{
	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7.\n");
		return -1;
	}
	out8(board->base_addr,port+0x80); // enable the port
	board->port_enable[port] = 1;
	//printf("0x%x\n",in8(board->base_addr));
	return 0;
}

// disables port on serial board
int disable_serial_port(struct serial_board* board, uint8_t port)
{
	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7.\n");
		return -1;
	}
	out8(board->base_addr,port+0x00); // disable the port
	board->port_enable[port] = 0;
	return 0;
}

// sets port address to addr on serial board
int set_port_addr(struct serial_board* board, uint8_t port, uint64_t addr)
{
	unsigned int i;
	uint8_t enable = 0x00;

	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7.\n");
		return -1;
	}

	// check for invalid address
	if ((addr < 0x100) || (addr > 0x3f8))
	{
		printf("Invalid address (must be from 0x100-0x3f8)\n");
		return -1;
	}

	// check for any matching addresses
	if (addr == board->base_addr)
	{
		printf("Address given is the base address.\n");
		return -1;
	}
	for (i=0;i<8;i++)
	{
		if (addr == board->port_addr[i])
		{
			printf("Address is in use by port %u.\n",i);
			return -1;
		}
	}

	// check to see if the port is enabled
	if (board->port_enable[port]) enable = 0x80;

	out8(board->base_addr+0,enable+port); // select the port
	out8(board->base_addr+1,addr/8); // set the port address
	board->port_addr[port] = addr;

	return 0;
}

// sets port interrupt level to IRQ on the serial board
int set_port_IRQ(struct serial_board* board, uint8_t port, uint8_t IRQ)
{
	uint8_t valid_IRQ[10] = {0x2,0x3,0x4,0x5,0x6,0x7,0xa,0xb,0xc,0xf};
	unsigned int i;
	unsigned int valid = 0;
	uint8_t enable = 0x00;

	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7).\n");
		return -1;
	}

	// check for invalid IRQ levels
	for (i=0;i<10;i++)
	{
		if (IRQ == valid_IRQ[i])
		{
			valid = 1;
			break;
		}
	}
	if (!valid)
	{
		printf("Invalid IRQ level.\n");
		return -1;
	}
	// check to see if the port is enabled
	if (board->port_enable[port]) enable = 0x80;

	out8(board->base_addr+0,enable+port+0x08); // select the port IRQ
	out8(board->base_addr+1,IRQ); // set the port IRQ level
	board->port_IRQ[port] = IRQ;

	return 0;
}

// takes the current state of the serial board and saves it to the EEPROM
// this includes addresses, IRQ levels, and port configurations
int save_to_EEPROM(struct serial_board* board)
{
	unsigned char q = 0x80; // EEPROM write mode
	unsigned int i;

	// write addresses and IRQs to EEPROM
	for (i=0; i<8; i++)
	{
		out8(board->base_addr+5,board->port_addr[i]/8); // set port address
		out8(board->base_addr+4,q); // select address port and set to write mode
		while (in8(board->base_addr+4) != 0x00); // wait until data is written

		out8(board->base_addr+5,board->port_IRQ[i]); // set port IRQ (2,3,4,5,6,7,a,b,c,f)
		out8(board->base_addr+4,q+0x08); // enable and select IRQ port
		while (in8(board->base_addr+4) != 0x00); // wait until data is written

		q++;
	}
	// write RS-422 configuration to EEPROM
	out8(board->base_addr+5,board->config[0]); // set to RS-422
	out8(board->base_addr+4,0x90); // ports 0 to 3
	while (in8(board->base_addr+4) != 0x00); // wait until data is written

	out8(board->base_addr+5,board->config[1]); // set to RS-422
	out8(board->base_addr+4,0x91); // ports 4 to 7
	while (in8(board->base_addr+4) != 0x00); // wait until data is written

	// verify EEPROM data
	q = 0x00; // EEPROM read mode

	uint64_t addr;
	uint8_t IRQ;

	for (i=0; i<8; i++)
	{
		out8(board->base_addr+4,q);
		while (in8(board->base_addr+4) != 0x00); // wait until data is read
		addr = in8(board->base_addr+5)*8; // read address

		out8(board->base_addr+4,q+0x08);
		while (in8(board->base_addr+4) != 0x00); // wait until data is read
		IRQ = in8(board->base_addr+5); // read IRQ

		if ((addr!=board->port_addr[i]) || (IRQ!=board->port_IRQ[i]))
		{
			printf("EEPROM address/IRQ failed to configure correctly.\n");
			return -1;
		}
		q++;
	}

	out8(board->base_addr+4,0x10);
	while (in8(board->base_addr+4) != 0x00); // wait until data is read
	if (in8(board->base_addr+5) != board->config[0])
	{
		printf("EEPROM config_03 failed to configure correctly.\n");
		return -1;
	}

	out8(board->base_addr+4,0x11);
	while (in8(board->base_addr+4) != 0x00); // wait until data is read
	if (in8(board->base_addr+5) != board->config[1])
	{
		printf("EEPROM config_47 failed to configure correoutputctly.\n");
		return -1;
	}

	return 0;
}

// reloads registers from EEPROM
int reload_from_EEPROM(struct serial_board* board)
{
	unsigned int q, i;
	uint8_t IRQ;
	uint64_t addr;

	// reload the EEPROM
	out8(board->base_addr+6,0x80);
	while (in8(board->base_addr+4) != 0x00); // wait until reloaded

	// read the EEPROM data to the serial_board structure
	q = 0x00; // EEPROM read mode
	for (i=0;i<8;i++)
	{
		out8(board->base_addr+4,q);
		while (in8(board->base_addr+4) != 0x00); // wait until data is read
		addr = in8(board->base_addr+5)*8; // read address
		board->port_addr[i] = addr; // store address

		out8(board->base_addr+4,q+0x08);
		while (in8(board->base_addr+4) != 0x00); // wait until data is read
		IRQ = in8(board->base_addr+5); // read IRQ
		board->port_IRQ[i] = IRQ; // store IRQ

		q++;
	}

	// reload configuration from EEPROM
	out8(board->base_addr+4,0x10);
	while (in8(board->base_addr+4) != 0x00); // wait until data is read
	board->config[0] = in8(board->base_addr+5);

	out8(board->base_addr+4,0x11);
	while (in8(board->base_addr+4) != 0x00); // wait until data is read
	board->config[1] = in8(board->base_addr+5);

	return 0;
}

// sets the configuration of the port on serial board
int set_port_config(struct serial_board* board, uint8_t port, uint8_t config)
{
	unsigned int reg;
	uint8_t new;

	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7).\n");
		return -1;
	}

	// check for invalid configurations
	if ((config<0) || (config>3))
	{
		printf("Invalid configuration specified.\n");
		return -1;
	}

	// determine configuration byte based on port number
	if (port < 4) reg = 0;
	else
	{
		reg = 1;
		port = port-4;
	}
	new = 255-(3*pow(4,port));
	board->config[reg] = board->config[reg] & new; // clear bits
	new = (config*pow(4,port));
	board->config[reg] = board->config[reg] | new; // set configuration bits

	out8(board->base_addr+0,0x10+reg); // select register
	out8(board->base_addr+1,board->config[reg]); // set configuration

	return 0;
}

// sets the bit for the the DIO port on the serial board
int set_serial_DIO(struct serial_board* board, uint8_t port)
{
	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7).\n");
		return -1;
	}
	board->DIO_config |= (uint8_t) pow(2,port); // set to output mode
	out8(board->base_addr+2,board->DIO_config);
	board->DIO_output |= (uint8_t) pow(2,port); // set the bit
	out8(board->base_addr+3,board->DIO_output);
	//printf("0x%x\n",board->DIO_output);

	return 0;
}

// clears the bit for the the DIO port on the serial board
int clear_serial_DIO(struct serial_board* board, uint8_t port)
{
	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7).\n");
		return -1;
	}
	board->DIO_config |= (uint8_t) pow(2,port); // set to output mode
	out8(board->base_addr+2,board->DIO_config);
	board->DIO_output &= 255-((uint8_t) pow(2,port)); // clear the bit
	out8(board->base_addr+3,board->DIO_output);
	//printf("0x%x\n",board->DIO_output);

	return 0;
}

// reads the bit for the the DIO port on the serial board and stores in value
int read_serial_DIO(struct serial_board* board, uint8_t port, uint8_t *value)
{
	uint8_t all;
	// check for invalid port numbers
	if ((port<0) || (port>7))
	{
		printf("Invalid port number (must be from 0-7).\n");
		return -1;
	}
	board->DIO_config |= 255-((uint8_t) pow(2,port)); // set to input mode
	out8(board->base_addr+2,board->DIO_config);
	all = in8(board->base_addr+3);
	*value = all & ((uint8_t) pow(2,port)); // report whether or not the bit is set
	return 0;
}
