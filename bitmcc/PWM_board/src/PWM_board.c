/* -----------------------------------------------------------------------
 * --------------------- RTD DM6916 PWM PC/104 Driver --------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver created to run the DM6916 PWM PC/104 module in
 * QNX. This includes functions for the 9 PWM generators as well as the 3
 * 8254 16-bit counters.
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
#include <sys/neutrino.h>
#include <hw/inout.h>

#include "PWM_board.h"

// initializes the PWM board at addr (base address)
int init_PWM (struct PWM_board* board, uint64_t addr)
{
	int i;

	// request I/O privileges for the thread
	if (ThreadCtl(_NTO_TCTL_IO, 0) == -1)
	{
		printf("Error: ThreadCtl() failed to set I/O privileges.");
		return -1;
	}

	// create pointer to PWM board address
	board->base_addr = mmap_device_io(PWM_REG_SIZE,addr);
	if (board->base_addr == MAP_DEVICE_FAILED)
	{
		printf("Error: mmap_device_io() failed to map device addresses.");
		return -1;
	}

	// configure default state for PWM registers
	// all PWM generators are disabled by default
	for (i=0;i<12;i++)
	{
		out8(board->base_addr+i,0x00);
		board->port[i] = 0x00;
	}

	disable_IRQ(board); // disable IRQ by default

	return 0;
}

// un-initializes the PWM board by disabling all ports and un-mapping device memory
int close_PWM(struct PWM_board* board)
{
	int i;

	// disable all PWM ports
	for (i=0;i<12;i++)
	{
		out8(board->base_addr+i,0x00);
		board->port[i] = 0x00;
	}

	// un-map device memory
	if (munmap_device_io(board->base_addr,PWM_REG_SIZE) == -1)
	{
		printf("Error: munmap_device_io() failed to un-map device addresses.");
		return -1;
	}
	return 0;
}

// enables port on PWM board
int enable_PWM_port(struct PWM_board* board, uint8_t port)
{
	if ((port%4 == 3) || (port > 11)) // ignore invalid ports
	{
		printf("Error: invalid port.\n");
		return -1;
	}
	unsigned int add = pow(2,port%4); // determine bit to be set
	unsigned int chip = 4*(port/4)+3; // determine which chip is used
	board->port[chip] = board->port[chip] | add; // set the bit
	out8(board->base_addr+chip,board->port[chip]); // set the register

	//printf("%u\n",chip);

	return 0;
}

// disables port on PWM board
int disable_PWM_port(struct PWM_board* board, uint8_t port)
{
	if ((port%4 == 3) || (port > 11)) // ignore invalid ports
	{
		printf("Error: invalid port.\n");
		return -1;
	}
	unsigned int add = pow(2,port%4); // determine bit to be cleared
	unsigned int chip = 4*(port/4)+3; // determine which chip is used
	board->port[chip] = board->port[chip] & (255-add); // clear the bit
	out8(board->base_addr+chip,board->port[chip]); // set the register

	//printf("%u\n",board->port[chip]);

	return 0;

}

// sets the PWM DIO pin associated with specified port
int set_PWM_DIO(struct PWM_board* board, uint8_t port)
{
	if ((port%4 == 3) || (port > 11)) // ignore invalid ports
	{
		printf("Error: invalid port.\n");
		return -1;
	}
	unsigned int add = pow(2,(port%4)+4); // determine bit to be set
	unsigned int chip = 4*(port/4)+3; // determine which chip is used
	board->port[chip] = board->port[chip] | add; // set the bit
	out8(board->base_addr+chip,board->port[chip]); // set the register

	//printf("%u\n",chip);

	return 0;
}

// clears the PWM DIO pin associated with specified port
int clear_PWM_DIO(struct PWM_board* board, uint8_t port)
{
	if ((port%4 == 3) || (port > 11)) // ignore invalid ports
	{
		printf("Error: invalid port.\n");
		return -1;
	}
	unsigned int add = pow(2,(port%4)+4); // determine bit to be cleared
	unsigned int chip = 4*(port/4)+3; // determine which chip is used
	board->port[chip] = board->port[chip] & (255-add); // clear the bit
	out8(board->base_addr+chip,board->port[chip]); // set the register

	//printf("%u\n",board->port[chip]);

	return 0;
}

// sets the duty cycle of port to value on PWM board (port must be enabled)
int set_duty_cycle(struct PWM_board* board, uint8_t port, int value)
{
	if ((port%4 == 3) || (port > 11)) // ignore invalid ports
	{
		printf("Error: invalid port.\n");
		return -1;
	}
	if ((value < -255) || (value > 255))
	{
		printf("Error: invalid value.\n");
		return -1;
	}

	unsigned int add = pow(2,port%4+4);
	unsigned int chip = 4*(port/4)+3; // determine which chip is used

	// check direction
	if (value < 0) board->port[chip] = board->port[chip] | add; // set the bit
	else board->port[chip] = board->port[chip] & (255-add); // clear the bit

	// check magnitude
	board->port[port] = abs(value);
	out8(board->base_addr+chip,board->port[chip]); // set the direction register
	out8(board->base_addr+port,board->port[port]); // set the register

	//printf("%u\n",board->port[port]);

	return 0;
}

// configures one of the three counters on PWM board
int configure_counter(struct PWM_board* board, uint8_t counter, uint8_t read, uint8_t mode, uint8_t format)
{
	// check for invalid values
	if ((counter < 0) || (counter > 3))
	{
		printf("Invalid counter selected.\n");
		return -1;
	}
	else if ((read < 0) || (read > 3))
	{
		printf("Invalid read operation.\n");
		return -1;
	}
	else if ((mode < 0) || (mode > 5))
	{
		printf("Invalid mode selected.\n");
		return -1;
	}
	else if ((format < 0) || (format > 1))
	{
		printf("Invalid format selected.\n");
		return -1;
	}
	uint8_t config = counter*64+read*16+mode*2+format;
	out8(board->base_addr+15,config);
	//printf("%d\n",config);

	return 0;
}

// sets the counter on the PWM board to count
int set_counter(struct PWM_board* board, uint8_t counter, uint16_t count)
{
	// check for invalid inputs
	if ((counter < 0) || (counter > 2))
	{
		printf("Invalid counter selected.\n");
		return -1;
	}
	uint8_t hex_count[2] = {count,count/256};
	out8(board->base_addr+12+counter,hex_count[0]);
	out8(board->base_addr+12+counter,hex_count[1]);

	//printf("%x %x\n",hex_count[0],hex_count[1]);
	return 0;
}

// enables interrupt requests from counters on PWM board
int enable_IRQ(struct PWM_board* board)
{
	out8(board->base_addr+16,0x01);
	board->IRQ_enable = 1;
	clear_IRQ(board);
	return 0;
}

// disables interrupt requests from counters on PWM board
int disable_IRQ(struct PWM_board* board)
{
	out8(board->base_addr+16,0x00);
	board->IRQ_enable = 0;
	return 0;
}

// checks the IRQ status of the PWM board
// if there is an IRQ, returns 1; if no IRQ, returns 0
int check_IRQ(struct PWM_board* board)
{
	// check interrupt status
	if (in8(board->base_addr+17) & 8) return 1;
	return 0;
}

// clears the interrupt on the PWM board
int clear_IRQ(struct PWM_board* board)
{
	in8(board->base_addr+16); // clear interrupt status
}

// sets the IRQ frequency generation from counter 2 on the PWM board
int set_IRQ_freq(struct PWM_board* board, float freq)
{
	// check for invalid frequencies
	if ((freq < 125) || (freq > 4e4))
	{
		printf("Invalid frequency specified (from 125 Hz - 40 kHz).\n");
	}

	// setup counter 2 to a rate of freq Hz
	configure_counter(board,2,LSBMSB,2,BINARY);
	set_counter(board,2,8e6/freq);
	board->IRQ_freq = 8e6/(8e6/freq);

	return 0;
}
