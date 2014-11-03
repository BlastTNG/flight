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
 * QNX. This file contains a test main function for the driver which
 * initializes the PWM board, enables a PWM port, sets the PWM duty cycle,
 * disables the port, and closes the PWM board. A timer is also setup
 * between two cascaded 82C54 counters that generates interrupts at a
 * desired rate (base clock is 8 MHz).
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
 * 18/06/13 - added timer functions for interrupt generation based on an
 * 8 MHz clock; IRQ handling functions written
 *
 * 03/09/13 - added digital output functions (set and clear bits)
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


int main()
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread
	InterruptEnable();

	static struct PWM_board pwm;
	int value = 0;

	init_PWM(&pwm,PWM_ADDR_DEFAULT);
	printf("PWM board initialized.\n");

	enable_PWM_port(&pwm,PWM_00);

	unsigned int base = 10000; // signal base for counter 0
	float freq = 50;

	// configure counter 0 and 1 to output 50 Hz signal
	configure_counter(&pwm,0,LSBMSB,2,BINARY);
	set_counter(&pwm,0,8e6/base);
	configure_counter(&pwm,1,LSBMSB,2,BINARY);
	set_counter(&pwm,1,base/freq);

	// set IRQ frequency to 125 Hz
	enable_IRQ(&pwm);
	set_IRQ_freq(&pwm,125);


	unsigned int counter = 0;
	while (1)
	{
		if (check_IRQ(&pwm))
		{
			clear_IRQ(&pwm);
			counter++;
			if ((counter % pwm.IRQ_freq) == 0) printf("%d Mark.\n",pwm.IRQ_freq);
		}

	}



	while (1)
	{
		printf("Enter duty cycle on scale from -255 to 255 (256 to exit):\n");
		scanf ("%d",&value);
		printf("\n");
		if (abs(value) == 256) break; // exit request
		else if ((value < -255) || (value > 255)) printf("Invalid value.\n");
		else
		{
			set_duty_cycle(&pwm,PWM_00,value); // set duty cycle of port 00
			printf("Duty cycle set to %d.\n",value);

		}
	}
	disable_IRQ(&pwm);
	disable_PWM_port(&pwm,PWM_00);

	close_PWM(&pwm);
	printf("Closed PWM board.\n");
	return 0;
}



