/* -----------------------------------------------------------------------
 * ------------------- K089150 Frameless Motor Driver --------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver created to run the the K089150 frameless
 * motors via the AMC AZbdc12a8 driver. The motor is driven via PWM
 * command from the DM6916 PWM board.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: September 3, 2013
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
#include "K089150_frame.h"

// initializes frameless motor fm on initialized PWM board at specific PWM port
int init_frameless(struct frameless_motor * fm, struct PWM_board * pwm, uint8_t port)
{
	fm->port = port; // assign PWM port
	fm->pwm = pwm; // point to PWM board
	enable_PWM_port(fm->pwm,fm->port); // enable PWM port
	command_torque_frameless(fm,0); // zero out torque

	return 0;
}

// closes frameless motor fm
int close_frameless(struct frameless_motor * fm)
{
	command_torque_frameless(fm,0); // zero out torque
	disable_PWM_port(fm->pwm,fm->port); // disable PWM port
	return 0;
}

// commands torque for specific frameless motor
int command_torque_frameless(struct frameless_motor * fm, double torque)
{
	int duty_cycle;

	// convert torque to current
	double current = torque/FRAMELESS_MOTOR_CNST; // MAY NEED TO SWITCH SIGN HERE

	// compute the duty cycle
	duty_cycle = (255*current)/FRAMELESS_MOTOR_MAX_CURRENT;

	// saturate duty cycle at max current
	if (duty_cycle > 255) duty_cycle = 255;
	else if (duty_cycle < -255) duty_cycle = -255;
	printf("Duty cycle: %d\n",duty_cycle);

	// set the duty cycle
	set_duty_cycle(fm->pwm, fm->port, duty_cycle);

	return 0;
}

