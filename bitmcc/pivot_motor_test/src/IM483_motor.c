/* -----------------------------------------------------------------------
 * ------------------- IM483-34P1 Stepper Motor Driver -------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver created to run the the stepper motor using the
 * DMM-32DX-AT Analog/Digital I/O board. This driver is a supplement to
 * the DSC Universal Driver to make it more usable for the IM483 stepper
 * motor.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: July 17, 2013
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

#include "dscud.h"
#include "IO_board.h"
#include "IM483_motor.h"

// pulses the IM483 motor n times in the given direction (0 or 1)
int pulse_IM483_steps(DSCB board, unsigned int dir, unsigned int n)
{
	int i;

	for (i=0;i<n;i++)
	{
		pulse_IM483(board,dir);
		delay(1);
	}

	return 0;
}

// sends a single pulse to the IM483 motor in the given direction (0 or 1)
int pulse_IM483(DSCB board, unsigned int dir)
{
	if (dir) dscDIOSetPin(board,2); // set the direction
	else dscDIOClearPin(board,2); // set the direction

	dscDIOSetPin(board,1); // pulse the motor
	dscDIOClearPin(board,1); // pulse the motor

	return 0;
}

// sets the resolution of the stepper motor to N microsteps per step
// rounds to the nearest power of 2 from 2 to 256
int set_IM483_resolution(DSCB board,float N)
{
	// check for invalid values
	if (N<2 || N>256)
	{
		printf("Invalid resolution (must be from 2-256)\n");
		return -1;
	}
	float temp = round(log(N)/0.6931471806)-1;
	unsigned int set = temp; // setting for resolution
	//printf("%u\n",set);

	// resolution select 0
	if (set&1) dscDIOSetPin(board,5);
	else dscDIOClearPin(board,5);

	// resolution select 1
	if (set&2) dscDIOSetPin(board,6);
	else dscDIOClearPin(board,6);

	// resolution select 2
	if (set&4) dscDIOSetPin(board,7);
	else dscDIOClearPin(board,7);

	// resolution select 3
	dscDIOClearPin(board,8); // use binary settings (set pin for decimal)

	return 0;
}
