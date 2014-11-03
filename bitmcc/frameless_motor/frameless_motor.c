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
 * motors via the AMC AZBDC12A8 driver. The motor is driven via PWM
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
#include <pthread.h>

#include "PWM_board.h"
#include "K089150_frame.h"

static struct PWM_board pwm;
static struct frameless_motor fm;

float amp = 1.0; // amplitude of the square wave [Nm]
float freq = 5; // frequency of the square wave [Hz]
float CLK_FREQ = 200; // clock frequency [Hz]

void * square_wave( )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	int period = CLK_FREQ/freq; // the period of the square wave [ticks]

	int counter = 0;

	struct sigevent event;

	// set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	// attach interrupt level 7
	int interruptID = InterruptAttachEvent (PWM_IRQ_DEFAULT, &event, _NTO_INTR_FLAGS_TRK_MSK);
	if (interruptID < 0) printf("Server : InterruptAttachEvent() failed\n");

	sleep(1);

	// user specifies motor torque
	while (1)
	{
		InterruptWait (0, NULL);
		InterruptUnmask (PWM_IRQ_DEFAULT,interruptID); // unmask the PWM clock interrupt
		clear_IRQ(&pwm);

		//printf("%d\n",counter);

		if ((counter%period) == 0)
		{
			amp = amp*-1;
			//printf("Wave: %4.2f s, %4.2f Nm\n",((float) counter)/CLK_FREQ,amp);
			command_torque_frameless(&fm,amp);
		}
		counter++;
	}
	InterruptDetach(interruptID);
}

int main()
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	pthread_t sq_wave_thread;

	// initialize hardware
	init_PWM(&pwm,PWM_ADDR_DEFAULT);
	init_frameless(&fm,&pwm,PWM_00);

	// start a clock
	enable_IRQ(&pwm); // enable interrupts on PWM board
	set_IRQ_freq(&pwm,CLK_FREQ); // set clock frequency to CLK_FREQ Hz

	printf("Hardware initialized. Generating a %2.0f Hz square wave...\n",freq);

	// start square wave thread
	pthread_create(&sq_wave_thread, NULL, &square_wave, NULL);

	printf("Press ENTER to terminate square wave.\n");
	char c = getchar();

	disable_IRQ(&pwm); // disable interrupts on PWM board

	// close hardware
	close_frameless(&fm);
	close_PWM(&pwm);
	printf("Closed hardware.\n");

	return 0;
}
