#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/neutrino.h>
#include "dscud.h"
#include "IO_board.h"
#include "PWM_board.h"
#include "ENIX50_RW_enc.h"

struct IO_board io;
struct PWM_board pwm;
struct rw_encoder rw_enc;
volatile uint32_t ADCS_clock = 0;

int main()
{
	ThreadCtl(_NTO_TCTL_IO, 0); // request I/O privileges for the thread
	struct sigevent event; // event for attached interrupts

	int CLK_FREQ = 10000.0; // clock frequency [Hz]

	init_IO(&io,IO_ADDR_DEFAULT,IO_IRQ_DEFAULT);
	init_PWM(&pwm,PWM_ADDR_DEFAULT);
	init_rw_encoder(&rw_enc,&io);

	enable_IRQ(&pwm); // enable interrupts on PWM board
	set_IRQ_freq(&pwm,CLK_FREQ); // set clock frequency
	ADCS_clock = 0;

	// set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	// attach interrupt level 7
	int interruptID = InterruptAttachEvent (PWM_IRQ_DEFAULT, &event, _NTO_INTR_FLAGS_TRK_MSK);
	if (interruptID < 0) printf("Server : InterruptAttachEvent() failed\n");


	while (1)
	{
		InterruptWait (0, NULL);
		InterruptUnmask (PWM_IRQ_DEFAULT,interruptID); // unmask the PWM clock interrupt

		if ((ADCS_clock%(CLK_FREQ/RW_ENC_UPDATE_FREQ)) == 0)
		{
			pulse_rw_encoder(&rw_enc,CLK_FREQ);
			if ((ADCS_clock%CLK_FREQ) == 0) printf("RW Speed: %g rad/s\n",rw_enc.speed);
		}
		ADCS_clock += 1; // increment the clock
		clear_IRQ(&pwm);
	}


	InterruptDetach(interruptID); // detach the clock interrupt
	disable_IRQ(&pwm); // disable clock on PWM board
	close_IO(&io);
	close_PWM(&pwm);

	printf("Hardware closed.\n");
	return 0;
}
