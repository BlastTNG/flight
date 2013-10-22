#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/neutrino.h>

#include "dscud.h"
#include "IO_board.h"
#include "ENIX50_RW_enc.h"

// initializes the RW encoder to a particular IO board
int init_rw_encoder(struct rw_encoder *enc, struct IO_board *io_board)
{
	enc->io = io_board; // assign IO board

	// initial values for counter and speed
	enc->counter = 0;
	enc->speed = 0;

	// ensure that first pulse is after at least half a cycle
	enc->AXORB_old = 1;
	enc->A_old = 1;

	return 0;
}

// pulses the RW encoder in order to get speed readings
// this function must be called on interrupt at CLK_FREQ to get correct results
// speed will be updated and available as soon as the first rising edge of AXORB is encountered
int pulse_rw_encoder(struct rw_encoder *enc, float freq)
{
	// get the current readings from RW encoder
	dscDIOInputPin(enc->io->dscb,RW_ENC_A_PIN,&enc->A);
	dscDIOInputPin(enc->io->dscb,RW_ENC_B_PIN,&enc->B);
	dscDIOInputPin(enc->io->dscb,RW_ENC_Z_PIN,&enc->Z);
	enc->AXORB = enc->A^enc->B;

	if (enc->AXORB > enc->AXORB_old) // rising edge of AXORB
	{
		enc->speed = 1.0*PI*(freq/enc->counter)/RW_ENC_RES; // using AXORB, so 2x resolution
		if (enc->A < enc->A_old) enc->speed = enc->speed*-1;
		enc->counter = 0; // reset counter
	}
	enc->AXORB_old = enc->AXORB;
	enc->A_old = enc->A;
	(enc->counter)++; // increment counter

	return 0;
}
