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
 * motor. This test script initializes the DMM-32DX-AT board, sets the
 * resolution for the IM483, and commands a few pulses after which the
 * board is uninitialized.
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

BYTE result;
DSCB dscb;
DSCCB dsccb;
//DSCCR dsccr;
ERRPARAMS errorParams;
BYTE board_type = DSC_DMM32DX;
BYTE config_bytes;

int main(int argc, char *argv[]) {
	int i;
	ThreadCtl(_NTO_TCTL_IO, 0);
	uint64_t base_addr = 0x300;

	// create pointer to address
	uintptr_t ba = mmap_device_io(16,base_addr);
	if (ba == MAP_DEVICE_FAILED)
	{
		printf("Error: mmap_device_io() failed to map device addresses.");
		return -1;
	}

	// get DSC version
	if ((result = dscInit( DSC_VERSION )) != DE_NONE )
	{
		printf("Error %u: Unable to get DSC version.\n",(unsigned int) result);
		dscGetLastError(&errorParams);
		return 0;
	}
	// ensure that all the members of the structure are initialized to 0.
	memset(&dsccb, 0, sizeof(DSCCB));

	// some DSCCP settings for initialization
	dsccb.base_address = base_addr;
	dsccb.int_level = 4;

	printf("Initializing I/O board...");
	// initialize the board
	if ((result = dscInitBoard(board_type, &dsccb, &dscb)) != DE_NONE)
	{
		printf("Error %u: Unable to initialize.\n",(unsigned int) result);
		dscGetLastError(&errorParams);
		return 0;
	}
	printf("done!\n");

	/* counter stuff
	// counter setup
	out8(ba+8,0x00); //	select counter setting (page 0)
	uint8_t config = in8(ba+10); // get current counter settings
	config |= 0x52; // use 10 kHz clock
	out8(ba+10,config); // apply settings
	if ((result=dscCounterSetRateSingle(dscb,1000,COUNTER_0)) != DE_NONE)
	{
		printf("Counter configuration error %u\n",result);
	}
	for (i=0;i<10000;i++)
	{
		dscCounterRead(dscb,&dsccr);
		printf("%u\n",(unsigned int) dsccr.counter0.value);
	}

	delay(10000);

	uint8_t freq = 1; // counter frequency [Hz]
	uint8_t config, cnt[2];
	unsigned int count;

	// set counter 0 configuration
	out8(ba+8,0x00); //	select counter setting (page 0)
	config = in8(ba+10); // get current counter settings
	//printf("0x%x\n",config);
	config |= 0x52; // use 10 kHz clock
	out8(ba+10,config); // apply settings
	//printf("0x%x\n",in8(ba+10));
	//printf("0x%x\n",in8(ba+15));
	out8(ba+15,0x34);
	count = 10000/freq;
	printf("0x%x 0x%x%x\n",count,(uint8_t) (count/256),(uint8_t) count);

	cnt[0] = count;
	cnt[1] = count/256;
	out8(ba+12,cnt[0]); // set counter
	out8(ba+12,cnt[1]); // set counter
	//printf("0x%x\n",in8(ba+15));
	delay(10000);
	*/


	// configure digital I/O to all output
	// set port A and C (0 and 2) to output and port B (1) to input
	config_bytes = 0x82;
	if (dscDIOSetConfig(dscb,&config_bytes) != DE_NONE) printf("Configuration failed.\n");

	// clear all I/O pins initially
	for (i=1;i<=24;i++) dscDIOClearPin(dscb,i);
	delay(100);

	// apply some settings
	printf("Applying some motor settings...\n");
	dscDIOSetPin(dscb,3); // enable outputs
	dscDIOSetPin(dscb,4); // turn off reset

	printf("Driving the stepper motor @ 2 micro/step...\n");
	set_IM483_resolution(dscb,2);
	delay(100);
	pulse_IM483_steps(dscb,1,1000);

	printf("Driving the stepper motor @ 10 micro/step...\n");
	set_IM483_resolution(dscb,10);
	delay(100);
	pulse_IM483_steps(dscb,0,1000);

	printf("Closing I/O board.\n");
	dscFree();

	return EXIT_SUCCESS;
}
