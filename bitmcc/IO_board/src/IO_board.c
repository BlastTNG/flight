/* ---------------------------------------------------------------------
 * ----------- DMM-32DX-AT Digital I/O PC/104 Module Driver ------------
 * ---------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver for the DMM-32DX-AT Digital I/O PC/104 module
 * for QNX. Based on the Diamond Systems Universal Driver (DSUD), this
 * driver allows for easier initialization of the I/O board and handling
 * of board properties.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 24, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <sys/neutrino.h>

#include "dscud.h"
#include "IO_board.h"

// runs through the DSC routines for I/O board initialization
int init_IO(struct IO_board* board, uint64_t addr, uint8_t IRQ)
{
	BYTE result;

	// get DSC version
	if ((board->result = dscInit( DSC_VERSION )) != DE_NONE )
	{
		printf("Error %u: Unable to get DSC version.",(unsigned int) board->result);
		dscGetLastError(&board->errorParams);
		return -1;
	}

	// ensure that all the members of the structure are initialized to 0.
	memset(&board->dsccb, 0, sizeof(DSCCB));

	// some DSCCB settings for initialization
	board->dsccb.base_address = addr;
	board->dsccb.int_level = IRQ;
	board->dsccb.DAC_Config = 0;

	board->base_addr = addr;
	board->IRQ_level = IRQ;

	// initialize the board
	if ((board->result = dscInitBoard(DSC_DMM32DX, &board->dsccb, &board->dscb)) != DE_NONE)
	{
		printf("Error %u: Unable to initialize board.",(unsigned int) board->result);
		dscGetLastError(&board->errorParams);
		return -1;
	}
	dscCancelOp(board->dscb); // close any leftover DSCB processes...

	// set DIO configuration for pivot motor
	BYTE config_bytes = 0x81; // all are output except for C0-C3
	if (dscDIOSetConfig(board->dscb,&config_bytes) != DE_NONE)
	{
		printf("Configuration failed.\n");
		return -1;
	}

	// set analog out configuration for RW controller (-10V<VOUT0<10V)
	// set some DSC settings for D/A conversions
	board->dscdasettings.polarity = BIPOLAR;
	board->dscdasettings.load_cal = TRUE;
	board->dscdasettings.range = 10.0;

	// apply DSC D/A settings
	if((result = dscDASetSettings(board->dscb,&(board->dscdasettings))) != DE_NONE )
	{
		printf("Error %u: Unable to apply DSC settings.\n",(unsigned int) result);
		dscGetLastError(&(board->errorParams));
		return -1;
	}

	// set analog in configuration for RW controller (-10V<VIN0<10V)
	memset(&(board->dscadsettings), 0, sizeof(DSCADSETTINGS));
	board->dscadsettings.current_channel = 0;
	board->dscadsettings.range = RANGE_10;
	board->dscadsettings.polarity = BIPOLAR;
	board->dscadsettings.gain = GAIN_1;
	board->dscadsettings.load_cal = (BYTE)TRUE;
	board->dscadsettings.addiff = 1;

	// apply DSC A/D settings
	if((result = dscADSetSettings(board->dscb,&(board->dscadsettings))) != DE_NONE )
	{
		printf("Error %u: Unable to apply DSC settings.",(unsigned int) result);
		dscGetLastError(&(board->errorParams));
		return 0;
	}

	// setup DSC A/D scan settings
	memset(&(board->dscadscan), 0, sizeof(DSCADSCAN));
	board->dscadscan.low_channel = 0;
	board->dscadscan.high_channel = 15;
	board->dscadscan.gain = board->dscadsettings.gain;
	board->dscadscan.sample_values = (DSCSAMPLE*)malloc(sizeof(DSCSAMPLE)*(board->dscadscan.high_channel - board->dscadscan.low_channel+1));
	memset(board->dscadscan.sample_values, 0, sizeof(DSCSAMPLE)*(board->dscadscan.high_channel-board->dscadscan.low_channel+1));

	return 0;
}

// closes the I/O board
int close_IO(struct IO_board* board)
{
	dscCancelOp (board->dscb); // stops all interrupting functions
	dscFree(); // free up the DSC
	return 0;
}

// sets a particular DIO pin to HIGH
BYTE dscDIOSetPin(DSCB board, unsigned int pin)
{
	if ((pin < 1) || (pin > 24)) return 1; // check for invalid pins
	BYTE port = (pin-1)/8;
	BYTE bit = 7-((pin-1)%8);
	//printf("%d %d\n",port,bit);
	return dscDIOSetBit(board,port,bit);
}

// sets a particular DIO pin to LOW
BYTE dscDIOClearPin(DSCB board, unsigned int pin)
{
	if ((pin < 1) || (pin > 24)) return 1; // check for invalid pins
	BYTE port = (pin-1)/8;
	BYTE bit = 7-((pin-1)%8);
	//printf("%d %d\n",port,bit);
	return dscDIOClearBit(board,port,bit);
}

// reads the digital value of a particular DIO pin
BYTE dscDIOInputPin(DSCB board, unsigned int pin, BYTE* digital_value)
{
	if ((pin < 1) || (pin > 24)) return 1; // check for invalid pins
	BYTE port = (pin-1)/8;
	BYTE bit = 7-((pin-1)%8);
	return dscDIOInputBit(board,port,bit,digital_value);
}
