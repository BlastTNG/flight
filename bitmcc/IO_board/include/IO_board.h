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

// define some default memory addresses
#define IO_ADDR_DEFAULT 0x300
#define IO_IRQ_DEFAULT 0x4

#ifndef IO_BOARD_H_
#define IO_BOARD_H_

struct IO_board
{
	uint64_t base_addr;
	uint8_t IRQ_level;
	DSCB dscb;
	DSCCB dsccb;
	ERRPARAMS errorParams;
	BYTE result;
	DSCDASETTINGS dscdasettings;
	DSCADSETTINGS dscadsettings;
	DSCADSCAN dscadscan;
};

// define some function prototypes
int init_IO(struct IO_board* , uint64_t , uint8_t );
int close_IO(struct IO_board* );
BYTE dscDIOSetPin(DSCB , unsigned int );
BYTE dscDIOClearPin(DSCB , unsigned int );
BYTE dscDIOInputPin(DSCB , unsigned int , BYTE* );

#endif /* IO_BOARD_H_ */
