/* -----------------------------------------------------------------------
 * ---------------------------- CRC FUNCTIONS ----------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * These are a few simple functions used for generating and checking
 * CRC values for message validation.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: August 7, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */

#ifndef CRC_FUNC_H_
#define CRC_FUNC_H_

#define CRC_POLY 0x1021		// polynomial basis for CRC generation

#ifdef __cplusplus

extern "C" {

#endif


// define some global variables
extern unsigned short *crctable;

// define some function prototypes

unsigned short *mk_crctable(unsigned short , unsigned short (*crcfn) (unsigned short, unsigned short, unsigned short));
unsigned short crchware(unsigned short , unsigned short , unsigned short );
void crccheck(unsigned short , unsigned short * , unsigned short * );

#ifdef __cplusplus

}

#endif

#endif /* CRC_FUNC_H_ */
