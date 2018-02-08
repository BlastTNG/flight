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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "linklist_crc.h" // CRC checks and generators for message validation

unsigned short *crctable = NULL;

// generates and returns a CRC table for serial validation of DPRALTE
unsigned short *mk_crctable(unsigned short poly, unsigned short (*crcfn) (unsigned short, unsigned short, unsigned short))
{
	unsigned short *crctable;
	int i;
	if((crctable = (unsigned short *)malloc(256*sizeof(unsigned))) == NULL)
	{
		return NULL;
	}
	for(i=0; i < 256; i++)
	{
		crctable[i] = (*crcfn)(i,poly,0);
	}
	return crctable;
}

// generator for CRC table
unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum)
{
	static int i;
	data <<= 8;
	for(i = 8; i > 0; i--)
	{
	if((data ^ accum) & 0x8000)
		accum = (accum << 1 ) ^ genpoly;
	else
		accum <<=1;
	data <<=1;
	}
	return accum;
}

// checks/generates a CRC value for received/sent message
void crccheck(unsigned short data, unsigned short *accumulator, unsigned short *crctable)
{
	*accumulator = ( *accumulator << 8 ) ^ crctable[( *accumulator >> 8) ^ data];
}

