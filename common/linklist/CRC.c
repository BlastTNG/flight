/* 
 * CRC.c: 
 *
 * This software is copyright 
 *  (C) 2015-2018 University of Toronto, Toronto, ON
 *
 * This file is part of the SuperBIT project, modified and adapted for BLAST-TNG.
 *
 * linklist is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * linklist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Jan 25, 2018 by Javier Romualdez
 */


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "CRC.h" // CRC checks and generators for message validation

#ifdef __cplusplus

extern "C"{

#endif

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
#ifdef __cplusplus

}

#endif
