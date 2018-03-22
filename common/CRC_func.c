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

#include "CRC_func.h" // CRC checks and generators for message validation

#ifdef __cplusplus
extern "C" {
#endif

uint16_t *crctable = NULL;

// generates and returns a CRC table for serial validation of DPRALTE
uint16_t *mk_crctable(uint16_t poly, uint16_t (*crcfn)(uint16_t, uint16_t, uint16_t))
{
	uint16_t *crctable;
	int i;
	if ((crctable = (uint16_t *)malloc(256*sizeof(unsigned))) == NULL) {
		return NULL;
	}
	for (i = 0; i < 256; i++) {
		crctable[i] = (*crcfn)(i, poly, 0);
	}
	return crctable;
}

// generator for CRC table
uint16_t crchware(uint16_t data, uint16_t genpoly, uint16_t accum)
{
	static int i;
	data <<= 8;
	for (i = 8; i > 0; i--) {
	  if ((data ^ accum) & 0x8000) {
      accum = (accum << 1) ^ genpoly;
	  } else {
      accum <<= 1;
    }
	  data <<= 1;
	}
	return accum;
}

// checks/generates a CRC value for received/sent message
void crccheck(uint16_t data, uint16_t *accumulator, uint16_t *crctable)
{
	*accumulator = (*accumulator << 8) ^ crctable[(*accumulator >> 8) ^ data];
}

#ifdef __cplusplus
}
#endif
