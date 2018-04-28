/* 
 * CRC.h: 
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


#ifndef INCLUDE_CRC_FUNC_H
#define INCLUDE_CRC_FUNC_H

#define CRC_POLY 0x1021		// polynomial basis for CRC generation

#ifdef __cplusplus
extern "C" {
#endif


// define some global variables
extern uint16_t *crctable;

// define some function prototypes

uint16_t *mk_crctable(uint16_t, uint16_t (*crcfn)(uint16_t, uint16_t, uint16_t));
uint16_t crchware(uint16_t, uint16_t, uint16_t);
void crccheck(uint16_t, uint16_t *, uint16_t *);

#ifdef __cplusplus
}

#endif

#endif /* INCLUDE_CRC_FUNC_H */
