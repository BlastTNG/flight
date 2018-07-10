/* 
 * pilot.h: 
 *
 * This software is copyright 
 *  (C) 2015-2018 University of Toronto, Toronto, ON
 *
 * This file is part of mcp, as used for the BLAST-TNG project.
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
 * Created on: Feb 15, 2018 by Javier Romualdez
 */

#ifndef INCLUDE_PILOT_H
#define INCLUDE_PILOT_H

#define PILOT_PORT 31213 // port that pilot data is sent to
#define PILOT_ADDR "192.168.1.55" // address that pilot data is sent to (blastgs1 for gnd ops)
// #define PILOT_ADDR "192.168.1.56" // address that pilot data is sent to (blastgs2 for gnd ops)
#define PILOT_MAX_PACKET_SIZE 1024 // maximum size of a packet to be sent over Pilot
#define PILOT_MAX_SIZE (superframe->size*2) // maximum compressed frame size to be send over Pilot
// TODO(javier): make allframe period commandable
#define PILOT_ALLFRAME_PERIOD 10 // number of seconds between sequential allframes

extern struct Fifo pilot_fifo;

void pilot_compress_and_send(void *);


#endif /* INCLUDE_PILOT_H */

