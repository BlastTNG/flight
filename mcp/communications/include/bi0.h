/* fcp: the EBEX flight control program
 *
 * This software is copyright (C) 2009 Columbia University
 *                            (C) 2016 University of Pennsylvania
 *
 * This file is part of fcp.
 *
 * fcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_BI0_H_
#define INCLUDE_BI0_H_

#include <stdint.h>

#define BI0_FRAME_BUFBITS (4)
#define BI0_FRAME_BUFLEN (1 << BI0_FRAME_BUFBITS)
#define BI0_FRAME_BUFMASK (BI0_FRAME_BUFLEN-1)
// TODO(javier): make allframe period commandable
#define BI0_ALLFRAME_PERIOD 10 // number of seconds between sequential allframes

#define BI0_MAX_BUFFER_SIZE (2*superframe_size) // maximum frame size at 1 Hz (i.e. 2 Mbits)
#define BI0_ZERO_PADDING 250 // number of bytes =0 to pad at the end of every packet sent 

#define BI0LOS_FLC_ADDR "192.168.1.200"
#define BI0LOS_FLC_PORT 50000
#define BI0LOS_BUFFER_PORT 50100
#define BI0LOS_GND_ADDR "192.168.1.201"
#define BI0LOS_GND_PORT 51515
#define BI0LOS_MAX_PACKET_SIZE 1388


extern struct Fifo bi0_fifo;

extern pthread_t watchdog_id;

void biphase_writer(void * arg);

#endif /* INCLUDE_BI0_H_ */
