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

#define BI0_MAX_BUFFER_SIZE 250000 // maximum frame size at 1 Hz (i.e. 2 Mbits)
#define BI0_ZERO_PADDING 1200 // number of bytes =0 to pad every 100 Hz

extern uint8_t bi0_idle;

extern pthread_t watchdog_id;

void biphase_writer(void * arg);


#endif /* INCLUDE_BI0_H_ */
