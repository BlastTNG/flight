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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <packet_slinger.h>

#include <blast.h>

#include "bi0.h"
#include "channels_tng.h"


bi0_buffer_t bi0_buffer;

void initialize_bi0_writer(void)
{
	int i;
	size_t max_rate_size = 0;

	//for (int rate = RATE_100HZ; rate < RATE_END; rate++) {
	//    if (max_rate_size < frame_size[rate]) max_rate_size = frame_size[rate];
	//}
    max_rate_size = frame_size[RATE_100HZ]; // Temporary, for now only implementing biphase on the 100 Hz framerate
	bi0_buffer.i_in = 10; /* preload the fifo */
	bi0_buffer.i_out = 0;
	for (i = 0; i < BI0_FRAME_BUFLEN; i++)
	{
		bi0_buffer.framelist[i] = calloc(1, max_rate_size);
		memset(bi0_buffer.framelist[i], 0, max_rate_size);
	}

}

void push_bi0_buffer(const void *m_frame)  // what was there before uint16_t *m_frame
{
	int i_in;

	i_in = (bi0_buffer.i_in + 1) & BI0_FRAME_BUFMASK;    // Does this automatically wrap when i_in becomes greater than the number of frames? I think so from BUFMASK
    bi0_buffer.framesize[i_in] = frame_size[RATE_100HZ]; // Joy added this, later will have to be modified for any rate
    memcpy(bi0_buffer.framelist[i_in], m_frame, frame_size[RATE_100HZ]; // Later will have to adapt to various sizes for various frequencies
	bi0_buffer.i_in = i_in;
}
