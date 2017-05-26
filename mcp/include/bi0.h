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


typedef struct
{
    int i_in;
    int i_out;
    uint8_t *framelist[BI0_FRAME_BUFLEN];
    size_t framesize[BI0_FRAME_BUFLEN];
} biphase_frames_t;

extern biphase_frames_t biphase_frames;
extern uint8_t *biphase_superframe_in; // This is pushed to biphase_frames

extern pthread_t watchdog_id;

void initialize_biphase_buffer(void);
void push_biphase_frames(void);
void biphase_writer(void);

void build_biphase_frame_200hz(const void *m_channel_data);
void build_biphase_frame_100hz(const void *m_channel_data);
void build_biphase_frame_1hz(const void *m_channel_data);

#endif /* INCLUDE_BI0_H_ */
