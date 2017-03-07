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

// TODO(Joy): Check with Seth if those numbers make sense for BLAST-TNG
#define BBC_MASTER_CLK         32000000       /* set by the oscilator */
#define BBC_MCLKS_PER_BBC_CLK  8
#define BI0_MCLKS_PER_BI0_CLK   32
#define BBC_ADC_MULTIPLIER     384            /* set by the ADC hardware */
#define BBC_ADCS_PER_SAMPLE    104            /* this sets the frame size */

#define BI0_WORD_SIZE 16
#define BI0_FRAME_SIZE ((BBC_MCLKS_PER_BBC_CLK * BBC_ADC_MULTIPLIER * BBC_ADCS_PER_SAMPLE) \
                      / (BI0_MCLKS_PER_BI0_CLK * BI0_WORD_SIZE))

typedef struct
{
    int i_in;
    int i_out;
    uint8_t *framelist[BI0_FRAME_BUFLEN];
    size_t framesize[BI0_FRAME_BUFLEN];
} bi0_buffer_t;

extern bi0_buffer_t bi0_buffer;

extern int16_t InCharge;
extern pthread_t watchdog_id;

void initialize_biphase_buffer(void);
void push_bi0_buffer(const void *m_frame);
void biphase_writer(void);

#endif /* INCLUDE_BI0_H_ */
