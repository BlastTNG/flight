/* 
 * uei_gyros.h
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of uei, created for the BLASTPol Project.
 *
 * uei is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * uei is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with uei; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 26, 2014 by seth
 */
 

#ifndef UEI_GYROS_H_
#define UEI_GYROS_H_

#include <PDNA.h>

typedef struct _gyro_data gyro_data_t;

#define GYRO_VALID(_x) (((_x) >> 31) & 1)
#define GYRO_SYNC(_x)  (((_x) >> 29) & 3)
#define GYRO_TYPE(_x)  (((_x) >> 27) & 3)
#define GYRO_PARITY(_x) (((_x) >> 26) & 1)
#define GYRO_PARITY_MASK 0x4000000

#define GYRO_CONTENT_MASK 0x3ffffff
/// GYRO_CONTENT takes a 26-bit value, left shifts to push the sign bit into the 32nd bit and restores the absolute value
#define GYRO_CONTENT(_x) (((int)((_x) & GYRO_CONTENT_MASK)<<6)>>6)
#define N_GYRO      6                                   //number of gyros (filtered channels on digital card)

// channel configuration helper macros
#define GYRO_CHANNEL_CFG DQCFG_501(DQ_SL501_OPER_NORM, \
								  DQ_SL501_MODE_232, \
								  DQ_SL501_BAUD_115200, \
								  DQ_SL501_WIDTH_8, \
								  DQ_SL501_STOP_1, \
								  DQ_SL501_PARITY_NONE)

static const int gyro_channel[6] = {0, 1, 2, 3, 4, 5};

typedef enum
{
    GYRO_SYNCHRONIZED       = 0,
    GYRO_INSUFFICIENT_DATA  = -1,
    GYRO_CANT_SYNC          = -2,
} gyro_returns_t;

void uei_gyro_get_vals (float *m_roll, float *m_yaw, float *m_el);
void gyro_read_routine(void* arg);

#endif /* UEI_GYROS_H_ */
