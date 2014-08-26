/* 
 * uei_of.h: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 6, 2014 by seth
 */

#ifndef UEI_OF_H_
#define UEI_OF_H_

typedef struct _gyro_data gyro_data_t;

#define GYRO_VALID(_x) (((_x) >> 31) & 1)
#define GYRO_SYNC(_x)  (((_x) >> 29) & 3)
#define GYRO_TYPE(_x)  (((_x) >> 27) & 3)
#define GYRO_PARITY(_x) (((_x) >> 26) & 1)
#define GYRO_PARITY_MASK 0x4000000

#define GYRO_CONTENT_MASK 0x3ffffff
#define GYRO_CONTENT(_x) (((int)((_x) & GYRO_CONTENT_MASK)<<6)>>6)

//related to analog channels
#define SR          1000                               //"10kHz" interrupt rate (4MHz/384)
#define N_CH        25                                  //number of channels (per daughter)
#define N_STAGES    4                                   //number of filter stages
#define N_GYRO      6                                   //number of gyros (filtered channels on digital card)


typedef enum
{
    GYRO_SYNCHRONIZED       = 0,
    GYRO_INSUFFICIENT_DATA  = -1,
    GYRO_CANT_SYNC          = -2,
} gyro_returns_t;


#endif /* UEI_OF_H_ */
