/* 
 * pointing.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
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
 * Created on: Mar 23, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_POINTING_H_
#define INCLUDE_POINTING_H_

#include "channels_tng.h"

#define GY_HISTORY_AGE_CS 4400

#define FLOAT_ALT 30480
#define FRAMES_TO_OK_ATFLOAT 100

#define OFFSET_GY_IFEL   (0)
#define OFFSET_GY_IFROLL (0)
#define OFFSET_GY_IFYAW  (0)

#define FIR_LENGTH (60*30 * SR)
#define GPS_FIR_LENGTH (60*30 * 1)

/* Calibrations of the az of each sensor  */
#define PSS_ALIGNMENT     0.0
// PSS azimuth in deg
#define PSS0_BETA    PSS_ALIGNMENT + 180.0 - 15.0
#define PSS1_BETA    PSS_ALIGNMENT + 180.0 - 36.4
#define PSS2_BETA    PSS_ALIGNMENT + 180.0 - 79.3
#define PSS3_BETA    PSS_ALIGNMENT + 79.3
#define PSS4_BETA    PSS_ALIGNMENT + 36.4
#define PSS5_BETA    PSS_ALIGNMENT + 15.0

// PSS elevation in deg
#define PSS0_ALPHA 25.0
#define PSS1_ALPHA 25.0
#define PSS2_ALPHA 25.0
#define PSS3_ALPHA 25.0
#define PSS4_ALPHA 25.0
#define PSS5_ALPHA 25.0

// PSS roll in deg
#define PSS0_PSI 180.0
#define PSS1_PSI 180.0
#define PSS2_PSI 180.0
#define PSS3_PSI 180.0
#define PSS4_PSI 180.0
#define PSS5_PSI 180.0

// distance from pinhole to sensor in mm
#define PSS0_D 10.12
#define PSS1_D 10.12
#define PSS2_D 10.12
#define PSS3_D 10.12
#define PSS4_D 10.12
#define PSS5_D 10.12

#define PSS_L  10.0     // 10 mm = effective length of active area
#define PSS_IMAX  8192.  // Maximum current (place holder for now)
#define PSS_XSTRETCH  1.  // 0.995
#define PSS_YSTRETCH  1.  // 1.008

#define SSS_ALIGNMENT     1.5532

#define NUM_CHARS_CHAN_P_ICC   128

typedef struct {
    void *pval;
    char ch_name[NUM_CHARS_CHAN_P_ICC];
    E_TYPE var_type;
    channel_t *ch;
} read_icc_t;

void set_position(double m_lat, double m_lon);
void SetRaDec(double ra, double dec);
void SetSafeDAz(double ref, double *A);
void UnwindDiff(double ref, double *A);
void trim_xsc(int);

#endif /* INCLUDE_POINTING_H_ */
