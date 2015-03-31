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

///TODO: Move Gyro Defs to gyro header file
#define GYRO_VAR (2.0E-6)
#define GY_HISTORY_AGE_CS 4400

//TODO: Characterize Magnetometer
#define MAGX_B 0.0
#define MAGX_M 1.0
#define MAGY_B 0.0
#define MAGY_M 1.0

#define FLOAT_ALT 30480
#define FRAMES_TO_OK_ATFLOAT 100

#define OFFSET_GY_IFEL   (0)
#define OFFSET_GY_IFROLL (0)
#define OFFSET_GY_IFYAW  (0)

#define FIR_LENGTH (60*30 * SR)
#define GPS_FIR_LENGTH (60*30 * 1)
//#define FIR_LENGTH (30 * SR)
//#define GPS_FIR_LENGTH (30 * 1)

/* Calibrations of the az of each sensor  */
/*#define MAG_ALIGNMENT   183.   //(4.2681)
#define PSS1_ALIGNMENT     43. // 343 + 60
#define PSS2_ALIGNMENT    120. // 135 -15
#define SSS_ALIGNMENT     -15.*/
//#define MAG_ALIGNMENT    -258.   //(4.2681)
//#define PSS1_ALIGNMENT      -153.8 // 343 + 60
//#define PSS2_ALIGNMENT      -226.3 // 135 -15
//#define SSS_ALIGNMENT   -90.
//#define DGPS_ALIGNMENT    3.65

#define MAG_ALIGNMENT     -28.0
#define PSS_ALIGNMENT     0.0
#define PSS1_ALIGNMENT    (PSS_ALIGNMENT - 50.0)
#define PSS2_ALIGNMENT    (PSS_ALIGNMENT - 85.0 + 3.9)
#define PSS3_ALIGNMENT    (PSS_ALIGNMENT - 120.0 + 3.9 + 0.53)
#define PSS4_ALIGNMENT    (PSS_ALIGNMENT - 155.0 + 3.9 + 0.53 + 3.31)

#define SSS_ALIGNMENT     1.5532

void SetRaDec(double ra, double dec);
#endif /* INCLUDE_POINTING_H_ */
