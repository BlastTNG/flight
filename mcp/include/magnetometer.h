/**
 * @file magnetometer.h
 *
 * @date Nov 23, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_MAGNETOMETER_H_
#define INCLUDE_MAGNETOMETER_H_

void initialize_magnetometer(void);

// TODO(anyone): Characterize Magnetometer
#define MAGX_B 0.0
#define MAGX_M 1.0
#define MAGY_B 0.0
#define MAGY_M 1.0

/* Magnetometer Az Calibration */
#define MAG_ALIGNMENT     -28.0

// convert mag readings to sine and cosine
// calibrated in Palestine, July 11, 2010
// Best fit to mag_x and mag_y
// y = -3000*sin(x-19)+33050 : mag_x
// y = 3000*cos(x-19)+33310 : mag_y
// x is dgps theta in degrees.
// The defines for x and y are no longer used.
// #define MAGX_M (-1.0/1290.0)
// #define MAGX_B (33500/1290.0)
// #define MAGY_M (-1.0/1290.0)
// #define MAGY_B (33400.0/1290.0)
#define MAGZ_M (-1/1290.0)
#define MAGZ_B (32768.0)

#define FAST_MAG

#endif /* INCLUDE_MAGNETOMETER_H_ */
