/*
 * @file conversions.h
 *
 * @date Jun 12, 2011
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2010 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_CONVERT_H_
#define INCLUDE_CONVERT_H_

#include <time.h>
#include <math.h>

#define RX_CONVERT(TYPE, VAL) ((VAL) * CONVERT_##TYPE##_M + CONVERT_##TYPE##_B)
#define TX_CONVERT(TYPE, VAL) (((VAL) - CONVERT_##TYPE##_B) / CONVERT_##TYPE##_M)

#define NARROW_MAX (65535.0)
#define WIDE_MAX (4294967295.0)
#define CONVERT_MICROSECONDS_M (1000000.0/NARROW_MAX)
#define CONVERT_MICROSECONDS_B (0.0)
#define CONVERT_VEL_M (200.0/NARROW_MAX)
#define CONVERT_VEL_B (-100.0)
#define CONVERT_TEMP_M (473.15/NARROW_MAX)
#define CONVERT_TEMP_B (-273.15)
#define CONVERT_PRES_M (5.0/NARROW_MAX)
#define CONVERT_PRES_B (0.0)
#define CONVERT_GB_M (2000.0/NARROW_MAX)
#define CONVERT_GB_B (0.0)
#define CONVERT_DISK_SPACE_TB_M (20000.0/NARROW_MAX)
#define CONVERT_DISK_SPACE_TB_B (0.0)
#define CONVERT_ANGLE_M ((4.0*M_PI/2.0)/NARROW_MAX)
#define CONVERT_ANGLE_B (-M_PI/2.0)
#define CONVERT_WIDE_ANGLE_M ((4.0*M_PI/2.0)/WIDE_MAX)
#define CONVERT_WIDE_ANGLE_B (-M_PI/2.0)
#define CONVERT_ANGLE_DEG_M ((4.0*180.0/2.0)/NARROW_MAX)
#define CONVERT_ANGLE_DEG_B (0.0)
#define CONVERT_BLOB_POS_M (1536.0/NARROW_MAX)
#define CONVERT_BLOB_POS_B (-768.0)
#define CONVERT_FEW_SECONDS_M (300.0/NARROW_MAX)
#define CONVERT_FEW_SECONDS_B (0.0)
#define CONVERT_0_TO_10_M (10.0/NARROW_MAX)
#define CONVERT_0_TO_10_B (0.0)
#define CONVERT_0_TO_20_M (20.0/NARROW_MAX)
#define CONVERT_0_TO_20_B (0.0)
#define CONVERT_BATTV_M (100.0/NARROW_MAX)
#define CONVERT_BATTV_B (0.0)
#define CONVERT_BATTI_M (200.0/NARROW_MAX)
#define CONVERT_BATTI_B (-100.0)
#define CONVERT_UNITY_M 1.0
#define CONVERT_UNITY_B 0.0

#define H2LI (4294967296.0/24.0)
#define LI2H (1.0/H2LI)
#define DEG2LI (4294967296.0/360.0)
#define LI2DEG (1.0/DEG2LI)
#define RAD2LI (4294967296.0/2/M_PI)
#define DEG2I (65536.0/360.0)
#define I2DEG (1.0/DEG2I)
#define RAD2I (65536.0/2/M_PI)
#define H2I (65536.0/24.0)
#define I2H (1.0/H2I)
#define VEL2I (65536.0/10.0)
#define I2VEL (1.0/VEL2I)

#define H2LI (4294967296.0/24.0)
#define LI2H (1.0/H2LI)
#define DEG2LI (4294967296.0/360.0)
#define LI2DEG (1.0/DEG2LI)
#define RAD2LI (4294967296.0/2/M_PI)
#define DEG2I (65536.0/360.0)
#define I2DEG (1.0/DEG2I)
#define RAD2I (65536.0/2/M_PI)
#define H2I (65536.0/24.0)
#define I2H (1.0/H2I)
#define VEL2I (65536.0/10.0)
#define I2VEL (1.0/VEL2I)

#define SOLV2I_M (0.1)
#define SOLV2I_B (-250)
#define SOLV2V_M (0.001*50.9/3.9)
#define SOLV2THERM (0.001/5.0*6200)

#define DEG2RAD         (M_PI/180.0)
#define DEG2HR          (1/15.0)            /* 1/15 */
#define SEC2HR          (1/3600.0)      /* 1/3600 */
#define DAY2SEC         86400.0         /* 3600*24 */
#define SEC2DAY         (1/DAY2SEC)     /* 1/3600/24 */
#define HR2RAD          (M_PI/12.0)         /* 15*PI/180 */
#define RAD2HR          (12.0/M_PI)         /* 12/PI */
#define ARCMIN2RAD      (DEG2RAD/60.0) /* DEG2RAD/60. */
#define RAD2SEC         (180. * 3600. / M_PI / 15.)  /* radians to seconds (of time) */
#define SEC2RAD         (1. / RAD2SEC)
#define RAD2ARCMIN      (60.0/DEG2RAD)    /* 60/DEG2RAD. */
#define RAD2ARCSEC      (60.0*RAD2ARCMIN) /* 60*60/DEG2RAD. */
#define ARCMIN2DEG      (1.0/60.0)      /* 1/60 */
#define DEG2ARCMIN      60.0000000000       /* 60 */
#define ARCMIN2ARCSEC   60.0000000000       /* 60 */
#define ARCSEC2ARCMIN   1.66666666667e-2    /* 1/60 */
#define ARCSEC2RAD      4.8481368111e-6     /* 2PI/360/60/60 */
#define MARCSEC2RAD     4.8481368111e-9     /* 1e-3 * 2PI/360/60/60 */
#define UARCSEC2RAD     4.8481368111e-12    /* 1e-6 * 2PI/360/60/60 */

#define RAD2DEG         (180.0/M_PI)        /* 180./PI */
#define HR2DEG          15.0000000000    /* 15 */
#define FSKY2SQDEG      41252.9612494    /* 360*360/PI */
#define SQDEG2FSKY      2.42406840555e-5 /* PI/360/360 */

#define CONVERT_SIDSLOPE_M (2000.0/NARROW_MAX) /*(NARROW_MAX/2000.0)*/
#define CONVERT_SIDSLOPE_B 0
#define CONVERT_GRTSLOPE_M (-30.0/NARROW_MAX) /*(NARROW_MAX/30.0)*/
#define CONVERT_GRTSLOPE_B 0

static inline double from_hours(double angle)
{
    return angle*M_PI/12.0;
}

static inline double to_hours(double angle)
{
    return angle*12.0/M_PI;
}

static inline double from_degrees(double angle)
{
    return angle*M_PI/180.;
}

static inline double to_degrees(double angle)
{
    return angle*180./M_PI;
}

static inline double from_arcmin(double angle)
{
    return (angle/60.)*M_PI/180.;
}

static inline double to_arcmin(double angle)
{
    return angle*(180./M_PI)*60.0;
}

static inline double from_arcsec(double angle)
{
    return (angle/3600.)*M_PI/180.;
}

static inline double to_arcsec(double angle)
{
    return angle*(180./M_PI)*3600.0;
}

static inline double from_seconds(time_t time)
{
    return ((double) time)*M_PI/(12.0*3600.0);
}

static inline time_t to_seconds(double angle)
{
    return (time_t) ((angle/M_PI)*(12.0*3600.0));
}

#endif /* INCLUDE_CONVERT_H_ */
