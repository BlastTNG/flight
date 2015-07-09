/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef TOOLS__ANGLES_H
#define TOOLS__ANGLES_H

#define _USE_MATH_DEFINES
#include <math.h>

double from_hours(double angle);
double to_hours(double angle);
double from_degrees(double angle);
double to_degrees(double angle);
double from_arcsec(double angle);
double to_arcsec(double angle);
double from_arcmin(double angle);
double to_arcmin(double angle);
double great_circle(double ra0, double dec0, double ra1, double dec1);
bool angles_within_basic(double ra0, double dec0, double ra1, double dec1, double distance);
bool angles_within(double ra0, double dec0, double ra1, double dec1, double distance);
double tanplane_distance(double x0, double y0, double x1, double y1, double iplatescale);
void equatorial_to_horizontal(double ra, double dec, double lat, double lst, double &az, double &el);
void horizontal_to_equatorial(double az, double el, double lat, double lst, double& ra, double& dec);

#endif
