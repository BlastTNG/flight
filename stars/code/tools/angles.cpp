/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "angles.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "slalib.h"

double from_hours(double angle)
{
    return angle*M_PI/12.;
}

double to_hours(double angle)
{
    return angle*12./M_PI;
}

double from_degrees(double angle)
{
    return angle*M_PI/180.;
}

double to_degrees(double angle)
{
    return angle*180./M_PI;
}

double from_arcmin(double angle)
{
    return (angle/60.)*M_PI/180.;
}

double to_arcmin(double angle)
{
    return angle*(180./M_PI)*60.;
}

double from_arcsec(double angle)
{
    return (angle/3600.)*M_PI/180.;
}

double to_arcsec(double angle)
{
    return angle*(180./M_PI)*3600.;
}

double great_circle(double ra0, double dec0, double ra1, double dec1)
{
    double delta_ra = ra1 - ra0;
    double y = sqrt(pow(cos(dec1)*sin(delta_ra), 2.0) + pow(cos(dec0)*sin(dec1) - sin(dec0)*cos(dec1)*cos(delta_ra), 2.0));
    double x = sin(dec0)*sin(dec1) + cos(dec0)*cos(dec1)*cos(delta_ra);
    return atan2(y, x);
}

bool angles_within_basic(double ra0, double dec0, double ra1, double dec1, double distance)
{
    double true_distance = great_circle(ra0, dec0, ra1, dec1);
    if (true_distance >= distance) {
        return false;
    }
    return true;
}

bool angles_within(double ra0, double dec0, double ra1, double dec1, double distance)
{
    if (fabs(dec0 - dec1) >= distance) {
        return false;
    }
    return angles_within_basic(ra0, dec0, ra1, dec1, distance);
}

double tanplane_distance(double x0, double y0, double x1, double y1, double iplatescale)
{
    double ra0 = 0.0;
    double dec0 = 0.0;
    double ra1 = 0.0;
    double dec1 = 0.0;
    double distance = 0.0;
    Slalib::slaDtp2s(x0*iplatescale, y0*iplatescale, 0.0, 0.0, &ra0, &dec0);
    Slalib::slaDtp2s(x1*iplatescale, y1*iplatescale, 0.0, 0.0, &ra1, &dec1);
    distance = great_circle(ra0, dec0, ra1, dec1);
    return distance;
}

void equatorial_to_horizontal(double ra, double dec, double lat, double lst, double &az, double &el)
{
    double ha = lst - ra;
    Slalib::slaDe2h(ha, dec, lat, &az, &el);
}

void horizontal_to_equatorial(double az, double el, double lat, double lst, double& ra, double& dec)
{
    double ha = 0.0;
    Slalib::slaDh2e(az, el, lat, &ha, &dec);
    ra = lst - ha;
}

