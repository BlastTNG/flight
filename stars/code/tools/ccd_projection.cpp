/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "ccd_projection.h"
#include "slalib.h"
#include "refraction.h"
#include "../shared/solving/filters.h"
#include "../shared/solving/settings.h"

void Tools::ccd_projection(double ra, double dec, double center_ra, double center_dec,
    double platescale, double cos_roll, double sin_roll, double& x, double& y,
    bool flipped_coordinate_system)
{
    int status = 0;
    double u = 0;
    double v = 0;
    Slalib::slaDs2tp(ra, dec, center_ra, center_dec, &u, &v, &status);
    if (flipped_coordinate_system) {
        u = -u;
    }
    u *= platescale;
    v *= platescale;
    x = cos_roll * u - sin_roll * v;
    y = sin_roll * u + cos_roll * v;
}

void Tools::ccd_projection(double ra, double dec, double center_ra, double center_dec,
    double platescale, double roll, double& x, double& y, bool flipped_coordinate_system)
{
    double cos_roll = cos(roll);
    double sin_roll = sin(roll);
    ccd_projection(ra, dec, center_ra, center_dec, platescale, cos_roll, sin_roll, x, y, flipped_coordinate_system);
}

void Tools::get_refraction_corrected_equatorial(double ra, double dec,
    Shared::Solving::Refraction& settings, Shared::Solving::Filters& filters,
    double& ra_corrected, double& dec_corrected)
{
    if (settings.enabled && filters.horizontal_known()) {
        double az = 0.0;
        double el = 0.0;
        get_refraction_corrected_horizontal(ra, dec, settings, filters, az, el);
        horizontal_to_equatorial(az, el, filters.lat(), filters.lst(), ra_corrected, dec_corrected);
    } else {
        ra_corrected = ra;
        dec_corrected = dec;
    }
}

void Tools::get_refraction_corrected_horizontal(double ra, double dec,
    Shared::Solving::Refraction& settings, Shared::Solving::Filters& filters,
    double& az_corrected, double &el_corrected)
{
    equatorial_to_horizontal(ra, dec, filters.lat(), filters.lst(), az_corrected, el_corrected);
    if (settings.enabled) {
        el_corrected += Tools::refraction_angle(el_corrected, settings.pressure_mbar, settings.temperature);
    }
}


