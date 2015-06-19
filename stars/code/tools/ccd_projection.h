/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef TOOLS__CCD_PROJECTION_H
#define TOOLS__CCD_PROJECTION_H

namespace Shared
{
    namespace Solving
    {
        struct Refraction;
        class Filters;
    }
}

namespace Tools
{

    void ccd_projection(double ra, double dec, double center_ra, double center_dec,
        double platescale, double cos_roll, double sin_roll, double& x, double& y,
        bool flipped_coordinate_system);

    void ccd_projection(double ra, double dec, double center_ra, double center_dec,
        double platescale, double roll, double& x, double& y, bool flipped_coordinate_system);

    void get_refraction_corrected_equatorial(double ra, double dec,
        Shared::Solving::Refraction& settings, Shared::Solving::Filters& filters,
        double& ra_corrected, double& dec_corrected);

    void get_refraction_corrected_horizontal(double ra, double dec,
        Shared::Solving::Refraction& settings, Shared::Solving::Filters& filters,
        double& az_corrected, double &el_corrected);

}

#endif

