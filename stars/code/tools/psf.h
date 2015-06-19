/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#ifndef TOOLS__PSF_H
#define TOOLS__PSF_H

#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

namespace Shared
{
    namespace Solving
    {
        class MotionPSF;
    }
}

namespace Tools
{
    namespace PSF
    {

struct motion_psf_pixel
{
    int x;
    int y;
    double value;
};

void get_kernel_separated_gauss(double *gauss, int halfwidth, double size);
double gaussian(double A, double sigma, double x, double y);
std::vector<motion_psf_pixel> get_motion_psf_pixels(Shared::Solving::MotionPSF& motion_psf, int exposure_num);
std::vector<motion_psf_pixel> get_all_motion_psf_pixels(Shared::Solving::MotionPSF& motion_psf, int num_exposures);


    }
}

#endif
