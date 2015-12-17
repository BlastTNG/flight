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

namespace Tools
{
    namespace PSF
    {

void get_kernel_separated_gauss(double *gauss, int halfwidth, double size);
double gaussian(double A, double sigma, double x, double y);

    }
}

#endif
