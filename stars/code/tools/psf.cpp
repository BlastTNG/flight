/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "psf.h"
#include "quick_cout.h"
#include <cstdio>
#include <algorithm>

using std::vector;
using std::min;
using std::max;
using namespace Tools::PSF;

void Tools::PSF::get_kernel_separated_gauss(double *gauss, int halfwidth, double size)
{
    int i;
    double u, sigma, total;
    sigma = size;
    total = 0;
    for(i=0; i<halfwidth*2+1; i++){
        u = i - halfwidth;
        gauss[i] = exp(-(u*u)/(2.0*sigma*sigma)) / (2.0*M_PI*sigma*sigma);
        total += gauss[i];
    }
    for(i=0; i<halfwidth*2+1; i++){
        gauss[i] /= total;
    }
}

double Tools::PSF::gaussian(double A, double sigma, double x, double y)
{
    return A*exp(-0.5*(pow(x/sigma, 2.0)+pow(y/sigma, 2.0)))/(2.0*M_PI*pow(sigma, 2.0));
}
