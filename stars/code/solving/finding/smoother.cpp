/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "smoother.h"
#include <math.h>
#include "../update.h"
#include "../../shared/image/raw.h"
#include "../../parameters/manager.h"
#include "../../tools/correlate.h"
#include "../../tools/psf.h"

#include "../../tools/quick_cout.h"

using namespace Solving::Finding;
using namespace Tools::PSF;
using std::vector;

Smoother::Smoother(Parameters::Manager& params)
{
	image_width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	image_height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);
    scratch = new double[image_width*image_height];
}

Smoother::~Smoother()
{
    delete [] scratch;
}

void Smoother::make_smooth(Shared::Image::Raw& image, double *pixels_smoothed, int halfwidth,
    double sigma)
{
    int length = 2*halfwidth+1;
    double* gauss = new double[length];
    Tools::PSF::get_kernel_separated_gauss(gauss, halfwidth, sigma);
    Tools::correlate2d(image.pixels, gauss, scratch, halfwidth, 0, image_width, image_height);
    Tools::correlate2d(scratch, gauss, pixels_smoothed, 0, halfwidth, image_width, image_height);
    delete [] gauss;
}

