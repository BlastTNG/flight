/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "smoother.h"
#include <math.h>
#include "../update.h"
#include "../../shared/image/raw.h"
#include "../../shared/solving/motion_psf.h"
#include "../../parameters/manager.h"
#include "../../tools/correlate.h"
#include "../../tools/psf.h"

#include "../../tools/quick_cout.h"

using namespace Solving::Finding;
using namespace Tools::PSF;
using std::vector;

#define shared_motion_psf (*(Shared::Solving::motion_psf_network_for_solver.r))

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

void Smoother::smooth_with_motion_psf(Shared::Image::Raw& image, double *pixels_smoothed)
{
    for (int i=0; i<image_width*image_height; i++) {
        pixels_smoothed[i] = 0.0;
    }
    if (shared_motion_psf.summation_mode) {
        vector<motion_psf_pixel> psf_pixels = get_all_motion_psf_pixels(shared_motion_psf, image.num_exposures);
        unsigned int num_psf_pixels = psf_pixels.size();
        if (num_psf_pixels > 0) {
            double scale = 1.0 / ((double) num_psf_pixels);
            cout << "scale is " << scale << endl;
            for (unsigned int i=0; i<num_psf_pixels && !done(); i++) {
                Tools::add_single_pixel_correlation(image.pixels, pixels_smoothed,
                    image_width, image_height, psf_pixels[i].x, psf_pixels[i].y, scale);
            }
        }
    } else {
        for (unsigned int buffer_num = 0; buffer_num < image.num_exposures; buffer_num++)
        {
            vector<motion_psf_pixel> psf_pixels = get_motion_psf_pixels(shared_motion_psf, buffer_num);
            for (unsigned int i=0; i<psf_pixels.size() && !done(); i++) {
                Tools::add_single_pixel_correlation(image.separate_buffers[buffer_num], pixels_smoothed,
                    image_width, image_height, psf_pixels[i].x, psf_pixels[i].y, psf_pixels[i].value);
            }
        }
    }
}

