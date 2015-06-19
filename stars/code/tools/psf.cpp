/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "psf.h"
#include "../shared/solving/motion_psf.h"
#include "quick_cout.h"
#include <cstdio>

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
    total = total;
    for(i=0; i<halfwidth*2+1; i++){
        gauss[i] /= total;
    }
}

double Tools::PSF::gaussian(double A, double sigma, double x, double y)
{
    return A*exp(-0.5*(pow(x/sigma, 2.0)+pow(y/sigma, 2.0)))/(2.0*M_PI*pow(sigma, 2.0));
}

vector<motion_psf_pixel> Tools::PSF::get_motion_psf_pixels(Shared::Solving::MotionPSF& motion_psf,
    int exposure_num)
{
    double single_psf_sigma = 0.75;

    double cos_roll = cos(motion_psf.hroll);
    double sin_roll = sin(motion_psf.hroll);
    double cos_el = cos(motion_psf.el);

    // pick halfwidths
    int halfwidth_x = 0;
    int halfwidth_y = 0;
    {
        double max_distance_x = 0.0;
        double max_distance_y = 0.0;
        double current_x = 0.0;
        double current_y = 0.0;
        for (unsigned int t=0; t<motion_psf.timesteps.size(); t++) {
            double delta_az_px = motion_psf.timesteps[t].gy_az * motion_psf.sample_period * motion_psf.platescale;
            double delta_el_px = motion_psf.timesteps[t].gy_el * motion_psf.sample_period * motion_psf.platescale;
            current_x += cos_roll * (-delta_az_px * cos_el) - sin_roll * (-delta_el_px);
            current_y += sin_roll * (-delta_az_px * cos_el) + cos_roll * (-delta_el_px);
            if (fabs(current_x) > max_distance_x) {
                max_distance_x = fabs(current_x);
            }
            if (fabs(current_y) > max_distance_y) {
                max_distance_y = fabs(current_y);
            }
        }
        halfwidth_x = int(ceil(max_distance_x+3.0*single_psf_sigma));
        halfwidth_y = int(ceil(max_distance_y+3.0*single_psf_sigma));
        halfwidth_x = min(halfwidth_x, 128);
        halfwidth_y = min(halfwidth_y, 128);
    }

    // initialize kernel
    int width = 2*halfwidth_x+1;
    int height = 2*halfwidth_y+1;
    double* kernel = new double[width*height];
    for (int v=0; v<height; v++) {
        for (int u=0; u<width; u++) {
            kernel[v*width+u] = 0.0;
        }
    }

    // fill the kernel
    {
        int num_timesteps_for_this_exposure = 0;
        for (unsigned int t=0; t<motion_psf.timesteps.size(); t++) {
            if (motion_psf.timesteps[t].exposure_num == exposure_num) {
                num_timesteps_for_this_exposure++;
            }
        }
        double A = 1.0 / double(num_timesteps_for_this_exposure);

        double current_x = 0.0;
        double current_y = 0.0;
        for (unsigned int t=0; t<motion_psf.timesteps.size(); t++) {
            double delta_az_px = motion_psf.timesteps[t].gy_az * motion_psf.sample_period * motion_psf.platescale;
            double delta_el_px = motion_psf.timesteps[t].gy_el * motion_psf.sample_period * motion_psf.platescale;
            current_x += cos_roll * (-delta_az_px * cos_el) - sin_roll * (-delta_el_px);
            current_y += sin_roll * (-delta_az_px * cos_el) + cos_roll * (-delta_el_px);
            if (motion_psf.timesteps[t].exposure_num == exposure_num) {
                for (int v=0; v<height; v++) {
                    for (int u=0; u<width; u++) {
                        double x = double(u - halfwidth_x) - current_x;
                        double y = double(v - halfwidth_y) - current_y;
                        kernel[v*width+u] += gaussian(A, single_psf_sigma, x, y);
                    }
                }
            }
        }

        // print the scaled kernel
        if (false) {
            for (int v=0; v<height; v++) {
                cout << v << ": ";
                for (int u=0; u<width; u++) {
                    printf("%3.1f ", kernel[v*width+u]*num_timesteps_for_this_exposure);
                }
                cout << endl;
            }
        }

    }

    // fill the pixel vector from the significant kernel elements
    vector<motion_psf_pixel> pixels;
    for (int v=0; v<height; v++) {
        for (int u=0; u<width; u++) {
            if(kernel[v*width+u] > 0.001) {
                motion_psf_pixel pixel;
                pixel.x = u - halfwidth_x;
                pixel.y = v - halfwidth_y;
                pixel.value = kernel[v*width+u];
                pixels.push_back(pixel);
            }
        }
    }

    // print the pixels
    if (false) {
        for (unsigned int i=0; i<pixels.size(); i++) {
            cout << "pixel " << pixels[i].x << " " << pixels[i].y << " " << pixels[i].value << endl;
        }
    }

    // free kernel
    delete [] kernel;

    return pixels;
}

vector<motion_psf_pixel> Tools::PSF::get_all_motion_psf_pixels(Shared::Solving::MotionPSF& motion_psf,
    int num_exposures)
{
    vector<motion_psf_pixel> all_pixels;
    vector<motion_psf_pixel> temp_pixels;

    cout << "get_all_motion_psf_pixels: num_exposures is " << num_exposures << endl;

    for (int i=0; i<num_exposures; i++) {
        temp_pixels = get_motion_psf_pixels(motion_psf, i);
        all_pixels.insert(all_pixels.end(), temp_pixels.begin(), temp_pixels.end());
    }
    return all_pixels;
}

