/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "fitter.h"
#include <levmar.h>
#include "../../shared/solving/settings.h"
#include "../../shared/image/raw.h"

using namespace Solving;
using namespace Solving::Finding;
using std::min;
using std::max;

#define shared_settings (*(Shared::Solving::settings.r))

namespace levmar_c_functions
{

struct gaussian_additional_data
{
    int* us;
    int* vs;
};

void gaussian(double* params, double* pixels, int num_params, int num_pixels, void* data)
{
    gaussian_additional_data* additional_data = (gaussian_additional_data*) data;
    int* us = additional_data->us;
    int* vs = additional_data->vs;
    double mu_u = params[1];
    double mu_v = params[2];
    double sigma_sq = pow(params[3], 2.0);
    double base = params[3];
    for (int i=0; i<num_pixels; i++) {
        pixels[i] = params[0] * exp(-(pow(us[i]-mu_u, 2.0) + pow(vs[i]-mu_v, 2.0)) / (2.0*sigma_sq)) + base;
    }
}

void double_gaussian(double* params, double* pixels, int num_params, int num_pixels, void* data)
{
    gaussian_additional_data* additional_data = (gaussian_additional_data*) data;
    int* us = additional_data->us;
    int* vs = additional_data->vs;
    double mu1_x = params[1];
    double mu1_y = params[2];
    double sigma1_sq = pow(params[3], 2.0);
    double mu2_x = params[5];
    double mu2_y = params[6];
    double sigma2_sq = pow(params[7], 2.0);
    double base = params[8];
    for (int i=0; i<num_pixels; i++) {
        pixels[i] = params[0] * exp(-(pow(us[i]-mu1_x, 2.0) + pow(vs[i]-mu1_y, 2.0)) / (2.0*sigma1_sq));
        pixels[i] += params[4] * exp(-(pow(us[i]-mu2_x, 2.0) + pow(vs[i]-mu2_y, 2.0)) / (2.0*sigma2_sq));
        pixels[i] += base;
    }
}

}

Fitter::Fitter()
{
    max_num_pixels = 50*50;
    pixels = new double[max_num_pixels];
    us = new int[max_num_pixels];
    vs = new int[max_num_pixels];

    int max_num_params = 9;
    workspace = (double *) malloc((LM_DIF_WORKSZ(max_num_params, max_num_pixels)+max_num_params* max_num_params)*sizeof(double));
    covar = workspace + LM_DIF_WORKSZ(max_num_params, max_num_pixels);
}

Fitter::~Fitter()
{
    delete [] pixels;
    delete [] us;
    delete [] vs;
    free(workspace);
}

void Fitter::fit_gaussian(Shared::Image::Raw& image, Blob& blob)
{
    int halfwidth = 10;
    int i = 0;
    for (int u = blob.u-halfwidth; u <= blob.u+halfwidth; u++) {
        for (int v = blob.v-halfwidth; v <= blob.v+halfwidth; v++) {
            if (i < max_num_pixels && image.is_inbounds(u, v)) {
                pixels[i] = double(image.get_pixel(u, v));
                us[i] = u;
                vs[i] = v;
                i++;
            }
        }
    }
    int num_pixels = i;
    levmar_c_functions::gaussian_additional_data additional_data;
    additional_data.us = us;
    additional_data.vs = vs;

    int num_params = 5;
    double* params = new double[num_params];
    params[0] = blob.flux;          // A / sqrt(2*pi*sigma*sigma)
    params[1] = double(blob.u);     // mu_u
    params[2] = double(blob.v);     // mu_v
    params[3] = 3.0;                // sigma
    params[4] = 0;                  // base

    dlevmar_dif(levmar_c_functions::gaussian, params, pixels, num_params, num_pixels,
        100, NULL, NULL, workspace, covar, (void*) &additional_data);

    blob.x = params[1] - (double(image.width)-1.0)/2.0;
    blob.y = params[2] - (double(image.height)-1.0)/2.0;
    blob.sigma_x = params[3];
    blob.fit_error_x = sqrt(covar[1*num_params+1]);
    blob.fit_error_y = sqrt(covar[2*num_params+2]);

    delete [] params;
}

void Fitter::fit_double_gaussian(Shared::Image::Raw& image, Blob& blob)
{
    int halfwidth = 10;
    int i = 0;
    for (int u = blob.u-halfwidth; u <= blob.u+halfwidth; u++) {
        for (int v = blob.v-halfwidth; v <= blob.v+halfwidth; v++) {
            if (i < max_num_pixels && image.is_inbounds(u, v)) {
                pixels[i] = double(image.get_pixel(u, v));
                us[i] = u;
                vs[i] = v;
                i++;
            }
        }
    }
    int num_pixels = i;
    levmar_c_functions::gaussian_additional_data additional_data;
    additional_data.us = us;
    additional_data.vs = vs;

    int num_params = 9;
    double* params = new double[num_params];
    params[0] = blob.flux;          // A1 / sqrt(2*pi*sigma1*sigma1)
    params[1] = double(blob.u);     // mu1_x
    params[2] = double(blob.v);     // mu1_y
    params[3] = 3.0;                // sigma1
    params[4] = blob.flux/5.0;      // A2 / sqrt(2*pi*sigma2*sigma2)
    params[5] = double(blob.u+5);   // mu2_x
    params[6] = double(blob.v+0);   // mu2_y
    params[7] = 3.0;                // sigma2
    params[8] = 0;                  // base

    dlevmar_dif(levmar_c_functions::double_gaussian, params, pixels, num_params, num_pixels,
        30, NULL, NULL, workspace, covar, (void*) &additional_data);

    if (params[0] >= params[4]) {
        blob.x = params[1] - (double(image.width)-1.0)/2.0;
        blob.y = params[2] - (double(image.height)-1.0)/2.0;
        blob.sigma_x = params[3];
    } else {
        blob.x = params[5] - (double(image.width)-1.0)/2.0;
        blob.y = params[6] - (double(image.height)-1.0)/2.0;
        blob.sigma_x = params[7];
    }

    delete [] params;
}

Blob Fitter::fit(Shared::Image::Raw& image, Blob& blob)
{
    using namespace Shared::Solving;
    double original_x = double(blob.u) - (double(image.width)-1.0)/2.0;
    double original_y = double(blob.v) - (double(image.height)-1.0)/2.0;
    switch (shared_settings.fitting_method) {

        case xC_solver_fitting_method_gaussian:
            fit_gaussian(image, blob);
            break;

        case xC_solver_fitting_method_double_gaussian:
            fit_double_gaussian(image, blob);
            break;

        case xC_solver_fitting_method_none:
        default:
            blob.x = double(blob.u) - (double(image.width)-1.0)/2.0;
            blob.y = double(blob.v) - (double(image.height)-1.0)/2.0;
            break;

    }
    double distance_moved_squared = pow(original_x - blob.x, 2.0) + pow(original_y - blob.y, 2.0);
    if (distance_moved_squared < 20*20 && image.is_xy_inbounds(blob.x, blob.y)) {
        blob.fit_was_good = true;
    } else {
        blob.fit_was_good = false;
    }
    return blob;
}

