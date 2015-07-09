/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "shape_fitter.h"
#include <levmar.h>
#include <cmath>
#include <cstdlib>
#include "../../shared/image/raw.h"

#include "../../tools/quick_cout.h"

using namespace Solving;
using namespace Solving::Refitting;


/*
#include "../../shared/solving/settings.h"

using std::min;
using std::max;

#define shared_settings (*(Shared::Solving::settings.r))
*/


namespace levmar_c_functions_shape
{

int max_num_blobs = 1;

struct gaussian_additional_data
{
    int* us;
    int* vs;
    double* pedestals;
    unsigned int* which_blobs;
};

// params[0] = shared sigma
// params[1] = blob0.A_un (un-normalized A)
// params[2] = blob0.mu_u
// params[3] = blob0.mu_v
// params[4] = blob1.A
// params[5] = blob1.mu_u
// params[6] = blob1.mu_v
// ...

void gaussian(double* params, double* pixels, int num_params, int num_pixels, void* data)
{
    gaussian_additional_data* additional_data = (gaussian_additional_data*) data;
    int* us = additional_data->us;
    int* vs = additional_data->vs;
    double* pedestals = additional_data->pedestals;
    unsigned int* which_blobs = additional_data->which_blobs;

    double sigma_sq = pow(params[0], 2.0);
    double mu_u = 0.0;
    double mu_v = 0.0;
    double A_un = 1.0;
    unsigned int which_blob = 0;
    for (int i=0; i<num_pixels; i++) {
        which_blob = which_blobs[i];
        A_un = params[which_blob*3+1];
        mu_u = params[which_blob*3+2];
        mu_v = params[which_blob*3+3];
        pixels[i] = A_un * exp(-(pow(us[i]-mu_u, 2.0) + pow(vs[i]-mu_v, 2.0)) / (2.0*sigma_sq));
        pixels[i] += pedestals[i];
    }
}

}

ShapeFitter::ShapeFitter()
{
    max_num_blobs = levmar_c_functions_shape::max_num_blobs;
    int max_halfwidth = 30;
    max_num_pixels = (max_halfwidth*2+1)*(max_halfwidth*2+1)*max_num_blobs;
    pixels = new double[max_num_pixels];
    us = new int[max_num_pixels];
    vs = new int[max_num_pixels];
    pedestals = new double[max_num_pixels];
    which_blobs = new unsigned int[max_num_pixels];

    int max_num_params = 3*max_num_blobs+1;
    workspace = (double *) malloc((LM_DIF_WORKSZ(max_num_params, max_num_pixels)+max_num_params*max_num_params)*sizeof(double));
    covar = workspace + LM_DIF_WORKSZ(max_num_params, max_num_pixels);
}

ShapeFitter::~ShapeFitter()
{
    delete [] pixels;
    delete [] us;
    delete [] vs;
    delete [] pedestals;
    delete [] which_blobs;
    free(workspace);
}

double ShapeFitter::get_covar(int num_params, int param)
{
    if (param > num_params) {
        return 0.0;
    }
    return sqrt(covar[param*num_params+param]);
}

void ShapeFitter::fit_gaussian(Shared::Image::Raw& image, std::vector<Blob>& blobs)
{
    int halfwidth = 10;
    int pixel_i = 0;
    int blob_i = 0;
    unsigned int* original_blob_indices = new unsigned int[max_num_blobs];
    for (unsigned int i = 0; i < blobs.size() && blob_i < max_num_blobs; i++) {
        //if (blob has a good enough snr) {
        if (true) {
            int blob_u = blobs[i].u;
            int blob_v = blobs[i].v;
            for (int u = blob_u-halfwidth; u <= blob_u+halfwidth; u++) {
                for (int v = blob_v-halfwidth; v <= blob_v+halfwidth; v++) {
                    if (pixel_i < max_num_pixels && image.is_inbounds(u, v)) {
                        pixels[pixel_i] = double(image.get_pixel(u, v));
                        us[pixel_i] = u;
                        vs[pixel_i] = v;
                        pedestals[pixel_i] = 0.0;
                        pedestals[pixel_i] += blobs[i].pedestal_params[0]*double(u);
                        pedestals[pixel_i] += blobs[i].pedestal_params[1]*double(v);
                        pedestals[pixel_i] += blobs[i].pedestal_params[2];


                        pedestals[pixel_i] = 0.8;


                        which_blobs[pixel_i] = blob_i;
                        pixel_i++;
                    }
                }
            }
            original_blob_indices[blob_i] = i;
            blob_i++;
        }
    }
    int num_pixels = pixel_i;
    int num_blobs = blob_i;
    levmar_c_functions_shape::gaussian_additional_data additional_data;
    additional_data.us = us;
    additional_data.vs = vs;
    additional_data.pedestals = pedestals;
    additional_data.which_blobs = which_blobs;

    int num_params = 3*num_blobs+1;
    double* params = new double[num_params];
    params[0] = 2.5;
    for (int blob_i=0; blob_i<num_blobs; blob_i++) {
        params[3*blob_i+1] = blobs[original_blob_indices[blob_i]].flux;
        params[3*blob_i+2] = double(blobs[original_blob_indices[blob_i]].u);
        params[3*blob_i+3] = double(blobs[original_blob_indices[blob_i]].v);
    }

    dlevmar_dif(levmar_c_functions_shape::gaussian, params, pixels, num_params, num_pixels,
        1000, NULL, NULL, workspace, covar, (void*) &additional_data);

/*
    for (int i=0; i<num_params*num_params; i++) {
        cout << "covar[" << i << "] " << covar[i] << endl;
    }
*/

    /*
    cout << "shape gives shared_sigma " << params[0];
    cout << " +/- " << get_covar(num_params, 0) << endl;
    for (int blob_i=0; blob_i<num_blobs; blob_i++) {
        cout << "blob fit with A " << params[3*blob_i+1];
        cout << " +/- " << get_covar(num_params, 3*blob_i+1) << " ";
        cout << " and u,v " << params[3*blob_i+2] << " " << params[3*blob_i+3];
        cout << " (errors) ";
        cout << get_covar(num_params, 3*blob_i+2) << " ";
        cout << get_covar(num_params, 3*blob_i+3) << endl;
    }
    */


    delete [] params;
    delete [] original_blob_indices;
}

/*

void Fitter::fit_gaussian(Shared::Image::Raw& image, Blob& blob)
{

    int num_params = 5;
    double* params = new double[num_params];
    params[0] = blob.flux;          // A / sqrt(2*pi*sigma*sigma)
    params[1] = double(blob.u);     // mu_x
    params[2] = double(blob.v);     // mu_y
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

        case Settings::fitting_gaussian:
            fit_gaussian(image, blob);
            break;

        case Settings::fitting_double_gaussian:
            fit_double_gaussian(image, blob);
            break;

        case Settings::fitting_none:
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
*/
