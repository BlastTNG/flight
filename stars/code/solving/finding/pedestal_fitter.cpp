/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "pedestal_fitter.h"
#include <levmar.h>
#include "../blob.h"
#include "../../shared/image/raw.h"
#include "../../tools/timing.h"

#include "../../tools/quick_cout.h"

using namespace Solving;
using namespace Solving::Finding;

namespace levmar_c_functions
{

struct pedestal_additional_data
{
    int* us;
    int* vs;
};

void pedestal_error_function(double* params, double* pixels, int num_params, int num_pixels, void* data)
{
    pedestal_additional_data* additional_data = (pedestal_additional_data*) data;
    int* us = additional_data->us;
    int* vs = additional_data->vs;
    for (int i=0; i<num_pixels; i++) {
        pixels[i] = params[0]*us[i] + params[1]*vs[i] + params[2];
    }
}

}

PedestalFitter::PedestalFitter()
{
    padding = 5;
    halfwidth = 30;
    max_num_pixels = (padding*2+halfwidth*2+1)*(padding*2+halfwidth*2+1);
    pixels = new double[max_num_pixels];
    us = new int[max_num_pixels];
    vs = new int[max_num_pixels];

    int max_num_params = 3;
    workspace = (double *) malloc((LM_DIF_WORKSZ(max_num_params, max_num_pixels)+max_num_params*max_num_params)*sizeof(double));
    covar = workspace + LM_DIF_WORKSZ(max_num_params, max_num_pixels);
}

PedestalFitter::~PedestalFitter()
{
    delete [] pixels;
    delete [] us;
    delete [] vs;
    free(workspace);
}

void PedestalFitter::fit_pedestal(Shared::Image::Raw& image, Blob& blob)
{
    int minu = blob.u - halfwidth - padding;
    int maxu = blob.u + halfwidth + padding;
    int minv = blob.v - halfwidth - padding;
    int maxv = blob.v + halfwidth + padding;
    int i = 0;
    double mean = 0.0;
    for (int u = minu; u <= maxu; u++ ) {
        for (int v = minv; v <= maxv; v++) {
            if (i < max_num_pixels && image.is_inbounds(u, v)) {
                if (u >= blob.u-halfwidth && u <= blob.u+halfwidth &&
                    v >= blob.v-halfwidth && v <= blob.v+halfwidth)
                {
                    // inside
                } else{
                    // outside
                    us[i] = u;
                    us[i] = v;
                    pixels[i] = double(image.get_pixel(u, v));
                    mean += pixels[i];
                    i++;
                }
            }
        }
    }
    int num_pixels = i;
    levmar_c_functions::pedestal_additional_data additional_data;
    additional_data.us = us;
    additional_data.vs = vs;

    int num_params = 3; // [a, b, c] in: z = a*u + b*v + c
    double* params = new double[num_params];
    params[0] = 0.0;
    params[1] = 0.0;
    params[2] = mean/double(num_pixels);

    dlevmar_dif(levmar_c_functions::pedestal_error_function, params, pixels, num_params, num_pixels,
        100, NULL, NULL, workspace, covar, (void*) &additional_data);

    delete [] params;
}

void PedestalFitter::fit_pedestals(Shared::Image::Raw& image, std::vector<Blob>& blobs)
{
    Tools::Timer timer;
    timer.start();
    for (unsigned int i=0; i<blobs.size(); i++) {
        //fit_pedestal(image, blobs[i]);
    }
    //cout << "fitting " << blobs.size() << " pedestals took " << timer.time() << endl;
}

