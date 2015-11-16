/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "fluxer.h"
#include <levmar.h>
#include <cstdlib>
#include <cmath>
#include "../../tools/timing.h"
#include "../solution.h"
#include "../blob.h"
#include "../star.h"
#include "../logger.h"

#include "../../tools/quick_cout.h"

using namespace Solving::Matching;
using namespace Solving;
using std::vector;

namespace levmar_fluxer
{

void error_function(double* params, double* blob_fluxes, int num_params, int num_blobs,
    void* data)
{
    additional_data* adata = (additional_data*) data;
    double* star_fluxes = adata->star_fluxes;
    double* errors = adata->errors;

    for (unsigned int i = 0; i < (unsigned int) num_blobs; i++) {
        blob_fluxes[i] = star_fluxes[i] * params[0] / errors[i];
    }
}

}

Fluxer::Fluxer(): max_num_blobs(100)
{
    int max_num_params = 1;
    workspace = (double *) malloc((LM_DIF_WORKSZ(max_num_params, max_num_blobs) +
        max_num_params*max_num_params) * sizeof(double));
    covar = workspace + LM_DIF_WORKSZ(max_num_params, max_num_blobs);
    adata.star_fluxes = new double[max_num_blobs];
    adata.errors = new double[max_num_blobs];
    blob_fluxes = new double[max_num_blobs];
    params = new double[max_num_params];
    info = new double[LM_INFO_SZ];
}

Fluxer::~Fluxer()
{
    delete [] info;
    delete [] params;
    delete [] blob_fluxes;
    delete [] adata.errors;
    delete [] adata.star_fluxes;
    free(workspace);
}

void Fluxer::fit(Solution& solution, vector<Blob>& blobs, vector<Star>& stars, bool print)
{
    if (blobs.size() < 1 || (blobs.size() != stars.size())) {
        logger.log(format("fluxer: warning: blobs.size = %i, stars.size = %i")
            % blobs.size() % stars.size());
        return;
    }

    int num_saturated_blobs = 0;
    unsigned int brightest_star_index = 0;
    unsigned int j=0;
    for (unsigned int i=0; i<blobs.size() && j<max_num_blobs; i++) {
        if (blobs[i].saturated) {
            num_saturated_blobs++;
        } else {
            double error = fabs(stars[i].mag)*0.0001;
            if (error <= 0) {
                error = 1000000.0;
            }
            adata.errors[j] = error;
            blob_fluxes[j] = blobs[i].flux / error;
            adata.star_fluxes[j] = stars[i].mag;
            if (adata.star_fluxes[j] > adata.star_fluxes[brightest_star_index]) {
                brightest_star_index = j;
            }
            j++;
        }
    }
    unsigned int num_blobs = j;
    unsigned int num_params = 1;

    logger.log(format("fluxer: ignoring %d saturated blobs") % num_saturated_blobs);
    logger.log(format("fluxer: fitting %d matched blobs") % num_blobs);
    if (num_blobs < 1) {
        logger.log("fluxer: warning: not enough blobs to fit");
        return;
    }

    params[0] = blob_fluxes[brightest_star_index] / adata.star_fluxes[brightest_star_index];
    dlevmar_dif(levmar_fluxer::error_function, params, blob_fluxes, num_params,
        num_blobs, 100, NULL, info, workspace, covar, (void*) &adata);

    logger.log(format("fluxer: found best fit line %.4e ADU/(e/m^2/s)") % params[0]);

    // params[0] is in ADU / (electrons / m^s / s)
    // s = [(ADU/e) * m^2 * s] / (ADU/e) / (m^2)
	double gain = 0.2048; //Previous value: 0.04096  11-17-15: Adjusting to match observed offset
    double aperture = 0.009696;

    solution.measured_exposure = params[0] / gain / aperture;
    logger.log(format("fluxer: found best fit exposure %.0f ms")
        % Time::to_milliseconds(solution.measured_exposure));

}

