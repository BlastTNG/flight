/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "solution_fitter.h"
#include <levmar.h>
#include <cstdlib>
#include <limits>
#include "../../tools/angles.h"
#include "../../tools/slalib.h"
#include "../../shared/solving/settings.h"
#include "../../tools/ccd_projection.h"
#include "../logger.h"
#include "../../tools/quick_cout.h"

using namespace Solving;
using namespace Solving::PatternMatching;
using std::vector;

#define shared_settings (*(Shared::Solving::settings.r))

SolutionFitter::SolutionFitter(): max_num_blobs(100)
{
    int max_num_params = 4;
    int max_num_measurements = 2 * max_num_blobs;
    workspace = (double *) malloc((LM_DIF_WORKSZ(max_num_params, max_num_measurements) +
        max_num_params*max_num_params) * sizeof(double));
    covar = workspace + LM_DIF_WORKSZ(max_num_params, max_num_measurements);
    adata.star_data = new double[max_num_measurements];
    adata.weights = new double[max_num_measurements];
    blob_measurements = new double[max_num_measurements];
    blob_fit_positions = new double[max_num_measurements];
    params = new double[max_num_params];
    info = new double[LM_INFO_SZ];
}

SolutionFitter::~SolutionFitter()
{
    delete [] info;
    delete [] params;
    delete [] blob_measurements;
    delete [] blob_fit_positions;
    delete [] adata.weights;
    delete [] adata.star_data;
    free(workspace);
}

namespace levmar_solution_fitter
{

void error_function(double* params, double* blob_data, int num_params, int num_measurements,
        void* data)
{
    additional_data* adata = (additional_data*) data;
    double* star_data = adata->star_data;
    double* weights = adata->weights;

    double platescale = 0.0;
    if (adata->platescale_fixed) {
        platescale = 1.0 / shared_settings.iplatescale_fixed;
    } else {
        platescale = params[3];
    }
    unsigned int num_blobs = num_measurements/2;
    double cos_roll = cos(params[2]);
    double sin_roll = sin(params[2]);

    double x = 0;
    double y = 0;
    for (unsigned int i=0; i<num_blobs; i++) {
        Tools::ccd_projection(star_data[2*i], star_data[2*i+1], params[0], params[1],
            platescale, cos_roll, sin_roll, x, y, adata->flipped_coordinate_system);
        blob_data[2*i] = x;
        blob_data[2*i+1] = y;
        blob_data[2*i] *= weights[2*i];
        blob_data[2*i+1] *= weights[2*i+1];
    }

}

}

double SolutionFitter::get_fit_error(double* blob_measurements, double* blob_fit_positions, int num_measurements, int num_params)
{
    double error_squared = 0.0;
    for (unsigned int i=0; i<(unsigned int)num_measurements; i++) {
        error_squared += pow(blob_measurements[i]-blob_fit_positions[i], 2.0);
    }
    if (num_measurements > num_params) {
        return sqrt(error_squared / (num_measurements - num_params));
    }
    return std::numeric_limits<double>::infinity();
}

double SolutionFitter::get_error(int num_params, double dec)
{
    double sigma_ra = sqrt(covar[0*num_params+0]);
    double sigma_ra_scaled = sigma_ra*cos(dec);
    double sigma_dec = sqrt(covar[1*num_params+1]);
    double average_position_error = (sigma_dec + sigma_ra_scaled)/2.0;
    return average_position_error;
}

void SolutionFitter::init_params(double blobs[], double stars[],
        bool platescale_fixed, bool flipped_coordinate_system)
{
    // shaves off three percent, not worth the extra complexity

    double bxdiffs = blobs[2]-blobs[0];
    double bydiffs = blobs[3]-blobs[1];
    double sxdiffs = stars[2]-stars[0];
    double sydiffs = stars[3]-stars[1];

    if (flipped_coordinate_system) {
        sxdiffs = -sxdiffs;
    }

    double theta = atan2(bydiffs*sxdiffs - bxdiffs*sydiffs, bxdiffs*sxdiffs + bydiffs*sydiffs);
    params[2] = theta;

    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double platescale = (cos_theta*bxdiffs + sin_theta*bydiffs) / sxdiffs;
    if (!platescale_fixed) {
        params[3] = platescale;
    }

    params[0] = stars[0] - (cos_theta*blobs[0] + sin_theta*blobs[1]) / platescale;
    params[1] = stars[1] - (cos_theta*blobs[1] - sin_theta*blobs[0]) / platescale;
}

void SolutionFitter::fit_helper(vector<Blob>& blobs, vector<Star>& stars, Solution& solution,
        Shared::Image::Raw& image, bool platescale_fixed, bool do_horizontal)
{
    unsigned int num_blobs = std::min((unsigned int) blobs.size(), max_num_blobs);
    unsigned int num_measurements = 2*num_blobs;
    unsigned int num_params = 3;

    for (unsigned int i=0; i<num_blobs; i++) {

        if (do_horizontal) {
            double az = 0;
            double el = 0;
            Tools::get_refraction_corrected_horizontal(stars[i].ra, stars[i].dec,
                shared_settings.refraction, image.filters, az, el);
            adata.star_data[2*i] = az;
            adata.star_data[2*i+1] = el;
        } else {
            double ra = 0;
            double dec = 0;
            Tools::get_refraction_corrected_equatorial(stars[i].ra, stars[i].dec,
                shared_settings.refraction, image.filters, ra, dec);
            adata.star_data[2*i] = ra;
            adata.star_data[2*i+1] = dec;
        }

        adata.weights[2*i] = 1.0;
        adata.weights[2*i+1] = 1.0;
        blob_measurements[2*i] = blobs[i].x * adata.weights[2*i];
        blob_measurements[2*i+1] = blobs[i].y * adata.weights[2*i+1];
    }
    if (do_horizontal) {
        adata.flipped_coordinate_system = false;
    } else {
        adata.flipped_coordinate_system = true;
    }
    adata.platescale_fixed = platescale_fixed;

    params[0] = adata.star_data[0];
    params[1] = adata.star_data[1];
    params[2] = 0.0;
    if (platescale_fixed) {
        num_params = 3; // ra, dec, roll
    } else {
        num_params = 4; // ra, dec, roll, platescale
        params[3] = 1.0 / from_arcsec(9.5);
    }

    dlevmar_dif(levmar_solution_fitter::error_function, params, blob_measurements, num_params,
        num_measurements, 100, NULL, info, workspace, covar, (void*) &adata);

    levmar_solution_fitter::error_function(params, blob_fit_positions, num_params, num_measurements, (void*) &adata);
    double fit_error = get_fit_error(blob_measurements, blob_fit_positions, num_measurements, num_params);

    // too close to flight
    //  * safety check on fit_error to prevent it from cutting good solutions
    if (fit_error > 1000.0) {
        fit_error = 1000.0;
    }

    if (do_horizontal) {
        solution.horizontal.valid = true;
        solution.horizontal.az = params[0];
        solution.horizontal.el = params[1];
        solution.horizontal.roll = params[2];
        solution.horizontal.sigma_az = sqrt(covar[0*num_params+0]);
        solution.horizontal.sigma_el = sqrt(covar[1*num_params+1]);
        solution.horizontal.sigma_roll = sqrt(covar[2*num_params+2]);
        solution.horizontal.sigma_pointing = get_error(num_params, solution.horizontal.el);
        solution.horizontal.fit_error = fit_error;
        if (platescale_fixed) {
            solution.horizontal.iplatescale = shared_settings.iplatescale_fixed;
        } else {
            solution.horizontal.iplatescale = 1.0 / params[3];
        }
    } else {
        solution.equatorial.valid = true;
        solution.equatorial.ra = params[0];
        solution.equatorial.dec = params[1];
        solution.equatorial.roll = params[2];
        solution.equatorial.sigma_ra = sqrt(covar[0*num_params+0]);
        solution.equatorial.sigma_dec = sqrt(covar[1*num_params+1]);
        solution.equatorial.sigma_roll = sqrt(covar[2*num_params+2]);
        solution.equatorial.sigma_pointing = get_error(num_params, solution.equatorial.dec);
        solution.equatorial.fit_error = fit_error;
        if (platescale_fixed) {
            solution.equatorial.iplatescale = shared_settings.iplatescale_fixed;
        } else {
            solution.equatorial.iplatescale = 1.0 / params[3];
        }
    }
}

void SolutionFitter::fill_star_ccd_positions(vector<Star>& stars, Solution& solution,
    Shared::Image::Raw& image)
{
    double x = 0;
    double y = 0;
    double star_ra = 0;
    double star_dec = 0;
    for (unsigned int i=0; i<stars.size(); i++) {
        Tools::get_refraction_corrected_equatorial(stars[i].ra, stars[i].dec, shared_settings.refraction,
            image.filters, star_ra, star_dec);
        Tools::ccd_projection(star_ra, star_dec, solution.equatorial.ra, solution.equatorial.dec,
            1.0/solution.equatorial.iplatescale, solution.equatorial.roll, x, y, true);
        stars[i].fitted_x = x;
        stars[i].fitted_y = y;
    }
}

void SolutionFitter::fit(vector<Blob>& blobs, vector<Star>& stars, Solution& solution,
    Shared::Image::Raw& image, bool final_pass)
{
    if (stars.size() != blobs.size()) {
        return;
    }
    if (blobs.size() < 2) {
        return;
    }
    bool platescale_fixed = false;
    if (blobs.size() == 2 || shared_settings.platescale_always_fixed) {
        //Shared::Solving::settings.update(); // why is this update here?
        platescale_fixed = true;
    }

    // fit for equatorial solution
    fit_helper(blobs, stars, solution, image, platescale_fixed, false);

    // fit for horizontal solution if capable
    if (image.filters.horizontal_known_and_filters_enabled() ||
        (image.filters.horizontal_known() && final_pass))
    {
        fit_helper(blobs, stars, solution, image, platescale_fixed, true);
    }

    // if final pass, fill star positions and print horizontal information
    if (final_pass) {
        if (image.filters.horizontal_known()) {
            logger.log(format(
                "solution fitter: final pass used lat %.5f (deg) lst %.5f (hours)")
                % to_degrees(image.filters.lat()) % to_hours(image.filters.lst()));
        }
        fill_star_ccd_positions(stars, solution, image);
    }
}

