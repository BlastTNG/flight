/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "blob.h"
#include <boost/format.hpp>
#include "../logging/logger.h"

using namespace Solving;

Blob::Blob()
{
    u = -1;
    v = -1;
    x = -1;
    y = -1;
    correlated_peak = -1;

    flux = -1;
    approximate_size = -1;
    base = -1;
    noise = 1.0;
    flux_confidence = -1;
    saturated = false;
    snr = 0.0;
    peak = 0.0;

    pedestal_params[0] = 0.0;
    pedestal_params[1] = 0.0;
    pedestal_params[2] = 0.0;

    sigma_x = 1.0;
    sigma_y = 1.0;
    fit_error_x = 1.0;
    fit_error_y = 1.0;
    est_flux = -1;
    num_fitting_iterations = -1;
    fit_was_good = false;

    id = -1;
    image_noise = 1.0;
    matched = false;
}

void Blob::print(Logging::Logger& logger, string prefix)
{
    logger.log(boost::format(prefix + "id %d pos %0.2f %0.2f flux %0.2f saturated %i")
        % id % x % y % flux % saturated);
}

