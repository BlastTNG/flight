/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__BLOB_H
#define SOLVING__BLOB_H

#include <string>

namespace Logging
{
    class Logger;
}

namespace Solving
{
    class Blob;
}

class Solving::Blob
{
  public:
    Blob();
    void print(Logging::Logger& logger, std::string prefix);

    static bool sort_by_correlated_peak(Blob first, Blob second)
    {
        return first.correlated_peak > second.correlated_peak;
    }
    static bool sort_by_flux_confidence(Blob first, Blob second)
    {
        return first.flux_confidence > second.flux_confidence;
    }

    // stage 1, found while searching cells for peaks
    int u; // ccd pixel indices
    int v; // ccd pixel indices
    double correlated_peak;

    // stage 2, found from estimation
    double flux;
    double approximate_size;
    double base;
    double noise;
    double flux_confidence;
    bool saturated;
    double snr;
    double peak;
    double peak_to_flux;

    // stage 2.5, found from pedestal fitting (z=au+bv+c, params=[a,b,c])
    double pedestal_params[3];

    // stage 3, found from fit
    double x; // ccd coordinates, relative to center
    double y; // ccd coordinates, relative to center
    double sigma_x;
    double sigma_y;
    double fit_error_x;
    double fit_error_y;
    double est_flux;
    int num_fitting_iterations;
    bool fit_was_good;

    // stage 4, assigned at the end of blob finding
    int id;
    double image_noise;

    // stage 5, after matches
    bool matched;

};

#endif

