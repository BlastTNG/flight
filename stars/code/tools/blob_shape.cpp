/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "math.h"

double blob_shape(double r, double A, double rfocus_unscaled, double sigma, double inner_flat)
{
    double value = 0.0;

    if (rfocus_unscaled < 0) {
        double rfocus = rfocus_unscaled;
        double normalization = 1.0; // needs to be calculated
        if (r < rfocus) {
            value = exp(-pow(r-rfocus, 2.0)/pow(sigma, 2.0));
            value = value*(1.0 - inner_flat) + inner_flat;
            value = A*normalization*value;
        } else {
            value = A*normalization*(exp(-pow(r-rfocus, 2.0)/pow(sigma, 2.0)));
        }
    } else {
        double normalization = 1.0; // needs to be calculated
        value = A*normalization*(exp(-pow(r, 2.0)/pow(sigma, 2.0)));
    }
    return value;
}

