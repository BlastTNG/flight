/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "estimator.h"
#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include <list>
#include <vector>
#include <limits>
#include <algorithm>
#include "../blob.h"
#include "../../tools/math.h"
#include "../../shared/image/raw.h"
#include "../../shared/solving/settings.h"
#include "../logger.h"

using namespace Solving::Finding;
using Solving::Blob;
using std::min;
using std::max;

#define shared_settings (*(Shared::Solving::settings.r))

struct neighbor
{
    double mean;
    double stdev;
};

void Estimator::get_square_stats(Shared::Image::Raw& image, int square_u, int square_v,
    int halfwidth, double& mean, double& stdev, bool& valid, int& numvals, bool include_center)
{
    int umin, umax, vmin, vmax = 0;
    double flux = 0;

    int padding=1;
    umin = max(square_u-halfwidth, padding);
    umax = min(square_u+halfwidth, image.width-1-padding);
    vmin = max(square_v-halfwidth, padding);
    vmax = min(square_v+halfwidth, image.height-1-padding);

    flux = 0;
    numvals = 0;
    for (int v=vmin; v<=vmax; v++) {
        for (int u=umin; u<=umax; u++) {
            if (include_center || (u != square_u || v != square_v)) {
                flux += image.pixels[v*image.width+u];
                numvals++;
            }
        }
    }
    if (numvals < 2) {
        valid = false;
        return;
    }
    mean = flux / numvals;
    stdev = 0.0;
    for (int v=vmin; v<=vmax; v++) {
        for (int u=umin; u<=umax; u++) {
            if (include_center || (u != square_u || v != square_v)) {
                stdev += pow(image.pixels[v*image.width+u] - mean, 2);
            }
        }
    }
    stdev = sqrt( stdev/(numvals-1) );
    valid = true;
    return;

}

void Estimator::estimate_flux_and_stdev(Shared::Image::Raw& image, int u, int v, int hw,
        double& flux, double& stdev, double& prob_of_something)
{
    double neighbor_mean = 0.0;
    double neighbor_stdev = 0.0;
    double base = 0.0;
    bool valid = false;
    int num_vals = 0;

    unsigned int num_neighbors = 0;
    for (int j=-1; j<=1; j++) {
        for (int i=-1; i<=1; i++) {
            if (i!=0 || j!=0) {
                get_square_stats(image, u+i*(2*hw+1), v+j*(2*hw+1), hw, neighbor_mean, neighbor_stdev, valid, num_vals, true);
                if (valid && num_neighbors<8 && num_vals>0) {
                    neighbor_means[num_neighbors] = neighbor_mean;
                    neighbor_stdevs[num_neighbors] = max(neighbor_stdev, 0.5/double(num_vals));
                    num_neighbors++;
                }
            }
        }
    }

    if (num_neighbors >= 2) {
        unsigned int i0 = -1;
        unsigned int i1 = -1;
        double mean0 = -std::numeric_limits<double>::infinity();
        double mean1 = -std::numeric_limits<double>::infinity();
        for (unsigned int i=0; i<num_neighbors; i++) {
            if (neighbor_means[i] > mean1) {
                i1 = i;
                mean1 = neighbor_means[i];
            }
            if (mean1 > mean0) {
                std::swap(mean0, mean1);
                std::swap(i0, i1);
            }
        }
        base = (neighbor_means[i0] + neighbor_means[i1])/2.0;
        stdev = (neighbor_stdevs[i0] + neighbor_stdevs[i1])/2.0;
    }
    else if (num_neighbors == 1) {
        base = neighbor_means[0];
        stdev = neighbor_stdevs[0];
    }
    else {
        prob_of_something = 0.0;
        return;
    }

    double center_mean = 0.0;
    double center_stdev = 0.5;
    get_square_stats(image, u, v, hw, center_mean, center_stdev, valid, num_vals, false);
    if (!valid) {
        prob_of_something = 0.0;
        return;
    }
    flux = max(0.0, (center_mean - flux)*num_vals);

    double effective_sigma = stdev / sqrt(double(num_vals));
    prob_of_something = max(0.0, (center_mean-base)/effective_sigma);
}

void Estimator::calculate_basic_flux_and_base(Shared::Image::Raw& image, Blob& blob)
{

    int hw;
    int umin, umax, vmin, vmax;
    double blob_width_squared, distance_squared;

    double approximate_size = max(blob.approximate_size, 4.0);
    blob_width_squared = pow(approximate_size*1.5, 2.);
    hw = int(round(approximate_size*3.0));
	
    umin = max(blob.u-hw, 0);
    umax = min(blob.u+hw, image.width);
    vmin = max(blob.v-hw, 0);
    vmax = min(blob.v+hw, image.height);

    double base = 0.0;
    double flux = 0.0;
    int base_counter = 0;
    int flux_counter = 0;
    bool saturated = false;
    double peak = 0.0;
    for (int u=umin; u<=umax; u++) {
        for (int v=vmin; v<=vmax; v++) {
            if (image.pixels[v*image.width+u] >= (image.depth-1)) {
                saturated = true;
            }
            if (image.pixels[v*image.width+u] > peak) {
                peak = image.pixels[v*image.width+u];
            }
            distance_squared = pow(double(u-blob.u), 2) + pow(double(v-blob.v), 2);
            if (distance_squared < blob_width_squared) {
                flux += image.pixels[v*image.width+u];
                flux_counter++;
            }
            else {
                base += image.pixels[v*image.width+u];
                base_counter++;
            }
        }
    }
    blob.saturated = saturated;
    if (base_counter > 0) {
        blob.base = base/base_counter;
    }
    else {
        logger.log("finder: estimator: warning: base not estimated");
        blob.base = 0.0;
    }
    blob.peak = peak - blob.base;
    blob.flux = flux - (blob.base*flux_counter);
    blob.peak_to_flux = 0.0;
    if (blob.flux > 0.0) {
        blob.peak_to_flux = blob.peak / blob.flux;
    }
    blob.snr = blob.flux / blob.noise;
}

Blob Estimator::estimate_blob(Shared::Image::Raw& image, Blob& blob, bool& valid) {
    std::list<Blob> blobs;
    double flux, sigma, confidence_metric;
    bool saturated = false;

    int minhw = 2;
    int maxhw = 4;
    if (shared_settings.robust_mode_enabled) {
        maxhw = 15;
    }

    for (int hw=minhw; hw<maxhw; hw+=1) {
        estimate_flux_and_stdev(image, blob.u, blob.v, hw, flux, sigma, confidence_metric);

        Blob new_blob;
        new_blob.u = blob.u;
        new_blob.v = blob.v;
        new_blob.correlated_peak = blob.correlated_peak;
        new_blob.flux = flux;
        new_blob.approximate_size = hw;
        new_blob.base = 0.0;
        new_blob.noise = sigma;
        new_blob.flux_confidence = confidence_metric;
        new_blob.saturated = saturated;
        blobs.push_back(new_blob);
    }

    blobs.sort(Blob::sort_by_flux_confidence);
    Blob return_blob;
    return_blob = *blobs.begin();

    if (return_blob.flux_confidence > 4.0) {
        valid = true;
    }
    else {
        valid = false;
    }
    calculate_basic_flux_and_base(image, return_blob);
    if (return_blob.snr >= shared_settings.snr_threshold) {
        valid = true;
    }
    else {
        valid = false;
    }
    return return_blob;
}

