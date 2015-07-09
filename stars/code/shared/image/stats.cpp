/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "stats.h"

using namespace Shared::Image;

Stats::Stats()
{
    counter_stars = -1;
    mean_known = false;
    mean = 0.0;
    noise_known = false;
    noise = 1.0;
    gain_known = false;
    gain = 24.4;
    gaindb = 0.0;
    num_pixels_saturated = 0;
    fraction_pixels_saturated = 0;
    autofocus_metric_valid = false;
    autofocus_metric = 0.0;
}

Stats& Stats::operator=(const Stats &rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        mean = rhs.mean;
        mean_known = rhs.mean_known;
        mean = rhs.mean;
        noise_known = rhs.noise_known;
        noise = rhs.noise;
        gain_known = rhs.gain_known;
        gain = rhs.gain;
        gaindb = rhs.gaindb;
        num_pixels_saturated = rhs.num_pixels_saturated;
        fraction_pixels_saturated = rhs.fraction_pixels_saturated;
        autofocus_metric_valid = rhs.autofocus_metric_valid;
        autofocus_metric = rhs.autofocus_metric;
    }
    return *this;
}

void Stats::clear(int counter_stars_)
{
    counter_stars = counter_stars_;
    mean_known = false;
    noise_known = false;
    gain_known = false;
    autofocus_metric_valid = false;
}

