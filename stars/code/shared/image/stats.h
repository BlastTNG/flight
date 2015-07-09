/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__STATS_H
#define SHARED__IMAGE__STATS_H

#include "../circular_buffer.h"

namespace Shared
{
    namespace Image
    {

class Stats
{
  public:
    Stats();
    Stats& operator=(const Stats& rhs);
    void init(Parameters::Manager& params);
    void clear(int counter_stars_);

    int counter_stars;
    bool mean_known;
    double mean;
    bool noise_known;
    double noise;
    bool gain_known;
    double gain;
    double gaindb;
    int num_pixels_saturated;
    double fraction_pixels_saturated;
    bool autofocus_metric_valid;
    double autofocus_metric;
};

// writes: solver
//  reads: main
extern Shared::CircularBufferPass <Stats> stats_solver_for_main;

// writes: main
//  reads: net
extern Shared::CircularBuffer <Stats> stats_main_for_net;

    }
}

#endif
