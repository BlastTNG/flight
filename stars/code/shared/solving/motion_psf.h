/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__SOLVING__MOTION_PSF_H
#define SHARED__SOLVING__MOTION_PSF_H

#include "../circular_buffer.h"
#include <vector>

namespace Shared
{
    namespace Image
    {
        class Raw;
    }

    namespace Solving
    {

struct MotionPSFTimestep
{
    int exposure_num;
    double gy_az;
    double gy_el;
};

class MotionPSF
{
  public:
    MotionPSF();
    void init(Parameters::Manager& params);
    bool valid(Shared::Image::Raw& image);

    double sample_period;
    bool enabled;
    int counter_fcp;
    int counter_stars;
    bool summation_mode;
    double hroll;
    double el;
    double platescale;
    std::vector<MotionPSFTimestep> timesteps;
};

// writes: network
//  reads: solver
extern Shared::CircularBuffer <MotionPSF> motion_psf_network_for_solver;

    }
}

#endif
