/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__SIMULATIONS__BRIGHTNESS_H
#define SHARED__SIMULATIONS__BRIGHTNESS_H

#include "../circular_buffer.h"

struct XSCBrightness;

namespace Shared
{
    namespace Simulations
    {

class Brightness
{
  public:
    Brightness();
    Brightness& operator=(const Brightness& rhs);
    Brightness& operator=(const XSCBrightness& rhs);
    void init(Parameters::Manager& params);

    int counter;
    bool allow_enable;
    bool enabled;
    double level_kepsa;
    double gain_db;
    double actual_exposure;
    double simulated_exposure;
};

// writes: network
//  reads: camera
extern Shared::CircularBuffer <Brightness> brightness;

    }
}

#endif
