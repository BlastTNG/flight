/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__CAMERA__RESULTS_H
#define SHARED__CAMERA__RESULTS_H

#include "../circular_buffer.h"
#include "../result.h"
#include "requests.h"

namespace Shared
{
    namespace Camera
    {

class Results
{
  public:
    Results();
    Results& operator=(const Results& rhs);
    bool is_gain_valid(Requests& requests);

    int counter_stars;
    bool connected;
    ::Shared::Result<double> set_gain;
    ::Shared::Result<double> get_gain;
};

// writes: camera
//  reads: main
extern Shared::CircularBufferPass <Results> results_for_main;

// writes: main
//  reads: network
extern Shared::CircularBuffer <Results> results_for_network;

    }
}

#endif
