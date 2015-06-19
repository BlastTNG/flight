/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__CAMERA__REQUESTS_H
#define SHARED__CAMERA__REQUESTS_H

#include "../circular_buffer.h"
#include "../request.h"

namespace Shared
{
    namespace Camera
    {

class Requests
{
  public:
    Requests();
    Requests& operator=(const Requests& rhs);

    ::Shared::Request<double> set_gain;
    ::Shared::Request<double> get_gain;
    unsigned int max_num_triggers;
    double multi_triggering_delay;
};

// writes: network
//  reads: main
extern Shared::CircularBufferPass <Requests> requests_for_main;

// writes: main
//  reads: camera
extern Shared::CircularBuffer <Requests> requests_for_camera;


    }
}

#endif
