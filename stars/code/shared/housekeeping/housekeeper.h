/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__HOUSEKEEPING__HOUSEKEEPER_H
#define SHARED__HOUSEKEEPING__HOUSEKEEPER_H

#include <vector>
#include "../circular_buffer.h"
#include "measurement.h"

namespace Shared
{
    namespace Housekeeping
    {

class Housekeeper
{
  public:
    std::vector<Measurement> measurements;
	int heater_state;
};

// writes: main
//  reads: camera
extern Shared::CircularBufferPass <Housekeeper> housekeeper_for_camera;

// writes: camera
//  reads: network
extern Shared::CircularBuffer <Housekeeper> housekeeper_for_network;

    }
}

#endif
