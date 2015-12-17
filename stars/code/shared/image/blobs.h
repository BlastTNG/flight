/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__BLOBS_H
#define SHARED__IMAGE__BLOBS_H

#include <vector>
#include "../circular_buffer.h"
#include "../../solving/blob.h"

namespace Shared
{
    namespace Image
    {

class Blobs
{
  public:
    Blobs();
    Blobs& operator=(const Blobs& rhs);
    void init(Parameters::Manager& params);

    int counter_stars;
    std::vector< ::Solving::Blob > blobs;
};

// writes: solver
//  reads: main
extern Shared::CircularBufferPass <Blobs> blobs_solver_for_main;

// writes: main
//  reads: network
extern Shared::CircularBuffer <Blobs> blobs_main_for_net;

    }
}

#endif
