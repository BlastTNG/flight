/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__GENERAL__SHUTDOWN_H
#define SHARED__GENERAL__SHUTDOWN_H

#include "../circular_buffer.h"

namespace Shared
{
    namespace General
    {

class Shutdown
{
  public:
    Shutdown();

    bool shutdown_now;
    bool include_restart;
};

// writes: network
//  reads: main
extern Shared::CircularBuffer <Shutdown> shutdown_for_main;

    }
}

#endif
