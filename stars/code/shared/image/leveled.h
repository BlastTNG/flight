/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__LEVELED_H
#define SHARED__IMAGE__LEVELED_H

#include "../circular_buffer.h"

namespace Shared
{
    namespace Image
    {

class Leveled
{
  public:
    Leveled();
    void init(Parameters::Manager& params);
    ~Leveled();
    Leveled& operator=(const Leveled& rhs);

    int image_width;
    int image_height;
    char* pixels;
    bool valid;
    int counter_stars;
};

// writes: solver
//  reads: main
extern Shared::CircularBuffer <Leveled> leveled;

    }
}

#endif
