/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__AUTOFOCUS__LATEST_IMAGE_H
#define SHARED__AUTOFOCUS__LATEST_IMAGE_H

#include "../circular_buffer.h"
#include "../../tools/timing.h"

namespace Shared
{
    namespace Autofocus
    {

class LatestImage
{
  public:
    LatestImage();
    LatestImage& operator=(const LatestImage& rhs);

    int counter_stars;
};

// writes: camera
//  reads: lens
extern Shared::CircularBuffer <LatestImage> latest_image;

    }
}

#endif
