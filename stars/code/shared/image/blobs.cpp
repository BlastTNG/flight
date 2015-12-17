/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "blobs.h"

using namespace Shared::Image;

Blobs::Blobs()
{
    counter_stars = -1;
    blobs.clear();
}

Blobs& Blobs::operator=(const Blobs&rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        blobs = rhs.blobs;
    }
    return *this;
}

