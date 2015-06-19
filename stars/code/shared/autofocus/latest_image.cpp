/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "latest_image.h"

using namespace Shared::Autofocus;

LatestImage::LatestImage()
{
    counter_stars = -1;
}

LatestImage& LatestImage::operator=(const LatestImage &rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
    }
    return *this;
}

