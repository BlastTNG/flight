/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "matching_progress.h"

using namespace Shared::Image;

MatchingProgress::MatchingProgress()
{
    counter_stars = -1;
    triplet_counter = -1;
    progress = 0.0;
}

MatchingProgress& MatchingProgress::operator=(const MatchingProgress &rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        triplet_counter = rhs.triplet_counter;
        progress = rhs.progress;
    }
    return *this;
}

