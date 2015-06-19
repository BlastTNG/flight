/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "matching.h"

using namespace Shared::Image;

Matching::Matching()
{
    counter_stars = -1;
    triplet_counter = -1;
    base_sets.clear();
}

Matching& Matching::operator=(const Matching &rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        triplet_counter = rhs.triplet_counter;
        base_sets = rhs.base_sets;
    }
    return *this;
}

