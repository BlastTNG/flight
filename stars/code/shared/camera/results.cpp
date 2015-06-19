/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "results.h"

using namespace Shared::Camera;

Results::Results()
{
    counter_stars = 0;
    connected = false;
}

Results& Results::operator=(const Results &rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        connected = rhs.connected;
        set_gain = rhs.set_gain;
        get_gain = rhs.get_gain;
    }
    return *this;
}

bool Results::is_gain_valid(Requests& requests)
{
    if (!get_gain.found) {
        return false;
    }
    if (set_gain.counter == requests.set_gain.counter &&
        get_gain.counter == requests.get_gain.counter)
    {
        return true;
    }
    return false;
}

