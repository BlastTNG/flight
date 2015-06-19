/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "requests.h"

using namespace Shared::Camera;

Requests::Requests()
{
    max_num_triggers = 4;
    multi_triggering_delay = 1.0;
}

Requests& Requests::operator=(const Requests &rhs)
{
    if (this != &rhs) {
        set_gain = rhs.set_gain;
        get_gain = rhs.get_gain;
        max_num_triggers = rhs.max_num_triggers;
        multi_triggering_delay = rhs.multi_triggering_delay;
    }
    return *this;
}

