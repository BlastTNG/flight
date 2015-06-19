/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "requests.h"

using namespace Shared::Lens;

Requests::Requests()
{
}

Requests& Requests::operator=(const Requests &rhs)
{
    if (this != &rhs) {
        for (unsigned int i=0; i<num_requests; i++) {
            commands[i] = rhs.commands[i];
        }
    }
    return *this;
}

