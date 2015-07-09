/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "packets.h"

using namespace Shared::Network;

Packets::Packets()
{
    in_charge = false;
}

Packets& Packets::operator=(const Packets &rhs)
{
    if (this != &rhs) {
        in_charge = rhs.in_charge;
        time_since_sent = rhs.time_since_sent;
        time_since_received = rhs.time_since_received;
    }
    return *this;
}

