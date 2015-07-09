/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__NETWORK__PACKETS_H
#define SHARED__NETWORK__PACKETS_H

#include "../circular_buffer.h"
#include "../../tools/timing.h"

namespace Shared
{
namespace Network
{

class Packets
{
  public:
    Packets();
    Packets& operator=(const Packets& rhs);

    bool in_charge;
    Tools::Timer time_since_sent;
    Tools::Timer time_since_received;
};

// writes: network
//  reads: main
extern Shared::CircularBuffer <Packets> packets_from_fc1;

// writes: network
//  reads: main
extern Shared::CircularBuffer <Packets> packets_from_fc2;

}
}

#endif
