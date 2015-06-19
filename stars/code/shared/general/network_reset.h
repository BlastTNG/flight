/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__GENERAL__NETWORK_RESET_H
#define SHARED__GENERAL__NETWORK_RESET_H

#include <string>
#include "../circular_buffer.h"

namespace Shared
{
    namespace General
    {

class NetworkReset
{
  public:
    NetworkReset();
    void init(Parameters::Manager& params);

    int reset_now_counter;
    bool reset_on_lull_enabled;
    double reset_on_lull_delay;
    std::string device_name;
};

class NetworkResetStatus
{
  public:
    NetworkResetStatus();
    bool resetting;
};

// writes: net_client
//  reads: net_reset
extern Shared::CircularBuffer <NetworkReset> network_reset_for_net_reset;

// writes: net_reset
//  reads: main
extern Shared::CircularBuffer <NetworkResetStatus> network_reset_status_for_main;

    }
}

#endif
