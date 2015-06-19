/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__AUTOFOCUS__REQUESTS_H
#define SHARED__AUTOFOCUS__REQUESTS_H

#include "../circular_buffer.h"
#include "../request.h"
extern "C" {
#include "../../networking/xsc_protocol/xsc_protocol.h"
}

namespace Shared
{
    namespace Autofocus
    {

class Requests
{
  public:
    Requests();
    void init(Parameters::Manager& params);
    Requests& operator=(const Requests& rhs);

    int run_counter;
    int focus_search_min;
    int focus_search_max;
    int focus_search_step;
    int abort_counter;
    bool abort_still_use_solution;
    xsc_autofocus_display_mode_t display_mode;
};

// writes: network
//  reads: main
extern Shared::CircularBufferPass <Requests> requests_network_to_main;

// writes: main
//  reads: lens
extern Shared::CircularBuffer <Requests> requests_main_to_lens;

    }
}

#endif
