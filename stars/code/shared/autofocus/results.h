/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__AUTOFOCUS__RESULTS_H
#define SHARED__AUTOFOCUS__RESULTS_H

#include "../circular_buffer.h"
#include "../../tools/timing.h"

namespace Shared
{
    namespace Autofocus
    {

class Results
{
  public:
    Results();
    Results& operator=(const Results& rhs);

    enum mode_t {
        mode_not_running,
        mode_running,
        mode_finished_and_gracing
    };

    mode_t mode;
    int current_focus_requested;
    Tools::Timer age_of_last_run;
    bool best_focus_known;
    int best_focus;
};

// writes: lens
//  reads: solver
extern Shared::CircularBufferPass <Results> results_lens_to_solver;

// writes: solver
//  reads: main
extern Shared::CircularBufferPass <Results> results_solver_to_main;

// writes: main
//  reads: network
extern Shared::CircularBuffer <Results> results_main_to_network;

    }
}

#endif
