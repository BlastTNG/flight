/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "results.h"

using namespace Shared::Autofocus;

Results::Results()
{
    mode = mode_not_running;
    current_focus_requested = 0;
    best_focus_known = false;
    best_focus = 0;
}

Results& Results::operator=(const Results &rhs)
{
    if (this != &rhs) {
        mode = rhs.mode;
        current_focus_requested = rhs.current_focus_requested;
        age_of_last_run = rhs.age_of_last_run;
        best_focus_known = rhs.best_focus_known;
        best_focus = rhs.best_focus;
    }
    return *this;
}

