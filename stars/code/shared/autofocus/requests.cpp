/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "requests.h"
#include "../../parameters/manager.h"

using namespace Shared::Autofocus;

Requests::Requests()
{
    run_counter = 0;
    focus_search_min = 2300;
    focus_search_max = 3300;
    focus_search_step = 10;
    abort_counter = 0;
    abort_still_use_solution = false;
    display_mode = xC_autofocus_display_mode_auto;
}

void Requests::init(Parameters::Manager& params)
{
    focus_search_min = params.general.try_get("imaging.autofocus.focus_search_min", 2300);
    focus_search_max = params.general.try_get("imaging.autofocus.focus_search_max", 3300);
    focus_search_step = params.general.try_get("imaging.autofocus.focus_search_step", 10);
}

Requests& Requests::operator=(const Requests &rhs)
{
    if (this != &rhs) {
        run_counter = rhs.run_counter;
        focus_search_min = rhs.focus_search_min;
        focus_search_max = rhs.focus_search_max;
        focus_search_step = rhs.focus_search_step;
        abort_counter = rhs.abort_counter;
        abort_still_use_solution = rhs.abort_still_use_solution;
        display_mode = rhs.display_mode;
    }
    return *this;
}

