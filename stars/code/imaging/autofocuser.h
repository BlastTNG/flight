/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__AUTOFOCUSER_H
#define IMAGING__AUTOFOCUSER_H

#include "../tools/timing.h"

namespace Imaging
{
    class Autofocuser;
}

class Imaging::Autofocuser
{
  public:
    Autofocuser();
    void make_focus_request();
    void end_run(bool use_solution);
    void update();

    int run_counter;
    int abort_counter;
    int last_counter_stars;
    bool waiting_for_image_since_focus_change;
    Tools::Timer time_since_last_finish;
    Tools::Timer time_since_image_captured_at_last_focus;
    Tools::Timer autofocus_run_time;
};

#endif
