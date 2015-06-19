/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__MAIN_SETTINGS_H
#define SHARED__MAIN_SETTINGS_H

#include "../circular_buffer.h"

namespace Shared
{
    namespace General
    {

class MainSettings
{
  public:
    MainSettings();
    void init(Parameters::Manager& params);

    double display_period;
    bool display_fullscreen;
    int display_fullscreen_counter;
    bool display_image_only;
    bool display_solving_filters;
    double display_image_brightness;
    int display_zoom_x;
    int display_zoom_y;
    double display_zoom_magnitude;
};

// writes: network
//  reads: main
extern Shared::CircularBufferPass <MainSettings> main_settings_net_for_main;

// writes: main
//  reads: solver
//   note: sharing the main settings with the solver thread comes from a poor design too close to flight
//         that muddles main settings with display settings, as the parameter display_image_brightness is used
//         by the solver's statistician
extern Shared::CircularBuffer <MainSettings> main_settings_main_for_solver;

    }
}

#endif
