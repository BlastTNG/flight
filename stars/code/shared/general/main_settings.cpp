/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "main_settings.h"
#include "../../parameters/manager.h"

using namespace Shared::General;

MainSettings::MainSettings()
{
    display_period = 1.0 / 20.0;
    display_fullscreen = true;
    display_fullscreen_counter = 0;
    display_image_only = false;
    display_solving_filters = false;
    display_image_brightness = 1.0;
    display_zoom_x = 320;
    display_zoom_y = 240;
    display_zoom_magnitude = 1.0;
}

void MainSettings::init(Parameters::Manager& params)
{
    display_period = 1.0 / params.general.try_get("main.display_frequency", 20.0);
    display_fullscreen = params.general.try_get("main.display_fullscreen", true);
    display_image_brightness = params.general.try_get("main.display_image_brightness", 1.0);
}

