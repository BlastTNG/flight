/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__MANAGER_H
#define DISPLAYING__MANAGER_H

#if defined(_MSC_VER)
    #include <windows.h>
#endif
#ifdef _MSC_VER
	#undef uint8_t
    #include "SDL.h"
    #undef main
#else
    #include "SDL.h"		//was SDL/SDL.h
#endif
#include "GL/gl.h"
#include "GL/glu.h"
#include "utilities.h"
#include "housekeeping.h"
#include "textures.h"
#include "texture.h"
#include "visual.h"
#include "autofocus.h"
#include "solving_filters.h"
#include "name.h"
#include "camera.h"
#include "image.h"
#include "lens.h"
#include "quit_box.h"

namespace Parameters
{
    class Manager;
}

namespace Displaying
{
    class DisplayManager;
}

class Displaying::DisplayManager
{
  public:
    DisplayManager(Parameters::Manager& params);
    ~DisplayManager();
    void set_video_mode();
    void reshape();
    void init();
    void process_events();
    void print_available_chars();
    void update_zoom(double delta_t);
    void draw();
    void wait_for_quit();

  private:
    bool using_camera;
    bool enabled;
    bool fullscreen;
    int shared_fullscreen_counter;
    SDL_Surface* screen;
    Size full_screen_size, normal_screen_size, current_screen_size;
    Size resolution;

    Textures textures;

    Housekeeping housekeeping;
    Visual visual;
    Autofocus autofocus;
    SolvingFilters solving_filters;
    Camera camera_block;
    Image image;
    Lens lens;
    QuitBox quit_box;
    Name name;

    double current_zoom_x;
    double current_zoom_y;
    double current_zoom_magnitude;

};

#endif

