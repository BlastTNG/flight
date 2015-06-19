/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__VISUAL_H
#define DISPLAYING__VISUAL_H

#include <stdio.h>
#include <string>
#include <math.h>
#include "texture.h"
#include "block.h"
#include "base_set.h"
#include "blob.h"
#include "utilities.h"
#include "../tools/timing.h"

namespace Displaying
{
    class Visual;
}

namespace Parameters
{
    class Manager;
}

class Displaying::Visual: public Block
{
  public:

    enum texture_status_types {
        no_image,
        image_not_rendered,
        image_rendered
    };

    Visual(Parameters::Manager& params);
    ~Visual();
    void init(std::string title, Textures& textures_);
    void clear();

    void draw3d(Position& pos, Size& scaling);
    void update_size(Size max_size);
    void update();
    void set_start_image();

    void draw_flash(double age);
    void draw_loading_screen();
    void draw_blob_stripe(double intercept);
    void draw_blobs(double age, double stripe_progress);
    void draw_base_sets(double stripe_progress);
    void draw_mask();
    void draw(Position &pos, double flip_scaling=1.0);

  private:
    bool display_enabled;
    std::string stars_dir;

    double main_window_height_;
    double some_height_;
    Size image_size;

    std::vector<Blob> display_blobs_;
    Tools::Timer blobs_timer;
    int base_blobs[3];
    double base_progress;
    std::vector<BaseSet> display_base_sets;
    Tools::Timer image_age_timer;
    Tools::Timer rendered_image_timer;
    Tools::Timer rendered_solution_timer;

    texture_status_types texture_status;
    int frame_number;

    bool loading_screen_begun;
    Tools::Timer loading_screen_timer;

    int current_id;
    int last_blobs_counter_stars;

    int counter_stars_for_last_blob_names;

    Texture loading_screen;
    Texture flash;
    Texture mask;
    Texture white;
    Texture rendered_image;
    Texture canvas;

};

#endif

