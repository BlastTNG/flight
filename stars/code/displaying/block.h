/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__BLOCK_H
#define DISPLAYING__BLOCK_H

#include <string>
#include "dynamic_value.h"
#include "utilities.h"
#include "textures.h"
#include "color.h"
#include "../tools/timing.h"

namespace Displaying
{
    class Block;
}

class Displaying::Block
{
  public:
    Block();
    virtual ~Block();
    virtual void init(std::string title, Textures& textures_);
    void set_title(std::string title);
    void draw_text(std::string text, Position& pos, bool hold_temporary=false);
    void draw_value(DynamicValue& value, Position pos);
    void begin_draw(Position& position, double flip_scaling=1.0);
    void end_draw();
    void draw_border();
    void draw_title();
    void set_color(Color color, bool just_once=false);
    void set_text_scaling(double scaling, bool just_once=false);
    void set_rjust(double width, bool just_once=false);
    void scale_color(double scaling);
    void get_text_size(std::string text, double& width, double& height);

    void apply_flip_scaling(Position& position, double flip_scaling);
    void change_size(Size new_size, bool slow=false);
    void change_size(double width, double height, bool slow=false);
    void update_size(bool slow=false);
    bool size_update_finished();
    Size size();
    double width();
    double height();
    virtual void draw(Position& position, double flip_scaling=1.0);

    std::string title;
    double padding;
    double text_height;
    Textures* textures;

  private:
    Size current_size;
    Size requested_size;
    Size previous_size;
    Tools::Timer sizing_timer;
    bool sizing_finished;

    Color color_permanent;
    Color color_temporary;
    bool color_use_temporary;

    double text_scaling_permanent;
    double text_scaling_temporary;
    bool text_scaling_use_temporary;

    bool rjust_permanent;
    double rjust_width_permanent;
    bool rjust_temporary;
    double rjust_width_temporary;
    bool rjust_use_temporary;

};

#endif

