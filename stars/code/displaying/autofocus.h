/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__AUTOFOCUS_H
#define DISPLAYING__AUTOFOCUS_H

#include "block.h"
#include "../shared/autofocus/datapoints.h"

namespace Displaying
{
    class Autofocus;
}

class Displaying::Autofocus: public Displaying::Block
{
  public:
    Autofocus();
    void update();
    bool display_enabled();
    void get_plot_ranges(double& value_min, double& value_max, double& focus_min, double& focus_max,
        Shared::Autofocus::datapoints_t metric_type, bool any_metric_except_sobel=false);
    void set_curve_color_and_width(Shared::Autofocus::datapoints_t metric_type);
    void plot_curves(double value_min, double value_max, double focus_min, double focus_max,
        double plot_offset, Shared::Autofocus::datapoints_t metric_type, double plot_width,
        double plot_height);
    void draw(Position& position);
};

#endif

