/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVER__STATISTICIAN_H
#define SOLVER__STATISTICIAN_H

#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include "../shared/image/raw.h"

namespace Parameters
{
    class Manager;
}

namespace Solving
{
    class Statistician;
}

class Solving::Statistician
{
  public:
    Statistician(Parameters::Manager& params);
    void get_stats(Shared::Image::Raw& raw);
    void make_display_data(Shared::Image::Raw& raw);
    void print_stats();

  private:
    void get_cell_stats(Shared::Image::Raw& raw, int icell, int jcell, double& mean, double& stdev,
        int& num_saturated);
    void get_default_display_bounds(Shared::Image::Raw& raw, double& default_lower_bound, double& default_width);

    double saturation_limit;
    int cell_size;
};

#endif
