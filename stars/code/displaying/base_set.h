/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__BASE_SET_H
#define DISPLAYING__BASE_SET_H

#include <math.h>
#include <vector>
#include "../tools/timing.h"
#include "../solving/base_set.h"

namespace Displaying
{
    class BaseSet;
    class Blob;
}

class Displaying::BaseSet
{
  public:
    BaseSet(Solving::BaseSet base_set_);
    void draw_lines(std::vector<Blob>& display_blobs, double progress, bool done, double global_brightness, int num_blobs);
    void draw(std::vector<Blob>& display_blobs, double progress, bool done, double global_brightness);

  private:
    Solving::BaseSet base_set;
    Tools::Timer timer;
    bool aged;
};

#endif
