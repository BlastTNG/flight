/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__IMAGE_H
#define DISPLAYING__IMAGE_H

#include "block.h"

namespace Displaying
{
    class Image;
}

class Displaying::Image: public Block
{
  public:
    Image();
    void update();
    void draw(Position& position);

  private:
    bool currently_solving;
    DynamicValue age;
    DynamicValue counter_stars;
    DynamicValue counter_fcp;
    DynamicValue lat;
    DynamicValue lst;
    DynamicValue mean;
    DynamicValue noise;
    DynamicValue gain;
    DynamicValue num_pixels_saturated;
    DynamicValue stage;
    DynamicValue ra;
    DynamicValue dec;
    DynamicValue az;
    DynamicValue el;
    DynamicValue hroll;
    DynamicValue matched;
    DynamicValue measured_exposure;
    DynamicValue pointing_error;
    DynamicValue fit_error;
    DynamicValue platescale;
};

#endif
