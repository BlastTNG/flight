/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__SOLVING_FILTERS_H
#define DISPLAYING__SOLVING_FILTERS_H

#include "block.h"

namespace Displaying
{
    class SolvingFilters;
}

class Displaying::SolvingFilters: public Block
{
  public:
    SolvingFilters();
    void update();
    void draw(Position& position);

  private:
    DynamicValue horizontal_location_enabled;
    DynamicValue horizontal_location_radius;
    DynamicValue horizontal_location_az;
    DynamicValue horizontal_location_el;

    DynamicValue horizontal_roll_limit_enabled;
    DynamicValue horizontal_roll_limit_min;
    DynamicValue horizontal_roll_limit_max;

    DynamicValue horizontal_el_limit_enabled;
    DynamicValue horizontal_el_limit_min;
    DynamicValue horizontal_el_limit_max;

    DynamicValue equatorial_location_enabled;
    DynamicValue equatorial_location_radius;
    DynamicValue equatorial_location_ra;
    DynamicValue equatorial_location_dec;

};

#endif
