/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__HOUSEKEEPING_H
#define DISPLAYING__HOUSEKEEPING_H

#include <vector>
#include "block.h"
#include "dynamic_value.h"
#include "utilities.h"
#include "../tools/timing.h"

namespace Displaying
{
    class Housekeeping;
}

class Displaying::Housekeeping: public Block
{
  public:
    Housekeeping();
    void update();
    void draw(Position &position);

  private:
    std::vector<DynamicValue> measurements;
    Tools::Timer disk_timer;
};

#endif

