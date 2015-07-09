/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__LENS_H
#define DISPLAYING__LENS_H

#include "block.h"
#include "dynamic_value.h"

namespace Displaying
{
    class Lens;
}

class Displaying::Lens: public Displaying::Block
{
  public:
    Lens();
    //void update(State::Lens& state);
    void update();
    void draw(Position& position);

  private:
    DynamicValue focus;
    DynamicValue aperture;
};

#endif

