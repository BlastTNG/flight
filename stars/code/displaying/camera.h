/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__CAMERA_H
#define DISPLAYING__CAMERA_H

#include "block.h"
#include "dynamic_value.h"

namespace Displaying
{
    class Camera;
}

class Displaying::Camera: public Displaying::Block
{
  public:
    Camera();
    void update();
    void draw(Position& position);

  private:
    DynamicValue gain;
};

#endif

