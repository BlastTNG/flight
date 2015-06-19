/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__BLOB_H
#define DISPLAYING__BLOB_H

#include <string>
#include "utilities.h"
#include "../tools/timing.h"

namespace Displaying
{
    class Blob;
}

class Displaying::Blob
{
  public:
    Blob(double x_, double y_, int blob_num_);
    void draw(Size& size_, double global_age, double intercept, bool matched, bool named);
    double x, y;
    int blob_num;
    std::string name;
    double born;

  private:
    double halfsize;
    double birth_angle;
    double rotation_speed;
    double birth_time;
    Tools::Timer timer;
};

#endif
