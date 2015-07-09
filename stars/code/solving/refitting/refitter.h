/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__REFITTING__REFITTER_H
#define SOLVING__REFITTING__REFITTER_H

#include "shape_fitter.h"

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Solving
{
    class Solution;
    namespace Refitting
    {
        class Refitter;
    }
}

class Solving::Refitting::Refitter
{
  public:
    void fit(Solution& solution, Shared::Image::Raw& image);

  private:
    ShapeFitter shape_fitter;
};

#endif
