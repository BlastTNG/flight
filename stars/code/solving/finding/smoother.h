/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__FINDING__SMOOTHER_H
#define SOLVING__FINDING__SMOOTHER_H

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Parameters
{
    class Manager;
}

namespace Solving
{
    namespace Finding
    {
        class Smoother;
    }
}

class Solving::Finding::Smoother
{
  public:
    Smoother(Parameters::Manager& params);
    ~Smoother();
    void make_smooth(Shared::Image::Raw& image, double *pixels_smoothed, int halfwidth, double sigma);

  private:
    int image_width;
    int image_height;
    double* scratch;
};

#endif

