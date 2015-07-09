/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__FINDING__FITTER_H
#define SOLVING__FINDING__FITTER_H

#include "../blob.h"

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Solving
{
    namespace Finding
    {
        class Fitter;
    }
}

class Solving::Finding::Fitter
{
  public:
    Fitter();
    ~Fitter();
    void fit_gaussian(Shared::Image::Raw& image, Solving::Blob& blob);
    void fit_double_gaussian(Shared::Image::Raw& image, Solving::Blob& blob);
    Blob fit(Shared::Image::Raw& image, Solving::Blob& blob);

  private:
    int max_num_pixels;
    double* pixels;
    int* us;
    int* vs;
    double* workspace;
    double* covar;
};

#endif
