/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__REFITTING__SHAPE_FITTER_H
#define SOLVING__REFITTING__SHAPE_FITTER_H

#include <vector>
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
    namespace Refitting
    {
        class ShapeFitter;
    }
}

class Solving::Refitting::ShapeFitter
{
  public:
    ShapeFitter();
    ~ShapeFitter();
    double get_covar(int num_params, int param);
    void fit_gaussian(Shared::Image::Raw& image, std::vector<Solving::Blob>& blobs);

  private:
    int max_num_blobs;
    int max_num_pixels;
    double* pixels;
    int* us;
    int* vs;
    double* pedestals;
    unsigned int* which_blobs;
    double* workspace;
    double* covar;
};

#endif
