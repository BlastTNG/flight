/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__FINDING__PEDESTAL_FITTER_H
#define SOLVING__FINDING__PEDESTAL_FITTER_H

#include <vector>

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Solving
{
    class Blob;
    namespace Finding
    {
        class PedestalFitter;
    }
}

class Solving::Finding::PedestalFitter
{
  public:
    PedestalFitter();
    ~PedestalFitter();
    void fit_pedestal(Shared::Image::Raw& image, Blob& blob);
    void fit_pedestals(Shared::Image::Raw& image, std::vector<Blob>& blobs);

  private:
    int halfwidth;
    int padding;
    int max_num_pixels;
    double* pixels;
    int* us;
    int* vs;
    double* workspace;
    double* covar;
};

#endif
