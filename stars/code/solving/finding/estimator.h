/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__FINDING__ESTIMATOR_H
#define SOLVING__FINDING__ESTIMATOR_H

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
        class Estimator;
    }
}

class Solving::Finding::Estimator
{
  public:
    Blob estimate_blob(Shared::Image::Raw& image, Blob& blob, bool& valid);

  private:
    void get_square_stats(Shared::Image::Raw& image, int x, int y, int halfwidth, double& mean, double& stdev,
        bool& valid, int& numvals, bool include_center);
    void estimate_flux_and_stdev(Shared::Image::Raw& image, int x, int y, int hw,
        double& flux, double& stdev, double& prob_of_something);
    void calculate_basic_flux_and_base(Shared::Image::Raw& image, Blob& blob);

    double neighbor_means[8];
    double neighbor_stdevs[8];
};

#endif
