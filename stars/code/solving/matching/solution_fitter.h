/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__MATCHING__SOLUTION_FITTER_H
#define SOLVING__MATCHING__SOLUTION_FITTER_H

#include <vector>
#include "../solution.h"
#include "../star.h"
#include "../blob.h"
#include "../../shared/image/raw.h"

namespace levmar_solution_fitter
{

struct additional_data
{
    double* star_data;
    double* weights;
    bool using_weights;
    bool flipped_coordinate_system;
    bool platescale_fixed;
};

}

namespace Solving
{
    namespace PatternMatching
    {
        class SolutionFitter;
    }
}

class Solving::PatternMatching::SolutionFitter
{
  public:
    SolutionFitter();
    ~SolutionFitter();
    double get_error(int num_params, double dec);
    double get_fit_error(double* blob_measurements, double* blob_fit_positions, int num_measurements, int num_params);
    void init_params(double blobs[], double stars[], bool platescale_fixed, bool horizontal);
    void fit_helper(std::vector<Blob>& blobs, std::vector<Star>& stars,
        Solution& solution, Shared::Image::Raw& image, bool platescale_fixed, bool do_horizontal);
    void fill_star_ccd_positions(std::vector<Star>& stars, Solution& solution,
        Shared::Image::Raw& image);
    void fit(std::vector<Blob>& blobs, std::vector<Star>& stars, Solution& solution,
        Shared::Image::Raw& image, bool final_pass=false);

  private:
    unsigned int max_num_blobs;
    double* workspace;
    double* covar;
    levmar_solution_fitter::additional_data adata;
    double* blob_measurements;
    double* blob_fit_positions;
    double* params;
    double* info;
};

#endif
