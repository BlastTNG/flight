/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__MATCHING__MATCHER_H
#define SOLVING__MATCHING__MATCHER_H

#include <math.h>
#include <vector>
#include <boost/math/distributions.hpp>
#include "../../tools/slalib.h"
#include "fluxer.h"
#include "catalog_manager.h"
#include "solution_fitter.h"
#include "../solution.h"
#include "../base_set.h"

namespace Parameters
{
    class Manager;
}

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Solving
{
    namespace Matching
    {
        class Matcher;
    }
}

class Solving::Matching::Matcher
{
  public:
    Matcher(Parameters::Manager& params);
    void init();
    void print_solutions(std::vector<Solution>& solutions);
    std::vector<Star> match_stars_to_blob(Solution& solution, Blob& blob, std::vector<Star>& stars,
        Shared::Image::Raw& image);
    void match_blobs_to_stars(std::vector<Blob>& blobs, Solution& solution, std::vector<Star>& stars,
        Shared::Image::Raw& image);
    void perform_matches(std::vector<Blob>& blobs, Solution& solution, Shared::Image::Raw& image);
    void calculate_pvalue(Solution& solution);
    bool get_next_base_set(BaseSet& base_set, int num_blobs, bool reset=false);
    void inform_blobs_of_matches(Solution& solution);
    void get_blob_names(Solution& solution);
    void fill_solutions_for_next_base_set(std::vector<Solution>& solutions,
        Shared::Image::Raw& image, BaseSet& base_set, std::vector<Blob>& blobs);
    bool match(std::vector<Blob>& blobs, Shared::Image::Raw& image, Solution& solution);

  private:
    Solving::Matching::CatalogManager catalog_manager;
    PatternMatching::SolutionFitter solution_fitter;
    Fluxer fluxer;
    int image_width, image_height;
    int result_update_success_counter, result_update_failure_counter;
    int base_set_counter;
};

#endif

