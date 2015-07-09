/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__SOLUTION_H
#define SOLVING__SOLUTION_H

#include <vector>
#include "blob.h"
#include "star.h"
#include "attitude.h"

namespace Logging
{
    class Logger;
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

class Solution;

}

class Solving::Solution
{
  public:
    Solution();
    void print(Logging::Logger& logger, bool print_matches=false, bool print_base_matches=false);
    void print_simple_code(Shared::Image::Raw& image);
    static bool sort_by_num_matches(Solution first, Solution second);


    //      --- Pattern Matching Stage 1 ---
    //  load from catalog
    std::vector<double> base_distances;
    std::vector<Star> base_stars;
    std::vector<Blob> base_blobs;

    //      --- Pattern Matching Stage 2 ---
    //  get the base equatorial solution
    SolutionAttitude equatorial;

    //      --- Pattern Matching Stage 3 ---
    //  get the base horizontal solution if there are any horizontal filters enabled
    SolutionAttitude horizontal;

    //      --- Pattern Matching Stage 4 ---
    //  filter on:
    //      1) equatorial solution
    //      2) horizontal solution

    //      --- Pattern Matching Stage 5 ---
    //  fill stars in field of view
    std::vector<Star> stars_in_fov;

    //      --- Pattern Matching Stage 6 ---
    //  perform matches
    std::vector<Star> matched_stars;
    std::vector<Blob> matched_blobs;
    int num_blobs_total;

    //      --- Pattern Matching Stage 5 ---
    //  get the final equatorial solution
    //  (previous definitions of ra, dec, equatorial_roll, platescale, error)

    //      --- Pattern Matching Stage 6 ---
    //  get the final horizontal solution if there are any horizontal filters enabled
    //  (previous definitions of az, el, horizontal_roll)

    //      --- Pattern Matching Stage 7 ---
    // filter on (when applicable):
    //   1) equatorial solution
    //   2) horizontal solution
    //   3) platescale
    //   4) error
    //   5) matched_blobs.size()
    //   5) matched_blobs.size() / num_blobs_total

    //      --- Pattern Matching Stage 8 ---
    // find exposure time (assuming gain 0.04096 ADU/e and aperture 0 for f1.8 200mm))
    double measured_exposure;

};

#endif

