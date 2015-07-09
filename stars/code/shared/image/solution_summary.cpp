/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "solution_summary.h"
#include "../../solving/solution.h"

using namespace Shared::Image;


SolutionSummary::SolutionSummary()
{
    counter_stars = -1;
    Solving::clear_solution_attitude(equatorial);
    Solving::clear_solution_attitude(horizontal);
    solving_time = 0.0;
    num_blobs_total = 0;
    num_blobs_matched = 0;
    measured_exposure = 0.0;
    blob_names.clear();
}

SolutionSummary& SolutionSummary::operator=(const SolutionSummary& rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        equatorial = rhs.equatorial;
        horizontal = rhs.horizontal;
        solving_time = rhs.solving_time;
        num_blobs_total = rhs.num_blobs_total;
        num_blobs_matched = rhs.num_blobs_matched;
        measured_exposure = rhs.measured_exposure;
        blob_names = rhs.blob_names;
    }
    return *this;
}

SolutionSummary& SolutionSummary::operator=(const ::Solving::Solution& rhs)
{
    equatorial = rhs.equatorial;
    horizontal = rhs.horizontal;
    num_blobs_total = rhs.num_blobs_total;
    num_blobs_matched = rhs.matched_blobs.size();
    measured_exposure = rhs.measured_exposure;
    return *this;
}

