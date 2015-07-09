/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__SOLUTION_H
#define SHARED__IMAGE__SOLUTION_H

#include "../circular_buffer.h"
#include <vector>
#include <string>
#include "../../solving/attitude.h"

namespace Solving
{
    class Solution;
}

namespace Shared
{
    namespace Network
    {
        class Image;
    }

    namespace Image
    {

struct BlobName
{
    int blob_id;
    std::string name;
    double mag;
};

class SolutionSummary
{
  public:
    SolutionSummary();
    SolutionSummary& operator=(const SolutionSummary& rhs);
    SolutionSummary& operator=(const ::Solving::Solution& rhs);

    int counter_stars;
    ::Solving::SolutionAttitude equatorial;
    ::Solving::SolutionAttitude horizontal;
    double solving_time;
    int num_blobs_total;
    int num_blobs_matched;
    double measured_exposure; // this is the estimated exposure time assuming fully open aperture and gain 0 dB for the EBEX camera
    std::vector<BlobName> blob_names;
};

// writes: solver
//  reads: main
extern Shared::CircularBufferPass <SolutionSummary> solution_summary_for_main;

// writes: main
//  reads: network
extern Shared::CircularBuffer <SolutionSummary> solution_summary_main_for_net;

    }
}


#endif
