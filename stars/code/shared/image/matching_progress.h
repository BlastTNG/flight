/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__MATCHING_PROGRESS_H
#define SHARED__IMAGE__MATCHING_PROGRESS_H

#include "../circular_buffer.h"

namespace Shared
{
namespace Image
{

class MatchingProgress
{
  public:
    MatchingProgress();
    MatchingProgress& operator=(const MatchingProgress& rhs);
    void init(Parameters::Manager& params);

    int counter_stars;
    int triplet_counter;
    double progress;
};

// writes: solver
//  reads: main
extern Shared::CircularBuffer <MatchingProgress> matching_progress;

}
}

#endif
