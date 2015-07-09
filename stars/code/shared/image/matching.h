/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__MATCHING_H
#define SHARED__IMAGE__MATCHING_H

#include <vector>
#include "../circular_buffer.h"
#include "../../solving/base_set.h"

namespace Shared
{
namespace Image
{

class Matching
{
  public:
    Matching();
    Matching& operator=(const Matching& rhs);
    void init(Parameters::Manager& params);

    int counter_stars;
    int triplet_counter;
    std::vector< ::Solving::BaseSet > base_sets;
};

// writes: solver
//  reads: main
extern Shared::CircularBuffer <Matching> matching;

}
}

#endif
