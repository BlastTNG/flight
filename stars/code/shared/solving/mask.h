/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__SOLVING__MASK_H
#define SHARED__SOLVING__MASK_H

#include "../circular_buffer.h"
#include <boost/cstdint.hpp>

namespace Shared
{
    namespace Solving
    {

class Mask
{
  public:
    Mask();
    void init(Parameters::Manager& params);
    Mask& operator=(const Mask& rhs);
    bool cell_masked(unsigned int corner_x, unsigned int corner_y);
    bool block_masked(unsigned int i, unsigned int j);

    bool enabled;
    int counter;
    static const int num_fields = 3;
    uint32_t fields[num_fields];

    int block_size;
    int num_blocks_x;
    int num_blocks_y;
    int bits_per_word;
};

// writes: network
//  reads: solver
extern Shared::CircularBufferPass <Mask> mask_network_for_solver;

// writes: solver
//  reads: main
extern Shared::CircularBuffer <Mask> mask_solver_for_main;

    }
}

#endif
