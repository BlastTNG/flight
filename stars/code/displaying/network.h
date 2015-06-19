/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__NETWORK_H
#define DISPLAYING__NETWORK_H

#include "block.h"
#include "dynamic_value.h"

namespace Displaying
{
    class Network;
}

class Displaying::Network: public Block
{
  public:
    Network();
    void draw_packets();
    void draw(Position& position);

  private:
    DynamicValue counter_fcp;
    DynamicValue counter_stars;
};

#endif

