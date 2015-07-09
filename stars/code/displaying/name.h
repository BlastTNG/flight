/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__NAME_H
#define DISPLAYING__NAME_H

#include "block.h"
#include "../tools/timing.h"

namespace Displaying
{
    class Name;
}

class Displaying::Name: public Block
{
  public:
    Name();
    void draw_packets();
    void draw_network_resetting();
    void draw(Position& position);

  private:
    Tools::Timer timer;
};

#endif

