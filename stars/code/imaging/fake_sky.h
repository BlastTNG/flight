/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__FAKE_SKY_H
#define IMAGING__FAKE_SKY_H

#include "../tools/stocc/stocc.h"

namespace Imaging
{
    class FakeSky;
}

class Imaging::FakeSky
{
  public:
    FakeSky();
    void match_brightness(unsigned short pixels[], int width, int height, int depth);

  private:
    StochasticLib2 sto;
};

#endif

