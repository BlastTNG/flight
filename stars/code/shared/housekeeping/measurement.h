/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__HOUSEKEEPING__MEASUREMENT_H
#define SHARED__HOUSEKEEPING__MEASUREMENT_H

#include <string>

namespace Shared
{
    namespace Housekeeping
    {

class Measurement
{
  public:
    std::string name;
    std::string units;
    double value;
    bool valid;
};

    }
}

#endif
