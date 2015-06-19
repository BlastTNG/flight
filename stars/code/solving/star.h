/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__STAR_H
#define SOLVING__STAR_H

#include <string>

namespace Logging
{
    class Logger;
}

namespace Solving
{
    class Star;
}

class Solving::Star
{
  public:
    Star(int id_, double ra_, double dec_, double mag_);
    Star();
    static bool sort_by_mag(Star first, Star second);
    void print(Logging::Logger& logger, std::string prefix);

    int id;
    double ra, dec, mag;
    double fitted_x;
    double fitted_y;
};

#endif

