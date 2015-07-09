/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__COLOR_H
#define DISPLAYING__COLOR_H

namespace Displaying
{
    class Color;
}

class Displaying::Color
{
  public:
    Color();
    Color(double r, double g, double b, double a);
    Color& operator=(const Color &rhs);
    Color& operator*=(const double &scale);
    const Color operator*(const double &scale) const;

    double values[4];
};

#endif
