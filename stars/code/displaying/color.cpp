/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "color.h"

using namespace Displaying;

Color::Color()
{
    values[0] = 1.0;
    values[1] = 1.0;
    values[2] = 1.0;
    values[3] = 1.0;
}

Color::Color(double r, double g, double b, double a)
{
    values[0] = r;
    values[1] = g;
    values[2] = b;
    values[3] = a;
}

Color& Color::operator=(const Color &rhs)
{
    if (this != &rhs) {
        values[0] = rhs.values[0];
        values[1] = rhs.values[1];
        values[2] = rhs.values[2];
        values[3] = rhs.values[3];
    }
    return *this;
}

Color& Color::operator*=(const double &scale)
{
    values[0] *= scale;
    values[1] *= scale;
    values[2] *= scale;
    return *this;
}

const Color Color::operator*(const double &scale) const
{
    return Color(*this) *= scale;
}

