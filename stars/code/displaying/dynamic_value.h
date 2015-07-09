/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__DYNAMIC_VALUE_H
#define DISPLAYING__DYNAMIC_VALUE_H

#include <string>
#include "color.h"

namespace Displaying
{
    class DynamicValue;
}

class Displaying::DynamicValue
{
  public:
    DynamicValue();
    void init(std::string name_);
    void set_value(double value_, int precision);
    void set_value(double value_, std::string units, int precision);
    void set_value(std::string value_);

    std::string name;
    std::string value;
    bool known;
    Color color;
};

#endif

