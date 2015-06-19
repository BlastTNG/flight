/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "dynamic_value.h"
#include <boost/format.hpp>
#include <iomanip>

using namespace Displaying;
using std::string;

DynamicValue::DynamicValue(): color()
{
    value = "";
    known = false;
}

void DynamicValue::init(string name_)
{
    name = name_;
}

void DynamicValue::set_value(double value_, int precision)
{
    try {
        value = (boost::format("%f")%boost::io::group(std::setprecision(precision), value_) ).str();
        known = true;
    } catch (int) { }
}

void DynamicValue::set_value(double value_, string units, int precision)
{
    try {
        value = (boost::format("%f")%boost::io::group(std::setprecision(precision), value_) ).str();
        value += units;
        known = true;
    } catch (int) { }
    //value = str( boost::format("%|1.1f|") % value_ ) + " " + units;
}

void DynamicValue::set_value(string value_)
{
    value = value_;
    known = true;
}

