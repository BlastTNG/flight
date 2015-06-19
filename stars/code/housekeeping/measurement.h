/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef HOUSEKEEPING__MEASUREMENT_H
#define HOUSEKEEPING__MEASUREMENT_H

#include <string>

namespace Housekeeping
{
    class Measurement;
}

class Housekeeping::Measurement
{
  public:
    Measurement();
    void add_value(double value);
    bool get_value(double& value);

    int channel;
    std::string name;
    double scale;
    double offset;
    std::string units;

  private:
    static const unsigned int num_values = 5;
    double values[num_values];
    unsigned int next_write_index;
    bool valid;
};

#endif
