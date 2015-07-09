/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "measurement.h"

using namespace Housekeeping;

Measurement::Measurement()
{
    channel = 0;
    name = "";
    scale = 1.0;
    offset = 0.0;
    units = "";
    for (unsigned int i=0; i<num_values; i++) {
        values[i] = 0.0;
    }
    next_write_index = 0;
    valid = false;
}

void Measurement::add_value(double value)
{
    values[next_write_index] = value;
    next_write_index++;
    if (next_write_index == num_values) {
        valid = true;
    }
    next_write_index %= num_values;
}

bool Measurement::get_value(double& value)
{
    value = 0.0;
    if (valid) {
        for (unsigned int i=0; i<num_values; i++) {
            value += values[i];
        }
        value /= num_values;
        value = value*scale  + offset;
    }
    return valid;
}

