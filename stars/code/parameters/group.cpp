/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "group.h"

using namespace Parameters;
using Main::logger;

double Group::try_get(std::string name, double default_value)
{
    double value = default_value;
    try {
        if (map.count(name)) {
            value = double(map[name].as<float>());
        } else {
            logger.log(format("failed to get parameter %s: not found in map") % name);
        }
    } catch(std::exception& e) {
        logger.log(format("failed to get parameter %s: %s") % name % e.what());
    }
    return value;
}
