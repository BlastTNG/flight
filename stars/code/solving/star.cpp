/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "star.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/format.hpp>
#include "../tools/angles.h"
#include "../logging/logger.h"

using namespace Solving;

Star::Star(int id_, double ra_, double dec_, double mag_): id(id_), ra(ra_), dec(dec_), mag(mag_)
{
    fitted_x = 0.0;
    fitted_y = 0.0;
}

Star::Star()
{
    Star(-1, -1.0, -1.0, -1.0);
}

bool Star::sort_by_mag(Star first, Star second)
{
    return first.mag < second.mag;
}

void Star::print(Logging::Logger& logger, string prefix)
{
    logger.log(format(prefix + "id %d ra (deg) %0.4f dec (deg) %0.4f mag %0.4e")
        % id % to_degrees(ra) % to_degrees(dec) % mag);
}

