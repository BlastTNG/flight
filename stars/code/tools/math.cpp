/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "math.h"

#ifdef _MSC_VER
int round(double value)
{
    if (value > 0) {
        return int(floor(value+0.5));
    }
    else if (value < 0) {
        return int(ceil(value-0.5));
    }
    return 0;
}
#endif
