/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "refraction.h"
#include <math.h>
#include "angles.h"

double Tools::refraction_angle(double el, double P_mbar, double T)
{
    // Duffet-Smith - Practical Astronomy
    // better than 6 arcsec
    if (el > from_degrees(19.225)) {
        return from_degrees(0.00452 * P_mbar * tan(M_PI/2.0 - el) / T);
    } else {
        el = to_degrees(el);
        double numer = P_mbar * (0.1594 + 0.0196*el + 0.00002*el*el);
        double denom = T * (1 + 0.505*el + 0.0845*el*el);
        return from_degrees(numer / denom);
    }
}

