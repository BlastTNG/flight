/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "attitude.h"
#include "../tools/angles.h"

void Solving::clear_solution_attitude(SolutionAttitude& attitude)
{
    attitude.valid = false;
    attitude.ra = 0.0;
    attitude.dec = 0.0;
    attitude.roll = 0.0;
    attitude.sigma_ra = M_PI;
    attitude.sigma_dec = M_PI;
    attitude.sigma_roll = M_PI;
    attitude.sigma_pointing = M_PI;
    attitude.iplatescale = from_arcsec(9.5);
}

