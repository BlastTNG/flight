/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__ATTITUDE_H
#define SOLVING__ATTITUDE_H

namespace Solving
{

struct SolutionAttitude
{
    bool valid;
    union {
        double ra;
        double az;
    };
    union {
        double dec;
        double el;
    };
    double roll;
    union {
        double sigma_ra;
        double sigma_az;
    };
    union {
        double sigma_dec;
        double sigma_el;
    };
    double sigma_roll;
    double sigma_pointing;
    double fit_error;
    double iplatescale;
};

void clear_solution_attitude(SolutionAttitude& attitude);

}

#endif
