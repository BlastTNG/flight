/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef TOOLS__REFRACTION_H
#define TOOLS__REFRACTION_H

namespace Tools
{
    double refraction_angle(double el, double P_mbar, double T);
}

#endif
