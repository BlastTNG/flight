/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__UTILITIES_H
#define DISPLAYING__UTILITIES_H

#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include "GL/gl.h"

namespace Displaying
{

struct Position
{
    GLdouble x;
    GLdouble y;
};

struct Size
{
    GLdouble w;
    GLdouble h;
};

}

#endif
