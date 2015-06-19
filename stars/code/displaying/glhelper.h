/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__GL_HELPER_H
#define DISPLAYING__GL_HELPER_H

#if defined(_MSC_VER)
    #include <windows.h>
#endif
#include "GL/gl.h"
#include "GL/glu.h"

namespace Displaying
{
    namespace GL
    {

inline void Color4f(double r, double g, double b, double a)
{
    glColor4f((GLfloat) r, (GLfloat) g, (GLfloat) b, (GLfloat) a);
}


    }
}


#endif
