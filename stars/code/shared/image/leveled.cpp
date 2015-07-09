/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "leveled.h"
#include <cstring>
#include "../../parameters/manager.h"

using namespace Shared::Image;

Leveled::Leveled()
{
    valid = false;
    counter_stars = -1;
    image_width = 0;
    image_height = 0;
}

void Leveled::init(Parameters::Manager& params)
{
    image_width = params.general.image_width;
    image_height = params.general.image_height;
    pixels = new char[4*image_width*image_height];
}

Leveled::~Leveled()
{
    delete [] pixels;
}

Leveled& Leveled::operator=(const Leveled &rhs)
{
    if (this != &rhs) {
        memcpy(pixels, rhs.pixels, 4*image_width*image_height*sizeof(char));
        valid = rhs.valid;
        counter_stars = rhs.counter_stars;
    }
    return *this;
}

