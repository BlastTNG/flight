/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef PARAMETERS__GENERAL_H
#define PARAMETERS__GENERAL_H

#include "group.h"
#include <string>

namespace Parameters
{
    class General;
}

class Parameters::General: public Group
{
  public:
    General(int width, int height, int depth, std::string stars_absolute_dir);
    void add_motion_psf();
    void add_bypass_with_blobs();
    void load(int argc, char* argv[]);

    int image_width, image_height, image_depth;
    std::string stars_dir;
    static const unsigned int max_num_bypass_blobs = 100;
    static const unsigned int max_exposure_time_cs = 128;
};

#endif

