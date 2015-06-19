/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef PARAMETERS__MANAGER_H
#define PARAMETERS__MANAGER_H

#include <string>
#include "general.h"
#include "housekeeping.h"

namespace Parameters
{
    class Manager;
}

class Parameters::Manager
{
  public:
    Manager(int width, int height, int depth, std::string stars_absolute_dir);
    void load(int argc, char* argv[]);

    std::string stars_dir;
    General general;
    Housekeeping housekeeping;
};

#endif
