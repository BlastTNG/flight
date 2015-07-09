/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef PARAMETERS__HOUSEKEEPING_H
#define PARAMETERS__HOUSEKEEPING_H

#include "group.h"

namespace Parameters
{
    class Housekeeping;
}

class Parameters::Housekeeping: public Group
{
  public:
    Housekeeping();
    void load(std::string filename);
};

#endif


