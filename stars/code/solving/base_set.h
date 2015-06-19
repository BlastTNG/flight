/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__BASE_SET_H
#define SOLVING__BASE_SET_H

namespace Solving
{
    class BaseSet;
}

class Solving::BaseSet
{
  public:
    unsigned int ids[3];
    enum base_type {pair, triplet};
    base_type type;
};

#endif
