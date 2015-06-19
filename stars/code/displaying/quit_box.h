/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__QUIT_BOX_H
#define DISPLAYING__QUIT_BOX_H

#include "block.h"

namespace Displaying
{
    class QuitBox;
}

class Displaying::QuitBox: public Displaying::Block
{
  public:
    QuitBox();
    void draw(Position& position);
};

#endif

