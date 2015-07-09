/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__FINDING__LEVELER_H
#define SOLVING__FINDING__LEVELER_H

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Parameters
{
    class Manager;
}

namespace Solving
{
    namespace Finding
    {
        class Leveler;
    }
}

class Solving::Finding::Leveler
{
  public:
    Leveler(Parameters::Manager& params);
    ~Leveler();
    void level(Shared::Image::Raw& image, unsigned short leveled[]);

  private:
    bool is_inbounds(int& i, int& j, int& width, int& height);

    int image_width;
    int image_height;
    unsigned short* coarse4;
};

#endif
