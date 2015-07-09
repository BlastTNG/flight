/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__TEXTURES_H
#define DISPLAYING__TEXTURES_H

#include "texture.h"
#include "glfont2/glfont2.h"

namespace Displaying
{
    class Textures;
}

namespace Parameters
{
    class Manager;
}

class Displaying::Textures
{
  public:
    enum texture_name {
        basic_drawing=0,
        font_texture,
        drawing_3d,
    };
    static const int num_textures = 3;

    Textures(Parameters::Manager& params);
    ~Textures();
    void init();
    GLuint get(texture_name name);
    glfont::GLFont font;

  private:
    bool display_enabled;
    std::string stars_dir;
    Displaying::Texture textures[num_textures];
};

#endif

