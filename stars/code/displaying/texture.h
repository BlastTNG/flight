/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef DISPLAYING__TEXTURE_H
#define DISPLAYING__TEXTURE_H

#include <string>
#include "utilities.h"
#include "GL/glu.h"

namespace Displaying
{
    class Texture;
}

class Displaying::Texture
{

  public:
    Texture();
    void init(int new_size, bool display_enabled);
    void init(bool display_enabled);
    void destroy(bool display_enabled);
    ~Texture();
    void size_to_at_least(int min_size);
    int bind_blank(int new_size, bool black);
    void bind_data(char* data, int width, int height);
    //void load_png(string file_name);
    void load_tiff(std::string file_name);
    void draw(Size size, double opacity);
    void draw(Size size);
    GLuint id;

  private:
    int size;
    double used_width, used_height;

};

#endif
