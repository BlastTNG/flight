/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "textures.h"
#include <boost/filesystem/operations.hpp>
#include "../parameters/manager.h"

using namespace Displaying;

Textures::Textures(Parameters::Manager& params)
{
    display_enabled = params.general.try_get("main.display_enabled", true);
    stars_dir = params.stars_dir;
}

Textures::~Textures()
{
    if (display_enabled) {
        font.Destroy();
    }
    for (int i=0; i<num_textures; i++) {
        textures[i].destroy(display_enabled);
    }
}

void Textures::init()
{
    using namespace boost::filesystem;
	textures[basic_drawing].init(display_enabled);
    textures[font_texture].init(display_enabled);
    //textures[drawing_3d].init(display_enabled);
    if (display_enabled) {
        font.Create(system_complete(stars_dir + "resources/fonts/arial24_with_degrees.glf").string(), get(font_texture));
    }
}

GLuint Textures::get(texture_name name)
{
    return textures[name].id;
}

