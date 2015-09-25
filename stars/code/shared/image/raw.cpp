/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "raw.h"
#include <cstring>
#include "../../parameters/manager.h"

using namespace Shared::Image;

Raw::Raw()
{
    which_sensor = "unknown";
    from_camera = false;
    num_exposures = 0;

    counter_stars = -1;
    key_counter_stars = -1;
    counter_fcp = -1;
    dirname = "";
    filename_base = "";
    filename = "";
    to_be_solved = false;

    focus_known = false;
    focus = -1;
    aperture_known = false;
    aperture = -1;
    gain_known = false;
    gain = -1.0;

    has_netisc_framenum = false;
    netisc_framenum = 0;

    width = 0;
    height = 0;
    depth = 0;
    single_depth = 0;
}

void Raw::init(Parameters::Manager& params)
{
	width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);
	depth = params.general.try_get("imaging.camera_real.image_depth", params.general.image_depth);

    single_depth = depth;
    pixels = new unsigned short[width*height];
    for (unsigned int i=0; i<max_num_exposures; i++) {
        separate_buffers[i] = new unsigned short[width*height];
    }
    pixels = new unsigned short[width*height];
    which_sensor = params.general.try_get("main.which_sensor", std::string("unknown"));
    filters.init(params);
}

Raw::~Raw()
{
    delete [] pixels;
    for (unsigned int i=0; i<max_num_exposures; i++) {
        delete [] separate_buffers[i];
    }
}

double Raw::get_pixel(int x, int y)
{
    if (x>=0 && x<width && y>=0 && y<height) {
        return pixels[y*width + x];
    }
    return 0.0;
}

bool Raw::is_inbounds(int& x, int& y)
{
    if (x<0 || x>=width) {
        return false;
    }
    if (y<0 || y>=height) {
        return false;
    }
    return true;
}

bool Raw::is_inbounds(double& x, double& y)
{
    if (x<0 || x>=width) {
        return false;
    }
    if (y<0 || y>=height) {
        return false;
    }
    return true;
}

bool Raw::is_xy_inbounds(double& x, double& y)
{
    if (x < -width/2.0 || x > width/2.0) {
        return false;
    }
    if (y < -height/2.0 || y > height/2.0) {
        return false;
    }
    return true;
}

Raw& Raw::operator=(const Raw &rhs)
{
    if (this != &rhs) {
        which_sensor = rhs.which_sensor;
        from_camera = rhs.from_camera;
        num_exposures = rhs.num_exposures;

        counter_stars = rhs.counter_stars;
        key_counter_stars = rhs.key_counter_stars;
        counter_fcp = rhs.counter_fcp;
        dirname = rhs.dirname;
        filename_base = rhs.filename_base;
        filename = rhs.filename;
        age = rhs.age;

        to_be_solved = rhs.to_be_solved;
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;
        single_depth = rhs.single_depth;
        memcpy(pixels, rhs.pixels, width*height*sizeof(unsigned short));

        for (unsigned int i=0; i<rhs.num_exposures; i++) {
            memcpy(separate_buffers[i], rhs.separate_buffers[i], width*height*sizeof(unsigned short));
        }

        focus_known = rhs.focus_known;
        focus= rhs.focus;
        aperture_known = rhs.aperture_known;
        aperture= rhs.aperture;
        gain_known = rhs.gain_known;
        gain = rhs.gain;

        has_netisc_framenum = rhs.has_netisc_framenum;
        netisc_framenum = rhs.netisc_framenum;

        filters = rhs.filters;
        //horizontal_known = rhs.horizontal_known;
        //lat = rhs.lat;
        //lst = rhs.lst;
    }
    return *this;
}

