/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__RAW_H
#define SHARED__IMAGE__RAW_H

#include <string>
#include "../circular_buffer.h"
#include "../../tools/timing.h"
#include "../solving/filters.h"

namespace Shared
{
    namespace Image
    {

class Raw
{
  public:
    Raw();
    ~Raw();
    void init(Parameters::Manager& params);
    double get_pixel(int x, int y);
    Raw& operator=(const Raw& rhs);
    bool is_inbounds(int& x, int& y);
    bool is_inbounds(double& x, double& y);
    bool is_xy_inbounds(double& x, double& y);

    std::string which_sensor;
    bool from_camera;
    unsigned int num_exposures;
    int counter_fcp;
    int counter_stars;
    int key_counter_stars;
    std::string dirname;
    std::string filename_base;
    std::string filename;
    Tools::Timer age;
    bool to_be_solved;
    int width;
    int height;
    int depth;
    int single_depth;
    unsigned short* pixels;

    static const unsigned int max_num_exposures = 4;
    unsigned short* separate_buffers[max_num_exposures];

    bool focus_known;
    int focus;
    bool aperture_known;
    int aperture;
    bool gain_known;
    double gain;

    Shared::Solving::Filters filters;

    bool has_netisc_framenum;
    int netisc_framenum;
};

// writes: camera
//  reads: solver
extern Shared::CircularBuffer <Raw> raw_from_camera;

// writes: camera
//  reads: net_image_client1
extern Shared::CircularBuffer <Raw> raw_for_image_client1;

// writes: camera
//  reads: net_image_client2
extern Shared::CircularBuffer <Raw> raw_for_image_client2;

    }
}

#endif
