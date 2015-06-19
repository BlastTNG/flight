/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__ABSTRACT_CAMERA_H
#define IMAGING__ABSTRACT_CAMERA_H

#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "fake_sky.h"
#include "../tools/timing.h"

namespace Parameters
{
    class Manager;
}

namespace Imaging
{
    class AbstractCamera;
}

class Imaging::AbstractCamera
{
  public:
    AbstractCamera(Parameters::Manager& params);
    virtual ~AbstractCamera();
    void update();
    void fill_general_admin();
    void fill_real_camera_admin(boost::posix_time::ptime& timestamp, bool multiple_triggers);
    void save_image(unsigned short* pixels, int buffer_num, bool multiple_triggers);
    void share_image();
    bool enough_space_to_save_images();
    bool need_to_try_camera_ready();
    virtual void wait_for_quit();

  protected:
    virtual bool init_camera() = 0;
    virtual void clean_up_camera() = 0;
    virtual void read_image_if_available() = 0;

    static const unsigned int max_num_buffers = 4;

    bool enabled;
    bool internal_triggering;
    double internal_exposure_time;
    double internal_period;
    std::string output_dir;

    bool camera_ready;
    Tools::Timer check_camera_ready_timer;
    double check_camera_ready_period;
    double max_exposure_time;

    unsigned short* intermediate_buffer;
    unsigned short* buffers[max_num_buffers];
    unsigned int num_buffers_filled;

    int last_remote_buffer_counter;
    int image_width, image_height;
	int last_lens_requests_focus_counter;

    FakeSky fake_sky;
};

#endif
