/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "../preprocessor.h"
#if (!PREPROCESSOR_USING_CAMERA)

#pragma once
#ifndef IMAGING__CAMERA_LINUX_H
#define IMAGING__CAMERA_LINUX_H

#include <boost/thread/thread.hpp>
#include "abstract_camera.h"

namespace Parameters
{
    class Manager;
}

namespace Imaging
{
    class Camera;
}

class Imaging::Camera: public Imaging::AbstractCamera
{
  public:
    Camera(Parameters::Manager& params);
    bool init_camera();
    void clean_up_camera();
    void read_image_if_available();
    void thread_function();
    void wait_for_quit();

  private:

    boost::thread thread;
};

#endif

#endif
