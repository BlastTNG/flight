/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#ifndef STAR_CAMERA_H
#define STAR_CAMERA_H

#include "tools/timing.h"
#include "imaging/lens.h"
#include "solving/solver.h"
#include "dmm.h"

namespace Parameters
{
    class Manager;
}

namespace Imaging
{
    class AbstractCamera;
};

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

class StarCamera
{
  public:
    StarCamera(Parameters::Manager& params, Shared::Image::Raw& solvers_working_image);
    void pick_camera(Parameters::Manager& params);
    void update_framerate();
    void set_thread_priority();
    void update_main();
	void initialize_ADCard();
    void run(Parameters::Manager& params);

    enum InputDeviceType {
        input_device_camera_windows,
        input_device_camera_filesystem,
        input_device_none
    };

  private:
    Imaging::AbstractCamera* camera;
    Imaging::Lens lens;
    Solving::Solver solver;

    Tools::Timer update_timer;
    Tools::Timer display_timer;
    double update_period;
    Tools::Timer framerate_timer;
    unsigned int framerate_counter;
    InputDeviceType input_device;

	dmm io_card;
};

#endif
