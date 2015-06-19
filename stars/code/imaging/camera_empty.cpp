/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "../preprocessor.h"
#if (!PREPROCESSOR_USING_CAMERA)

#include "camera_empty.h"
#include "../shared/general/quit.h"

using namespace Imaging;

Camera::Camera(Parameters::Manager& params): AbstractCamera(params),
    #pragma warning(push)
    #pragma warning(disable: 4355)
    thread(boost::bind(&Camera::thread_function, this))
    #pragma warning(pop)
{
}

bool Camera::init_camera()
{
    return false;
}

void Camera::clean_up_camera()
{
}

void Camera::read_image_if_available()
{
}

void Camera::thread_function()
{
    while (!Shared::General::quit) {
        update();
        if (enabled) {
            if (need_to_try_camera_ready()) {
                if (init_camera()) {
                    camera_ready = true;
                }
            }
            if (camera_ready) {
                read_image_if_available();
            }
        }
        usleep(50000);
    }
    clean_up_camera();
}

void Camera::wait_for_quit()
{
    thread.join();
}

#endif
