/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "star_camera.h"
#include <string>
#include "preprocessor.h"
#include "imaging/camera_empty.h"
#include "imaging/camera_windows.h"
#include "imaging/camera_filesystem.h"
#include "parameters/manager.h"
#include "networking/network_manager.h"
#include "displaying/manager.h"
#include "housekeeping/housekeeper.h"
#include "logger_main.h"
#include "tools/quick_cout.h"
#include "shared/update.h"
#include "shared/general/main_settings.h"
#include "shared/general/shutdown.h"
#include "shared/general/quit.h"

using Main::logger;
#define shared_main_settings (*(Shared::General::main_settings_net_for_main.r))
#define shared_shutdown (*(Shared::General::shutdown_for_main.r))

StarCamera::StarCamera(Parameters::Manager& params, Shared::Image::Raw& solvers_working_image):
    lens(params), solver(params, solvers_working_image)
{
    update_period = 1.0 / params.general.try_get("main.update_frequency", 20.0);
    framerate_counter = 0;
    std::string input_device_str = params.general.try_get("imaging.camera.device", string("none"));
    input_device = input_device_none;
    if (input_device_str == "camera_windows") {
        input_device = input_device_camera_windows;
    }
    if (input_device_str == "camera_filesystem") {
        input_device = input_device_camera_filesystem;
    }
    camera = NULL;
}

void StarCamera::pick_camera(Parameters::Manager& params)
{
    switch (input_device) {
        case input_device_camera_windows:
            #if PREPROCESSOR_USING_CAMERA
            camera = new Imaging::CameraWindows(params, &io_card);
            #endif
            break;
        case input_device_camera_filesystem:
            camera = new Imaging::CameraFilesystem(params);
            logger.log("using camera_filesystem");
            break;
        case input_device_none:
            camera = NULL;
            break;
    }
}

void StarCamera::update_framerate()
{
    if (framerate_timer.started()) {
        framerate_counter++;
        if (framerate_timer.time() > 5.0) {
            double framerate = double(framerate_counter) / framerate_timer.time();
            logger.log(format("framerate is %d")%framerate);
            framerate_timer.start();
            framerate_counter = 0;
        }
    } else {
        framerate_timer.start();
        framerate_counter = 0;
    }
}

void StarCamera::set_thread_priority()
{
    #ifdef WIN32
    DWORD thread_priority;
    thread_priority = GetThreadPriority(GetCurrentThread());
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
    thread_priority = GetThreadPriority(GetCurrentThread());
    logger.log(format("main has thread priority %i")%thread_priority);
    #endif
}

void StarCamera::update_main()
{
    Shared::update(Shared::ThreadNames::main);
    if (shared_shutdown.shutdown_now) {
        Shared::General::quit = true;
    }
    logger.update();
}

void StarCamera::run(Parameters::Manager& params)
{
    Displaying::DisplayManager display_manager(params);
    display_manager.draw();

	io_card.dmm_initialize();

    pick_camera(params);
    Housekeeping::Housekeeper housekeeper(params, &io_card);
    Networking::NetworkManager network_manager;

    //set_thread_priority();

    display_manager.draw();
    update_timer.start();
    display_timer.start();
    while (!Shared::General::quit) {
        if (update_timer.time() > update_period) {
            double time_passed_since_last_update = update_timer.time();
            update_timer.start();
            update_main();
            display_manager.process_events();
            display_manager.update_zoom(time_passed_since_last_update);
            housekeeper.update();
        }
        if (display_timer.time() > shared_main_settings.display_period) {
            update_framerate();
            display_timer.start();
            display_manager.draw();
        }
        usleep(2000);
    }
    display_manager.draw();

    if (camera) {
        camera->wait_for_quit();
    }
    solver.wait_for_quit();
    lens.wait_for_quit();
    network_manager.wait_for_quit();
    display_manager.wait_for_quit();
}

