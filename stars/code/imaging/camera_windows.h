/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "../preprocessor.h"
#if PREPROCESSOR_USING_CAMERA

#pragma once
#ifndef IMAGING__CAMERA_WINDOWS_H
#define IMAGING__CAMERA_WINDOWS_H

#ifdef _WIN32
    #include <windows.h>
#endif
#include <boost/thread/thread.hpp>
#include "abstract_camera.h"
#include <QCamAPI.h>
#include <QCamImgfnc.h>
// QCamAPI.h is idiotic and defines types with a macro.  This breaks lots of stuff
#undef uint32_t
#undef uint16_t
#undef uint8_t
#include <cstdint>

#include "../dmm.h"

namespace Parameters
{
    class Manager;
}

namespace Imaging
{
    class CameraWindows;
}

class Imaging::CameraWindows: public Imaging::AbstractCamera
{
  public:
    CameraWindows(Parameters::Manager& params, dmm *card);

    bool init_camera();
    void clean_up_camera();
    void read_image_if_available();

    void get_trigger_mode();
    void set_trigger_mode();

	void init_gain();
	void get_gain();
    void process_requests();

    void thread_function();
    void wait_for_quit();

  private:
    QCam_Err camerror;
	QCam_CamListItem camlist[10];  // List of connected cameras		//Added by KNS to list cameras
	QCam_Handle camhandle;
	QCam_SettingsEx settings;
	unsigned char *frameBuf1;
    QCam_Frame frame;
	unsigned long FrameSize;	
	unsigned short FrameNumber;

    double gain_min;
    double gain_max;
	dmm *io_card;

    // thread belongs to derived class to prevent pure virtual function call
    boost::thread thread;
};

#endif

#endif
