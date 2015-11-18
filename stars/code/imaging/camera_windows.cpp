/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "../preprocessor.h"
#if PREPROCESSOR_USING_CAMERA

#include "camera_windows.h"

#include "../parameters/manager.h"
#include "../shared/image/raw.h"
#include "../shared/camera/results.h"
#include "../shared/camera/requests.h"
#include "../shared/general/quit.h"
#include "logger_camera.h"

using namespace Imaging;
using Cameraing::logger;

#define shared_image (*(Shared::Image::raw_from_camera.w))
#define shared_requests (*(Shared::Camera::requests_for_camera.r))
#define shared_results (*(Shared::Camera::results_for_main.w))

CameraWindows::CameraWindows(Parameters::Manager& params, dmm *card): AbstractCamera(params),
    gain_min(0.0), gain_max(-1.0),
    #pragma warning(push)
    #pragma warning(disable: 4355)
    thread(boost::bind(&CameraWindows::thread_function, this))
    #pragma warning(pop)
{
	io_card = card;
}

bool CameraWindows::init_camera()
{
	using namespace Shared::Image;
	using std::string;
	unsigned long listlen = 0;
	
	isCapturing = false;

	camerror = QCam_LoadDriver();
	if (camerror != qerrSuccess) {
		logger.log(format("error: could not initialize camera system: %d") % camerror);
		return false;
	}

	while (!Shared::General::quit)
	{
		io_card->cycle_camera();

		Sleep(3000);
		logger.log(format("Inside init_camera driver..."));

		unsigned long listlen = sizeof(camlist) / sizeof(camlist[0]);
		camerror = QCam_ListCameras(camlist, &listlen);
		if (camerror != qerrSuccess) {
			logger.log(format("error listing cameras: %d") % camerror);
			continue;
		}

		if (listlen == 0) {
			logger.log(format("error: No cameras found in system!"));
			continue;
		}

		camerror = QCam_OpenCamera(camlist[0].cameraId, &camhandle);
		if (camerror != qerrSuccess) {
			logger.log(format("error opening camera: %d") % camerror);
			continue;
		}

		logger.log(format("camera head successfully opened"));
		
		break;		
	}

	if (Shared::General::quit) return false;

	QCam_CreateCameraSettingsStruct(&settings);
	QCam_InitializeCameraSettings(camhandle, &settings);
	QCam_ReadDefaultSettings(camhandle, (QCam_Settings*)&settings);
	
	QCam_SetParam((QCam_Settings*)&settings, qprmImageFormat, qfmtMono16);

	{
		unsigned long xpix, ypix, bpp;
		QCam_GetInfo(camhandle, qinfCcdWidth, &xpix);
		QCam_GetInfo(camhandle, qinfCcdHeight, &ypix);
		QCam_GetInfo(camhandle, qinfBitDepth, &bpp);
		logger.log(format("Found CCD %lu x %lu @ %lu bits") % xpix % ypix % bpp);
		FrameSize = QCam_CalcImageSize(qfmtMono16, xpix, ypix);

		// Allocate memory for the frame buffers 

		frame.pBuffer = new byte[FrameSize];
		frameBuf1 = (byte*)frame.pBuffer;
		frame.bufferSize = FrameSize;
		frame.width = xpix;
		frame.height = ypix;
	}

	QCam_SetParam((QCam_Settings*)&settings, qprmCoolerActive, 1);
	QCam_SetParam((QCam_Settings*)&settings, qprmHighSensitivityMode, 1);
	QCam_SetParam((QCam_Settings*)&settings, qprmBlackoutMode, 1);
	QCam_SetParam((QCam_Settings*)&settings, qprmDoPostProcessing, 0);
	QCam_SetParam((QCam_Settings*)&settings, qprmTriggerDelay, 0);

	//Set the ROI to our display size
	QCam_SetParam((QCam_Settings*)&settings, qprmRoiX, 0);
	QCam_SetParam((QCam_Settings*)&settings, qprmRoiY, 0);
	QCam_SetParam((QCam_Settings*)&settings, qprmRoiWidth, frame.width);
	QCam_SetParam((QCam_Settings*)&settings, qprmRoiHeight, frame.height);

	camerror = QCam_SendSettingsToCam(camhandle, (QCam_Settings*)&settings);
	if (camerror != qerrSuccess) {
		logger.log(format("error sending camera settings: %d") % camerror);
	}
	init_gain();

	set_trigger_mode();

	QCam_SetStreaming(camhandle, 1);
	logger.log(format("Setting streaming on handle %u\n") % camhandle);
	return true;
}

void CameraWindows::clean_up_camera()
{
	if (camera_ready) {
		logger.log(format("Releasing streaming on handle %u\n") % camhandle);
		QCam_SetStreaming(camhandle, 0);
		logger.log("Releasing settings\n");
		QCam_ReleaseCameraSettingsStruct(&settings);
		logger.log("Closing Camera\n");
		QCam_CloseCamera(camhandle);
		logger.log("Releasing driver\n");
		QCam_ReleaseDriver();
	}
}


void CameraWindows::get_trigger_mode()
{
    int error = 0;
	unsigned long trigger_mode;
	camerror = QCam_GetParam((QCam_Settings*)&settings, qprmTriggerType, &trigger_mode);
    if (camerror == qerrSuccess) {
        logger.log(format("got trigger mode %d") % trigger_mode);
        if ((trigger_mode == qcTriggerFreerun) || (trigger_mode == qcTriggerSoftware)) {
            internal_triggering = true;
        }
        else {
            internal_triggering = false;
        }
    }
}

void CameraWindows::set_trigger_mode()
{
    int error = 0;
    unsigned long trigger_mode;
    unsigned long mode6interval = 20000;
    int polarity = 0;

    if (internal_triggering) {
        trigger_mode = qcTriggerFreerun;
		QCam_SetParam((QCam_Settings*)&settings, qprmTriggerDelay, unsigned long(mode6interval));
    } else {
		trigger_mode = qcTriggerStrobeLow;
		QCam_SetParam((QCam_Settings*)&settings, qprmTriggerDelay, 0);
    }

	//camerror = QCam_SetParam((QCam_Settings*)&settings, qprmExposure, unsigned long(internal_exposure_time*1000000.0));
	
	camerror = QCam_SetParam64((QCam_Settings*)&settings, qprm64Exposure, unsigned long(internal_exposure_time*1000000000.0));
	logger.log(format("set exposure time %fs (%d)") % internal_exposure_time % camerror);
	camerror = QCam_SetParam((QCam_Settings*)&settings, qprmTriggerType, trigger_mode);

	camerror = QCam_SendSettingsToCam(camhandle, (QCam_Settings*)&settings);
	
    if (camerror == qerrSuccess) {
        logger.log(format("set trigger mode %d") % trigger_mode);
	}
	else {
		logger.log(format("Error %d setting trigger") % camerror);
	}
	
    get_trigger_mode();
}

void CameraWindows::read_image_if_available()
{
		
	
	logger.log("Grabbing\n");
	isCapturing = true;
	camerror = QCam_GrabFrame(camhandle, &frame);
	isCapturing = false;
	logger.log(format("Done grabbing (%d) frame with timestamp %u\n") % camerror % frame.timeStamp);
	
	if (Shared::General::quit) return;

	
		boost::posix_time::ptime timestamp = boost::posix_time::microsec_clock::local_time();
		shared_image.age.start();

		num_buffers_filled = 0;
		Tools::Timer series_age_timer;
		series_age_timer.start();
		
		logger.log(format("image found with number %u") % frame.frameNumber);
			
		intermediate_buffer = (unsigned short*)frameBuf1;

		if (camerror == qerrSuccess) {
			logger.log("downloading complete");
			int k, kp = 0;
			for (int j = 0; j < shared_image.height; j++) {
				k = j*shared_image.width;
				kp = (image_height - 1 - j)*shared_image.width;
				memcpy(&(buffers[num_buffers_filled][k]), &(intermediate_buffer[kp]), 2 * shared_image.width);
			}
			num_buffers_filled++;
		}
		else {
			logger.log(format("Could not download image from camera: %d\n") % camerror);
			return;
		}
		
		if (num_buffers_filled > 0) {
			bool multiple_triggers = false;
			if (num_buffers_filled > 1) {
				multiple_triggers = true;
			}
			shared_image.num_exposures = num_buffers_filled;
			memset(shared_image.pixels, 0, sizeof(unsigned short)*shared_image.width*shared_image.height);
			for (unsigned int i = 0; i < num_buffers_filled && i < shared_image.max_num_exposures; i++) {

				for (int j = 0; j < shared_image.width*shared_image.height; j++) {
					shared_image.pixels[j] += buffers[i][j];
				}

				memcpy(shared_image.separate_buffers[i], buffers[i], sizeof(unsigned short)*shared_image.width*shared_image.height);
			}
			fill_general_admin();
			fill_real_camera_admin(timestamp, multiple_triggers);
			logger.log(format("sharing image with counter_stars %d") % shared_image.counter_stars);
			share_image();
			if (enough_space_to_save_images()) {
				logger.log("saving image(s)");
				for (unsigned int i = 0; i < num_buffers_filled; i++) {
					save_image(buffers[i], i, multiple_triggers);
				}
			}
			else {
				logger.log("not enough space to save image(s)");
			}
		}
	
}

void CameraWindows::init_gain()
{
	unsigned long minval = 0;
	unsigned long maxval= 0;
	if ((QCam_GetParamMin((QCam_Settings*)&settings, qprmNormalizedGain, &minval) == qerrSuccess)
		&& (QCam_GetParamMax((QCam_Settings*)&settings, qprmNormalizedGain, &maxval) == qerrSuccess)) {

        logger.log(format("got gain properties, range is %f to %f") % ((float)minval / 1000000.0) % ((float)maxval / 1000000.0));				
        gain_min = double(maxval);
        gain_max = double(minval);
	}
    get_gain();
}

void CameraWindows::get_gain()
{
    unsigned long gain2;
	float gain = 0;
	QCam_GetParam((QCam_Settings*)&settings, qprmNormalizedGain, &gain2);
	gain = (float (gain2))/1000000;
    logger.log(format("got gain %f") % gain);
    shared_results.get_gain.found = true;
    shared_results.get_gain.counter = shared_requests.get_gain.counter;
    shared_results.get_gain.value = gain;
    Shared::Camera::results_for_main.share();
}

void CameraWindows::process_requests()
{
    if (shared_requests.get_gain.counter != shared_results.get_gain.counter) {
        shared_results.get_gain.counter = shared_requests.get_gain.counter;
        get_gain();
    }
    if (shared_requests.set_gain.counter != shared_results.set_gain.counter) {
        shared_results.set_gain.counter = shared_requests.set_gain.counter;
        if (shared_requests.set_gain.value >= gain_min && shared_requests.set_gain.value <= gain_max) {
            unsigned long gainval = unsigned long(shared_requests.set_gain.value);
            logger.log(format("setting gain to %f") % gainval);
			QCam_SetParam((QCam_Settings*)&settings, qprmNormalizedGain, gainval);
			QCam_SendSettingsToCam(camhandle, (QCam_Settings*)&settings);
        }
        get_gain();
    }
}

void CameraWindows::thread_function()
{
    #ifdef WIN32
    DWORD thread_priority;
	//int wait_time = 30;				//time between taking pictures, in seconds
    //thread_priority = GetThreadPriority(GetCurrentThread());
    //SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
    //thread_priority = GetThreadPriority(GetCurrentThread());
    logger.log(format("main has thread priority %i")%thread_priority);
    #endif

    while (!Shared::General::quit) {
        update();
        if (enabled) {
            if (need_to_try_camera_ready()) {
                if (init_camera()) {
                    camera_ready = true;
                    shared_results.connected = true;
                    Shared::Camera::results_for_main.share();
                }
			}
            if (camera_ready) {
				logger.log("Camera is ready, trying to read image");
                read_image_if_available();
                process_requests();
            }
			if (internal_triggering) {
				for (int i = 1; (!Shared::General::quit) && i < 100; i++) {
					usleep(int(internal_period * 10000));
				}
			}
        }
		last_remote_buffer_counter++;
    }
	logger.log("Cleaning up camera");
    clean_up_camera();
}

void CameraWindows::wait_for_quit()
{
	logger.log("Abort QCam\n");
	if (isCapturing) QCam_Abort(camhandle);
	thread.interrupt();
    thread.join();
}

#endif
