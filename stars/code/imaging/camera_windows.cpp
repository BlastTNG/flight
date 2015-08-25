/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "../preprocessor.h"
#if PREPROCESSOR_USING_CAMERA

#include "camera_windows.h"
//#include "QCamApi.h"
#include "dscud.h"	
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

CameraWindows::CameraWindows(Parameters::Manager& params): AbstractCamera(params),
    gain_min(0.0), gain_max(-1.0),
    #pragma warning(push)
    #pragma warning(disable: 4355)
    thread(boost::bind(&CameraWindows::thread_function, this))
    #pragma warning(pop)
{
}

bool CameraWindows::init_camera()
{
	using namespace Shared::Image;
	using std::string;
	// int camerror;

	DSCCB dsccb;		   //struct w/ info for board init
	DSCB dscb;			   //board reference
	BYTE dscbresult;	   //variable for error handling
	ERRPARAMS errparams;   //variable that spits out daq errors
	static const BYTE port = 0;				//, bit;

	if (dscInit(DSC_VERSION) != DE_NONE)
	{
		dscGetLastError(&errparams);
		logger.log(format("Error initializing driver %s\n") % errparams.errstring);
		Sleep(300);
		return false;
	}

	logger.log(format("initialized driver..."));

	dsccb.base_address = 0x300;
	dsccb.int_level = 7;

	if ((dscbresult = dscInitBoard(DSC_DMM, &dsccb, &dscb)) != DE_NONE)
	{
		dscGetLastError(&errparams);
		logger.log(format("Talking to DMM-XT failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
		return false;
	}
	logger.log(format("DAQ is a-go"));

	logger.log(format("Power cycling the camera..."));
	logger.log(format("Lowering pins on the DIO... "));
	if ((dscbresult = dscDIOOutputByte(dscb, port, 0x0)) != DE_NONE)
	{
		dscGetLastError(&errparams);
		logger.log(format("failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
		//       return dscbresult;
	}
	Sleep(500);
	logger.log(format("Raising Bit 3 on the DIO... "));	//camera on/off
	if ((dscbresult = dscDIOSetBit(dscb, port, 4)) != DE_NONE)
	{
		dscGetLastError(&errparams);
		logger.log(format("failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
		//      return false;
	}
	Sleep(500);
	logger.log(format("Lowering pins on the DIO... "));
	if ((dscbresult = dscDIOOutputByte(dscb, port, 0x0)) != DE_NONE)
	{
		dscGetLastError(&errparams);
		logger.log(format("failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
		return false;
	}
	Sleep(3000);
	// Initialize the camera driver
	logger.log(format("Inside init_camera driver..."));

	camerror = QCam_LoadDriver();
	if (camerror != qerrSuccess) {
		logger.log(format("error: could not initialize camera system: "));
		QCam_ReleaseDriver();
		return false;
	}
	unsigned long listlen = sizeof(camlist) / sizeof(camlist[0]);
	camerror = QCam_ListCameras(camlist, &listlen);
	if (camerror != qerrSuccess) {
		logger.log(format("error opening camera"));
		QCam_ReleaseDriver();
		return false;
	}

	if (listlen == 0) {
		//QCam_ReleaseDriver();
		//logger.log(format("releasing driver"));
		logger.log(format("Power cycling the DIO, again..."));
		logger.log(format("Lowering pins on the DIO... "));
		logger.log(format("releasing driver"));
		QCam_ReleaseDriver();

		if ((dscbresult = dscDIOOutputByte(dscb, port, 0x0)) != DE_NONE)
		{
			dscGetLastError(&errparams);
			logger.log(format("failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
			//        return dscbresult;
		}
		Sleep(500);
		logger.log(format("Raising Bit 3 on the DIO..."));
		if ((dscbresult = dscDIOSetBit(dscb, port, 4)) != DE_NONE)
		{
			dscGetLastError(&errparams);
			logger.log(format("failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
			//        return dscbresult;
		}
		Sleep(500);
		logger.log(format("Lowering pins on the DIO..."));

		if ((dscbresult = dscDIOOutputByte(dscb, port, 0x0)) != DE_NONE)
		{
			dscGetLastError(&errparams);
			logger.log(format("failed: %s (%s)\n") % dscGetErrorString(dscbresult) % errparams.errstring);
			//        return dscbresult;
		}

		Sleep(5000);

		camerror = QCam_LoadDriver();
		if (camerror != qerrSuccess) {
			logger.log(format("error: could not initialize camera system: "));	//%s
			QCam_ReleaseDriver();
			return false;
		}

		listlen = sizeof(camlist) / sizeof(camlist[0]);
		if (listlen == 0){
			logger.log(format("error: No cameras found in system!"));
			QCam_ReleaseDriver();
			return false;
		}
		camerror = QCam_ListCameras(camlist, &listlen);
		if (camerror != qerrSuccess) {
			logger.log(format("error opening camera"));
			QCam_ReleaseDriver();
			return false;
		}
	}
	if ((listlen > 0) && (camlist[0].isOpen == false)) {
		camerror = QCam_OpenCamera(camlist[0].cameraId, &camhandle);
	}
	//camerror = QCam_OpenCamera(1, &camhandle);
	if (camerror != qerrSuccess) {
		logger.log(format("error opening camera head"));
		QCam_ReleaseDriver();
		return false;
	}
	/*	camerror = MP_OpenCamera(1, &camhandle);
	if (camerror < 0) {
	logger.log(format("error opening camera head: %s") % string(MP_GetErrMsg(camerror)));
	MP_CloseSys();
	return false;*/
	//}
	logger.log(format("camera head successfully opened"));
	session_id = 0;

	camerror = QCam_GetInfo(camhandle, qinfUniqueId, &session_id);
	if (camerror != qerrSuccess) {
		logger.log(format("couldn't open camera handle"));
		QCam_ReleaseDriver();
		return false;
	}

	//camerror = MP_GetCam1394Sid(camhandle, &session_id);
	//   if (camerror < 0) {
	//       logger.log(format("couldn't open camera handle: %s") % string(MP_GetErrMsg(camerror)));
	//       MP_CloseSys();
	//       return false;
	//   }
	logger.log(format("got session_id %d") % session_id);
	//   /*MP_SetTriggerState(camhandle, 1);*/

	//FWWindowType window = {image_width, 0, 0, image_height};
	/*QCam_ReadDefaultSettings( camhandle, &settings );
	QCam_SendSettingsToCam( camhandle, &settings );*/

	settings.size = sizeof(settings);
	QCam_ReadDefaultSettings(camhandle, &settings);
	QCam_SetParam(&settings, qprmImageFormat, qfmtMono16);
	QCam_SendSettingsToCam(camhandle, &settings);

	unsigned long xpix, ypix, bpp;
	QCam_GetInfo(camhandle, qinfCcdWidth, &xpix);
	QCam_GetInfo(camhandle, qinfCcdHeight, &ypix);
	QCam_GetInfo(camhandle, qinfBitDepth, &bpp);

	//QCam_SetParam( &settings, qprmImageFormat, qfmtMono16);
	//unsigned long 	xpix = width, ypix = height;
	FrameSize = xpix*ypix * 2;

	// Allocate memory for the frame buffers 
	frameBuf1 = new unsigned char[FrameSize];

	frame.pBuffer = frameBuf1; //new unsigned char[FrameSize];
	frame.bufferSize = FrameSize;
	frame.width = xpix;
	frame.height = ypix;
	QCam_qcShutterControl(0);
	QCam_SetParam(&settings, qprmTriggerDelay, unsigned long(int(internal_period * 1000000000)));
	QCam_SetParam(&settings, qprmExposure, unsigned long(int(internal_exposure_time*1000000.0)));
	QCam_SetParam(&settings, qprmCoolerActive, 1);
	QCam_SetParam(&settings, qprmHighSensitivityMode, 0);
	QCam_SetParam(&settings, qprmBlackoutMode, 0);

	QCam_SendSettingsToCam(camhandle, &settings);

	//MP_SetMechShutter(camhandle, 0, 1, 0);
	//   MP_SetBitWindow(camhandle, 12, 0);  //set the bit depth
	//   MP_SetFWFormat(camhandle, window, FW_COLORFMT_MONO16); //set the image transfer format
	//   MP_SetTriggerState(camhandle, 1);
	//   //MP_SetTriggerMode(camhandle, 1, 0);
	//   MP_SetMode6Interval(camhandle, float(int(internal_period*1000.0)));
	//   MP_SetTriggerPolarity(camhandle, 0);
	//   MP_SetTriggerSource(camhandle, 0);
	//   MP_SetTriggerIntTime(camhandle, float(int(internal_exposure_time*1000.0)));

	//int width, height, bpp;
	/*unsigned long width, height, bpp;
	QCam_GetInfo( camhandle, qinfCcdWidth,  &width );
	QCam_GetInfo( camhandle, qinfCcdHeight, &height );
	QCam_GetInfo( camhandle, qinfBitDepth, &bpp );*/
	//	  imaq1394GetAttribute (session_id, IMG1394_ATTR_IMAGE_WIDTH, &width);
	//   imaq1394GetAttribute (session_id, IMG1394_ATTR_IMAGE_HEIGHT, &height);
	//   imaq1394GetAttribute (session_id, IMG1394_ATTR_BYTES_PER_PIXEL, &bpp);

	//   unsigned long newgain = 500000;		//set the gain manually
	//QCam_SetParam( &settings, qprmNormalizedGain, newgain);
	//QCam_SendSettingsToCam(camhandle, &settings);
	init_gain();



	//    Rect rectangle = {0, 0, 0, 0};
	//    rectangle.width = unsigned long(image_width);
	//    rectangle.height = unsigned long(image_height);
	////    IMG_ERR imaq_error;
	//    char error_message[256];
	//    for (int i=0; i<256; i++) {
	//        error_message[i] = '\0';
	//   }


	unsigned long trigger_mode;
	//int trigger_mode = 0;
	int count = 0;
	unsigned long last_remote_buffer_counter;

	////   MP_GetTriggerMode(camhandle, &trigger_mode, &count);
	//QCam_GetParam( &settings, qprmTriggerType, &trigger_mode );
	//
	//   imaq_error = imaq1394ConfigureAcquisition(session_id, 1, 2, rectangle);
	//   if (imaq_error == IMG1394_ERR_GOOD) {
	//       logger.log("configure acquisition succcess");
	//   }
	//   else {
	//       imaq1394ShowError(imaq_error, error_message, 256);
	//       logger.log(format("configure acqusition error: %d = %s")
	//           % imaq_error % error_message);
	//   }

	//   imaq_error = imaq1394StartAcquisition(session_id);
	//   if (imaq_error == IMG1394_ERR_GOOD) {
	//       logger.log("start acquisition succcess");
	//   }
	//   else {
	//       imaq1394ShowError(imaq_error, error_message, 256);
	//       logger.log(format("start acqusition error: %d = %s")
	//           % imaq_error % error_message);
	//   }

	//imaq1394GetAttribute(session_id, IMG1394_ATTR_LAST_TRANSFERRED_BUFFER_NUM, &last_remote_buffer_counter);
	//   MP_GetTriggerMode(camhandle, &trigger_mode, &count);
	QCam_GetParam(&settings, qprmFrameBufferLength, &last_remote_buffer_counter);
	//QCam_GetInfo( &settings, qinfImageSize, &last_remote_buffer_counter);
	QCam_GetParam(&settings, qprmTriggerType, &trigger_mode);
	set_trigger_mode();

	return true;
}

void CameraWindows::clean_up_camera()
{
	if (camera_ready) {
		QCam_SetStreaming(camhandle, 0);
		QCam_CloseCamera(camhandle);
		QCam_ReleaseDriver();
	}
		
	/*if (camera_ready) {
        imaq1394StopAcquisition(session_id);
    	MP_SetTriggerState(camhandle, 0);
        MP_CloseCamera(camhandle);
        MP_CloseSys();
    }*/
}


void CameraWindows::get_trigger_mode()
{
    int error = 0;
	unsigned long trigger_mode;
	//int trigger_mode = 0;
    //int count = 0;
	//QCam_ReadSettingsFromCam(camhandle, &settings);
    camerror = QCam_GetParam( &settings, qprmTriggerType, &trigger_mode );
	//error = MP_GetTriggerMode(camhandle, &trigger_mode, &count);
    if (camerror == qerrSuccess) {
        logger.log(format("got trigger mode %d") % trigger_mode);
        if (trigger_mode == 0) {
            internal_triggering = false;
        }
        else if (trigger_mode == 5) {
            internal_triggering = false;
        }
        else {
            internal_triggering = true;
        }
    }
}

void CameraWindows::set_trigger_mode()
{
    int error = 0;
    int trigger_mode;
	//int trigger_mode = 0;
    int triggersource = 0;
    int mode6interval = 20000;
    int polarity = 0;

    if (internal_triggering) {
        trigger_mode = 6;
    } else {
        trigger_mode = 1;
    }

	QCam_qcShutterControl(0);
	QCam_SetParam( &settings, qprmTriggerDelay, unsigned long(mode6interval));
	QCam_SetParam( &settings, qprmExposure, unsigned long(int(internal_exposure_time*1000000.0)));
	
	
    //MP_SetMechShutter(camhandle, 1, 1, 1);
    //MP_SetTriggerSource(camhandle, triggersource);
    //MP_SetTriggerState(camhandle, 1);
    //MP_SetMode6Interval(camhandle, mode6interval);
    //MP_SetTriggerPolarity(camhandle, polarity);
	camerror = QCam_SetParam( &settings, qprmTriggerType, qcTriggerFreerun);
	//camerror = QCam_SetParam( &settings, qprmTriggerType, qcTriggerSoftware ); 
   QCam_SendSettingsToCam(camhandle, &settings);
	// error = MP_SetTriggerMode(camhandle, trigger_mode, 0);
    if (camerror == qerrSuccess) {
        logger.log(format("set trigger mode %d") % trigger_mode);
    }
	
    get_trigger_mode();
}

void CameraWindows::read_image_if_available()
{
    unsigned long remote_buffer_counter;
	
	//int remote_buffer_counter;
	
//		QCam_Trigger(camhandle);
		QCam_GetInfo(camhandle, qinfImageSize, &remote_buffer_counter);	
		//QCam_GetParam(&settings, qprmFrameBufferLength, &remote_buffer_counter);
		/*frame.pBuffer = new unsigned char[remote_buffer_counter];
		frame.bufferSize = remote_buffer_counter;*/
		
	//imaq1394GetAttribute(session_id, IMG1394_ATTR_LAST_TRANSFERRED_BUFFER_NUM, &remote_buffer_counter);
    if (remote_buffer_counter != last_remote_buffer_counter) {
        boost::posix_time::ptime timestamp = boost::posix_time::microsec_clock::local_time();
        shared_image.age.start();

        num_buffers_filled = 0;
        Tools::Timer series_age_timer;
        series_age_timer.start();
        while (
                  (num_buffers_filled < max_num_buffers) &&
                  (num_buffers_filled < shared_requests.max_num_triggers) &&
                  (
                      (shared_requests.max_num_triggers == 1) ||
                      (series_age_timer.time() < shared_requests.multi_triggering_delay)
                  )
              )
        {
          //  QCam_Trigger(camhandle);
			QCam_GetInfo(camhandle, qinfImageSize, &remote_buffer_counter);
		/*	frame.pBuffer = new unsigned char[remote_buffer_counter];
			frame.bufferSize = remote_buffer_counter;*/
						
			//imaq1394GetAttribute(session_id, IMG1394_ATTR_LAST_TRANSFERRED_BUFFER_NUM, &remote_buffer_counter);
            if (remote_buffer_counter != last_remote_buffer_counter) {
                logger.log(format("image found with buffer %d")%frame.bufferSize);
				//logger.log(format("image found with remote_buffer_counter %d (last_remote_buffer_counter was %d)")%remote_buffer_counter%last_remote_buffer_counter);
                last_remote_buffer_counter = remote_buffer_counter;
				//last_remote_buffer_counter = frame.bufferSize;
			
               //IMG_ERR QCam_Err;
				//IMG_ERR imaq_error;
                //uInt32 grabbed = 0;
                //char error_message[256];
                /*imaq_error = imaq1394GetBuffer2(session_id, remote_buffer_counter, &grabbed,
                    IMG1394_ONOVERWRITE_GET_OLDEST, (void **)&intermediate_buffer);
                imaq1394ShowError(imaq_error, error_message, 256);*/
               QCam_SetStreaming(camhandle, 1);
			   camerror = QCam_GrabFrame(camhandle, &frame);
			   //QCam_Trigger(camhandle);
			   intermediate_buffer = (unsigned short*)frameBuf1;
			  // shared_image.height = image_height;
			 //  shared_image.width = image_width;
					if (camerror == qerrSuccess) {
				//if (imaq_error == IMG1394_ERR_GOOD) {
                    logger.log("downloading complete");
                    int k, kp = 0;
					   for (int j=0; j<shared_image.height; j++) {
                        for (int i=0; i<shared_image.width; i++) {
                        //for (int j=0; j<image_height; j++) {
                       // for (int i=0; i<image_width; i++) {    
							//k = j*image_width + i;
							//kp = j*image_height + i;
							k = j*shared_image.width + i;
                            //kp = j*shared_image.height + i;
							kp = (image_height-1-j)*shared_image.width + i;
                            //buffers[num_buffers_filled][k] = ((unsigned short*)frame.pBuffer)[kp];
							//buffers[num_buffers_filled][k] = ((unsigned short*)frameBuf1)[kp];
							buffers[num_buffers_filled][k] = ((unsigned short*)intermediate_buffer)[kp];
							//buffers[num_buffers_filled][k] = (intermediate_buffer)[kp];
                        }
                    }
                    num_buffers_filled++;
				}
            }
        }
        if (num_buffers_filled > 0) {
            bool multiple_triggers = false;
            if (num_buffers_filled > 1) {
                multiple_triggers = true;
            }
            shared_image.num_exposures = num_buffers_filled;
            memset(shared_image.pixels, 0, sizeof(unsigned short)*shared_image.width*shared_image.height);
            for (unsigned int i=0; i<num_buffers_filled && i<shared_image.max_num_exposures; i++) {
                
                for (int j=0; j<shared_image.width*shared_image.height; j++) {
                    shared_image.pixels[j] += buffers[i][j];
                }
                
                memcpy(shared_image.separate_buffers[i], buffers[i], sizeof(unsigned short)*shared_image.width*shared_image.height);
            }
            fill_general_admin();
            fill_real_camera_admin(timestamp, multiple_triggers);
            logger.log(format("sharing image with counter_stars %d")%shared_image.counter_stars);
            share_image();
            if (enough_space_to_save_images()) {
                logger.log("saving image(s)");
                for (unsigned int i=0; i<num_buffers_filled; i++) {
                    save_image(buffers[i], i, multiple_triggers);
                }
            } else {
                logger.log("not enough space to save image(s)");
            }
        }
    }
}

void CameraWindows::init_gain()
{
    //int is_avail, oneshot_avail, auto_avail;
	unsigned long minval = 0;
	//float minval = 0;
	unsigned long maxval= 0;
	//float maxval = 0;
    if (QCam_GetParamMin( &settings, qprmNormalizedGain, &minval ) && QCam_GetParamMax( &settings, qprmNormalizedGain, &maxval ) == 0) {

	//if (MP_PropertiesGain(camhandle, NULL, &is_avail, &maxval, &minval, &oneshot_avail, &auto_avail) == 0) {
        // for some reason minval and maxval seem to be swapped in function call
        logger.log(format("got gain properties, range is %d to %d")		//%u switched from %d KNS
            % minval % maxval);				//switched minval and maxval
        gain_min = double(maxval);
        gain_max = double(minval);
	}
    get_gain();
}

void CameraWindows::get_gain()
{
    unsigned long gain2;
	float gain = 0;
	//float gain = 0;
    QCam_GetParam( &settings, qprmNormalizedGain, &gain2 );
	//MP_GetGain(camhandle, NULL, &gain);
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
            QCam_SetParam( &settings, qprmNormalizedGain, gainval );
			QCam_SendSettingsToCam(camhandle, &settings);
			//MP_SetGain(camhandle, NULL, gainval);
        }
        get_gain();
    }
}

void CameraWindows::thread_function()
{
    #ifdef WIN32
    DWORD thread_priority;
	//int wait_time = 30;				//time between taking pictures, in seconds
    thread_priority = GetThreadPriority(GetCurrentThread());
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    thread_priority = GetThreadPriority(GetCurrentThread());
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
                read_image_if_available();
                process_requests();
            }
        }
        //usleep(50000);
		//usleep(wait_time*1000000);			//added a variable KNS, was 50000
		usleep(int(internal_period*1000000));
		last_remote_buffer_counter++;
    }
    clean_up_camera();
}

void CameraWindows::wait_for_quit()
{
    thread.join();
}

#endif
