//
// C++ Interface: mycam
//
// Description: expasion upon the CSBIGCam class to add more USB support
//
// Author: Steve Benton <steve.benton@utoronto.ca>, (C) 2006
//
#ifndef STDMYCAM_H
#define STDMYCAM_H

#include <string>
#include "csbigcam.h"
#include "sbigudrv.h"
#include "clensadapter.h"
#include "blobimage.h"
#include "camconfig.h"

using namespace std;

/**
	@author Steve Benton <steve.benton@utoronto.ca>
*/

class MyCam : public CSBIGCam
{
private:
	int m_nUSBNum;             //the number of the USB port: one of 0, 1, 2, 3
	CLensAdapter m_cAdapter;   //the adapter for the lens used on the camera
	int m_nPictureInterval;    //number of milliseconds between pictures, 0 for triggered
	unsigned int m_nFocusResolution; //autofocus will step by (totalFocalRange)/m_nFocusResolution
	unsigned int m_nFocusRange; //autofocus will step through (totalFocalRange)/m_nFocusRange
	unsigned long int m_iFrame;//frame number, increments each grabImage

	
public:
	//constructors/destructor
    MyCam();
	MyCam(OpenDeviceParams odp);
	MyCam(SBIG_DEVICE_TYPE dev);
    ~MyCam();
	
	//overwritten functions from CSBIGCam
	void Init(MyCamConfigParams params=defaultCameraParams);
	PAR_ERROR GrabImage(BlobImage *img, SBIG_DARK_FRAME dark);
	PAR_ERROR StartExposure(SHUTTER_COMMAND shutterState);
	PAR_ERROR EndExposure(void);
	
	//USB utility functions
	PAR_ERROR QueryUSB(QueryUSBResults &qur);
	PAR_ERROR OpenUSBDevice(int num);
	
	//lens adapter functions
	//focus images go to viewer:
	//LENS_ERROR autoFocus(BlobImage *img, int forced = 0, const char* path = NULL);
	//focus images get saved:
	LENS_ERROR autoFocus(BlobImage *img, int forced = 0, string path = "/data/rawdir");
	CLensAdapter* getLensAdapter() { return &m_cAdapter; }
	
	//accesors
	void setPictureInterval(int in_interval) { m_nPictureInterval = (in_interval >= 0)?in_interval:0; }
	int getPictureInterval(void) { return m_nPictureInterval; }
	void setFocusResolution(unsigned int res) { m_nFocusResolution = res; }
	unsigned int getFocusResolution(void) { return m_nFocusResolution; }
	void setFocusRange(unsigned int rng) { m_nFocusRange = rng; }
	unsigned int getFocusRange(void) { return m_nFocusRange; }

	string getSerialNum();

};

#endif
