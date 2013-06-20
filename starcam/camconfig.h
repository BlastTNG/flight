/* camconfig.h
   defines structures of data used to configure starcam operation
   defines global variables to contain data, and constants with default values
*/

#ifndef CAMCONFIG_H
#define CAMCONFIG_H

#include <string>
#include "clensadapterdefs.h"
#include "sbigudrv.h"
#include "csbigimgdefs.h"
#include "camstruct.h"

using namespace std;

#ifndef INVALID_HANDLE_VALUE              //used by CSBIGCam
#define INVALID_HANDLE_VALUE -1
#endif
//constant used for MyCam
#define INVALID_USB_NUM -1

typedef unsigned short int MAPTYPE;         //used in frameblob

struct LensAdapterConfigParams {
	LENS_COMMAND lastCommand;
	LENS_ERROR lastError;
	int portFD;
	string deviceName;
	int focalRange;
	unsigned int focusTol;
};

struct SBIGCamConfigParams {
	PAR_ERROR lastError;
	PAR_COMMAND lastCommand;
	short drvHandle;
	CAMERA_TYPE cameraType;
	CCD_REQUEST activeCCD;
	double exposureTime;
	unsigned short readoutMode;
	ABG_STATE7 ABGState;
	int subframeLeft, subframeTop, subframeWidth, subframeHeight;
};

struct FrameBlobConfigParams {
	MAPTYPE* map;
	unsigned int bits;
	unsigned int xpix;
	unsigned int ypix;
	double platescale;
	double gain;
	double readoutOffset;
	double readoutNoise;
	double mapmean;
	double sigma;
	MAPTYPE satval;
	unsigned int grid;
	double threshold;
	int disttol;
	unsigned int maxblobs;
	double stddev;
};

struct SBIGImgConfigParams {
	int height, width;
	unsigned short* pImg;
	double CCDTemperature;
	double exposureTime;
	double eachExposure;
	double trackExposure;
	double focalLength;
	double apertureArea;
	double responseFactor;
	double pixelHeight, pixelWidth;
	double eGain;
	unsigned short background;
	unsigned short range;
	unsigned short numberExposures;
	unsigned short saturationLevel;
	unsigned short pedestal;
	unsigned short exposureState;
	unsigned short readoutMode;
	string note;
	string observer;
	string history;
	string filter;
	string software;
	string cameraModel;
	SBIG_IMAGE_FORMAT defaultImageFormat;
	int subframeTop, subframeLeft;
	unsigned short horizontalBinning, verticalBinning;
// #if INCLUDE_FITSIO
// 	string FITSObject;
// 	string FITSTelescope;
// 	double apertureDiameter;
// #endif
};

struct PyramidConfigParams {
    double fov;
    char *catalogname;
    char *katalogname;
} ;

struct ViewerConfigParams {
	int refreshTime;
	int width, height;
};

struct MyCamConfigParams {
	LensAdapterConfigParams lensParams;
	SBIGCamConfigParams SBIGParams;
	int USBNum;
	int pictureInterval;
	unsigned int focusResolution;
	unsigned int focusRange;
};

struct BlobImageConfigParams {
	FrameBlobConfigParams blobParams;
	SBIGImgConfigParams SBIGParams;
    PyramidConfigParams pyrParams;
	ViewerConfigParams viewerParams;
	string badpixFilename;
	unsigned long timeError;
	double matchTol;
	double platescale;
};

//create global constants of default values. These are a centralized source of all defaults
static const MyCamConfigParams defaultCameraParams = {
	{                               //lens params:
		LC_NULL,                        //lastCommand
		LE_NO_ERROR,                    //lastError
		-1,                             //portFD
		"",                             //deviceName
		-1,                             //focalRange
		1                               //focusTol
	},
	{                               //SBIG camera params:
		CE_NO_ERROR,                    //lastError
		CC_NULL,                        //lastCommand
		INVALID_HANDLE_VALUE,           //drvHandle
		NO_CAMERA,                      //cameraType
		CCD_IMAGING,                    //activeCCD
		1,                              //exposureTime
		0,                              //readoutMode
		ABG_CLK_MED7,                   //ABGState
		0, 0, 0, 0                      //subframe left, top, width, height
//		100, 100, 100, 100                      //subframe left, top, width, height
	},
	INVALID_USB_NUM,                //USBNum
	0,                              //pictureInterval
	100                             //focusResolution
};

static const BlobImageConfigParams defaultImageParams = {
	{                               //frame blob params:
		NULL,                           //map
		16,                             //bits
		CAM_WIDTH,                              //xpix
		CAM_HEIGHT,                              //ypix
		6.19/60.0/60.0,                  //platescale
		1,                              //gain
		0,                              //readoutOffset
		0,                              //readoutNoise
		0,                              //mapmean
		1,                              //sigma
		65535,                          //satval
		20,                             //grid
		5.0,                            //threshold
		20*20,                          //disttol
		99,                             //maxblobs
		0                               //stddev
	},
	{                               //SBIG image params:
		0, 0,                           //height, width
		NULL,                           //pImg
		25,                             //CCDTemperature
		0.1,                            //exposureTime
		0.1,                            //eachExposure
		0,                              //trackExposure
		7.87,                           //focalLength
		24.84,                          //apertureArea
		2000.0,                         //responseFactor
		0.009, 0.009,                   //pixelHeight, pixelWidth
		1.48,                           //eGain
		0,                              //background
		65535,                          //range
		1,                              //numberExposures
		65535,                          //saturationLevel
		0,                              //pedestal
		ES_ABG_LOW | ES_ABG_RATE_FIXED | ES_DCS_ENABLED | ES_DCR_DISABLED |\
				ES_AUTOBIAS_ENABLED,    //exposureState
		0,                              //readoutMode
		"",                             //note
		"THE GOOD",               	//observer
		"0",                            //history
		"None",                         //filter
		"Starcam control",              //software
		"ST-402",                       //cameraModel
		SBIF_COMPRESSED,                //defaultImageFormat
		0, 0,                           //subframeTop, subframeLeft
		1, 1                            //horizontalBinning, verticalBinning
// #if INCLUDE_FITSIO
// 		,"",                            //FITSObject
// 		"",                             //FITSTelescope
// 		2.81                            //apertureDiameter
// #endif
	},
    {                               //pyramid params
        0.035,                          //fov
		(char*)"/home/spider/pyramid/pyr_mag8/gsc_mag08_res20.bin", //catalogname
		(char*)"/home/spider/pyramid/pyr_mag8/k.bin"                //katalogname
    },
	{                               //viewer params
		10,                          //refreshTime
		1530, 1020                       //width, height
	},
	"",                             //badpixFilename
	0,                              //timeError
	8.0e-5,                         //matchTol
	3.001e-5                        //platescale	
};

#endif   //CAMCONFIG_H
