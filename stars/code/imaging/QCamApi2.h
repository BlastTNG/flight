/*
|==============================================================================
| Copyright (C) 2002 Quantitative Imaging Corporation.  All Rights Reserved.
| Reproduction or disclosure of this file or its contents without the prior
| written consent of Quantitative Imaging Corporation is prohibited.
|==============================================================================
|
| File:			QCamApi.h
|
| Project/lib:	QCam Driver
|
| Target:		Mac OS 9, Mac OS X, Win32
|
| Description:	
|
| Notes:		This interface is not reentrant.  See "QCam API.pdf" for
|				details.
|
|==============================================================================
| dd/mon/yy  Author		Notes
|------------------------------------------------------------------------------
| 30/Aug/00  DavidL		Original.
| 31/Jan/01  DavidL		Version 1.00.
| 01/Feb/02  DavidL		Driver release 1.50.  We will try to keep this up to
|						date from now on.
| 14/Feb/02  DavidL		Add qprmSyncb, qinfFirmwareBuild, qinfUniqueId, and
|						QCam_GetSerialString().
| 02/Apr/02  DavidL		Change constant qcCcdICX252AK to qcCcdICX252AQ.  Sorry
|						if you were using that!  Just change the constants in
|						your code to qcCcdICX252AQ and you're done.
| 13/May/02  DavidL		Add qinfIsModelB, QCam_IsParamSupported().  Add error
|						checking for bad uPub ROI, fix corrupt bayerPattern
|						field in QCam_Frame after QueueSettings().  When a
|						camera is unplugged you now get error qerrUnplugged.
| 13/Jun/02  DavidL		Add qcCallbackExposeDone.
| 18/Sep/02  JonS		Added QCam_Param64 enum. type with types 
|						qprm64Exposure, qprm64ExposureRed, and 
|						qprm64ExposureBlue.
|						Added new QCam_GetParam64(), QCam_SetParam64(), 
|						QCam_GetParam64Max(), QCam_GetParam64Min() and
|						QCam_IsParam64Supported() functions.
|						Added new QCam_qcCameraType: qcCameraRetIT.
|						Also added new enum. type QCam_IntensifierModel with types
|						qcITVSStdGenIIIA and qcITVSEBGenIIIA.
|						Added new QCam_Info types qinfIntensifierModel,
|						qinfExposureRes, and qinfTriggerDelayRes.
|						Added new QCam_Param types qprmIntensifierGain and
|						qprmTriggerDelay.
| 28/Feb/03  BradR		Added Sparse Tabling support (32 and 64 bit) along with
|						QCam_IsRangeParam and QCam_IsSparseParam (32 and 64 bit).
| 25/Jul/03  ChrisS		Added mode support.  Currently this is only used with
|						MicroPublisher cameras.
| 05/Sep/03  ChrisS     Added sparse table and range table support for signed 32
|						bit integer parameters.
|						Added Normalized Gain support.  This includes parameters
|						qprmNormalizedGain, qprmS32NormalizedGaindB, and info
|						keys qinfNormGainSigFigs and qinfNormGaindBRes.
|						Added Absolute Offset support.  This includes parameter
|						qprmS32AbsoluteOffset.
| 03/Oct/03  ChrisS		Added Normalized Intensifier Gain support.  This includes
|						qprmNormIntensGaindB and qprm64NormIntensGain.  Info
|						items added for ITGain are qinfNormITGainSigFigs and
|						qinfNormITGaindBRes.
|						Increased the size of the QCam_Settings structure to 64.
| 09/Dec/03  ChrisS		Added QCam_LibVersion to replace QCam_Version.
|						QCam_LibVersion returns an additional build revision.
|==============================================================================
*/

#ifndef QCAMAPI_H_INCLUDE
#define QCAMAPI_H_INCLUDE

#ifndef QCAMAPI
	#ifdef _WIN32
		#define QCAMAPI __stdcall
		#define UNSIGNED64 unsigned __int64
	#else
		#define QCAMAPI
		#define UNSIGNED64 unsigned long long
	#endif
#endif // QCAMAPI

#ifdef __cplusplus
extern "C" {
#endif

//===== INCLUDE FILES =========================================================

//===== #DEFINES ==============================================================

//===== TYPE DEFINITIONS ======================================================

typedef enum
{
	// Model-A cameras (old camera hardware):
	qcCameraUnknown			= 0,
	qcCameraMi2				= 1,		// MicroImager II and Retiga 1300
	qcCameraPmi				= 2,
	qcCameraRet1350			= 3,		// Retiga EX
	qcCameraQICam			= 4,

	// Model-B cameras (new camera hardware):
	qcCameraRet1300B		= 5,
	qcCameraRet1350B		= 6,
	qcCameraQICamB			= 7,
	qcCameraMicroPub		= 8,		// First versions of MicroPub were model A,
										// but both A and B return the same type.
	qcCameraRetIT		 	= 9,		// Intensified Retiga
	qcCameraQICamIR			= 10,

	// These names are for engineering and OEM use:
	qcCameraX				= 1000,
	qcCameraOem1			= 1001,
	qcCameraOem2			= 1002

}
QCam_qcCameraType;

typedef enum
{
	qcCcdMonochrome		= 0,
	qcCcdColorBayer		= 1,
	qctype_last			= 2
}
QCam_qcCcdType;

typedef enum
{
	qcCcdKAF1400		= 0,
	qcCcdKAF1600		= 1,
	qcCcdKAF1600L		= 2,
	qcCcdKAF4200		= 3,
	qcCcdICX085AL		= 4,
	qcCcdICX085AK		= 5,
	qcCcdICX285AL		= 6,
	qcCcdICX285AK		= 7,
	qcCcdICX205AL		= 8,
	qcCcdICX205AK		= 9,
	qcCcdICX252AQ		= 10,
	qcCcdS70311006		= 11,
	qcCcdICX282AQ		= 12,
	qcCcdICX407AL		= 13,
	qcCcdS70310908		= 14,
	qcCcdVQE3618L		= 15,
	qcCcdX				= 255		// Don't use this value
}
QCam_qcCcd;

typedef enum
{
	qcItVsStdGenIIIA	= 0,
	qcItVsEbGenIIIA		= 1
}
QCam_qcIntensifierModel;

typedef enum
{
	qcBayerRGGB			= 0,
	qcBayerGRBG			= 1,
	qcBayerGBRG			= 2,
	qcBayerBGGR			= 3
}
QCam_qcBayerPattern;

typedef enum
{
	qcTriggerNone		= 0,		// Depreciated

	// Freerun mode: expose images as fast as possible
	qcTriggerFreerun	= 0,

	// Hardware trigger modes (software trigger also works):
	qcTriggerEdgeHi		= 1,		// Edge triggers exposure start
	qcTriggerEdgeLow	= 2,
	qcTriggerPulseHi	= 3,		// Integrate over pulse
	qcTriggerPulseLow	= 4,

	// Software trigger (software only, no hardware):
	qcTriggerSoftware	= 5,

	qcTrigger_last		= 6
}
QCam_qcTriggerType;

typedef enum
{
	qcWheelRed			= 0,
	qcWheelGreen		= 1,
	qcWheelBlack		= 2,
	qcWheelBlue			= 3,
	qcWheel_last		= 4
}
QCam_qcWheelColor;

typedef enum
{
	qcReadout20M		= 0,
	qcReadout10M		= 1,
	qcReadout5M			= 2,
	qcReadout2M5		= 3,
	qcReadout_last		= 4
}
QCam_qcReadoutSpeed;

typedef enum
{
	qcShutterAuto		= 0,
	qcShutterClose		= 1,
	qcShutterOpen		= 2,
	qcShutter_last		= 3
}
QCam_qcShutterControl;

typedef enum
{
	qcSyncbTrigmask		= 0,
	qcSyncbExpose		= 1,
	qcSyncbOem1			= 0,
	qcSyncbOem2			= 1
}
QCam_qcSyncb;

typedef enum
{
	qcCallbackDone			= 1,	// Callback when QueueFrame (or QueueSettings) is done
	qcCallbackExposeDone	= 2		// Callback when exposure done (readout starts);
									//    model-B and all MicroPublishers only;
									//    callback is not guaranteed to occur
}
QCam_qcCallbackFlags;

typedef enum
{
	qmdStandard				= 0,	// Default camera mode, legacy mode of operation
	qmdRealTimeViewing		= 1,	// Real Time Viewing mode, MicroPublisher model only
	qmd_last				= 2,
	_qmd_force32			= 0xFFFFFFFF
}
QCam_Mode;

typedef enum
{
	qerrSuccess				= 0,
	qerrNotSupported		= 1,	// Function is not supported for this device
	qerrInvalidValue		= 2,	// Invalid parameter value
	qerrBadSettings			= 3,	// Bad QCam_Settings struct
	qerrNoUserDriver		= 4,
	qerrNoFirewireDriver	= 5,	// No firewire device driver installed
	qerrDriverConnection	= 6,
	qerrDriverAlreadyLoaded	= 7,	// Too many calls to QCam_LoadDriver()
	qerrDriverNotLoaded		= 8,	// Did not call QCam_LoadDriver()
	qerrInvalidHandle		= 9,	// Invalid QCam_Handle
	qerrUnknownCamera		= 10,	// Camera type is unknown to this version of QCam
	qerrInvalidCameraId		= 11,	// Invalid camera id used in QCam_OpenCamera
	qerrNoMoreConnections	= 12,	// Obsolete...
	qerrHardwareFault		= 13,
	qerrFirewireFault		= 14,
	qerrCameraFault			= 15,
	qerrDriverFault			= 16,
	qerrInvalidFrameIndex	= 17,
	qerrBufferTooSmall		= 18,	// Frame buffer (pBuffer) is too small for image
	qerrOutOfMemory			= 19,
	qerrOutOfSharedMemory	= 20,
	qerrBusy				= 21,
	qerrQueueFull			= 22,	// Cannot queue more items, queue is full
	qerrCancelled			= 23,
	qerrNotStreaming		= 24,	// Streaming must be on before calling this command
	qerrLostSync			= 25,	// This frame is trash, frame sync was lost
	qerrBlackFill			= 26,	// This frame is damanged, some data is missing
	qerrFirewireOverflow	= 27,	// Firewire overflow - restart streaming
	qerrUnplugged			= 28,	// Camera has been unplugged or turned off
	qerrAccessDenied		= 29,	// The camera is already open
	qerrStreamFault			= 30,	// Stream Allocation Failed.  Is there enough Bandwidth
	_qerr_force32			= 0xFFFFFFFF
}
QCam_Err;

//
// For 16-bit data: the data is in words, least-significant-bit aligned.
//
// When reading RGB formats, read the name as bytes...  For example, Xrgb32
// means that byte[ 0 ] is not used, byte[ 1 ] is red, byte[ 2 ] is green,
// byte[ 3 ] is blue.  On a mac, this would mean that the least significant
// byte of a 32-bit word is RED.
//
// For Mi2/Retiga:
//		- color CCD requires bayerX (or rawX) format in 1x1
//		- color CCD requires monoX (or rawX) format in binning other than 1x1
//		- if you choose color formats on monochrome CCD you will get a 3-shot
//			rgb filter image; color CCD cameras do not have rgb filter capability
//
typedef enum
{
	qfmtRaw8				= 0,	// Raw CCD output
	qfmtRaw16				= 1,	// Raw CCD output
	qfmtMono8				= 2,	// Data is bytes
	qfmtMono16				= 3,	// Data is shorts, LSB aligned
	qfmtBayer8				= 4,	// Bayer mosaic; data is bytes
	qfmtBayer16				= 5,	// Bayer mosaic; data is shorts, LSB aligned
	qfmtRgbPlane8			= 6,	// Separate color planes
	qfmtRgbPlane16			= 7,	// Separate color planes
	qfmtBgr24				= 8,	// Common Windows format
	qfmtXrgb32				= 9,	// Format of Mac pixelmap
	qfmtRgb48				= 10,
	qfmtBgrx32				= 11,	// Common Windows format
	qfmt_last				= 12
}
QCam_ImageFormat;

//
// Deprecated parameters:
// Gain		- Cameras produced after Mar 1, 2004 will no longer support the
//			  parameter "qprmGain".  The normalized gain parameter
//			  "qprmNormalizedGain" must be used in its place.
//			  Consult API documentation for more details.
// Offset   - Cameras produced after Mar 1, 2004 will no longer support the
//			  parameter "qprmOffset".  The absolute offset parameter
//			  "qprmS32AbsoluteOffset" must be used in its place.
//			  Consult API documentation for more details.

typedef enum
{
	qprmGain				= 0,	// Camera gain (gain on CCD output)
	qprmOffset				= 1,	// Camera offset (offset in CCD ADC)
	qprmExposure			= 2,	// Exposure in microseconds
	qprmBinning				= 3,	// Binning, for cameras with square binning
	qprmHorizontalBinning	= 4,	// Binning, if camera has separate horiz value
	qprmVerticalBinning		= 5,	// Binning, if camera has separate vert value
	qprmReadoutSpeed		= 6,	// See readout speed constants
	qprmTriggerType			= 7,	// See trigger constants
	qprmColorWheel			= 8,	// Manual control of wheel color
	qprmCoolerActive		= 9,	// 1 turns on cooler, 0 turns off
	qprmExposureRed			= 10,	// For LCD filter mode: exposure (ms) of red shot
	qprmExposureBlue		= 11,	// For LCD filter mode: exposure (ms) of green shot
	qprmImageFormat			= 12,	// See QCam_ImageFormat
	qprmRoiX				= 13,	// Upper left X of ROI
	qprmRoiY				= 14,	// Upper left Y of ROI
	qprmRoiWidth			= 15,	// Width of ROI, in pixels
	qprmRoiHeight			= 16,	// Height of ROI, in pixels
	qprmReserved1			= 17,
	qprmShutterState		= 18,	// Shutter position
	qprmReserved2			= 19,
	qprmSyncb				= 20,	// SyncB output on some model-B cameras
	qprmReserved3			= 21,
	qprmIntensifierGain		= 22,	// Gain value for the intensifier (Intensified cameras only)
	qprmTriggerDelay		= 23,	// Trigger delay in nanoseconds.
	qprmCameraMode			= 24,	// Camera mode
	qprmNormalizedGain		= 25,	// Normalized camera gain (micro units)
	qprmNormIntensGaindB	= 26,	// Normalized intensifier gain dB (micro units)
 	qprm_last				= 27,
	_qprm_force32			= 0xFFFFFFFF
}
QCam_Param;

typedef enum
{
	qprmS32NormalizedGaindB = 0,	// Normalized camera gain in dB (micro units)
	qprmS32AbsoluteOffset	= 1,	// Absolute camera offset (offset in CCD ADC)
	qprmS32_last			= 2,
	_qprmS32_force32		= 0xFFFFFFFF
}
QCam_ParamS32;

typedef enum
{
	qprm64Exposure		= 0,		// Exposure in nanoseconds
	qprm64ExposureRed	= 1,		// For LCD filter mode: exposure (ns) of red shot
	qprm64ExposureBlue	= 2,		// For LCD filter mode: exposure (ns) of green shot
	qprm64NormIntensGain= 3,		// Normalized intensifier gain (micro units)
 	qprm64_last			= 4,
	_qprm64_force32		= 0xFFFFFFFF
}
QCam_Param64;

typedef enum
{
	qinfCameraType			= 0,
	qinfSerialNumber		= 1,	// Only for model-A (not MicroPublishers);
									//   otherwise returns 0
	qinfHardwareVersion		= 2,
	qinfFirmwareVersion		= 3,
	qinfCcd					= 4,
	qinfBitDepth			= 5,	// Maximum number of bits
	qinfCooled				= 6,	// 1 if camera has cooler
	qinfReserved1			= 7,	// Factory test values
	qinfImageWidth			= 8,	// Width of ROI, in pixels
	qinfImageHeight			= 9,	// Height of ROI, in pixels
	qinfImageSize			= 10,	// Size of image, in bytes
	qinfCcdType				= 11,	// Monochrome, bayer, etc.
	qinfCcdWidth			= 12,	// Maximum width
	qinfCcdHeight			= 13,	// Maximum height
	qinfFirmwareBuild		= 14,
	qinfUniqueId			= 15,	// Same as uniqueId in QCam_CamListItem
	qinfIsModelB			= 16,	// 1 for model-B functionality, 0 otherwise
	qinfIntensifierModel	= 17,	// Intensifier tube model, see QCam_qcIntensifierModel
	qinfExposureRes			= 18,	// Exposure Time resolution (in nanoseconds)
	qinfTriggerDelayRes		= 19, 	// Trigger Delay Resolution (in nanoseconds)
	qinfStreamVersion		= 20,	// Streaming Version
	qinfNormGainSigFigs		= 21,	// Normalized Gain Significant Figures resolution
	qinfNormGaindBRes		= 22,	// Normalized Gain dB resolution (micro units)
	qinfNormITGainSigFigs	= 23,	// Normalized Intensifier Gain Significant Figures
	qinfNormITGaindBRes		= 24,	// Normalized Intensifier Gain dB resolution (micro units)
	qinf_last				= 25,
	_qinf_force32			= 0xFFFFFFFF
}
QCam_Info;

typedef void* QCam_Handle;


typedef struct
{
	unsigned long size;		// You must set this to sizeof( QCam_Settings )
	unsigned long _private_data[ 64 ];
}
QCam_Settings;


typedef struct
{
	unsigned long		cameraId;
	unsigned long		cameraType;
	unsigned long		uniqueId;
	unsigned long		isOpen;				// 1 if already open

	unsigned long		_reserved[ 10 ];
}
QCam_CamListItem;


//
// For calls to QCam_GrabFrame() etc...
//
// These fields must be filled before the Api call:
//		pBuffer
//		bufferSize
//
// These fields are set (by QCam) during the Api call:
//		format
//		width
//		height
//		size
//		bits
//		frameNumber
//		bayerPattern
//		errorCode
//		timeStamp
//
typedef struct
{
	void*				pBuffer;			// Image buffer, 4-byte aligned
	unsigned long		bufferSize;			// Length of buffer, in bytes

	unsigned long		format;				// Format of image
	unsigned long		width;				// Image width, in pixels
	unsigned long		height;				// Image height, in pixels
	unsigned long		size;				// Size of image data, in bytes
	unsigned short		bits;				// Bit depth
	unsigned short		frameNumber;		// Rolling frame number
	unsigned long		bayerPattern;		// For bayer CCDs: mosaic pattern
	unsigned long		errorCode;			// QCam_Err-type code for this frame
	unsigned long		timeStamp;			// Exposure time stamp

	unsigned long		_reserved[ 8 ];
}
QCam_Frame;


typedef void ( QCAMAPI *QCam_AsyncCallback )
(
	void*				userPtr,			// User defined
	unsigned long		userData,			// User defined
	QCam_Err			errcode,			// Error code
	unsigned long		flags				// Combination of QCam_qcCallbackFlags
);


//===== FUNCTION PROTOTYPES ===================================================

//-----	Loading and releasing the QCam driver -----

// Load the QCam driver.  Call before using any QCam Api functions.
extern QCam_Err QCAMAPI QCam_LoadDriver
(
	void
);

// Release the QCam driver.
extern void QCAMAPI QCam_ReleaseDriver
(
	void
);

// Get the version of this module (the QCam Driver).
extern QCam_Err QCAMAPI QCam_LibVersion
(
	unsigned short*		verMajor,
	unsigned short*		verMinor,
	unsigned short*		verBuild
);

//----- Listing, opening, and closing cameras -----

// List the connected cameras.
extern QCam_Err QCAMAPI QCam_ListCameras
(
	QCam_CamListItem*	pList,				// IN: ptr to array of items
	unsigned long*		pNumberInList		// IN: length of list (array)
											// OUT: number of cameras; may be
											//      greater than array length!
);

// Open a camera.
extern QCam_Err QCAMAPI QCam_OpenCamera
(
	unsigned long		cameraId,			// IN: Id of camera to open
	QCam_Handle*		pHandle				// OUT: handle to camera 
);

// Close a camera.
extern QCam_Err QCAMAPI QCam_CloseCamera
(
	QCam_Handle			handle				// IN: handle to camera
);

//----- Retrieving information from the camera -----

// Only for model-B and all MicroPublisher cameras.
extern QCam_Err QCAMAPI QCam_GetSerialString
(
	QCam_Handle			handle,				// IN: handle to camera
	char*				string,				// IN: destination of string copy
	unsigned long		size				// IN: size of string buffer
);

extern QCam_Err QCAMAPI QCam_GetInfo
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Info			infoKey,			// IN: information key
	unsigned long*		pValue				// OUT: information value
);

//----- Sending and receiving camera settings -----
//
// The camera settings (which determines the camera mode) is sent to the
// camera all at once.  The QCam_Settings structure represents all the camera
// parameters.  For example, you might read the settings structure from the
// camera, change the exposure parameter and the binning parameters, then
// write the settings structure back to the camera.
//
// The QCam_Settings struct is an opaque structure.  Access the structure
// with QCam_GetParam() and QCam_SetParam().  The QCam_Settings struct can
// be saved or restored from a file, and is forward and backward compatible
// as the QCam driver evolves.
//
// A new QCam_Settings struct must be initialized by a call to
// QCam_ReadDefaultSettings() or QCam_ReadSettingsFromCam().
//
// A QCam_Settings struct is associated with the camera by which it was
// initialized.  If you have a QCam_Settings struct for one camera (say
// camera A) and want to use it for a different camera (say camera B), or
// if your QCam_Settings struct was loaded from a file, call the following
// function:
//
//		QCam_TranslateSettings()	- fix the settings (from camera A) so
//									  they work on camera B; this involves
//									  changing bad or unsupported paramters
//									  to camera B's default values
//
// After this call, the QCam_Settings struct is associated with the new
// camera.
//

// Get the camera's default settings.
extern QCam_Err QCAMAPI QCam_ReadDefaultSettings
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Settings*		pSettings			// IN: opaque settings struct
);

// Read camera settings.
extern QCam_Err QCAMAPI QCam_ReadSettingsFromCam
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Settings*		pSettings			// IN: opaque settings struct
);

// Set the camera.  Your settings struct reflects any tweaking required
// (specifically, roi parameters).
extern QCam_Err QCAMAPI QCam_SendSettingsToCam
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Settings*		pSettings			// IN: opaque settings struct
);

// Changes your settings struct just as with QCam_SendSettingsToCam(), but
// does not actually set the camera.
extern QCam_Err QCAMAPI QCam_PreflightSettings
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Settings*		pSettings			// IN: opaque settings struct
);

// Translate a settings struct so another camera can use it.  Parameters which
// are out of range or unsupported on the new camera are set to their default
// values.  After this call, you can set and get parameters from the
// translated QCam_Settings struct.
extern QCam_Err QCAMAPI QCam_TranslateSettings
(
	QCam_Handle			handle,				// IN: handle to a camera
	QCam_Settings*		pSettings			// IN: opaque settings struct
											//     initialized by another camera
);


//-----	Modifying parameters in the camera settings struct -----
//
// These functions allow access to individual parameters in the QCam_Settings
// structure.
//
// Before using these functions, the QCam_Settings struct must have been
// initialized by one of the following:
//
//		QCam_ReadDefaultSettings()
//		QCam_ReadSettingsFromCam()
//		QCam_TranslateSettings()
//
// All of the above functions require a camera handle, ie. the camera
// must first be opened.
//
// Parameters can be one of three different types
// 1. Unsigned 32 bit (QCam_Param)
// 2. Signed   32 bit (QCam_ParamS32)
// 3. Unsigned 64 bit (QCam_Param64)
//
// Range tables and sparse tables support all the above types.


// Get parameter from settings struct.
extern QCam_Err QCAMAPI QCam_GetParam
(
	QCam_Settings const* pSettings,			// IN: opaque settings struct
	QCam_Param			paramKey,			// IN: parameter key
	unsigned long*		pValue				// OUT: parameter value
);

extern QCam_Err QCAMAPI QCam_GetParamS32
(
	QCam_Settings const* pSettings,			// IN: opaque settings struct
	QCam_ParamS32		paramKey,			// IN: parameter key
	signed long*		pValue				// OUT: parameter value
);

extern QCam_Err QCAMAPI QCam_GetParam64
(
	QCam_Settings const* pSettings,			// IN: opaque settings struct
	QCam_Param64 		paramKey,			// IN: parameter key
	UNSIGNED64*			pValue				// OUT: parameter value
);


// Set a parameter in a settings struct.
extern QCam_Err QCAMAPI QCam_SetParam
(
	QCam_Settings*		pSettings,			// IN: opaque settings struct
	QCam_Param			paramKey,			// IN: parameter key
	unsigned long		value				// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_SetParamS32
(
	QCam_Settings*		pSettings,			// IN: opaque settings struct
	QCam_ParamS32		paramKey,			// IN: parameter key
	signed long			value				// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_SetParam64
(
	QCam_Settings*		pSettings,			// IN: opaque settings struct
	QCam_Param64		paramKey,			// IN: parameter key
	UNSIGNED64			value				// IN: parameter key
);

// Get the parameter minimum.
extern QCam_Err QCAMAPI QCam_GetParamMin
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param				paramKey,		// IN: parameter key
	unsigned long*			pValue			// OUT: parameter minimum
);

extern QCam_Err QCAMAPI QCam_GetParamS32Min
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_ParamS32			paramKey,		// IN: parameter key
	signed long*			pValue			// OUT: parameter minimum
);

extern QCam_Err QCAMAPI QCam_GetParam64Min
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param64			paramKey,		// IN: parameter key
	UNSIGNED64*				pValue			// OUT: parameter minimum
);

// Get the parameter maximum.
extern QCam_Err QCAMAPI QCam_GetParamMax
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param				paramKey,		// IN: parameter key
	unsigned long*			pValue			// OUT: parameter maximum
);

extern QCam_Err QCAMAPI QCam_GetParamS32Max
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_ParamS32			paramKey,		// IN: parameter key
	signed long*			pValue			// OUT: parameter maximum
);

extern QCam_Err QCAMAPI QCam_GetParam64Max
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param64			paramKey,		// IN: parameter key
	UNSIGNED64*				pValue			// OUT: parameter maximum
);

// Get the parameter sparse table
extern QCam_Err QCAMAPI QCam_GetParamSparseTable
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param				paramKey,		// IN: parameter key
	unsigned long*			pSparseTable,	// OUT: parameter sparse table
	int*					uSize			// IN: size / OUT: number of entries
);

extern QCam_Err QCAMAPI QCam_GetParamSparseTableS32
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_ParamS32			paramKey,		// IN: parameter key
	signed long*			pSparseTable,	// OUT: parameter sparse table
	int*					uSize			// IN: size / OUT: number of entries
);

extern QCam_Err QCAMAPI QCam_GetParamSparseTable64
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param64			paramKey,		// IN: parameter key
	UNSIGNED64*				pSparseTable,	// OUT: parameter sparse table
	int*					uSize			// IN: size / OUT: number of entries
);

// Returns qerrSuccess if SparseTabling supported, qerrNotSupported otherwise
extern QCam_Err QCAMAPI QCam_IsSparseTable
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param				paramKey		// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_IsSparseTableS32
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_ParamS32			paramKey		// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_IsSparseTable64
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param64			paramKey		// IN: parameter key
);

// Returns qerrSuccess if RangeTabling supported, qerrNotSupported otherwise
extern QCam_Err QCAMAPI QCam_IsRangeTable
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param				paramKey		// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_IsRangeTableS32
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_ParamS32			paramKey		// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_IsRangeTable64
(
	QCam_Settings const*	pSettings,		// IN: opaque settings struct
	QCam_Param64			paramKey		// IN: parameter key
);


// Is a parameter supported on this camera?  Returns qerrSuccess if supported,
// qerrNotSupported if not available.
extern QCam_Err QCAMAPI QCam_IsParamSupported
(
	QCam_Handle			handle,				// IN: handle to a camera
	QCam_Param			paramKey			// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_IsParamS32Supported
(
	QCam_Handle			handle,				// IN: handle to a camera
	QCam_ParamS32		paramKey			// IN: parameter key
);

extern QCam_Err QCAMAPI QCam_IsParam64Supported
(
	QCam_Handle			handle,				// IN: handle to a camera
	QCam_Param64		paramKey			// IN: parameter key
);


//-----	Frame grabbing misc -----

// Start/stop firewire streaming.  The camera's firewire port must be streaming
// continuously to transmit an image.  If you call a Grab() function without
// firewire streaming on, the QCam driver will start streaming, capture the
// image, then stop streaming.  For higher frame rates, such as preview mode,
// it is an advantage to turn on firewire streaming manually.  (The disadvantage
// of firewire streaming when you are not capturing images: the OS must process
// empty firewire packets.)
extern QCam_Err QCAMAPI QCam_SetStreaming
(
	QCam_Handle			handle,				// IN: handle to a camera
	unsigned long		enable				// IN: non-0 to enable, 0 to disable
);

// Trigger an exposure to start (software trigger).  The trigger mode must be
// set to a hardware or software-only mode.  Firewire streaming must be started
// (see QCam_SetStreaming).
//
// You can guarantee that the frame resulting from QCam_Trigger was exposed
// after this function call was entered.
//
// WARNING: Software triggering is unreliable in model-A cameras!  See SDK
// documentation.  If you need QCam_Trigger(), you should consider restricting
// your support to model-B cameras.  (Model-A MicroPublishers also do not have
// reliable software triggering.)
extern QCam_Err QCAMAPI QCam_Trigger
(
	QCam_Handle			handle				// IN: handle to a camera
);

// Stop all pending frame-grabs, and clear the queue.  You will not receive
// any more QueueFrame() and QueueSettings() callbacks after this function
// has returned.
extern QCam_Err QCAMAPI QCam_Abort
(
	QCam_Handle			handle				// IN: handle to camera
);


//----- Frame grabbing synchronous -----

extern QCam_Err QCAMAPI QCam_GrabFrame
(
	QCam_Handle			handle,				// IN: handle to a camera
	QCam_Frame*			pFrame				// IN: frame
);


//-----	Frame grabbing, camera settings - asynchronous -----

// Queue a frame buffer.  Returns immediately.  Callback occurs when the frame
// has been captured.  The frame struct and associated buffer must persist until
// the frame has been grabbed.
extern QCam_Err QCAMAPI QCam_QueueFrame
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Frame*			pFrame,				// IN: frame
	QCam_AsyncCallback	callback,			// IN: completion callback; can be NULL
	unsigned long		cbFlags,			// IN: qcCallbackFlags
	void*				userPtr,			// IN: user specified value for callback
	unsigned long		userData			// IN: user specified value for callback
);

// Queue a change in camera settings.  Returns immediately.  Callback occurs
// when the settings are changed.  Your settings structure must persist until
// the settings have been changed.
extern QCam_Err QCAMAPI QCam_QueueSettings
(
	QCam_Handle			handle,				// IN: handle to camera
	QCam_Settings*		pSettings,			// IN: settings struct
	QCam_AsyncCallback	callback,			// IN: completion callback; can be NULL
	unsigned long		cbFlags,			// IN: qcCallbackFlags
	void*				userPtr,			// IN: user specified value for callback
	unsigned long		userData			// IN: user specified value for callback
);


//----- Deprecated functions -----
//
// Please use the alternative API call "QCam_LibVersion"  "QCam_LibVersion"
// is identical to "QCam_Version" but it also returns the library build 
// version.
// Get the version of this module (the QCam Driver).
extern QCam_Err QCAMAPI QCam_Version
(
	unsigned short*		verMajor,
	unsigned short*		verMinor
);

// These functions were written when only synchronous frame-grabbing was
// available.  Since postprocessing was required in some formats, the full
// frame rate could be achieved only by post-processing on a second thread.
// If you are tempted to use these functions in a new project, please
// reconsider your design.
//

// Setup the frame buffers, used to store image data.
extern QCam_Err QCAMAPI QCam_SetFrameBuffers
(
	QCam_Handle			handle,				// IN: handle to a camera
	QCam_Frame*			pFrames,			// IN: list of frames.  Null Ok.
	unsigned long		number				// IN: number of frames in list
);

// Grab one frame.  Returns when one frame has been received and processed.
extern QCam_Err QCAMAPI QCam_GrabFrameNow
(
	QCam_Handle			handle,				// IN: handle to camera
	unsigned long		frameIndex			// IN: index of frame to use
);

// Grab a raw frame.  Returns when the raw frame has been grabbed.  You must
// call QCam_ProcessRaw() on this frame if your call to QCam_GrabRawNow()
// succeeds.
extern QCam_Err QCAMAPI QCam_GrabRawNow
(
	QCam_Handle			handle,				// IN: handle to camera
	unsigned long		frameIndex			// IN: index of frame to use
);

// Process the raw data in a frame.  You must call this function after
// QCam_GrabRawNow() otherwise your data may not be formatted properly.
extern QCam_Err QCAMAPI QCam_ProcessRaw
(
	QCam_Handle			handle,
	unsigned long		frameIndex
);



#ifdef __cplusplus
} // end extern "C"
#endif

#endif // QCAMAPI_H_INCLUDE
