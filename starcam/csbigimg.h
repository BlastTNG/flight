/*$T csbigimg.h GC 1.139 01/12/05 00:11:32 */

/*

 csbigimg.h - Contains the definition of the interface to
     the SBIG Image Class

 1. This software (c)2004 Santa Barbara Instrument Group.
 2. This free software is provided as an example of how
    to communicate with SBIG cameras.  It is provided AS-IS
    without any guarantees by SBIG of suitability for a
    particular purpose and without any guarantee to be
    bug-free.  If you use it you agree to these terms and
    agree to do so at your own risk.
    3. Any distribution of this source code to include these
    terms.

*/
#ifndef _CSBIGIMG_
#define _CSBIGIMG_

#ifndef _PARDRV_
	#include "sbigudrv.h"
#endif
#include <time.h>
#include <string>
#include "csbigimgdefs.h"
#include "camconfig.h"

using namespace std;

	#if INCLUDE_FITSIO
		#include "fitsio.h"
	#endif	/* INCLUDE_FITSIO */

class CSBIGImg
{
/* */
private:
	int					m_nHeight, m_nWidth;			// image size in pixels
	int					m_nSubFrameTop, m_nSubFrameLeft;// for partial frames the top-left pixel
	unsigned short		*m_pImage;						// pointer to image data
	double				m_dCCDTemperature;				// CCD Temp at start of exposure
	double				m_dExposureTime;				// Exposure time in seconds
	double				m_dTrackExposure;				// Exposure when tracking
	double				m_dEachExposure;				// Snapshot time in seconds
	double				m_dFocalLength;					// Lens/Telescope Focal Length in inches
	double				m_dApertureArea;				// Lens/Telescope Aperture Are in Sq-Inches
	double				m_dResponseFactor;				// Magnitude Calibration Factor
	double				m_dPixelHeight, m_dPixelWidth;	// Pixel Dimensions in mm
	double				m_dEGain;						// Electronic Gain, e-/ADU
	unsigned short		m_uBackground, m_uRange;		// Display Background and Range
	unsigned short		m_uNumberExposures;				// Number of exposures co-added
	unsigned short		m_uSaturationLevel;				// Pixels at this level are saturated
	unsigned short		m_uPedestal;					// Image Pedestal
	unsigned short		m_uExposureState;				// Exposure State
	unsigned short		m_uReadoutMode;					// Camera Readout Mode use to acquire image
	unsigned short		m_uHorizontalBinning;			// Binning used in X
	unsigned short		m_uVerticalBinning;				// Binning used in Y
	string				m_cImageNote;					// Note attached to image
	string				m_cObserver;					// Observer name
	string				m_cHistory;						// Image History string of modification chars
	string				m_cFilter;						// Filter name imaged through
	string				m_cSoftware;					// Software App Name and Version
	string				m_cCameraModel;					// Model of camera used to acquire image
	int					m_isCompressed;					// clc
	MY_LOGICAL			m_bImageModified;				// True when modified and not saved
	SBIG_IMAGE_FORMAT	m_nDefaultImageFormat;			// default image format for Saves
	struct tm			m_sDecodedImageStartTime;		// Decoded time light exposure started

/* */
public:
	/* Constructors/Destructor */
	CSBIGImg(void);
	CSBIGImg(int height, int width);
	~				CSBIGImg(void);

	void			Init(SBIGImgConfigParams params=defaultImageParams.SBIGParams);
	void			DeleteImageData(void);

	/* Accessor Functions */
	int				GetHeight(void)					{ return m_nHeight; }
	int				GetWidth(void)					{ return m_nWidth; }
	unsigned short	*GetImagePointer(void)			{ return m_pImage; }
	struct tm		GetImageStartTime(void)			{ return m_sDecodedImageStartTime; }
	void			SetCCDTemperature(double temp)	{ m_dCCDTemperature = temp; }
	double			GetCCDTemperature(void)			{ return m_dCCDTemperature; }
	void			SetDefaultImageForamt(SBIG_IMAGE_FORMAT fmt) { m_nDefaultImageFormat = fmt; }
	SBIG_IMAGE_FORMAT GetDefaultImageFormat(void)	{ return m_nDefaultImageFormat; }
	void			SetExposureTime(double exp)		{ m_dExposureTime = exp; }
	double			GetExposureTime(void)			{ return m_dExposureTime; }
	void			SetEachExposure(double exp)		{ m_dEachExposure = exp; }
	double			GetEachExposure(void)			{ return m_dEachExposure; }
	void			SetFocalLength(double fl)		{ m_dFocalLength = fl; }
	double			GetFocalLength(void)			{ return m_dFocalLength; }
	void			SetApertureArea(double ap)		{ m_dApertureArea = ap; }
	double			GetApertureArea(void)			{ return m_dApertureArea; }
	void			SetResponseFactor(double resp)	{ m_dResponseFactor = resp; }
	double			GetResponseFactor(void)			{ return m_dResponseFactor; }
	void			SetPixelHeight(double ht)		{ m_dPixelHeight = ht; }
	double			GetPixelHeight(void)			{ return m_dPixelHeight; }
	void			SetPixelWidth(double wd)		{ m_dPixelWidth = wd; }
	double			GetPixelWidth(void)				{ return m_dPixelWidth; }
	void			SetEGain(double gn)				{ m_dEGain = gn; }
	double			GetEGain(void)					{ return m_dEGain; }
	void			SetBackground(unsigned short back)	{ m_uBackground = back; }
	unsigned short	GetBackground(void)				{ return m_uBackground; }
	void			SetRange(unsigned short range)	{ m_uRange = range; }
	unsigned short	GetRange(void)					{ return m_uRange; }
	void			SetSaturationLevel(unsigned short sat)	{ m_uSaturationLevel = sat; }
	unsigned short	GetSaturationLevel(void)		{ return m_uSaturationLevel; }
	void			SetNumberExposures(unsigned short no)	{ m_uNumberExposures = no; }
	unsigned short	GetNumberExposures(void)		{ return m_uNumberExposures; }
	void			SetTrackExposure(double exp)	{ m_dTrackExposure = exp; }
	double			GetTrackExposure(void)			{ return m_dTrackExposure; }
	void			SetReadoutMode(unsigned short rm)	{ m_uReadoutMode = rm; }
	unsigned short	GetReadoutMode(void)			{ return m_uReadoutMode; }
	void			SetPedestal(unsigned short ped)		{ m_uPedestal = ped; }
	unsigned short	GetPedestal(void)				{ return m_uPedestal; }
	void			SetExposureState(unsigned short es)		{ m_uExposureState = es; }
	unsigned short	GetExposureState(void)			{ return m_uExposureState; }
	void			SetImageNote(string str)		{ m_cImageNote = str; }
	string			GetImageNote(void)				{ return m_cImageNote; }
	void			SetObserver(string str)			{ m_cObserver = str; }
	string			GetObserver(void)				{ return m_cObserver; }
	void			SetHistory(string str)			{ m_cHistory = str; }
	string			GetHistory(void)				{ return m_cHistory; }
	void			SetFilter(string str)			{ m_cFilter = str; }
	string			GetFilter(void)					{ return m_cFilter; }
	void			SetSoftware(string str)			{ m_cSoftware = str; }
	string			GetSoftware(void)				{ return m_cSoftware; }
	void			SetCameraModel(string str)		{ m_cCameraModel = str; }
	string			GetCameraModel(void)			{ return m_cCameraModel; }
	MY_LOGICAL		GetImageModified(void)			{ return m_bImageModified; }
	void			SetImageModified(MY_LOGICAL isMod)	{ m_bImageModified = isMod; }

	/* More Accessor Functions */
	void			SetImageStartTime(void);
	void			SetImageStartTime(time_t startTime);
	void			SetImageStartTime(int mon, int dd, int yy, int hr, int min, int sec);
	void			SetImageStartTime(struct tm *pStartTime);
	void			SetSubFrame(int nLeft, int nTop);
	void			GetSubFrame(int &nLeft, int &nTop);
	void			SetBinning(unsigned short nHoriz, unsigned short nVert);
	void			GetBinning(unsigned short &nHoriz, unsigned short &nVert);
	void			AddHistory(string str);

	/* File IO Routines */
	SBIG_FILE_ERROR SaveImage(const char *pFullPath, SBIG_IMAGE_FORMAT fmt = SBIF_DEFAULT);
	SBIG_FILE_ERROR OpenImage(const char *pFullPath);

	/* Utility Functions */
	MY_LOGICAL		AllocateImageBuffer(int height, int width);
	void			CreateSBIGHeader(char *pHeader, MY_LOGICAL isCompressed);
	MY_LOGICAL		ParseHeader(char *pHeader, MY_LOGICAL &isCompressed);
	SBIG_FILE_ERROR SaveCompressedImage(const char *pFullPath, char *pHeader);
	SBIG_FILE_ERROR ReadCompressedImage(FILE *fh);
	SBIG_FILE_ERROR SaveUncompressedImage(const char *pFullPath, char *pHeader);
	SBIG_FILE_ERROR ReadUncompressedImage(FILE *fh);
	int				CompressSBIGData(unsigned char *pCmpData, int imgRow);
	void			IntelCopyBytes(unsigned char *pRevData, int imgRow);
	void			AutoBackgroundAndRange(void);

	/* Image Processing Funcions */
	void			VerticalFlip(void);
	void			HorizontalFlip(void);

#if INCLUDE_FITSIO
private:
	string			m_cFITSObject;						// Name of object in FITS header
	string			m_cFITSTelescope;					// Name of telescope in FITS header
	double			m_dApertureDiameter;				// Diameter of telescope

public:
	/* FITS Accessor functions */
	void			SetFITSObject(string str)		{ m_cFITSObject = str; }
	string			GetFITSObject(void)				{ return m_cFITSObject;}
	void			SetFITSTelescope(string str)	{ m_cFITSTelescope = str; }
	string			GetFITSTelescope(void)			{ return m_cFITSTelescope;}
	void			SetApertureDiameter(double ap)	{ m_dApertureDiameter = ap; }
	double			GetApertureDiameter(void)		{ return m_dApertureDiameter; }
	
	/* FITS file format utilities */
	SBIG_FILE_ERROR SaveFITS(const char *pFullPath);
    SBIG_FILE_ERROR History2FITS(fitsfile *fptr);
#endif
};

#endif /* #ifndef _CSBIGIMG_ */
