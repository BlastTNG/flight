/*$T csbigimg.cpp GC 1.139 01/12/05 00:02:24 */

/*

	csbigimg.cpp - SBIG Image Class

	1. This software (c)2004-2005 Santa Barbara Instrument Group.
	2. This free software is provided as an example of how
	to communicate with SBIG cameras.  It is provided AS-IS
	without any guarantees by SBIG of suitability for a
	particular purpose and without any guarantee to be
	bug-free.  If you use it you agree to these terms and
	agree to do so at your own risk.
	3. Any distribution of this source code to include these
	terms.

	Revision History
	Date		Modification
	=========================================================
	1/26/04		Initial release

	1/9/05		Added OpenImage() and ParseHeader() methods
				for reading SBIG Compressed and Uncompressed
				images.
				Added missing Get/SetFilter() and
				Get/SetSoftware() functions
				Added AutoBackgroundAndRange() method

	1/10/05		Added VerticalFlip() and HorizontalFlip()
				Changed way we keep track of image start time
				by using a struct tm instead of a time_t since
				reading headers is now required

	1/11/05		Added SaveFITS() routine. Saves files in FITS
				format using the CFITSIO library. CLC.

	1/12/05		Added History2FITS() routine. Generates a FITS
				HISTORY card for each history element. CLC.

	1/14/05		Removed debug printf's from SaveFITS() routine.
				AllocateImageBuffer allocated 2x as much memory as
				needed (looks like a cut & paste error from C code).
				Fixed. CLC.

	1/17/05		Added the Get/SetSubFrame() methods for specifying
				 top-left corner of partial frames
				Added the Get/SetBinning() methods for specifying
				 the binning used to acquire the image

	2/18/05		Fixed bugs in ParseHeader() that would fail or bomb
				 on old headers

*/
#include "csbigimg.h"
#include <stdio.h>
#include <string.h>
#include <string>
/*

Local Constants

*/
static const char	*HEADER_KEYWORDS[] =
{
	"File_version = ",
	"Data_version = ",
	"Exposure = ",
	"Focal_length = ",
	"Aperture = ",
	"Response_factor = ",
	"Note = ",
	"Background = ",
	"Range = ",
	"Height = ",
	"Width = ",
	"Date = ",
	"Time = ",
	"Exposure_state = ",
	"Temperature = ",
	"Number_exposures = ",
	"Each_exposure = ",
	"History = ",
	"Observer = ",
	"X_pixel_size = ",
	"Y_pixel_size = ",
	"Pedestal = ",
	"E_gain = ",
	"User_1 = ",
	"User_2 = ",
	"User_3 = ",
	"User_4 = ",
	"Filter = ",
	"Readout_mode = ",
	"Track_time = ",
	"Sat_level = ",
	"End"
};

/*

 CSBIGImg:

 Standard constructor.  Init member variables.

*/
CSBIGImg::CSBIGImg(void)
{
	Init();
}

/*

 CSBIGImg:

 Alternate constructor.  Try to allocate the image buffer.

*/
CSBIGImg::CSBIGImg(int height, int width)
{
	Init();
	AllocateImageBuffer(height, width);
}

/*

 ~CSBIGImg:

 Deallocate the image buffer.

*/
CSBIGImg::~CSBIGImg(void)
{
	DeleteImageData();
}

/*

 Init:

 Initialize the member variables with reasonable default values.

*/
void CSBIGImg::Init(SBIGImgConfigParams params/*=defaultImageParams.SBIGParams*/)
{
	time_t	tm;

	m_nHeight = params.height;
	m_nWidth = params.width;
	m_pImage = params.pImg;
	m_dCCDTemperature = params.CCDTemperature;
	m_dExposureTime = params.exposureTime;
	m_dEachExposure = params.eachExposure;
	m_dTrackExposure = params.trackExposure;
	m_dFocalLength = params.focalLength;
	m_dApertureArea = params.apertureArea;
	m_dResponseFactor = params.responseFactor;
	m_dPixelHeight = params.pixelHeight;
	m_dPixelWidth = params.pixelWidth;
	m_dEGain = params.eGain;
	m_uBackground = params.background;
	m_uRange = params.range;
	m_uNumberExposures = params.numberExposures;
	m_uSaturationLevel = params.saturationLevel;
	m_uPedestal = params.pedestal;
	m_uExposureState = params.exposureState;
	m_uReadoutMode = params.readoutMode;
	m_cImageNote = params.note;
	m_cObserver = params.observer;
	m_cHistory = params.history;
	m_cFilter = params.filter;
	m_cSoftware = params.software;
	m_cCameraModel = params.cameraModel;
	m_nDefaultImageFormat = params.defaultImageFormat;
	m_nSubFrameTop = params.subframeTop;
	m_nSubFrameLeft = params.subframeLeft;
	m_uHorizontalBinning = params.horizontalBinning;
	m_uVerticalBinning = params.verticalBinning;
#if INCLUDE_FITSIO
	m_cFITSObject = params.FITSObject;
	m_cFITSTelescope = params.FITSTelescope;
	m_dApertureDiameter = params.apertureDiameter;
#endif
	tm = time(NULL);
	SetImageStartTime(tm);
	m_bImageModified = TRUE;
}
/* original method has been replaced
void CSBIGImg::Init(void)
{
	string	s1, s2;
	time_t	tm;

	m_nHeight = m_nWidth = 0;
	m_pImage = NULL;
	tm = time(NULL);
	SetImageStartTime(tm);
	m_dCCDTemperature = 25.0;
	m_dExposureTime = m_dEachExposure = 1.0;
	m_dTrackExposure = 0.0;
	m_dFocalLength = 80.0;
	m_dApertureArea = PI * 4.0 * 4.0;
	m_dResponseFactor = 2000.0;
	m_dPixelHeight = m_dPixelWidth = 0.009;
	m_dEGain = 2.3;
	m_uBackground = 0;
	m_uRange = 65535;
	m_uNumberExposures = 1;
	m_uSaturationLevel = 65535;
	m_uPedestal = 0;
	m_uExposureState = ES_ABG_LOW | ES_ABG_RATE_FIXED | ES_DCS_ENABLED | ES_DCR_DISABLED | ES_AUTOBIAS_ENABLED;
	m_uReadoutMode = 0;
	m_cImageNote = "";
	m_cObserver = "Image acquired with CSBIGImg";
	m_cHistory = "0";
	m_cFilter = "None";
	s1 = "CSBIGImg Ver ";
	s2 = VERSION_STR;
	m_cSoftware = s1 + s2;
	m_cCameraModel = "ST-7";
	m_bImageModified = TRUE;
	m_nDefaultImageFormat = SBIF_UNCOMPRESSED;
	m_nSubFrameTop = m_nSubFrameLeft = 0;
	m_uHorizontalBinning = m_uVerticalBinning = 1;
#if INCLUDE_FITSIO
	m_cFITSObject = "";
	m_cFITSTelescope = "";
	m_dApertureDiameter = 4.0;
#endif
}
*/

/* */
void CSBIGImg::DeleteImageData(void)
{
	if (m_pImage)
		delete[] m_pImage;
	m_pImage = NULL;
}

/*

  SetImageStartTime:

  This set of overloaded functions records the time the
  image started.  Each is defined separately below:

  SetImageStartTime(void)
  Set to the current time.

  SetImageStartTime(struct tm *pStartTime)
  SetImageStartTime(time_t startTime)
  SetImageStartTime(int mm, int dd, int yy, int hr, int min, int sec)
  Set to the passed time in it's individual format.

*/
void CSBIGImg::SetImageStartTime(void)
{
	SetImageStartTime(time(NULL));
}

void CSBIGImg::SetImageStartTime(struct tm *pStartTime)
{
	memcpy(&m_sDecodedImageStartTime, pStartTime, sizeof(struct tm));
}

void CSBIGImg::SetImageStartTime(time_t startTime)
{
	/*~~~~~~~~~~~~~*/
	struct tm	*plt;
	/*~~~~~~~~~~~~~*/

	plt = gmtime(&startTime);
	memcpy(&m_sDecodedImageStartTime, plt, sizeof(struct tm));
}

void CSBIGImg::SetImageStartTime(int mm, int dd, int yy, int hr, int min, int sec)
{
	m_sDecodedImageStartTime.tm_mon = mm - 1;		// month in struct tm is 0 - 11, not 1 - 12
	m_sDecodedImageStartTime.tm_mday = dd;
	m_sDecodedImageStartTime.tm_year = yy - 1900;	// year is zero based in 1900
	m_sDecodedImageStartTime.tm_hour = hr;
	m_sDecodedImageStartTime.tm_min = min;
	m_sDecodedImageStartTime.tm_sec = sec;
}

/*

  GetSubFrame:
  SetSubFrame:

  For partial frame images these contain the coordinate
  of the upper-left most pixel.

*/
void CSBIGImg::SetSubFrame(int nLeft, int nTop)
{
	m_nSubFrameLeft = nLeft;
	m_nSubFrameTop = nTop;
}

void CSBIGImg::GetSubFrame(int &nLeft, int &nTop)
{
	nLeft = m_nSubFrameLeft;
	nTop = m_nSubFrameTop;
}

/*

  GetBinning:
  SetBinning:

  These contain the binning used to 
  acquire the image.

*/
void CSBIGImg::SetBinning(unsigned short uHoriz, unsigned short uVert)
{
	m_uHorizontalBinning = uHoriz;
	m_uVerticalBinning = uVert;
}

void CSBIGImg::GetBinning(unsigned short &uHoriz, unsigned short &uVert)
{
	uHoriz = m_uHorizontalBinning;
	uVert = m_uVerticalBinning;
}

/*

  AddHistory:

  Add the passed char to the history string and if it's
  only the '0' char in the string remove it.

*/
void CSBIGImg::AddHistory(string str)
{
	if ( m_cHistory.length() == 1 && m_cHistory[0] == '0' )
		m_cHistory = str;
	else
		m_cHistory += str;
}

/*

 SaveImage:

 Save the image in passed path and format.
 Returns any file errors that occur.

*/
SBIG_FILE_ERROR CSBIGImg::SaveImage(const char *pFullPath, SBIG_IMAGE_FORMAT fmt /* = SBIF_DEFAULT */ )
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	char			header[HEADER_LEN];
	SBIG_FILE_ERROR res;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	if (fmt == SBIF_DEFAULT)
		fmt = m_nDefaultImageFormat;
	switch (fmt) {
		case SBIF_COMPRESSED:
			res = SaveCompressedImage(pFullPath, header);
			break;

		case SBIF_UNCOMPRESSED:
			res = SaveUncompressedImage(pFullPath, header);
			break;

		case SBIF_FITS:
#if INCLUDE_FITSIO
			/* save file in FITS format */
			res = SaveFITS(pFullPath);
#else
			return SBFE_FORMAT_ERROR;
#endif
			break;

		default:
			res = SBFE_FORMAT_ERROR;
			break;
	}

	if (res == SBFE_NO_ERROR) {
		SetImageModified(FALSE);
		m_nDefaultImageFormat = fmt;
	}

	return res;
}

/*

  SaveCompressedImage:

  Create and write the image header then compress and write each
  row in the image.

*/
SBIG_FILE_ERROR CSBIGImg::SaveCompressedImage(const char *pFullPath, char *pHeader)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	SBIG_FILE_ERROR res;
	FILE			*fp;
	int				i, cmpWidth;
	unsigned char	*pCmpData;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	CreateSBIGHeader(pHeader, TRUE);
	res = SBFE_MEMORY_ERROR;
	pCmpData = new unsigned char[m_nWidth * 2 + 2];
	if (pCmpData) {
		res = SBFE_OPEN_ERROR;
		if ((fp = fopen(pFullPath, "wb")) != 0) {
			res = SBFE_WRITE_ERROR;
			if (fwrite(pHeader, HEADER_LEN, 1, fp) == 1) {
				for (i = 0; i < m_nHeight; i++) {
					cmpWidth = CompressSBIGData(pCmpData, i);
					if (fwrite(pCmpData, 1, cmpWidth, fp) != (size_t) cmpWidth)
						break;
				}

				if (i == m_nHeight)
					res = SBFE_NO_ERROR;
				fclose(fp);
			}
		}

		delete[] pCmpData;
	}
	return res;
}

/*

  SaveUncompressedImage:

  Create and write the image header then write the image
  data.  We save the image as unsigned shorts but since this
  can run on machines with different byte order make the
  file byte order comply with the Intel byte order (low
  byte first).

*/
SBIG_FILE_ERROR CSBIGImg::SaveUncompressedImage(const char *pFullPath, char *pHeader)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	FILE			*fp;
	SBIG_FILE_ERROR res;
	int				i;
	unsigned char	*pRevData;
	unsigned short	byteTest = 0x1234;
	MY_LOGICAL		reverseBytes;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	CreateSBIGHeader(pHeader, FALSE);
	res = SBFE_OPEN_ERROR;
	if ((fp = fopen(pFullPath, "wb")) != 0) {
		res = SBFE_WRITE_ERROR;
		if (fwrite(pHeader, HEADER_LEN, 1, fp) == 1) {
			reverseBytes = *((unsigned char *) &byteTest) != 0x34;
			if (reverseBytes) {
				pRevData = new unsigned char[m_nWidth * 2];
				if (pRevData) {
					for (i = 0; i < m_nHeight; i++) {
						IntelCopyBytes(pRevData, i);
						if (fwrite(pRevData, 2 * m_nWidth, 1, fp) != 1)
							break;
					}

					delete pRevData;
					if (i == m_nHeight)
						res = SBFE_NO_ERROR;
				}
				else
					res = SBFE_MEMORY_ERROR;
			}
			else {
				if (fwrite(m_pImage, 2 * m_nWidth, m_nHeight, fp) == (size_t) m_nHeight)
					res = SBFE_NO_ERROR;
			}
		}

		fclose(fp);
	}
	return res;
}

/*
    OpenImage:

    Open the image in the passed path.
    Returns any file errors that occur.

*/
SBIG_FILE_ERROR CSBIGImg::OpenImage(const char *pFullPath)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	char			header[HEADER_LEN];
	FILE			*fh;
	SBIG_FILE_ERROR res;
	MY_LOGICAL		isCompressed;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if ((fh = fopen(pFullPath, "rb")) == NULL)
		return SBFE_OPEN_ERROR;
	else {
		do {	// allow break out
			// read and pars header
			res = SBFE_FORMAT_ERROR;
			if (fread(header, 1, HEADER_LEN, fh) != HEADER_LEN)
				break;
			if (!ParseHeader(header, isCompressed))
				break;

			// allocate image buffer
			res = SBFE_MEMORY_ERROR;
			if (!AllocateImageBuffer(m_nHeight, m_nWidth))
				break;

			if (isCompressed)
				res = ReadCompressedImage(fh);
			else
				res = ReadUncompressedImage(fh);

			if (res != SBFE_NO_ERROR) {
				delete m_pImage;
				m_pImage = NULL;
			}
		} while (FALSE);
	}

	fclose(fh);
	if (res == SBFE_NO_ERROR) {
		SetImageModified(FALSE);
		m_nDefaultImageFormat = isCompressed ? SBIF_COMPRESSED : SBIF_UNCOMPRESSED;
	}

	return res;
}

/*

  ReadCompressedImage:

  Read from the file and uncompress a compressed image.

*/
SBIG_FILE_ERROR CSBIGImg::ReadCompressedImage(FILE *fh)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	SBIG_FILE_ERROR res;
	int				i, j;
	unsigned char	*pcb = NULL, *pc, code;
	unsigned short	*pVid, vid;
	unsigned short	delta;
	unsigned short	len;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	// read compressed image data
	do {					// allow break out
		// allocate a row buffer
		res = SBFE_MEMORY_ERROR;
		pcb = new unsigned char[2 * m_nWidth];
		if (pcb == NULL)
			break;

		//    fprintf(stderr, "\nCSBIGImg::OpenImage():Allocated row buffer");
		// read each row in turn
		res = SBFE_FORMAT_ERROR;
		for (i = 0; i < m_nHeight;) {

			// read the first 2 bytes which are the length of the compressed data
			pVid = m_pImage + (long)i * m_nWidth;
			if (fread(pcb, sizeof(unsigned short), 1, fh) != 1)
				break;
			len = *((unsigned short *)pcb);

			//     fprintf(stderr, "\nCSBIGImg::OpenImage():Row %d len = %u", i, len);
			// check the length for invalid or uncompressed
			if (len > 2 * m_nWidth)
				break;	// row too long for compressed data
			else if (len < m_nWidth + 1)
				break;	// row too short for compressed data
			else if (len == 2 * m_nWidth) {

				// see if an uncompressed row
				//      fprintf(stderr, "\nCSBIGImg::OpenImage():Row %d reading uncompressed data", i);
				if (fread(pVid, 2, m_nWidth, fh) != (size_t) m_nWidth)
					break;
				//      fprintf(stderr, "\nCSBIGImg::OpenImage():Row %d uncompressed data read", i);
				i++;	// goto next row
			}
			else {

				// compressed data, read the rest of the line
				//      fprintf(stderr, "\nCSBIGImg::OpenImage():Row %d reading compressed data", i);
				if (fread(pcb, 1, len, fh) != (size_t) len)
					break;
				//      fprintf(stderr, "\nCSBIGImg::OpenImage():Row %d compressed data read", i);
				pc = pcb;
				vid = *pc++;					// first pixel is 2 bytes, lsb first
				vid += (*pc++) << 8;
				len -= 2;
				*pVid++ = vid;
				for (j = 1; j < m_nWidth && len > 0; j++) {
					code = *pc++;
					len--;
					if (code == 0x80) {

						// 0x80 means pixel in next 2 bytes
						if (len < 2)
							break;				// whoops, buffer underflow, format error
						vid = *pc++;			// ls byte first
						vid += (*pc++) << 8;
						len -= 2;
					}
					else {

						// byte read is an 8 bit signed delta
						delta = code;
						if (code & 0x80)
							delta += 0xFF00;	// sign extend
						vid += delta;
					}

					*pVid++ = vid;
				}

				if (j != m_nWidth || len != 0)
					break;						// again a format error
				i++;			// goto next row
			}					// compressed row

			if (i == m_nHeight) // made it through without error
				res = SBFE_NO_ERROR;
		}						// for i=0 to height
	} while (FALSE);
	if (pcb)
		delete[] pcb;
	return res;
}

/*

  ReadUncompressedImage:

  Read an uncompressed image which is written as
  an array of unsigned shorts in Intel byte order
  (low byte first).  If after reading the data the
  byte order needs to be reversed do so.

*/
SBIG_FILE_ERROR CSBIGImg::ReadUncompressedImage(FILE *fh)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	SBIG_FILE_ERROR res;
	int				i;
	unsigned short	*pVid;
	unsigned short	byteTest = 0x1234;
	MY_LOGICAL		reverseBytes;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	// read uncompressed data which is a simple binary image
	if (fread(m_pImage, 2 * m_nWidth, m_nHeight, fh) != (size_t) m_nHeight)
		res = SBFE_FORMAT_ERROR;
	else {
		reverseBytes = *((unsigned char *) &byteTest) != 0x34;
		if (reverseBytes) {
			for (i=0; i<m_nHeight; i++) {
				pVid = m_pImage + (long)i*m_nWidth;
				IntelCopyBytes((unsigned char *)pVid, i);
			}
		}
		res = SBFE_NO_ERROR;
	}
	return res;
}

/*

 AllocateImageBuffer:

 Delete any existing buffer then try to allocate one of the
 given size.  Returns TRUE if successful.

*/
MY_LOGICAL CSBIGImg::AllocateImageBuffer(int height, int width)
{
	if (m_pImage)
		delete[] m_pImage;
	m_nHeight = m_nWidth = 0;
	if (height > 0 && width > 0) {
		m_pImage = new unsigned short[(long)height * width];    //CLC
		if (m_pImage) {
			m_nHeight = height;
			m_nWidth = width;
		}
		memset(m_pImage, 0, 2L * m_nHeight * m_nWidth);
	}
	return m_pImage != NULL;
}

/* */
void CSBIGImg::CreateSBIGHeader(char *pHeader, MY_LOGICAL isCompressed)
{
	/*~~~~~~~~~~~~~*/
	char		*p;
	struct tm	*plt;
	/*~~~~~~~~~~~~~*/

	plt = &m_sDecodedImageStartTime;
	memset(pHeader, 0, HEADER_LEN);
	p = pHeader;
	p += sprintf(p, "SBIG %sImage\n\r", isCompressed ? "Compressed " : "");
	p += sprintf(p, "%s%d\n\r", HEADER_KEYWORDS[HH_FILE_VERSION], FILE_VERSION);
	p += sprintf(p, "%s%d\n\r", HEADER_KEYWORDS[HH_DATA_VERSION], DATA_VERSION);
	p += sprintf(p,
				 "%s%ld\n\r",
				 HEADER_KEYWORDS[HH_EXPOSURE],
				 m_dExposureTime < 0.01 ? 1 : (long)(m_dExposureTime * 100.0 + 0.5));
	p += sprintf(p, "%s%1.3lf\n\r", HEADER_KEYWORDS[HH_FOCAL_LENGTH], m_dFocalLength);
	p += sprintf(p, "%s%1.4lf\n\r", HEADER_KEYWORDS[HH_APERTURE], m_dApertureArea);
	p += sprintf(p, "%s%1.3lf\n\r", HEADER_KEYWORDS[HH_RESPONSE_FACTOR], m_dResponseFactor);
	p += sprintf(p, "%s%s\n\r", HEADER_KEYWORDS[HH_NOTE], m_cImageNote.length() == 0 ? "-" : m_cImageNote.c_str());
	p += sprintf(p, "%s%u\n\r", HEADER_KEYWORDS[HH_BACKGROUND], m_uBackground);
	p += sprintf(p, "%s%u\n\r", HEADER_KEYWORDS[HH_RANGE], m_uRange);
	p += sprintf(p, "%s%d\n\r", HEADER_KEYWORDS[HH_HEIGHT], m_nHeight);
	p += sprintf(p, "%s%d\n\r", HEADER_KEYWORDS[HH_WIDTH], m_nWidth);
	p += sprintf(p,
				 "%s%02d/%02d/%02d\n\r",
				 HEADER_KEYWORDS[HH_DATE],
				 plt->tm_mon + 1,
				 plt->tm_mday,
				 plt->tm_year % 100);
	p += sprintf(p, "%s%02d:%02d:%02d\n\r", HEADER_KEYWORDS[HH_TIME], plt->tm_hour, plt->tm_min, plt->tm_sec);
	p += sprintf(p, "%s%u\n\r", HEADER_KEYWORDS[HH_EXPOSURE_STATE], m_uExposureState);
	p += sprintf(p, "%s%1.2lf\n\r", HEADER_KEYWORDS[HH_TEMPERATURE], m_dCCDTemperature);
	p += sprintf(p, "%s%d\n\r", HEADER_KEYWORDS[HH_NUMBER_EXPOSURES], m_uNumberExposures);
	p += sprintf(p,
				 "%s%ld\n\r",
				 HEADER_KEYWORDS[HH_EACH_EXPOSURE],
				 m_dEachExposure < 0.01 ? 1 : (long)(m_dEachExposure * 100.0 + 0.5));
	p += sprintf(p, "%s%s\n\r", HEADER_KEYWORDS[HH_HISTORY], m_cHistory.c_str());
	p += sprintf(p, "%s%s\n\r", HEADER_KEYWORDS[HH_OBSERVER], m_cObserver.length() == 0 ? "-" : m_cObserver.c_str());
	p += sprintf(p, "%s%1.4lf\n\r", HEADER_KEYWORDS[HH_X_PIXEL_SIZE], m_dPixelWidth);
	p += sprintf(p, "%s%1.4lf\n\r", HEADER_KEYWORDS[HH_Y_PIXEL_SIZE], m_dPixelHeight);
	p += sprintf(p, "%s%u\n\r", HEADER_KEYWORDS[HH_PEDESTAL], m_uPedestal);
	p += sprintf(p, "%s%1.2lf\n\r", HEADER_KEYWORDS[HH_E_GAIN], m_dEGain);

	/* create user parameters */
	p += sprintf(p, "%s%s\n\r", HEADER_KEYWORDS[HH_USER_1], m_cSoftware.length() == 0 ? "-" : m_cSoftware.c_str());
	p += sprintf(p,
				 "%s%s\n\r",
				 HEADER_KEYWORDS[HH_USER_2],
				 m_cCameraModel.length() == 0 ? "-" : m_cCameraModel.c_str());
	p += sprintf(p,
				 "%sExposure = %1.3lf, Each_exposure = %1.3lf\n\r",
				 HEADER_KEYWORDS[HH_USER_3],
				 m_dExposureTime,
				 m_dEachExposure);
	p += sprintf(p, "%s%s%d\n\r",HEADER_KEYWORDS[HH_USER_4],  "Y2KYear = ", plt->tm_year + 1900);

	/* create filter string */
	p += sprintf(p, "%s%s\n\r", HEADER_KEYWORDS[HH_FILTER], m_cFilter.length() == 0 ? "-" : m_cFilter.c_str());

	/* create readout mode */
	p += sprintf(p, "%s%u\n\r", HEADER_KEYWORDS[HH_READOUT_MODE], m_uReadoutMode);

	/* create track time */
	p += sprintf(p,
				 "%s%ld\n\r",
				 HEADER_KEYWORDS[HH_TRACK_TIME],
				 m_dTrackExposure < 0.01 ? 0 : (long)(m_dTrackExposure * 100.0 + 0.5));

	/* create saturation level */
	p += sprintf(p, "%s%u\n\r", HEADER_KEYWORDS[HH_SAT_LEVEL], m_uSaturationLevel);
	p += sprintf(p, "%s\n\r%c", HEADER_KEYWORDS[HH_END], 0x1a);
}

/* */
MY_LOGICAL CSBIGImg::ParseHeader(char *pHeader, MY_LOGICAL &isCompressed)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	char			*p, *p1;
	char const		*ph;
	MY_LOGICAL		res = FALSE;
	int				i, mm, dd, yy, hh, ss;
	double			d, d2;
	char			s[80], model[80];
	long			l;
	unsigned int	u;
	struct tm		tm;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	p = pHeader;
	ph = "Start of Header";
	do
	{	// allow break to exit this loop
		// get the first line in the header
		if (sscanf(p, "%79[^\n]", s) < 1)
			break;

		//  fprintf(stderr,"\nCSBIGImg::ParseHeader():Got first line:%s", s);
		// First word is either the Camera Model or "SBIG"
		// detect the optional camera model starting the first line
		SetCameraModel("ST-7"); // init to default
		if (sscanf(s, "%s", model) < 1)
			break;

		//  fprintf(stderr,"\nCSBIGImg::ParseHeader():Got model:%s", model);
		if (strcmp(model, "SBIG") != 0)
			SetCameraModel(model);

		// If the image is compressed the next word will be "Compressed"
		isCompressed = FALSE;
		if (strcmp(s + strlen(model) + 1, "Compressed Image") == 0)
			isCompressed = TRUE;
		else if (strcmp(s + strlen(model) + 1, "Image") != 0)
			break;

		//  fprintf(stderr,"\nCSBIGImg::ParseHeader():Got compressed:%s", isCompressed ? "Yes" : "No");
		// Get File Version
		ph = HEADER_KEYWORDS[HH_FILE_VERSION];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%d", &i) != 1 || i<0 || i > FILE_VERSION)
			break;

		// Get Data Version
		ph = HEADER_KEYWORDS[HH_DATA_VERSION];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%d", &i) != 1 || i != DATA_VERSION)
			break;

		// Get Exposure
		ph = HEADER_KEYWORDS[HH_EXPOSURE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%ld", &l) != 1)
			break;
		if (l < 1)
			l = 1;
		SetExposureTime((double)l / 100.0);

		// Get Focal Length
		ph = HEADER_KEYWORDS[HH_FOCAL_LENGTH];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		if (d < 0.001)
			d = 0.001;
		SetFocalLength(d);

		// Get Aperture
		ph = HEADER_KEYWORDS[HH_APERTURE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		if (d < 0.001)
			d = 0.001;
		SetApertureArea(d);

		// Get Response Factor
		ph = HEADER_KEYWORDS[HH_RESPONSE_FACTOR];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		if (d < 0.001)
			d = 0.001;
		SetResponseFactor(d);

		// Get Note
		ph = HEADER_KEYWORDS[HH_NOTE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%70[^\n]", s) != 1)
			break;
		if (strlen(s) == 1 && s[0] == '-')
			s[0] = 0;
		SetImageNote(s);

		// Get Background
		ph = HEADER_KEYWORDS[HH_BACKGROUND];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%ld", &l) != 1)
			break;
		SetBackground(l < 0 ? 0 : l > 65535 ? 65535 : l);

		// Get Range
		ph = HEADER_KEYWORDS[HH_RANGE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%ld", &l) != 1)
			break;
		SetRange(l < 1 ? 1 : l > 65535 ? 65535 : l);

		// Get Height
		ph = HEADER_KEYWORDS[HH_HEIGHT];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%d", &m_nHeight) != 1)
			break;

		// Get Width
		ph = HEADER_KEYWORDS[HH_WIDTH];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%d", &m_nWidth) != 1)
			break;

		// Get Date
		ph = HEADER_KEYWORDS[HH_DATE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%[^\n]", s) != 1)
			break;
		if (sscanf(s, "%d%*c%d%*c%d", &mm, &dd, &yy) != 3)
			break;
		tm.tm_mon = mm - 1;
		tm.tm_mday = dd;
		tm.tm_year = (yy <= 85 ? yy + 100 : (yy < 100 ? yy : yy - 1900));

		//  fprintf(stderr,"\nCSBIGImg::ParseHeader():Date = %s", s);
		// Get Time
		ph = HEADER_KEYWORDS[HH_TIME];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%[^\n]", s) != 1)
			break;
		if (sscanf(s, "%d:%d:%d", &hh, &mm, &ss) != 3)
			break;
		tm.tm_hour = hh;
		tm.tm_min = mm;
		tm.tm_sec = ss;
		SetImageStartTime(&tm);

		//		fprintf(stderr,"\nCSBIGImg::ParseHeader():Time = %s", s);
		// Get Exposure State
		ph = HEADER_KEYWORDS[HH_EXPOSURE_STATE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%u", &u) != 1)
			break;
		SetExposureState(u);

		// Get Temperature
		ph = HEADER_KEYWORDS[HH_TEMPERATURE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		SetCCDTemperature(d);

		// Get Number Exposures
		ph = HEADER_KEYWORDS[HH_NUMBER_EXPOSURES];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%ld", &l) != 1)
			break;
		SetNumberExposures((l < 1 ? 1 : (l > 65535 ? 65535 : l)));

		// Get Each Exposure
		ph = HEADER_KEYWORDS[HH_EACH_EXPOSURE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%ld", &l) != 1)
			break;
		l = (l < 1 ? 1 : l);
		SetEachExposure((double)l / 100.0);

		// Get History
		ph = HEADER_KEYWORDS[HH_HISTORY];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%68[^\n]", s) != 1)
			break;
		if (strlen(s) == 1 && s[0] == '-')
			s[0] = 0;
		SetHistory(s);

		// Get Observer
		ph = HEADER_KEYWORDS[HH_OBSERVER];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%67[^\n]", s) != 1)
			break;
		if (strlen(s) == 1 && s[0] == '-')
			s[0] = 0;
		SetObserver(s);

		// Get X Pixel Size
		ph = HEADER_KEYWORDS[HH_X_PIXEL_SIZE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		if (d < 0.0001)
			d = 0.0001;
		SetPixelWidth(d);

		// Get Y Pixel Size
		ph = HEADER_KEYWORDS[HH_Y_PIXEL_SIZE];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		if (d < 0.0001)
			d = 0.0001;
		SetPixelHeight(d);

		// Get Pedestal
		ph = HEADER_KEYWORDS[HH_PEDESTAL];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%ld", &l) != 1)
			break;
		SetPedestal((l < 1 ? 1 : (l > 65535 ? 65535 : l)));

		// Get E Gain
		ph = HEADER_KEYWORDS[HH_E_GAIN];
		if ((p = strstr(p, ph)) == NULL)
			break;
		if (sscanf(p += strlen(ph), "%lf", &d) != 1)
			break;
		if (d < 0.01)
			d = 0.01;
		SetEGain(d);

		// the following parameters are optional so don't bail
		// if they aren't found and initialize them to some
		// reasonable value
		SetSoftware("");
		SetFilter("Unknown");
		SetTrackExposure(0.0);
		SetSaturationLevel(65535);

		// Get User 1 = Software Version
		ph = HEADER_KEYWORDS[HH_USER_1];
		if ((p1 = strstr(p, ph)) != NULL)
			if (sscanf(p = p1 + strlen(ph), "%64[^\n]", s) == 1)
				if (strlen(s) > 1 || s[0] != '-')
					SetSoftware(s);

		// Get User 2 = Camera Model
		ph = HEADER_KEYWORDS[HH_USER_2];
		if ((p1 = strstr(p, ph)) != NULL)
			if (sscanf(p = p1 + strlen(ph), "%64[^\n]", s) == 1)
				if (strlen(s) > 1 || s[0] != '-')
					SetCameraModel(s);

		// Get User 3 = Expanded Exposure, Each Exposure
		ph = HEADER_KEYWORDS[HH_USER_3];
		if ((p1 = strstr(p, ph)) != NULL) {
			if (sscanf(p = p1 + strlen(ph), "%64[^\n]", s) == 1) {
				if (strlen(s) > 1 || s[0] != '-') {
					if (sscanf(s, "Exposure = %lf, Each_exposure = %lf", &d, &d2) == 2) {
						SetExposureTime(d < 0.001 ? 0.001 : d);
						SetEachExposure(d2 < 0.001 ? 0.001 : d2);
					}
				}
			}
		}

		// Get User 4 = Y2K Year
		ph = HEADER_KEYWORDS[HH_USER_4];
		if ((p1 = strstr(p, ph)) != NULL) {
			if (sscanf(p = p1 + strlen(ph), "%64[^\n]", s) == 1) {
				if (strlen(s) > 1 || s[0] != '-') {
					if (sscanf(s, "Y2K%*cear = %d", &i) == 1) {
						tm.tm_year = i - 1900;
						SetImageStartTime(&tm);

						//      fprintf(stderr,"\nCSBIGImg::ParseHeader():User_4 = %s, Year = %d", s, i);
					}
				}
			}
		}

		// Get Filter
		ph = HEADER_KEYWORDS[HH_FILTER];
		if ((p1 = strstr(p, ph)) != NULL)
			if (sscanf(p = p1 + strlen(ph), "%64[^\n]", s) == 1)
				if (strlen(s) > 1 || s[0] != '-')
					SetFilter(s);

		// Get Readout Mode
		ph = HEADER_KEYWORDS[HH_READOUT_MODE];
		if ((p1 = strstr(p, ph)) != NULL)
			if (sscanf(p = p1 + strlen(ph), "%ld", &l) == 1)
				SetReadoutMode((l < 0 ? 0 : (l > 65535 ? 65535 : l)));

		// Get Track Exposure
		ph = HEADER_KEYWORDS[HH_TRACK_TIME];
		if ((p1 = strstr(p, ph)) != NULL)
			if (sscanf(p = p1 + strlen(ph), "%ld", &l) == 1) {
				l = (l < 0 ? 0 : l);
				SetTrackExposure((double)l / 100.0);
			}

		// Get Saturation Level
		ph = HEADER_KEYWORDS[HH_SAT_LEVEL];
		if ((p1 = strstr(p, ph)) != NULL)
			if (sscanf(p = p1 + strlen(ph), "%ld", &l) == 1)
				SetSaturationLevel((l < 0 ? 0 : (l > 65535 ? 65535 : l)));

		// Make sure there's an End
		ph = HEADER_KEYWORDS[HH_END];
		if ((p = strstr(p, ph)) == NULL)
			break;

		// if we go here the header is valid
		res = TRUE;
	} while (FALSE);

	//    fprintf(stderr,"\nCSBIGImg::ParseHeader():Last header parsed = %s", ph);
	return res;
}

/*

	CompressSBIGData:

	Compress the imgRow row of pixel data into the pCmpData buffer,
	returning the length of the combressed data in bytes.

*/
int CSBIGImg::CompressSBIGData(unsigned char *pCmpData, int imgRow)
{
	/*~~~~~~~~~~~~~~~~~~~~~~*/
	unsigned short	us, *pImg;
	unsigned char	*puc;
	int				cmpLen, i;
	long			delta;
	/*~~~~~~~~~~~~~~~~~~~~~~*/

	pImg = m_pImage + (long)imgRow * m_nWidth;
	puc = pCmpData + 2;						// offset passed length
	cmpLen = 0;

	// encode first pixel as is
	us = *pImg++;
	*puc++ = (unsigned char)(us & 0xFF);	// ls byte first
	*puc++ = (unsigned char)(us >> 8);
	cmpLen += 2;

	// compress remaining pixels
	for (i = 1; i < m_nWidth; i++) {
		delta = (long)(*pImg) - us;
		us = *pImg++;
		if (delta >= -127 && delta <= 127) {

			// encode pixel as delta;
			*puc++ = (unsigned char)delta;
			cmpLen++;
			if (cmpLen >= 2 * m_nWidth)		// make syre don't overwrite buffer
				break;
		}
		else {

			// encode pixel directly
			if (cmpLen + 3 >= 2 * m_nWidth)
				break;
			*puc++ = 0x80;
			*puc++ = (unsigned char)(us & 0xFF);	// ls byte first
			*puc++ = (unsigned char)(us >> 8);
			cmpLen += 3;
		}
	}

	if (i < m_nWidth) {

		// compressed data is longer, simply copy uncompressed data
		// note we don't use memcpy here because the the byte order
		// in memory may be different that ls then ms required by
		// the file
		IntelCopyBytes(pCmpData + 2, imgRow);
		cmpLen = 2 * m_nWidth;
	}

	// encode length at start of buffer
	pCmpData[0] = (unsigned char)(cmpLen & 0xFF);	// ls byte of len
	pCmpData[1] = (unsigned char)(cmpLen >> 8);
	return cmpLen + 2;
}

/*

	IntelCopyBytes:

	Copy the imgRow row of pixels to the passed buffer
	preserving the Intel byte order (ls them ms).

*/
void CSBIGImg::IntelCopyBytes(unsigned char *pRevData, int imgRow)
{
	/*~~~~~~~~~~~~~~~~~~~~~~*/
	int				i;
	unsigned short	us, *pImg;
	unsigned char	*puc;
	/*~~~~~~~~~~~~~~~~~~~~~~*/

	pImg = m_pImage + (long)imgRow * m_nWidth;
	puc = pRevData;
	for (i = 0; i < m_nWidth; i++) {
		us = *pImg++;
		*puc++ = (unsigned char)(us & 0xFF);	// ls byte first
		*puc++ = (unsigned char)(us >> 8);
	}
}

/*

	AutoBackgroundAndRange:

	By making a histogram of the image set the
	Background and Range for auto-contrast.

	This has no affect on the actual pixel data.

*/
void CSBIGImg::AutoBackgroundAndRange(void)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	unsigned long	hist[4096];
	int				i, j;
	unsigned short	*pVid;
	unsigned long	totalPixels, histSum;
	unsigned long	s20, s99;
	unsigned short	p20, p99;
	long			back, range;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	// skip if no image data
	if (m_pImage == NULL)
		return;

	// calculate the pixel histogram with 4096 bins
	memset(hist, 0, sizeof(hist));
	pVid = m_pImage;
	for (i = 0; i < m_nHeight; i++)
		for (j = 0; j < m_nWidth; j++)
			hist[(*pVid++) >> 4]++;

	// integrate the histogram and find the 20% and 99% points
	totalPixels = (unsigned long)m_nWidth * m_nHeight;
	s20 = (20 * totalPixels) / 100;
	s99 = (99 * totalPixels) / 100;
	histSum = 0;
	p20 = p99 = 65535;
	for (i = 0; i < 4096; i++) {
		histSum += hist[i];
		if (histSum >= s20 && p20 == 65535)
			p20 = i;
		if (histSum >= s99 && p99 == 65535)
			p99 = i;
	}

	// set the range to 110% of the difference between
	// the 99% and 20% histogram points, not letting
	// it be too low or overflow unsigned short
	range = (16L * (p99 - p20) * 11) / 10;
	if (range < 64)
		range = 64;
	else if (range > 65535)
		range = 65535;

	// set the background to the 20% point lowered
	// by 10% of the range so it's not completely
	// black.  Also check for overrange and don't
	// let a saturated image show up a black
	back = 16L * p20 - range / 10;
	if (back < 0)
		back = 0;
	else if (p20 >= 4080)	// saturated image?
		back = 16L * 4080 - range;
	m_uBackground = (unsigned short)back;
	m_uRange = (unsigned short)range;
}

/*

	HorizontalFlip:

	Flip the image horizontally about the center.

*/
void CSBIGImg::HorizontalFlip(void)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~*/
	int				i, j;
	unsigned short	*pVid, vid;
	/*~~~~~~~~~~~~~~~~~~~~~~~*/

	if (m_pImage == NULL)
		return;

	for (i = 0; i < m_nHeight; i++) {
		pVid = m_pImage + (long)i * m_nWidth;
		for (j = 0; j < m_nWidth / 2; j++) {
			vid = pVid[j];
			pVid[j] = pVid[m_nWidth - 1 - j];
			pVid[m_nWidth - 1 - j] = vid;
		}
	}

	m_cHistory += "L";	// per SBIG Image Format Doc
	SetImageModified(TRUE);
}

/*

	VerticalFlip:

	Flip the image vertically about the center.

*/
void CSBIGImg::VerticalFlip(void)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	int				i, j;
	unsigned short	*pVid1, *pVid2, vid;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if (m_pImage == NULL)
		return;

	for (j = 0; j < m_nWidth; j++) {
		for (i = 0; i < m_nHeight / 2; i++) {
			pVid1 = m_pImage + (long)i * m_nWidth + j;
			pVid2 = m_pImage + (long)(m_nHeight - i - 1) * m_nWidth + j;
			vid = *pVid1;
			*pVid1 = *pVid2;
			*pVid2 = vid;
		}
	}

	m_cHistory += "M";	// per SBIG Image Format doc
	SetImageModified(TRUE);
}
#if INCLUDE_FITSIO
/*

  FITS Utilities

  These are only compiled if the INCLUDE_FITSIO compile
  time option is set.  They depend on the CFITSIO library
  which can be found at:

  <http://heasarc.gsfc.nasa.gov/docs/software/fitsio/fitsio.html>

*/

/*

  SaveFITS:

  Save a image file in FITS format. Uses routines from
  the CFITSIO lib.

 */
SBIG_FILE_ERROR CSBIGImg::SaveFITS(const char *filename)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	fitsfile	*fptr;
	int			status = 0;
	long		naxes[2] = { m_nWidth, m_nHeight }; //image width by height
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	remove(filename);	// Delete old file if it already exists
	if (fits_create_file(&fptr, filename, &status)) { // create new FITS file
		return SBFE_OPEN_ERROR;
    }

	if (fits_create_img(fptr, USHORT_IMG, 2, naxes, &status)) {
		return SBFE_OPEN_ERROR;
    }

	/*
     * write the array of unsigned integers to the FITS file
     */
	if (fits_write_img(fptr, TUSHORT, 1, m_nHeight * m_nWidth, m_pImage, &status))
		return SBFE_WRITE_ERROR;

    char        timeBuf[128], dateBuf[128], expStateBuf[5];
	struct tm	*plt = &m_sDecodedImageStartTime;
	time_t curTime = time(NULL);
	sprintf(timeBuf,"%04d-%02d-%02dT%02d.%02d.%02d.000",
				 plt->tm_year+1900,
				 plt->tm_mon + 1,
				 plt->tm_mday,
				 plt->tm_hour,
				 plt->tm_min,
				 plt->tm_sec);
	plt = gmtime(&curTime);
	sprintf(dateBuf,"%04d-%02d-%02d",
				 plt->tm_year+1900,
				 plt->tm_mon + 1,
				 plt->tm_mday);
	sprintf(expStateBuf,"%X", m_uExposureState);

    status = 0;

    char *cmt1 = "SBIG FITS header format per:";
    char *cmt2 = " http://www.sbig.com/pdffiles/SBFITSEXT_1r0.pdf";
	long fitsPedestal = (long)m_uPedestal - 100;
	long fitsWhite    = (long)m_uBackground + m_uRange;
	double pixWidth   = m_dPixelWidth*1000.0;
	double pixHeight  = m_dPixelHeight*1000.0;
	double focalLen	  = m_dFocalLength * 25.4;
	double apDiam     = m_dApertureDiameter * 25.4;
	double apArea     = m_dApertureArea * 25.4 * 25.4;

    fits_update_key(fptr, TSTRING, "COMMENT", (void *)cmt1, "", &status);
    fits_update_key(fptr, TSTRING, "COMMENT", (void *)cmt2, "", &status);

    fits_write_key(fptr, TSTRING, "OBJECT",   (void *)m_cFITSObject.c_str(), "", &status);
    fits_write_key(fptr, TSTRING, "TELESCOP", (void *)m_cFITSTelescope.c_str(), "", &status);
    fits_write_key(fptr, TSTRING, "INSTRUME", (void *)m_cCameraModel.c_str(), "Camera Model", &status);
    fits_write_key(fptr, TSTRING, "OBSERVER", (void *)m_cObserver.c_str(), "", &status);
	fits_write_key(fptr, TSTRING, "DATE-OBS", (void *)timeBuf, "GMT START OF EXPOSURE", &status);
    fits_write_key(fptr, TDOUBLE, "EXPTIME",  (void *)&m_dExposureTime, "EXPOSURE IN SECONDS", &status);
    fits_write_key(fptr, TDOUBLE, "CCD-TEMP", (void *)&m_dCCDTemperature, "CCD TEMP IN DEGREES C", &status);
    fits_write_key(fptr, TDOUBLE, "XPIXSZ",   (void *)&pixWidth, "PIXEL WIDTH IN MICRONS", &status);
    fits_write_key(fptr, TDOUBLE, "YPIXSZ",   (void *)&pixHeight, "PIXEL HEIGHT IN MICRONS", &status);
    fits_write_key(fptr, TUSHORT, "XBINNING", (void *)&m_uHorizontalBinning, "HORIZONTAL BINNING FACTOR", &status);
    fits_write_key(fptr, TUSHORT, "YBINNING", (void *)&m_uVerticalBinning, "VERTICAL BINNING FACTOR", &status);
    fits_write_key(fptr, TINT,    "XORGSUBF", (void *)&m_nSubFrameLeft, "SUB_FRAME ORIGIN X_POS", &status);
    fits_write_key(fptr, TINT,    "YORGSUBF", (void *)&m_nSubFrameTop, "SUB_FRAME ORIGIN Y_POS", &status);
    fits_write_key(fptr, TDOUBLE, "EGAIN",    (void *)&m_dEGain, "ELECTRONS PER ADU", &status);
    fits_write_key(fptr, TDOUBLE, "FOCALLEN", (void *)&focalLen, "FOCAL LENGTH IN MM", &status);
    fits_write_key(fptr, TDOUBLE, "APTDIA",   (void *)&apDiam, "APERTURE DIAMETER IN MM", &status);
    fits_write_key(fptr, TDOUBLE, "APTAREA",  (void *)&apArea, "APERTURE AREA IN SQ-MM", &status);
    fits_write_key(fptr, TUSHORT, "CBLACK",   (void *)&m_uBackground, "BLACK ADU FOR DISPLAY", &status);
    fits_write_key(fptr, TLONG,   "CWHITE",   (void *)&fitsWhite, "WHITE ADU FOR DISPLAY", &status);
    fits_write_key(fptr, TLONG,   "PEDESTAL", (void *)&fitsPedestal, "ADD TO ADU FOR 0-BASE", &status);
    fits_write_key(fptr, TUSHORT, "DATAMAX",  (void *)&m_uSaturationLevel, "SATURATION LEVEL", &status);
    fits_write_key(fptr, TSTRING, "SBSTDVER", (void *)"SBFITSEXT Version 1.0", "SBIG FITS EXTENSIONS VER", &status);
    fits_write_key(fptr, TSTRING, "SWACQUIR", (void *)m_cSoftware.c_str(), "DATA ACQ SOFTWARE", &status);
    fits_write_key(fptr, TSTRING, "SWCREATE", (void *)m_cSoftware.c_str(), "", &status);
	fits_write_key(fptr, TSTRING, "FILTER",   (void *)m_cFilter.c_str(), "OPTICAL FILTER NAME", &status);
    fits_write_key(fptr, TUSHORT, "SNAPSHOT", (void *)&m_uNumberExposures, "NUMBER IMAGES COADDED", &status);
	fits_write_key(fptr, TSTRING, "DATE",     (void *)dateBuf, "GMT DATE WHEN THIS FILE CREATED", &status);
    fits_write_key(fptr, TUSHORT, "RESMODE",  (void *)&m_uReadoutMode, "RESOLUTION MODE", &status);
	fits_write_key(fptr, TSTRING, "EXPSTATE", (void *)expStateBuf, "EXPOSURE STATE (HEX)", &status);
    fits_write_key(fptr, TDOUBLE, "RESPONSE", (void *)&m_dResponseFactor, "CCD RESPONSE FACTOR", &status);
	fits_write_key(fptr, TSTRING, "NOTE",     (void *)m_cImageNote.c_str(), "", &status);

    if (status != 0)
		return SBFE_FITS_HEADER_ERROR;

    if (History2FITS(fptr) != SBFE_NO_ERROR)
		return SBFE_FITS_HEADER_ERROR;

	if (fits_close_file(fptr, &status)) 				// close the file
		return SBRE_CLOSE_ERROR;

	return SBFE_NO_ERROR;
}

/*

  History2FITS:
  
  Generates a FITS HISTORY card for each history element.

*/
static const char HISTORY_CHARS[] = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvw";
static const char *HISTORY_PHRASES[] = {
	"Unknown Modification", "Image Co-Addition", "Scale ADU Values","Crop Image",
	"Dark Subtraction", "Remove Cool Pixels", "Flat Field", "Smooth Pixels",
	"Sharpen Pixels", "Pseudo Flat Field", "Quantize ADU Values", "Remove Warm Pixels",
	"Flip Horizontally", "Flip Vertically", "Zoom In", "Further Additional Modifications",
	"Log Scale ADU Values", "Combine Pixels", "Auto Dark Subtraction", "Replicate Pixels",
	"Clip ADU Values", "Compress Dynamic Range", "RGB Merge Version 2",
	"RGB Merge Version 3", "Translate Image", "Invert ADU Values", "Sharpen Pixels Asymmetrically",
	"Expand Dynamic Range", "Modernize", "Resample Pixel Size", "Average Images",
	"Add/Subtract ADU Constant", "Multiply/Divide ADU by Constant",
	"Enlarge Image", "Reduce Image",
	"Repair Column", "Adaptive Dark Subtraction", "Make Pseudo 3-D Image",
	"Auto Dark Subtraction with Hot Pixel Removal","Dark Subtraction with Hot Pixel Removal",
	"LR Deconvolve Image","Spatial Median Filter", "Set ADU Saturation Level",
	"DDP Image", "Rotate Image", "Fix TDI Background", "ME CCD Spike Removal",
	"Fix Bloomed Stars", "Remove Image Gradient",
	"Extract RGB", "Extract Luminance", "Rotate Clockwise", "Rotate Counter-Clockwise",
	"Median Combine 3 Images", "Rotate 180", "Raw Single Shot Color"};
SBIG_FILE_ERROR CSBIGImg::History2FITS(fitsfile	*fptr)
{
	int			status = 0;
    const char  *msg, *p;
    char		c, *cp = (char*)m_cHistory.c_str();
	int			index;
	MY_LOGICAL	first = TRUE;

    while (*cp) {
		c = *cp++;
		if ( (p=strchr(HISTORY_CHARS, c)) != NULL ) {
			index = (p - HISTORY_CHARS)/sizeof(const char);
			msg = HISTORY_PHRASES[index];
		} else if ( c == '0' )
			continue;
		else
			msg = "???";

		if ( first )
			fits_write_key(fptr, TSTRING, "SWMODIFY", (void *)m_cSoftware.c_str(), "", &status);
		first = FALSE;

	    if (fits_write_history(fptr,msg,&status)) {
		    return SBFE_FITS_HEADER_ERROR;
        }
    }
	return SBFE_NO_ERROR;
}
#endif
