/*
|==============================================================================
| Copyright (C) 2001 Quantitative Imaging Corporation.  All Rights Reserved.
| Reproduction or disclosure of this file or its contents without the prior
| written consent of Quantitative Imaging Corporation is prohibited.
|==============================================================================
|
| File:			FileTiff.cpp
|
| Project/lib:	QCapture
|
| Target:		Win32
|
| Description:	Win32 implementation of CFileTiffRead/CFileTiffWrite.
|
| Notes:		
|
|==============================================================================
| dd/mon/yy  Author		Notes
|------------------------------------------------------------------------------
| 02/Jan/01	 DavidL		Original.
| 22/May/01  DavidL		Stripped of QImgTypes.
|==============================================================================
*/

//===== INCLUDE FILES =========================================================

#include "FileTiff.h"
#include "tiffio.h"
#include "QCamImgfnc.h"
#include <Assert.h>

//===== #DEFINES ==============================================================

#define ASSERT( x )	assert( x )

//===== TYPE DEFINITIONS ======================================================

//===== FUNCTION PROTOTYPES ===================================================

//===== DATA (PUBLIC) =========================================================

//===== DATA (PRIVATE) ========================================================

//===== IMPLEMENTATION ========================================================


CFileTiffRead::CFileTiffRead
(
	void
)
{

	TIFFSetWarningHandler( 0 );
	TIFFSetErrorHandler( 0 );

	m_tiff = 0;

}


CFileTiffRead::~CFileTiffRead
(
	void
)
{

	if ( m_tiff )
		TIFFClose( m_tiff );

}


bool CFileTiffRead::Open
(
	const char*			filename
)
{

	ASSERT( ! m_tiff );	// Shouldn't call Open twice per object

	m_tiff = TIFFOpen( filename, "r" );

	return m_tiff ? true : false;
}


bool CFileTiffRead::GetImageSize
(
	unsigned long*		size
)
{
	unsigned long		stripBytes;
	unsigned long		numStrips;


	// Make sure the file was opened OK.
	if ( ! m_tiff )
		return false;

	if ((( stripBytes = TIFFStripSize( m_tiff ))) &&
		(( numStrips = TIFFNumberOfStrips( m_tiff ))))
	{
		*size = stripBytes * numStrips;
		return true;
	}

	return false;
}


bool CFileTiffRead::Read
(
	QCam_Frame*			frame
)
{
	unsigned long		length;
	unsigned long		width;
	unsigned long		stripBytes;
	unsigned long		numStrips;
	unsigned short		bitsPerSample;
	unsigned short		samplesPerPixel;


	// Make sure the file was opened OK.
	if ( ! m_tiff )
		return false;

	// Get the strip dimensions.
	if (( ! ( stripBytes = TIFFStripSize( m_tiff ))) ||
		( ! ( numStrips = TIFFNumberOfStrips( m_tiff ))))
	{
		return false;
	}

	// Make sure the buffer is big enough to take the image.
	ASSERT( frame->bufferSize >= stripBytes * numStrips );
	if ( frame->bufferSize < stripBytes * numStrips )
		return false;

	// Get the image height and width.
	if (( ! TIFFGetField( m_tiff, TIFFTAG_IMAGELENGTH, &length )) ||
		( ! TIFFGetField( m_tiff, TIFFTAG_IMAGEWIDTH, &width )))
	{
		return false;
	}

	frame->height = length;
	frame->width = width;
	frame->size = length * width;

	// Get bits per sample.
	if ( ! TIFFGetField( m_tiff, TIFFTAG_BITSPERSAMPLE, &bitsPerSample ))
		return false;

	// Samples per pixel might not exist in a grayscale image.
	if ( ! TIFFGetField( m_tiff, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel ))
		samplesPerPixel = 1;

	if (( bitsPerSample == 8 ) && ( samplesPerPixel == 1 ))
	{
		frame->format = qfmtMono8;
		frame->bits = 8;
	}
	else if (( bitsPerSample == 8 ) && ( samplesPerPixel == 3 ))
	{
		frame->format = qfmtBgr24;
		frame->bits = 8;
	}
	else if (( bitsPerSample == 16 ) && ( samplesPerPixel == 1 ))
	{
		frame->format = qfmtMono16;
		frame->bits = 16;
	}
	else if (( bitsPerSample == 16 ) && ( samplesPerPixel == 3 ))
	{
		frame->format = qfmtRgb48;
		frame->bits = 16;
	}
	else
		return false;

	unsigned char* destPtr = static_cast< unsigned char* >( frame->pBuffer );

	// Read in the image.
	for ( unsigned long strip = 0; strip < numStrips; strip++ )
	{
		TIFFReadEncodedStrip( m_tiff, strip, destPtr, ( tsize_t )-1 );

		// For BGR24 we need to reverse the B and R bytes.  RGB48 is in the
		// correct format.
		if ( frame->format == qfmtBgr24 )
		{
			for ( unsigned long i = 0; i < stripBytes; i += 3 )
			{
				byte temp = destPtr[ i ];
				destPtr[ i ] = destPtr[ i+2 ];
				destPtr[ i+2 ] = temp;
			}
		}

		destPtr += stripBytes;
	}

	return true;

}


CFileTiffWrite::CFileTiffWrite
(
	void
)
{
	m_tiff = 0;
}


CFileTiffWrite::~CFileTiffWrite
(
	void
)
{

	if ( m_tiff )
		TIFFClose( m_tiff );

}



bool CFileTiffWrite::Open
(
	const char*			filename
)
{

	ASSERT( ! m_tiff );	// Shouldn't call Open twice per object

	m_tiff = TIFFOpen( filename, "w" );

	return m_tiff ? true : false;
}



bool CFileTiffWrite::Write
(
	QCam_Frame*			frame
)
{
	unsigned long		stripBytes;
	unsigned long		numStrips;
	unsigned short		bitsPerSample;
	unsigned short		samplesPerPixel;
	unsigned short		photometric;


	// Make sure the file was opened OK.
	if ( ! m_tiff )
		return false;

	numStrips = frame->height;
	stripBytes = QCam_CalcImageSize(( QCam_ImageFormat )frame->format, frame->width, 1 );

	// Format dependent TIFF fields...
	switch ( frame->format )
	{
	case qfmtMono8:		bitsPerSample =	8;	samplesPerPixel = 1;	photometric = 1;	break;
	case qfmtMono16:	bitsPerSample =	16;	samplesPerPixel = 1;	photometric = 1;	break;
	case qfmtBgr24:		bitsPerSample =	8;	samplesPerPixel = 3;	photometric = 2;	break;
	case qfmtRgb48:		bitsPerSample =	16;	samplesPerPixel = 3;	photometric = 2;	break;
	default:
		ASSERT( false );
		return false;
	};

	// All required fields for RGB and grayscale, except Xres, Yres, and res-unit.
	TIFFSetField( m_tiff, TIFFTAG_ROWSPERSTRIP, ( unsigned long ) 1 );
	TIFFSetField( m_tiff, TIFFTAG_BITSPERSAMPLE, bitsPerSample );
	TIFFSetField( m_tiff, TIFFTAG_SAMPLESPERPIXEL, samplesPerPixel );
	TIFFSetField( m_tiff, TIFFTAG_IMAGEWIDTH, frame->width );
	TIFFSetField( m_tiff, TIFFTAG_IMAGELENGTH, frame->height );
	TIFFSetField( m_tiff, TIFFTAG_PHOTOMETRIC, photometric );
	TIFFSetField( m_tiff, TIFFTAG_PLANARCONFIG, ( unsigned short ) 1 );	// RGBRGB
	TIFFSetField( m_tiff, TIFFTAG_COMPRESSION, ( unsigned short ) 1 );	// None

	//
	// Write our data here...  We need to handle bgr24 and 16-bits as
	// special cases.
	//
	if ( frame->format == qfmtBgr24 )
	{
		unsigned char* srcPtr = static_cast< unsigned char* >( frame->pBuffer );

		// We need a temporary buffer for bgr24 format.  Each strip of the
		// image is B R reversed before writing the strip.

		unsigned char* tempStrip = new unsigned char[ stripBytes ];

		for ( unsigned long strip = 0; strip < numStrips; strip++ )
		{
			// Reverse B R for this strip.
			for ( unsigned long i = 0; i < stripBytes; i += 3 )
			{
				tempStrip[ i+2 ] = *( srcPtr++ );
				tempStrip[ i+1 ] = *( srcPtr++ );
				tempStrip[ i ] = *( srcPtr++ );
			}

			TIFFWriteEncodedStrip( m_tiff, strip, tempStrip, stripBytes );
		}

		delete [] tempStrip;
	}
	else if ( QCam_is16bit(( QCam_ImageFormat ) frame->format ))
	{
		unsigned short* srcPtr = static_cast< unsigned short* >( frame->pBuffer );

		unsigned long	stripWords;			// unsigned short per strip
		unsigned short	dataShift;			// #bits shift left to MSB align

		stripWords = stripBytes >> 1;

		ASSERT( frame->bits > 8 );
		ASSERT( frame->bits <= 16 );
		dataShift = 16 - frame->bits;

		// We need a temporary buffer for 16 bit formats.  Each strip of the
		// image needs its data MSB aligned.
		unsigned short* tempStrip = new unsigned short[ stripWords ];
		
		for ( unsigned long strip = 0; strip < numStrips; strip++ )
		{
			// MSB align this strip.
			for ( unsigned long i = 0; i < stripWords; i++ )
			{
				tempStrip[ i ] = ( *srcPtr ) << dataShift;
				srcPtr++;
			}

			TIFFWriteEncodedStrip( m_tiff, strip, tempStrip, stripBytes );
		}

		delete [] tempStrip;

	}
	else
	{
		unsigned char* srcPtr = static_cast< unsigned char* >( frame->pBuffer );

		for ( unsigned long strip = 0; strip < numStrips; strip++ )
		{
			TIFFWriteEncodedStrip( m_tiff, strip, srcPtr, stripBytes );
			srcPtr += stripBytes;
		}
	}

	return true;
}



