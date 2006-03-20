/*
|==============================================================================
| Copyright (C) 2001 Quantitative Imaging Corporation.  All Rights Reserved.
| Reproduction or disclosure of this file or its contents without the prior
| written consent of Quantitative Imaging Corporation is prohibited.
|==============================================================================
|
| File:			FileTiff.h
|
| Project/lib:	QCapture
|
| Target:		all
|
| Description:	Wrapper classes for libtiff.  www.libtiff.org
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

#ifndef FILETIFF_H_INCLUDE
#define FILETIFF_H_INCLUDE

//===== INCLUDE FILES =========================================================

//#include <QImgTypes.h>
#include "QCamApi.h"

//===== #DEFINES ==============================================================

//===== TYPE DEFINITIONS ======================================================

struct tiff;

class CFileTiffRead
{
public:

	CFileTiffRead();
	~CFileTiffRead();

	bool Open( const char* filename );

	bool GetImageSize( unsigned long* size );		// Size, in bytes
	bool Read( QCam_Frame* frame );

private:

	struct tiff*		m_tiff;

};


class CFileTiffWrite
{
public:

	CFileTiffWrite();
	~CFileTiffWrite();

	bool Open( const char* filename );

	bool Write( QCam_Frame* frame );

private:

	struct tiff*		m_tiff;

};

//===== FUNCTION PROTOTYPES ===================================================

//===== DATA ==================================================================


#endif // FILETIFF_H_INCLUDE

