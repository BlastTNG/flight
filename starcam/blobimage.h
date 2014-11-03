//extend CSBIGImg to add blob finding functionality

#ifndef BLOBIMAGE_H
#define BLOBIMAGE_H

#include <string>
#include <sys/time.h>
#include "csbigimg.h"
#include "csbigimgdefs.h"
#include "frameblob.h"
#include "camconfig.h"
#include "camstruct.h"
#include "pyramid.h"

//flag for compiling with use of lost in space star matching
#define USE_PYRAMID 0

/**
	@author Steve Benton <steve.benton@utoronto.ca>
*/

class BlobImage : public CSBIGImg
{
private:
	frameblob m_cBlob;         //object for blob finding in the image
	string m_sBadpixFilename;  //name of file containing coordinates of bad pixels on CCD
	timeval m_sImageStartTime; //privde higher resolution timestamps than CSBIGImg does
	unsigned long m_nTimeError;//upper bound on imprecision of image start time (us)
	string m_sCamID;           //identifies camera that took picture
	bool m_bIsChanged;         //allows viewer to tell if contents have changed
	unsigned long int m_iFrame;//frame number
#if USE_PYRAMID	
	Pyramid m_cPyramid;        //object for pattern recognition
	double m_dMatchTol;        //tolerance value (focal plane distance) for pattern matchcing
	double m_dPlatescale;      //conversion factor between pixel position and radians of sky
#endif
	
public:
	BlobImage(BlobImageConfigParams params = defaultImageParams);
	BlobImage(int height, int width, BlobImageConfigParams params = defaultImageParams);
    ~BlobImage();
	
	//overridden (or overloaded) parent methods
	void Init(BlobImageConfigParams params = defaultImageParams);
	MY_LOGICAL AllocateImageBuffer(int height, int width);
	void DeleteImageData();
	SBIG_FILE_ERROR OpenImage(const char *pFullPath);
	void SetImageStartTime(void);      //set start time to current time
	void SetImageStartTime(timeval* ref);    //sets start time and error based on ref
	timeval GetImageStartTime(void) { return m_sImageStartTime; }
	
	//image processing methods
//	void InitFrameblob();            //no longer needed
	void copyImageFrom(const unsigned short*);
	SBIG_FILE_ERROR FixBadpix(string filename);
	int findBlobs();
	int crapifyImage();
	int highPassFilter(int box_size, int n_boxes);
#if USE_PYRAMID
	int matchStars(solution_t **sol);
#endif
	
	//accessors
	frameblob* getFrameBlob(void) { return &m_cBlob; }
	string getBadpixFilename(void) { return m_sBadpixFilename; }
	void setBadpixFilename(string name) { m_sBadpixFilename = name; }
	void setBadpixFilename(const char* name);
	void setCameraID(string in_id) { m_sCamID = in_id; }
	string getCameraID(void) { return m_sCamID; }
	bool isChanged() { return m_bIsChanged; }
	void setChanged(bool flag) { m_bIsChanged = flag; }
	void setFrameNum(unsigned long int frameNum) { m_iFrame = frameNum; }
#if USE_PYRAMID
	Pyramid* getPyramid() { return &m_cPyramid; }
	void setMatchTol(double tol) { m_dMatchTol = tol; }
	double getMatchTol(void) { return m_dMatchTol; }
	void setPlatescale(double scale) { m_dPlatescale = scale; }
	double getPlatescale(void) { return m_dPlatescale; } 
#endif
	
	StarcamReturn* createReturnStruct(StarcamReturn* arg);
	
	//functions to help in testing the blob finder
	void drawBox(double x, double y, double side, int bnum, bool willChange=true);
	string createFilename();
	string createDirectory(string root, int boxflag);
	SBIG_FILE_ERROR SaveImageIn(string root="/usr/local/starcam/pictures/", int boxflag=0);
	
};

#endif
