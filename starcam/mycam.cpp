//
// C++ Implementation: mycam
//
// Description: 
//
//
// Author: Steve Benton <steve.benton@utoronto.ca>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include "mycam.h"
#include "sbigudrv.h"
#include "csbigcam.h"
#include "clensadapter.h"
#include "blobimage.h"
#include "focusstruct.h"
#include "camcommunicator.h"

extern "C" {
#include "udp.h"
}

#define MYCAM_DEBUG 0
#define MYCAM_TIMING 0
#define AUTOFOCUS_DEBUG 1

//minimum step size in autofocus, only used when suspect stickiness
#define MIN_AUTO_STEP 10

int copyFlag = 0;

using namespace std;
//use same constructors as CSBIGCam, make sure proper Init is called though.
MyCam::MyCam() : CSBIGCam() { Init(); }
MyCam::MyCam(OpenDeviceParams odp) : CSBIGCam(odp) { Init(); }
MyCam::MyCam(SBIG_DEVICE_TYPE dev) : CSBIGCam(dev) { Init(); }


//perform same operations as the CSBIGCam destructor
MyCam::~MyCam()
{
	CloseDevice();
	CloseDriver();
	m_cAdapter.closeConnection();
}

/*
  
 Init:
	 
 Initialize the base variables. Most of this is done my parent class.
 
*/
void MyCam::Init(MyCamConfigParams params/*=defaultCameraParams*/)
{
	CSBIGCam::Init(params.SBIGParams);
	m_cAdapter.Init(params.lensParams);
	m_nUSBNum = params.USBNum;
	m_nPictureInterval = params.pictureInterval;
	m_nFocusResolution = params.focusResolution;
	m_nFocusRange = params.focusRange;
	m_iFrame = 0;
}
/* original version has been replaced
void MyCam::Init()
{
	CSBIGCam::Init();
	m_nUSBNum = INVALID_USB_NUM;
	m_dPictureInterval = 0;
}
*/

/*

 QueryUSB:

 Runs the CC_QUERY_USB command against the universal driver
 
*/
PAR_ERROR MyCam::QueryUSB(QueryUSBResults &qur)
{
	return CSBIGCam::SBIGUnivDrvCommand(CC_QUERY_USB, NULL, &qur);
}

/*

 OpenUSBDevice:
 
 Opens a device on USB with "port" number num.
 Num should be the index of a QueryUSBResults.usbInfo with cameraFound == TRUE
 
*/
PAR_ERROR MyCam::OpenUSBDevice(int num)
{
	OpenDeviceParams odp;
	m_nUSBNum = (num < 0 || num > 3)?INVALID_USB_NUM:num;
	switch (num) {
		case 0:		odp.deviceType=DEV_USB1; break;
		case 1:		odp.deviceType=DEV_USB2; break;
		case 2:		odp.deviceType=DEV_USB3; break;
		case 3:		odp.deviceType=DEV_USB4; break;
		default:	odp.deviceType=DEV_NONE; break;
	}
	return CSBIGCam::SBIGUnivDrvCommand(CC_OPEN_DEVICE, &odp, NULL);
}

/*

 autoFocus:
 
 enter a focus mode that maximizes flux of brightest star in FOV
 steps down from focus at infinity in steps of (total range / m_nFocusResolution)
 a BlobImage is passed to avoid time consuming initialization of a new one
 will stop after decreasing for 3 consecutive measurements, or no longer identifying blob
 when forced is non-zero (TRUE), stops will be ignored until moving is impossible
 path is the location of files used by image viewer
 //TODO can add sub-step interpolation of maximum
 
*/
//LENS_ERROR MyCam::autoFocus(BlobImage *img, int forced/*=0*/, const char* path/*=NULL*/)
LENS_ERROR MyCam::autoFocus(BlobImage *img, int forced/*=0*/, string path)
{
	frameblob *blob = img->getFrameBlob();
	StarcamReturn returnStruct;
  bloblist* focblobs;
	if (!forced)  //can actually measure focal range
		if (m_cAdapter.findFocalRange() != LE_NO_ERROR) return m_cAdapter.getLastError();
	int range = (forced)?3270:m_cAdapter.getFocalRange();
	int step = range / m_nFocusResolution;         //number of motor counts between measurements
	int numsteps = m_nFocusResolution/m_nFocusRange; 
	focusStruct focuser[numsteps];
	//check if motor "stickiness" may have confused things
	if (forced && step < MIN_AUTO_STEP) step = MIN_AUTO_STEP;
#if AUTOFOCUS_DEBUG
	cout << "[autoFocus debug]: range=" << range <<" res=" << m_nFocusResolution << "step=" << step << endl;
#endif
	int remaining = 0;	//distance remainder returned from preciseMove
	int toInf = 0;
	int endpos = 0;
	int j=0;
	//focusing stuff:
	int m=1,n=0,o=0,p=0;
	int atfocus=0;
	int nblobhere = 0;
	int nblobplus = 0;
	int nblobminus = 0;
	double xhere = 0;
	double xplus = 0;
	double xminus = 0;
	double yhere = 0;
	double yplus = 0;
	double yminus = 0;
	int maxflux = 0;

  int sock;
  sock = udp_bind_port(FOCUS_PORT,1);
  if (sock == -1)
    cout << "unable to bind to port" << endl;
  string rtn_str = "";
	string sought = "\n";

	for (int i=0; i < numsteps; i++) {
//#if AUTOFOCUS_DEBUG
//		cout << "[autoFocus debug]: taking exposure" << endl;
//#endif
		if (this->GrabImage(img, SBDF_LIGHT_ONLY) != CE_NO_ERROR) {
#if AUTOFOCUS_DEBUG
			cout << "[autoFocus debug]: grabImage failed in autoFocus" << endl;
#endif
			return LE_AUTOFOCUS_ERROR;
		}

		img->findBlobs();
		focuser[i].numblobs = (blob->get_numblobs() > 15)?15:blob->get_numblobs();

#if AUTOFOCUS_DEBUG
		cout << "autofocus: Found " << blob->get_numblobs() << " blobs" << endl;
#endif
      		if (blob->get_numblobs()) {
			focblobs = blob->getblobs();
			while ((focblobs != NULL) && (j<15)) { //keep 15 blobs per images in focusStruct
#if AUTOFOCUS_DEBUG
				cout << "autofocus:   ..." << focblobs->getx() << " " << focblobs->gety() << " " << focblobs->getflux() << endl;
#endif
				focuser[i].flux[j] = focblobs->getflux();
				focuser[i].x[j] = focblobs->getx();
				focuser[i].y[j] = focblobs->gety();
				img->drawBox(focblobs->getx(), focblobs->gety(), 40, j+1);
				focblobs = focblobs->getnextblob();
				j++;
			}
		j=0;
		}

#if AUTOFOCUS_DEBUG
		cout << "[autoFocus debug]: saving focus image in : " << path << endl;
#endif
		if (img->SaveImageIn(path) != SBFE_NO_ERROR) {
#if AUTOFOCUS_DEBUG
		    cerr << "[autoFocus debug]: autoFocus failed to save image" << endl;
#endif
		}
		if (img->SaveImage("/data/etc/tmp_bad.sbig") != SBFE_NO_ERROR) {
#if AUTOFOCUS_DEBUG
		    cerr << "[autoFocus debug]: autoFocus failed to save viewer image" << endl;
#endif
		}
    copyFlag = 1;
		img->createReturnStruct(&returnStruct);
    rtn_str = CamCommunicator::buildReturn(&returnStruct);
    //remove all newlines and add a single one at the end
	  string::size_type pos = rtn_str.find(sought, 0);
	  while (pos != string::npos) {
      rtn_str.replace(pos, sought.size(), "");
      pos = rtn_str.find(sought, pos - sought.size());
    }
    rtn_str += "\n";
    udp_bcast(sock, SC_PORT, strlen(rtn_str.c_str()), rtn_str.c_str(), 0);
    cout << "AUTOFOCUS BROADCASTING string " << rtn_str << endl;

		focuser[i].focpos = toInf;
		//move to next location
		if (m_cAdapter.preciseMove(-step, remaining, forced) != LE_NO_ERROR)
			return m_cAdapter.getLastError();

		toInf += step + remaining;
		if (i==(numsteps-1)) endpos = toInf;
	}
  close(sock);

	while (m<(numsteps-1)) { //loop over all images
		nblobhere = focuser[m].numblobs;
		nblobplus = focuser[m+1].numblobs; //#blobs in next img
		nblobminus = focuser[m-1].numblobs;//#blobs in prev img
		while (n<nblobhere) { //loop over all blobs in this image
			xhere = focuser[m].x[n];
			yhere = focuser[m].y[n];
			while (o<nblobplus)  { //loop over all blobs in next image
				xplus = focuser[m+1].x[o];
		     		yplus = focuser[m+1].y[o];
				if ((abs(xplus-xhere) < 20) && (abs(yplus-yhere) < 20)) { 
					//if meets test in next image, check prev image:
					while (p<nblobminus) { //loop over all blobs in prev image	
						xminus = focuser[m-1].x[p];
						yminus = focuser[m-1].y[p];
		 				if ((abs(xminus-xhere) < 20) && (abs(yminus-yhere) < 20)) {
							if ((focuser[m].flux[n] > focuser[m+1].flux[o]) &&
							    (focuser[m].flux[n] > focuser[m-1].flux[p])) {
								if ((focuser[m].flux[n]) > maxflux) {
									maxflux = focuser[m].flux[n];
									atfocus = m;
								}
							}
						}
						p++;//go to next blob in prev image 
					}
				}
				o++;//go to next blob in next image 
			}
			o=0;
			p=0;
			n++;//go to next blob in this image...	
		}
		n=0;
		m++;//go to next image...		
	}	
#if AUTOFOCUS_DEBUG
	cerr << "maxflux = " << maxflux << ", focus position = " << focuser[atfocus].focpos << endl;
#endif
	//move back to focus position
	int toFocus = endpos - focuser[atfocus].focpos;
#if AUTOFOCUS_DEBUG
	cerr << "at endpos: " << endpos << ", need to move " << toFocus << " steps toFocus (=endpos-focpos)" << endl;
#endif
	if (m_cAdapter.preciseMove(toFocus, remaining, forced) != LE_NO_ERROR)
		return m_cAdapter.getLastError();
	return LE_NO_ERROR;

}

//Original autofocus function: needlessly complicated
// LENS_ERROR MyCam::autoFocus()
// {
// 	if (m_cAdapter.getFocalRange() == -1)    //if focal range isn't defined yet, find it
// 		if (m_cAdapter.findFocalRange() != LE_NO_ERROR) 
// 			return m_cAdapter.getLastError();
// 	
// 	int totalRange = m_cAdapter.getFocalRange();
// 	int step_size = totalRange / 16;    //initial size of focus steps
// 	int move;                           //amount to move by (to improve focus)
// 	int position = totalRange - step_size;  //position of the lens center spot
// 	int offset = 0;                     //position of lens relative to center sopt
// 	int remaining;                      //return value for preciseMove
// 	string return_str;                  //return value from lens commands
// 	BlobImage img;
// 	frameblob *blob = img.getFrameBlob();
// 	int smaller, center, larger;        //blob flux at three focus positions
// 	
// 	//move lens to initial center position
// 	if (m_cAdapter.runCommand(LC_MOVE_FOCUS_INFINITY, return_str) != LE_NO_ERROR)
// 		return m_cAdapter.getLastError();
// 	if (m_cAdapter.preciseMove(-step_size, remaining) != LE_NO_ERROR)
// 		return m_cAdapter.getLastError();
// 	offset -= remaining;
// 	
// 	for (int i=0; i<10; i++) {         //limit the number of iterations
// 		//find intensity of brightest blob at center position
// 		if ( (this->GrabImage(&img, SBDF_LIGHT_ONLY)) != CE_NO_ERROR )
// 			return LE_AUTOFOCUS_ERROR;
// 		img.findBlobs();
// 		if (blob->get_numblobs())
// 			center = blob->getblobs()->getflux();     //flux of brightest blob
// 		else
// 			center = -1;
// 		
// 		//find intensity of brightest blob at smaller (closer to zero) position
// 		if (m_cAdapter.preciseMove(-step_size, remaining) != LE_NO_ERROR)
// 			return m_cAdapter.getLastError();
// 		offset += -step_size - remaining;
// 		if ( (this->GrabImage(&img, SBDF_LIGHT_ONLY)) != CE_NO_ERROR )
// 			return LE_AUTOFOCUS_ERROR;
// 		img.findBlobs();
// 		if (blob->get_numblobs())
// 			smaller = blob->getblobs()->getflux();
// 		else
// 			smaller = -1;
// 
// 		//find intensity of brightest blob at larger (closer to infinity) position
// 		if (m_cAdapter.preciseMove(2*step_size, remaining) != LE_NO_ERROR)
// 			return m_cAdapter.getLastError();
// 		offset += 2*step_size - remaining;
// 		if ( (this->GrabImage(&img, SBDF_LIGHT_ONLY)) != CE_NO_ERROR )
// 			return LE_AUTOFOCUS_ERROR;
// 		img.findBlobs();
// 		if (blob->get_numblobs())
// 			larger = blob->getblobs()->getflux();
// 		else
// 			larger = -1;
// 		
// 		//find how far to move (relative to center) the lens to improve focus
// 		//first check cases where blobs were not found in all the images
// 		if (center == -1) {
// 			if (smaller == -1 && larger == -1) //no blobs were found anywhere
// 				move = step_size;
// 			else move = (smaller >= larger)?-step_size:step_size;
// 		} 
// 		else {       //blob found at center
// 			if (smaller == -1 && larger == -1) {
// 				step_size /= 2;
// 				move = 0;
// 			}
// 			else if (smaller == -1 || larger == -1) 
// 				move = (smaller == -1)?step_size:-step_size;
// 			else {  //blobs were found at each position (assume they are the same ones)
// 				//check concavity of flux profile
// 				if (2*center > smaller + larger) {
// 					//concave down...use quadratic interpolation
// 					move = (smaller - larger)/2/(smaller + larger - 2*center) * step_size;
// 					step_size /= 2;
// 				}
// 				//otherwise move toward the largest of the sides
// 				else move = (smaller >= larger)?-step_size:step_size;
// 			}
// 		}
// 		while (position + move - totalRange > (int)m_cAdapter.getFocusTol()
// 					 || position + move < (int)m_cAdapter.getFocusTol())
// 		{ //if position + move exceeds top or bottom of range, only go half as far 
// 			move /= 2;
// 			step_size /= 2;
// 		}
// 		m_cAdapter.preciseMove(move-offset, remaining);
// 		offset = move - remaining;
// 	}
// 	
// 	return LE_NO_ERROR;
// }

/*
  
 hex2double:
	 
 Convert the passed hex value to double.
 The hex value is assumed to be in the
 format: XXXXXX.XX
	 
*/
static double hex2double(unsigned long ul)
{
	double res, mult;
	int i;
	
	res = 0.0;
	mult = 1.0;
	for (i=0; i<8; i++)
	{
		res += mult * (double)(ul & 0x0F);
		ul >>= 4;
		mult *= 10.0;
	}
	return res / 100.0;
	
}


/*

 GrabImage:
 
 Overrides the CSBIGCam function to use BlobImage and provide better header information
 Note: Could get lens information from adapter, but that seems unnecessary unless
    the aperture will be changing in flight. Otherwise setting correct default is sufficient.
 
*/
PAR_ERROR MyCam::GrabImage(BlobImage *pImg, SBIG_DARK_FRAME dark)
{
#if MYCAM_DEBUG
	cout << "[MyCam debug]: inside GrabImage." << endl;
#endif	
	timeval reference;                                   //used for exposure time, not just debugging
#if MYCAM_TIMING
	timeval reference2, grabstart, grabstop;             //structs for retrieving time information
	unsigned int elapsed;                                //used to hold time in us between timevals
	gettimeofday(&grabstart, NULL);
	gettimeofday(&reference, NULL);
	double expTime, readTime, totalTime;
#endif
	int left, top, width, height;
	GetCCDInfoResults0 gcir;
	GetCCDInfoParams gcip;
	double ccdTemp;
	double foclen;
	string lens_ret_str, foc_str;
	double focpos;
	unsigned short vertNBinning, hBin, vBin;
	unsigned short rm;
	string s;
	unsigned short es;
	MY_LOGICAL expComp;
	PAR_ERROR err;
	StartReadoutParams srp;
	int i;
	ReadoutLineParams rlp;
	int subFrameWidth, subFrameHeight, subFrameTop, subFrameLeft;
	CSBIGCam::GetSubFrame(subFrameLeft, subFrameTop, subFrameWidth, subFrameHeight);
	//NNG CSBIGCam::SetDriverControl(DCP_HIGH_THROUGHPUT, TRUE);
	// Get the image dimensions
	vertNBinning = CSBIGCam::GetReadoutMode() >> 8;
	if ( vertNBinning == 0 )
		vertNBinning = 1;
	rm = CSBIGCam::GetReadoutMode() & 0xFF;
	hBin = vBin = 1;
	if ( rm < 3 )
		hBin = vBin = (rm + 1);
	else if ( rm < 6 ) {
		hBin = (rm - 5);
		vBin = vertNBinning;
	} else if ( rm < 9 )
		hBin = vBin = (rm - 8);
	else if ( rm == 9 )
		hBin = vBin = 9;
	gcip.request = (CSBIGCam::GetActiveCCD() == CCD_IMAGING ? CCD_INFO_IMAGING : CCD_INFO_TRACKING);
	if ( SBIGUnivDrvCommand(CC_GET_CCD_INFO, &gcip, &gcir) != CE_NO_ERROR )
		return CSBIGCam::GetError();
	if ( rm >= gcir.readoutModes )
		return CE_BAD_PARAMETER;
	if ( subFrameWidth == 0 || subFrameHeight == 0 ) {
		left = top = 0;
		width = gcir.readoutInfo[rm].width;
		height = gcir.readoutInfo[rm].height / vertNBinning;
	} else {
		left = subFrameLeft;
		top = subFrameTop;
		width = subFrameWidth;
		height = subFrameHeight;
	}

	// try to allocate the image buffer
	if ( !pImg->AllocateImageBuffer(height, width) )
		return CE_MEMORY_ERROR;
	pImg->SetImageModified(TRUE);         //used by csbigimg in refrence to saving, not viewing

	// initialize some image header params
	if ( CSBIGCam::GetCCDTemperature(ccdTemp) != CE_NO_ERROR )
		return CSBIGCam::GetError();
	pImg->setCameraID(this->getSerialNum());
	pImg->setFrameNum(m_iFrame++);
	pImg->SetCCDTemperature(ccdTemp);
	//command returns: "xxxmm,f28", where xxx=foclen
	if ((m_cAdapter.runCommand(LC_IDENTIFY_LENS, lens_ret_str)) == LE_NO_ERROR){
		foc_str = lens_ret_str.substr(0,3);
		foclen = atof(foc_str.c_str());
		pImg->SetFocalLength(foclen);
	}
	if ((m_cAdapter.runCommand(LC_GET_FOCUS_POSITION, lens_ret_str)) == LE_NO_ERROR){
		focpos = atof(lens_ret_str.c_str());
		pImg->SetApertureArea(focpos); // change header to put focus position where "Aperture" used to be
	}
	pImg->SetEachExposure(CSBIGCam::GetExposureTime());
	pImg->SetEGain(hex2double(gcir.readoutInfo[rm].gain));
	pImg->SetPixelHeight(hex2double(gcir.readoutInfo[rm].pixelHeight) * vertNBinning / 1000.0);
	pImg->SetPixelWidth(hex2double(gcir.readoutInfo[rm].pixelWidth) / 1000.0);
	es = ES_DCS_ENABLED | ES_DCR_DISABLED | ES_AUTOBIAS_ENABLED;
	if ( CSBIGCam::GetCameraType() == ST5C_CAMERA )
		es |= (ES_ABG_CLOCKED | ES_ABG_RATE_MED);
	else if ( CSBIGCam::GetCameraType() == ST237_CAMERA )
		es |= (ES_ABG_CLOCKED | ES_ABG_RATE_FIXED);
	else if ( CSBIGCam::GetActiveCCD() == CCD_TRACKING )
		es |= (ES_ABG_CLOCKED | ES_ABG_RATE_MED);
	else
		es |= ES_ABG_LOW;
	pImg->SetExposureState(es);
	pImg->SetExposureTime(CSBIGCam::GetExposureTime());
	pImg->SetNumberExposures(1);
	pImg->SetReadoutMode(CSBIGCam::GetReadoutMode());
	s = GetCameraTypeString();
	if ( CSBIGCam::GetCameraType() == ST5C_CAMERA || ( CSBIGCam::GetCameraType() == ST237_CAMERA && s.find("ST-237A", 0) == string::npos) )
		pImg->SetSaturationLevel(4095);
	else
		pImg->SetSaturationLevel(65535);
	s = gcir.name;
	//add identifier to camera name (distinguish two cameras)
	ostringstream sout;
	string IDNum = pImg->getCameraID();
	//NNG sout << s << " #" << m_nUSBNum;
	sout << s << " #" << IDNum;
	pImg->SetCameraModel(sout.str());
	pImg->SetBinning(hBin, vBin);
	pImg->SetSubFrame(left, top);
	
	// end any exposure in case one in progress
	CSBIGCam::EndExposure();
	if ( CSBIGCam::GetError() != CE_NO_ERROR && CSBIGCam::GetError() != CE_NO_EXPOSURE_IN_PROGRESS )
		return CSBIGCam::GetError();
	
	// start the exposure
#if MYCAM_DEBUG
	cout << "[MyCam debug]: starting 1st exposure (may not be a 2nd)." << endl;
#endif	
#if MYCAM_TIMING
	gettimeofday(&reference2, NULL);
	elapsed = (reference2.tv_sec - reference.tv_sec)*1000000 + reference2.tv_usec - reference.tv_usec;
	cout << "[MyCam timing]: time for all steps up to exposure: " << elapsed/1000.0 << "ms" << endl;
#endif
	gettimeofday(&reference, NULL);       //find a reference time before exposure, not part of debugging
	if ( CSBIGCam::StartExposure(dark == SBDF_LIGHT_ONLY ? SC_OPEN_SHUTTER : SC_CLOSE_SHUTTER) != CE_NO_ERROR )
		return CSBIGCam::GetError();
	//it turns out this command will typically be ~300ms after reference is taken
	pImg->SetImageStartTime(&reference);

	// wait for exposure to complete (sleep for most of the time, allow 5ms leeway)
// #if MYCAM_DEBUG
// 	cout << "[MyCam debug]: waiting for exposure, sleeping for: " 
// 		 << (int)(CSBIGCam::GetExposureTime()*1000000 - 25000)/1000.0 << "ms" << endl;
// #endif	
// 	usleep((int)(CSBIGCam::GetExposureTime()*1000000) - 25000);              //sleep for 10ms less than exposure time
	do {
	} while ((err = CSBIGCam::IsExposureComplete(expComp)) == CE_NO_ERROR && !expComp );  //spend remainder of exposure testing for end
	CSBIGCam::EndExposure();
	if ( err != CE_NO_ERROR )
		return err;
	if ( CSBIGCam::GetError() != CE_NO_ERROR )
		return CSBIGCam::GetError();
#if MYCAM_TIMING
	gettimeofday(&reference2, NULL);
	elapsed = (reference2.tv_sec - reference.tv_sec)*1000000 + reference2.tv_usec - reference.tv_usec;
	cout << "[MyCam timing]: time for exposure: " << elapsed/1000.0 << "ms" << endl;
	expTime = elapsed/1000.0;
	gettimeofday(&reference, NULL);       //reset reference time
#endif
	
	// readout the CCD
	srp.ccd = CSBIGCam::GetActiveCCD();
	srp.left = left;
	srp.top = top;
	srp.height = height;
	srp.width = width;
	srp.readoutMode = CSBIGCam::GetReadoutMode();
#if MYCAM_DEBUG
	cout << "[MyCam debug]: starting to read out CCD" << endl;
#endif	
	if ( (err = CSBIGCam::StartReadout(srp)) == CE_NO_ERROR ) {
		rlp.ccd = CSBIGCam::GetActiveCCD();
		rlp.pixelStart = left;
		rlp.pixelLength = width;
		rlp.readoutMode = CSBIGCam::GetReadoutMode();
		for (i=0; i<height && err==CE_NO_ERROR; i++ )
			err = CSBIGCam::ReadoutLine(rlp, FALSE, pImg->GetImagePointer() + (long)i * width);
	}
	CSBIGCam::EndReadout();
	if ( err != CE_NO_ERROR )
		return err;
	if ( CSBIGCam::GetError() != CE_NO_ERROR )
		return CSBIGCam::GetError();
	pImg->setChanged(true);
#if MYCAM_TIMING
	gettimeofday(&reference2, NULL);
	elapsed = (reference2.tv_sec - reference.tv_sec)*1000000 + reference2.tv_usec - reference.tv_usec;
	cout << "[MyCam timing]: time for readout: " << elapsed/1000.0 << "ms" << endl;
	readTime = elapsed/1000.0;
	gettimeofday(&reference, NULL);       //reset reference time
#endif

	// we're done unless we wanted a dark also image
	if ( dark == SBDF_DARK_ALSO ) {
	// start the light exposure
#if MYCAM_DEBUG
		cout << "[MyCam debug]: starting 2nd exposure." << endl;
#endif	
		gettimeofday(&reference, NULL);
		if ( CSBIGCam::StartExposure(SC_OPEN_SHUTTER) != CE_NO_ERROR )
			return CSBIGCam::GetError();
		pImg->SetImageStartTime(&reference);
			
		// wait for exposure to complete (sleep for most of the time, allow 5ms leeway)
#if MYCAM_DEBUG
		cout << "[MyCam debug]: waiting for exposure, sleeping for: " 
		 	 << (int)(CSBIGCam::GetExposureTime()*1000000 - 10000)/1000.0 << "ms" << endl;
#endif	
		usleep((int)(CSBIGCam::GetExposureTime()*1000000) - 10000);
		do {
		} while ((err = CSBIGCam::IsExposureComplete(expComp)) == CE_NO_ERROR && !expComp );
		CSBIGCam::EndExposure();
		if ( err != CE_NO_ERROR )
			return err;
		if ( CSBIGCam::GetError() != CE_NO_ERROR )
			return CSBIGCam::GetError();
		// readout the CCD
			
		srp.ccd = CSBIGCam::GetActiveCCD();
		srp.left = left;
		srp.top = top;
		srp.height = height;
		srp.width = width;
		srp.readoutMode = CSBIGCam::GetReadoutMode();
		if ( (err = CSBIGCam::StartReadout(srp)) == CE_NO_ERROR ) {
			rlp.ccd = CSBIGCam::GetActiveCCD();
			rlp.pixelStart = left;
			rlp.pixelLength = width;
			rlp.readoutMode = CSBIGCam::GetReadoutMode();
			for (i=0; i<height && err==CE_NO_ERROR; i++ )
				err = CSBIGCam::ReadoutLine(rlp, TRUE, pImg->GetImagePointer() + (long)i * width);
		}
		CSBIGCam::EndReadout();
		if ( err != CE_NO_ERROR )
			return err;
		if ( CSBIGCam::GetError() != CE_NO_ERROR )
			return CSBIGCam::GetError();
		pImg->setChanged(true);

		// record dark subtraction in history
		if ( CSBIGCam::GetCameraType() == ST5C_CAMERA || CSBIGCam::GetCameraType() == ST237_CAMERA )
			pImg->SetHistory("f");
		else
			pImg->SetHistory("R");
	}
#if MYCAM_TIMING
	gettimeofday(&reference2, NULL);
	gettimeofday(&grabstop, NULL);
	elapsed = (reference2.tv_sec - reference.tv_sec)*1000000 + reference2.tv_usec - reference.tv_usec;
	cout << "[MyCam timing]: time for second exposure (as applicable): " << elapsed/1000.0 << "ms" << endl;
	unsigned int duration = (grabstop.tv_sec - grabstart.tv_sec)*1000000 + grabstop.tv_usec - grabstart.tv_usec;
	cout << "[MyCam timing]: grab image took total time of " << duration/1000.0 << "ms" << endl;
	totalTime = duration/1000.0;
	//don't write timing data to a file any more
//	ofstream fout("/home/steve/starcam/thesis/sources/timing.txt", ios::out | ios::app);
//	if (!fout) cerr << "Error: failed to open time recording file";
//	fout << expTime << "\t\t" << readTime << "\t\t" << totalTime << endl;
//	fout.close();
#endif

	return CE_NO_ERROR;	
	
}

/*
  
	StartExposure:
		
	Start an exposure in the camera.  Should be matched
	with an EndExposure call.
	overrides CSBIGCam routine to try and add speed improvements
	
*/
PAR_ERROR MyCam::StartExposure(SHUTTER_COMMAND shutterState)
{
	StartExposureParams sep;
	
//	sep.ccd = CSBIGCam::GetActiveCCD();
	//adding START_SKIP_VDD should speed up the process, but may result in a glow on the side of the image
	sep.ccd = CSBIGCam::GetActiveCCD() + START_SKIP_VDD;
	sep.exposureTime = (unsigned long)(CSBIGCam::GetExposureTime() * 100.0 + 0.5 + EXP_SEND_TRIGGER_OUT);
	if ( sep.exposureTime < 1 )
		sep.exposureTime = 1;
	sep.abgState = CSBIGCam::GetABGState();
	
	sep.openShutter = shutterState;
	//can try manually opening the shutter, and the leaving the state the same in CC_START_EXPOSURE
	//this would probably be best if coupled with not closing the shutter diring readout (may screw things up)
// 	sep.openShutter = SC_LEAVE_SHUTTER;
// 	MiscellaneousControlParams mcp;
// 	mcp.fanEnable=TRUE;
// 	mcp.shutterCommand=SC_OPEN_SHUTTER;
// 	mcp.ledState=LED_ON;
// 	if (CSBIGCam::SBIGUnivDrvCommand(CC_MISCELLANEOUS_CONTROL, &mcp, NULL) != CE_NO_ERROR)
// 		return CSBIGCam::GetError();	
	
	if ( CSBIGCam::CheckLink() )
		return CSBIGCam::SBIGUnivDrvCommand(CC_START_EXPOSURE, &sep, NULL);
	else
		return CSBIGCam::GetError();
}

/*
  
	EndExposure:
		
	End or abort an exposure in the camera.  Should be
	matched to a StartExposure but no damage is done
	by calling it by itself if you don't know if an
	exposure was started for example.
	overrides CSBIGCam routine to try and add speed improvements
	
*/
PAR_ERROR MyCam::EndExposure(void)
{
	EndExposureParams eep;
	
	eep.ccd = CSBIGCam::GetActiveCCD();
	//can try skipping wait for mottor to stop (should be coupled with manual motor control above)
// 	eep.ccd = CSBIGCam::GetActiveCCD() + END_SKIP_DELAY;
	
	if ( CSBIGCam::CheckLink() )
		return CSBIGCam::SBIGUnivDrvCommand(CC_END_EXPOSURE, &eep, NULL);
	else
		return CSBIGCam::GetError();
}

string MyCam::getSerialNum()
{
	GetCCDInfoParams gcip;
	GetCCDInfoResults2 gcir;

	gcip.request = 2;  //extended info
	
	if ( CSBIGCam::SBIGUnivDrvCommand(CC_GET_CCD_INFO, &gcip, &gcir) == CE_NO_ERROR )
		return (string)gcir.serialNumber;
	else return "unknown";
}
