#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include "mycam.h"
#include "blobimage.h"
#include "frameblob.h"
#include "bloblist.h"
#include "camcommunicator.h"
#include "sbigudrv.h"
#include "csbigimgdefs.h"
#include "camconfig.h"
//#include "clensadapter.h"

using namespace std;

//enable display of debugging info to console
#define STARCAM_DEBUG 1
//enable use of image viewer window
#define USE_IMAGE_VIEWER 1

//global variables

//camera and image objects (and associated mutexes)
MyCam globalCam;
BlobImage globalImages[2];                //one image can be processed while other is obtained
pthread_mutex_t camLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t imageLock[2] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};

//flags, etc. for synchronizing image exposure and processing
int globalImageReadyFlags[2] = {0, 0};    // 1 when an image is ready, 0 when processing complete
pthread_mutex_t imageReadyLock[2] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};
pthread_cond_t imageReadyCond[2] = {PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER};
pthread_cond_t processingCompleteCond[2] = {PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER};

//camera trigger flags, etc.
int globalCameraTriggerFlag = 0;
pthread_mutex_t cameraTriggerLock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cameraTriggerCond = PTHREAD_COND_INITIALIZER;

//image viewer
#if USE_IMAGE_VIEWER
#include <qapplication.h>
#include "imageviewer.h"
//ImageViewer *viewer;
void* viewer;       //needs to be cast as ImageViewer*
bool showBoxes = false;  //TODO make commandable
#endif

//function declarations
void* pictureLoop(void* arg);
void* processingLoop(void* arg);
void* readLoop(void* arg);
// void* displayLoop(void* arg);
string interpretCommand(string cmd);
void lock(pthread_mutex_t* mutex, const char* lockname, const char* funcname);
void unlock(pthread_mutex_t* mutex, const char* lockname, const char* funcname);
void wait(pthread_cond_t* cond, pthread_mutex_t* mutex, const char* condname, 
		  const char* lockname, const char* funcname);
int initCommands();
int maintainInitFile(string cmd, string val);

//any command that changes starcam from its default state will be stored here to
//be executed at start time and resume previous operating state
const char* initFilename = "/usr/local/starcam/init.txt";
const char* badpixFilename = "/usr/local/starcam/badpix.txt";
const string adapterPath = "/dev/ttyACM0";
const string commTarget = "nogrod.spider";
//const string commTarget = "parker.astro.utoronto.ca"
const string imgPath = "/usr/local/starcam/pictures";     //path to save images in

int main(int argc, char *argv[])
{
	//set up the camera
	PAR_ERROR err;
	LENS_ERROR lerr;
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Opening the camera driver." << endl;
#endif
	if ((err = globalCam.OpenDriver()) != CE_NO_ERROR)
		return err;
	
	//query the usb bus for possible cameras..for some reason screws up the next step, comment out when not needed
// 	cout << "[Starcam debug]: Querying the USB bus for usable devices..." << endl;
// 	QueryUSBResults qur;  //results structure
// 	if ((err= globalCam.QueryUSB(qur)) != CE_NO_ERROR)
// 		return err;
// 	cout << "...search for a camera was " << ((qur.camerasFound)?"successful":"unsuccessful") << endl;
	
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Opening the camera device." << endl;
#endif
	if ((err = globalCam.OpenUSBDevice(0)) != CE_NO_ERROR)
		return err;
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Attempting to establish link to camera." << endl;
#endif
	for (int i=0; i<6; i++) {
#if STARCAM_DEBUG
		cout << "[Starcam debug]:      ...attempt " << i << endl;
#endif
		if ((err = globalCam.EstablishLink()) == CE_NO_ERROR) break;
	}
	if (err != CE_NO_ERROR)
		return err;

#if STARCAM_DEBUG
	cout << "[Starcam debug]: Opening a connection to the lens adapter..." << endl;
#endif
 	if ((lerr = globalCam.getLensAdapter()->openConnection(adapterPath)) != LE_NO_ERROR)
 		return lerr;
		
#if USE_IMAGE_VIEWER
	//set up image viewer applications
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Setting up viewer window..." << endl;
#endif
	QApplication a(argc, argv);
	viewer = (void*)new ImageViewer(defaultImageParams.viewerParams.width, defaultImageParams.viewerParams.height,
							 defaultImageParams.viewerParams.refreshTime, 0, "viewer");
	a.setMainWidget((ImageViewer*)viewer);
	((ImageViewer*)viewer)->show();
#endif
	
	//initialize the images
	for (int i=0; i<2; i++) {
		globalImages[i].setBadpixFilename(badpixFilename);
		globalImages[i].getFrameBlob()->load_badpix(badpixFilename);
	}
	
	//run the stored initialization commands
	if (initCommands() < 0) return -1;
	
	//set up communications...will block until other end opens
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Opening command connection...will block until other end opens." << endl;
#endif
	CamCommunicator comm;
	if (comm.openClient(commTarget) < 0) 
		return -1;
	
	//start threads for picture taking, image processing, and command reading
	pthread_t threads[3];
	pthread_attr_t scheduleAttr;
	struct sched_param scheduleParams;
	pthread_attr_init(&scheduleAttr);
	pthread_attr_getschedparam(&scheduleAttr, &scheduleParams);
	scheduleParams.sched_priority++;
	if (pthread_attr_setschedparam(&scheduleAttr, &scheduleParams) != 0) return -1;
	pthread_create(&threads[0], &scheduleAttr, &pictureLoop, NULL);
	scheduleParams.sched_priority--;   //run processing at default priority, lower than pictures
	if (pthread_attr_setschedparam(&scheduleAttr, &scheduleParams) != 0) return -1;
	pthread_create(&threads[1], &scheduleAttr, &processingLoop, (void*)&comm);
	scheduleParams.sched_priority += 2;        //run command reading at highest priority
	if (pthread_attr_setschedparam(&scheduleAttr, &scheduleParams) != 0) return -1;
	pthread_create(&threads[2], &scheduleAttr, &readLoop, (void*)&comm);
	
// 	pthread_create(&threads[0], NULL, &pictureLoop, NULL);
// 	pthread_create(&threads[1], NULL, &processingLoop, (void*)&comm);
// 	pthread_create(&threads[2], NULL, &readLoop, (void*)&comm);

#if USE_IMAGE_VIEWER
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Executing viewer application window" << endl;
#endif
	a.exec();
#endif
	
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Waiting for threads to complete...ie never" << endl;
#endif
	//wait for threads to return
	for (int i=0; i<4; i++) 
		pthread_join(threads[i], NULL);

	// shut down cameras if threads return (an error has occured)
#if STARCAM_DEBUG
	cout << "[Starcam debug]: All threads have returned...there are errors somehwere...go find them" << endl;
#endif

#if USE_IMAGE_VIEWER
	delete (ImageViewer*)viewer;
#endif

	if ((err = globalCam.CloseDevice()) != CE_NO_ERROR )
		return err;
	if ( (err = globalCam.CloseDriver()) != CE_NO_ERROR )
		return err;
	
	return EXIT_SUCCESS;
}

/*

 pictureLoop:
 
 Takes pictures in an infinite loop in response to either a timer or a trigger
 Designed to run concurrently with processingLoop and readLoop
 Might have to wait for image processing in progress to complete
 
 */
void* pictureLoop(void* arg)
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Inside pictureLoop." << endl;
#endif
	int imageIndex = 0;                //index in image array to use
	int interval;                      //local storage of picture interval (in us)
	static PAR_ERROR err = CE_NO_ERROR;   //static enum lets a pointer to it be returned
	while (1) {
		
		//depending on picture interval, either sleep or wait for trigger
		lock(&camLock, "camLock", "pictureLoop");
			interval = globalCam.getPictureInterval() * 1000;
		unlock(&camLock, "camLock", "pictureLoop");
		if (interval == 0) {           //camera is in triggered mode, wait for trigger
			lock(&cameraTriggerLock, "cameraTriggerLock", "pictureLoop");
				while (!globalCameraTriggerFlag) {
// 					pthread_cond_wait(&cameraTriggerCond, &cameraTriggerLock);
					wait(&cameraTriggerCond, &cameraTriggerLock, "cameraTriggerCond",
						  "cameraTriggerLock", "pictureLoop");
				}
				globalCameraTriggerFlag = 0;      //unset flag before continuing
			unlock(&cameraTriggerLock, "cameraTriggerLock", "pictureLoop");
		}
		else {                        //camera is in timed mode
			usleep(interval);
		}
		
		//wait for image to finish being processed
		lock(&imageReadyLock[imageIndex], "imageReadyLock", "pictureLoop");
			while (globalImageReadyFlags[imageIndex] == 1) {
// 				pthread_cond_wait(&processingCompleteCond[imageIndex], 
// 								   &imageReadyLock[imageIndex]);
				wait(&processingCompleteCond[imageIndex], &imageReadyLock[imageIndex],
					"processingCompleteCond", "imageReadyLock", "pictureLoop");
			}
		unlock(&imageReadyLock[imageIndex], "imageReadyLock", "pictureLoop");
		
		//grab new image (lock camera so settings can't change during exposure)
		lock(&camLock, "camLock", "pictureLoop");
			err = globalCam.GrabImage(&globalImages[imageIndex], SBDF_LIGHT_ONLY);
			if (err != CE_NO_ERROR) {
#if STARCAM_DEBUG
				cout << "[Starcam debug]: pictureLoop: error: " << globalCam.GetErrorString(err) << endl;
#endif
				return (void*)&err;
			}
		unlock(&camLock, "camLock", "pictureLoop");
		
		//trigger processing of this image
		lock(&imageReadyLock[imageIndex], "imageReadyLock", "pictureLoop");
			globalImageReadyFlags[imageIndex] = 1;
		unlock(&imageReadyLock[imageIndex], "imageReadyLock", "pictureLoop");
		pthread_cond_signal(&imageReadyCond[imageIndex]);
		
		//move on to next image while this one is being processed
		imageIndex = (imageIndex == 0)?1:0;
	}
	
	return NULL;         //should never reach here
}

/*

 processingLoop:
 
 In an infinito loop processes images as they become available
 Designed to run concurrently with pictureLoop and readLoop
 
*/
void* processingLoop(void* arg)
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Inside processingLoop." << endl;
#endif
	int imageIndex = 0;
	static SBIG_FILE_ERROR err;
	CamCommunicator* comm = (CamCommunicator*)arg;    //enables returns to be sent
	
	while (1) {
		//wait for image to be available for processing
		lock(&imageReadyLock[imageIndex], "imageReadyLock", "processingLoop");
			while (globalImageReadyFlags[imageIndex] == 0) {
// 				pthread_cond_wait(&imageReadyCond[imageIndex], 
// 								   &imageReadyLock[imageIndex]);
				wait(&imageReadyCond[imageIndex], &imageReadyLock[imageIndex],
					"imageReadyCond", "imageReadyLock", "processingLoop");
			}
		unlock(&imageReadyLock[imageIndex], "imageReadyLock", "processingLoop");
		
		//process the image
		lock(&imageLock[imageIndex], "imageLock", "processingLoop");
			//save the file
			globalImages[imageIndex].AutoBackgroundAndRange();
#if USE_IMAGE_VIEWER
			((ImageViewer*)viewer)->load(&globalImages[imageIndex], FALSE);
#endif
			err = globalImages[imageIndex].SaveImageIn(imgPath, 0);
			if (err != SBFE_NO_ERROR) {
#if STARCAM_DEBUG
				cout << "[Starcam debug]: processingLoop: File error: " << err << endl;
#endif
				return (void *) &err;
			}
			
			//process the image
			globalImages[imageIndex].findBlobs();
			frameblob* fblob = globalImages[imageIndex].getFrameBlob();
			bloblist* blobs;
#if STARCAM_DEBUG
			cout << "[Starcam debug]: processingLoop: Found " << fblob->get_numblobs() 
				<< " blobs." << endl;
			if (fblob->get_numblobs()) {
				cout << "[Starcam debug]: processingLoop: Their locations (x,y) are " << endl;
				blobs = fblob->getblobs();
				while (blobs != NULL) {
					cout << "[Starcam debug]: processingLoop:       ...(" << blobs->getx() 
							<< "," << blobs->gety() << ")" << endl;
					globalImages[imageIndex].drawBox(blobs->getx(), blobs->gety(), 20, showBoxes);
					blobs = blobs->getnextblob();
				}
			}
			//in debug mode also save a file with boxes
			cout << "Saving image in: " << imgPath << endl;
			err = globalImages[imageIndex].SaveImageIn(imgPath, 1);
			if (err != SBFE_NO_ERROR) {
				cout << "[Starcam debug]: processingLoop: File error: " << err << endl;
				return (void*) &err;
			}
#endif
			
			//send a return value to the flight computer
#if STARCAM_DEBUG
			cout << "[Starcam debug]: processingLoop: sending image return value." << endl;
#endif
			StarcamReturn returnStruct;
			globalImages[imageIndex].createReturnStruct(&returnStruct);
			comm->sendReturn(&returnStruct);
		unlock(&imageLock[imageIndex], "imageLock", "processingLoop");
		
		//signal that processing is complete
		lock(&imageReadyLock[imageIndex], "imageReadyLock", "processingLoop");
			globalImageReadyFlags[imageIndex] = 0;
		unlock(&imageReadyLock[imageIndex], "imageReadyLock", "processingLoop");
		pthread_cond_signal(&processingCompleteCond[imageIndex]);
		
		
		imageIndex = (imageIndex == 0)?1:0;
	}
	return NULL;
}

/*

 readLoop:
 
 simply a wrapper function for calling the CamCommunicator::readLoop member in a pthread
 
*/
void* readLoop(void* arg)
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: Inside readLoop (handled by CamCommunicator class)." << endl;
#endif
	CamCommunicator* comm = (CamCommunicator*)arg;
	while (true) {     //when something bad happens, should try to start again
		comm->readLoop(&interpretCommand);
#if STARCAM_DEBUG
		cout << "[Starcam debug]: readLoop: an error occured (and readLoop ended)" << endl;
#endif
		if (comm->sendReturnString("Error: readLoop has ended. Trying to restart it") < 0)
		{ //something really bad has happened
#if STARCAM_DEBUG
			cout << "[Starcam debug]: readLoop: error is really bad, can't communicate" << endl;
#endif
			
		}
		//TODO should think about how errors should be handled
	}
	return NULL;
}

// void* displayLoop(void* arg)
// {
// 	QApplication *a = (QApplication *)arg;
// 	static int res = a->exec();
// 	return (void*)&res;
// }

/*

 interpretCommand:
 
 interprets a command obtained from readLoop
 this funciton is passed as a pointer to CamCommunicator::readLoop
 returns a string to be sent back to flight computer
 when unsuccessful, the return string should start with "Error:"
 when successful, return "<command echo> successful"
 
 any part of the public interface of one of the global objects can be exposed
 to flight computer by adding an entry here
 
*/
string interpretCommand(string cmd)
{
#if STARCAM_DEBUG
	cout << "\n\n[Starcam debug]: Inside interpretCommand with: \"" << cmd << "\"" << endl;
#endif
	//separate command part of cmd from the value part
	string::size_type valPos = cmd.find("=", 0);
	string valStr = "";
	istringstream sin;      //for reading value portion of command
	if (valPos != string::npos) {      //cmd contains a value part
		valStr = cmd.substr(valPos+1, cmd.length()-(valPos+1));
		sin.str(valStr);
		cmd = cmd.substr(0, valPos);
	}
	if (cmd[0] == 'C')         //command is for the camera
	{
		if (cmd == "CtrigExp") {       //trigger a camera exposure
			if (globalCam.getPictureInterval() != 0) 
				return "Error: Not in triggered-exposure mode";
			lock(&cameraTriggerLock, "cameraTriggerLock", "interpretCommand");
				globalCameraTriggerFlag = 1;
				pthread_cond_signal(&cameraTriggerCond);
			unlock(&cameraTriggerLock, "cameraTriggerLock", "interpretCommand");
			return (cmd + " successful");
		}
		else if (cmd == "CtrigFocus") {     //trigger autofocus
			LENS_ERROR err;
			lock(&camLock, "camLock", "interpretCommand");
				BlobImage img;
#if USE_IMAGE_VIEWER
				((ImageViewer*)viewer)->load(&img, TRUE);
				err = globalCam.autoFocus(&img, 0);
#else
				err = globalCam.autoFocus(&img, 0);
#endif
			unlock(&camLock, "camLock", "interpretCommand");
			if (err != LE_NO_ERROR)
				return (string)"Error: Autofocus returned: " + CLensAdapter::getErrorString(err);
			else return (cmd + " successful");
		}
		else if (cmd == "CtrigFocusF") {     //trigger forced move autofocus
			LENS_ERROR err;
			lock(&camLock, "camLock", "interpretCommand");
				BlobImage img;
#if USE_IMAGE_VIEWER
				((ImageViewer*)viewer)->load(&img, TRUE);
				err = globalCam.autoFocus(&img, 1);
#else
				err = globalCam.autoFocus(&img, 1);
#endif
			unlock(&camLock, "camLock", "interpretCommand");
			if (err != LE_NO_ERROR)
				return (string)"Error: Autofocus returned: " + CLensAdapter::getErrorString(err);
			else return (cmd + " successful");
		}
		else if (cmd == "CsetExpTime") {           //set exposure time
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			double expTime;
			sin >> expTime;
			expTime /= 1000.0; //changed to using ms
			lock(&camLock, "camLock", "interpretCommand");
				globalCam.SetExposureTime(expTime);
			unlock(&camLock, "camLock", "interpretCommand");
			if (maintainInitFile(cmd, valStr) == 0)
				return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
		else if (cmd == "CsetExpInt") {         //set exposure interval (0 for triggered)
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			//if previously in triggered mode, trigger any waiting exposure
			//when switching back to triggered mode, this may cause an extra exposure
			if (globalCam.getPictureInterval() == 0) {
				lock(&cameraTriggerLock, "cameraTriggerLock", "interpretCommand");
					globalCameraTriggerFlag = 1;
					pthread_cond_signal(&cameraTriggerCond);
				unlock(&cameraTriggerLock, "cameraTriggerLock", "interpretCommand");
			}
			int expInt;
			sin >> expInt;
			lock(&camLock, "camLock", "interpretCommand");
				globalCam.setPictureInterval(expInt);
			unlock(&camLock, "camLock", "interpretCommand");
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
		else if (cmd == "CsetFocRsln") {           //set focus resolution
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			unsigned int resolution;
			sin >> resolution;
			if (resolution == 0) return "Error: focus resolution must be nonzero.";
			lock(&camLock, "camLock", "interpretCommand");
				globalCam.setFocusResolution(resolution);
			unlock(&camLock, "camLock", "interpretCommand");
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
		else {
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Can't parse camera command: \"" 
				 << cmd << "\"" << endl;
#endif
			return (string)"Error: Failed to parse camera command: " + cmd;
 		}
	}
	else if (cmd[0] == 'I')       //command is for images
	{
	        //TODO this needs to be adapted for two cameras (one file for each camera)
		if (cmd == "IsetBadpix") {    //set a bad pixel, value should have form "x y"
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			int x, y, camnum;
			sin >> camnum >> x >> y;
			ofstream fout(badpixFilename, ios::out | ios::app);
			if (!fout) return "Error: failed to open bad pixel file";
			fout << x << " " << y << "\n";          //assume same entry won't be made multiple times
			fout.close();
			//reload bad pixels into map (in frameblob)
			for (int i=0; i<2; i++) {
				lock(&imageLock[i], "imageLock", "interpretCommand");
					globalImages[i].getFrameBlob()->load_badpix(badpixFilename);
				unlock(&imageLock[i], "imageLock", "interpretCommand");
			}
			return (cmd + " successful");
		}
		else if (cmd == "IsetMaxBlobs") {        //set maximum number of blobs that will be found
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			unsigned int maxblobs;
			sin >> maxblobs;
			for (int i=0; i<2; i++) {
				lock(&imageLock[i], "imageLock", "interpretCommand");
					globalImages[i].getFrameBlob()->set_maxblobs(maxblobs);
				unlock(&imageLock[i], "imageLock", "interpretCommand");
			}
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
		else if (cmd == "IsetGrid") {        //set grid size for blob detection
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			unsigned int grid;
			sin >> grid;
			for (int i=0; i<2; i++) {
				lock(&imageLock[i], "imageLock", "interpretCommand");
					globalImages[i].getFrameBlob()->set_grid(grid);
				unlock(&imageLock[i], "imageLock", "interpretCommand");
			}
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
		else if (cmd == "IsetThreshold") {        //set threshold (in # sigma) for blob detection
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			double threshold;
			sin >> threshold;
			for (int i=0; i<2; i++) {
				lock(&imageLock[i], "imageLock", "interpretCommand");
					globalImages[i].getFrameBlob()->set_threshold(threshold);
				unlock(&imageLock[i], "imageLock", "interpretCommand");
			}
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
		else if (cmd == "IsetDisttol") {        //set minimum distance squared (pixels) between blobs
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value.";
			int disttol;
			sin >> disttol;
			for (int i=0; i<2; i++) {
				lock(&imageLock[i], "imageLock", "interpretCommand");
					globalImages[i].getFrameBlob()->set_disttol(disttol);
				unlock(&imageLock[i], "imageLock", "interpretCommand");
			}
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		}
#if USE_IMAGE_VIEWER
		//this isnt' so needed anymore since now it's only a polling rate, not full refreshes
		else if (cmd == "IsetRefresh") {        //set viewer refresh period (in ms)
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value.";
			int msec;
			sin >> msec;
			((ImageViewer*)viewer)->setRefreshTime(msec);
			return (cmd + " successful");
		}
#endif
		else {
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Can't parse image command: \"" 
				 << cmd << "\"" << endl;
#endif
			return (string)"Error: failed to parse image command: " + cmd;
		}
	}
	else if (cmd[0] == 'L')       //command is for the lens
	{
		if (cmd == "Lmove") {       //make a precise (proportional feedback) move
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			int counts, remaining;
			sin >> counts;
			lock(&camLock, "camLock", "interpretCommand");
				LENS_ERROR err = globalCam.getLensAdapter()->preciseMove(counts, remaining, 0);
			unlock(&camLock, "camLock", "interpretCommand");
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Lens move of: " << counts
			 	 << " counts has: " << remaining << " counts left to move" << endl;
#endif
			if (err == LE_NO_ERROR) return (cmd + " successful");
			else return (string)"Error: Move returned " + CLensAdapter::getErrorString(err);
		} 
		else if (cmd == "Lforce") {       //make a forced move (ignore "false" stops)
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			int counts, remaining;
			sin >> counts;
			lock(&camLock, "camLock", "interpretCommand");
				LENS_ERROR err = globalCam.getLensAdapter()->preciseMove(counts, remaining, 1);
			unlock(&camLock, "camLock", "interpretCommand");
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Forced lens move of: " << counts
			 	 << " counts has: " << remaining << " counts left to move" << endl;
#endif
			if (err == LE_NO_ERROR) return (cmd + " successful");
			else return (string)"Error: Forced move returned " + CLensAdapter::getErrorString(err);
		} 
		else if (cmd == "LsetTol") {     //set tolerance of precise moves
			if (valStr == "" || valStr == " ")
				return (string)"Error: the command " + cmd + " requires a value";
			unsigned int tol;
			sin >> tol;
			lock(&camLock, "camLock", "interpretCommand");
				globalCam.getLensAdapter()->setFocusTol(tol);
			unlock(&camLock, "camLock", "interpretCommand");
			if (maintainInitFile(cmd, valStr) == 0)
			        return (cmd + " successful");
			else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
		} 
		else if (cmd == "L") {                 //try to send arbitrary lens command
			string returnVal;
			sin >> cmd;                        //read the lens command
			lock(&camLock, "camLock", "interpretCommand");
				globalCam.getLensAdapter()->runCommand(cmd, returnVal);
			unlock(&camLock, "camLock", "interpretCommand");
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Lens command: \"" << cmd << "\"" 
			 	 << "Returned: \"" << returnVal << "\"" << endl;
#endif
			return cmd + " returned: " + returnVal;
		}
		else {
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Can't parse lens command: \"" 
				 << cmd << "\"" << endl;
#endif
			return (string)"Error: failed to parse lens command: " + cmd;
		}
	}
	else if (cmd[0] == 'O')     //overall commands
	{
	  if (cmd == "Oconf") {   //send back current operating configuration
	    ostringstream sout;
	    sout << "<conf>";
	    lock(&camLock, "camLock", "interpretCommand");
	      sout << globalCam.getPictureInterval() << " "
		   << globalCam.GetExposureTime() << " "
		   << globalCam.getFocusResolution() << " "
		   << globalCam.getLensAdapter()->getFocusTol() << " ";
	    unlock(&camLock, "camLock", "interpretCommand");
	    lock(&imageLock[0], "imageLock", "interpretCommand");
	      sout << globalImages[0].getFrameBlob()->get_maxblobs() << " "
		   << globalImages[0].getFrameBlob()->get_grid() << " "
		   << globalImages[0].getFrameBlob()->get_threshold() << " "
		   << globalImages[0].getFrameBlob()->get_disttol();
	    unlock(&imageLock[0], "imageLock", "interpretCommand");
	    return sout.str();
	  }
	  else {
#if STARCAM_DEBUG
	    cout << "[Starcam debug]: interpretCommand: Can't parse overall command: \"" 
	      << cmd << "\"" << endl;
#endif
			return (string)"Error: failed to parse overall command: " + cmd;
	  }
	}
	else {
#if STARCAM_DEBUG
			cout << "[Starcam debug]: interpretCommand: Unknown device specifier: '" 
				 << cmd[0] << "'" << endl;
#endif
			return (string)"Error: Unknown device specifier (1st char)" + cmd.substr(0,1);
	}
	
	return "Error: execution should never reach here!!";
}

/*

 lock:
 
 simply calls pthread_mutex_lock, but provides debugging information
 
*/
void lock(pthread_mutex_t* mutex, const char* lockname, const char* funcname)
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: " << funcname << ": Waiting for " << lockname << endl;
#endif
	pthread_mutex_lock(mutex);
#if STARCAM_DEBUG
	cout << "[Starcam debug]: " << funcname << ": Obtained " << lockname << endl;
#endif
}

/*

 unlock:
 
 simply calls pthread_mutex_unlock but provides debugging information
 
*/
void unlock(pthread_mutex_t* mutex, const char* lockname, const char* funcname)
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: " << funcname << ": Releasing " << lockname << endl;
#endif
	pthread_mutex_unlock(mutex);
}

/*

 wait:
 
 simply calls pthread_cond_wait, but provides debuging information
 
*/
void wait(pthread_cond_t* cond, pthread_mutex_t* mutex, const char* condname, 
		  const char* lockname, const char* funcname)
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: " << funcname << ": Waiting for " << condname 
		 << " and releasing " << lockname << endl;
#endif
	pthread_cond_wait(cond, mutex);
}


/*

 initCommands:
 
 reads and executes commands stored in the init command file
 when an error occurs, -1 is returned otherwise returns 0
 
*/
int initCommands()
{
#if STARCAM_DEBUG
	cout << "[Starcam debug]: inside initCommands." << endl;
#endif
	//open file for reading...if it fails try creating it
	ifstream fin(initFilename, ios::in);
	if (!fin) {
		//try creating the file (by opening for writing)
		ofstream temp(initFilename, ios::out);
		if (!temp) return -1;
		temp.close();
		fin.open(initFilename, ios::in);
		if (!fin) return -1;
	}
	
	char buf[256];
	string cmd, retVal;
	
	while (!fin.eof())
	{
		fin.getline(buf, 256);
		cmd = buf;
		//check for empty file before running commands
		if (cmd == "") return 0;
		retVal = interpretCommand(cmd);
#if STARCAM_DEBUG
		cout << "[Starcam debug]: init command returned: " << retVal << endl;
#endif
		if (retVal.substr(0,5) == "Error")  {
			return -1;
		}
	}
	
	fin.close();
	return 0;
}

/*

 maintainInitFile:
 
 when commands are executed that change starcam state, they need to be stored
 so if program needs to restart it's state will be unchanged
 This function maintains the file so that it has the minimum necessary number of
 commands that need to be read and excuted.
 Returns -1 if an error occurs, 0 otherwise
 
*/
int maintainInitFile(string cmd, string val)
{
	//open file for reading...if it fails try creating it
	ifstream fin(initFilename, ios::in);
	if (!fin) {
		//try creating the file (by opening for writing)
		ofstream temp(initFilename, ios::out);
		if (!temp) return -1;
		temp.close();
		fin.open(initFilename, ios::in);
		if (!fin) return -1;
	}
	
	ostringstream sout;                    //what will be output to file at end
	int found = 0;                         //indicates if command was found in file
	char buf[256];
	string cmd_in;
	string::size_type valPos;              //position of command-value separator in commands
	
	//read in file contents and replace exisitng version of cmd if found
	while (!fin.eof())
	{
		fin.getline(buf, 256);
		cmd_in = buf;
		valPos = cmd_in.find("=", 0);
		if (valPos != string::npos) cmd_in = cmd_in.substr(0,valPos);
		if (cmd == cmd_in)
		{      //cmd was found in the file, only keep the new version
			sout << cmd << ((val.length()!=0)?"=":"") << val << "\n";
			found = 1;
		} else if (cmd_in != "") {
			sout << buf << "\n";
		}
	}
	fin.close();
	if (!found) sout << cmd << ((val.length()!=0)?"=":"") << val << "\n";
	
	//output the revised contents to file
	ofstream fout(initFilename, ios::out | ios::trunc);
	if (!fout) return -1;
	fout << sout.str();
	fout.close();
	
	return 0;
}
