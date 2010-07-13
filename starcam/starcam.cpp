#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <cstdlib>
#include <cstdarg>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <sys/io.h>
#include "mycam.h"
#include "blobimage.h"
#include "frameblob.h"
#include "bloblist.h"
#include "camcommunicator.h"
#include "camcommserver.h"
#include "sbigudrv.h"
#include "csbigimgdefs.h"
#include "camconfig.h"
//#include "clensadapter.h"

using namespace std;

//enable display of debugging info to console
#define STARCAM_DEBUG 1
#define SC_THREAD_DEBUG 0

//enable use of image viewer window
//this no longer starts the window, only writes a file for it to use
#define USE_IMAGE_VIEWER 1

//0=no saving, 1=save raw image, 2=save version with boxes too
#define SAVE_SC_IMAGES 0

//camera and image objects (and associated mutexes)
MyCam globalCam;
BlobImage globalImages[2];                //one image can be processed while other is obtained
pthread_mutex_t camLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t imageLock[2] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};
pthread_mutex_t viewerLock = PTHREAD_MUTEX_INITIALIZER;

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
const char* viewerPath = "/data/etc/current.sbig";
bool showBoxes = false;
#endif

//function declarations
void* pictureLoop(void* arg);
void* processingLoop(void* arg);
void* startCommunications(void* arg);
string interpretCommand(string cmd);
void powerCycle();
void lock(pthread_mutex_t* mutex, const char* lockname, const char* funcname);
void unlock(pthread_mutex_t* mutex, const char* lockname, const char* funcname);
void wait(pthread_cond_t* cond, pthread_mutex_t* mutex, const char* condname, 
    const char* lockname, const char* funcname);
int initCommands();
int maintainInitFile(string cmd, string val);

//any command that changes starcam from its default state will be stored here to
//be executed at start time and resume previous operating state
const char* initFilename = "/data/etc/init.txt";
const char* badpixFilename = "/data/etc/badpix.txt";
const string adapterPath = "/dev/ttyACM0";
const string imgPath = "/data/rawdir";     //path to save images in

//logging function to clean up a bunch of the messy #ifs
enum loglevel {debug, data, info, warning, error};
void sclog(loglevel level, char* fmt, ...)
{
  char msg[1024];
  va_list argptr;
  va_start(argptr, fmt);
  vsnprintf(msg, 1024, fmt, argptr);
  va_end(argptr);

  switch (level) {
    case debug:
    case data:
#if STARCAM_DEBUG
      cerr << "[Starcam debug]: " << msg << endl;
#endif
      break;
    case info:
      cout << "--Starcam--" << msg << endl;
      break;
    case warning:
      cerr << "==Starcam Warning==" << msg << endl;
      break;
    case error:
      cerr << "**Starcam ERROR**" << msg << endl;
      break;
  }
}

PAR_ERROR openCameras()
{
  PAR_ERROR err;
  sclog(info, "Opening the camera driver.");
  if ((err = globalCam.OpenDriver()) != CE_NO_ERROR)
    return err;

  //query the usb bus for possible cameras..for some reason screws up the next step, comment out when not needed
   	cout << "[Starcam debug]: Querying the USB bus for usable devices..." << endl;
   	QueryUSBResults qur;  //results structure
   	if ((err= globalCam.QueryUSB(qur)) != CE_NO_ERROR)
   		return err;
   	cout << "...search for a camera was " << ((qur.camerasFound)?"successful":"unsuccessful") << endl;

  sclog(info, "Opening the camera device.");
  if ((err = globalCam.OpenUSBDevice(0)) != CE_NO_ERROR)
    return err;
  sclog(info, "Attempting to establish link to camera.");
  for (int i=0; i<6; i++) {
    sclog(info, "     ...attempt %d", i);
    if ((err = globalCam.EstablishLink()) == CE_NO_ERROR) break;
  }
  if (err != CE_NO_ERROR)
    return err;

  return CE_NO_ERROR;
}

int main(int argc, char *argv[])
{
  //set up the camera
  PAR_ERROR err;
  LENS_ERROR lerr;

  if ((err = openCameras()) != CE_NO_ERROR)
    return err;

  sclog(info, "Opening a connection to the lens adapter...");
  if ((lerr = globalCam.getLensAdapter()->openConnection(adapterPath)) != LE_NO_ERROR)
    return lerr;

  //initialize the images
  for (int i=0; i<2; i++) {
    globalImages[i].setBadpixFilename(badpixFilename);
    globalImages[i].getFrameBlob()->load_badpix(badpixFilename);
  }

  //run the stored initialization commands
  if (initCommands() < 0) return -1;

  //set up communications...really easy now
  CamCommServer comm;

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
  pthread_create(&threads[2], &scheduleAttr, &startCommunications, (void*)&comm);

  sclog(info, "Waiting for threads to complete...ie never");
  //wait for threads to return
  for (int i=0; i<4; i++) 
    pthread_join(threads[i], NULL);

  // shut down cameras if threads return (an error has occured)
  sclog(info, "All threads have returned...there are errors somehwere...go find them");

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
  sclog(info, "Starting up picture loop");
  int imageIndex = 0;                //index in image array to use
  int interval;                      //local storage of picture interval (in us)
  static PAR_ERROR err = CE_NO_ERROR;   //static enum lets a pointer to it be returned
  int failureCount = 0;
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
      failureCount++;
      if (failureCount == 3) {
	sclog(error, "pictureLoop: repeated errors: %s, restarting camera.", globalCam.GetErrorString(err).c_str());
	powerCycle();
      }
      else if (failureCount > 5) {
	sclog(error, "pictureLoop: too many repeated errors: %s", globalCam.GetErrorString(err).c_str());
	exit(1);
      }
      else sclog(warning, "pictureLoop: error: %s", globalCam.GetErrorString(err).c_str());
    }
    else failureCount = 0;
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
  sclog(info, "Starting up image processing loop");
  int imageIndex = 0;
  static SBIG_FILE_ERROR err;
  CamCommServer* comm = (CamCommServer*)arg;

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
#if SAVE_SC_IMAGES
    sclog(debug, "processingLoop: Saving image in: %s", imgPath.c_str());
    err = globalImages[imageIndex].SaveImageIn(imgPath, 0);
    if (err != SBFE_NO_ERROR) {
      sclog(warning, "processingLoop: File error during boxless write: %d", err);
    }
#endif
#if USE_IMAGE_VIEWER
    if (!showBoxes) {
      sclog(debug, "processingLoop: Saving viewer image in: %s", viewerPath);
      err = globalImages[imageIndex].SaveImage(viewerPath);
      if (err != SBFE_NO_ERROR) {
	sclog(warning, "processingLoop: File error during viewer write: %d", err);
      }
    }
#endif

    //process the image
    globalImages[imageIndex].findBlobs();
    frameblob* fblob = globalImages[imageIndex].getFrameBlob();
    bloblist* blobs;
    if (STARCAM_DEBUG || showBoxes || SAVE_SC_IMAGES == 2) {  //add boxes or output blob data
      sclog(data, "processingLoop: Found %d blobs.", fblob->get_numblobs());
      sclog(data, "processingLoop: Their locations (x,y) are: ");
      if (fblob->get_numblobs()) {
	blobs = fblob->getblobs();
	while (blobs != NULL) {
	  sclog(data, "processingLoop:   ...(%g,%g)", blobs->getx(), blobs->gety());
	  if (showBoxes || SAVE_SC_IMAGES == 2)
	    globalImages[imageIndex].drawBox(blobs->getx(), blobs->gety(), 20);
	  blobs = blobs->getnextblob();
	}
      }
    }
#if SAVE_SC_IMAGES == 2
    sclog(debug, "processignLoop: Saving image with boxes in: %s", imgPath.c_str());
    err = globalImages[imageIndex].SaveImageIn(imgPath, 1);
    if (err != SBFE_NO_ERROR) {
      sclog(warning, "processingLoop: File error during box write: %d", err);
    }
#endif
#if USE_IMAGE_VIEWER
    if (showBoxes) {
      sclog(debug, "processingLoop: Saving viewer image in: %s", viewerPath);
      err = globalImages[imageIndex].SaveImage(viewerPath);
      if (err != SBFE_NO_ERROR) {
	sclog(warning, "processingLoop: File error during boxed viewer write: %d", err);
      }
    }
#endif

    //send a return value to the flight computer
    sclog(debug, "processingLoop: sending image return value.");
    StarcamReturn returnStruct;
    globalImages[imageIndex].createReturnStruct(&returnStruct);
    comm->sendAll(CamCommunicator::buildReturn(&returnStruct));
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
 * startCommunications:
 *
 * starts the server that listens for new socket connections
 * the interpretCommand function is used by the server to interpret any 
 * received commands
 */
void* startCommunications(void* arg)
{
  sclog(info, "Starting to listen for communications");
  CamCommServer* comm = (CamCommServer*)arg;
  while (1) {
    comm->startServer(&interpretCommand);
    sclog(error, "Communications server failed to start.");
    sleep(1);
  }
  return NULL;
}

/*

interpretCommand:

interprets a command
this function is passed as a pointer to CamCommServer::startServer
returns a string to be sent back to flight computer
when unsuccessful, the return string should start with "Error:"
when successful, return "<command echo> successful"

any part of the public interface of one of the global objects can be exposed
to flight computer by adding an entry here

*/
string interpretCommand(string cmd)
{
  sclog(info, "Interpreting command: \"%s\"", cmd.c_str());
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
      //also don't allow processing to take place
      lock(&imageLock[0], "imageLock", "interpretCommand");
      lock(&imageLock[1], "imageLock", "interpretCommand");
      BlobImage img;
#if USE_IMAGE_VIEWER
      err = globalCam.autoFocus(&img, 0, viewerPath);
#else
      err = globalCam.autoFocus(&img, 0, NULL);
#endif
      unlock(&imageLock[1], "imageLock", "interpretCommand");
      unlock(&imageLock[0], "imageLock", "interpretCommand");
      unlock(&camLock, "camLock", "interpretCommand");
      if (err != LE_NO_ERROR)
	return (string)"Error: Autofocus returned: " + CLensAdapter::getErrorString(err);
      else return (cmd + " successful");
    }
    else if (cmd == "CtrigFocusF") {     //trigger forced move autofocus
      LENS_ERROR err;
      lock(&camLock, "camLock", "interpretCommand");
      //also don't allow processing to take place
      lock(&imageLock[0], "imageLock", "interpretCommand");
      lock(&imageLock[1], "imageLock", "interpretCommand");
      BlobImage img;
#if USE_IMAGE_VIEWER
      err = globalCam.autoFocus(&img, 1, viewerPath);
#else
      err = globalCam.autoFocus(&img, 1, NULL);
#endif
      unlock(&imageLock[1], "imageLock", "interpretCommand");
      unlock(&imageLock[0], "imageLock", "interpretCommand");
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
    else if (cmd == "CsetFocRnge") {           //set focus range
      if (valStr == "" || valStr == " ")
	return (string)"Error: the command " + cmd + " requires a value";
      unsigned int range;
      sin >> range;
      if (range == 0) return "Error: focus range must be nonzero.";
      lock(&camLock, "camLock", "interpretCommand");
      globalCam.setFocusRange(range);
      unlock(&camLock, "camLock", "interpretCommand");
      if (maintainInitFile(cmd, valStr) == 0)
	return (cmd + " successful");
      else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
    }
    else if (cmd == "Cpower") {
      lock(&camLock, "camLock", "interpretCommand");
      powerCycle();
      unlock(&camLock, "camlock", "interpretCommand");
      return (cmd + " successful");
    }
    else {
      sclog(warning, "interpretCommand: bad camera command");
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
    else {
      sclog(warning, "interpretCommand: bad image command");
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
      sclog(data, "interpretCommand: Lens move of: %d counts has %d left", counts, remaining);
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
      sclog(data, "interpretCommand: Forced lens move of: %d counts has %d left", counts, remaining);
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
      sclog(data, "interpretCommand: Lens command: \"%s\" returned \"%s\"", cmd.c_str(), returnVal.c_str());
      return cmd + " returned: " + returnVal;
    }
    else {
      sclog(warning, "interpretCommand: bad lens command");
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
	<< globalCam.getFocusRange() << " "
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
    if (cmd == "OshowBox") {
#if USE_IMAGE_VIEWER
      if (valStr == "" || valStr == " ")
	return (string)"Error: the command " + cmd + " requires a value.";
	sin >> showBoxes;
      if (maintainInitFile(cmd, valStr) == 0)
	return (cmd + " successful");
      else return (string)"Error: " + cmd + "=" + valStr + " failed to update init file";
#else
      return (cmd + "successful, not implemented");
#endif
    }
    else {
      sclog(warning, "interpretCommand: bad overall command");
      return (string)"Error: failed to parse overall command: " + cmd;
    }
  }
  else {
    sclog(warning, "interpretCommand: Unknown device specifier: '%c'", cmd[0]);
    return (string)"Error: Unknown device specifier (1st char)" + cmd.substr(0,1);
  }

  return "Error: execution should never reach here!!";
}

/*
 * powerCycle:
 *
 * uses a switch on the parallel port to power cycle the star cameras
 */
void powerCycle()
{
  static bool hasperms = false;

  if (!hasperms) {
    if (ioperm(0x378, 0x0F, 1) != 0) {
      sclog(warning, "powerCycle couldn't set port permissions...are you root?");
      return;
    }
    else hasperms = true;
  }

  sclog(info, "Power cycling the cameras!");
  if (globalCam.CloseDevice() != CE_NO_ERROR)
    sclog(warning, "Trouble safely shutting down camera, proceeding anyway.");
  if (globalCam.CloseDriver() != CE_NO_ERROR)
    sclog(warning, "Trouble safely shutting down driver");

  outb(0xFF, 0x378);
  usleep(1000000);
  outb(0x00, 0x378);

  PAR_ERROR err;
  sleep(3);
  while ((err = openCameras()) != CE_NO_ERROR) {
    sclog(error, "Problem reconnecting to camera: %s", globalCam.GetErrorString(err).c_str());
    sleep(5);
  }
  sclog(info, "Star camera reconnected");
}

/*

lock:

simply calls pthread_mutex_lock, but provides debugging information

*/
void lock(pthread_mutex_t* mutex, const char* lockname, const char* funcname)
{
#if SC_THREAD_DEBUG
  cout << "[Starcam debug]: " << funcname << ": Waiting for " << lockname << endl;
#endif
  pthread_mutex_lock(mutex);
#if SC_THREAD_DEBUG
  cout << "[Starcam debug]: " << funcname << ": Obtained " << lockname << endl;
#endif
}

/*

unlock:

simply calls pthread_mutex_unlock but provides debugging information

*/
void unlock(pthread_mutex_t* mutex, const char* lockname, const char* funcname)
{
#if SC_THREAD_DEBUG
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
#if SC_THREAD_DEBUG
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
  sclog(info, "Running stored initialization commands");
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
    sclog(debug, "init command returned: %s", retVal.c_str());
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
