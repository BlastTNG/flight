/* mcp: the Spider master control program
 *
 * starcamera.cpp: star camera control and readout functions
 *     this file is a logical part of tx.c
 *     uses cpp to make calling class functions easier
 *
 * This software is copyright (C) 2002-2007 University of Toronto
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <limits.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include <string>
#include <sstream>

extern "C" {
#include "blast.h"
#include "tx.h"
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "command_struct.h"
}
#include "sbsccommunicator.h"
#include "sbsc_protocol.h"

//allow any host to be the star camera
#define CAM_SERVERNAME "192.168.1.11"

//TODO check that this is correct
#define SBSC_SERIAL "08073507"

extern "C" void nameThread(const char*);  /* in mcp.c */

extern "C" short int InCharge;		  /* in tx.c */

static SBSCCommunicator* camComm;
static pthread_t camcomm_id;

static void* camReadLoop(void* arg);
static string parseReturn(string rtnStr);

static SBSCReturn camRtn[3];
static short int i_cam = 0; //read index in above buffer

extern "C" {

/*
 * used to make commanding available to rest of mcp
 */
int sendSBSCCommand(const char *cmd)
{
  //this is okay unless I want to handle link dying during transmission
  if (!InCharge) return 0;
  return camComm->sendCommand(cmd);
}


/*
 * open a connection the the star camera computer
 * also creates the communications thread
 */
void openSBSC()
{
  bprintf(info, "connecting to the SBSC");
  camComm = new SBSCCommunicator();
  pthread_create(&camcomm_id, NULL, &camReadLoop, NULL);
}

/*
 * update all camera related fields
 * meant to be called in mcp slow loop (5Hz)
 */
void cameraFields()
{
  static int firsttime = 1;
  SBSCReturn* sbsc = NULL;
  static bool unrecFlag = false;

  static NiosStruct* forceAddr = NULL;
  static NiosStruct* expIntAddr = NULL;
  static NiosStruct* expTimeAddr = NULL;
  static NiosStruct* focResAddr = NULL;
  static NiosStruct* moveTolAddr = NULL;
  static NiosStruct* maxBlobAddr = NULL;
  static NiosStruct* gridAddr = NULL;
  static NiosStruct* threshAddr = NULL;
  static NiosStruct* blobMdistAddr = NULL;
  static NiosStruct* trigSpeedAddr = NULL;

  static NiosStruct* sbscFrameAddr = NULL;
  static NiosStruct* sbscMeanAddr = NULL;
  static NiosStruct* sbscSigmaAddr = NULL;
  static NiosStruct* sbscTimeAddr = NULL;
  static NiosStruct* sbscUsecAddr = NULL;
  static NiosStruct* sbscCcdTempAddr = NULL;
  static NiosStruct* sbscNumBlobsAddr = NULL;

  static NiosStruct* sbscBlobX[3];
  static NiosStruct* sbscBlobY[3];
  static NiosStruct* sbscBlobF[3];
  static NiosStruct* sbscBlobS[3];

  //initialization
  if (firsttime) {
    firsttime = 0;
    forceAddr = GetNiosAddr("force_sbsc");
    expIntAddr = GetNiosAddr("exp_int_sbsc");
    expTimeAddr = GetNiosAddr("exp_time_sbsc");
    focResAddr = GetNiosAddr("foc_res_sbsc");
    moveTolAddr = GetNiosAddr("move_tol_sbsc");
    maxBlobAddr = GetNiosAddr("maxblob_sbsc");
    gridAddr = GetNiosAddr("grid_sbsc");
    threshAddr = GetNiosAddr("thresh_sbsc");
    blobMdistAddr = GetNiosAddr("mdist_sbsc");

    sbscFrameAddr = GetNiosAddr("frame_sbsc");
    sbscMeanAddr = GetNiosAddr("mapmean_sbsc");
    sbscSigmaAddr = GetNiosAddr("mapsigma_sbsc");
    sbscTimeAddr = GetNiosAddr("sec_sbsc");
    sbscUsecAddr = GetNiosAddr("usec_sbsc");
    sbscCcdTempAddr = GetNiosAddr("ccd_t_sbsc");
    sbscNumBlobsAddr = GetNiosAddr("nblobs_sbsc");

    trigSpeedAddr = GetNiosAddr("trig_speed_sbsc");

    for (int i=0; i<3; i++) {
      char buf[99];
      sprintf(buf, "blob%02d_x_sbsc", i);
      sbscBlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_y_sbsc", i);
      sbscBlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_f_sbsc", i);
      sbscBlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_s_sbsc", i);
      sbscBlobS[i] = GetNiosAddr(buf);
    }
  }

  WriteData(forceAddr, CommandData.cam.forced, NIOS_QUEUE);
  WriteData(expIntAddr, CommandData.cam.expInt, NIOS_QUEUE);
  WriteData(expTimeAddr, CommandData.cam.expTime, NIOS_QUEUE);
  WriteData(focResAddr, CommandData.cam.focusRes, NIOS_QUEUE);
  WriteData(moveTolAddr, CommandData.cam.moveTol, NIOS_QUEUE);
  WriteData(maxBlobAddr, CommandData.cam.maxBlobs, NIOS_QUEUE);
  WriteData(gridAddr, CommandData.cam.grid, NIOS_QUEUE);
  WriteData(threshAddr, (int)(CommandData.cam.threshold*1000), NIOS_QUEUE);
  WriteData(blobMdistAddr, CommandData.cam.minBlobDist, NIOS_QUEUE);
  WriteData(trigSpeedAddr, CommandData.cam.trigSpeed*32768/100, NIOS_QUEUE);

  //persistently identify cameras by serial number (camID)
  if (camRtn[i_cam].camID == SBSC_SERIAL)  {
    sbsc = &camRtn[i_cam];
    unrecFlag = false;
  }
  else if (!unrecFlag) { //don't keep printing same error
    bprintf(err, "unrecognized camera ID");
    sbsc = NULL;
    unrecFlag = true;
  }

  if (sbsc != NULL) {
    WriteData(sbscFrameAddr, sbsc->frameNum, NIOS_QUEUE);
    WriteData(sbscMeanAddr, (int)sbsc->mapmean, NIOS_QUEUE);
    WriteData(sbscSigmaAddr, (int)(sbsc->sigma*10), NIOS_QUEUE);
    WriteData(sbscTimeAddr, sbsc->imagestarttime.tv_sec, NIOS_QUEUE);
    WriteData(sbscUsecAddr, sbsc->imagestarttime.tv_usec, NIOS_QUEUE);
    //it looks like this is in deg C. just scale to get better resolution
    WriteData(sbscCcdTempAddr, (int)(sbsc->ccdtemperature*100), NIOS_QUEUE);
    WriteData(sbscNumBlobsAddr, sbsc->numblobs, NIOS_QUEUE);

    for (int i=0; i<sbsc->numblobs; i++)
    {
      //TODO this needs to be tested in images where there are blobs
      WriteData(sbscBlobX[i],(unsigned int)(sbsc->x[i]/CAM_WIDTH*SHRT_MAX),
	  NIOS_QUEUE);
      WriteData(sbscBlobY[i],(unsigned int)(sbsc->y[i]/CAM_WIDTH*SHRT_MAX),
	  NIOS_QUEUE);
      WriteData(sbscBlobF[i], (unsigned int)sbsc->flux[i], NIOS_QUEUE);
      unsigned int snr = (sbsc->snr[i] >= SHRT_MAX / 100.0) ? 
	SHRT_MAX : (unsigned int)sbsc->snr[i]*100;
      WriteData(sbscBlobS[i], snr, NIOS_QUEUE);
    }
  }

}

}       //extern "C"

/*
 * wrapper for the read loop in sbsccommunicator
 * mcp main should make a thread for this
 */
static void* camReadLoop(void* arg)
{
  nameThread("SBSC");
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (camComm->openClient(CAM_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      errorshown = true;
    }
  }
  bprintf(startup, "talking to camera");

  sendSBSCCommand("Oconf");  //request configuration data

  while(true) {
    camComm->readLoop(&parseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}

/*
 * function used be readloop that handles strings returned from the camera
 * it will write data to frames and log errors
 */
static string parseReturn(string rtnStr)
{
  /* debugging only
     bprintf(info, "return string: %s", rtnStr.c_str());
     */
  //if (rtnStr.find("<str>", 0) == 0) //response is string
  if (rtnStr.substr(0,5) == "<str>") //response is string
  {
    string Rstr = rtnStr.substr(5, rtnStr.size() - 11);

    if (Rstr[0] == 'E') //it is an error
      bprintf(err, "%s", Rstr.substr(6, Rstr.size()-6).c_str());

    //else if (Rstr.find("<conf>", 0) == 0) //contains config data
    else if (Rstr.substr(0,6) == "<conf>") //contains config data
    {
      Rstr = Rstr.substr(6, Rstr.size()-6);
      istringstream sin;
      sin.str(Rstr);
      double temp;  //value sent for expTime is a double
      sin >> CommandData.cam.expInt
	>> temp
	>> CommandData.cam.focusRes
	>> CommandData.cam.moveTol
	>> CommandData.cam.maxBlobs
	>> CommandData.cam.grid
	>> CommandData.cam.threshold
	>> CommandData.cam.minBlobDist;
      CommandData.cam.expTime = (int)(temp * 1000);
    }
    //otherwise it is success notice for another command

  } else { //response is exposure data
    camComm->interpretReturn(rtnStr, &camRtn[(i_cam+1)%2]);
    i_cam = (i_cam+1)%2;
  }
  return "";  //doesn't send a response back to camera
}

