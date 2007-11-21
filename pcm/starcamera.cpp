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
#include "command_struct.h"
}
#include "blast.h"
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "camcommunicator.h"
#include "camstruct.h"
#include "command_struct.h"

//allow any host to be the star camera
#define CAM_SERVERNAME "aragog.spider"

static CamCommunicator* camComm;
static pthread_t camcomm_id;

static void* camReadLoop(void* arg);
static string parseReturn(string rtnStr);

static StarcamReturn cam1Rtn[3];
static short int i_cam = 0; //read index in above buffer

extern "C" {

/*
 * used to make commanding available to rest of mcp
 */
int sendCamCommand(const char *cmd)
{
  //this is okay unless I want to handle link dying during transmission
  return camComm->sendCommand(cmd);
}


/*
 * open a connection the the star camera computer
 * also creates the communications thread
 */
void openCamera()
{
  bprintf(startup, "Starcam: connecting to the star camera");
  camComm = new CamCommunicator();
  pthread_create(&camcomm_id, NULL, &camReadLoop, NULL);
}

/*
 * update all camera related fields
 * meant to be called in mcp slow loop (5Hz)
 */
void cameraFields()
{
  static bool firsttime = true;
  StarcamReturn* sc1 = NULL;
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

  static NiosStruct* sc1FrameAddr = NULL;
  static NiosStruct* sc1MeanAddr = NULL;
  static NiosStruct* sc1SigmaAddr = NULL;
  static NiosStruct* sc1TimeAddr = NULL;
  static NiosStruct* sc1UsecAddr = NULL;
  static NiosStruct* sc1CcdTempAddr = NULL;
  static NiosStruct* sc1NumBlobsAddr = NULL;

  static NiosStruct* sc1BlobX[15];
  static NiosStruct* sc1BlobY[15];
  static NiosStruct* sc1BlobF[15];
  static NiosStruct* sc1BlobS[15];

  //initialization
  if (firsttime) {
    forceAddr = GetNiosAddr("sc_force");
    expIntAddr = GetNiosAddr("sc_exp_int");
    expTimeAddr = GetNiosAddr("sc_exp_time");
    focResAddr = GetNiosAddr("sc_foc_res");
    moveTolAddr = GetNiosAddr("sc_move_tol");
    maxBlobAddr = GetNiosAddr("sc_maxblob");
    gridAddr = GetNiosAddr("sc_grid");
    threshAddr = GetNiosAddr("sc_thresh");
    blobMdistAddr = GetNiosAddr("sc_mdist");

    sc1FrameAddr = GetNiosAddr("sc1_frame");
    sc1MeanAddr = GetNiosAddr("sc1_mapmean");
    sc1SigmaAddr = GetNiosAddr("sc1_mapsigma");
    sc1TimeAddr = GetNiosAddr("sc1_sec");
    sc1UsecAddr = GetNiosAddr("sc1_usec");
    sc1CcdTempAddr = GetNiosAddr("sc1_ccd_t");
    sc1NumBlobsAddr = GetNiosAddr("sc1_numblobs");

    for (int i=0; i<15; i++) {
      char buf[99];
      sprintf(buf, "sc1_blob%02d_x", i);
      sc1BlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "sc1_blob%02d_y", i);
      sc1BlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "sc1_blob%02d_f", i);
      sc1BlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "sc1_blob%02d_s", i);
      sc1BlobS[i] = GetNiosAddr(buf);
    }

    firsttime = false;
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

  //persistently identify cameras by serial number (camID)
  if (cam1Rtn[i_cam].camID == "060400935") {
    sc1 = &cam1Rtn[i_cam];
    unrecFlag = false;
  }
  else if (!unrecFlag) { //don't keep printing same error
    bprintf(err, "Starcam: unrecognized camera ID");
    sc1 = NULL;
    unrecFlag = true;
  }

  if (sc1 != NULL) {
    WriteData(sc1FrameAddr, sc1->frameNum, NIOS_QUEUE);
    WriteData(sc1MeanAddr, (int)sc1->mapmean, NIOS_QUEUE);
    WriteData(sc1SigmaAddr, (int)(sc1->sigma*10), NIOS_QUEUE);
    WriteData(sc1TimeAddr, sc1->imagestarttime.tv_sec, NIOS_QUEUE);
    WriteData(sc1UsecAddr, sc1->imagestarttime.tv_usec, NIOS_QUEUE);
    //it looks like this is in deg C. just scale to get better resolution
    WriteData(sc1CcdTempAddr, (int)(sc1->ccdtemperature*100), NIOS_QUEUE);
    WriteData(sc1NumBlobsAddr, sc1->numblobs, NIOS_QUEUE);

    for (int i=0; i<sc1->numblobs; i++)
    {
      //TODO this needs to be tested in images where there are blobs
      WriteData(sc1BlobX[i],(unsigned int)(sc1->x[i]/CAM_WIDTH*SHRT_MAX),
	  NIOS_QUEUE);
      WriteData(sc1BlobY[i],(unsigned int)(sc1->y[i]/CAM_WIDTH*SHRT_MAX),
	  NIOS_QUEUE);
      WriteData(sc1BlobF[i], (unsigned int)sc1->flux[i], NIOS_QUEUE);
      unsigned int snr = (sc1->snr[i] >= SHRT_MAX / 100.0) ? 
	SHRT_MAX : (unsigned int)sc1->snr[i]*100;
      WriteData(sc1BlobS[i], snr, NIOS_QUEUE);
    }
  }

}

}       //extern "C"

/*
 * wrapper for the read loop in CamCommunicator
 * mcp main should make a thread for this
 */
static void* camReadLoop(void* arg)
{
  if (camComm->openClient(CAM_SERVERNAME) < 0)
    bprintf(err, "Starcam: failed to accept star camera connection");
  else bprintf(startup, "Starcam: talking to star camera");

  sendCamCommand("Oconf");  //request configuration data

  while(true) {
    camComm->readLoop(&parseReturn);
    //readLoop returns when something bad happens; TODO handle
    bprintf(err, "Starcam: readLoop returned. This is bad. Restarting.");
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
  bprintf(info, "Starcam: return string: %s", rtnStr.c_str());
  */
  if (rtnStr.find("<str>", 0) == 0) //respone is string
  {
    string Rstr = rtnStr.substr(5, rtnStr.size() - 11);

    if (Rstr[0] == 'E') //it is an error
      bprintf(err, "Starcam: %s", Rstr.substr(6, Rstr.size()-6).c_str());

    else if (Rstr.find("<conf>", 0) == 0) //contains config data
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
    camComm->interpretReturn(rtnStr, &cam1Rtn[(i_cam+1)%2]);
    i_cam = (i_cam+1)%2;
  }
  return "";  //doesn't send a response back to camera
}

