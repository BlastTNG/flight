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
#include "share/blast.h"
#include "tx.h"
#include "share/channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "command_struct.h"
}
#include "rsccommunicator.h"
#include "sbsc_protocol.h"

//allow any host to be the star camera
#define CAM_SERVERNAME "192.168.1.11"

#define THEGOOD_SERIAL "08073507"
#define THEBAD_SERIAL "110794466"

extern "C" void nameThread(const char*);  /* in mcp.c */

extern "C" short int InCharge;		  /* in tx.c */

static RSCCommunicator* camComm;
static pthread_t rsccomm_id;

static void* RSCReadLoop(void* arg);
static string parseReturn(string rtnStr);

static SBSCReturn camRtn[3];
static short int i_cam = 0; //read index in above buffer

extern "C" {

/*
 * used to make commanding available to rest of mcp
 */
int sendRSCCommand(const char *cmd)
{
  //this is okay unless I want to handle link dying during transmission
  if (!InCharge) return 0;
  return camComm->sendCommand(cmd);
}


/*
 * open a connection the the star camera computer
 * also creates the communications thread
 */
//TODO add to mcp.c
void openRSC()
{
  bprintf(info, "connecting to the RSC");
  camComm = new RSCCommunicator();
  pthread_create(&rsccomm_id, NULL, &RSCReadLoop, NULL);
}

/*
 * update all camera related fields
 * meant to be called in mcp slow loop (5Hz)
 */
void RSCFields() 
//TODO add all these fields to tx_struct!
//TODO add to tx.c after cameraFields, change cameraFields to BSCFields
{
  static int firsttime = 1;
  SBSCReturn* rsc = NULL;
  static int which = 0;
  static bool unrecFlag = false;

  static NiosStruct* TheGoodforceAddr = NULL;
  static NiosStruct* TheBadforceAddr = NULL;
  static NiosStruct* TheGoodexpIntAddr = NULL;
  static NiosStruct* TheBadexpIntAddr = NULL;
  static NiosStruct* TheGoodexpTimeAddr = NULL;
  static NiosStruct* TheBadexpTimeAddr = NULL;
  static NiosStruct* TheGoodfocResAddr = NULL;
  static NiosStruct* TheBadfocResAddr = NULL;
  static NiosStruct* TheGoodmoveTolAddr = NULL;
  static NiosStruct* TheBadmoveTolAddr = NULL;
  static NiosStruct* TheGoodmaxBlobAddr = NULL;
  static NiosStruct* TheBadmaxBlobAddr = NULL;
  static NiosStruct* TheGoodgridAddr = NULL;
  static NiosStruct* TheBadgridAddr = NULL;
  static NiosStruct* TheGoodthreshAddr = NULL;
  static NiosStruct* TheBadthreshAddr = NULL;
  static NiosStruct* TheGoodblobMdistAddr = NULL;
  static NiosStruct* TheBadblobMdistAddr = NULL;

  static NiosStruct* TheGoodFrameAddr = NULL;
  static NiosStruct* TheBadFrameAddr = NULL;
  static NiosStruct* TheGoodMeanAddr = NULL;
  static NiosStruct* TheBadMeanAddr = NULL;
  static NiosStruct* TheGoodSigmaAddr = NULL;
  static NiosStruct* TheBadSigmaAddr = NULL;
  static NiosStruct* TheGoodTimeAddr = NULL;
  static NiosStruct* TheBadTimeAddr = NULL;
  static NiosStruct* TheGoodUsecAddr = NULL;
  static NiosStruct* TheBadUsecAddr = NULL;
  static NiosStruct* TheGoodCcdTempAddr = NULL;
  static NiosStruct* TheBadCcdTempAddr = NULL;
  static NiosStruct* TheGoodNumBlobsAddr = NULL;
  static NiosStruct* TheBadNumBlobsAddr = NULL;

  static NiosStruct* TheGoodBlobX[5];
  static NiosStruct* TheBadBlobX[5];
  static NiosStruct* TheGoodBlobY[5];
  static NiosStruct* TheBadBlobY[5];
  static NiosStruct* TheGoodBlobF[5];
  static NiosStruct* TheBadBlobF[5];
  static NiosStruct* TheGoodBlobS[5];
  static NiosStruct* TheBadBlobS[5];

  //initialization
  if (firsttime) {
    firsttime = 0;
    TheGoodforceAddr = GetNiosAddr("force_thegood");
    TheBadforceAddr = GetNiosAddr("force_thebad");
    TheGoodexpIntAddr = GetNiosAddr("exp_int_thegood");
    TheBadexpIntAddr = GetNiosAddr("exp_int_thebad");
    TheGoodexpTimeAddr = GetNiosAddr("exp_time_thegood");
    TheBadexpTimeAddr = GetNiosAddr("exp_time_thebad");
    TheGoodfocResAddr = GetNiosAddr("foc_res_thegood");
    TheBadfocResAddr = GetNiosAddr("foc_res_thebad");
    TheGoodmoveTolAddr = GetNiosAddr("move_tol_thegood");
    TheBadmoveTolAddr = GetNiosAddr("move_tol_thebad");
    TheGoodmaxBlobAddr = GetNiosAddr("maxblob_thegood");
    TheBadmaxBlobAddr = GetNiosAddr("maxblob_thebad");
    TheGoodgridAddr = GetNiosAddr("grid_thegood");
    TheBadgridAddr = GetNiosAddr("grid_thebad");
    TheGoodthreshAddr = GetNiosAddr("thresh_thegood");
    TheBadthreshAddr = GetNiosAddr("thresh_thebad");
    TheGoodblobMdistAddr = GetNiosAddr("mdist_thegood");
    TheBadblobMdistAddr = GetNiosAddr("mdist_thebad");

    TheGoodFrameAddr = GetNiosAddr("frame_thegood");
    TheBadFrameAddr = GetNiosAddr("frame_thebad");
    TheGoodMeanAddr = GetNiosAddr("mapmean_thegood");
    TheBadMeanAddr = GetNiosAddr("mapmean_thebad");
    TheGoodSigmaAddr = GetNiosAddr("mapsigma_thegood");
    TheBadSigmaAddr = GetNiosAddr("mapsigma_thebad");
    TheGoodTimeAddr = GetNiosAddr("sec_thegood");
    TheBadTimeAddr = GetNiosAddr("sec_thebad");
    TheGoodUsecAddr = GetNiosAddr("usec_thegood");
    TheBadUsecAddr = GetNiosAddr("usec_thebad");
    TheGoodCcdTempAddr = GetNiosAddr("ccd_t_thegood");
    TheBadCcdTempAddr = GetNiosAddr("ccd_t_thebad");
    TheGoodNumBlobsAddr = GetNiosAddr("nblobs_thegood");
    TheBadNumBlobsAddr = GetNiosAddr("nblobs_thebad");

    for (int i=0; i<5; i++) {
      char buf[99];
      sprintf(buf, "blob%02d_x_thegood", i);
      TheGoodBlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_x_thebad", i);
      TheBadBlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_y_thegood", i);
      TheGoodBlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_y_thebad", i);
      TheBadBlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_f_thegood", i);
      TheGoodBlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_f_thebad", i);
      TheBadBlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_s_thegood", i);
      TheGoodBlobS[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_s_thebad", i);
      TheBadBlobS[i] = GetNiosAddr(buf);
    }
  }
//TODO add fields to CommandData, change CommandData.cam to CommandData.theugly
  WriteData(TheGoodforceAddr, CommandData.thegood.forced, NIOS_QUEUE);
  WriteData(TheBadforceAddr, CommandData.thebad.forced, NIOS_QUEUE);
  WriteData(TheGoodexpIntAddr, CommandData.thegood.expInt, NIOS_QUEUE);
  WriteData(TheBadexpIntAddr, CommandData.thebad.expInt, NIOS_QUEUE);
  WriteData(TheGoodexpTimeAddr, CommandData.thegood.expTime, NIOS_QUEUE);
  WriteData(TheBadexpTimeAddr, CommandData.thebad.expTime, NIOS_QUEUE);
  WriteData(TheGoodfocResAddr, CommandData.thegood.focusRes, NIOS_QUEUE);
  WriteData(TheBadfocResAddr, CommandData.thebad.focusRes, NIOS_QUEUE);
  WriteData(TheGoodmoveTolAddr, CommandData.thegood.moveTol, NIOS_QUEUE);
  WriteData(TheBadmoveTolAddr, CommandData.thebad.moveTol, NIOS_QUEUE);
  WriteData(TheGoodmaxBlobAddr, CommandData.thegood.maxBlobs, NIOS_QUEUE);
  WriteData(TheBadmaxBlobAddr, CommandData.thebad.maxBlobs, NIOS_QUEUE);
  WriteData(TheGoodgridAddr, CommandData.thegood.grid, NIOS_QUEUE);
  WriteData(TheBadgridAddr, CommandData.thebad.grid, NIOS_QUEUE);
  WriteData(TheGoodthreshAddr, (int)(CommandData.thegood.threshold*1000), NIOS_QUEUE);
  WriteData(TheBadthreshAddr, (int)(CommandData.thebad.threshold*1000), NIOS_QUEUE);
  WriteData(TheGoodblobMdistAddr, CommandData.thegood.minBlobDist, NIOS_QUEUE);
  WriteData(TheBadblobMdistAddr, CommandData.thebad.minBlobDist, NIOS_QUEUE);

  //persistently identify cameras by serial number (camID)
  if (camRtn[i_cam].camID == THEGOOD_SERIAL)  {
    rsc = &camRtn[i_cam];
    which = 0;
    unrecFlag = false;
  }
  else if (camRtn[i_cam].camID == THEBAD_SERIAL)  {
    rsc = &camRtn[i_cam];
    which = 1;
    unrecFlag = false;
  }
  else if (!unrecFlag) { //don't keep printing same error
    bprintf(err, "unrecognized camera ID");
    rsc = NULL;
    unrecFlag = true;
  }

  if (rsc != NULL) {
    if (which == 0) { 
    	WriteData(TheGoodFrameAddr, rsc->frameNum, NIOS_QUEUE);
    	WriteData(TheGoodMeanAddr, (int)rsc->mapmean, NIOS_QUEUE);
    	WriteData(TheGoodSigmaAddr, (int)(rsc->sigma*10), NIOS_QUEUE);
    	WriteData(TheGoodTimeAddr, rsc->imagestarttime.tv_sec, NIOS_QUEUE);
    	WriteData(TheGoodUsecAddr, rsc->imagestarttime.tv_usec, NIOS_QUEUE);
    	//it looks like this is in deg C. just scale to get better resolution
    	WriteData(TheGoodCcdTempAddr, (int)(rsc->ccdtemperature*100), NIOS_QUEUE);
    	WriteData(TheGoodNumBlobsAddr, rsc->numblobs, NIOS_QUEUE);

    	for (int i=0; i<rsc->numblobs; i++)
    	{
    	  WriteData(TheGoodBlobX[i],(unsigned int)(rsc->x[i]/CAM_WIDTH*SHRT_MAX),
		  NIOS_QUEUE);
	      WriteData(TheGoodBlobY[i],(unsigned int)(rsc->y[i]/CAM_WIDTH*SHRT_MAX),
		  NIOS_QUEUE);
	      WriteData(TheGoodBlobF[i], (unsigned int)rsc->flux[i], NIOS_QUEUE);
	      unsigned int snr = (rsc->snr[i] >= SHRT_MAX / 100.0) ? 
		SHRT_MAX : (unsigned int)rsc->snr[i]*100;
	      WriteData(TheGoodBlobS[i], snr, NIOS_QUEUE);
	}
    } else if (which == 1) {
    	WriteData(TheBadFrameAddr, rsc->frameNum, NIOS_QUEUE);
    	WriteData(TheBadMeanAddr, (int)rsc->mapmean, NIOS_QUEUE);
    	WriteData(TheBadSigmaAddr, (int)(rsc->sigma*10), NIOS_QUEUE);
    	WriteData(TheBadTimeAddr, rsc->imagestarttime.tv_sec, NIOS_QUEUE);
    	WriteData(TheBadUsecAddr, rsc->imagestarttime.tv_usec, NIOS_QUEUE);
    	//it looks like this is in deg C. just scale to get better resolution
    	WriteData(TheBadCcdTempAddr, (int)(rsc->ccdtemperature*100), NIOS_QUEUE);
    	WriteData(TheBadNumBlobsAddr, rsc->numblobs, NIOS_QUEUE);

    	for (int i=0; i<rsc->numblobs; i++)
    	{
    	  WriteData(TheBadBlobX[i],(unsigned int)(rsc->x[i]/CAM_WIDTH*SHRT_MAX),
		  NIOS_QUEUE);
	      WriteData(TheBadBlobY[i],(unsigned int)(rsc->y[i]/CAM_WIDTH*SHRT_MAX),
		  NIOS_QUEUE);
	      WriteData(TheBadBlobF[i], (unsigned int)rsc->flux[i], NIOS_QUEUE);
	      unsigned int snr = (rsc->snr[i] >= SHRT_MAX / 100.0) ? 
		SHRT_MAX : (unsigned int)rsc->snr[i]*100;
	      WriteData(TheBadBlobS[i], snr, NIOS_QUEUE);
	}
    }

  }

}

}       //extern "C"

/*
 * wrapper for the read loop in rsccommunicator
 * mcp main should make a thread for this
 */
static void* RSCReadLoop(void* arg)
{
  nameThread("RSC");
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (camComm->openClient(CAM_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      errorshown = true;
    }
  }
  bprintf(startup, "talking to camera");

  sendRSCCommand("Oconf");  //request configuration data

  while(true) {
    camComm->readLoop(&parseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}

/*
 * function used by readloop that handles strings returned from the camera
 * it will write data to frames and log errors
 */
static string parseReturn(string rtnStr)
{
  /* debugging only
     bprintf(info, "return string: %s", rtnStr.c_str());
     */
  if (rtnStr.substr(0,5) == "<str>") //response is string
  {
    string Rstr = rtnStr.substr(5, rtnStr.size() - 11);

    if (Rstr[0] == 'E') //it is an error
      bprintf(err, "%s", Rstr.substr(6, Rstr.size()-6).c_str());

    else if (Rstr.substr(0,6) == "<TheGoodconf>") //contains The Good config data
    {
      Rstr = Rstr.substr(6, Rstr.size()-6);
      istringstream sin;
      sin.str(Rstr);
      double temp;  //value sent for expTime is a double
      sin >> CommandData.thegood.expInt
	>> temp
	>> CommandData.thegood.focusRes
	>> CommandData.thegood.moveTol
	>> CommandData.thegood.maxBlobs
	>> CommandData.thegood.grid
	>> CommandData.thegood.threshold
	>> CommandData.thegood.minBlobDist;
      CommandData.thegood.expTime = (int)(temp * 1000);
    }
    else if (Rstr.substr(0,6) == "<TheBadconf>") //contains The Bad config data
    {
      Rstr = Rstr.substr(6, Rstr.size()-6);
      istringstream sin;
      sin.str(Rstr);
      double temp;  //value sent for expTime is a double
      sin >> CommandData.thebad.expInt
	>> temp
	>> CommandData.thebad.focusRes
	>> CommandData.thebad.moveTol
	>> CommandData.thebad.maxBlobs
	>> CommandData.thebad.grid
	>> CommandData.thebad.threshold
	>> CommandData.thebad.minBlobDist;
      CommandData.thebad.expTime = (int)(temp * 1000);
    }
    //otherwise it is success notice for another command

  } else { //response is exposure data
    camComm->interpretReturn(rtnStr, &camRtn[(i_cam+1)%2]);
    i_cam = (i_cam+1)%2;
  }
  return "";  //doesn't send a response back to camera
}

