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
#include <iostream>

extern "C" {
#include "share/blast.h"
#include "tx.h"
#include "share/channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "command_struct.h"
}
#include "camcommunicator.h"
#include "camstruct.h"

//allow any host to be the star camera
#define THEGOOD_SERVERNAME "192.168.1.11"
#define THEBAD_SERVERNAME  "192.168.1.12"
#define THEUGLY_SERVERNAME "192.168.1.13"

#define THEGOOD_SERIAL "110794466"
#define THEBAD_SERIAL  "08073506"
#define THEUGLY_SERIAL "08073507"

extern "C" void nameThread(const char*);  /* in mcp.c */

extern "C" short int InCharge;		  /* in tx.c */

extern "C" int EthernetSC[3];      /* tx.c */

extern double goodPos[10];	/* table.cpp */
extern double trigPos[10];	/* camcommunicator.cpp */

static CamCommunicator* TheGoodComm;
static CamCommunicator* TheBadComm;
static CamCommunicator* TheUglyComm;
static pthread_t TheGoodcomm_id;
static pthread_t TheBadcomm_id;
static pthread_t TheUglycomm_id;

static void* TheGoodReadLoop(void* arg);
static void* TheBadReadLoop(void* arg);
static void* TheUglyReadLoop(void* arg);
static string TheGoodparseReturn(string rtnStr);
static string TheBadparseReturn(string rtnStr);
static string TheUglyparseReturn(string rtnStr);

static StarcamReturn camRtn[3];
static short int i_cam = 0; //read index in above buffer

extern "C" {

/*
 * used to make commanding available to rest of mcp
 */
int sendTheGoodCommand(const char *cmd)
{
  if (!InCharge) return 0;
  if (EthernetSC[0] == 0) return TheGoodComm->sendCommand(cmd);
  return 0;
}
int sendTheBadCommand(const char *cmd)
{
  if (!InCharge) return 0;
  if (EthernetSC[1] == 0) return TheBadComm->sendCommand(cmd);
  return 0;
}
int sendTheUglyCommand(const char *cmd)
{
  if (!InCharge) return 0;
  if (EthernetSC[2] == 0) return TheUglyComm->sendCommand(cmd);
  return 0;
}

/*
 * open a connection the the star camera computers
 * also creates the communications threads
 */
void openSC()
{
  bprintf(info, "connecting to the Star Cameras");
  TheGoodComm = new CamCommunicator();
  TheBadComm  = new CamCommunicator();
  TheUglyComm = new CamCommunicator();
  pthread_create(&TheGoodcomm_id, NULL, &TheGoodReadLoop, NULL);
  pthread_create(&TheBadcomm_id, NULL, &TheBadReadLoop, NULL);
  pthread_create(&TheUglycomm_id, NULL, &TheUglyReadLoop, NULL);
}

/*
 * update all camera related fields
 * meant to be called in mcp slow loop (5Hz)
 */
void cameraFields()
{
  static int firsttime = 1;
  StarcamReturn* rsc = NULL;
  StarcamReturn* bsc = NULL;
  static int which;
  static bool unrecFlag = false;
  static unsigned long int posFrame;

  static NiosStruct* TheGoodforceAddr = NULL;
  static NiosStruct* TheBadforceAddr = NULL;
  static NiosStruct* TheUglyforceAddr = NULL;
  static NiosStruct* TheGoodexpIntAddr = NULL;
  static NiosStruct* TheBadexpIntAddr = NULL;
  static NiosStruct* TheUglyexpIntAddr = NULL;
  static NiosStruct* TheGoodexpTimeAddr = NULL;
  static NiosStruct* TheBadexpTimeAddr = NULL;
  static NiosStruct* TheUglyexpTimeAddr = NULL;
  static NiosStruct* TheGoodfocResAddr = NULL;
  static NiosStruct* TheBadfocResAddr = NULL;
  static NiosStruct* TheUglyfocResAddr = NULL;
  static NiosStruct* TheGoodmoveTolAddr = NULL;
  static NiosStruct* TheBadmoveTolAddr = NULL;
  static NiosStruct* TheUglymoveTolAddr = NULL;
  static NiosStruct* TheGoodmaxBlobAddr = NULL;
  static NiosStruct* TheBadmaxBlobAddr = NULL;
  static NiosStruct* TheUglymaxBlobAddr = NULL;
  static NiosStruct* TheGoodgridAddr = NULL;
  static NiosStruct* TheBadgridAddr = NULL;
  static NiosStruct* TheUglygridAddr = NULL;
  static NiosStruct* TheGoodthreshAddr = NULL;
  static NiosStruct* TheBadthreshAddr = NULL;
  static NiosStruct* TheUglythreshAddr = NULL;
  static NiosStruct* TheGoodblobMdistAddr = NULL;
  static NiosStruct* TheBadblobMdistAddr = NULL;
  static NiosStruct* TheUglyblobMdistAddr = NULL;

  static NiosStruct* TheGoodFrameAddr = NULL;
  static NiosStruct* TheBadFrameAddr = NULL;
  static NiosStruct* TheUglyFrameAddr = NULL;
  static NiosStruct* TheGoodMeanAddr = NULL;
  static NiosStruct* TheBadMeanAddr = NULL;
  static NiosStruct* TheUglyMeanAddr = NULL;
  static NiosStruct* TheGoodSigmaAddr = NULL;
  static NiosStruct* TheBadSigmaAddr = NULL;
  static NiosStruct* TheUglySigmaAddr = NULL;
  static NiosStruct* TheGoodTimeAddr = NULL;
  static NiosStruct* TheBadTimeAddr = NULL;
  static NiosStruct* TheUglyTimeAddr = NULL;
  static NiosStruct* TheGoodUsecAddr = NULL;
  static NiosStruct* TheBadUsecAddr = NULL;
  static NiosStruct* TheUglyUsecAddr = NULL;
  static NiosStruct* TheGoodCcdTempAddr = NULL;
  static NiosStruct* TheBadCcdTempAddr = NULL;
  static NiosStruct* TheUglyCcdTempAddr = NULL;
  static NiosStruct* TheGoodFocPosAddr = NULL;
  static NiosStruct* TheBadFocPosAddr = NULL;
  static NiosStruct* TheUglyFocPosAddr = NULL;
  static NiosStruct* TheGoodNumBlobsAddr = NULL;
  static NiosStruct* TheBadNumBlobsAddr = NULL;
  static NiosStruct* TheUglyNumBlobsAddr = NULL;

  static NiosStruct* TheGoodBlobX[5];
  static NiosStruct* TheBadBlobX[5];
  static NiosStruct* TheUglyBlobX[5];
  static NiosStruct* TheGoodBlobY[5];
  static NiosStruct* TheBadBlobY[5];
  static NiosStruct* TheUglyBlobY[5];
  static NiosStruct* TheGoodBlobF[5];
  static NiosStruct* TheBadBlobF[5];
  static NiosStruct* TheUglyBlobF[5];
  static NiosStruct* TheGoodBlobS[5];
  static NiosStruct* TheBadBlobS[5];
  static NiosStruct* TheUglyBlobS[5];

  //initialization
  if (firsttime) {
    firsttime = 0;
    TheGoodforceAddr = GetNiosAddr("force_thegood");
    TheBadforceAddr = GetNiosAddr("force_thebad");
    TheUglyforceAddr = GetNiosAddr("force_theugly");
    TheGoodexpIntAddr = GetNiosAddr("exp_int_thegood");
    TheBadexpIntAddr = GetNiosAddr("exp_int_thebad");
    TheUglyexpIntAddr = GetNiosAddr("exp_int_theugly");
    TheGoodexpTimeAddr = GetNiosAddr("exp_time_thegood");
    TheBadexpTimeAddr = GetNiosAddr("exp_time_thebad");
    TheUglyexpTimeAddr = GetNiosAddr("exp_time_theugly");
    TheGoodfocResAddr = GetNiosAddr("foc_res_thegood");
    TheBadfocResAddr = GetNiosAddr("foc_res_thebad");
    TheUglyfocResAddr = GetNiosAddr("foc_res_theugly");
    TheGoodmoveTolAddr = GetNiosAddr("move_tol_thegood");
    TheBadmoveTolAddr = GetNiosAddr("move_tol_thebad");
    TheUglymoveTolAddr = GetNiosAddr("move_tol_theugly");
    TheGoodmaxBlobAddr = GetNiosAddr("maxblob_thegood");
    TheBadmaxBlobAddr = GetNiosAddr("maxblob_thebad");
    TheUglymaxBlobAddr = GetNiosAddr("maxblob_theugly");
    TheGoodgridAddr = GetNiosAddr("grid_thegood");
    TheBadgridAddr = GetNiosAddr("grid_thebad");
    TheUglygridAddr = GetNiosAddr("grid_theugly");
    TheGoodthreshAddr = GetNiosAddr("thresh_thegood");
    TheBadthreshAddr = GetNiosAddr("thresh_thebad");
    TheUglythreshAddr = GetNiosAddr("thresh_theugly");
    TheGoodblobMdistAddr = GetNiosAddr("mdist_thegood");
    TheBadblobMdistAddr = GetNiosAddr("mdist_thebad");
    TheUglyblobMdistAddr = GetNiosAddr("mdist_theugly");

    TheGoodFrameAddr = GetNiosAddr("frame_thegood");
    TheBadFrameAddr = GetNiosAddr("frame_thebad");
    TheUglyFrameAddr = GetNiosAddr("frame_theugly");
    TheGoodMeanAddr = GetNiosAddr("mapmean_thegood");
    TheBadMeanAddr = GetNiosAddr("mapmean_thebad");
    TheUglyMeanAddr = GetNiosAddr("mapmean_theugly");
    TheGoodSigmaAddr = GetNiosAddr("mapsigma_thegood");
    TheBadSigmaAddr = GetNiosAddr("mapsigma_thebad");
    TheUglySigmaAddr = GetNiosAddr("mapsigma_theugly");
    TheGoodTimeAddr = GetNiosAddr("sec_thegood");
    TheBadTimeAddr = GetNiosAddr("sec_thebad");
    TheUglyTimeAddr = GetNiosAddr("sec_theugly");
    TheGoodUsecAddr = GetNiosAddr("usec_thegood");
    TheBadUsecAddr = GetNiosAddr("usec_thebad");
    TheUglyUsecAddr = GetNiosAddr("usec_theugly");
    TheGoodCcdTempAddr = GetNiosAddr("ccd_t_thegood");
    TheBadCcdTempAddr = GetNiosAddr("ccd_t_thebad");
    TheUglyCcdTempAddr = GetNiosAddr("ccd_t_theugly");
    TheGoodFocPosAddr = GetNiosAddr("focpos_thegood");
    TheBadFocPosAddr = GetNiosAddr("focpos_thebad");
    TheUglyFocPosAddr = GetNiosAddr("focpos_theugly");
    TheGoodNumBlobsAddr = GetNiosAddr("nblobs_thegood");
    TheBadNumBlobsAddr = GetNiosAddr("nblobs_thebad");
    TheUglyNumBlobsAddr = GetNiosAddr("nblobs_theugly");

    for (int i=0; i<5; i++) {
      char buf[99];
      sprintf(buf, "blob%02d_x_thegood", i);
      TheGoodBlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_x_thebad", i);
      TheBadBlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_x_theugly", i);
      TheUglyBlobX[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_y_thegood", i);
      TheGoodBlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_y_thebad", i);
      TheBadBlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_y_theugly", i);
      TheUglyBlobY[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_f_thegood", i);
      TheGoodBlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_f_thebad", i);
      TheBadBlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_f_theugly", i);
      TheUglyBlobF[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_s_thegood", i);
      TheGoodBlobS[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_s_thebad", i);
      TheBadBlobS[i] = GetNiosAddr(buf);
      sprintf(buf, "blob%02d_s_theugly", i);
      TheUglyBlobS[i] = GetNiosAddr(buf);
    }
  }

  WriteData(TheGoodforceAddr, CommandData.thegood.forced, NIOS_QUEUE);
  WriteData(TheBadforceAddr, CommandData.thebad.forced, NIOS_QUEUE);
  WriteData(TheUglyforceAddr, CommandData.theugly.forced, NIOS_QUEUE);
  WriteData(TheGoodexpIntAddr, CommandData.thegood.expInt, NIOS_QUEUE);
  WriteData(TheBadexpIntAddr, CommandData.thebad.expInt, NIOS_QUEUE);
  WriteData(TheUglyexpIntAddr, CommandData.theugly.expInt, NIOS_QUEUE);
  WriteData(TheGoodexpTimeAddr, CommandData.thegood.expTime, NIOS_QUEUE);
  WriteData(TheBadexpTimeAddr, CommandData.thebad.expTime, NIOS_QUEUE);
  WriteData(TheUglyexpTimeAddr, CommandData.theugly.expTime, NIOS_QUEUE);
  WriteData(TheGoodfocResAddr, CommandData.thegood.focusRes, NIOS_QUEUE);
  WriteData(TheBadfocResAddr, CommandData.thebad.focusRes, NIOS_QUEUE);
  WriteData(TheUglyfocResAddr, CommandData.theugly.focusRes, NIOS_QUEUE);
  WriteData(TheGoodmoveTolAddr, CommandData.thegood.moveTol, NIOS_QUEUE);
  WriteData(TheBadmoveTolAddr, CommandData.thebad.moveTol, NIOS_QUEUE);
  WriteData(TheUglymoveTolAddr, CommandData.theugly.moveTol, NIOS_QUEUE);
  WriteData(TheGoodmaxBlobAddr, CommandData.thegood.maxBlobs, NIOS_QUEUE);
  WriteData(TheBadmaxBlobAddr, CommandData.thebad.maxBlobs, NIOS_QUEUE);
  WriteData(TheUglymaxBlobAddr, CommandData.theugly.maxBlobs, NIOS_QUEUE);
  WriteData(TheGoodgridAddr, CommandData.thegood.grid, NIOS_QUEUE);
  WriteData(TheBadgridAddr, CommandData.thebad.grid, NIOS_QUEUE);
  WriteData(TheUglygridAddr, CommandData.theugly.grid, NIOS_QUEUE);
  WriteData(TheGoodthreshAddr, (int)(CommandData.thegood.threshold*1000), NIOS_QUEUE);
  WriteData(TheBadthreshAddr, (int)(CommandData.thebad.threshold*1000), NIOS_QUEUE);
  WriteData(TheUglythreshAddr, (int)(CommandData.theugly.threshold*1000), NIOS_QUEUE);
  WriteData(TheGoodblobMdistAddr, CommandData.thegood.minBlobDist, NIOS_QUEUE);
  WriteData(TheBadblobMdistAddr, CommandData.thebad.minBlobDist, NIOS_QUEUE);
  WriteData(TheUglyblobMdistAddr, CommandData.theugly.minBlobDist, NIOS_QUEUE);

  //persistently identify cameras by serial number (camID)
  if (camRtn[i_cam].camID == THEGOOD_SERIAL)  {
    rsc = &camRtn[i_cam];
    which = 1;
    unrecFlag = false;
  }
  else if (camRtn[i_cam].camID == THEBAD_SERIAL)  {
    rsc = &camRtn[i_cam];
    which = 2;
    unrecFlag = false;
  }
  else if (camRtn[i_cam].camID == THEUGLY_SERIAL)  {
    bsc = &camRtn[i_cam];
    unrecFlag = false;
  }
  else if (!unrecFlag) { //don't keep printing same error
    bprintf(err, "unrecognized camera ID");
    rsc = NULL;
    bsc = NULL;
    unrecFlag = true;
  }

  if (rsc != NULL) {
    if (which == 1) { 
    	WriteData(TheGoodFrameAddr, rsc->frameNum, NIOS_QUEUE);
    	WriteData(TheGoodMeanAddr, (int)rsc->mapmean, NIOS_QUEUE);
    	WriteData(TheGoodSigmaAddr, (int)(rsc->sigma*10), NIOS_QUEUE);
    	WriteData(TheGoodTimeAddr, rsc->imagestarttime.tv_sec, NIOS_QUEUE);
    	WriteData(TheGoodUsecAddr, rsc->imagestarttime.tv_usec, NIOS_QUEUE);
    	//it looks like this is in deg C. just scale to get better resolution
    	WriteData(TheGoodCcdTempAddr, (int)(rsc->ccdtemperature*100), NIOS_QUEUE);
    	WriteData(TheGoodFocPosAddr, (int)(rsc->focusposition*100), NIOS_QUEUE);
    	WriteData(TheGoodNumBlobsAddr, rsc->numblobs, NIOS_QUEUE);
	
	if (rsc->numblobs > 0) {
		for (int i=0; i<5; i++)
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
	}
	if (rsc->numblobs > 8) {
		for (int j=0; j<10; j++) {
			if ((goodPos[j] == 90.0) && (rsc->frameNum != posFrame)) {
				goodPos[j] = trigPos[j]; //overwrite the first 'dead' one it finds
				cout << rsc->numblobs << " BLOBS" <<  ", setting GOODPOS #" << j << " to " << goodPos[j] << endl;
				posFrame = rsc->frameNum;
				break;
			}
		}
	}
    } else if (which == 2) {
    	WriteData(TheBadFrameAddr, rsc->frameNum, NIOS_QUEUE);
    	WriteData(TheBadMeanAddr, (int)rsc->mapmean, NIOS_QUEUE);
    	WriteData(TheBadSigmaAddr, (int)(rsc->sigma*10), NIOS_QUEUE);
    	WriteData(TheBadTimeAddr, rsc->imagestarttime.tv_sec, NIOS_QUEUE);
    	WriteData(TheBadUsecAddr, rsc->imagestarttime.tv_usec, NIOS_QUEUE);
    	//it looks like this is in deg C. just scale to get better resolution
    	WriteData(TheBadCcdTempAddr, (int)(rsc->ccdtemperature*100), NIOS_QUEUE);
    	WriteData(TheBadFocPosAddr, (int)(rsc->focusposition*100), NIOS_QUEUE);
    	WriteData(TheBadNumBlobsAddr, rsc->numblobs, NIOS_QUEUE);

	if (rsc->numblobs > 0) {
		for (int i=0; i<5; i++)
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
  if (bsc != NULL) {
    	WriteData(TheUglyFrameAddr, bsc->frameNum, NIOS_QUEUE);
    	WriteData(TheUglyMeanAddr, (int)bsc->mapmean, NIOS_QUEUE);
    	WriteData(TheUglySigmaAddr, (int)(bsc->sigma*10), NIOS_QUEUE);
    	WriteData(TheUglyTimeAddr, bsc->imagestarttime.tv_sec, NIOS_QUEUE);
    	WriteData(TheUglyUsecAddr, bsc->imagestarttime.tv_usec, NIOS_QUEUE);
    	//it looks like this is in deg C. just scale to get better resolution
    	WriteData(TheUglyCcdTempAddr, (int)(bsc->ccdtemperature*100), NIOS_QUEUE);
    	WriteData(TheUglyFocPosAddr, (int)(bsc->focusposition*100), NIOS_QUEUE);
    	WriteData(TheUglyNumBlobsAddr, bsc->numblobs, NIOS_QUEUE);

	if (bsc->numblobs > 0) {
    		for (int i=0; i<5; i++)
    		{
      			WriteData(TheUglyBlobX[i],(unsigned int)(bsc->x[i]/CAM_WIDTH*SHRT_MAX),
		  		NIOS_QUEUE);
      			WriteData(TheUglyBlobY[i],(unsigned int)(bsc->y[i]/CAM_WIDTH*SHRT_MAX),
		  		NIOS_QUEUE);
      			WriteData(TheUglyBlobF[i], (unsigned int)bsc->flux[i], NIOS_QUEUE);
      			unsigned int snr = (bsc->snr[i] >= SHRT_MAX / 100.0) ? 
			SHRT_MAX : (unsigned int)bsc->snr[i]*100;
      			WriteData(TheUglyBlobS[i], snr, NIOS_QUEUE);
    		}
	}
  }	

}

}       //extern "C"

/*
 * wrapper for the read loop in camcommunicator
 * mcp main should make a thread for this
 */
static void* TheGoodReadLoop(void* arg)
{
  nameThread("GoodSC");
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (TheGoodComm->openClient(THEGOOD_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      errorshown = true;
    }
  }
  bprintf(startup, "talking to The Good Star Camera");

  sendTheGoodCommand("Oconf");  //request configuration data

  while(true) {
    TheGoodComm->readLoop(&TheGoodparseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}
static void* TheBadReadLoop(void* arg)
{
  nameThread("BadSC");
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (TheBadComm->openClient(THEBAD_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      errorshown = true;
    }
  }
  bprintf(startup, "talking to The Bad Star Camera");

  sendTheBadCommand("Oconf");  //request configuration data

  while(true) {
    TheBadComm->readLoop(&TheBadparseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}
static void* TheUglyReadLoop(void* arg)
{
  nameThread("UglySC");
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (TheUglyComm->openClient(THEUGLY_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      errorshown = true;
    }
  }
  bprintf(startup, "talking to The Ugly Star Camera");

  sendTheUglyCommand("Oconf");  //request configuration data

  while(true) {
    TheUglyComm->readLoop(&TheUglyparseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}


static string TheGoodparseReturn(string rtnStr)
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
    else if (Rstr.substr(0,6) == "<conf>") //contains The Ugly config data
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
    //otherwise it is success notice for another command

  } else { //response is exposure data
    TheGoodComm->interpretReturn(rtnStr, &camRtn[(i_cam+1)%2]);
    i_cam = (i_cam+1)%2;
  }
  return "";  //doesn't send a response back to camera
}
static string TheBadparseReturn(string rtnStr)
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
    else if (Rstr.substr(0,6) == "<conf>") //contains The Ugly config data
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
    TheBadComm->interpretReturn(rtnStr, &camRtn[(i_cam+1)%2]);
    i_cam = (i_cam+1)%2;
  }
  return "";  //doesn't send a response back to camera
}
static string TheUglyparseReturn(string rtnStr)
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
    else if (Rstr.substr(0,6) == "<conf>") //contains The Ugly config data
    {
      Rstr = Rstr.substr(6, Rstr.size()-6);
      istringstream sin;
      sin.str(Rstr);
      double temp;  //value sent for expTime is a double
      sin >> CommandData.theugly.expInt
	>> temp
	>> CommandData.theugly.focusRes
	>> CommandData.theugly.moveTol
	>> CommandData.theugly.maxBlobs
	>> CommandData.theugly.grid
	>> CommandData.theugly.threshold
	>> CommandData.theugly.minBlobDist;
      CommandData.theugly.expTime = (int)(temp * 1000);
    }
    //otherwise it is success notice for another command

  } else { //response is exposure data
    TheUglyComm->interpretReturn(rtnStr, &camRtn[(i_cam+1)%2]);
    i_cam = (i_cam+1)%2;
  }
  return "";  //doesn't send a response back to camera
}

