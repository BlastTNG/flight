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
#include "channels_tng.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "command_struct.h"
}
#include "sbsccommunicator.h"
#include "sbsc_protocol.h"

//allow any host to be the star camera
#define CAM_SERVERNAME "192.168.1.11"

#define SBSC_SERIAL "110794466"

extern "C" void nameThread(const char*);  /* in mcp.c */

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
  int next_i = (CommandData.cam.i_uplink_w + 1) % SBSC_CMD_Q_SIZE;
  if (next_i == CommandData.cam.i_uplink_r) {
    bprintf(err, "Overflow of star camera command queue");
    return -1;
  } else {
    strncpy(CommandData.cam.uplink_cmd[CommandData.cam.i_uplink_w],
	cmd, SBSC_COMM_BUF_SIZE);
    CommandData.cam.uplink_cmd[next_i][SBSC_COMM_BUF_SIZE-1] = '\0';
    CommandData.cam.i_uplink_w = next_i;
  }
  return 0;
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
  static int blobindex = 0;

  channel_t* forceAddr = NULL;
  channel_t* expIntAddr = NULL;
  channel_t* expTimeAddr = NULL;
  channel_t* focResAddr = NULL;
  channel_t* moveTolAddr = NULL;
  channel_t* maxBlobAddr = NULL;
  channel_t* gridAddr = NULL;
  channel_t* threshAddr = NULL;
  channel_t* delayAddr = NULL;
  channel_t* blobMdistAddr = NULL;

  channel_t* sbscFrameAddr = NULL;
  channel_t* sbscMeanAddr = NULL;
  channel_t* sbscSigmaAddr = NULL;
  channel_t* sbscTimeAddr = NULL;
  channel_t* sbscUsecAddr = NULL;
  channel_t* sbscCcdTempAddr = NULL;
  channel_t* sbscFocPosAddr = NULL;
  channel_t* sbscNumBlobsAddr = NULL;

  channel_t* sbscBlobX[5];
  channel_t* sbscBlobY[5];
  channel_t* sbscBlobF[5];
  channel_t* sbscBlobS[5];
  channel_t* sbscBlobIdxAddr;

  channel_t* sbscRAAddr = NULL;
  channel_t* sbscDECAddr = NULL;

  //initialization
  if (firsttime) {
    firsttime = 0;
    forceAddr = channels_find_by_name("force_sbsc");
    expIntAddr = channels_find_by_name("exp_int_sbsc");
    expTimeAddr = channels_find_by_name("exp_time_sbsc");
    focResAddr = channels_find_by_name("foc_res_sbsc");
    moveTolAddr = channels_find_by_name("move_tol_sbsc");
    maxBlobAddr = channels_find_by_name("maxblob_sbsc");
    gridAddr = channels_find_by_name("grid_sbsc");
    threshAddr = channels_find_by_name("thresh_sbsc");
    delayAddr = channels_find_by_name("delay_sbsc");
    blobMdistAddr = channels_find_by_name("mdist_sbsc");

    sbscFrameAddr = channels_find_by_name("frame_sbsc");
    sbscMeanAddr = channels_find_by_name("mapmean_sbsc");
    sbscSigmaAddr = channels_find_by_name("mapsigma_sbsc");
    sbscTimeAddr = channels_find_by_name("sec_sbsc");
    sbscUsecAddr = channels_find_by_name("usec_sbsc");
    sbscCcdTempAddr = channels_find_by_name("ccd_t_sbsc");
    sbscFocPosAddr = channels_find_by_name("focpos_sbsc");
    sbscNumBlobsAddr = channels_find_by_name("nblobs_sbsc");

    for (int i=0; i<3; i++) {
      char buf[99];
      sprintf(buf, "blob%02d_x_sbsc", i);
      sbscBlobX[i] = channels_find_by_name(buf);
      sprintf(buf, "blob%02d_y_sbsc", i);
      sbscBlobY[i] = channels_find_by_name(buf);
      sprintf(buf, "blob%02d_f_sbsc", i);
      sbscBlobF[i] = channels_find_by_name(buf);
      sprintf(buf, "blob%02d_s_sbsc", i);
      sbscBlobS[i] = channels_find_by_name(buf);
    }
    sbscBlobIdxAddr = channels_find_by_name("blob_idx_sbsc");
    sbscRAAddr = channels_find_by_name("ra_sbsc");
    sbscDECAddr = channels_find_by_name("dec_sbsc");
  }

  SET_VALUE(forceAddr, CommandData.cam.forced);
  SET_VALUE(expIntAddr, CommandData.cam.expInt);
  SET_VALUE(expTimeAddr, CommandData.cam.expTime);
  SET_VALUE(focResAddr, CommandData.cam.focusRes);
  SET_VALUE(moveTolAddr, CommandData.cam.moveTol);
  SET_VALUE(maxBlobAddr, CommandData.cam.maxBlobs);
  SET_VALUE(gridAddr, CommandData.cam.grid);
  SET_VALUE(threshAddr, (int)(CommandData.cam.threshold*1000));
  SET_VALUE(delayAddr, (int)(CommandData.cam.delay*1000));
  SET_VALUE(blobMdistAddr, CommandData.cam.minBlobDist);

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
    SET_VALUE(sbscFrameAddr, sbsc->frameNum);
    SET_VALUE(sbscMeanAddr, (int)sbsc->mapmean);
    SET_VALUE(sbscSigmaAddr, (int)(sbsc->sigma*10));
    SET_VALUE(sbscTimeAddr, sbsc->imagestarttime.tv_sec);
    SET_VALUE(sbscUsecAddr, sbsc->imagestarttime.tv_usec);
    //it looks like this is in deg C. just scale to get better resolution
    SET_VALUE(sbscCcdTempAddr, (int)(sbsc->ccdtemperature*100));
    SET_VALUE(sbscFocPosAddr, (int)(sbsc->focusposition*10));
    SET_VALUE(sbscNumBlobsAddr, sbsc->numblobs);

    for (int i=0; i<3; i++)
    {
      SET_VALUE(sbscBlobX[i],(unsigned int)(sbsc->x[blobindex * 3 + i]/CAM_WIDTH*SHRT_MAX));
      SET_VALUE(sbscBlobY[i],(unsigned int)(sbsc->y[blobindex * 3 + i]/CAM_WIDTH*SHRT_MAX));
      SET_VALUE(sbscBlobF[i], (unsigned int)sbsc->flux[blobindex * 3 + i]);
      unsigned int snr = (sbsc->snr[blobindex * 3 + i] >= SHRT_MAX / 100.0) ? 
	SHRT_MAX : (unsigned int)sbsc->snr[blobindex * 3 + i]*100;
      SET_VALUE(sbscBlobS[i], snr);
    }
    SET_VALUE(sbscBlobIdxAddr, blobindex);
    blobindex = (blobindex + 1) % 5;
    SET_VALUE(sbscRAAddr, (int)(sbsc->ra*1000));
    SET_VALUE(sbscDECAddr, (int)(sbsc->dec*1000));
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

  while(true) {
    errorshown = false;
    while (camComm->openClient(CAM_SERVERNAME) < 0) {
      if (!errorshown) {
	bprintf(err, "failed to accept camera connection");
	errorshown = true;
      }
    }
    bprintf(startup, "talking to camera");

    camComm->readLoop(&parseReturn);
    bprintf(warning, "unexpected return from readLoop");
    camComm->closeConnection();
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

