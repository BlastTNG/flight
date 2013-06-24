/* mcp: the Spider master ccntrol program
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
#include <string.h>
#include <unistd.h>

extern "C" {
#include "blast.h"
#include "tx.h"
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "command_struct.h"
}
#include "camcommunicator.h"
#include "camstruct.h"

#include <stdlib.h>
#include "pyramid.h"

using namespace std;

#define THEGOOD_SERVERNAME "192.168.1.11"
#define THEBAD_SERVERNAME  "192.168.1.12"
#define THEUGLY_SERVERNAME "192.168.1.13"


extern "C" void nameThread(const char*);  /* in mcp.c */

extern "C" short int InCharge;		  /* in tx.c */

extern "C" int EthernetSC[3];      /* tx.c */

string cam_serial[3]={"110794466","08073506","08073507"};
extern double goodPos[10];	/* table.cpp */
short int bsc_trigger;		/* flag for boresite exposure, set by motors.c */
extern short int exposing;	//in table.cpp
extern short int docalc;	//in table.cpp
extern short int zerodist[10];	//in table.cpp
double trigPos[10];

//Stuff for Pyramid
#define CAT "/data/etc/spider/gsc_mag08_res20.bin"
#define KCAT "/data/etc/spider/k.bin"
double FOV = 2.5*M_PI/180.0;
double ra_thegood, dec_thegood, roll_thegood;
double ra_thebad, dec_thebad, roll_thebad;
double ra_theugly, dec_theugly, roll_theugly;
Pyramid pyr;

static CamCommunicator* TheGoodComm;
static CamCommunicator* TheBadComm;
static CamCommunicator* TheUglyComm;
static pthread_t TheGoodcomm_id;
static pthread_t TheBadcomm_id;
static pthread_t TheUglycomm_id;

static void SolveField(StarcamReturn* solrtn, double& ra0, double& dec0, double& r0);
static void* TheGoodReadLoop(void* arg);
static void* TheBadReadLoop(void* arg);
static void* TheUglyReadLoop(void* arg);
static string TheGoodparseReturn(string rtnStr);
static string TheBadparseReturn(string rtnStr);
static string TheUglyparseReturn(string rtnStr);

static StarcamReturn camRtn[3][3];
static short int i_cam[3] = {0,0,0}; //read index in above buffer

extern "C" {

static NiosStruct* GetSCNiosAddr(const char* field, int which)
{
	  char buffer[FIELD_LEN];
	  switch (which) {
		  case 0:  
	            sprintf(buffer, "%s_%s", field, "thegood");
		    break;
		  case 1:
		    sprintf(buffer, "%s_%s", field, "thebad");
		    break;
		  case 2:
		    sprintf(buffer, "%s_%s", field, "theugly");
		    break;
	  }
	  return GetNiosAddr(buffer);
}

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
  pyr.Init(FOV, (char *)CAT, (char *)KCAT);
  TheGoodComm = new CamCommunicator();
  TheBadComm  = new CamCommunicator();
  TheUglyComm = new CamCommunicator();
  pthread_create(&TheGoodcomm_id, NULL, &TheGoodReadLoop, NULL);
  pthread_create(&TheBadcomm_id, NULL, &TheBadReadLoop, NULL);
  pthread_create(&TheUglycomm_id, NULL, &TheUglyReadLoop, NULL);
}

/*
 * send exposure commands to cameras
 * called in mcp fast controls
 */
void cameraTriggers()
{
  static int bscwait,rscwait;
  static int exposecount;
  static int firsttime;
  if (firsttime) {
    firsttime=0;
    bscwait = 0;
    rscwait = 0;
    exposecount = 0;
  }
  if (bsc_trigger) {
		if (!CommandData.StarCam[0].paused) sendTheGoodCommand("CtrigExp");
		bsc_trigger = 0;
  }   
  rscwait++;
  if ((rscwait%10)==0) {
	if (!CommandData.StarCam[1].paused) {
		sendTheBadCommand("CtrigExp");
		exposing = 1;
		rscwait = 0;
	}
	if (!CommandData.StarCam[2].paused) {
		sendTheUglyCommand("CtrigExp");
		exposing = 1;
		rscwait = 0;
	}
	for (int i=0; i<10; i++) {
		if (goodPos[i] == 0.0) { 
			trigPos[i] = ACSData.enc_table;
			zerodist[i] = 1;
		}
	}
  }

  if (exposing) {
	exposecount++;
	if (exposecount == 2) {
		exposecount = 0;
		exposing = 0;
		docalc = 1;
	}
   }
}

/*
 * update all camera related fields
 * meant to be called in mcp slow loop (5Hz)
 */
void cameraFields(int which)
{
  static int firsttime[3] = {1, 1, 1};
  StarcamReturn* sc = NULL;
  static bool unrecFlag = false;
  static int blobindex[3] = {0,0,0};
  static unsigned long int posFrame;
  static int i_cam_local;

  static NiosStruct* ForceAddr[3];
  static NiosStruct* ExpIntAddr[3];
  static NiosStruct* ExpTimeAddr[3];
  static NiosStruct* FocResAddr[3];
  static NiosStruct* MoveTolAddr[3];
  static NiosStruct* MaxBlobAddr[3];
  static NiosStruct* GridAddr[3];
  static NiosStruct* ThreshAddr[3];
  static NiosStruct* BlobMdistAddr[3];

  static NiosStruct* FrameAddr[3];
  static NiosStruct* MeanAddr[3];
  static NiosStruct* SigmaAddr[3];
  static NiosStruct* TimeAddr[3];
  static NiosStruct* UsecAddr[3];
  static NiosStruct* CcdTempAddr[3];
  static NiosStruct* FocPosAddr[3];
  static NiosStruct* NumBlobsAddr[3];
  static NiosStruct* RaAddr[3];
  static NiosStruct* DecAddr[3];
  static NiosStruct* RollAddr[3];

  static NiosStruct* Blob0XAddr[3];
  static NiosStruct* Blob1XAddr[3];
  static NiosStruct* Blob2XAddr[3];
  static NiosStruct* Blob0YAddr[3];
  static NiosStruct* Blob1YAddr[3];
  static NiosStruct* Blob2YAddr[3];
  static NiosStruct* Blob0FAddr[3];
  static NiosStruct* Blob1FAddr[3];
  static NiosStruct* Blob2FAddr[3];
  static NiosStruct* Blob0SAddr[3];
  static NiosStruct* Blob1SAddr[3];
  static NiosStruct* Blob2SAddr[3];
  static NiosStruct* BlobIdxAddr[3];

  //initialization
  if (firsttime[which]) {
    firsttime[which] = 0;
    ForceAddr[which] = GetSCNiosAddr("force",which);
    ExpIntAddr[which] = GetSCNiosAddr("exp_int",which);
    ExpTimeAddr[which] = GetSCNiosAddr("exp_time",which);
    FocResAddr[which] = GetSCNiosAddr("foc_res",which);
    MoveTolAddr[which] = GetSCNiosAddr("move_tol",which);
    MaxBlobAddr[which] = GetSCNiosAddr("maxblob",which);
    GridAddr[which] = GetSCNiosAddr("grid",which);
    ThreshAddr[which] = GetSCNiosAddr("thresh",which);
    BlobMdistAddr[which] = GetSCNiosAddr("mdist",which);

    FrameAddr[which] = GetSCNiosAddr("frame",which);
    MeanAddr[which] = GetSCNiosAddr("mapmean",which);
    SigmaAddr[which] = GetSCNiosAddr("mapsigma",which);
    TimeAddr[which] = GetSCNiosAddr("sec",which);
    UsecAddr[which] = GetSCNiosAddr("usec",which);
    CcdTempAddr[which] = GetSCNiosAddr("ccd_t",which);
    FocPosAddr[which] = GetSCNiosAddr("focpos",which);
    NumBlobsAddr[which] = GetSCNiosAddr("nblobs",which);
    RaAddr[which] = GetSCNiosAddr("ra",which);
    DecAddr[which] = GetSCNiosAddr("dec",which);
    RollAddr[which] = GetSCNiosAddr("roll",which);

    Blob0XAddr[which] = GetSCNiosAddr("blob00_x",which);
    Blob1XAddr[which] = GetSCNiosAddr("blob01_x",which);
    Blob2XAddr[which] = GetSCNiosAddr("blob02_x",which);
    Blob0YAddr[which] = GetSCNiosAddr("blob00_y",which);
    Blob1YAddr[which] = GetSCNiosAddr("blob01_y",which);
    Blob2YAddr[which] = GetSCNiosAddr("blob02_y",which);
    Blob0FAddr[which] = GetSCNiosAddr("blob00_f",which);
    Blob1FAddr[which] = GetSCNiosAddr("blob01_f",which);
    Blob2FAddr[which] = GetSCNiosAddr("blob02_f",which);
    Blob0SAddr[which] = GetSCNiosAddr("blob00_s",which);
    Blob1SAddr[which] = GetSCNiosAddr("blob01_s",which);
    Blob2SAddr[which] = GetSCNiosAddr("blob02_s",which);

    BlobIdxAddr[which] = GetSCNiosAddr("blob_idx",which);
  }

  WriteData(ForceAddr[which], CommandData.StarCam[which].paused, NIOS_QUEUE);
  WriteData(ExpIntAddr[which], CommandData.StarCam[which].expInt, NIOS_QUEUE);
  WriteData(ExpTimeAddr[which], CommandData.StarCam[which].expTime, NIOS_QUEUE);
  WriteData(FocResAddr[which], CommandData.StarCam[which].focusRes, NIOS_QUEUE);
  WriteData(MoveTolAddr[which], CommandData.StarCam[which].moveTol, NIOS_QUEUE);
  WriteData(MaxBlobAddr[which], CommandData.StarCam[which].maxBlobs, NIOS_QUEUE);
  WriteData(GridAddr[which], CommandData.StarCam[which].grid, NIOS_QUEUE);
  WriteData(ThreshAddr[which], (int)(CommandData.StarCam[which].threshold*1000), NIOS_QUEUE);
  WriteData(BlobMdistAddr[which], CommandData.StarCam[which].minBlobDist, NIOS_QUEUE);

//-----------------

  i_cam_local = i_cam[which];
  //persistently identify cameras by serial number (camID)
  if (camRtn[which][i_cam_local].camID == cam_serial[which])  {
    sc = &camRtn[which][i_cam_local];
    unrecFlag = false;
  }
  else if (!unrecFlag) { //don't keep printing same error
//    bprintf(err, "unrecognized camera ID");
    sc = NULL;
    unrecFlag = true;
  }

  if (sc != NULL) {
    	WriteData(FrameAddr[which], sc->frameNum, NIOS_QUEUE);
    	WriteData(MeanAddr[which], (int)sc->mapmean, NIOS_QUEUE);
    	WriteData(SigmaAddr[which], (int)(sc->sigma*10), NIOS_QUEUE);
    	WriteData(TimeAddr[which], sc->imagestarttime.tv_sec, NIOS_QUEUE);
    	WriteData(UsecAddr[which], sc->imagestarttime.tv_usec, NIOS_QUEUE);
    	//it looks like this is in deg C. just scale to get better resolution
    	WriteData(CcdTempAddr[which], (int)(sc->ccdtemperature*100), NIOS_QUEUE);
    	WriteData(FocPosAddr[which], (int)(sc->focusposition*10), NIOS_QUEUE);
    	WriteData(NumBlobsAddr[which], sc->numblobs, NIOS_QUEUE);

	WriteData(RaAddr[which], (int)(ra_thegood*100*180/M_PI), NIOS_QUEUE);
	WriteData(DecAddr[which], (int)(dec_thegood*100*180/M_PI), NIOS_QUEUE);
	WriteData(RollAddr[which], (int)(roll_thegood*100*180/M_PI), NIOS_QUEUE);
	if (sc->numblobs > 0) {
		WriteData(Blob0XAddr[which],(unsigned int)(sc->x[blobindex[which] * 3 + 0]/CAM_WIDTH*SHRT_MAX),
			  NIOS_QUEUE);
    		WriteData(Blob1XAddr[which],(unsigned int)(sc->x[blobindex[which] * 3 + 1]/CAM_WIDTH*SHRT_MAX),
			  NIOS_QUEUE);
    		WriteData(Blob2XAddr[which],(unsigned int)(sc->x[blobindex[which] * 3 + 2]/CAM_WIDTH*SHRT_MAX),
			  NIOS_QUEUE);
		WriteData(Blob0YAddr[which],(unsigned int)(sc->y[blobindex[which] * 3 + 0]/CAM_WIDTH*SHRT_MAX),
			  NIOS_QUEUE);
		WriteData(Blob1YAddr[which],(unsigned int)(sc->y[blobindex[which] * 3 + 1]/CAM_WIDTH*SHRT_MAX),
			  NIOS_QUEUE);
		WriteData(Blob2YAddr[which],(unsigned int)(sc->y[blobindex[which] * 3 + 2]/CAM_WIDTH*SHRT_MAX),
			  NIOS_QUEUE);
		WriteData(Blob0FAddr[which], (unsigned int)sc->flux[blobindex[which] * 3 + 0], NIOS_QUEUE);
		WriteData(Blob1FAddr[which], (unsigned int)sc->flux[blobindex[which] * 3 + 1], NIOS_QUEUE);
		WriteData(Blob2FAddr[which], (unsigned int)sc->flux[blobindex[which] * 3 + 2], NIOS_QUEUE);
		unsigned int snr = (sc->snr[blobindex[which] * 3 + 0] >= SHRT_MAX / 100.0) ? 
			SHRT_MAX : (unsigned int)sc->snr[blobindex[which] * 3 + 0]*100;
		WriteData(Blob0SAddr[which], snr, NIOS_QUEUE);
		snr = (sc->snr[blobindex[which] * 3 + 1] >= SHRT_MAX / 100.0) ? 
			SHRT_MAX : (unsigned int)sc->snr[blobindex[which] * 3 + 1]*100;
		WriteData(Blob1SAddr[which], snr, NIOS_QUEUE);
		snr = (sc->snr[blobindex[which] * 3 + 2] >= SHRT_MAX / 100.0) ? 
			SHRT_MAX : (unsigned int)sc->snr[blobindex[which] * 3 + 2]*100;
		WriteData(Blob2SAddr[which], snr, NIOS_QUEUE);
		
	}
	WriteData(BlobIdxAddr[which], blobindex[which], NIOS_QUEUE);
	blobindex[which] = (blobindex[which] + 1) % 5;
	if ((which == 1) && (sc->numblobs > 8)) {
		for (int j=0; j<10; j++) {
			if ((goodPos[j] == 0.0) && (sc->frameNum != posFrame)) {
				goodPos[j] = trigPos[j]; //overwrite the first 'dead' one it finds
				posFrame = sc->frameNum;
				break;
			}
		}
	}
  }	

}

}       //extern "C"

/*
  runs pyramid and solves the field
*/
static void SolveField(StarcamReturn* solrtn, double& ra0, double& dec0, double& r0) {
  double plate_scale = 9.3/180./3600.*M_PI; 
  double XC = 1530/2;
  double YC = 1020/2;
  double FTOL = 20.*M_PI/180./3600.;//star-blob association angular tolerance (default 20 arcsec as in netisc)
  int retval_pyr,k;
  int n_blobs = 0;
  solution_t *sol = new solution_t [MAXSOLUTION];
  int nsol;

  n_blobs = ((solrtn->numblobs > 15) ? 15 : solrtn->numblobs);
  if (n_blobs > 4) {
	  double *x = new double [n_blobs];
	  double *y = new double [n_blobs];
	  double *x_p = new double [n_blobs];
	  double *y_p = new double [n_blobs];
	  memset(x, 0, sizeof(x) );
	  memset(y, 0, sizeof(y) );
	  memset(x_p, 0, sizeof(x_p) );
	  memset(y_p, 0, sizeof(y_p) );

	  for (k = 0; k < n_blobs; k++) {
		x[k] = solrtn->x[k];
		y[k] = 1020 - solrtn->y[k];
		x_p[k] = (x[k] - XC)*plate_scale;
		y_p[k] = (y[k] - YC)*plate_scale;
  	}
  	//launch pyramid
  	retval_pyr = pyr.GetSolution(FTOL, x_p, y_p,n_blobs, &sol, &nsol, &ra0, &dec0, &r0);
  	delete[] x;
  	delete[] y;
  	delete[] x_p;
  	delete[] y_p;
  }
}

/*
 * wrapper for the read loop in camcommunicator
 * mcp main should make a thread for this
 */
static void* TheGoodReadLoop(void* arg)
{
  int firsttime=1;
  nameThread("GoodSC");
  while (!InCharge) {
    if (firsttime==1) {
      bprintf(info,"Not in charge, waiting....");
      firsttime=0;
    }
    usleep(20000);
  }
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (TheGoodComm->openClient(THEGOOD_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      EthernetSC[0]=3;
      errorshown = true;
    }
  }
  bprintf(startup, "talking to The Good Star Camera");
  EthernetSC[0]=0;

  sendTheGoodCommand("Oconf");  //request configuration data

  while(true) {
    TheGoodComm->readLoop(&TheGoodparseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}
static void* TheBadReadLoop(void* arg)
{
  int firsttime=1;
  nameThread("BadSC");
  while (!InCharge) {
    if (firsttime==1) {
      bprintf(info,"Not in charge, waiting....");
      firsttime=0;
    }
    usleep(20000);
  }
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (TheBadComm->openClient(THEBAD_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      EthernetSC[1]=3;
      errorshown = true;
    }
  }
  bprintf(startup, "talking to The Bad Star Camera");
  EthernetSC[1]=0;

  sendTheBadCommand("Oconf");  //request configuration data

  while(true) {
    TheBadComm->readLoop(&TheBadparseReturn);
    //sleep(1);	//catchall for varous busy-waiting scenarios
  }

  return NULL;
}
static void* TheUglyReadLoop(void* arg)
{
  int firsttime=1;
  nameThread("UglySC");
  while (!InCharge) {
    if (firsttime==1) {
      bprintf(info,"Not in charge, waiting....");
      firsttime=0;
    }
    usleep(20000);
  }
  bputs(startup, "startup\n");
  bool errorshown = false;

  while (TheUglyComm->openClient(THEUGLY_SERVERNAME) < 0) {
    if (!errorshown) {
      bprintf(err, "failed to accept camera connection");
      EthernetSC[2]=3;
      errorshown = true;
    }
  }
  bprintf(startup, "talking to The Ugly Star Camera");
  EthernetSC[2]=0;

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
      sin >> CommandData.StarCam[0].expInt
	>> temp
	>> CommandData.StarCam[0].focusRes
	>> CommandData.StarCam[0].moveTol
	>> CommandData.StarCam[0].maxBlobs
	>> CommandData.StarCam[0].grid
	>> CommandData.StarCam[0].threshold
	>> CommandData.StarCam[0].minBlobDist;
      CommandData.StarCam[0].expTime = (int)(temp * 1000);
    }
    //otherwise it is success notice for another command

  } else { //response is exposure data
    TheGoodComm->interpretReturn(rtnStr, &camRtn[0][(i_cam[0]+1)%2]);
    SolveField(&camRtn[0][(i_cam[0]+1)%2],ra_thegood,dec_thegood,roll_thegood);
    i_cam[0] = (i_cam[0]+1)%2;
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
      sin >> CommandData.StarCam[1].expInt
	>> temp
	>> CommandData.StarCam[1].focusRes
	>> CommandData.StarCam[1].moveTol
	>> CommandData.StarCam[1].maxBlobs
	>> CommandData.StarCam[1].grid
	>> CommandData.StarCam[1].threshold
	>> CommandData.StarCam[1].minBlobDist;
      CommandData.StarCam[1].expTime = (int)(temp * 1000);
    }
    //otherwise it is success notice for another command

  } else { //response is exposure data
    TheBadComm->interpretReturn(rtnStr, &camRtn[1][(i_cam[1]+1)%2]);
    SolveField(&camRtn[1][(i_cam[1]+1)%2],ra_thebad,dec_thebad,roll_thebad);
    i_cam[1] = (i_cam[1]+1)%2;
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
      sin >> CommandData.StarCam[2].expInt
	>> temp
	>> CommandData.StarCam[2].focusRes
	>> CommandData.StarCam[2].moveTol
	>> CommandData.StarCam[2].maxBlobs
	>> CommandData.StarCam[2].grid
	>> CommandData.StarCam[2].threshold
	>> CommandData.StarCam[2].minBlobDist;
      CommandData.StarCam[2].expTime = (int)(temp * 1000);
    }
    //otherwise it is success notice for another command

  } else { //response is exposure data
    TheUglyComm->interpretReturn(rtnStr, &camRtn[2][(i_cam[2]+1)%2]);
    SolveField(&camRtn[2][(i_cam[2]+1)%2],ra_theugly,dec_theugly,roll_theugly);
    i_cam[2] = (i_cam[2]+1)%2;
  }
  return "";  //doesn't send a response back to camera
}

