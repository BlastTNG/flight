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
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>      //for gethostbyname
#include <cstdlib>

extern "C" {
#include "blast.h"
#include "tx.h"
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "udp.h"
#include "command_struct.h"
}
#include "camcommunicator.h"
#include "camstruct.h"

#include <stdlib.h>
#include "pyramid.h"

using namespace std;

#define SC_TIMEOUT 100 /* milliseconds */

extern "C" void radec2azel(double ra, double dec, time_t lst, double lat, double *az, double *el);
extern "C" void nameThread(const char*);  /* in mcp.c */
extern "C" short int InCharge;		  /* in tx.c */
extern "C" int EthernetSC[3];      /* tx.c */

const char* thegood_servername = "good";//"192.168.1.247";
const char* thebad_servername = "bad";//"192.168.1.248";
const char* theugly_servername = "ugly";//"192.168.1.249";

string cam_serial[3]={"110794466","08073506","08073507"};
extern double goodPos[10];	/* table.cpp */
short int bsc_trigger;		/* flag for boresite exposure, set by motors.c */
extern short int exposing;	//in table.cpp
extern short int docalc;	//in table.cpp
//extern short int zerodist[10];	//in table.cpp
//double trigPos[10];

//Stuff for Pyramid
#define CAT "/data/etc/spider/gsc_mag08_res20.bin"
#define KCAT "/data/etc/spider/k.bin"
double FOV = 2.5*M_PI/180.0;
double SCra[3], SCdec[3], SCroll[3], SCaz[3], SCel[3];
#define BAD_AZ_OFF -105.0
#define UGLY_AZ_OFF 105.0
Pyramid pyr;

static pthread_t send_id;
static pthread_t read_id;

static void SolveField(StarcamReturn* solrtn, double& ra0, double& dec0, double& r0, int which);
static void* SendLoop(void* arg);
static void* ReadLoop(void* arg);
static string ParseReturn(string rtnStr, int which);

static StarcamReturn camRtn[3][3];
static short int i_cam[3] = {0,0,0}; //read index in above buffer

const char* SCcmd[3];
int SCcmd_flag[3] = {0,0,0};
int trigger_flag = 0;

extern "C" {

static NiosStruct* GetSCNiosAddr(const char* field, int which)
{
	  char buffer[FIELD_LEN];
	  switch (which) {
		  case 0:  
	            sprintf(buffer, "%s_%s", field, "g");
		    break;
		  case 1:
		    sprintf(buffer, "%s_%s", field, "b");
		    break;
		  case 2:
		    sprintf(buffer, "%s_%s", field, "u");
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
  SCcmd[0] = cmd;
  SCcmd_flag[0] = 1;
  return 0;
}
int sendTheBadCommand(const char *cmd)
{
  if (!InCharge) return 0;
  SCcmd[1] = cmd;
  SCcmd_flag[1] = 1;
  return 0;
}
int sendTheUglyCommand(const char *cmd)
{
  if (!InCharge) return 0;
  SCcmd[2] = cmd;
  SCcmd_flag[2] = 1;
  return 0;
}

/*
 * open a connection the the star camera computers
 * also creates the communications threads
 */
void openSC()
{
  pyr.Init(FOV, (char *)CAT, (char *)KCAT);
  pthread_create(&send_id, NULL, &SendLoop, NULL);
  pthread_create(&read_id, NULL, &ReadLoop, NULL);
}

/*
 * send exposure commands to cameras
 * called in mcp fast controls
 */
void cameraTriggers()
{
  static int rscwait;
  static int exposecount;
  static int firsttime;
  if (firsttime) {
    firsttime=0;
    rscwait = 0;
    exposecount = 0;
  }
  if (bsc_trigger) {
		if (!CommandData.StarCam[0].paused) sendTheGoodCommand("CtrigExp");
		bsc_trigger = 0;
  }   
  rscwait++;
  if (rscwait>(ACSData.bbc_rate*CommandData.rsc_wait)) {
	  trigger_flag = 1;
	  //bprintf(info,"EXPOSING...");
	  exposing = 1;
	  rscwait = 0;
//	  for (int i=0; i<10; i++) {
//		if (goodPos[i] == 0.0) { 
//			trigPos[i] = ACSData.enc_table;
//			zerodist[i] = 1;
//		}
//	  }
  }

  if (exposing) {
	exposecount++;
	if (exposecount>(ACSData.bbc_rate*(2+CommandData.StarCam[1].expTime/1000))) {
		exposecount = 0;
		//bprintf(info,"...DONE");
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
//  static unsigned long int posFrame;
  static int i_cam_local;

  static NiosStruct* ForceAddr[3];
  static NiosStruct* ExpIntAddr[3];
  static NiosStruct* ExpTimeAddr[3];
  static NiosStruct* FocResAddr[3];
  static NiosStruct* FocRngAddr[3];
  static NiosStruct* MoveTolAddr[3];
  static NiosStruct* MaxBlobAddr[3];
  static NiosStruct* GridAddr[3];
  static NiosStruct* ThreshAddr[3];
  static NiosStruct* BlobMdistAddr[3];
  static NiosStruct* PlateAddr[3];

  static NiosStruct* FrameAddr[3];
  static NiosStruct* MeanAddr[3];
  static NiosStruct* SigmaAddr[3];
  static NiosStruct* TimeAddr[3];
  static NiosStruct* UsecAddr[3];
  static NiosStruct* CcdTempAddr[3];
  static NiosStruct* CpuTempAddr[3];
  static NiosStruct* FocPosAddr[3];
  static NiosStruct* NumBlobsAddr[3];
  static NiosStruct* RaAddr[3];
  static NiosStruct* DecAddr[3];
  static NiosStruct* RollAddr[3];
  static NiosStruct* AzAddr[3];
  static NiosStruct* ElAddr[3];

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
  static NiosStruct* DelayAddr;
//  static NiosStruct* TriggerAddr;

  //initialization
  if (firsttime[which]) {
    firsttime[which] = 0;
    ForceAddr[which] = GetSCNiosAddr("force",which);
    ExpIntAddr[which] = GetSCNiosAddr("exp_int",which);
    ExpTimeAddr[which] = GetSCNiosAddr("exp_time",which);
    FocResAddr[which] = GetSCNiosAddr("foc_res",which);
    FocRngAddr[which] = GetSCNiosAddr("foc_rng",which);
    MoveTolAddr[which] = GetSCNiosAddr("move_tol",which);
    MaxBlobAddr[which] = GetSCNiosAddr("maxblob",which);
    GridAddr[which] = GetSCNiosAddr("grid",which);
    ThreshAddr[which] = GetSCNiosAddr("thresh",which);
    BlobMdistAddr[which] = GetSCNiosAddr("mdist",which);
    PlateAddr[which] = GetSCNiosAddr("plate",which);

    FrameAddr[which] = GetSCNiosAddr("frame",which);
    MeanAddr[which] = GetSCNiosAddr("mapmean",which);
    SigmaAddr[which] = GetSCNiosAddr("mapsigma",which);
    TimeAddr[which] = GetSCNiosAddr("sec",which);
    UsecAddr[which] = GetSCNiosAddr("usec",which);
    CcdTempAddr[which] = GetSCNiosAddr("t_ccd",which);
    CpuTempAddr[which] = GetSCNiosAddr("t_cpu",which);
    FocPosAddr[which] = GetSCNiosAddr("focpos",which);
    NumBlobsAddr[which] = GetSCNiosAddr("nblobs",which);
    RaAddr[which] = GetSCNiosAddr("ra",which);
    DecAddr[which] = GetSCNiosAddr("dec",which);
    RollAddr[which] = GetSCNiosAddr("roll",which);
    AzAddr[which] = GetSCNiosAddr("az",which);
    ElAddr[which] = GetSCNiosAddr("el",which);

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
    DelayAddr = GetNiosAddr("trig_delay");
//    TriggerAddr = GetNiosAddr("bsc_trigger");
  }

  WriteData(ForceAddr[which], CommandData.StarCam[which].paused, NIOS_QUEUE);
  WriteData(ExpIntAddr[which], CommandData.StarCam[which].expInt, NIOS_QUEUE);
  WriteData(ExpTimeAddr[which], CommandData.StarCam[which].expTime, NIOS_QUEUE);
  WriteData(FocResAddr[which], CommandData.StarCam[which].focusRes, NIOS_QUEUE);
  WriteData(FocRngAddr[which], CommandData.StarCam[which].focusRng, NIOS_QUEUE);
  WriteData(MoveTolAddr[which], CommandData.StarCam[which].moveTol, NIOS_QUEUE);
  WriteData(MaxBlobAddr[which], CommandData.StarCam[which].maxBlobs, NIOS_QUEUE);
  WriteData(GridAddr[which], CommandData.StarCam[which].grid, NIOS_QUEUE);
  WriteData(ThreshAddr[which], (int)(CommandData.StarCam[which].threshold*1000), NIOS_QUEUE);
  WriteData(BlobMdistAddr[which], CommandData.StarCam[which].minBlobDist, NIOS_QUEUE);
  WriteData(PlateAddr[which], (int)(CommandData.StarCam[which].platescale*1000), NIOS_QUEUE);
  WriteData(DelayAddr,(int)(CommandData.bsc_delay*1000), NIOS_QUEUE);
//  WriteData(TriggerAddr,bsc_trigger, NIOS_QUEUE);

//-----------------

  i_cam_local = i_cam[which];
  //persistently identify cameras by serial number (camID)
  if (camRtn[which][i_cam_local].camID == cam_serial[which])  {
    sc = &camRtn[which][i_cam_local];
    unrecFlag = false;
  }
  else if (!unrecFlag) { //don't keep printing same error
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
    	WriteData(CpuTempAddr[which], sc->cputemperature, NIOS_QUEUE);
    	WriteData(FocPosAddr[which], (int)(sc->focusposition*10), NIOS_QUEUE);
    	WriteData(NumBlobsAddr[which], sc->numblobs, NIOS_QUEUE);

	    WriteData(RaAddr[which], (unsigned int)(SCra[which] * H2I), NIOS_QUEUE);
	    WriteData(DecAddr[which], (unsigned int)(SCdec[which] * DEG2I), NIOS_QUEUE);
	    WriteData(RollAddr[which], (int)(SCroll[which] * DEG2I), NIOS_QUEUE);
	    WriteData(AzAddr[which], (unsigned int)(SCaz[which] * DEG2I), NIOS_QUEUE);
	    WriteData(ElAddr[which], (unsigned int)(SCel[which] * DEG2I), NIOS_QUEUE);
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
//	if ((which == 1) && (sc->numblobs > 8)) {
//		for (int j=0; j<10; j++) {
//			if ((goodPos[j] == 0.0) && (sc->frameNum != posFrame)) {
//				goodPos[j] = trigPos[j]; //overwrite the first 'dead' one it finds
//				posFrame = sc->frameNum;
//				break;
//			}
//		}
//	}
  }	

}

}       //extern "C"

/*
  runs pyramid and solves the field
*/
static void SolveField(StarcamReturn* solrtn, double& ra0, double& dec0, double& r0, int which) {
  double plate_scale = 9.3/180.0/3600.*M_PI;
  double XC = 1530/2;
  double YC = 1020/2;
  double FTOL = 20.*M_PI/180./3600.;//star-blob association angular tolerance (default 20 arcsec as in netisc)
  int k;
  int n_blobs = 0;
  solution_t *sol = new solution_t [MAXSOLUTION];
  int nsol;

  plate_scale = CommandData.StarCam[which].platescale/180./3600.*M_PI; 
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
  	pyr.GetSolution(FTOL, x_p, y_p,n_blobs, &sol, &nsol, &ra0, &dec0, &r0);
  	delete[] x;
  	delete[] y;
  	delete[] x_p;
  	delete[] y_p;
  }
}

static void* SendLoop(void* arg)
{
        nameThread("SCsend");
	while (!InCharge) {
		usleep(20000);
	}
	int sock_thegood, sock_thebad, sock_theugly;
  	string rtn_str = "";
    	string sought = "\n";
    	string::size_type pos_thegood;
    	string::size_type pos_thebad;
    	string::size_type pos_theugly;

	sock_thegood = udp_bind_port(SC_PORT_GOOD, 1);
	if (sock_thegood == -1)
		bprintf(tfatal, "Unable to bind to The Good port");
	sock_thebad = udp_bind_port(SC_PORT_BAD, 1);
	if (sock_thebad == -1)
		bprintf(tfatal, "Unable to bind to The Bad port");
	sock_theugly = udp_bind_port(SC_PORT_UGLY, 1);
	if (sock_theugly == -1)
		bprintf(tfatal, "Unable to bind to The Ugly port");

	while(1) {
		while ((!SCcmd_flag[0]) && (!SCcmd_flag[1]) && (!SCcmd_flag[2]) && (!trigger_flag)) {
			sleep(1);
	        }
		if (SCcmd_flag[0]) {
			string cmd_thegood = string(SCcmd[0]);
    			pos_thegood = cmd_thegood.find(sought, 0);
    			//remove all newlines and add a single one at the end
    			while (pos_thegood != string::npos) {
    	  			cmd_thegood.replace(pos_thegood, sought.size(), "");
    	  			pos_thegood = cmd_thegood.find(sought, pos_thegood - sought.size());
    			}
    			cmd_thegood += "\n";
			if (udp_bcast(sock_thegood, GOOD_PORT, strlen(cmd_thegood.c_str()), cmd_thegood.c_str(), !InCharge))
				bprintf(warning, "Error broadcasting The Good command.\n");
			SCcmd_flag[0] = 0;
		}
		if (SCcmd_flag[1]) {
			string cmd_thebad = string(SCcmd[1]);
    			pos_thebad = cmd_thebad.find(sought, 0);
    			//remove all newlines and add a single one at the end
    			while (pos_thebad != string::npos) {
    	  			cmd_thebad.replace(pos_thebad, sought.size(), "");
    	  			pos_thebad = cmd_thebad.find(sought, pos_thebad - sought.size());
    			}
    			cmd_thebad += "\n";
			if (udp_bcast(sock_thebad, BAD_PORT, strlen(cmd_thebad.c_str()), cmd_thebad.c_str(), !InCharge))
				bprintf(warning, "Error broadcasting The Bad command.\n");
			SCcmd_flag[1] = 0;
		}
		if (SCcmd_flag[2]) {
			string cmd_theugly = string(SCcmd[2]);
    			pos_theugly = cmd_theugly.find(sought, 0);
    			//remove all newlines and add a single one at the end
    			while (pos_theugly != string::npos) {
    	  			cmd_theugly.replace(pos_theugly, sought.size(), "");
    	  			pos_theugly = cmd_theugly.find(sought, pos_theugly - sought.size());
    			}
    			cmd_theugly += "\n";
			if (udp_bcast(sock_theugly, UGLY_PORT, strlen(cmd_theugly.c_str()), cmd_theugly.c_str(), !InCharge))
				bprintf(warning, "Error broadcasting The Ugly command.\n");
			SCcmd_flag[2] = 0;
		}
		if (trigger_flag) {
			string cmd_trigger = string("CtrigExp\n");
			if (!CommandData.StarCam[1].paused) {
				if (udp_bcast(sock_thebad, BAD_PORT, strlen(cmd_trigger.c_str()), cmd_trigger.c_str(), !InCharge))
					bprintf(warning, "Error broadcasting The Bad trigger Command.\n");
			}
			if (!CommandData.StarCam[2].paused) {
				if (udp_bcast(sock_theugly, UGLY_PORT, strlen(cmd_trigger.c_str()), cmd_trigger.c_str(), !InCharge))
					bprintf(warning, "Error broadcasting The Ugly trigger Command.\n");
			}
			trigger_flag = 0;
			
		}
	}
	return NULL;
}

/*
 * wrapper for the read loop in camcommunicator
 * mcp main should make a thread for this
 */
static void* ReadLoop(void* arg)
{
        nameThread("SCread");
	while (!InCharge) {
		usleep(20000);
	}

	int which=0;
	int sock, port;
	char buf[UDP_MAXSIZE];	/* message buffer */
  	string rtnStr = "";
	string line = "";
	string::size_type pos;
	int recvlen;		/* # bytes in acknowledgement message */
	char peer[UDP_MAXHOST];

	/* create a socket */
	sock = udp_bind_port(SC_PORT, 1);

	if (sock == -1)
		bprintf(tfatal, "Unable to bind to port");

        EthernetSC[0]=0;

	while (1) {
		rtnStr = "";
		recvlen = udp_recv(sock, SC_TIMEOUT, peer, &port, UDP_MAXSIZE, buf);
		if (recvlen > 0) {
//			bprintf(info,"received %i bytes from %s",recvlen,peer);
//			bprintf(info,"message is: %s",buf);
		}
		if (strcmp(peer,thegood_servername)==0) {
//		        bprintf(info,"It's from the good");
			which=0;
		} else if (strcmp(peer,thebad_servername)==0) which=1;
		else if (strcmp(peer,theugly_servername)==0) which=2;
    		buf[recvlen] = '\0';
    		line += buf;
		while ((pos = line.find("\n",0)) != string::npos) {
			rtnStr = ParseReturn(line.substr(0,pos),which);
			line = line.substr(pos+1, line.length()-(pos+1)); //set line to text after "\n"
		}
	}
  	return NULL;
}

static string ParseReturn(string rtnStr, int which)
{
	int i_point;
	i_point = GETREADINDEX(point_index);
      	if (rtnStr.substr(0,5) == "<str>") { //response is string
//		bprintf(info,"String");
		string Rstr = rtnStr.substr(5, rtnStr.size() - 11);
		if (Rstr[0] == 'E') //it is an errori
			bprintf(err, "%s", Rstr.substr(6, Rstr.size()-6).c_str());
		else if (Rstr.substr(0,6) == "<conf>") //contains config data
		{
//			bprintf(info,"CONFIG-O");
			Rstr = Rstr.substr(6, Rstr.size()-6);
			istringstream sin;
			sin.str(Rstr);
			double temp;  //value sent for expTime is a double
			sin >> CommandData.StarCam[which].expInt
				>> temp
				>> CommandData.StarCam[which].focusRes
				>> CommandData.StarCam[which].moveTol
				>> CommandData.StarCam[which].maxBlobs
				>> CommandData.StarCam[which].grid
				>> CommandData.StarCam[which].threshold
				>> CommandData.StarCam[which].minBlobDist;
				CommandData.StarCam[which].expTime = (int)(temp * 1000);
		}
	} else { //response is exposure data
//		bprintf(info,"Exposure data");
		CamCommunicator::interpretReturn(rtnStr, &camRtn[which][(i_cam[which]+1)%2]);
		if (CommandData.pyramid) {
      SolveField(&camRtn[which][(i_cam[which]+1)%2],SCra[which],SCdec[which],SCroll[which],which);
      SCra[which]=(SCra[which]*180/M_PI+360.0)/15.0;
		  SCdec[which]*=180/M_PI;
	    SCroll[which]*=180/M_PI;
		  radec2azel(SCra[which],SCdec[which], PointingData[i_point].lst, PointingData[i_point].lat, &SCaz[which], &SCel[which]);
		//if (which==1) SCaz[which] += BAD_AZ_OFF - ACSData.enc_table;
		//if (which==2) SCaz[which] += UGLY_AZ_OFF - ACSData.enc_table;
    }
		i_cam[which] = (i_cam[which]+1)%2;
	}
	return ""; //doesn't send a response back to camera
}	
