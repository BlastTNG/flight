/* mcp: the BLAST master control program
 *
 * table.cpp: rotary table control functions 
 *     uses cpp to make calling class functions easier
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of mcp.
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

extern "C" {
#include "share/blast.h"
#include "tx.h"
#include "share/channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "command_struct.h"
}
#include "drivecommunicator.h"
#include "motorcommand.h"

//speed limit in dps
#define MAX_TABLE_SPEED 30.0
//unit conversion from dps to internal controller units
#define DPS_TO_TABLE (1.0/2.496)
//don't update speed unless it changes by this (1000 == 0.08arcsec/sec)
#define TABLE_SPEED_TOL 1000
//minimum table move to actually execute (in deg)
#define MIN_TABLE_MOVE 0.1
static int tableSpeed = 0;
short int exposing;
short int docalc;
short int zerodist[10];
double goodPos[10] = {95.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

#define TABLE_DEVICE "/dev/ttySI8"
#define TABLE_ADDR 0x0ff0 //destination address on IDM controller bus (only one drive)
static DriveCommunicator* tableComm = NULL;
static pthread_t tablecomm_id;

static void* rotaryTableComm(void *arg);

extern "C" {

/*opens communications with table motor controller*/
void openTable()
{
  bprintf(info, "connecting to the rotary table");
  tableComm = new DriveCommunicator(TABLE_DEVICE);
  if (tableComm->getError() != DC_NO_ERROR) {
	bprintf(err, "Table: rotary table initialization gave error code: %d", tableComm->getError());
  }
  pthread_create(&tablecomm_id, NULL, &rotaryTableComm, NULL);
}

/*closes communications with table, frees memory */
void closeTable()
{
  if (tableComm != NULL) {
	delete tableComm;
  }
}

/* figures out desired rotary table speed
 * result sent to table communication thread with global
 * control loop performed externally
 */
void updateTableSpeed()
{
  double vel, dt;
  static double calcdist=0.0, dist=0.0, movedist=0.0;
  static double targVel;
  timeval timestruct;
  static double targPos;
  static double movePos;
  static double lastTime, lastPos;
  double thisTime, thisPos;
  static int firsttime = 1;
  static double yawdist[10];
  static int startmove = 1;
  static int sendvel = 1;
  static NiosStruct* dpsAddr = NULL;
  int i;
//  static int j;

  //initialization
  if (firsttime) {
    lastPos = ACSData.enc_table;
    gettimeofday(&timestruct, NULL);
    lastTime = (double)timestruct.tv_sec + timestruct.tv_usec/1000000.0;
    targVel = 0;
    tableSpeed = 0;
    firsttime = 0;
    dpsAddr = GetNiosAddr("dps_table");
    return;
  }

  //find table speed
  gettimeofday(&timestruct, NULL);
  thisTime = (double)timestruct.tv_sec + timestruct.tv_usec/1000000.0;
  thisPos = ACSData.enc_table;
  if (thisTime == lastTime) {
    bprintf(err, "System: time not updating");
    return;
  }
  dt = (thisTime - lastTime);
  vel = (thisPos - lastPos) / dt;
  lastPos = thisPos;
  lastTime = thisTime;

  for (i=0; i<10; i++) {
	if (zerodist[i]) {	//zero yawdist every time a trigPos is set (in camcommunicator.cpp)
//		cout << "ZEROING " << i << endl;
		yawdist[i] = 0.0;
		zerodist[i] = 0;
  	}
  	yawdist[i] -= ACSData.ifyaw_gy*dt;
  	if (yawdist[i] > 180) yawdist[i] -= 360.0;
  	if (yawdist[i] < -180) yawdist[i] += 360.0;
  }
  if (docalc) {
//	cout << "doing CALC" << endl;
  	//figure out targPos
  	//1) yaw distance moved since goodPos[i] was at trigPos[i]
	
	  //2) actual encoder position of goodPos now = goodPos(=trigPos) + yawdist
	  for (i=0; i<10; i++) {
		if (goodPos[i] == 90.0) targPos = 90.0;
		else targPos = goodPos[i] + yawdist[i];
//		cout << "GOODPOS #" << i << " encoder pos is " << goodPos[i] << ", with YAWDIST = " << yawdist[i] << ", and TARGPOS = " << targPos << endl;
		if ((targPos > 135) || (targPos < 45)) {//   if it's out-of-bounds, set targPos to 90
//			cout << "GOODPOS #" << i << " is out of bounds" << endl;
			targPos = 90.0;
	  		calcdist = thisPos - targPos;
	  		if (calcdist > 180) calcdist -= 360;
	  		if (calcdist < -180) calcdist += 360;
		} else {
			calcdist = thisPos - targPos; // it's not out of bounds, check how far away it is
	  		if (calcdist > 180) calcdist -= 360;
	  		if (calcdist < -180) calcdist += 360;
			if ((fabs(calcdist)) > 5.0) { // if it's too far, set targPos to 90
//				cout << "GOODPOS #" << i << " is too far away(" << calcdist << "), setting TARGPOS to 90" << endl;
				targPos = 90.0;
	  			calcdist = thisPos - targPos;
	  			if (calcdist > 180) calcdist -= 360;
	  			if (calcdist < -180) calcdist += 360;
			} else {	
//				cout << "I can make it to GOODPOS #" << i << " at CALCDIST= " << calcdist << endl;
				if (targPos != 90) break; // if it survives the test, use it, otherwise try next one
			}
		}
	  }
  docalc = 0;
  sendvel = 1;
  // after all this, calcdist should = here - 90, or here - smthg else
  // calcdist is calculated once, and sets the targVel
  // dist is calculated every time to tell you how close you are to targPos
  }
  dist = thisPos - targPos;
  if (dist > 180) dist -= 360;
  if (dist < -180) dist += 360;

  //write speed to frame
  int data = (int)((vel/70.0)*32767.0); //allow much room to avoid overflow
  WriteData(dpsAddr, data, NIOS_QUEUE);

  //find new target velocity
  if (CommandData.table.mode==1) {
	// MOVE
	startmove = 1;
	targPos = CommandData.table.pos;
	targVel = 6.0;//CommandData.table.vel;
	movedist = thisPos - targPos;
  	if (movedist > 180) movedist -= 360;
  	if (movedist < -180) movedist += 360;
        if (movedist > 0) targVel = -6.0;
	if ((fabs(movedist) < 0.5)) targVel = 0;
  } else if (CommandData.table.mode==2) {
	// RELMOVE
	if (startmove) {
		movePos = thisPos + CommandData.table.move;
		startmove=0;
	}
	targVel = 6.0;//CommandData.table.vel;
	movedist = thisPos - movePos;
  	if (movedist > 180) movedist -= 360;
  	if (movedist < -180) movedist += 360;
        if (movedist > 0) targVel = -6.0;
	if ((fabs(movedist) < 0.5)) targVel = 0;
  } else {
	//TRACKING
	startmove = 1;
  	if (exposing)  {
		targVel = -ACSData.ifyaw_gy;
  	} else {
		// having figured out calcdist, set a targVel that will get you there in 1s, and don't update targVel until you get within 0.5
		if ((calcdist < 0) && (sendvel == 1)) {
			//TODO change this to the a ? b | c
			if ((fabs(calcdist)) > 10.0) { // FIXME failsafe in case dist still ends up being >10
				targVel = 10.0;   // this shouldn't be here if the calc code worked
				sendvel = 0;
			} else {
				targVel = -calcdist; // go there at a speed proportional to its distance, up to 10dps
//				cout << "TARGVEL set to " << targVel << endl;
				sendvel = 0;
			}
		}
		if ((calcdist > 0) && (sendvel == 1)) {
			if ((fabs(calcdist)) > 10.0) {
				targVel = -10.0;
				sendvel = 0;
			} else { 
				targVel = -calcdist;
				sendvel = 0;
			}
		}
		if ((fabs(dist)) < 0.3) {
			targVel = -ACSData.ifyaw_gy;//0.0;
		}
  	}

  }
  /*if((j%50)==0)*/ //cout << "TARGVEL= " << targVel << endl;
//  j++;
  if (targVel > MAX_TABLE_SPEED) targVel = MAX_TABLE_SPEED;
  else if (targVel < -MAX_TABLE_SPEED) targVel = -MAX_TABLE_SPEED;
  tableSpeed = (int)(targVel/MAX_TABLE_SPEED * (INT_MAX-1));
}

} //extern "C"

/* function to create a thread in main for table communications
 * controlled by global integer set in updateTableSpeed (main loop)
 * integer is (?) atomic so needs no access control
 */
void* rotaryTableComm(void* arg)
{
  static int currSpeed=0;
  double dTableSpeed;  //speed (global int) converted to double
  timeval time;
  static double lastTime=0;
  double thisTime;
  //perform initialization
  while (!tableComm->isOpen()) {  //needed when original open fails
    sleep(1);
    tableComm->openConnection(tableComm->getDeviceName());
  }
    
  //turn on motor power
  MotorCommand axison(TABLE_ADDR, 0x0102);
  axison.buildCommand();
  tableComm->sendCommand(&axison);

//  bputs(startup, "Motors: rotary table startup");
  while (1) {
//    usleep(1000);  //needed with if statement
//    if (abs(currSpeed-tableSpeed) > TABLE_SPEED_TOL) {
      currSpeed = tableSpeed;
      dTableSpeed = (double) tableSpeed * 
        MAX_TABLE_SPEED * DPS_TO_TABLE / (INT_MAX - 1);
      tableComm->sendSpeedCommand(TABLE_ADDR,dTableSpeed);
      if (tableComm->getError() != DC_NO_ERROR) {
        bprintf(err, "Motors: rotary table comm failure, trying re-synch");
        while (tableComm->getError() != DC_NO_ERROR) {
          //command failed, keep trying to resynchronize communications
          usleep(10000);
          tableComm->synchronize();
        }
        //may also need to resend volatile memory commands (if I use any)
        bprintf(info, "Motors: successful reconnection to rotary table");
      }

      //can also put queries here for (eg) motor temperature
      gettimeofday(&time, NULL);
      thisTime = time.tv_sec + time.tv_usec/1000000.0;
//      bprintf(info, "table update time is: %gs", (thisTime-lastTime));
      lastTime = thisTime;
//    }
  }
  return NULL;
}
