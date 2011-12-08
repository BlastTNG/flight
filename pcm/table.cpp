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
short int zerodist;
double goodPos = 90;

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
  static double yawdist;
  double dist;
  static double targVel;
  timeval timestruct;
  double targPos=0.0;
  static double lastTime, lastPos;
  double thisTime, thisPos;
  static int firsttime = 1;
  static int zerodist = 1;
  static int startmove = 1;
  static NiosStruct* dpsAddr = NULL;

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

  //figure out position of goodPos, and move there at 5dps

  //1) yaw distance moved since goodPos was declared
  if (zerodist) {	//zero yawdist every time a trigPos is set (in camcommunicator.cpp)
	yawdist = 0.0;
	zerodist = 0;
  }
  yawdist += ACSData.ifyaw_gy*dt;
  if (yawdist > 180) yawdist -= 360.0;
  if (yawdist < -180) yawdist += 360.0;

  //2) actual encoder position of goodPos now = goodPos(=trigPos) + yawdist
  targPos = goodPos + yawdist;
  dist = thisPos - targPos;
  if (goodPos == 90.0) {
	dist = 90.0 - thisPos;
  }
  if (dist > 180) dist -= 360;
  if (dist < -180) dist += 360;
  //write speed to frame
  int data = (int)((vel/70.0)*32767.0); //allow much room to avoid overflow
  WriteData(dpsAddr, data, NIOS_QUEUE);

  //find new target velocity
  if (CommandData.table.mode==1) {
	targVel = CommandData.table.vel;
	if (thisPos == CommandData.table.pos) targVel = 0;
  } else if (CommandData.table.mode==2) {
	if (startmove) {
		targPos = thisPos + CommandData.table.move;
		startmove=0;
	}
	targVel = CommandData.table.vel;
	if (thisPos == targPos) targVel = 0;
  } else {
  	if (exposing)  {
		targVel = -ACSData.ifyaw_gy;
  	} else {
		if (dist > 0) targVel = 5.0;
		if (dist < 0) targVel = -5.0;
		if ((fabs(dist)) < 0.5) {
			targVel = 0.0;
		}
  	}
  }
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
