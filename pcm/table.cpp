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
#define MAX_TABLE_SPEED 45.0
//unit conversion from dps to internal controller units
#define DPS_TO_TABLE (1.0/2.496)
//don't update speed unless it changes by this (1000 == 0.08arcsec/sec)
#define TABLE_SPEED_TOL 1000
//minimum table move to actually execute (in deg)
#define MIN_TABLE_MOVE 0.1
static int tableSpeed = 0;


#define TABLE_DEVICE "/dev/ttySI8" //TODO change this?
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
  double vel, azVel, dt;
  static double targVel, relVel;
  timeval timestruct;
  static double lastTime, lastPos;
  double thisTime, thisPos;
  static int firsttime = 1;
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

  //find gondola az rotation rate
  //TODO replace this naive version once there is a pointing solution
  azVel = -ACSData.ifyaw_gy;

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

  //write speed to frame
  int data = (int)((vel/70.0)*32767.0); //allow much room to avoid overflow
  WriteData(dpsAddr, data, NIOS_QUEUE);

  //update the remaining relative move, find relVel needed to move it
  CommandData.table.RelMove -= (vel - azVel) * dt;
  while (CommandData.table.RelMove > 360) CommandData.table.RelMove -= 360;
  while (CommandData.table.RelMove < -360) CommandData.table.RelMove += 360;
  if (fabs(CommandData.table.RelMove) < MIN_TABLE_MOVE)
    CommandData.table.RelMove = 0;
  relVel = CommandData.table.RelMove * CommandData.table.MoveGain;
  if (relVel > MAX_TABLE_SPEED) relVel = MAX_TABLE_SPEED;
  else if (relVel < -MAX_TABLE_SPEED) relVel = -MAX_TABLE_SPEED;
  //if going too fast in one direction, reverse direction of move
#if 0      //this causes annoying spins around...fix or leave out
  if (fabs(CommandData.table.RelMove) > 40) { //only do this for large moves
    if (azVel > MAX_TABLE_SPEED/2 && relVel > azVel) { //too fast +
      CommandData.table.RelMove -= 360; //go in -'ve direction
      relVel = 0;   //find new relVel next time
    } else if (azVel < -MAX_TABLE_SPEED/2 && relVel < azVel) { //too fast -
      CommandData.table.RelMove += 360; //go in +'ve direction
      relVel = 0;
    }
  }
#endif

  //find new target velocity
  targVel = azVel + relVel;
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
