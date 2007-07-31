/* mcp: the Spider master control program
 *
 * motors.cpp: motor control related functions
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

extern "C" {
#include "blast.h"
#include "tx.h"
#include "command_struct.h"
}
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "drivecommunicator.h"
#include "motorcommand.h"

//speed limit in dps
#define MAX_TABLE_SPEED 45.0
//unit conversion from dps to internal controller units
#define DPS_TO_TABLE (1.0/2.496)
//don't update speed unless it changes by this (1000 == 0.08arcsec/sec)
#define TABLE_SPEED_TOL 1000
//minimum table move to actaully execute (in deg)
#define MIN_TABLE_MOVE 0.1
static int tableSpeed = 0;

//device node for serial port; TODO play with udev to make constant
#define TABLE_DEVICE "/dev/ttyUSB0"
//destination address on IDM controller bus (only one drive)
#define TABLE_ADDR 0x0ff0
static DriveCommunicator* tableComm = NULL;
static pthread_t tablecomm_id;

extern "C" {

static void* rotaryTableComm(void *arg);

/* opens communications with motor contorllers */
void openMotors()
{
  bprintf(info, "Motors: connecting to motors");
  tableComm = new DriveCommunicator(TABLE_DEVICE);
  if (tableComm->getError() != DC_NO_ERROR) {
    bprintf(err, "Motors: roary table initialization gave error code: %d",
	tableComm->getError());
  }
//  tableComm->maxCommSpeed(TABLE_ADDR);  //done in firmware
  //turn on motor power
  MotorCommand axison(TABLE_ADDR, 0x0102);
  axison.buildCommand();
  tableComm->sendCommand(&axison);
  pthread_create(&tablecomm_id, NULL, &rotaryTableComm, NULL);
}

/* closes communications with motor controllers, frees memory */
void closeMotors()
{
  if (tableComm != NULL) {
    tableComm->closeConnection();
    delete tableComm;  //causes a glibc "free(): invalid pointer" warning
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
  azVel = ACSData.gyro4;

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
  CommandData.tableRelMove -= (vel - azVel) * dt;
  while (CommandData.tableRelMove > 360) CommandData.tableRelMove -= 360;
  while (CommandData.tableRelMove < -360) CommandData.tableRelMove += 360;
  if (fabs(CommandData.tableRelMove) < MIN_TABLE_MOVE)
    CommandData.tableRelMove = 0;
  relVel = CommandData.tableRelMove * CommandData.tableMoveGain;
  if (relVel > MAX_TABLE_SPEED) relVel = MAX_TABLE_SPEED;
  else if (relVel < -MAX_TABLE_SPEED) relVel = -MAX_TABLE_SPEED;
  //if going too fast in one direction, reverse direction of move
  if (CommandData.tableRelMove > 5) { //only do this for large moves
    if (azVel > MAX_TABLE_SPEED/2 && relVel > azVel) { //too fast +
      CommandData.tableRelMove -= 360; //go in -'ve direction
      relVel = 0;   //find new relVel next time
    } else if (azVel < -MAX_TABLE_SPEED/2 && relVel < azVel) { //too fast -
      CommandData.tableRelMove += 360; //go in +'ve direction
      relVel = 0;
    }
  }

  //find new target velocity
  targVel = azVel + relVel;
  if (targVel > MAX_TABLE_SPEED) targVel = MAX_TABLE_SPEED;
  else if (targVel < -MAX_TABLE_SPEED) targVel = -MAX_TABLE_SPEED;
  tableSpeed = (int)(targVel/MAX_TABLE_SPEED * (INT_MAX-1));
}

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

  bputs(startup, "Motors: rotary table startup");
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

/*
 * updates slow motor fields in bbc frame
 */
void slowMotorFields()
{
  static bool firsttime = true;
  static NiosStruct* gPTableAddr = NULL;
  static NiosStruct* gITableAddr = NULL;
  static NiosStruct* gDTableAddr = NULL;
  static NiosStruct* tableMoveAddr = NULL;
  static NiosStruct* gTableMoveAddr = NULL;

  //initialization
  if (firsttime) {
    gPTableAddr = GetNiosAddr("g_p_table");
    gITableAddr = GetNiosAddr("g_i_table");
    gDTableAddr = GetNiosAddr("g_d_table");
    tableMoveAddr = GetNiosAddr("table_move");
    gTableMoveAddr = GetNiosAddr("g_table_move");
    firsttime = false;
  }

  WriteData(gPTableAddr, CommandData.tableGain.P, NIOS_QUEUE);
  WriteData(gITableAddr, CommandData.tableGain.I, NIOS_QUEUE);
  WriteData(gDTableAddr, CommandData.tableGain.D, NIOS_QUEUE);
  WriteData(tableMoveAddr, (unsigned int)
      (CommandData.tableRelMove*10.0), NIOS_QUEUE);
  WriteData(gTableMoveAddr, (unsigned int)
      ((CommandData.tableMoveGain/100.0)*SHRT_MAX), NIOS_QUEUE);

}

}       //extern "C"
