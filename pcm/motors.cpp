/* mcp: the Spider master control program
 *
 * motors.cpp: motor control related functions
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

extern "C" {
#include "blast.h"
#include "tx.h"
}
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "drivecommunicator.h"
#include "motorcommand.h"
#include "command_struct.h"

//speed limit in dps
#define MAX_TABLE_SPEED 45.0
//unit conversion from dps to internal controller units
#define DPS_TO_TABLE (1.0/2.496)
//don't update speed unless it changes by this (1000 == 0.08arcsec/sec)
#define TABLE_SPEED_TOL 1000
static int tableSpeed = 0;

//device node for serial port; TODO play with udev to make constant
#define TABLE_DEVICE "/dev/ttyUSB0"
//destination address on IDM controller bus (only one drive)
#define TABLE_ADDR 0x0ff0
static DriveCommunicator* tableComm;

extern "C" {

/* opens communications with motor contorllers */
void openMotors()
{
  tableComm = new DriveCommunicator(TABLE_DEVICE);
  if (tableComm->getError() != DC_NO_ERROR) {
    bprintf(err, "table initialization gave error code: %d",
	tableComm->getError());
    berror(err, "Motors: Error connecting to rotary table");
  }
  tableComm->maxCommSpeed(TABLE_ADDR);
  //turn on motor power
  MotorCommand axison(TABLE_ADDR, 0x0102);
  axison.buildCommand();
  tableComm->sendCommand(&axison);
}

/* closes communications with motor controllers, frees memory */
void closeMotors()
{
  delete tableComm;
}

/* PI loop to figure out rotary table speed
 * result sent to table communication thread with global
 */
void updateTableSpeed()
{
  double vel, azVel, error;
  static double targVel;
  double P, I;
  timeval timestruct;
  static double lastTime, lastPos, integral;
  double thisTime, thisPos;
  static int firsttime = 1;
  static NiosStruct* dpsAddr = NULL;

  //find gondola az rotation rate
  //TODO replace this naive version once there is a pointing solution
  azVel = ACSData.gyro4;

  //control loop: compute target speed
  if (firsttime) {  //initialize history
    lastPos = ACSData.enc_table;
    gettimeofday(&timestruct, NULL);
    lastTime = (double)timestruct.tv_sec + timestruct.tv_usec/1000000.0;
    integral = 0;
    targVel = 0;
    tableSpeed = 0;
    firsttime = 0;
    dpsAddr = GetNiosAddr("dps_table");
    return;
  }

  //find table velocity
  gettimeofday(&timestruct, NULL);
  thisTime = (double)timestruct.tv_sec + timestruct.tv_usec/1000000.0;
  thisPos = ACSData.enc_table;
  if (thisTime == lastTime) {
    berror(err, "System: time not updating");
    return;
  }
  vel = (thisPos - lastPos) / (thisTime - lastTime);
//  bprintf(info, "time elapsed: %g", (thisTime-lastTime));
  lastPos = thisPos;
  lastTime = thisTime;

  //write speed to frame
  int data = (int)((vel/45.0)*32767.0);
  WriteData(dpsAddr, data, NIOS_FLUSH);

  //set new target velocity
  P = CommandData.table_gain.P / 1000.0; 
  I = CommandData.table_gain.I / 1000.0;
  error = azVel - vel;  //TODO actual setpoint should be -azVel
  integral = integral*0.999 + 0.001*error;  //exponential weight
  targVel += P*error + I*integral;
  if (targVel > MAX_TABLE_SPEED) targVel = MAX_TABLE_SPEED;
  else if (targVel < -MAX_TABLE_SPEED) targVel = -MAX_TABLE_SPEED;
  tableSpeed = (int)(targVel/MAX_TABLE_SPEED * (INT_MAX-1));
}

/* function to create a thread in main for table communications
 * controlled by global integer set in updateTableSpeed (main loop)
 * integer is (?) atomic so needs no access control
 */
void rotaryTableComm()
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
      //TODO can also put queries here for (eg) motor temperature

      gettimeofday(&time, NULL);
      thisTime = time.tv_sec + time.tv_usec/1000000.0;
//      bprintf(info, "table update time is: %gs", (thisTime-lastTime));
      lastTime = thisTime;
//    }
  }
}

}       //extern "C"
