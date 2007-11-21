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
#include "pivotcommand.h"
#include "reactcommand.h"
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

//Set limits on gondola speed and motor currents and speed.
// Maximum reaction wheel speed
#define MAX_RWHEEL_SPEED (200.0/60.0*360.0)
// Maximum reaction wheel current
#define MAX_RWHEEL_CURRENT 17.5
// Maximum Gondola Speed
#define MAX_GOND_SPEED 45.0
// Maximum Pivot Speed 
#define MAX_PIVOT_SPEED 45.0

//#define NO_MOTORS

//device node for serial port; TODO play with udev to make constant
#define TABLE_DEVICE "/dev/ttyUSB0"
//destination address on IDM controller bus (only one drive)
#define TABLE_ADDR 0x0ff0
static DriveCommunicator* tableComm = NULL;
static pthread_t tablecomm_id;
static pthread_t pivotcomm_id;
static pthread_t reactcomm_id;

// device node address for the pivot motor controller
#define PIVOT_DEVICE "/dev/ttySI0"
// device node address for the pivot motor controller
#define REACT_DEVICE "/dev/ttySI2"

int write_index=0; //lmf: Following what is done in pointing.c
                   // CHECK TO MAKE SURE this is valid!
extern "C" {

static void* rotaryTableComm(void *arg);
static void* pivotComm(void *arg);
static void* reactComm(void *arg);

double gTargetVel;  // Gondola Target velocity
double ireq,pvreq;  // Reaction wheel current requested, and Pivot
                    // velocity requested.
unsigned long int vpiv;

/* opens communications with motor controllers */
void openMotors()
{
  bprintf(info, "Motors: connecting to motors");
  tableComm = new DriveCommunicator(TABLE_DEVICE);
  if (tableComm->getError() != DC_NO_ERROR) {
    bprintf(err, "Motors: rotary table initialization gave error code: %d",
	tableComm->getError());
  }
//  tableComm->maxCommSpeed(TABLE_ADDR);  //done in firmware
  //turn on motor power
  MotorCommand axison(TABLE_ADDR, 0x0102);
  axison.buildCommand();
  tableComm->sendCommand(&axison);
  pthread_create(&tablecomm_id, NULL, &rotaryTableComm, NULL);

  open_pivot(PIVOT_DEVICE);
  pthread_create(&pivotcomm_id, NULL, &pivotComm, NULL);
  open_react(REACT_DEVICE);
  pthread_create(&reactcomm_id, NULL, &reactComm, NULL);
}

/* closes communications with motor controllers, frees memory */
void closeMotors()
{
  if (tableComm != NULL) {
    tableComm->closeConnection();
    delete tableComm;  //causes a glibc "free(): invalid pointer" warning
  }
  if (pivotComm != NULL) {
    close_pivot();
  }
  if (reactComm != NULL) {
    close_react();
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
  azVel = ACSData.gyro2;

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
#if 0      //this causes annoying spins around...fix or leave out
  if (fabs(CommandData.tableRelMove) > 40) { //only do this for large moves
    if (azVel > MAX_TABLE_SPEED/2 && relVel > azVel) { //too fast +
      CommandData.tableRelMove -= 360; //go in -'ve direction
      relVel = 0;   //find new relVel next time
    } else if (azVel < -MAX_TABLE_SPEED/2 && relVel < azVel) { //too fast -
      CommandData.tableRelMove += 360; //go in +'ve direction
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
 * Looks for updates in the requested velocity, and updates 
 * the known encoder velocity.
 *
 */
void* pivotComm(void* arg)
{
  //  int laststat=0;
  //  double dps=0.0;
  //  int data=0;
  pivotinfo.loop=0;
  int firsttime = 1;
  double vlast=0;
  double vcur=0;
  // wait until the pivot controller has been initialized.
  while(pivotinfo.open==0)
    {
      sleep(1);
      if (firsttime) 
	{
	  bputs(err,"pivotComm: Pivot controller port is not open. Retrying\n");
	  //	  firsttime = 0;
	}
      open_pivot(PIVOT_DEVICE);
    }
  // Configure the serial port.
  configure_pivot();

  bprintf(info,"pivotComm: Here are the conversion factors for RW.");
  bprintf(info,"RWVEL_DPS=%d, RWVEL_OFFSET=%d, RWCUR_DPS=%d, RWCUR_");
  while(1){
    // Am I ready to send a command?
    // if
#ifndef NO_MOTORS // put motor commands below....
    if(firsttime)
      { 
	bputs(info,"pivotComm: NO_MOTORS not defined--Commands will be sent to the pivot.");        
        vpiv=getquery(QUER_VEL);
        
      }

   if(pivotinfo.loop==0) // There is no loop
      {
        if(firsttime)
	  {
            bputs(info,"pivotComm: Starting loop...");
	  }
	//start_loop(pvreq);        
      }
   /*    if(pivotinfo.loop==1)
      {
        change_piv_vel(pvreq);
      }
    */ 
#endif //NO_MOTORS

#ifdef NO_MOTORS // !!!!DO NOT PUT MOTOR COMMANDS BELOW!!!!
    if(firsttime)
      {
        bputs(info,"pivotComm: NO_MOTORS defined--NO Commands will be sent to the pivot.");
      }
#endif //NO_MOTORS

//     bprintf(info,"4: %i, %f, %i",vpiv, dps,motorpos);
     if(pivotinfo.loop> 0)
       {
	 vpiv=getquery(QUER_VEL);
       }
     else
       {
         vpiv=0; // Otherwise this query returns the max slew speed.
       }

        firsttime = 0;
  } 
  

  return NULL;  
}

void* reactComm(void* arg)
{
  // Make sure the connection to the reaction wheel controller has been initialized.
  int firsttime = 1;
  while(reactinfo.open==0)
    {
      sleep(1);
      if (firsttime)
	{
	  bputs(err,"reactComm: Reaction wheel port is not open. Retrying.\n");
	  firsttime = 0;
	}
      open_react(REACT_DEVICE);
    }
  // Configure the serial port.
  //  configure_react();

  return NULL;
}

/*
 * Reads from the command structure what mode we are supposed to be in
 * and figures out from the sensors what velocity we should be going.
 *
 */
void getTargetVel()
{
  // gTargetVel is a global variable
  double theta,amp,acrit,per,vswitch,dvdt,tswitch,t1;
  static double vlast,vdir;
  int data;
  static int firsttime = 1;
  static int accelmode,wait;
  static NiosStruct* dpsGondReq   = NULL;
  static NiosStruct* isGondAccel  = NULL;
  static NiosStruct* gondTheta    = NULL;
  static NiosStruct* dpspsGondReq = NULL;

  
  if(firsttime==1)
    {
      bprintf(info,"Motors: Running getTargetVel for the first time.");
      dpsGondReq  =GetNiosAddr("dps_gond_req");
      isGondAccel =GetNiosAddr("is_gond_accel");
      gondTheta   =GetNiosAddr("gond_theta");
      dpspsGondReq=GetNiosAddr("dpsps_gond_req");
    }

  switch(CommandData.spiderMode){
  case point:
if(firsttime==1)    bprintf(info,"Motors: We are Pointing.");
    gTargetVel=0.0;
    // lmf: This just a place holder.  Obviously some day we may
    // want to point at an actual location.
    break;
  case spin:
if(firsttime==1)    bprintf(info,"Motors: We are spinning.");
    gTargetVel=CommandData.spiderSpin.dps;
    break;
  case scan:
if(firsttime==1)    bprintf(info,"Motors: We are scanning.");
    // Okay where are we?
    if(firsttime==1)
      {
        accelmode=0;
        vdir=1.0;
        vlast = ACSData.gyro2;
        wait=0;
      }
    amp=CommandData.spiderScan.W;
    acrit=CommandData.spiderScan.Wcrit;
    per=CommandData.spiderScan.P;
    theta=PointingData[GETREADINDEX(point_index)].az-CommandData.spiderScan.C;
    data=(int) ((theta/360.0)*65535.0);
    WriteData(gondTheta, data, NIOS_QUEUE);

    // if we weren't in accel mode are we sure that we shouldn't be in 
    // constant acceleration mode?
    if(accelmode==0)
      {
        if(amp-(fabs(theta)) <= acrit)
	    {
            accelmode=1;
	    }
      }
    WriteData(isGondAccel, accelmode, NIOS_QUEUE);

    if(accelmode==0)
      {
        // What should our abs velocity be?
        gTargetVel=amp*(2*M_PI/per)*cos(asin(theta/amp))*vdir;
        return;
      }
      else
      {
        // If not we are in constant acceleration mode
        t1=asin((amp-acrit)/amp)/(2*M_PI/per);
          
        vswitch=amp*(2*M_PI/per)*cos(t1);
          // This is the velocity amplitude at which we go back to our
          // sinusoidal scan.
        
        tswitch=M_PI/(2*M_PI/per)-2*t1;
          // This is how long it would take for our sinusoidal scan
          // to get back to the critical point.
        
        dvdt=2.0*vswitch/tswitch*(-1.0)*vdir; 
          // This is the constant acceleration we want.
        data=(int) ((dvdt/2.0)*32767.0);
        WriteData(dpspsGondReq, data, NIOS_QUEUE);

        // Get the current time and velocities
      
        // new velocity is previous velocity - dvdt*dt
        if(!wait) // unless we have been told not to change
	  {
	      gTargetVel=vlast-dvdt*0.01; // This assumes that we are calling
          }                         // this function at 100 Hz.

        // Check:  Am I going faster than the switch velocity?
        if(gTargetVel*vdir*(-1.0) > vswitch )
	    {
            // Am I past the acrit value?
            if(amp-fabs(theta)>acrit)
	        {
		  accelmode=0;
		  vdir=vdir*-1.0;
                wait=0;
	        }
            else
	        {
                wait=1;
	      }
	    }
      }        
    vlast=gTargetVel;

  //write target speed to frame
      break;
    default:
    berror(err, "getTargetVel: Invalid mode");
    gTargetVel=0.0;
    return;
    break;
  }
  data = (int)((gTargetVel/60.0)*32767.0); //allow much room to avoid overflow
    WriteData(dpsGondReq, data, NIOS_QUEUE);

  if(firsttime==1)
    { 
      bprintf(info,"getTargetVel: gTargetVel is %f",gTargetVel); 
      firsttime=0;
    }
  
}
// Looks at the target Velocity gTargetVel and sends commands to the motors.

void updateMotorSpeeds()
{
  double dps;
  int data=0;
  double vcurr = ACSData.gyro2; // Gyro is in dps
  double verr=vcurr-gTargetVel; // Velocity error term.
  double vreac= RWData.vel;
  static int isfirst =1;
  static NiosStruct* dpsPivReq = NULL;
  static NiosStruct* iReacReq  = NULL;
  static NiosStruct* dpsPiv   = NULL;

  if(isfirst==1)
    {
      dpsPiv  =GetNiosAddr("dps_piv");
      dpsPivReq = GetNiosAddr("dps_piv_req");
      iReacReq  = GetNiosAddr("i_reac_req");
      vpiv=0;
    }
    // Update the pivot velocity, which is stored in controller units in vpiv
    // (set in pivotComm)
  dps=((double) vpiv)/COUNTS_PER_DEGREE;
  data=(int) ((dps/70.0)*32767.0);
  WriteData(dpsPiv, data, NIOS_QUEUE);

// What mode are we in?
switch(CommandData.spiderMode){
case point:
  // lmf: For now use spin gains.
  // Later we will need to implement pointing gains.
  ireq=CommandData.spiderGain.sp_r * verr;
  pvreq=CommandData.spiderGain.sp_p * vreac+ ACSData.gyro2;
  break;    
case spin:
  ireq=CommandData.spiderGain.sp_r * verr;
  pvreq=CommandData.spiderGain.sp_p * vreac+ ACSData.gyro2;
  break;
  case scan:
    ireq=CommandData.spiderGain.sc_r * verr;
    pvreq=CommandData.spiderGain.sc_p1 * vreac+ ACSData.gyro2
                   + CommandData.spiderGain.sc_p1 * verr;
    break;
  default:
    berror(err, "updateMotorSpeeds: Invalid mode");
    ireq=0.0;
    pvreq=0.0;
    break;
  }  
  // Now check to make sure that we are not beyond any predefined
  // limits.

  // Are we going faster than the maximum reaction wheel 
  // speed and did we request current drive us faster in the 
  // direction?
  if (fabs(vreac) > MAX_RWHEEL_SPEED && vreac*ireq > 0)
    {
      bprintf(warning,"updateMotorSpeeds:Current reaction wheel speed %f is beyond the speed %f.\n Reaction wheel current set to 0.0.\n",vreac,MAX_RWHEEL_SPEED);
      ireq=0.0;
    }
  // Is the requested reaction wheel current beyond the current limits set?
  if (fabs(ireq) > MAX_RWHEEL_CURRENT)
    {
      ireq=MAX_RWHEEL_CURRENT*ireq/fabs(ireq);
      bprintf(warning,"updateMotorSpeeds: Requested ireq is above the current limits, setting %f\n",ireq);
    }
  // Is the pivot going too fast?
  // lmf: write this one once I figure out how to store the pivot velocity.

  // Is the gondola going too fast?  
  if (fabs(ACSData.gyro2)>MAX_GOND_SPEED)
    {
      
      // lmf: Once we have a better idea of the gains we'll put something
      // more intelligent here.
      ireq=MAX_RWHEEL_CURRENT*ireq/fabs(ireq);
//      bprintf(warning,"updateMotorSpeeds:Gondola is rotating too fast!\n Setting ireq and pvreq to 0.0.");

    }
  data=(int)((pvreq/60.0)*32767.0);
  WriteData(dpsPivReq, data, NIOS_QUEUE);
  data=(int)((ireq/20.0)*32767.0);
  WriteData(iReacReq, data, NIOS_QUEUE);
  
  // Make the command strings and send them.
  
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
