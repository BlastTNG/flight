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
#include "motordefs.h"
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
// 200 RPM=1200 dps
// 150 RPM=900 dps
#define MAX_RWHEEL_SPEED (150.0/60.0*360.0) 

#define MAX_GOND_SPEED 45.0
// Maximum Pivot Speed (dps)
#define MAX_PIVOT_SPEED 45.0


#define NO_MOTORS // Does not send commands to the pivot motor
//#define NO_RW_MOTOR

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
  int vpiv;
  int encpos;
  int encposShort;
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
  ireq=1.0;
  pthread_create(&reactcomm_id, NULL, &reactComm, NULL);
}

/* closes communications with motor controllers, frees memory */
void closeMotors()
{
  if (pivotinfo.fd>0) {
    close_pivot();
  }
  if (reactinfo.fd>0) {
    close_react();
  }
  if (tableComm != NULL) {
    delete tableComm;  //causes a glibc "free(): invalid pointer" error
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
      //dTableSpeed = -0.05*DPS_TO_TABLE;   //TODO erase me!
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
 * Pthread that handles communication with the pivot controller.
 * 
 *
 */
void* pivotComm(void* arg)
{
  sleep(5); // Give mcp enough time to setup the BLAST Card interface.
  //  int laststat=0;
  //  double dps=0.0;
  //  int data=0;
  struct NiosStruct* encPos = NULL; 
  struct NiosStruct* encVel = NULL; 
  pivotinfo.loop=0;
  pivotinfo.closing=0;
  int firsttime = 1;
  int first = 1;
  int encposOld;
  double prevTime, curTime;
  double encvel=0.0;
  //  unsigned long int vmagpiv;
  struct timeval timer;  

// wait until the pivot controller has been initialized.
  while(pivotinfo.open==0)
    {
      sleep(1);
      if (first) 
	{
	  bputs(err,"pivotComm: Pivot controller port is not open. Retrying\n");
     	  first = 0;
	}
      open_pivot(PIVOT_DEVICE);
    }
  // Configure the serial port.
  configure_pivot();

  encPos = GetNiosAddr("enc_pos");
  encVel = GetNiosAddr("enc_vel");

  //  bprintf(info,"pivotComm: Here are the conversion factors for RW.");
  //  bprintf(info,"RWVEL_DPS=%d, RWVEL_OFFSET=%d, RWCUR_DPS=%d, RWCUR_");
  while(1){
  if(pivotinfo.closing==0)
    {
    // Am I ready to send a command?
    // if
#ifndef NO_MOTORS // put motor commands below....
    if(firsttime)
      { 
	bputs(info,"pivotComm: NO_MOTORS not defined--Commands will be sent to the pivot.");        
	//        vmagpiv=getquery(QUER_VEL);
        encpos=getquery(QUER_POS);
        encposShort=encpos%COUNTS_PER_ETURN;
        encposOld = encpos;
        gettimeofday(&timer, NULL);
        prevTime=(double)timer.tv_sec + timer.tv_usec/1000000.0;
	//        vpiv=((int)vmagpiv)*pivotinfo.ldir;
      }

   if(pivotinfo.loop==0) // There is no loop
      {
	bputs(info,"pivotComm: Starting loop...");
	start_loop(pvreq);        
      }
   if(pivotinfo.loop==1)
      {
#ifdef DEBUG_PIV
	bprintf(info,"pivotComm:  Changing velocity");
#endif
	change_piv_vel(pvreq);
      }

   // TODO: Add checks to my pivotcommand functions and try to send a 
   //       command to the pivot to reconfigure the port if there was
   //       a serial port configuration error during the loop.
     
#endif //NO_MOTORS

#ifdef NO_MOTORS // !!!!DO NOT PUT MOTOR COMMANDS BELOW!!!!
    if(firsttime)
      {
        bputs(info,"pivotComm: NO_MOTORS defined--NO Commands will be sent to the pivot.");
      }
#endif //NO_MOTORS

    encpos=getquery(QUER_POS);
    encposShort=encpos%COUNTS_PER_ETURN;
    gettimeofday(&timer, NULL);
    curTime=(double)timer.tv_sec + timer.tv_usec/1000000.0;
    encvel=((int)(((double)(encpos-encposOld))/((curTime-prevTime)*COUNTS_PER_ETURN)*360.0));
    encposOld=encpos;
    prevTime=curTime;
#if 0
//     bprintf(info,"4: %i, %f, %i",vpiv, dps,motorpos);
     if(pivotinfo.loop> 0)
       {
	 //	 vmagpiv=getquery(QUER_VEL);
	 //         vpiv=((int)vmagpiv)*pivotinfo.ldir;
       }
     else
       {
	 //         vpiv=0; // Otherwise this query returns the max slew speed.
       }
#endif
        firsttime = 0;
      WriteData(encPos, ((int)(((double)(encposShort))/COUNTS_PER_ETURN*65535.0)), NIOS_QUEUE);
      WriteData(encVel, ((int)(encvel/360.0*32767.0)), NIOS_QUEUE);
      }//pivotcomm.closing==0
  } //while(1)
  

  return NULL;  
}

void* reactComm(void* arg)
{
  sleep(5);
  int n;
  // Make sure the connection to the reaction wheel controller has been initialized.
  int firsttime = 1;
  int firsterr=1;
  double curr=0;
  // Initialize values in the reactinfo structure.
  reactinfo.open=0;
  reactinfo.loop=0;
  reactinfo.init=0;
  reactinfo.err=0;
  reactinfo.closing=0;
  reactinfo.disabled=10;
  reactinfo.writeset=0;
  while(reactinfo.open==0)
    {
      sleep(1);
      if (firsterr)
	{
	  bputs(err,"reactComm: Reaction wheel port is not open. Attempting to open a conection.\n");
	  firsterr = 0;
	}
      open_react(REACT_DEVICE);
    }
  // Configure the serial port.
    configure_react();
#ifdef DISABLE_RW
    bprintf(info,"reactComm: DISABLE_RW defined, RW controller will be kept disabled.");
#endif
#ifndef DISABLE_RW
    bprintf(info,"reactComm: DISABLE_RW defined.  Commands will be sent to the motor.");
#endif
  while(1)
    {
      curr=ireq;
      if(firsttime) bprintf(info,"reactComm: Requested current is %f",curr);
 
#ifdef DEBUG_RW
      bprintf(info,"reactComm: Requested current is %f",curr);
#endif
  if(reactinfo.closing==0)
    {
      if(reactinfo.loop==0 || (reactinfo.loop==-1 && reactinfo.err==0))
	{
#ifndef NO_RW_MOTORS
          startRWLoop();
#endif
	}
      if(reactinfo.loop==1)
	{
#ifndef NO_RW_MOTORS
	  //          bprintf(info,"reactComm: Requesting the current to be set to %f",curr);
	  n=setRWCurrent(curr);
#endif
	}
    }//reactinfo.closing==0
  else
    {
#ifndef NO_RW_MOTORS
      bprintf(info, "reactComm: Setting RW Current to zero for shutdown.");
      n=setRWCurrent(0.0); 
      break;
#endif
    }//reactinfo.closing==1

      if(firsttime) {firsttime=0;}
    }
#ifdef DEBUG_RW
  bprintf(info,"reactComm: exited loop");
#endif // DEBUG_RW
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
  double xc,amp,per;
  double vg,az,vt2,wind,phi;
  double theta=0.0;
  double vmax=0.0;
  double vr=0.0;
  double a=0.0;
  double vt1=0.0;
  double x1=0.0;
  double x2=0.0;
  int accelmode=0; // A flag to identify when we are in 
                   // constant accel mode.
  static double vlast;
  int data;
  static int firsttime = 1;
  static NiosStruct* dpsGondReq   = NULL;
  static NiosStruct* isGondAccel  = NULL;
  static NiosStruct* gondTheta    = NULL;
  static NiosStruct* dpspsGondReq = NULL;
  static NiosStruct* scanDVelMax = NULL;
  static NiosStruct* scanVelMax = NULL;
  static NiosStruct* scanX1 = NULL;
  static NiosStruct* scanX2 = NULL;
  static NiosStruct* scanAzVt1 = NULL;
  
  if(firsttime==1)
    {
      bprintf(info,"Motors: Running getTargetVel for the first time.");
      dpsGondReq  =GetNiosAddr("dps_gond_req");
      isGondAccel =GetNiosAddr("is_gond_accel");
      gondTheta   =GetNiosAddr("gond_theta");
      dpspsGondReq=GetNiosAddr("dpsps_gond_req");
      scanVelMax=GetNiosAddr("scan_vel_max");
      scanDVelMax=GetNiosAddr("scan_d_vel_max");
      scanAzVt1=GetNiosAddr("scan_az_vt1");
      scanX1=GetNiosAddr("scan_x1");
      scanX2=GetNiosAddr("scan_x2");
    }
  switch(CommandData.spiderMode){
  case point:
if(firsttime==1)    bprintf(info,"Motors: We are Pointing.");
    gTargetVel=0.0;
    // lmf: This just a place holder.  Obviously some day we may
    // wish to point at an actual location.
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
	//        vdir=1.0;
        vlast = ACSData.gyro2;
        
      }

    // Calculate the scan parameters. 
    amp=CommandData.spiderScan.W;
    per=CommandData.spiderScan.P;
    xc=CommandData.spiderScan.C;
    vt2=CommandData.spiderScan.vt2;
    wind=CommandData.spiderScan.wind;
    phi=CommandData.spiderScan.phi; // Phase angle width of the 
                                    // constant accel portion of 
                                    // the scan 

    // Check that our scan amplitudes are all reasonable
    // TODO: If one of these parameters is wrong, go into 
    // a safety mode, like point anti-sun.
    if(per<0.0)
      {
        berror(err,"getTargetVel: Period =%f.",per);
	return;
      }
    if(amp<0.0)
      {
        berror(err,"getTargetVel: Scan amplitude is  =%f.",per);
	return;
      }
    if(phi<0.0 || phi>= 90.0)
      {
	berror(warning,"getTargetVel: Constant accel phase angle width is out of range");
        bputs(warning,"Setting phi=15.0 degrees as default.");
        phi=15.0;
      }
    vmax=amp*2.0*M_PI/per;  
    vt1=vmax*sin(phi*M_PI/180);
    x1=xc-amp*cos(phi*M_PI/180);
    x2=xc+amp*cos(phi*M_PI/180);

    // a is the maximum change in velocity within a single timestep (0.01 sec)
    a=amp*(2.0*M_PI/per)*(2.0*M_PI/per)*cos(phi/180.0*M_PI)*0.01;

    az = PointingData[GETREADINDEX(point_index)].az;
    vg = ACSData.gyro2; // Gondola velocity
    // theta is the angle from the center of the scan
    theta=az-xc;

    if(az<(x1-wind))
      {
	vr=vt2-(vt2-vt1)/(x1-wind)*az;
      }
    else if(az>=(x2+wind))
      {
        vr=(vt1-vt2)/(360-x2-wind)*(az-x2-wind)-vt1;
      }
    else if(az<x1)
      {
	vr = vt1;
      }
    else if(az>x2)
      {
	vr = -vt1;
      } 
    else if(vg>0.0)
      {
	vr = vmax*sqrt(1.0-((az-xc)*(az-xc)/(amp*amp)));
      }
    else
      {
	vr = (-1.0)*vmax*sqrt(1.0-((az-xc)*(az-xc)/(amp*amp)));
      }

    // If the accel is too high adjust the speed appropriately.
    // Maybe this should be vgond?... or not, 100 Hz probably 
    // isn't enough time for the system to respond...
   if((vr-vlast)>a) 
      {
        vr=vlast+a;
	accelmode=1;
      } 
   if((vlast-vr)>a) 
      {
	vr=vlast-a;
        accelmode=1;
      }   
    gTargetVel=vr;
    vlast=vr;

    
#if 0 // lmf: Old scan mode... that never worked properly ...
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
#endif 
      break;
    default:
    berror(err, "getTargetVel: Invalid mode");
    gTargetVel=0.0;
    return;
    break;
  }
    // Write stuff to the frame!
    WriteData(isGondAccel, accelmode, NIOS_QUEUE); 
    WriteData(scanDVelMax, ((int)(a/0.5*65535.0)), NIOS_QUEUE); 
    WriteData(scanVelMax, ((int)(vmax/40.0*65535.0)), NIOS_QUEUE); 
  data = (int)((gTargetVel/60.0)*32767.0); 
    WriteData(dpsGondReq, data, NIOS_QUEUE);
    data=(int) ((theta/360.0)*65535.0);
    WriteData(gondTheta, data, NIOS_QUEUE);
    WriteData(scanAzVt1, ((int)(vt1/10.0*65535.0)), NIOS_QUEUE); 
    WriteData(scanX1, ((int)(x1/360.0*65535.0)), NIOS_QUEUE); 
    WriteData(scanX2, ((int)(x2/360.0*65535.0)), NIOS_QUEUE); 
  if(firsttime==1)
    { 
      bprintf(info,"getTargetVel: gTargetVel is %f",gTargetVel); 
      firsttime=0;
    }
  
}
// Looks at the target Velocity gTargetVel and sends commands to the motors.

void updateMotorSpeeds()
{
  static int testind=0;
  double dps;
  int data=0;
  double vcurr = ACSData.gyro2; // Gyro is in dps
  double verr=vcurr-gTargetVel; // Velocity error term.
  double vreac= RWData.vel;
  double gaccel = PointingData[GETREADINDEX(point_index)].dvdt;
  static int isfirst =1;
  static NiosStruct* dpsPivReq = NULL;
  static NiosStruct* iReacReq  = NULL;
  static NiosStruct* dpsPiv   = NULL;
  static NiosStruct* dpsRWFilt    = NULL;
  static NiosStruct* dpsErr    = NULL;
  
  if(isfirst==1)
    {
      dpsPiv  =GetNiosAddr("dps_piv");
      dpsPivReq = GetNiosAddr("dps_piv_req");
      iReacReq  = GetNiosAddr("i_reac_req");
      dpsRWFilt  =GetNiosAddr("dps_rw_filt");
      dpsErr=GetNiosAddr("dps_err");
      vpiv=0;
    }
    // Update the pivot velocity, which is stored in controller units in vpiv
    // (set in pivotComm)
  dps=((double) vpiv)/COUNTS_PER_DEGREE;
  data=(int) ((dps/70.0)*32767.0);
  //  bprintf(info,"updateMotorSpeeds: vpiv= %d, dps=%f, velocity= %i",vpiv,dps,data );
  WriteData(dpsPiv, data, NIOS_QUEUE);
  //  bprintf(info,"updateMotorSpeeds: vpiv= %d, dps=" );
// What mode are we in?
  WriteData(dpsRWFilt, ((int) (vreac/3000.0*32767.0)), NIOS_QUEUE);
  WriteData(dpsErr,((int) ((verr/72.0)*32767.0)), NIOS_QUEUE);
switch(CommandData.spiderMode){
case point:
  // lmf: For now use spin gains.
  // TODO: implement pointing gains.
  ireq=CommandData.spiderGain.sp_r1 * verr;
  pvreq=CommandData.spiderGain.sp_p2 * vreac;
  break;    
case spin:
  // TODO: Add new gains into the control loop
  pvreq=CommandData.spiderGain.sp_p1 * verr + CommandData.spiderGain.sp_p2 * vreac-vcurr;
  ireq=CommandData.spiderGain.sp_r1 * verr + CommandData.spiderGain.sp_r2 *gaccel;
  if(testind%100==0)
    {
        bprintf(info,"updateMotors: pvreq= %f, ireq= %f",pvreq,ireq);
    }
#if 0
  //  ireq=0;
  switch(testind/1000){
  case 0:
    ireq=0.0;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 1:
    ireq=-1;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 2:
    ireq=1;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 3:
    ireq=-1.5;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 4:
    ireq=1.5;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 5:
    ireq=-2;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 6:
    ireq=2;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 7:
    ireq=-3;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  case 8:
    ireq=3;
    if(testind%1000==0) bprintf(info,"updateMotorSpeeds: ireq = %f",ireq);
    break;
  default:
    if(testind%1000==0) ireq=0.0;
    break;
  }
  if(testind < 4000)
    {
  pvreq=-15.0;
    }
  if(testind >=4000 && testind < 8000)
    {
    pvreq=0;
    }
  if(testind >=8000)
    {
      pvreq=15;
    }
#endif
  break;
  case scan:
    ireq=CommandData.spiderGain.sc_r1 * verr;
    pvreq=CommandData.spiderGain.sc_p2 * vreac+ ACSData.gyro2
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
      //       bprintf(warning,"updateMotorSpeeds:Current reaction wheel speed %f is beyond the speed %f.\n Reaction wheel current set to 0.0.\n",vreac,MAX_RWHEEL_SPEED);
      ireq=0.0;
    }
  if (fabs(ireq) > MAX_RWHEEL_CURRENT)
    {
      if(ireq < 0.0) ireq=(-1.0)*MAX_RWHEEL_CURRENT;
      if(ireq > 0.0) ireq=MAX_RWHEEL_CURRENT;
      //        bprintf(warning,"updateMotorSpeeds: Requested ireq is above the current limits, setting %f\n",ireq);
    }
  // Is the pivot going too fast?
  // TODO: It seems that querying the pivot controller only gives the 
  //       requested pivot velocity.  Might be worth writing code that
  //       estimates true pivot velocity, by querying the position.

  if (fabs(pvreq) > MAX_PIVOT_SPEED)
    {
      //        bprintf(warning,"updateMotorSpeeds: Requested pivot Speed %f is above the max pivot speed %f.",pvreq,MAX_PIVOT_SPEED);
      //        bprintf(warning,"updateMotorSpeeds: setting pivot speed to maximum pivot speed."); 
	if(pvreq > 0)
	  {
	    pvreq= MAX_PIVOT_SPEED;
	  }
	else
	  {
	    pvreq=(-1.0)*MAX_PIVOT_SPEED;
	  }
    }

  // Is the gondola going too fast?  
  if (fabs(ACSData.gyro2)>MAX_GOND_SPEED)
    {
      
      // lmf: Once we have a better idea of the gains we'll put something
      // more intelligent here.
      ireq=0.0;
      pvreq=0.0;
      bprintf(warning,"updateMotorSpeeds:Gondola is rotating too fast!\n Setting ireq and pvreq to 0.0.");

    }
  data=(int)((pvreq/60.0)*32767.0);
  WriteData(dpsPivReq, data, NIOS_QUEUE);
  data=(int)((ireq/20.0)*32767.0);
  WriteData(iReacReq, data, NIOS_QUEUE);
  
  // Make the command strings and send them.
  testind++;  
  if(isfirst){
    isfirst=0;
  }
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
  static NiosStruct* spinGainR1Addr = NULL;
  static NiosStruct* spinGainR2Addr = NULL;
  static NiosStruct* spinGainP1Addr = NULL;
  static NiosStruct* spinGainP2Addr = NULL;
  static NiosStruct* scanGainR1Addr = NULL;
  static NiosStruct* scanGainR2Addr = NULL;
  static NiosStruct* scanGainP1Addr = NULL;
  static NiosStruct* scanGainP2Addr = NULL;
  static NiosStruct* scanAzCentreAddr = NULL;
  static NiosStruct* scanAzPeriodAddr = NULL;  
  static NiosStruct* scanAzWidthAddr = NULL; 
  static NiosStruct* scanAzPhiAddr = NULL;
  static NiosStruct* scanAzVt2Addr = NULL;
  static NiosStruct* scanAzWindAddr = NULL;
  static NiosStruct* pointAzAddr = NULL;
  static NiosStruct* pointTolAddr = NULL; 
  //initialization
  if (firsttime) {
    gPTableAddr = GetNiosAddr("g_p_table");
    gITableAddr = GetNiosAddr("g_i_table");
    gDTableAddr = GetNiosAddr("g_d_table");
    tableMoveAddr = GetNiosAddr("table_move");
    gTableMoveAddr = GetNiosAddr("g_table_move");
    spinGainR1Addr = GetNiosAddr("spin_gain_r1");
    spinGainR2Addr = GetNiosAddr("spin_gain_r2");
    spinGainP1Addr = GetNiosAddr("spin_gain_p1");
    spinGainP2Addr = GetNiosAddr("spin_gain_p2");
    scanGainR1Addr = GetNiosAddr("scan_gain_r1");
    scanGainR2Addr = GetNiosAddr("scan_gain_r2");
    scanGainP1Addr = GetNiosAddr("scan_gain_p1");
    scanGainP2Addr = GetNiosAddr("scan_gain_p2");
    scanAzCentreAddr = GetNiosAddr("scan_az_centre");
    scanAzPeriodAddr   = GetNiosAddr("scan_az_period");
    scanAzWidthAddr  = GetNiosAddr("scan_az_width");
    scanAzPhiAddr  = GetNiosAddr("scan_az_phi");
    scanAzVt2Addr  = GetNiosAddr("scan_az_vt2");
    scanAzWindAddr  = GetNiosAddr("scan_az_wind");
    pointAzAddr    = GetNiosAddr("point_az");
    pointTolAddr    = GetNiosAddr("point_tol");
    firsttime = false;
  }

  WriteData(gPTableAddr, CommandData.tableGain.P, NIOS_QUEUE);
  WriteData(gITableAddr, CommandData.tableGain.I, NIOS_QUEUE);
  WriteData(gDTableAddr, CommandData.tableGain.D, NIOS_QUEUE);
  WriteData(tableMoveAddr, (unsigned int)
      (CommandData.tableRelMove*10.0), NIOS_QUEUE);
  WriteData(gTableMoveAddr, (unsigned int)
      ((CommandData.tableMoveGain/100.0)*SHRT_MAX), NIOS_QUEUE);
  WriteData(spinGainR1Addr, ((int)(CommandData.spiderGain.sp_r1/SPR1_LIM*32767.0)), NIOS_QUEUE);
  WriteData(spinGainR2Addr, ((int)(CommandData.spiderGain.sp_r2/SPR2_LIM*32767.0)), NIOS_QUEUE);
  WriteData(spinGainP1Addr, ((int)(CommandData.spiderGain.sp_p1/SPP1_LIM*32767.0)), NIOS_QUEUE);
  WriteData(spinGainP2Addr, ((int)(CommandData.spiderGain.sp_p2/SPP2_LIM*32767.0)), NIOS_QUEUE);
  WriteData(scanGainR1Addr, ((int)(CommandData.spiderGain.sc_r1/SCR1_LIM*32767.0)), NIOS_QUEUE);
  WriteData(scanGainR2Addr, ((int)(CommandData.spiderGain.sc_r2/SCR2_LIM*32767.0)), NIOS_QUEUE);
  WriteData(scanGainP1Addr, ((int)(CommandData.spiderGain.sc_p1/SCP1_LIM*32767.0)), NIOS_QUEUE);
  WriteData(scanGainP2Addr, ((int)(CommandData.spiderGain.sc_p2/SCP2_LIM*32767.0)), NIOS_QUEUE);

  WriteData(scanAzCentreAddr, ((int)(CommandData.spiderScan.C/360.0*65535.0)), NIOS_QUEUE);
  WriteData(scanAzPeriodAddr, ((int)(CommandData.spiderScan.P/300.0*65535.0)), NIOS_QUEUE);
  WriteData(scanAzWidthAddr, ((int)(CommandData.spiderScan.W/120.0*65535.0)), NIOS_QUEUE);
  WriteData(scanAzPhiAddr, ((int)(CommandData.spiderScan.phi/90.0*65535.0)), NIOS_QUEUE);
  WriteData(scanAzVt2Addr, ((int)(CommandData.spiderScan.vt2/10.0*32767.0)), NIOS_QUEUE);
  WriteData(scanAzWindAddr, ((int)(CommandData.spiderScan.wind/60.0*65535.0)), NIOS_QUEUE);
  WriteData(pointAzAddr, ((int)(CommandData.spiderPoint.az/360.0*65535.0)), NIOS_QUEUE);
  WriteData(pointTolAddr, ((int)(CommandData.spiderPoint.tol/360.0*65535.0)), NIOS_QUEUE);
}

}       //extern "C"
