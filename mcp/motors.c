/* mcp: the BLAST master control program
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
#include <sys/time.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include "channels.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"
#include "copleycommand.h"
#include "amccommand.h"
#include "motordefs.h"

#define MIN_EL 10
#define MAX_EL 59

#define VPIV_FILTER_LEN 40
struct RWMotorDataStruct RWMotorData[3]; // defined in point_struct.h
int rw_motor_index; 

struct ElevMotorDataStruct ElevMotorData[3]; // defined in point_struct.h
int elev_motor_index; 

struct PivotMotorDataStruct PivotMotorData[3]; // defined in point_struct.h
int pivot_motor_index; 

struct AxesModeStruct axes_mode = {
  .el_dir = 1,
  .az_dir = 0
}; /* low level velocity mode */

void SetSafeDAz(double ref, double *A); /* in pointing.c */
void SetSafeDAzC(double ref, double *A, double *C); /* in pointing.c */
void UnwindDiff(double ref, double *A);

void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
    double *el);

/* in radbox.c */
void radbox_endpoints( double az[4], double el[4], double el_in,
    double *az_left, double *az_right, double *min_el,
    double *max_el, double *az_of_bot );

static pthread_t reactcomm_id;
static pthread_t elevcomm_id;
static pthread_t pivotcomm_id;

// device node address for the reaction wheel motor controller
#define REACT_DEVICE "/dev/ttySI9"
#define ELEV_DEVICE "/dev/ttySI11"
#define PIVOT_DEVICE "/dev/ttySI13"

static void* reactComm(void *arg);
static void* elevComm(void *arg);
static void* pivotComm(void *arg);

extern short int InCharge; /* tx.c */

/* opens communications with motor controllers */
void openMotors()
{
  bprintf(info, "Motors: connecting to motors");
  pthread_create(&reactcomm_id, NULL, &reactComm, NULL);
  pthread_create(&elevcomm_id, NULL, &elevComm, NULL);
  pthread_create(&pivotcomm_id, NULL, &pivotComm, NULL);
}

void closeMotors()
{
  int i=0;
  reactinfo.closing=1;
  elevinfo.closing=1; // Tell the serial threads to shut down.
  pivotinfo.closing=1;

  while(reactinfo.open==1 && elevinfo.open==1 && pivotinfo.open==1 && i++<100) usleep(10000);
}

static int last_mode = -1;

static double calcVPiv(void)
{
  double vpiv=0.0;
  static double buf_vPiv[VPIV_FILTER_LEN]; // Buffer for Piv boxcar filter.
  static time_t buf_t[VPIV_FILTER_LEN]; 
  static double a=0.0,alast=0.0; 
  static unsigned int ib_last=0;
  static int firsttime = 1;
  int i_point;
  int i;
  double dtheta=0.0;
  static int j=0;
  i_point = GETREADINDEX(point_index);
  if (firsttime) {
    firsttime = 0;
    // Initialize the buffer.  Assume all zeros to begin
    for(i=0;i>(VPIV_FILTER_LEN-1);i++) buf_vPiv[i]=0.0;
    for(i=0;i>(VPIV_FILTER_LEN-1);i++) buf_t[i]=0.0;
  }
  a+=(ACSData.res_piv_raw-buf_vPiv[ib_last]);
  //  dt=((double)(gettimeofday-buf_t[ib_last]));
  buf_vPiv[ib_last]=ACSData.res_piv_raw;
  //  dummy=PointingData[i_point].t
    //  buf_t[ib_last]=PointingData[i_point].t;
  ib_last=(ib_last+VPIV_FILTER_LEN+1)%VPIV_FILTER_LEN;
  dtheta=(a-alast)/VPIV_FILTER_LEN;
  alast=a;
  vpiv=dtheta/0.010016; 
  //  if (j%100 == 1) bprintf(info,"CalcVPiv vpiv = %f, res_piv_raw = %f, a = %f, alast = %f, dtheta = %f ",vpiv,ACSData.res_piv_raw,a/VPIV_FILTER_LEN,alast/VPIV_FILTER_LEN,dtheta);
  j++;
  return vpiv;
}
/************************************************************************/
/*                                                                      */
/*   GetVElev: get the current elevation velocity, given current        */
/*   pointing mode, etc..                                               */
/*                                                                      */
/*   Units are 0.1*gyro unit                                            */
/************************************************************************/
static double GetVElev(void)
{
  double vel = 0;
  static double last_vel = 0;
  double dvel;
  int i_point;
  double max_dv = 1.6;
  double el_for_limit, el, el_dest;
  double dy;
  i_point = GETREADINDEX(point_index);

  if (axes_mode.el_mode == AXIS_VEL) {
    vel = axes_mode.el_vel;
  } else if (axes_mode.el_mode == AXIS_POSITION) {
    el = PointingData[i_point].el;
    el_dest = axes_mode.el_dest;
    dy = el_dest - el;
    if (dy<0) {
      vel = -sqrt(-dy);
    } else {
      vel = sqrt(dy);
    }
    vel *= 0.3;
    //    vel = (axes_mode.el_dest - PointingData[i_point].el) * 0.36;
  } else if (axes_mode.el_mode == AXIS_LOCK) {
    /* for the lock, only use the elevation encoder */
    vel = (axes_mode.el_dest - ACSData.enc_el_raw) * 0.64;
 }

  /* correct offset and convert to Gyro Units */
  vel -= (PointingData[i_point].gy_ifel_offset - PointingData[i_point].gy_ifel_earth);

  if (CommandData.use_elenc) {
    el_for_limit = ACSData.enc_el_raw;
  } else {
    el_for_limit = PointingData[i_point].el;
  }

  if (el_for_limit < MIN_EL) {
    if (vel<=0) { // if we are going down
      vel = (MIN_EL - el_for_limit)*0.36; // go to the stop 
    }
  }
  if (el_for_limit > MAX_EL) {
    if (vel>=0) { // if we are going up
      vel = (MAX_EL - el_for_limit)*0.36; // go to the stop 
    }
    //vel = -0.2; // go down
  }

  /* Limit Maximim speed to 0.5 dps*/
  if (vel > 0.5)
    vel = 0.5;
  if (vel < -0.5)
    vel = -0.5;

  vel *= DPS_TO_GY16;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv)
    vel = last_vel + max_dv;
  if (dvel < -max_dv)
    vel = last_vel - max_dv;
  last_vel = vel;

  //bprintf(info, "GetVEl: vel=%f", vel);
  return (vel*10.0); // Factor of 10.0 is to improve the dynamic range
}

/************************************************************************/
/*                                                                      */
/*   GetVAz: get the current az velocity, given current                 */
/*   pointing mode, etc..                                               */
/*                                                                      */
/*   Units are in 0.1*gyro units                                        */
/************************************************************************/
static double GetVAz(void)
{
  double vel = 0.0;
  static double last_vel = 0;
  double dvel;
  int i_point;
  double vel_offset;
  double az, az_dest;
  double max_dv = 20;
  double dx;

  i_point = GETREADINDEX(point_index);

  if (axes_mode.az_mode == AXIS_VEL) {
    vel = axes_mode.az_vel;
  } else if (axes_mode.az_mode == AXIS_POSITION) {
    az = PointingData[i_point].az;
    az_dest = axes_mode.az_dest;
    SetSafeDAz(az, &az_dest);
    dx = az_dest - az;
    if (dx<0) {
      vel = -sqrt(-dx);
    } else {
      vel = sqrt(dx);
    }
    vel *= 0.3;
    //vel = -(az - az_dest) * 0.36;
  }

  vel_offset =
    -(PointingData[i_point].gy_ifroll_offset- PointingData[i_point].gy_ifroll_earth)*
    cos(PointingData[i_point].el * M_PI / 180.0) -
    (PointingData[i_point].gy_ifyaw_offset- PointingData[i_point].gy_ifyaw_earth)*
    sin(PointingData[i_point].el * M_PI / 180.0);

  vel -= vel_offset;
  vel *= DPS_TO_GY16;

  /* Limit Maximim speed */
  if (vel > 2000.0)
    vel = 2000.0;
  if (vel < -2000.0)
    vel = -2000.0;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv)
    vel = last_vel + max_dv;
  if (dvel < -max_dv)
    vel = last_vel - max_dv;
  last_vel = vel;

  return (vel*10.0); // Factor of 10 is to increase dynamic range
}

/************************************************************************/
/*                                                                      */
/*     GetIPivot: get the current request for the pivot in DAC units    */
/*       Proportional to the reaction wheel speed error                 */
/*                                                                      */
/************************************************************************/
static double GetIPivot(unsigned int g_piv, unsigned int disabled)
{
  double I_req = 0.0;
  int I_req_dac= 0;
  int i_point;
 
  i_point = GETREADINDEX(point_index);
  I_req = (-1.0)*(double)g_piv*(ACSData.rw_vel_raw-CommandData.pivot_gain.SP); 

  // TODO: Add in term proportional to velocity error.
  if(disabled) { // Don't attempt to send current to the motors if we are disabled.
    I_req=0.0;
  }

  /* Convert to DAC Units*/
  if(fabs(I_req)<0.02) {
    I_req_dac=16384+PIV_DAC_OFF;
  } else {
    if(I_req>0.0) {
      I_req_dac=I_req+16384+PIV_DAC_OFF+PIV_DEAD_BAND;
    } else {
      I_req_dac=I_req+16384+PIV_DAC_OFF-PIV_DEAD_BAND;
    }
  }

  // Check to make sure the DAC value is in the proper range
  if(I_req_dac <= 0) {
    I_req_dac=1;
  }
  if(I_req_dac >  32767) {
    I_req_dac=32767;
  }
  return I_req_dac;
}

/************************************************************************/
/*                                                                      */
/*    WriteMot: motors, and, for convenience, the inner frame lock      */
/*                                                                      */
/************************************************************************/
void WriteMot(int TxIndex, unsigned short *RxFrame)
{
  static struct NiosStruct* elVreqAddr;
  static struct NiosStruct* azVreqAddr;
  static struct NiosStruct* cosElAddr;
  static struct NiosStruct* sinElAddr;

  static struct NiosStruct* gPElAddr;
  static struct NiosStruct* gIElAddr;
  static struct NiosStruct* gPAzAddr;
  static struct NiosStruct* gIAzAddr;
  static struct NiosStruct* gPVPivotAddr;
  static struct NiosStruct* setReacAddr;
  static struct NiosStruct* pivVReqDACAddr;
  static struct NiosStruct* pivVCalcAddr;
 
  //TODO temporary
  static struct NiosStruct* dacAmplAddr[5];
  int i;
  static int wait = 100; /* wait 20 frames before controlling. */
  double el_rad;
  unsigned int ucos_el;
  unsigned int usin_el;

  int v_elev, v_az, v_piv, elGainP, elGainI;
  int azGainP, azGainI, pivGainRW;
  int i_point;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elVreqAddr = GetNiosAddr("el_vreq");
    azVreqAddr = GetNiosAddr("az_vreq");
    cosElAddr = GetNiosAddr("cos_el");
    sinElAddr = GetNiosAddr("sin_el");
    pivVReqDACAddr = GetNiosAddr("piv_dac");
    gPElAddr = GetNiosAddr("g_p_el");
    gIElAddr = GetNiosAddr("g_i_el");
    gPAzAddr = GetNiosAddr("g_p_az");
    gIAzAddr = GetNiosAddr("g_i_az");
    gPVPivotAddr = GetNiosAddr("g_pv_pivot");
    setReacAddr = GetNiosAddr("set_reac");
    pivVCalcAddr = GetNiosAddr("piv_dps_calc");
    dacAmplAddr[0] = GetNiosAddr("dac1_ampl");
    dacAmplAddr[1] = GetNiosAddr("dac2_ampl");
    //    dacAmplAddr[2] = GetNiosAddr("dac3_ampl"); // is now piv_dac
    //    dacAmplAddr[3] = GetNiosAddr("dac4_ampl"); // is now dac_el
    //    dacAmplAddr[4] = GetNiosAddr("dac5_ampl"); // is now rw_dac 
  }

  i_point = GETREADINDEX(point_index);

  //TODO need to change the write to the BLASTbus here and in the DSP
  // code so that it writes a 15 bit number.  Otherwise Narsil shows 
  // twice the current value and it is rather confusing. 
  //TODO temporary
  if (wait <= 0)
    for (i=0; i<2; i++)
      if (CommandData.Temporary.setLevel[i]) {
	WriteData(dacAmplAddr[i], CommandData.Temporary.dac_out[i], NIOS_QUEUE);
	CommandData.Temporary.setLevel[i] = 0;
      }

  /***************************************************/
  /**           Elevation Drive Motors              **/
  /* elevation speed */
  v_elev = floor(GetVElev() + 0.5);
  /* Unit of v_elev are 0.1 gyro units */
  if (v_elev > 32767)
    v_elev = 32767;
  if (v_elev < -32768)
    v_elev = -32768;
  WriteData(elVreqAddr, 32768 + v_elev, NIOS_QUEUE);

  /* zero motor gains if the pin is in */
  if ((CommandData.pin_is_in && !CommandData.force_el)
      || CommandData.disable_el)
    elGainP = elGainI = 0;
  else {
    elGainP = CommandData.ele_gain.P;
    elGainI = CommandData.ele_gain.I;	
  }
  /* proportional term for el motor */
  WriteData(gPElAddr, elGainP, NIOS_QUEUE);
  /* integral term for el_motor */
  WriteData(gIElAddr, elGainI, NIOS_QUEUE);


  /***************************************************/
  /*** Send elevation angles to acs1 from acs2 ***/
  /* cos of el enc */
  el_rad = (M_PI / 180.0) * PointingData[i_point].el; /* convert to radians */
  ucos_el = (unsigned int)((cos(el_rad) + 1.0) * 32768.0);
  WriteData(cosElAddr, ucos_el, NIOS_QUEUE);
  /* sin of el enc */
  usin_el = (unsigned int)((sin(el_rad) + 1.0) * 32768.0);
  WriteData(sinElAddr, usin_el, NIOS_QUEUE);

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = floor(GetVAz() + 0.5);
  /* Units for v_az are 0.1*(16 bit gyro units)*/
  if (v_az > 32767)
    v_az = 32767;
  if (v_az < -32768)
    v_az = -32768;
  WriteData(azVreqAddr, 32768 + v_az, NIOS_QUEUE);


  if ((CommandData.disable_az) || (wait > 0)) {
    azGainP = 0;
    azGainI = 0;
    pivGainRW = 0;
    v_piv=GetIPivot(pivGainRW,1);
  } else {
    azGainP = CommandData.azi_gain.P;
    azGainI = CommandData.azi_gain.I;
    pivGainRW = CommandData.pivot_gain.PV;
    v_piv=GetIPivot(pivGainRW,0);
  }
  /* requested pivot velocity*/
  WriteData(pivVReqDACAddr, v_piv*2, NIOS_QUEUE);
  /* p term for az motor */
  WriteData(gPAzAddr, azGainP, NIOS_QUEUE);
  /* I term for az motor */
  WriteData(gIAzAddr, azGainI, NIOS_QUEUE);
  /* p term to rw vel for pivot motor */
  WriteData(gPVPivotAddr, pivGainRW, NIOS_QUEUE);
  /* setpoint for reaction wheel */
  WriteData(setReacAddr, CommandData.pivot_gain.SP*65536.0/2.5, NIOS_QUEUE);
  /* Pivot velocity */
  WriteData(pivVCalcAddr, (calcVPiv()/20.0*32768.0), NIOS_QUEUE);

  if (wait > 0)
    wait--;
}

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/
#define AZ_ACCEL (0.001)
#define MIN_SCAN 0.2
static void SetAzScanMode(double az, double left, double right, double v,
    double D)
{
    if (axes_mode.az_vel < -v + D)
      axes_mode.az_vel = -v + D;
    if (axes_mode.az_vel > v + D)
      axes_mode.az_vel = v + D;

    if (az < left) {
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel < v + D)
        axes_mode.az_vel += AZ_ACCEL;
    } else if (az > right) {
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > -v + D)
        axes_mode.az_vel -= AZ_ACCEL;
    } else {
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > 0) {
        axes_mode.az_vel = v + D;
        if (az > right - v) /* within 1 sec of turnaround */
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        else
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      } else {
        axes_mode.az_vel = -v + D;
        if (az < left + v) /* within 1 sec of turnaround */
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        else
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      }
    }
  }

static void DoAzScanMode(void)
{
  static double last_x=0, last_w = 0;
  double az, left, right, v,w;
  int i_point;

  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = CommandData.pointing_mode.Y;
  axes_mode.el_vel  = 0.0;

  i_point = GETREADINDEX(point_index);
  az = PointingData[i_point].az; /* FIXME - extrapolate velocity */

  w = CommandData.pointing_mode.w;
  right = CommandData.pointing_mode.X + w / 2;
  left = CommandData.pointing_mode.X - w / 2;

  SetSafeDAz(left, &az);

  v = CommandData.pointing_mode.vaz;

  if (last_x!= CommandData.pointing_mode.X || last_w != w) {
    if (az < left) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else if (az > right) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = right;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else {
      // once we are within the new az/w range, we can mark this as 'last'.
      last_x = CommandData.pointing_mode.X;
      last_w = w;
      SetAzScanMode(az, left, right, v, 0);
    }
  } else {
    SetAzScanMode(az, left, right, v, 0);
  }
}

#define EL_BORDER 1.0
static void DoVCapMode(void)
{
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt, v_el;
  double lst;
  int i_point;
  double y, r,v;
  double x2, xw;
  double left, right;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  if (el > 80)
    el = 80; /* very bad situation - dont know how this can happen */
  if (el < -10)
    el = -10; /* very bad situation - dont know how this can happen */

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;
  SetSafeDAz(az, &caz);

  /* get elevation limits */
  if (cel < MIN_EL)
    cel = MIN_EL;
  if (cel > MAX_EL)
    cel = MAX_EL;
  r = CommandData.pointing_mode.w;
  el1 = cel + r;
  el2 = cel - r;
  if (el1 > MAX_EL)
    el1 = MAX_EL;
  if (el2 < MIN_EL)
    el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1 + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    axes_mode.el_dir = -1;
    return;
  } else if (el < el2 - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    axes_mode.el_dir = 1;
    return;
  } else if (el > el1) { /* turn around */
    axes_mode.el_dir = -1;
  } else if (el < el2) { /* turn around */
    axes_mode.el_dir = 1;
  }
  v_el = CommandData.pointing_mode.del * axes_mode.el_dir;

  /* we must be in range for elevation - go to el-vel mode */
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

  /** Get x limits **/
  y = el - cel;
  x2 = r * r - y * y;
  if (x2 < 0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN)
    xw = MIN_SCAN;
  xw /= cos(el * M_PI / 180.0);
  left = caz - xw;
  right = caz + xw;

  /* set az v */
  v = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v, daz_dt);
}

static void DoVBoxMode(void)
{
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt, v_el;
  double lst;
  int i_point;
  double y, x, v;
  double left, right;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  if (el > 80)
    el = 80; /* very bad situation - dont know how this can happen */
  if (el < -10)
    el = -10; /* very bad situation - dont know how this can happen */

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;
  SetSafeDAz(az, &caz);

  /* get elevation limits */
  if (cel < MIN_EL)
    cel = MIN_EL;
  if (cel > MAX_EL)
    cel = MAX_EL;
  y = CommandData.pointing_mode.h / 2.0;
  el1 = cel + y;
  el2 = cel - y;
  if (el1 > MAX_EL)
    el1 = MAX_EL;
  if (el2 < MIN_EL)
    el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1 + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    axes_mode.el_dir = -1;
    return;
  } else if (el < el2 - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    axes_mode.el_dir = 1;
    return;
  } else if (el> el1) { /* turn around */
    axes_mode.el_dir = -1;
  } else if (el < el2) { /* turn around */
    axes_mode.el_dir = 1;
  }
  v_el = CommandData.pointing_mode.del * axes_mode.el_dir;

  /* we must be in range for elevation - go to el-vel mode */
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

  /** Get x limits **/
  x = CommandData.pointing_mode.w / 2.0;
  x = x / cos(el * M_PI / 180.0);

  left = caz - x;
  right = caz + x;

  /* set az v */
  v = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v, daz_dt);
}

static void DoRaDecGotoMode(void)
{
  double caz, cel;
  double lst, az;
  int i_point;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;

  az = PointingData[i_point].az;

  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  SetSafeDAz(az, &caz);

  axes_mode.az_mode = AXIS_POSITION;
  axes_mode.az_dest = caz;
  axes_mode.az_vel = 0.0;
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = cel;
  axes_mode.el_vel = 0.0;
  isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
}

static void DoNewCapMode(void)
{
  double caz, cel, r, x2, y, xw;
  double bottom, top, left, right;
  double next_left, next_right, az_distance;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  int i_point;
  int new_step = 0;

  static double last_X=0, last_Y=0, last_w=0;
  static double v_el = 0;
  static double targ_el=0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  SetSafeDAz(az, &caz);

  r = CommandData.pointing_mode.w;
  bottom = cel - r;
  top = cel + r;

  // FIXME: reboot proofing...

  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (last_mode != P_CAP)) {
    if ( (fabs(az - (caz)) < 0.1) &&
        (fabs(el - (bottom)) < 0.05)) {
      last_X = CommandData.pointing_mode.X;
      last_Y = CommandData.pointing_mode.Y;
      last_w = CommandData.pointing_mode.w;
    } else {
      last_w = 0; // remember we are moving...
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = caz;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      v_el = 0.0;
      targ_el = -r;
      axes_mode.el_dir = 1;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }

  /** Get x limits at the next elevation row **/
  y = targ_el; //el - cel + CommandData.pointing_mode.del*el_dir;
  x2 = r * r - y * y;
  if (x2 < 0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN)
    xw = MIN_SCAN;
  xw /= cos(el * M_PI / 180.0);
  next_left = caz - xw;
  next_right = caz + xw;

  /** Get x limits at the current elevation **/
  y = el - cel;
  x2 = r * r - y * y;
  if (x2 < 0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN)
    xw = MIN_SCAN;
  xw /= cos(el * M_PI / 180.0);
  left = caz - xw;
  right = caz + xw;

  /* set az v */
  v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v_az, daz_dt);

  /** set El V **/
  new_step = 0;
  if (az<left) {
    if (axes_mode.az_dir < 0) {
      az_distance = next_right - left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el - (el-cel))/t;
    // set targ_el for the next step
    targ_el += CommandData.pointing_mode.del*axes_mode.el_dir;
    if (targ_el>=r) {
      targ_el = r;
      axes_mode.el_dir=-1;
    } else if (targ_el<=-r) {
      targ_el = -r;
      axes_mode.el_dir = 1;
    }
  }

  el1 = cel + r;
  el2 = cel - r;
  if (el1 > MAX_EL)
    el1 = MAX_EL;
  if (el2 < MIN_EL)
    el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1 + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    //axes_mode.el_dir = -1;
    if (v_el > 0) {
      v_el = -v_el;
    }
    return;
  } else if (el < el2 - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    //axes_mode.el_dir = 1;
    if (v_el < 0) {
      v_el = -v_el;
    }
    return;
  }
  /* else if (el > el1) { */
  /*     axes_mode.el_dir = -1; */
  /*   } else if (el < el2) {  */
  /*     axes_mode.el_dir = 1; */
  /*   }     */

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

static void DoNewBoxMode(void)
{
  double caz, cel, w, h;
  double bottom, top, left, right;
  double az, az2, el, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  int i_point;
  int new_step = 0;
  int new = 0;

  static double last_X=0, last_Y=0, last_w=0, last_h = 0;
  static double v_el = 0;
  static double targ_el=0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  SetSafeDAz(az, &caz);

  w = CommandData.pointing_mode.w/cos(el * M_PI / 180.0);
  h = CommandData.pointing_mode.h;
  bottom = cel - h*0.5;
  top = cel + h*0.5;
  left = caz - w*0.5;
  right = caz + w*0.5;

  if (top > MAX_EL)
    top = MAX_EL;
  if (bottom < MIN_EL)
    bottom = MIN_EL;

  // FIXME: reboot proofing...

  new = 0;

  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (CommandData.pointing_mode.h != last_h) ||
      (last_mode != P_BOX)) {
    new = 1;
  }
  if (el < bottom - 0.5) new = 1;
  if (el > top + 0.5) new = 1;
  if (az < left - 2.0) new = 1;
  if (az > right + 2.0) new = 1;

  /* If a new command, reset to bottom row */
  if (new) {
    if ( (fabs(az - left) < 0.1) &&
        (fabs(el - bottom) < 0.05)) {
      last_X = CommandData.pointing_mode.X;
      last_Y = CommandData.pointing_mode.Y;
      last_w = CommandData.pointing_mode.w;
      last_h = CommandData.pointing_mode.h;
    } else {
      last_w = 0; // remember we are moving...
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      v_el = 0.0;
      targ_el = -h*0.5;
      axes_mode.el_dir = 1;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }

  /* set az v */
  v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v_az, daz_dt);

  /** set El V **/
  new_step = 0;
  if (az<left) {
    if (axes_mode.az_dir < 0) {
      t = w/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      t = w/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el - (el-cel))/t;
    // set targ_el for the next step
    targ_el += CommandData.pointing_mode.del*axes_mode.el_dir;
    if (targ_el>h*0.5) {
      targ_el = h*0.5;
      axes_mode.el_dir=-1;
    } else if (targ_el<-h*0.5) {
      targ_el = -h*0.5;
      axes_mode.el_dir = 1;
    }
  }
  /* check for out of range in el */
  if (el > top + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = top;
    axes_mode.el_dir = -1;
    if (v_el > 0) {
      v_el = -v_el;
    }
    return;
  } else if (el < bottom - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = bottom;
    axes_mode.el_dir = 1;
    if (v_el < 0) {
      v_el = -v_el;
    }
    return;
  }

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

void DoQuadMode(void) // aka radbox
{
  double bottom, top, left, right, next_left, next_right, az_distance;
  double az, az2, el, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  int i, i_point;
  int new_step = 0;
  double c_az[4], c_el[4]; // corner az and corner el
  double az_of_bot;
  int new;
  //int i_top, i_bot, new;

  static double last_ra[4] = {0,0,0,0}, last_dec[4] = {0,0,0,0};
  static double v_el = 0;
  static double targ_el=0.0; // targ_el is in degrees from bottom

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  /* convert ra/decs to az/el */
  for (i=0; i<4; i++) {
    radec2azel(CommandData.pointing_mode.ra[i],
        CommandData.pointing_mode.dec[i],
        lst, lat,
        c_az+i, c_el+i);
  }

  /* get sky drift speed */
  radec2azel(CommandData.pointing_mode.ra[0],
      CommandData.pointing_mode.dec[0],
      lst+1.0, lat,
      &az2, &el2);

  UnwindDiff(az, &az2);

  daz_dt = drem(az2 - c_az[0], 360.0);
  del_dt = el2 - c_el[0];

  radbox_endpoints(c_az, c_el, el, &left, &right, &bottom, &top, &az_of_bot);

  SetSafeDAz(az, &left); // don't cross the sun
  UnwindDiff(left, &right);

  SetSafeDAz(az, &az_of_bot); // correct left

  if (right-left < MIN_SCAN) {
    left = (left+right)/2.0 - MIN_SCAN/2.0;
    right = left + MIN_SCAN;
  }

  new = 0;
  if (last_mode != P_QUAD) new = 1;
  if (el < bottom - 0.5) new = 1;
  if (el > top + 0.5) new = 1;
  if (az < left - 2.0) new = 1;
  if (az > right + 2.0) new = 1;

  for (i=0; i<4; i++) {
    if (CommandData.pointing_mode.ra[i] != last_ra[i]) new = 1;
    if (CommandData.pointing_mode.dec[i] != last_dec[i]) new = 1;
  }

  if (new) {
    if ( (fabs(az - az_of_bot) < 0.1) &&
        (fabs(el - bottom) < 0.05)) {
      for (i=0; i<4; i++) {
        last_ra[i] = CommandData.pointing_mode.ra[i];
        last_dec[i] = CommandData.pointing_mode.dec[i];
      }
    } else {
      last_dec[0] = -99.9745; // remember it is new....
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = az_of_bot;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      v_el = 0.0;
      targ_el = 0.0;
      axes_mode.el_dir = 1;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }

  if (targ_el<0) {
    targ_el = 0;
  }
  if (targ_el>top-bottom) {
    targ_el = top-bottom;
  }

  radbox_endpoints(c_az, c_el, targ_el+bottom, &next_left,
      &next_right, &bottom, &top, &az_of_bot);


  // make next close to this...
  UnwindDiff(left, &next_left);
  UnwindDiff(left, &next_right);

  if (next_right-next_left < MIN_SCAN) {
    next_left = (next_left+next_right)/2.0 - MIN_SCAN/2.0;
    next_right = next_left + MIN_SCAN;
  }

  /* set az v */
  v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v_az, daz_dt);

  /** set El V **/
  new_step = 0;
  if (az<left) {
    if (axes_mode.az_dir < 0) {
      az_distance = next_right - left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el+bottom - el)/t;
    // set targ_el for the next step
    targ_el += CommandData.pointing_mode.del*axes_mode.el_dir;
    if (targ_el>top-bottom) {
      targ_el = top-bottom;
      axes_mode.el_dir=-1;
    } else if (targ_el<0) {
      targ_el = 0;
      axes_mode.el_dir = 1;
    }
  }

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

/******************************************************************
 *                                                                *
 * Update Axis Modes: Set axes_mode based on                      *
 *    CommandData.pointing_mode                                   *
 *                                                                *
 ******************************************************************/
void UpdateAxesMode(void)
{
  switch (CommandData.pointing_mode.mode) {
    case P_DRIFT:
      axes_mode.el_mode = AXIS_VEL;
      axes_mode.el_vel = CommandData.pointing_mode.del;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = CommandData.pointing_mode.vaz;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast =
        (sqrt(CommandData.pointing_mode.vaz * CommandData.pointing_mode.vaz
              + CommandData.pointing_mode.del * CommandData.pointing_mode.del)
         > MAX_ISC_SLOW_PULSE_SPEED) ? 1 : 0;
      break;
    case P_AZEL_GOTO:
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = CommandData.pointing_mode.X;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
    case P_AZ_SCAN:
      DoAzScanMode();
      break;
    case P_VCAP:
      DoVCapMode();
      break;
    case P_VBOX:
      DoVBoxMode();
      break;
    case P_BOX:
      DoNewBoxMode();
      break;
    case P_CAP:
      DoNewCapMode();
      break;
    case P_RADEC_GOTO:
      DoRaDecGotoMode();
      break;
    case P_QUAD: // aka radbox
      DoQuadMode();
      break;
    case P_LOCK:
      axes_mode.el_mode = AXIS_LOCK;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
    default:
      bprintf(warning, "Pointing: Unknown Elevation Pointing Mode %d: "
          "stopping\n", CommandData.pointing_mode.mode);
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      axes_mode.el_mode = AXIS_VEL;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
  }
  last_mode = CommandData.pointing_mode.mode;
}

// Makes a motor info bit field
unsigned short int makeMotorField(struct MotorInfoStruct* motorinfo)
{
  unsigned short int b = 0; 
  b |= (motorinfo->open) & 0x0001;

  if (motorinfo->reset) {
    b|= 0x02;
#ifdef MOTORS_VERBOSE
    bprintf(info, "%sComm makeMotorField: reset set",motorinfo->motorstr);
#endif
  }
 
  //b |= ((motorinfo->reset) & 0x0001)<<1; 
  b |= ((motorinfo->init) & 0x0003)<<2; 
  b |= ((motorinfo->disabled) & 0x0003)<<4; 
  switch(motorinfo->bdrate) {
  case 9600:
    b |= 0x0000<<6;
    break;
  case 38400:
    b |= 0x0001<<6;
    break;
  case 112500:
    b |= 0x0002<<6;
    break;
  default:
    b |= 0x0003;
  }
  b |= ((motorinfo->writeset) & 0x0003)<<8 ; 
  b |= ((motorinfo->err) & 0x001f)<<10 ; 
  b |= ((motorinfo->closing) & 0x0001)<<15 ; 
  return b;
}
// TODO-lmf: Need to add in conditional statements for when MCP is run by the NICC
//           We don't want the NICC sending 
void* reactComm(void* arg)
{
  //mark1
  int n=0, j=0;
  int i=0;
  int temp_raw,curr_raw,stat_raw,faultreg_raw;
  int firsttime=1,resetcount=0;
  long vel_raw=0;
  // Initialize values in the reactinfo structure.                            
  reactinfo.open=0;
  reactinfo.init=0;
  reactinfo.err=0;
  reactinfo.err_count=0;
  reactinfo.closing=0;
  reactinfo.reset=0;
  reactinfo.disabled=2;
  reactinfo.bdrate=9600;
  reactinfo.writeset=0;
  strncpy(reactinfo.motorstr,"react",6);


  while(!InCharge) {
    if(firsttime==1) {
      bprintf(info,"reactComm: I am not incharge thus I will not communicate with the RW motor.");
      firsttime=0;
    }
    //in case we switch to ICC when serial communications aren't working
    RWMotorData[0].rw_vel_raw=ACSData.rw_vel_raw;
    RWMotorData[1].rw_vel_raw=ACSData.rw_vel_raw;
    RWMotorData[2].rw_vel_raw=ACSData.rw_vel_raw;
    usleep(20000);
  }

  firsttime=1;
  bprintf(info,"reactComm: Bringing the reaction wheel online.");
  // Initialize structure RWMotorData.  Follows what was done in dgps.c
  RWMotorData[0].rw_vel_raw=0;
  RWMotorData[0].temp=0;
  RWMotorData[0].current=0.0;
  RWMotorData[0].status=0;
  RWMotorData[0].fault_reg=0;
  RWMotorData[0].drive_info=0;
  RWMotorData[0].err_count=0;

  // Try to open the port.
  while (reactinfo.open==0) {
    open_copley(REACT_DEVICE,&reactinfo); // sets reactinfo.open=1 if sucessful

    if (i==10) bputs(err,"reactComm: Reaction wheel port could not be opened after 10 attempts.\n");

    i++;
    if (reactinfo.open==1) {
#ifdef MOTORS_VERBOSE
      bprintf(info,"reactComm: Opened the serial port on attempt number %i",i); 
#endif
    } else sleep(1);  
  }

  // Configure the serial port.  If after 10 attempts the port is not initialized it enters 
  // the main loop where it will trigger a reset command.                                             
  i=0;
  while (reactinfo.init==0 && i <=9) {
    configure_copley(&reactinfo);
    if (reactinfo.init==1) {
      bprintf(info,"reactComm: Initialized the controller on attempt number %i",i); 
    } else if (i==9) {
      bprintf(info,"reactComm: Could not initialize the controller after %i attempts.",i); 
    } else {
      sleep(1);
    }
    i++;
  }
  rw_motor_index = 1; // index for writing to the RWMotor data struct
  while (1){

    if((reactinfo.err & COP_ERR_MASK) > 0 ) {
      reactinfo.err_count+=1;
      if(reactinfo.err_count >= COPLEY_ERR_TIMEOUT) {
	reactinfo.reset=1;
      }
    }
    if(CommandData.reset_reac==1 ) {
      reactinfo.reset=1;
      CommandData.reset_reac=0;
    }

    RWMotorData[rw_motor_index].drive_info=makeMotorField(&reactinfo); // Make bitfield of controller info structure.
    RWMotorData[rw_motor_index].err_count=(reactinfo.err_count > 65535) ? 65535: reactinfo.err_count;

    if(reactinfo.closing==1){
      rw_motor_index=INC_INDEX(rw_motor_index);
      close_copley(&reactinfo);
      usleep(10000);      
    } else if (reactinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,"reactComm: Resetting connection to Reaction Wheel controller.");
      } else if ((resetcount % 10)==0) {
	bprintf(warning,"reactComm: reset-> Unable to connect to Reaction Wheel after %i attempts.",resetcount);
      }

      resetcount++;
      rw_motor_index=INC_INDEX(rw_motor_index);
      resetCopley(REACT_DEVICE,&reactinfo); // if successful sets reactinfo.reset=0
      usleep(10000);  // give time for motor bits to get written
      if (reactinfo.reset==0) {
	resetcount=0;
      }

    } else if(reactinfo.init==1){
      if(CommandData.disable_az==0 && reactinfo.disabled > 0) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"reactComm: Attempting to enable the reaction wheel motor controller.");
#endif
	n=enableCopley(&reactinfo);
	if(n==0){    
	  bprintf(info,"reactComm: Reaction wheel motor controller is now enabled.");
	  reactinfo.disabled=0;
	}
      } 
      if(CommandData.disable_az==1 && (reactinfo.disabled==0 || reactinfo.disabled==2)) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"reactComm: Attempting to disable the reaction wheel motor controller.");
#endif
	n=disableCopley(&reactinfo);
	if(n==0){    
	  bprintf(info,"reactComm: Reaction wheel motor controller is now disabled.");
	  reactinfo.disabled=1;
	}
      } 

      vel_raw=queryCopleyInd(COP_IND_VEL,&reactinfo); // Units are 0.1 counts/sec
      RWMotorData[rw_motor_index].rw_vel_raw=((double) vel_raw)/RW_ENC_CTS/10.0*360.0; 
      j=j%4;
      switch(j) {
      case 0:
	temp_raw=queryCopleyInd(COP_IND_TEMP,&reactinfo);
        RWMotorData[rw_motor_index].temp=temp_raw; // units are deg Cel
	break;
      case 1:
	curr_raw=queryCopleyInd(COP_IND_CURRENT,&reactinfo);
        RWMotorData[rw_motor_index].current=((double) (curr_raw))/100.0; // units are Amps
	break;
      case 2:
	stat_raw=queryCopleyInd(COP_IND_STATUS,&reactinfo);
        RWMotorData[rw_motor_index].status=stat_raw; 
	break;
      case 3:
	faultreg_raw=queryCopleyInd(COP_IND_FAULTREG,&reactinfo);
        RWMotorData[rw_motor_index].fault_reg=faultreg_raw; 
	break;
      }      
      j++;
      if (firsttime) {
#ifdef MOTORS_VERBOSE
        bprintf(info,"reactComm: Raw reaction wheel velocity is %i",vel_raw);
#endif
	firsttime=0;
      }
      rw_motor_index=INC_INDEX(rw_motor_index);

    } else {
      usleep(10000);
    }
    i++;
  }
  return NULL;
}


void* elevComm(void* arg)
{

  int n=0, j=0;
  int i=0;
  long unsigned pos_raw;
  int temp_raw,curr_raw,stat_raw,faultreg_raw;
  int firsttime=1,resetcount=0;


  // Initialize values in the elevinfo structure.                            
  elevinfo.open=0;
  elevinfo.init=0;
  elevinfo.err=0;
  elevinfo.err_count=0;
  elevinfo.closing=0;
  elevinfo.reset=0;
  elevinfo.disabled=2;
  elevinfo.bdrate=9600;
  elevinfo.writeset=0;
  strncpy(elevinfo.motorstr,"elev\0",6);

  while(!InCharge) {
    if(firsttime==1) {
      bprintf(info,"elevComm: I am not incharge thus I will not communicate with the elevation drive.");
      firsttime=0;
    }

    //in case we switch to ICC when serial communications aren't working
    ElevMotorData[0].enc_el_raw=ACSData.enc_el_raw;
    ElevMotorData[1].enc_el_raw=ACSData.enc_el_raw;
    ElevMotorData[2].enc_el_raw=ACSData.enc_el_raw;
    usleep(20000);
  }

  bprintf(info,"elevComm: Bringing the elevation drive online.");
  i=0;
  firsttime=1;

  // Initialize structure ElevMotorData.  Follows what was done in dgps.c
  ElevMotorData[0].enc_el_raw=0;
  ElevMotorData[0].temp=0;
  ElevMotorData[0].current=0.0;
  ElevMotorData[0].status=0;
  ElevMotorData[0].fault_reg=0;
  ElevMotorData[0].drive_info=0;
  ElevMotorData[0].err_count=0;

  // Try to open the port.
  while(elevinfo.open==0) {
    open_copley(ELEV_DEVICE,&elevinfo); // sets elevinfo.open=1 if sucessful
    
    if(i==10) {
      bputs(err,"elevComm: Elevation drive serial port could not be opened after 10 attempts.\n");
    }
    i++;
    
    if(elevinfo.open==1) {
#ifdef MOTORS_VERBOSE
      bprintf(info,"elevComm: Opened the serial port on attempt number %i",i); 
#endif
    }
    else sleep(1);
  }
 
  // Configure the serial port.  If after 10 attempts the port is not initialized it enters 
  // the main loop where it will trigger a reset command.                                             
  i=0;
  while (elevinfo.init==0 && i <=9) {
    configure_copley(&elevinfo);
    if(elevinfo.init==1) {
      bprintf(info,"elevComm: Initialized the controller on attempt number %i",i); 
    } else if (i==9) {
      bprintf(info,"elevComm: Could not initialize the controller after %i attempts.",i); 
    } else {
      sleep(1);
    }
    i++;
  }

  elev_motor_index = 1; // index for writing to the ElevMotor data struct
  while (1) {

    if ((elevinfo.err & COP_ERR_MASK) > 0 ) {
      elevinfo.err_count+=1;
      if (elevinfo.err_count >= COPLEY_ERR_TIMEOUT) {
	elevinfo.reset=1;
      }
    }

    if (CommandData.reset_elev==1 ) {
      elevinfo.reset=1;
      CommandData.reset_elev=0;
    }

    ElevMotorData[elev_motor_index].drive_info=makeMotorField(&elevinfo); // Make bitfield of controller info structure.
    ElevMotorData[elev_motor_index].err_count=(elevinfo.err_count > 65535) ? 65535: elevinfo.err_count;

    if(elevinfo.closing==1){
      elev_motor_index=INC_INDEX(elev_motor_index);
      close_copley(&elevinfo);
      usleep(10000);      
    } else if (elevinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,"elevComm: Resetting connection to elevation drive controller.");
      } else if ((resetcount % 10)==0) {
	bprintf(warning,"elevComm: reset-> Unable to connect to elevation drive after %i attempts.",resetcount);
      }

      resetcount++;
      elev_motor_index=INC_INDEX(elev_motor_index);
      resetCopley(ELEV_DEVICE,&elevinfo); // if successful sets elevinfo.reset=0

      usleep(10000);  // give time for motor bits to get written
      if (elevinfo.reset==0) {
	resetcount=0;
	bprintf(info,"elevinfo: PING! elevinfo.init=%i, elevinfo.open=%i, elevinfo.err=%i",elevinfo.init, elevinfo.open,elevinfo.err);
      }

    } else if (elevinfo.init==1) {
      if((CommandData.disable_el==0 || CommandData.force_el==1 ) && elevinfo.disabled > 0) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"elevComm: Attempting to enable the elevation motor controller.");
#endif
	n=enableCopley(&elevinfo);
	if(n==0){    
	  bprintf(info,"elevComm: Elevation motor controller is now enabled.");
	  elevinfo.disabled=0;
	}
      } 
      if((CommandData.disable_el==1 && CommandData.force_el==0 ) && (elevinfo.disabled==0 || elevinfo.disabled==2)) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"elevComm: Attempting to disable the elevation motor controller.");
#endif
	n=disableCopley(&elevinfo);
	if(n==0){    
	  bprintf(info,"elevComm: Elevation motor controller is now disabled.");
	  elevinfo.disabled=1;
	}
      } 

      pos_raw=queryCopleyInd(COP_IND_POS,&elevinfo); // Units are counts
                                                     // For Elev 524288 cts = 360 deg
      //TODO-lmf: Add in some sort of zeropoint.
      ElevMotorData[elev_motor_index].enc_el_raw=((double) (pos_raw % ((long int) ELEV_ENC_CTS)))/ELEV_ENC_CTS*360.0-ENC_EL_RAW_OFFSET;
      //   getCopleySlowInfo(j,elev_motor_index,&ElevMotorData,&elevinfo); // Reads one of temperature, current, status and fault register and
                           // writes to the appropriate frame 

      if (firsttime) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"elevComm: Raw elevation encoder position is %i",pos_raw);
#endif
	firsttime=0;
      }     
      j=j%4;
      switch(j) {
      case 0:
	temp_raw=queryCopleyInd(COP_IND_TEMP,&elevinfo);
        ElevMotorData[elev_motor_index].temp=temp_raw; // units are deg Cel
	break;
      case 1:
	curr_raw=queryCopleyInd(COP_IND_CURRENT,&elevinfo);
        ElevMotorData[elev_motor_index].current=((double) (curr_raw))/100.0; // units are Amps
	break;
      case 2:
	stat_raw=queryCopleyInd(COP_IND_STATUS,&elevinfo);
        ElevMotorData[elev_motor_index].status=stat_raw; // units are Amps
	break;
      case 3:
	faultreg_raw=queryCopleyInd(COP_IND_FAULTREG,&elevinfo);
        ElevMotorData[elev_motor_index].fault_reg=faultreg_raw; // units are Amps
	break;
      }      
      j++;
      elev_motor_index=INC_INDEX(elev_motor_index);
    } else {
      usleep(10000);
    }
  }
  return NULL;
}

void* pivotComm(void* arg) 
{
  int n=0, j=0;
  int i=0;
  long unsigned pos_raw=0;
  int firsttime=1,resetcount=0;
  unsigned int dp_stat_raw=0, db_stat_raw=0, ds1_stat_raw=0;
  short int current_raw=0;
  int piv_vel_raw=0;
  unsigned int tmp=0;
  // Initialize values in the pivotinfo structure.                            
  pivotinfo.open=0;
  pivotinfo.init=0;
  pivotinfo.err=0;
  pivotinfo.err_count=0;
  pivotinfo.closing=0;
  pivotinfo.reset=0;
  pivotinfo.disabled=2;
  pivotinfo.bdrate=9600;
  pivotinfo.writeset=0;
  strncpy(pivotinfo.motorstr,"pivot",6);

  while (!InCharge) {
    if (firsttime==1) {
      bprintf(info,"pivotComm: I am not incharge thus I will not communicate with the pivot motor.");
      firsttime=0;
    }
    usleep(20000);
  }

  bprintf(info,"pivotComm: Bringing the pivot drive online.");
  firsttime=1;

  i=0;

  // Initialize structure PivotMotorData.  Follows what was done in dgps.c
  PivotMotorData[0].res_piv_raw=0;
  PivotMotorData[0].current=0;
  PivotMotorData[0].db_stat=0;
  PivotMotorData[0].dp_stat=0;
  PivotMotorData[0].ds1_stat=0;
  PivotMotorData[0].dps_piv=0;
  PivotMotorData[0].drive_info=0;
  PivotMotorData[0].err_count=0;

  // Try to open the port.
  while (pivotinfo.open==0) {
    open_amc(PIVOT_DEVICE,&pivotinfo); // sets pivotinfo.open=1 if sucessful

    if (i==10) {
      bputs(err,"pivotComm: Pivot controller serial port could not be opened after 10 attempts.\n");
    }
    i++;

    if (pivotinfo.open==1) {
#ifdef MOTORS_VERBOSE
      bprintf(info,"pivotComm: Opened the serial port on attempt number %i",i);
#endif
    }
    else sleep(1);
  }

  // Configure the serial port.  If after 10 attempts the port is not initialized it enters 
  // the main loop where it will trigger a reset command.                                             
  i=0;
  while (pivotinfo.init==0 && i <=9) {
    configure_amc(&pivotinfo);
    if (pivotinfo.init==1) {
      bprintf(info,"pivotComm: Initialized the controller on attempt number %i",i); 
    } else if (i==9) {
      bprintf(info,"pivotComm: Could not initialize the controller after %i attempts.",i); 
    } else {
      sleep(1);
    }
    i++;
  }

  while (1) {

    if((pivotinfo.err & AMC_ERR_MASK) > 0 ) {
      pivotinfo.err_count+=1;
      if(pivotinfo.err_count >= AMC_ERR_TIMEOUT) {
	pivotinfo.reset=1;
      }
    }
    if(CommandData.reset_piv==1 ) {
      pivotinfo.reset=1;
      CommandData.reset_piv=0;
    }

    PivotMotorData[pivot_motor_index].drive_info=makeMotorField(&pivotinfo); // Make bitfield of controller info structure.
    PivotMotorData[pivot_motor_index].err_count=(pivotinfo.err_count > 65535) ? 65535: pivotinfo.err_count;

    if(pivotinfo.closing==1){
      pivot_motor_index=INC_INDEX(pivot_motor_index);
      close_amc(&pivotinfo);
      usleep(10000);      
    } else if (pivotinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,"pivotComm: Resetting connection to pivot controller.");
      } else if ((resetcount % 50)==0) {
	bprintf(warning,"pivotComm: reset->Unable to connect to pivot after %i attempts.",resetcount);
      }

      resetcount++;
      pivot_motor_index=INC_INDEX(pivot_motor_index);
      resetAMC(PIVOT_DEVICE,&pivotinfo); // if successful sets pivotinfo.reset=0

      if (pivotinfo.reset==0) resetcount=0;

      usleep(10000);  // give time for motor bits to get written

    } else if (pivotinfo.init==1) {
      if(CommandData.disable_az==0 && pivotinfo.disabled == 1) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"pivotComm: Attempting to enable the pivot motor contoller.");
#endif
	n=enableAMC(&pivotinfo);
	if(n==0) {
	  bprintf(info,"pivotComm: Pivot motor is now enabled");
	  pivotinfo.disabled=0;
	}
      }
      if(CommandData.disable_az==1 && (pivotinfo.disabled==0 || pivotinfo.disabled==2)) {
#ifdef MOTORS_VERBOSE
	bprintf(info,"pivotComm: Attempting to disable the pivot motor controller.");
#endif
	n=disableAMC(&pivotinfo);
	if(n==0){    
	  bprintf(info,"pivotComm: Pivot motor controller is now disabled.");
	  pivotinfo.disabled=1;
	}
      } 

      if(firsttime){
	firsttime=0;
	tmp = queryAMCInd(0x32,8,1,&pivotinfo);
	bprintf(info,"pivotComm: Ki = %i",tmp);
	tmp = queryAMCInd(0xd8,0x24,1,&pivotinfo);
	bprintf(info,"pivotComm: Ks = %i",tmp);
	tmp = queryAMCInd(0xd8,0x0c,1,&pivotinfo);
	bprintf(info,"pivotComm: d8.0ch = %i",tmp);
	tmp = queryAMCInd(216,12,1,&pivotinfo);
	bprintf(info,"pivotComm: v2 d8.0ch = %i",tmp);
	tmp = queryAMCInd(0xd8,0x12,1,&pivotinfo);
	bprintf(info,"pivotComm: d8.12h = %i",tmp);
	tmp = queryAMCInd(216,18,1,&pivotinfo);
	bprintf(info,"pivotComm: v2 d8.12h = %i",tmp);
	tmp = queryAMCInd(0xd8,0x13,1,&pivotinfo);
	bprintf(info,"pivotComm: d8.13h = %i",tmp);
	tmp = queryAMCInd(216,19,1,&pivotinfo);
	bprintf(info,"pivotComm: v2 d8.13h = %i",tmp);
      }

      pos_raw=getAMCResolver(&pivotinfo);
      //      bprintf(info,"pivotComm: Resolver Position is: %i",pos_raw);
      PivotMotorData[pivot_motor_index].res_piv_raw=((double) pos_raw)/PIV_RES_CTS*360.0; 

      j=j%5;
      switch(j) {
      case 0:
	current_raw=queryAMCInd(16,3,1,&pivotinfo);
        PivotMotorData[pivot_motor_index].current=((double)current_raw)/8192.0*20.0; // *2^13 / peak drive current
	                                                                             // Units are Amps
	//        bprintf(info,"pivotComm: current_raw= %i, current= %f",current_raw,PivotMotorData[pivot_motor_index].current);
	break;
      case 1:
	db_stat_raw=queryAMCInd(2,0,1,&pivotinfo);
        PivotMotorData[pivot_motor_index].db_stat=db_stat_raw;
	break;
      case 2:
	dp_stat_raw=queryAMCInd(2,1,1,&pivotinfo);
        PivotMotorData[pivot_motor_index].dp_stat=dp_stat_raw;
	break;
      case 3:
	ds1_stat_raw=queryAMCInd(2,3,1,&pivotinfo);
        PivotMotorData[pivot_motor_index].ds1_stat=ds1_stat_raw;
	break;
      case 4:
	piv_vel_raw=((int) queryAMCInd(17,2,2,&pivotinfo));
        PivotMotorData[pivot_motor_index].dps_piv=piv_vel_raw*0.144;
	break;
      }
      j++;
      pivot_motor_index=INC_INDEX(pivot_motor_index);
    } else {
      usleep(10000);
    }
  }
  return NULL;
}
