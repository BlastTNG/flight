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
#include <stdlib.h>
#include <sys/time.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "channels_tng.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"
#include "copleycommand.h"
#include "amccommand.h"
#include "motordefs.h"

#include <angles.h>
#include <conversions.h>
#include <mputs.h>
#include <pointing.h>
#include <radbox.h>
#include <motors.h>
#include <ec_motors.h>

struct MotorDataStruct RWMotorData[3] = {{0}};
struct MotorDataStruct ElevMotorData[3] = {{0}};
struct MotorDataStruct PivotMotorData[3] = {{0}};
int motor_index = 0;

struct AxesModeStruct axes_mode = {
  .el_dir = 1,
  .az_dir = 0,
  .i_dith = 0
}; /* low level velocity mode */


extern short int InCharge; /* tx.c */

extern int StartupVeto; /* mcp.c */

double az_accel = 0.1;

static int last_mode = -1;

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
    int i_point, i_elev;
    double max_dv = 1.6;
    double el_for_limit, el, el_dest;
    double dy;

    i_point = GETREADINDEX(point_index);
    i_elev = GETREADINDEX(motor_index);

    if (axes_mode.el_mode == AXIS_VEL) {
        vel = axes_mode.el_vel;
    }
    else if (axes_mode.el_mode == AXIS_POSITION) {
        el = PointingData[i_point].el;
        el_dest = axes_mode.el_dest;
        dy = el_dest - el;
        if (dy < 0) {
            vel = -sqrt(-dy);
        }
        else {
            vel = sqrt(dy);
        }
        vel *= (double) CommandData.ele_gain.PT / 1000.0;
        //    vel = (axes_mode.el_dest - PointingData[i_point].el) * 0.36;
    }
    else if (axes_mode.el_mode == AXIS_LOCK) {
        /* for the lock, only use the elevation encoder */
        vel = (axes_mode.el_dest - ElevMotorData[i_elev].position) * 0.64;
    }

    /* correct offset and convert to Gyro Units */
    vel -= (PointingData[i_point].offset_ifel_gy - PointingData[i_point].ifel_earth_gy);

    if (CommandData.use_elenc) {
        el_for_limit = ElevMotorData[i_elev].position;
    }
    else {
        el_for_limit = PointingData[i_point].el;
    }

    if (el_for_limit < MIN_EL) {
        if (vel <= 0) { // if we are going down
            vel = (MIN_EL - el_for_limit) * 0.36; // go to the stop
        }
    }
    if (el_for_limit > MAX_EL) {
        if (vel >= 0) { // if we are going up
            vel = (MAX_EL - el_for_limit) * 0.36; // go to the stop
        }
        //vel = -0.2; // go down
    }

    /* Limit Maximim speed to 0.5 dps*/
    if (vel > MAX_V_EL)
        vel = MAX_V_EL;
    if (vel < (-1.0) * MAX_V_EL)
        vel = (-1.0) * MAX_V_EL;

    vel *= DPS_TO_GY16;

    /* limit Maximum acceleration */
    dvel = vel - last_vel;
    if (dvel > max_dv)
        vel = last_vel + max_dv;
    if (dvel < -max_dv)
        vel = last_vel - max_dv;
    last_vel = vel;

    //bprintf(info, "GetVEl: vel=%f", vel);
    return (vel * 10.0); // Factor of 10.0 is to improve the dynamic range
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
    vel *= (double)CommandData.azi_gain.PT*(15.0/10000.0);
    //vel = -(az - az_dest) * 0.36;
  }

  vel_offset =
    -(PointingData[i_point].offset_ifroll_gy - PointingData[i_point].ifroll_earth_gy)*
    cos(PointingData[i_point].el * M_PI / 180.0) -
    (PointingData[i_point].offset_ifyaw_gy - PointingData[i_point].ifyaw_earth_gy)*
    sin(PointingData[i_point].el * M_PI / 180.0);

  vel -= vel_offset;
  /* Limit Maximim speed */
  if (vel > MAX_V_AZ)
    vel = MAX_V_AZ;
  if (vel < -MAX_V_AZ)
    vel = -MAX_V_AZ;

  vel *= DPS_TO_GY16;


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
//TODO: Change GetIPivot to return units of 0.01A for motor controller
static double GetIPivot(int v_az_req_gy, unsigned int g_rw_piv, unsigned int g_err_piv, double frict_off_piv, unsigned int disabled)
{
  static channel_t* pRWTermPivAddr;
  static channel_t* pErrTermPivAddr;
  static channel_t* frictTermPivAddr;
  static channel_t* frictTermUnfiltPivAddr; // For debugging only.  Remove later!
  static double buf_frictPiv[FPIV_FILTER_LEN]; // Buffer for Piv friction term boxcar filter.
  static double a=0.0; 
  static unsigned int ib_last=0;
  double I_req = 0.0;
  int I_req_dac = 0;
  int i_point, i_rw;
  double v_az_req,i_frict,i_frict_filt;
  double p_rw_term, p_err_term;
  int p_rw_term_dac, p_err_term_dac;
 
  static int i=0;
  static unsigned int firsttime = 1;

  if(firsttime) {
    pRWTermPivAddr = channels_find_by_name("p_rw_term_piv");
    pErrTermPivAddr = channels_find_by_name("p_err_term_piv");
    frictTermPivAddr = channels_find_by_name("frict_term_piv");
    frictTermUnfiltPivAddr = channels_find_by_name("frict_term_uf_piv");
    // Initialize the buffer.  Assume all zeros to begin
    for(i=0;i<(FPIV_FILTER_LEN-1);i++) buf_frictPiv[i]=0.0;
    firsttime = 0;
  }

  v_az_req = ((double) v_az_req_gy) * GY16_TO_DPS/10.0; // Convert to dps 

  i_point = GETREADINDEX(point_index);
  i_rw = GETREADINDEX(motor_index);
  p_rw_term = (-1.0)*((double)g_rw_piv/10.0)*(RWMotorData[i_rw].velocity-CommandData.pivot_gain.SP);
  p_err_term = (double)g_err_piv*5.0*(v_az_req-PointingData[i_point].v_az);
  I_req = p_rw_term+p_err_term;


  if(disabled) { // Don't attempt to send current to the motors if we are disabled.
    I_req=0.0;
  }

  // Calculate static friction offset term
  if(fabs(I_req)<100) {
    i_frict=0.0;
  } else {
    if(I_req>0.0) {
      i_frict=frict_off_piv;
    } else {
      i_frict=(-1.0)*frict_off_piv;
    }
  }

  /* Convert to DAC Units*/

  if(fabs(I_req)<100) {
    I_req_dac=16384+PIV_DAC_OFF;
  } else {
    if(I_req>0.0) {
      I_req_dac=I_req+16384+PIV_DAC_OFF+PIV_DEAD_BAND;
    } else {
      I_req_dac=I_req+16384+PIV_DAC_OFF-PIV_DEAD_BAND;
    }
  }

  a+=(i_frict-buf_frictPiv[ib_last]);
  buf_frictPiv[ib_last]=i_frict;
  ib_last=(ib_last+FPIV_FILTER_LEN+1)%FPIV_FILTER_LEN;
  i_frict_filt=a/((double) FPIV_FILTER_LEN);

  I_req_dac += i_frict_filt*PIV_I_TO_DAC;
  //  if(i%20==1) bprintf(info,"Motors: a=%f,ib_last=%i,i_frict=%f,i_frict_filt=%f,I_req=%f,I_req_dac_init=%i,I_req_dac=%i",a,ib_last,i_frict,i_frict_filt,I_req,I_req_dac_init,I_req_dac);

  if(fabs(p_rw_term)<100) {
    p_rw_term_dac=16384+PIV_DAC_OFF;
  } else {
    if(p_rw_term>0.0) {
      p_rw_term_dac=p_rw_term+16384+PIV_DAC_OFF+PIV_DEAD_BAND;
    } else {
      p_rw_term_dac=p_rw_term+16384+PIV_DAC_OFF-PIV_DEAD_BAND;
    }
  }

  if(fabs(p_err_term)<100) {
    p_err_term_dac=16384+PIV_DAC_OFF;
  } else {
    if(p_err_term>0.0) {
      p_err_term_dac=p_err_term+16384+PIV_DAC_OFF+PIV_DEAD_BAND;
    } else {
      p_err_term_dac=p_err_term+16384+PIV_DAC_OFF-PIV_DEAD_BAND;
    }
  }


  // Check to make sure the DAC value is in the proper range
  if(I_req_dac <= 0) {
    I_req_dac=1;
  }
  if(I_req_dac >  32767) {
    I_req_dac=32767;
  }
  // Check to make sure the P-terms are in the proper range
  if(p_rw_term_dac <= 0) {
    p_rw_term_dac=1;
  }
  if(p_rw_term_dac >  32767) {
    p_rw_term_dac=32767;
  }
  if(p_err_term_dac <= 0) {
    p_err_term_dac=1;
  }
  if(p_err_term_dac >  32767) {
    p_err_term_dac=32767;
  }

  i++;

  SET_INT16(pRWTermPivAddr,p_rw_term);
  SET_INT16(pErrTermPivAddr,p_err_term);
  SET_INT16(frictTermPivAddr,i_frict_filt*32767.0/2.0);
  SET_INT16(frictTermUnfiltPivAddr,i_frict*32767.0/2.0);
  return I_req_dac;
}

/************************************************************************/
/*                                                                      */
/*    WriteMot: motors, and, for convenience, the inner frame lock      */
/*                                                                      */
/************************************************************************/
void WriteMot(void)
{
  static channel_t* velReqElAddr;
  static channel_t* velReqAzAddr;
  static channel_t* cosElAddr;
  static channel_t* sinElAddr;

  static channel_t* gPElAddr;
  static channel_t* gIElAddr;
  static channel_t* gPtElAddr;
  static channel_t* gPAzAddr;
  static channel_t* gIAzAddr;
  static channel_t* gPtAzAddr;
  static channel_t* gPVPivAddr;
  static channel_t* gPEPivAddr;
  static channel_t* setRWAddr;
  static channel_t* frictOffPivAddr;
  static channel_t* dacPivAddr;
  static channel_t* velCalcPivAddr;
  static channel_t* accelAzAddr;
 
  double el_rad;
  unsigned int ucos_el;
  unsigned int usin_el;

  int v_elev, v_az, i_piv, elGainP, elGainI;
  int azGainP, azGainI, pivGainRW, pivGainErr;
  double pivFrictOff;
  int i_point;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    velReqElAddr = channels_find_by_name("vel_req_el");
    velReqAzAddr = channels_find_by_name("vel_req_az");
    cosElAddr = channels_find_by_name("cos_el");
    sinElAddr = channels_find_by_name("sin_el");
    dacPivAddr = channels_find_by_name("mc_piv_i_cmd");
    gPElAddr = channels_find_by_name("g_p_el");
    gIElAddr = channels_find_by_name("g_i_el");
    gPtElAddr = channels_find_by_name("g_pt_el");
    gPAzAddr = channels_find_by_name("g_p_az");
    gIAzAddr = channels_find_by_name("g_i_az");
    gPtAzAddr = channels_find_by_name("g_pt_az");
    gPVPivAddr = channels_find_by_name("g_pv_piv");
    gPEPivAddr = channels_find_by_name("g_pe_piv");
    setRWAddr = channels_find_by_name("set_rw");
    frictOffPivAddr = channels_find_by_name("frict_off_piv");
    velCalcPivAddr = channels_find_by_name("vel_calc_piv");
    accelAzAddr = channels_find_by_name("accel_az");
  }

  i_point = GETREADINDEX(point_index);


  /***************************************************/
  /**           Elevation Drive Motors              **/
  /* elevation speed */
  v_elev = floor(GetVElev() + 0.5);
  /* Unit of v_elev are 0.1 gyro units */
  if (v_elev > 32767)
    v_elev = 32767;
  if (v_elev < -32768)
    v_elev = -32768;
  SET_VALUE(velReqElAddr, 32768 + v_elev);

  /* zero motor gains if the pin is in */
  if ((CommandData.pin_is_in && !CommandData.force_el)
      || CommandData.disable_el)
    elGainP = elGainI = 0;
  else {
    elGainP = CommandData.ele_gain.P;
    elGainI = CommandData.ele_gain.I;	
  }
  /* proportional term for el motor */
  SET_VALUE(gPElAddr, elGainP);
  el_set_p(elGainP);
  /* integral term for el_motor */
  SET_VALUE(gIElAddr, elGainI);
  el_set_i(elGainI);
  /* pointing gain term for elevation drive */
  SET_VALUE(gPtElAddr, CommandData.ele_gain.PT);
  //TODO:Figure out what to do about the Pointing gain term

  /***************************************************/
  /*** Send elevation angles to acs1 from acs2 ***/
  /* cos of el enc */
  el_rad = (M_PI / 180.0) * PointingData[i_point].el; /* convert to radians */
  ucos_el = (unsigned int)((cos(el_rad) + 1.0) * 32768.0);
  SET_VALUE(cosElAddr, ucos_el);
  /* sin of el enc */
  usin_el = (unsigned int)((sin(el_rad) + 1.0) * 32768.0);
  SET_VALUE(sinElAddr, usin_el);

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = floor(GetVAz() + 0.5);
  /* Units for v_az are 0.1*(16 bit gyro units)*/
  if (v_az > 32767)
    v_az = 32767;
  if (v_az < -32768)
    v_az = -32768;
  SET_VALUE(velReqAzAddr, 32768 + v_az);


  if (CommandData.disable_az) {
    azGainP = 0;
    azGainI = 0;
    pivGainRW = 0;
    pivGainErr = 0;
    pivFrictOff = 0.0;
    i_piv=GetIPivot(0,pivGainRW,pivGainErr,pivFrictOff,1);
  } else {
    azGainP = CommandData.azi_gain.P;
    azGainI = CommandData.azi_gain.I;
    pivGainRW = CommandData.pivot_gain.PV;
    pivGainErr = CommandData.pivot_gain.PE;
    pivFrictOff = CommandData.pivot_gain.F;
    i_piv=GetIPivot(v_az,pivGainRW,pivGainErr,pivFrictOff,0);
  }
  //  bprintf(info,"Motors: pivFrictOff= %f, CommandData.pivot_gain.F = %f",pivFrictOff,CommandData.pivot_gain.F);
  /* requested pivot current*/
  SET_VALUE(dacPivAddr, i_piv*2);
  /* p term for az motor */
  SET_VALUE(gPAzAddr, azGainP);
  /* I term for az motor */
  SET_VALUE(gIAzAddr, azGainI);
  /* pointing gain term for az drive */
  SET_VALUE(gPtAzAddr, CommandData.azi_gain.PT);

  /* p term to rw vel for pivot motor */
  SET_VALUE(gPVPivAddr, pivGainRW);
  /* p term to vel error for pivot motor */
  SET_VALUE(gPEPivAddr, pivGainErr);
  /* setpoint for reaction wheel */
  SET_VALUE(setRWAddr, CommandData.pivot_gain.SP*32768.0/200.0);
  /* Pivot current offset to compensate for static friction. */
  SET_VALUE(frictOffPivAddr, pivFrictOff/2.0*65535);
  /* Pivot velocity */
  SET_VALUE(velCalcPivAddr, piv_get_velocity());
  /* Azimuth Scan Acceleration */
  SET_VALUE(accelAzAddr, (CommandData.az_accel/2.0*65536.0));

}

/***************************************************************/
/*                                                             */
/* GetElDither: set the current elevation dither offset.       */
/*                                                             */
/***************************************************************/
static void GetElDither(unsigned int inc) {
  // Set up the random variable.

  if (inc) { 
    (axes_mode.i_dith)++;
    bprintf(info,"GetElDither: Incrementing axes_mode.i_dith to %i",axes_mode.i_dith);
  }
  if (CommandData.pointing_mode.n_dith <= 0) {
    axes_mode.el_dith=0.0;
    if (inc) bprintf(info,"No dither: axes_mode.el_dith = %f",axes_mode.el_dith);
  } else {
    //    bprintf(info,"GetElDither: CommandData.pointing_mode.n_dith = %i",CommandData.pointing_mode.n_dith);

    axes_mode.i_dith%=(CommandData.pointing_mode.n_dith);
    //bprintf(info,"GetElDither: axes_mode.i_dith is now %i",axes_mode.i_dith);
    axes_mode.el_dith=2.0*(CommandData.pointing_mode.del)*((double) axes_mode.i_dith)/((double)(CommandData.pointing_mode.n_dith));
    //bprintf(info,"GetElDither: axes_mode.el_dith is finally %i",axes_mode.i_dith);
    
  }					    


  if (inc) bprintf(info,"***Dither Time!!!***  El Dither = %f",axes_mode.el_dith);
  
  if (axes_mode.el_dith > CommandData.pointing_mode.del) {
    axes_mode.el_dith += (-2.0)*CommandData.pointing_mode.del;
    if (inc) bprintf(info,"GetElDither: Wrapping dither... axes_mode.el_dith=%f",axes_mode.el_dith);
  }
  
  return;
}

static void InitElDither() {
  static int j = 0;
  if (CommandData.pointing_mode.next_i_dith >= 0) {
    axes_mode.i_dith = CommandData.pointing_mode.next_i_dith;
    //    bprintf(info,"InitElDither:%i nid = %i, next_dith=%i,  axes_mode.i_dith = %i",j,CommandData.pointing_mode.next_i_dith,CommandData.pointing_mode.next_i_dith,axes_mode.i_dith);
    CommandData.pointing_mode.next_i_dith = -1;
  } else {
    CommandData.pointing_mode.next_i_dith = -1;
    //    bprintf(info,"InitElDither:%i CommandData.pointing_mode.next_i_dith =%i, so axes_mode.i_dith = %i",j,CommandData.pointing_mode.next_i_dith,axes_mode.i_dith);  
  }

  if (CommandData.pointing_mode.next_i_hwpr >= 0 && CommandData.pointing_mode.next_i_hwpr < 4) {
    CommandData.hwpr.i_pos = CommandData.pointing_mode.next_i_hwpr;
    CommandData.hwpr.mode = HWPR_GOTO_I;
    CommandData.hwpr.is_new = 1;
    //    bprintf(info,"InitElDither:%i Sending HWPR to index = %i",j,CommandData.pointing_mode.next_i_hwpr);
    CommandData.pointing_mode.next_i_hwpr=-1;
  } else {
    //    bprintf(info,"InitElDither:%i CommandData.pointing_mode.next_i_hwpr =%i, so we will do nothing",j,CommandData.pointing_mode.next_i_hwpr);  
    CommandData.pointing_mode.next_i_hwpr = -1;
  }

  //  bprintf(info,"InitElDither: axes_mode.el_dith = %f",axes_mode.el_dith);
  j++;
  GetElDither(NO_DITH_INC);
  return;
}

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/
#define MIN_SCAN 0.2
static void SetAzScanMode(double az, double left, double right, double v, double D)
{
    if (axes_mode.az_vel < -v + D)
        axes_mode.az_vel = -v + D;
    if (axes_mode.az_vel > v + D)
        axes_mode.az_vel = v + D;

    ///TODO: Update AzScanMode with XSC data
    if (az < left) {
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        axes_mode.az_mode = AXIS_VEL;
        if (axes_mode.az_vel < v + D)
            axes_mode.az_vel += az_accel;
    }
    else if (az > right) {
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        axes_mode.az_mode = AXIS_VEL;
        if (axes_mode.az_vel > -v + D)
            axes_mode.az_vel -= az_accel;
    }
    else {
        axes_mode.az_mode = AXIS_VEL;
        if (axes_mode.az_vel > 0) {
            axes_mode.az_vel = v + D;
//        if (az > right - 2.0*v) /* within 2 sec of turnaround */
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
//        else
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
        }
        else {
            axes_mode.az_vel = -v + D;
//        if (az < left + 2.0*v) /* within 2 sec of turnaround */
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
//        else
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
        }
    }
}

static void SetElScanMode(double el, double bottom, double top, double v, double D)
{
//    double before_trig;
    double el_accel = EL_ACCEL / SR;
    if (axes_mode.el_vel < -v + D)
        axes_mode.el_vel = -v + D;
    if (axes_mode.el_vel > v + D)
        axes_mode.el_vel = v + D;

    //TODO: Update ElScanMode with XSC routines
    if (el < bottom) {
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        axes_mode.el_mode = AXIS_VEL;
        if (axes_mode.el_vel < v + D)
            axes_mode.el_vel += el_accel;
    }
    else if (el > top) {
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        axes_mode.el_mode = AXIS_VEL;
        if (axes_mode.el_vel > -v + D)
            axes_mode.el_vel -= el_accel;
    }
    else {
        axes_mode.el_mode = AXIS_VEL;
        if (axes_mode.el_vel > 0) {
            axes_mode.el_vel = v + D;
//            if (el > top - 2.0 * v) { /* within 2 sec of turnaround */
//              isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
//            }
//        else
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
        }
        else {
            axes_mode.el_vel = -v + D;
//        if (el < bottom + 2.0*v) /* within 2 sec of turnaround */
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
//        else
//          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
  az = PointingData[i_point].az; 

  w = CommandData.pointing_mode.w;
  right = CommandData.pointing_mode.X + w / 2;
  left = CommandData.pointing_mode.X - w / 2;

  SetSafeDAz(left, &az);

  v = CommandData.pointing_mode.vaz;

  //TODO: Update DoAzScanMode with XSC Routine
  if (last_x!= CommandData.pointing_mode.X || last_w != w) {
    if (az < left) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else if (az > right) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = right;
      axes_mode.az_vel = 0.0;
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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

static void DoElScanMode(void)
{
  static double last_y=0, last_h = 0;
  double el, top, bottom, v,h;
  //  double az, left, right, v,w;
  int i_point;
  static int first_time=1;
  
  axes_mode.az_mode = AXIS_POSITION;
  axes_mode.az_dest = CommandData.pointing_mode.X;
  axes_mode.az_vel  = 0.0;

  i_point = GETREADINDEX(point_index);
  el = PointingData[i_point].el; 

  h = CommandData.pointing_mode.h;
  top = CommandData.pointing_mode.Y + h / 2;
  bottom = CommandData.pointing_mode.Y - h / 2;

  if (first_time) bprintf(info,"Starting an elevation scan! h = %f, top=%f , bottom=%f",h,top,bottom);
  first_time=0;

  //  SetSafeDAz(left, &az); // Don't think I need this because I should be staying constant in az. Test!

  v = CommandData.pointing_mode.vel;

  //TODO: Update DoElScanMode with XSC Routines
  if (last_y!= CommandData.pointing_mode.Y || last_h != h) {
    if (el < bottom) {
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else if (el > top) {
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = top;
      axes_mode.el_vel = 0.0;
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else {
      // once we are within the new az/w range, we can mark this as 'last'.
      last_y = CommandData.pointing_mode.Y;
      last_h = h;
      SetElScanMode(el, bottom, top, v, 0);
    }
  } else {
    SetElScanMode(el, bottom, top, v, 0);
  }
}

#define EL_BORDER 1.0
#define AZ_BORDER 1.0
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
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
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
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
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

  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  SetSafeDAz(az, &caz);

  axes_mode.az_mode = AXIS_POSITION;
  axes_mode.az_dest = caz;
  axes_mode.az_vel = 0.0;
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = cel;
  axes_mode.el_vel = 0.0;
  //TODO:Update DoRADecGotoMode with XSC
//  isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
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
  int new_scan = 0;

  static double last_X=0, last_Y=0, last_w=0;
  static double v_el = 0;
  static double targ_el=0.0;

  // Stuff for the elevation offset/hwpr trigger
  static int el_dir_last = 0; 
  static int n_scan = 0;
  static int el_next_dir = 0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

  /* get raster center and sky drift speed */
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);

  /* add in elevation dither */
  cel += axes_mode.el_dith;
  el2 += axes_mode.el_dith;

  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  SetSafeDAz(az, &caz);

  r = CommandData.pointing_mode.w;
  bottom = cel - r;
  top = cel + r;


  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (last_mode != P_CAP)) {
    InitElDither(); 
    if ( (fabs(az - (caz)) < 0.1) &&
        (fabs(el - (bottom)) < 0.05)) {
      last_X = CommandData.pointing_mode.X;
      last_Y = CommandData.pointing_mode.Y;
      last_w = CommandData.pointing_mode.w;
      n_scan = 0;
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
      el_next_dir = 1;
      //TODO:Update DoNewCapMode with XSC Routine
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
  if (xw < MIN_SCAN*0.5)
    xw = MIN_SCAN*0.5;
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
  if (xw < MIN_SCAN*0.5)
    xw = MIN_SCAN*0.5;
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
      t = az_distance/v_az + 2.0*v_az/(az_accel * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/(az_accel * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el - (el-cel))/t;
    // set targ_el for the next step
    //    bprintf(info,"Az Step:targ_el = %f, el = %f, cel = %f,el-cel = %f, el_next_dir = %i,axes_mode.el_dir=%i,  v_el (target)= %f",targ_el,el,cel,el-cel,el_next_dir,axes_mode.el_dir,v_el);
    targ_el += CommandData.pointing_mode.del*el_next_dir;
    axes_mode.el_dir = el_next_dir;
    //    bprintf(info,"Az Step: Next Step targ_el = %f",targ_el);
    if (targ_el>=r) {
      targ_el = r;
      el_next_dir=-1;
      bprintf(info,"Approaching the top: next targ_el = %f, r = %f,el_next_dir = %i,axes_mode.el_dir=%i, v_el = %f",targ_el,r,el_next_dir,axes_mode.el_dir,v_el);
    } else if (targ_el<=-r) {
      targ_el = -r;
      el_next_dir=1;
      bprintf(info,"Approaching the bottom: next targ_el = %f, -r = %f,el_next_dir = %i,axes_mode.el_dir=%i, v_el = %f",targ_el,(-1.0)*r,el_next_dir,axes_mode.el_dir,v_el);
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

  if ( ((axes_mode.el_dir - el_dir_last)== 2) && 
       (CommandData.pointing_mode.nw == 0) ) {
    n_scan +=1;
    new_scan = 1;

    bprintf(info,"DoNewCapMode: Sending signal to rotate HWPR. n_scan = %i",n_scan);
    
    /* Set flags to rotate the HWPR */
    CommandData.hwpr.mode = HWPR_STEP;
    CommandData.hwpr.is_new = HWPR_STEP;

    if(n_scan % 4 == 0 && n_scan != 0) {
      GetElDither(DITH_INC);
      bprintf(info,"We're dithering! El Dither = %f", axes_mode.el_dith);
    }

  }

  el_dir_last = axes_mode.el_dir;

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

static void DoElBoxMode(void)
{
  double caz, cel, w, h;
  double bottom, top, left, right;
  double az, az2, el, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_el, t=1;
  int i_point;
  int new_step = 0;
  int new = 0;
  int new_scan = 0;
  int turn_az = 0;
  static int j = 0;

  static double last_X=0, last_Y=0, last_w=0, last_h = 0;
  static double v_az = 0;
  static double targ_az=0.0;

  // Stuff for hwpr rotation triggering
  static int az_dir_last = 0; 
  static int n_scan = 0;
  static int az_next_dir = 0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_el = fabs(CommandData.pointing_mode.vel);

  /* get raster center and sky drift speed */
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);
  /* sky drift terms */
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
  new = 0;

  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (CommandData.pointing_mode.h != last_h) ||
      (last_mode != P_EL_BOX)) {
    InitElDither();
    new = 1;
  }
  if (el < bottom - 0.5) new = 1;
  if (el > top + 0.5) new = 1;
  if (az < left - 2.0) new = 1;
  if (az > right + 2.0) new = 1;

  /* If a new command, reset to bottom row */
  if (new) {
    n_scan = 0;
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
      v_az = 0.0;
      targ_az = -w*0.5;
      az_next_dir = 1;
      //TODO:Update ElBoxMode with XSC Routine
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }
  /* set az v */

  v_el = CommandData.pointing_mode.vel;
  SetElScanMode(el, bottom, top, v_el, del_dt);

  /** set Az V **/
  new_step = 0;
  if (el<bottom) {
    if (axes_mode.el_dir < 0) {
      t = h/v_el + 2.0*v_el/(EL_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.el_dir = 1;
  } else if (el>top) {
    if (axes_mode.el_dir > 0) {
      t = h/v_el + 2.0*v_el/(EL_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.el_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_az = (targ_az - (az-caz))/t;
    // set targ_az for the next step
    targ_az += CommandData.pointing_mode.daz*az_next_dir; // This is actually the next target az....
    axes_mode.az_dir = az_next_dir;
    if (targ_az>w*0.5) { // If the target az for the next step is outside the az box range
      targ_az = w*0.5;
      az_next_dir=-1;
      bprintf(info,"Approaching the top %i: next targ_az = %f, h*0.5 = %f, az_next_dir = %i,axes_mode.az_dir=%i,  v_az = %f",j,targ_az,h*0.5,az_next_dir,axes_mode.az_dir,v_az);
    } else if (targ_az<-w*0.5) {
      targ_az = -w*0.5;
      az_next_dir = 1;
      bprintf(info,"Approaching the bottom %i: next targ_az = %f, h*0.5 = %f,az_next_dir = %i,axes_mode.az_dir=%i, v_az = %f",j,targ_az,h*0.5,az_next_dir,axes_mode.az_dir,v_az);
    }
  }
  /* check for out of range in az */
  if (az > right + AZ_BORDER) {
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = cel;
    axes_mode.el_vel = 0.0;
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_vel = 0.0;
    bprintf(info,"%i: az_vel=%f",j,axes_mode.az_vel);

    axes_mode.az_dest = left;
    axes_mode.az_dir = -1;
    if (v_az > 0) {
      v_az = -v_az;
    }
    return;
  } else if (az < left - AZ_BORDER) {
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = cel;
    axes_mode.el_vel = 0.0;
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_vel = 0.0;
    axes_mode.az_dest = bottom;
    axes_mode.az_dir = 1;
    if (v_az < 0) {
      v_az = -v_az;
    }
    return;
  }

  if ( ((axes_mode.az_dir - az_dir_last)== 2) &&
       (CommandData.pointing_mode.nw == 0) ) {

    n_scan +=1;
    new_scan = 1;
    bprintf(info,"DoElBoxMode: Sending signal to rotate HWPR. n_scan = %i",n_scan);

    /* Set flags to rotate the HWPR */
    CommandData.hwpr.mode = HWPR_STEP;
    CommandData.hwpr.is_new = HWPR_STEP;

    //    if(n_scan % 4 == 0 && n_scan != 0) {
    //      GetElDither(DITH_INC);
    //      bprintf(info,"We're dithering! El Dither = %f", axes_mode.el_dith);
    //    }

  }

  az_dir_last = axes_mode.az_dir;

  if(!turn_az) {
    axes_mode.az_mode = AXIS_VEL;
    axes_mode.az_vel = v_az + daz_dt;
  }

  j++;

  return;

}
#define JJLIM 100
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
  int new_scan = 0;
  int turn_el = 0;
  static int j = 0;
  static double last_X=0, last_Y=0, last_w=0, last_h = 0;
  static double v_el = 0;
  static double targ_el=0.0;

  // Stuff for the elevation offset/hwpr trigger
  static int el_dir_last = 0; 
  static int n_scan = 0;
  static int el_next_dir = 0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

  /* get raster center and sky drift speed */
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);

  /* add the elevation dither term */
  cel += axes_mode.el_dith;
  el2 += axes_mode.el_dith;

  /* sky drift terms */
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  SetSafeDAz(az, &caz);

  w = CommandData.pointing_mode.w/cos(el * M_PI / 180.0);
  h = CommandData.pointing_mode.h;
  bottom = cel - h*0.5;
  top = cel + h*0.5;
  left = caz - w*0.5;
  right = caz + w*0.5;
  j++;

  if (top > MAX_EL)
    top = MAX_EL;
  if (bottom < MIN_EL)
    bottom = MIN_EL;

  //  if (j%JJLIM == 0) bprintf(info,"cel =%f, el = %f,axes_mode.el_dith = %f, w=%f, h=%f, bottom = %f, top = %f, left = %f, right = %f",cel, el,axes_mode.el_dith, w, h, bottom , top, left, right);

  new = 0;

  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (CommandData.pointing_mode.h != last_h) ||
      (last_mode != P_BOX)) {
    new = 1;
    InitElDither();
  }
  if (el < bottom - 0.5) new = 1;
  if (el > top + 0.5) new = 1;
  if (az < left - 2.0) new = 1;
  if (az > right + 2.0) new = 1;

  /* If a new command, reset to bottom row */
  if (new) {
    n_scan = 0;
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
      el_next_dir = 1;
      //TODO:Update DoNewBoxMode with XSC Routine
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
      t = w/v_az + 2.0*v_az/(az_accel * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      t = w/v_az + 2.0*v_az/(az_accel * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el - (el-cel))/t;
    // set targ_el for the next step
    //    bprintf(info,"Az Step:targ_el = %f, el = %f, cel = %f,el-cel = %f, el_next_dir = %i,axes_mode.el_dir=%i,  v_el (target)= %f",targ_el,el,cel,el-cel,el_next_dir,axes_mode.el_dir,v_el);
    targ_el += CommandData.pointing_mode.del*el_next_dir; // This is actually the next target el....
    //    bprintf(info,"Az Step: Next Step targ_el = %f",targ_el);
    axes_mode.el_dir = el_next_dir;
    if (targ_el>h*0.5) { // If the target el for the next step is outside the el box range
      targ_el = h*0.5;
      el_next_dir=-1;
      bprintf(info,"Approaching the top: next targ_el = %f, h*0.5 = %f, el_next_dir = %i,axes_mode.el_dir=%i,  v_el = %f",targ_el,h*0.5,el_next_dir,axes_mode.el_dir,v_el);
    } else if (targ_el<-h*0.5) {
      targ_el = -h*0.5;
      el_next_dir = 1;
      bprintf(info,"Approaching the bottom: next targ_el = %f, h*0.5 = %f,el_next_dir = %i,axes_mode.el_dir=%i, v_el = %f",targ_el,h*0.5,el_next_dir,axes_mode.el_dir,v_el);
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

  if ( ((axes_mode.el_dir - el_dir_last)== 2) &&
       (CommandData.pointing_mode.nw == 0) ) {

    n_scan +=1;
    new_scan = 1;
    bprintf(info,"DoNewBoxMode: Sending signal to rotate HWPR. n_scan = %i",n_scan);

    /* Set flags to rotate the HWPR */
    CommandData.hwpr.mode = HWPR_STEP;
    CommandData.hwpr.is_new = HWPR_STEP;

    if(n_scan % 4 == 0 && n_scan != 0) {
      GetElDither(DITH_INC);
      bprintf(info,"We're dithering! El Dither = %f", axes_mode.el_dith);
    }

  }

  el_dir_last = axes_mode.el_dir;

  if(!turn_el) {
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = v_el + del_dt;
  }
  j++;
  return;
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
  int new_scan = 0;
 
  //int i_top, i_bot, new;

  static double last_ra[4] = {0,0,0,0}, last_dec[4] = {0,0,0,0};
  static double v_el = 0;
  static double targ_el=0.0; // targ_el is in degrees from bottom

  // Stuff for the elevation offset/hwpr trigger
  static int el_dir_last = 0; 
  static int n_scan = 0;
  static int el_next_dir = 0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  /* convert ra/decs to az/el */
  for (i=0; i<4; i++) {
      equatorial_to_horizontal(CommandData.pointing_mode.ra[i],
        CommandData.pointing_mode.dec[i],
        lst, lat,
        c_az+i, c_el+i);
    *(c_el+i) += axes_mode.el_dith;
  }

  /* get sky drift speed */
  equatorial_to_horizontal(CommandData.pointing_mode.ra[0],
      CommandData.pointing_mode.dec[0],
      lst+1.0, lat,
      &az2, &el2);

  el2 += axes_mode.el_dith;

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
  if (last_mode != P_QUAD) {
    new = 1;
    v_el = 0.0;
    targ_el = 0.0;
    el_next_dir = 1;
  }
    
  if (el < bottom - 1.0) new = 1;
  if (el > top + 1.0) new = 1;
  if (az < left - 6.0) new = 1;
  if (az > right + 6.0) new = 1;

  for (i=0; i<4; i++) {
    if (CommandData.pointing_mode.ra[i] != last_ra[i]) new = 1;
    if (CommandData.pointing_mode.dec[i] != last_dec[i]) new = 1;
  }

  if (new) {
    InitElDither();
    n_scan = 0;
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
      el_next_dir = 1;
      //TODO:Update DoQuadMode with XSC Routine
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
  if ((axes_mode.az_vel < 0) && (axes_mode.az_dir>=0)) { // turn around
    axes_mode.az_dir = -1;
    az_distance = az - next_left;
    if (az_distance<MIN_SCAN) az_distance = MIN_SCAN;
    t = az_distance/v_az + 2.0*v_az/(az_accel * SR);
    new_step = 1;
  } else if ((axes_mode.az_vel > 0) && (axes_mode.az_dir<=0)) { // turn around
    axes_mode.az_dir = 1;
    az_distance = next_right - az;
    if (az_distance<MIN_SCAN) az_distance = MIN_SCAN;
    t = az_distance/v_az + 2.0*v_az/(az_accel * SR);
    new_step = 1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el+bottom - el)/t;
    // set targ_el for the next step
    //    bprintf(info,"Az Step:targ_el = %f, bottom = %f, el = %f,el-bottom = %f, el_next_dir = %i,axes_mode.el_dir=%i,  v_el (target)= %f",targ_el,bottom,el,el-bottom,el_next_dir,axes_mode.el_dir,v_el);
    targ_el += CommandData.pointing_mode.del*el_next_dir;
    axes_mode.el_dir = el_next_dir;
    if (targ_el>top-bottom) {
      targ_el = top-bottom;
      el_next_dir=-1; 
      bprintf(info,"Approaching the top: next targ_el = %f, top-bottom = %f, el_next_dir = %i,axes_mode.el_dir=%i,  v_el = %f",targ_el,top-bottom,el_next_dir,axes_mode.el_dir,v_el);
    } else if (targ_el<0) {
      targ_el = 0;
      el_next_dir = 1;
      bprintf(info,"Approaching the bottom: next targ_el = %f, top-bottom = %f, el_next_dir = %i,axes_mode.el_dir=%i,  v_el = %f",targ_el,top-bottom,el_next_dir,axes_mode.el_dir,v_el);
    }
  }

  if ( ((axes_mode.el_dir - el_dir_last)== 2) &&
       (CommandData.pointing_mode.nw == 0) ) {

    n_scan +=1;
    new_scan = 1;
    bprintf(info,"DoNewQuadMode: Sending signal to rotate HWPR. n_scan = %i",n_scan);

    /* Set flags to rotate the HWPR */
    CommandData.hwpr.mode = HWPR_STEP;
    CommandData.hwpr.is_new = HWPR_STEP;

    if(n_scan % 4 == 0 && n_scan != 0) {
      GetElDither(DITH_INC);
      bprintf(info,"We're dithering! El Dither = %f", axes_mode.el_dith);
    }

  }

  el_dir_last = axes_mode.el_dir;

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
  az_accel = CommandData.az_accel/SR;
  switch (CommandData.pointing_mode.mode) {
    case P_DRIFT:
      axes_mode.el_mode = AXIS_VEL;
      axes_mode.el_vel = CommandData.pointing_mode.del;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = CommandData.pointing_mode.vaz;
      //Todo:Update UpdateAxesMode with XSC
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast =
//        (sqrt(CommandData.pointing_mode.vaz * CommandData.pointing_mode.vaz
//              + CommandData.pointing_mode.del * CommandData.pointing_mode.del)
//         > MAX_ISC_SLOW_PULSE_SPEED) ? 1 : 0;
      break;
    case P_AZEL_GOTO:
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = CommandData.pointing_mode.X;
      axes_mode.az_vel = 0.0;
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
    case P_AZ_SCAN:
      DoAzScanMode();
      break;
    case P_EL_SCAN:
      DoElScanMode();
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
    case P_EL_BOX:
      DoElBoxMode();
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
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
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
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
  }
  last_mode = CommandData.pointing_mode.mode;
}

// Only prints if verb_level_req >= verb_level_comp
void bprintfverb(buos_t l, unsigned short int verb_level_req, unsigned short int verb_level_comp, const char* fmt, ...) {
  char message[BUOS_MAX];
  va_list argptr;

  //  bprintf(info,"DEBUG: verb_level_req = %i, verb_level_comp = %i",verb_level_req,verb_level_comp);
  if(verb_level_req >= verb_level_comp) {
    va_start(argptr, fmt);
    vsnprintf(message, BUOS_MAX, fmt, argptr);
    va_end(argptr);   

    bputs(l,message);
  }                                                                                                    
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

//motor control parameters
#define MOTORSR 200.0
#define INTEGRAL_LENGTH  5.0  //length of the integral time constant in seconds
#define INTEGRAL_CUTOFF (1.0/(INTEGRAL_LENGTH*MOTORSR))

static int16_t calculate_el_current(float m_vreq_el)
{
    static float el_integral = 0.0;
    float p_el = 0.0, i_el = 0.0;       //control loop gains
    float error_el = 0.0, P_term_el = 0.0, I_term_el = 0.0; //intermediate control loop results
    int16_t dac_out;

    //el gyros measure -el
    error_el = ACSData.ifel_gy + m_vreq_el;

    P_term_el = p_el*error_el;


    if( (p_el == 0.0) || (i_el == 0.0) ) {
        el_integral = 0.0;
    } else {
        el_integral = (1.0 - INTEGRAL_CUTOFF)*el_integral + INTEGRAL_CUTOFF*error_el;
    }

    I_term_el = el_integral * p_el * i_el;
    if (I_term_el > 32767.0) {
        I_term_el = 32767.0;
        el_integral = el_integral *0.9;
    }
    if (I_term_el < -32767.0) {
        I_term_el = -32767.0;
        el_integral = el_integral * 0.9;
    }

    //sign difference in controller requires using -(P_term + I_term)
    dac_out =-(P_term_el + I_term_el);

    if (dac_out > INT16_MAX) dac_out = INT16_MAX;
    if (dac_out < INT16_MIN) dac_out = INT16_MIN;
    return dac_out;
}

static int16_t calculate_rw_current(float v_req_az)
{
    static float az_integral = 0.0;
    double cos_el, sin_el;
    float p_az = 0.0, i_az = 0.0;       //control loop gains
    float error_az = 0.0, P_term_az = 0.0, I_term_az = 0.0; //intermediate control loop results
    int16_t dac_out;

    int i_point = GETREADINDEX(point_index);

    sincos(from_degrees(PointingData[i_point].el), &sin_el, &cos_el);

    //roll, yaw contributions to az both -'ve (?)
    error_az = (ACSData.ifroll_gy * sin_el + ACSData.ifyaw_gy * cos_el) + v_req_az;

    P_term_az = p_az * error_az;

    if ((p_az == 0.0) || (i_az == 0.0)) {
        az_integral = 0.0;
    }
    else {
        az_integral = (1.0 - INTEGRAL_CUTOFF) * az_integral + INTEGRAL_CUTOFF * error_az;
    }

    I_term_az = az_integral * p_az * i_az;
    if (I_term_az > 32767.0) {
        I_term_az = 32767.0;
        az_integral = az_integral * 0.9;
    }
    if (I_term_az < -32767.0) {
        I_term_az = -32767.0;
        az_integral = az_integral * 0.9;
    }

    //TODO check sign of output
    dac_out = -(P_term_az + I_term_az);

    if (dac_out > INT16_MAX) dac_out = INT16_MAX;
    if (dac_out < INT16_MIN) dac_out = INT16_MIN;
    return dac_out;

}

void command_motors(void)
{
    static channel_t* velReqElAddr;
    static channel_t* velReqAzAddr;
    static channel_t* piv_currentcmd_addr;

    static channel_t* el_current_addr;
    static channel_t* rw_current_addr;
    static channel_t* piv_current_addr;

    float v_req_el = 0.0;
    float v_req_az = 0.0;

    /******** Obtain correct indexes the first time here ***********/
    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
      velReqElAddr = channels_find_by_name("vel_req_el");
      velReqAzAddr = channels_find_by_name("vel_req_az");
      piv_currentcmd_addr = channels_find_by_name("mc_piv_i_cmd");
    }
    /*******************************************************************\
    * Drive the Elevation motor                                         *
    \*******************************************************************/

    v_req_el = (float)(GET_UINT16(velReqElAddr)-32768.0)*(-0.0016276041666666666666666666666667);  // = vreq/614.4

    //TODO: limits in gyro units: revisit these
    if ((v_req_el < -15000.0) || (v_req_el > 15000.0))
        v_req_el = 0; // no really really crazy values!

    el_set_current(calculate_el_current(v_req_el));

    /*******************************************************************\
    * Drive the Reaction Wheel                                          *
    \*******************************************************************/

    //TODO: Move velReqAz to a floating point value
    v_req_az = (float)(GET_UINT16(velReqAzAddr)-32768.0)*0.0016276041666666666666666666666667;  // = vreq/614.4
    rw_set_current(calculate_rw_current(v_req_az));

    /*******************************************************************\
    * Drive the Pivot Motor                                             *
    \*******************************************************************/

    piv_set_current(GET_UINT16(piv_currentcmd_addr));

}

//void* reactComm(void* arg)
//{
//  //mark1
//  int n=0, j=0;
//  int i=0;
//  int temp_raw,curr_raw,stat_raw,faultreg_raw;
//  int firsttime=1,resetcount=0;
//  long vel_raw=0;
//  // Initialize values in the reactinfo structure.
//  reactinfo.open=0;
//  reactinfo.init=0;
//  reactinfo.err=0;
//  reactinfo.err_count=0;
//  reactinfo.closing=0;
//  reactinfo.reset=0;
//  reactinfo.disabled=2;
//  reactinfo.bdrate=9600;
//  reactinfo.writeset=0;
//  reactinfo.verbose=0;
//  strncpy(reactinfo.motorstr,"react",6);
//
//  nameThread("RWCom");
//
//  while(!InCharge) {
//    if(firsttime==1) {
//      bprintf(info,"I am not incharge thus I will not communicate with the RW motor.");
//      firsttime=0;
//    }
//    //in case we switch to ICC when serial communications aren't working
////    RWMotorData[0].vel_rw=ACSData.vel_rw;
////    RWMotorData[1].vel_rw=ACSData.vel_rw;
////    RWMotorData[2].vel_rw=ACSData.vel_rw;
//    usleep(20000);
//  }
//
//  firsttime=1;
//  bprintf(info,"Bringing the reaction wheel online.");
//  // Initialize structure RWMotorData.  Follows what was done in dgps.c
//  //  RWMotorData[0].vel_rw=0;
//  RWMotorData[0].temp=0;
//  RWMotorData[0].current=0.0;
//  RWMotorData[0].status=0;
//  RWMotorData[0].fault_reg=0;
//  RWMotorData[0].drive_info=0;
//  RWMotorData[0].err_count=0;
//
//  // Try to open the port.
//  while (reactinfo.open==0) {
//    reactinfo.verbose=CommandData.verbose_rw;
//    open_copley(REACT_DEVICE,&reactinfo); // sets reactinfo.open=1 if sucessful
//
//    if (i==10) bputs(err,"Reaction wheel port could not be opened after 10 attempts.\n");
//
//    i++;
//    if (reactinfo.open==1) {
//      bprintfverb(info,reactinfo.verbose,MC_VERBOSE,"Opened the serial port on attempt number %i",i);
//    } else sleep(1);
//  }
//
//  // Configure the serial port.  If after 10 attempts the port is not initialized it enters
//  // the main loop where it will trigger a reset command.
//  i=0;
//  while (reactinfo.init==0 && i <=9) {
//    reactinfo.verbose=CommandData.verbose_rw;
//    configure_copley(&reactinfo);
//    if (reactinfo.init==1) {
//      bprintf(info,"Initialized the controller on attempt number %i",i);
//    } else if (i==9) {
//      bprintf(info,"Could not initialize the controller after %i attempts.",i);
//    } else {
//      sleep(1);
//    }
//    i++;
//  }
//  rw_motor_index = 1; // index for writing to the RWMotor data struct
//  while (1){
//    reactinfo.verbose=CommandData.verbose_rw;
//    if((reactinfo.err & COP_ERR_MASK) > 0 ) {
//      reactinfo.err_count+=1;
//      if(reactinfo.err_count >= COPLEY_ERR_TIMEOUT) {
//	reactinfo.reset=1;
//      }
//    }
//    if(CommandData.reset_rw==1 ) {
//      reactinfo.reset=1;
//      CommandData.reset_rw=0;
//    }
//
//    RWMotorData[rw_motor_index].drive_info=makeMotorField(&reactinfo); // Make bitfield of controller info structure.
//    RWMotorData[rw_motor_index].err_count=(reactinfo.err_count > 65535) ? 65535: reactinfo.err_count;
//
//    // If we are still in the start up veto make sure the drive is disabled.
//    if(StartupVeto > 0) {
//      CommandData.disable_az=1;
//    }
//
//    if(reactinfo.closing==1){
//      rw_motor_index=INC_INDEX(rw_motor_index);
//      close_copley(&reactinfo);
//      usleep(10000);
//    } else if (reactinfo.reset==1){
//      if(resetcount==0) {
//	bprintf(warning,"Resetting connection to Reaction Wheel controller.");
//      } else if ((resetcount % 10)==0) {
//	//	bprintf(warning,"reset-> Unable to connect to Reaction Wheel after %i attempts.",resetcount);
//      }
//
//      resetcount++;
//      rw_motor_index=INC_INDEX(rw_motor_index);
//      resetCopley(REACT_DEVICE,&reactinfo); // if successful sets reactinfo.reset=0
//      usleep(10000);  // give time for motor bits to get written
//      if (reactinfo.reset==0) {
//	resetcount=0;
//        bprintf(info,"Controller successfuly reset!");
//      }
//
//    } else if(reactinfo.init==1){
//      if(CommandData.disable_az==0 && reactinfo.disabled > 0) {
//	bprintfverb(info,reactinfo.verbose,MC_VERBOSE,"Attempting to enable the reaction wheel motor controller.");
//	n=enableCopley(&reactinfo);
//	if(n==0){
//	  bprintf(info,"Reaction wheel motor controller is now enabled.");
//	  reactinfo.disabled=0;
//	}
//      }
//      if(CommandData.disable_az==1 && (reactinfo.disabled==0 || reactinfo.disabled==2)) {
//	bprintfverb(info,reactinfo.verbose,MC_VERBOSE,"Attempting to disable the reaction wheel motor controller.");
//	n=disableCopley(&reactinfo);
//	if(n==0){
//	  bprintf(info,"Reaction wheel motor controller is now disabled.");
//	  reactinfo.disabled=1;
//	}
//      }
//
//      vel_raw=queryCopleyInd(COP_IND_VEL,&reactinfo); // Units are 0.1 counts/sec
//      RWMotorData[rw_motor_index].vel_rw=((double) vel_raw)/RW_ENC_CTS/10.0*360.0;
//      j=j%4;
//      switch(j) {
//      case 0:
//	temp_raw=queryCopleyInd(COP_IND_TEMP,&reactinfo);
//        RWMotorData[rw_motor_index].temp=temp_raw; // units are deg Cel
//	break;
//      case 1:
//	curr_raw=queryCopleyInd(COP_IND_CURRENT,&reactinfo);
//        RWMotorData[rw_motor_index].current=((double) (curr_raw))/100.0; // units are Amps
//	break;
//      case 2:
//	stat_raw=queryCopleyInd(COP_IND_STATUS,&reactinfo);
//        RWMotorData[rw_motor_index].status=stat_raw;
//	break;
//      case 3:
//	faultreg_raw=queryCopleyInd(COP_IND_FAULTREG,&reactinfo);
//        RWMotorData[rw_motor_index].fault_reg=faultreg_raw;
//	break;
//      }
//      j++;
//      if (firsttime) {
//	bprintfverb(info,reactinfo.verbose,MC_VERBOSE,"Raw reaction wheel velocity is %i",vel_raw);
//	firsttime=0;
//      }
//      rw_motor_index=INC_INDEX(rw_motor_index);
//
//    } else {
//      rw_motor_index=INC_INDEX(rw_motor_index);
//      reactinfo.reset=1;
//      usleep(10000);
//    }
//    i++;
//  }
//  return NULL;
//}
//
//
//void* elevComm(void* arg)
//{
//
//  int n=0, j=0;
//  int i=0;
//  long unsigned pos_raw;
//  int temp_raw,curr_raw,stat_raw,faultreg_raw;
//  int firsttime=1,resetcount=0;
//
//
//  // Initialize values in the elevinfo structure.
//  elevinfo.open=0;
//  elevinfo.init=0;
//  elevinfo.err=0;
//  elevinfo.err_count=0;
//  elevinfo.closing=0;
//  elevinfo.reset=0;
//  elevinfo.disabled=2;
//  elevinfo.bdrate=9600;
//  elevinfo.writeset=0;
//  strncpy(elevinfo.motorstr,"elev\0",6);
//  elevinfo.verbose=1;
//
//  nameThread("ElCom");
//
//  while(!InCharge) {
//    if(firsttime==1) {
//      bprintf(info,"I am not incharge thus I will not communicate with the elevation drive.");
//      firsttime=0;
//    }
//
//    //in case we switch to ICC when serial communications aren't working
////    ElevMotorData[0].enc_raw_el=ACSData.enc_raw_el;
////    ElevMotorData[1].enc_raw_el=ACSData.enc_raw_el;
////    ElevMotorData[2].enc_raw_el=ACSData.enc_raw_el;
//    usleep(20000);
//  }
//
//  bprintf(info,"Bringing the elevation drive online.");
//  i=0;
//  firsttime=1;
//
//  // Initialize structure ElevMotorData.  Follows what was done in dgps.c
//  //  ElevMotorData[0].enc_raw_el=0;
//  ElevMotorData[0].temp=0;
//  ElevMotorData[0].current=0.0;
//  ElevMotorData[0].status=0;
//  ElevMotorData[0].fault_reg=0;
//  ElevMotorData[0].drive_info=0;
//  ElevMotorData[0].err_count=0;
//
//  // Try to open the port.
//  while(elevinfo.open==0) {
//    elevinfo.verbose=CommandData.verbose_el;
//    open_copley(ELEV_DEVICE,&elevinfo); // sets elevinfo.open=1 if sucessful
//
//    if(i==10) {
//      bputs(err,"Elevation drive serial port could not be opened after 10 attempts.\n");
//    }
//    i++;
//
//    if(elevinfo.open==1) {
//	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Opened the serial port on attempt number %i",i);
//    } else {
//      sleep(1);
//    }
//  }
//
//  // Configure the serial port.  If after 10 attempts the port is not initialized it enters
//  // the main loop where it will trigger a reset command.
//  i=0;
//  while (elevinfo.init==0 && i <=9) {
//    elevinfo.verbose=CommandData.verbose_el;
//    configure_copley(&elevinfo);
//    if(elevinfo.init==1) {
//      bprintf(info,"Initialized the controller on attempt number %i",i);
//    } else if (i==9) {
//      bprintf(info,"Could not initialize the controller after %i attempts.",i);
//    } else {
//      sleep(1);
//    }
//    i++;
//  }
//
//  elev_motor_index = 1; // index for writing to the ElevMotor data struct
//  while (1) {
//    elevinfo.verbose=CommandData.verbose_el;
//    if ((elevinfo.err & COP_ERR_MASK) > 0 ) {
//      elevinfo.err_count+=1;
//      if (elevinfo.err_count >= COPLEY_ERR_TIMEOUT) {
//	elevinfo.reset=1;
//      }
//    }
//
//    if (CommandData.reset_elev==1 ) {
//      elevinfo.reset=1;
//      CommandData.reset_elev=0;
//    }
//
//    ElevMotorData[elev_motor_index].drive_info=makeMotorField(&elevinfo); // Make bitfield of controller info structure.
//    ElevMotorData[elev_motor_index].err_count=(elevinfo.err_count > 65535) ? 65535: elevinfo.err_count;
//
//    // If we are still in the start up veto make sure the drive is disabled.
//    if(StartupVeto > 0) {
//      CommandData.disable_el=1;
//    }
//
//    if(elevinfo.closing==1){
//      elev_motor_index=INC_INDEX(elev_motor_index);
//      close_copley(&elevinfo);
//      usleep(10000);
//    } else if (elevinfo.reset==1){
//      if(resetcount==0) {
//	bprintf(warning,"Resetting connection to elevation drive controller.");
//      } else if ((resetcount % 10)==0) {
//	//	bprintf(warning,"reset-> Unable to connect to elevation drive after %i attempts.",resetcount);
//      }
//
//      resetcount++;
//      elev_motor_index=INC_INDEX(elev_motor_index);
//      resetCopley(ELEV_DEVICE,&elevinfo); // if successful sets elevinfo.reset=0
//
//      usleep(10000);  // give time for motor bits to get written
//      if (elevinfo.reset==0) {
//	resetcount=0;
//        bprintf(info,"Controller successfuly reset!");
//      }
//
//    } else if (elevinfo.init==1) {
//      if((CommandData.disable_el==0 || CommandData.force_el==1 ) && elevinfo.disabled > 0) {
//	bprintf(info,"Attempting to enable the elevation motor controller.,CommandData.disable_el=%i,CommandData.force_el=%i,elevinfo.disabled=%i",CommandData.disable_el,CommandData.force_el,elevinfo.disabled);
//	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Attempting to enable the elevation motor controller.");
//	n=enableCopley(&elevinfo);
//	if(n==0){
//	  bprintf(info,"Elevation motor controller is now enabled.");
//	  elevinfo.disabled=0;
//	}
//      }
//      if((CommandData.disable_el==1 && CommandData.force_el==0 ) && (elevinfo.disabled==0 || elevinfo.disabled==2)) {
//	bprintf(info,"Attempting to enable the elevation motor controller.,CommandData.disable_el=%i,CommandData.force_el=%i,elevinfo.disabled=%i",CommandData.disable_el,CommandData.force_el,elevinfo.disabled);
//	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Attempting to disable the elevation motor controller.");
//	n=disableCopley(&elevinfo);
//	if(n==0){
//	  bprintf(info,"Elevation motor controller is now disabled.");
//	  elevinfo.disabled=1;
//	}
//      }
//
//      pos_raw=queryCopleyInd(COP_IND_POS,&elevinfo); // Units are counts
//                                                     // For Elev 524288 cts = 360 deg
//      ElevMotorData[elev_motor_index].enc_raw_el=((double) (pos_raw % ((long int) ELEV_ENC_CTS)))/ELEV_ENC_CTS*360.0-ENC_RAW_EL_OFFSET;
//      //   getCopleySlowInfo(j,elev_motor_index,&ElevMotorData,&elevinfo); // Reads one of temperature, current, status and fault register and
//                           // writes to the appropriate frame
//
//      if (firsttime) {
//	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Raw elevation encoder position is %i",pos_raw);
//	firsttime=0;
//      }
//      j=j%4;
//      switch(j) {
//      case 0:
//	temp_raw=queryCopleyInd(COP_IND_TEMP,&elevinfo);
//        ElevMotorData[elev_motor_index].temp=temp_raw; // units are deg Cel
//	break;
//      case 1:
//	curr_raw=queryCopleyInd(COP_IND_CURRENT,&elevinfo);
//        ElevMotorData[elev_motor_index].current=((double) (curr_raw))/100.0; // units are Amps
//	break;
//      case 2:
//	stat_raw=queryCopleyInd(COP_IND_STATUS,&elevinfo);
//        ElevMotorData[elev_motor_index].status=stat_raw; // units are Amps
//	break;
//      case 3:
//	faultreg_raw=queryCopleyInd(COP_IND_FAULTREG,&elevinfo);
//        ElevMotorData[elev_motor_index].fault_reg=faultreg_raw; // units are Amps
//	break;
//      }
//      j++;
//      elev_motor_index=INC_INDEX(elev_motor_index);
//    } else {
//      elevinfo.reset=1;
//      elev_motor_index=INC_INDEX(elev_motor_index);
//      usleep(10000);
//    }
//  }
//  return NULL;
//}
//
//void* pivotComm(void* arg)
//{
//  int n=0, j=0;
//  int i=0;
//  long unsigned pos_raw=0;
//  int firsttime=1,resetcount=0;
//  unsigned int dp_stat_raw=0, db_stat_raw=0, ds1_stat_raw=0;
//  short int current_raw=0;
//  int piv_vel_raw=0;
//  unsigned int tmp=0;
//  // Initialize values in the pivotinfo structure.
//  pivotinfo.open=0;
//  pivotinfo.init=0;
//  pivotinfo.err=0;
//  pivotinfo.err_count=0;
//  pivotinfo.closing=0;
//  pivotinfo.reset=0;
//  pivotinfo.disabled=2;
//  pivotinfo.bdrate=9600;
//  pivotinfo.writeset=0;
//  strncpy(pivotinfo.motorstr,"pivot",6);
//  pivotinfo.verbose=0;
//
//  nameThread("PivCom");
//
//  while (!InCharge) {
//    if (firsttime==1) {
//      bprintf(info,"I am not incharge thus I will not communicate with the pivot motor.");
//      firsttime=0;
//    }
//    usleep(20000);
//  }
//
//  bprintf(info,"Bringing the pivot drive online.");
//  firsttime=1;
//
//  i=0;
//
//  // Initialize structure PivotMotorData.  Follows what was done in dgps.c
//  PivotMotorData[0].res_piv=0;
//  PivotMotorData[0].current=0;
//  PivotMotorData[0].db_stat=0;
//  PivotMotorData[0].dp_stat=0;
//  PivotMotorData[0].ds1_stat=0;
//  PivotMotorData[0].dps_piv=0;
//  PivotMotorData[0].drive_info=0;
//  PivotMotorData[0].err_count=0;
//
//  // Try to open the port.
//  while (pivotinfo.open==0) {
//    pivotinfo.verbose=CommandData.verbose_piv;
//    open_amc(PIVOT_DEVICE,&pivotinfo); // sets pivotinfo.open=1 if sucessful
//
//    if (i==10) {
//      bputs(err,"Pivot controller serial port could not be opened after 10 attempts.\n");
//    }
//    i++;
//
//    if (pivotinfo.open==1) {
//      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Opened the serial port on attempt number %i",i);
//    }
//    else sleep(1);
//  }
//
//  // Configure the serial port.  If after 10 attempts the port is not initialized it enters
//  // the main loop where it will trigger a reset command.
//  i=0;
//  while (pivotinfo.init==0 && i <=9) {
//    pivotinfo.verbose=CommandData.verbose_piv;
//    configure_amc(&pivotinfo);
//    if (pivotinfo.init==1) {
//      bprintf(info,"Initialized the controller on attempt number %i",i);
//    } else if (i==9) {
//      bprintf(info,"Could not initialize the controller after %i attempts.",i);
//    } else {
//      sleep(1);
//    }
//    i++;
//  }
//
//  while (1) {
//    pivotinfo.verbose=CommandData.verbose_piv;
//    if((pivotinfo.err & AMC_ERR_MASK) > 0 ) {
//      pivotinfo.err_count+=1;
//      if(pivotinfo.err_count >= AMC_ERR_TIMEOUT) {
//	pivotinfo.reset=1;
//      }
//    }
//    if(CommandData.reset_piv==1 ) {
//      pivotinfo.reset=1;
//      CommandData.reset_piv=0;
//    }
//    if(CommandData.restore_piv==1 ) {
//      restoreAMC(&pivotinfo);
//      CommandData.restore_piv=0;
//    }
//
//    PivotMotorData[pivot_motor_index].drive_info=makeMotorField(&pivotinfo); // Make bitfield of controller info structure.
//    PivotMotorData[pivot_motor_index].err_count=(pivotinfo.err_count > 65535) ? 65535: pivotinfo.err_count;
//    // If we are still in the start up veto make sure the drive is disabled.
//    if(StartupVeto > 0) {
//      CommandData.disable_az=1;
//    }
//
//    if(pivotinfo.closing==1){
//      pivot_motor_index=INC_INDEX(pivot_motor_index);
//      close_amc(&pivotinfo);
//      usleep(10000);
//    } else if (pivotinfo.reset==1){
//      if(resetcount==0) {
//	bprintf(warning,"Resetting connection to pivot controller.");
//      } else if ((resetcount % 50)==0) {
//	bprintfverb(warning,pivotinfo.verbose,MC_VERBOSE,"reset->Unable to connect to pivot after %i attempts.",resetcount);
//      }
//
//      bprintfverb(warning,pivotinfo.verbose,MC_EXTRA_VERBOSE,"Attempting to reset the pivot controller.",resetcount);
//      resetcount++;
//      pivot_motor_index=INC_INDEX(pivot_motor_index);
//      resetAMC(PIVOT_DEVICE,&pivotinfo); // if successful sets pivotinfo.reset=0
//
//      if (pivotinfo.reset==0) {
//	resetcount=0;
//        bprintf(info,"Controller successfuly reset!");
//      }
//      usleep(10000);  // give time for motor bits to get written
//
//    } else if (pivotinfo.init==1) {
//      if(CommandData.disable_az==0 && pivotinfo.disabled == 1) {
//      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Attempting to enable the pivot motor contoller.");
//	n=enableAMC(&pivotinfo);
//	if(n==0) {
//	  bprintf(info,"Pivot motor is now enabled");
//	  pivotinfo.disabled=0;
//	}
//      }
//      if(CommandData.disable_az==1 && (pivotinfo.disabled==0 || pivotinfo.disabled==2)) {
//      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Attempting to disable the pivot motor controller.");
//	n=disableAMC(&pivotinfo);
//	if(n==0){
//	  bprintf(info,"Pivot motor controller is now disabled.");
//	  pivotinfo.disabled=1;
//	}
//      }
//
//      if(firsttime){
//	firsttime=0;
//	tmp = queryAMCInd(0x32,8,1,&pivotinfo);
//	bprintf(info,"Ki = %i",tmp);
//	tmp = queryAMCInd(0xd8,0x24,1,&pivotinfo);
//	bprintf(info,"Ks = %i",tmp);
//	tmp = queryAMCInd(0xd8,0x0c,1,&pivotinfo);
//	bprintf(info,"d8.0ch = %i",tmp);
//	tmp = queryAMCInd(216,12,1,&pivotinfo);
//	bprintf(info,"v2 d8.0ch = %i",tmp);
//	tmp = queryAMCInd(0xd8,0x12,1,&pivotinfo);
//	bprintf(info,"d8.12h = %i",tmp);
//	tmp = queryAMCInd(216,18,1,&pivotinfo);
//	bprintf(info,"v2 d8.12h = %i",tmp);
//	tmp = queryAMCInd(0xd8,0x13,1,&pivotinfo);
//	bprintf(info,"d8.13h = %i",tmp);
//	tmp = queryAMCInd(216,19,1,&pivotinfo);
//	bprintf(info,"v2 d8.13h = %i",tmp);
//      }
//
//      pos_raw=getAMCResolver(&pivotinfo);
//      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Resolver Position is: %i",pos_raw);
//      PivotMotorData[pivot_motor_index].res_piv=((double) pos_raw)/PIV_RES_CTS*360.0;
//
//      j=j%5;
//      switch(j) {
//      case 0:
//	current_raw=queryAMCInd(16,3,1,&pivotinfo);
//        PivotMotorData[pivot_motor_index].current=((double)current_raw)/8192.0*20.0; // *2^13 / peak drive current
//	                                                                             // Units are Amps
//	bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"current_raw= %i, current= %f",current_raw,PivotMotorData[pivot_motor_index].current);
//	break;
//      case 1:
//	db_stat_raw=queryAMCInd(2,0,1,&pivotinfo);
//        PivotMotorData[pivot_motor_index].db_stat=db_stat_raw;
//	bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"db_stat_raw= %i, db_stat= %f",db_stat_raw,PivotMotorData[pivot_motor_index].db_stat);
//	break;
//      case 2:
//	dp_stat_raw=queryAMCInd(2,1,1,&pivotinfo);
//        PivotMotorData[pivot_motor_index].dp_stat=dp_stat_raw;
//	bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"dp_stat_raw= %i, dp_stat= %f",dp_stat_raw,PivotMotorData[pivot_motor_index].dp_stat);
//	break;
//      case 3:
//	ds1_stat_raw=queryAMCInd(2,3,1,&pivotinfo);
//        PivotMotorData[pivot_motor_index].ds1_stat=ds1_stat_raw;
//	bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"ds1_stat_raw= %i, ds1_stat= %f",ds1_stat_raw,PivotMotorData[pivot_motor_index].ds1_stat);
//	break;
//      case 4:
//	piv_vel_raw=((int) queryAMCInd(17,2,2,&pivotinfo));
//        PivotMotorData[pivot_motor_index].dps_piv=piv_vel_raw*0.144;
//	bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"piv_vel_raw= %i, piv_vel= %f",piv_vel_raw,PivotMotorData[pivot_motor_index].dps_piv);
//	break;
//      }
//      j++;
//      pivot_motor_index=INC_INDEX(pivot_motor_index);
//    } else {
//      pivotinfo.reset=1;
//      pivot_motor_index=INC_INDEX(pivot_motor_index);
//      usleep(10000);
//    }
//  }
//  return NULL;
//}
