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

#include "share/channels.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"
#include "amccommand.h"
#include "motordefs.h"

#define MIN_EL 23.9 // JAS -- 10 for SPIDER
#define MAX_EL 55 // JAS -- 51 for SPIDER

#define VPIV_FILTER_LEN 40
#define FPIV_FILTER_LEN 1000

#define RW_BASE 0.95    // base for exponential filter used to compute RW
                        // speed

#define V_AZ_MIN 0.05 // JAS -- smallest measured az speed we trust given gyro
                      //        noise/offsets

void nameThread(const char*);	/* mcp.c */

struct RWMotorDataStruct RWMotorData[3]; // defined in point_struct.h
int rw_motor_index; 

//struct ElevMotorDataStruct ElevMotorData[3]; // defined in point_struct.h
//int elev_motor_index; JAS -- no elev motor for Spider

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
//static pthread_t elevcomm_id;
static pthread_t pivotcomm_id;

// device node address for the reaction wheel motor controller
#define REACT_DEVICE "/dev/ttySI9"
//#define ELEV_DEVICE "/dev/ttySI11"
#define PIVOT_DEVICE "/dev/ttySI13"

static void* reactComm(void *arg);
//static void* elevComm(void *arg);
static void* pivotComm(void *arg);

extern short int InCharge; /* tx.c */

extern int StartupVeto; /* mcp.c */

extern short int bsc_trigger; /* Semaphore for BSC trigger */
#define DELAY 3.685/SR*20 /* number of seconds between sending exposure command and pulse_bsc */

double az_accel = 0.1;

//TODO temporarily declare react and elev structs, formerly in compleycommand
//should replace with ones properly declared elsewhere
struct MotorInfoStruct reactinfo;
struct MotorInfoStruct elevinfo;

/* opens communications with motor controllers */
void openMotors()
{
  bprintf(info, "Motors: connecting to motors");
  pthread_create(&reactcomm_id, NULL, &reactComm, NULL);
//  pthread_create(&elevcomm_id, NULL, &elevComm, NULL); 
//  JAS -- no elevcomm for Spider
  pthread_create(&pivotcomm_id, NULL, &pivotComm, NULL);
}

void closeMotors()
{
  int i=0;
  reactinfo.closing=1;
//  elevinfo.closing=1; // Tell the serial threads to shut down.
  pivotinfo.closing=1;

//  while(reactinfo.open==1 && elevinfo.open==1 && pivotinfo.open==1 && i++<100) usleep(10000);
  while(reactinfo.open==1 && pivotinfo.open==1 && i++<100) usleep(10000);
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
    for(i=0;i<(VPIV_FILTER_LEN-1);i++) buf_vPiv[i]=0.0;
    for(i=0;i<(VPIV_FILTER_LEN-1);i++) buf_t[i]=0.0;
  }
  a+=(ACSData.res_piv-buf_vPiv[ib_last]);
  //  dt=((double)(gettimeofday-buf_t[ib_last]));
  buf_vPiv[ib_last]=ACSData.res_piv;
  //  dummy=PointingData[i_point].t
    //  buf_t[ib_last]=PointingData[i_point].t;
  ib_last=(ib_last+VPIV_FILTER_LEN+1)%VPIV_FILTER_LEN;
  dtheta=(a-alast)/VPIV_FILTER_LEN;
  alast=a;
  vpiv=dtheta/0.010016; 
  //  if (j%100 == 1) bprintf(info,"CalcVPiv vpiv = %f, res_piv = %f, a = %f, alast = %f, dtheta = %f ",vpiv,ACSData.res_piv,a/VPIV_FILTER_LEN,alast/VPIV_FILTER_LEN,dtheta);
  j++;
  return vpiv;
}

/* calcVRW(): compute filtered RW velocity from RW resolver position read 
              from AMC controller over RS-232 */
static double calcVRW(void)
{

//v -- velocity input to filter
//u -- velocity output from filter

  double v, u, x, dx; 
  static double last_u = 0.0, last_x = 0.0;
  static double last_v = 0.0;
  int i_rw;
  static int frame_count = 0;
  //static int since_last = 0;

  frame_count++;

  /*if (++since_last < 30) {
    return last_u;
  }
  since_last = 0;*/

  i_rw = GETREADINDEX(rw_motor_index);

  x = RWMotorData[i_rw].res_rw;

  dx = x - last_x;

  if ( dx > 180.0 ) {
    dx -= 360.0;
  } else if (dx < -180.0) {
    dx += 360.0;
  }
   
  if (dx==0.0) {
    if (frame_count>100) {
      v = 0.0;
    } else {
      v = last_v;  
    }
  } else {
    v = dx*(SR/((double)frame_count));
    frame_count = 0; 
  }  

  last_x = x;
  last_v = v;

  u = ( RW_BASE*last_u + (1-RW_BASE)*v );
  last_u = u;
//bprintf(info, "i_rw = %d, x = %f deg, v = %f dps, u = %f dps", i_rw, x, v, u);

  return u;

}

double calcVSerRW(void)
{

  double w, w_filt; // filtered/unfiltered ang. velocity respectively
  static double last_w_filt = 0.0;
  int i_rw;

  i_rw = GETREADINDEX(rw_motor_index);
  w = RWMotorData[i_rw].dps_rw;

  w_filt = ( RW_BASE*last_w_filt + (1-RW_BASE)*w );
  last_w_filt = w_filt;

  return w_filt;
  //return w;
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
    vel *= (double)CommandData.ele_gain.PT/10000.0;
    //    vel = (axes_mode.el_dest - PointingData[i_point].el) * 0.36;
  } else if (axes_mode.el_mode == AXIS_LOCK) {
    /* for the lock, only use the elevation encoder */
    vel = (axes_mode.el_dest - ACSData.enc_raw_el) * 0.64;
 }

  /* correct offset and convert to Gyro Units */
  vel -= (PointingData[i_point].offset_ifel_gy 
         - PointingData[i_point].ifel_earth_gy);

  if (CommandData.use_elenc) {
    el_for_limit = ACSData.enc_raw_el;
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
/*   Units are in gyro units (1 bit = 0.001 dps)                        */
/*   Note: was formerly 0.1*gyro units (BLAST-Pol)                      */
/************************************************************************/
static double GetVAz(void)
{
  double vel = 0.0;
  static double last_vel = 0.0;
  double dvel;
  int i_point;
  //double vel_offset;
  double az, az_dest;
  double t_bbus;
  double max_dv;// = 20;
  double dx;

  i_point = GETREADINDEX(point_index);
  
  t_bbus = 1.0/SR;
  max_dv = 1.05*(CommandData.az_accel_max)*t_bbus;
  max_dv *= DPS_TO_GY16;
  //max_dv = 1000;

  if (axes_mode.az_mode == AXIS_VEL) {
      vel = axes_mode.az_vel;
    //bprintf(info, "vel according to axes_mode: %f", vel);
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
    //bprintf(info, "az_dest: %f, az: %f, dx: %f, vel: %f", az_dest, az, dx, vel);
    vel *= (double)CommandData.azi_gain.PT/10000.0;
  }

  /*vel_offset = -(PointingData[i_point].offset_ifroll_gy 
               - PointingData[i_point].ifroll_earth_gy)
               * cos(PointingData[i_point].el * M_PI / 180.0) 
               -(PointingData[i_point].offset_ifyaw_gy 
               - PointingData[i_point].ifyaw_earth_gy)
               * sin(PointingData[i_point].el * M_PI / 180.0);*/

  /* gyros are on the outer frame for Spider...
 
     vel_offset =-(PointingData[i_point].offset_ifyaw_gy 
                  - PointingData[i_point].ifyaw_earth_gy)

    ...and we don't care about vel_offset anyway */

  //vel -= vel_offset;
  vel *= DPS_TO_GY16;
  //bprintf(info, "vel intermediate from GetVAz() (gyro units): %f", vel);
  /* limit maximum speed */
  /*if (vel > 2000.0)
    vel = 2000.0;
  if (vel < -2000.0)
    vel = -2000.0;*/

  /* limit maximum acceleration */
  dvel = vel - last_vel;
  
  if (dvel > max_dv) {
    vel = last_vel + max_dv;
  }

  if (dvel < -max_dv) {
    vel = last_vel - max_dv;
  }
  
  last_vel = vel;

//return (vel*10.0); // Factor of 10 was to increase dynamic range
  
  //bprintf(info, "vel output from GetVAz() (gyro units): %f", vel);
  return (vel);        // need larger speed range at expense of speed
                       // resolution for Spider.
}

/************************************************************************/
/*                                                                      */
/*     GetIPivot: get the current request for the pivot in DAC units    */
/*       Proportional to the reaction wheel speed error                 */
/*                                                                      */
/************************************************************************/
static double GetIPivot(int v_az_req_gy, unsigned int g_rw_piv, unsigned int g_err_piv, double frict_off_piv, unsigned int disabled)
{
  static struct NiosStruct* pRWTermPivAddr;
  static struct NiosStruct* pErrTermPivAddr;
  static struct NiosStruct* frictTermPivAddr;
  static struct NiosStruct* frictTermUnfiltPivAddr; // For debugging only.  Remove later!
  static double buf_frictPiv[FPIV_FILTER_LEN]; // Buffer for Piv friction term boxcar filter.
  static double a=0.0; 
  static unsigned int ib_last=0;
  double I_req = 0.0;
  int I_req_dac = 0;
  int I_req_dac_init = 0;
  int i_point;
  double v_az_req,i_frict,i_frict_filt;
  double p_rw_term, p_err_term;
  int p_rw_term_dac, p_err_term_dac;
 
  static int i=0;
  static unsigned int firsttime = 1;

  if(firsttime) {
    pRWTermPivAddr = GetNiosAddr("p_rw_term_piv");
    pErrTermPivAddr = GetNiosAddr("p_err_term_piv");
    frictTermPivAddr = GetNiosAddr("frict_term_piv");
    frictTermUnfiltPivAddr = GetNiosAddr("frict_term_uf_piv");
    // Initialize the buffer.  Assume all zeros to begin
    for(i=0;i<(FPIV_FILTER_LEN-1);i++) buf_frictPiv[i]=0.0;
    firsttime = 0;
  }

  //v_az_req = ((double) v_az_req_gy) * GY16_TO_DPS/10.0; // Convert to dps 
  v_az_req = ((double) v_az_req_gy) * GY16_TO_DPS; // Convert to dps 

  i_point = GETREADINDEX(point_index);
  p_rw_term = (-1.0)*((double)g_rw_piv/10.0)*(ACSData.vel_rw-CommandData.pivot_gain.SP);
  /*if (ACSData.vel_rw != 0.0) {
    bprintf(info, "measured rw vel (read from frame): %f", ACSData.vel_rw);
  }*/
  p_err_term = (double)g_err_piv*(v_az_req-PointingData[point_index].v_az);
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
  I_req_dac_init=I_req_dac;

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

  WriteData(pRWTermPivAddr,p_rw_term,NIOS_QUEUE);
  WriteData(pErrTermPivAddr,p_err_term,NIOS_QUEUE);
  WriteData(frictTermPivAddr,i_frict_filt*32767.0/2.0,NIOS_QUEUE);
  WriteData(frictTermUnfiltPivAddr,i_frict*32767.0/2.0,NIOS_QUEUE);
  return I_req_dac;
}

/************************************************************************/
/*                                                                      */
/*    WriteMot: motors, and, for convenience, the inner frame lock      */
/*                                                                      */
/************************************************************************/
void WriteMot(int TxIndex)
{
  static struct NiosStruct* velReqElAddr;
  static struct NiosStruct* velReqAzAddr;
  static struct NiosStruct* cosElAddr;
  static struct NiosStruct* sinElAddr;

  static struct NiosStruct* gPElAddr;
  static struct NiosStruct* gIElAddr;
  static struct NiosStruct* gPtElAddr;
  static struct NiosStruct* gPAzAddr;
  static struct NiosStruct* gIAzAddr;
  static struct NiosStruct* gPtAzAddr;
  static struct NiosStruct* gPVPivAddr;
  static struct NiosStruct* gPEPivAddr;
  static struct NiosStruct* setRWAddr;
  static struct NiosStruct* frictOffPivAddr;
  static struct NiosStruct* dacPivAddr;
  static struct NiosStruct* velCalcPivAddr;
  static struct NiosStruct* velRWAddr;
  static struct NiosStruct* accelAzAddr;
 
  // Used only for Lab Controller tests
  static struct NiosStruct* dacAmplAddr[5];
  int i;
  static int wait = 100; /* wait 20 frames before controlling. */
  double el_rad;
  unsigned int ucos_el;
  unsigned int usin_el;

  int v_elev, v_az, i_piv, elGainP, elGainI;
  double v_rw;
  int azGainP, azGainI, pivGainRW, pivGainErr;
  double pivFrictOff;
  int i_point;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    velReqElAddr = GetNiosAddr("vel_req_el");
    velReqAzAddr = GetNiosAddr("vel_req_az");
    cosElAddr = GetNiosAddr("cos_el");
    sinElAddr = GetNiosAddr("sin_el");
    dacPivAddr = GetNiosAddr("dac_piv");
    gPElAddr = GetNiosAddr("g_p_el");
    gIElAddr = GetNiosAddr("g_i_el");
    gPtElAddr = GetNiosAddr("g_pt_el");
    gPAzAddr = GetNiosAddr("g_p_az");
    gIAzAddr = GetNiosAddr("g_i_az");
    gPtAzAddr = GetNiosAddr("g_pt_az");
    gPVPivAddr = GetNiosAddr("g_pv_piv");
    gPEPivAddr = GetNiosAddr("g_pe_piv");
    setRWAddr = GetNiosAddr("set_rw");
    frictOffPivAddr = GetNiosAddr("frict_off_piv");
    velCalcPivAddr = GetNiosAddr("vel_calc_piv");
    velRWAddr = GetNiosAddr("vel_rw");
    accelAzAddr = GetNiosAddr("accel_az");

    dacAmplAddr[0] = GetNiosAddr("v_pump_bal");    // is now ifpm_ampl
    //    dacAmplAddr[0] = GetNiosAddr("dac1_ampl"); // is now ifpm_ampl
    dacAmplAddr[1] = GetNiosAddr("dac2_ampl");
    //    dacAmplAddr[2] = GetNiosAddr("dac3_ampl"); // is now dac_piv
    //    dacAmplAddr[3] = GetNiosAddr("dac4_ampl"); // is now dac_el
    //    dacAmplAddr[4] = GetNiosAddr("dac5_ampl"); // is now dac_rw 
  }

  i_point = GETREADINDEX(point_index);

  //NOTE: this is only used to program the extra DAC - not used for
  // flight.
  if (wait <= 0)
    for (i=1; i<2; i++)
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
  WriteData(velReqElAddr, 32768 + v_elev, NIOS_QUEUE);

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
  /* pointing gain term for elevation drive */
  WriteData(gPtElAddr, CommandData.ele_gain.PT, NIOS_QUEUE);


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
  //bprintf(info, "GetVAz() output after casting to int = %d", v_az);
  /* Units for v_az are 16 bit gyro units*/
  if (v_az > 32767)
    v_az = 32767;
  if (v_az < -32768)
    v_az = -32768;
  WriteData(velReqAzAddr, 32768 + v_az, NIOS_QUEUE);

  v_rw = calcVRW();

  WriteData(velRWAddr, v_rw*(65535.0/2400.0) + 32768.0, NIOS_QUEUE);

  if ((CommandData.disable_az) || (wait > 0)) {
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
  WriteData(dacPivAddr, i_piv*2, NIOS_QUEUE);
  /* p term for az motor */
  WriteData(gPAzAddr, azGainP, NIOS_QUEUE);
  /* I term for az motor */
  WriteData(gIAzAddr, azGainI, NIOS_QUEUE);
  /* pointing gain term for az drive */
  WriteData(gPtAzAddr, CommandData.azi_gain.PT, NIOS_QUEUE);

  /* p term to rw vel for pivot motor */
  WriteData(gPVPivAddr, pivGainRW, NIOS_QUEUE);
  /* p term to vel error for pivot motor */
  WriteData(gPEPivAddr, pivGainErr, NIOS_QUEUE);
  /* setpoint for reaction wheel */
  WriteData(setRWAddr, CommandData.pivot_gain.SP*32768.0/200.0, NIOS_QUEUE);
  /* Pivot current offset to compensate for static friction. */
  WriteData(frictOffPivAddr, pivFrictOff/2.0*65535, NIOS_QUEUE);
  /* Pivot velocity */
  WriteData(velCalcPivAddr, (calcVPiv()/20.0*32768.0), NIOS_QUEUE);
  /* Azimuth Scan Acceleration */
  WriteData(accelAzAddr, (CommandData.az_accel/2.0*65536.0), NIOS_QUEUE);

  if (wait > 0)
    wait--;
}

/***************************************************************/
/*                                                             */
/* GetElDither: set the current elevation dither offset.       */
/*                                                             */
/***************************************************************/
static void GetElDither() {
  time_t seconds;
  int tmp_rand;
  static int first_time = 1;
  double dith_step;
  // Set up the random variable.
  if(first_time) {
    time(&seconds);
    srand((unsigned int) seconds);
    first_time = 0;
  }
  dith_step = CommandData.pointing_mode.dith;
  bprintf(info,"***Dither Time!!!***  dith_step = %f",dith_step);
  
  if (dith_step < -0.000277778 && dith_step > 0.000277778) { // If |dith_step| < 1'' no dither
    axes_mode.el_dith = 0.0;
    bprintf(info,"No dither: axes_mode.el_dith = %f",axes_mode.el_dith);
  } else if (dith_step < 0.00) { // Random mode! May want to remove later...
    tmp_rand = rand();
    axes_mode.el_dith = CommandData.pointing_mode.del*(tmp_rand/RAND_MAX-0.5);      
    bprintf(info,"Random dither: axes_mode.el_dith = %f, tmp_rand = %i",axes_mode.el_dith,tmp_rand);
  } else {
    axes_mode.el_dith += dith_step;
    bprintf(info,"Stepping dither: axes_mode.el_dith = %f, CommandData.pointing_mode.del=%f",axes_mode.el_dith,CommandData.pointing_mode.del);
    bprintf(info,"GetElDither: dith_step =%f, CommandData.pointing_mode.del =%f",dith_step,CommandData.pointing_mode.del);
    if(axes_mode.el_dith > CommandData.pointing_mode.del) {
      axes_mode.el_dith += (-1.0)*CommandData.pointing_mode.del;
      bprintf(info,"GetElDither: Wrapping dither... axes_mode.el_dith=%f",axes_mode.el_dith);
    }
  }
  return;
}

static void ClearElDither() {
  axes_mode.el_dith = 0.0;
  //  bprintf(info,"ClearElDither: axes_mode.el_dith = %f",axes_mode.el_dith);
  return;
}

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/
#define MIN_SCAN 0.1
static void SetAzScanMode(double az, double left, double right, double v,
    double D)
{
    double before_trig;
    if (axes_mode.az_vel < -v + D)
      axes_mode.az_vel = -v + D;
    if (axes_mode.az_vel > v + D)
      axes_mode.az_vel = v + D;

    if (az < left) {
      //TODO: removed isc/osc, but want to keep fast/slow logic for BS cam
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel < v + D)
        axes_mode.az_vel += az_accel;
    } else if (az > right) {
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > -v + D)
        axes_mode.az_vel -= az_accel;
    } else {
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > 0) {
        axes_mode.az_vel = v + D;
#if 0
        if (az > right - 2.0*v) /* within 2 sec of turnaround */
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        else
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
#endif
      } else {
        axes_mode.az_vel = -v + D;
#if 0
        if (az < left + 2.0*v) /* within 2 sec of turnaround */
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        else
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
#endif
      }
    }
    /* BSC Trigger flag */
    before_trig = DELAY - v/CommandData.az_accel 
    + CommandData.theugly.expTime/2000;
    if (az < left + before_trig*v) {
      bsc_trigger = 1;  
    } else if (az > right - before_trig*v) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
}

/* JAS - sinusoidal scan in azimuth at a fixed amplitude around a given
         centre point */
static void DoSineMode(void)
{
  double az, el;
  //double lst, lat;
  double centre, left, right, v_az;//, top, bottom, v_az;
  //double az_of_bot;
  double v_az_max, ampl, turn_around;
  double accel_spider;
  int i_point;
  double t_before; // time at which to send BSC trigger command

  t_before = DELAY + CommandData.theugly.expTime/2000.0;
 
  accel_spider = az_accel*SR; // convert back from deg/s in one Bbus interval
                              // to (deg/s)/s

  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = CommandData.pointing_mode.Y;
  axes_mode.el_vel = 0.0;

  i_point = GETREADINDEX(point_index);
  //lst = PointingData[i_point].lst;
  //lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;// + 28.0;

  centre = CommandData.pointing_mode.X;
  ampl = (CommandData.pointing_mode.w)/2.0;

  right = centre + ampl;
 
  left = centre - ampl;

  SetSafeDAz(az, &left);     // don't cross sun between here and left
  SetSafeDAz(left, &right);  // don't cross sun between left and right
 
  if (right < left) {
    left -= 360.0;
  }

  v_az_max = sqrt(accel_spider * ampl);
  //turn_around = fabs( (centre - ampl*cos(asin(V_AZ_MIN/v_az_max))) - left );
  turn_around = ampl*(1 - sqrt(1-(V_AZ_MIN*V_AZ_MIN)/(v_az_max*v_az_max)));


  if (right-left < MIN_SCAN) {
    left = centre - MIN_SCAN/2.0; 
    right = left + MIN_SCAN;
    ampl = right - centre;
  }

  axes_mode.az_mode = AXIS_VEL; // applies in all cases below:

  /* case 1: moving into quad from beyond left endpoint: */
  if (az < left - turn_around) {
    v_az = sqrt(2.0*accel_spider*(left-az)) + V_AZ_MIN;

    v_az = (v_az > v_az_max) ? v_az_max : v_az;
  
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm beyond the left endpoint.");
  
  /* case 2: moving into quad from beyond right endpoint: */
  } else if (az > right + turn_around) {
    v_az = -sqrt(2.0*accel_spider*(az-right)) - V_AZ_MIN;

    v_az = (v_az < -v_az_max) ? -v_az_max : v_az;
    
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm beyond the right endpoint.");
  /* case 3: moving from left to right endpoints */
  } else if ( (az > left) && (az < right) 
             && (PointingData[i_point].v_az > 0.0) ) {
             //&& (PointingData[i_point].v_az > V_AZ_MIN) ) {
    
    v_az = sqrt(accel_spider*ampl)*sin(acos((centre-az)/ampl));

    //v_az = (v_az < V_AZ_MIN) ? V_AZ_MIN : v_az;

    if (az >= ((right+turn_around) + 
        ampl*(cos(sqrt(accel_spider/ampl)*t_before) - 1.0))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
 
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in between the endpoints and moving right.");

  /* case 4: moving from right to left endpoints */
  } else if ( (az > left) && (az < right) 
              && (PointingData[i_point].v_az < 0.0) ) {
              //&& (PointingData[i_point].v_az < -V_AZ_MIN) ) {

    v_az = sqrt(accel_spider*ampl)*sin(-acos((centre-az)/ampl)); 

    //v_az = (v_az > -V_AZ_MIN) ? -V_AZ_MIN : v_az;

    if (az <= ((left-turn_around) + 
        ampl*(1 - cos(sqrt(accel_spider/ampl)*t_before)))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }

    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in between the endpoints and moving left.");

  /* case 5: in left turn-around zone */ 
  } else if ( (az <= left) && (az >= (left-turn_around)) ) {
    
    //v_az = V_AZ_MIN;
    v_az += az_accel;
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in the left turn-around zone.");

  /* case 6: in right turn-around zone */   
  } else if ( (az >= right) && (az <= (right+turn_around)) ) {

    //v_az = -V_AZ_MIN;
    v_az -= az_accel;
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in the right turn-around zone.");
  } 
}

/* JAS - "modified quad" scan mode for Spider */
static void DoSpiderMode(void)
{
  double az, el;
  double lst, lat;
  double c_az[4], c_el[4]; // corner az and corner el
  double centre, left, right, top, bottom, v_az;
  double az_of_bot;
  double v_az_max, ampl, turn_around;
  double accel_spider;
  int i, i_point;
  double t_before; // time at which to send BSC trigger command

  t_before = DELAY + CommandData.theugly.expTime/2000.0;
 
  accel_spider = az_accel*SR; // convert back from deg/s in one Bbus interval
                              // to (deg/s)/s

  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = CommandData.pointing_mode.Y;
  axes_mode.el_vel = 0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  /* input unchanging lst for testing purposes */
  //lst = 23400.0;
  lat = PointingData[i_point].lat;
  /* input unchanging latitude for testing purposes: 
     McMurdo station at 71 deg. 51 arcmin S */
  //lat = -77.85;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;// + 28.0;

  /* convert ra/decs to az/el */
  for (i = 0; i < 4; i++) {
    radec2azel(CommandData.pointing_mode.ra[i],CommandData.pointing_mode.dec[i],
               lst, lat, c_az+i, c_el+i);
  }

  radbox_endpoints(c_az, c_el, el, &left, &right, &bottom, &top, &az_of_bot);
  //bprintf(info, "az/el corner points are: (%f,%f), (%f,%f), (%f,%f), (%f,%f)",
   //       c_az[0], c_el[0], c_az[1], c_el[1], c_az[2], c_el[2], c_az[3], c_el[3]
     //    );

  SetSafeDAz(az, &left);     // don't cross sun between here and left
  SetSafeDAz(left, &right);  // don't cross sun between left and right
 
  if (right < left) {
    left -= 360.0;
  }

  centre = (left + right) / 2.0;
  
  ampl = right - centre;
  v_az_max = sqrt(accel_spider * ampl);
  turn_around = fabs( (centre - ampl*cos(asin(V_AZ_MIN/v_az_max))) - left );
  //turn_around = 1.0;
  //bprintf(info, "left = %f, right = %f, centre = %f, ampl = %f, v_az_max = %f, turn_around = %f", left, right, centre, ampl, v_az_max, turn_around);

  if (right-left < MIN_SCAN) {
    left = centre - MIN_SCAN/2.0; 
    right = left + MIN_SCAN;
    ampl = right - centre;
  }

  axes_mode.az_mode = AXIS_VEL; // applies in all cases below:

  /* case 1: moving into quad from beyond left endpoint: */
  if (az < left - turn_around) {
    v_az = sqrt(2.0*accel_spider*(left-az)) + V_AZ_MIN;

    v_az = (v_az > v_az_max) ? v_az_max : v_az;
  
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm beyond the left endpoint.");
  
  /* case 2: moving into quad from beyond right endpoint: */
  } else if (az > right + turn_around) {
    v_az = -sqrt(2.0*accel_spider*(az-right)) - V_AZ_MIN;

    v_az = (v_az < -v_az_max) ? -v_az_max : v_az;
    
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm beyond the right endpoint.");
  /* case 3: moving from left to right endpoints */
  } else if ( (az > left) && (az < right) 
	     && (PointingData[i_point].v_az > 0.0) ) {
             //&& (PointingData[i_point].v_az > V_AZ_MIN) ) {
    
    v_az = sqrt(accel_spider*ampl)*sin(acos((centre-az)/ampl));

    if (az >= ((right+turn_around) + 
        ampl*(cos(sqrt(accel_spider/ampl)*t_before) - 1.0))) {
      bsc_trigger = 1;
    }
 
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in between the endpoints and moving right.");

  /* case 4: moving from right to left endpoints */
  } else if ( (az > left) && (az < right) 
	      && (PointingData[i_point].v_az < 0.0) ) {
              //&& (PointingData[i_point].v_az < -V_AZ_MIN) ) {

    v_az = sqrt(accel_spider*ampl)*sin(-acos((centre-az)/ampl)); 

    if (az <= ((left-turn_around) + 
        ampl*(1 - cos(sqrt(accel_spider/ampl)*t_before)))) {
      bsc_trigger = 1;
    }

    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in between the endpoints and moving left.");

  /* case 5: in left turn-around zone */ 
  } else if ( (az <= left) && (az >= (left-turn_around)) ) {
    
    v_az = V_AZ_MIN;
 
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in the left turn-around zone.");

  /* case 6: in right turn-around zone */   
  } else if ( (az >= right) && (az <= (right+turn_around)) ) {

    v_az = -V_AZ_MIN;
    
    axes_mode.az_vel = v_az;
    //bprintf(info, "I'm in the right turn-around zone.");
  } 
 //bprintf(info, "v_az req = %f", v_az);
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

  if (last_x!= CommandData.pointing_mode.X || last_w != w) {
    if (az < left) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else if (az > right) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = right;
      axes_mode.az_vel = 0.0;
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
  //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
  bsc_trigger = 1;
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
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, lat,
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
    ClearElDither(); // sets dither to 0...
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
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
      GetElDither();
      bprintf(info,"We're dithering! El Dither = %f", axes_mode.el_dith);
    }

  }

  el_dir_last = axes_mode.el_dir;

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

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
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, lat,
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
    ClearElDither();
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
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
      GetElDither();
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
    radec2azel(CommandData.pointing_mode.ra[i],
        CommandData.pointing_mode.dec[i],
        lst, lat,
        c_az+i, c_el+i);
    *(c_el+i) += axes_mode.el_dith;
  }

  /* get sky drift speed */
  radec2azel(CommandData.pointing_mode.ra[0],
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
    ClearElDither();
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
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
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
      GetElDither();
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
#if 0
      isc_pulses[0].is_fast = isc_pulses[1].is_fast =
        (sqrt(CommandData.pointing_mode.vaz * CommandData.pointing_mode.vaz
              + CommandData.pointing_mode.del * CommandData.pointing_mode.del)
         > MAX_ISC_SLOW_PULSE_SPEED) ? 1 : 0;
#endif
      bsc_trigger = 1;
      break;
    case P_AZEL_GOTO:
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = CommandData.pointing_mode.X;
      axes_mode.az_vel = 0.0;
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      bsc_trigger = 1;
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
    case P_SPIDER:
      DoSpiderMode();
      break;
    case P_SINE:
      DoSineMode();
      break;
    case P_LOCK:
      axes_mode.el_mode = AXIS_LOCK;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = 0.0;
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      bsc_trigger = 1;
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
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      bsc_trigger = 1;
      break;
  }
  last_mode = CommandData.pointing_mode.mode;
}

// TODO can probably move to blast library, and make bprintf a special case
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

void* reactComm(void* arg)
{

  /* JAS -- reactComm thread from Spider is a clone of pivotComm thread from 
     BLAST-Pol, since both motors now have AMC controllers */

  int bridge_flag=0;  // indicates result of attempt to enable or disable drive
                      // bridge 

  int thread_count=0; // increments once in every run of the infinite
                      // loop

  int i_serial=0;     // counts number of attempts to open or initialize serial
                      // port

  long unsigned res_rw = 0;
  int firsttime=1, resetcount=0;
  unsigned int dp_stat_raw=0, db_stat_raw=0, ds1_stat_raw=0;
  short int current_raw=0;
  int rw_vel_raw = 0;
  unsigned int tmp=0;
  
  /* Initialize values in the reactinfo structure */
                            
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
  reactinfo.verbose=0;

  nameThread("RWCom");

  while (!InCharge) {
    if (firsttime==1) {
      bprintf(info,
      "I am not in charge, thus I will not communicate with the RW motor.");
      firsttime=0;
    }
    usleep(20000);
  }

  bprintf(info,"Bringing the reaction wheel drive online.");
  firsttime=1;

  i_serial=0;

  /* Initialize RWMotorData structure */

  RWMotorData[0].res_rw=0;
  RWMotorData[0].current=0;
  RWMotorData[0].db_stat=0;
  RWMotorData[0].dp_stat=0;
  RWMotorData[0].ds1_stat=0;
  RWMotorData[0].dps_rw=0;
  RWMotorData[0].drive_info=0;
  RWMotorData[0].err_count=0;

  /* Try to open the port */

  while (reactinfo.open==0) {
    reactinfo.verbose=CommandData.verbose_rw;
    open_amc(REACT_DEVICE, &reactinfo); // sets reactinfo.open=1 if sucessful

    if (i_serial==10) {
      bputs(err,
      "RW controller serial port could not be opened after 10 attempts.\n");
    }
    i_serial++;

    if (reactinfo.open==1) {
      bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
     "Opened the serial port on attempt number %i",i_serial);
    }
    else sleep(1);
  }

  /* Configure the serial port.  If, after 10 attempts, the port is not 
     initialized, then the thread enters the main loop, where a reset command 
     is triggered. */
  
  i_serial=0;

  while (reactinfo.init==0 && i_serial<=9) {
    reactinfo.verbose=CommandData.verbose_rw;
    configure_amc(&reactinfo);
    if (reactinfo.init==1) {
      bprintf(info,"Initialized the controller on attempt number %i"
              ,i_serial); 
    } else if (i_serial==9) {
      bprintf(info,"Could not initialize the controller after %i attempts.",
              i_serial); 
    } else {

      sleep(1);
    }
    i_serial++;
  }

  rw_motor_index = 1; // JAS -- this line exists in original RW thread, but not
                      // in pivot thread. Is it necessary?

  while (1) {
    reactinfo.verbose=CommandData.verbose_rw;
    if( (reactinfo.err & AMC_ERR_MASK) > 0 ) {
      reactinfo.err_count+=1;
      if(reactinfo.err_count >= AMC_ERR_TIMEOUT) {
	reactinfo.reset=1;
      }
    }
    if(CommandData.reset_rw==1 ) {
      reactinfo.reset=1;
      CommandData.reset_rw=0;
    }
    if(CommandData.restore_rw==1 ) { // JAS -- CommandData.restore_rw may not
                                     // exist. Need to check this.
      restoreAMC(&reactinfo);
      CommandData.restore_rw=0;
    }
    /* Make bitfield of controller info structure. */
    RWMotorData[rw_motor_index].drive_info=makeMotorField(&reactinfo); 
    RWMotorData[rw_motor_index].err_count=(reactinfo.err_count > 65535) 
                                          ? 65535: reactinfo.err_count;

    /* If we are still in the start up veto, then make sure the drive is 
       disabled. */

    if(StartupVeto > 0) {
      CommandData.disable_az=1;
    }

    if(reactinfo.closing==1){
      rw_motor_index=INC_INDEX(rw_motor_index);
      close_amc(&reactinfo);
      usleep(10000);      
    } else if (reactinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,
        "Resetting serial connection to reaction wheel controller.");
      } else if ((resetcount % 50)==0) {
	bprintfverb(warning,reactinfo.verbose,MC_VERBOSE,
        "reset->Unable to connect to reaction wheel after %i attempts."
        ,resetcount);
      }
      
      //bprintfverb(warning,reactinfo.verbose,MC_EXTRA_VERBOSE,
      //"Attempting to reset the reaction wheel controller.",resetcount);
      //JAS -- it did not make sense to me that resetcount was passed as an 
      //argument to the above print statement. 
      
      bprintfverb(warning,reactinfo.verbose,MC_EXTRA_VERBOSE,
      "Attempting to reset the reaction wheel controller.");
      resetcount++;
      rw_motor_index=INC_INDEX(rw_motor_index);
      resetAMC(REACT_DEVICE, &reactinfo); // if successful, sets 
                                          // reactinfo.reset=0

      if (reactinfo.reset==0) {
	resetcount=0;
        bprintf(info,"Controller successfully reset!");
      }
      usleep(10000);  // give time for motor bits to get written

    } else if (reactinfo.init==1) {
      if(CommandData.disable_az==0 && reactinfo.disabled == 1) {
        bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
        "Attempting to enable the reaction wheel motor contoller.");
        bridge_flag=enableAMC(&reactinfo);
	if(bridge_flag==0) {
	  bprintf(info,"Reaction wheel motor is now enabled");
	  reactinfo.disabled=0;
	}
      }
      if(CommandData.disable_az==1 && (reactinfo.disabled==0 || 
         reactinfo.disabled==2)) {
        bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
        "Attempting to disable the reaction wheel motor controller.");
	bridge_flag=disableAMC(&reactinfo);
	if(bridge_flag==0){    
	  bprintf(info,"Reaction wheel motor controller is now disabled.");
	  reactinfo.disabled=1;
	}
      } 

      if(firsttime){
	firsttime=0;
	tmp = queryAMCInd(0x32,8,1,&reactinfo);
	bprintf(info,"Ki = %i",tmp);
	tmp = queryAMCInd(0xd8,0x24,1,&reactinfo);
	bprintf(info,"Ks = %i",tmp);
	tmp = queryAMCInd(0xd8,0x0c,1,&reactinfo);
	bprintf(info,"d8.0ch = %i",tmp);
	tmp = queryAMCInd(216,12,1,&reactinfo);
	bprintf(info,"v2 d8.0ch = %i",tmp);
	tmp = queryAMCInd(0xd8,0x12,1,&reactinfo);
	bprintf(info,"d8.12h = %i",tmp);
	tmp = queryAMCInd(216,18,1,&reactinfo);
	bprintf(info,"v2 d8.12h = %i",tmp);
	tmp = queryAMCInd(0xd8,0x13,1,&reactinfo);
	bprintf(info,"d8.13h = %i",tmp);
	tmp = queryAMCInd(216,19,1,&reactinfo);
	bprintf(info,"v2 d8.13h = %i",tmp);
      }

      res_rw = getAMCResolver(&reactinfo);
      bprintfverb(info,reactinfo.verbose,MC_VERBOSE,"Resolver Position is: %i"
                  ,res_rw);
      RWMotorData[rw_motor_index].res_rw = fmod((((double) res_rw)/PIV_RES_CTS)
                                           *360.0*4.0, 360.0);

      thread_count %= 5;
      switch(thread_count) {
      case 0:
	current_raw=queryAMCInd(16,3,1,&reactinfo);
        RWMotorData[rw_motor_index].current=((double)current_raw)/8192.0*60.0;
        // divide by scaling factor which is (2^13 / peak drive current) 
        // to get units in amps
	// bprintf(info,"current_raw= %i, current= %f",current_raw,
        // PivotMotorData[pivot_motor_index].current);
	break;
      case 1:
	db_stat_raw=queryAMCInd(2,0,1,&reactinfo);
        RWMotorData[rw_motor_index].db_stat=db_stat_raw;
	break;
      case 2:
	dp_stat_raw=queryAMCInd(2,1,1,&reactinfo);
        RWMotorData[rw_motor_index].dp_stat=dp_stat_raw;
	break;
      case 3:
	ds1_stat_raw=queryAMCInd(2,3,1,&reactinfo);
        RWMotorData[rw_motor_index].ds1_stat=ds1_stat_raw;
	break;
      case 4:
	rw_vel_raw=((int) queryAMCInd(17,2,2,&reactinfo)); 
        RWMotorData[rw_motor_index].dps_rw = rw_vel_raw*(20000.0/131072.0)
                                             *(360.0/4096.0);
        //RWMotorData[rw_motor_index].dps_rw=rw_vel_raw*0.144;
        //bprintf(info, "RW Speed = %d", RWMotorData[rw_motor_index].dps_rw);
	break;
      }
      thread_count++;
      rw_motor_index=INC_INDEX(rw_motor_index);
    } else {
      reactinfo.reset=1;
      rw_motor_index=INC_INDEX(rw_motor_index);
      usleep(10000);
    }
  }
  return NULL;
}
  /* JAS -- old reactComm thread from BLAST-Pol (when reaction wheel motor had
      a Copley controller rather than the AMC one being used for Spider)*/

  /*  //mark1
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
  reactinfo.verbose=0;
  strncpy(reactinfo.motorstr,"react",6);

  nameThread("RWCom");

  while(!InCharge) {
    if(firsttime==1) {
      bprintf(info,"I am not incharge thus I will not communicate with the 
      RW motor.");
      firsttime=0;
    }
    //in case we switch to ICC when serial communications aren't working
    RWMotorData[0].vel_rw=ACSData.vel_rw;
    RWMotorData[1].vel_rw=ACSData.vel_rw;
    RWMotorData[2].vel_rw=ACSData.vel_rw;
    usleep(20000);
  }

  firsttime=1;
  bprintf(info,"Bringing the reaction wheel online.");
  // Initialize structure RWMotorData.  Follows what was done in dgps.c
  //  RWMotorData[0].vel_rw=0;
  RWMotorData[0].temp=0;
  RWMotorData[0].current=0.0;
  RWMotorData[0].status=0;
  RWMotorData[0].fault_reg=0;
  RWMotorData[0].drive_info=0;
  RWMotorData[0].err_count=0;

  // Try to open the port.
  while (reactinfo.open==0) {
    reactinfo.verbose=CommandData.verbose_rw;
    open_copley(REACT_DEVICE,&reactinfo); // sets reactinfo.open=1 if sucessful

    if (i==10){
     bputs(err,
     "Reaction wheel port could not be opened after 10 attempts.\n");
    }
    i++;
    if (reactinfo.open==1) {
      bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
      "Opened the serial port on attempt number %i",i); 
    } else sleep(1);  
  }

  // Configure the serial port. If after 10 attempts the port is not 
  // initialized it enters the main loop where it will trigger a reset command
 
  i=0;
  while (reactinfo.init==0 && i <=9) {
    reactinfo.verbose=CommandData.verbose_rw;
    configure_copley(&reactinfo);
    if (reactinfo.init==1) {
      bprintf(info,"Initialized the controller on attempt number %i",i); 
    } else if (i==9) {
      bprintf(info,"Could not initialize the controller after %i attempts."
      ,i); 
    } else {
      sleep(1);
    }
    i++;
  }
  rw_motor_index = 1; // index for writing to the RWMotor data struct
  while (1){
    reactinfo.verbose=CommandData.verbose_rw;
    if((reactinfo.err & COP_ERR_MASK) > 0 ) {
      reactinfo.err_count+=1;
      if(reactinfo.err_count >= COPLEY_ERR_TIMEOUT) {
	reactinfo.reset=1;
      }
    }
    if(CommandData.reset_rw==1 ) {
      reactinfo.reset=1;
      CommandData.reset_rw=0;
    }
    
    // Make bitfield of controller info structure.
    RWMotorData[rw_motor_index].drive_info=makeMotorField(&reactinfo); 
    RWMotorData[rw_motor_index].err_count=(reactinfo.err_count > 65535) 
                                           ? 65535: reactinfo.err_count;

    // If we are still in the start up veto make sure the drive is disabled.
    if(StartupVeto > 0) {
      CommandData.disable_az=1;
    }

    if(reactinfo.closing==1){
      rw_motor_index=INC_INDEX(rw_motor_index);
      close_copley(&reactinfo);
      usleep(10000);      
    } else if (reactinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,"Resetting connection to Reaction Wheel controller.");
      } else if ((resetcount % 10)==0) {
	//	bprintf(warning,
                "reset-> Unable to connect to Reaction Wheel after %i attempts.
                ",resetcount);
      }

      resetcount++;
      rw_motor_index=INC_INDEX(rw_motor_index);
      resetCopley(REACT_DEVICE,&reactinfo); 
      // if successful sets reactinfo.reset=0
      usleep(10000);  // give time for motor bits to get written
      if (reactinfo.reset==0) {
	resetcount=0;
        bprintf(info,"Controller successfuly reset!");
      }

    } else if(reactinfo.init==1){
      if(CommandData.disable_az==0 && reactinfo.disabled > 0) {
	bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
        "Attempting to enable the reaction wheel motor controller.");
	n=enableCopley(&reactinfo);
	if(n==0){    
	  bprintf(info,"Reaction wheel motor controller is now enabled.");
	  reactinfo.disabled=0;
	}
      } 
      if(CommandData.disable_az==1 && (reactinfo.disabled==0 || 
         reactinfo.disabled==2)) {
	bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
        "Attempting to disable the reaction wheel motor controller.");
	n=disableCopley(&reactinfo);
	if(n==0){    
	  bprintf(info,"Reaction wheel motor controller is now disabled.");
	  reactinfo.disabled=1;
	}
      } 

      vel_raw=queryCopleyInd(COP_IND_VEL,&reactinfo); // Units are 0.1 counts/s
      RWMotorData[rw_motor_index].vel_rw=((double) vel_raw)/RW_ENC_CTS/10.0
                                          *360.0; 
      j=j%4;
      switch(j) {
      case 0:
	temp_raw=queryCopleyInd(COP_IND_TEMP,&reactinfo);
        RWMotorData[rw_motor_index].temp=temp_raw; // units are deg Cel
	break;
      case 1:
	curr_raw=queryCopleyInd(COP_IND_CURRENT,&reactinfo);
        RWMotorData[rw_motor_index].current=((double) (curr_raw))/100.0; 
        // units are Amps
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
	bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
        "Raw reaction wheel velocity is %i",vel_raw);
	firsttime=0;
      }
      rw_motor_index=INC_INDEX(rw_motor_index);

    } else {
      rw_motor_index=INC_INDEX(rw_motor_index);
      reactinfo.reset=1;
      usleep(10000);
    }
    i++; 
  }
  return NULL;
}
  */

// JAS -- Elevation motor serial thread commented out since irrelevant 
//        for Spider

/*void* elevComm(void* arg)
{
#if 0
  //SJB: BLAST-Pol version. removed because it depends on copley stuff

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
  elevinfo.verbose=1;

  nameThread("ElCom");

  while(!InCharge) {
    if(firsttime==1) {
      bprintf(info,"I am not incharge thus I will not communicate with the elevation drive.");
      firsttime=0;
    }

    //in case we switch to ICC when serial communications aren't working
    ElevMotorData[0].enc_raw_el=ACSData.enc_raw_el;
    ElevMotorData[1].enc_raw_el=ACSData.enc_raw_el;
    ElevMotorData[2].enc_raw_el=ACSData.enc_raw_el;
    usleep(20000);
  }

  bprintf(info,"Bringing the elevation drive online.");
  i=0;
  firsttime=1;

  // Initialize structure ElevMotorData.  Follows what was done in dgps.c
  //  ElevMotorData[0].enc_raw_el=0;
  ElevMotorData[0].temp=0;
  ElevMotorData[0].current=0.0;
  ElevMotorData[0].status=0;
  ElevMotorData[0].fault_reg=0;
  ElevMotorData[0].drive_info=0;
  ElevMotorData[0].err_count=0;

  // Try to open the port.
  while(elevinfo.open==0) {
    elevinfo.verbose=CommandData.verbose_el;
    open_copley(ELEV_DEVICE,&elevinfo); // sets elevinfo.open=1 if sucessful
    
    if(i==10) {
      bputs(err,"Elevation drive serial port could not be opened after 10 attempts.\n");
    }
    i++;
    
    if(elevinfo.open==1) {
	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Opened the serial port on attempt number %i",i); 
    } else { 
      sleep(1);
    }
  }
 
  // Configure the serial port.  If after 10 attempts the port is not initialized it enters 
  // the main loop where it will trigger a reset command.                                             
  i=0;
  while (elevinfo.init==0 && i <=9) {
    elevinfo.verbose=CommandData.verbose_el;
    configure_copley(&elevinfo);
    if(elevinfo.init==1) {
      bprintf(info,"Initialized the controller on attempt number %i",i); 
    } else if (i==9) {
      bprintf(info,"Could not initialize the controller after %i attempts.",i); 
    } else {
      sleep(1);
    }
    i++;
  }

  elev_motor_index = 1; // index for writing to the ElevMotor data struct
  while (1) {
    elevinfo.verbose=CommandData.verbose_el;
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

    // If we are still in the start up veto make sure the drive is disabled.
    if(StartupVeto > 0) {
      CommandData.disable_el=1;
    }

    if(elevinfo.closing==1){
      elev_motor_index=INC_INDEX(elev_motor_index);
      close_copley(&elevinfo);
      usleep(10000);      
    } else if (elevinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,"Resetting connection to elevation drive controller.");
      } else if ((resetcount % 10)==0) {
	//	bprintf(warning,"reset-> Unable to connect to elevation drive after %i attempts.",resetcount);
      }

      resetcount++;
      elev_motor_index=INC_INDEX(elev_motor_index);
      resetCopley(ELEV_DEVICE,&elevinfo); // if successful sets elevinfo.reset=0

      usleep(10000);  // give time for motor bits to get written
      if (elevinfo.reset==0) {
	resetcount=0;
        bprintf(info,"Controller successfuly reset!");
      }

    } else if (elevinfo.init==1) {
      if((CommandData.disable_el==0 || CommandData.force_el==1 ) && elevinfo.disabled > 0) {
	bprintf(info,"Attempting to enable the elevation motor controller.,CommandData.disable_el=%i,CommandData.force_el=%i,elevinfo.disabled=%i",CommandData.disable_el,CommandData.force_el,elevinfo.disabled);
	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Attempting to enable the elevation motor controller.");
	n=enableCopley(&elevinfo);
	if(n==0){    
	  bprintf(info,"Elevation motor controller is now enabled.");
	  elevinfo.disabled=0;
	}
      } 
      if((CommandData.disable_el==1 && CommandData.force_el==0 ) && (elevinfo.disabled==0 || elevinfo.disabled==2)) {
	bprintf(info,"Attempting to enable the elevation motor controller.,CommandData.disable_el=%i,CommandData.force_el=%i,elevinfo.disabled=%i",CommandData.disable_el,CommandData.force_el,elevinfo.disabled);
	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Attempting to disable the elevation motor controller.");
	n=disableCopley(&elevinfo);
	if(n==0){    
	  bprintf(info,"Elevation motor controller is now disabled.");
	  elevinfo.disabled=1;
	}
      } 

      pos_raw=queryCopleyInd(COP_IND_POS,&elevinfo); // Units are counts
                                                     // For Elev 524288 cts = 360 deg
      ElevMotorData[elev_motor_index].enc_raw_el=((double) (pos_raw % ((long int) ELEV_ENC_CTS)))/ELEV_ENC_CTS*360.0-ENC_RAW_EL_OFFSET;
      //   getCopleySlowInfo(j,elev_motor_index,&ElevMotorData,&elevinfo); // Reads one of temperature, current, status and fault register and
                           // writes to the appropriate frame 

      if (firsttime) {
	bprintfverb(info,elevinfo.verbose,MC_VERBOSE,"Raw elevation encoder position is %i",pos_raw);
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
      elevinfo.reset=1;
      elev_motor_index=INC_INDEX(elev_motor_index);
      usleep(10000);
    }
  }
  return NULL;
}*/

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
  pivotinfo.verbose=0;

  nameThread("PivCom");

  while (!InCharge) {
    if (firsttime==1) {
      bprintf(info,"I am not incharge thus I will not communicate with the pivot motor.");
      firsttime=0;
    }
    usleep(20000);
  }

  bprintf(info,"Bringing the pivot drive online.");
  firsttime=1;

  i=0;

  // Initialize structure PivotMotorData.  Follows what was done in dgps.c
  PivotMotorData[0].res_piv=0;
  PivotMotorData[0].current=0;
  PivotMotorData[0].db_stat=0;
  PivotMotorData[0].dp_stat=0;
  PivotMotorData[0].ds1_stat=0;
  PivotMotorData[0].dps_piv=0;
  PivotMotorData[0].drive_info=0;
  PivotMotorData[0].err_count=0;

  // Try to open the port.
  while (pivotinfo.open==0) {
    pivotinfo.verbose=CommandData.verbose_piv;
    open_amc(PIVOT_DEVICE,&pivotinfo); // sets pivotinfo.open=1 if sucessful

    if (i==10) {
      bputs(err,"Pivot controller serial port could not be opened after 10 attempts.\n");
    }
    i++;

    if (pivotinfo.open==1) {
      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Opened the serial port on attempt number %i",i);
    }
    else sleep(1);
  }

  // Configure the serial port.  If after 10 attempts the port is not initialized it enters 
  // the main loop where it will trigger a reset command.                                             
  i=0;
  while (pivotinfo.init==0 && i <=9) {
    pivotinfo.verbose=CommandData.verbose_piv;
    configure_amc(&pivotinfo);
    if (pivotinfo.init==1) {
      bprintf(info,"Initialized the controller on attempt number %i",i); 
    } else if (i==9) {
      bprintf(info,"Could not initialize the controller after %i attempts.",i); 
    } else {
      sleep(1);
    }
    i++;
  }

  while (1) {
    pivotinfo.verbose=CommandData.verbose_piv;
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
    if(CommandData.restore_piv==1 ) {
      restoreAMC(&pivotinfo);
      CommandData.restore_piv=0;
    }

    PivotMotorData[pivot_motor_index].drive_info=makeMotorField(&pivotinfo); // Make bitfield of controller info structure.
    PivotMotorData[pivot_motor_index].err_count=(pivotinfo.err_count > 65535) ? 65535: pivotinfo.err_count;
    // If we are still in the start up veto make sure the drive is disabled.
    if(StartupVeto > 0) {
      CommandData.disable_az=1;
    }

    if(pivotinfo.closing==1){
      pivot_motor_index=INC_INDEX(pivot_motor_index);
      close_amc(&pivotinfo);
      usleep(10000);      
    } else if (pivotinfo.reset==1){
      if(resetcount==0) {
	bprintf(warning,"Resetting connection to pivot controller.");
      } else if ((resetcount % 50)==0) {
	bprintfverb(warning,pivotinfo.verbose,MC_VERBOSE,"reset->Unable to connect to pivot after %i attempts.",resetcount);
      }

      bprintfverb(warning,pivotinfo.verbose,MC_EXTRA_VERBOSE,"Attempting to reset the pivot controller.",resetcount);
      resetcount++;
      pivot_motor_index=INC_INDEX(pivot_motor_index);
      resetAMC(PIVOT_DEVICE,&pivotinfo); // if successful sets pivotinfo.reset=0

      if (pivotinfo.reset==0) {
	resetcount=0;
        bprintf(info,"Controller successfuly reset!");
      }
      usleep(10000);  // give time for motor bits to get written

    } else if (pivotinfo.init==1) {
      if(CommandData.disable_az==0 && pivotinfo.disabled == 1) {
      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Attempting to enable the pivot motor contoller.");
	n=enableAMC(&pivotinfo);
	if(n==0) {
	  bprintf(info,"Pivot motor is now enabled");
	  pivotinfo.disabled=0;
	}
      }
      if(CommandData.disable_az==1 && (pivotinfo.disabled==0 || pivotinfo.disabled==2)) {
      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Attempting to disable the pivot motor controller.");
	n=disableAMC(&pivotinfo);
	if(n==0){    
	  bprintf(info,"Pivot motor controller is now disabled.");
	  pivotinfo.disabled=1;
	}
      } 

      if(firsttime){
	firsttime=0;
	tmp = queryAMCInd(0x32,8,1,&pivotinfo);
	bprintf(info,"Ki = %i",tmp);
	tmp = queryAMCInd(0xd8,0x24,1,&pivotinfo);
	bprintf(info,"Ks = %i",tmp);
	tmp = queryAMCInd(0xd8,0x0c,1,&pivotinfo);
	bprintf(info,"d8.0ch = %i",tmp);
	tmp = queryAMCInd(216,12,1,&pivotinfo);
	bprintf(info,"v2 d8.0ch = %i",tmp);
	tmp = queryAMCInd(0xd8,0x12,1,&pivotinfo);
	bprintf(info,"d8.12h = %i",tmp);
	tmp = queryAMCInd(216,18,1,&pivotinfo);
	bprintf(info,"v2 d8.12h = %i",tmp);
	tmp = queryAMCInd(0xd8,0x13,1,&pivotinfo);
	bprintf(info,"d8.13h = %i",tmp);
	tmp = queryAMCInd(216,19,1,&pivotinfo);
	bprintf(info,"v2 d8.13h = %i",tmp);
      }

      pos_raw=getAMCResolver(&pivotinfo);
      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,"Resolver Position is: %i",pos_raw);
      PivotMotorData[pivot_motor_index].res_piv=((double) pos_raw)/PIV_RES_CTS*360.0; 

      j=j%5;
      switch(j) {
      case 0:
	current_raw=queryAMCInd(16,3,1,&pivotinfo);
        PivotMotorData[pivot_motor_index].current=((double)current_raw)/8192.0*20.0; // *2^13 / peak drive current
	                                                                             // Units are Amps
	//        bprintf(info,"current_raw= %i, current= %f",current_raw,PivotMotorData[pivot_motor_index].current);
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
        PivotMotorData[pivot_motor_index].dps_piv = piv_vel_raw
                                                    *(20000.0/131072.0)
                                                    *(360.0/16384.0);

	break;
      }
      j++;
      pivot_motor_index=INC_INDEX(pivot_motor_index);
    } else {
      pivotinfo.reset=1;
      pivot_motor_index=INC_INDEX(pivot_motor_index);
      usleep(10000);
    }
  }
  return NULL;
}
