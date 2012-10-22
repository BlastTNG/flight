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

#include "channels.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"
#include "amccommand.h"
#include "motordefs.h"
#include "calibrate.h"

// TODO: Revise these el limits for Spider flight:
#define MIN_EL 35
#define MAX_EL 45

#define VPIV_FILTER_LEN 40
#define FPIV_FILTER_LEN 1000

#define RW_BASE 0.95    // base for exponential filter used to compute RW
                        // speed

#define V_AZ_MIN 0.05  //smallest measured az speed we trust given gyro
                        //   noise/offsets

/* elevation drive related defines adapted from az-el.c in minicp: */
#define MAX_STEP 10000    // maximum step rate to send to el motors
#define CM_PULSES 5000   // Cool Muscle pulses per rotation
#define IN_TO_MM 25.4
#define ROT_PER_INCH 5    // linear actuator rotations per inch of travel
#define EL_GEAR_RATIO 7.0 
#define L_C 1207.79       // distance from cryo axis to lin. act. axis (mm)
#define L_R 400.0         // length of rocker arm in mm
#define L_L 978.94        // actuator length in mm (fully retracted)

#define ANG 103.3         // angle in degrees relevant to geometry of system
                          // = 180 - 90.76 + 14.06, where the 2nd angle is
                          // between the rocker arm and bore-sight, and the
                          // 3rd angle is between L_C and horizontal.

#define CNTS_PER_DEG (65536.0/360.0)
#define CM_PER_ENC (1000.0/72000.0)
#define TOLERANCE 0.05    // max acceptable el pointing error (deg)
#define TWIST_TOL 0.01    // max acceptable twist error (deg)

/* variables storing scan region (relative to defined quad) */
static int scan_region = 0;
static int scan_region_last = 0;

/* possible regions of sinsuoidal scan mode */
#define SCAN_BEYOND_L 1
#define SCAN_BEYOND_R 2
#define SCAN_L_TO_R 3
#define SCAN_R_TO_L 4
#define SCAN_L_TURN 5
#define SCAN_R_TURN 6

void nameThread(const char*);	/* mcp.c */

struct RWMotorDataStruct RWMotorData[3]; // defined in point_struct.h
int rw_motor_index; 

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
static pthread_t pivotcomm_id;

// device node address for the reaction wheel motor controller
#define REACT_DEVICE "/dev/ttySI9"
#define PIVOT_DEVICE "/dev/ttySI13"

static void* reactComm(void *arg);
static void* pivotComm(void *arg);


static double dxdtheta(double theta);

extern short int InCharge; /* tx.c */

extern int StartupVeto; /* mcp.c */

extern short int bsc_trigger; /* Semaphore for BSC trigger */

#define DELAY (3.685*20.0) // delay between starcam exposure command and pulse_bsc in units of Bbus frame intervals */


/* opens communications with motor controllers */
void openMotors()
{
  bprintf(info, "Motors: connecting to motors");
  pthread_create(&reactcomm_id, NULL, &reactComm, NULL);
  pthread_create(&pivotcomm_id, NULL, &pivotComm, NULL);
}

void closeMotors()
{
  int i=0;
  /* Tell the serial threads to shut down */
  reactinfo.closing=1;
  pivotinfo.closing=1;

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
    v = dx*(ACSData.bbc_rate/((double)frame_count));
    frame_count = 0; 
  }  

  last_x = x;
  last_v = v;

  u = ( RW_BASE*last_u + (1-RW_BASE)*v );
  last_u = u;
  //bprintf(info, "i_rw = %d, x = %f deg, v = %f dps, u = %f dps", 
  //i_rw, x, v, u);

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

/*************************************************************************

    SetVElev: Set elevation drive velocity request using gain terms 
               This is just a utility called by GetVElev.

    NEW in Spider!
*************************************************************************/
static double SetVElev(double g_com, double g_diff, double dy, double err, 
                       double v_last, double max_dv, double enc) 
{

  double v, v_com, v_diff, v_max;

  v_max = fabs( ((double)MAX_STEP*IN_TO_MM) / 
	     ((double)CM_PULSES*EL_GEAR_RATIO*ROT_PER_INCH*dxdtheta(enc)) );
  
  if (dy >= 0) {
    v_com = g_com*sqrt(dy);
    
    if (v_com > v_max) {
      v_com = v_max;
    }
    
    v_diff = -g_diff*err;
    
    if (v_diff > v_max) {
      v_diff = v_max;
    } else if (v_diff < -v_max) {
      v_diff = -v_max;
    }
    
    v = v_com + v_diff;
  
  } else {
    v_com = -g_com*sqrt(-dy);
    
    if (v_com < -v_max) {
      v_com = -v_max;
    } 
    
    v_diff = -g_diff*err;
    
    if (v_diff > v_max) {
      v_diff = v_max;
    } else if (v_diff < -v_max) {
      v_diff = -v_max;
    }
    
    v = v_com + v_diff;
    
  }
/* don't increase/decrease request by more than max_dv: */
  v = ((v - v_last) > max_dv) ? (v_last + max_dv) : v;
  v = ((v - v_last) < -max_dv) ? (v_last - max_dv) : v;    

  //bprintf(info,"v: %g g_diff: %g dy: %g err: %g", v, g_diff, dy, err);
  return v;

}

/************************************************************************/
/*                                                                      */
/*   GetVElev: get the current elevation velocity request, given current*/
/*   pointing mode, etc..                                               */
/*                                                                      */
/************************************************************************/
static void GetVElev(double* v_P, double* v_S)
{

/* JAS -- complete rewrite of this function for Spider */

// S = STARBOARD
// P = PORT

/* various dynamical variables */
  double enc_port, enc_strbrd;
  static double enc_port_last, enc_strbrd_last;
  double el_dest;
  double dy; 

  double err;           // error between encoder position and mean position
                        // (positive for left, negative for right) 

  //double err_max = 0.005; // maximum permissible difference between left and
                          // right encoders.
  double max_dv;
  double g_com;
  double g_diff=0.0;
  double v_P_max;
  double v_S_max;

/* requested velocities (prev. values) */
  static double v_S_last = 0.0;
  static double v_P_last = 0.0;  
  static double el_dest_last = 0.0;
  
  static double del_strbrd_targ = 0.0;
  static double enc_strbrd_ref = 0.0;
  double del_strbrd;
  static int on_delay = 0;
  static double el_dest_this = -1.0;

//  static int motors_off = 1;
  static int since_arrival = 0;
  

   if (el_dest_this < 15.0) {
     el_dest_this = ACSData.enc_mean_el;
   }


  if (axes_mode.el_dest > MAX_EL) {
    
    el_dest = axes_mode.el_dest = MAX_EL;
    
  } else if (axes_mode.el_dest < MIN_EL) {
    
    el_dest = axes_mode.el_dest = MIN_EL;
    
  } else {
    
    el_dest = axes_mode.el_dest;
    
  }

  /* port = sum/2 + diff/2 */
  enc_port = ACSData.enc_mean_el + ACSData.enc_diff_el/2.0;
  enc_strbrd = ACSData.enc_mean_el - ACSData.enc_diff_el/2.0;

  if ( !(CommandData.power.elmot_auto) || (on_delay >= 35) ) {
    el_dest_this = el_dest;
    g_diff = CommandData.ele_gain.diff;
  }

  dy = el_dest_this - ACSData.enc_mean_el;

  err = -(ACSData.enc_diff_el)/2.0;
  err += CommandData.ele_gain.twist*0.5; // user-settable fake offset 
                                     // to test the twist correction


  g_com = CommandData.ele_gain.com * (double)(fabs(dy)>TOLERANCE);
  max_dv = 1.05 * CommandData.ele_gain.com*CommandData.ele_gain.com * (1.0/(2.0*ACSData.bbc_rate));  // 5% higher than deceleration...

  g_diff *= (double)(fabs(err)>TWIST_TOL);

  *v_P = SetVElev(g_com, -g_diff, dy, err, v_P_last, max_dv, enc_port);
  *v_S = SetVElev(g_com, g_diff, dy, err, v_S_last, max_dv, enc_strbrd);
 
  if ( !(CommandData.disable_el) && CommandData.power.elmot_auto ) {
    if ( fabs(el_dest - ACSData.enc_mean_el) < TOLERANCE ) {
      since_arrival++;
      if (since_arrival >= 500) {
	if (CommandData.power.elmot_is_on) {      
          CommandData.power.elmot.set_count = 0;
          CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
	}
        on_delay = 0;
      }
    } else if ( fabs(el_dest-el_dest_last) > TOLERANCE ) {
      since_arrival = 0;
      on_delay++;
      if ( !(CommandData.power.elmot_is_on) ) {
        CommandData.power.elmot.rst_count = 0;
        CommandData.power.elmot.set_count = LATCH_PULSE_LEN;  
      }
    }
  }
  
  el_dest_last = el_dest_this;
  
  /* don't command a velocity greater than limit from max pulse rate */

  v_P_max = fabs( ((double)MAX_STEP*IN_TO_MM) / 
	    ((double)CM_PULSES*EL_GEAR_RATIO*ROT_PER_INCH*dxdtheta(enc_port)) );
 
  v_S_max = fabs( ((double)MAX_STEP*IN_TO_MM) / 
	  ((double)CM_PULSES*EL_GEAR_RATIO*ROT_PER_INCH*dxdtheta(enc_strbrd)) );

  if (*v_P > v_P_max) {
    *v_P = v_P_max;
  } else if (*v_P < -v_P_max) {
    *v_P = -v_P_max;
  }
  if (*v_S > v_S_max) {
    *v_S = v_S_max;
  } else if (*v_S < -v_S_max) {
    *v_S = -v_S_max;
  }

  /* error checking: if one motor is stalled, stop the other one */
  if (enc_strbrd_ref == 0.0) {
    enc_strbrd_ref = enc_strbrd;
  }
  
  del_strbrd_targ += *v_S / ACSData.bbc_rate;
  del_strbrd = enc_strbrd - enc_strbrd_ref;
  if (fabs(del_strbrd_targ)>0.05) {
    if (fabs(del_strbrd)<0.025) {
      //bprintf(info, "strbrd stall detected: %g %g %g", del_strbrd_targ, del_strbrd, *v_S);
      *v_P = *v_S = 0.0;
      // FIXME: handle the stall here...
    } else {        
      //bprintf(info, "strbrd no stall : %g %g", del_strbrd_targ, del_strbrd);
    }
    del_strbrd_targ = 0;
    enc_strbrd_ref = enc_strbrd;
  }
   
  enc_port_last = enc_port;
  enc_strbrd_last = enc_strbrd;

  v_P_last = *v_P; 
  v_S_last = *v_S; 

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
  double vel_offset;
  double az, az_dest;
  double t_bbus;
  double max_dv;// = 20;
  double dx;

  i_point = GETREADINDEX(point_index);
  
  t_bbus = 1.0/(ACSData.bbc_rate);
  max_dv = 1.05*(CommandData.az_accel_max)*t_bbus;
  max_dv *= DPS_TO_GY16;
  //max_dv = 1000;

  if (axes_mode.az_mode == AXIS_VEL) {
     vel = axes_mode.az_vel;
    // vel = -axes_mode.az_vel; // temporary negative sign since DSP control loop seems wrong (POSITIVE FEEDBACK!)
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
    //vel *= -(double)CommandData.azi_gain.PT/10000.0; // temp negative sign
  }

  /*vel_offset = -(PointingData[i_point].offset_ifroll_gy 
               - PointingData[i_point].ifroll_earth_gy)
               * cos(PointingData[i_point].el * M_PI / 180.0) 
               -(PointingData[i_point].offset_ifyaw_gy 
               - PointingData[i_point].ifyaw_earth_gy)
               * sin(PointingData[i_point].el * M_PI / 180.0);*/

  // gyros are on the outer frame for Spider...
 
//  vel_offset =-(PointingData[i_point].offset_ifyaw_gy 
    //            - PointingData[i_point].ifyaw_earth_gy);

  // TODO: signs above appear to be wrong???
  vel_offset = (PointingData[i_point].offset_ifyaw_gy 
                + PointingData[i_point].ifyaw_earth_gy);

  vel -= vel_offset;
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
/* GetVPivot: get the velocity request for the pivot in DAC units       */
/*            Proportional to the reaction wheel speed error            */       
/*            (and other things). NOTE: used to be GetIPivot, but now   */
/*            we are going to try running the pivot in velocity mode    */
/*                                                                      */
/************************************************************************/
static double GetVPivot(int TxIndex, unsigned int gI_v_rw,unsigned int gP_v_rw, 
			  unsigned int gP_v_az, unsigned int gP_t_rw,
		          unsigned int gP_v_req, unsigned int disabled)
{
  static struct NiosStruct* pVRWTermPivAddr;
  static struct NiosStruct* iVRWTermPivAddr;
  static struct NiosStruct* pVAzTermPivAddr;
  static struct NiosStruct* pTRWTermPivAddr;
  static struct NiosStruct* pVReqAzTermPivAddr;
  
  static struct BiPhaseStruct* dacRWAddr;

  double v_req = 0.0;
  int v_req_dac = 0;
  int i_point;
  double I_v_rw_term, P_v_rw_term, P_v_az_term, P_t_rw_term, P_v_req_term;
  int I_v_rw_term_dac, P_v_rw_term_dac, P_v_az_term_dac, P_t_rw_term_dac,
      P_v_req_term_dac;
      
  unsigned short int dac_rw;

  static unsigned int firsttime = 1;

  static double int_v_rw = 0.0;  // integrated reaction wheel speed

  double a = 0.9998;

  /* TODO: Remove this note.
   * v_piv = (I_rw * int_v_rw) + (P_rw * v_rw) + (P_g * v_g) + (P_t_rw * t_rw)
   * with appropriate signs */

   // g_I_v_rw, g_P_v_rw, g_P_v_az, g_P_t_rw;

  if(firsttime) {
    pVRWTermPivAddr = GetNiosAddr("p_v_rw_term_piv");
    iVRWTermPivAddr = GetNiosAddr("i_v_rw_term_piv");
    pVAzTermPivAddr = GetNiosAddr("p_v_az_term_piv");
    pTRWTermPivAddr = GetNiosAddr("p_t_rw_term_piv");
    pVReqAzTermPivAddr = GetNiosAddr("p_v_req_az_term_piv");
    
    dacRWAddr = GetBiPhaseAddr("dac_rw");
    
    firsttime = 0;
  }
  i_point = GETREADINDEX(point_index);

  int_v_rw = (1.0-a)*ACSData.vel_rw + a*int_v_rw; 

  dac_rw = ReadCalData(dacRWAddr);

  /* Calculate control terms */
  P_v_rw_term = ( ((double) gP_v_rw)/1000.0 )*(ACSData.vel_rw - CommandData.pivot_gain.SP);
  P_t_rw_term = ( ((double)gP_t_rw)/1000.0 )*((double)(dac_rw-32768)); 
  //P_v_az_term = -1.0*( (double)gP_v_az )*(PointingData[i_point].v_az);
  P_v_az_term = 1.0*( (double)gP_v_az )*(PointingData[i_point].v_az);
  I_v_rw_term = ( (double)gI_v_rw/100.0 )*(int_v_rw);
  if ( (CommandData.pointing_mode.mode == P_SPIDER) || 
       (CommandData.pointing_mode.mode == P_SINE) ) {
     if ( (scan_region != SCAN_BEYOND_L) && (scan_region != SCAN_BEYOND_R) ){
       P_v_req_term = -( (double)gP_v_req )*axes_mode.az_vel;
     } else {
       P_v_req_term = 0.0;
     }
  } else {
    P_v_req_term = 0.0;
  }    
  v_req = P_v_rw_term + P_t_rw_term + P_v_az_term + I_v_rw_term + P_v_req_term;

  if(disabled) { // Don't request a velocity if we are disabled.
    v_req=0.0;
  }

  /* Convert to DAC Units*/

  if (v_req > 0.0) {
    v_req_dac=v_req+32768+PIV_DAC_OFF+PIV_DEAD_BAND;
  } else {
    v_req_dac=v_req+32768+PIV_DAC_OFF-PIV_DEAD_BAND;
  }

  if(P_v_rw_term>0.0) {
    P_v_rw_term_dac=P_v_rw_term+32768+PIV_DAC_OFF+PIV_DEAD_BAND;
  } else {
    P_v_rw_term_dac=P_v_rw_term+32768+PIV_DAC_OFF-PIV_DEAD_BAND;
  }

  if(P_t_rw_term>0.0) {
    P_t_rw_term_dac=P_t_rw_term+32768+PIV_DAC_OFF+PIV_DEAD_BAND;
  } else {
    P_t_rw_term_dac=P_t_rw_term+32768+PIV_DAC_OFF-PIV_DEAD_BAND;
  }
 
  if(P_v_az_term>0.0) {
    P_v_az_term_dac=P_v_az_term+32768+PIV_DAC_OFF+PIV_DEAD_BAND;
  } else {
    P_v_az_term_dac=P_v_az_term+32768+PIV_DAC_OFF-PIV_DEAD_BAND;
  }

  if(I_v_rw_term>0.0) {
    I_v_rw_term_dac=I_v_rw_term+32768+PIV_DAC_OFF+PIV_DEAD_BAND;
  } else {
    I_v_rw_term_dac=I_v_rw_term+32768+PIV_DAC_OFF-PIV_DEAD_BAND;
  }
  
  if(P_v_req_term>0.0) {
    P_v_req_term_dac=P_v_req_term+32768+PIV_DAC_OFF+PIV_DEAD_BAND;
  } else {
    P_v_req_term_dac=P_v_req_term+32768+PIV_DAC_OFF-PIV_DEAD_BAND;
  }
  
  // Check to make sure the DAC value is in the proper range
  if(v_req_dac <= 0) {
    v_req_dac=1;
  }

  if(v_req_dac > 65535) {
    v_req_dac=65535;
  }

  // Check to make sure the control terms are in the proper range
  if(P_v_rw_term_dac <= 0) {
    P_v_rw_term_dac=1;
  }
  if(P_v_rw_term_dac > 65535) {
    P_v_rw_term_dac=65535;
  }

  if(P_t_rw_term_dac <= 0) {
    P_t_rw_term_dac=1;
  }
  if(P_t_rw_term_dac > 65535) {
    P_t_rw_term_dac=65535;
  }

  if(P_v_az_term_dac <= 0) {
    P_v_az_term_dac=1;
  }
  if(P_v_az_term_dac > 65535) {
    P_v_az_term_dac=65535;
  }

  if(I_v_rw_term_dac <= 0) {
    I_v_rw_term_dac=1;
  }
  if(I_v_rw_term_dac > 65535) {
    I_v_rw_term_dac=65535;
  }

  if(P_v_req_term_dac <= 0) {
    P_v_req_term_dac=1;
  }
  if(P_v_req_term_dac > 65535) {
    P_v_req_term_dac=65535;
  }

  /* Write control terms to frame */
  if (TxIndex == 0) {
    WriteData(pVRWTermPivAddr, P_v_rw_term_dac, NIOS_QUEUE);
    WriteData(iVRWTermPivAddr, I_v_rw_term_dac, NIOS_QUEUE);
    WriteData(pVAzTermPivAddr, P_v_az_term_dac, NIOS_QUEUE);
    WriteData(pTRWTermPivAddr, P_t_rw_term_dac, NIOS_QUEUE);
    WriteData(pVReqAzTermPivAddr, P_v_req_term_dac, NIOS_QUEUE);
  }
  return v_req_dac;
}

/* dxdtheta returns derivative of lin. act. extension. w.r.t. elevation angle 
 * (mm/deg) */

static double dxdtheta(double theta)
{
  double deriv;

  deriv = -(L_C*L_R*sin((ANG - theta)*M_PI/180.0)) / 
          (sqrt(pow(L_C,2) + pow(L_R,2)-2*L_C*L_R*cos((ANG-theta)*M_PI/180.0)));

  deriv *= (M_PI/180.0); // convert from mm/rad to mm/deg

  return deriv; 
}


/************************************************************************/
/*                                                                      */
/*    WriteMot: motors, and, for convenience, the inner frame lock      */
/*                                                                      */
/************************************************************************/
void WriteMot(int TxIndex)
{
  static struct NiosStruct* velReqAzAddr;
  static struct NiosStruct* gComElAddr;
  static struct NiosStruct* gDiffElAddr;
  static struct NiosStruct* gPAzAddr;
  static struct NiosStruct* gIAzAddr;
  static struct NiosStruct* gPtAzAddr;
  static struct NiosStruct* gVRWPivAddr;
  static struct NiosStruct* gIRWPivAddr;
  static struct NiosStruct* gVAzPivAddr;
  static struct NiosStruct* gTRWPivAddr;
  static struct NiosStruct* gVReqAzPivAddr;
  static struct NiosStruct* setRWAddr;
  static struct NiosStruct* dacPivAddr;
  static struct NiosStruct* velCalcPivAddr;
  static struct NiosStruct* velRWAddr;
  static struct NiosStruct* accelAzAddr;
  static struct NiosStruct* accelMaxAzAddr;
  static struct NiosStruct* step1ElAddr;      // PORT
  static struct NiosStruct* step2ElAddr;      // STARBOARD

  // Used only for Lab Controller tests
  static struct NiosStruct* dac2AmplAddr;

  static int wait = 100; /* wait 20 frames before controlling. */

  int v_az, v_piv;
  double elGainCom, elGainDiff;
  double v_el_P = 0.0; // port
  double v_el_S = 0.0; // starboard

  double el_deriv_P; // deriv. of lin. act. extension w.r.t. el angle
  double el_deriv_S; // deriv. of lin. act. extension w.r.t. el angle
  double el_rps_P; // rev/s of el motor
  double el_rps_S; // rev/s of el motor
  double enc_P;    // port encoder angle
  double enc_S;    // starboard encoder angle
  int step_rate_P; // pulse rate (Hz) of step input to el motor
  int step_rate_S; // pulse rate (Hz) of step input to el motor
   
  double v_rw;
  int azGainP, azGainI, pivGainVelRW, pivGainVelAz, pivGainPosRW, pivGainTorqueRW, pivGainVelReqAz;
  int i_point;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    velReqAzAddr = GetNiosAddr("vel_req_az");
    dacPivAddr = GetNiosAddr("dac_piv");
    gComElAddr = GetNiosAddr("g_com_el");
    gDiffElAddr = GetNiosAddr("g_diff_el");
    gPAzAddr = GetNiosAddr("g_p_az");
    gIAzAddr = GetNiosAddr("g_i_az");
    gPtAzAddr = GetNiosAddr("g_pt_az");
    gVRWPivAddr = GetNiosAddr("g_v_rw_piv");
    gIRWPivAddr = GetNiosAddr("g_i_rw_piv");
    gVAzPivAddr = GetNiosAddr("g_v_az_piv");
    gTRWPivAddr = GetNiosAddr("g_t_rw_piv");
    gVReqAzPivAddr = GetNiosAddr("g_v_req_az_piv");
    setRWAddr = GetNiosAddr("set_rw");
    velCalcPivAddr = GetNiosAddr("vel_calc_piv");
    velRWAddr = GetNiosAddr("vel_rw");
    accelAzAddr = GetNiosAddr("accel_az");
    accelMaxAzAddr = GetNiosAddr("accel_max_az");
    dac2AmplAddr = GetNiosAddr("dac2_ampl");
    step1ElAddr = GetNiosAddr("step_1_el");
    step2ElAddr = GetNiosAddr("step_2_el");
  }

  i_point = GETREADINDEX(point_index);

  //NOTE: this is only used to program the extra DAC - not used for
  // flight.
  if (TxIndex == 0) {  //only write at slow frame rate
    if (CommandData.Temporary.setLevel[1] && wait <= 0) {
      WriteData(dac2AmplAddr, CommandData.Temporary.dac_out[1], NIOS_QUEUE);
      CommandData.Temporary.setLevel[1] = 0;
    }
  }

  /***************************************************/
  /**           Elevation Drive Motors              **/
  /* elevation speed */
  //v_elev = floor(GetVElev() + 0.5);
  
  GetVElev(&v_el_P, &v_el_S);
  
  enc_P = ACSData.enc_mean_el + ACSData.enc_diff_el/2.0;
  enc_S = ACSData.enc_mean_el - ACSData.enc_diff_el/2.0;

  /* convert elevation velocities from dps to Cool Muscle motor units */
  el_deriv_P = dxdtheta(enc_P);
  el_deriv_S = dxdtheta(enc_S);
  
  /*compute rotations per second of Cool Muscle Motor: */
  
  el_rps_P = (el_deriv_P*v_el_P/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO;
  el_rps_S = (el_deriv_S*v_el_S/IN_TO_MM)*ROT_PER_INCH*EL_GEAR_RATIO;
  
  
  /* check to see if we're in manual pulse mode */

  if (CommandData.ele_gain.manual_pulses) {
    step_rate_P = CommandData.ele_gain.pulse_port;
    step_rate_S = CommandData.ele_gain.pulse_starboard;
  } else {
    step_rate_P = (int) (el_rps_P*CM_PULSES);
    step_rate_S = (int) (el_rps_S*CM_PULSES);
  } 
  
  if ( step_rate_P > MAX_STEP ) {
    step_rate_P = MAX_STEP;
  } else if ( step_rate_P < -MAX_STEP ) {
    step_rate_P = -MAX_STEP;
  }

  if ( step_rate_S > MAX_STEP ) {
    step_rate_S = MAX_STEP;
  } else if ( step_rate_S < -MAX_STEP ) {
    step_rate_S = -MAX_STEP;
  }
  
  /* no motor pulses if the pin is in */
  if ((CommandData.lock.pin_is_in && !CommandData.force_el)
      || CommandData.disable_el) {
    WriteCalData(step1ElAddr, 0.0, NIOS_QUEUE);
    WriteCalData(step2ElAddr, 0.0, NIOS_QUEUE);
  } else {
    WriteCalData(step1ElAddr, -step_rate_P, NIOS_QUEUE);
    WriteCalData(step2ElAddr, -step_rate_S, NIOS_QUEUE);	
  }

  if (TxIndex == 0) {   //only write at slow frame rate
  
    elGainCom = CommandData.ele_gain.com;
    elGainDiff = CommandData.ele_gain.diff;	

    /*common-mode gain term for el motors*/
    WriteCalData(gComElAddr, elGainCom, NIOS_QUEUE);

    /*differential gain term for el motors */
    WriteCalData(gDiffElAddr, elGainDiff, NIOS_QUEUE);
    
  }

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = floor(GetVAz() + 0.5);
  
  /* Units for v_az are 16 bit gyro units*/
  if (v_az > 32767)
    v_az = 32767;
  if (v_az < -32768)
    v_az = -32768;
  WriteData(velReqAzAddr, 32768 + v_az, NIOS_QUEUE);

  v_rw = calcVRW();

  if (TxIndex == 0)  //only write at slow frame rate
    WriteData(velRWAddr, v_rw*(65535.0/2400.0) + 32768.0, NIOS_QUEUE);

  if ((CommandData.disable_az) || (wait > 0)) {
    azGainP = 0;
    azGainI = 0;
    pivGainVelRW = 0;
    pivGainVelAz = 0;
    pivGainPosRW = 0;
    pivGainTorqueRW = 0;
    pivGainVelReqAz = 0;
    v_piv=GetVPivot(TxIndex,pivGainPosRW,pivGainVelRW,pivGainVelAz,
		     pivGainTorqueRW,pivGainVelReqAz,1);
  } else {
    azGainP = CommandData.azi_gain.P;
    azGainI = CommandData.azi_gain.I;
    pivGainVelRW = CommandData.pivot_gain.V_RW;
    pivGainVelAz = CommandData.pivot_gain.V_AZ;
    pivGainPosRW = CommandData.pivot_gain.P_RW;
    pivGainTorqueRW = CommandData.pivot_gain.T_RW;
    pivGainVelReqAz = CommandData.pivot_gain.V_REQ;
    v_piv=GetVPivot(TxIndex,pivGainPosRW,pivGainVelRW,pivGainVelAz,
		     pivGainTorqueRW,pivGainVelReqAz,0);
  }
  
  /* Even if az drive is disabled, write non-zero values of gains
   * to frame so that we can see what they are */
  pivGainVelRW = CommandData.pivot_gain.V_RW;
  pivGainVelAz = CommandData.pivot_gain.V_AZ;
  pivGainPosRW = CommandData.pivot_gain.P_RW;
  pivGainTorqueRW = CommandData.pivot_gain.T_RW;
  pivGainVelReqAz = CommandData.pivot_gain.V_REQ;
 
  /* requested pivot current*/
  WriteData(dacPivAddr, v_piv, NIOS_QUEUE);

  if (TxIndex == 0) { //only write at slow frame rate
    /* p term for az motor */
    WriteData(gPAzAddr, azGainP, NIOS_QUEUE);
    /* I term for az motor */
    WriteData(gIAzAddr, azGainI, NIOS_QUEUE);
    /* pointing gain term for az drive */
    WriteData(gPtAzAddr, CommandData.azi_gain.PT, NIOS_QUEUE);

    /* p term to rw vel for pivot motor */
    WriteData(gVRWPivAddr, pivGainVelRW, NIOS_QUEUE);
    /* p term to az vel for pivot motor */
    WriteData(gVAzPivAddr, pivGainVelAz, NIOS_QUEUE);
    /* i term to rw vel for pivot motor */
    WriteData(gIRWPivAddr, pivGainPosRW, NIOS_QUEUE);
    /* p term to rw torque for pivot motor */
    WriteData(gTRWPivAddr, pivGainTorqueRW, NIOS_QUEUE);
    /* p term to az vel request for pivot motor */
    WriteData(gVReqAzPivAddr, pivGainVelReqAz, NIOS_QUEUE);
    /* setpoint for reaction wheel */
    WriteData(setRWAddr, CommandData.pivot_gain.SP*32768.0/500.0, NIOS_QUEUE);
    /* Pivot velocity */
    WriteData(velCalcPivAddr, (calcVPiv()/20.0*32768.0), NIOS_QUEUE);
    /* Azimuth Scan Acceleration */
    WriteData(accelAzAddr, (CommandData.az_accel/2.0*65536.0), NIOS_QUEUE);
    /* Azimuth Scan Max Acceleration */
    WriteCalData(accelMaxAzAddr, CommandData.az_accel_max, NIOS_QUEUE);
  }

  if (wait > 0)
    wait--;
}

#if 0
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
#endif

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/
#define MIN_SCAN 0.1
static void SetAzScanMode(double az, double left, double right, double v,
    double D)
{
    double az_accel_dv = (CommandData.az_accel)/(ACSData.bbc_rate);
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
        axes_mode.az_vel += az_accel_dv;
    } else if (az > right) {
      //isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > -v + D)
        axes_mode.az_vel -= az_accel_dv;
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
    before_trig = (DELAY/ACSData.bbc_rate) - v/CommandData.az_accel 
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
  double centre, left, right, v_az=0.0, a_az;//, top, bottom, v_az;
  double v_az_max, ampl, turn_around;
  double az_accel;
  double az_accel_dv;
  int i_point;
  double t_before; // time at which to send BSC trigger command
  static double last_v = 0.0;
  
  t_before = (DELAY/ACSData.bbc_rate) + CommandData.theugly.expTime/2000.0;
 
  az_accel = CommandData.az_accel;
  az_accel_dv = az_accel/(ACSData.bbc_rate);

  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = CommandData.pointing_mode.Y;
  axes_mode.el_vel = 0.0;

  i_point = GETREADINDEX(point_index);
  
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  centre = CommandData.pointing_mode.X;
  ampl = (CommandData.pointing_mode.w)/2.0;

  SetSafeDAz(az, &centre); // don't cross the sun between here and centre
    
  right = centre + ampl;
  
  left = centre - ampl;

  /*SetSafeDAz(az, &left);     // don't cross sun between here and left
  SetSafeDAz(left, &right);  // don't cross sun between left and right
 
  if (right < left) {
    left -= 360.0;
  }*/

  v_az_max = sqrt(az_accel * ampl);
  
  // |distance| from end point when V_req = V_AZ_MIN
  turn_around = ampl*(1 - sqrt(1-(V_AZ_MIN*V_AZ_MIN)/(v_az_max*v_az_max)));

  // This should never ever matter.  MIN_SCAN = 0.1 deg
  if (right-left < MIN_SCAN) {
    left = centre - MIN_SCAN/2.0; 
    right = left + MIN_SCAN;
    ampl = right - centre;
  }

  axes_mode.az_mode = AXIS_VEL; // applies in all cases below:

  /* case 1: moving into scan from beyond left endpoint: */
  if (az < left - CommandData.pointing_mode.overshoot_band) {
    v_az = sqrt(2.0*az_accel*(left - CommandData.pointing_mode.overshoot_band - az)) + V_AZ_MIN;
    a_az = -az_accel; 
    //a_az = 0.0;
    if (v_az > v_az_max) {
      v_az = v_az_max;
      a_az = 0.0;
    }
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;
  
  /* case 2: moving into quad from beyond right endpoint: */
  } else if (az > right + CommandData.pointing_mode.overshoot_band) {
    v_az = -sqrt(2.0*az_accel*(az-(right+CommandData.pointing_mode.overshoot_band))) - V_AZ_MIN;
    a_az = az_accel;
    //a_az = 0.0;
    if (v_az < -v_az_max) {
      v_az = -v_az_max;
      a_az = 0.0;
    } 
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 3: moving from left to right endpoints */
  } else if ( (az > (left+turn_around)) && (az < (right-turn_around)) 
             && ((PointingData[i_point].v_az
                  +PointingData[i_point].offset_ifyaw_gy) > 0.0 ) ) {
             //&& (PointingData[i_point].v_az > V_AZ_MIN) ) {
    v_az = sqrt(az_accel*ampl)*sin(acos((centre-az)/ampl));
    a_az = az_accel*( (centre - az)/ampl ); 

    // star camera trigger (lemur?)
    if (az >= (right + 
        ampl*(cos(sqrt(az_accel/ampl)*t_before) - 1.0))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 4: moving from right to left endpoints */
  } else if ( (az > (left+turn_around)) && (az < (right-turn_around)) 
              && ( (PointingData[i_point].v_az
	            + PointingData[i_point].offset_ifyaw_gy) < 0.0 ) ) {
              //&& (PointingData[i_point].v_az < -V_AZ_MIN) ) {
    v_az = sqrt(az_accel*ampl)*sin(-acos((centre-az)/ampl)); 
    a_az = az_accel*( (centre - az)/ampl );
  
    // star camera trigger (lemur?)
    if (az <= (left + 
        ampl*(1.0 - cos(sqrt(az_accel/ampl)*t_before)))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }

    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 5: in left turn-around zone */ 
  } else if ( az <= left+turn_around ) {
    //v_az = V_AZ_MIN;
    v_az = last_v + az_accel_dv;
    if (v_az > V_AZ_MIN) {
      v_az = V_AZ_MIN;
    }
    a_az = az_accel*( (centre - az)/ampl );
    
    axes_mode.az_vel = v_az; 
    axes_mode.az_accel = a_az;
    
  /* case 6: in right turn-around zone */   
  } else if (az >= right-turn_around) {
    //v_az = -V_AZ_MIN;
    v_az = last_v - az_accel_dv;
    if (v_az < -V_AZ_MIN) {
      v_az = -V_AZ_MIN;
    }
    a_az = az_accel*( (centre - az)/ampl );
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;
  }

  last_v = v_az;
}

/* JAS - "modified quad" scan mode for Spider */
static void DoSpiderMode(void)
{
  double ra_start, dec_start;
  double az_start, el_start;
  double az, el;
  double lst, lat;
  double c_az[4], c_el[4]; // corner az and corner el
  double a_az; // requested acceleration based on position
  static double last_v = 0.0;
  
  /* parameters of "instantaneous" sinusoidal az scan: */
  double centre, left, right, top, bottom, v_az;
  double az_of_bot;
  double v_az_max, ampl, turn_around;
  double az_accel;
  double az_accel_dv;
  
  double t_before; // time at which to send BSC trigger command
  double t_step;   // amount time (s) before turn-around at which el micro-step is  
                   // commanded
  int i, i_point;
  int N_scans; // number of azimuth half-scans per el step
 
  static int past_step_point = 0;
  static int past_step_point_last = 0;
  
  static int n_scans = 0; // number of azimuth half-scans elapsed;
  
  static int in_scan = 0;
  
  /*pre-step el destination: used for computing box endpoints during each half-scan*/
  
  static double el_dest_pre = 37.5; // arbitrary
  
  t_before = (DELAY/ACSData.bbc_rate) + CommandData.theugly.expTime/2000.0;
  
  t_step = 1.2 + 0.5; // "on_delay" in GetIElev + (1/2)*(step duration) 
  
  az_accel = CommandData.az_accel;
  az_accel_dv = az_accel/(ACSData.bbc_rate);
  
  N_scans = CommandData.pointing_mode.Nscans;
  
  ra_start = CommandData.pointing_mode.X;
  dec_start = CommandData.pointing_mode.Y;

  i_point = GETREADINDEX(point_index);
  
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;
  
  /* convert ra/decs to az/el */
  for (i = 0; i < 4; i++) {
    radec2azel(CommandData.pointing_mode.ra[i],CommandData.pointing_mode.dec[i],
               lst, lat, c_az+i, c_el+i);
  }
  
  radec2azel(ra_start, dec_start, lst, lat, &az_start, &el_start);

  if (CommandData.pointing_mode.new_spider) {
    n_scans = 0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = el_dest_pre = el_start;
    axes_mode.el_vel = 0.0;
    CommandData.pointing_mode.del = CommandData.pointing_mode.el_step;
    CommandData.pointing_mode.new_spider = 0;
  }
   
  radbox_endpoints(c_az, c_el, el_dest_pre, &left, &right, &bottom, &top, &az_of_bot);
  
  /* Rigmarole to make sure we don't cross the sun between here and scan centre */
 
  if (left > right) {
    left -=360.0;
  }

  ampl = (right - left) / 2.0;
  centre = (left + right) / 2.0;
  
  SetSafeDAz(az, &centre);
  
  left = centre - ampl;
  right = centre + ampl;
  
  // This should never ever matter.  MIN_SCAN = 0.1 deg
  if (right-left < MIN_SCAN) {
    left = centre - MIN_SCAN/2.0; 
    right = left + MIN_SCAN;
    ampl = right - centre;
  }

  /*if ( (PointingData[i_point].sun_az > left) && 
       (PointingData[i_point].sun_az < right) ) {

    bprintf(err, "The sun is inside the scan region! Stopping gondola.");

    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = 0;
    CommandData.pointing_mode.vaz = 0.0;
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;
    CommandData.pointing_mode.mode = P_DRIFT;
    
    return;

  }
*/

  v_az_max = sqrt(az_accel * ampl);
  
  
  // |distance| from end point when V_req = V_AZ_MIN
  turn_around = ampl*(1 - sqrt(1-(V_AZ_MIN*V_AZ_MIN)/(v_az_max*v_az_max)));

 

  axes_mode.az_mode = AXIS_VEL; // applies in all cases below:

  /* case 1: moving into scan from beyond left endpoint: */
  if (az < left - CommandData.pointing_mode.overshoot_band) {
    in_scan = 0;
    scan_region = SCAN_BEYOND_L;
    scan_region_last = scan_region;
    v_az = sqrt(2.0*az_accel*(left - CommandData.pointing_mode.overshoot_band - az)) + V_AZ_MIN;
    a_az = -az_accel; 
    //a_az = 0.0;
    if (v_az > v_az_max) {
      v_az = v_az_max;
      a_az = 0.0;
    }
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;
  
  /* case 2: moving into quad from beyond right endpoint: */
  } else if (az > right + CommandData.pointing_mode.overshoot_band) {
    scan_region = SCAN_BEYOND_R;
    scan_region_last = scan_region;
    v_az = -sqrt(2.0*az_accel*(az-(right+CommandData.pointing_mode.overshoot_band))) - V_AZ_MIN;
    a_az = az_accel;
    //a_az = 0.0;
    if (v_az < -v_az_max) {
      v_az = -v_az_max;
      a_az = 0.0;
    } 
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 3: moving from left to right endpoints */
  } else if ( (az > (left+turn_around)) && (az < (right-turn_around)) 
             && ( (PointingData[i_point].v_az
	           +PointingData[i_point].offset_ifyaw_gy)  > 0.0) ) {
             //&& (PointingData[i_point].v_az > V_AZ_MIN) ) {
    scan_region = SCAN_L_TO_R;
 
  
    if ( ((az - left) > ampl/10.0) && ( (az-left) < (ampl/10.0 + 1.0) ) ) { 
      // ampl/10 deg. in from left turn-around
      el_dest_pre = axes_mode.el_dest;
    }
  
    scan_region_last = scan_region;	       
    v_az = sqrt(az_accel*ampl)*sin(acos((centre-az)/ampl));
    a_az = az_accel*( (centre - az)/ampl ); 

    // star camera trigger (lemur?)
    if (az >= (right + 
        ampl*(cos(sqrt(az_accel/ampl)*t_before) - 1.0))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
    
    // start el step at position corresponding to t_step seconds before turn-around
    if (az >= (right + ampl*(cos(sqrt(az_accel/ampl)*t_step) - 1.0))) {
      past_step_point = 1;
      if ( past_step_point_last == 0 ) {
        n_scans++;
	if ( (n_scans % N_scans) == 0 ) { // step in elevation
          bprintf(info, "stepping in el at right turn around");
          axes_mode.el_mode = AXIS_POSITION;
          axes_mode.el_dest = el_start + (n_scans/N_scans)*CommandData.pointing_mode.del;
          axes_mode.el_vel = 0.0;
        }  
      }
      past_step_point_last = past_step_point;
      //bprintf(info, "elapsed half-scans (n)=%d, half-scans per el step (N)=%d, (n mod N)=%d", n_scans, N_scans, (n_scans%N_scans));
    }
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 4: moving from right to left endpoints */
  } else if ( (az > (left+turn_around)) && (az < (right-turn_around)) 
              && ( (PointingData[i_point].v_az
		    +PointingData[i_point].offset_ifyaw_gy) < 0.0) ) {
              //&& (PointingData[i_point].v_az < -V_AZ_MIN) ) {
    scan_region = SCAN_R_TO_L;
  
    if ( ((right - az) > ampl/10.0) && ((right - az) < (ampl/10.0 + 1.0))  ) {
      // ampl/10 degrees in from right turn-around
      el_dest_pre = axes_mode.el_dest;
    }
  
    scan_region_last = scan_region;
    v_az = sqrt(az_accel*ampl)*sin(-acos((centre-az)/ampl)); 
    a_az = az_accel*( (centre - az)/ampl );
  
    // star camera trigger (lemur?)
    if (az <= (left + 
        ampl*(1.0 - cos(sqrt(az_accel/ampl)*t_before)))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
    
    // start el step at position corresponding to t_step seconds before turn-around
    if (az <= (left + ampl*(1.0 - cos(sqrt(az_accel/ampl)*t_step))) ) {
      past_step_point = 1;
      if ( past_step_point_last == 0 ) {
        n_scans++;
	
	if ( (n_scans % N_scans) == 0 ) { // step in elevation
          bprintf(info, "stepping in el at left turn around");
          axes_mode.el_mode = AXIS_POSITION;
          axes_mode.el_dest = el_start + (n_scans/N_scans)*CommandData.pointing_mode.del;
          axes_mode.el_vel = 0.0;
        }
        
      }
      past_step_point_last = past_step_point;
     
      //bprintf(info, "elapsed half-scans (n)=%d, half-scans per el step (N)=%d, (n mod N)=%d", n_scans, N_scans, (n_scans%N_scans));
    
    }
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 5: in left turn-around zone */ 
  } else if ( az <= left+turn_around ) {
    scan_region = SCAN_L_TURN;
    scan_region_last = scan_region;

    past_step_point = 0; // reset for the next half-scan
    past_step_point_last = past_step_point;
    //if (scan_region_last == SCAN_R_TO_L) {
    //  n_scans++;
    //  bprintf(info, "elapsed half-scans (n)=%d, half-scans per el step (N)=%d, (n mod N)=%d", n_scans, N_scans, (n_scans%N_scans));
    //  if ( (n_scans % N_scans) == 0 ) { // step in elevation
    //    bprintf(info, "stepping in el at left turn around");
    //    axes_mode.el_mode = AXIS_POSITION;
     //   axes_mode.el_dest = el_start + (n_scans/N_scans)*CommandData.pointing_mode.del;
      //  axes_mode.el_vel = 0.0;
      //}
    //}
    
    v_az = last_v + az_accel_dv;
    if (v_az > V_AZ_MIN) {
      v_az = V_AZ_MIN;
    }
    a_az = az_accel*( (centre - az)/ampl );
    
    axes_mode.az_vel = v_az; 
    axes_mode.az_accel = a_az;
    
  /* case 6: in right turn-around zone */   
  } else if (az >= right-turn_around) {
    scan_region = SCAN_R_TURN;
    scan_region_last = scan_region;
    
    past_step_point = 0; // reset for the next half-scan
    past_step_point_last = past_step_point;

   // if (scan_region_last == SCAN_L_TO_R) {
  //    n_scans++;
//       bprintf(info, "elapsed half-scans (n)=%d, half-scans per el step (N)=%d, (n mod N)=%d", n_scans, N_scans, (n_scans%N_scans));
     // if ( (n_scans % N_scans) == 0 ) { // step in elevation
       // bprintf(info, "stepping in el at right turn around");
       // axes_mode.el_mode = AXIS_POSITION;
      //  axes_mode.el_dest = el_start + (n_scans/N_scans)*CommandData.pointing_mode.del;
     //   axes_mode.el_vel = 0.0;
      //}
    //}
    
    v_az = last_v - az_accel_dv;
    if (v_az < -V_AZ_MIN) {
      v_az = -V_AZ_MIN;
    }
    a_az = az_accel*( (centre - az)/ampl );
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;
  }

  if (n_scans >= N_scans*CommandData.pointing_mode.Nsteps) {
    bprintf(info, "Number of scans completed: %d. Resetting elevation\n",
	    n_scans); 
    n_scans = 0;
  }

  last_v = v_az;

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

#if 0
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
#endif

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
#if 0
static void DoNewCapMode(void)
{
  double caz, cel, r, x2, y, xw;
  double bottom, top, left, right;
  double next_left, next_right, az_distance;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  double az_accel;
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
  
  az_accel = CommandData.az_accel; 

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
      t = az_distance/v_az + 2.0*v_az/az_accel;
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/az_accel;
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
  double az_accel;
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

  az_accel = CommandData.az_accel;
  
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
      t = w/v_az + 2.0*v_az/az_accel;
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      t = w/v_az + 2.0*v_az/az_accel;
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
  double az_accel;
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

  az_accel = CommandData.az_accel;
  
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
      t = az_distance/v_az + 2.0*v_az/az_accel;
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/az_accel;
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
#endif

/******************************************************************
 *                                                                *
 * Update Axis Modes: Set axes_mode based on                      *
 *    CommandData.pointing_mode                                   *
 *                                                                *
 ******************************************************************/
void UpdateAxesMode(void)
{
  axes_mode.az_accel = 0.0; // default for scan modes that don't set
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
 /*   case P_VCAP:
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
      break;*/
    case P_RADEC_GOTO:
      DoRaDecGotoMode();
      break;
  /*  case P_QUAD: // aka radbox
      DoQuadMode();
      break;*/
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

  rw_motor_index = 1; 

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
    if(CommandData.restore_rw==1 ) { 
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
	tmp = queryAMCInd(3, 2, 1, &reactinfo);
	bprintf(info, "Sys. Protect status bitfield = 0x%04x", tmp);
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
        //RWMotorData[rw_motor_index].current=((double)current_raw)/8192.0*60.0;
	// TODO: changed peak drive current to 20 A since we are 
	//       using smaller controller for RW temporarily
	RWMotorData[rw_motor_index].current=(((double)current_raw)/8192.0)*60.0;
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
	tmp = queryAMCInd(3, 2, 1, &pivotinfo);
	bprintf(info, "Sys. Protect status bitfield = 0x%04x", tmp);
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
