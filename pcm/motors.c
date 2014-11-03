/* pcm: the Spider master control program
 *
 * This software is copyright (C) 2002-2013 University of Toronto
 *
 * This file is part of pcm.
 *
 * pcm is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * pcm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pcm; if not, write to the Free Software Foundation, Inc.,
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
#define MIN_EL 14 
#define MAX_EL 45

#define VPIV_FILTER_LEN 40
#define FPIV_FILTER_LEN 1000

#define RW_BASE 0.95    // base for exponential filter used to compute RW
                        // speed
#define V_AZ_MIN 0.05   //smallest measured az speed we trust given gyro
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
#define TOLERANCE 0.02    // max acceptable el pointing error (deg)
#define TWIST_TOL 0.5     // max acceptable port-strbrd enc diff (deg)
#define EL_REL_MIN 0.005  // min size of el relative move (= 1 count)
#define EL_ON_DELAY 0.40  // turn-on delay of el drive when in auto mode

/* Structures containing AMC controller status info and file descriptors */
struct MotorInfoStruct reactinfo;
struct MotorInfoStruct pivotinfo;

/* variables storing scan region (relative to defined quad) */
static int scan_region = 0;
static int scan_region_last = 0;

/* pre-step el destination: used for computing box endpoints during each 
 * half-scan in Spider Scan mode */
static double el_dest_pre = 37.5; // arbitrary

/* number of azimuth half-scans elapsed in Spider Scan or Sine Scan modes */
static int n_scans = 0;

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
#define REACT_DEVICE "/dev/ttySI13"
#define PIVOT_DEVICE "/dev/ttySI9"

static void* reactComm(void *arg);
static void* pivotComm(void *arg);


static double dxdtheta(double theta);

extern short int InCharge; /* tx.c */

extern int StartupVeto; /* mcp.c */

extern short int bsc_trigger; /* Semaphore for BSC trigger */

/* opens communications with motor controllers */
void openMotors()
{
  //bprintf(info, "Motors: connecting to motors");
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

static void SetVElevRelMove(double d, double v0, double* a, int* count1, int*
		            count2, int* count3)
{

  /* Rel move  has three phases:
     t = 0 to t = t1: constant acceleration to crusing speed
     t = t1 to t = t2: crusing at constant speed v0
     t = t2 to t = t3: constant deceleration to a stop */
	
  double t1, t2, t3;
  
  *a = (v0*v0)/EL_REL_MIN;

  t1 = v0/(*a);
  
  if ( (d < EL_REL_MIN) || (v0 < EL_REL_MIN) ) {
    t1 = t2 = t3 = 0.0;
  } else {

    t2 = t1 + (d-EL_REL_MIN)/v0;

    t3 = t2 + v0/(*a);
  }

  *count1 = t1*ACSData.bbc_rate;
  *count2 = t2*ACSData.bbc_rate;
  *count3 = t3*ACSData.bbc_rate;

}

static void GetVElevRelMove(double* v_P, double* v_S)
{
  static int count_p = 0;
  static int count_s = 0;

  int count_p_1, count_p_2, count_p_3;  
  int count_s_1, count_s_2, count_s_3;  

  double a_s, a_p;

  SetVElevRelMove(fabs(CommandData.pointing_mode.d_el_p), 
		  fabs(CommandData.pointing_mode.v_el_p), &a_p, &count_p_1, 
		  &count_p_2, &count_p_3);
  
  SetVElevRelMove(fabs(CommandData.pointing_mode.d_el_s), 
		  fabs(CommandData.pointing_mode.v_el_s), &a_s, &count_s_1, 
		  &count_s_2, &count_s_3);

  if (CommandData.pointing_mode.el_rel_move) {
    count_p = count_p_3;
    count_s = count_s_3;
    CommandData.pointing_mode.el_rel_move = 0;
  }
 
  if (count_p > 0) { 
    
    if ( (count_p_3 - count_p) < count_p_1 ) {
      if (CommandData.pointing_mode.d_el_p < 0) {
        *v_P -= a_p/ACSData.bbc_rate;
      } else {
        *v_P += a_p/ACSData.bbc_rate;
      }
    } else if ( (count_p_3 - count_p) < count_p_2 ) {
      if (CommandData.pointing_mode.d_el_p < 0) {
        *v_P = -fabs(CommandData.pointing_mode.v_el_p); 
      } else {
        *v_P = fabs(CommandData.pointing_mode.v_el_p); 
      }
    } else if ( (count_p_3 - count_p) < count_p_3 ) {
      if (CommandData.pointing_mode.d_el_p < 0) {
        *v_P += a_p/ACSData.bbc_rate;
      } else {
        *v_P -= a_p/ACSData.bbc_rate;
      }
    } else {
      *v_P = 0.0;
    }
    count_p--; 
  }
  if (count_s > 0) { 
    
    if ( (count_s_3 - count_s) < count_s_1 ) {
      if (CommandData.pointing_mode.d_el_s < 0) {
        *v_S -= a_s/ACSData.bbc_rate;
      } else {
        *v_S += a_s/ACSData.bbc_rate;
      }
    } else if ( (count_s_3 - count_s) < count_s_2 ) {
      if (CommandData.pointing_mode.d_el_s < 0) {
        *v_S = -fabs(CommandData.pointing_mode.v_el_s); 
      } else {
        *v_S = fabs(CommandData.pointing_mode.v_el_s); 
      }
    } else if ( (count_s_3 - count_s) < count_s_3 ) {
      if (CommandData.pointing_mode.d_el_s < 0) {
        *v_S += a_s/ACSData.bbc_rate;
      } else {
        *v_S -= a_s/ACSData.bbc_rate;
      }
    } else {
      *v_S = 0.0;
    }
    count_s--; 
  }

  if ( (count_p == 0) && (count_s == 0) ) {
    CommandData.pointing_mode.el_mode = P_EL_NONE;
  }  
}

/*************************************************************************

    SetVElev: Set elevation drive velocity request using gain terms 
               This is just a utility called by GetVElev.

    NEW in Spider!
*************************************************************************/
static double SetVElev(double g_com, double dy,
                       double v_last, double max_dv, double enc) 
{

  double v, v_max;

  v_max = fabs( ((double)MAX_STEP*IN_TO_MM) / 
	     ((double)CM_PULSES*EL_GEAR_RATIO*ROT_PER_INCH*dxdtheta(enc)) );
  
  if (dy >= 0) {
    v = g_com*sqrt(dy);
    
    if (v > v_max) {
      v = v_max;
    }
  
  } else {
    v = -g_com*sqrt(-dy);
    
    if (v < -v_max) {
      v = -v_max;
    } 
    
  }
/* don't increase/decrease request by more than max_dv: */
  v = ((v - v_last) > max_dv) ? (v_last + max_dv) : v;
  v = ((v - v_last) < -max_dv) ? (v_last - max_dv) : v;    

  return v;

}

static void GetVElevGoto(double* v_P, double* v_S)
{

// S = STARBOARD
// P = PORT

/* various dynamical variables */
  double enc_port, enc_strbrd;
  double el_dest;
  double dy;

  double max_dv;
  double g_com;

  int isStopped = 1;

/* requested velocities (prev. values) */
  static double v_S_last = 0.0;
  static double v_P_last = 0.0;  
  
  static int since_arrival = 0;

  /* check limits (shouldn't be possible to command out of bounds el in cow
   * though) */
  if (axes_mode.el_dest > MAX_EL) {
    el_dest = axes_mode.el_dest = CommandData.pointing_mode.Y = MAX_EL;
  } else if (axes_mode.el_dest < MIN_EL) {
    el_dest = axes_mode.el_dest = CommandData.pointing_mode.Y = MIN_EL;
  } else {
    el_dest = axes_mode.el_dest;
  }

  /* port = sum/2 + diff/2 */
  enc_port = ACSData.enc_mean_el + ACSData.enc_diff_el/2.0;
  enc_strbrd = ACSData.enc_mean_el - ACSData.enc_diff_el/2.0;
  
  dy = el_dest - ACSData.enc_mean_el;

  g_com = CommandData.ele_gain.com * (double)(fabs(dy)>TOLERANCE); 
  max_dv = 1.05 * CommandData.ele_gain.com*CommandData.ele_gain.com 
	   * (1.0/(2.0*ACSData.bbc_rate));  // 5% higher than deceleration...

  *v_P = SetVElev(g_com, dy, v_P_last, max_dv, enc_port);
  *v_S = SetVElev(g_com, dy, v_S_last, max_dv, enc_strbrd);

  /* set both speeds equal to the minimum of the two */
  *v_S = (*v_S < *v_P) ? *v_S : *v_P;
  *v_P = *v_S;

  if ( fabs(el_dest - ACSData.enc_mean_el ) < TOLERANCE ) { 
    since_arrival++;
    isStopped = (since_arrival >= 2*ACSData.bbc_rate);
  }  else {
    since_arrival = 0;
    isStopped = 0;
  }
  /* check for a stalled motor */
  if ( fabs(ACSData.enc_diff_el-CommandData.twist_default) > CommandData.twist_limit) {
    isStopped = 1;
    el_dest = axes_mode.el_dest = CommandData.pointing_mode.Y
            = ACSData.enc_mean_el;
    CommandData.power.elmot_auto = 2;
  } else {
    isStopped = 0;
  }

  if (isStopped) { // stop servoing
    CommandData.pointing_mode.el_mode = P_EL_NONE;
  }

  v_P_last = *v_P; 
  v_S_last = *v_S; 

} 

/************************************************************************/
/*                                                                      */
/*   GetVElev: get the current elevation velocity request, given current*/
/*   pointing mode, etc..                                               */
/*                                                                      */
/************************************************************************/
static void GetVElev(double* v_P, double* v_S)
{
  static int on_delay = 0;
  static int was_not_auto = 1;

  if (was_not_auto && (CommandData.power.elmot_auto)) {
    on_delay = 0;
  }
  was_not_auto = !CommandData.power.elmot_auto;

  if ( !(CommandData.power.elmot_auto) ) {
    on_delay = EL_ON_DELAY*ACSData.bbc_rate + 2.0; // 1st factor is delay 
                                                   // in seconds 
  }
  
  // if !P_EL_NONE and auto and off
  //   Turn On
  if ( (CommandData.pointing_mode.el_mode != P_EL_NONE) 
      && (CommandData.power.elmot_auto) ) {
    on_delay++;
    if ( !(CommandData.power.elmot_is_on) ) { 
      CommandData.power.elmot.rst_count = 0;
      CommandData.power.elmot.set_count = LATCH_PULSE_LEN;  
    }
  }
             
  if ( (CommandData.pointing_mode.el_mode == P_EL_GOTO) && 
        (on_delay >= EL_ON_DELAY*ACSData.bbc_rate) ) {
    GetVElevGoto(v_P, v_S);
  } else if ( (CommandData.pointing_mode.el_mode == P_EL_RELMOVE) && 
              (on_delay >= EL_ON_DELAY*ACSData.bbc_rate) ) {
    GetVElevRelMove(v_P, v_S);
  } else {
    *v_P = *v_S = 0.0;
    // if we are auto and on and if P_EL_NONE
    //   turn off
    if ( (CommandData.power.elmot_is_on) && (CommandData.power.elmot_auto) 
        && (CommandData.pointing_mode.el_mode == P_EL_NONE) ) {      
      CommandData.power.elmot.set_count = 0;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
      if (CommandData.power.elmot_auto == 2) {
        CommandData.power.elmot_auto = 0;
      }
      on_delay = 0; // /eset for the next time we turn on
    }
  }
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

  /*vel_offset = -(PointingData[i_point].offset_ofroll_gy 
               - PointingData[i_point].ifroll_earth_gy)
               * cos(PointingData[i_point].el * M_PI / 180.0) 
               -(PointingData[i_point].offset_ofyaw_gy 
               - PointingData[i_point].ifyaw_earth_gy)
               * sin(PointingData[i_point].el * M_PI / 180.0);*/

  // gyros are on the outer frame for Spider...
 
//  vel_offset =-(PointingData[i_point].offset_ofyaw_gy 
    //            - PointingData[i_point].ifyaw_earth_gy);

  // TODO: signs above appear to be wrong???
  vel_offset = (PointingData[i_point].offset_ofyaw_gy 
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
/*            (and other things). NOTE: only used when running the piv  */
/*            in velocity mode                                          */
/*                                                                      */
/************************************************************************/
static double GetVPivot(int write_slow, unsigned int gP_v_rw,  
                        unsigned int gP_t_rw, unsigned int gP_v_req, 
                        unsigned int disabled)
{
  static struct NiosStruct* pVRWTermPivAddr;
  static struct NiosStruct* pTRWTermPivAddr;
  static struct NiosStruct* pVReqAzTermPivAddr;
  
  static struct BiPhaseStruct* dacRWAddr;

  double v_req = 0.0;
  int v_req_dac = 0;
  double P_v_rw_term, P_t_rw_term, P_v_req_term;
  int P_v_rw_term_dac, P_t_rw_term_dac,
      P_v_req_term_dac;
      
  unsigned short int dac_rw;

  static unsigned int firsttime = 1;

  static double int_v_rw = 0.0;  // integrated reaction wheel speed

  double a = 0.9998;

  if(firsttime) {
    pVRWTermPivAddr = GetNiosAddr("term_p_v_rw_piv");
    pTRWTermPivAddr = GetNiosAddr("term_p_t_rw_piv");
    pVReqAzTermPivAddr = GetNiosAddr("term_p_v_req_az_piv");
    
    dacRWAddr = GetBiPhaseAddr("dac_rw");
    
    firsttime = 0;
  }

  int_v_rw = (1.0-a)*ACSData.vel_rw + a*int_v_rw; 

  dac_rw = ReadCalData(dacRWAddr);

  /* Calculate control terms */
  P_v_rw_term = -( ((double) gP_v_rw)/1000.0 )*(ACSData.vel_rw - CommandData.pivot_gain.SP);
  P_t_rw_term = -( ((double)gP_t_rw)/1000.0 )*((double)(dac_rw-32768)); 
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
  v_req = P_v_rw_term + P_t_rw_term + P_v_req_term;

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

  if(P_v_req_term_dac <= 0) {
    P_v_req_term_dac=1;
  }
  if(P_v_req_term_dac > 65535) {
    P_v_req_term_dac=65535;
  }

  /* Write control terms to frame, but slowly... */
  if (write_slow) {
      WriteData(pVRWTermPivAddr, P_v_rw_term_dac, NIOS_QUEUE);
      WriteData(pVReqAzTermPivAddr, P_v_req_term_dac, NIOS_QUEUE);
      WriteData(pTRWTermPivAddr, P_t_rw_term_dac, NIOS_QUEUE);
  }
  
  return v_req_dac;
}

/************************************************************************/
/*                                                                      */
/*     GetIPivot: get the current request for the pivot in DAC units    */
/*       Proportional to the reaction wheel speed error. NOTE: used only*/
/*       when running piv in torque mode                                */
/*                                                                      */
/************************************************************************/
static double GetIPivot(int v_az_req_gy, unsigned int g_rw_piv, 
                        unsigned int g_err_piv, double frict_off_piv, 
                        unsigned int disabled)
{
  static struct NiosStruct* pRWTermPivAddr;
  static struct NiosStruct* pErrTermPivAddr;
  static struct NiosStruct* frictTermPivAddr;
  static double buf_frictPiv[FPIV_FILTER_LEN]; // Buffer for Piv friction 
                                               // term boxcar filter.
  static double a=0.0; 
  static unsigned int ib_last=0;
  double I_req = 0.0;
  int I_req_dac = 0;
  double v_az_req,i_frict,i_frict_filt;
  double p_rw_term, p_err_term;
  int p_rw_term_dac, p_err_term_dac;
 
  static int i=0;
  static unsigned int firsttime = 1;

  if(firsttime) {
    pRWTermPivAddr = GetNiosAddr("term_p_rw_piv");
    pErrTermPivAddr = GetNiosAddr("term_p_err_piv");
    frictTermPivAddr = GetNiosAddr("term_frict_piv");

    // Initialize the buffer.  Assume all zeros to begin
    for(i=0;i<(FPIV_FILTER_LEN-1);i++) buf_frictPiv[i]=0.0;
    firsttime = 0;
  }

  v_az_req = ((double) v_az_req_gy) * GY16_TO_DPS/10.0; // Convert to dps 

  p_rw_term = (-1.0)*((double)g_rw_piv/10.0)
              *(ACSData.vel_rw-CommandData.pivot_gain.SP);
  p_err_term = (double)g_err_piv*5.0*(v_az_req-PointingData[point_index].v_az);
  I_req = p_rw_term+p_err_term;

  if(disabled) { // Don't attempt to send current to the motors if we are 
                 // disabled.
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
  return I_req_dac;
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
void WriteMot(int write_slow)
{
  static struct NiosStruct* velReqAzAddr;
  static struct NiosStruct* gComElAddr;
  static struct NiosStruct* twistDefaultAddr;
  static struct NiosStruct* twistLimitAddr;
  
  static struct NiosStruct* gPAzAddr;
  static struct NiosStruct* gIAzAddr;
  static struct NiosStruct* gPtAzAddr;

  /* pivot velocity mode gains */
  static struct NiosStruct* gVRWPivAddr;
  static struct NiosStruct* gTRWPivAddr;
  static struct NiosStruct* gVReqAzPivAddr;

  /* pivot torque mode gains */
  static struct NiosStruct* gPVPivAddr;
  static struct NiosStruct* gPEPivAddr;
  static struct NiosStruct* frictOffPivAddr;

  static struct NiosStruct* setRWAddr;
  static struct NiosStruct* dacPivAddr;
  static struct NiosStruct* accelAzAddr;
  static struct NiosStruct* accelMaxAzAddr;
  static struct NiosStruct* step1ElAddr;      // PORT
  static struct NiosStruct* step2ElAddr;      // STARBOARD
  static struct NiosStruct* isTurnAroundAddr;
  static struct NiosStruct* modePivAddr;
  static struct NiosStruct* modeElAddr;

  // Used only for Lab Controller tests
  static struct NiosStruct* dac2AmplAddr;

  static int wait = 100; /* wait 100 frames before controlling. */

  int v_az, v_piv=0, i_piv=0;
  double elGainCom;
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
   
  int azGainP, azGainI, pivGainVelRW, pivGainTorqueRW, pivGainVelReqAz, 
      pivGainRW, pivGainErr;
  double pivFrictOff;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    velReqAzAddr = GetNiosAddr("vel_req_az");
    dacPivAddr = GetNiosAddr("dac_piv");
    gComElAddr = GetNiosAddr("g_com_el");
    twistDefaultAddr = GetNiosAddr("twist_default");
    twistLimitAddr = GetNiosAddr("twist_limit");
    gPAzAddr = GetNiosAddr("g_p_az");
    gIAzAddr = GetNiosAddr("g_i_az");
    gPtAzAddr = GetNiosAddr("g_pt_az");

    /* pivot velocity mode gains */
    gVRWPivAddr = GetNiosAddr("g_v_rw_piv");
    gTRWPivAddr = GetNiosAddr("g_t_rw_piv");
    gVReqAzPivAddr = GetNiosAddr("g_v_req_az_piv");
    
    /* pivot torque mode gains */
    gPVPivAddr = GetNiosAddr("g_pv_piv");
    gPEPivAddr = GetNiosAddr("g_pe_piv");
    frictOffPivAddr = GetNiosAddr("frict_off_piv");

    setRWAddr = GetNiosAddr("set_rw");
    accelAzAddr = GetNiosAddr("accel_az");
    accelMaxAzAddr = GetNiosAddr("accel_max_az");
    dac2AmplAddr = GetNiosAddr("dac2_ampl");
    step1ElAddr = GetNiosAddr("step_1_el");
    step2ElAddr = GetNiosAddr("step_2_el");
    isTurnAroundAddr = GetNiosAddr("is_turn_around");
    modePivAddr = GetNiosAddr("mode_piv");
    modeElAddr = GetNiosAddr("mode_el");
  }

  //NOTE: this is only used to program the extra DAC - not used for
  // flight.
  if (write_slow) {  //only write at slow frame rate
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

  if (write_slow) {   //only write at slow frame rate
  
    elGainCom = CommandData.ele_gain.com;

    /*common-mode gain term for el motors*/
    WriteCalData(gComElAddr, elGainCom, NIOS_QUEUE);

    WriteCalData(twistDefaultAddr, CommandData.twist_default, NIOS_QUEUE);
    WriteCalData(twistLimitAddr, CommandData.twist_limit, NIOS_QUEUE);
  }

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = floor(GetVAz() + 0.5);
  
  /* Units for v_az are 16 bit gyro units*/
  if (v_az > 32767) {
    v_az = 32767;
  }
  
  if (v_az < -32768) {
    v_az = -32768;
  }
  
  WriteData(velReqAzAddr, 32768 + v_az, NIOS_QUEUE);
   
  if ((CommandData.disable_az) || (wait > 0)) {
    azGainP = 0;
    azGainI = 0;
    /* velocity mode gains */
    pivGainVelRW = 0;
    pivGainTorqueRW = 0;
    pivGainVelReqAz = 0;
    /* torque mode gains */
    pivGainRW = 0;
    pivGainErr = 0;
    pivFrictOff = 0.0;
    if (CommandData.pointing_mode.piv_mode == P_PIV_VEL) {
      v_piv=GetVPivot(write_slow,pivGainVelRW,pivGainTorqueRW,pivGainVelReqAz,
                      1);
    } else if (CommandData.pointing_mode.piv_mode == P_PIV_TORQUE) {
      i_piv=GetIPivot(0,pivGainRW,pivGainErr,pivFrictOff,1);
    }
  } else {
    azGainP = CommandData.azi_gain.P;
    azGainI = CommandData.azi_gain.I;
    if (CommandData.pointing_mode.piv_mode == P_PIV_VEL) {
      pivGainVelRW = CommandData.pivot_gain.V_RW;
      pivGainTorqueRW = CommandData.pivot_gain.T_RW;
      pivGainVelReqAz = CommandData.pivot_gain.V_REQ;
      v_piv=GetVPivot(write_slow,pivGainVelRW, pivGainTorqueRW,pivGainVelReqAz,
                      0);
    } else if (CommandData.pointing_mode.piv_mode == P_PIV_TORQUE) {
      pivGainRW = CommandData.pivot_gain.PV;
      pivGainErr = CommandData.pivot_gain.PE;
      pivFrictOff = CommandData.pivot_gain.F;
      i_piv=GetIPivot(v_az,pivGainRW,pivGainErr,pivFrictOff,0);
    } 
  }
  
  /* Even if az drive is disabled, write non-zero values of gains
   * to frame so that we can see what they are */
  pivGainVelRW = CommandData.pivot_gain.V_RW;
  pivGainTorqueRW = CommandData.pivot_gain.T_RW;
  pivGainVelReqAz = CommandData.pivot_gain.V_REQ;
  pivGainRW = CommandData.pivot_gain.PV;
  pivGainErr = CommandData.pivot_gain.PE;
  pivFrictOff = CommandData.pivot_gain.F;
 
  /* requested pivot velocity or current*/
  if (CommandData.pointing_mode.piv_mode == P_PIV_VEL) {
    WriteData(dacPivAddr, v_piv, NIOS_QUEUE);
  } else if (CommandData.pointing_mode.piv_mode == P_PIV_TORQUE) {
    WriteData(dacPivAddr, i_piv*2, NIOS_QUEUE);
  }

  // write informational terms once per slow frame, spread out.
  if (write_slow) {
      /* p term for az motor */
      WriteData(gPAzAddr, azGainP, NIOS_QUEUE);
      /* I term for az motor */
      WriteData(gIAzAddr, azGainI, NIOS_QUEUE);

      /************* piv vel mode terms **************/
      /* pointing gain term for az drive */
      WriteData(gPtAzAddr, CommandData.azi_gain.PT, NIOS_QUEUE);
      /* p term to rw vel for pivot motor */
      WriteData(gVRWPivAddr, pivGainVelRW, NIOS_QUEUE);
      /* p term to rw torque for pivot motor */
      WriteData(gTRWPivAddr, pivGainTorqueRW, NIOS_QUEUE);
      /* p term to az vel request for pivot motor */
      WriteData(gVReqAzPivAddr, pivGainVelReqAz, NIOS_QUEUE);

      /************* piv torque mode terms **************/
      /* p term to rw vel for pivot motor */
      WriteData(gPVPivAddr, pivGainRW, NIOS_QUEUE);
      /* p term to vel error for pivot motor */
      WriteData(gPEPivAddr, pivGainErr, NIOS_QUEUE);
      /* Pivot current offset to compensate for static friction. */
      WriteData(frictOffPivAddr, pivFrictOff/2.0*65535, NIOS_QUEUE);

      /* setpoint for reaction wheel */
      WriteData(setRWAddr, CommandData.pivot_gain.SP*32768.0/500.0, NIOS_QUEUE);
      /* Azimuth Scan Acceleration */
      WriteData(accelAzAddr, (CommandData.az_accel/2.0*65536.0), NIOS_QUEUE);
      /* Azimuth Scan Max Acceleration */
      WriteCalData(accelMaxAzAddr, CommandData.az_accel_max, NIOS_QUEUE);

      /* pivot drive control mode */
      WriteData(modePivAddr, CommandData.pointing_mode.piv_mode, NIOS_QUEUE);
      
      /* elevation power control mode */
      WriteData(modeElAddr, CommandData.power.elmot_auto || (CommandData.power.elmot_is_on<<2), NIOS_QUEUE);
  }
   
  /* Turn Around Flag */
  WriteData(isTurnAroundAddr, CommandData.pointing_mode.is_turn_around, NIOS_QUEUE);
  
  if (wait > 0)
    wait--;
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
    before_trig = (CommandData.bsc_delay/ACSData.bbc_rate) - v/CommandData.az_accel 
    + CommandData.StarCam[0].expTime/2000;
    if (az < left + before_trig*v) {
      bsc_trigger = 1;  
    } else if (az > right - before_trig*v) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
}

/* DoSineMode - low-level scan routine. Part of the implementation of both
 * SineScan and SpiderScan */
static void DoSineMode(double centre, double ampl, double el_start)
{
  /* scan parameters */
  double az;
  double az_accel;
  double az_accel_dv;
  double az_accel_max_dv = 0;
  double v_az_max;
  double left, right; 
  double turn_around;
  double v_az=0.0; // velocity request
  double a_az;     // requested acceleration based on position
  double t_before; // time at which to send BSC trigger command
  double t_step;   // step in el at t_step before turn-around 
  double daz_step; // dist. at t_step seconds before turn-around
  double sine_sun_az; // sun az for use in this function
  static double last_v = 0.0;

  int N_scans; // number of azimuth half-scans per el step
  int i_point;

  if (CommandData.pointing_mode.new_sine) {
    n_scans = 0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = el_dest_pre = el_start;
    axes_mode.el_vel = 0.0;
    CommandData.pointing_mode.del = CommandData.pointing_mode.el_step;
    CommandData.pointing_mode.new_sine = 0;
  }

  i_point = GETREADINDEX(point_index);

  t_before = (CommandData.bsc_delay/ACSData.bbc_rate) + CommandData.StarCam[0].expTime/2000.0;
  //t_step = 0.35; // "on_delay" in GetIElev + (1/2)*(step duration) 
  t_step = 0.5; 

  az_accel = CommandData.az_accel;
  az_accel_dv = az_accel/(ACSData.bbc_rate);
  //TODO: HACK for beam mapping, remove it!
  if (CommandData.pointing_mode.is_beam_map) {
    az_accel_max_dv = (CommandData.az_accel_max)/(ACSData.bbc_rate);
  }

  N_scans = CommandData.pointing_mode.Nscans;

  /* propagate az sol'n forward by az_delay */ 
  az = PointingData[i_point].az + PointingData[i_point].v_az 
       * (CommandData.pointing_mode.az_delay/ACSData.bbc_rate);

  /* make sure we don't cross the sun between here and scan centre */
  SetSafeDAz(az, &centre);
  
  left = centre - ampl;
  right = centre + ampl;
  
  /* This should never ever matter.  MIN_SCAN = 0.1 deg */
  if (right-left < MIN_SCAN) {
    left = centre - MIN_SCAN/2.0; 
    right = left + MIN_SCAN;
    ampl = right - centre;
  }
  
  sine_sun_az = PointingData[i_point].sun_az;

  /* get sun_az into correct range relative to scan endpoints, for 
   * direct comparision */
  UnwindDiff(left, &sine_sun_az);

   if ( (sine_sun_az > left) && 
       (sine_sun_az < right) ) {

    bprintf(err, 
    "Dude, WTF? The sun is inside the scan region! Stopping gondola.");

    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = 0;
    CommandData.pointing_mode.vaz = 0.0;
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;
    CommandData.pointing_mode.mode = P_DRIFT;
    
    return;

  }
  

  v_az_max = sqrt(az_accel * ampl);
  
  // |distance| from end point when V_req = V_AZ_MIN
  turn_around = ampl*(1 - sqrt(1-(V_AZ_MIN*V_AZ_MIN)/(v_az_max*v_az_max)));

  axes_mode.az_mode = AXIS_VEL; // applies in all cases below:

  /* case 1: moving into scan from beyond left endpoint: */
  if (az < left - CommandData.pointing_mode.overshoot_band) {
    scan_region = SCAN_BEYOND_L;
    scan_region_last = scan_region;
    CommandData.pointing_mode.is_turn_around = 0;
    v_az = sqrt(2.0*az_accel*(left - CommandData.pointing_mode.overshoot_band -
           az)) + V_AZ_MIN;
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
    CommandData.pointing_mode.is_turn_around = 0;
    v_az = -sqrt(2.0*az_accel*(az-(right + 
           CommandData.pointing_mode.overshoot_band))) - V_AZ_MIN;
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
	           +PointingData[i_point].offset_ofyaw_gy)  > 0.0) ) {
             //&& (PointingData[i_point].v_az > V_AZ_MIN) ) {
    scan_region = SCAN_L_TO_R;
    if (scan_region_last == SCAN_R_TO_L) {
      n_scans++;
    }
    scan_region_last = scan_region;	       

    CommandData.pointing_mode.is_turn_around = 0;
  
    if ( ((az - left) > ampl/10.0) && ( (az-left) < (ampl/10.0 + 1.0) ) ) { 
      // ampl/10 deg. in from left turn-around
      el_dest_pre = axes_mode.el_dest;
    }
  
    v_az = sqrt(az_accel*ampl)*sin(acos((centre-az)/ampl));
    
    //TODO: HACK for beam mapping, remove it!
    if (CommandData.pointing_mode.is_beam_map) {
      v_az = CommandData.pointing_mode.vaz;
    }

    a_az = az_accel*( (centre - az)/ampl ); 

    /* star camera trigger (lemur?) */
    if (az >= (right + 
        ampl*(cos(sqrt(az_accel/ampl)*t_before) - 1.0))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
    
    /* start el step at position corresponding to t_step seconds before 
     * turn-around OR 0.05 deg. before turn-around, whichever is larger  */
    daz_step = ampl*(cos(sqrt(az_accel/ampl)*t_step) - 1.0);
    daz_step = fabs(daz_step) > 0.05 ? daz_step : -0.05;
    
    //TODO: HACK for beam mapping, remove it!
    if (CommandData.pointing_mode.is_beam_map) {
      daz_step = -turn_around - 0.02;
    }

    if (az >= (right + daz_step)) {
      if ( ((n_scans+1) - N_scans*CommandData.pointing_mode.Nsteps) 
          == N_scans) {
       //bprintf(info, "Number of scans completed: %d. Resetting elevation\n",
        //n_scans); 
        n_scans = 0; 
      }
      if ( ((n_scans+1) % N_scans) == 0 ) { // step in elevation
        //bprintf(info, "stepping in el at right turn around");
        axes_mode.el_mode = AXIS_POSITION;
        CommandData.pointing_mode.el_mode = P_EL_GOTO;
        axes_mode.el_dest = el_start + ((n_scans+1)/N_scans)
                            *CommandData.pointing_mode.del;
        /*
        bprintf(info, 
        "n_scans = %d, N_scans = %d, el_start = %f, del = %f, dest = %f",
        n_scans, N_scans, el_start, CommandData.pointing_mode.del, 
        axes_mode.el_dest);
        */
        axes_mode.el_vel = 0.0;
      }  
    }
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 4: moving from right to left endpoints */
  } else if ( (az > (left+turn_around)) && (az < (right-turn_around)) 
              && ( (PointingData[i_point].v_az
		          + PointingData[i_point].offset_ofyaw_gy) < 0.0) ) {
              //&& (PointingData[i_point].v_az < -V_AZ_MIN) ) {
    scan_region = SCAN_R_TO_L;
    if (scan_region_last == SCAN_L_TO_R) {
      n_scans++;
    }
    scan_region_last = scan_region;

    CommandData.pointing_mode.is_turn_around = 0;
  
    if ( ((right - az) > ampl/10.0) && ((right - az) < (ampl/10.0 + 1.0))  ) {
      // ampl/10 degrees in from right turn-around
      el_dest_pre = axes_mode.el_dest;
    }
  
    v_az = sqrt(az_accel*ampl)*sin(-acos((centre-az)/ampl)); 

    //TODO: HACK for beam mapping, remove it!
    if (CommandData.pointing_mode.is_beam_map) {
      v_az = -CommandData.pointing_mode.vaz;
    }

    a_az = az_accel*( (centre - az)/ampl );
  
    // star camera trigger (lemur?)
    if (az <= (left + 
        ampl*(1.0 - cos(sqrt(az_accel/ampl)*t_before)))) {
      bsc_trigger = 1;
    } else {
      bsc_trigger = 0;
    }
    
    /* start el step at position corresponding to t_step seconds before 
     * turn-around OR 0.05 deg. before turn_around, whichever is larger */
    daz_step = ampl*(1.0 - cos(sqrt(az_accel/ampl)*t_step));
    daz_step = fabs(daz_step) > 0.05 ? daz_step : 0.05;

    //TODO: HACK for beam mapping, remove it!
    if (CommandData.pointing_mode.is_beam_map) {
      daz_step = turn_around + 0.02;
    }
    
    if (az <= (left + daz_step)) {
      if ( ((n_scans+1) - N_scans*CommandData.pointing_mode.Nsteps) == N_scans) {
        //bprintf(info, "Number of scans completed: %d. Resetting elevation\n",
        //n_scans); 
        n_scans = 0; 
      }
      if ( ((n_scans+1) % N_scans) == 0 ) { // step in elevation
        //bprintf(info, "stepping in el at left turn around");
        axes_mode.el_mode = AXIS_POSITION;
        CommandData.pointing_mode.el_mode = P_EL_GOTO;
        axes_mode.el_dest = el_start + ((n_scans+1)/N_scans)
                            *CommandData.pointing_mode.del;
        /*
        bprintf(info, 
        "n_scans = %d, N_scans = %d, el_start = %f, del = %f, dest = %f",
        n_scans, N_scans, el_start, CommandData.pointing_mode.del, 
        axes_mode.el_dest);
        */
        axes_mode.el_vel = 0.0;
      }
    }
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;

  /* case 5: in left turn-around zone */ 
  } else if ( az <= left+turn_around ) {
    scan_region = SCAN_L_TURN;
    CommandData.pointing_mode.is_turn_around = 1;
      
    v_az = last_v + az_accel_dv;
    
    if (v_az > V_AZ_MIN) {
      v_az = V_AZ_MIN;
    }
    
    //TODO: HACK for beam mapping, remove it!
    if (CommandData.pointing_mode.is_beam_map) {
      v_az = last_v + az_accel_max_dv;
    }

    a_az = az_accel*( (centre - az)/ampl );
    
    axes_mode.az_vel = v_az; 
    axes_mode.az_accel = a_az;
    
  /* case 6: in right turn-around zone */   
  } else if (az >= right-turn_around) {
    scan_region = SCAN_R_TURN;
    CommandData.pointing_mode.is_turn_around = 1;
    
    v_az = last_v - az_accel_dv;
    
    if (v_az < -V_AZ_MIN) {
      v_az = -V_AZ_MIN;
    }

    //TODO: HACK for beam mapping:
    if (CommandData.pointing_mode.is_beam_map) {
      v_az = last_v - az_accel_max_dv;
    }

    a_az = az_accel*( (centre - az)/ampl );
    
    axes_mode.az_vel = v_az;
    axes_mode.az_accel = a_az;
  }


  last_v = v_az;
}


/* modified quad (radbox) scan mode for Spider */
static void DoSpiderMode(void)
{
  /* parameters of quad */
  double ra_start, dec_start;
  double az_start, el_start;
  double lst, lat;
  double c_az[4], c_el[4]; // corner az and corner el
  
  /* parameters of "instantaneous" sinusoidal az scan: */
  double ampl, centre, left, right, top, bottom;
  double az_of_bot;

  int i, i_point;

  ra_start = CommandData.pointing_mode.X;
  dec_start = CommandData.pointing_mode.Y;

  i_point = GETREADINDEX(point_index);
  
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  
  /* convert ra/decs to az/el */
  for (i = 0; i < 4; i++) {
    radec2azel(CommandData.pointing_mode.ra[i],
               CommandData.pointing_mode.dec[i],
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

  radbox_endpoints(c_az, c_el, el_dest_pre, &left, &right, &bottom, &top, 
                   &az_of_bot);

  if (left > right) {
    left -=360.0;
  }
  ampl = (right - left) / 2.0;
  centre = (left + right) / 2.0;

  DoSineMode(centre, ampl, el_start);
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
  if (axes_mode.el_dest > MAX_EL) {
    axes_mode.el_dest = MAX_EL;
  } else if (axes_mode.el_dest < MIN_EL) {
    axes_mode.el_dest = MIN_EL;
  }
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
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = ACSData.enc_mean_el;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = CommandData.pointing_mode.vaz;
      bsc_trigger = 1;
      break;
    case P_AZEL_GOTO:
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = CommandData.pointing_mode.X;
      axes_mode.az_vel = 0.0;
      bsc_trigger = 1;
      break;
    case P_AZ_SCAN:
      DoAzScanMode();
      break;
    case P_RADEC_GOTO:
      DoRaDecGotoMode();
      break;
    case P_SPIDER:
      DoSpiderMode();
      break;
    case P_SINE:
      DoSineMode(CommandData.pointing_mode.X, (CommandData.pointing_mode.w)/2.0,
                 CommandData.pointing_mode.Y);
      break;
    case P_LOCK:
      axes_mode.el_mode = AXIS_LOCK;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = 0.0;
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
  reactinfo.mode = 1; // irrelevant for RW

  nameThread("RWCom");

  while (!InCharge) {
    if (firsttime==1) {
      //bprintf(info, "Not in charge: waiting.");
      firsttime=0;
    }
    usleep(20000);
  }

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

    if (reactinfo.open==0) {
      if (i_serial==10) {
        bputs(err, "Port could not be opened after 10 attempts.\n");
      }
      i_serial++;
      sleep(1);
    }
  }
  
  if (i_serial>=9) {
    bprintf(info, "Opened port on attempt %i",i_serial);
  }
  
  /* Configure the serial port.  If, after 10 attempts, the port is not 
     initialized, then the thread enters the main loop, where a reset command 
     is triggered. */
  
  i_serial=0;

  while (reactinfo.init==0 && i_serial<=9) {
    reactinfo.verbose=CommandData.verbose_rw;
    configure_amc(&reactinfo);
    if (reactinfo.init==1) {
      //bprintf(info,"Initialized the controller on attempt number %i"
      //        ,i_serial); 
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
        "Resetting serial connection.");
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
          //bprintf(info,"Reaction wheel motor is now enabled");
          reactinfo.disabled=0;
        }
      }
      if(CommandData.disable_az==1 && (reactinfo.disabled==0 || 
         reactinfo.disabled==2)) {
         bprintfverb(info,reactinfo.verbose,MC_VERBOSE,
         "Attempting to disable the reaction wheel motor controller.");
	       bridge_flag=disableAMC(&reactinfo);
	       if(bridge_flag==0){    
	         //bprintf(info,"Reaction wheel motor controller is now disabled.");
	         reactinfo.disabled=1;
	       }
      } 

      if (firsttime && 0) {
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
                                           *360.0, 360.0);

      thread_count %= 5;

      switch(thread_count) {
        case 0:
	        current_raw=queryAMCInd(16,3,1,&reactinfo);
	        break;
        case 1:
	        db_stat_raw=queryAMCInd(2,0,1,&reactinfo);
	        break;
        case 2:
	        dp_stat_raw=queryAMCInd(2,1,1,&reactinfo);
	        break;
        case 3:
	        ds1_stat_raw=queryAMCInd(2,3,1,&reactinfo);
	        break;
        case 4:
	        rw_vel_raw=((int) queryAMCInd(17,2,2,&reactinfo)); 
	        break;
        default:
          break;
      }

      thread_count++;

      // divide current by scaling factor which is (2^13 / peak drive current) 
      // to get units in amps
	    RWMotorData[rw_motor_index].current=(((double)current_raw)/8192.0)*60.0;
      RWMotorData[rw_motor_index].db_stat=db_stat_raw;
      RWMotorData[rw_motor_index].dp_stat=dp_stat_raw;
      RWMotorData[rw_motor_index].ds1_stat=ds1_stat_raw;
      RWMotorData[rw_motor_index].dps_rw = rw_vel_raw*(20000.0/131072.0)
                                           *(360.0/16384.0);
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
  pivotinfo.mode = 0; 

  nameThread("PivCom");

  while (!InCharge) {
    if (firsttime==1) {
      firsttime=0;
    }
    usleep(20000);
  }

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

    if (pivotinfo.open==0) {
      if (i==10) {
        bputs(err, "Port could not be opened after 10 attempts.\n");
      }
      i++;
      sleep(1);
    }
  }
  
  if (i>=9) {
    bprintf(info, "Opened port on attempt %i",i);
  }
  

  // Configure the serial port. If after 10 attempts the port is not initialized
  // , it enters the main loop where it will trigger a reset command.                                             
  i=0;
  while (pivotinfo.init==0 && i <=9) {
    pivotinfo.verbose=CommandData.verbose_piv;
    configure_amc(&pivotinfo);
    if (pivotinfo.init==0) {
      sleep(1);
    }
    i++;
  }

  if (pivotinfo.init==0) {
    bprintf(info,"Could not initialize after 10 attempts."); 
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

    // Make bitfield of controller info structure.
    PivotMotorData[pivot_motor_index].drive_info=makeMotorField(&pivotinfo);
    PivotMotorData[pivot_motor_index].err_count=(pivotinfo.err_count > 65535) ?
                                                65535: pivotinfo.err_count;
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
	      //bprintf(warning,"Resetting connection to pivot controller.");
      } else if ((resetcount % 50)==0) {
	      bprintfverb(warning,pivotinfo.verbose,MC_VERBOSE,
        "reset->Unable to connect to pivot after %i attempts.",resetcount);
      }

      bprintfverb(warning,pivotinfo.verbose,MC_EXTRA_VERBOSE,
                  "Attempting to reset the pivot controller.",resetcount);
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
        bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,
        "Attempting to enable the pivot motor contoller.");
	      n=enableAMC(&pivotinfo);
	      if(n==0) {
	        //bprintf(info,"Pivot motor is now enabled");
	        pivotinfo.disabled=0;
	      }
      }
      if(CommandData.disable_az==1 && 
        (pivotinfo.disabled==0 || pivotinfo.disabled==2)) {
        bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,
        "Attempting to disable the pivot motor controller.");
	      n=disableAMC(&pivotinfo);
	      if(n==0) {    
	        //bprintf(info,"Pivot motor controller is now disabled.");
	        pivotinfo.disabled=1;
	      }
      } 

      if (firsttime && 0) {
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
      bprintfverb(info,pivotinfo.verbose,MC_VERBOSE,
      "Resolver Position is: %i",pos_raw);
      PivotMotorData[pivot_motor_index].res_piv=((double) pos_raw)
                                                /PIV_RES_CTS*360.0; 
      j=j%6;
      switch(j) {
      case 0:
	      current_raw=queryAMCInd(16,3,1,&pivotinfo);
	      break;
      case 1:
	      db_stat_raw=queryAMCInd(2,0,1,&pivotinfo);
	      break;
      case 2:
	      dp_stat_raw=queryAMCInd(2,1,1,&pivotinfo);
	      break;
      case 3:
	      ds1_stat_raw=queryAMCInd(2,3,1,&pivotinfo);
	      break;
      case 4:
	      piv_vel_raw=((int) queryAMCInd(17,2,2,&pivotinfo));
	      break;
      case 5:
        /* check pivot drive control mode */
        /* set as necessary */
        tmp = queryAMCInd(0xd3, 0x00, 1, &pivotinfo);
        //bprintf(info, "Drive mode bitfield = 0x%04x", tmp);
        if (tmp != CommandData.pointing_mode.piv_mode) {
          if (CommandData.pointing_mode.piv_mode == P_PIV_VEL) {
            send_amccmd(0xD1, 0x00, 0x0000, 1, 0, &pivotinfo);
            bprintf(info, "changing from torque mode to velocity mode");
            pivotinfo.mode = CommandData.pointing_mode.piv_mode;
          } else if (CommandData.pointing_mode.piv_mode == P_PIV_TORQUE) {
            send_amccmd(0xD1, 0x00, 0x0001, 1, 0, &pivotinfo);
            bprintf(info, "changing from velocity mode to torque mode");
            pivotinfo.mode = CommandData.pointing_mode.piv_mode;
          } else {
            bputs(err, "Unknown pivot control mode specified\n");
          }
        }
        break;
      default:
        break;
      }

      // *2^13 / peak drive current, Units are Amps
      PivotMotorData[pivot_motor_index].current=((double)current_raw)
                                                /8192.0*20.0; 
      PivotMotorData[pivot_motor_index].db_stat=db_stat_raw;
      PivotMotorData[pivot_motor_index].dp_stat=dp_stat_raw;
      PivotMotorData[pivot_motor_index].ds1_stat=ds1_stat_raw;
      PivotMotorData[pivot_motor_index].dps_piv = piv_vel_raw
                                                  *(20000.0/131072.0)
                                                  *(360.0/16384.0);
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
