#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h> 
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <termios.h> 
#include <ctype.h>
#include <pthread.h>

#include "pointing_struct.h"

double getlst(time_t t, double lon); // defined in starpos.c

int point_index=0;
struct PointingDataStruct PointingData[3];

struct SolutionStruct {
  double angle;    // solution's current angle
  double varience; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
};

#define M2DV(x) ((x/60.0)*(x/60.0))

#define GY_HISTORY 30000
#define EL_GY_GAIN_ERROR 1.0407
#define GY2_TMP_OFFSET (0.01246)
#define GY3_TMP_OFFSET (0.01689)
void GyroOffsets(int index) {
  static short *gyro1_history = NULL;
  static float *elev_history = NULL;
  static int i_history=0, n_h = 0;
  static long long sum_gyro1 = 0;
  static int wait = 30;
  float e1, e2;

  /*****************************************/
  /*   Allocate Memory                     */
  if (gyro1_history == NULL) {
    gyro1_history = (short *)malloc(GY_HISTORY * sizeof(short));
    elev_history = (float *)malloc(GY_HISTORY * sizeof(float));
  }

  // FIXME: temporary offset calculation needs to go away!
  PointingData[index].gy2_offset = GY2_TMP_OFFSET;
  PointingData[index].gy3_offset = GY3_TMP_OFFSET;
  
  if (wait>0) { // don't start accumulating right away...
    wait--;
    PointingData[index].gy1_offset = 0.0;
    return;
  }

  /*****************************************/
  /*   Calculate offset                    */
  e1 = elev_history[i_history];
  e2 = elev_history[i_history] = PointingData[index].el;
  
  gyro1_history[i_history] = ACSData.gyro1*(1.0/0.00091506980885);
  sum_gyro1+=gyro1_history[i_history];

  i_history++;
  if (i_history >= GY_HISTORY) i_history=0;
  if (n_h>=GY_HISTORY) {
    PointingData[index].gy1_offset =
      ((e2-e1) - EL_GY_GAIN_ERROR*sum_gyro1*(0.00091506980885/100.0)) *
      (100.0/(float)GY_HISTORY);
    sum_gyro1-=gyro1_history[i_history];	
  } else if (n_h>0) {
    PointingData[index].gy1_offset =
      ((e2-elev_history[0]) - EL_GY_GAIN_ERROR*sum_gyro1*(0.00091506980885/100.0) ) *
      (100.0/(float)n_h);
    n_h++;
  } else {
    n_h++;
  }
}

/* Gyro noise: 7'/rt(hour) */
/** the new solution is a weighted mean of:
    the old solution evolved by gyro motion and
    the new solution. **/
#define GYRO_VAR 3.7808641975309e-08
void EvolveSolution(struct SolutionStruct *s,
		      double gyro, double new_angle, int new_reading) {
  double w1, w2;
  s->angle += gyro/100.0;
  s->varience += GYRO_VAR;

  if (new_reading) {
    w1 = 1.0/(s->varience);
    w2 = s->samp_weight;

    s->angle = (w1*s->angle + new_angle * w2)/(w1+w2);
    s->varience = 1.0/(w1+w2);
  }
}
 
/*****************************************************************
  do sensor selection;
  update the pointing;
*/
/* Elevation encoder uncertainty: */
void Pointing(){
  double gy_az, gy_roll, gy2, gy3, el_rad;
  static int no_dgps_pos = 0, last_i_dgpspos = 0;
  int i_dgpspos;
  
  static double gy_roll_amp = 0.0;
  static double gy1_offset=0.0;
  static double gy2_offset=0.0;
  static double gy3_offset=0.0;
  
  static struct SolutionStruct EncEl = {0.0, 360.0*360.0,
					1.0/M2DV(6), M2DV(6)};
  static struct SolutionStruct NullAz = {0.0, 360.0*360.0,
					 1.0/M2DV(6), M2DV(6)};
  

  i_dgpspos = GETREADINDEX(dgpspos_index);
  
  /** Set the official Lat and Long: for now use SIP COM1... **/
  if (i_dgpspos != last_i_dgpspos) {
    i_dgpspos = last_i_dgpspos;
    PointingData[point_index].lat = DGPSPos[i_dgpspos].lat;
    PointingData[point_index].lon = DGPSPos[i_dgpspos].lon;
    no_dgps_pos = 0;
  } else {
    no_dgps_pos++;
    if (no_dgps_pos>3000) { // no dgps for 30 seconds - revert to sip
      PointingData[point_index].lat = SIPData.GPSpos.lat;
      PointingData[point_index].lon = SIPData.GPSpos.lon;
    }
  }

  /** set time related things **/
  PointingData[point_index].mcp_frame = ACSData.mcp_frame;
  PointingData[point_index].t = time(NULL); // for now use CPU time
  PointingData[point_index].lst = getlst(PointingData[point_index].t,
				       PointingData[point_index].lon);
	 
  /*************************************/
  /**      do elevation solution      **/
  EvolveSolution(&EncEl, ACSData.gyro1 + gy1_offset,
		 ACSData.enc_elev, 1);

  /* for now, use enc_elev solution for elev */
  PointingData[point_index].el = EncEl.angle;

  /*******************************/
  /**      do az solution      **/
  gy2 = ACSData.gyro2;
  gy3 = ACSData.gyro3;
  el_rad = PointingData[point_index].el* M_PI/180.0;;
  gy_az = -(gy2 + gy2_offset) * cos(el_rad) +
	  -(gy3 + gy3_offset) * sin(el_rad);
  EvolveSolution(&NullAz, gy_az, 0.0, 0);
  PointingData[point_index].az = NullAz.angle;

  /***********************/
  /** Find gyro offsets **/
  GyroOffsets(point_index);
  gy1_offset = PointingData[point_index].gy1_offset;
  gy2_offset = PointingData[point_index].gy2_offset;
  gy3_offset = PointingData[point_index].gy3_offset;

  /************************/
  /* set roll damper gain */
  gy_roll = fabs(-gy2 * sin(el_rad) + gy3 * cos(el_rad));

  if (gy_roll>gy_roll_amp) gy_roll_amp = gy_roll;
  else gy_roll_amp*=0.9999;

  if (gy_roll_amp > 1.0) gy_roll_amp *= 0.999; // probably a spike 
  
  PointingData[point_index].gy_roll_amp = gy_roll_amp;

  point_index = INC_INDEX(point_index);
} 


