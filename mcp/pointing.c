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

int point_index=0;
struct PointingDataStruct PointingData[3];

extern struct ACSDataStruct ACSData;
extern struct SIPDataStruct SIPData;

struct SolutionStruct {
  double angle;    // solution's current angle
  double varience; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
};

#define M2DV(x) ((x/60.0)*(x/60.0))

#define GY_HISTORY 30000
#define EL_GY_GAIN_ERROR 1.0407
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
      (EL_GY_GAIN_ERROR*sum_gyro1*(0.00091506980885/100.0) + (e2-e1)) *
      (100.0/(float)GY_HISTORY);
    sum_gyro1-=gyro1_history[i_history];	
  } else {
    PointingData[index].gy1_offset =
      (EL_GY_GAIN_ERROR*sum_gyro1*(0.00091506980885/100.0) + (e2-elev_history[0])) *
      (100.0/(float)n_h);
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
  int new_index; 
  double gy_az, gy_roll, gy2, gy3, el_rad;
  static double gy_roll_amp = 0.0;

  static struct SolutionStruct EncEl = {0.0, 360.0*360.0,
					1.0/M2DV(6), M2DV(6)};
  static struct SolutionStruct NullAz = {0.0, 360.0*360.0,
					 1.0/M2DV(6), M2DV(6)};
  
  new_index = INC_INDEX(point_index);

  /** Set the official Lat and Long: for now use SIP COM1... **/
  PointingData[new_index].lat = SIPData.GPSpos.lat;
  PointingData[new_index].lon = SIPData.GPSpos.lon;

  /** set time related things **/
  PointingData[new_index].mcp_frame = ACSData.mcp_frame;
  PointingData[new_index].t = time(NULL); // for now use CPU time

  /*************************************/
  /**      do elevation solution      **/
  EvolveSolution(&EncEl, -ACSData.gyro1, ACSData.enc_elev, 1);
  //printf("%g %g %g\n", EncEl.angle, ACSData.enc_elev, ACSData.gyro1);

  /* for now, use enc_elev solution for elev */
  PointingData[new_index].el = EncEl.angle;

  GyroOffsets(new_index);

  /*******************************/
  /**      do az solution      **/
  gy2 = ACSData.gyro2;
  gy3 = ACSData.gyro3;
  el_rad = PointingData[new_index].el* M_PI/180.0;;
  gy_az = gy2 * cos(el_rad) + gy3 * sin(el_rad);
  EvolveSolution(&NullAz, gy_az, 0.0, 0);
  PointingData[new_index].az = NullAz.angle;

  /************************/
  /* set roll damper gain */
  gy_roll = fabs(-gy2 * sin(el_rad) + gy3 * cos(el_rad));

  if (gy_roll>gy_roll_amp) gy_roll_amp = gy_roll;
  else gy_roll_amp*=0.9999;

  if (gy_roll_amp > 1.0) gy_roll_amp *= 0.999; // probably a spike 
  
  PointingData[new_index].gy_roll_amp = gy_roll_amp;
  
  point_index = new_index;
} 


