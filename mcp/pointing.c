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

#define GY1_GAIN_ERROR 1.0407
#define GY1_OFFSET (0.0075)
#define GY2_OFFSET (0.0086)
#define GY3_OFFSET (0.0150)

void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
                double *el);
double getlst(time_t t, double lon); // defined in starpos.c

int point_index=0;
struct PointingDataStruct PointingData[3];

struct ElSolutionStruct {
  double angle;    // solution's current angle
  double varience; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double gy_int; // integral of the gyro since the last solution
  double gy_offset; // averaged offset measurement
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
};

struct AzSolutionStruct {
  double angle;    // solution's current angle
  double varience; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double gy2_int; // integral of the gyro since the last solution
  double gy3_int; // integral of the gyro since the last solution
  double gy2_offset; // offset associated with solution
  double gy3_offset;
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
};

struct HistoryStruct {
  double *elev_history;
  double *gyro1_history;
  double *gyro2_history;
  double *gyro3_history;
  int i_history;  // points to last valid point.  Not thread safe.
} hs = {NULL, NULL, NULL, NULL, 0};

struct {
  double az;
  double el;
  int fresh;
} NewAzEl = {0.0, 0.0, 0};


#define M2DV(x) ((x/60.0)*(x/60.0))

#define GY_HISTORY 300
void RecordHistory(int index) {
  /*****************************************/
  /*   Allocate Memory                     */
  if (hs.gyro1_history == NULL) {
    hs.gyro1_history = (double *)malloc(GY_HISTORY * sizeof(double));
    memset(hs.gyro1_history, 0, GY_HISTORY * sizeof(double));
    hs.gyro2_history = (double *)malloc(GY_HISTORY * sizeof(double));
    memset(hs.gyro2_history, 0, GY_HISTORY * sizeof(double));
    hs.gyro3_history = (double *)malloc(GY_HISTORY * sizeof(double));
    memset(hs.gyro3_history, 0, GY_HISTORY * sizeof(double));
    hs.elev_history  = (double *)malloc(GY_HISTORY * sizeof(double));
    memset(hs.elev_history, 0, GY_HISTORY * sizeof(double));
  }

  /*****************************************/
  /* record history                        */
  hs.i_history++;
  if (hs.i_history >= GY_HISTORY) hs.i_history=0;
  
  hs.gyro1_history[hs.i_history] = ACSData.gyro1;
  hs.gyro2_history[hs.i_history] = ACSData.gyro2;
  hs.gyro3_history[hs.i_history] = ACSData.gyro3;
  hs.elev_history[hs.i_history] = PointingData[index].el;
}

/* Gyro noise: 7'/rt(hour) */
/** the new solution is a weighted mean of:
    the old solution evolved by gyro motion and
    the new solution. **/
#define GYRO_VAR 3.7808641975309e-08
void EvolveElSolution(struct ElSolutionStruct *s,
		      double gyro, double new_angle, int new_reading) {
  double w1, w2;
  double new_offset = 0;
  double fs;

  gyro *= GY1_GAIN_ERROR;
  
  s->angle += gyro/100.0;
  s->varience += GYRO_VAR;

  s->gy_int += gyro/100.0; // in degrees

  if (new_reading) {    
    w1 = 1.0/(s->varience);
    w2 = s->samp_weight;

    s->angle = (w1*s->angle + new_angle * w2)/(w1+w2);
    s->varience = 1.0/(w1+w2);
    
    if (s->n_solutions>10) { // only calculate if we have had at least 10
      new_offset = ((new_angle - s->last_input) - s->gy_int) /
		   (0.01*(double)s->since_last);

      if (s->n_solutions < 1000) {
	fs = 20.0*s->FC;
      } else {
	fs = s->FC;
      }

      s->gy_offset = fs * new_offset + (1.0-fs) * s->gy_offset;
    }
    s->since_last = 0;
    s->n_solutions++;
    s->gy_int = 0.0;
    s->last_input = new_angle;      
  }
  s->since_last++;
}

//FIXME: need to add gyro offset stuff - extention of el version
void EvolveAzSolution(struct AzSolutionStruct *s,
		      double gy2, double gy3, double el,
		      double new_angle, int new_reading) {
  double w1, w2;
  double gy_az;

  el *= M_PI/180.0; // want el in radians
  gy_az = -gy2 * cos(el) + -gy3 * sin(el);

  s->angle += gy_az/100.0;
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
  double gy_roll, gy2, gy3, el_rad;
  static int no_dgps_pos = 0, last_i_dgpspos = 0;
  int i_dgpspos;
  int i_point_read;
  
  static double gy_roll_amp = 0.0;
  
  static struct ElSolutionStruct EncEl = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(6), //sample weight
					  M2DV(6), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, // gy integral
					  GY1_OFFSET, // gy offset
					  0.00004, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct NullAz = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(6), //sample weight
					  M2DV(6), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, 0.0, // gy integrals
					  GY2_OFFSET, GY3_OFFSET, // gy offsets
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };

  i_dgpspos = GETREADINDEX(dgpspos_index);
  i_point_read = GETREADINDEX(point_index);


  /*************************************/
  /** Record history for gyro offsets **/
  RecordHistory(i_point_read);
  
  /************************************************/
  /** Set the official Lat and Long: prefer dgps **/
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

  /*****************************/
  /** set time related things **/
  PointingData[point_index].mcp_frame = ACSData.mcp_frame;
  PointingData[point_index].t = time(NULL); // for now use CPU time
  PointingData[point_index].lst = getlst(PointingData[point_index].t,
				       PointingData[point_index].lon);
	 
  /*************************************/
  /**      do elevation solution      **/
  EvolveElSolution(&EncEl, ACSData.gyro1 +
		 PointingData[i_point_read].gy1_offset,
		 ACSData.enc_elev, 1);

  /* for now, use enc_elev solution for elev */
  PointingData[point_index].gy1_offset = EncEl.gy_offset;

  PointingData[point_index].el = EncEl.angle + EncEl.trim;

  /*******************************/
  /**      do az solution      **/
  EvolveAzSolution(&NullAz,
		   ACSData.gyro2 + PointingData[i_point_read].gy2_offset,
		   ACSData.gyro3 + PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   0.0, 0);

  PointingData[point_index].az = NullAz.angle + NullAz.trim;
  PointingData[point_index].gy2_offset = NullAz.gy2_offset;
  PointingData[point_index].gy3_offset = NullAz.gy3_offset;

  /************************/
  /* set roll damper gain */
  gy2 = ACSData.gyro2;
  gy3 = ACSData.gyro3;
  el_rad = PointingData[point_index].el * M_PI/180.0,
  gy_roll = fabs(-gy2 * sin(el_rad) + gy3 * cos(el_rad));
  if (gy_roll>gy_roll_amp) gy_roll_amp = gy_roll;
  else gy_roll_amp*=0.9999;
  if (gy_roll_amp > 1.0) gy_roll_amp *= 0.999; // probably a spike 
  PointingData[point_index].gy_roll_amp = gy_roll_amp;

  /********************/
  /* Set Manual Trims */
  if (NewAzEl.fresh) {
    EncEl.trim = NewAzEl.el - EncEl.angle;	
    NullAz.trim = NewAzEl.az - NullAz.angle;	
    NewAzEl.fresh = 0;
  }
  
  point_index = INC_INDEX(point_index);
} 

// called from the command thread in command.h
void SetRaDec(double ra, double dec) {
  int i_point;
  i_point = GETREADINDEX(point_index);
  
  radec2azel(ra, dec, PointingData[i_point].lst,
	     PointingData[i_point].lat,
 	     &(NewAzEl.az), &(NewAzEl.el));

  NewAzEl.fresh = 1;
}


