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

#include "isc_protocol.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "lut.h"
#include "sslutNA.h"

#define GY1_OFFSET (-0.1365)
#define GY2_OFFSET (0.008)
#define GY3_OFFSET (0.140)

#define MAX_ISC_AGE 200

extern struct ISCSolutionStruct ISCSolution[3]; // isc.c
extern int iscdata_index; // isc.c

void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
                double *el);
void azel2radec(double *ra_out, double *dec_out,
		double az, double el, time_t lst, double lat);

double getlst(time_t t, double lon); // defined in starpos.c
double GetJulian(time_t t);

/* Functions in the file 'geomag.c' */
void MagModelInit(int maxdeg);
void GetMagModel(float alt, float glat, float glon, float time,
    float *dec, float *dip, float *ti, float *gv);

int point_index=0;
struct PointingDataStruct PointingData[3];

struct ElAttStruct {
  double el;
  double gy_offset;
  double weight;
};

struct AzAttStruct {
  double az;
  double gy2_offset;
  double gy3_offset;
  double weight;
};

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

// gyros, with earth's rotation removed
struct {
  double gy1;
  double gy2;
  double gy3;
} RG;

void SunPos(double tt, double *ra, double *dec); // in starpos.c

#define M2DV(x) ((x/60.0)*(x/60.0))

#define MAG_ALIGNMENT 0.0;

// limit to 0 to 360.0
void NormalizeAngle(double *A) {
  *A = fmod(*A, 360.0);
  if (*A<0) *A += 360.0;
}

// adjust *A to be within +-180 of ref
void UnwindDiff(double ref, double *A) {
  *A = ref + drem(*A - ref, 360.0);
}
      
/************************************************************************/
/*                                                                      */
/*   MagRead:  use the world magnetic model, atan2 and a lookup table   */
/*             to convert mag_x and mag_y to mag_az                     */
/*                                                                      */
/************************************************************************/
int MagConvert(double *mag_az) {
  float year;
  static float fdec, dip, ti, gv;
  static double dec;
  static time_t t, oldt;
  struct tm now;
  int i_point_read;
  static int firsttime = 1;
  static struct LutType magLut = {"/data/etc/mag.lut",0,NULL,NULL,0};
  double raw_mag_az;
  
  i_point_read = GETREADINDEX(point_index);

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    /* Initialise magnetic model reader: I'm not sure what the '12' is, but */
    /* I think it has something to do with the accuracy of the modelling -- */
    /* probably shouldn't change this value.  (Adam H.) */
    MagModelInit(12);
    LutInit(&magLut);

    oldt = -1;
    firsttime = 0;
  }

  /* Every 300 s = 5 min, get new data from the magnetic model. */
  /* */
  /* dec = magnetic declination (field direction in az) */
  /* dip = magnetic inclination (field direction in ele) */
  /* ti  = intensity of the field in nT */
  /* gv  = modified form of dec used in polar reasons -- haven't researched */
  /*       this one */
  /* */
  /* The year must be between 2000.0 and 2005.0 with current model data */
  /* */
  /* The functions called are in 'geomag.c' (Adam. H) */
  if ((t = PointingData[i_point_read].t) > oldt + 10) {
    oldt = t;
    gmtime_r(&t, &now);
    year = 1900 + now.tm_year + now.tm_yday / 365.25;

    GetMagModel(SIPData.GPSpos.alt / 1000.0, PointingData[i_point_read].lat,
        PointingData[i_point_read].lon, year, &fdec, &dip, &ti, &gv);

    dec = fdec;
    
  }

  /* The dec is the correction to the azimuth of the magnetic field. */
  /* If negative is west and positive is east, then: */
  /* */
  /*   true bearing = magnetic bearing + dec */
  /* */
  /* Thus, depending on the sign convention, you have to either add or */
  /* subtract dec from az to get the true bearing. (Adam H.) */

  raw_mag_az = (180.0/M_PI) * atan2(ACSData.mag_y, ACSData.mag_x);
  
  *mag_az = LutCal(&magLut, raw_mag_az);
  
  *mag_az += dec + MAG_ALIGNMENT;
    
  NormalizeAngle(mag_az);

  NormalizeAngle(&dec);
  
  PointingData[point_index].mag_model = dec;
  
  return (1);
}

int DGPSConvert(double *dgps_az, double *dgps_pitch, double *dgps_roll) {
  static int last_i_dgpsatt = 0;
  int i_dgpsatt;

  i_dgpsatt = GETREADINDEX(dgpsatt_index);
  *dgps_az = DGPSAtt[i_dgpsatt].az;
  NormalizeAngle(dgps_az);

  *dgps_pitch = DGPSAtt[i_dgpsatt].pitch;
  NormalizeAngle(dgps_pitch);

  *dgps_roll = DGPSAtt[i_dgpsatt].roll;
  NormalizeAngle(dgps_roll);

  if (i_dgpsatt != last_i_dgpsatt) {
    if (DGPSAtt[i_dgpsatt].att_ok==1) {
      return (1);
    }
  }
  //*dgps_az = 0;
  return(0);
}

// return 1 if new sun, and 0 otherwise
#define MIN_SS_PRIN 7
int SSConvert(double *ss_az) {
  static SSLut_t SSLut;
  static int firsttime = 1;
  int i_point, i_ss, iter;
  double az, sun_az, sun_el;
  double sun_ra, sun_dec, jd;
  static int last_i_ss = -1;
  
  int eflag;
  
  if (firsttime) {
    SSLut_GetLut(&SSLut, "/data/etc/sslut.dat");
    firsttime = 0;
  }
  
  i_ss = GETREADINDEX(ss_index);
  i_point = GETREADINDEX(point_index);

  
  /* get current sun az, el */
  jd = GetJulian(PointingData[i_point].t);
  SunPos(jd, &sun_ra, &sun_dec);
  sun_ra *= (12.0/M_PI);
  sun_dec *= (180.0/M_PI);
  
  radec2azel(sun_ra, sun_dec, PointingData[i_point].lst,
	     PointingData[i_point].lat, &sun_az, &sun_el);

  NormalizeAngle(&sun_az);
  PointingData[point_index].sun_az = sun_az;

  if (i_ss == last_i_ss) return (0); 
  if (SunSensorData[i_ss].prin < MIN_SS_PRIN) return (0);
  
  if (sun_el<0) sun_el = 10.0;
  
  eflag = SSLut_find((double)SunSensorData[i_ss].raw_az, &az,
		     sun_el*M_PI/180.0,
		     &SSLut, &iter);

  if (eflag!=0) return (0);
  
  *ss_az = az*180.0/M_PI + 180.0 + sun_az;
  NormalizeAngle(ss_az);
 
  return (1);
}

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
  
  hs.gyro1_history[hs.i_history] = RG.gy1;
  hs.gyro2_history[hs.i_history] = RG.gy2;
  hs.gyro3_history[hs.i_history] = RG.gy3;
  hs.elev_history[hs.i_history] = PointingData[index].el*M_PI/180.0;
}

#define GYRO_VAR 3.7808641975309e-08
void EvolveSCSolution(struct ElSolutionStruct *e, struct AzSolutionStruct *a,
		      double gy1, double gy1_off,
		      double gy2, double gy2_off,
		      double gy3, double gy3_off, double enc_el) {
  double gy_az;
  static int last_isc_framenum = 0xfffffff;
  int i_isc, i_point;
  double new_az, new_el, ra, dec;
  
  // when we get a new frame, use these to correct for history
  double gy_el_delta = 0;
  double gy_az_delta = 0;
  int i,j;
  
  double w1, w2;
  
  // evolve el
  e->angle += (gy1 + gy1_off)/100.0;
  e->varience += GYRO_VAR;

  // evolve az
  enc_el *= M_PI/180.0;
  gy_az = -(gy2+gy2_off) * cos(enc_el) + -(gy3+gy3_off) * sin(enc_el);
  a->angle += gy_az/100.0;
  a->varience += GYRO_VAR;

  i_isc = GETREADINDEX(iscdata_index);
  if (ISCSolution[i_isc].framenum!=last_isc_framenum) { // new solution
    if (isc_pulses.age < MAX_ISC_AGE) {
      // get az and el for new solution
      i_point = GETREADINDEX(point_index);
      ra = ISCSolution[i_isc].ra * (12.0/M_PI);
      dec = ISCSolution[i_isc].dec * (180.0/M_PI);
      radec2azel(ra, dec, PointingData[i_point].lst, PointingData[i_point].lat,
		 &new_az, &new_el);
      // this solution is isc_pulses.age old: how much have we moved?
      gy_el_delta = 0;
      gy_az_delta = 0;
      for (i=0; i<isc_pulses.age; i++) {
	j = hs.i_history-i;
	if (j<0) j+= GY_HISTORY;

	gy_el_delta += (hs.gyro1_history[j] + gy1_off)*(1.0/100.0);
	gy_az_delta +=
	  (-(hs.gyro2_history[j]+gy2_off) * cos(hs.elev_history[j]) +
	   -(hs.gyro3_history[j]+gy3_off) * sin(hs.elev_history[j]))*(1.0/100.0);
      }
    
      // evolve el solution
      e->angle -= gy_el_delta; // rewind to when the frame was grabbed
      w1 = 1.0/(e->varience);
      if (ISCSolution[i_isc].sigma > M_PI) {
	w2 = 0;
      } else {
	w2 = 10.0*ISCSolution[i_isc].sigma * (180.0/M_PI); //e->samp_weight;
	if (w2>0) w2 = 1/(w2*w2);
	else w2 = 0; // shouldn't happen
      }
      
      UnwindDiff(e->angle, &new_el);
      e->angle = (w1*e->angle + new_el * w2)/(w1+w2);      
      e->varience = 1.0/(w1+w2);
      e->angle += gy_el_delta; // add back to now
      NormalizeAngle(&(e->angle));
      
      // evolve az solution
      a->angle -= gy_az_delta; // rewind to when the frame was grabbed
      w1 = 1.0/(a->varience);
      // w2 already set 

      UnwindDiff(a->angle, &new_az);
      a->angle = (w1*a->angle + new_az * w2)/(w1+w2);
      a->varience = 1.0/(w1+w2);
      a->angle += gy_az_delta; // add back to now
      NormalizeAngle(&(a->angle));
    }
    
    last_isc_framenum = ISCSolution[i_isc].framenum;
    isc_pulses.age = -1; // reset counter.
  }
}

/* Gyro noise: 7'/rt(hour) */
/** the new solution is a weighted mean of:
    the old solution evolved by gyro motion and
    the new solution. **/
void EvolveElSolution(struct ElSolutionStruct *s,
		      double gyro, double gy_off,
		      double new_angle, int new_reading) {
  double w1, w2;
  double new_offset = 0;
  double fs;

  s->angle += (gyro+gy_off)/100.0;
  s->varience += GYRO_VAR;

  s->gy_int += gyro/100.0; // in degrees

  if (new_reading) {    
    w1 = 1.0/(s->varience);
    w2 = s->samp_weight;

    s->angle = (w1*s->angle + new_angle * w2)/(w1+w2);
    s->varience = 1.0/(w1+w2);

    /** calculate offset **/
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

// Weighted mean of ElAtt and ElSol
void AddElSolution(struct ElAttStruct *ElAtt, struct ElSolutionStruct *ElSol,
		   int add_offset) {
  double weight, var;

  var = ElSol->varience + ElSol->sys_var;

  if (var>0) weight = 1.0/var;
  else weight = 1.0E30; // should be impossible

  ElAtt->el = (weight * (ElSol->angle + ElSol->trim) +
	       ElAtt->weight * ElAtt->el) /
	      (weight + ElAtt->weight);

  if (add_offset) {
    ElAtt->gy_offset = (weight * ElSol->gy_offset +
			ElAtt->weight * ElAtt->gy_offset) /
		       (weight + ElAtt->weight);
  }
  
  ElAtt->weight+=weight;
}

// Weighted mean of AzAtt and AzSol
void AddAzSolution(struct AzAttStruct *AzAtt, struct AzSolutionStruct *AzSol,
		   int add_offset) {
  double weight, var, az;
  
  var = AzSol->varience + AzSol->sys_var;
  az = AzSol->angle + AzSol->trim;

  if (var>0) weight = 1.0/var;
  else weight = 1.0E30; // should be impossible

  UnwindDiff(AzAtt->az, &az);
  AzAtt->az = (weight * (az) +
	       AzAtt->weight * AzAtt->az) /
	      (weight + AzAtt->weight);
  NormalizeAngle(&(AzAtt->az));
  
  if (add_offset) {
    AzAtt->gy2_offset = (weight * AzSol->gy2_offset +
			 AzAtt->weight * AzAtt->gy2_offset) /
			(weight + AzAtt->weight);
    AzAtt->gy3_offset = (weight * AzSol->gy3_offset +
			 AzAtt->weight * AzAtt->gy3_offset) /
			(weight + AzAtt->weight);
  }
  
  AzAtt->weight+=weight;
}

//FIXME: need to add rotation of earth correction
void EvolveAzSolution(struct AzSolutionStruct *s,
		      double gy2, double gy2_offset, double gy3,
		      double gy3_offset,
		      double el, double new_angle, int new_reading) {
  double w1, w2;
  double gy_az;
  double new_offset, fs, daz;

  el *= M_PI/180.0; // want el in radians
  gy_az = -(gy2+gy2_offset) * cos(el) + -(gy3+gy3_offset) * sin(el);

  s->angle += gy_az/100.0;
  s->varience += GYRO_VAR;

  s->gy2_int += gy2/100.0; // in degrees
  s->gy3_int += gy3/100.0; // in degrees

  if (new_reading) {    
    w1 = 1.0/(s->varience);
    w2 = s->samp_weight;

    UnwindDiff(s->angle, &new_angle);
    s->angle = (w1*s->angle + new_angle * w2)/(w1+w2);
    s->varience = 1.0/(w1+w2);
    NormalizeAngle(&(s->angle));
    
    if (s->n_solutions>10) { // only calculate if we have had at least 10
      if (s->n_solutions < 1000) {
	fs = 20.0*s->FC;
      } else {
	fs = s->FC;
      }

      daz = drem(new_angle - s->last_input, 360.0);
      
      /* Do Gyro2 */
      new_offset = -(daz*cos(el) + s->gy2_int)/(0.01*(double)s->since_last);
      s->gy2_offset = fs*new_offset + (1.0-fs)*s->gy2_offset;

      /* Do Gyro3 */
      new_offset = -(daz*sin(el) + s->gy3_int)/(0.01*(double)s->since_last);
      s->gy3_offset = fs*new_offset + (1.0-fs)*s->gy3_offset;

    }
    s->since_last = 0;
    s->n_solutions++;
    s->gy2_int = 0.0;
    s->gy3_int = 0.0;
    s->last_input = new_angle;      
  }
  s->since_last++;
}

/*****************************************************************
  do sensor selection;
  update the pointing;
*/
/* Elevation encoder uncertainty: */
void Pointing(){
  double R, cos_e, cos_l, cos_a;
  double sin_e, sin_l, sin_a;
  double ra, dec, az, el;
  
  int ss_ok, mag_ok, dgps_ok;
  double ss_az, mag_az;
  double dgps_az, dgps_pitch, dgps_roll;
  double gy_roll, gy2, gy3, el_rad, clin_elev;
  static int no_dgps_pos = 0, last_i_dgpspos = 0;
  
  int i_dgpspos;
  int i_point_read;

  static struct LutType elClinLut = {"/data/etc/clin_elev.lut",0,NULL,NULL,0};
  
  static double gy_roll_amp = 0.0;

  struct ElAttStruct ElAtt = {0.0, 0.0, 0.0};
  struct AzAttStruct AzAtt = {0.0, 0.0, 0.0, 0.0};
  
  static struct ElSolutionStruct EncEl = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(6), //sample weight
					  M2DV(6), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, // gy integral
					  GY1_OFFSET, // gy offset
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct ElSolutionStruct ClinEl = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(6), //sample weight
					  M2DV(60), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, // gy integral
					  GY1_OFFSET, // gy offset
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct ElSolutionStruct ISCEl = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(0.2), //sample weight
					  M2DV(0.2), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, // gy integral
					  GY1_OFFSET, // gy offset
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct NullAz = {92.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(6), //sample weight
					  M2DV(6000), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, 0.0, // gy integrals
					  GY2_OFFSET, GY3_OFFSET, // gy offsets
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct MagAz = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(60), //sample weight
					  M2DV(90), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, 0.0, // gy integrals
					  GY2_OFFSET, GY3_OFFSET, // gy offsets
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct DGPSAz = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(10), //sample weight
					  M2DV(10), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, 0.0, // gy integrals
					  GY2_OFFSET, GY3_OFFSET, // gy offsets
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct SSAz =  {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(8), //sample weight
					  M2DV(10), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, 0.0, // gy integrals
					  GY2_OFFSET, GY3_OFFSET, // gy offsets
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct ISCAz = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(0.3), //sample weight
					  M2DV(0.2), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, 0.0, // gy integrals
					  GY2_OFFSET, GY3_OFFSET, // gy offsets
					  0.0001, // filter constant
					  0, 0 // n_solutions, since_last
  };

  if (elClinLut.n==0) LutInit(&elClinLut);
  
  i_dgpspos = GETREADINDEX(dgpspos_index);
  i_point_read = GETREADINDEX(point_index);

  // Make aristotle correct
  R = 15.0/3600.0;
  cos_e = cos(PointingData[i_point_read].el * (M_PI/180.0));
  sin_e = sin(PointingData[i_point_read].el * (M_PI/180.0));
  cos_l = cos(PointingData[i_point_read].lat * (M_PI/180.0));
  sin_l = sin(PointingData[i_point_read].lat * (M_PI/180.0));
  cos_a = cos(PointingData[i_point_read].az * (M_PI/180.0));
  sin_a = sin(PointingData[i_point_read].az * (M_PI/180.0));
  
  PointingData[point_index].gy1_earth = R*(-cos_l*sin_a);
  PointingData[point_index].gy2_earth = R*(cos_e*sin_l - cos_l*sin_e*cos_a);
  PointingData[point_index].gy3_earth = R*(sin_e*sin_l + cos_l*cos_e*cos_a);
  
  RG.gy1 = ACSData.gyro1 - PointingData[point_index].gy1_earth;
  RG.gy2 = ACSData.gyro2 - PointingData[point_index].gy2_earth;
  RG.gy3 = ACSData.gyro3 - PointingData[point_index].gy3_earth;
  
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
  /**      do ISC Solution            **/
  EvolveSCSolution(&ISCEl, &ISCAz,
		   RG.gy1, PointingData[i_point_read].gy1_offset,
		   RG.gy2, PointingData[i_point_read].gy2_offset,
		   RG.gy3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el);

  /*************************************/
  /**      do elevation solution      **/
  clin_elev = LutCal(&elClinLut, ACSData.clin_elev);
  
  EvolveElSolution(&ClinEl, RG.gy1, PointingData[i_point_read].gy1_offset,
		   clin_elev, 1);
  EvolveElSolution(&EncEl, RG.gy1, PointingData[i_point_read].gy1_offset,
		   ACSData.enc_elev, 1);

  if (CommandData.use_elenc) {
    AddElSolution(&ElAtt, &EncEl, 1);
  }

  if (CommandData.use_elclin) {
    AddElSolution(&ElAtt, &ClinEl, 1);
  }

  if (CommandData.use_isc) {
    AddElSolution(&ElAtt, &ISCEl, 0);
  }
  
  PointingData[point_index].gy1_offset = ElAtt.gy_offset;
  PointingData[point_index].el = ElAtt.el;

  /*******************************/
  /**      do az solution      **/
  /** Convert Sensors **/
  mag_ok = MagConvert(&mag_az);
  ss_ok = SSConvert(&ss_az);
  dgps_ok = DGPSConvert(&dgps_az, &dgps_pitch, &dgps_roll);

  /** evolve solutions **/
  EvolveAzSolution(&NullAz,
		   RG.gy2, PointingData[i_point_read].gy2_offset,
		   RG.gy3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   0.0, 0);
  /** MAG Az **/
  EvolveAzSolution(&MagAz,
		   RG.gy2, PointingData[i_point_read].gy2_offset,
		   RG.gy3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   mag_az, mag_ok);

  /** DGPS Az **/
  EvolveAzSolution(&DGPSAz,
		   RG.gy2, PointingData[i_point_read].gy2_offset,
		   RG.gy3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   dgps_az, dgps_ok);

  /** Sun Sensor **/
  EvolveAzSolution(&SSAz,
		   RG.gy2, PointingData[i_point_read].gy2_offset,
		   RG.gy3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   ss_az, ss_ok);  

  AddAzSolution(&AzAtt, &NullAz, 1);
  /** add az solutions **/
  if (CommandData.use_mag) {
    AddAzSolution(&AzAtt, &MagAz, 1);
  }
  if (CommandData.use_sun) {
    AddAzSolution(&AzAtt, &SSAz, 1);
  }
  if (CommandData.use_gps) {
    AddAzSolution(&AzAtt, &DGPSAz, 1);
  }
  if (CommandData.use_isc) {
    AddAzSolution(&AzAtt, &ISCAz, 0);
  }
  
  PointingData[point_index].az = AzAtt.az;
  PointingData[point_index].gy2_offset = AzAtt.gy2_offset;
  PointingData[point_index].gy3_offset = AzAtt.gy3_offset;

  /** calculate ra/dec for convenience on the ground **/
  azel2radec(&ra, &dec,
	     PointingData[point_index].az,  PointingData[point_index].el,
	     PointingData[point_index].lst, PointingData[point_index].lat);
  radec2azel(ra, dec, PointingData[point_index].lst,
	     PointingData[point_index].lat,
	     &az, &el);

  PointingData[point_index].ra = ra;
  PointingData[point_index].dec = dec;
  /** record solutions in pointing data **/
  
  PointingData[point_index].enc_el = EncEl.angle;
  PointingData[point_index].enc_sigma = sqrt(EncEl.varience + EncEl.sys_var);
  PointingData[point_index].clin_el = ClinEl.angle;
  PointingData[point_index].clin_sigma = sqrt(ClinEl.varience + ClinEl.sys_var);
  
  PointingData[point_index].mag_az = mag_az;
  PointingData[point_index].mag_sigma = sqrt(MagAz.varience + MagAz.sys_var);
  PointingData[point_index].dgps_az = dgps_az;
  PointingData[point_index].dgps_pitch = dgps_pitch;
  PointingData[point_index].dgps_roll = dgps_roll;
  PointingData[point_index].dgps_sigma = sqrt(DGPSAz.varience + DGPSAz.sys_var);
  PointingData[point_index].ss_az = ss_az;
  PointingData[point_index].ss_sigma = sqrt(SSAz.varience + SSAz.sys_var);

  PointingData[point_index].isc_az = ISCAz.angle;
  PointingData[point_index].isc_el = ISCEl.angle;
  PointingData[point_index].isc_sigma = sqrt(ISCEl.varience + ISCEl.sys_var);
    
  /************************/
  /* set roll damper gain */
  gy2 = RG.gy2;
  gy3 = RG.gy3;
  el_rad = PointingData[point_index].el * M_PI/180.0,
  gy_roll = fabs(-gy2 * sin(el_rad) + gy3 * cos(el_rad));
  if (gy_roll>gy_roll_amp) gy_roll_amp = gy_roll;
  else gy_roll_amp*=0.9999;
  if (gy_roll_amp > 1.0) gy_roll_amp *= 0.999; // probably a spike 
  PointingData[point_index].gy_roll_amp = gy_roll_amp;

  /********************/
  /* Set Manual Trims */
  if (NewAzEl.fresh == -1) {
    ClinEl.trim = 0.0;
    EncEl.trim = 0.0;
    NullAz.trim = 0.0;
    MagAz.trim = 0.0;
    DGPSAz.trim = 0.0;
    SSAz.trim = 0.0;
    NewAzEl.fresh = 0;
  }
  
  if (NewAzEl.fresh) {
    ClinEl.trim = NewAzEl.el - ClinEl.angle;	
    EncEl.trim = NewAzEl.el - EncEl.angle;	
    NullAz.trim = NewAzEl.az - NullAz.angle;
    MagAz.trim = NewAzEl.az - MagAz.angle;
    DGPSAz.trim = NewAzEl.az - DGPSAz.angle;
    SSAz.trim = NewAzEl.az - SSAz.trim;
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

void SetTrimToISC() {
  int i_point;
  
  i_point = GETREADINDEX(point_index);

  NewAzEl.az = PointingData[i_point].isc_az;
  NewAzEl.el = PointingData[i_point].isc_el;

  NewAzEl.fresh = 1;
}

void AzElTrim(double az, double el) {
  NewAzEl.az = az;
  NewAzEl.el = el;

  NewAzEl.fresh = 1;
}

void ClearTrim() {
  NewAzEl.fresh = -1;
}
