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

#define GY1_GAIN_ERROR 1.0407
#define GY1_OFFSET (0.0075)
#define GY2_OFFSET (0.0123)
#define GY3_OFFSET (0.0143)

#define MAX_ISC_AGE 200

extern int isc_trigger_since_last; // set in tx.c - frames since pulse


extern server_frame ISCData[3]; // isc.c
extern int iscdata_index; // isc.c

void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
                double *el);
double getlst(time_t t, double lon); // defined in starpos.c

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


#define M2DV(x) ((x/60.0)*(x/60.0))

#define MAG_ALIGNMENT (0.0)
/************************************************************************
*                                                                      *
*   MagRead:  use the world magnetic model, atan2 and a lookup table   *
*             to convert mag_x and mag_y to mag_az                     *
*                                                                      *
 ************************************************************************/
void MagConvert() {
  double mag_az;
  float year;
  static float dec, dip, ti, gv;
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
  if ((t = PointingData[i_point_read].t) > oldt + 300) {
    oldt = t;
    gmtime_r(&t, &now);
    year = 1900 + now.tm_year + now.tm_yday / 365.25;

    GetMagModel(SIPData.GPSpos.alt / 1000.0, PointingData[i_point_read].lat,
        PointingData[i_point_read].lon, year, &dec, &dip, &ti, &gv);

    dec *= M_PI / 180.0;
    dip *= M_PI / 180.0;
  }

  /* The dec is the correction to the azimuth of the magnetic field. */
  /* If negative is west and positive is east, then: */
  /* */
  /*   true bearing = magnetic bearing + dec */
  /* */
  /* Thus, depending on the sign convention, you have to either add or */
  /* subtract dec from az to get the true bearing. (Adam H.) */

  raw_mag_az = atan2(ACSData.mag_y, ACSData.mag_x);
  mag_az = -LutCal(&magLut, raw_mag_az);
  
  mag_az += dec + MAG_ALIGNMENT + M_PI;
  mag_az *= 180.0/M_PI;

  PointingData[point_index].mag_az = mag_az;
  PointingData[point_index].mag_model = dec*180.0/M_PI;  
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
  
  hs.gyro1_history[hs.i_history] = ACSData.gyro1;
  hs.gyro2_history[hs.i_history] = ACSData.gyro2;
  hs.gyro3_history[hs.i_history] = ACSData.gyro3;
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
  gy1 *= GY1_GAIN_ERROR;
  e->angle += (gy1 + gy1_off)/100.0;
  e->varience += GYRO_VAR;

  // evolve az
  enc_el *= M_PI/180.0;
  gy_az = -(gy2+gy2_off) * cos(enc_el) + -(gy3+gy3_off) * sin(enc_el);
  a->angle += gy_az/100.0;
  a->varience += GYRO_VAR;

  i_isc = GETREADINDEX(iscdata_index);
  if (ISCData[i_isc].framenum!=last_isc_framenum) { // new solution
    if (isc_trigger_since_last < MAX_ISC_AGE) {
      // get az and el for new solution
      i_point = GETREADINDEX(point_index);
      ra = ISCData[i_isc].ra * (12.0/M_PI);
      dec = ISCData[i_isc].dec * (180.0/M_PI);
      radec2azel(ra, dec, PointingData[i_point].lst, PointingData[i_point].lat,
		 &new_az, &new_el);

      // this solution is isc_trigger_since_last old: how much have we moved?
      gy_el_delta = 0;
      gy_az_delta = 0;
      for (i=0; i<isc_trigger_since_last; i++) {
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
      w2 = e->samp_weight;
      e->angle = (w1*e->angle + new_el * w2)/(w1+w2);
      e->varience = 1.0/(w1+w2);
      e->angle += gy_el_delta; // add back to now

      // evolve az solution
      a->angle -= gy_az_delta; // rewind to when the frame was grabbed
      w1 = 1.0/(a->varience);
      w2 = a->samp_weight;
      a->angle = (w1*a->angle + new_az * w2)/(w1+w2);
      a->varience = 1.0/(w1+w2);
      a->angle += gy_az_delta; // add back to now
    }
    
    last_isc_framenum = ISCData[i_isc].framenum;
    isc_trigger_since_last = -1; // reset counter.
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

  gyro *= GY1_GAIN_ERROR;
  
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
void AddElSolution(struct ElAttStruct *ElAtt, struct ElSolutionStruct *ElSol) {
  double weight, var;

  var = ElSol->varience + ElSol->sys_var;

  if (var>0) weight = 1.0/var;
  else weight = 1.0E30; // should be impossible

  ElAtt->el = (weight * (ElSol->angle + ElSol->trim) +
	       ElAtt->weight * ElAtt->el) /
	      (weight + ElAtt->weight);

  ElAtt->gy_offset = (weight * ElSol->gy_offset +
		     ElAtt->weight * ElAtt->gy_offset) /
		     (weight + ElAtt->weight);
  ElAtt->weight+=weight;
}

// Weighted mean of AzAtt and AzSol
void AddAzSolution(struct AzAttStruct *AzAtt, struct AzSolutionStruct *AzSol) {
  double weight, var;

  var = AzSol->varience + AzSol->sys_var;

  if (var>0) weight = 1.0/var;
  else weight = 1.0E30; // should be impossible

  AzAtt->az = (weight * (AzSol->angle + AzSol->trim) +
	       AzAtt->weight * AzAtt->az) /
	      (weight + AzAtt->weight);

  AzAtt->gy2_offset = (weight * AzSol->gy2_offset +
		     AzAtt->weight * AzAtt->gy2_offset) /
		     (weight + AzAtt->weight);
  AzAtt->gy3_offset = (weight * AzSol->gy3_offset +
		     AzAtt->weight * AzAtt->gy3_offset) /
		     (weight + AzAtt->weight);
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

    s->angle = (w1*s->angle + new_angle * w2)/(w1+w2);
    s->varience = 1.0/(w1+w2);
    
    if (s->n_solutions>10) { // only calculate if we have had at least 10
      if (s->n_solutions < 1000) {
	fs = 20.0*s->FC;
      } else {
	fs = s->FC;
      }

      daz = new_angle - s->last_input;
      
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
  double gy_roll, gy2, gy3, el_rad, clin_elev;
  static int no_dgps_pos = 0, last_i_dgpspos = 0;
  static int last_i_dgpsatt = 0;
  
  int i_dgpspos, i_dgpsatt;
  int i_point_read;

  static struct LutType elClinLut = {"/data/etc/clin_elev.lut",0,NULL,NULL,0};
  
  static double gy_roll_amp = 0.0;

  struct ElAttStruct ElAtt = {0.0, 0.0, 0.0};
  
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
  static struct ElSolutionStruct ClinEl = {0.0, // starting angle
					  360.0*360.0, // varience
					  1.0/M2DV(6), //sample weight
					  M2DV(60), // systemamatic varience
					  0.0, // trim 
					  0.0, // last input
					  0.0, // gy integral
					  GY1_OFFSET, // gy offset
					  0.00004, // filter constant
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
					  0.00004, // filter constant
					  0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct NullAz = {95.0, // starting angle
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
  i_dgpsatt = GETREADINDEX(dgpsatt_index);
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
  /**      do ISC Solution            **/
  EvolveSCSolution(&ISCEl, &ISCAz,
		   ACSData.gyro1, 
		   PointingData[i_point_read].gy1_offset,
		   ACSData.gyro2, PointingData[i_point_read].gy2_offset,
		   ACSData.gyro3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el);

  /*************************************/
  /**      do elevation solution      **/
  clin_elev = LutCal(&elClinLut, ACSData.clin_elev);
  
  EvolveElSolution(&ClinEl, ACSData.gyro1, 
		 PointingData[i_point_read].gy1_offset,
		 clin_elev, 1);
  EvolveElSolution(&EncEl, ACSData.gyro1, 
		 PointingData[i_point_read].gy1_offset,
		 ACSData.enc_elev, 1);

  if (CommandData.use_elenc) {
    AddElSolution(&ElAtt, &EncEl);
  }

  if (CommandData.use_elclin) {
    AddElSolution(&ElAtt, &ClinEl);
  }

  PointingData[point_index].gy1_offset = ElAtt.gy_offset;
  PointingData[point_index].el = ElAtt.el;

  /*******************************/
  /**      do az solution      **/
  EvolveAzSolution(&NullAz,
		   ACSData.gyro2, PointingData[i_point_read].gy2_offset,
		   ACSData.gyro3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   0.0, 0);
  /** MAG Az **/
  MagConvert();
  EvolveAzSolution(&MagAz,
		   ACSData.gyro2, PointingData[i_point_read].gy2_offset,
		   ACSData.gyro3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   PointingData[point_index].mag_az, 1);

  /** DGPS Az **/
  if (i_dgpsatt != last_i_dgpsatt) {
    if (DGPSAtt[i_dgpsatt].att_ok==1) {
      PointingData[point_index].dgps_az = DGPSAtt[i_dgpsatt].az;
    }
  }
  EvolveAzSolution(&DGPSAz,
		   ACSData.gyro2, PointingData[i_point_read].gy2_offset,
		   ACSData.gyro3, PointingData[i_point_read].gy3_offset,
		   PointingData[point_index].el,
		   PointingData[point_index].dgps_az,
		   DGPSAtt[i_dgpsatt].att_ok);

  if (CommandData.use_mag) {
    PointingData[point_index].az = MagAz.angle + MagAz.trim;
    PointingData[point_index].gy2_offset = MagAz.gy2_offset;
    PointingData[point_index].gy3_offset = MagAz.gy3_offset;
  } else {
    PointingData[point_index].az = NullAz.angle + NullAz.trim;
    PointingData[point_index].gy2_offset = NullAz.gy2_offset;
    PointingData[point_index].gy3_offset = NullAz.gy3_offset;
  }
  
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
    ClinEl.trim = NewAzEl.el - ClinEl.angle;	
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


