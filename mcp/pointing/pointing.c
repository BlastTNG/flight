
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
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <termios.h>
#include <ctype.h>
#include <pthread.h>

#include "blast.h"
#include "mcp.h"
#include "pointing.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "lut.h"
#include "tx.h"
#include "fir.h"

#include <dsp1760.h>
#include <angles.h>
#include <xsc_network.h>
#include <conversions.h>
#include <time_lst.h>
#include <utilities_pointing.h>
#include <blast_sip_interface.h>

// Include gsl package for the old sun sensor
#include <gsl/gsl_rng.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/* Functions in the file 'geomag.c' */
void MagModelInit(int maxdeg, const char* wmmFile);
void GetMagModel(float alt, float glat, float glon, float time,
    float *dec, float *dip, float *ti, float *gv);

int point_index = 0;
struct PointingDataStruct PointingData[3];
struct XSCPointingState xsc_pointing_state[2] = {{.counter_fcp = 0}};

struct ElAttStruct {
  double el;
  double offset_gy;
  double weight;
};

struct AzAttStruct {
  double az;
  double offset_ifroll_gy;
  double offset_ifyaw_gy;
  double weight;
};

struct ElSolutionStruct {
  double angle;    // solution's current angle
  double variance; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double gy_int; // integral of the gyro since the last solution
  double offset_gy; // averaged offset measurement
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
  struct FirStruct *fs;
};

struct AzSolutionStruct {
  double angle;    // solution's current angle
  double variance; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double ifroll_gy_int; // integral of the gyro since the last solution
  double ifyaw_gy_int; // integral of the gyro since the last solution
  double offset_ifroll_gy; // offset associated with solution
  double offset_ifyaw_gy;
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
  struct FirStruct *fs2;
  struct FirStruct *fs3;
};

static struct HistoryStruct {
  double *elev_history;
  double *ifel_gy_history;
  double *ifroll_gy_history;
  double *ifyaw_gy_history;
  int i_history;  // points to last valid point.  Not thread safe.
} hs = {NULL, NULL, NULL, NULL, 0};

static struct {
  double az;
  double el;
  int fresh;
  double rate;
} NewAzEl = {0.0, 0.0, 0, 360.0};

// gyros, with earth's rotation removed
static struct {
  double ifel_gy;
  double ifroll_gy;
  double ifyaw_gy;
} RG;

#define MAX_SUN_EL 5.0

static double sun_az, sun_el; // set in SSConvert and used in UnwindDiff

void SunPos(double tt, double *ra, double *dec); // in starpos.c

#define M2DV(x) ((x / 60.0) * (x / 60.0))

// limit to 0 to 360.0
void NormalizeAngle(double *A)
{
  *A = fmod(*A, 360.0);
  if (*A < 0)
    *A += 360.0;
}

void UnwindDiff(double ref, double *A)
{
  *A = ref + remainder(*A - ref, 360.0);
}

// adjust *A to be within +-180 of ref
void SetSafeDAz(double ref, double *A)
{
  *A = ref + remainder(*A - ref, 360.0);
  if (sun_el < MAX_SUN_EL)
    return;

  sun_az = ref + remainder(sun_az - ref, 360.0);

  if ((ref < sun_az) && (sun_az < *A)) {
    *A -= 360.0;
  } else if ((ref > sun_az) && (sun_az > *A)) {
    *A += 360.0;
  }
}

/************************************************************************/
/*                                                                      */
/*   MagRead:  use the world magnetic model, atan2 and a lookup table   */
/*             to convert mag_x and mag_y to mag_az                     */
/*                                                                      */
/************************************************************************/
static int MagConvert(double *mag_az)
{
  float year;
  double mvx, mvy, mvz;
  double raw_mag_az, raw_mag_pitch;
  static float fdec, dip, ti, gv;
  static double dec=0;
  time_t t;
  struct tm now;
  int i_point_read;
  static time_t oldt;
  static int firsttime = 1;
  double magx_m, magx_b, magy_m, magy_b;

  i_point_read = GETREADINDEX(point_index);

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    /* Initialise magnetic model reader: I'm not sure what the '12' is, but */
    /* I think it has something to do with the accuracy of the modelling -- */
    /* probably shouldn't change this value.  (Adam H.) */
      //TODO:Re-enable Magmodel
//    MagModelInit(12, "/data/etc/blast/WMM.COF");

    oldt = 1;
    firsttime = 0;
  }

  /* Every 300 s = 5 min, get new data from the magnetic model.
   *
   * dec = magnetic declination (field direction in az)
   * dip = magnetic inclination (field direction in ele)
   * ti  = intensity of the field in nT
   * gv  = modified form of dec used in polar regions -- haven't researched
   *       this one
   *
   * The year must be between 2010.0 and 2015.0 with current model data
   *
   * The functions called are in 'geomag.c' (Adam. H) */
  if ((t = PointingData[i_point_read].t) > oldt + 10) {
    oldt = t;
    gmtime_r(&t, &now);
    year = 1900 + now.tm_year + now.tm_yday / 365.25;

//    GetMagModel(PointingData[i_point_read].alt / 1000.0,
//        PointingData[i_point_read].lat, -PointingData[i_point_read].lon,
//        year, &fdec, &dip, &ti, &gv);

    dec = fdec;

  }

  /* The dec is the correction to the azimuth of the magnetic field. */
  /* If negative is west and positive is east, then: */
  /* */
  /*   true bearing = magnetic bearing + dec */
  /* */
  /* Thus, depending on the sign convention, you have to either add or */
  /* subtract dec from az to get the true bearing. (Adam H.) */

  mvx = (ACSData.mag_x-MAGX_B)/MAGX_M;
  mvy = (ACSData.mag_y-MAGY_B)/MAGY_M;
  
  magx_m = 1.0/((double)(CommandData.cal_xmax_mag - CommandData.cal_xmin_mag));
  magy_m = -1.0/((double)(CommandData.cal_ymax_mag - CommandData.cal_ymin_mag));
  
  magx_b = (CommandData.cal_xmax_mag + CommandData.cal_xmin_mag)*0.5;
  magy_b = (CommandData.cal_ymax_mag + CommandData.cal_ymin_mag)*0.5;
  
  mvx = magx_m*(ACSData.mag_x - magx_b);
  mvy = magy_m*(ACSData.mag_y - magy_b);
  mvz = MAGZ_M*(ACSData.mag_z - MAGZ_B);


  raw_mag_az = (-1.0)*(180.0 / M_PI) * atan2(mvy, mvx);
  raw_mag_pitch = (180.0/M_PI) * atan2(mvz,sqrt(mvx*mvx + mvy*mvy));
  *mag_az = raw_mag_az;
//  ACSData.mag_pitch = raw_mag_pitch+(double)dip;


#if 0
#warning THE MAGNETIC MODEL HAS BEEN DISABLED
  dec = 0; // disable mag model.
#endif

  *mag_az += dec + MAG_ALIGNMENT;

  NormalizeAngle(mag_az);

  NormalizeAngle(&dec);

  PointingData[point_index].mag_model_dec = dec;

  return (1);
}

// PSSConvert versions added 12 June 2010 -GST
// PSS1 for Lupus, PSS2 for Vela, PSS3 and PSS4 TBD
#define  PSS_L  10.     // 10 mm = effective length of active area
#define  PSS1_D  10.     // 10 mm = Distance between pinhole and sensor
#define  PSS2_D  10.     // 10 mm = Distance between pinhole and sensor
#define  PSS3_D 10.5    // 
#define  PSS4_D 10.34   //
#define  PSS_IMAX  8192.  // Maximum current (place holder for now)
#define  PSS_XSTRETCH  1.  // 0.995
#define  PSS_YSTRETCH  1.  // 1.008
#define  PSS1_BETA  (-PSS1_ALIGNMENT)
#define  PSS2_BETA  (-PSS2_ALIGNMENT)
#define  PSS3_BETA  (-PSS3_ALIGNMENT)
#define  PSS4_BETA  (-PSS4_ALIGNMENT)
#define  PSS1_ALPHA   25.
#define  PSS2_ALPHA 24.3  // This angle should be 25 degrees.  Boom bent?
#define  PSS3_ALPHA   25.
#define  PSS4_ALPHA   25.
#define  PSS1_PSI    -15.5
#define  PSS2_PSI   11.
#define  PSS3_PSI   0
#define  PSS4_PSI   0

//TODO: Separate Sun Az calc
static int PSSConvert(double *azraw_pss, double *elraw_pss) {
//TODO:Reenable PSSConvert
    int     i_point;
    double  sun_ra, sun_dec;
//  double        az[4];
//  double	azraw[4];
//  double	elraw[4];
//  double new_val;
//
//  static double i1[4], i2[4], i3[4], i4[4];
//  double        itot[4];
//  double        x[4], y[4];
//  double        usun[4][3], u2[4][3];
//  gsl_matrix    *rot[4];
//  gsl_matrix    *rxalpha[4], *rzpsi[4];
//
//  double	weight[4];
//  double	weightsum;
//  double        pss_d[4], beta[4], alpha[4], psi[4];
//  double        norm[4];
//  double        pss_imin;
//
//  i1[0] = ACSData.pss1_i1 - 32768.;
//  i2[0] = ACSData.pss1_i2 - 32768.;
//  i3[0] = ACSData.pss1_i3 - 32768.;
//  i4[0] = ACSData.pss1_i4 - 32768.;
//  i1[1] = ACSData.pss2_i1 - 32768.;
//  i2[1] = ACSData.pss2_i2 - 32768.;
//  i3[1] = ACSData.pss2_i3 - 32768.;
//  i4[1] = ACSData.pss2_i4 - 32768.;
////  i1[2] = ACSData.pss3_i1 - 32768.;
////  i2[2] = ACSData.pss3_i2 - 32768.;
////  i3[2] = ACSData.pss3_i3 - 32768.;
////  i4[2] = ACSData.pss3_i4 - 32768.;
////  i1[3] = ACSData.pss4_i1 - 32768.;
////  i2[3] = ACSData.pss4_i2 - 32768.;
////  i3[3] = ACSData.pss4_i3 - 32768.;
////  i4[3] = ACSData.pss4_i4 - 32768.;
//
//  for (i=0; i<4; i++) {
//  	itot[i] = i1[i]+i2[i]+i3[i]+i4[i];
//  }
//
//  pss_imin = CommandData.cal_imin_pss/M_16PRE;
//
    i_point = GETREADINDEX(point_index);
//
//  PointingData[point_index].pss1_snr = itot[0]/PSS_IMAX;  // 10.
//  weight[0]= PointingData[point_index].pss1_snr;
//  PointingData[point_index].pss2_snr = itot[1]/PSS_IMAX;  // 10.
//  weight[1]= PointingData[point_index].pss2_snr;
////  PointingData[point_index].pss3_snr = itot[2]/PSS_IMAX;  // 10.
////  weight[2]= PointingData[point_index].pss3_snr;
////  PointingData[point_index].pss4_snr = itot[3]/PSS_IMAX;  // 10.
////  weight[3]= PointingData[point_index].pss4_snr;
//
//  if (fabs(itot[0]) < pss_imin) {
//    	PointingData[point_index].pss1_snr = 1.;  // 1.
//	weight[0] = 0.0;
//  }
//  if (fabs(itot[1]) < pss_imin) {
//    	PointingData[point_index].pss2_snr = 1.;  // 1.
//	weight[1] = 0.0;
//  }
////  if (fabs(itot[2]) < pss_imin) {
////    	PointingData[point_index].pss3_snr = 1.;  // 1.
////	weight[2] = 0.0;
////  }
////  if (fabs(itot[3]) < pss_imin) {
////    	PointingData[point_index].pss4_snr = 1.;  // 1.
////	weight[3] = 0.0;
////  }
//
//  // Define pss_d (distance to pinhole)
//  pss_d[0] = PSS1_D + CommandData.cal_d_pss1;
//  pss_d[1] = PSS2_D + CommandData.cal_d_pss2;
//  pss_d[2] = PSS3_D + CommandData.cal_d_pss3;
//  pss_d[3] = PSS4_D + CommandData.cal_d_pss4;
//
//  for (i=0; i<4; i++) {
//  	x[i] = -PSS_XSTRETCH*(PSS_L/2.)*((i2[i]+i3[i])-(i1[i]+i4[i]))/itot[i];
//  	y[i] = -PSS_YSTRETCH*(PSS_L/2.)*((i2[i]+i4[i])-(i1[i]+i3[i]))/itot[i];
//  	norm[i] = sqrt(x[i]*x[i] + y[i]*y[i] + pss_d[i]*pss_d[i]);
//  	usun[i][0] = -x[i] / norm[i];
//  	usun[i][1] = -y[i] / norm[i];
//  	usun[i][2] = pss_d[i] / norm[i];
//  }
//
//  // Then spot is at the edge of the sensor
//  if ((fabs(x[0]) > 4.) | (fabs(y[0]) > 4.)) {
//	PointingData[point_index].pss1_snr = 0.1;  // 0.1
//	weight[0]=0.0;
//  }
//  if ((fabs(x[1]) > 4.) | (fabs(y[1]) > 4.)) {
//	PointingData[point_index].pss2_snr = 0.1;  // 0.1
//	weight[1]=0.0;
//  }
////  if ((fabs(x[2]) > 4.) | (fabs(y[2]) > 4.)) {
////	PointingData[point_index].pss3_snr = 0.1;  // 0.1
////	weight[2]=0.0;
////  }
////  if ((fabs(x[3]) > 4.) | (fabs(y[3]) > 4.)) {
////	PointingData[point_index].pss4_snr = 0.1;  // 0.1
////	weight[3]=0.0;
////  }

    /* get current sun az, el */
    calc_sun_position(PointingData[i_point].t, &sun_ra, &sun_dec);
    sun_ra *= (12.0 / M_PI);
    sun_dec *= (180.0 / M_PI);

    if (sun_ra < 0)
    sun_ra += 24;

    equatorial_to_horizontal(sun_ra, sun_dec, PointingData[i_point].lst,
            PointingData[i_point].lat, &sun_az, &sun_el);

    NormalizeAngle(&sun_az);
    PointingData[point_index].sun_az = sun_az;
    PointingData[point_index].sun_el = sun_el;
//
//  weightsum=weight[0]+weight[1]+weight[2]+weight[3];
//  if (weightsum == 0 ) {
//    return 0;
//  }
//
//  // Define beta (az rotation)
//  beta[0] = (M_PI/180.)*(PSS1_BETA + CommandData.cal_off_pss1);
//  beta[1] = (M_PI/180.)*(PSS2_BETA + CommandData.cal_off_pss2);
//  beta[2] = (M_PI/180.)*(PSS3_BETA + CommandData.cal_off_pss3);
//  beta[3] = (M_PI/180.)*(PSS4_BETA + CommandData.cal_off_pss4);
//
//  // Define alpha (el rotation)
//  alpha[0] = (M_PI/180.)*PSS1_ALPHA;
//  alpha[1] = (M_PI/180.)*PSS2_ALPHA;
//  alpha[2] = (M_PI/180.)*PSS3_ALPHA;
//  alpha[3] = (M_PI/180.)*PSS4_ALPHA;
//  // Define psi (roll)
//  psi[0] = (M_PI/180.)*PSS1_PSI;
//  psi[1] = (M_PI/180.)*PSS2_PSI;
//  psi[2] = (M_PI/180.)*PSS3_PSI;
//  psi[3] = (M_PI/180.)*PSS4_PSI;
//
//  //TODO: Remove GSL nonsense.  Replace with calculation
//  for (i=0; i<4; i++) {
//  	rot[i] = gsl_matrix_alloc(3,3);
//  	rxalpha[i] = gsl_matrix_alloc(3,3);
//  	rzpsi[i] = gsl_matrix_alloc(3,3);
//
//	gsl_matrix_set(rxalpha[i], 0, 0, 1.); gsl_matrix_set(rxalpha[i], 0, 1, 0.);             gsl_matrix_set(rxalpha[i], 0, 2, 0.);
//	gsl_matrix_set(rxalpha[i], 1, 0, 0.); gsl_matrix_set(rxalpha[i], 1, 1, cos(-alpha[i])); gsl_matrix_set(rxalpha[i], 1, 2, -sin(-alpha[i]));
//	gsl_matrix_set(rxalpha[i], 2, 0, 0.); gsl_matrix_set(rxalpha[i], 2, 1, sin(-alpha[i])); gsl_matrix_set(rxalpha[i], 2, 2, cos(-alpha[i]));
//
//	gsl_matrix_set(rzpsi[i], 0, 0, cos(psi[i]));  gsl_matrix_set(rzpsi[i], 0, 1, -sin(psi[i])); gsl_matrix_set(rzpsi[i], 0, 2, 0.);
//	gsl_matrix_set(rzpsi[i], 1, 0, sin(psi[i]));  gsl_matrix_set(rzpsi[i], 1, 1, cos(psi[i]));  gsl_matrix_set(rzpsi[i], 1, 2, 0.);
//	gsl_matrix_set(rzpsi[i], 2, 0, 0.);           gsl_matrix_set(rzpsi[i], 2, 1, 0.);           gsl_matrix_set(rzpsi[i], 2, 2, 1.);
//
//	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
//                 1.0, rxalpha[i], rzpsi[i],
//                 0.0, rot[i]);
//
//	// identity is the inverse of the rotation matrix
//	u2[i][0] = gsl_matrix_get(rot[i], 0, 0)*usun[i][0]
//        	 + gsl_matrix_get(rot[i], 0, 1)*usun[i][1]
//        	 + gsl_matrix_get(rot[i], 0, 2)*usun[i][2];
//	u2[i][1] = gsl_matrix_get(rot[i], 1, 0)*usun[i][0]
//        	 + gsl_matrix_get(rot[i], 1, 1)*usun[i][1]
//        	 + gsl_matrix_get(rot[i], 1, 2)*usun[i][2];
//	u2[i][2] = gsl_matrix_get(rot[i], 2, 0)*usun[i][0]
//        	 + gsl_matrix_get(rot[i], 2, 1)*usun[i][1]
//        	 + gsl_matrix_get(rot[i], 2, 2)*usun[i][2];
//
//	// az is "az_rel_sun"
//	az[i] = atan(u2[i][0]/u2[i][2]);                // az is in radians
//  	azraw[i] = sun_az + (180./M_PI)*(az[i] - beta[i]);
//  	elraw[i] = (180./M_PI)*atan(u2[i][1]/sqrt(u2[i][0]*u2[i][0]+u2[i][2]*u2[i][2]));
//  }
//  PointingData[point_index].pss1_azraw = azraw[0];
//  PointingData[point_index].pss2_azraw = azraw[1];
////  PointingData[point_index].pss3_azraw = azraw[2];
////  PointingData[point_index].pss4_azraw = azraw[3];
//  PointingData[point_index].pss1_elraw = elraw[0];
//  PointingData[point_index].pss2_elraw = elraw[1];
////  PointingData[point_index].pss3_elraw = elraw[2];
////  PointingData[point_index].pss4_elraw = elraw[3];
//  for (i=0; i<4; i++) {
//  	gsl_matrix_free(rot[i]);
//  	gsl_matrix_free(rxalpha[i]);
//  	gsl_matrix_free(rzpsi[i]);
//  }
//
//  new_val = (weight[0]*azraw[0] + weight[1]*azraw[1] + weight[2]*azraw[2] + weight[3]*azraw[3])/weightsum;
//
//  if ((!isinf(new_val)) && (!isnan(new_val))) {
//    *azraw_pss = new_val;
//  } else {
//    *azraw_pss = 0.0;
//    return 0;
//  }
//
//  new_val = (weight[0]*elraw[0] + weight[1]*elraw[1] + weight[2]*elraw[2] + weight[3]*elraw[3])/weightsum;
//  if ((!isinf(new_val)) && (!isnan(new_val))) {
//    *elraw_pss = new_val;
//  } else {
//    *elraw_pss = 0.0;
//    return 0;
//  }
//
//  NormalizeAngle(azraw_pss);
//  NormalizeAngle(elraw_pss);

//  PointingData[point_index].pss_azraw = *azraw_pss;
//  PointingData[point_index].pss_elraw = *elraw_pss;

  return 1;

}
////////////////NNG//////////////////


//make history 30s long so that nobody ever exceeds it (with sc_max_age)
#define GY_HISTORY 3000
static void RecordHistory(int index)
{
  /*****************************************/
  /*   Allocate Memory                     */
  if (hs.ifel_gy_history == NULL) {
    hs.ifel_gy_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.ifel_gy_history, 0, GY_HISTORY * sizeof(double));
    hs.ifroll_gy_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.ifroll_gy_history, 0, GY_HISTORY * sizeof(double));
    hs.ifyaw_gy_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.ifyaw_gy_history, 0, GY_HISTORY * sizeof(double));
    hs.elev_history  = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.elev_history, 0, GY_HISTORY * sizeof(double));
  }

  /*****************************************/
  /* record history                        */
  hs.i_history++;
  if (hs.i_history >= GY_HISTORY)
    hs.i_history = 0;

  hs.ifel_gy_history[hs.i_history] = RG.ifel_gy;
  hs.ifroll_gy_history[hs.i_history] = RG.ifroll_gy;
  hs.ifyaw_gy_history[hs.i_history] = RG.ifyaw_gy;
  hs.elev_history[hs.i_history] = PointingData[index].el * M_PI / 180.0;
}

int possible_solution(double az, double el, int i_point) {
  double mag_az, enc_el, d_az;
  
  // test for insanity
  if (!finite(az)) return(0);
  if (!finite(el)) return(0);
  if (el > 70.0) return (0);
  if (el < 0.0) return(0);

  mag_az = PointingData[i_point].mag_az; 

  if (CommandData.use_elenc) {
    enc_el = ACSData.enc_elev;
    if (el - enc_el > 5.0) return (0);
    if (enc_el - el > 5.0) return (0);
  }
  
  if (CommandData.use_mag) {
    d_az = az - mag_az;

    if (d_az > 180.0) d_az -= 360;
    if (d_az < -180.0) d_az += 360;
  
    if (d_az > 30.0) return (0);
    if (d_az < -30.0) return (0);
  }


  return(1);
}

static bool XSCHasNewSolution(int which)
{
    if (!xsc_server_data(which).channels[xN_image_eq_valid].value_bool) {
        // The latest solution isn't good
        return false;
    }
    if (xsc_server_data(which).channels[xN_image_ctr_stars].value_int == xsc_pointing_state[which].last_solution_stars_counter) {
        // The solution is old
        return false;
    }
    if (xsc_server_data(which).channels[xN_image_ctr_stars].value_int < 0 || ACSData.last_trigger_counter_stars[which] < 0) {
        // One or both of the stars counters were not yet running at the time of the trigger
        return false;
    }
    if (xsc_server_data(which).channels[xN_image_ctr_fcp].value_int < 0 ||  ACSData.last_trigger_counter_fcp[which] < 0) {
        // One or both of the fcp counters were not yet running at the time of the trigger
        return false;
    }
    if (xsc_server_data(which).channels[xN_image_ctr_stars].value_int != ACSData.last_trigger_counter_stars[which]) {
        // The stars counters were misaligned at the time of the trigger
        return false;
    }
    if (xsc_server_data(which).channels[xN_image_ctr_fcp].value_int != ACSData.last_trigger_counter_fcp[which]) {
        // The fcp counters were misaligned at the time of the trigger
        return false;
    }
    return true;
}

static void EvolveXSCSolution(struct ElSolutionStruct *e,
    struct AzSolutionStruct *a, double gy1, double gy1_off, double gy2,
    double gy2_off, double gy3, double gy3_off, double old_el, int which)
{

  //int i_point;
  double gy_az;
  double new_az, new_el, ra, dec;

  // When we get a new frame, use these to correct for history
  double gy_el_delta = 0;
  double gy_az_delta = 0;
  int i, j;

  double w1, w2;

  double el_frame = from_degrees(old_el + 22.0);

  // evolve el
  e->angle += (gy1 + gy1_off) / SR;
  e->variance += GYRO_VAR;

  // evolve az
  gy_az = (gy2 + gy2_off) * sin(el_frame) + (gy3 + gy3_off) * cos(el_frame);
  a->angle += gy_az / SR;
  a->variance += GYRO_VAR;

  if (XSCHasNewSolution(which)) {
    xsc_pointing_state[which].last_solution_stars_counter = xsc_server_data(which).channels[xN_image_ctr_stars].value_int;
    blast_info(" xsc%i: received new solution", which);
    if (ACSData.last_trigger_age_cs[which] < GY_HISTORY_AGE_CS) {
        blast_info(" xsc%i: new solution young enough to accept", which);
      //i_point = GETREADINDEX(point_index);
      ra = to_hours(xsc_server_data(which).channels[xN_image_eq_ra].value_double);
      dec = to_degrees(xsc_server_data(which).channels[xN_image_eq_dec].value_double);
      blast_dbg("xsc%i solution is ra, dec: %f, %f (deg)\n", which, to_degrees(from_hours(ra)), dec);

      equatorial_to_horizontal(ra, dec, ACSData.last_trigger_lst[which], ACSData.last_trigger_lat[which], &new_az, &new_el);

      blast_info("converted xsc%i solution to az el %f, %f\n", which, new_az, new_el);
      xsc_pointing_state[which].az = new_az;
      xsc_pointing_state[which].el = new_el;

      //bprintf(loglevel_info, "Pointing: Solution from XSC%i: Ra = %f hours\n", which, ra);
      //bprintf(loglevel_info, "Pointing: Solution from XSC%i: Dec = %f degrees\n", which, dec);
      //bprintf(loglevel_info, "Pointing: Solution from XSC%i: sigma = %f rad\n", which, xsc_server_data(which).sigma);
      //bprintf(loglevel_info, "Pointing: Solution from XSC%i: az = %f degrees\n", which, new_az);
      //bprintf(loglevel_info, "Pointing: Solution from XSC%i: el = %f degrees\n", which, new_el);

      /* Add BDA offset -- there's a pole here at EL = 90 degrees! */

      new_az += to_degrees(approximate_az_from_cross_el(CommandData.XSC[which].cross_el_trim, from_degrees(old_el)));
      new_el += to_degrees(CommandData.XSC[which].el_trim);

      // This solution is xsc_pointing_data.age_last_stars_solution old: how much have we moved?
      gy_el_delta = 0;
      gy_az_delta = 0;
      for (i = 0; i < ACSData.last_trigger_age_cs[which]; i++) {
        j = hs.i_history - i;
        if (j < 0)
          j += GY_HISTORY_AGE_CS;

        gy_el_delta += (hs.ifel_gy_history[j] + gy1_off) * (1.0 / SR);
        //TODO: Double-check these static offsets
        gy_az_delta += ((hs.ifyaw_gy_history[j] + gy2_off) * sin(hs.elev_history[j] + 22.0*M_PI/180.0)
        + (hs.ifroll_gy_history[j] + gy3_off) * cos(hs.elev_history[j] + 22.0*M_PI/180.0))
        * (1.0 / SR);
      }

      // Evolve el solution
      e->angle -= gy_el_delta; // rewind to when the frame was grabbed
      a->angle -= gy_az_delta; // rewind to when the frame was grabbed

      //bprintf(loglevel_info,"POINTING: Rewinded old SC AZ EL is %f %f\n",a->angle, e->angle);
      //bprintf(loglevel_info, "CHAPPY: Az averaging old: %f,  and new: %f\n", a->angle, new_az);

      w1 = 1.0 / (e->variance);
      if (xsc_server_data(which).channels[xN_image_eq_sigma_pointing].value_double > M_PI) {
        w2 = 0.0;
      } else {
        w2 = 10.0 * xsc_server_data(which).channels[xN_image_eq_sigma_pointing].value_double
          * (180.0 / M_PI); //e->samp_weight;
        if (w2 > 0.0)
          w2 = 1.0 / (w2 * w2);
        else
          w2 = 0.0; // shouldn't happen
      }
      //bprintf(loglevel_info,"POINTING: ESC%i_SIGMA and SC weight w2:  %f %f\n", which,
        //xsc_server_data(which).sigma,w2);

      UnwindDiff(e->angle, &new_el);
      UnwindDiff(a->angle, &new_az);

      e->angle = (w1 * e->angle + new_el * w2) / (w1 + w2);
      //bprintf(loglevel_info,"POINTING: Rewinded averaged SC  EL is %f\n", e->angle);
      e->variance = 1.0 / (w1 + w2);
      e->angle += gy_el_delta; // add back to now
      e->angle = normalize_angle_360(e->angle);

      // evolve az solution
      w1 = 1.0 / (a->variance);
      // w2 already set

      a->angle = (w1 * a->angle + new_az * w2) / (w1 + w2);
      //bprintf(loglevel_info,"POINTING: Rewinded averaged SC AZ is %f\n", a->angle);
      a->variance = 1.0 / (w1 + w2);
      a->angle += gy_az_delta; // add back to now
      a->angle = normalize_angle_360(a->angle);

      //bprintf(loglevel_info, "CHAPPY: Az result is: %f\n", a->angle);
      //bprintf(loglevel_info,"POINTING: Evolved SC AZ EL is %f %f\n",a->angle, e->angle);

    }
  }
}

/* Gyro noise: 7' / rt(hour) */
/** the new solution is a weighted mean of:
  the old solution evolved by gyro motion and
  the new solution. **/
static void EvolveElSolution(struct ElSolutionStruct *s,
    double gyro, double gy_off,
    double new_angle, int new_reading)
{
  static int i=0;
  double w1, w2;
  double new_offset = 0;
 
  s->angle += (gyro + gy_off) / SR;
  s->variance += GYRO_VAR; ///TODO: The El solution is not necessarily only one gyro

  s->gy_int += gyro / SR; // in degrees

  if (new_reading) {
    w1 = 1.0 / (s->variance);
    w2 = s->samp_weight;

    UnwindDiff(s->angle, &new_angle);
    s->angle = (w1 * s->angle + new_angle * w2) / (w1 + w2);
    s->variance = 1.0 / (w1 + w2);
    NormalizeAngle(&(s->angle));

    if (CommandData.pointing_mode.nw == 0) { /* not in slew veto */
      /** calculate offset **/
      if (s->n_solutions > 10) { // only calculate if we have had at least 10
        new_offset = ((new_angle - s->last_input) - s->gy_int) /
          ((1.0/SR) * (double)s->since_last);
	
        if (fabs(new_offset) > 500.0)
          new_offset = 0; // 5 deg step is bunk!
	
	
        s->offset_gy = filter(new_offset, s->fs);
      }
      s->since_last = 0;
      if (s->n_solutions<10000) {
        s->n_solutions++;
      }
    }

    s->gy_int = 0.0;
    s->last_input = new_angle;
  }
  s->since_last++;
  i++;
}

// Weighted mean of ElAtt and ElSol
static void AddElSolution(struct ElAttStruct *ElAtt,
    struct ElSolutionStruct *ElSol, int add_offset)
{
  double weight, var;

  var = ElSol->variance + ElSol->sys_var;

  if (var > 0)
    weight = 1.0 / var;
  else
    weight = 1.0E30; // should be impossible

  ElAtt->el = (weight * (ElSol->angle + ElSol->trim) +
      ElAtt->weight * ElAtt->el) /
    (weight + ElAtt->weight);

  if (add_offset) {
    ElAtt->offset_gy = (weight * ElSol->offset_gy +
        ElAtt->weight * ElAtt->offset_gy) /
      (weight + ElAtt->weight);
  }

  ElAtt->weight += weight;
}

// Weighted mean of AzAtt and AzSol
static void AddAzSolution(struct AzAttStruct *AzAtt,
    struct AzSolutionStruct *AzSol, int add_offset)
{
  double weight, var, az;
  var = AzSol->variance + AzSol->sys_var;
  az = AzSol->angle + AzSol->trim;

  if (var > 0)
    weight = 1.0 / var;
  else
    weight = 1.0E30; // should be impossible

  UnwindDiff(AzAtt->az, &az);
  AzAtt->az = (weight * (az) + AzAtt->weight * AzAtt->az) /
    (weight + AzAtt->weight);
  NormalizeAngle(&(AzAtt->az));

  if (add_offset) {
    AzAtt->offset_ifroll_gy = (weight * AzSol->offset_ifroll_gy +
        AzAtt->weight * AzAtt->offset_ifroll_gy) / (weight + AzAtt->weight);
    AzAtt->offset_ifyaw_gy = (weight * AzSol->offset_ifyaw_gy +
        AzAtt->weight * AzAtt->offset_ifyaw_gy) / (weight + AzAtt->weight);
  }

  AzAtt->weight += weight;
}

static void EvolveAzSolution(struct AzSolutionStruct *s, double ifroll_gy,
    double offset_ifroll_gy, double ifyaw_gy, double offset_ifyaw_gy, double el, double new_angle,
    int new_reading)
{
  double w1, w2;
  double gy_az;
  double new_offset, daz;

  el *= M_PI / 180.0; // want el in radians
  gy_az = (ifroll_gy + offset_ifroll_gy) * sin(el) + (ifyaw_gy + offset_ifyaw_gy) * cos(el);

  s->angle += gy_az / SR;
  s->variance += GYRO_VAR; ///TODO: The Az solution is not necessarily only one gyro

  s->ifroll_gy_int += ifroll_gy / SR; // in degrees
  s->ifyaw_gy_int += ifyaw_gy / SR; // in degrees

  if (new_reading) {
    w1 = 1.0 / (s->variance);
    w2 = s->samp_weight;

    UnwindDiff(s->angle, &new_angle);
    s->angle = (w1 * s->angle + new_angle * w2) / (w1 + w2);
    s->variance = 1.0 / (w1 + w2);
    NormalizeAngle(&(s->angle));

    if (CommandData.pointing_mode.nw == 0) { /* not in slew veto */
      if (s->n_solutions > 10) { // only calculate if we have had at least 10
	
	daz = remainder(new_angle - s->last_input, 360.0);
	
	/* Do Gyro_IFroll */
	new_offset = -(daz * cos(el) + s->ifroll_gy_int) /
	  ((1.0/SR) * (double)s->since_last);
	s->offset_ifroll_gy = filter(new_offset, s->fs2);;
	
	/* Do Gyro_IFyaw */
	new_offset = -(daz * sin(el) + s->ifyaw_gy_int) /
	  ((1.0/SR) * (double)s->since_last);
	s->offset_ifyaw_gy = filter(new_offset, s->fs3);;
	
      }
      s->since_last = 0;
      if (s->n_solutions<10000) {
        s->n_solutions++;
      }
    }
    s->ifroll_gy_int = 0.0;
    s->ifyaw_gy_int = 0.0;
    s->last_input = new_angle;
  }
  s->since_last++;
}

void AutoTrimToSC();    //defined below

/*****************************************************************
  do sensor selection;
  update the pointing;
  */
/* Elevation encoder uncertainty: */
void Pointing(void)
{
  double R, cos_e, cos_l, cos_a;
  double sin_e, sin_l, sin_a;
  double ra, dec, az, el;
  static int j=0;

  int mag_ok;
  int pss_ok;
  static unsigned pss_since_ok = 500;
  double mag_az;
  double pss_az = 0;
  double pss_el = 0;
  double clin_elev;
  static double last_good_lat=0, last_good_lon=0, last_good_alt = 0;
  static int i_at_float = 0;
  double trim_change;

  static int firsttime = 1;

  int i_point_read;

  static struct LutType elClinLut = {"/data/etc/blast/clin_elev.lut",0,NULL,NULL,0};

  struct ElAttStruct ElAtt = {0.0, 0.0, 0.0};
  struct AzAttStruct AzAtt = {0.0, 0.0, 0.0, 0.0};

  static struct ElSolutionStruct EncEl = {0.0, // starting angle
    360.0 * 360.0, // starting variance
    1.0 / M2DV(60), //sample weight
    M2DV(20), // systematic variance
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    OFFSET_GY_IFEL, // gy offset
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL // firstruct					
  };
  static struct ElSolutionStruct EncMotEl = {0.0, // starting angle
    360.0 * 360.0, // starting variance
    1.0 / M2DV(60), //sample weight
    M2DV(20), // systematic variance
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    OFFSET_GY_IFEL, // gy offset
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL // firstruct					
  };
  static struct ElSolutionStruct ClinEl = {0.0, // starting angle
    360.0 * 360.0, // starting variance
    1.0 / M2DV(60), //sample weight
    M2DV(60), // systematic variance
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    OFFSET_GY_IFEL, // gy offset
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL // firstruct					
  };
  static struct ElSolutionStruct ISCEl = {0.0, // starting angle
    719.9 * 719.9, // starting variance
    1.0 / M2DV(0.2), //sample weight
    M2DV(0.2), // systemamatic variance
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    OFFSET_GY_IFEL, // gy offset
    0.0001, // filter constant
    0, 0 // n_solutions, since_last
  };
  static struct ElSolutionStruct OSCEl = {0.0, // starting angle
    719.9 * 719.9, // starting varience
    1.0 / M2DV(0.2), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    OFFSET_GY_IFEL, // gy offset
    0.0001, // filter constant
    0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct NullAz = {91.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(6), //sample weight
    M2DV(6000), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    OFFSET_GY_IFROLL, OFFSET_GY_IFYAW, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct MagAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(120.0), //sample weight
    M2DV(90), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    OFFSET_GY_IFROLL, OFFSET_GY_IFYAW, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };

  static struct AzSolutionStruct PSSAz =  {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(30), //sample weight
    M2DV(60), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    OFFSET_GY_IFROLL, OFFSET_GY_IFYAW, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
    };
  //TODO:Replace ISC/OSC Az Solutions with XSC
  static struct AzSolutionStruct ISCAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(0.3), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    OFFSET_GY_IFROLL, OFFSET_GY_IFYAW, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct OSCAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(0.3), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    OFFSET_GY_IFROLL, OFFSET_GY_IFYAW, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };

    if (firsttime) {
        firsttime = 0;
        ClinEl.trim = CommandData.clin_el_trim;
        EncEl.trim = CommandData.enc_el_trim;
        EncMotEl.trim = CommandData.enc_motor_el_trim;
        NullAz.trim = CommandData.null_az_trim;
        MagAz.trim = CommandData.mag_az_trim;
        PSSAz.trim = CommandData.pss_az_trim;

        ClinEl.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        initFir(ClinEl.fs, FIR_LENGTH);
        EncEl.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        initFir(EncEl.fs, FIR_LENGTH);
        EncMotEl.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        initFir(EncMotEl.fs, FIR_LENGTH);

        NullAz.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        NullAz.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        initFir(NullAz.fs2, (int) (10)); // not used
        initFir(NullAz.fs3, (int) (10)); // not used

        MagAz.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        MagAz.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        initFir(MagAz.fs2, FIR_LENGTH);
        initFir(MagAz.fs3, FIR_LENGTH);

        PSSAz.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        PSSAz.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        initFir(PSSAz.fs2, FIR_LENGTH);
        initFir(PSSAz.fs3, FIR_LENGTH);

        // the first t about to be read needs to be set
        PointingData[GETREADINDEX(point_index)].t = mcp_systime(NULL); // CPU time

        /* Load lat/lon from disk */
        last_good_lon = PointingData[0].lon = PointingData[1].lon = PointingData[2].lon = CommandData.lon;
        last_good_lat = PointingData[0].lat = PointingData[1].lat = PointingData[2].lat = CommandData.lat;
    }

    if (elClinLut.n == 0)
        LutInit(&elClinLut);

    i_point_read = GETREADINDEX(point_index);

    // Make aristotle correct
    R = 15.0 / 3600.0;
    sincos(from_degrees(PointingData[i_point_read].el), &sin_e, &cos_e);
    sincos(from_degrees(PointingData[i_point_read].lat), &sin_l, &cos_l);
    sincos(from_degrees(PointingData[i_point_read].az), &sin_a, &cos_a);

    PointingData[point_index].ifel_earth_gy = R * (-cos_l * sin_a);
    PointingData[point_index].ifroll_earth_gy = R * (cos_e * sin_l - cos_l * sin_e * cos_a);
    PointingData[point_index].ifyaw_earth_gy = R * (sin_e * sin_l + cos_l * cos_e * cos_a);
    RG.ifel_gy = ACSData.ifel_gy - PointingData[point_index].ifel_earth_gy;
    RG.ifroll_gy = ACSData.ifroll_gy - PointingData[point_index].ifroll_earth_gy;
    RG.ifyaw_gy = ACSData.ifyaw_gy - PointingData[point_index].ifyaw_earth_gy;

    PointingData[point_index].v_az = RG.ifyaw_gy * cos_e + RG.ifroll_gy * sin_e;
    /*************************************/
    /** Record history for gyro offsets **/
    RecordHistory(i_point_read);

    PointingData[point_index].t = mcp_systime(NULL); // CPU time

    /************************************************/
    /** Set the official Lat and Lon **/
    last_good_lat = SIPData.GPSpos.lat;
    last_good_lon = SIPData.GPSpos.lon;
    last_good_alt = SIPData.GPSpos.alt;

    PointingData[point_index].lat = last_good_lat;
    PointingData[point_index].lon = last_good_lon;
    PointingData[point_index].alt = last_good_alt;

    /* At float check */
    if (PointingData[point_index].alt < FLOAT_ALT) {
        PointingData[point_index].at_float = 0;
        i_at_float = 0;
    }
    else {
        i_at_float++;
        if (i_at_float > FRAMES_TO_OK_ATFLOAT) {
            PointingData[point_index].at_float = 1;
        }
        else {
            PointingData[point_index].at_float = 0;
        }
    }

    /* Save lat/lon */
    CommandData.lat = PointingData[point_index].lat;
    CommandData.lon = PointingData[point_index].lon;

    /*****************************/
    /** set time related things **/
    PointingData[point_index].mcp_frame = ACSData.mcp_frame;
    PointingData[point_index].t = mcp_systime(NULL); // for now use CPU time

    /** Set LST and local sidereal date **/
    PointingData[point_index].lst = to_seconds(time_lst_unix(PointingData[point_index].t, from_degrees(PointingData[point_index].lon)));


    /*************************************/
    /**      do ISC Solution            **/
    EvolveXSCSolution(&ISCEl, &ISCAz,
        RG.ifel_gy,   PointingData[i_point_read].offset_ifel_gy,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el, 0);

    /*************************************/
    /**      do OSC Solution            **/
    EvolveXSCSolution(&OSCEl, &OSCAz,
        RG.ifel_gy,   PointingData[i_point_read].offset_ifel_gy,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el, 1);

    /*************************************/
    /**      do elevation solution      **/
    clin_elev = LutCal(&elClinLut, ACSData.clin_elev);
    PointingData[i_point_read].clin_el_lut = clin_elev;

    EvolveElSolution(&ClinEl, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            clin_elev, 1);
    EvolveElSolution(&EncEl, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            ACSData.enc_elev, 1);
    EvolveElSolution(&EncMotEl, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            ACSData.enc_motor_elev, 1);

    if (CommandData.use_elenc) {
        AddElSolution(&ElAtt, &EncEl, 1);
    }
    if (CommandData.use_elmotenc) {
        AddElSolution(&ElAtt, &EncMotEl, 1);
    }

    if (CommandData.use_elclin) {
        AddElSolution(&ElAtt, &ClinEl, 1);
    }
    if (CommandData.use_xsc0) {
        AddElSolution(&ElAtt, &ISCEl, 0);
    }

    if (CommandData.use_xsc1) {
        AddElSolution(&ElAtt, &OSCEl, 0);
    }

    if (CommandData.el_autogyro)
        PointingData[point_index].offset_ifel_gy = ElAtt.offset_gy;
    else
        PointingData[point_index].offset_ifel_gy = CommandData.offset_ifel_gy;

    PointingData[point_index].el = ElAtt.el;

    /*******************************/
    /**      do az solution      **/
    /** Convert Sensors **/
    mag_ok = MagConvert(&mag_az);

    PointingData[point_index].mag_az = mag_az;

    pss_ok = PSSConvert(&pss_az, &pss_el);
    if (pss_ok) {
        pss_since_ok = 0;
    }
    else {
        pss_since_ok++;
    }
    PointingData[point_index].pss_ok = pss_ok;

    /** evolve solutions **/
    EvolveAzSolution(&NullAz,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        0.0, 0);
    /** MAG Az **/
    EvolveAzSolution(&MagAz,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        mag_az, mag_ok);

    /** PSS **/
    EvolveAzSolution(&PSSAz,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        pss_az, pss_ok);

    if (CommandData.fast_offset_gy > 0) {
        CommandData.fast_offset_gy--;
    }

    //blast_info("off: %g %g %g %g\n", EncEl.angle, ClinEl.angle, EncEl.offset_gy, ClinEl.offset_gy);

    AddAzSolution(&AzAtt, &NullAz, 1);
    /** add az solutions **/
    if (CommandData.use_mag) {
        AddAzSolution(&AzAtt, &MagAz, 1);
    }
    if (CommandData.use_pss) {
        AddAzSolution(&AzAtt, &PSSAz, 1);
    }
    if (CommandData.use_xsc0) {
        AddAzSolution(&AzAtt, &ISCAz, 0);
    }
    if (CommandData.use_xsc1) {
        AddAzSolution(&AzAtt, &OSCAz, 0);
    }

    PointingData[point_index].offset_ifrollmag_gy = MagAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyawmag_gy = MagAz.offset_ifyaw_gy;

    PointingData[point_index].offset_ifrollpss_gy = PSSAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyawpss_gy = PSSAz.offset_ifyaw_gy;

    //if(j==500) blast_info("Pointing use_mag = %i, use_sun = %i, use_gps = %i, use_isc = %i, use_osc = %i",CommandData.use_mag,CommandData.use_sun, CommandData.use_gps, CommandData.use_isc, CommandData.use_osc);
    PointingData[point_index].az = AzAtt.az;
    if (CommandData.az_autogyro) {
        PointingData[point_index].offset_ifroll_gy = AzAtt.offset_ifroll_gy;
        PointingData[point_index].offset_ifyaw_gy = AzAtt.offset_ifyaw_gy;
    }
    else {
        PointingData[point_index].offset_ifroll_gy = CommandData.offset_ifroll_gy;
        PointingData[point_index].offset_ifyaw_gy = CommandData.offset_ifyaw_gy;
    }

    /** calculate ra/dec for convenience on the ground **/
    horizontal_to_equatorial(PointingData[point_index].az, PointingData[point_index].el, PointingData[point_index].lst, PointingData[point_index].lat, &ra, &dec);
    equatorial_to_horizontal(ra, dec, PointingData[point_index].lst, PointingData[point_index].lat, &az, &el);

    PointingData[point_index].ra = ra;
    PointingData[point_index].dec = dec;
    /** record solutions in pointing data **/
    //  if (j%500==0) blast_info("Pointing: PointingData.enc_el = %f",PointingData[point_index].enc_el);
    PointingData[point_index].enc_el = EncEl.angle;
    PointingData[point_index].enc_sigma = sqrt(EncEl.variance + EncEl.sys_var);
    PointingData[point_index].enc_motor_el = EncMotEl.angle;
    PointingData[point_index].enc_motor_sigma = sqrt(EncMotEl.variance + EncMotEl.sys_var);
    PointingData[point_index].clin_el = ClinEl.angle;
    PointingData[point_index].clin_sigma = sqrt(ClinEl.variance + ClinEl.sys_var);

    PointingData[point_index].mag_az = MagAz.angle;
    PointingData[point_index].mag_sigma = sqrt(MagAz.variance + MagAz.sys_var);

    // Added 22 June 2010 GT
    PointingData[point_index].pss_az = PSSAz.angle;
    PointingData[point_index].pss_sigma = sqrt(PSSAz.variance + PSSAz.sys_var);

    //TODO:Replace ISC/OSC Pointing data with XSC
    PointingData[point_index].xsc_az[0] = ISCAz.angle;
    PointingData[point_index].xsc_el[0] = ISCEl.angle;
    PointingData[point_index].xsc_sigma[0] = sqrt(ISCEl.variance + ISCEl.sys_var);
    PointingData[point_index].offset_ifel_gy_xsc[0] = ISCEl.offset_gy;
    PointingData[point_index].offset_ifroll_gy_xsc[0] = ISCAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyaw_gy_xsc[0] = ISCAz.offset_ifyaw_gy;

    PointingData[point_index].xsc_az[1] = OSCAz.angle;
    PointingData[point_index].xsc_el[1] = OSCEl.angle;
    PointingData[point_index].xsc_sigma[1] = sqrt(OSCEl.variance + OSCEl.sys_var);
    PointingData[point_index].offset_ifel_gy_xsc[1] = OSCEl.offset_gy;
    PointingData[point_index].offset_ifroll_gy_xsc[1] = OSCAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyaw_gy_xsc[1] = OSCAz.offset_ifyaw_gy;

    /********************/
    /* Set Manual Trims */
    if (CommandData.autotrim_enable)
        AutoTrimToSC();

    if (NewAzEl.fresh == -1) {
        ClinEl.trim = 0.0;
        EncEl.trim = 0.0;
        EncMotEl.trim = 0.0;
        NullAz.trim = 0.0;
        MagAz.trim = 0.0;
        PSSAz.trim = 0.0;
        NewAzEl.fresh = 0;
    }

    if (NewAzEl.fresh == 1) {
        trim_change = (NewAzEl.el - ClinEl.angle) - ClinEl.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        ClinEl.trim += trim_change;

        trim_change = (NewAzEl.el - EncEl.angle) - EncEl.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        EncEl.trim += trim_change;

        trim_change = (NewAzEl.el - EncMotEl.angle) - EncMotEl.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        EncMotEl.trim += trim_change;

        trim_change = (NewAzEl.az - NullAz.angle) - NullAz.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        NullAz.trim += trim_change;

        trim_change = (NewAzEl.az - MagAz.angle) - MagAz.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        MagAz.trim += trim_change;

        if (pss_since_ok < 500) {
            trim_change = (NewAzEl.az - PSSAz.angle) - PSSAz.trim;
            if (trim_change > NewAzEl.rate)
                trim_change = NewAzEl.rate;
            else if (trim_change < -NewAzEl.rate)
                trim_change = -NewAzEl.rate;
            PSSAz.trim += trim_change;
        }

        NewAzEl.fresh = 0;
    }

    point_index = INC_INDEX(point_index);

    CommandData.clin_el_trim = ClinEl.trim;
    CommandData.enc_el_trim = EncEl.trim;
    CommandData.enc_motor_el_trim = EncMotEl.trim;
    CommandData.null_az_trim = NullAz.trim;
    CommandData.mag_az_trim = MagAz.trim;
    CommandData.pss_az_trim = PSSAz.trim;
    j++;

    /* If we are in a slew veto decrement the veto count*/
    if (CommandData.pointing_mode.nw > 0)
        CommandData.pointing_mode.nw--;
}

// called from the command thread in command.h
void SetRaDec(double ra, double dec)
{
  int i_point;
  i_point = GETREADINDEX(point_index);

  equatorial_to_horizontal(ra, dec, PointingData[i_point].lst,
      PointingData[i_point].lat,
      &(NewAzEl.az), &(NewAzEl.el));

  NewAzEl.fresh = 1;
}

// called from the command thread in command.h
void set_position(double m_lat, double m_lon)
{
    SIPData.GPSpos.lat = m_lat;
    SIPData.GPSpos.lon = m_lon;
}

void SetTrimToSC(int which)
{
  int i_point;
  i_point = GETREADINDEX(point_index);
  NewAzEl.az = PointingData[i_point].xsc_az[which];
  NewAzEl.el = PointingData[i_point].xsc_el[which];
  NewAzEl.fresh = 1;
}


void InitializePointingData()
{
  int which;
  for (which=0; which<2; which++) {
    xsc_pointing_state[which].last_trigger.counter_fcp = 0;
    xsc_pointing_state[which].last_trigger.counter_stars = -1;
    xsc_pointing_state[which].last_trigger.age_cs = 100000;
    xsc_pointing_state[which].last_trigger.age_of_end_of_trigger_cs = 100000;
    xsc_pointing_state[which].last_trigger.motion_caz_px = 0.0;
    xsc_pointing_state[which].last_trigger.motion_el_px = 0.0;
    xsc_pointing_state[which].last_trigger.lat = 0.0;
    xsc_pointing_state[which].last_trigger.lst = 0;
    xsc_pointing_state[which].counter_fcp = -1;
    xsc_pointing_state[which].last_counter_fcp = -1;
    xsc_pointing_state[which].last_solution_stars_counter = -1;
    xsc_pointing_state[which].az = 0.0;
    xsc_pointing_state[which].el = 0.0;
    xsc_pointing_state[which].exposure_time_cs = 300;
    xsc_pointing_state[which].predicted_motion_px = 0.0;
  }
  blast_info("InitializePointingData, xsc.az is %f\n", xsc_pointing_state[1].az);
}

void AutoTrimToSC()
{
    int i_point = GETREADINDEX(point_index);
    int isc_good = 0, osc_good = 0;
    static int which = 0;
    time_t t = mcp_systime(NULL);

    if (PointingData[i_point].xsc_sigma[0] > CommandData.autotrim_thresh) {
        CommandData.autotrim_isc_last_bad = t;
    }
    if (PointingData[i_point].xsc_sigma[1] > CommandData.autotrim_thresh) {
        CommandData.autotrim_osc_last_bad = t;
    }

    if (t - CommandData.autotrim_isc_last_bad > CommandData.autotrim_time)
        isc_good = 1;
    if (t - CommandData.autotrim_osc_last_bad > CommandData.autotrim_time)
        osc_good = 1;

    //sticky choice
    if (isc_good && !osc_good && which == 1)
        which = 0;
    if (osc_good && !isc_good && which == 0)
        which = 1;

    if (isc_good || osc_good) {
        NewAzEl.az = PointingData[i_point].xsc_az[which];
        NewAzEl.el = PointingData[i_point].xsc_el[which];
        NewAzEl.rate = CommandData.autotrim_rate / SR;
        NewAzEl.fresh = 1;
    }

}

/**
 * Trims one star camera offset relative to the other.
 * @param m_source Which camera should be used as the zero point for offset
 */
void trim_xsc(int m_source)
{
    int i_point;
    int dest = (m_source == 0);
    double delta_az;
    double delta_el;
    i_point = GETREADINDEX(point_index);
    delta_az = PointingData[i_point].xsc_az[dest] - PointingData[i_point].xsc_az[m_source];
    delta_el = PointingData[i_point].xsc_el[dest] - PointingData[i_point].xsc_el[m_source];
    CommandData.ISCState[dest].azBDA -= DEG2RAD*(delta_az*cos(PointingData[i_point].el*M_PI / 180.0));
    CommandData.ISCState[dest].elBDA -= DEG2RAD*(delta_el);

}

void AzElTrim(double az, double el)
{
  NewAzEl.az = az;
  NewAzEl.el = el;

  NewAzEl.rate = 360.0; //allow arbitrary trim changes
  NewAzEl.fresh = 1;
}

void ClearTrim()
{
  NewAzEl.fresh = -1;
}
