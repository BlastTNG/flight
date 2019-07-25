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

#include "pointing.h"

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

// Include gsl package for PSS array
#include <gsl/gsl_rng.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_matrix.h>

#include "blast.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "lut.h"
#include "tx.h"
#include "fir.h"
#include "ec_motors.h"

#include "dsp1760.h"
#include "EGM9615.h"
#include "geomag2015.h"
#include "angles.h"
#include "framing.h"
#include "xsc_network.h"
#include "xsc_pointing.h"
#include "conversions.h"
#include "time_lst.h"
#include "utilities_pointing.h"
#include "magnetometer.h"
#include "gps.h"
#include "sip.h"

extern int InCharge;
int point_index = 0;
struct PointingDataStruct PointingData[3];
struct XSCPointingState xsc_pointing_state[2] = {{.counter_mcp = 0}};

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
  double variance; // solution's current sample variance
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic variance - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double gy_int; // integral of the gyro since the last solution
  double offset_gy; // averaged offset measurement
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
  struct FirStruct *fs;
// TODO(laura): These next fields are for debugging and should be removed before flight.
  double new_offset_ifel_gy;
  double int_ifel;
  double prev_sol_el;
};

struct AzSolutionStruct {
  double angle;    // solution's current angle
  double variance; // solution's current sample variance
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic variance - can't do better than this
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
// TODO(laura): These next fields are for debugging and should be removed before flight.
  double new_offset_ifyaw_gy;
  double new_offset_ifroll_gy;
  double d_az;
  double int_ifroll;
  double int_ifyaw;
  double prev_sol_az;
};

static struct {
  double az;
  double el;
  int fresh;
  double rate;
} NewAzEl = {0.0, 0.0, 0, 360.0};

typedef struct {
  double *elev_history;
  double *ifel_gy_history;
  double *ifel_gy_offset;
  double *ifroll_gy_history;
  double *ifroll_gy_offset;
  double *ifyaw_gy_history;
  double *ifyaw_gy_offset;
  int i_history;
} gyro_history_t;

typedef struct {
    double ifel_gy;
    double ifel_gy_offset;
    double ifroll_gy;
    double ifroll_gy_offset;
    double ifyaw_gy;
    double ifyaw_gy_offset;
} gyro_reading_t;

#define MAX_SUN_EL 5.0

static double sun_az, sun_el; // set in SSConvert and used in UnwindDiff

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
static int MagConvert(double *mag_az, double *m_el, uint8_t mag_index) {
    static MAGtype_MagneticModel * MagneticModels[1], *TimedMagneticModel;
    static MAGtype_Ellipsoid Ellip;
    static MAGtype_Geoid Geoid;

    float year;
    double mvx, mvy, mvz;
    double raw_mag_az, raw_mag_pitch;
    static double magx_m, magy_m, magx_b, magy_b;
    static double dip;
    static double dec = 0;
    time_t t;
    struct tm now;
    int i_point_read;
    static time_t oldt;
    static int firsttime = 1;
    static uint32_t mag_count = 0;
//    double magx_m, magx_b, magy_m, magy_b;
    int epochs = 1;
    int NumTerms, nMax = 0;

    i_point_read = GETREADINDEX(point_index);

    /******** Obtain correct indexes the first time here ***********/
    if (firsttime) {
        if (!MAG_robustReadMagModels("/data/etc/blast/WMM.COF", &MagneticModels, epochs)) {
            blast_err("/data/etc/blast/WMM.COF not found. Be sure to `make install` mcp.");
            return 0;
        }
        if (nMax < MagneticModels[0]->nMax) nMax = MagneticModels[0]->nMax;
        NumTerms = ((nMax + 1) * (nMax + 2) / 2);
        TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */
        if (MagneticModels[0] == NULL || TimedMagneticModel == NULL) {
            blast_err("Could not allocate memory for magnetic model!");
            return 0;
        }
        MAG_SetDefaults(&Ellip, &Geoid); /* Set default values and constants */
        /* Check for Geographic Poles */

        /* Set EGM96 Geoid parameters */
        Geoid.GeoidHeightBuffer = GeoidHeightBuffer;
        Geoid.Geoid_Initialized = 1;

        oldt = 1;
        firsttime = 0;
    }

    /* Every 10 s, get new data from the magnetic model.
     *
     * dec = magnetic declination (field direction in az)
     * dip = magnetic inclination (field direction in ele)
     *
     * The year must be between 2015.0 and 2020.0 with current model data
     *
     * The functions called are in 'geomag2015.c' */
    if ((t = PointingData[i_point_read].t) > oldt + 10) {
        MAGtype_CoordSpherical CoordSpherical;
        MAGtype_CoordGeodetic CoordGeodetic;
        MAGtype_Date UserDate;
        MAGtype_GeoMagneticElements GeoMagneticElements;
        oldt = t;

        gmtime_r(&t, &now);
        year = 1900 + now.tm_year + now.tm_yday / 365.25;
        UserDate.DecimalYear = year;

        Geoid.UseGeoid = 1;
        CoordGeodetic.HeightAboveGeoid = PointingData[i_point_read].alt / 1000.0;
        CoordGeodetic.phi = PointingData[i_point_read].lat;
        CoordGeodetic.lambda = -PointingData[i_point_read].lon;
        MAG_ConvertGeoidToEllipsoidHeight(&CoordGeodetic, &Geoid);

        /*Convert from geodetic to Spherical Equations: 17-18, WMM Technical report*/
        MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical);
        /* Time adjust the coefficients, Equation 19, WMM Technical report */
        MAG_TimelyModifyMagneticModel(UserDate, MagneticModels[0], TimedMagneticModel);
        /* Computes the geoMagnetic field elements and their time change*/
        MAG_Geomag(Ellip, CoordSpherical, CoordGeodetic, TimedMagneticModel, &GeoMagneticElements);

        dec = GeoMagneticElements.Decl;
        dip = GeoMagneticElements.Incl;
        PointingData[point_index].mag_strength[mag_index] = GeoMagneticElements.H;
    }

    /* The dec is the correction to the azimuth of the magnetic field. */
    /* If negative is west and positive is east, then: */
    /* */
    /*   true bearing = magnetic bearing + dec */
    /* */
    /* Thus, depending on the sign convention, you have to either add or */
    /* subtract dec from az to get the true bearing. (Adam H.) */

//    mvx = (ACSData.mag_x - MAGX_B) / MAGX_M;
//    mvy = (ACSData.mag_y - MAGY_B) / MAGY_M;

    // TODO(seth): Reset calibration values to Reasonable for gauss
//    magx_m = 1.0 / ((double) (CommandData.cal_xmax_mag - CommandData.cal_xmin_mag));
//    magy_m = -1.0 / ((double) (CommandData.cal_ymax_mag - CommandData.cal_ymin_mag));
//
//    magx_b = (CommandData.cal_xmax_mag + CommandData.cal_xmin_mag) * 0.5;
//    magy_b = (CommandData.cal_ymax_mag + CommandData.cal_ymin_mag) * 0.5;

//    mvx = magx_m * (ACSData.mag_x - magx_b);
//    mvy = magy_m * (ACSData.mag_y - magy_b);

    magx_m = (CommandData.cal_xmax_mag[mag_index]-CommandData.cal_xmin_mag[mag_index])/2.0;
    magx_b = (CommandData.cal_xmax_mag[mag_index]+CommandData.cal_xmin_mag[mag_index])/2.0;
    magy_m = (CommandData.cal_ymax_mag[mag_index]-CommandData.cal_ymin_mag[mag_index])/2.0;
    magy_b = (CommandData.cal_ymax_mag[mag_index]+CommandData.cal_ymin_mag[mag_index])/2.0;

    mvx = (ACSData.mag_x[mag_index]-magx_b)/magx_m;
    mvy = (ACSData.mag_y[mag_index]-magy_b)/magy_m;
    mvz = MAGZ_M * (ACSData.mag_z[mag_index] - MAGZ_B);

    raw_mag_az = (-1.0) * (180.0 / M_PI) * atan2(mvy, mvx);
    raw_mag_pitch = (180.0 / M_PI) * atan2(mvz, sqrt(mvx * mvx + mvy * mvy));
    *mag_az = raw_mag_az + dec + CommandData.cal_mag_align[mag_index];
    *m_el = raw_mag_pitch + dip;
/*
    if (((mag_count % 20000) == 0) || ((mag_count % 20000) == 1)) {
        blast_info("cal_xmin_mag = %f, cal_xmax_mag = %f, cal_ymin_mag = %f, cal_ymax_mag = %f",
                   CommandData.cal_xmin_mag[mag_index], CommandData.cal_xmax_mag[mag_index],
                   CommandData.cal_ymin_mag[mag_index], CommandData.cal_ymax_mag[mag_index]);
        blast_info("ACSData.mag_x = %f, ACSData.mag_y = %f", ACSData.mag_x[mag_index], ACSData.mag_y[mag_index]);
        blast_info("magx_m = %f, magx_b = %f, magy_m = %f, magy_b = %f", magx_m, magx_b, magy_m, magy_b);
        blast_info("mvx = %f, mvy = %f, mvz = %f", mvx, mvy, mvz);
        blast_info("raw_mag_az = %f, dec = %f, cal_mag_align = %f, mag_az = %f",
                   raw_mag_az, dec, CommandData.cal_mag_align[mag_index], *mag_az);
    }
*/

#if 0
#warning THE MAGNETIC MODEL HAS BEEN DISABLED
    dec = 0; // disable mag model.
#endif

    NormalizeAngle(mag_az);
    NormalizeAngle(&dec);

    PointingData[point_index].mag_model_dec[mag_index] = dec;
    PointingData[point_index].mag_model_dip[mag_index] = dip;

    mag_count++;
    return (1);
}

static int PSSConvert(double *azraw_pss, double *elraw_pss) {
    int     i_point;
    double  sun_ra, sun_dec;
    double  az[NUM_PSS];
    double	azraw[NUM_PSS];
    double	elraw[NUM_PSS];
    double  new_val;

    static double i_pss[NUM_PSS][NUM_PSS_V];
    double itot[NUM_PSS], itotabs[NUM_PSS];
    double        x[NUM_PSS], y[NUM_PSS];
    double        usun[NUM_PSS][3], u2[NUM_PSS][3];
    gsl_matrix    *rot[NUM_PSS];
    gsl_matrix    *rxalpha[NUM_PSS], *rzpsi[NUM_PSS];

    double weight[NUM_PSS];
    double weightsum;
	// the rough values are defined in pointing.h, don't include commanded cal values
	static double pss_d_rough[NUM_PSS]; // sensor to pinhole in mm
	static double beta_rough[NUM_PSS]; // az in deg
	static double alpha_rough[NUM_PSS]; // el in deg
	static double psi_rough[NUM_PSS]; // roll in deg
	// these are the final values, actually used in the calculation
	static double pss_d[NUM_PSS], beta[NUM_PSS], alpha[NUM_PSS], psi[NUM_PSS];
    double norm[NUM_PSS];
    double pss_imin;
	int i;
	int j;
	int k;
	static int firsttime = 1;

	if (firsttime) {
		pss_d_rough[0] = PSS0_D;
	    pss_d_rough[1] = PSS1_D;
	    pss_d_rough[2] = PSS2_D;
	    pss_d_rough[3] = PSS3_D;
	    pss_d_rough[4] = PSS4_D;
	    pss_d_rough[5] = PSS5_D;

	    beta_rough[0] = PSS0_BETA;
	    beta_rough[1] = PSS1_BETA;
	    beta_rough[2] = PSS2_BETA;
	    beta_rough[3] = PSS3_BETA;
	    beta_rough[4] = PSS4_BETA;
	    beta_rough[5] = PSS5_BETA;

		alpha_rough[0] = PSS0_ALPHA;
        alpha_rough[1] = PSS1_ALPHA;
        alpha_rough[2] = PSS2_ALPHA;
        alpha_rough[3] = PSS3_ALPHA;
        alpha_rough[4] = PSS4_ALPHA;
        alpha_rough[5] = PSS5_ALPHA;

		psi_rough[0] = PSS0_PSI;
        psi_rough[1] = PSS1_PSI;
        psi_rough[2] = PSS2_PSI;
        psi_rough[3] = PSS3_PSI;
        psi_rough[4] = PSS4_PSI;
        psi_rough[5] = PSS5_PSI;

		firsttime = 0;
	}

	for (j = 0; j < NUM_PSS; j++) {
		for (k = 0; k < NUM_PSS_V; k++) {
			i_pss[j][k] = ACSData.pss_i[j][k];
		}
	}
    for (j = 0; j < NUM_PSS; j++) {
		itot[j] = 0;
		itotabs[j] = 0;
		for (k = 0; k < NUM_PSS_V; k++) {
			// calculate total current for x,y calculation
    		itot[j] += i_pss[j][k];
			// and the sum of absolute valued currents for SNR
			itotabs[j] += fabs(i_pss[j][k]);
		}
    }

    pss_imin = CommandData.cal_imin_pss;
	// blast_info("PSS1 values: v1_1_pss:%f v2_1_pss:%f v3_1_pss:%f v4_1_pss:%f", i_pss[0][0],
	// 				i_pss[0][1], i_pss[0][2], i_pss[0][3]);
	// blast_info("PSS itot[0]=%f, pss_imin=%f, fabs(itot[0])=%f", itot[0], pss_imin, fabs(itot[0]));

    i_point = GETREADINDEX(point_index);

	for (j = 0; j < NUM_PSS; j++) {
		if (fabs(itot[j]) > pss_imin) {
			PointingData[point_index].pss_snr[j] = fabs(itot[j])/CommandData.pss_noise; // 10.
    		weight[j]= PointingData[point_index].pss_snr[j];
		} else {
      		PointingData[point_index].pss_snr[j] = 1.;  // 1.
      		weight[j] = 0.0;
    	}
	}

	for (j = 0; j < NUM_PSS; j++) {
		pss_d[j] = pss_d_rough[j] + CommandData.cal_d_pss[j];
	}

    for (j = 0; j < NUM_PSS; j++) {
    	x[j] = -PSS_XSTRETCH*(PSS_L/2.)*((i_pss[j][3]+i_pss[j][2])-(i_pss[j][0]+i_pss[j][1]))/itot[j];
    	y[j] = -PSS_YSTRETCH*(PSS_L/2.)*((i_pss[j][3]+i_pss[j][1])-(i_pss[j][0]+i_pss[j][2]))/itot[j];
    	norm[j] = sqrt(x[j]*x[j] + y[j]*y[j] + pss_d[j]*pss_d[j]);
    	usun[j][0] = -x[j] / norm[j];
    	usun[j][1] = -y[j] / norm[j];
    	usun[j][2] = pss_d[j] / norm[j];
    }

    // Then spot is at the edge of the sensor
	for (j = 0; j < NUM_PSS; j++) {
    	if ((fabs(x[j]) > 4.) | (fabs(y[j]) > 4.)) {
      		PointingData[point_index].pss_snr[j] = 0.1;  // 0.1
      		weight[j]=0.0;
		}
	}

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

  weightsum = 0;
  for (j = 0; j < NUM_PSS; j++) {
	  weightsum += weight[j];
  }

  if (weightsum == 0) {
    return 0;
  }

  for (j = 0; j < NUM_PSS; j++) {
  // Define beta (az rotation)
  	beta[j] = (M_PI/180.)*(beta_rough[j] + CommandData.cal_az_pss[j] + CommandData.cal_az_pss_array);
  // Define alpha (el rotation)
  	alpha[j] = (M_PI/180.)*(alpha_rough[j] + CommandData.cal_el_pss[j]);
  // Define psi (roll)
  	psi[j] = (M_PI/180.)*(psi_rough[j] + CommandData.cal_roll_pss[j]);
  }

  for (i = 0; i < NUM_PSS; i++) {
  	rot[i] = gsl_matrix_alloc(3, 3);
  	rxalpha[i] = gsl_matrix_alloc(3, 3);
  	rzpsi[i] = gsl_matrix_alloc(3, 3);

	gsl_matrix_set(rxalpha[i], 0, 0, 1.);
	gsl_matrix_set(rxalpha[i], 0, 1, 0.);
  	gsl_matrix_set(rxalpha[i], 0, 2, 0.);
	gsl_matrix_set(rxalpha[i], 1, 0, 0.);
	gsl_matrix_set(rxalpha[i], 1, 1, cos(-alpha[i]));
  	gsl_matrix_set(rxalpha[i], 1, 2, -sin(-alpha[i]));
	gsl_matrix_set(rxalpha[i], 2, 0, 0.);
	gsl_matrix_set(rxalpha[i], 2, 1, sin(-alpha[i]));
  	gsl_matrix_set(rxalpha[i], 2, 2, cos(-alpha[i]));

    gsl_matrix_set(rzpsi[i], 0, 0, cos(psi[i]));
  	gsl_matrix_set(rzpsi[i], 0, 1, -sin(psi[i]));
    gsl_matrix_set(rzpsi[i], 0, 2, 0.);
    gsl_matrix_set(rzpsi[i], 1, 0, sin(psi[i]));
	gsl_matrix_set(rzpsi[i], 1, 1, cos(psi[i]));
    gsl_matrix_set(rzpsi[i], 1, 2, 0.);
    gsl_matrix_set(rzpsi[i], 2, 0, 0.);
	gsl_matrix_set(rzpsi[i], 2, 1, 0.);
    gsl_matrix_set(rzpsi[i], 2, 2, 1.);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
                 1.0, rxalpha[i], rzpsi[i],
                 0.0, rot[i]);

    // identity is the inverse of the rotation matrix
    u2[i][0] = gsl_matrix_get(rot[i], 0, 0)*usun[i][0]
        	 + gsl_matrix_get(rot[i], 0, 1)*usun[i][1]
        	 + gsl_matrix_get(rot[i], 0, 2)*usun[i][2];
    u2[i][1] = gsl_matrix_get(rot[i], 1, 0)*usun[i][0]
        	 + gsl_matrix_get(rot[i], 1, 1)*usun[i][1]
        	 + gsl_matrix_get(rot[i], 1, 2)*usun[i][2];
    u2[i][2] = gsl_matrix_get(rot[i], 2, 0)*usun[i][0]
        	 + gsl_matrix_get(rot[i], 2, 1)*usun[i][1]
        	 + gsl_matrix_get(rot[i], 2, 2)*usun[i][2];

    // az is "az_rel_sun"
    az[i] = atan(u2[i][0]/u2[i][2]);                // az is in radians
  	azraw[i] = sun_az + (180./M_PI)*(az[i] - beta[i]);
  	elraw[i] = (180./M_PI)*atan(u2[i][1]/sqrt(u2[i][0]*u2[i][0]+u2[i][2]*u2[i][2]));
  }

  for (j = 0; j < NUM_PSS; j++) {
	PointingData[point_index].pss_azraw[j] = azraw[j];
    PointingData[point_index].pss_elraw[j] = elraw[j];
  }
  for (i = 0; i < NUM_PSS; i++) {
  	gsl_matrix_free(rot[i]);
  	gsl_matrix_free(rxalpha[i]);
  	gsl_matrix_free(rzpsi[i]);
  }

  new_val = 0;
  for (i = 0; i < NUM_PSS; i++) {
	  new_val += weight[i]*azraw[i]/weightsum;
  }

  if ((!isinf(new_val)) && (!isnan(new_val))) {
    *azraw_pss = new_val;
  } else {
    *azraw_pss = 0.0;
    return 0;
  }

  new_val = 0;
  for (i = 0; i < NUM_PSS; i++) {
	  new_val += weight[i]*elraw[i]/weightsum;
  }

  if ((!isinf(new_val)) && (!isnan(new_val))) {
    *elraw_pss = new_val;
  } else {
    *elraw_pss = 0.0;
    return 0;
  }

  NormalizeAngle(azraw_pss);
  NormalizeAngle(elraw_pss);

  PointingData[point_index].pss_array_azraw = *azraw_pss;
  PointingData[point_index].pss_array_elraw = *elraw_pss;

  return 1;
}

/**
 * Store the gyro data samples in a ring buffer, allowing us to "rewind" our Az/El data to
 * previous point in history.  This is mostly useful for high-latency sensors such as the
 * star cameras.
 */
static void record_gyro_history(int m_index, gyro_history_t *m_gyhist, gyro_reading_t *m_newgy)
{
    /*****************************************/
    /*   Allocate Memory                     */
    if (m_gyhist->ifel_gy_history == NULL) {
        m_gyhist->ifel_gy_history = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));
        m_gyhist->ifel_gy_offset = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));

        m_gyhist->ifroll_gy_history = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));
        m_gyhist->ifroll_gy_offset = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));

        m_gyhist->ifyaw_gy_history = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));;
        m_gyhist->ifyaw_gy_offset = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));

        m_gyhist->elev_history = (double *) calloc(GY_HISTORY_AGE_CS, sizeof(double));
    }

    /*****************************************/
    /* record history                        */
    if (m_gyhist->i_history >= GY_HISTORY_AGE_CS) m_gyhist->i_history = 0;

    m_gyhist->ifel_gy_history[m_gyhist->i_history] = m_newgy->ifel_gy;
    m_gyhist->ifel_gy_offset[m_gyhist->i_history] = m_newgy->ifel_gy_offset;

    m_gyhist->ifroll_gy_history[m_gyhist->i_history] = m_newgy->ifroll_gy;
    m_gyhist->ifroll_gy_offset[m_gyhist->i_history] = m_newgy->ifroll_gy_offset;

    m_gyhist->ifyaw_gy_history[m_gyhist->i_history] = m_newgy->ifyaw_gy;
    m_gyhist->ifyaw_gy_offset[m_gyhist->i_history] = m_newgy->ifyaw_gy_offset;

    m_gyhist->elev_history[m_gyhist->i_history] =  from_degrees(PointingData[m_index].el);
    m_gyhist->i_history++;
}

// TODO(laura): This function doesn't appear to be called by anything in mcp.
// Is this BLASTPol code that Seth overwrote?
//
// int possible_solution(double az, double el, int i_point) {
//   double mag_az, enc_el, d_az;
//
//   // test for insanity
//   if (!finite(az)) return(0);
//   if (!finite(el)) return(0);
//   if (el > 70.0) return (0);
//   if (el < 0.0) return(0);
//
//   mag_az = PointingData[i_point].mag_az;
//
//   if (CommandData.use_mag) {
//     d_az = az - mag_az;
//
//     if (d_az > 180.0) d_az -= 360;
//     if (d_az < -180.0) d_az += 360;
//
//     if (d_az > 30.0) return (0);
//     if (d_az < -30.0) return (0);
//   }
//
//
//   return(1);
// }

static xsc_last_trigger_state_t *XSCHasNewSolution(int which)
{
    xsc_last_trigger_state_t *trig_state = NULL;

    // The latest solution isn't good
    if (!XSC_SERVER_DATA(which).channels.image_eq_valid) {
        return NULL;
    }

    // The camera system has just started
    if (XSC_SERVER_DATA(which).channels.image_ctr_stars < 0 || XSC_SERVER_DATA(which).channels.image_ctr_mcp < 0) {
        return NULL;
    }

    // The solution has already been processed
    if (XSC_SERVER_DATA(which).channels.image_ctr_stars == xsc_pointing_state[which].last_solution_stars_counter) {
        return NULL;
    }

    /* Joy is commenting this out, replacing with previous EBEX logic
    while ((trig_state = xsc_get_trigger_data(which))) {
        if (XSC_SERVER_DATA(which).channels.image_ctr_mcp == trig_state->counter_mcp)
            break;
        blast_dbg("Discarding trigger data with counter_mcp %d", trig_state->counter_mcp);
        free(trig_state);
    } 
    */
    while ((trig_state = xsc_get_trigger_data(which))) {
        if ((XSC_SERVER_DATA(which).channels.image_ctr_mcp == trig_state->counter_mcp)
          & (XSC_SERVER_DATA(which).channels.image_ctr_stars == trig_state->counter_stars)) {
            break;
        }
        blast_dbg("Discarding trigger data with counter_mcp %d", trig_state->counter_mcp);
        blast_dbg("Discarding trigger data with image_ctr_mcp %d", XSC_SERVER_DATA(which).channels.image_ctr_mcp);
        blast_dbg("Discarding trigger data with counter_stars %d", trig_state->counter_stars);
        blast_dbg("Discarding trigger data with image_ctr_stars %d", XSC_SERVER_DATA(which).channels.image_ctr_stars);
        free(trig_state);
    }
    /*
    trig_state = xsc_get_trigger_data(which);
    if (XSC_SERVER_DATA(which).channels.image_ctr_stars != trig_state->counter_stars) {
        free(trig_state);
        return NULL;
    }
    if (XSC_SERVER_DATA(which).channels.image_ctr_mcp != trig_state->counter_mcp) {
        free(trig_state);
        return NULL;
    }
    */

    return trig_state;
}


static void EvolveXSCSolution(struct ElSolutionStruct *e, struct AzSolutionStruct *a, gyro_reading_t *m_rg,
                              gyro_history_t *m_hs, double old_el, int which)
{
    xsc_last_trigger_state_t *trig_state = NULL;
    double gy_az;
    double new_az, new_el, ra, dec;

    double el_frame = from_degrees(old_el);

    // evolve el
    e->angle += (m_rg->ifel_gy + m_rg->ifel_gy_offset) / SR;
    e->variance += GYRO_VAR;

    // evolve az
    gy_az = (m_rg->ifroll_gy + m_rg->ifroll_gy_offset) * sin(el_frame)
            + (m_rg->ifyaw_gy + m_rg->ifyaw_gy_offset) * cos(el_frame);
    a->angle += gy_az / SR;
    a->variance += (GYRO_VAR);
    a->int_ifroll += m_rg->ifroll_gy / SR;
    a->int_ifyaw += m_rg->ifyaw_gy / SR;
    e->int_ifel += m_rg->ifel_gy / SR;
    a->since_last++;

    if ((trig_state = XSCHasNewSolution(which))) {
        double w1, w2;
        int delta_100hz = get_100hz_framenum() - trig_state->trigger_time;

        // When we get a new frame, use these to correct for history
        double gy_el_delta = 0;
        double gy_az_delta = 0;

        xsc_pointing_state[which].last_solution_stars_counter = XSC_SERVER_DATA(which).channels.image_ctr_stars;
        blast_info(" xsc%i: received new solution", which);
        if (delta_100hz < GY_HISTORY_AGE_CS) {
            blast_info(" xsc%i: new solution young enough to accept", which);
            ra = to_hours(XSC_SERVER_DATA(which).channels.image_eq_ra);
            dec = to_degrees(XSC_SERVER_DATA(which).channels.image_eq_dec);

            equatorial_to_horizontal(ra, dec, trig_state->lst,
                                     trig_state->lat, &new_az, &new_el);

            xsc_pointing_state[which].az = new_az;
            xsc_pointing_state[which].el = new_el;

            blast_dbg("Solution from XSC%i: Ra:%f, Dec:%f", which, to_degrees(from_hours(ra)), dec);
            blast_dbg("Solution from XSC%i: az:%f, el:%f", which, new_az, new_el);


            /* Add BDA offset -- there's a pole here at EL = 90 degrees! */

            new_az += to_degrees(approximate_az_from_cross_el(CommandData.XSC[which].cross_el_trim,
                                                              from_degrees(old_el)));
            new_el += to_degrees(CommandData.XSC[which].el_trim);

            e->new_offset_ifel_gy = ((new_el - e->prev_sol_el) - e->int_ifel) /
            ((1.0/SR) * (double)a->since_last);
            a->d_az = remainder(new_az - a->prev_sol_az, 360.0);
            a->new_offset_ifroll_gy = -(a->d_az * cos((new_el + e->prev_sol_el)/180.0/2.0*M_PI) + a->int_ifroll) /
            ((1.0/SR) * (double)a->since_last);
            a->new_offset_ifyaw_gy = -(a->d_az * sin((new_el + e->prev_sol_el)/180.0/2.0*M_PI) + a->int_ifyaw) /
            ((1.0/SR) * (double)a->since_last);
            blast_info("new_offset_ifel_gy = %f, new_el = %f, prev_el = %f, int_if_el = %f, since_last = %f",
                       e->new_offset_ifel_gy, new_el, e->prev_sol_el, e->int_ifel, (double)a->since_last);
            blast_info("new_offset_ifroll_gy = %f, new_offset_ifyaw_gy = %f, d_az = %f",
                       a->new_offset_ifroll_gy , a->new_offset_ifyaw_gy, a->d_az);
            blast_info("int_if_yaw = %f, int_if_roll = %f, new_az = % f, prev_az = %f",
                       a->int_ifyaw , a->int_ifroll, new_az, a->prev_sol_az);

            // Now that we have calculated the integrated gyros, reset the gyro integrations and prev solutions.
            blast_info("Resetting, prev_sol, int_ifroll");
            a->prev_sol_az = new_az;
            a->int_ifroll = 0.0;
            a->int_ifyaw = 0.0;
            e->prev_sol_el = new_el;
            e->int_ifel = 0.0;
            a->since_last = 0;

            // This solution is xsc_pointing_data.age_last_stars_solution old: how much have we moved?
            gy_el_delta = 0;
            gy_az_delta = 0;
            for (int i = 0; i < delta_100hz; i++) {
                int j = m_hs->i_history - i;
                if (j < 0) j += GY_HISTORY_AGE_CS;

                gy_el_delta += (m_hs->ifel_gy_history[j] + m_hs->ifel_gy_offset[j]) / SR;
                gy_az_delta += ((m_hs->ifyaw_gy_history[j]  + m_hs->ifyaw_gy_offset[j])
                                    * sin(m_hs->elev_history[j])
                              + (m_hs->ifroll_gy_history[j] + m_hs->ifroll_gy_offset[j])
                                      * cos(m_hs->elev_history[j])) / SR;
            }

            // Evolve el solution
            e->angle -= gy_el_delta;// rewind to when the frame was grabbed
            a->angle -= gy_az_delta;// rewind to when the frame was grabbed

            blast_dbg(" Az averaging old: %f,  and new: %f\n", a->angle, new_az);

            w1 = 1.0 / (e->variance);
            if (XSC_SERVER_DATA(which).channels.image_eq_sigma_pointing > M_PI) {
                w2 = 0.0;
            } else {
                w2 = 10.0 * XSC_SERVER_DATA(which).channels.image_eq_sigma_pointing * (180.0 / M_PI);
                if (w2 > 0.0)
                w2 = 1.0 / (w2 * w2);
                else
                w2 = 0.0;// shouldn't happen
            }

            UnwindDiff(e->angle, &new_el);
            UnwindDiff(a->angle, &new_az);

            e->angle = (w1 * e->angle + new_el * w2) / (w1 + w2);

            blast_dbg("Rewound old SC AZ EL is %f %f\n", a->angle, e->angle);

            e->variance = 1.0 / (w1 + w2);
            e->angle += gy_el_delta;// add back to now
            e->angle = normalize_angle_360(e->angle);

            // evolve az solution
            w1 = 1.0 / (a->variance);
            // w2 already set

            a->angle = (w1 * a->angle + new_az * w2) / (w1 + w2);
            blast_dbg("Rewinded averaged SC AZ is %f\n", a->angle);
            a->variance = 1.0 / (w1 + w2);
            a->angle += gy_az_delta;// add back to now
            a->angle = normalize_angle_360(a->angle);

            blast_dbg(" Az result is: %f\n", a->angle);
            blast_dbg("Evolved SC AZ EL is %f %f\n", a->angle, e->angle);
        }
        free(trig_state);
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
  static int i = 0;
  double w1, w2;
  double new_offset = 0;

  s->angle += (gyro + gy_off) / SR;
  s->variance += GYRO_VAR;

  s->gy_int += gyro / SR; // in degrees
  s->int_ifel = s->gy_int;

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
        s->new_offset_ifel_gy = new_offset;
        s->offset_gy = fir_filter(new_offset, s->fs);
      }
      s->since_last = 0;
      if (s->n_solutions < 10000) {
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
  s->variance += (2 * GYRO_VAR);

  s->ifroll_gy_int += ifroll_gy / SR; // in degrees
  s->ifyaw_gy_int += ifyaw_gy / SR; // in degrees
  s->int_ifroll = s->ifroll_gy_int;
  s->int_ifyaw = s->ifyaw_gy_int;

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
	s->d_az = daz;

	/* Do Gyro_IFroll */
	new_offset = -(daz * cos(el) + s->ifroll_gy_int) /
	  ((1.0/SR) * (double)s->since_last);
	s->new_offset_ifroll_gy = new_offset;
	s->offset_ifroll_gy = fir_filter(new_offset, s->fs2);;

	/* Do Gyro_IFyaw */
	new_offset = -(daz * sin(el) + s->ifyaw_gy_int) /
	  ((1.0/SR) * (double)s->since_last);
	s->new_offset_ifyaw_gy = new_offset;
	s->offset_ifyaw_gy = fir_filter(new_offset, s->fs3);;
      }
      s->since_last = 0;
      if (s->n_solutions < 10000) {
        s->n_solutions++;
      }
    }
    s->ifroll_gy_int = 0.0;
    s->ifyaw_gy_int = 0.0;
    s->last_input = new_angle;
  }
  s->since_last++;
}

static void xsc_calculate_full_pointing_estimated_location(int which)
{
    int pointing_read_index = GETREADINDEX(point_index);
    double az = from_degrees(PointingData[pointing_read_index].az);
    double el = from_degrees(PointingData[pointing_read_index].el);
    double xsc_az = az - approximate_az_from_cross_el(CommandData.XSC[which].cross_el_trim, el);
    double xsc_el = el - CommandData.XSC[which].el_trim;
    double xsc_ra_hours = 0.0;
    double xsc_dec_deg = 0.0;
    horizontal_to_equatorial(to_degrees(xsc_az), to_degrees(xsc_el),
                             PointingData[pointing_read_index].lst,
                             PointingData[pointing_read_index].lat, &xsc_ra_hours, &xsc_dec_deg);

    PointingData[point_index].estimated_xsc_az_deg[which] = to_degrees(xsc_az);
    PointingData[point_index].estimated_xsc_el_deg[which] = to_degrees(xsc_el);
    PointingData[point_index].estimated_xsc_ra_hours[which] = xsc_ra_hours;
    PointingData[point_index].estimated_xsc_dec_deg[which] = xsc_dec_deg;
}

static void AutoTrimToSC()
{
    int i_point = GETREADINDEX(point_index);
    int isc_good = 0, osc_good = 0;
    static int which = 0;
    time_t t = mcp_systime(NULL);

    if (PointingData[i_point].xsc_sigma[0] > CommandData.autotrim_thresh) {
        CommandData.autotrim_xsc0_last_bad = t;
    }
    if (PointingData[i_point].xsc_sigma[1] > CommandData.autotrim_thresh) {
        CommandData.autotrim_xsc1_last_bad = t;
    }

    if (t - CommandData.autotrim_xsc0_last_bad > CommandData.autotrim_time)
        isc_good = 1;
    if (t - CommandData.autotrim_xsc1_last_bad > CommandData.autotrim_time)
        osc_good = 1;

    // sticky choice
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

static inline double exponential_moving_average(double m_running_avg, double m_newval, double m_halflife)
{
    double alpha = 2.0 / (1.0 + 2.8854 * m_halflife);
    return alpha * m_newval + (1.0 - alpha) * m_running_avg;
}

// Called if we are not in charge.  Reads important fields (shared from the ICC) that
// cannot be read out when not the ICC.
void ReadICCPointing(read_icc_t *m_read_icc)
{
    int i;
    static int first_time = 1;
    if (first_time) {
        blast_info("Not in charge");
        for (i = 0; m_read_icc[i].pval; i++) {
            m_read_icc[i].ch = channels_find_by_name(m_read_icc[i].ch_name);
            blast_info("writing channel %s", m_read_icc[i].ch_name);
        }
        blast_info("Getting pointing sensor data from %d shared linklist channels.", i);
        first_time = 0;
    }
    for (i = 0; m_read_icc[i].pval; i++) {
        if (!m_read_icc[i].ch) continue;

        switch (m_read_icc[i].var_type) {
            case TYPE_INT8:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(int8_t*)m_read_icc[i].pval);
            break;
        case TYPE_UINT8:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(uint8_t*)(m_read_icc[i].pval));
            break;
        case TYPE_INT16:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(int16_t*)(m_read_icc[i].pval));
            break;
        case TYPE_UINT16:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(uint16_t*)(m_read_icc[i].pval));
            break;
        case TYPE_INT32:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(int32_t*)(m_read_icc[i].pval));
            break;
        case TYPE_UINT32:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(uint32_t*)(m_read_icc[i].pval));
            break;
        case TYPE_INT64:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(int64_t*)(m_read_icc[i].pval));
            break;
        case TYPE_UINT64:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(uint64_t*)(m_read_icc[i].pval));
           break;
        case TYPE_FLOAT:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(float*)(m_read_icc[i].pval));
            break;
        case TYPE_DOUBLE:
            GET_SCALED_VALUE(m_read_icc[i].ch, *(double*)(m_read_icc[i].pval));
            break;
        default:
            blast_err("Invalid type %d", m_read_icc[i].var_type);
        }
//        SET_SCALED_VALUE(m_read_icc[i].ch, &(()m_read_icc[i].pval));
    }
}
// TODO(seth): Split up Pointing() in manageable chunks for each sensor
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
    static int j = 0;

    int mag_ok_n;
    int mag_ok_s;
    int pss_ok;
    int dgps_ok;
    static unsigned pss_since_ok = 500;
    double mag_az_n;
    double mag_az_s;
    double mag_el_n;
    double mag_el_s;
    double pss_az = 0;
    double pss_el = 0;
    double dgps_az = 0;
    double clin_elev;
    static double last_good_lat = 0, last_good_lon = 0, last_good_alt = 0;
    static double last_gy_total_vel = 0.0;
    static int i_at_float = 0;
    double trim_change;

    static int enc_motor_ready;
    static int enc_motor_ok;
    enc_motor_ready = is_el_motor_ready();
    static int firsttime = 1;

    int i_point_read;

    static struct LutType elClinLut = { "/data/etc/blast/clino_tng.lut", 0, NULL, NULL, 0 };

    struct ElAttStruct ElAtt = { 0.0, 0.0, 0.0 };
    struct AzAttStruct AzAtt = { 0.0, 0.0, 0.0, 0.0 };

    static struct ElSolutionStruct EncMotEl = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(60),
        .sys_var = M2DV(20), // systematic variance
        .offset_gy = OFFSET_GY_IFEL, // gy offset
        .FC = 0.0001, // filter constant
    };
    static struct ElSolutionStruct ClinEl = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(60),
        .sys_var = M2DV(20), // systematic variance
        .offset_gy = OFFSET_GY_IFEL, // gy offset
        .FC = 0.0001, // filter constant
    };
    static struct ElSolutionStruct ISCEl = {
        .variance = 719.9 * 719.9, // starting variance
        .samp_weight = 1.0 / M2DV(0.2),
        .sys_var = M2DV(0.2), // systematic variance
        .offset_gy = OFFSET_GY_IFEL, // gy offset
        .FC = 0.0001, // filter constant
        .prev_sol_el = 0.0,
        .int_ifel = 0.0,
    };
    static struct ElSolutionStruct OSCEl = {
        .variance = 719.9 * 719.9, // starting variance
        .samp_weight = 1.0 / M2DV(0.2),
        .sys_var = M2DV(0.2), // systematic variance
        .offset_gy = OFFSET_GY_IFEL, // gy offset
        .FC = 0.0001, // filter constant
        .prev_sol_el = 0.0,
        .int_ifel = 0.0,
    };
    static struct ElSolutionStruct MagElN = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(120),
        .sys_var = M2DV(90), // systematic variance
        .offset_gy = OFFSET_GY_IFEL, // gy offset
        .FC = 0.0001, // filter constant
    };
    static struct ElSolutionStruct MagElS = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(120),
        .sys_var = M2DV(90), // systematic variance
        .offset_gy = OFFSET_GY_IFEL, // gy offset
        .FC = 0.0001, // filter constant
    };
    static struct AzSolutionStruct NullAz = {
        .angle = 91.0,
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(6),
        .sys_var = M2DV(6000), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
    };
    static struct AzSolutionStruct MagAzN = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(120),
        .sys_var = M2DV(90), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
    };
    static struct AzSolutionStruct MagAzS = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(120),
        .sys_var = M2DV(90), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
    };

    static struct AzSolutionStruct PSSAz =  {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(30),
        .sys_var = M2DV(60), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
    };
    static struct AzSolutionStruct DGPSAz =  {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(120),
        .sys_var = M2DV(90), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
    };
    // TODO(seth): Replace ISC/OSC Az Solutions with XSC
    static struct AzSolutionStruct ISCAz = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(0.3),
        .sys_var = M2DV(0.2), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
        .prev_sol_az = 0.0,
        .int_ifroll = 0.0,
        .int_ifyaw = 0.0,
        .since_last = 0,
    };
    static struct AzSolutionStruct OSCAz = {
        .variance = 360.0 * 360.0,
        .samp_weight = 1.0 / M2DV(0.3),
        .sys_var = M2DV(0.2), // systematic variance
        .offset_ifroll_gy = OFFSET_GY_IFROLL,
        .offset_ifyaw_gy = OFFSET_GY_IFYAW,
        .FC = 0.0001, // filter constant
        .prev_sol_az = 0.0,
        .int_ifroll = 0.0,
        .int_ifyaw = 0.0,
        .since_last = 0,
    };

		static read_icc_t read_shared_pdata[] = {
        {(void *) &(ISCAz.angle), "x0_point_az", TYPE_DOUBLE},
        {(void *) &(OSCAz.angle), "x1_point_az", TYPE_DOUBLE},
        {(void *) &(ISCEl.angle), "x0_point_el", TYPE_DOUBLE},
        {(void *) &(OSCEl.angle), "x1_point_el", TYPE_DOUBLE},
        {(void *) &(ISCEl.variance), "x0_point_var", TYPE_DOUBLE},
        {(void *) &(OSCEl.variance), "x1_point_var", TYPE_DOUBLE},
        {(void *) &(ACSData.enc_motor_elev), "mc_el_motor_pos", TYPE_DOUBLE},
        {(void *) &(enc_motor_ready), "ok_motor_enc", TYPE_UINT8},
        {(void *) &(NewAzEl.fresh), "rate_atrim", TYPE_INT32},
        {(void *) &(NewAzEl.rate), "fresh_trim", TYPE_DOUBLE},
        {(void *) &(NewAzEl.az), "new_az", TYPE_DOUBLE},
        {(void *) &(NewAzEl.el), "new_el", TYPE_DOUBLE},

        // terminator
        {0}
		};


		static gyro_history_t hs = {NULL};
		static gyro_reading_t RG = {0.0};

    if (firsttime) {
        firsttime = 0;
        ClinEl.trim = CommandData.clin_el_trim;
        EncMotEl.trim = CommandData.enc_motor_el_trim;
        NullAz.trim = CommandData.null_az_trim;
        MagAzN.trim = CommandData.mag_az_trim[0];
        MagAzS.trim = CommandData.mag_az_trim[1];
        PSSAz.trim = CommandData.pss_az_trim;
        DGPSAz.trim = CommandData.dgps_az_trim;

        ClinEl.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(ClinEl.fs, FIR_LENGTH, 0, 0);
        EncMotEl.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(EncMotEl.fs, FIR_LENGTH, 0, 0);
        MagElN.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(MagElN.fs, FIR_LENGTH, 0, 0);
        MagElS.fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(MagElS.fs, FIR_LENGTH, 0, 0);

        NullAz.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        NullAz.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(NullAz.fs2, (int) (10), 0, 0); // not used
        init_fir(NullAz.fs3, (int) (10), 0, 0); // not used

        MagAzN.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        MagAzN.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        MagAzS.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        MagAzS.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(MagAzN.fs2, FIR_LENGTH, 0, 0);
        init_fir(MagAzN.fs3, FIR_LENGTH, 0, 0);
        init_fir(MagAzS.fs2, FIR_LENGTH, 0, 0);
        init_fir(MagAzS.fs3, FIR_LENGTH, 0, 0);

        PSSAz.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        PSSAz.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(PSSAz.fs2, FIR_LENGTH, 0, 0);
        init_fir(PSSAz.fs3, FIR_LENGTH, 0, 0);

        DGPSAz.fs2 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        DGPSAz.fs3 = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct));
        init_fir(DGPSAz.fs2, FIR_LENGTH, 0, 0);
        init_fir(DGPSAz.fs3, FIR_LENGTH, 0, 0);

        // the first t about to be read needs to be set
        PointingData[GETREADINDEX(point_index)].t = mcp_systime(NULL); // CPU time

        /* Load lat/lon from disk */
        last_good_lon = PointingData[0].lon = PointingData[1].lon = PointingData[2].lon = CommandData.lon;
        last_good_lat = PointingData[0].lat = PointingData[1].lat = PointingData[2].lat = CommandData.lat;

        // Initialize the slew veto counter to the stored value of slew_veto.
        CommandData.pointing_mode.nw = CommandData.slew_veto;
    }

    if (elClinLut.n == 0)
        LutInit(&elClinLut);

    i_point_read = GETREADINDEX(point_index);

//  Only add the encoder solution if we are getting data from the El drive.  Or if we
//  are not InCharge and getting shared El data from the other FCC.
    enc_motor_ok = enc_motor_ready;

    // If we are not in charge then we need to read some pointing data from the ICC
//     if (j % 500 == 0) {
//         blast_info("Before sharing: ISCAz.angle %f, ISCEl.angle %f, ISCEl.variance %f"
//                    " ISCEl.sigma %f, XelTrim %f, ElTrim %f",
//                    ISCAz.angle, ISCEl.angle, ISCEl.variance, sqrt(ISCEl.variance + ISCEl.sys_var),
//                    CommandData.XSC[0].cross_el_trim, CommandData.XSC[0].cross_el_trim);
//         blast_info("OSCAz.angle %f, OSCEl.angle %f, OSCEl.variance %f, OSCEl.sigma %f,"
//                    "XelTrim %f, ElTrim %f",
//                    OSCAz.angle, OSCEl.angle, OSCEl.variance, sqrt(OSCEl.variance + OSCEl.sys_var),
//                    CommandData.XSC[1].cross_el_trim, CommandData.XSC[1].cross_el_trim);
//     }
    if ((!InCharge) && PointingData[i_point_read].recv_shared_data) {
         ReadICCPointing(read_shared_pdata);
    }

    // Make aristotle correct
    R = 15.0 / 3600.0;
    sincos(from_degrees(PointingData[i_point_read].el), &sin_e, &cos_e);
    sincos(from_degrees(PointingData[i_point_read].lat), &sin_l, &cos_l);
    sincos(from_degrees(PointingData[i_point_read].az), &sin_a, &cos_a);

    PointingData[point_index].ifel_earth_gy = R * (-cos_l * sin_a);
    PointingData[point_index].ifroll_earth_gy = R * (cos_e * sin_l - cos_l * sin_e * cos_a);
    PointingData[point_index].ifyaw_earth_gy = R * (sin_e * sin_l + cos_l * cos_e * cos_a);
    RG.ifel_gy = ACSData.ifel_gy - PointingData[point_index].ifel_earth_gy;
    RG.ifel_gy_offset = PointingData[i_point_read].offset_ifel_gy;
    RG.ifroll_gy = ACSData.ifroll_gy - PointingData[point_index].ifroll_earth_gy;
    RG.ifroll_gy_offset = PointingData[i_point_read].offset_ifroll_gy;
    RG.ifyaw_gy = ACSData.ifyaw_gy - PointingData[point_index].ifyaw_earth_gy;
    RG.ifyaw_gy_offset = PointingData[i_point_read].offset_ifyaw_gy;

    PointingData[point_index].gy_az = RG.ifyaw_gy * cos_e + RG.ifroll_gy * sin_e;
    PointingData[point_index].gy_el = RG.ifel_gy;
    PointingData[point_index].gy_total_vel = sqrt(pow((RG.ifel_gy), 2) +
                                                  pow(PointingData[point_index].gy_az*cos_e, 2));
    double current_gy_total_accel = (PointingData[point_index].gy_total_vel - last_gy_total_vel)*SR;
    last_gy_total_vel = PointingData[point_index].gy_total_vel;
    PointingData[point_index].gy_total_accel = exponential_moving_average(PointingData[i_point_read].gy_total_accel,
                                                                          current_gy_total_accel, 15.0);

    /*************************************/
    /** Record history for gyro offsets **/
    record_gyro_history(point_index, &hs, &RG);

    PointingData[point_index].t = mcp_systime(NULL); // CPU time

    /************************************************/
    /** Set the official Lat and Lon **/
    if (GPSData.isnew) {
        last_good_lat = GPSData.latitude;
        last_good_lon = GPSData.longitude;
        GPSData.isnew = 0;
    } else {
        last_good_lat = SIPData.GPSpos.lat;
        last_good_lon = SIPData.GPSpos.lon;
    }
    last_good_alt = SIPData.GPSpos.alt;

    PointingData[point_index].lat = last_good_lat;
    PointingData[point_index].lon = last_good_lon;
    PointingData[point_index].alt = last_good_alt;

    /* At float check */
    if (PointingData[point_index].alt < FLOAT_ALT) {
        PointingData[point_index].at_float = 0;
        i_at_float = 0;
    } else {
        i_at_float++;
        if (i_at_float > FRAMES_TO_OK_ATFLOAT) {
            PointingData[point_index].at_float = 1;
        } else {
            PointingData[point_index].at_float = 0;
        }
    }

    /* Save lat/lon */
    CommandData.lat = PointingData[point_index].lat;
    CommandData.lon = PointingData[point_index].lon;

    /*****************************/
    /** set time related things **/
    PointingData[point_index].t = mcp_systime(NULL); // for now use CPU time

    /** Set LST and local sidereal date **/
    PointingData[point_index].lst = to_seconds(time_lst_unix(PointingData[point_index].t,
                                                             from_degrees(PointingData[point_index].lon)));


    /**
     * Get the Magnetometer Data
     */
    mag_ok_n = MagConvert(&(mag_az_n), &(mag_el_n), 0);
    mag_ok_s = MagConvert(&(mag_az_s), &(mag_el_s), 1);

    PointingData[point_index].mag_az_raw[0] = mag_az_n;
    PointingData[point_index].mag_el_raw[0] = mag_el_n;
    PointingData[point_index].mag_az_raw[1] = mag_az_s;
    PointingData[point_index].mag_el_raw[1] = mag_el_s;
    /**
     * Get the DGPS data from CSBF GPS
     */
    PointingData[point_index].dgps_az_raw = CSBFGPSAz.az;
    /*************************************/
    /**      do ISC Solution            **/
    EvolveXSCSolution(&ISCEl, &ISCAz, &RG, &hs, PointingData[i_point_read].el, 0);

    /*************************************/
    /**      do OSC Solution            **/
    EvolveXSCSolution(&OSCEl, &OSCAz, &RG, &hs, PointingData[i_point_read].el, 1);

    /*************************************/
    /**      do elevation solution      **/
    clin_elev = LutCal(&elClinLut, ACSData.clin_elev);
    PointingData[i_point_read].clin_el_lut = clin_elev;

    // TODO(seth): Only set "new solution" when we really have new data
    EvolveElSolution(&ClinEl, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            clin_elev, 1);
    EvolveElSolution(&EncMotEl, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            ACSData.enc_motor_elev, enc_motor_ok);
    EvolveElSolution(&MagElN, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            mag_el_n, mag_ok_n);
    EvolveElSolution(&MagElS, RG.ifel_gy,
            PointingData[i_point_read].offset_ifel_gy,
            mag_el_s, mag_ok_s);
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

    pss_ok = PSSConvert(&pss_az, &pss_el);
    if (pss_ok) {
        pss_since_ok = 0;
    } else {
        pss_since_ok++;
    }
    dgps_ok = CSBFGPSAz.att_ok;
    PointingData[point_index].pss_ok = pss_ok;
    PointingData[point_index].dgps_ok = dgps_ok;

    /** evolve solutions **/
    EvolveAzSolution(&NullAz,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        0.0, 0);
    /** MAG Az from North **/
    EvolveAzSolution(&MagAzN,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        mag_az_n, mag_ok_n);
    /** MAG Az from South **/
    EvolveAzSolution(&MagAzS,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        mag_az_s, mag_ok_s);

    /** PSS **/
    EvolveAzSolution(&PSSAz,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        pss_az, pss_ok);

    /** PSS **/
    EvolveAzSolution(&DGPSAz,
        RG.ifroll_gy, PointingData[i_point_read].offset_ifroll_gy,
        RG.ifyaw_gy,  PointingData[i_point_read].offset_ifyaw_gy,
        PointingData[point_index].el,
        dgps_az, dgps_ok);

    if (CommandData.fast_offset_gy > 0) {
        CommandData.fast_offset_gy--;
    }

    AddAzSolution(&AzAtt, &NullAz, 1);
    /** add az solutions **/
    if (CommandData.use_mag1) {
        AddAzSolution(&AzAtt, &MagAzN, 1);
    }
    if (CommandData.use_mag2) {
        AddAzSolution(&AzAtt, &MagAzS, 1);
    }
    if (CommandData.use_pss) {
        AddAzSolution(&AzAtt, &PSSAz, 1);
    }
    if (CommandData.use_dgps) {
        AddAzSolution(&AzAtt, &DGPSAz, 1);
    }
    if (CommandData.use_xsc0) {
        AddAzSolution(&AzAtt, &ISCAz, 0);
    }
    if (CommandData.use_xsc1) {
        AddAzSolution(&AzAtt, &OSCAz, 0);
    }

    PointingData[point_index].offset_ifrollmag_gy[0] = MagAzN.offset_ifroll_gy;
    PointingData[point_index].offset_ifyawmag_gy[0] = MagAzN.offset_ifyaw_gy;
    PointingData[point_index].offset_ifrollmag_gy[1] = MagAzS.offset_ifroll_gy;
    PointingData[point_index].offset_ifyawmag_gy[1] = MagAzS.offset_ifyaw_gy;
    PointingData[point_index].offset_ifrolldgps_gy = DGPSAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyawdgps_gy = DGPSAz.offset_ifyaw_gy;

    PointingData[point_index].offset_ifrollpss_gy = PSSAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyawpss_gy = PSSAz.offset_ifyaw_gy;
    PointingData[point_index].offset_ifelmotenc_gy = EncMotEl.offset_gy;
    PointingData[point_index].offset_ifelclin_gy = ClinEl.offset_gy;
    PointingData[point_index].az = AzAtt.az;
    PointingData[point_index].weight_az = AzAtt.weight;
    if (CommandData.az_autogyro) {
        PointingData[point_index].offset_ifroll_gy = AzAtt.offset_ifroll_gy;
        PointingData[point_index].offset_ifyaw_gy = AzAtt.offset_ifyaw_gy;
    } else {
        PointingData[point_index].offset_ifroll_gy = CommandData.offset_ifroll_gy;
        PointingData[point_index].offset_ifyaw_gy = CommandData.offset_ifyaw_gy;
    }

    /** calculate ra/dec for convenience on the ground **/
    horizontal_to_equatorial(PointingData[point_index].az, PointingData[point_index].el,
                             PointingData[point_index].lst, PointingData[point_index].lat, &ra, &dec);
    equatorial_to_horizontal(ra, dec, PointingData[point_index].lst, PointingData[point_index].lat, &az, &el);

    PointingData[point_index].ra = ra;
    PointingData[point_index].dec = dec;
    /** record solutions in pointing data **/
    //  if (j%500==0) blast_info("Pointing: PointingData.enc_el = %f", PointingData[point_index].enc_el);
    PointingData[point_index].enc_motor_ok = enc_motor_ok;
    PointingData[point_index].enc_motor_el = EncMotEl.angle;
    PointingData[point_index].enc_motor_sigma = sqrt(EncMotEl.variance + EncMotEl.sys_var);
    PointingData[point_index].clin_el = ClinEl.angle;
    PointingData[point_index].clin_sigma = sqrt(ClinEl.variance + ClinEl.sys_var);

    PointingData[point_index].mag_ok[0] = mag_ok_n;
    PointingData[point_index].mag_az[0] = MagAzN.angle;
    PointingData[point_index].mag_el[0] = MagElN.angle;
    PointingData[point_index].mag_sigma[0] = sqrt(MagAzN.variance + MagAzN.sys_var);
    PointingData[point_index].mag_ok[1] = mag_ok_s;
    PointingData[point_index].mag_az[1] = MagAzS.angle;
    PointingData[point_index].mag_el[1] = MagElS.angle;
    PointingData[point_index].mag_sigma[1] = sqrt(MagAzS.variance + MagAzS.sys_var);
    PointingData[point_index].dgps_az = DGPSAz.angle;
    PointingData[point_index].dgps_sigma = sqrt(DGPSAz.variance + DGPSAz.sys_var);

    PointingData[point_index].null_az = NullAz.angle;

    // Added 22 June 2010 GT
    PointingData[point_index].pss_ok = pss_ok;
    PointingData[point_index].pss_az = PSSAz.angle;
    PointingData[point_index].pss_sigma = sqrt(PSSAz.variance + PSSAz.sys_var);

    PointingData[point_index].xsc_az[0] = ISCAz.angle;
    PointingData[point_index].xsc_el[0] = ISCEl.angle;
    PointingData[point_index].xsc_var[0] = ISCEl.variance;
    PointingData[point_index].xsc_sigma[0] = sqrt(ISCEl.variance + ISCEl.sys_var);
    PointingData[point_index].offset_ifel_gy_xsc[0] = ISCEl.offset_gy;
    PointingData[point_index].offset_ifroll_gy_xsc[0] = ISCAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyaw_gy_xsc[0] = ISCAz.offset_ifyaw_gy;

    PointingData[point_index].xsc_az[1] = OSCAz.angle;
    PointingData[point_index].xsc_el[1] = OSCEl.angle;
    PointingData[point_index].xsc_var[1] = OSCEl.variance;
    PointingData[point_index].xsc_sigma[1] = sqrt(OSCEl.variance + OSCEl.sys_var);
    PointingData[point_index].offset_ifel_gy_xsc[1] = OSCEl.offset_gy;
    PointingData[point_index].offset_ifroll_gy_xsc[1] = OSCAz.offset_ifroll_gy;
    PointingData[point_index].offset_ifyaw_gy_xsc[1] = OSCAz.offset_ifyaw_gy;

    PointingData[point_index].new_offset_ifel_elmotenc_gy = EncMotEl.new_offset_ifel_gy;
    PointingData[point_index].int_ifel_elmotenc = EncMotEl.int_ifel;
    PointingData[point_index].new_offset_ifyaw_mag1_gy = MagAzN.new_offset_ifyaw_gy;
    PointingData[point_index].new_offset_ifroll_mag1_gy = MagAzN.new_offset_ifroll_gy;
    PointingData[point_index].d_az_mag1 = MagAzN.d_az;
    PointingData[point_index].int_ifroll_mag1 = MagAzN.int_ifroll;
    PointingData[point_index].int_ifyaw_mag1 = MagAzN.int_ifyaw;
    PointingData[point_index].new_offset_ifyaw_mag2_gy = MagAzS.new_offset_ifyaw_gy;
    PointingData[point_index].new_offset_ifroll_mag2_gy = MagAzS.new_offset_ifroll_gy;
    PointingData[point_index].d_az_mag2 = MagAzS.d_az;
    PointingData[point_index].int_ifroll_mag2 = MagAzS.int_ifroll;
    PointingData[point_index].int_ifyaw_mag2 = MagAzS.int_ifyaw;
    PointingData[point_index].new_offset_ifel_xsc0_gy = ISCEl.new_offset_ifel_gy;
    PointingData[point_index].new_offset_ifroll_xsc0_gy = ISCAz.new_offset_ifroll_gy;
    PointingData[point_index].new_offset_ifyaw_xsc0_gy = ISCAz.new_offset_ifyaw_gy;
    PointingData[point_index].int_ifel_xsc0 = ISCEl.int_ifel;
    PointingData[point_index].int_ifroll_xsc0 = ISCAz.int_ifroll;
    PointingData[point_index].int_ifyaw_xsc0 = ISCAz.int_ifyaw;
    PointingData[point_index].d_az_xsc0 = ISCAz.d_az;
    PointingData[point_index].prev_sol_az_xsc0 = ISCAz.prev_sol_az;
    PointingData[point_index].prev_sol_el_xsc0 = ISCEl.prev_sol_el;
    PointingData[point_index].new_offset_ifel_xsc1_gy = OSCEl.new_offset_ifel_gy;
    PointingData[point_index].new_offset_ifroll_xsc1_gy = OSCAz.new_offset_ifroll_gy;
    PointingData[point_index].new_offset_ifyaw_xsc1_gy = OSCAz.new_offset_ifyaw_gy;
    PointingData[point_index].int_ifel_xsc1 = OSCEl.int_ifel;
    PointingData[point_index].int_ifroll_xsc1 = OSCAz.int_ifroll;
    PointingData[point_index].int_ifyaw_xsc1 = OSCAz.int_ifyaw;
    PointingData[point_index].d_az_xsc1 = OSCAz.d_az;
    PointingData[point_index].prev_sol_az_xsc1 = OSCAz.prev_sol_az;
    PointingData[point_index].prev_sol_el_xsc1 = OSCEl.prev_sol_el;
    PointingData[point_index].autotrim_rate_xsc = NewAzEl.rate;
    PointingData[point_index].fresh = NewAzEl.fresh;
    PointingData[point_index].new_az = NewAzEl.az;
    PointingData[point_index].new_el = NewAzEl.el;

    xsc_calculate_full_pointing_estimated_location(0);
    xsc_calculate_full_pointing_estimated_location(1);
    /********************/
    /* Set Manual Trims */
    if (CommandData.autotrim_enable)
        AutoTrimToSC();

    if (NewAzEl.fresh == -1) {
        ClinEl.trim = 0.0;
        EncMotEl.trim = 0.0;
        NullAz.trim = 0.0;
        MagAzN.trim = 0.0;
        MagAzS.trim = 0.0;
        PSSAz.trim = 0.0;
        DGPSAz.trim = 0.0;
        NewAzEl.fresh = 0;
    }

    if ((NewAzEl.fresh == 1) && InCharge) {
        trim_change = (NewAzEl.el - ClinEl.angle) - ClinEl.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        ClinEl.trim += trim_change;

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

        trim_change = (NewAzEl.az - MagAzN.angle) - MagAzN.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        MagAzN.trim += trim_change;

        trim_change = (NewAzEl.az - MagAzS.angle) - MagAzS.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        MagAzS.trim += trim_change;

        if (pss_since_ok < 500) {
            trim_change = (NewAzEl.az - PSSAz.angle) - PSSAz.trim;
            if (trim_change > NewAzEl.rate)
                trim_change = NewAzEl.rate;
            else if (trim_change < -NewAzEl.rate)
                trim_change = -NewAzEl.rate;
            PSSAz.trim += trim_change;
        }

        trim_change = (NewAzEl.az - DGPSAz.angle) - DGPSAz.trim;
        if (trim_change > NewAzEl.rate)
            trim_change = NewAzEl.rate;
        else if (trim_change < -NewAzEl.rate)
            trim_change = -NewAzEl.rate;
        DGPSAz.trim += trim_change;

        NewAzEl.fresh = 0;
    } else {
        ClinEl.trim = CommandData.clin_el_trim;
        EncMotEl.trim = CommandData.enc_motor_el_trim;
        NullAz.trim = CommandData.null_az_trim;
        MagAzN.trim = CommandData.mag_az_trim[0];
        MagAzS.trim = CommandData.mag_az_trim[1];
        PSSAz.trim = CommandData.pss_az_trim;
        DGPSAz.trim = CommandData.dgps_az_trim;
    }

    point_index = INC_INDEX(point_index);

    CommandData.clin_el_trim = ClinEl.trim;
    CommandData.enc_motor_el_trim = EncMotEl.trim;
    CommandData.null_az_trim = NullAz.trim;
    CommandData.mag_az_trim[0] = MagAzN.trim;
    CommandData.mag_az_trim[1] = MagAzS.trim;
    CommandData.pss_az_trim = PSSAz.trim;
    CommandData.dgps_az_trim = DGPSAz.trim;
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

  NewAzEl.rate = 360.0; // star cameras are right
  NewAzEl.fresh = 1;
}

void InitializePointingData()
{
    int which;
    for (which = 0; which < 2; which++) {
        xsc_pointing_state[which].last_trigger.counter_mcp = 0;
        xsc_pointing_state[which].last_trigger.counter_stars = -1;
        xsc_pointing_state[which].last_trigger.lat = 0.0;
        xsc_pointing_state[which].last_trigger.lst = 0;
        xsc_pointing_state[which].counter_mcp = -1;
        xsc_pointing_state[which].last_counter_mcp = -1;
        xsc_pointing_state[which].last_solution_stars_counter = -1;
        xsc_pointing_state[which].az = 0.0;
        xsc_pointing_state[which].el = 0.0;
        xsc_pointing_state[which].exposure_time_cs = 300;
        xsc_pointing_state[which].predicted_streaking_px = 0.0;
    }
    blast_info("InitializePointingData, xsc.az is %f\n", xsc_pointing_state[1].az);
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
    CommandData.XSC[dest].el_trim -= from_degrees(delta_el);
    CommandData.XSC[dest].cross_el_trim -= from_degrees(delta_az*cos(from_degrees(PointingData[i_point].el)));
}

void AzElTrim(double az, double el)
{
  NewAzEl.az = az;
  NewAzEl.el = el;

  NewAzEl.rate = 360.0; // allow arbitrary trim changes
  NewAzEl.fresh = 1;
}

void ClearTrim()
{
  NewAzEl.fresh = -1;
}
