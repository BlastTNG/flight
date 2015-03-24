#ifndef _utilities_pointing_H
#define _utilities_pointing_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <conversions.h>

#define PI				M_PI
#define PIBYTWO			M_PI_2
#define TWOPI			(2.0 * M_PI)
#define RJD0			2.45490e6        /* For calculating a reduced Julian day  */

//#endif

/* Structure definitions */

typedef struct {
  int year;   // e.g. 2009
  int month;  // 1--12
  int day;    // 1--31 
  double secs;// Seconds since the beginning of the day.
} CIVILTIME;

/* Public functions declarations */
void azel_2_hadec(double az[], double el[], int n, double cosL, double sinL,
		  double *ha, double *dec, double *cosdec);
void azel_2_hadecbeta(double az[], double el[], int n, double cosL, double sinL,
		      double *ha, double *dec, double *beta, double *cosdec); 
void azel_2_hadec_multidetector(double az, double el, double daz[], double cosdel[],
				double sindel[],long int n, double cosL, double sinL,
				double *ha, double *dec, double *cosdec); 
void azel_2_hadec_multidetector_new(double az, double el, double cosdaz[], double sindaz[],
				    double cosdel[], double sindel[], long int n,
				    double cosL, double sinL,
				    double *ha, double *dec, double *cosdec); 
int hadec_2_azel(double HA[],double Dec[],int n, double L,double *az, double *el);
void rael_2_az(double RA, double el, double lst, double cosL, double sinL,double *az);
void determinelst(CIVILTIME ct[], double longitude[], int n, double *LST);
void determinelst_new(CIVILTIME ct[], double longitude[], int n, double *LST);
void jdcnv(CIVILTIME ct[], int n, double *jd);
void jdcnv_new(CIVILTIME ct[], int n, double *jd);
void get_rjd(CIVILTIME ct, double sample_times_sec[],long int numbersamples, double *rjd);
void ha2ra(double ha[], double lst[], long int n, double *ra);
void convert_unixtime_to_civiltime(time_t t, CIVILTIME *ct);

/*
 * Returns an angle that is between 0 and limit.
 *
 * angle -- The angle to be reduced.
 * limit -- The maximum value for the angle.
 *
 * Return: An angle that is between 0 and limit.
 */
static inline double range_fast(double *m_val, double m_limit)
{
	double val = *m_val;
	while (val < 0.0)
		val += m_limit;
	while (val >= m_limit)
		val -= m_limit;

	*m_val = val;
	return val;
}

#endif
