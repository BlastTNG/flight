#ifndef TOOLS_SLALIB_H
#define TOOLS_SLALIB_H

#include <math.h>
#include <ctype.h>
#include <string.h>

namespace Slalib
{

// Routines lifted from SLALib

/* 2pi */
#define D2PI 6.2831853071795864769252867665590057683943387987502


/* pi/(180*3600):  arcseconds to radians */
#define DAS2R 4.8481368110953599358991410235794797595635330237270e-6

/* dmod(A,B) - A modulo B (double) */
#define dmod(A,B) ((B)!=0.0?((A)*(B)>0.0?(A)-(B)*floor((A)/(B))\
                                        :(A)+(B)*floor(-(A)/(B))):(A))

// Normalize angle to range 0-2PI
double slaDranrm ( double angle );
float slaRanorm ( float angle );

// Calculate map centre 
void slaTps2c ( double xi, double eta, double ra, double dec,
                double *raz1, double *decz1,
                double *raz2, double *decz2, int *n );

// Dependencies for precess
void slaPrebn ( double bep0, double bep1, double rmatp[3][3] );

void slaDmxv ( double dm[3][3], double va[3], double vb[3] );

void slaDeuler ( char *order, double phi, double theta,
                 double psi, double rmat[3][3] );

void slaDcs2c ( double a, double b, double v[3] );

void slaDcc2s ( double v[3], double *a, double *b );

void slaPrec ( double ep0, double ep1, double rmatp[3][3] );

// Precess coordinates
void slaPreces ( char sys[3], double ep0, double ep1,
                 double *ra, double *dc );

// Date and epoch routines
void slaCaldj ( int iy, int im, int id, double *djm, int *j );

void slaCldj ( int iy, int im, int id, double *djm, int *j );

double slaEpj ( double date );

// Other stuff

void slaDtp2s ( double xi, double eta, double raz, double decz,
                double *ra, double *dec );

void slaDs2tp ( double ra, double dec, double raz, double decz,
                double *xi, double *eta, int *j );

void slaDh2e ( double az, double el, double phi, double *ha, double *dec );

void slaDe2h ( double ha, double dec, double phi, double *az, double *el );

double slaPa ( double ha, double dec, double phi );

}

#endif

