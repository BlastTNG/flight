// Astrometry routines for star cameras - depends on SLALIB

#ifndef __ASTRO_H
#define __ASTRO_H

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "HTMCatalog.h"
#include "slalib.h"
#include "pyramid.h"       // Pyramid star matching code

#ifndef PI
#define PI 3.14159265358979
#endif

// Used for sorting the reduced star catalogue
int compare( const void *arg1, const void *arg2 );

// calculate the apparent RA and DEC of blobs in a CCD frame given the 
// RA and DEC of the map centre, 
// and the tangent plane offsets of the blobs from the centre. 
void tan2radec(  double ra_0, double dec_0, double *ra_off, 
                 double *dec_off, double *ra, double *dec, int nblobs);

// Calculate tangent plane offsets from ra and dec
void radec2tan(  double ra_0, double dec_0, double *ra, double *dec, 
                 double *ra_off, double *dec_off, int nblobs);

// Calculate GST from UT date
double get_gst( int year, int month, int day, double dayfrac );

// Solve for map centre + rotation
int map_centre( double *x, double *y, double *flux, int nblobs, 
                double *ra, double *dec, double lst, double lat, 
                double *ra_0, double *dec_0, double *rot, 
                double platescale, double *var );

// Solve for map centre
int map_centre3( double *x, double *y, double *flux, int nblobs, 
                 double *ra, double *dec, 
                 double lst, double lat, double *ra_0, 
                 double *dec_0, double rot, double platescale, 
                 double *var );

// Calculate the apparent RA and DEC given az, el, lat and LST
void calc_ra_dec( double az, double el, double lat, double lst, 
                  double *RA, double *DEC );

// Calculate the Az and El given RA, Dec, Lat and LST
void calc_alt_az( double ra, double dec, double lat, double lst, 
                  double *el, double *az );

// Calculate the apparent parallactic angle given LST, RA, DEC, LAT
// (rotation of az/el wrt ra/dec)
double calc_parallactic( double lst, double ra, double dec, double lat );

// Convert between coordinates on celestial sphere and unit vectors
void cel2vec( double ra, double dec, double *x, double *y, double *z );
void vec2cel( double x, double y, double z, double *ra, double *dec );

// Extract a small star catalogue for a given region on the sky
void catalogue( double ra_0, double dec_0, double radius, double mag_0, 
                double **ra, double **dec, double **mag, int *ncat );

// Fast solution for the frame matching
int match_frame( double ra_0_guess, double dec_0_guess, double size, 
                 double maglimit, double tolerance, 
                 double lst, double lat, double rot, 
                 double platescale, double sig_tol, 
                 double match_tol, double quit_tol, double rot_tol,
                 double *x, double *y, double *flux, double *ra, 
                 double *dec, double *mag, int nblobs, int *abtflag,
                 int brightStarMode, double brightRA, double brightDEC);

// Simple frame matching algorithm
int match_frame_simple( double ra_0_guess, double dec_0_guess, 
                        double size, double maglimit, double tolerance, 
                        double lst, double lat, double rot, 
                        double platescale, 
                        double *x, double *y, double *ra, 
                        double *dec, double *mag, int nblobs );

// Calculate epoch from the system date (return value in years)
double calc_epoch( void );

// Calculate a pointing solution
int calc_pointing( double ra_0_guess, double dec_0_guess, 
                   int lost, double epoch,
                   double lat, double lst, double *x, double *y, 
                   double *flux, int nblobs, int minplateblobs,  
                   double radius, double maglimit, 
                   double tolerance, double tol_simp, 
                   double sig_tol, double match_tol, 
                   double quit_tol, double rot_tol, double *ra_0, 
                   double *dec_0, double *var, double *rot, 
                   double *platescale,
                   double *star_ra, double *star_dec, double *star_mag, 
                   int *abtflag, int brightStarMode, double brightRA, 
                   double brightDEC );

// Initialize / close star catalogue
void astro_init_catalogue( const char *pathname, char *catalogname,
                           char *katalogname );
void astro_close_catalogue( void );

#endif
