/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of mcp.
 * 
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
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

// ***************************************************
// *                                                 *
// *  starpos: routines for accurate location of     *
// *       heavenly objects                          *
// *                                                 *
// *  Programmed by Adam Hincks                      *
// *                                                 *
// *  See the readme file for a working overview on  *
// *  how to use this file, and for a discussionon   *
// *  the accuracy it provides.                      *
// *                                                 *
// ***************************************************

#include "ephem_read.h"
#ifndef TYPES_DEFINED
#include "ephem_types.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <limits.h>

#include "command_struct.h"
#include "mcp.h"

struct ScheduleType _S[2][3]; /* sched.c */

#define EPHEM_FILE "/data/etc/ephem.2000"

#define AU         149597870.691                // in km
#define DIES       86400.0                      // seconds in a day
#define PI         3.1415926535897932384626434  // good ol' pi
#define AUPERC     21.095  // conversion from km/s to AU/century
#define MUPERCC    0.00000000987                // mu / c^2 in au
#define INV_C      0.0057755                    // 1 / c

//--------------------------------------------------------
//
// HourToRad: converts hms to radians
//
// h: hours
// m: minutes
// s: seconds
// 
//--------------------------------------------------------  

double HourToRad(int h, int m, float s) {
  if (h < 0) {
    m *= -1;
    s *= - 1;
  }

  return PI * ((s / 60.0 + m) / 60.0 + h) * 15.0 / 180.0;
}


//--------------------------------------------------------
//
// DegToRad: converts dms to radians
//
// d: degrees
// m: arcminutes
// s: arcseconds
// 
//--------------------------------------------------------  

double DegToRad(int d, int m, float s) {
  if (d < 0) {
    m *= -1;
    s *= -1;
  }

  return PI * ((s / 60.0 + m) / 60.0 + d) / 180.0;
}


//--------------------------------------------------------
//
// GetJulian: convert C date to Julian date.  Algorithm from
//      "Practical Astronomy with you Calculator", p. 7
//
//   *now: time structure from time.h library
//  
//--------------------------------------------------------

double GetJulian(time_t t) {
  int y, m;
  double d;
  int t_a, t_b, t_c, t_d;
  struct tm now;

  gmtime_r(&t, &now);

  y = now.tm_year + 1900;
  m = now.tm_mon + 1;
  d = now.tm_mday + (now.tm_hour + (now.tm_sec / 60.0 + now.tm_min) / 60.0)
    / 24.0;

  if (m == 1 || m == 2) {
    m += 12;
    y--;
  }

  if (y > 1582 || (y == 1582 && (m > 10 || (m == 10 && d >= 15)))) {
    t_a = (int)(y / 100);
    t_b = 2 - t_a + (int)(t_a / 4);
  }
  else
    t_b = 0;

  if (y < 0)
    t_c = (int)((365.25 * y) - 0.75);
  else
    t_c = (int)(365.25 * y);

  t_d = (int)(30.6001 * (m + 1));

  return t_b + t_c + t_d + d + 1720994.5;
}


//--------------------------------------------------------
//
// Norm: calculate the norm of a vector
//
//   *vector: the vector
//   num: dimension of vector
//
//   Returns: the norm
//
//--------------------------------------------------------

double Norm(double *vector, int num) {
  int i;
  double sum;

  sum = 0;
  for (i = 0; i < num; i++)
    sum += vector[i] * vector[i];

  return sqrt(sum);
}


//--------------------------------------------------------
//
// PrintAngle: takes a decimal angle and printfs it in
//      degrees/hours, minutes, & seconds
//
//   angle: the angle to print
//
//--------------------------------------------------------

void PrintAngle(double angle) {
  char sign;

  if (angle < 0) {
    sign = '-';
    angle *= -1;
  }
  else
    sign = '+';

  bprintf(info, "Ephemeris: %c%d %dm %.4gs", sign, (int)(angle), (int)((angle -
          (int)angle) * 60.0), ((angle - (int)angle) * 60.0 -
                                (int)((angle - (int)angle) * 60.0)) * 60);
}


//--------------------------------------------------------
//
// CommonCorrections: corrections to an object's position
//      which is identical for stars, planets, & the sun.
//      Steps 4-6 are done here for the stellar and
//      planetary reductions in the "Astronomical
//      Almanac", pp. B36-B41.  Algorithms for finding the
//      rotation matrices for precession & nutation taken 
//      from "Practical Astronomy with your Calculator",
//      pp. 57-61.
//
//   t: Julian date
//   p: position vector before common corrections
//   baryEdot: barycentric velocity vector of the earth
//   *ra, *dec: where to write the final ra and dec
//
//--------------------------------------------------------

void CommonCorrections(double t, double p[3], double baryEdot[3], double *ra,
    double *dec) {
  int i, j;
  double zeta, zed, theta, cx, sx, cz, sz, ct, st, A, L, O, T;
  double delta_phi, delta_eps, eps;
  double Prec[3][3], Nut[3][3], R[3][3];
  double p2[3], Beta, V[3], p_dot_V, p3[3];


  // STEP 4
  // Correct for the earth's velocity through space (I think this takes
  // care of abberation).
  for (i = 0; i < 3; i++)
    V[i] = INV_C * baryEdot[i];
  Beta = sqrt(1 - pow(Norm(V, 3), 2));
  p_dot_V = p[0] * V[0] + p[1] * V[1] + p[2] * V[2];
  for (i = 0; i < 3; i++)
    p2[i] = (Beta * p[i] + V[i] * (1 + p_dot_V) / (1 + Beta)) / 
      (1 + p_dot_V);

  // STEP 5
  // Find rotation matrix to correct for precession and nutation

  // Find precession matrix
  //   T: centuries since J2000.0
  //   zeta, zed, theta: parameters used in calculation
  //   Prec: the rotation matrix to correct for precession
  T = (t - 2451545.0) / 36525.0;
  zeta  = PI * (0.6406161 * T + 0.0000839 * T * T + 0.0000050 * T * T * T) /
    180.0;
  zed   = PI * (0.6406161 * T + 0.0003041 * T * T + 0.0000051 * T * T * T) /
    180.0;
  theta = PI * (0.5567530 * T - 0.0001185 * T * T - 0.0000116 * T * T * T) /
    180.0;

  cx = cos(zeta);
  sx = sin(zeta);
  cz = cos(zed);
  sz = sin(zed);
  ct = cos(theta);
  st = sin(theta);
  Prec[0][0] = cx * ct * cz - sx * sz;
  Prec[0][1] = -1 * sx * ct * cz - cx * sz;
  Prec[0][2] = -1 * st * cz;
  Prec[1][0] = cx * ct * sz + sx * cz;
  Prec[1][1] = cx * cz - sx * ct * sz;
  Prec[1][2] = -1 * st * sz;
  Prec[2][0] = cx * st;
  Prec[2][1] = -1 * sx * st;
  Prec[2][2] = ct;

  // Find rotation matrix for nutation
  //   eps: current mean obliquity of the ecliptic
  //   T: we use centuries since 1900 Jan. 0.5
  //   T, L, O: parameters for calculation
  //   deltap_phi, delta_eps: change to the mean longitude and
  //      obliquity of the ecliptic
  //   Nut: the rotation matrix for nutation
  eps = 0.409092813922 - 0.000226965518154 * T - 0.00000000290888 * T * T + 
    0.000000008775 * T * T * T;
  T = (t - 2415020.0) / 36525.0;
  A = 100.002136 * T;
  L = PI * (279.6967 + 360.0 * (A - (int)A)) / 180.0;
  A = 5.372617 * T;
  O = PI * (259.1833 - 360.0 * (A - (int)A)) / 180.0;
  delta_phi = PI * (-17.2 * sin(O) - 1.3 * sin(2 * L)) / (3600.0 * 180.0);
  delta_eps = PI * (9.2 * cos(O) + 0.5 * cos(2 * L)) / (3600.0 * 180.0);
  Nut[0][0] = 1;
  Nut[0][1] = -1 * delta_phi * cos(eps);
  Nut[0][2] = -1 * delta_phi * sin(eps);
  Nut[1][0] = delta_phi * cos(eps);
  Nut[1][1] = 1;
  Nut[1][2] = -1 * delta_eps;
  Nut[2][0] = delta_phi * sin(eps);
  Nut[2][1] = delta_eps;
  Nut[2][2] = 1;

  // Multiply Nut x Prec to get combined rotation matrix
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++)
      R[i][j] = Nut[i][0] * Prec[0][j] + Nut[i][1] * Prec[1][j] +
        Nut[i][2] * Prec[2][j];
  }
  // Find p3 = R * p2
  for (i = 0; i < 3; i++)
    p3[i] = R[i][0] * p2[0] + R[i][1] * p2[1] + R[i][2] * p2[2];

  // STEP 6
  // Convert back to ra & dec (with correct quadrant for ra), thanks to atan2
  *dec = asin(p3[2]);
  *ra = atan2(p3[1], p3[0]);
}

//--------------------------------------------------------
// 
// StarPos: stellar reduction -- takes information for
//      a star from J2000.0, and returns current
//      ra and dec.  Algorithm taken from "Astronomical
//      Almanac 2001" B39-B40; in this function are steps
//      1 - 3; "CommonCorrections" function does the rest
//      
//
//   t: Julian date
//   ra/dec: J2000.0 right ascension, declination (rads)
//   mra/mdec = proper motions in ra & dec (rads/century),
//      0 if not available
//   rvel = radial velocity (km/s), 0 if not available
//   pi = parallax (rads), 0 if not available
//   *ra/*dec: where to write the current ra & dec
//
//--------------------------------------------------------


void StarPos(double t, double ra0, double dec0, double mra, double mdec,
    double pi, double rvel, double *ra, double *dec) {

  int i;
  stateType tstate;
  double baryE[3], baryEdot[3], baryS[3];
  double q[3], m[3];
  double P[3], E[3], p[3], e[3];
  double p_dot_e;
  double T;
  double p1[3];

  // STEP 1
  // Set TDB = TT -- just use the t passed to the function.

  // STEP 2
  // Get barycentric ephemeris for earth and sun.  Use functions in the
  // ephem_read.o file which reads in the ephemeris from EPHEM_FILE
  //   baryE: earth's barycentric position (AU)
  //   baryEdot: earth's barycentric velocity (AU/day)
  //   baryS: sun's position (AU)
  Interpolate_State(t, EARTH, &tstate);
  Interpolate_Position(t, SUN, baryS);
  for (i = 0; i < 3; i++) {
    baryE[i] = (tstate.Position[i]) / AU;
    baryEdot[i] = tstate.Velocity[i] * DIES / AU;
    baryS[i] = baryS[i] / AU;
  }

  // Calculate vectors for the star etc.
  //   q: barycentric position vector of star
  //   m: "space motion" vector of star
  //   P: geocentric vector of star at t
  //   E: heliocentric position of earth
  //   p, e: unit vectors of P, E
  q[0] = cos(ra0) * cos(dec0);
  q[1] = sin(ra0) * cos(dec0);
  q[2] = sin(dec0);
  m[0] = -1 * mra * cos(dec0) * sin(ra0) - mdec * sin(dec0) * cos(ra0) +
    rvel * AUPERC * pi * cos(dec0) * cos(ra0);
  m[1] = mra * cos(dec0) * cos(ra0) - mdec * sin(dec0) * sin(ra0) +
    rvel * AUPERC * pi * cos(dec0) * sin(ra0);
  m[2] = mdec * cos(dec0) + rvel * AUPERC * pi * sin(dec0);

  T = (t - 2451545.0) / 36525.0;
  for (i = 0; i < 3; i++) {
    P[i] = q[i] + T * m[i] - pi * baryE[i];
    E[i] = baryE[i] - baryS[i];
  }
  for (i = 0; i < 3; i++) {
    p[i] = P[i] / Norm(P, 3);
    e[i] = E[i] / Norm(E, 3);
  }

  // STEP 3
  // Correct for light deflection due to GR effects of sun.
  //   p1: p corrected for light deflection
  p_dot_e = p[0] * e[0] + p[1] * e[1] + p[2] * e[2];
  for (i = 0; i < 3; i++)
    p1[i] = p[i] + (MUPERCC * Norm(E, 3)) * (e[i] - p_dot_e * p[i]) /
      (1 + p_dot_e);

  // STEPS 4-6 -- use the common corrections function; it will set ra and dec.
  // as well, so we are finished here
  CommonCorrections(t, p1, baryEdot, ra, dec);
}


//--------------------------------------------------------
//
// PlanetPos: Finds the ra and dec of a planet, including
//      the moon. Algorithm taken from "Astronomical
//      Almanac 2001" B36-B37; in this function are steps
//      1 - 3; "CommonCorrections" function does the rest
//
//   tt: Julian date
//   target: integer representing the planet of interest;
//      see ephem_types.h for an index
//   *ra, *dec: where to write the final result
//
//--------------------------------------------------------

void PlanetPos(double tt, int target, double *ra, double *dec) {
  stateType tstate;
  int i, j;
  double g, t;
  double baryE[3], baryEdot[3], baryQ[3], baryS[3];
  double tau, normP, normE = 0, normQ, oldnormP;
  double P[3], E[3], Q[3];
  double p[3], e[3], q[3];
  double p_dot_q, e_dot_p, q_dot_e;
  double p1[3];

  // STEP 1
  // Convert from TT to TDB time.
  //   g = parameter used to approximate the conversion
  g = 6.24007567466 + 0.0172019703436 * (tt - 2451545.0);
  t = tt + 0.001658 * sin(g) + 0.000014 * sin(2 * g);

  // STEP 2
  // Get the barycentric position of earth, sun, and planet.  Light travel
  // time is important, so the positions of the planets and sun must be
  // solved for iteratively (that's what the do . . . while loop is for).
  //   baryE: barycentric position vector of earth
  //   baryEdot: barycentric velocity vector of earth
  //   tau: light time for the iterative solution
  //   baryS, baryQ: barycentric position vectors for sun & planet
  //   P, E, Q: displacement vectors
  //   normP, normE, normQ: norms of these vectors
  //   oldnormP: for checking for convergence of iterative solution
  //   p, e, q: unit vectors of P, E, Q
  Interpolate_State(t, EARTH, &tstate);
  for (i = 0; i < 3; i++) {
    baryE[i] = tstate.Position[i] / AU;
    baryEdot[i] = tstate.Velocity[i] * DIES / AU;
  }

  // Iterate for the planet & sun's position
  tau = 0;
  normP = UINT_MAX; // Make this number ridiculous so that normP -
  j = 0;                      // oldnorm will be large the first time through
  do { 
    oldnormP = normP;
    Interpolate_Position(t - tau, SUN, baryS);
    Interpolate_Position(t - tau, target, baryQ);
    if (target == MOON) {     // The ephemeris for the moon is relative to
      for (i = 0; i < 3; i++) // the geocentre
        baryQ[i] += baryE[i] * AU;
    }
    if (tau == 0) {           // First time through get value of E, normE
      for (i = 0; i < 3; i++)
        E[i] = baryE[i] - baryS[i] / AU;
      normE = Norm(E, 3);
    }
    for (i = 0; i < 3; i++) {
      P[i] = baryQ[i] / AU - baryE[i];
      Q[i] = (baryQ[i] - baryS[i]) / AU;
    }
    normP = Norm(P, 3);
    normQ = Norm(Q, 3);

    tau = INV_C * (normP + 2 * MUPERCC * log((normE + normP + normQ) /
          (normE - normP + normQ)));
    if (j++ > 1000) {      // Something has gone horribly wrong!
      *ra = 999;
      *dec = 999;
      return;
    }
  } while (fabs(normP - oldnormP) < 0.00000001); // Iterate until difference
  // between adjacent values
  // is small

  for (i = 0; i < 3; i++) {
    p[i] = P[i] / normP;
    q[i] = Q[i] / normQ;
    e[i] = E[i] / normE;
  }

  // STEP 3
  // Correct for light deflection due to GR effects of sun
  //   p_dot_q etc: dot products
  //   p1: p corrected for light deflection
  p_dot_q = p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
  e_dot_p = e[0] * p[0] + e[1] * p[1] + e[2] * p[2];
  q_dot_e = q[0] * e[0] + q[1] * e[1] + q[2] * e[2];

  for (i = 0; i < 3; i++)
    p1[i] = p[i] + 2 * MUPERCC * normE * (p_dot_q * e[i] - e_dot_p * q[i]) /
      (1 + q_dot_e);

  // STEPS 4-6 -- use the common corrections function; it will set ra and dec.
  // as well, so we are finished here
  CommonCorrections(t, p1, baryEdot, ra, dec);
}


//--------------------------------------------------------
//
// SunPos: Finds the ra and dec of the sun. Algorithm 
//      taken from "Astronomical Almanac 2001" B36-B37,
//      B39; in this function are steps 1 - 3;
//      "CommonCorrections" function does the rest
//
//   tt: Julian date
//   *ra, *dec: where to write the final result
//
//--------------------------------------------------------

void SunPos(double tt, double *ra, double *dec) {
  stateType tstate;
  int i, j;
  double g, t;
  double baryE[3], baryEdot[3], baryS[3];
  double tau, normP, oldnormP;
  double P[3], p[3];

  // STEP 1
  // Convert from TT to TDB time.
  //   g = parameter used to approximate the conversion
  g = 6.24007567466 + 0.0172019703436 * (tt - 2451545.0);
  t = tt + 0.001658 * sin(g) + 0.000014 * sin(2 * g);

  // STEP 2
  // Get the barycentric position of earth, snd sun.  Light travel
  // time is important, so the position of the sun must be solved for 
  // iteratively (that's what the do . . . while loop is for).
  //   baryE: barycentric position vector of earth
  //   baryEdot: barycentric velocity vector of earth
  //   tau: light time for the iterative solution
  //   baryS: barycentric position vector for sun
  //   P: displacement vector
  //   normP: norms of these vectors
  //   oldnormP: for checking for convergence of iterative solution
  //   p: unit vectors of P
  Interpolate_State(t, EARTH, &tstate);
  for (i = 0; i < 3; i++) {
    baryE[i] = tstate.Position[i] / AU;
    baryEdot[i] = tstate.Velocity[i] * DIES / AU;
  }

  // Iterate for the planet & sun's position
  tau = 0;
  normP = UINT_MAX; // make this ridiculous so that the first time
  j = 0;                      // through, normP - oldnormP is big
  do { 
    oldnormP = normP;
    Interpolate_Position(t - tau, SUN, baryS);

    for (i = 0; i < 3; i++) 
      P[i] = baryS[i] / AU - baryE[i];
    normP = Norm(P, 3);

    tau = INV_C * normP;
    if (j++ > 1000) {      // Something has gone horribly wrong
      *ra = 999;
      *dec = 999;
      return;
    }
  } while (fabs(normP - oldnormP) < 0.00000001);

  for (i = 0; i < 3; i++)
    p[i] = P[i] / normP;

  // STEP 3
  // There is no light deflection due to the sun's GR effects since we are 
  // looking at the sun

  // STEPS 4-6 -- use the common corrections function; it will set ra and dec.
  // as well, so we are finished here
  CommonCorrections(t, p, baryEdot, ra, dec);

}

//--------------------------------------------------------
//
// ReductionInit: Must be called before SReduction can be
//      used.
//
//--------------------------------------------------------

void ReductionInit() {
  Initialize_Ephemeris(EPHEM_FILE);
}

/*********************************************************************/
/*                                                                   */
/*     conversion routines from various mysterious sources           */
/*                                                                   */
/*********************************************************************/
/************************************************************************/
/*get local sidereal time:  lon is in degrees west                      */
/************************************************************************/
double getlst(time_t t, double lon) {

  /* gmt is set to a time when gst was zero */
  /* we are assuming that frodo is set to gmt. */
  //  struct tm gmt = {56, 5, 4, // s, m, h 
  //		   21, 06, 103, 0,0,0,0}; // day, month (0-11), year-1900

  //t -= (mktime(&gmt) - timezone); 

  //  printf("%li %li %li %f\n", t, 1093312156, t - 1093312156, lon);

  // S.t0 is from first line in schedule file: see sched.c;

  t -= _S[CommandData.sucks][CommandData.lat_range].t0;

  t *= 1.002737909; // gst in seconds

  t -= lon * (24.0 * 3600.0 / 360.0);
  t = t % (24 * 3600);

  return(t);
}

/************************************************************************/
/*  converts ra (hours), dec (degrees) into az, el (degrees);           */
/************************************************************************/
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
    double *el) {

  double h;
  double sin_dec, cos_dec, sin_lat, cos_lat, cos_H;
  double sin_el, cos_el, cos_az;

  /* convert to radians */
  lat *= (M_PI/180.0);
  ra *= (M_PI/12.0);
  dec *= (M_PI/180.0);

  /* Hour angle in radians */
  h = (double)lst*(2.0*M_PI/(24.0*3600.0)) - ra;

  if (h < 0.0)
    h += 2*M_PI;

  cos_H = cos(h);
  sin_dec = sin(dec);
  cos_dec = cos(dec);
  sin_lat = sin(lat);
  cos_lat = cos(lat);

  /* Convert to az, el */     
  sin_el = sin_dec * sin_lat + (cos_dec*cos_lat*cos_H);
  (*el) = asin(sin_el);
  cos_el = cos((*el));
  (*el) *=  (180.0/M_PI);

  cos_az = (sin_dec - sin_lat*sin_el)/(cos_lat*cos_el);
  (*az) = acos(cos_az) * 180.0/M_PI;

  /* Get the signs right */
  if ((h > 0) && (h < M_PI))
    (*az) *= -1;

  //az going from 0 to 360 deg E of N
  while ((*az) < 0.0)
    (*az) += 360.0;
  while ((*az) >= 360.0)
    (*az) -= 360.0;

  //*az = (*az);
}

/************************************************************************/
/*  converts az, el (deg), into ra (h) dec (deg)                        */
/************************************************************************/
void azel2radec(double *ra_out, double *dec_out,
		double az, double el, time_t lst, double lat) {

  double sd, cosA, h1;
  double sin_lat, cos_lat, sin_el;
  double lst_h;
  double ra, dec;

  // covert inputs to radians
  lat *= M_PI/180.0;
  az *= M_PI/180.0;
  el *= M_PI/180.0;
  lst_h = (double)lst*(2.0*M_PI/(24.0*3600.0));

  sin_lat = sin(lat);
  cos_lat = cos(lat);
  sin_el = sin(el);
  sd = sin_el*sin_lat + cos(el)*cos_lat*cos(az);

  dec = asin(sd);

  cosA = ( sin_el - sin_lat*sd ) / ( cos_lat*cos(dec) );
  h1 = acos(cosA);

  if (sin(az) > 0.0) {
    ra = lst_h + h1 - 2*M_PI;
  } else {
    ra = lst_h - h1;
  }

  if (ra < 0.0)
    ra += 2*M_PI;
  else if (ra >= 2*M_PI)
    ra -= 2*M_PI;

  *ra_out = ra * 12.0/M_PI;
  *dec_out = dec*180.0/M_PI;
}
