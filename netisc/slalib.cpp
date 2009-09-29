# include "slalib.h"

//-----------------------------------------------------------------------------
// Routines lifted from SLALib

double slaDranrm ( double angle )
/*
**  - - - - - - - - - -
**   s l a D r a n r m
**  - - - - - - - - - -
**
**  Normalize angle into range 0-2 pi.
**
**  (double precision)
**
**  Given:
**     angle     double      the angle in radians
**
**  The result is angle expressed in the range 0-2 pi (double).
**
**  Defined in slamac.h:  D2PI, dmod
**
**  Last revision:   19 March 1996
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double w;

   w = dmod ( angle, D2PI );
   return ( w >= 0.0 ) ? w : w + D2PI;
}

float slaRanorm ( float angle )
/*
**  - - - - - - - - - -
**   s l a R a n o r m
**  - - - - - - - - - -
**
**  Normalize angle into range 0-2 pi.
**
**  (single precision)
**
**  Given:
**     angle     double      the angle in radians
**
**  The result is angle expressed in the range 0-2 pi (single
**  precision).
**
**  Last revision:   15 July 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
  return (float) slaDranrm ( (double) angle );
}

void slaTps2c ( double xi, double eta, double ra, double dec,
                double *raz1, double *decz1,
                double *raz2, double *decz2, int *n )
/*
**  - - - - - - - - -
**   s l a T p s 2 c
**  - - - - - - - - -
**
**  From the tangent plane coordinates of a star of known RA,Dec,
**  determine the RA,Dec of the tangent point.
**
**  (single precision)
**
**  Given:
**     xi,eta        float   tangent plane rectangular coordinates
**     ra,dec        float   spherical coordinates
**
**  Returned:
**     *raz1,*decz1  float   spherical coordinates of TP, solution 1
**     *raz2,*decz2  float   spherical coordinates of TP, solution 2
**     *n            int     number of solutions:
**                            0 = no solutions returned (note 2)
**                            1 = only the first solution is useful (note 3)
**                            2 = both solutions are useful (note 3)
**
**  Notes:
**
**  1  The raz1 and raz2 values are returned in the range 0-2pi.
**
**  2  Cases where there is no solution can only arise near the poles.
**     For example, it is clearly impossible for a star at the pole
**     itself to have a non-zero xi value, and hence it is
**     meaningless to ask where the tangent point would have to be
**     to bring about this combination of xi and dec.
**
**  3  Also near the poles, cases can arise where there are two useful
**     solutions.  The argument n indicates whether the second of the
**     two solutions returned is useful;  n=1 indicates only one useful
**     solution, the usual case;  under these circumstances, the second
**     solution corresponds to the "over-the-pole" case, and this is
**     reflected in the values of raz2 and decz2 which are returned.
**
**  4  The decz1 and decz2 values are returned in the range +/-pi, but
**     in the usual, non-pole-crossing, case, the range is +/-pi/2.
**
**  5  This routine is the spherical equivalent of the routine slaTpv2c.
**
**  Called:  slaRanorm
**
**  Last revision:   5 June 1995
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
  double x2, y2, sd, cd, sdf, r2, r, s, c;

  x2 = xi * xi;
  y2 = eta * eta;
  sd = (double) sin ( (double) dec );
  cd = (double) cos ( (double) dec );
  sdf = sd * (double) sqrt ( (double) ( 1.0f + x2 + y2 ) );
  r2 = cd * cd * ( 1.0f + y2 ) - sd * sd * x2;
  if ( r2 >= 0.0f ) {
     r = (double) sqrt ( (double) r2 );
     s = sdf - eta * r;
     c = sdf * eta + r;
     if ( xi == 0.0f && r == 0.0f ) {
        r = 1.0f;
     }
     *raz1 = slaDranrm ( ra - (double) atan2 ( (double) xi, (double) r ) );
     *decz1 = (double) atan2 ( (double) s, (double) c );
     r = -r;
     s = sdf - eta * r;
     c = sdf * eta + r;
     *raz2 = slaDranrm ( ra - (double) atan2 ( (double) xi, (double) r ) );
     *decz2 = (double) atan2 ( (double) s, (double) c );
     *n = ( fabs ( (double) sdf ) < 1.0 ) ? 1 : 2;
  } else {
     *n = 0;
  }
}

void slaPrebn ( double bep0, double bep1, double rmatp[3][3] )
/*
**  - - - - - - - - -
**   s l a P r e b n
**  - - - - - - - - -
**
**  Generate the matrix of precession between two epochs,
**  using the old, pre-IAU1976, Bessel-Newcomb model, using
**  Kinoshita's formulation (double precision)
**
**  Given:
**     BEP0    double        beginning Besselian epoch
**     BEP1    double        ending Besselian epoch
**
**  Returned:
**     RMATP   double[3][3]  precession matrix
**
**  The matrix is in the sense   v(bep1)  =  rmatp * v(bep0)
**
**  Reference:
**     Kinoshita, H. (1975) 'Formulas for precession', SAO Special
**     Report No. 364, Smithsonian Institution Astrophysical
**     Observatory, Cambridge, Massachusetts.
**
**  Called:  slaDeuler
**
**  Defined in slamac.h:  DAS2R
**
**  Last revision:   30 October 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double bigt, t, tas2r, w, zeta, z, theta;

/* Interval between basic epoch B1850.0 and beginning epoch in TC */
   bigt  = ( bep0 - 1850.0 ) / 100.0;

/* Interval over which precession required, in tropical centuries */
   t = ( bep1 - bep0 ) / 100.0;

/* Euler angles */
   tas2r = t * DAS2R;
   w = 2303.5548 + ( 1.39720 + 0.000059 * bigt ) * bigt;
   zeta = (w + ( 0.30242 - 0.000269 * bigt + 0.017996 * t ) * t ) * tas2r;
   z = (w + ( 1.09478 + 0.000387 * bigt + 0.018324 * t ) * t ) * tas2r;
   theta = ( 2005.1125 + ( - 0.85294 - 0.000365* bigt ) * bigt +
           ( - 0.42647 - 0.000365 * bigt - 0.041802 * t ) * t ) * tas2r;

/* Rotation matrix */
   slaDeuler ( "ZYZ", -zeta, theta, -z, rmatp );
}


void slaDmxv ( double dm[3][3], double va[3], double vb[3] )
/*
**  - - - - - - - -
**   s l a D m x v
**  - - - - - - - -
**
**  Performs the 3-d forward unitary transformation:
**     vector vb = matrix dm * vector va
**
**  (double precision)
**
**  Given:
**     dm       double[3][3]    matrix
**     va       double[3]       vector
**
**  Returned:
**     vb       double[3]       result vector
**
**  Note:  va and vb may be the same array.
**
**  Last revision:   6 November 1999
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   int i, j;
   double w, vw[3];

/* Matrix dm * vector va -> vector vw */
   for ( j = 0; j < 3; j++ ) {
      w = 0.0;
      for ( i = 0; i < 3; i++ ) {
         w += dm[j][i] * va[i];
      }
      vw[j] = w;
   }

/* Vector vw -> vector vb */
   for ( j = 0; j < 3; j++ ) {
      vb[j] = vw[j];
   }
}

void slaDeuler ( char *order, double phi, double theta,
                 double psi, double rmat[3][3] )
/*
**  - - - - - - - - - -
**   s l a D e u l e r
**  - - - - - - - - - -
**
**  Form a rotation matrix from the Euler angles - three successive
**  rotations about specified Cartesian axes.
**
**  (double precision)
**
**  Given:
**    *order char     specifies about which axes the rotations occur
**    phi    double   1st rotation (radians)
**    theta  double   2nd rotation (   "   )
**    psi    double   3rd rotation (   "   )
**
**  Returned:
**    rmat   double[3][3]  rotation matrix
**
**  A rotation is positive when the reference frame rotates
**  anticlockwise as seen looking towards the origin from the
**  positive region of the specified axis.
**
**  The characters of order define which axes the three successive
**  rotations are about.  A typical value is 'zxz', indicating that
**  rmat is to become the direction cosine matrix corresponding to
**  rotations of the reference frame through phi radians about the
**  old z-axis, followed by theta radians about the resulting x-axis,
**  then psi radians about the resulting z-axis.
**
**  The axis names can be any of the following, in any order or
**  combination:  x, y, z, uppercase or lowercase, 1, 2, 3.  Normal
**  axis labelling/numbering conventions apply;  the xyz (=123)
**  triad is right-handed.  Thus, the 'zxz' example given above
**  could be written 'zxz' or '313' (or even 'zxz' or '3xz').  Order
**  is terminated by length or by the first unrecognized character.
**
**  Fewer than three rotations are acceptable, in which case the later
**  angle arguments are ignored.  Zero rotations leaves rmat set to the
**  identity matrix.
**
**  Last revision:   9 December 1996
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   int j, i, l, n, k;
   double result[3][3], rotn[3][3], angle, s, c , w, wm[3][3];
   char axis;

/* Initialize result matrix */
   for ( j = 0; j < 3; j++ ) {
      for ( i = 0; i < 3; i++ ) {
         result[i][j] = ( i == j ) ? 1.0 : 0.0;
      }
   }

/* Establish length of axis string */
   l = (int)strlen ( order );

/* Look at each character of axis string until finished */
   for ( n = 0; n < 3; n++ ) {
      if ( n <= l ) {

      /* Initialize rotation matrix for the current rotation */
         for ( j = 0; j < 3; j++ ) {
            for ( i = 0; i < 3; i++ ) {
               rotn[i][j] = ( i == j ) ? 1.0 : 0.0;
            }
         }

      /* Pick up the appropriate Euler angle and take sine & cosine */
         switch ( n ) {
         case 0 :
           angle = phi;
           break;
         case 1 :
           angle = theta;
           break;
         default:
           angle = psi;
           break;
         }
         s = sin ( angle );
         c = cos ( angle );

      /* Identify the axis */
         axis =  order[n];
         if ( ( axis == 'X' ) || ( axis == 'x' ) || ( axis == '1' ) ) {

         /* Matrix for x-rotation */
            rotn[1][1] = c;
            rotn[1][2] = s;
            rotn[2][1] = -s;
            rotn[2][2] = c;
         }
         else if ( ( axis == 'Y' ) || ( axis == 'y' ) || ( axis == '2' ) ) {

         /* Matrix for y-rotation */
            rotn[0][0] = c;
            rotn[0][2] = -s;
            rotn[2][0] = s;
            rotn[2][2] = c;
         }
         else if ( ( axis == 'Z' ) || ( axis == 'z' ) || ( axis == '3' ) ) {

         /* Matrix for z-rotation */
            rotn[0][0] = c;
            rotn[0][1] = s;
            rotn[1][0] = -s;
            rotn[1][1] = c;
         } else {

         /* Unrecognized character - fake end of string */
            l = 0;
         }

      /* Apply the current rotation (matrix rotn x matrix result) */
         for ( i = 0; i < 3; i++ ) {
            for ( j = 0; j < 3; j++ ) {
               w = 0.0;
               for ( k = 0; k < 3; k++ ) {
                  w += rotn[i][k] * result[k][j];
               }
               wm[i][j] = w;
            }
         }
         for ( j = 0; j < 3; j++ ) {
            for ( i= 0; i < 3; i++ ) {
               result[i][j] = wm[i][j];
            }
         }
      }
   }

/* Copy the result */
   for ( j = 0; j < 3; j++ ) {
      for ( i = 0; i < 3; i++ ) {
         rmat[i][j] = result[i][j];
      }
   }
}


void slaDcs2c ( double a, double b, double v[3] )
/*
**  - - - - - - - - -
**   s l a D c s 2 c
**  - - - - - - - - -
**
**  Spherical coordinates to direction cosines.
**
**  (double precision)
**
**  Given:
**     a,b       double      spherical coordinates in radians
**                           (RA,Dec), (long,lat) etc
**
**  Returned:
**     v         double[3]   x,y,z unit vector
**
**  The spherical coordinates are longitude (+ve anticlockwise
**  looking from the +ve latitude pole) and latitude.  The
**  Cartesian coordinates are right handed, with the x axis
**  at zero longitude and latitude, and the z axis at the
**  +ve latitude pole.
**
**  Last revision:   31 October 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double cosb;

   cosb = cos ( b );
   v[0] = cos ( a ) * cosb;
   v[1] = sin ( a ) * cosb;
   v[2] = sin ( b );
}


void slaDcc2s ( double v[3], double *a, double *b )
/*
**  - - - - - - - - -
**   s l a D c c 2 s
**  - - - - - - - - -
**
**  Direction cosines to spherical coordinates.
**
**  (double precision)
**
**  Given:
**     v      double[3]   x,y,z vector
**
**  Returned:
**     *a,*b  double      spherical coordinates in radians
**
**  The spherical coordinates are longitude (+ve anticlockwise
**  looking from the +ve latitude pole) and latitude.  The
**  Cartesian coordinates are right handed, with the x axis
**  at zero longitude and latitude, and the z axis at the
**  +ve latitude pole.
**
**  If v is null, zero a and b are returned.
**  At either pole, zero a is returned.
**
**  Last revision:   31 October 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double x, y, z, r;

   x = v[0];
   y = v[1];
   z = v[2];
   r = sqrt ( x * x + y * y );

   *a = ( r != 0.0 ) ? atan2 ( y, x ) : 0.0;
   *b = ( z != 0.0 ) ? atan2 ( z, r ) : 0.0;
}

void slaPrec ( double ep0, double ep1, double rmatp[3][3] )
/*
**  - - - - - - - -
**   s l a P r e c
**  - - - - - - - -
**
**  Form the matrix of precession between two epochs (IAU 1976, FK5).
**
**  (double precision)
**
**  Given:
**     ep0    double         beginning epoch
**     ep1    double         ending epoch
**
**  Returned:
**     rmatp  double[3][3]   precession matrix
**
**  Notes:
**
**  1)  The epochs are TDB (loosely ET) Julian epochs.
**
**  2)  The matrix is in the sense   v(ep1)  =  rmatp * v(ep0) .
**
**  3)  Though the matrix method itself is rigorous, the precession
**      angles are expressed through canonical polynomials which are
**      valid only for a limited time span.  There are also known
**      errors in the IAU precession rate.  The absolute accuracy
**      of the present formulation is better than 0.1 arcsec from
**      1960AD to 2040AD, better than 1 arcsec from 1640AD to 2360AD,
**      and remains below 3 arcsec for the whole of the period
**      500BC to 3000AD.  The errors exceed 10 arcsec outside the
**      range 1200BC to 3900AD, exceed 100 arcsec outside 4200BC to
**      5600AD and exceed 1000 arcsec outside 6800BC to 8200AD.
**      The SLALIB routine slaPrecl implements a more elaborate
**      model which is suitable for problems spanning several
**      thousand years.
**
**  References:
**     Lieske,J.H., 1979. Astron. Astrophys.,73,282.
**          equations (6) & (7), p283.
**     Kaplan,G.H., 1981. USNO circular no. 163, pa2.
**
**  Called:  slaDeuler
**
**  Defined in slamac.h:  DAS2R
**
**  Last revision:   10 July 1994
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double t0, t, tas2r, w, zeta, z, theta;

/* Interval between basic epoch J2000.0 and beginning epoch (JC) */
   t0 = ( ep0 - 2000.0 ) / 100.0;

/* Interval over which precession required (JC) */
   t =  ( ep1 - ep0 ) / 100.0;

/* Euler angles */
   tas2r = t * DAS2R;
   w = 2306.2181 + ( ( 1.39656 - ( 0.000139 * t0 ) ) * t0 );
   zeta = (w + ( ( 0.30188 - 0.000344 * t0 ) + 0.017998 * t ) * t ) * tas2r;
   z = (w + ( ( 1.09468 + 0.000066 * t0 ) + 0.018203 * t ) * t ) * tas2r;
   theta = ( ( 2004.3109 + ( - 0.85330 - 0.000217 * t0 ) * t0 )
          + ( ( -0.42665 - 0.000217 * t0 ) - 0.041833 * t ) * t ) * tas2r;

/* Rotation matrix */
   slaDeuler ( "ZYZ", -zeta, theta, -z, rmatp );
}


void slaPreces ( char sys[3], double ep0, double ep1,
                 double *ra, double *dc )
/*
**  - - - - - - - - - -
**   s l a P r e c e s
**  - - - - - - - - - -
**
**  Precession - either FK4 (Bessel-Newcomb, pre-IAU1976) or
**  FK5 (Fricke, post-IAU1976) as required.
**
**  Given:
**     sys        char[]     precession to be applied: "FK4" or "FK5"
**     ep0,ep1    double     starting and ending epoch
**     ra,dc      double     RA,Dec, mean equator & equinox of epoch ep0
**
**  Returned:
**     *ra,*dc    double     RA,Dec, mean equator & equinox of epoch ep1
**
**  Called:    slaDranrm, slaPrebn, slaPrec, slaDcs2c,
**             slaDmxv, slaDcc2s
**
**  Notes:
**
**  1)  The epochs are Besselian if sys='FK4' and Julian if 'FK5'.
**      For example, to precess coordinates in the old system from
**      equinox 1900.0 to 1950.0 the call would be:
**          slaPreces ( "FK4", 1900.0, 1950.0, &ra, &dc )
**
**  2)  This routine will not correctly convert between the old and
**      the new systems - for example conversion from B1950 to J2000.
**      For these purposes see slaFk425, slaFk524, slaFk45z and
**      slaFk54z.
**
**  3)  If an invalid sys is supplied, values of -99.0,-99.0 will
**      be returned for both ra and dc.
**
**  Last revision:   22 December 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double pm[3][3], v1[3], v2[3];

/* Validate sys */
   if ( ( toupper ( sys[0] ) != 'F' )
     || ( toupper ( sys[1] ) != 'K' )
     || ( sys[2] != '4' && sys[2] != '5' ) ) {
         *ra = -99.0;          /* Error */
         *dc = -99.0;
   } else {

   /* Generate appropriate precession matrix */
      if ( sys[2] == '4' )
         slaPrebn ( ep0, ep1, pm );
      else
         slaPrec ( ep0, ep1, pm );

   /* Convert RA,Dec to x,y,z */
      slaDcs2c ( *ra, *dc, v1 );

   /* Precess */
      slaDmxv ( pm, v1, v2 );

   /* Back to RA,Dec */
      slaDcc2s ( v2, ra, dc );
      *ra = slaDranrm ( *ra );
   }
}

void slaCaldj ( int iy, int im, int id, double *djm, int *j )
/*
**  - - - - - - - - -
**   s l a C a l d j
**  - - - - - - - - -
**
**  Gregorian calendar to Modified Julian Date.
**
**  (Includes century default feature:  use slaCldj for years
**   before 100AD.)
**
**  Given:
**     iy,im,id   int      year, month, day in Gregorian calendar
**
**  Returned:
**     *djm       double   Modified Julian Date (JD-2400000.5) for 0 hrs
**     *j         int      status:
**                           0 = ok
**                           1 = bad year   (MJD not computed)
**                           2 = bad month  (MJD not computed)
**                           3 = bad day    (MJD computed)
**
**  Acceptable years are 00-49, interpreted as 2000-2049,
**                       50-99,     "       "  1950-1999,
**                       100 upwards, interpreted literally.
**
**  Called:  slaCldj
**
**  Last revision:   21 October 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   int ny;

/* Default century if appropriate */
   if ( ( iy >= 0 ) && ( iy <= 49 ) )
      ny = iy + 2000;
   else if ( ( iy >= 50 ) && ( iy <= 99 ) )
      ny = iy + 1900;
   else
      ny = iy;

/* Modified Julian Date */
   slaCldj ( ny, im, id, djm, j );
}

void slaCldj ( int iy, int im, int id, double *djm, int *j )
/*
**  - - - - - - - -
**   s l a C l d j
**  - - - - - - - -
**
**  Gregorian calendar to Modified Julian Date.
**
**  Given:
**     iy,im,id     int    year, month, day in Gregorian calendar
**
**  Returned:
**     *djm         double Modified Julian Date (JD-2400000.5) for 0 hrs
**     *j           int    status:
**                           0 = OK
**                           1 = bad year   (MJD not computed)
**                           2 = bad month  (MJD not computed)
**                           3 = bad day    (MJD computed)
**
**  The year must be -4699 (i.e. 4700BC) or later.
**
**  The algorithm is derived from that of Hatcher 1984 (QJRAS 25, 53-55).
**
**  Last revision:   29 August 1994
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   long iyL, imL;

/* Month lengths in days */
   static int mtab[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/* Validate year */
   if ( iy < -4699 ) { *j = 1; return; }

/* Validate month */
   if ( ( im < 1 ) || ( im > 12 ) ) { *j = 2; return; }

/* Allow for leap year */
   mtab[1] = ( ( ( iy % 4 ) == 0 ) &&
             ( ( ( iy % 100 ) != 0 ) || ( ( iy % 400 ) == 0 ) ) ) ?
             29 : 28;

/* Validate day */
   *j = ( id < 1 || id > mtab[im-1] ) ? 3 : 0;

/* Lengthen year and month numbers to avoid overflow */
   iyL = (long) iy;
   imL = (long) im;

/* Perform the conversion */
   *djm = (double)
        ( ( 1461L * ( iyL - ( 12L - imL ) / 10L + 4712L ) ) / 4L
        + ( 306L * ( ( imL + 9L ) % 12L ) + 5L ) / 10L
        - ( 3L * ( ( iyL - ( 12L - imL ) / 10L + 4900L ) / 100L ) ) / 4L
        + (long) id - 2399904L );
}

double slaEpj ( double date )
/*
**  - - - - - - -
**   s l a E p j
**  - - - - - - -
**
**  Conversion of Modified Julian Date to Julian epoch.
**
**  (double precision)
**
**  Given:
**     date     double      Modified Julian Date (JD - 2400000.5)
**
**  The result is the Julian epoch.
**
**  Reference:
**     Lieske,J.H., 1979. Astron. Astrophys.,73,282.
**
**  Last revision:   31 October 1993
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
  return 2000.0 + ( date - 51544.5 ) / 365.25;
}

void slaDtp2s ( double xi, double eta, double raz, double decz,
                double *ra, double *dec )
/*
**  - - - - - - - - -
**   s l a D t p 2 s
**  - - - - - - - - -
**
**  Transform tangent plane coordinates into spherical.
**
**  (double precision)
**
**  Given:
**     xi,eta      double   tangent plane rectangular coordinates
**     raz,decz    double   spherical coordinates of tangent point
**
**  Returned:
**     *ra,*dec    double   spherical coordinates (0-2pi,+/-pi/2)
**
**  Called:  slaDranrm
**
**  Last revision:   3 June 1995
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
  double sdecz, cdecz, denom;

  sdecz = sin ( decz );
  cdecz = cos ( decz );
  denom = cdecz - eta * sdecz;
  *ra = slaDranrm ( atan2 ( xi, denom ) + raz );
  *dec = atan2 ( sdecz + eta * cdecz, sqrt ( xi * xi + denom * denom ) );
}


void slaDs2tp ( double ra, double dec, double raz, double decz,
                double *xi, double *eta, int *j )
/*
**  - - - - - - - - -
**   s l a D s 2 t p
**  - - - - - - - - -
**
**  Projection of spherical coordinates onto tangent plane
**  ('gnomonic' projection - 'standard coordinates').
**
**  (double precision)
**
**  Given:
**     ra,dec      double   spherical coordinates of point to be projected
**     raz,decz    double   spherical coordinates of tangent point
**
**  Returned:
**     *xi,*eta    double   rectangular coordinates on tangent plane
**     *j          int      status:   0 = OK, star on tangent plane
**                                    1 = error, star too far from axis
**                                    2 = error, antistar on tangent plane
**                                    3 = error, antistar too far from axis
**
**  Last revision:   18 July 1996
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
#define TINY 1e-6
{
   double sdecz, sdec, cdecz, cdec, radif, sradif, cradif, denom;


/* Trig functions */
   sdecz = sin ( decz );
   sdec = sin ( dec );
   cdecz = cos ( decz );
   cdec = cos ( dec );
   radif = ra - raz;
   sradif = sin ( radif );
   cradif = cos ( radif );

/* Reciprocal of star vector length to tangent plane */
   denom = sdec * sdecz + cdec * cdecz * cradif;

/* Handle vectors too far from axis */
   if ( denom > TINY ) {
      *j = 0;
   } else if ( denom >= 0.0 ) {
      *j = 1;
      denom = TINY;
   } else if ( denom > -TINY ) {
      *j = 2;
      denom = -TINY;
   } else {
      *j = 3;
   }

   /* Compute tangent plane coordinates (even in dubious cases) */
   *xi = cdec * sradif / denom;
   *eta = ( sdec * cdecz - cdec * sdecz * cradif ) / denom;
}

void slaDh2e ( double az, double el, double phi, double *ha, double *dec )
/*
**  - - - - - - - -
**   s l a D h 2 e
**  - - - - - - - -
**
**  Horizon to equatorial coordinates:  Az,El to HA,Dec
**
**  (double precision)
**
**  Given:
**     az          double       azimuth
**     el          double       elevation
**     phi         double       observatory latitude
**
**  Returned:
**     *ha         double       hour angle
**     *dec        double       declination
**
**  Notes:
**
**  1)  All the arguments are angles in radians.
**
**  2)  The sign convention for azimuth is north zero, east +pi/2.
**
**  3)  HA is returned in the range +/-pi.  Declination is returned
**      in the range +/-pi/2.
**
**  4)  The is latitude is (in principle) geodetic.  In critical
**      applications, corrections for polar motion should be applied.
**
**  5)  In some applications it will be important to specify the
**      correct type of elevation in order to produce the required
**      type of HA,Dec.  In particular, it may be important to
**      distinguish between the elevation as affected by refraction,
**      which will yield the "observed" HA,Dec, and the elevation
**      in vacuo, which will yield the "topocentric" HA,Dec.  If the
**      effects of diurnal aberration can be neglected, the
**      topocentric HA,Dec may be used as an approximation to the
**      "apparent" HA,Dec.
**
**  6)  No range checking of arguments is done.
**
**  7)  In applications which involve many such calculations, rather
**      than calling the present routine it will be more efficient to
**      use inline code, having previously computed fixed terms such
**      as sine and cosine of latitude.
**
**  Last revision:   21 February 1996
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double sa, ca, se, ce, sp, cp, x, y, z, r;

/* Useful trig functions */
   sa = sin ( az );
   ca = cos ( az );
   se = sin ( el );
   ce = cos ( el );
   sp = sin ( phi );
   cp = cos ( phi );

/* HA,Dec as x,y,z */
   x = - ca * ce * sp + se * cp;
   y = - sa * ce;
   z = ca * ce * cp + se * sp;

/* To spherical */
   r = sqrt ( x * x + y * y );
   *ha = ( r == 0.0 ) ? 0.0 : atan2 ( y, x ) ;
   *dec = atan2 ( z, r );
}

void slaDe2h ( double ha, double dec, double phi, double *az, double *el )
/*
**  - - - - - - - -
**   s l a D e 2 h
**  - - - - - - - -
**
**  Equatorial to horizon coordinates:  HA,Dec to Az,El
**
**  (double precision)
**
**  Given:
**     ha          double       hour angle
**     dec         double       declination
**     phi         double       observatory latitude
**
**  Returned:
**     *az         double       azimuth
**     *el         double       elevation
**
**  Notes:
**
**  1)  All the arguments are angles in radians.
**
**  2)  Azimuth is returned in the range 0-2pi;  north is zero,
**      and east is +pi/2.  Elevation is returned in the range
**      +/-pi/2.
**
**  3)  The latitude must be geodetic.  In critical applications,
**      corrections for polar motion should be applied.
**
**  4)  In some applications it will be important to specify the
**      correct type of hour angle and declination in order to
**      produce the required type of azimuth and elevation.  In
**      particular, it may be important to distinguish between
**      elevation as affected by refraction, which would
**      require the "observed" HA,Dec, and the elevation
**      in vacuo, which would require the "topocentric" HA,Dec.
**      If the effects of diurnal aberration can be neglected, the
**      "apparent" HA,Dec may be used instead of the topocentric
**      HA,Dec.
**
**  5)  No range checking of arguments is carried out.
**
**  6)  In applications which involve many such calculations, rather
**      than calling the present routine it will be more efficient to
**      use inline code, having previously computed fixed terms such
**      as sine and cosine of latitude, and (for tracking a star)
**      sine and cosine of declination.
**
**  Defined in slamac.h:  D2PI
**
**  Last revision:   10 July 1994
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double sh, ch, sd, cd, sp, cp, x, y, z, r, a;

/* Useful trig functions */
   sh = sin ( ha );
   ch = cos ( ha );
   sd = sin ( dec );
   cd = cos ( dec );
   sp = sin ( phi );
   cp = cos ( phi );

/* Az,El as x,y,z */
   x = - ch * cd * sp + sd * cp;
   y = - sh * cd;
   z = ch * cd * cp + sd * sp;

/* To spherical */
   r = sqrt ( x * x + y * y );
   a = ( r == 0.0 ) ? 0.0 : atan2 ( y, x ) ;
   *az = ( a < 0.0 ) ? a + D2PI : a;
   *el = atan2 ( z, r );
}

double slaPa ( double ha, double dec, double phi )
/*+
**  - - - - - -
**   s l a P a
**  - - - - - -
**
**  HA, Dec to Parallactic Angle.
**
**  (double precision)
**
**  Given:
**     ha     d     hour angle in radians (geocentric apparent)
**     dec    d     declination in radians (geocentric apparent)
**     phi    d     observatory latitude in radians (geodetic)
**
**  The result is in the range -pi to +pi
**
**  Notes:
**
**  1)  The parallactic angle at a point in the sky is the position
**      angle of the vertical, i.e. the angle between the direction to
**      the pole and to the zenith.  In precise applications care must
**      be taken only to use geocentric apparent HA,Dec and to consider
**      separately the effects of atmospheric refraction and telescope
**      mount errors.
**
**  2)  At the pole a zero result is returned.
**
**  Last revision:   16 August 1994
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double cp, cqsz, sqsz;

   cp = cos ( phi );
   sqsz = cp * sin ( ha );
   cqsz = sin ( phi ) * cos ( dec) - cp * sin ( dec) * cos ( ha );
   return ( ( sqsz != 0.0 || cqsz != 0.0 ) ? atan2 ( sqsz, cqsz ) : 0.0 );
}

double slaGmsta ( double date, double ut )
/*
**  - - - - - - - - -
**   s l a G m s t a
**  - - - - - - - - -
**
**  Conversion from Universal Time to Greenwich mean sidereal time,
**  with rounding errors minimized.
**
**  (double precision)
**
**  Given:
*     date   double     UT1 date (MJD: integer part of JD-2400000.5))
**    ut     double     UT1 time (fraction of a day)
**
**  The result is the Greenwich Mean Sidereal Time (double precision,
**  radians, in the range 0 to 2pi).
**
**  There is no restriction on how the UT is apportioned between the
**  date and ut1 arguments.  Either of the two arguments could, for
**  example, be zero and the entire date+time supplied in the other.
**  However, the routine is designed to deliver maximum accuracy when
**  the date argument is a whole number and the ut argument lies in
**  the range 0 to 1, or vice versa.
**
**  The algorithm is based on the IAU 1982 expression (see page S15 of
**  the 1984 Astronomical Almanac).  This is always described as giving
**  the GMST at 0 hours UT1.  In fact, it gives the difference between
**  the GMST and the UT, the steady 4-minutes-per-day drawing-ahead of
**  ST with respect to UT.  When whole days are ignored, the expression
**  happens to equal the GMST at 0 hours UT1 each day.
**
**  In this routine, the entire UT1 (the sum of the two arguments date
**  and ut) is used directly as the argument for the standard formula.
**  The UT1 is then added, but omitting whole days to conserve accuracy.
**
**  See also the routine slaGmst, which accepts the UT1 as a single
**  argument.  Compared with slaGmst, the extra numerical precision
**  delivered by the present routine is unlikely to be important in
**  an absolute sense, but may be useful when critically comparing
**  algorithms and in applications where two sidereal times close
**  together are differenced.
**
**  Called:  slaDranrm
**
**  Defined in slamac.h:  DS2R, dmod
**
**  Last revision:   13 April 1998
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
{
   double d1, d2, t;

/* Julian centuries since J2000. */
   if ( date < ut ) {
      d1 = date;
      d2 = ut;
   } else {
      d1 = ut;
      d2 = date;
   }
   t = ( d1 + ( d2 - 51544.5 ) ) / 36525.0;

/* GMST at this UT1. */
   return slaDranrm ( DS2R * ( 24110.54841
                           + ( 8640184.812866
                           + ( 0.093104
                             - 6.2e-6 * t ) * t ) * t
                             + 86400.0 * ( dmod ( d1, 1.0 ) +
                                           dmod ( d2, 1.0 ) ) ) );
}

