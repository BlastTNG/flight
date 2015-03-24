#include "angles.h"
#include <conversions.h>
#include <math.h>
#include <time.h>

/* 2pi */
#define D2PI 6.2831853071795864769252867665590057683943387987502



double angular_distance(double ra0, double dec0, double ra1, double dec1)
{
	return acos(sin(dec0)*sin(dec1) + cos(dec0)*cos(dec1)*cos(ra0-ra1));
}

double approximate_az_from_cross_el(double cross_el, double el)
{
    if (el >= from_degrees(0.0) && el < from_degrees(85.0)) {
        return cross_el / cos(el);
    }
    return cross_el;
}

double wrap_to(double angle, double max)
{
    angle = remainder(angle, max);
    if (angle < 0) angle += max;
    return angle;
}

int wrap_to_ints(int angle, int max)
{
    angle = remainder(angle, max);
    if (angle < 0) angle += max;
    return angle;
}

bool limit_value_to_ints(int* value, int min, int max)
{
    if (*value < min) {
        *value = min;
        return true;
    }
    if (*value > max) {
        *value = max;
        return true;
    }
    return false;
}

bool limit_value_to(double* value, double min, double max)
{
    if (*value < min) {
        *value = min;
        return true;
    }
    if (*value > max) {
        *value = max;
        return true;
    }
    return false;
}

double unwind_around(double reference, double angle)
{
    // Adjust angle to be +/- 180 of the reference.
    return (reference + remainder(angle - reference, 360.0));
}

// limit to 0 to 360.0
double normalize_angle_360(double m_angle)
{
    while (m_angle < 0.0) m_angle += 360.0;
    while (m_angle >= 360.0) m_angle -= 360.0;
    return m_angle;
}

double normalize_angle_180(double m_angle)
{
    while (m_angle < -180.0) m_angle += 360.0;
    while (m_angle >= 180.0) m_angle -= 360.0;
    return m_angle;
}

static void slaDh2e ( double az, double el, double phi, double *ha, double *dec )
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

static void slaDe2h ( double ha, double dec, double phi, double *az, double *el )
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

void equatorial_to_horizontal(double ra_hours, double dec_deg, time_t lst_s, double lat_deg, 
                              double* az_deg, double* el_deg)
{
    double az = 0.0;
    double el = 0.0;
    double ha = from_seconds(lst_s) - from_hours(ra_hours);
   
    slaDe2h (ha, from_degrees(dec_deg), from_degrees(lat_deg), &az, &el);
    *az_deg = to_degrees(az);
    *el_deg = to_degrees(el);
}

void horizontal_to_equatorial(double az_deg, double el_deg, time_t lst_s, double lat_deg, 
                              double* ra_hours, double* dec_deg)
{
    double ha = 0.0;
    double dec = 0.0;

    slaDh2e (from_degrees(az_deg), from_degrees(el_deg), from_degrees(lat_deg), &ha, &dec);
    *ra_hours = to_hours(from_seconds(lst_s) - ha);
    *ra_hours = wrap_to(*ra_hours, 24.0);
    *dec_deg = to_degrees(dec);
}

