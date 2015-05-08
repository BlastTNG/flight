/**
 * @file time_lst.c
 *
 * @date Aug 5, 2011
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include <math.h>
#include <time.h>

#include <conversions.h>
#include <utilities_pointing.h>

#include <time_nutation.h>
#include <time_lst.h>

#include "blast.h"

/**
 * Reference : Division I Working Group on “Precession and the Ecliptic”
 * 				2002-2005, Transactions of the International Astronomical Union
 *             <DOI: 10.1017/S1743921306004364>
 */

typedef struct {
	double si;
	double ci;
	short l;
	short lp;
	short f;
	short d;
	short om;
	short l_ve;
	short l_ea;
	short pre;
} eoe_terms_t;

typedef struct
{
	double jd;
	double leap_sec;
	double mjd_off;
	double mjd_scale;
} tt_conv_t;

typedef struct
{
	double jd;
	double tt_off;
} tt_pred_t;

static eoe_terms_t eoe_terms[] = {
	{2640.96e-6, -0.39e-6,  0,  0,  0,  0,  1,  0,  0,  0},
	{  63.52e-6, -0.02e-6,  0,  0,  0,  0,  2,  0,  0,  0},
	{  11.75e-6,  0.01e-6,  0,  0,  2, -2,  3,  0,  0,  0},
	{  11.21e-6,  0.01e-6,  0,  0,  2, -2,  1,  0,  0,  0},
	{  -4.55e-6,  0.00e-6,  0,  0,  2, -2,  2,  0,  0,  0},
	{   2.02e-6,  0.00e-6,  0,  0,  2,  0,  3,  0,  0,  0},
	{   1.98e-6,  0.00e-6,  0,  0,  2,  0,  1,  0,  0,  0},
	{  -1.72e-6,  0.00e-6,  0,  0,  0,  0,  3,  0,  0,  0},
	{  -1.41e-6, -0.01e-6,  0,  1,  0,  0,  1,  0,  0,  0},
	{  -1.26e-6, -0.01e-6,  0,  1,  0,  0, -1,  0,  0,  0},
	{  -0.63e-6,  0.00e-6,  1,  0,  0,  0, -1,  0,  0,  0},
	{  -0.63e-6,  0.00e-6,  1,  0,  0,  0,  1,  0,  0,  0},
	{   0.46e-6,  0.00e-6,  0,  1,  2, -2,  3,  0,  0,  0},
	{   0.45e-6,  0.00e-6,  0,  1,  2, -2,  1,  0,  0,  0},
	{   0.36e-6,  0.00e-6,  0,  0,  4, -4,  4,  0,  0,  0},
	{  -0.24e-6, -0.12e-6,  0,  0,  1, -1,  1, -8, 12,  0},
	{   0.32e-6,  0.00e-6,  0,  0,  2,  0,  0,  0,  0,  0},
	{   0.28e-6,  0.00e-6,  0,  0,  2,  0,  2,  0,  0,  0},
	{   0.27e-6,  0.00e-6,  1,  0,  2,  0,  3,  0,  0,  0},
	{   0.26e-6,  0.00e-6,  1,  0,  2,  0,  1,  0,  0,  0},
	{  -0.21e-6,  0.00e-6,  0,  0,  2, -2,  0,  0,  0,  0},
	{   0.19e-6,  0.00e-6,  0,  1, -2,  2, -3,  0,  0,  0},
	{   0.18e-6,  0.00e-6,  0,  1, -2,  2, -1,  0,  0,  0},
	{  -0.10e-6,  0.05e-6,  0,  0,  0,  0,  0,  8,-13, -1},
	{   0.15e-6,  0.00e-6,  0,  0,  0,  2,  0,  0,  0,  0},
	{  -0.14e-6,  0.00e-6,  2,  0, -2,  0, -1,  0,  0,  0},
	{   0.14e-6,  0.00e-6,  1,  0,  0, -2,  1,  0,  0,  0},
	{  -0.14e-6,  0.00e-6,  0,  1,  2, -2,  2,  0,  0,  0},
	{   0.14e-6,  0.00e-6,  1,  0,  0, -2, -1,  0,  0,  0},
	{   0.13e-6,  0.00e-6,  0,  0,  4, -2,  4,  0,  0,  0},
	{  -0.11e-6,  0.00e-6,  0,  0,  2, -2,  4,  0,  0,  0},
	{   0.11e-6,  0.00e-6,  1,  0, -2,  0, -3,  0,  0,  0},
	{   0.11e-6,  0.00e-6,  1,  0, -2,  0, -1,  0,  0,  0}
};

tt_conv_t tt_hist[] =
	{
		{ 2437300.5, 1.4228180, 37300., 0.001296 },
		{ 2437512.5, 1.3728180, 37300., 0.001296 },
		{ 2437665.5, 1.8458580, 37665., 0.0011232 },
		{ 2438334.5, 1.9458580, 37665., 0.0011232 },
		{ 2438395.5, 3.2401300, 38761., 0.001296 },
		{ 2438486.5, 3.3401300, 38761., 0.001296 },
		{ 2438639.5, 3.4401300, 38761., 0.001296 },
		{ 2438761.5, 3.5401300, 38761., 0.001296 },
		{ 2438820.5, 3.6401300, 38761., 0.001296 },
		{ 2438942.5, 3.7401300, 38761., 0.001296 },
		{ 2439004.5, 3.8401300, 38761., 0.001296 },
		{ 2439126.5, 4.3131700, 39126., 0.002592 },
		{ 2439887.5, 4.2131700, 39126., 0.002592 },
		{ 2441317.5, 10.0, 41317., 0.0 },
		{ 2441499.5, 11.0, 41317., 0.0 },
		{ 2441683.5, 12.0, 41317., 0.0 },
		{ 2442048.5, 13.0, 41317., 0.0 },
		{ 2442413.5, 14.0, 41317., 0.0 },
		{ 2442778.5, 15.0, 41317., 0.0 },
		{ 2443144.5, 16.0, 41317., 0.0 },
		{ 2443509.5, 17.0, 41317., 0.0 },
		{ 2443874.5, 18.0, 41317., 0.0 },
		{ 2444239.5, 19.0, 41317., 0.0 },
		{ 2444786.5, 20.0, 41317., 0.0 },
		{ 2445151.5, 21.0, 41317., 0.0 },
		{ 2445516.5, 22.0, 41317., 0.0 },
		{ 2446247.5, 23.0, 41317., 0.0 },
		{ 2447161.5, 24.0, 41317., 0.0 },
		{ 2447892.5, 25.0, 41317., 0.0 },
		{ 2448257.5, 26.0, 41317., 0.0 },
		{ 2448804.5, 27.0, 41317., 0.0 },
		{ 2449169.5, 28.0, 41317., 0.0 },
		{ 2449534.5, 29.0, 41317., 0.0 },
		{ 2450083.5, 30.0, 41317., 0.0 },
		{ 2450630.5, 31.0, 41317., 0.0 },
		{ 2451179.5, 32.0, 41317., 0.0 },
		{ 2453736.5, 33.0, 41317., 0.0 },
		{ 2454832.5, 34.0, 41317., 0.0 },
		{ 2456109.5, 35.0, 41317., 0.0 },	// 2012 Jul 1
		{ 2457204.5, 36.0, 41317., 0.0 }    // 2015 Jul 1
	};

tt_pred_t tt_predictions[] =
	{
		{ 2456110.5, 67.0 },
		{ 2456201.5, 67.1 },
		{ 2456293.5, 67.3 },
		{ 2456384.5, 67.4 },
		{ 2456476.5, 67.4 },
		{ 2456567.5, 67.6 },
		{ 2456659.5, 67.7 },
		{ 2456750.5, 67.8 },
		{ 2456842.5, 68. },
		{ 2457299.5, 69. },
		{ 2458027.5, 70. },
		{ 2458757.5, 71. }
	};

/**
 * Computes the approximate difference between UT1 and UTC.  This is on the order
 * of 0.1 second, so if that is unimportant to you, you should skip this and just
 * use UTC. See ftp://maia.usno.navy.mil/ser7/iersexp.sup
 * @param m_year Year of UTC
 * @param m_month Month of UTC
 * @param m_day Day of UTC
 * @param m_delta_utc Pointer to receive UT1 - UTC
 */

void time_UT1_UTC(int m_year, int m_month, int m_day, double *m_delta_utc)
{
	struct julian_date utc_jd;
	double t;
	double ut2_ut1;

	calendar_to_julian_date(m_year, m_month, m_day, &utc_jd);

	t = 2000.0 + (utc_jd.mjd - 51544.03) / 365.2422;
	t *= 2.0 * M_PI;
	ut2_ut1 = 0.022 * sin(t)
				- 0.012 * cos(t)
				- 0.006 * sin(2 * t)
				+ 0.007 * cos(2 * t);

	*m_delta_utc = -0.3195 - 0.00070 * (utc_jd.mjd - 55785.0) - ut2_ut1;
}

/**
 * Computes (or looks up) the difference TT-UTC. Will always return the
 * most accurate available information depending on the date
 * @param m_year UTC Year
 * @param m_month UTC Month
 * @param m_delta_utc TT-UTC (fractional seconds)
 */
void time_TT_UTC (int m_year, int m_month, double *m_delta_utc)
{
	struct julian_date utc_jd;
	double tjd;
	size_t i;
	size_t num_hist = sizeof(tt_hist)/sizeof(tt_hist[0]);
	size_t num_pred = sizeof(tt_predictions)/ sizeof(tt_predictions[0]);

	calendar_to_julian_date(m_year, m_month, 1, &utc_jd);
	tjd = utc_jd.epoch + utc_jd.mjd;

	/**
	 * Outside of our historical and semi-accurate prediction range,
	 * we must use a polynomial approximation for delta-t
	 */
	if (tjd < tt_hist[0].jd
		|| tjd > tt_predictions[num_pred - 1].jd)
	{
		time_est_deltat(m_year, m_month, m_delta_utc);
		return;
	}

	/**
	 * If we are older than our first prediction, use the historical
	 * record
	 */
	if (tjd < tt_predictions[0].jd)
	{
		for (i = 1; i < num_hist; i++)
		{
			if (tjd < tt_hist[i].jd) break;
		}
		*m_delta_utc = TAI_TT_OFFSET + tt_hist[i - 1].leap_sec + (utc_jd.mjd - tt_hist[i-1].mjd_off) * tt_hist[i-1].mjd_scale;
	}
	else
	{
		for (i = 1; i < num_pred; i++)
		{
			if (tjd < tt_predictions[i].jd) break;
		}
		*m_delta_utc = tt_predictions[i - 1].tt_off;
	}
}

/**
 * Estimates Delta-T = TT - UT1 (or UTC at this accuracy) using a polynomial.
 * See http://eclipse.gsfc.nasa.gov/SEcat5/deltatpoly.html
 * @param m_year Year of UTC
 * @param m_month Month of UTC
 * @param m_deltat Pointer to receive TT - UT1
 * @return -1 on failure, 0 on success
 */
int time_est_deltat(int m_year, int m_month, double *m_deltat)
{
	double x;
	double yr_off;

	if (m_year < -1999 || m_year > 3000 || m_month < 1 || m_month > 12)
		return -1;

	yr_off = m_year + (m_month - 0.5) / 12;

	switch (m_year)
	{
		case -1999 ... -501:
			x = (m_year - 1820) / 100;
			*m_deltat = -20 + 32 * x * x;
			break;
		case -500 ... 499:
			x = yr_off / 100;
			*m_deltat = 10583.6
						+ (-1014.41
						+ (33.78311
						+ (-5.952053
						+ (-0.1798452
						+ (0.022174192
						+ 0.0090316521 * x) * x) * x) * x) * x) * x;
			break;
		case 500 ... 1599:
			x = (yr_off - 1000) / 100;
			*m_deltat = 1574.2
						+ (-556.01
						+ (71.23472
						+ (0.319781
						+ (-0.8503463
						+ (-0.005050998
						+ 0.0083572073 * x) * x) * x) * x) * x) * x;
			break;
		case 1600 ... 1699:

			x = yr_off - 1600;
			*m_deltat = 120
						+ (-0.9808
						+ (-0.01532
						+ x / 7129.0) * x) * x;
			break;
		case 1700 ... 1799:

			x = yr_off - 1700;
			*m_deltat = 8.83
						+ (0.1603
						+ (-0.0059285
						+ (0.00013336
						- x / 1174000.0) * x) * x) * x;
			break;
		case 1800 ... 1859:

			x = yr_off - 1800;
			*m_deltat = 13.72
						+ (-0.332447
						+ (0.0068612
						+ (0.0041116
						+ (-0.00037436
						+ (0.0000121272
						+ (-0.0000001699
						+ 0.000000000875 * x) * x) * x) * x) * x) * x) * x;
			break;
		case 1860 ... 1899:
			x = yr_off - 1860;
			*m_deltat = 7.62
						+ (0.5737
						+ (-0.251754
						+ (0.01680668
						+ (-0.0004473624
						+ x / 233174.0) * x) * x) * x) * x;
			break;
		case 1900 ... 1919:
			x = yr_off - 1900;
			*m_deltat = -2.79
						+ (1.494119
						+ (-0.0598939
						+ (0.0061966
						- 0.000197 * x) * x) * x) * x;
			break;
		case 1920 ... 1940:
			x = yr_off - 1920;
			*m_deltat = 21.20
						+ (0.84493
						+ (-0.076100
						+ 0.0020936 * x) * x) * x;
			break;
		case 1941 ... 1960:
			x = yr_off - 1950;
			*m_deltat = 29.07
						+ (0.407
						+ (-1.0 / 233.0
						+ x / 2547.0) * x) * x;
			break;
		case 1961 ... 1985:
			x = yr_off - 1975;
			*m_deltat = 45.45 + (1.067 + (-1.0 / 260.0 - x / 718.0) * x) * x;
			break;
		case 1986 ... 2004:
			x = yr_off - 2000;
			*m_deltat = 63.86
						+ (0.3345
						+ (-0.060374
						+ (0.0017275
						+ (0.000651814
						+ 0.00002373599 * x) * x) * x) * x) * x;
			break;

		case 2005 ... 2049:
			x = yr_off - 2000;
			*m_deltat = 62.92 + (0.32217 + 0.005589 * x) * x;
			break;
		case 2050 ... 2149:
			x = (yr_off - 1820) / 100;
			*m_deltat = -20 + 32 * x * x - 0.5628 * (2150 - yr_off);
			break;
		default:
			x = (m_year - 1820) / 100;
			*m_deltat = -20 + 32 * x * x;
			break;
	}
	if (m_year < 1955 || m_year > 2005)
		*m_deltat += -0.000012932 * (yr_off - 1955) * (yr_off - 1955);

	return 0;
}
/**
 * Returns a correction due to the equinox model of IAU 2000a
 * @param m_tdb Baryocentric, Dynamic Time
 * @return Correction term in radians
 */
static double equinox_correction(struct julian_date *m_tdb)
{
	size_t i;
	double jct, obl, d_lon, d_obl, l, lp, f, d, om, l_ve, l_ea, pre, phi;
	static double eqe = 0.0;
	static struct julian_date last_tdb = {NAN, NAN};

	if (last_tdb.epoch == m_tdb->epoch && last_tdb.mjd == m_tdb->mjd)
		return eqe;

	last_tdb.epoch = m_tdb->epoch;
	last_tdb.mjd = m_tdb->mjd;

	jct = JULIAN_CENTURIES(m_tdb->epoch, m_tdb->mjd);
	l = iau2000a_fundamental_arguments(ARG_ANOMALY_MOON, jct);
	lp = iau2000a_fundamental_arguments(ARG_ANOMALY_SUN, jct);
	f = iau2000a_fundamental_arguments(ARG_LATITUDE_MOON, jct);
	d = iau2000a_fundamental_arguments(ARG_ELONGATION_MOON, jct);
	om = iau2000a_fundamental_arguments(ARG_LONGITUDE_NODE, jct);
	l_ve = iau2000a_fundamental_arguments(ARG_LONGITUDE_VENUS, jct);
	l_ea = iau2000a_fundamental_arguments(ARG_LONGITUDE_EARTH, jct);
	pre = iau2000a_fundamental_arguments(ARG_PRECESSION, jct);

	/// Sum equinox equation terms
	eqe = 0;
	for (i = 0; i < sizeof(eoe_terms) / sizeof(eoe_terms[0]); i++)
	{
		phi = 	  eoe_terms[i].l * l
				+ eoe_terms[i].lp * lp
				+ eoe_terms[i].f * f
				+ eoe_terms[i].d * d
				+ eoe_terms[i].om * om
				+ eoe_terms[i].l_ve * l_ve
				+ eoe_terms[i].l_ea * l_ea
				+ eoe_terms[i].pre * pre;

		eqe += eoe_terms[i].si * sin(phi)
			+ eoe_terms[i].ci * cos(phi) - 0.87e-6 * jct * sin(om);
	}
	eqe *= ARCSEC2RAD;

	/// Add nutation and obliquity corrections
	iau2000a_nutation(m_tdb, &d_lon, &d_obl);
	obl = iau2000a_mean_obliquity(m_tdb);

	eqe = eqe + d_lon * cos(obl + d_obl);

	return eqe;
}

/**
 * Computes the local mean sidereal time following
 * @param m_ut1 Universal Time (N.B. Using UTC here is accurate +/- 1s)
 * @param m_tdb Baryocentric Dynamical Time (N.B. Using TT here is acceptable for all but most demanding apps)
 * @param m_longitude Longitude in radians, East is positive
 * @return Local mean Sidereal Time in radians
 */
double mean_sidereal_time(struct julian_date *m_ut1, struct julian_date *m_tdb, double m_longitude)
{
	double rot_angle;
	double gmst;
	double dut = (m_ut1->epoch - J2000_EPOCH) + m_ut1->mjd;
	double jct = JULIAN_CENTURIES(m_tdb->epoch, m_tdb->mjd);

	/// Rotation of the Earth
	rot_angle = (0.7790572732640 + 0.00273781191135448 * dut +
		fmod(m_ut1->epoch, 1.0) + fmod(m_ut1->mjd, 1.0)) * (2.0 * M_PI);

	/// Greenwich Mean Sidereal Time
	gmst = rot_angle +
		(0.014506 +
		(4612.156534 +
		(1.3915817 +
		(-0.00000044 +
		(-0.000029956 -
		0.0000000368 * jct) * jct) * jct) * jct) * jct) * ARCSEC2RAD;
	gmst += m_longitude;

	gmst = range_fast(&gmst, 2.0 * M_PI);

	return gmst;
}

/**
 * Computes the Julian Day number from a UNIX timestamp (starts at 1970).
 * @param m_unixtime time_t unix timestamp
 * @param m_jd Pointer to the calculated Julian Day
 * @return 0 on success, -1 on failure
 */
int unix_to_julian_date(time_t m_unixtime, struct julian_date *m_jd)
{
	struct tm conv_time;
	double day_sec = 24.0 * 60.0 * 60.0;

	gmtime_r(&m_unixtime, &conv_time);
	calendar_to_julian_date(conv_time.tm_year + 1900, conv_time.tm_mon + 1, conv_time.tm_mday, m_jd);

	m_jd->mjd += (60.0 * ( (double) (60 * conv_time.tm_hour + conv_time.tm_min)) + conv_time.tm_sec) / day_sec;

	return 0;
}

/**
 * Calculates the local apparent sidereal time (mean sidereal time + nutation)
 * @param m_ut Uniform time
 * @param m_tdb Baryocentric, Dynamical time
 * @param m_longitude Longitude of observation in radians
 * @return Local Apparent Sidereal Time in radians
 */
static double time_lst(struct julian_date *m_ut, struct julian_date *m_tdb, double m_longitude)
{
	double ast;

	ast = mean_sidereal_time(m_ut, m_tdb, m_longitude) + equinox_correction(m_tdb);

	return range_fast(&ast, (2.0 * M_PI));
}

/**
 * Calculates the local apparent sidereal time starting from a UNIX timestamp (accuracy 1s)
 *
 * Uses historical data for leap seconds up to 2012 Jul 01 and predictions past this point.
 * @param m_time UNIX time
 * @param m_longitude Longitude of observation in radians
 * @return LST in radians
 */
double time_lst_unix(time_t m_time, double m_longitude)
{
	struct julian_date utc;
	struct julian_date tt;
	double tt_utc;
	struct tm tm_time;
	double day_sec = 24.0 * 60.0 * 60.0;

	unix_to_julian_date(m_time, &utc);
	gmtime_r(&m_time, &tm_time);
	time_TT_UTC(tm_time.tm_year + 1900, tm_time.tm_mon + 1, &tt_utc);

	tt.epoch = utc.epoch;
	tt.mjd = utc.mjd + tt_utc / day_sec;

	return time_lst(&utc, &tt, m_longitude);

}
