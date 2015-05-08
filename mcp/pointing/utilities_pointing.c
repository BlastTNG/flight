#include <time_lst.h>
#include <time_julian.h>
#include <time_nutation.h>

#include "utilities_pointing.h"

#include <math.h>
#include <time.h>


/**
 * Calculates the RA and Dec of the sun, given the current UTC
 * @param m_utc Seconds since Unix epoch in GMT
 * @param m_ra Right Ascension of the sun in radians
 * @param m_dec Declination of the sun in radians
 */
void calc_sun_position (time_t m_utc, double *m_ra, double *m_dec)
{
    double julian_days;
    double ecliptic_lon;
    double ecliptic_obliquity;
    struct julian_date julian;


    unix_to_julian_date(m_utc, &julian);
    julian_days = julian.epoch - J2000_EPOCH + julian.mjd;

    // Calculate ecliptic coordinates
    {
        double mean_longitude;
        double mean_anomaly;
        double omega;
        double sin_omega;
        double cos_omega;

        omega = 2.1429 - 0.0010394594 * julian_days;
        sincos(omega, &sin_omega, &cos_omega);
        mean_longitude = 4.8950630 + 0.017202791698 * julian_days; // Radians
        mean_anomaly = 6.2400600 + 0.0172019699 * julian_days;
        ecliptic_lon = mean_longitude +
                0.03341607 * sin(mean_anomaly) +
                0.00034894 * sin(2 * mean_anomaly) -
                0.0001134 - 0.0000203 * sin_omega;
        ecliptic_obliquity = 0.4090928 - 6.2140e-9 * julian_days + 0.0000396 * cos_omega;
    }

    // Calculate celestial coordinates ( right ascension and declination ) in radians
    {
        double sin_el;
        double cos_el;
        double sin_eo;
        double cos_eo;
        double dY;

        sincos(ecliptic_lon, &sin_el, &cos_el);
        sincos(ecliptic_obliquity, &sin_eo, &cos_eo);

        dY = cos_eo * sin_el;
        *m_ra = atan2(dY, cos_el);
        if (*m_ra < 0.0)
            *m_ra += (2.0 * M_PI);
        *m_dec = asin(sin_eo * sin_el);
    }

}


