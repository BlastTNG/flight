#ifndef ANGLES_H
#define ANGLES_H

#include <time.h>
#include <stdbool.h>

double angular_distance(double ra0, double dec0, double ra1, double dec1);
double approximate_az_from_cross_el(double cross_el, double el);
double wrap_to(double angle, double max);
int wrap_to_ints(int angle, int max);
bool limit_value_to(double* value, double min, double max);
bool limit_value_to_ints(int* value, int min, int max);
double unwind_around(double reference, double angle);
void equatorial_to_horizontal(double ra_hours, double dec_deg, time_t lst_s, double lat_deg, 
                              double* az_deg, double* el_deg);
void horizontal_to_equatorial(double az_deg, double el_deg, time_t lst_s, double lat_deg, 
                              double* ra_hours, double* dec_deg);

double normalize_angle_360(double m_angle);
double normalize_angle_180(double m_angle);

#endif

