#ifndef TYPES_DEFINED
#include "ephem_types.h"
#endif
#include <time.h>

double HourToRad(int h, int m, float s);
double DegToRad(int d, int m, float s);
double GetJulian(struct tm *now);
void PrintAngle(double angle);
void StarPos(double t, double ra0, double dec0, double mra, double mdec,
		            double pi, double rvel, double *ra, double *dec);
void PlanetPos(double tt, int target, double *ra, double *dec);
void SunPos(double tt, double *ra, double *dec);
void ReductionInit();
