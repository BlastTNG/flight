// Routines for reading ISC temp./pressure sensors, also control ISC heater

#ifndef __readTemp_H
#define __readTemp_H
#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <dscud.h>    

// Load in the settings file, set stuff up
// return 1 for success, 0 for failure
int tempSetup( int in_tempControl, unsigned int in_tempsleeptime, 
	       float in_tempSetLimit, float in_tempOffset, 
	       float in_temppressuregain, float in_temppressureoffset );

// Read / log sensors, set relay on the heater as necessary
void tempDoStuff( double *temp1, double *temp2, double *temp3, double *temp4, 
		  double *pressure, int *heaterOn );

// Turn off the heaters on shutdown
void tempShutdown( void );

#endif
