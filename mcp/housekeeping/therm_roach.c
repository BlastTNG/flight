/* 
* thermistor calculator for roach's
* This software is Copyright adrian sinclair 2018
* ...Actually this software is copyleft adrian sinclair 2018 
*/
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "therm_roach.h"

/**
Calculates the temperature of roach thermistor given a voltage
ref: https://www.murata.com/~/media/webrenewal/support/library/catalog/
    products/thermistor/ntc/r44e.ashx
*/

float temp(float voltage)
{
    float T = -99.0;
    // convert voltage to resistance
    if ((voltage < 5.0) & (voltage > 0.0)) {
      float R_0 = 10000.0; // reference resistor in Ohms
      float R_T = R_0/(5.0/voltage-1.0);
      T = 1.0/(log(fabs(R_T/R_0))/3374.5+1.0/298.0)-273.0; // calibration eq.
    } else {
      T = -99.0;
    }
    return T;
}
