/*
        motorcmd.h: Send commands over the serial port to the stepper motor
        controller. 
*/

#include <windows.h>
#include <stdio.h>
#include "serial.h"

#ifndef MOTORCMD_H
#define MOTORCMD_H

#define MOTOR_TIMEOUT 1000     // timeout for motor controller in ms

int motorcmd( LPCTSTR Device, int motornum, char *cmdstring );
int motorabort( LPCTSTR Device, int motornum );

#endif
