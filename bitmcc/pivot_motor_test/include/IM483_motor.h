/* -----------------------------------------------------------------------
 * ------------------- IM483-34P1 Stepper Motor Driver -------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver created to run the the stepper motor using the
 * DMM-32DX-AT Analog/Digital I/O board. This driver is a supplement to
 * the DSC Universal Driver to make it more usable for the IM483 stepper
 * motor.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: July 17, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */

#ifndef IM483_MOTOR_H_
#define IM483_MOTOR_H_

#define PIVOT_DEG_PER_STP  0.018    // 100:1 gear reducer on 1.8 deg/stp motor

// define some function prototypes
int pulse_IM483_steps(DSCB , unsigned int , unsigned int );
int pulse_IM483(DSCB , unsigned int );
int set_IM483_resolution(DSCB, float );

#endif /* IM483_MOTOR_H_ */
