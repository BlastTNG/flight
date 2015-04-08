/* 
 * motors.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Mar 31, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_MOTORS_H_
#define INCLUDE_MOTORS_H_


#define MIN_EL 19.5
#define MAX_EL 55

#define MAX_V_AZ 2.0 // was 2000 in gyro units
#define MAX_V_EL 0.5 // was 0.5

#define VPIV_FILTER_LEN 40
#define FPIV_FILTER_LEN 1000

#define EL_ACCEL 0.5

#define NO_DITH_INC 0
#define DITH_INC 1

/**********************************************/
/*  Motor Data Struct                         */
/*  - Stores encoder/velocity information     */
/*  from the motors                           */
/*  - Written to struct in ec_motors.c        */
/*  - Written to the frame in the main thread */
/*  USE A CIRCULAR BUFFER !!!                 */
///TODO: Add State/Desired State here
struct MotorDataStruct{
  double velocity;                  // in degrees per second
  int temp;                         // drive temperature in deg Celcius
  double current;                   // drive current read from controller
  double position;                  // Resolver position (if available)
  unsigned int status;              // drive status
  unsigned int fault_reg;           // drive fault register
  unsigned short int drive_info;    // motorinfo struct
  unsigned int err_count;           // count of serious serial errors
};
extern struct MotorDataStruct RWMotorData[3];
extern struct MotorDataStruct ElevMotorData[3];
extern struct MotorDataStruct PivotMotorData[3];

extern int motor_index;

void command_motors(void);
void UpdateAxesMode(void);
void WriteMot(void);

#endif /* INCLUDE_MOTORS_H_ */
