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

#include <stdint.h>

#define MIN_EL 19.5
#define MAX_EL 55

#define MAX_V_AZ 2.0 // was 2000 in gyro units
#define MAX_V_EL 0.5 // was 0.5

#define MAX_RW_CURRENT 2000 // 20 Amps in 0.01A units
#define MIN_RW_CURRENT (-2000) // 20 Amps in 0.01A units

#define MAX_EL_CURRENT 2000 // 20 Amps in 0.01A units
#define MIN_EL_CURRENT (-2000) // 20 Amps in 0.01A units

#define MAX_PIV_CURRENT 3000 // 30 Amps in 0.01A units
#define MIN_PIV_CURRENT (-3000) // 30 Amps in 0.01A units

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
// TODO(seth): Add State/Desired State here
typedef struct
{
    int32_t velocity;             // in 0.1 counts per second
    int16_t temp;                 // drive temperature in degrees Celsius
    double current;               // drive current read from controller
    int32_t position;             // Position used for calculations
    int32_t motor_position;       // Motor position
    uint32_t load_state;          // BiSS state bits for load encoder on El
    uint32_t status;              // drive status
    uint32_t fault_reg;           // drive fault register
    uint16_t drive_info;          // motorinfo struct
    uint16_t state;               // commanded state
    uint16_t net_status;          // Network status
    uint32_t err_count;           // count of serious serial errors
} motor_data_t;

extern motor_data_t RWMotorData[3];
extern motor_data_t ElevMotorData[3];
extern motor_data_t PivotMotorData[3];

extern int motor_index;

void command_motors(void);
void update_axes_mode(void);
void write_motor_channels_5hz(void);
void write_motor_channels_200hz(void);
void store_axes_mode_data(void);

#endif /* INCLUDE_MOTORS_H_ */
