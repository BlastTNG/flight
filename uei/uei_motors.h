/* 
 * uei_motors.h: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
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
 * Created on: Aug 12, 2014 by seth
 */

#ifndef UEI_MOTORS_H_
#define UEI_MOTORS_H_

#include <ecrt.h>

#define RW_ETHERCAT_ALIAS 1
#define EL_ETHERCAT_ALIAS 2
#define PV_ETHERCAT_ALIAS 3

#define COPLEY_ETHERCAT_VENDOR 0x000000ab
#define AEP_090_036_PRODCODE 0x00000380
#define BEL_090_020_PRODCODE 0x0 ///TODO: Fix this with the BEL product code when we have it

/*******************************************************************
 * This section encodes a number of CanBus/EtherCAT index/subindex values
 * for the Copley motor controller commands.  This are referenced from
 * http://www.copleycontrols.com/Motion/pdf/Parameter_Dictionary.pdf
 *
 * They are stored as index, subindex macros for use in registering
 * PDO/SDO calls.
 *
 * Each also optionally has a Write Mapped PDO index.  These
 * act for registering variable locations to PDOs.  Their indices cannot
 * overlap with otherwise known dictionary entries in the Copley document.
 */

#define ECAT_CURRENT_LOOP_CP 0x2380, 1 /* Proportional Gain UINT16 */
#define ECAT_CURRENT_LOOP_CP_PDO_WR 0x1b00

#define ECAT_CURRENT_LOOP_CI 0x2380, 2 /* Integral Gain UINT16 */
#define ECAT_CURRENT_LOOP_CI_PDO_WR 0x1b01

#define ECAT_CURRENT_LOOP_VAL 0x2340, 0 /* Actual current in 0.01A INT16 */
#define ECAT_CURRENT_LOOP_VAL_PDO_WR 0x1b02

#define ECAT_DRIVE_STATE 0x2300, 0 /* Desired state of the drive UINT16 */
#define ECAT_DRIVE_STATE_PDO_WR 0x1b03
#  define ECAT_STATE_DISABLED 0
#  define ECAT_STATE_PROG_CURRENT 1
#  define ECAT_STATE_PROG_VELOCITY 11

#define ECAT_MOTOR_POSITION 0x2240, 0 /* Encoder position in counts INT32 */
#define ECAT_DRIVE_TEMP 0x2202, 0 /* A/D Reading in degrees C INT16 */

#define ECAT_DRIVE_STATUS 0x1002, 0 /* Drive status bitmap UINT32 */
#  define ECAT_STATUS_SHORTCIRCUIT 			(1<<0)
#  define ECAT_STATUS_DRIVE_OVERTEMP 		(1<<1)
#  define ECAT_STATUS_OVERVOLTAGE 			(1<<2)
#  define ECAT_STATUS_UNDERVOLTAGE 			(1<<3)
#  define ECAT_STATUS_TEMP_SENS_ACTIVE 		(1<<4)
#  define ECAT_STATUS_ENCODER_FEEDBACK_ERR 	(1<<5)
#  define ECAT_STATUS_PHASING_ERROR 		(1<<6)
#  define ECAT_STATUS_CURRENT_LIMITED 		(1<<7)
#  define ECAT_STATUS_VOLTAGE_LIMITED 		(1<<8)
#  define ECAT_STATUS_POS_LIMIT_SW 			(1<<9)
#  define ECAT_STATUS_NEG_LIMIT_SW 			(1<<10)
#  define ECAT_STATUS_ENABLE_NOT_ACTIVE		(1<<11)
#  define ECAT_STATUS_SW_DISABLE			(1<<12)
#  define ECAT_STATUS_STOPPING				(1<<13)
#  define ECAT_STATUS_BRAKE_ON				(1<<14)
#  define ECAT_STATUS_PWM_DISABLED			(1<<15)
#  define ECAT_STATUS_POS_SW_LIMIT			(1<<16)
#  define ECAT_STATUS_NEG_SW_LIMIT			(1<<17)
#  define ECAT_STATUS_TRACKING_ERROR		(1<<18)
#  define ECAT_STATUS_TRACKING_WARNING		(1<<19)
#  define ECAT_STATUS_DRIVE_RESET			(1<<20)
#  define ECAT_STATUS_POS_WRAPPED			(1<<21)
#  define ECAT_STATUS_DRIVE_FAULT			(1<<22)
#  define ECAT_STATUS_VEL_LIMIT				(1<<23)
#  define ECAT_STATUS_ACCEL_LIMIT			(1<<24)
#  define ECAT_STATUS_TRACK_WINDOW			(1<<25)
#  define ECAT_STATUS_HOME_SWITCH_ACTIVE	(1<<26)
#  define ECAT_STATUS_IN_MOTION				(1<<27)
#  define ECAT_STATUS_VEL_WINDOW			(1<<28)
#  define ECAT_STATUS_PHASE_UNINIT			(1<<29)
#  define ECAT_STATUS_CMD_FAULT				(1<<30)

#define ECAT_LATCHED_DRIVE_STATUS 0x2181, 0 /* Drive status bitmap UINT32 */
#define ECAT_LATCHED_DRIVE_STATUS_PDO_WR 0x1b04

#define ECAT_LATCHED_DRIVE_FAULT 0x2183, 0 /* Drive faults bitmap UINT32 */
#define ECAT_LATCHED_DRIVE_FAULT_PDO_WR 0x1b05
#  define ECAT_FAULT_DATA_CRC				(1<<0)
#  define ECAT_FAULT_INT_ERR				(1<<1)
#  define ECAT_FAULT_SHORT_CIRCUIT			(1<<2)
#  define ECAT_FAULT_DRIVE_OVER_TEMP		(1<<3)
#  define ECAT_FAULT_MOTOR_OVER_TEMP		(1<<4)
#  define ECAT_FAULT_OVER_VOLT				(1<<5)
#  define ECAT_FAULT_UNDER_VOLT				(1<<6)
#  define ECAT_FAULT_FEEDBACK_FAULT			(1<<7)
#  define ECAT_FAULT_PHASING_ERR			(1<<8)
#  define ECAT_FAULT_TRACKING_ERR			(1<<9)
#  define ECAT_FAULT_CURRENT_LIMIT			(1<<10)
#  define ECAT_FAULT_FPGA_ERR1				(1<<11)
#  define ECAT_FAULT_CMD_LOST				(1<<12)
#  define ECAT_FAULT_FPGA_ERR2				(1<<13)
#  define ECAT_FAULT_SAFETY_CIRCUIT_FAULT	(1<<14)
#  define ECAT_FAULT_CANT_CONTROL_CURRENT	(1<<15)
#  define ECAT_FAULT_WIRING_DISCONNECT		(1<<16)

#define ECAT_CURRENT_LOOP_OFFSET 0x60F6, 3 /* Added to commanded current to compensate for drift. 0.01A INT16 */
#define ECAT_CURRENT_LOOP_OFFSET_PDO_WR 0x1b06
#define ECAT_MOTOR_ENC_WRAP_POS 0x2220, 0 /* Position value at which the motor encoder position will wrap. INT32 */
#define ECAT_MOTOR_ENC_WRAP_POS_PDO_WR 0x1b07
#define ECAT_MOTOR_TEMP_VOLTAGE 0x2209, 0 /* Voltage at the input to motor T sensor INT16 */


#define ECAT_PDO_CURRENT_LOOP_STATUS_RD 0x1a00
#define ECAT_PDO_DRIVE_INFO_RD 0x1a01
#define ECAT_PDO_LATCHED_DRIVE_FLAGS_RD 0x1a02
#define ECAT_PDO_MOTOR_POS_RD 0x1a03
#define ECAT_PDO_TEMPS_RD 0x1a04


int uei_ethercat_initialize (void);

void motor_cmd_routine(void *m_arg);

void uei_ethercat_cleanup(void);
int16_t ethercat_get_current(void);

#endif /* UEI_MOTORS_H_ */
