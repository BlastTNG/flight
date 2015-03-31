/* 
 * ec_motors.h: 
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
 * Created on: Mar 27, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_EC_MOTORS_H_
#define INCLUDE_EC_MOTORS_H_

#include <stdint.h>

/**
 * N.B. Here, RX/TX are from the controller's perspective, so RX is
 * received by the controller and TX is transmitted by the controller
 */
#define ECAT_RXPDO_ASSIGNMENT 0x1c12
#define ECAT_TXPDO_ASSIGNMENT 0x1c13

#define ECAT_TXPDO_MAPPING 0x1a00
#define ECAT_RXPDO_MAPPING 0x1600

#define ECAT_DC_CYCLE_NS 500000 /* Distributed Clock Cycle in nanoseconds */

typedef union {
    uint32_t val;
    struct {
        uint8_t size;
        uint8_t subindex;
        uint16_t index;
    };
} pdo_mapping_t;

typedef enum {
    ECAT_MOTOR_COLD,
    ECAT_MOTOR_INIT,
    ECAT_MOTOR_FOUND_PARTIAL,
    ECAT_MOTOR_FOUND,
    ECAT_MOTOR_RUNNING_PARTIAL,
    ECAT_MOTOR_RUNNING
} ec_motor_state_t;


#define COPLEY_ETHERCAT_VENDOR 0x000000ab
#define AEP_090_036_PRODCODE 0x00000380
#define BEL_090_030_PRODCODE 0x00001110

/*******************************************************************
 * This section encodes a number of CanBus/EtherCAT index/subindex values
 * for the Copley motor controller commands.  This are referenced from
 * http://www.copleycontrols.com/Motion/pdf/Parameter_Dictionary.pdf
 *
 * They are stored as index, subindex macros for use in registering
 * PDO/SDO calls.
 *
 */

#define ECAT_CURRENT_LOOP_CP 0x2380, 1 /* Proportional Gain UINT16 */
#define ECAT_CURRENT_LOOP_CI 0x2380, 2 /* Integral Gain UINT16 */
#define ECAT_CURRENT_LOOP_VAL 0x2340, 0 /* Actual current in 0.01A INT16 */
#define ECAT_DRIVE_STATE 0x2300, 0 /* Desired state of the drive UINT16 */
#  define ECAT_DRIVE_STATE_DISABLED 0
#  define ECAT_DRIVE_STATE_PROG_CURRENT 1
#  define ECAT_DRIVE_STATE_PROG_VELOCITY 11

#define ECAT_MOTOR_POSITION 0x2240, 0 /* Encoder position in counts INT32 */
#define ECAT_DRIVE_TEMP 0x2202, 0 /* A/D Reading in degrees C INT16 */

#define ECAT_DRIVE_STATUS 0x1002, 0 /* Drive status bitmap UINT32 */
#  define ECAT_STATUS_SHORTCIRCUIT          (1<<0)
#  define ECAT_STATUS_DRIVE_OVERTEMP        (1<<1)
#  define ECAT_STATUS_OVERVOLTAGE           (1<<2)
#  define ECAT_STATUS_UNDERVOLTAGE          (1<<3)
#  define ECAT_STATUS_TEMP_SENS_ACTIVE      (1<<4)
#  define ECAT_STATUS_ENCODER_FEEDBACK_ERR  (1<<5)
#  define ECAT_STATUS_PHASING_ERROR         (1<<6)
#  define ECAT_STATUS_CURRENT_LIMITED       (1<<7)
#  define ECAT_STATUS_VOLTAGE_LIMITED       (1<<8)
#  define ECAT_STATUS_POS_LIMIT_SW          (1<<9)
#  define ECAT_STATUS_NEG_LIMIT_SW          (1<<10)
#  define ECAT_STATUS_ENABLE_NOT_ACTIVE     (1<<11)
#  define ECAT_STATUS_SW_DISABLE            (1<<12)
#  define ECAT_STATUS_STOPPING              (1<<13)
#  define ECAT_STATUS_BRAKE_ON              (1<<14)
#  define ECAT_STATUS_PWM_DISABLED          (1<<15)
#  define ECAT_STATUS_POS_SW_LIMIT          (1<<16)
#  define ECAT_STATUS_NEG_SW_LIMIT          (1<<17)
#  define ECAT_STATUS_TRACKING_ERROR        (1<<18)
#  define ECAT_STATUS_TRACKING_WARNING      (1<<19)
#  define ECAT_STATUS_DRIVE_RESET           (1<<20)
#  define ECAT_STATUS_POS_WRAPPED           (1<<21)
#  define ECAT_STATUS_DRIVE_FAULT           (1<<22)
#  define ECAT_STATUS_VEL_LIMIT             (1<<23)
#  define ECAT_STATUS_ACCEL_LIMIT           (1<<24)
#  define ECAT_STATUS_TRACK_WINDOW          (1<<25)
#  define ECAT_STATUS_HOME_SWITCH_ACTIVE    (1<<26)
#  define ECAT_STATUS_IN_MOTION             (1<<27)
#  define ECAT_STATUS_VEL_WINDOW            (1<<28)
#  define ECAT_STATUS_PHASE_UNINIT          (1<<29)
#  define ECAT_STATUS_CMD_FAULT             (1<<30)

#define ECAT_LATCHED_DRIVE_FAULT 0x2183, 0 /* Drive faults bitmap UINT32 */
#  define ECAT_FAULT_DATA_CRC               (1<<0)
#  define ECAT_FAULT_INT_ERR                (1<<1)
#  define ECAT_FAULT_SHORT_CIRCUIT          (1<<2)
#  define ECAT_FAULT_DRIVE_OVER_TEMP        (1<<3)
#  define ECAT_FAULT_MOTOR_OVER_TEMP        (1<<4)
#  define ECAT_FAULT_OVER_VOLT              (1<<5)
#  define ECAT_FAULT_UNDER_VOLT             (1<<6)
#  define ECAT_FAULT_FEEDBACK_FAULT         (1<<7)
#  define ECAT_FAULT_PHASING_ERR            (1<<8)
#  define ECAT_FAULT_TRACKING_ERR           (1<<9)
#  define ECAT_FAULT_CURRENT_LIMIT          (1<<10)
#  define ECAT_FAULT_FPGA_ERR1              (1<<11)
#  define ECAT_FAULT_CMD_LOST               (1<<12)
#  define ECAT_FAULT_FPGA_ERR2              (1<<13)
#  define ECAT_FAULT_SAFETY_CIRCUIT_FAULT   (1<<14)
#  define ECAT_FAULT_CANT_CONTROL_CURRENT   (1<<15)
#  define ECAT_FAULT_WIRING_DISCONNECT      (1<<16)

#define ECAT_CURRENT_LOOP_OFFSET 0x60F6, 3 /* Added to commanded current to compensate for drift. 0.01A INT16 */
#define ECAT_MOTOR_ENC_WRAP_POS 0x2220, 0 /* Position value at which the motor encoder position will wrap. INT32 */

#define ECAT_CTL_STATUS 0x6041,0 /* Status word for Motor controller */
#  define ECAT_CTL_STATUS_READY             (1<<0)
#  define ECAT_CTL_STATUS_ON                (1<<1)
#  define ECAT_CTL_STATUS_OP_ENABLED        (1<<2)
#  define ECAT_CTL_STATUS_FAULT             (1<<3)
#  define ECAT_CTL_STATUS_VOLTAGE_ENABLED   (1<<4)
#  define ECAT_CTL_STATUS_QUICK_STOP        (1<<5)
#  define ECAT_CTL_STATUS_DISABLED          (1<<6)
#  define ECAT_CTL_STATUS_WARNING           (1<<7)
#  define ECAT_CTL_STATUS_LAST_ABORTED      (1<<8)
#  define ECAT_CTL_STATUS_USING_CANOPEN     (1<<9)
#  define ECAT_CTL_STATUS_TARGET_REACHED    (1<<10)
#  define ECAT_CTL_STATUS_LIMIT             (1<<11)
#  define ECAT_CTL_STATUS_MOVING            (1<<14)
#  define ECAT_CTL_STATUS_HOME_CAP          (1<<15)

#define ECAT_CTL_WORD 0x6040,0 /* Control work for motor controller */
#  define ECAT_CTL_ON                       (1<<0)
#  define ECAT_CTL_ENABLE_VOLTAGE           (1<<1)
#  define ECAT_CTL_QUICK_STOP               (1<<2) /* Clear to enable quick stop */
#  define ECAT_CTL_ENABLE                   (1<<3)
#  define ECAT_CTL_RESET_FAULT              (1<<7)
#  define ECAT_CTL_HALT                     (1<<8)
#endif /* INCLUDE_EC_MOTORS_H_ */
