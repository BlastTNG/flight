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
#define EC_RXPDO_ASSIGNMENT 0x1c12
#define EC_TXPDO_ASSIGNMENT 0x1c13

#define EC_TXPDO_MAPPING 0x1a00
#define EC_RXPDO_MAPPING 0x1600

#define EC_DC_CYCLE_NS 500000 /* Distributed Clock Cycle in nanoseconds */

typedef union {
    uint32_t val;
    struct {
        uint8_t size;
        uint8_t subindex;
        uint16_t index;
    };
} pdo_mapping_t;

typedef enum {
    EC_MOTOR_COLD,
    EC_MOTOR_INIT,
    EC_MOTOR_FOUND_PARTIAL,
    EC_MOTOR_FOUND,
    EC_MOTOR_RUNNING_PARTIAL,
    EC_MOTOR_RUNNING
} ec_motor_state_t;

#endif /* INCLUDE_EC_MOTORS_H_ */
