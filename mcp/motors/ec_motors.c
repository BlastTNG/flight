/* 
 * ec_motors.c: 
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
 * Created on: Mar 26, 2015 by Seth Hillbrand
 */


#include <phenom/thread.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <glib.h>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>

#include <blast_time.h>
#include <calibrate.h>
#include <ec_motors.h>
#include <motors.h>
#include <mcp.h>
#include <mputs.h>

static ph_thread_t *motor_ctl_id;

// device node Serial Numbers
#define RW_SN 0x01bbbb65
#define PIV_SN 0x02924687
#define EL_SN 0x01238408
#define RW_ADDR 0x3
#define EL_ADDR 0x2
#define PIV_ADDR 0x1

/**
 * Structure for storing the PDO assignments and their offsets in the
 * memory map
 */
#define PDO_NAME_LEN 32
typedef struct {
    char        name[PDO_NAME_LEN];
    uint16_t    index;
    uint8_t     subindex;
    int         offset;
} pdo_channel_map_t;
static GSList *pdo_list;


/**
 * Index numbers for the slave array.  0 is the master (flight computer)
 */
static int rw_index = 0;
static int piv_index = 0;
static int el_index = 0;

/**
 * Ethercat driver status
 */
static ec_motor_state_t controller_state = ECAT_MOTOR_COLD;

/**
 * Memory mapping for the PDO variables
 */
static char io_map[1024];

static int motors_exit = false;

#define N_MCs 4

/**
 * The following pointers reference data points read from/written to
 * PDOs for each motor controller.  Modifying the data at the address for the write
 * word will cause that change to be transmitted to the motor controller at
 * the next PDO cycle (0.5ms)
 */
static int32_t dummy_var = 0;
static int32_t dummy_write_var = 0;

/// Read words
static int32_t *motor_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var , &dummy_var };
static int32_t *motor_velocity[N_MCs] = { &dummy_var, &dummy_var, &dummy_var , &dummy_var };
static uint32_t *enc_state[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                      (uint32_t*) &dummy_var, (uint32_t*) &dummy_var };
static int32_t *actual_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var, &dummy_var };
static int16_t *motor_current[N_MCs] = { (int16_t*) &dummy_var, (int16_t*) &dummy_var,
                                         (int16_t*) &dummy_var, (int16_t*) &dummy_var };
static uint32_t *status_register[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                            (uint32_t*) &dummy_var, (uint32_t*) &dummy_var };
static int16_t *amp_temp[N_MCs] = { (int16_t*) &dummy_var, (int16_t*) &dummy_var,
                                    (int16_t*) &dummy_var, (int16_t*) &dummy_var };
static uint16_t *status_word[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                        (uint16_t*) &dummy_var, (uint16_t*) &dummy_var };

static uint32_t *latched_register[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                             (uint32_t*) &dummy_var, (uint32_t*) &dummy_var };
static uint16_t *control_word_read[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                              (uint16_t*) &dummy_var, (uint16_t*) &dummy_var };
static uint16_t *network_status[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                           (uint16_t*) &dummy_var, (uint16_t*) &dummy_var };

/// Write words
static uint16_t *control_word[N_MCs] = { (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var,
                                         (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var };
static int16_t *target_current[N_MCs] = { (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var,
                                          (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var };

/**
 * This set of functions return the latched faults of each motor controller
 * @return uint32 value latched bitmap
 */
uint32_t rw_get_latched(void)
{
    return *latched_register[rw_index];
}
uint32_t el_get_latched(void)
{
    return *latched_register[el_index];
}
uint32_t piv_get_latched(void)
{
    return *latched_register[piv_index];
}

/**
 * This set of functions returns the control word of each motor controller (after being set)
 * @return uint16 control word bitmap
 */
uint16_t rw_get_ctl_word(void)
{
    return *control_word_read[rw_index];
}
int16_t el_get_ctl_word(void)
{
    return *control_word_read[el_index];
}
int16_t piv_get_ctl_word(void)
{
    return *control_word_read[piv_index];
}

/**
 * This set of functions returns the networks status word of each motor controller
 * @return uint16 network status word bitmap
 */
uint16_t rw_get_net_status(void)
{
    return *network_status[rw_index];
}
int16_t el_get_net_status(void)
{
    return *network_status[el_index];
}
int16_t piv_get_net_status(void)
{
    return *network_status[piv_index];
}


/**
 * This set of functions return the absolute position read by each motor controller
 * @return int32 value of the position (modulo the wrap position value)
 */
int32_t rw_get_position(void)
{
    return *actual_position[rw_index];
}
int32_t el_get_position(void)
{
    return *actual_position[el_index];
}
int32_t el_get_motor_position(void)
{
    return *motor_position[el_index];
}
int32_t piv_get_position(void)
{
    return *actual_position[piv_index];
}

/**
 * This set of functions return the absolute position read by each motor controller
 * @return double value of the position in degrees (no set zero point)
 */
double rw_get_position_degrees(void)
{
    return rw_get_position() * RW_ENCODER_SCALING;
}
double el_get_position_degrees(void)
{
    return el_get_position() * EL_LOAD_ENCODER_SCALING;
}
double el_get_motor_position_degrees(void)
{
    return el_get_motor_position() * EL_MOTOR_ENCODER_SCALING;
}
double piv_get_position_degrees(void)
{
    return piv_get_position() * PIV_RESOLVER_SCALING;
}

/**
 * This set of functions return the calculated motor velocity of each motor
 * controller
 * @return double value of the velocity in degrees per second
 */
double rw_get_velocity_dps(void)
{
    return *motor_velocity[rw_index] * 0.1 * RW_ENCODER_SCALING;
}
double el_get_velocity_dps(void)
{
    return *motor_velocity[el_index] * 0.1 * EL_MOTOR_ENCODER_SCALING;
}
double piv_get_velocity_dps(void)
{
    return *motor_velocity[piv_index] * 0.1 * PIV_RESOLVER_SCALING;
}

/**
 * This set of functions return the calculated motor velocity of each motor
 * controller
 * @return int32 value of the velocity in 0.1 counts per second
 */
int32_t rw_get_velocity(void)
{
    return *motor_velocity[rw_index];
}
int32_t el_get_velocity(void)
{
    return *motor_velocity[el_index];
}
int32_t piv_get_velocity(void)
{
    return *motor_velocity[piv_index];
}

/**
 * This set of functions return the state bits of the external encoder if it exists
 * @return int32 value of the velocity in counts per second
 */
uint32_t rw_get_load_state(void)
{
    return *enc_state[rw_index];
}
uint32_t el_get_load_state(void)
{
    return *enc_state[el_index];
}
uint32_t piv_get_load_state(void)
{
    return *enc_state[piv_index];
}

/**
 * This set of functions returns the current in units of 0.01A
 * @return int16 value of the current
 */
int16_t rw_get_current(void)
{
    return *motor_current[rw_index];
}
int16_t el_get_current(void)
{
    return *motor_current[el_index]*EL_MOTOR_CURRENT_SCALING;
}
int16_t piv_get_current(void)
{
    return *motor_current[piv_index];
}

/**
 * Returns a bitmap of what the motor controller is currently doing
 * @return int16 bitmap
 */
uint16_t rw_get_status_word(void)
{
    return *status_word[rw_index];
}
uint16_t el_get_status_word(void)
{
    return *status_word[el_index];
}
uint16_t piv_get_status_word(void)
{
    return *status_word[piv_index];
}

/**
 * Returns a bitmap of the state of the motor controller
 * @return int32 bitmap
 */
uint32_t rw_get_status_register(void)
{
    return *status_register[rw_index];
}
uint32_t el_get_status_register(void)
{
    return *status_register[el_index];
}
uint32_t piv_get_status_register(void)
{
    return *status_register[piv_index];
}

/**
 * Gets the current amplifier temperature in units of degrees C
 * @return int16 degrees C
 */
int16_t rw_get_amp_temp(void)
{
    return *amp_temp[rw_index];
}
int16_t el_get_amp_temp(void)
{
    return *amp_temp[el_index];
}
int16_t piv_get_amp_temp(void)
{
    return *amp_temp[piv_index];
}

/**
 * Sets the requested current for the motor
 * @param m_current int32 requested current in units of 0.01 amps
 */
void rw_set_current(int16_t m_cur)
{
    *target_current[rw_index] = m_cur;
}
void el_set_current(int16_t m_cur)
{
    *target_current[el_index] = m_cur*EL_MOTOR_CURRENT_SCALING;
}
void piv_set_current(int16_t m_cur)
{
    *target_current[piv_index] = m_cur;
}

/**
 * Enables the operation of the motor controller.  These are currently set using the
 * PDO interface
 */
void rw_enable(void)
{
    *control_word[rw_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
}
void el_enable(void)
{
    *control_word[el_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
}
void piv_enable(void)
{
    *control_word[piv_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
}

/**
 * Disables the operation of the motor controller.  These are currently set using the
 * PDO interface
 */
void rw_disable(void)
{
    *control_word[rw_index] |= ECAT_CTL_HALT;
}
void el_disable(void)
{
    *control_word[el_index] |= ECAT_CTL_HALT;
}
void piv_disable(void)
{
    *control_word[piv_index] |= ECAT_CTL_HALT;
}

/**
 * Sets the emergency quick stop flag for each motor
 */
void rw_quick_stop(void)
{
    *control_word[rw_index] &= (~ECAT_CTL_QUICK_STOP);
}
void el_quick_stop(void)
{
    *control_word[el_index] &= (~ECAT_CTL_QUICK_STOP);
}
void piv_quick_stop(void)
{
    *control_word[piv_index] &= (~ECAT_CTL_QUICK_STOP);
}

/**
 * Sets 'Reset Fault' flag for each motor
 */
void rw_reset_fault(void)
{
    *control_word[rw_index] |= ECAT_CTL_RESET_FAULT;
}
void el_reset_fault(void)
{
    *control_word[el_index] |= ECAT_CTL_RESET_FAULT;
}
void piv_reset_fault(void)
{
    *control_word[piv_index] |= ECAT_CTL_RESET_FAULT;
}

/**
 * Sets the current limits for each motor.
 * TODO: Update the current limits for each motor controller/motor pair
 */
void rw_init_current_limit(void)
{
    if (rw_index) {
        ec_SDOwrite16(rw_index, 0x2110, 0, 3600);   // 36 Amps peak current limit
        ec_SDOwrite16(rw_index, 0x2111, 0, 1200);   // 12 Amps continuous current limit
    }
}
void el_init_current_limit(void)
{
    if (el_index) {
        ec_SDOwrite16(el_index, 0x2110, 0, 3600);   // 36 Amps peak current limit
        ec_SDOwrite16(el_index, 0x2111, 0, 1200);    // 12 Amps continuous current limit
    }
}
void piv_init_current_limit(void)
{
    if (piv_index) {
        ec_SDOwrite16(piv_index, 0x2110, 0, 3000);   // 30 Amps peak current limit
        ec_SDOwrite16(piv_index, 0x2111, 0, 1500);   // 15 Amps continuous current limit
    }
}

/**
 * Sets the current default PID values for each motor
 * TODO: Update the current PIDs for each motor after tuning
 */
void rw_init_current_pid(void)
{
    if (rw_index) {
        ec_SDOwrite16(rw_index, ECAT_CURRENT_LOOP_CP, RW_DEFAULT_CURRENT_P);
        ec_SDOwrite16(rw_index, ECAT_CURRENT_LOOP_CI, RW_DEFAULT_CURRENT_I);
        ec_SDOwrite16(rw_index, ECAT_CURRENT_LOOP_OFFSET, RW_DEFAULT_CURRENT_OFF);
    }
}
void el_init_current_pid(void)
{
    if (el_index) {
        ec_SDOwrite16(el_index, ECAT_CURRENT_LOOP_CP, EL_DEFAULT_CURRENT_P);
        ec_SDOwrite16(el_index, ECAT_CURRENT_LOOP_CI, EL_DEFAULT_CURRENT_I);
        ec_SDOwrite16(el_index, ECAT_CURRENT_LOOP_OFFSET, EL_DEFAULT_CURRENT_OFF);
    }
}
void piv_init_current_pid(void)
{
    if (piv_index) {
        ec_SDOwrite16(piv_index, ECAT_CURRENT_LOOP_CP, PIV_DEFAULT_CURRENT_P);
        ec_SDOwrite16(piv_index, ECAT_CURRENT_LOOP_CI, PIV_DEFAULT_CURRENT_I);
        ec_SDOwrite16(piv_index, ECAT_CURRENT_LOOP_OFFSET, PIV_DEFAULT_CURRENT_OFF);
    }
}

static void rw_init_encoder(void)
{
    if (rw_index) {
        ec_SDOwrite32(rw_index, ECAT_ENCODER_WRAP, RW_ENCODER_COUNTS);
        ec_SDOwrite32(rw_index, ECAT_COUNTS_PER_REV, RW_COUNTS_PER_REV);
    }
}

static void el_init_encoder(void)
{
    if (el_index) {
        ec_SDOwrite32(el_index, ECAT_ENCODER_WRAP, EL_MOTOR_ENCODER_COUNTS);
        ec_SDOwrite32(el_index, ECAT_COUNTS_PER_REV, EL_MOTOR_COUNTS_PER_REV);
        ec_SDOwrite32(el_index, ECAT_LOAD_WRAP, EL_LOAD_ENCODER_COUNTS);
        ec_SDOwrite16(el_index, ECAT_LOAD_DIR, 1); // The load encoder runs in the opposite direction of the motor
    }
}

static void piv_init_resolver(void)
{
    if (piv_index) {
        ec_SDOwrite32(piv_index, ECAT_ENCODER_WRAP, PIV_RESOLVER_COUNTS);
        ec_SDOwrite32(piv_index, ECAT_COUNTS_PER_REV, PIV_RESOLVER_COUNTS);
        ec_SDOwrite16(piv_index, ECAT_RESOLVER_CYCLES_PER_REV, 1);
    }
}

/**
 * Finds all motor controllers on the network and sets them to pre-operational state
 * @return -1 on error, number of controllers found otherwise
 */
static int find_controllers(void)
{
    /// TODO: Update find_controllers to utilize specific adapter for motor controllers (control via system)
    char name[16] = "eth1";
    int ret_init;
    int ret_config;

    if (controller_state == ECAT_MOTOR_COLD) {
        if (!(ret_init = ec_init(name))) {
            berror(err, "Could not initialize %s", name);
            goto find_err;
        }
    }

    controller_state = ECAT_MOTOR_INIT;
    ret_config = ec_config(false, &io_map);
    if (ret_config <= 0) {
        berror(err, "No motor controller slaves found on the network!");
        goto find_err;
    }
    blast_startup("ec_config returns %d slaves found", ret_config);

    if (ret_config < 3)
        controller_state = ECAT_MOTOR_FOUND_PARTIAL;
    else
        controller_state = ECAT_MOTOR_FOUND;

    /* wait for all slaves to reach SAFE_OP state */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3) != EC_STATE_SAFE_OP) {
        controller_state = ECAT_MOTOR_RUNNING_PARTIAL;
        blast_err("Not all slaves reached safe operational state.");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                blast_err("Slave %d State=%2x StatusCode=%4x : %s", i, ec_slave[i].state,
                        ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    } else {
        controller_state = ECAT_MOTOR_RUNNING;
    }

    for (int i = 1; i <= ec_slavecount; i++) {
        /**
         * Configure the index values for later use.  These are mapped to the hard-set
         * addresses on the motor controllers (look for the dials on the side)
         */
        if (ec_slave[i].aliasadr == RW_ADDR) {
            int32_t serial = 0;
            int size = 4;
            ec_SDOread(i, 0x2384, 1, false, &size, &serial, EC_TIMEOUTRXM);
            blast_startup("Reaction Wheel Motor Controller %d: %s: SN: %d",
                          ec_slave[i].aliasadr, ec_slave[i].name, serial);
            rw_index = i;
        } else if (ec_slave[i].aliasadr == PIV_ADDR) {
            int32_t serial = 0;
            int size = 4;
            ec_SDOread(i, 0x2384, 1, false, &size, &serial, EC_TIMEOUTRXM);
            blast_startup("Pivot Motor Controller %d: %s: SN: %d",
                          ec_slave[i].aliasadr, ec_slave[i].name, serial);
            piv_index = i;
        } else if (ec_slave[i].aliasadr == EL_ADDR) {
            int32_t serial = 0;
            int size = 4;
            ec_SDOread(i, 0x2384, 1, false, &size, &serial, EC_TIMEOUTRXM);
            blast_startup("Elevation Motor Controller %d: %s: SN: %d",
                          ec_slave[i].aliasadr, ec_slave[i].name, serial);
            el_index = i;
        } else {
            blast_warn("Got unknown MC %s at position %d with alias %d",
                       ec_slave[i].name, ec_slave[i].configadr, ec_slave[i].aliasadr);
        }
    }
    return ec_slavecount;

find_err:
    return -1;
}

/**
 * Configure the PDO assignments.  PDO (Process Data Objects) are sent on configured SYNC cycles
 * as a static block of up to 8 bytes each.  Locally, we map these objects to a chunk of memory
 * that can be read or modified outside of the loop that send these objects to the controllers.
 *
 * In general, we save these parameters to the FLASH of the controller before flight but this function
 * acts to ensure a correct mapping upon startup, should the flash be corrupted.
 *
 * @param m_slave Slave index number to configure.
 * @return -1 on failure otherwise 0
 */
static int motor_pdo_init(int m_slave)
{
    pdo_mapping_t map;

    if (ec_slave[m_slave].state != EC_STATE_SAFE_OP && ec_slave[m_slave].state != EC_STATE_PRE_OP) {
        blast_err("Motor Controller %d (%s) is not in pre-operational state!  Cannot configure.",
                  m_slave, ec_slave[m_slave].name);
        return -1;
    }

    blast_startup("Configuring PDO Mappings for controller %d (%s)", m_slave, ec_slave[m_slave].name);

    /**
     * To program the PDO mapping, we first must clear the old state
     */

    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_ASSIGNMENT, 0, 0)) blast_err("Failed mapping!");
    for (int i = 0; i < 4; i++) {
        if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING + i, 0, 0)) blast_err("Failed mapping!");
    }

    /**
     * Define the PDOs that we want to send to the flight computer from the Controllers
     */
    map_pdo(&map, ECAT_MOTOR_POSITION, 32);  // Motor Position
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING, 1, map.val)) blast_err("Failed mapping!");

    map_pdo(&map, ECAT_VEL_ACTUAL, 32);     // Actual Motor Velocity
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING, 2, map.val)) blast_err("Failed mapping!");


    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING, 0, 2)) /// Set the 0x1a00 map to contain 2 elements
        blast_err("Failed mapping!");
    if (!ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT, 1, ECAT_TXPDO_MAPPING)) /// 0x1a00 maps to the first PDO
        blast_err("Failed mapping!");

    /**
     * Second map (0x1a01 register)
     */

    map_pdo(&map, ECAT_ACTUAL_POSITION, 32); // Actual Position (load for El, duplicates ECAT_MOTOR_POSITION for others)
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+1, 1, map.val)) blast_err("Failed mapping!");

    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING+1, 0, 1)) /// Set the 0x1a01 map to contain 1 element
        blast_err("Failed mapping!");
    if (!ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT, 2, ECAT_TXPDO_MAPPING + 1)) /// 0x1a01 maps to the second PDO
        blast_err("Failed mapping!");

    /**
     * Third map (0x1a02 register)
     */
    map_pdo(&map, ECAT_DRIVE_STATUS, 32); // Status Register
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+2, 1, map.val)) blast_err("Failed mapping!");

    map_pdo(&map, ECAT_CTL_STATUS, 16); // Status Word
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+2, 2, map.val)) blast_err("Failed mapping!");

    map_pdo(&map, ECAT_DRIVE_TEMP, 16); // Amplifier Temp (deg C)
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+2, 3, map.val)) blast_err("Failed mapping!");

    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING+2, 0, 3)) /// Set the 0x1a02 map to contain 3 elements
        blast_err("Failed mapping!");
    if (!ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT, 3, ECAT_TXPDO_MAPPING + 2)) /// 0x1a02 maps to the third PDO
        blast_err("Failed mapping!");

    /**
     * Fourth map (0x1a03 register)
     */
    map_pdo(&map, ECAT_LATCHED_DRIVE_FAULT, 32); // Latched Fault Register
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+3, 1, map.val)) blast_err("Failed mapping!");

    map_pdo(&map, ECAT_CURRENT_ACTUAL, 16); // Measured current output
    if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+3, 2, map.val)) blast_err("Failed mapping!");


    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING+3, 0, 2)) /// Set the 0x1a03 map to contain 2 elements
        blast_err("Failed mapping!");
    if (!ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT, 4, ECAT_TXPDO_MAPPING + 3)) /// 0x1a03 maps to the fourth PDO
        blast_err("Failed mapping!");

    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_ASSIGNMENT, 0, 4)) /// There are four maps in the TX PDOs
        blast_err("Failed mapping!");


    /**
     * To program the PDO mapping, we first must clear the old state
     */
    for (int i = 0; i < 4; i++) ec_SDOwrite8(m_slave, ECAT_RXPDO_MAPPING + i, 0, 0);
    ec_SDOwrite8(m_slave, ECAT_RXPDO_ASSIGNMENT, 0, 0);
    /**
     * Define the PDOs that we want to send from the flight computer to the Controllers
     */
    /**
     * First map (0x1600 register)
     */
    map_pdo(&map, ECAT_CTL_WORD, 16);   // Control Word
    ec_SDOwrite32(m_slave, ECAT_RXPDO_MAPPING, 1, map.val);

    map_pdo(&map, ECAT_CURRENT_LOOP_CMD, 16);    // Target Current
    ec_SDOwrite32(m_slave, ECAT_RXPDO_MAPPING, 2, map.val);

    ec_SDOwrite8(m_slave, ECAT_RXPDO_MAPPING, 0, 2); /// Set the 0x1600 map to contain 2 elements
    ec_SDOwrite16(m_slave, ECAT_RXPDO_ASSIGNMENT, 1, ECAT_RXPDO_MAPPING); /// Set the 0x1600 map to the first PDO

    ec_SDOwrite8(m_slave, ECAT_RXPDO_ASSIGNMENT, 0, 1); /// There is on map in the RX PDOs

    ec_SDOwrite32(m_slave, 0x2420, 0, 8);
    ec_SDOwrite32(m_slave, 0x1010, 1, 0x65766173);

    return 0;
}

/**
 * Generic mapping function for ethercat slave PDO variables.  This function works with
 * motor_pdo_init to set up the PDO input/output memory mappings for the motor controllers
 * @param m_index position on the ethercat chain
 */
static void map_index_vars(int m_index)
{
    bool found;

    /**
     * Inputs.  Each is sequentially mapped to the IOMap memory space
     * for the motor controller
     */
#define PDO_SEARCH_LIST(_obj, _map) { \
    found = false; \
    for (GSList *el = pdo_list; (el); el = g_slist_next(el)) { \
        pdo_channel_map_t *ch = (pdo_channel_map_t*)el->data; \
        if (ch->index == object_index(_obj) && \
                ch->subindex == object_subindex(_obj)) { \
            _map[m_index] = (typeof(_map[0])) (ec_slave[m_index].inputs + ch->offset); \
            found = true; \
        } \
    } \
    if (!found) blast_err("Could not find PDO map for %s", #_map); \
    }


    PDO_SEARCH_LIST(ECAT_MOTOR_POSITION, motor_position);
    PDO_SEARCH_LIST(ECAT_VEL_ACTUAL, motor_velocity);
    PDO_SEARCH_LIST(ECAT_LOAD_STATUS, enc_state);
    PDO_SEARCH_LIST(ECAT_ACTUAL_POSITION, actual_position);
    PDO_SEARCH_LIST(ECAT_DRIVE_STATUS, status_register);
    PDO_SEARCH_LIST(ECAT_CTL_STATUS, status_word);
    PDO_SEARCH_LIST(ECAT_DRIVE_TEMP, amp_temp);
    PDO_SEARCH_LIST(ECAT_LATCHED_DRIVE_FAULT, latched_register);
    PDO_SEARCH_LIST(ECAT_CURRENT_ACTUAL, motor_current);
    PDO_SEARCH_LIST(ECAT_NET_STATUS, network_status);
#undef PDO_SEARCH_LIST

    // TODO(seth): Add dynamic mapping to outputs
    /// Outputs
    control_word[m_index] = (uint16_t*) (ec_slave[m_index].outputs);
    target_current[m_index] = (int16_t*) (control_word[m_index] + 1);
}
/**
 * Interface function to @map_index_vars.  Maps the variables for each of the motor
 * controllers found on the bus.  If the motor controller is not found, its map remains
 * attached to the @dummy_var position
 */
static void map_motor_vars(void)
{
    if (el_index) map_index_vars(el_index);
    if (rw_index) map_index_vars(rw_index);
    if (piv_index) map_index_vars(piv_index);
}

/**
 * There must be only a single distributed clock master on the entire bus.  This should be one of the
 * controllers and so we choose the first slave that has dc enabled as the SYNC master.  Everyone
 * else on the bus gets the same DC_CYCLE (in nanoseconds) but have their sync disabled.
 */
static void motor_configure_timing(void)
{
    int found_dc_master = 0;
    ec_configdc();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (!found_dc_master && ec_slave[i].hasdc) {
            ec_dcsync0(i, true, ECAT_DC_CYCLE_NS, ec_slave[i].pdelay);
            found_dc_master = 1;
        } else {
            ec_dcsync0(i, false, ECAT_DC_CYCLE_NS, ec_slave[i].pdelay);
        }
        ec_SDOwrite16(i, 0x1C32, 1, 0);
    }
}

static int motor_set_operational()
{
    /* send one processdata cycle to init SM in slaves */
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_slave[0].state = EC_STATE_OPERATIONAL;

    /* send one valid process data to make outputs in slaves happy*/
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* request OP state for all slaves */
    ec_writestate(0);

    /* wait for all slaves to reach OP state */
    for (int i = 0; i < 40; i++) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        if (ec_statecheck(0, EC_STATE_OPERATIONAL, 50000) == EC_STATE_OPERATIONAL) break;
    }

    /**
     * If we've reached fully operational state, return
     */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        controller_state = ECAT_MOTOR_RUNNING;
        return 0;
    }

    /**
     * Something has prevented a motor controller from entering operational mode (EtherCAT Ops)
     */
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
            blast_err("Slave %d State=%2x StatusCode=%4x : %s", i, ec_slave[i].state,
                    ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
    }
    return -1;
}

static void read_motor_data()
{
    int motor_i = motor_index;

    RWMotorData[motor_i].current = rw_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    RWMotorData[motor_i].drive_info = rw_get_status_word();
    RWMotorData[motor_i].fault_reg = rw_get_latched();
    RWMotorData[motor_i].status = rw_get_status_register();
    RWMotorData[motor_i].position = rw_get_position();
    RWMotorData[motor_i].motor_position = rw_get_position();
    RWMotorData[motor_i].temp = rw_get_amp_temp();
    RWMotorData[motor_i].velocity = rw_get_velocity();
    RWMotorData[motor_i].load_state = rw_get_load_state();
    RWMotorData[motor_i].state = rw_get_ctl_word();
    RWMotorData[motor_i].net_status = rw_get_net_status();

    ElevMotorData[motor_i].current = el_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    ElevMotorData[motor_i].drive_info = el_get_status_word();
    ElevMotorData[motor_i].fault_reg = el_get_latched();
    ElevMotorData[motor_i].status = el_get_status_register();
    ElevMotorData[motor_i].position = el_get_position();
    ElevMotorData[motor_i].motor_position = el_get_motor_position();
    ElevMotorData[motor_i].temp = el_get_amp_temp();
    ElevMotorData[motor_i].velocity = el_get_velocity();
    ElevMotorData[motor_i].load_state = el_get_load_state();
    ElevMotorData[motor_i].state = el_get_ctl_word();
    ElevMotorData[motor_i].net_status = el_get_net_status();

    PivotMotorData[motor_i].current = piv_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    PivotMotorData[motor_i].drive_info = piv_get_status_word();
    PivotMotorData[motor_i].fault_reg = piv_get_latched();
    PivotMotorData[motor_i].status = piv_get_status_register();
    PivotMotorData[motor_i].position = piv_get_position();
    PivotMotorData[motor_i].temp = piv_get_amp_temp();
    PivotMotorData[motor_i].velocity = piv_get_velocity();
    PivotMotorData[motor_i].load_state = piv_get_load_state();
    PivotMotorData[motor_i].state = piv_get_ctl_word();
    PivotMotorData[motor_i].net_status = piv_get_net_status();

    motor_index = INC_INDEX(motor_index);
}

void mc_readPDOassign(int Slave) {
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc = 0;
    int len = 0;
    int offset = 0;

    len = sizeof(rdat);
    rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(Slave, ECAT_TXPDO_ASSIGNMENT, 0x00, FALSE, &len, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    if ((wkc <= 0) || (rdat <= 0))  return;

    /* number of available sub indexes */
    nidx = rdat;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++) {
        len = sizeof(rdat);
        rdat = 0;
        /* read PDO assign */
        wkc = ec_SDOread(Slave, ECAT_TXPDO_ASSIGNMENT, (uint8) idxloop, FALSE, &len, &rdat, EC_TIMEOUTRXM);
        /* result is index of PDO */
        idx = etohl(rdat);
        if (idx <= 0) continue;

        len = sizeof(subcnt);
        subcnt = 0;
        /* read number of subindexes of PDO */
        wkc = ec_SDOread(Slave, idx, 0x00, FALSE, &len, &subcnt, EC_TIMEOUTRXM);
        subidx = subcnt;
        /* for each subindex */
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++) {
            pdo_channel_map_t *channel = NULL;
            pdo_mapping_t pdo_map = { 0 };
            len = sizeof(pdo_map);
            /* read SDO that is mapped in PDO */
            wkc = ec_SDOread(Slave, idx, (uint8) subidxloop, FALSE, &len, &pdo_map, EC_TIMEOUTRXM);

            channel = malloc(sizeof(pdo_channel_map_t));
            channel->index = pdo_map.index;
            channel->subindex = pdo_map.subindex;
            channel->offset = offset;
            pdo_list = g_slist_prepend(pdo_list, channel);

            /// Offset is the number of bytes into the memory map this element is.  First element is 0 bytes in.
            offset += (pdo_map.size / 8);
        }
    }
}

static void* motor_control(void* arg)
{
    int expectedWKC, wkc;
    int ret, len;
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 2000000}; /// 500HZ interval

    nameThread("Motors");
    blast_startup("Starting Motor Control");

    ph_thread_set_name("Motors");
    // TODO(seth): setup state machine for looping in motors
    find_controllers();

    for (int i = 1; i <= ec_slavecount; i++) {
        motor_pdo_init(i);
        mc_readPDOassign(i);
    }
    /// We re-configure the map now that we have assigned the PDOs
    ec_config_map(&io_map);
    map_motor_vars();

    /**
     * Get the current value of each RX word to avoid stomping on the current state
     */
    for (int i = 1; i <= ec_slavecount; i++) {
        len = 4;
        ec_SDOread(i, ECAT_CURRENT_LOOP_CMD, false, &len, target_current[i], EC_TIMEOUTRXM);
        len = 2;
        ec_SDOread(i, ECAT_CTL_WORD, false, &len, control_word[i], EC_TIMEOUTRXM);
    }

    /// Set the default current limits
    rw_init_current_limit();
    el_init_current_limit();
    piv_init_current_limit();

    rw_init_current_pid();
    el_init_current_pid();
    piv_init_current_pid();

    /// Set the encoder defaults
    rw_init_encoder();
    el_init_encoder();
    piv_init_resolver();

    /// Start the Distributed Clock cycle
    motor_configure_timing();

    /// Put the motors in Operational mode (EtherCAT Operation)
    motor_set_operational();

    for (int i = 1; i <= ec_slavecount; i++) {
        ec_SDOwrite16(i, ECAT_DRIVE_STATE, ECAT_DRIVE_STATE_PROG_CURRENT);
    }

    /// Our work counter (WKC) provides a count of the number of items to handle.
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    clock_gettime(CLOCK_REALTIME, &ts);
    while (!motors_exit) {
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (ret && ret != -EINTR) {
            blast_err("error while sleeping, code %d (%s)\n", ret, strerror(-ret));
            break;
        }
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < expectedWKC) bprintf(none, "Possible missing data in communicating with Motor Controllers");
        read_motor_data();

        while (ec_iserror()) {
            blast_err("%s", ec_elist2string());
        }
    }

    return 0;
}


/* opens communications with motor controllers */
void initialize_motors(void)
{
  memset(ElevMotorData, 0, sizeof(ElevMotorData));
  memset(RWMotorData, 0, sizeof(RWMotorData));
  memset(PivotMotorData, 0, sizeof(PivotMotorData));

  motor_ctl_id =  ph_thread_spawn(motor_control, NULL);
}

void shutdown_motors(void)
{
    blast_info("Shutting down motors");
}
