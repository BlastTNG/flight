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
#include <command_struct.h>
#include <ec_motors.h>
#include <motors.h>
#include <mcp.h>
#include <mputs.h>

static ph_thread_t *motor_ctl_id;
static ph_thread_t *ecmonitor_ctl_id;

extern int16_t InCharge;

/**
 * Number of ethercat controllers (including the HWP encoder)
 */
#define N_MCs 5 // If you change this, also change EC_MAXSLAVE in ethercatmain.h

// device node Serial Numbers
#define RW_SN 0x01bbbb65
#define PIV_SN 0x02924687
#define EL_SN 0x01238408
#define RW_ADDR 0x3
#define EL_ADDR 0x2
#define PIV_ADDR 0x1
#define FUCHS_MFG_ID 0x00ad // HWP encoder
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
static GSList *pdo_list[N_MCs];


/**
 * Index numbers for the slave array.  0 is the master (flight computer)
 */
static int rw_index = 0;
static int piv_index = 0;
static int el_index = 0;
static int hwp_index = 0;

/**
 * Memory mapping for the PDO variables
 */
static char io_map[4096];

static int motors_exit = false;

/**
 * Ethercat driver status
 */

static ec_device_state_t controller_state[N_MCs] = {{0}};
/*
static ec_motor_state_t controller_state[N_MCs] = {ECAT_MOTOR_COLD, ECAT_MOTOR_COLD, ECAT_MOTOR_COLD, ECAT_MOTOR_COLD};
*/
ec_state_t ec_mcp_state = {0};

/**
 * The following pointers reference data points read from/written to
 * PDOs for each motor controller.  Modifying the data at the address for the write
 * word will cause that change to be transmitted to the motor controller at
 * the next PDO cycle (0.5ms)
 */
static int32_t dummy_var = 0;
static int32_t dummy_write_var = 0;

/// Read words
static int32_t *motor_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var , &dummy_var , &dummy_var };
static int32_t *motor_velocity[N_MCs] = { &dummy_var, &dummy_var, &dummy_var , &dummy_var , &dummy_var };
static int32_t *actual_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var, &dummy_var , &dummy_var };
static int16_t *motor_current[N_MCs] = { (int16_t*) &dummy_var, (int16_t*) &dummy_var,
                                         (int16_t*) &dummy_var, (int16_t*) &dummy_var , (int16_t*) &dummy_var };
static uint32_t *status_register[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                            (uint32_t*) &dummy_var, (uint32_t*) &dummy_var , (uint32_t*) &dummy_var };
static int16_t *amp_temp[N_MCs] = { (int16_t*) &dummy_var, (int16_t*) &dummy_var,
                                    (int16_t*) &dummy_var, (int16_t*) &dummy_var };
static uint16_t *status_word[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                        (uint16_t*) &dummy_var, (uint16_t*) &dummy_var , (uint16_t*) &dummy_var };
static uint16_t *network_status_word[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                        (uint16_t*) &dummy_var, (uint16_t*) &dummy_var , (uint16_t*) &dummy_var };
static uint32_t *latched_register[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                             (uint32_t*) &dummy_var, (uint32_t*) &dummy_var , (uint32_t*) &dummy_var };
static uint16_t *control_word_read[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                              (uint16_t*) &dummy_var, (uint16_t*) &dummy_var , (uint16_t*) &dummy_var };
/// Write words
static uint16_t *control_word[N_MCs] = { (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var,
                                         (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var ,
                                         (uint16_t*) &dummy_write_var };
static int16_t *target_current[N_MCs] = { (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var,
                                          (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var ,
                                          (int16_t*) &dummy_write_var };

/// Read word from encoder
static uint32_t *hwp_position[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                         (uint32_t*) &dummy_var, (uint32_t*) &dummy_var};

int check_slave_comm_ready(int m_index) {
    if (m_index < 1) return 0; // m_index must be > 0
    if (!controller_state[m_index].comms_ok) {
        return 0; // no errors in mapping
    } else {
        return 1;
    }
}

uint32_t hwp_get_position(void)
{
    if (check_slave_comm_ready(hwp_index)) {
        return *hwp_position[hwp_index];
    } else {
        return 0;
    }
}

uint16_t hwp_get_state(void)
{
    if (check_slave_comm_ready(hwp_index)) {
        return 1;
    } else {
        return 0;
    }
}
/**
 * This set of functions return the latched faults of each motor controller
 * @return uint32 value latched bitmap
 */
uint32_t rw_get_latched(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *latched_register[rw_index];
    } else {
        return 0;
    }
}
uint32_t el_get_latched(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *latched_register[el_index];
    } else {
        return 0;
    }
}
uint32_t piv_get_latched(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *latched_register[piv_index];
    } else {
        return 0;
    }
}

/**
 * This set of functions returns the control word of each motor controller (read back from the controller)
 * @return uint16 control word bitmap
 */
uint16_t rw_get_ctl_word(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *control_word_read[rw_index];
    } else {
        return 0;
    }
}
int16_t el_get_ctl_word(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *control_word_read[el_index];
    } else {
        return 0;
    }
}
int16_t piv_get_ctl_word(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *control_word_read[piv_index];
    } else {
        return 0;
    }
}

/**
 * This set of functions returns the control word of each motor controller (sent to the controller)
 * @return uint16 control word bitmap
 */
uint16_t rw_get_ctl_word_write(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *control_word[rw_index];
    } else {
        return 0;
    }
}
int16_t el_get_ctl_word_write(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *control_word[el_index];
    } else {
        return 0;
    }
}
int16_t piv_get_ctl_word_write(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *control_word[piv_index];
    } else {
        return 0;
    }
}

/**
 * This set of functions returns the network status word of each motor controller 
 * @return uint16 network status word bitmap
 */
uint16_t rw_get_network_status_word(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *network_status_word[rw_index];
    } else {
        return 0;
    }
}
uint16_t el_get_network_status_word(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *network_status_word[el_index];
    } else {
        return 0;
    }
}
uint16_t piv_get_network_status_word(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *network_status_word[piv_index];
    } else {
        return 0;
    }
}

/**
 * This set of functions return the absolute position read by each motor controller
 * @return int32 value of the position (modulo the wrap position value)
 */
int32_t rw_get_position(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *actual_position[rw_index];
    } else {
        return 0;
    }
}
int32_t el_get_position(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *actual_position[el_index];
    } else {
        return 0;
    }
}
int32_t el_get_motor_position(void) // Offsetting motor units to correspond with elevation
{
    if (check_slave_comm_ready(el_index)) {
        return *motor_position[el_index] + ENC_RAW_EL_OFFSET/EL_MOTOR_ENCODER_SCALING;
    } else {
        return 0;
    }
}
int32_t piv_get_position(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *actual_position[piv_index];
    } else {
        return 0;
    }
}

/**
 * This set of functions return the absolute position read by each motor controller
 * @return double value of the position in degrees (no set zero point)
 */
double rw_get_position_degrees(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return rw_get_position() * RW_ENCODER_SCALING;
    } else {
        return 0;
    }
}
double el_get_position_degrees(void)
{
    if (check_slave_comm_ready(el_index)) {
        return el_get_position() * EL_LOAD_ENCODER_SCALING;
    } else {
        return 0;
    }
}
double el_get_motor_position_degrees(void)
{
    if (check_slave_comm_ready(el_index)) {
        return el_get_motor_position() * EL_MOTOR_ENCODER_SCALING;
    } else {
        return 0;
    }
}
double piv_get_position_degrees(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return piv_get_position() * PIV_RESOLVER_SCALING;
    } else {
        return 0;
    }
}

/**
 * This set of functions return the calculated motor velocity of each motor
 * controller
 * @return double value of the velocity in degrees per second
 */
double rw_get_velocity_dps(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *motor_velocity[rw_index] * 0.1 * RW_ENCODER_SCALING;
    } else {
        return 0;
    }
}
double el_get_velocity_dps(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *motor_velocity[el_index] * 0.1 * EL_MOTOR_ENCODER_SCALING;
    } else {
        return 0;
    }
}
double piv_get_velocity_dps(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *motor_velocity[piv_index] * 0.1 * PIV_RESOLVER_SCALING;
    } else {
        return 0;
    }
}

/**
 * This set of functions return the calculated motor velocity of each motor
 * controller
 * @return int32 value of the velocity in 0.1 counts per second
 */
int32_t rw_get_velocity(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *motor_velocity[rw_index];
    } else {
        return 0;
    }
}
int32_t el_get_velocity(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *motor_velocity[el_index];
    } else {
        return 0;
    }
}
int32_t piv_get_velocity(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *motor_velocity[piv_index];
    } else {
        return 0;
    }
}

/**
 * This set of functions returns the current in units of 0.01A
 * @return int16 value of the current
 */
int16_t rw_get_current(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *motor_current[rw_index];
    } else {
        return 0;
    }
}
int16_t el_get_current(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *motor_current[el_index]*EL_MOTOR_CURRENT_SCALING;
    } else {
        return 0;
    }
}
int16_t piv_get_current(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *motor_current[piv_index];
    } else {
        return 0;
    }
}

/**
 * Returns a bitmap of what the motor controller is currently doing
 * @return int16 bitmap
 */
uint16_t rw_get_status_word(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *status_word[rw_index];
    } else {
        return 0;
    }
}
uint16_t el_get_status_word(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *status_word[el_index];
    } else {
        return 0;
    }
}
uint16_t piv_get_status_word(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *status_word[piv_index];
    } else {
        return 0;
    }
}

/**
 * Returns a bitmap of the state of the motor controller
 * @return int32 bitmap
 */
uint32_t rw_get_status_register(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *status_register[rw_index];
    } else {
        return 0;
    }
}
uint32_t el_get_status_register(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *status_register[el_index];
    } else {
        return 0;
    }
}
uint32_t piv_get_status_register(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *status_register[piv_index];
    } else {
        return 0;
    }
}

/**
 * Gets the current amplifier temperature in units of degrees C
 * @return int16 degrees C
 */
int16_t rw_get_amp_temp(void)
{
    if (check_slave_comm_ready(rw_index)) {
        return *amp_temp[rw_index];
    } else {
        return 0;
    }
}
int16_t el_get_amp_temp(void)
{
    if (check_slave_comm_ready(el_index)) {
        return *amp_temp[el_index];
    } else {
        return 0;
    }
}
int16_t piv_get_amp_temp(void)
{
    if (check_slave_comm_ready(piv_index)) {
        return *amp_temp[piv_index];
    } else {
        return 0;
    }
}

/**
 * Sets the requested current for the motor
 * @param m_current int32 requested current in units of 0.01 amps
 */
void rw_set_current(int16_t m_cur)
{
    if (check_slave_comm_ready(rw_index)) {
        *target_current[rw_index] = m_cur;
    }
}
void el_set_current(int16_t m_cur)
{
    if (check_slave_comm_ready(el_index)) {
        *target_current[el_index] = m_cur*EL_MOTOR_CURRENT_SCALING;
    }
}
void piv_set_current(int16_t m_cur)
{
    if (check_slave_comm_ready(piv_index)) {
        *target_current[piv_index] = m_cur;
    }
}

/**
 * Enables the operation of the motor controller.  These are currently set using the
 * PDO interface
 */
void rw_enable(void)
{
    if (check_slave_comm_ready(rw_index)) {
        *control_word[rw_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
    }
}
void el_enable(void)
{
    if (check_slave_comm_ready(el_index)) {
        *control_word[el_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
    }
}
void piv_enable(void)
{
    if (check_slave_comm_ready(piv_index)) {
        *control_word[piv_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
    }
}

/**
 * Disables the operation of the motor controller.  These are currently set using the
 * PDO interface
 */
void rw_disable(void)
{
    if (check_slave_comm_ready(rw_index)) {
        *control_word[rw_index] &= (~ECAT_CTL_ENABLE);
    }
}
void el_disable(void)
{
    if (check_slave_comm_ready(el_index)) {
        *control_word[el_index] &= (~ECAT_CTL_ENABLE);
    }
}
void piv_disable(void)
{
    if (check_slave_comm_ready(piv_index)) {
        *control_word[piv_index] &= (~ECAT_CTL_ENABLE);
    }
}

/**
 * Sets the emergency quick stop flag for each motor
 */
void rw_quick_stop(void)
{
    if (check_slave_comm_ready(rw_index)) {
        *control_word[rw_index] &= (~ECAT_CTL_QUICK_STOP);
    }
}
void el_quick_stop(void)
{
    if (check_slave_comm_ready(el_index)) {
        *control_word[el_index] &= (~ECAT_CTL_QUICK_STOP);
    }
}
void piv_quick_stop(void)
{
    if (check_slave_comm_ready(piv_index)) {
        *control_word[piv_index] &= (~ECAT_CTL_QUICK_STOP);
    }
}

/**
 * Sets 'Reset Fault' flag for each motor
 */
void rw_reset_fault(void)
{
    if (check_slave_comm_ready(rw_index)) {
        *control_word[rw_index] |= ECAT_CTL_RESET_FAULT;
    }
}
void el_reset_fault(void)
{
    if (check_slave_comm_ready(el_index)) {
        *control_word[el_index] |= ECAT_CTL_RESET_FAULT;
    }
}
void piv_reset_fault(void)
{
    if (check_slave_comm_ready(piv_index)) {
        *control_word[piv_index] |= ECAT_CTL_RESET_FAULT;
    }
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

static void ec_init_heartbeat(int slave_index)
{
    if (slave_index) {
        ec_SDOwrite16(slave_index, ECAT_HEARTBEAT_TIME, HEARTBEAT_MS);
        ec_SDOwrite32(slave_index, ECAT_LIFETIME_FACTOR, LIFETIME_FACTOR_EC);
    }
}

/**
 * Finds all motor controllers on the network and sets them to pre-operational state
 * @return -1 on error, number of controllers found otherwise
 */
static int find_controllers(void)
{
    blast_info("initial io_map pointer %p pointer to pointer %p", io_map, &io_map);
    ec_mcp_state.n_found = ec_config(false, &io_map);
    blast_info("after ec_config io_map pointer %p pointer to pointer %p", io_map, &io_map);
    if (ec_mcp_state.n_found <= 0) {
        berror(err, "No motor controller slaves found on the network!");
        goto find_err;
    }
    blast_startup("ec_config returns %d slaves found", ec_mcp_state.n_found);

    if (ec_mcp_state.n_found < (N_MCs -1))
        ec_mcp_state.status = ECAT_MOTOR_FOUND_PARTIAL;
    else
        ec_mcp_state.status = ECAT_MOTOR_FOUND;

    /* wait for all slaves to reach SAFE_OP state */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * (N_MCs - 1)) != EC_STATE_SAFE_OP) {
        ec_mcp_state.status = ECAT_MOTOR_RUNNING_PARTIAL;
        blast_err("Not all slaves reached safe operational state.");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                blast_err("Slave %d State=%2x StatusCode=%4x : %s", i, ec_slave[i].state,
                        ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    } else {
        ec_mcp_state.status = ECAT_MOTOR_RUNNING;
    }
    ec_mcp_state.slave_count = ec_slavecount;
    for (int i = 1; i <= ec_slavecount; i++) {
        controller_state[i].index = i;
        /**
         * There is only one PEPER FUCHS encoder on the chain, so we test for this
         * first.  It doesn't have a fixed index number like the motor controllers,
         * so we can't test in the usual manner
         */
        if (ec_slave[i].eep_man == FUCHS_MFG_ID) {
            int32_t serial = 0;
            int size = 4;
            ec_SDOread(i, 0x650B, 0, false, &size, &serial, EC_TIMEOUTRXM);
            blast_startup("PEPPERL+FUCHS encoder %d: %s: SN: %d",
                        ec_slave[i].aliasadr, ec_slave[i].name, serial);
            hwp_index = i;
            blast_info("ec_slave[%d].outputs = %p", i, ec_slave[i].outputs);
            controller_state[i].is_hwp = 1;
            break;
        }
        /**
         * Configure the index values for later use.  These are mapped to the hard-set
         * addresses on the motor controllers (look for the dials on the side)
         */
        int32_t serial = 0;
        int size = 4;
        ec_SDOread(i, 0x2384, 1, false, &size, &serial, EC_TIMEOUTRXM);
        if (serial == RW_SN) {
            blast_startup("Reaction Wheel Motor Controller %d: %s: SN: %4x",
                          ec_slave[i].aliasadr, ec_slave[i].name, serial);
            rw_index = i;
            blast_info("Setting rw_index to %d", rw_index);
             blast_info("ec_slave[%d].outputs = %p", i, ec_slave[i].outputs);
           controller_state[i].is_mc = 1;
        } else if (serial == PIV_SN) {
            blast_startup("Pivot Motor Controller %d: %s: SN: %4x",
                          ec_slave[i].aliasadr, ec_slave[i].name, serial);
            piv_index = i;
            blast_info("Setting piv_index to %d", piv_index);
            blast_info("ec_slave[%d].outputs = %p", i, ec_slave[i].outputs);
            controller_state[i].is_mc = 1;
        } else if (serial == EL_SN) {
            blast_startup("Elevation Motor Controller %d: %s: SN: %4x",
                          ec_slave[i].aliasadr, ec_slave[i].name, serial);
            el_index = i;
            controller_state[i].is_mc = 1;
            blast_info("Setting el_index to %d", el_index);
            blast_info("ec_slave[%d].outputs = %p", i, ec_slave[i].outputs);
        } else {
            blast_warn("Got unknown MC %s at position %d with alias %d",
                       ec_slave[i].name, ec_slave[i].configadr, ec_slave[i].aliasadr);
            controller_state[i].ec_unknown = 1;
        }
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
    return ec_slavecount;

find_err:
    return -1;
}
/**
 * Configure the HWP PDO assignments.
 */
static int hwp_pdo_init(void)
{
    pdo_mapping_t map;

    if (ec_slave[hwp_index].state != EC_STATE_SAFE_OP
            && ec_slave[hwp_index].state != EC_STATE_PRE_OP) {
        blast_err("Encoder index %d (%s) is not in pre-operational state!  Cannot configure.",
                hwp_index, ec_slave[hwp_index].name);
        return -1;
    }

     blast_startup("Configuring PDO Mappings for encoder index %d (%s)",
             hwp_index, ec_slave[hwp_index].name);

    /**
     * To program the PDO mapping, we first must clear the old state
     */

    if (!ec_SDOwrite8(hwp_index, ECAT_TXPDO_ASSIGNMENT, 0, 0)) blast_err("Failed mapping!");
    for (int i = 0; i < 4; i++) {
        if (!ec_SDOwrite8(hwp_index, ECAT_TXPDO_MAPPING + i, 0, 0)) blast_err("Failed mapping!");
    }

    /**
     * Define the PDOs that we want to send to the flight computer from the Controllers
     */

//       map_pdo(&map, ECAT_FUCHS_POSITION, 32);  // Motor Position
//       if (!ec_SDOwrite32(hwp_index, ECAT_TXPDO_MAPPING, 1, map.val)) blast_err("Failed mapping!");
//       map_pdo(&map, ECAT_FUCHS_OP_STATUS, 32);     // Actual Motor Velocity
//       if (!ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING, 2, map.val)) blast_err("Failed mapping!");
//
//      if (!ec_SDOwrite8(hwp_index, ECAT_TXPDO_MAPPING, 0, 2)) /// Set the 0x1a00 map to contain 1 elements
//          blast_err("Failed mapping!");
//      if (!ec_SDOwrite16(hwp_index, ECAT_TXPDO_ASSIGNMENT, 1, ECAT_TXPDO_MAPPING)) /// 0x1a00 maps to the first PDO
//          blast_err("Failed mapping!");

    return 0;
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
    int retval = 0;

    if (ec_slave[m_slave].state != EC_STATE_SAFE_OP && ec_slave[m_slave].state != EC_STATE_PRE_OP) {
        blast_err("Motor Controller %d (%s) is not in pre-operational state!  Cannot configure.",
                  m_slave, ec_slave[m_slave].name);
        return -1;
    }

    blast_startup("Configuring PDO Mappings for controller %d (%s)", m_slave, ec_slave[m_slave].name);
    blast_info("ec_slave[%d].outputs = %p", m_slave, ec_slave[m_slave].outputs);

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

    map_pdo(&map, ECAT_NET_STATUS, 16); // Network Status (including heartbeat monitor)
    retval = ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+1, 2, map.val);
    if (!retval) {
        blast_err("Failed mapping!");
    }
    blast_info("bytes written %i, %2x, map.val %d!", retval, ECAT_TXPDO_MAPPING+1, map.val);
    map_pdo(&map, ECAT_CTL_WORD, 16); // Control Word
    retval = ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+1, 3, map.val);
    if (!retval) {
        blast_err("Failed mapping!");
    }
    blast_info("bytes written %i, %2x, map.val %d!", retval, ECAT_TXPDO_MAPPING+1, map.val);

    if (!ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING+1, 0, 3)) /// Set the 0x1a01 map to contain 2 elements
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

    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    blast_info("ec_slave[%d].outputs = %p", m_slave, ec_slave[m_slave].outputs);

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

    ec_SDOwrite32(m_slave, 0x2420, 0, 8);           // MISC settings for aborting trajectory, saving PDO map
    ec_SDOwrite32(m_slave, 0x1010, 1, 0x65766173);  // Save all objects (the 0x65766173 is hex for 'save')

    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
    blast_info("ec_slave[%d].outputs = %p", m_slave, ec_slave[m_slave].outputs);
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
    GSList *test;
    test = pdo_list[m_index];
    /**
     * Inputs.  Each is sequentially mapped to the IOMap memory space
     * for the motor controller
     */
	blast_info("Starting map_index_vars for index %d", m_index);
    blast_info("Initial pdolist pointer: %p", test);
#define PDO_SEARCH_LIST(_obj, _map) { \
    found = false; \
    for (GSList *el = pdo_list[m_index]; (el); el = g_slist_next(el)) { \
        pdo_channel_map_t *ch = (pdo_channel_map_t*)el->data; \
        if (ch->index == object_index(_obj) && \
                ch->subindex == object_subindex(_obj)) { \
            _map[m_index] = (typeof(_map[0])) (ec_slave[m_index].inputs + ch->offset); \
            found = true; \
        } \
    } \
    if (!found) { \
        blast_err("Could not find PDO map for %s", #_map); \
    } else { \
    	blast_info("Found PDO map for %s", #_map); \
    } \
    }
    if (controller_state[m_index].is_mc) {
        PDO_SEARCH_LIST(ECAT_MOTOR_POSITION, motor_position);
        PDO_SEARCH_LIST(ECAT_VEL_ACTUAL, motor_velocity);
        PDO_SEARCH_LIST(ECAT_ACTUAL_POSITION, actual_position);
        PDO_SEARCH_LIST(ECAT_DRIVE_STATUS, status_register);
        PDO_SEARCH_LIST(ECAT_CTL_STATUS, status_word);
        PDO_SEARCH_LIST(ECAT_DRIVE_TEMP, amp_temp);
        PDO_SEARCH_LIST(ECAT_LATCHED_DRIVE_FAULT, latched_register);
        PDO_SEARCH_LIST(ECAT_CURRENT_ACTUAL, motor_current);
        PDO_SEARCH_LIST(ECAT_NET_STATUS, network_status_word);
        PDO_SEARCH_LIST(ECAT_CTL_WORD, control_word_read);
        while (ec_iserror()) {
            blast_err("%s", ec_elist2string());
        }
    } else if (controller_state[m_index].is_hwp) {
        PDO_SEARCH_LIST(ECAT_FUCHS_POSITION, hwp_position);
        while (ec_iserror()) {
            blast_err("%s", ec_elist2string());
        }
    }
#undef PDO_SEARCH_LIST

    // TODO(seth): Add dynamic mapping to outputs
    /// Outputs
    if (controller_state[m_index].is_mc) {
        control_word[m_index] = (uint16_t*) (ec_slave[m_index].outputs);
        target_current[m_index] = (int16_t*) (control_word[m_index] + 1);
        if (!(ec_slave[m_index].outputs)) {
            blast_err("Error: IOmap was not configured correctly!  Setting slave_error = 1 for slave %d...", m_index);
            controller_state[m_index].slave_error = 1;
        } else {
            controller_state[m_index].slave_error = 0;
        }
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}
/**
 * Interface function to @map_index_vars.  Maps the variables for each of the motor
 * controllers found on the bus.  If the motor controller is not found, its map remains
 * attached to the @dummy_var position
 */
static void map_motor_vars(void)
{
	blast_info("Starting map_motor_vars.");
    if (el_index) {
        blast_info("mapping el_motors to index: %d", el_index);
        map_index_vars(el_index);
    }
    if (rw_index) {
        blast_info("mapping rw_motors to index: %d", rw_index);
        map_index_vars(rw_index);
    }
    if (piv_index) {
        blast_info("mapping piv_motors to index: %d", piv_index);
        map_index_vars(piv_index);
    }
    if (hwp_index) {
        blast_info("mapping hwp to index: %d", hwp_index);
        map_index_vars(hwp_index);
        // hwp_position = (uint32_t*)ec_slave[hwp_index].inputs;
    }

    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
    blast_info("Finished map_motor_vars.");
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
    while (ec_iserror()) {
        blast_err("Error after ec_iserror(), %s", ec_elist2string());
    }
    for (int i = 1; i <= ec_slavecount; i++) {
        if (!found_dc_master && ec_slave[i].hasdc) {
            ec_dcsync0(i, true, ECAT_DC_CYCLE_NS, ec_slave[i].pdelay);
            found_dc_master = 1;
        } else {
            ec_dcsync0(i, false, ECAT_DC_CYCLE_NS, ec_slave[i].pdelay);
        }
        while (ec_iserror()) {
            blast_err("Slave %i, %s", i, ec_elist2string());
        }
        /**
         * Set the SYNC Manager mode to free-running so that we get data from the drive as soon as
         * available.  Otherwise, the data will be held until the SYNC0 time updates
         */
        ec_SDOwrite16(i, 0x1C32, 1, 0);
        while (ec_iserror()) {
            blast_err("Slave %i, %s", i, ec_elist2string());
        }
        blast_info("Slave %i, has_dc = %d, found_dc_master = %d", i, ec_slave[i].hasdc, found_dc_master);
        if (ec_slave[i].hasdc && found_dc_master) {
            controller_state[i].has_dc = 1;
            blast_info("setting has_dc to 1");
        } else {
            controller_state[i].has_dc = 0;
            blast_info("setting has_dc to 0");
        }
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
        blast_info("We have reached a fully operational state.");
        ec_mcp_state.status = ECAT_MOTOR_RUNNING;
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
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
    return -1;
}

static uint8_t check_ec_ready(int index)
{
    uint16_t m_state = 0;
    if ((index <= 0) || (index >= N_MCs)) {
        return(0);
    }
    if (!controller_state[index].comms_ok) {
        return(0);
    }
    if (controller_state[index].is_mc) {
        m_state = *status_word[index];
        if (!(m_state & ECAT_CTL_STATUS_READY)) {
            return(0);
        }
    }
    if (check_slave_comm_ready(index)) {
        if (!((*control_word_read[index]) == (*control_word[index]))) {
            return(0);
        }
    } else {
        return 0;
    }

    return(1);
}

// LMF note: Deprecated this doesn't seem to work in the new ethercat configuration.
static uint8_t check_for_network_problem(uint16_t net_status, bool firsttime)
{
    if (firsttime) {
        blast_info("net_status = %2x", net_status);
    }
    // Device is not in operational mode (bits 0 and 1 have value of 3)
    if (!(net_status & ECAT_NET_NODE_CHECK)) {
        if (firsttime) {
            blast_info("Device is not in operational mode.");
        }
        return(1);
    }
    if (!(net_status & ECAT_NET_SYNC_CHECK)) {
        if (firsttime) {
            blast_info("Device network error.");
        }
        return(1);
    }
    if (!(net_status & ECAT_NET_COBUS_OFF_CHECK)) {
        if (firsttime) {
            blast_info("Network CANOPEN Bus is off.");
        }
        return(1);
    }
    if (firsttime) {
        blast_info("No error in network status %u, returning 0.", net_status);
    }
    return(0);
}

// Checks to see whether we have we have communicated with the El Motor and it is returning
// a reasonable network status.
uint8_t is_el_motor_ready() {
    return(check_ec_ready(el_index));
//    return(!check_for_network_problem(ElevMotorData[el_index].network_status, 0));
}

uint8_t is_rw_motor_ready() {
    return(check_ec_ready(rw_index));
//    return(!check_for_network_problem(RWMotorData[rw_index].network_status, 0));
}

uint8_t is_pivot_motor_ready() {
    return(check_ec_ready(piv_index));
//    return(!check_for_network_problem(RWMotorData[piv_index].network_status, 0));
}

static void read_motor_data()
{
    int motor_i = motor_index;
    static bool firsttime = 1;
    RWMotorData[motor_i].current = rw_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    RWMotorData[motor_i].drive_info = rw_get_status_word();
    RWMotorData[motor_i].fault_reg = rw_get_latched();
    RWMotorData[motor_i].status = rw_get_status_register();
    RWMotorData[motor_i].network_status = rw_get_network_status_word();
    RWMotorData[motor_i].position = rw_get_position();
    RWMotorData[motor_i].motor_position = rw_get_position();
    RWMotorData[motor_i].temp = rw_get_amp_temp();
    RWMotorData[motor_i].velocity = rw_get_velocity();
    RWMotorData[motor_i].control_word_read = rw_get_ctl_word();
    RWMotorData[motor_i].control_word_write = rw_get_ctl_word_write();
    RWMotorData[motor_i].network_problem = !is_rw_motor_ready();

    ElevMotorData[motor_i].current = el_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    ElevMotorData[motor_i].drive_info = el_get_status_word();
    ElevMotorData[motor_i].fault_reg = el_get_latched();
    ElevMotorData[motor_i].status = el_get_status_register();
    ElevMotorData[motor_i].network_status = el_get_network_status_word();
    ElevMotorData[motor_i].position = el_get_position();
    ElevMotorData[motor_i].motor_position = el_get_motor_position();
    ElevMotorData[motor_i].temp = el_get_amp_temp();
    ElevMotorData[motor_i].velocity = el_get_velocity();
    ElevMotorData[motor_i].control_word_read = el_get_ctl_word();
    ElevMotorData[motor_i].control_word_write = el_get_ctl_word_write();
    ElevMotorData[motor_i].network_problem  = !is_el_motor_ready();

    PivotMotorData[motor_i].current = piv_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    PivotMotorData[motor_i].drive_info = piv_get_status_word();
    PivotMotorData[motor_i].fault_reg = piv_get_latched();
    PivotMotorData[motor_i].status = piv_get_status_register();
    PivotMotorData[motor_i].network_status = piv_get_network_status_word();
    PivotMotorData[motor_i].position = piv_get_position();
    PivotMotorData[motor_i].temp = piv_get_amp_temp();
    PivotMotorData[motor_i].velocity = piv_get_velocity();
    PivotMotorData[motor_i].control_word_read
     = piv_get_ctl_word();
    PivotMotorData[motor_i].control_word_write = piv_get_ctl_word_write();
    PivotMotorData[motor_i].network_problem  = !is_pivot_motor_ready();

    firsttime = 0;
    motor_index = INC_INDEX(motor_index);
}

void mc_readPDOassign(int m_slave) {
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
//    GSList *m_pdo_list;
    int wkc = 0;
    int len = 0;
    int offset = 0;

    len = sizeof(rdat);
    rdat = 0;

//    m_pdo_list = pdo_list[m_slave];
    blast_info("Starting mc_readPDOassign for slave %i", m_slave);
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(m_slave, ECAT_TXPDO_ASSIGNMENT, 0x00, FALSE, &len, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    blast_info("Result from ec_SDOread at index %04x, wkc = %i, len = %i, rdat = %04x",
               ECAT_TXPDO_ASSIGNMENT, wkc, len, rdat);
    if ((wkc <= 0) || (rdat <= 0))  {
    	blast_info("no data returned from ec_SDOread ... returning.");
    	return;
    }

    /* number of available sub indexes */
    nidx = rdat;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++) {
        len = sizeof(rdat);
        rdat = 0;
        /* read PDO assign */
        wkc = ec_SDOread(m_slave, ECAT_TXPDO_ASSIGNMENT, (uint8) idxloop, FALSE, &len, &rdat, EC_TIMEOUTRXM);
        /* result is index of PDO */
        idx = etohl(rdat);
        if (idx <= 0) {
        	continue;
        } else {
            blast_info("found idx = %2x at wkc = %i, idxloop = %i", idx, wkc, idxloop);
        }
        len = sizeof(subcnt);
        subcnt = 0;
        /* read number of subindexes of PDO */
        wkc = ec_SDOread(m_slave, idx, 0x00, FALSE, &len, &subcnt, EC_TIMEOUTRXM);
        subidx = subcnt;
        blast_info("Number of subindexes: %i", subidx);
        /* for each subindex */
        blast_info("Reading out the SDOs");
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++) {
            pdo_channel_map_t *channel = NULL;
            pdo_mapping_t pdo_map = { 0 };
            len = sizeof(pdo_map);
            /* read SDO that is mapped in PDO */
            wkc = ec_SDOread(m_slave, idx, (uint8) subidxloop, FALSE, &len, &pdo_map, EC_TIMEOUTRXM);
            channel = malloc(sizeof(pdo_channel_map_t));
            channel->index = pdo_map.index;
            channel->subindex = pdo_map.subindex;
            channel->offset = offset;
            pdo_list[m_slave] = g_slist_prepend(pdo_list[m_slave], channel);
            blast_info("Read SDO subidxloop = %i, wkc = %i, idx = %i, len = %i", subidxloop, wkc, idx, len);
            blast_info("Appending channel to m_pdo_list = %p: index = %2x, subindex = %i, offset = %i",
                       pdo_list[m_slave], channel->index, channel->subindex, channel->offset);

            /// Offset is the number of bytes into the memory map this element is.  First element is 0 bytes in.
            offset += (pdo_map.size / 8);
        }
    }
}

// Set defaults for the current limits, and pids
void set_ec_motor_defaults()
{
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

    /// Set up the heartbeat which tracks communication errors with the slaves
    ec_init_heartbeat(rw_index);
    ec_init_heartbeat(el_index);
    ec_init_heartbeat(piv_index);
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}

int close_ec_motors()
{
    ec_close(); // Attempt to close down the motors
    return(1);
}
// Attempts to connect to the EtherCat devices.
int configure_ec_motors()
{
    find_controllers();

    for (int i = 1; i <= ec_slavecount; i++) {
        if (controller_state[i].is_hwp) {
            // hwp_pdo_init();
            mc_readPDOassign(i);
        } else if (controller_state[i].is_mc) {
            motor_pdo_init(i);
            mc_readPDOassign(i);
        }
    }
    /// We re-configure the map now that we have assigned the PDOs
    blast_info("Reconfigure the map now that we have assigned the PDOs");
    if (ec_config_map(&io_map) <= 0) blast_warn("Warning ec_config_map(&io_map) return null map size.");
    map_motor_vars();

    /**
     * Set the initial values of both commands to "safe" default values
     */
    for (int i = 1; i <= ec_slavecount; i++) {
        if ((controller_state[i].is_mc) && !(controller_state[i].slave_error)) {
            blast_info("Initializing target current for ec_slave %d, ptr %p", i, target_current);
            *target_current[i] = 0;
            *control_word[i] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
        }
    }

    if (CommandData.disable_az) {
        rw_disable();
        piv_disable();
    } else {
        rw_enable();
        piv_enable();
    }
    if (CommandData.disable_el) {
        el_disable();
    } else {
        el_enable();
    }

    set_ec_motor_defaults();
    /// Start the Distributed Clock cycle
    motor_configure_timing();

    /// Put the motors in Operational mode (EtherCAT Operation)
    blast_info("Setting the EtherCAT devices in operational mode.");
    motor_set_operational();

    for (int i = 1; i <= ec_slavecount; i++) {
        if ((controller_state[i].slave_error == 0) && (controller_state[i].has_dc == 1)) {
            controller_state[i].comms_ok = 1;
            blast_info("Setting comms_ok to 1 for index %d, slave_error = %d, has_dc = %d",
                i, controller_state[i].slave_error , controller_state[i].has_dc);
        } else {
            controller_state[i].comms_ok = 0;
            blast_info("Setting comms_ok to 0 for index %d, slave_error = %d, has_dc = %d",
                i, controller_state[i].slave_error , controller_state[i].has_dc);
        }
        if (controller_state[i].is_mc) {
            ec_SDOwrite16(i, ECAT_DRIVE_STATE, ECAT_DRIVE_STATE_PROG_CURRENT);
        }
    }
    return(1);
}


int reset_ec_motors()
{
    int i = 0;
    rw_index = 0;
    el_index = 0;
    piv_index = 0;
    hwp_index = 0;
    for (i = 1; i < N_MCs; i++) {
        controller_state[i].index = 0;
        controller_state[i].is_hwp = 0;
        controller_state[i].ec_unknown = 0;
        controller_state[i].is_mc = 0;
        controller_state[i].has_dc = 0;
        controller_state[i].comms_ok = 0;
        controller_state[i].slave_error = 0;
    }
    configure_ec_motors();
    return(1);
}

static int check_ec_network_status()
{
    static bool has_warned = 0;
    int retval = 1;
    int motor_i = motor_index;
    if ((!check_ec_ready(rw_index) && CommandData.ec_devices.fix_rw) ||
        (!check_ec_ready(el_index) && CommandData.ec_devices.fix_el) ||
        (!check_ec_ready(piv_index) && CommandData.ec_devices.fix_piv)) {
            retval = 0;
            if (!has_warned) {
                blast_warn("check_ec_network_status shows a network problem. RW = %u, El = %u, Piv = %u",
                           RWMotorData[motor_i].network_problem, ElevMotorData[motor_i].network_problem,
                           PivotMotorData[motor_i].network_problem);
                has_warned = 1;
            }
    } else {
       has_warned = 0;
    }
    return(retval);
}

void shutdown_motors(void)
{
    blast_info("Shutting down motors");
    close_ec_motors();
    blast_info("Finished shutting down motors");
}

static void* motor_control(void* arg)
{
    int expectedWKC, wkc;
    int ret;
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 2000000}; /// 500HZ interval
    char name[16] = "eth1";
	bool firsttime = 1;

    nameThread("Motors");
	while (!InCharge) {
		usleep(100000);
		if (firsttime) {
			blast_info("Not in charge.  Waiting for control.");
			firsttime = 0;
		}
	}
    blast_startup("Starting Motor Control");

    ph_thread_set_name("Motors");

    if (!(ret = ec_init(name))) {
        berror(err, "Could not initialize %s, exiting.", name);
        exit(0);
    } else {
    	blast_info("Initialized %s", name);
    }
    blast_info("Attempting configure to EtherCat devices.");
    configure_ec_motors();

    /// Our work counter (WKC) provides a count of the number of items to handle.
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    blast_info("expectedWKC = %i", expectedWKC);

    clock_gettime(CLOCK_REALTIME, &ts);
    while (!shutdown_mcp) {
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (!check_ec_network_status()) {
            ec_mcp_state.network_error_count++;
            if (((ec_mcp_state.network_error_count) % 1000) == 1) {
                blast_info("network_error_count = %u", ec_mcp_state.network_error_count);
            }
            if (ec_mcp_state.network_error_count >= NETWORK_ERR_RESET_THRESH) {
                ec_mcp_state.network_error_count = 0;
                if (!reset_ec_motors()) {
                    blast_err("Reset of EtherCat devices failed!");
                } else {
                    blast_info("Reset EtherCat connection.");
                }
            }
        }
        if (CommandData.ec_devices.reset) {
            if (!reset_ec_motors()) blast_err("Reset of EtherCat devices failed!");
            CommandData.ec_devices.reset = 0;
        }

        if (CommandData.disable_az) {
            rw_disable();
            piv_disable();
        } else {
            rw_enable();
            piv_enable();
        }
        if (CommandData.disable_el) {
            el_disable();
        } else {
            el_enable();
        }

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
    shutdown_motors();

    return 0;
}

/* opens communications with motor controllers */
int initialize_motors(void)
{
    memset(ElevMotorData, 0, sizeof(ElevMotorData));
    memset(RWMotorData, 0, sizeof(RWMotorData));
    memset(PivotMotorData, 0, sizeof(PivotMotorData));

    motor_ctl_id =  ph_thread_spawn(motor_control, NULL);
    return 0;
}

uint8_t make_ec_status_field(int m_index)
{
    uint8_t m_stats = 0;
    if ((m_index < 1) || (m_index >= N_MCs)) return m_stats;
    m_stats |= (m_index & 0x07);
    m_stats |= ((controller_state[m_index].comms_ok & 0x01) << 3);
    m_stats |= ((controller_state[m_index].slave_error & 0x01) << 4);
    m_stats |= ((controller_state[m_index].has_dc & 0x01) << 5);
    m_stats |= ((controller_state[m_index].is_mc & 0x01) << 6);
    m_stats |= ((controller_state[m_index].is_hwp & 0x01) << 7);
    return m_stats;
}
// Called in store_1hz_acs of acs.c
void store_1hz_ethercat(void)
{
    static int firsttime = 1;
    static channel_t *NFoundECAddr;
    static channel_t *SlaveCountECAddr;
    static channel_t *StatusECAddr;
    static channel_t *StatusECHWPRAddr;
    static channel_t *StatusECRWAddr;
    static channel_t *StatusECElAddr;
    static channel_t *StatusECPivAddr;

    if (firsttime) {
        NFoundECAddr = channels_find_by_name("n_found_ec");
        SlaveCountECAddr = channels_find_by_name("slave_count_ec");
        StatusECAddr = channels_find_by_name("status_ec");
        StatusECHWPRAddr = channels_find_by_name("status_ec_hwpr");
        StatusECRWAddr = channels_find_by_name("status_ec_rw");
        StatusECElAddr = channels_find_by_name("status_ec_el");
        StatusECPivAddr = channels_find_by_name("status_ec_piv");
        firsttime = 0;
    }
    SET_UINT8(NFoundECAddr, ec_mcp_state.n_found);
    SET_UINT8(SlaveCountECAddr, ec_mcp_state.slave_count);
    SET_UINT8(StatusECAddr, ec_mcp_state.status);
    if (hwp_index) SET_UINT8(StatusECHWPRAddr, make_ec_status_field(hwp_index));
    if (rw_index) SET_UINT8(StatusECRWAddr, make_ec_status_field(rw_index));
    if (el_index) SET_UINT8(StatusECElAddr, make_ec_status_field(el_index));
    if (piv_index) SET_UINT8(StatusECPivAddr, make_ec_status_field(piv_index));
}

