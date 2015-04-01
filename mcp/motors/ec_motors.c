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



#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <pthread.h>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercattype.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>

#include <blast_time.h>
#include <ec_motors.h>
#include <motors.h>
#include <mcp.h>

static pthread_t motor_ctl_id;

// device node Serial Numbers
#define RW_SN 0x01bbbb5b
#define PIV_SN 0x02924687
#define EL_SN 0x01238408
#define RW_ADDR 0x1001
#define EL_ADDR 0x1002
#define PIV_ADDR 0x1003

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
static char io_map[4096];

static int motors_exit = false;

#define N_MCs 3

/**
 * The following pointers reference data points read from/written to
 * PDOs for each motor controller.  Modifying the data at the address for the write
 * word will cause that change to be transmitted to the motor controller at
 * the next PDO cycle (0.5ms)
 */
static int32_t dummy_var = 0;

/// Read words
static int32_t *motor_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var };
static int32_t *motor_velocity[N_MCs] = { &dummy_var, &dummy_var, &dummy_var };
static int32_t *enc_velocity[N_MCs] = { &dummy_var, &dummy_var, &dummy_var };
static int16_t *motor_current[N_MCs] = { (int16_t*)&dummy_var, (int16_t*)&dummy_var, (int16_t*)&dummy_var };
static uint32_t *status_register[N_MCs] = { (uint32_t*)&dummy_var, (uint32_t*)&dummy_var, (uint32_t*)&dummy_var };
static int16_t *amp_temp[N_MCs] = { (int16_t*)&dummy_var, (int16_t*)&dummy_var, (int16_t*)&dummy_var };
static uint16_t *status_word[N_MCs] = { (uint16_t*)&dummy_var, (uint16_t*)&dummy_var, (uint16_t*)&dummy_var };

/// Write words
static uint16_t *control_word[N_MCs] = { (uint16_t*)&dummy_var, (uint16_t*)&dummy_var, (uint16_t*)&dummy_var };
static int16_t *target_current[N_MCs] = { (int16_t*)&dummy_var, (int16_t*)&dummy_var, (int16_t*)&dummy_var };
static uint16_t *current_p[N_MCs] = { (uint16_t*)&dummy_var, (uint16_t*)&dummy_var, (uint16_t*)&dummy_var };
static uint16_t *current_i[N_MCs] = { (uint16_t*)&dummy_var, (uint16_t*)&dummy_var, (uint16_t*)&dummy_var };
static int16_t *current_offset[N_MCs] = { (int16_t*)&dummy_var, (int16_t*)&dummy_var, (int16_t*)&dummy_var };

/**
 * This set of functions return the absolute position read by each motor controller
 * @return int32 value of the position (modulo the wrap position value)
 */
int32_t rw_get_position(void)
{
    return *motor_position[rw_index];
}
int32_t el_get_position(void)
{
    return *motor_position[el_index];
}
int32_t piv_get_position(void)
{
    return *motor_position[piv_index];
}

/**
 * This set of functions return the calculated motor velocity of each motor
 * controller
 * @return int32 value of the velocity in counts per second
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
 * This set of functions return the external encoder velocity of each motor
 * controller
 * @return int32 value of the velocity in counts per second
 */
int32_t rw_get_enc_velocity(void)
{
    return *enc_velocity[rw_index];
}
int32_t el_get_enc_velocity(void)
{
    return *enc_velocity[el_index];
}
int32_t piv_get_enc_velocity(void)
{
    return *enc_velocity[piv_index];
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
    return *motor_current[el_index];
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
 * @param m_current int32 requested current in units of 0.1 encoder counts / second
 */
void rw_set_current(int16_t m_cur)
{
    *target_current[rw_index] = m_cur;
}
void el_set_current(int16_t m_cur)
{
    *target_current[el_index] = m_cur;
}
void piv_set_current(int16_t m_cur)
{
    *target_current[piv_index] = m_cur;
}

/**
 * Sets the current Proportional Gain
 * @param m_p uint16 Gain
 */
void rw_set_p(int16_t m_p)
{
    *current_p[rw_index] = m_p;
}
void el_set_p(int16_t m_p)
{
    *current_p[el_index] = m_p;
}
void piv_set_p(int16_t m_p)
{
    *current_p[piv_index] = m_p;
}

/**
 * Sets the current Integral Gain
 * @param m_i uint16 Gain
 */
void rw_set_i(int16_t m_i)
{
    *current_i[rw_index] = m_i;
}
void el_set_i(int16_t m_i)
{
    *current_i[el_index] = m_i;
}
void piv_set_i(int16_t m_i)
{
    *current_i[piv_index] = m_i;
}

/**
 * Sets the current Integral Gain
 * @param m_offset uint16 Gain
 */
void rw_set_offset(int16_t m_offset)
{
    *current_offset[rw_index] = m_offset;
}
void el_set_offset(int16_t m_offset)
{
    *current_offset[el_index] = m_offset;
}
void piv_set_offset(int16_t m_offset)
{
    *current_offset[piv_index] = m_offset;
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
 * Finds all motor controllers on the network and sets them to pre-operational state
 * @return -1 on error, number of controllers found otherwise
 */
static int find_controllers(void)
{
    char name[16] = "eth0";
    int ret_init;
    int ret_config;

    if (controller_state == ECAT_MOTOR_COLD)
    {
        if (!(ret_init = ec_init(name))) {
            berror(err, "Could not initialize %s", name);
            goto find_err;
        }
    }

    controller_state = ECAT_MOTOR_INIT;
    if (!(ret_config = ec_config(false, &io_map))) {
        berror(err, "No motor controller slaves found on the network!");
        goto find_err;
    }
    bprintf(startup, "ec_config returns %d slaves found", ret_config);

    if (ret_config < 3) controller_state = ECAT_MOTOR_FOUND_PARTIAL;
    else controller_state = ECAT_MOTOR_FOUND;

    /* wait for all slaves to reach SAFE_OP state */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3) != EC_STATE_SAFE_OP) {
        controller_state = ECAT_MOTOR_RUNNING_PARTIAL;
        bprintf(err, "Not all slaves reached safe operational state.");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                bprintf(err, "Slave %d State=%2x StatusCode=%4x : %s", i, ec_slave[i].state,
                        ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }
    else {
        controller_state = ECAT_MOTOR_RUNNING;
    }

    for (int i = 1; i <= ec_slavecount; i++) {
        bprintf(startup, "Motor Controller %d: %s", i, ec_slave[i].name);

        /**
         * Configure the index values for later use.  These are mapped to the hard-set
         * addresses on the motor controllers (look for the dials on the side)
         */
        if (ec_slave[i].configadr == RW_ADDR) rw_index = i;
        if (ec_slave[i].configadr == PIV_ADDR) piv_index = i;
        if (ec_slave[i].configadr == EL_ADDR) el_index = i;
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
        bprintf(err, "Motor Controller %d (%s) is not in pre-operational state!  Cannot configure.", m_slave, ec_slave[m_slave].name);
        return -1;
    }

    bprintf(startup, "Configuring PDO Mappings for controller %d (%s)", m_slave, ec_slave[m_slave].name);

    /**
     * To program the PDO mapping, we first must clear the old state
     */

    ec_SDOwrite8(m_slave, ECAT_TXPDO_ASSIGNMENT, 0, 0);
    for (int i = 0; i < 4; i++) ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING + i, 0, 0);

    /**
     * Define the PDOs that we want to send to the flight computer from the Controllers
     */
    map_pdo(&map, ECAT_MOTOR_POSITION,32);  // Motor Position
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING, 1, map.val);

    map_pdo(&map, ECAT_VEL_ACTUAL, 32);     // Motor Velocity (internal)
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING, 2, map.val);
    ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING, 0, 2); /// Set the 0x1a00 map to contain 2 elements
    ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT, 1, ECAT_TXPDO_MAPPING); /// Set the 0x1a00 map to the first PDO

    /**
     * Second map (0x1a01 register)
     */
    map_pdo(&map, ECAT_VEL_ENCODER, 32);    // Encoder Velocity
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+1, 1, map.val);

    map_pdo(&map, ECAT_CURRENT_ACTUAL, 16); // Measured current output
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+1, 2, map.val);

    ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING+1, 0, 2); /// Set the 0x1a01 map to contain 2 elements
    ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT+1, 2, ECAT_TXPDO_MAPPING); /// Set the 0x1a01 map to the second PDO

    /**
     * Third map (0x1a02 register)
     */
    map_pdo(&map, ECAT_DRIVE_STATUS, 32); // Status Register
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+2, 1, map.val);

    map_pdo(&map, ECAT_CTL_STATUS, 16); // Status Word
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+2, 2, map.val);

    map_pdo(&map, ECAT_DRIVE_TEMP, 16); // Amplifier Temp (deg C)
    ec_SDOwrite32(m_slave, ECAT_TXPDO_MAPPING+2, 3, map.val);

    ec_SDOwrite8(m_slave, ECAT_TXPDO_MAPPING+2, 0, 3); /// Set the 0x1a01 map to contain 3 elements
    ec_SDOwrite16(m_slave, ECAT_TXPDO_ASSIGNMENT+2, 3, ECAT_TXPDO_MAPPING); /// Set the 0x1a02 map to the third PDO

    ec_SDOwrite8(m_slave, ECAT_TXPDO_ASSIGNMENT, 0, 3); /// There are three maps in the TX PDOs


    /**
     * To program the PDO mapping, we first must clear the old state
     */
    ec_SDOwrite8(m_slave, ECAT_RXPDO_ASSIGNMENT, 0, 0);
    for (int i = 0; i < 4; i++) ec_SDOwrite8(m_slave, ECAT_RXPDO_MAPPING + i, 0, 0);
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

    /**
     * Second map (0x1601 register)
     */
    map_pdo(&map, ECAT_CURRENT_LOOP_CP, 16); // current Loop Proportional Gain (P term)
    ec_SDOwrite32(m_slave, ECAT_RXPDO_MAPPING + 1, 1, map.val);

    map_pdo(&map, ECAT_CURRENT_LOOP_CI, 16); // Current Loop Integral Gain (I term)
    ec_SDOwrite32(m_slave, ECAT_RXPDO_MAPPING + 1, 2, map.val);

    map_pdo(&map, ECAT_CURRENT_LOOP_OFFSET, 16); // Current Offset (int16)
    ec_SDOwrite32(m_slave, ECAT_RXPDO_MAPPING + 1, 3, map.val);

    ec_SDOwrite8(m_slave, ECAT_RXPDO_MAPPING + 1, 0, 3); /// Set the 0x1601 map to contain 3 elements
    ec_SDOwrite16(m_slave, ECAT_RXPDO_ASSIGNMENT, 2, ECAT_RXPDO_MAPPING + 1); /// Set the 0x1601 map to the second PDO


    ec_SDOwrite8(m_slave, ECAT_RXPDO_ASSIGNMENT, 0, 3); /// There are three maps in the RX PDOs


    return 0;
}

/**
 * Generic mapping function for ethercat slave PDO variables.  This function works with
 * motor_pdo_init to set up the PDO input/output memory mappings for the motor controllers
 * @param m_index position on the ethercat chain
 */
static void map_index_vars(int m_index)
{
    /**
     * Inputs.  Each is sequentially mapped to the IOMap memory space
     * for the motor controller
     */
    motor_position[m_index] = (int32_t*) (ec_slave[m_index].inputs);
    motor_velocity[m_index] = (int32_t*) (motor_position[m_index] + 1);
    enc_velocity[m_index] = (int32_t*) (motor_velocity[m_index] + 1);
    motor_current[m_index] = (int16_t*) (enc_velocity[m_index] + 1);
    status_register[m_index] = (uint32_t*) (motor_current[m_index] + 1);
    status_word[m_index] = (uint16_t*) (status_register[m_index] + 1);
    amp_temp[m_index] = (int16_t*) (status_word[m_index] + 1);

    /// Outputs
    control_word[m_index] = (uint16_t*) (ec_slave[m_index].outputs);
    target_current[m_index] = (int16_t*) (control_word[m_index] + 1);
    current_p[m_index] = (uint16_t*) (target_current[m_index] + 1);
    current_i[m_index] = (uint16_t*) (current_p[m_index] + 1);
    current_offset[m_index] = (int16_t*) (current_i[m_index] + 1);
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
        }
        else {
            ec_dcsync0(i, false, ECAT_DC_CYCLE_NS, ec_slave[i].pdelay);
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
        controller_state = ECAT_MOTOR_RUNNING;
        return 0;
    }

    /**
     * Something has prevented a motor controller from entering operational mode (EtherCAT Ops)
     */
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
            bprintf(err, "Slave %d State=%2x StatusCode=%4x : %s", i, ec_slave[i].state,
                    ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
    }
    return -1;
}

static void write_motor_data()
{
    int motor_i = motor_index;

    RWMotorData[motor_i].current = rw_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    RWMotorData[motor_i].drive_info = rw_get_status_word();
    RWMotorData[motor_i].fault_reg = 0; ///TODO:Consider whether to map the latching register
    RWMotorData[motor_i].status = rw_get_status_register();
    RWMotorData[motor_i].position = rw_get_position(); ///TODO:Add split between resolver and internal motor pos
    RWMotorData[motor_i].temp = rw_get_amp_temp();
    RWMotorData[motor_i].velocity = rw_get_velocity();

    ElevMotorData[motor_i].current = el_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    ElevMotorData[motor_i].drive_info = el_get_status_word();
    ElevMotorData[motor_i].fault_reg = 0; ///TODO:Consider whether to map the latching register
    ElevMotorData[motor_i].status = el_get_status_register();
    ElevMotorData[motor_i].position = el_get_position(); ///TODO:Add split between resolver and internal motor pos
    ElevMotorData[motor_i].temp = el_get_amp_temp();
    ElevMotorData[motor_i].velocity = el_get_velocity();

    PivotMotorData[motor_i].current = piv_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    PivotMotorData[motor_i].drive_info = piv_get_status_word();
    PivotMotorData[motor_i].fault_reg = 0; ///TODO:Consider whether to map the latching register
    PivotMotorData[motor_i].status = piv_get_status_register();
    PivotMotorData[motor_i].position = piv_get_position(); ///TODO:Add split between resolver and internal motor pos
    PivotMotorData[motor_i].temp = piv_get_amp_temp();
    PivotMotorData[motor_i].velocity = piv_get_velocity();

}

static void* motor_control(void* arg)
{
    int expectedWKC, wkc;
    int ret, len;
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 2000000}; /// 500HZ interval

    find_controllers();

    for (int i = 1; i <= ec_slavecount; i++) {
        motor_pdo_init(i);
    }
    /// We re-configure the map now that we have assigned the PDOs
    ec_config_map(&io_map);
    map_motor_vars();

    /**
     * Get the current value of each RX word to avoid stomping on the current state
     */
    for (int i = 1; i <= ec_slavecount; i++) {
        len = 4;
        ec_SDOread(i, ECAT_VEL_CMD, false, &len, target_current[i], EC_TIMEOUTRXM);
        len = 2;
        ec_SDOread(i, ECAT_CTL_WORD, false, &len, control_word[i], EC_TIMEOUTRXM);
        ec_SDOread(i, ECAT_CURRENT_LOOP_CP, false, &len, current_p[i], EC_TIMEOUTRXM);
        ec_SDOread(i, ECAT_CURRENT_LOOP_CI, false, &len, current_i[i], EC_TIMEOUTRXM);
        ec_SDOread(i, ECAT_CURRENT_LOOP_OFFSET, false, &len, current_offset[i], EC_TIMEOUTRXM);
    }

    /// Start the Distributed Clock cycle
    motor_configure_timing();

    /// Put the motors in Operational mode (EtherCAT Operation)
    motor_set_operational();

    /// Our work counter (WKC) provides a count of the number of items to handle.
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

    clock_gettime(CLOCK_REALTIME, &ts);
    while (!motors_exit) {

        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (ret && ret != -EINTR)
        {
            bprintf(err, "error while sleeping, code %d (%s)\n", ret, strerror(-ret));
            break;
        }

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < expectedWKC) bprintf(warning, "Possible missing data in communicating with Motor Controllers");
        write_motor_data();
    }

    return 0;
}


/* opens communications with motor controllers */
void initialize_motors(void)
{
  bprintf(info, "Motors: connecting to motors");
  memset(ElevMotorData, 0, sizeof(ElevMotorData));
  memset(ElevMotorData, 0, sizeof(RWMotorData));
  memset(ElevMotorData, 0, sizeof(PivotMotorData));

  pthread_create(&motor_ctl_id, NULL, &motor_control, NULL);
  pthread_detach(motor_ctl_id);

}

void shutdown_motors(void)
{

}
