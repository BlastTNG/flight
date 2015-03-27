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

#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <pthread.h>

#include <ethercattype.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>

#include <ec_motors.h>

static pthread_t motor_ctl_id;

// device node Serial Numbers
#define RW_SN 0x01bbbb5b
#define PIV_SN 0x02924687
#define EL_SN 0x01234

/**
 * Index numbers for the slave array.  0 is the master (flight computer)
 */
static int rw_index = 0;
static int piv_index = 0;
static int el_index = 0;

/**
 * Ethercat driver status
 */
static ec_motor_state_t controller_state = EC_MOTOR_COLD;

/**
 * Memory mapping for the PDO variables
 */
static char io_map[4096];

static int motors_exit = false;

/**
 * Finds all motor controllers on the network and sets them to pre-operational state
 * @return -1 on error, number of controllers found otherwise
 */
static int find_controllers(void)
{
    char name[16] = "eth0";
    int ret_init;
    int ret_config;

    if (controller_state == EC_MOTOR_COLD)
    {
        if (!(ret_init = ec_init(name))) {
            berror(err, "Could not initialize %s");
            goto find_err;
        }
    }

    controller_state = EC_MOTOR_INIT;
    if (!(ret_config = ec_config(false, &io_map))) {
        berror(err, "No motor controller slaves found on the network!");
        goto find_err;
    }
    bprintf(startup, "ec_config returns %d slaves found", ret_config);

    if (ret_config < 3) controller_state = EC_MOTOR_FOUND_PARTIAL;
    else controller_state = EC_MOTOR_FOUND;

    /* wait for all slaves to reach SAFE_OP state */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3) != EC_STATE_SAFE_OP) {
        controller_state = EC_MOTOR_RUNNING_PARTIAL;
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
        controller_state = EC_MOTOR_RUNNING;
    }

    for (int i = 1; i <= ec_slavecount; i++) {
        bprintf(startup, "Motor Controller %d: %s", i, ec_slave[i].name);
    }
    return ec_slavecount;

find_err:
    return -1;
}

static int motor_pdo_init(int m_slave)
{
    int len;
    pdo_mapping_t map;

    if (ec_slave[m_slave].state != EC_STATE_SAFE_OP && ec_slave[m_slave].state != EC_STATE_PRE_OP) {
        bprintf(err, "Motor Controller %d (%s) is not in pre-operational state!  Cannot configure.", m_slave, ec_slave[m_slave].name);
        return -1;
    }

    bprintf(startup, "Configuring PDO Mappings for controller %d (%s)", m_slave, ec_slave[m_slave].name);

    /**
     * To program the PDO mapping, we first must clear the old state
     */

    ec_SDOwrite8(m_slave, EC_TXPDO_ASSIGNMENT, 0, 0);
    for (int i = 0; i < 4; i++) ec_SDOwrite8(m_slave, EC_TXPDO_MAPPING + i, 0, 0);

    /**
     * Define the PDOs that we want to send to the flight computer from the Controllers
     */
    map.index = 0x6063; // Actual position (counts)
    map.subindex = 0;
    map.size = 32;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING, 1, map.val);

    map.index = 0x6069; // Actual velocity (0.1 counts/sec)
    map.subindex = 0;
    map.size = 32;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING, 2, map.val);
    ec_SDOwrite8(m_slave, EC_TXPDO_MAPPING, 0, 2); /// Set the 0x1a00 map to contain 2 elements
    ec_SDOwrite16(m_slave, EC_TXPDO_ASSIGNMENT, 1, EC_TXPDO_MAPPING); /// Set the 0x1a00 map to the first PDO

    /**
     * Second map (0x1a01 register)
     */
    map.index = 0x6077; // Actual torque (rated torque/1000)
    map.subindex = 0;
    map.size = 16;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING+1, 1, map.val);

    map.index = 0x6040; // Actual current (0.01A)
    map.subindex = 0;
    map.size = 16;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING+1, 2, map.val);
    ec_SDOwrite8(m_slave, EC_TXPDO_MAPPING+1, 0, 2); /// Set the 0x1a01 map to contain 2 elements
    ec_SDOwrite16(m_slave, EC_TXPDO_ASSIGNMENT+1, 2, EC_TXPDO_MAPPING); /// Set the 0x1a01 map to the second PDO

    /**
     * Third map (0x1a02 register)
     */
    map.index = 0x1002; // Status Register
    map.subindex = 0;
    map.size = 32;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING+2, 1, map.val);

    map.index = 0x6041; // Status Word
    map.subindex = 0;
    map.size = 16;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING+2, 2, map.val);

    map.index = 0x6041; // Amplifier Temp (deg C)
    map.subindex = 0;
    map.size = 16;
    ec_SDOwrite32(m_slave, EC_TXPDO_MAPPING+2, 3, map.val);
    ec_SDOwrite8(m_slave, EC_TXPDO_MAPPING+2, 0, 3); /// Set the 0x1a01 map to contain 3 elements
    ec_SDOwrite16(m_slave, EC_TXPDO_ASSIGNMENT+2, 3, EC_TXPDO_MAPPING); /// Set the 0x1a02 map to the third PDO

    ec_SDOwrite8(m_slave, EC_TXPDO_ASSIGNMENT, 0, 3); /// There are three maps in the TX PDOs


    /**
     * To program the PDO mapping, we first must clear the old state
     */
    ec_SDOwrite8(m_slave, EC_RXPDO_ASSIGNMENT, 0, 0);
    for (int i = 0; i < 4; i++) ec_SDOwrite8(m_slave, EC_RXPDO_MAPPING + i, 0, 0);
    /**
     * Define the PDOs that we want to send from the flight computer to the Controllers
     */

    map.index = 0x6040; // Control Word
    map.subindex = 0;
    map.size = 16;
    ec_SDOwrite32(m_slave, EC_RXPDO_MAPPING, 1, map.val);

    map.index = 0x6071; // Target Torque
    map.subindex = 0;
    map.size = 16;
    ec_SDOwrite32(m_slave, EC_RXPDO_MAPPING, 2, map.val);
    ec_SDOwrite8(m_slave, EC_RXPDO_MAPPING, 0, 2); /// Set the 0x1600 map to contain 2 elements
    ec_SDOwrite16(m_slave, EC_RXPDO_ASSIGNMENT, 1, EC_RXPDO_MAPPING); /// Set the 0x1600 map to the first PDO

    /**
     * Get the current value of each RX word to avoid stomping on the current state
     */
    len = 2;
    ec_SDOread(m_slave, 0x6040, 0, false, &len, &ec_slave[m_slave].outputs, EC_TIMEOUTRXM);
    ec_SDOread(m_slave, 0x6071, 0, false, &len, &ec_slave[m_slave].outputs + 2, EC_TIMEOUTRXM);

    return 0;
}

static void motor_configure_timing(void)
{
    int found_dc_master = 0;
    for (int i = 1; i <= ec_slavecount; i++) {
        if (!found_dc_master && ec_slave[i].hasdc) {
            ec_dcsync0(i, true, EC_DC_CYCLE_NS, ec_slave[i].pdelay);
        }
        else {
            ec_dcsync0(i, false, EC_DC_CYCLE_NS, ec_slave[i].pdelay);
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
        controller_state = EC_MOTOR_RUNNING;
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

static void* motor_control(void* arg)
{
    find_controllers();

    /* configure DC options for every DC capable slave found in the list */
    bprintf(startup, "DC capable Motor Controllers : %d\n",ec_configdc());

    for (int i = 1; i <= ec_slavecount; i++) {
        motor_pdo_init(i);
    }
    /// We re-configure the map now that we have assigned the PDOs
    ec_config_map(&io_map);

    /// Start the Distributed Clock cycle
    motor_configure_timing();

    /// Put the motors in Operational mode (EtherCAT Operation)
    motor_set_operational();

    while (!motors_exit) {

    }
}


/* opens communications with motor controllers */
void initialize_motors(void)
{
  bprintf(info, "Motors: connecting to motors");
  pthread_create(&motor_ctl_id, NULL, &motor_control, NULL);
}

void shutdown_motors(void)
{

}
