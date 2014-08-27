/* 
 * uei_motors.c: 
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


#include <native/task.h>
#include <native/event.h>
#include <native/timer.h>

#include <ecrt.h>

#include "uei_motors.h"

extern int stop;
extern float gy_ifroll;
extern float gy_ifyaw;

typedef struct {

    uint16_t *amp_state;
    int16_t *amp_temp;
    int16_t *motor_temp;

    uint16_t *current_loop_ci;
    uint16_t *current_loop_cp;
    int16_t *current_loop_offset;
    int16_t *current_val;

    int32_t *motor_position;
    int32_t *motor_position_wrap_val;

    uint32_t *drive_status;
    uint32_t *latched_drive_status;
    uint32_t *latched_drive_faults;

    unsigned int config_error;

} copley_state_t;


static copley_state_t rx_controller_state = {0};
static copley_state_t el_controller_state = {0};
static copley_state_t pv_controller_state = {0};

static ec_slave_config_t *rx_controller = NULL;
static ec_slave_config_t *pv_controller = NULL;
static ec_slave_config_t *el_controller = NULL;

static ec_master_t *master;
static ec_domain_t *domain;
static ec_domain_state_t domain1_state = {0};
static ec_master_state_t master_state = {0};

static unsigned int current_ci_off[3];
static unsigned int current_cp_off[3];
static unsigned int current_offset_off[3];
static unsigned int current_val_off[3];
static unsigned int status_off[3];
static unsigned int state_off[3];
static unsigned int drive_temp_off[3];
static unsigned int latched_fault_off[3];
static unsigned int latched_status_off[3];
static unsigned int motor_pos_off[3];
static unsigned int motor_temp_v_off[3];
static unsigned int motor_enc_wrap_off[3];

static ec_pdo_entry_info_t copley_pdo_entries[] = {
//    {ECAT_CURRENT_LOOP_CI, 16},
//    {ECAT_CURRENT_LOOP_CP, 16},
//    {ECAT_CURRENT_LOOP_OFFSET, 16},
    {ECAT_CURRENT_LOOP_VAL, 16},

    {ECAT_DRIVE_STATE, 16},
//    {ECAT_DRIVE_STATUS, 32},
//
//    {ECAT_LATCHED_DRIVE_FAULT, 32},
//    {ECAT_LATCHED_DRIVE_STATUS, 32},
//
//    {ECAT_MOTOR_ENC_WRAP_POS, 32},
//    {ECAT_MOTOR_POSITION, 32},
//
//    {ECAT_DRIVE_TEMP, 16},
//    {ECAT_MOTOR_TEMP_VOLTAGE, 16},
};


static ec_pdo_info_t copley_pdos[] = {
//    {ECAT_CURRENT_LOOP_CI_PDO_WR, 1, copley_pdo_entries + 0}, /* Loop Ci - WRITE */
//    {ECAT_CURRENT_LOOP_CP_PDO_WR, 1, copley_pdo_entries + 1}, /* Loop Cp - WRITE */
//    {ECAT_CURRENT_LOOP_OFFSET_PDO_WR, 1, copley_pdo_entries + 2}, /* Loop Offset - WRITE */
    {ECAT_CURRENT_LOOP_VAL_PDO_WR, 1, copley_pdo_entries}, /* Loop Current Val - WRITE */
    {ECAT_DRIVE_STATE_PDO_WR, 1, copley_pdo_entries + 1}, /* Drive State - WRITE */
//    {ECAT_MOTOR_ENC_WRAP_POS_PDO_WR, 1, copley_pdo_entries + 8}, /* Encoder Wrap Pos - WRITE */
};
static ec_sync_info_t copley_pdo_syncs[] = {
    {1, EC_DIR_INPUT},
    {2, EC_DIR_OUTPUT, 2, copley_pdos + 0, EC_WD_ENABLE},
    {0xff}
};

int16_t ethercat_get_current(void)
{
	return EC_READ_U16(rx_controller_state.current_val);
}
void static inline ethercat_set_offsets(copley_state_t *m_state, const uint8_t *m_data, int m_index)
{

    m_state->amp_state = (uint16_t*)(m_data + state_off[m_index]);
    m_state->amp_temp = (int16_t*)(m_data + drive_temp_off[m_index]);
    m_state->current_loop_ci = (uint16_t*)(m_data + current_ci_off[m_index]);
    m_state->current_loop_cp = (uint16_t*)(m_data + current_cp_off[m_index]);
    m_state->current_loop_offset = (int16_t*)(m_data + current_offset_off[m_index]);
    m_state->current_val = (int16_t*)(m_data + current_val_off[m_index]);
    m_state->drive_status = (uint32_t*)(m_data + status_off[m_index]);
    m_state->latched_drive_faults = (uint32_t*)(m_data + latched_fault_off[m_index]);
    m_state->latched_drive_status = (uint32_t*)(m_data + latched_status_off[m_index]);
    m_state->motor_position = (int32_t*)(m_data + motor_pos_off[m_index]);
    m_state->motor_position_wrap_val = (int32_t*)(m_data + motor_enc_wrap_off[m_index]);
    m_state->motor_temp = (int16_t*)(m_data + motor_temp_v_off[m_index]);
}
/*****************************************************************************/

static inline void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    memcpy(&domain1_state, &ds, sizeof(ds));
}

/*****************************************************************************/

static inline void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    memcpy(&master_state, &ms, sizeof(ms));
}

int uei_ethercat_initialize (void)
{

    uint8_t *data;

    ec_pdo_entry_reg_t rw_pdos[] = {
//		{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_CI,  current_ci_off, NULL},
//		{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_CP,  current_cp_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_OFFSET,  current_offset_off, NULL},
    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_VAL,  current_val_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_MOTOR_POSITION,  motor_pos_off, NULL},
    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_DRIVE_STATE,  state_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_DRIVE_STATUS,  status_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_DRIVE_TEMP,  drive_temp_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_LATCHED_DRIVE_FAULT,  latched_fault_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_LATCHED_DRIVE_STATUS,  latched_status_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_MOTOR_TEMP_VOLTAGE,  motor_temp_v_off, NULL},
//    	{RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_MOTOR_ENC_WRAP_POS,  motor_enc_wrap_off, NULL},
    	{0, 0,      0x00,         0x00, 0x0, 0x0,           NULL, NULL}};
//    ec_pdo_entry_reg_t pv_pdos[] = {
//		{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_CURRENT_LOOP_CI,  current_ci_off+1, NULL},
//		{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_CURRENT_LOOP_CP,  current_cp_off+1, NULL},
//		{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_CURRENT_LOOP_OFFSET,  current_offset_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_CURRENT_LOOP_VAL,  current_val_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_DRIVE_STATUS,  status_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_DRIVE_TEMP,  drive_temp_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_LATCHED_DRIVE_FAULT,  latched_fault_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_LATCHED_DRIVE_STATUS,  latched_status_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_MOTOR_POSITION,  motor_pos_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_MOTOR_TEMP_VOLTAGE,  motor_temp_v_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_MOTOR_ENC_WRAP_POS,  motor_enc_wrap_off+1, NULL},
//    	{PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE, ECAT_DRIVE_STATE,  state_off+1, NULL},
//    	{0, 0,      0x00,         0x00, 0x0, 0x0,           NULL, NULL}};
//    ec_pdo_entry_reg_t el_pdos[] = {
//		{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_CI,  current_ci_off+2, NULL},
//		{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_CP,  current_cp_off+2, NULL},
//		{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_OFFSET,  current_offset_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_CURRENT_LOOP_VAL,  current_val_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_DRIVE_STATUS,  status_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_DRIVE_TEMP,  drive_temp_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_LATCHED_DRIVE_FAULT,  latched_fault_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_LATCHED_DRIVE_STATUS,  latched_status_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_MOTOR_POSITION,  motor_pos_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_MOTOR_TEMP_VOLTAGE,  motor_temp_v_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_MOTOR_ENC_WRAP_POS,  motor_enc_wrap_off+2, NULL},
//    	{EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE, ECAT_DRIVE_STATE,  state_off+2, NULL},
//    	{0, 0,      0x00,         0x00, 0x0, 0x0,           NULL, NULL}};

    master = ecrt_request_master(0);
    if (!master){
        printf("Could not request master!\n");
        return -1;
    }

    domain = ecrt_master_create_domain(master);
    if (!domain) {
        printf("Could not create domain!\n");
        return -1;
    }

    printf("Created Domain\n");

    if (!(rx_controller = ecrt_master_slave_config(master,RW_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE))) {
        fprintf(stderr, "Failed to get slave configuration for Reaction Wheel controller!\n");
        return -1;
    }
//    if (!(pv_controller = ecrt_master_slave_config(master,PV_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, BEL_090_020_PRODCODE))) {
//        fprintf(stderr, "Failed to get slave configuration for Pivot Motor controller!\n");
//        return -1;
//    }
//    if (!(el_controller = ecrt_master_slave_config(master,EL_ETHERCAT_ALIAS, 0, COPLEY_ETHERCAT_VENDOR, AEP_090_036_PRODCODE))) {
//        fprintf(stderr, "Failed to get slave configuration for Elevation Motor controller!\n");
//        return -1;
//    }

	if (ecrt_slave_config_pdos(rx_controller, EC_END, copley_pdo_syncs)) {
		perror("ecrt_slave_config_pdos() failed for RX controller.");
		ecrt_release_master(master);
		return 3;
	}
//	if (ecrt_slave_config_pdos(pv_controller, 1, copley_pdo_syncs)) {
//		perror("ecrt_slave_config_pdos() failed for Pivot controller.");
//		ecrt_release_master(master);
//		return 3;
//	}
//	if (ecrt_slave_config_pdos(el_controller, 1, copley_pdo_syncs)) {
//		perror("ecrt_slave_config_pdos() failed for Elevation controller.");
//		ecrt_release_master(master);
//		return 3;
//	}

	/// Register the PDO list and variable mappings
//	if (ecrt_domain_reg_pdo_entry_list(domain, rw_pdos)) {
//		perror("ecrt_domain_reg_pdo_entry_list() failed for reaction wheel!");
//		ecrt_release_master(master);
//		return -1;
//	}
//	if (ecrt_domain_reg_pdo_entry_list(domain, pv_pdos)) {
//		perror("ecrt_domain_reg_pdo_entry_list() failed for pivot motor!");
//		ecrt_release_master(master);
//		return -1;
//	}
//	if (ecrt_domain_reg_pdo_entry_list(domain, el_pdos)) {
//		perror("ecrt_domain_reg_pdo_entry_list() failed for Elevation motor!");
//		ecrt_release_master(master);
//		return -1;
//	}
    state_off[0] = ecrt_slave_config_reg_pdo_entry(rx_controller,
                ECAT_DRIVE_STATE, domain, NULL);
    current_val_off[0] = ecrt_slave_config_reg_pdo_entry(rx_controller,
    		ECAT_CURRENT_LOOP_VAL, domain, NULL);
    // configure SYNC signals for this slave
	ecrt_slave_config_dc(rx_controller, 0, 1000000000ll / 100, 4400000, 0, 0);

    printf("Set Master/Slave Configuration\n");

    if (ecrt_master_activate(master) < 0) {
        printf("Could not activate master!\n");
        return -1;
    }

    if (!(data = ecrt_domain_data(domain))) {
    	perror("ecrt_domain_data() failed!");
    	ecrt_release_master(master);
    	return -1;
    }

    ethercat_set_offsets(&rx_controller_state, data, 0);
//    ethercat_set_offsets(&pv_controller_state, data, 1);
//    ethercat_set_offsets(&el_controller_state, data, 2);

    check_domain1_state();
    check_master_state();

    printf("Data: %p\t Current: %p\t State: %p\n", data, rx_controller_state.current_val, rx_controller_state.amp_state);
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    EC_WRITE_S16(data + current_val_off[0], 0);
    EC_WRITE_U16(data + state_off[0], ECAT_STATE_DISABLED);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);
    check_domain1_state();
    check_master_state();
    return 0;
}

void uei_ethercat_cleanup(void)
{
    ecrt_master_receive(master);
    ecrt_domain_process(domain);
    EC_WRITE_S16(rx_controller_state.current_val, 0);
    EC_WRITE_U16(rx_controller_state.amp_state, ECAT_STATE_DISABLED);

//    EC_WRITE_S16(pv_controller_state.current_val, 0);
//    EC_WRITE_U16(pv_controller_state.amp_state, ECAT_STATE_DISABLED);
//
//    EC_WRITE_S16(el_controller_state.current_val, 0);
//    EC_WRITE_U16(el_controller_state.amp_state, ECAT_STATE_DISABLED);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    ecrt_release_master(master);
}

void motor_cmd_routine(void *m_arg)
{
    int ret;

    RT_TIMER_INFO timer_info;
    long long task_period;
    unsigned long overruns = 0;
    int16_t req_current = 0;
    int sync_ref_counter=0;

    float cos_el;
    float sin_el;
    float v_req_az;
    float V_REQ_AZ = 0;

    float P_term_az, error_az;
    float p_az = 20.0;
    float i_az = 1.0;
    static float az_integral = 0.0;
    float I_term_az, INTEGRAL_CUTOFF=0.5;


    printf("Starting Motor Commanding task\n");

    rt_timer_inquire(&timer_info);
    if (timer_info.period == TM_ONESHOT)
    {
        // When using an aperiodic timer, task period is specified in ns
        task_period = rt_timer_ns2ticks(1000000000ll / 100);
    }
    else
    {
        // When using a periodic timer, task period is specified in number of timer periods
        task_period = (1000000000ll / 100) / timer_info.period;
    }

    ret = rt_task_set_periodic(NULL, TM_NOW, task_period);
    if (ret)
    {
        printf("error while set periodic, code %d\n", ret);
        return;
    }

    // Make sure we are in primary mode before entering the timer loop
    rt_task_set_mode(0, T_PRIMARY, NULL);

    while (!stop)
    {
        unsigned long ov;

        // Wait for next time period
        ret = rt_task_wait_period(&ov);
        if (ret && ret != -ETIMEDOUT)
        {
            printf("error while rt_task_wait_period, code %d (%s)\n", ret,
                    strerror(-ret));
            break;
        }

        overruns = overruns + ov;
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

		// write application time to master
		ecrt_master_application_time(master, rt_timer_tsc2ns(rt_timer_tsc()));

		if (sync_ref_counter) {
			sync_ref_counter--;
		} else {
			sync_ref_counter = 1; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}
		ecrt_master_sync_slave_clocks(master);

        /*******************************************************************\
        * Card0: Drive the Azimuth Motor (Reaction Wheel)                   *
        \*******************************************************************/
        /* Read sin and cos of the inner frame elevation, calculated by mcp */
//        cos_el = 1.0; //( COS_EL*0.000030517578125 ) - 1.0;
//        sin_el = 0.0; //( SIN_EL*0.000030517578125 ) - 1.0;
//
//        v_req_az = 0.0; //(float)(V_REQ_AZ-32768.0)*0.0016276041666666666666666666666667;  // = vreq/614.4
//
//        //roll, yaw contributions to az both -'ve (?)
//        error_az  = (gy_ifroll*sin_el + gy_ifyaw*cos_el) + v_req_az;
//
//        P_term_az = p_az*error_az;
//
//        if( (p_az == 0.0) || (i_az == 0.0) ) {
//            az_integral = 0.0;
//        } else {
//            az_integral = (1.0 - INTEGRAL_CUTOFF)*az_integral + INTEGRAL_CUTOFF*error_az;
//        }
//
//        I_term_az = az_integral * p_az * i_az;
//        if (I_term_az > 100.0) {
//            I_term_az = 100.0;
//            az_integral = az_integral *0.9;
//        }
//        if (I_term_az < -100.0) {
//            I_term_az = -100.0;
//            az_integral = az_integral * 0.9;
//        }
//        if (P_term_az > 1.0 || P_term_az < -1.0) printf("error_az: %f\tI: %f\tP: %f\n", error_az, I_term_az, P_term_az);
//        req_current =  0.5 *(-(P_term_az + I_term_az) ) ;
        req_current = 100;
        if (req_current > 200)
            printf("Error!  Requested current is %d\n", req_current);
        else {
            EC_WRITE_S16(rx_controller_state.current_val, req_current);
        }

        ecrt_domain_queue(domain);
        ecrt_master_send(master);

    }
    //switch to secondary mode
    ret = rt_task_set_mode(T_PRIMARY, 0, NULL);
    if (ret)
    {
        printf("error while rt_task_set_mode, code %d\n", ret);
        return;
    }

}
