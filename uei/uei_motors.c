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

extern int stop;
extern float gy_ifroll;
extern float gy_ifyaw;

typedef struct {

    ec_sdo_request_t *amp_state_sdo;            /// 0x2300 (INT16) 0=disabled, 1=commanded current, 11=commanded velocity

    ec_sdo_request_t *amp_temp_sdo;             /// 0x2202 (INT16) (Celcius)

    ec_sdo_request_t *motor_velocity_sdo;       /// 0x606C (INT32)
    ec_sdo_request_t *motor_position_sdo;       /// 0x6064 (INT32)

    ec_sdo_request_t *programmed_current_sdo;   /// 0x2340 (INT16)

    unsigned int config_error;

} copley_state_t;

copley_state_t rx_controller_state = {0};
ec_slave_config_t *rx_controller = NULL;
ec_master_t *master;
ec_domain_t *domain;

int uei_ethercat_initialize (void)
{
    uint16_t data_16[4] = {0};
    uint8_t *data = (uint8_t*)data_16;
    uint32_t abort_code;

    master = ecrt_request_master(0);
    if (!master){
        printf("Could not request master!\n");
        return -1;
    }

    data_16[0] = 0;
    if (ecrt_master_sdo_download_complete(master, 0, 0x2340, data, 2, &abort_code) < 0) {
        printf("Could not set current value!\n");
        return -1;
    }

    data_16[0] = cpu_to_le16(1); /// Set to commanded current mode
    if (ecrt_master_sdo_download_complete(master, 0, 0x2300, data, 2, &abort_code) < 0) {
        printf("Could not set amplifier state!\n");
        return -1;
    }

    domain = ecrt_master_create_domain(master);
    if (!domain) {
        printf("Could not create domain!\n");
        return -1;
    }

    printf("Created Domain\n");

    if (!(rx_controller = ecrt_master_slave_config(master,0, 0, 0x000000ab,
            0x00000380))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    printf("Set Master/Slave Configuration\n");

    rx_controller_state.amp_state_sdo = ecrt_slave_config_create_sdo_request(rx_controller, 0x2300, 0, 2);
    rx_controller_state.amp_temp_sdo = ecrt_slave_config_create_sdo_request(rx_controller, 0x2202, 0, 2);
    rx_controller_state.motor_position_sdo = ecrt_slave_config_create_sdo_request(rx_controller, 0x6064, 0, 4);
    rx_controller_state.motor_velocity_sdo = ecrt_slave_config_create_sdo_request(rx_controller, 0x6063, 0, 4);
    rx_controller_state.programmed_current_sdo = ecrt_slave_config_create_sdo_request(rx_controller, 0x2340, 0, 2);


    if (ecrt_master_activate(master) < 0) {
        printf("Could not activate master!\n");
        return -1;
    }

//    ecrt_master_reset(master);
    return 0;
}

void uei_ethercat_cleanup(void)
{
    uint16_t data_16[4] = {0};
    uint8_t *data = (uint8_t*)data_16;
    uint32_t abort_code;

    ecrt_master_deactivate(master);
    ecrt_master_sdo_download_complete(master, 0, 0x2300, data, 2, &abort_code);
    ecrt_release_master(master);
}

void motor_cmd_routine(void *m_arg)
{
    int ret;

    RT_TIMER_INFO timer_info;
    long long task_period;
    unsigned long overruns = 0;
    int16_t req_current = 0;

    float cos_el;
    float sin_el;
    float v_req_az;
    float V_REQ_AZ = 0;

    float P_term_az, error_az;
    float p_az = 2.0;
    float i_az = 1.0;
    static float az_integral = 0.0;
    float I_term_az, INTEGRAL_CUTOFF=0.5;


    printf("Starting Motor Commanding task\n");

    rt_timer_inquire(&timer_info);
    if (timer_info.period == TM_ONESHOT)
    {
        // When using an aperiodic timer, task period is specified in ns
        task_period = rt_timer_ns2ticks(1000000000ll / 1000);
    }
    else
    {
        // When using a periodic timer, task period is specified in number of timer periods
        task_period = (1000000000ll / 1000) / timer_info.period;
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

        /*******************************************************************\
        * Card0: Drive the Azimuth Motor (Reaction Wheel)                   *
        \*******************************************************************/
        /* Read sin and cos of the inner frame elevation, calculated by mcp */
        cos_el = 1.0; //( COS_EL*0.000030517578125 ) - 1.0;
        sin_el = 0.0; //( SIN_EL*0.000030517578125 ) - 1.0;

        v_req_az = 0.0; //(float)(V_REQ_AZ-32768.0)*0.0016276041666666666666666666666667;  // = vreq/614.4

        //roll, yaw contributions to az both -'ve (?)
        error_az  = (gy_ifroll*sin_el + gy_ifyaw*cos_el) + v_req_az;

        P_term_az = p_az*error_az;

        if( (p_az == 0.0) || (i_az == 0.0) ) {
            az_integral = 0.0;
        } else {
            az_integral = (1.0 - INTEGRAL_CUTOFF)*az_integral + INTEGRAL_CUTOFF*error_az;
        }

        I_term_az = az_integral * p_az * i_az;
        if (I_term_az > 100.0) {
            I_term_az = 100.0;
            az_integral = az_integral *0.9;
        }
        if (I_term_az < -100.0) {
            I_term_az = -100.0;
            az_integral = az_integral * 0.9;
        }

        req_current =  0.5 *(-(P_term_az + I_term_az) ) ;

        if (req_current > 200)
            printf("Error!  Requested current is %d\n", req_current);
        else {
            if (ecrt_sdo_request_state(rx_controller_state.programmed_current_sdo) != EC_REQUEST_BUSY) {

                EC_WRITE_S16(
                        ecrt_sdo_request_data(
                                rx_controller_state.programmed_current_sdo),
                        req_current);
                ecrt_sdo_request_write(rx_controller_state.programmed_current_sdo);
            }
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
