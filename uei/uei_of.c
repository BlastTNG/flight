/* 
 * uei_of.c: 
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
 * Created on: Aug 6, 2014 by seth
 */

/**
 * The outer frame UEI code handles the following:
 *
 * - Motor Control
 * - Elevation encoder
 * - OF Thermometry
 * - Inclinometer
 * - Magnetometer
 * - Pump state
 * - Sun Sensors
 * - Charge Controllers
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>
#include <sched.h>
#include <sys/mman.h>
#include <byteswap.h>

#include <wordexp.h>
#include <execinfo.h>

#include <native/task.h>
#include <native/event.h>
#include <native/timer.h>
#include <PDNA.h>

#include <mosquitto.h>

#include "uei_of.h"
#include "uei_filter.h"
#include "uei_framing.h"
#include "uei_motors.h"
#include "uei_control_vals.h"
#include "uei_gyros.h"

#include "uei_sl501.h"


static int uei_handle;

int stop = 0;

// Handler for SIGINT
void handler(int sig)
{
    printf("Caught interrupt!\n");
    stop = 1;
}

void warn_upon_switch(int sig)
{
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
    exit(1);
}

static int uei_of_initialize(void)
{
    struct sched_param schedp;
    int ret, i;

    DqInitDAQLib();

    printf("Initialized DAQLib\n");
    signal(SIGINT, handler);
    signal(SIGTERM, handler);
    signal(SIGSEGV, warn_upon_switch);
//  signal(SIGXCPU, warn_upon_switch);

    // no memory-swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Configure this process to run with the real-time scheduler
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    // open communication with IOM and receive IOM crucial identification data
    if ((ret = DqOpenIOM("127.0.0.1", DQ_UDP_DAQ_PORT, 2000, &uei_handle, NULL))
            < 0) {
        printf("Error %d In Initializing Communication with IOM\n", ret);
        return ret;
    }

    printf("Opened IOM\n");

    for (i = 0; i < N_GYRO; i++) {
        if ((ret = uei_serial_232_initialize(uei_handle, GYRO_SERIAL_SLOT, gyro_channel[i], GYRO_CHANNEL_CFG)) < 0) {
            printf("Error %d while setting serial parameters on channel %d\n", ret, i);
            return ret;
        }
    }

    if ((ret = DqAdv501Enable(uei_handle, GYRO_SERIAL_SLOT, TRUE)) < 0) {
        printf("Error %d in DqAdv501Enable()\n", ret);
        return ret;
    }

    return 0;
}



void uei_cleanup(void)
{
    int ret;

    uei_ethercat_cleanup();

    if ((ret = DqAdv501Enable(uei_handle, GYRO_SERIAL_SLOT, FALSE)) < 0) {
        printf("Error %d in DqAdv501Enable()\n", ret);
    }

    if (uei_handle) {
        DqCloseIOM(uei_handle);
    }

    DqCleanUpDAQLib();
}

int main(void)
{
    int ret;

    RT_TASK gyro_read_task;
    RT_TASK motor_cmd_task;
    RT_TASK mosq_task;

    if (uei_of_initialize() < 0)
        exit(1);
    if (uei_ethercat_initialize() < 0)
        exit(1);
    if (uei_framing_init() < 0)
        exit(1);

    printf("Initialized!\n");

    atexit(uei_cleanup);

    // Create our Tasks
    ret = rt_task_create(&gyro_read_task, "read_serial", 0, T_HIPRIO, T_JOINABLE|T_FPU);
    if (ret) {
    	perror("failed to create task");
        exit(1);
    }

    ret = rt_task_create(&motor_cmd_task, "motor_cmds", 0, 30, T_JOINABLE);
    if (ret) {
    	perror("failed to create motor task");
        exit(1);
    }

    ret = rt_task_create(&mosq_task, "mosq_monitor", 0, 40, T_JOINABLE);
    if (ret) {
        perror("failed to create Mosquitto task");
        exit(1);
    }


    ret = rt_task_start(&gyro_read_task, &gyro_read_routine, (void*)(&uei_handle));
    if (ret) {
    	perror("failed to start gyro_read_task");
        exit(1);
    }

    ret = rt_task_start(&motor_cmd_task, &motor_cmd_routine, NULL);
    if (ret) {
    	perror("failed to start motor_cmd_task");
        exit(1);
    }

    ret = rt_task_start(&mosq_task, &uei_framing_loop, NULL);
    if (ret) {
    	perror("failed to start frame handling routine");
        exit(1);
    }

    while (!stop) {
    	float yaw,pitch,roll;
        usleep(100000);
//        printf("%d\n", ethercat_get_current());
        uei_gyro_get_vals(&roll,&yaw,&pitch);
        printf("%f\t--\t%f\t%f\t%f\n", rt_timer_ticks2ns(rt_timer_read()) / 1000000000.0,
        		yaw, pitch,roll);
        fflush(stdout);
    }

    rt_task_join(&gyro_read_task);
    rt_task_join(&motor_cmd_task);
    rt_task_join(&mosq_task);

}
