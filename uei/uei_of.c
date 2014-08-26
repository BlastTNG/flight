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

#include "gyros_ideal.h"

static int handle;

int stop = 0;

#define GYRO_SERIAL_SLOT 2
static const int gyro_channel[6] = {0, 1, 2, 3, 4, 5};
float raw_gyro[6] = {0.0};

float gy_ifroll, gy_ifyaw, gy_ifel;
int gyro_timeout[N_GYRO] = {0,0,0,0,0,0};

// channel configuration helper macros
#define CHANNEL_CFG DQCFG_501(DQ_SL501_OPER_NORM, \
                              DQ_SL501_MODE_232, \
                              DQ_SL501_BAUD_115200, \
                              DQ_SL501_WIDTH_8, \
                              DQ_SL501_STOP_1, \
                              DQ_SL501_PARITY_NONE)

// Max expected size of messages received from the serial ports
#define RX_BUFSIZE 64

#define GYRO1_OFFSET 0.0
#define GYRO2_OFFSET 0.0
#define GYRO3_OFFSET 0.0

#define GYRO4_OFFSET 0.0
#define GYRO5_OFFSET 0.0
#define GYRO6_OFFSET 0.0

#define GYRO_TIMEOUT ((int)SR*3)

unsigned int gymask = 21;
unsigned int gyfault = 0;

uei_filter_t gyro_filter[N_GYRO] = {{0}};

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
}


static int uei_of_serial_initialize(int m_device, int m_channel)
{
    int ret = 0;
    // Set channel configuration
    if ((ret = DqAdv501SetChannelCfg(handle, m_device, m_channel, CHANNEL_CFG))
            < 0) {
        printf("error %d in DqAdv501SetChannelCfg()\n", ret);
        return ret;
    }

    // Disable waiting for specified byte count.  Immediately returns available bytes.
    if ((ret = DqAdv501SetTimeout(handle, m_device, m_channel, 0)) < 0) {
        printf("error %d in DqAdv501SetTimeout\n", ret);
        return ret;
    }

    // Disable FIFO queuing.  Any bytes available will be sent
    if ((ret = DqAdv501SetTermLength(handle, m_device, m_channel, 0)) < 0) {
        printf("error %d in DqAdv501SetTermLength\n", ret);
        return ret;
    }

    // Sets RX to direct mode which disables interrupt processing and allows us to read directly
    // from the serial port's FIFO without any delay.
    if ((ret = DqAdv501SetWatermark(handle, m_device, m_channel,
            DQL_IOCTL501_SETRXWM_DIRECT, 0)) < 0) {
        printf("error %d in DqAdv501SetWatermark\n", ret);
        return ret;
    }

    return ret;
}

static int uei_of_initialize(void)
{
    struct sched_param schedp;
    int ret, i;

    DqInitDAQLib();

    printf("Initialized DAQLib\n");
    signal(SIGINT, handler);
    signal(SIGTERM, handler);
//  signal(SIGXCPU, warn_upon_switch);

    // no memory-swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Configure this process to run with the real-time scheduler
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    // open communication with IOM and receive IOM crucial identification data
    if ((ret = DqOpenIOM("127.0.0.1", DQ_UDP_DAQ_PORT, 2000, &handle, NULL))
            < 0) {
        printf("Error %d In Initializing Communication with IOM\n", ret);
        return ret;
    }

    printf("Opened IOM\n");

    for (i = 0; i < N_GYRO; i++) {
        if ((ret = uei_of_serial_initialize(GYRO_SERIAL_SLOT, gyro_channel[i])) < 0) {
            printf("Error %d while setting serial parameters on channel %d\n", ret, i);
            return ret;
        }
    }

    if ((ret = DqAdv501Enable(handle, GYRO_SERIAL_SLOT, TRUE)) < 0) {
        printf("Error %d in DqAdv501Enable()\n", ret);
        return ret;
    }

    return 0;
}

/**
 * Synchronizes the gyro data stream from bytes to gyro words.
 * In the standard case, it takes a single 32-bit word from the input and
 * returns 0, indicating that the stream is synchronized.
 *
 * If we are not synchronized, we require at least 5 words of gyro data (20 bytes)
 * to synchronize
 * @param m_word  Pointer to the input data stream
 * @param m_length Length of data in #m_word
 * @param m_which Index of the gyro
 * @return 0 on synchronized.  -1 on not enough data, -2 on cannot sync. Positive value for
 *  offset to next valid gyro word (in bytes)
 */
static int uei_of_sync_gyro(uint8_t *m_word, int m_length, int m_which, uint32_t **m_gyrodata)
{
    static uint32_t next_sync[N_GYRO] = {0};
    static bool in_sync[N_GYRO] = {false};
    int i, j;

    if (m_length < 4) return GYRO_INSUFFICIENT_DATA;

    (*m_gyrodata) = (uint32_t*) m_word;
    if (in_sync[m_which] && GYRO_SYNC((*m_gyrodata)[0]) == next_sync[m_which]) {
        next_sync[m_which] = (next_sync[m_which] + 1) & 3;
        return GYRO_SYNCHRONIZED;
    }

    // We lost the sync.  Reset the search to 0;
    if (in_sync[m_which]) {
        printf("Lost SYNC.  Expecting %u but received %u\n", next_sync[m_which], GYRO_SYNC((*m_gyrodata)[0]));
        next_sync[m_which] = 0;
        in_sync[m_which] = false;
    }

    if (m_length < 20)
        return GYRO_INSUFFICIENT_DATA;

    for (i = 0; i < 4; i++) {
        (*m_gyrodata) = (uint32_t*) (m_word + i);
        for (j = 0; j < 4; j++) {
            if (((GYRO_SYNC((*m_gyrodata)[j]) + 1) & 3) != GYRO_SYNC((*m_gyrodata)[j + 1]))
                break;
        }
        if (j == 4) {
            in_sync[m_which] = true;
            next_sync[m_which] = GYRO_SYNC((*m_gyrodata)[1]);
            return i;
        }
    }

    return GYRO_CANT_SYNC;
}

static inline void uei_process_gyro_packets(uint32_t *m_data, int m_which_gyro)
{
	raw_gyro[m_which_gyro] =  6e-5 * GYRO_CONTENT(*m_data);
	if (!GYRO_VALID(*m_data)) {
		printf("Invalid data packet on Gyro %d!\n", m_which_gyro);
		return;
	}
    uei_filter_put(&gyro_filter[m_which_gyro], raw_gyro[m_which_gyro]);
}

void uei_gyro_update_output(void)
{

    float gy_ifyaw1, gy_ifyaw2;
    float gy_ifel1, gy_ifel2;
    float gy_ifroll1, gy_ifroll2;

    /* timesed ifroll and ifel by -1.0 to make the signs correct in mcp. */
    gy_ifyaw1  = uei_filter_get(&gyro_filter[0]);
    gy_ifroll1 = uei_filter_get(&gyro_filter[1])*(-1.0);
    gy_ifel1   = uei_filter_get(&gyro_filter[2]);
    gy_ifyaw2  = uei_filter_get(&gyro_filter[3])*(-1.0);
    gy_ifroll2 = uei_filter_get(&gyro_filter[4])*(-1.0);
    gy_ifel2   = uei_filter_get(&gyro_filter[5])*(-1.0);

    /* rotate gyros into inner frame coordinates, and mask*/
    gy_ifroll= (gy_inv[gymask][0][0]*gy_ifroll1 + gy_inv[gymask][0][1]*gy_ifroll2 + gy_inv[gymask][0][2]*gy_ifyaw1 + gy_inv[gymask][0][3]*gy_ifyaw2 + gy_inv[gymask][0][4]*gy_ifel1 + gy_inv[gymask][0][5]*gy_ifel2);
    gy_ifyaw = (gy_inv[gymask][1][0]*gy_ifroll1 + gy_inv[gymask][1][1]*gy_ifroll2 + gy_inv[gymask][1][2]*gy_ifyaw1 + gy_inv[gymask][1][3]*gy_ifyaw2 + gy_inv[gymask][1][4]*gy_ifel1 + gy_inv[gymask][1][5]*gy_ifel2);
    gy_ifel  = (gy_inv[gymask][2][0]*gy_ifroll1 + gy_inv[gymask][2][1]*gy_ifroll2 + gy_inv[gymask][2][2]*gy_ifyaw1 + gy_inv[gymask][2][3]*gy_ifyaw2 + gy_inv[gymask][2][4]*gy_ifel1 + gy_inv[gymask][2][5]*gy_ifel2);

}

void gyro_read_routine(void* arg)
{
    int i, ret;
    unsigned char indata[N_GYRO][RX_BUFSIZE] = {{0}};
    int offset_bytes[N_GYRO] = {0};

    int avail;
    RT_TIMER_INFO timer_info;
    long long task_period;
    unsigned long overruns = 0;

    uint32_t *gyro_data;

    printf("Starting Gyroscope reading task\n");

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
    // And ask Xenomai to warn us upon switches to secondary mode with signal SIGXCPU
    rt_task_set_mode(0, T_PRIMARY, NULL);


    while (!stop)
    {
        int rx_bytes;
        int consumed_bytes;
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

        for (i = 0; i < N_GYRO; i++) {
            ret = DqAdv501ReadRecvFIFO(handle, 2, gyro_channel[i], RX_BUFSIZE - offset_bytes[i], indata[i] + offset_bytes[i], &rx_bytes, &avail);
            if(ret < 0) {
                printf("Error %d in DqAdv501ReadRecvFIFO()\n", ret);
                continue;
            }
            consumed_bytes = 0;

            while (ret != GYRO_INSUFFICIENT_DATA) {
                if ( (rx_bytes + offset_bytes[i] - consumed_bytes) < 0) {
                    break;
                }
                ret = uei_of_sync_gyro(indata[i] + consumed_bytes, rx_bytes + offset_bytes[i] - consumed_bytes, i, &gyro_data);

                switch (ret)
                {
                    case GYRO_INSUFFICIENT_DATA:
                        memmove(indata[i], indata[i] + consumed_bytes, rx_bytes + offset_bytes[i] - consumed_bytes);
                        offset_bytes[i] = rx_bytes + offset_bytes[i] - consumed_bytes;
                        break;
                    case GYRO_CANT_SYNC:
                    	printf("Can't SYNC Gyro %d!\n", i);
                        consumed_bytes += 4;
                        break;
                    default: {
                        consumed_bytes += (ret + 4);
                        uei_process_gyro_packets(gyro_data, gyro_channel[i]);
                    }
                }
            }
        }

        uei_gyro_update_output();

    }

    //switch to secondary mode
    ret = rt_task_set_mode(T_PRIMARY, 0, NULL);
    if (ret)
    {
        printf("error while rt_task_set_mode, code %d\n", ret);
        return;
    }

}

void uei_cleanup(void)
{
    int ret;

    uei_ethercat_cleanup();

    if ((ret = DqAdv501Enable(handle, GYRO_SERIAL_SLOT, FALSE)) < 0) {
        printf("Error %d in DqAdv501Enable()\n", ret);
    }

    if (handle) {
        DqCloseIOM(handle);
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


    ret = rt_task_start(&gyro_read_task, &gyro_read_routine, NULL);
    if (ret) {
    	perror("failed to start gyro_read_task");
        exit(1);
    }

    ret = rt_task_start(&motor_cmd_task, &motor_cmd_routine, NULL);
    if (ret) {
    	perror("failed to start motor_cmd_task");
        exit(1);
    }

    ret = rt_task_start(&mosq_task, &uei_framing_routine, NULL);
    if (ret) {
    	perror("failed to start frame handling routine");
        exit(1);
    }

    while (!stop) {
        usleep(1000000);
        printf("%f\t--\t%f\t%f\t%f\n", rt_timer_ticks2ns(rt_timer_read()) / 1000000000.0,
        		uei_filter_get(&gyro_filter[0]),
        		uei_filter_get(&gyro_filter[1]),
        		uei_filter_get(&gyro_filter[2]));
        printf("\t\t--\t%f\t%f\t%f\n",
                raw_gyro[0], raw_gyro[1], raw_gyro[2]);
        fflush(stdout);
    }

    rt_task_join(&gyro_read_task);
    rt_task_join(&motor_cmd_task);
    rt_task_join(&mosq_task);

}
