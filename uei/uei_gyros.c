/* 
 * uei_gyros.c
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of uei, created for the BLASTPol Project.
 *
 * uei is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * uei is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with uei; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 26, 2014 by seth
 */
 
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <native/task.h>
#include <native/event.h>
#include <native/timer.h>
#include <PDNA.h>

#include "uei_filter.h"
#include "uei_gyros.h"
#include "gyros_ideal.h"

extern int stop;

float gy_ifroll, gy_ifyaw, gy_ifel;
int gyro_timeout[N_GYRO] = {0,0,0,0,0,0};

// Max expected size of messages received from the serial ports
#define RX_BUFSIZE 64

#define GYRO_TIMEOUT ((int)SR*3)

unsigned int gymask = 21;
unsigned int gyfault = 0;

uei_filter_t gyro_filter[N_GYRO] = {{0}};

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
	if (!GYRO_VALID(*m_data)) {
		printf("Invalid data packet on Gyro %d!\n", m_which_gyro);
		return;
	}
    uei_filter_put(&gyro_filter[m_which_gyro], 6e-5 * GYRO_CONTENT(*m_data));
}

static void uei_gyro_update_output(void)
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

void uei_gyro_get_vals (float *m_roll, float *m_yaw, float *m_el)
{
	*m_roll = gy_ifroll;
	*m_yaw = gy_ifyaw;
	*m_el = gy_ifel;
}

void uei_gyro_set_mask (uint32_t m_mask)
{
	gymask = m_mask;
}

void gyro_read_routine(void* arg)
{
    int i, ret;
    unsigned char indata[N_GYRO][RX_BUFSIZE] = {{0}};
    int offset_bytes[N_GYRO] = {0};
    int uei_handle = *(int*)arg;

    int bytes_remaining;
    RT_TIMER_INFO timer_info;
    long long task_period;
    unsigned long overruns = 0;

    uint32_t *gyro_data;

    printf("Starting Gyroscope reading task\n");

    rt_timer_inquire(&timer_info);
    if (timer_info.period == TM_ONESHOT)
    {
        // When using an aperiodic timer, task period is specified in ns
        task_period = rt_timer_ns2ticks(1000000000ll / 200);
    }
    else
    {
        // When using a periodic timer, task period is specified in number of timer periods
        task_period = (1000000000ll / 200) / timer_info.period;
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
            ret = DqAdv501ReadRecvFIFO(uei_handle, 2, gyro_channel[i], RX_BUFSIZE - offset_bytes[i], indata[i] + offset_bytes[i], &rx_bytes, &bytes_remaining);
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
