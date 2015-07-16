/* 
 * uei_if.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
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
 * Created on: Jul 9, 2014 by Seth Hillbrand
 */


#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sched.h>

#include <native/task.h>
#include <native/event.h>
#include <native/timer.h>

#include <PDNA.h>
#include <mosquitto.h>

#include <channels_tng.h>
#include "uei_framing.h"

extern channel_t channel_list[]; //tx_struct_tng.c

#define IF_AI201_LAYER 2
/// Layer 2 - AI-201-100
static uei_channel_map_t if_ai201_map[] = {
        {NULL,"td_vcs2_filt", 0, DQ_AI201_GAIN_1_100},
        {NULL,"td_3he_fridge", 1, DQ_AI201_GAIN_1_100},
        {NULL,"td_d01_pumped_pot", 2, DQ_AI201_GAIN_1_100},
        {NULL,"td_d07_vcs1_filt", 3, DQ_AI201_GAIN_1_100},
        {NULL,"td_d06_250ppa", 5, DQ_AI201_GAIN_1_100},
        {NULL,"td_d15_vcs1_hx", 6, DQ_AI201_GAIN_1_100},
        {NULL,"td_d15_vcs2_hx", 7, DQ_AI201_GAIN_1_100},
        {NULL,"td_d05_m3", 8, DQ_AI201_GAIN_1_100},
        {NULL,"td_d11_vcs2", 9, DQ_AI201_GAIN_1_100},
        {NULL,"td_d12_vcs1", 10, DQ_AI201_GAIN_1_100},
        {NULL,"td_d04_m4", 11, DQ_AI201_GAIN_1_100},
        {NULL,"td_d02_ob_filter", 12, DQ_AI201_GAIN_1_100},
        {NULL,"td_d09_lhe_filter", 13, DQ_AI201_GAIN_1_100},
        {NULL,"td_d03_hwp", 14, DQ_AI201_GAIN_1_100},
        {NULL, "", -1, -1}
};

#define IF_DNRP_LAYER  14
/// Layer 14 is the internal sensors on the motherboard
static uei_channel_map_t if_internal_sensors[] = {
        { NULL, "uei_if_2.5V",    0, 0 },
        { NULL, "uei_if_2.5Vnic", 1, 0 },
        { NULL, "uei_if_3.3V",    2, 0 },
        { NULL, "uei_if_3.3Vnic", 3, 0 },
        { NULL, "uei_if_24V",     4, 0 },
        { NULL, "uei_if_24Vnic",  5, 0 },
        { NULL, "uei_if_Vin",     6, 0 },
        { NULL, "uei_if_1.5V",    7, 0 },
        { NULL, "uei_if_1.2V",    8, 0 },
        { NULL, "uei_if_8Vfan",   9, 0 },
        { NULL, "uei_if_i_in",    10, 0 },
        { NULL, "uei_if_temp1",   11, 0 },
        { NULL, "uei_if_temp2",   12, 0 },
        { NULL, "", -1, -1}
};
static int stop = 0;
static const double sample_frequency = 200.0;

static int uei_handle = 0;
static int dmap_id = 0;
static int dmap_channel_count = 0;

// Handler for SIGINT
void handler(int sig)
{
    printf("Caught interrupt!\n");
    stop = 1;
}

static void uei_if_cleanup(void)
{
    int ret;

    if (uei_handle) {

        DqRtDmapStop(uei_handle, dmap_id);
        DqRtDmapClose(uei_handle, dmap_id);

        DqCloseIOM(uei_handle);
    }

    DqCleanUpDAQLib();
}

static int uei_if_set_channel_map(void)
{
    uei_channel_map_t *cur_channel = NULL;
    int entry;
    int ret;

    /**
     * Begin the DMAP by getting the timestamp from each layer.
     * The timestamp is shared between all layers in the DMAP but we
     * store each separately because the DMAP read function is layer-specific
     */
    entry = DQ_LNCL_TIMESTAMP;
    ret = DqRtDmapAddChannel(uei_handle, dmap_id, IF_AI201_LAYER, DQ_SS0IN, &entry, 1);
    dmap_channel_count++;
     for (cur_channel = if_ai201_map; cur_channel->channel_num > -1; cur_channel++) {
        cur_channel->channel = channels_find_by_name(cur_channel->name);

        entry = cur_channel->channel_num |cur_channel->gain;
        ret = DqRtDmapAddChannel(uei_handle, dmap_id, IF_AI201_LAYER, DQ_SS0IN, &entry, 1);
        if (ret < 0) {
            printf("Error adding %s to AI201-100 DMAP structure: %d\n", cur_channel->name, ret);
        } else {
            dmap_channel_count++;
        }
    }

     /// Add the internal map
     entry = DQ_LNCL_TIMESTAMP;
     ret = DqRtDmapAddChannel( uei_handle, dmap_id, IF_DNRP_LAYER, DQ_SS0IN, &entry, 1);
     if (ret < 0) {
         printf("Error adding Timestamp to internal layer map");
     } else dmap_channel_count++;

     for (cur_channel = if_internal_sensors; cur_channel->channel_num > -1; cur_channel++) {
         cur_channel->channel = channels_find_by_name(cur_channel->name);

         entry = cur_channel->channel_num;
         ret = DqRtDmapAddChannel(uei_handle, dmap_id, IF_DNRP_LAYER, DQ_SS0IN, &entry, 1);
         if (ret < 0) {
             printf("Error adding %s to internal monitor DMAP structure: %d\n", cur_channel->name, ret);
         } else {
             dmap_channel_count++;
         }

     }
    return 0;
}

static int uei_if_initialize(void)
{
    struct sched_param schedp;
    int ret = 0;
    int i;

    DqInitDAQLib();
    printf("Initialized DAQLib\n");

    //populate nios addresses, based off of tx_struct, derived
    channels_initialize(channel_list);

    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    // no memory-swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Configure this process to run with the real-time scheduler
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    // open communication with IOM and receive IOM crucial identification data
    if ((ret = DqOpenIOM("127.0.0.1", DQ_UDP_DAQ_PORT, 2000, &uei_handle, NULL)) < 0) {
        printf("Error %d In Initializing Communication with IOM\n", ret);
        goto finish_up;
    }

    printf("Opened IOM\n");

    if ((ret = DqRtDmapInit(uei_handle, &dmap_id, 2*sample_frequency)) < 0) {
        printf("DqRtDmapInit: error %d\n", ret);
        goto finish_up;
    }

    /**
     * Setup the channel mapping
     */
    uei_if_set_channel_map();

    /**
     * Start the DMAP process
     */
    ret = DqRtDmapStart(uei_handle, dmap_id);
    if (ret < 0)
    {
        printf("Error %d in DqRtDmapStart\n", ret);
        goto finish_up;
    }

finish_up:
    if (ret < 0) uei_if_cleanup();

    return ret;
}

void uei_if_sample_cards(void* arg)
{
    RTIME t1, t2;
    RT_TIMER_INFO timer_info;
    double duration;
    long long task_period;
    int ret;

    const int num_samples = 64;
    double dbuffer[num_samples];
    uint32_t u32buffer[num_samples];
    uint16_t u16buffer[num_samples];

    // Get timer info, the way task period is programmed depends on the timer type:
    // periodic or aperiodic.
    rt_timer_inquire(&timer_info);
    if (timer_info.period == TM_ONESHOT) {
        // When using an aperiodic timer, task period is specified in ns
        task_period = rt_timer_ns2ticks(1000000000ll / sample_frequency);
    } else {
        // When using a periodic timer, task period is specified in number of timer periods
        task_period = (1000000000ll / sample_frequency) / timer_info.period;
    }

    ret = rt_task_set_periodic(NULL, TM_NOW, task_period);
    if (ret) {
        printf("error while set periodic, code %d\n", ret);
        return;
    }

    // Make sure we are in primary mode before entering the timer loop
    // And ask Xenomai to warn us upon switches to secondary mode with signal SIGXCPU
    rt_task_set_mode(0, T_PRIMARY|T_WARNSW, NULL);

    t1 = rt_timer_read();

    while (!stop) {
        unsigned long ov;

        ret = rt_task_wait_period(&ov);
        if (ret && ret != -ETIMEDOUT) {
            printf("error while rt_task_wait_period, code %d (%s)\n", ret, strerror(-ret));
            break;
        }

        if ((ret = DqRtDmapRefresh(uei_handle, dmap_id)) < 0 ) {
            printf("DqRtDmapRefresh: error %d\n", ret);
            break;
        }

//        DqRtDmapReadScaledData(uei_handle, dmap_id, IF_AI201_LAYER, dbuffer, num_samples);
        DqRtDmapReadRawData16(uei_handle, dmap_id, IF_AI201_LAYER, u16buffer, num_samples);
        uei_store_analog16_data(if_ai201_map, u16buffer);

        DqRtDmapReadRawData32(uei_handle, dmap_id, IF_DNRP_LAYER, u32buffer, num_samples);
        uei_store_analog32_data(if_internal_sensors, u32buffer);
    }
}

int main(void)
{
    int ret;
    struct timespec next;
    long long periodns;

    RT_TASK sample_task;
    RT_TASK mosq_task;

    if (uei_if_initialize() < 0)
        exit(1);
    if (uei_framing_init(SRC_IF_UEI) < 0)
        exit(1);

    printf("Initialized!\n");

    atexit(uei_cleanup);

    // Create our Tasks

    ret = rt_task_create(&mosq_task, "mosq_publish", 0, 40, T_JOINABLE);
    if (ret) {
        perror("failed to create Mosquitto task");
        exit(1);
    }

    ret = rt_task_start(&mosq_task, &uei_framing_routine, &SRC_LOOKUP_TABLE[SRC_IF_UEI]);
    if (ret) {
        perror("failed to start frame handling routine");
        exit(1);
    }

    while (!stop) {
        usleep(100000);
    }

    rt_task_join(&mosq_task);

}
