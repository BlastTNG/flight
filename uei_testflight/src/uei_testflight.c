/* 
 * uei_testflight.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of uei_testflight, created for the BLASTPol Project.
 *
 * uei_testflight is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * uei_testflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with uei_testflight; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 28, 2015 by Seth Hillbrand
 */

#include <stdlib.h>
#include <stdio.h>

#include <stdint.h>
#include <stdbool.h>

#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>

#include <PDNA.h>


// Acquired samples are stored  in a circular buffer
// which is dumped to a file when the program terminates
#define CIRCULAR_BUFFER_SIZE  10000

#define BASE_SLOT    14
#define IRIG650_SLOT 6
#define AI225_SLOT   5
#define DO432_SLOT   4
#define SL508_SLOT   3
#define DI448_SLOT   2
#define AI201_SLOT   1

static bool stop = 0;
static double frequency = 1.0;
static int hd;
static int async_hd;
static bool gps_has_data = false;

// Add specified amount of ns to timespec
static inline void timespec_add_ns(struct timespec *m_time, unsigned int m_ns)
{
#define NSECS_PER_SEC 1000000000L
    m_ns += m_time->tv_nsec;
    while (m_ns >= NSECS_PER_SEC)
    {
        m_ns -= NSECS_PER_SEC;
        m_time->tv_sec++;
    }
    m_time->tv_nsec = m_ns;
}

// Handler for SIGINT
void sighandler(int m_sig)
{
    stop = true;
}

void *async_thread(void* m_args) {
    int ret;

    int size;
    pDQEVENT pEvent;
    pEV650_ID pEv650;
    pDQRDCFG cube_config;

    FILE *fp;
    char filename = "/data/async.dat";

    uint32_t error_mask = 0;
    struct timespec timespec;
    bool received_error = false;

    fp = fopen(filename, "a");

    clock_gettime(CLOCK_MONOTONIC, &timespec);
    fprintf(fp, "%ld.%ld:\tRestart ASYNC Stream\n", (long) timespec.tv_sec, (long) timespec.tv_nsec);

    DqRtAsyncOpenIOM(hd, &async_hd, DQ_UDP_DAQ_PORT_ASYNC, 500, 0, &cube_config);

    /**
     * Configure the IRIG error event to happen once until we deal with it
     */
    DqAdv650ConfigEvents(async_hd, IRIG650_SLOT, CT650_EVENT_CHERR, DQEVENT_ONCE, EV650_ERROR, &error_mask);

    /**
     * Configure the IRIG PPS Clock event
     */
    DqAdv650ConfigEvents(async_hd, IRIG650_SLOT, CT650_EVENT_CHPPS, 0, EV650_PPS_CLK, NULL);

    /**
     * Configure the IRIG GPS receive data event
     */
    DqAdv650ConfigEvents(async_hd, IRIG650_SLOT, CT650_EVENT_CH0, 0, EV650_GPSRX, NULL);

    while (!stop) {
        pEvent = NULL;
        ret = DqCmdReceiveEvent(async_hd, 0, 1000 * 1000, &pEvent, &size);
        if ((ret < 0) && (ret != DQ_TIMEOUT_ERROR)) {
            fprintf(fp, "%d:\tERR: %s\n", (int) time(NULL), DqTranslateError(ret));
        }
        if (ret >= 0) {
            clock_gettime(CLOCK_REALTIME, &timespec);
            switch (pEvent->dev) {
                case IRIG650_SLOT:
                {
                    pEV650_ID pEv650 = (pEV650_ID) pEvent->data;
                    switch (pEvent->event) {
                        case EV650_ERROR:
                            fpritnf(fp, "%ld.%ld:%u\tError Event:", (long) timespec.tv_sec, (long) timespec.tv_nsec,
                                    pEv650->tstamp);
                            fpritnf(fp, "\t%08X,%08X,%08X,%08X\n", pEv650->data[0], pEv650->data[1], pEv650->data[2],
                                    pEv650->data[3]);
                            received_error = true;
                            break;

                        case EV650_PPS_CLK:
                            fpritnf(fp, "%ld.%ld:%u\tPPS Event\n", (long) timespec.tv_sec, (long) timespec.tv_nsec,
                                    pEv650->tstamp);

                            /**
                             * Re-enable the errors only once per second
                             */
                            if (received_error) {
                                DqAdv650ConfigEvents(async_hd, IRIG650_SLOT, CT650_EVENT_CHERR, DQEVENT_ONCE,
                                        EV650_ERROR, &error_mask);
                                received_error = false;
                            }
                            break;

                        case EV650_GPSRX:
                            fpritnf(fp, "%ld.%ld:%u\tGPS Event\n", (long) timespec.tv_sec, (long) timespec.tv_nsec,
                                    pEv650->tstamp);
                            gps_has_data = true;

                    }
                    break;
                    default:
                    break;
                }
            }
        }
    }
}

int main(int argc, char* argv[]) {
    int i, dev, ch, count = 0;
    int ret;
    int dmapid;
    DQRDCFG *DQRdCfg = NULL;

    struct sched_param schedp;
    struct timeval tv1, tv2;
    double duration;
    struct timespec next;
    long long periodns;

    double input_buffer[64];
    size_t num_channels[15] = { 0 };

    uint32 timekeeper_mode, timekeeper_flags;
    uint32 timecode_mode, timecode_output;
    uint32 tdMode, tdInput;
    uint32 status;

    uint32 chentry;

    FILE *gps_fp;
    FILE *analog_fp;

    gps_fp = fopen("/data/gps.dat", "a");
    analog_fp = fopen("/data/analog.dat", "a");

    signal(SIGINT, sighandler);

    // Configure this process to run with the real-time scheduler
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    DqInitDAQLib();
    DqOpenIOM("127.0.0.1", DQ_UDP_DAQ_PORT, 1000, &hd, &DQRdCfg);

    /**
     * Get the PPS source from the GPS module
     * Set the flags to automatically continue if source is lost
     */
    timekeeper_mode = CT650_TKPPS_GPS;
    timekeeper_flags = CT650_TKFLG_AUTOFOLLOW | CT650_TKFLG_USENOMINAL;
    ret = DqAdv650ConfigTimekeeper(hd, IRIG650_SLOT, timekeeper_mode, timekeeper_flags);
    if (ret < 0) {
        printf("Error %d in DqAdv650ConfigTimekeeper\n", ret);
    }

    /**
     * Allow handling of GPS data by the 650 internal interrupts
     */

    ret = DqAdv650EnableGPSTracking(hd, IRIG650_SLOT, true, &status);

    /**
     * Single buffer mode
     * TTL0 outputs the GPS PPS
     * TTL1 outputs the IRIG PPS
     */
    ret = DqAdv650AssignTTLOutputs(hd, IRIG650_SLOT, CT650_OUT_TTLEN0, 0,
    CT650_OUT_CFG_SRC_1GPS, CT650_OUT_CFG_SRC_1PPS, 0, 0);
    if (ret < 0) {
        printf("Error %d in DqAdv650SetTimecodeOutput\n", ret);
    }

    // Use posix timer to tick at the desired frequency
    periodns = (long long) floor(1000000000.0 / frequency);
    DqRtDmapInit(hd, &dmapid, 2 * frequency);

    /**
     * Add the AI-201, using the first channel as a timestamp
     */
    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, AI201_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[AI201_SLOT]++;

    for (i = 0; i < 24; i++) {
        chentry = i | DQ_LNCL_GAIN(0);
        DqRtDmapAddChannel(hd, dmapid, AI201_SLOT, DQ_SS0IN, &chentry, 1);
        num_channels[AI201_SLOT]++;
    }

    /**
     * Add the DIO-448.  First two channels are for Digital Input but the
     * remaining channels are single-ended analog
     */

    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DI448_SLOT]++;

    for (i = DQL_CHAN448_CHSE(2); i < DQL_CHAN448_CHSE(48); i++) {
        DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, i, 1);
        num_channels[DI448_SLOT]++;
    }

    /**
     * Add the DO-432 analog input channels (single ended)
     */
    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, DO432_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DO432_SLOT]++;

    for (i = 32; i < 64; i++) {
        DqRtDmapAddChannel(hd, dmapid, DO432_SLOT, DQ_SS0IN, i, 1);
        num_channels[DO432_SLOT]++;
    }

    /**
     * Add the AI-225 analog input channels (single ended)
     */
    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, AI225_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[AI225_SLOT]++;

    for (i = 0; i < 25; i++) {
        DqRtDmapAddChannel(hd, dmapid, AI225_SLOT, DQ_SS0IN, i, 1);
        num_channels[AI225_SLOT]++;
    }

    /**
     * Allocate and clear the buffer for reading the input dmap
     */
    input_buffer = (double*) malloc(num_channels * sizeof(double));
    memset(input_buffer, 0, num_channels * sizeof(double));

    // Start the layers
    DqRtDmapStart(hd, dmapid);

    gettimeofday(&tv1, NULL);
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (!stop) {
        DqRtDmapRefresh(hd, dmapid);

        // read inputs retrieved by the last refresh
        for (i = 0; i < 15; i++) {
            if (num_channels[i] <= 0) continue;

            DqRtDmapReadScaledData(hd, dmapid, i, input_buffer, num_channels[i]);

            fprintf(analog_fp, "Layer %d, ", i);
            for (int j = 0; j < num_channels[i]; j++) {
                fprintf("%0.8f, ", input_buffer[j]);
            }
            fprintf(analog_fp, "\n");

        }

        if (gps_has_data) {
            char data[256] = 0;
            gps_has_data = false;
        }

        count++;

        // print status once a second
        if (count && (0 == (count % (int) frequency))) {
            gettimeofday(&tv2, NULL);
            duration = ((tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec) / 1000000.0);
            printf("Acquired %d scans in %fs (%f scans/s)\n", count, duration, count / duration);
        }

        timespec_add_ns(&next, periodns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    DqRtDmapStop(hd, dmapid);

    DqRtDmapClose(hd, dmapid);

    DqCloseIOM(hd);
    DqCleanUpDAQLib();

    return 0;
}
