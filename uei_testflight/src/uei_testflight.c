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

#define _GNU_SOURCE 1
#include <features.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include <PDNA.h>

#include <minmea.h>


// Acquired samples are stored  in a circular buffer
// which is dumped to a file when the program terminates
#define CIRCULAR_BUFFER_SIZE  10000

#define BASE_SLOT    (layers[6].devn)
#define IRIG650_SLOT (layers[5].devn)
#define AI225_SLOT   (layers[4].devn)
#define DO432_SLOT   (layers[3].devn)
#define SL508_SLOT   (layers[2].devn)
#define DI448_SLOT   (layers[1].devn)
#define AI201_SLOT   (layers[0].devn)

typedef struct {
    uint32_t    serial;
    int         devn;
} layers_t;

static layers_t layers[7] = {
        {.serial =  111058 }, /** AI-201 **/
        {.serial =   64693 }, /** DIO-448 **/
        {.serial =   90721 }, /** SL-508 **/
        {.serial =   94276 }, /** DIO-432 **/
        {.serial =  121256 }, /** AI-225 **/
        {.serial =  118317 }, /** IRIG-650 **/
        {.serial =  119384 }  /** PWR Diag **/
};

static bool stop = 0;
static double frequency = 1000.0;
static int hd;
static  DQRDCFG *DQRdCfg = NULL;

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

    FILE *fp;
    char *filename = "/mnt/data/async.dat";

    struct timespec timespec;
    uint32_t last_status = 0;

    fp = fopen(filename, "a");

    printf("Starting ASYNC thread\n");
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    fprintf(fp, "%ld.%ld: Restart ASYNC Stream\n", (long) timespec.tv_sec, (long) timespec.tv_nsec);

    while (!stop) {
        int flags = 0;
        EV650_STS event_sts;

        clock_gettime(CLOCK_MONOTONIC, &timespec);
        ret = DqAdv650GetEventStatus(hd, IRIG650_SLOT, CT650_EVENT_CH0, flags, &event_sts);
        if (ret < 0) {
            printf("Error %d in DqAdv650GetEventStatus\n", ret);
        }
        if (last_status != event_sts.event_sts) {
            fprintf(fp, "EvtSts: %ld.%ld: AD:%x STS:%x TStamp:%x\n",
                    (long) timespec.tv_sec, (long) timespec.tv_nsec,
                    event_sts.event_ad, event_sts.event_sts,
                    event_sts.event_tstamp);
            last_status = event_sts.event_sts;
        }
        usleep(50);

    }
    fclose(fp);

    return NULL;
}



void* read_GPS_serial_line(void *arg)
{
    int ret = DQ_SUCCESS;
    sigset_t set;
    char buffer[DQ_MAX_UDP_PAYLOAD_100];
    uint8_t term[2] = {0x0D, 0x0A};
    FILE *fp;
    char *filename = "/mnt/data/gps_serial.dat";

    struct timeval tv1;
    struct timespec next;
    long long periodns = 10LL * NSECS_PER_SEC;

    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    fp = fopen(filename, "a");

    // Set channel configuration
    if ((ret = DqAdv501SetChannelCfg(hd, SL508_SLOT, 0,
                DQCFG_501(DQ_SL501_OPER_NORM,
                          DQ_SL501_MODE_232,
                          DQ_SL501_BAUD_4800,
                          DQ_SL501_WIDTH_8,
                          DQ_SL501_STOP_1,
                          DQ_SL501_PARITY_NONE))) < 0) {
        printf("error %d in DqAdv501SetChannelCfg()\n", ret);
    }

    // Specifies how long the SL-50x waits for the requested amount of bytes to be
    // received before only returning the bytes received so far
    if ((ret = DqAdv501SetTimeout(hd, SL508_SLOT, 0, 10000)) < 0) {
        printf("error %d in DqAdv501SetTimeout\n", ret);
    }

    // Specifies the maximum of bytes to keep in the FIFO before returning them to
    // the user
    if ((ret = DqAdv501SetTermLength(hd, SL508_SLOT, 0, 1)) < 0) {
        printf("error %d in DqAdv501SetTermLength\n", ret);
    }

    if ((ret = DqAdv501SetTermString(hd, SL508_SLOT, 0, 2, term)) < 0) {
        printf("error %d in DqAdv501SetTermString\n", ret);
    }

    if ((ret = DqAdv501Enable(hd, SL508_SLOT, TRUE)) < 0) {
        printf("Error %d in DqAdv501Enable()\n", ret);
    }

    gettimeofday(&tv1, NULL);
    clock_gettime(CLOCK_MONOTONIC, &next);

    while(!stop)
    {
        uint16_t buffer_length = DQ_MAX_UDP_PAYLOAD_100;
        uint16_t received = 0;
        int success;
        int has_more = 1;
        uint8_t error_code;
        uint8_t *bufp = (uint8_t*)buffer;

        while (received < MINMEA_MAX_LENGTH + 3) {
            buffer_length = DQ_MAX_UDP_PAYLOAD_100 - received;
            ret = DqAdv501RecvMessage(hd, SL508_SLOT, 0, bufp, &buffer_length, &success, &error_code, &has_more);
            received += buffer_length;

            /**
             * Check to ensure our line starts with a valid character for the GPS read
             */
            if (buffer[0] != '$') received = 0;

            bufp = (uint8_t*)buffer + received;
            if(ret < 0)
            {
                printf("Error %d receiving from serial port\n", ret);
                break;
            }
            if ((received > 4) &&
                    (*(bufp - 1) == 0x0A) &&
                    (*(bufp - 2) == 0x0D))
                break;
        }

        *bufp = '\0';

        if (success) {

            switch (minmea_sentence_id(buffer, false)) {
                case MINMEA_SENTENCE_RMC: {
                    struct minmea_sentence_rmc frame;
                    if (minmea_parse_rmc(&frame, buffer)) {
                        fprintf(fp, "$RMC: %ld.%ld\t coordinates and speed: (%f,%f) %f\n",
                                (long) next.tv_sec, (long) next.tv_nsec,
                                minmea_tocoord(&frame.latitude),
                                minmea_tocoord(&frame.longitude),
                                minmea_tofloat(&frame.speed)
                                );
                    }else {
                        fprintf(fp, "$RMC sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GGA: {
                    struct minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, buffer)) {
                        fprintf(fp, "$GGA: %ld.%ld\t fix quality: %d\t Altitude: %0.8f\t Time: %02d:%02d:%02d.%06d\n",
                                (long) next.tv_sec, (long) next.tv_nsec,
                                frame.fix_quality,
                                minmea_tofloat(&frame.altitude),
                                frame.time.hours, frame.time.minutes, frame.time.seconds, frame.time.microseconds);
                    }
                    else {
                        fprintf(fp, "$GGA sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GST: {
                    struct minmea_sentence_gst frame;
                    if (minmea_parse_gst(&frame, buffer)) {
                        fprintf(fp, "$GST: %ld.%ld\t floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)\n",
                                (long) next.tv_sec, (long) next.tv_nsec,
                                minmea_tofloat(&frame.latitude_error_deviation),
                                minmea_tofloat(&frame.longitude_error_deviation),
                                minmea_tofloat(&frame.altitude_error_deviation));
                    }
                    else {
                        fprintf(fp, "$GST sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GSV: {
                    struct minmea_sentence_gsv frame;
                    if (minmea_parse_gsv(&frame, buffer)) {
                        if (frame.msg_nr == frame.total_msgs)
                            fprintf(fp, "$GSV: %ld.%ld\t satellites in view: %d\n",
                                (long) next.tv_sec, (long) next.tv_nsec, frame.total_sats);
                    }
                    else {
                        fprintf(fp, "$GSV sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GSA: {
                    struct minmea_sentence_gsa frame;
                    if (minmea_parse_gsa(&frame, buffer)) {
                        fprintf(fp, "$GSA: %ld.%ld\t Fix quality:\t %f, %f, %f\n",
                                (long) next.tv_sec, (long) next.tv_nsec,
                                minmea_tofloat(&frame.pdop),
                                minmea_tofloat(&frame.hdop),
                                minmea_tofloat(&frame.vdop));
                    }
                    else {
                        fprintf(fp, "$GSA sentence not parsed\n");
                    }
                } break;

                case MINMEA_INVALID: {
                    fprintf(fp, "$xxxxx sentence is not valid:\n\t%s\n", buffer);
                } break;

                default: {
                    buffer[6] = '\0';
                    fprintf(fp, "%s sentence is not parsed\n", buffer);
                } break;

            }
        }

        if (!has_more) {
            timespec_add_ns(&next, periodns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        }
    }

    fclose(fp);

    return NULL;
}

int get_status(FILE *m_fp)
{
    uint8 devNum = 0x7E | DQ_LASTDEV;
    uint32 entries = 1;
    uint32 status[4*(DQ_MAXDEVN+1)]; // 4 uint32s per layer plus IOM
    uint32 statusSize = sizeof(status)/4;
    int ret = DQ_SUCCESS;
    int i;
    struct timespec cur_time;

    DqCmdReadStatus(hd, &devNum, &entries, status, &statusSize);
    clock_gettime(CLOCK_MONOTONIC, &cur_time);

    for (i = 0; i < 8; i++ ) {
        fprintf(m_fp, "%lu:%lu, Layer %d, Model %d, ST:%.8x, POST:%.8x, FW:%.8x, LG:%.8x\n",
                (unsigned long)cur_time.tv_sec, (unsigned long)cur_time.tv_nsec, i,
                DQRdCfg->devmod[i],
                status[STS_STATE], status[STS_POST], status[STS_FW], status[STS_LOGIC]);
    }

    return ret;
}

int read_GPS_IRIG(FILE *m_fp)
{
    char data[256] = {0};
    int ret_size = 0;
    uint32_t gps_status = 0;
    uint32_t status = 0;
    uint32_t time_val = 0;
    uint32_t date_val = 0;
    int ret = DQ_SUCCESS;
    struct timespec cur_time;

    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    do {
        ret_size = 0;
        ret = DqAdv650ReadGPS(hd, IRIG650_SLOT, 0, 255, data, &ret_size);
        if (ret < 0) {
            printf("Error %d in DqAdv650ReadGPS\n", ret);
        }
        data[ret_size] = 0; data[255] = 0;
        if (ret_size > 0 )
            fprintf(m_fp, "%ld.%ld: %s\n", (long) cur_time.tv_sec, (long) cur_time.tv_nsec, data);
    } while (ret_size > 0);

    ret = DqAdv650GetGPSStatus(hd, IRIG650_SLOT, 0, &gps_status, &status, &time_val, &date_val);
    if (ret < 0) {
        printf("Error %d in DqAdv650GetGPSStatus\n", ret);
    }

    fprintf(m_fp, "Status %ld.%ld: %08X, %08X, %08X, %08X\n",
            (long) cur_time.tv_sec, (long) cur_time.tv_nsec, gps_status, status, time_val, date_val);

    return ret;
}

int main(int argc, char* argv[]) {
    int i, count = 0;
    int ret;
    uint32_t status;
    int dmapid;
    uint32_t last_pps = UINT32_MAX;

    struct timeval tv1;
    struct timespec next;
    long long periodns;

    uint32_t input32[64];

    double input_buffer[64];
    size_t num_channels[15] = { 0 };

    uint32 chentry;

    FILE *gps_fp;
    FILE *analog_fp;
    FILE *pps_fp;
    FILE *status_fp;

    pthread_t async_t;
    pthread_t gps_t;

    time_t last_secs = 0;

    gps_fp = fopen("/mnt/data/gps.dat", "a");
    analog_fp = fopen("/mnt/data/analog.dat", "a");
    pps_fp = fopen("/mnt/data/pps.dat", "a");
    status_fp = fopen("/mnt/data/layer_status.dat", "a");

    signal(SIGINT, sighandler);

    ret = DqInitDAQLib();
    if (ret < 0) {
        printf("Error %d in DqInitDAQLib\n", ret);
        return ret;
    }
    ret = DqOpenIOM("127.0.0.1", DQ_UDP_DAQ_PORT, 1000, &hd, &DQRdCfg);
    if (ret < 0) {
        printf("Error %d in DqOpenIOM\n", ret);
        return ret;
    }

    for (i = 0; i < 7; i++)
    {
        uint32_t devn;
        DqGetDevnBySerial(hd, layers[i].serial, &devn, NULL, NULL, NULL);
        layers[i].devn = (int)devn;

        if (DQRdCfg->devmod[layers[i].devn])
        {
            printf("Model: %x Option: %x Dev: %d SN: %u\n",
                    DQRdCfg->devmod[layers[i].devn], DQRdCfg->option[layers[i].devn],layers[i].devn, layers[i].serial);
        }
    }

    /**
     * Set the initial watchdog timer to 25 seconds.  This should be enough time
     * for us to get into the main routine
     */
   // DqCmdSetWatchDog(hd, DQ_WD_CLEAR_ON_OSTASK, 25000, NULL, NULL);

    pthread_create(&async_t, NULL, async_thread, NULL);
    pthread_detach(async_t);

    pthread_create(&gps_t, NULL, read_GPS_serial_line, NULL);
    pthread_detach(gps_t);

    /**
     * Allow handling of GPS data by the 650 internal interrupts
     */

    ret = DqAdv650EnableGPSTracking(hd, IRIG650_SLOT, true, &status);
    if (ret < 0) {
        printf("Error %d in DqAdv650EnableGPSTracking\n", ret);
    } else {
//        while (!(status & CT650_GPS_ACC_GPSFIXV)) {
//            uint32_t status2 =0;
//            printf("Got GPS Status 0x%X, 0x%X\n", status, status2);
//            sleep(1);
//            ret = DqAdv650GetGPSStatus(hd, IRIG650_SLOT, 0, &status, &status2, NULL, NULL);
//        }
    }


    /**
     * Get the PPS source from the GPS module
     * Set the flags to automatically continue if source is lost
     */

    uint32 timekeeper_mode, timekeeper_flags;
    EV650_CFG event_cfg;

    timekeeper_mode = CT650_TKPPS_GPS;
    timekeeper_flags = CT650_TKFLG_AUTOFOLLOW | CT650_TKFLG_USENOMINAL;
    ret = DqAdv650ConfigTimekeeper(hd, IRIG650_SLOT, timekeeper_mode, timekeeper_flags);
    if (ret < 0) {
        printf("Error %d in DqAdv650ConfigTimekeeper\n", ret);
    }

    /**
     * Configure the IRIG PPS Clock event
     */
    memset(&event_cfg, 0, sizeof(event_cfg));
    event_cfg.event_cfg = CT650_EVT_CFG_EN|CT650_EVT_CFG_EV1IRQ|CT650_EVT_CFG_EV0IRQ|
                          CT650_EVT_CFG_RPT|
                          CT650_EVT_CFG_EDGE|
                          CT650_EVT_PPS;

    ret = DqAdv650SetEvents(hd, IRIG650_SLOT, CT650_EVENT_CH0, UINT32_MAX, &event_cfg, NULL);
    if (ret < 0) {
        printf("Error %d in DqAdv650ConfigEvents (PPS CLK): Errno: %d\n", ret, errno);
    }

    /**
     * Single buffer mode
     * TTL0 outputs the GPS PPS
     * TTL1 outputs the IRIG PPS
     */
    ret = DqAdv650AssignTTLOutputs(hd, IRIG650_SLOT, CT650_OUT_TTLEN0, 0,
    CT650_OUT_CFG_SRC_1GPS, CT650_OUT_CFG_SRC_1PPS, 0, 0);
    if (ret < 0) {
        printf("Error %d in DqAdv650AssignTTLOutputs\n", ret);
    }

    /**
     * Enable the IRIG-650 card
     */
    ret = DqAdv650Enable(hd, IRIG650_SLOT, true);
    if (ret < 0) {
        printf("Error %d in DqAdv650Enable\n", ret);
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
        chentry = i | DQ_AI201_GAIN_1;
        DqRtDmapAddChannel(hd, dmapid, AI201_SLOT, DQ_SS0IN, &chentry, 1);
        num_channels[AI201_SLOT]++;
    }

    /**
     * Add the DIO-448.  First two channels are for Digital Input but the
     * remaining channels are single-ended analog
     */
    uint32_t aiCl[DQ_DIO448_LINES];
    for (i=0; i<DQ_DIO448_LINES; i++) {
       aiCl[i] = i;
    }
    // Configure hysteresis on all 48 input lines
    DqAdv448SetLevels(hd, DI448_SLOT, DQ_DIO448_LINES, aiCl, 1.0, 3.0);

    // Set debouncer time interval for all channels
    // interval is set in 100usecs increment
    DqAdv448SetDebouncer(hd, DI448_SLOT, DQ_DIO448_LINES, aiCl, 10);

    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DI448_SLOT]++;

    // Get the first de-bounced channel
    chentry = 2;
    DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DI448_SLOT]++;

    // Get the second de-bounced channel
    chentry = 3;
    DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DI448_SLOT]++;

    chentry = DQL_CHAN448_STATUS;
    DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DI448_SLOT]++;

    for (i = DQL_CHAN448_CHSE(0); i < DQL_CHAN448_CHSE(48); i++) {
        chentry = i;
        DqRtDmapAddChannel(hd, dmapid, DI448_SLOT, DQ_SS0IN, &chentry, 1);
        num_channels[DI448_SLOT]++;
    }

    /**
     * Add the DO-432 analog input channels (single ended)
     */
    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, DO432_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[DO432_SLOT]++;

    for (i = 32; i < 64; i++) {
        chentry = i;
        DqRtDmapAddChannel(hd, dmapid, DO432_SLOT, DQ_SS0IN, &chentry, 1);
        num_channels[DO432_SLOT]++;
    }

    /**
     * Add the AI-225 analog input channels (single ended)
     */
    chentry = DQ_LNCL_TIMESTAMP;
    DqRtDmapAddChannel(hd, dmapid, AI225_SLOT, DQ_SS0IN, &chentry, 1);
    num_channels[AI225_SLOT]++;

    for (i = 0; i < 25; i++) {
        chentry = i;
        DqRtDmapAddChannel(hd, dmapid, AI225_SLOT, DQ_SS0IN, &chentry, 1);
        num_channels[AI225_SLOT]++;
    }

    // Start the layers
    DqRtDmapStart(hd, dmapid);

    gettimeofday(&tv1, NULL);
    clock_gettime(CLOCK_MONOTONIC, &next);

    fprintf(gps_fp, "%ld.%ld:\tRestart GPS Stream\n", (long) next.tv_sec, (long) next.tv_nsec);
    fprintf(analog_fp, "%ld.%ld:\tRestart Analog Stream\n", (long) next.tv_sec, (long) next.tv_nsec);

    printf("Starting loop\n");

    while (!stop) {

        DqRtDmapRefresh(hd, dmapid);

        DqRtDmapReadRawData32(hd, dmapid, DI448_SLOT, input32, num_channels[DI448_SLOT]);
//        printf("0x%0.8X, 0x%0.8X, 0x%0.8X, 0x%0.8X\n", input32[0], input32[1], input32[2], input32[3]);
        if ((input32[1] & 0x7) != last_pps) {
            last_pps = input32[1] & 0x7;
            fprintf(pps_fp, "%ld.%ld, 0x%02X\n", (long) next.tv_sec, (long) next.tv_nsec, last_pps);
        }

        /**
         * Reset the watchdog timer for 10 seconds
         */
        //DqCmdSetWatchDog(hd, DQ_WD_CLEAR_ON_OSTASK, 10000, NULL, NULL);

        // read inputs retrieved by the last refresh once every 10 seconds
        if ((last_secs != next.tv_sec) && ((next.tv_sec % 10) == 0)) {
            last_secs = next.tv_sec;
            for (i = 0; i < 15; i++) {
                if (num_channels[i] <= 0) continue;

                DqRtDmapReadScaledData(hd, dmapid, i, input_buffer, num_channels[i]);

                fprintf(analog_fp, "Board %x, %ld.%ld, ", DQRdCfg->devmod[i], (long) next.tv_sec, (long) next.tv_nsec);
                for (int j = 0; j < num_channels[i]; j++) {
                    fprintf(analog_fp, "%0.8f, ", input_buffer[j]);
                }
                fprintf(analog_fp, "\n");

            }

            {
                uint32_t diag_channels[13] = { 0, 1, 2, 4, 6, 7, 8, 9, 10, 12, 13, 14, 15 };
                uint32_t bdata[13];
                ret = DqAdvDnxpRead(hd, BASE_SLOT, 13, diag_channels, bdata, input_buffer);
                if (ret < 0) {
                    printf("Error %d in DqAdvDnxpRead\n", ret);
                }
                fprintf(analog_fp, "Diag Layer, %ld.%ld, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f\n",
                        (long) next.tv_sec, (long) next.tv_nsec,
                        input_buffer[0], input_buffer[1], input_buffer[2], input_buffer[3],
                        input_buffer[4], input_buffer[5], input_buffer[6], input_buffer[7],
                        input_buffer[8], input_buffer[9], input_buffer[10], input_buffer[11], input_buffer[12]);
            }


            read_GPS_IRIG(gps_fp);

            get_status(status_fp);

        }
        count++;

        timespec_add_ns(&next, periodns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    /**
     * Turn off the watchdog timer now that we have called the exit
     */
   // DqCmdSetWatchDog(hd, DQ_WD_CLEAR_DISABLED, 0, NULL, NULL);

    DqRtDmapStop(hd, dmapid);

    DqRtDmapClose(hd, dmapid);

    DqCloseIOM(hd);
    DqCleanUpDAQLib();

    fclose(gps_fp);
    fclose(status_fp);
    fclose(analog_fp);
    fclose(pps_fp);

    return 0;
}
