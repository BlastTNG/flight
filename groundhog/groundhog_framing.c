/* 
 * groundhog_framing.c: 
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
 * Created on: Aug 12, 2014 by seth
 * Adapted for groundhog by Joy Didier, Feb 2018
 */

#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

#include <mosquitto.h>

#include "blast.h"
#include "blast_time.h"
#include "channels_tng.h"
#include "groundhog_framing.h"
#include "crc.h"
#include "FIFO.h"
#include "derived.h"
#include "mputs.h"
#include "linklist_compress.h"

struct DownLinkStruct downlink[NUM_DOWNLINKS] = {
    [PILOT] = {"pilot"},
    [BI0] = {"bi0"},
    [HIGHRATE] = {"highrate"}
};

static int frame_stop;
static struct mosquitto *mosq = NULL;
pthread_mutex_t mqqt_lock;

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        blast_info("%s\n", str);
}

void framing_publish(void *m_frame, char *telemetry, E_RATE m_rate)
{    
    pthread_mutex_lock(&mqqt_lock); 

    static char frame_name[32];
    char rate_name[16];
    strcpy(rate_name, RATE_LOOKUP_TABLE[m_rate].text);
    rate_name[strlen(rate_name)-1] = 'z';
    snprintf(frame_name, sizeof(frame_name), "frames/%s/%s", telemetry, rate_name);
    // if (frame_stop) return;
    if (frame_size[m_rate]) {
        mosquitto_publish(mosq, NULL, frame_name,
                          frame_size[m_rate], m_frame, 0, false);
    }
  
    pthread_mutex_unlock(&mqqt_lock);
}

void framing_publish_1hz(void *m_frame, char *telemetry)
{
    static char frame_name[32];
    snprintf(frame_name, sizeof(frame_name), "frames/%s/1Hz", telemetry);
    // if (frame_stop) return;
    if (frame_size[RATE_1HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                          frame_size[RATE_1HZ], m_frame, 0, false);
    }
}

void framing_publish_5hz(void *m_frame, char *telemetry)
{
    static char frame_name[32];
    snprintf(frame_name, sizeof(frame_name), "frames/%s/5Hz", telemetry);
    // if (frame_stop) return;
    if (frame_size[RATE_5HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_5HZ], m_frame, 0, false);
    }
}

void framing_publish_100hz(void *m_frame, char *telemetry)
{
    static char frame_name[32];
    snprintf(frame_name, sizeof(frame_name), "frames/%s/100Hz", telemetry);
    // if (frame_stop) return;
    if (frame_size[RATE_100HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_100HZ], m_frame, 0, false);
    }
}

void framing_publish_200hz(void *m_frame, char *telemetry)
{
    static char frame_name[32];
    snprintf(frame_name, sizeof(frame_name), "frames/%s/200Hz", telemetry);
    // if (frame_stop) return;
    if (frame_size[RATE_200HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_200HZ], m_frame, 0, false);
    }
}

void framing_publish_244hz(void *m_frame, char *telemetry)
{
    static char frame_name[32];
    snprintf(frame_name, sizeof(frame_name), "frames/%s/244Hz", telemetry);
    // if (frame_stop) return;
    if (frame_size[RATE_244HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_244HZ], m_frame, 0, false);
    }
}

void framing_publish_488hz(void *m_frame, char *telemetry)
{
    static char frame_name[32];
    snprintf(frame_name, sizeof(frame_name), "frames/%s/488Hz", telemetry);
    // if (frame_stop) return;
    if (frame_size[RATE_488HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_488HZ], m_frame, 0, false);
    }
}

/**
 * Initializes the mosquitto library and associated framing routines.
 * @return
 */
// int framing_init(channel_t *channel_list, derived_tng_t *m_derived)
int framing_init(void)
{
    channel_header_t *channels_pkg = NULL;
    derived_header_t *derived_pkg = NULL;

    char id[20] = "groundhog_blastgs01";
    char host[10] = "localhost";
    char topic[64];

    int ret = 0;
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if (!mosq) {
        perror("mosquitto_new() failed");
        return -1;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);

    ret = mosquitto_connect_async(mosq, host, port, keepalive); 
    if (ret == MOSQ_ERR_SUCCESS) {
   	printf("Succesfully connected groundhog to mosquitto server on %s\n", host);
    } else if (ret == MOSQ_ERR_INVAL) {
        blast_info("Unable to connect to mosquitto server: Invalid Parameters!\n");
    } else {
        if (errno == EINPROGRESS) {
           /* Do nothing, connection in progress */
           blast_info("Connection in progress...\n");
        } else {
            blast_info("Unable to connect to mosquitto server: %s\n", strerror(errno));
            return -1;
        }
    }

    /**
     * Set up the channels and derived packages for subscribers
     */ 
    if (!(channels_pkg = channels_create_map(channel_list))) {
        blast_info("Exiting framing routine because we cannot get the channel list");
        return -1;
    }

    mosquitto_reconnect_delay_set(mosq, 1, 10, 1);
    mosquitto_loop_start(mosq);


    for (int i = 0; i < NUM_DOWNLINKS; i++ ) {
        snprintf(topic, sizeof(topic), "channels/%s", downlink[i].name);
        mosquitto_publish(mosq, NULL, topic,
                sizeof(channel_header_t) + channels_pkg->length * sizeof(struct channel_packed), channels_pkg, 1, true);

        if (!(derived_pkg = channels_create_derived_map(derived_list))) {
            blast_info("Failed sending derived packages for telemetry %s\n", downlink[i].name);
        } else {
            snprintf(topic, sizeof(topic), "derived/%s", downlink[i].name);
            mosquitto_publish(mosq, NULL, topic,
                    sizeof(derived_header_t) + derived_pkg->length * sizeof(derived_tng_t), derived_pkg, 1, true);
            bfree(err, derived_pkg);
        }
    }
    bfree(err, channels_pkg);

    // initialize the mutex
    if (pthread_mutex_init(&mqqt_lock, NULL) != 0) {
        blast_fatal("Unable to initialized mutex to MQQT server");
        exit(2);
    }

    // initialize channel data for each downlink
    for (int rate = 0; rate < RATE_END; rate++) {
        size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
				char rate_name[16];
				strcpy(rate_name, RATE_LOOKUP_TABLE[rate].text);
				rate_name[strlen(rate_name)-1] = 'z';

        for (int i = 0; i < NUM_DOWNLINKS; i++) {
            if (rate == 0) allocFifo(&downlink[i].fifo, NUM_FRAMES, superframe_size);
            downlink[i].data[rate] = calloc(1, allocated_size);

						sprintf(downlink[i].frame_name[rate], "frames/%s/%s", downlink[i].name, rate_name);
						blast_info("There will be a topic with name %s", downlink[i].frame_name[rate]);
        }
    }

    return 0;
}

#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))

void groundhog_publish(void *arg) {
    // data is available now to be published
    bool current_data[NUM_DOWNLINKS] = {false};

    // data is available for the next 1 Hz frame publishing loop
    bool new_data[NUM_DOWNLINKS] = {false};

		int counter_488hz = 1;
		int counter_244hz = 1;
		int counter_200hz = 1;
		int counter_100hz = 1;
		int counter_5hz = 1;
		int counter_1hz = 1;

	  int frame_488_counter = 0;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    while (true) {
        // check every time to see if there's new data 
        for (int i = 0; i < NUM_DOWNLINKS; i++) {
            new_data[i] = !fifoIsEmpty(&downlink[i].fifo);
        }

				const struct timespec interval_ts = {.tv_sec = 0, .tv_nsec = MCP_NS_PERIOD};
				ts = timespec_add(ts, interval_ts);
				clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

				if (!--counter_1hz) {
						counter_1hz = HZ_COUNTER(1);
            for (int i = 0; i < NUM_DOWNLINKS; i++) {
                if (current_data[i]) {
						        extract_frame_from_superframe(downlink[i].data[RATE_1HZ], RATE_1HZ, getFifoRead(&downlink[i].fifo));
						        framing_publish(downlink[i].data[RATE_1HZ], downlink[i].name, RATE_1HZ);

                    // decrement the fifo
                    decrementFifo(&downlink[i].fifo);
                    current_data[i] = false;
                }

                // queue new data to read
                if (new_data[i]) { 
                    current_data[i] = true;
                    new_data[i] = false;
                }
            }
						//printf("1Hz\n");
				}
				if (!--counter_5hz) {
						counter_5hz = HZ_COUNTER(5);
            for (int i = 0; i < NUM_DOWNLINKS; i++) {
                if (current_data[i]) {
						        extract_frame_from_superframe(downlink[i].data[RATE_5HZ], RATE_5HZ, getFifoRead(&downlink[i].fifo));
						        framing_publish(downlink[i].data[RATE_5HZ], downlink[i].name, RATE_5HZ);
                }
            }
						//printf("5Hz\n");
				}
				if (!--counter_100hz) {
						counter_100hz = HZ_COUNTER(100);
            for (int i = 0; i < NUM_DOWNLINKS; i++) {
                if (current_data[i]) {
						        extract_frame_from_superframe(downlink[i].data[RATE_100HZ], RATE_100HZ, getFifoRead(&downlink[i].fifo));
						        framing_publish(downlink[i].data[RATE_100HZ], downlink[i].name, RATE_100HZ);
                }
            }
						//printf("100Hz\n");
				}
				if (!--counter_200hz) {
						counter_200hz = HZ_COUNTER(200);
            for (int i = 0; i < NUM_DOWNLINKS; i++) {
                if (current_data[i]) {
						        extract_frame_from_superframe(downlink[i].data[RATE_200HZ], RATE_200HZ, getFifoRead(&downlink[i].fifo));
						        framing_publish(downlink[i].data[RATE_200HZ], downlink[i].name, RATE_200HZ);
                }
            }
						//printf("200Hz\n");
				}
				if (!--counter_244hz) {
						counter_244hz = HZ_COUNTER(244);
            for (int i = 0; i < NUM_DOWNLINKS; i++) {
                if (current_data[i]) {
						        extract_frame_from_superframe(downlink[i].data[RATE_244HZ], RATE_244HZ, getFifoRead(&downlink[i].fifo));
						        framing_publish(downlink[i].data[RATE_244HZ], downlink[i].name, RATE_244HZ);
                }
            }
						//printf("244Hz\n");
				}
				if (!--counter_488hz) {
						counter_488hz = HZ_COUNTER(488);
            for (int i = 0; i < NUM_DOWNLINKS; i++) {
                if (current_data[i]) {
						        extract_frame_from_superframe(downlink[i].data[RATE_488HZ], RATE_488HZ, getFifoRead(&downlink[i].fifo));
						        framing_publish(downlink[i].data[RATE_488HZ], downlink[i].name, RATE_488HZ);
                }
            }
						//printf("488Hz\n");
				}
    }
}

void framing_shutdown(void)
{
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, true);
    frame_stop = 1;
}

