/* 
 * framing.c: 
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
 */

#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

#include <mosquitto.h>

#include <blast.h>
#include <channels_tng.h>
#include <blast_time.h>

static int frame_stop;
static pthread_t frame_thread;
static struct mosquitto *mosq = NULL;

static void frame_handle_data(const char *m_fc, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_lookup_t *rate;
    SRC_lookup_t *src;

    if (!m_fc || !m_rate) {
        bprintf(err, "Err in pointers\n");
        return;
    }
    if (!m_len) {
        bprintf(warning, "Zero-length string for frame\n");
        return;
    }

    for (rate = RATE_lookup_table; rate->position < RATE_END; rate++) {
        if (strcmp(rate->text, m_rate) == 0) break;
    }
    if (rate->position == RATE_END) {
        bprintf(warning, "Did not recognize rate %s!\n", m_rate);
        return;
    }

    for (src = SRC_lookup_table; src->position < SRC_END; src++) {
        if (strncmp(src->text, m_fc, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
    }
    if (src->position == SRC_END) {
        printf("Did not recognize source %s\n", m_fc);
        return;
    }
    channels_store_data(src->position, rate->position, m_data, m_len);

}

static void frame_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    char **topics;
    int count;

    if(message->payloadlen){
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {

            if ( count == 3 && strcmp(topics[1], "frames") == 0) {
                frame_handle_data(topics[0], topics[2], message->payload, message->payloadlen);
            }

            mosquitto_sub_topic_tokens_free(&topics, count);
        }

    }
    fflush(stdout);
}

static void frame_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
    if(!result){
        /* Subscribe to broker information topics on successful connect. */
        mosquitto_subscribe(mosq, NULL, "$SYS/#", 2);
    }else{
        berror(err, "Connect failed");
    }
}

static void frame_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;

    bprintf(info, "Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        bprintf(info, "\t %d", granted_qos[i]);
    }
}

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        bprintf(info, "%s\n", str);
}


static void *framing_routine(void *m_arg)
{
    int ret;
    int counter_100hz = 1;
    int counter_5hz=40;
    int counter_1hz=200;
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 5000000}; /// 200HZ interval

    printf("Starting Framing task\n");

    clock_gettime(CLOCK_REALTIME, &ts);

    while (!frame_stop)
    {
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (ret && ret != -EINTR)
        {
            bprintf(err, "error while sleeping, code %d (%s)\n", ret, strerror(-ret));
            break;
        }
        if (frame_size[SRC_FC][RATE_200HZ]) {
            mosquitto_publish(mosq, NULL, "uei_of/frames/200HZ",
                    frame_size[SRC_OF_UEI][RATE_200HZ], channel_data[SRC_OF_UEI][RATE_200HZ],0, false);
        }

        if (!counter_100hz--) {
            counter_100hz = 1;
            if (frame_size[SRC_OF_UEI][RATE_100HZ]) {
                mosquitto_publish(mosq, NULL, "uei_of/frames/100HZ",
                        frame_size[SRC_OF_UEI][RATE_100HZ], channel_data[SRC_OF_UEI][RATE_100HZ],0, false);
            }
        }
        if (!counter_5hz--) {
            counter_5hz = 40;
            if (frame_size[SRC_OF_UEI][RATE_5HZ]) {
                mosquitto_publish(mosq, NULL, "uei_of/frames/5HZ",
                        frame_size[SRC_OF_UEI][RATE_5HZ], channel_data[SRC_OF_UEI][RATE_5HZ], 0, false);
            }
        }
        if (!counter_1hz--) {
            counter_1hz = 200;
            if (frame_size[SRC_OF_UEI][RATE_5HZ]) {
                mosquitto_publish(mosq, NULL, "uei_of/frames/1HZ",
                        frame_size[SRC_OF_UEI][RATE_1HZ], channel_data[SRC_OF_UEI][RATE_1HZ], 0, false);
            }
        }

        if ((ret = mosquitto_loop(mosq, 0, 1)) != MOSQ_ERR_SUCCESS) {
            bprintf(err, "Received %d from mosquitto_loop", ret);
        }
    }

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

}

/**
 * Initializes the mosquitto library and associated framing routines.
 * @return
 */
int framing_init(void)
{
    const char *id = "fc1";
    const char *host = "fc1";
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    channels_initialize(NULL);

    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if (!mosq) {
        perror("mosquitto_new() failed");
        return -1;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);

    mosquitto_connect_callback_set(mosq, frame_connect_callback);
    mosquitto_message_callback_set(mosq, frame_message_callback);
    mosquitto_subscribe_callback_set(mosq, frame_subscribe_callback);

    if (mosquitto_connect(mosq, host, port, keepalive)) {
        fprintf(stderr, "Unable to connect.\n");
        return -1;
    }

    mosquitto_subscribe(mosq, NULL, "fc1/frames/#", 2);
    mosquitto_subscribe(mosq, NULL, "fc2/frames/#", 2);
    mosquitto_subscribe(mosq, NULL, "uei_if/frames/#", 2);
    mosquitto_subscribe(mosq, NULL, "uei_of/frames/#", 2);


    pthread_create(&frame_thread, NULL, &framing_routine, NULL);
    pthread_detach(frame_thread);
    return 0;
}

void framing_shutdown(void)
{
    frame_stop = 1;
}

