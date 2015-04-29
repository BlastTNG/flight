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
#include <blast_time.h>
#include <channels_tng.h>
#include <derived.h>
#include <mputs.h>

static int frame_stop;
static pthread_t frame_thread;
static struct mosquitto *mosq = NULL;

static void frame_handle_data(const char *m_fc, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;
    SRC_LOOKUP_T *src;

    if (!m_fc || !m_rate) {
        bprintf(err, "Err in pointers\n");
        return;
    }
    if (!m_len) {
        bprintf(warning, "Zero-length string for frame\n");
        return;
    }

    for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
        if (strcmp(rate->text, m_rate) == 0) break;
    }
    if (rate->position == RATE_END) {
        bprintf(warning, "Did not recognize rate %s!\n", m_rate);
        return;
    }

    //TODO:Think about mapping FC1/FC2
    for (src = SRC_LOOKUP_TABLE; src->position < SRC_END; src++) {
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

    channel_t *mcp_1hz_framenum_addr;
    channel_t *mcp_5hz_framenum_addr;
    channel_t *mcp_100hz_framenum_addr;
    channel_t *mcp_200hz_framenum_addr;
    uint32_t mcp_1hz_framenum = 0;
    uint32_t mcp_5hz_framenum = 0;
    uint32_t mcp_100hz_framenum = 0;
    uint32_t mcp_200hz_framenum = 0;

    int counter_100hz = 1;
    int counter_5hz=40;
    int counter_1hz=200;
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 5000000}; /// 200HZ interval

    nameThread("framing");
    bprintf(startup, "Starting Framing task\n");

    clock_gettime(CLOCK_REALTIME, &ts);

    mcp_1hz_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
    mcp_5hz_framenum_addr = channels_find_by_name("mcp_5hz_framecount");
    mcp_100hz_framenum_addr = channels_find_by_name("mcp_100hz_framecount");
    mcp_200hz_framenum_addr = channels_find_by_name("mcp_200hz_framecount");
    //TODO: Move MOSQ publish routine into main loop
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
            mcp_200hz_framenum++;
            SET_UINT32(mcp_200hz_framenum_addr, mcp_200hz_framenum);
            mosquitto_publish(mosq, NULL, "frames/fc/1/200HZ",
                    frame_size[SRC_FC][RATE_200HZ], channel_data[SRC_FC][RATE_200HZ],0, false);
        }

        if (!counter_100hz--) {
            counter_100hz = 1;
            mcp_100hz_framenum++;
            SET_UINT32(mcp_100hz_framenum_addr, mcp_100hz_framenum);
            if (frame_size[SRC_FC][RATE_100HZ]) {
                mosquitto_publish(mosq, NULL, "frames/fc/1/100HZ",
                        frame_size[SRC_FC][RATE_100HZ], channel_data[SRC_FC][RATE_100HZ],0, false);
            }
        }
        if (!counter_5hz--) {
            counter_5hz = 39;
            mcp_5hz_framenum++;
            SET_UINT32(mcp_5hz_framenum_addr, mcp_5hz_framenum);
            if (frame_size[SRC_FC][RATE_5HZ]) {
                mosquitto_publish(mosq, NULL, "frames/fc/1/5HZ",
                        frame_size[SRC_FC][RATE_5HZ], channel_data[SRC_FC][RATE_5HZ], 0, false);
            }
        }
        if (!counter_1hz--) {
            counter_1hz = 199;
            mcp_1hz_framenum++;
            SET_UINT32(mcp_1hz_framenum_addr, mcp_1hz_framenum);
            if (frame_size[SRC_FC][RATE_1HZ]) {
                mosquitto_publish(mosq, NULL, "frames/fc/1/1HZ",
                        frame_size[SRC_FC][RATE_1HZ], channel_data[SRC_FC][RATE_1HZ], 0, false);
            }
        }

        if ((ret = mosquitto_loop(mosq, 0, 1)) != MOSQ_ERR_SUCCESS) {
            switch(ret) {
                case MOSQ_ERR_INVAL:
                    bprintf(err, "Invalid Parameters for mosquitto_loop");
                    break;
                case MOSQ_ERR_NOMEM:
                    bprintf(err, "Out of memory in mosquitto loop");
                    break;
                case MOSQ_ERR_NO_CONN:
                    bprintf(err, "Not connected");
                    //TODO: Implement state loop for mosquitto
                    break;
                case MOSQ_ERR_CONN_LOST:
                    bprintf(err, "Lost connection with mosquitto server");
                    break;
                case MOSQ_ERR_PROTOCOL:
                    bprintf(err, "Protocol error communicating with mosquitto server");
                    break;
                case MOSQ_ERR_ERRNO:
                    berror(err, "System error in mosquitto comms");
                    break;
                default:
                    bprintf(err, "Received %d from mosquitto_loop", ret);
            }

        }
    }

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return NULL;
}

/**
 * Initializes the mosquitto library and associated framing routines.
 * @return
 */
int framing_init(channel_t *channel_list, derived_tng_t *m_derived)
{
    channel_header_t *channels_pkg = NULL;
    derived_header_t *derived_pkg = NULL;

    const char *id = "fc1";
    const char *host = "fc1";
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

    mosquitto_connect_callback_set(mosq, frame_connect_callback);
    mosquitto_message_callback_set(mosq, frame_message_callback);
    mosquitto_subscribe_callback_set(mosq, frame_subscribe_callback);

    //TODO:Move mosquitto to state machine to re-connect if needed
    if (mosquitto_connect(mosq, host, port, keepalive)) {
        fprintf(stderr, "Unable to connect.\n");
        return -1;
    }

    mosquitto_subscribe(mosq, NULL, "frames/#", 2);

    /**
     * Set up the channels and derived packages for subscribers
     */
    if (!(channels_pkg = channels_create_map(channel_list))) {
        bprintf(err, "Exiting framing routine because we cannot get the channel list");
        return NULL;
    }
    mosquitto_publish(mosq, NULL, "channels/fc/1",
            sizeof(channel_header_t) + channels_pkg->length * sizeof(struct channel_packed), channels_pkg, 1, true);
    bfree(err, channels_pkg);

    if (!(derived_pkg = channels_create_derived_map(m_derived))) bprintf(warning, "Failed sending derived packages");
    else {
        mosquitto_publish(mosq, NULL, "derived/fc/1",
                sizeof(derived_header_t) + derived_pkg->length * sizeof(derived_tng_t), derived_pkg, 1, true);
        bfree(err, derived_pkg);
    }

    pthread_create(&frame_thread, NULL, &framing_routine, channel_list);
    pthread_detach(frame_thread);
    return 0;
}

void framing_shutdown(void)
{
    frame_stop = 1;
}

