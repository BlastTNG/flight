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

#include <blast.h>
#include <blast_time.h>
#include <channels_tng.h>
#include <crc.h>
#include <derived.h>
#include <mputs.h>
#include <linklist_compress.h>

static int frame_stop;
static struct mosquitto *mosq = NULL;


void initialize_circular_superframes(superframes_list_t *superframes)
{
    int i;
    superframes.i_in = 0;
    superframes.i_out = 0;
    for (i = 0; i < NUM_FRAMES; i++) {
        superframes.framelist[i] = calloc(1, superframe_size);
        memset(superframes.framelist[i], 0, superframe_size);
    }
}

void push_superframe(const void *m_frame, superframes_list_t *superframes)
{
    int i_in;
    i_in = (superframes.i_in + 1) & (NUM_FRAMES-1);
    superframes.framesize[i_in] = superframe_size;
    memcpy(superframes.framelist[i_in], m_frame, superframe_size);
    superframes.i_in = i_in;
}

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        blast_info("%s\n", str);
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


static void framing_handle_data(const char *m_src, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;

    if (!m_src || !m_rate) {
        printf("Err in pointers\n");
        return;
    }
    if (!m_len) {
        printf("Zero-length string for frame\n");
        return;
    }

    for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
        if (strncasecmp(rate->text, m_rate, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
    }
    if (rate->position == RATE_END) {
        printf("Did not recognize rate %s!\n", m_rate);
        return;
    }
}

static void framing_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    char **topics;
    int count;

    if (message->payloadlen) {
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {
            if (count == 4 && topics[0] && strcmp(topics[0], "frames") == 0) {
                framing_handle_data(topics[1], topics[3], message->payload, message->payloadlen);
            }
            mosquitto_sub_topic_tokens_free(&topics, count);
        }
    }
}


/**
 * Initializes the mosquitto library and associated framing routines.
 * @return
 */
int framing_init(channel_t *channel_list, derived_tng_t *m_derived)
{
    channel_header_t *channels_pkg = NULL;
    derived_header_t *derived_pkg = NULL;

    char id[15] = "groundhog_blastgs01";
    char host[9] = "blastgs01";
    char topic[64];

    int ret = 0;
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    struct telemetries m_telemetries = {3, {"biphase", "pilot", "tdrss"}};

    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if (!mosq) {
        perror("mosquitto_new() failed");
        return -1;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);
    mosquitto_message_callback_set(mosq, framing_message_callback);

    ret = mosquitto_connect_async(mosq, host, port, keepalive); 
    if (ret == MOSQ_ERR_SUCCESS) {
   	printf("Succesfully connected decomd to mosquitto server on %s\n", host);
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


    for (int i = 0; i < m_telemetries.number; i++ ) {
        snprintf(topic, sizeof(topic), "channels/%s", m_telemetries.types[i]);
        mosquitto_publish(mosq, NULL, topic,
                sizeof(channel_header_t) + channels_pkg->length * sizeof(struct channel_packed), channels_pkg, 1, true);
        bfree(err, channels_pkg);

        if (!(derived_pkg = channels_create_derived_map(m_derived))) {
            blast_info("Failed sending derived packages for telemetry %s\n", m_telemetries.types[i]);
        } else {
            snprintf(topic, sizeof(topic), "derived/%s", m_telemetries.types[i]);
            mosquitto_publish(mosq, NULL, topic,
                    sizeof(derived_header_t) + derived_pkg->length * sizeof(derived_tng_t), derived_pkg, 1, true);
            bfree(err, derived_pkg);
        }
    }

    return 0;
}

void framing_shutdown(void)
{
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, true);
    frame_stop = 1;
}

