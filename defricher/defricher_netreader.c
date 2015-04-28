/* 
 * defricher_netreader.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of defricher, created for the BLASTPol Project.
 *
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 6, 2015 by Seth Hillbrand
 */

#include <limits.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include <mosquitto.h>

#include <lookup.h>
#include <blast.h>
#include <channels_tng.h>

#include "defricher.h"
#include "defricher_utils.h"
#include "defricher_writer.h"

extern int frame_stop;

static struct mosquitto *mosq;
pthread_t netread_thread;
extern channel_t *channels;

static char client_id[HOST_NAME_MAX+1] = {0};
static char remote_host[HOST_NAME_MAX+1] = {0};
static int port = 1883;
static int keepalive = 60;

static void frame_handle_data(const char *m_src, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;
    SRC_LOOKUP_T *src;

    if (!m_src || !m_rate) {
        defricher_err("Err in pointers");
        return;
    }
    if (!m_len) {
        defricher_warn("Zero-length string for frame");
        return;
    }

    for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
        if (strncasecmp(rate->text, m_rate, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
    }
    if (rate->position == RATE_END) {
        defricher_err("Did not recognize rate %s!\n", m_rate);
        return;
    }

    //TODO:Think about mapping FC1/FC2
    for (src = SRC_LOOKUP_TABLE; src->position < SRC_END; src++) {
        if (strncasecmp(src->text, m_src, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
    }
    if (src->position == SRC_END) {
        defricher_err("Did not recognize source %s\n", m_src);
        return;
    }

    channels_store_data(src->position, rate->position, m_data, m_len);
    defricher_write_packet(channels, src->position, rate->position);
}

static void frame_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    char **topics;
    int count;
    static uint32_t last_crc = 0;

    if(message->payloadlen){
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {

            if ( count == 4 && topics[0] && strcmp(topics[0], "frames") == 0) {
                frame_handle_data(topics[1], topics[3], message->payload, message->payloadlen);
            }
            if ( count == 3 && topics[0] && strcmp(topics[0], "channels") == 0) {
                if (((channel_header_t*)message->payload)->crc != last_crc) {
                    defricher_info( "Received updated Channels.  Ready to initialize new DIRFILE!");
                    channels_read_map(message->payload, message->payloadlen, &channels);
                    channels_initialize(channels);
                    last_crc = ((channel_header_t*)message->payload)->crc;
                    defricher_request_new_dirfile();
                }
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
        defricher_strerr( "Connect failed");
    }
}

static void frame_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;

    defricher_info( "Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        defricher_info( "\t %d", granted_qos[i]);
    }
}

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        defricher_info( "%s\n", str);
}


static void *netreader_routine(void *m_arg)
{
    int ret;

    defricher_info( "Starting Framing task\n");

    while (!frame_stop)
    {
        ret = mosquitto_loop(mosq, 100, 1);
        switch(ret) {
            case MOSQ_ERR_SUCCESS:
                break;
            case MOSQ_ERR_NO_CONN:
                if (rc.auto_reconnect){
                    sleep(5);
                    mosquitto_reconnect(mosq);
                } else {
                    defricher_err("Not connected to %s.  Quitting.", remote_host);
                    frame_stop = 1;
                    ri.writer_done = 1;
                }
                break;
            case MOSQ_ERR_CONN_LOST:
                if (rc.auto_reconnect){
                    mosquitto_reconnect(mosq);
                } else {
                    defricher_err("Lost connection to %s", remote_host);
                    frame_stop = 1;
                    ri.writer_done = 1;
                }
                break;
            default:
                defricher_err("Received %d from mosquitto_loop", ret);
                sleep(1);
                break;
        }

        fflush(NULL);
    }

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return NULL;
}

/**
 * Initializes the mosquitto library and associated framing routines.
 * @return
 */
int netreader_init(const char *m_host)
{
    bool clean_session = false;

    gethostname(client_id, HOST_NAME_MAX);

    mosquitto_lib_init();
    mosq = mosquitto_new(client_id, clean_session, NULL);
    if (!mosq) {
        defricher_strerr("mosquitto_new() failed");
        return -1;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);

    mosquitto_connect_callback_set(mosq, frame_connect_callback);
    mosquitto_message_callback_set(mosq, frame_message_callback);
    mosquitto_subscribe_callback_set(mosq, frame_subscribe_callback);

    if (mosquitto_connect(mosq, m_host, port, keepalive)) {
        defricher_strerr("Unable to connect.\n");
        return -1;
    }

    mosquitto_subscribe(mosq, NULL, "frames/#", 2);
    mosquitto_subscribe(mosq, NULL, "channels/#", 2);


    pthread_create(&netread_thread, NULL, &netreader_routine, NULL);
    return 0;
}

