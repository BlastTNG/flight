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
#include "blast.h"
#include "channels_tng.h"
#include "FIFO.h"

#include "defricher.h"
#include "defricher_utils.h"
#include "defricher_writer.h"

#include "defricher_data.h"

#ifndef HOST_NAME_MAX
    #define HOST_NAME_MAX 255
#endif

static struct mosquitto *mosq;

static char client_id[HOST_NAME_MAX+1] = {0};
static char remote_host[HOST_NAME_MAX+1] = {0};
static int port = 1883;
static int keepalive = 15;

static void frame_handle_data(const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;

    if (!m_rate) {
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

    if (ready_to_read(rate->position)) {
        channels_check_size_of_frame(rate->position, m_len);
        memcpy(getFifoWrite(&fifo_data[rate->position]), m_data, m_len);
        incrementFifo(&fifo_data[rate->position]);
        defricher_queue_packet(rate->position);
    }
}

static void frame_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    char **topics;
    int count;
    static uint32_t last_crc = 0;
    static uint32_t last_derived_crc = 0;
    char *telemetry = (char *) userdata;
    bool correct_topic = false;
    
    if(message->payloadlen){
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {
            if (strcmp(telemetry, "lab") == 0) {
                correct_topic = ((count == 4) && topics[0] && strcmp(topics[0], "frames") == 0);
            } else {
                correct_topic = ((count == 3) && topics[0] && strcmp(topics[0], "frames") == 0 && strcmp(topics[1], telemetry) == 0);
            }
            if (correct_topic) {
                if (ri.channels_ready) {
                    if (!strcasecmp(topics[count-1], "200HZ")) ri.read ++;
                    frame_handle_data(topics[count-1], message->payload, message->payloadlen);
                }
            }
            if (strcmp(telemetry, "lab") == 0) {
                correct_topic = ((count == 3) && topics[0] && strcmp(topics[0], "channels") == 0);
            } else {
                correct_topic = ((count == 2) && topics[0] && strcmp(topics[0], "channels") == 0 && strcmp(topics[1], telemetry) == 0);
            }
            if (correct_topic) {
                if (((channel_header_t*)message->payload)->crc != last_crc && !ri.new_channels) {
                    defricher_info( "Received updated Channels.  Ready to initialize new DIRFILE!");
                    if (channels_read_map(message->payload, message->payloadlen, &new_channels) > 0 ) {
                        defricher_startup("Ready to init channels");
                        ri.new_channels = true;
                        last_crc = ((channel_header_t*)message->payload)->crc;
                    }
                }
            }
            if (strcmp(telemetry, "lab") == 0) {
                correct_topic = ((count == 3) && topics[0] && strcmp(topics[0], "derived") == 0);
            } else {
                correct_topic = ((count == 2) && topics[0] && strcmp(topics[0], "derived") == 0 && strcmp(topics[1], telemetry) == 0);
            }
            if (correct_topic) {
                if (((derived_header_t*)message->payload)->crc != last_derived_crc) {
                    defricher_info( "Received updated Derived Channels.");
                    if (channels_read_derived_map(message->payload, message->payloadlen, &derived_channels) > 0 ) {
                        defricher_request_updated_derived();
                        last_derived_crc = ((derived_header_t*)message->payload)->crc;
                    }
                }
            }
            if (strcmp(telemetry, "lab") == 0) {
               correct_topic = false;
            } else {
               correct_topic = ((count == 2) && topics[0] && strcmp(topics[0], "linklists") == 0); 
            }
            if (correct_topic) {
                defricher_info("Received data from %s == %s (size %d == %d)\n", rc.linklist_file, topics[1], rc.ll->blk_size, message->payloadlen);
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

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & MOSQ_LOG_ERR)
        defricher_err( "%s\n", str);
    else if (level & MOSQ_LOG_WARNING)
        defricher_warn( "%s\n", str);
}


static void *netreader_routine(void *m_arg)
{
    int ret;

    defricher_info( "Starting Framing task\n");

    while (!ri.reader_done)
    {
        ret = mosquitto_loop(mosq, 100, 1);
        switch(ret) {
            case MOSQ_ERR_SUCCESS:
                break;
            case MOSQ_ERR_NO_CONN:
                if (rc.auto_reconnect){
                    sleep(5);
                    defricher_warn("No connection to %s.  Retrying...", remote_host);
                    mosquitto_reconnect(mosq);
                } else {
                    defricher_err("Not connected to %s.  Quitting.", remote_host);
                    ri.reader_done = 1;
                    ri.writer_done = 1;
                }
                break;
            case MOSQ_ERR_CONN_LOST:
                if (rc.auto_reconnect){
                    defricher_warn("Lost connection to %s.  Trying to reconnect...", remote_host);
                    mosquitto_reconnect(mosq);
                } else {
                    defricher_err("Lost connection to %s", remote_host);
                    ri.reader_done = 1;
                    ri.writer_done = 1;
                }
                break;
            default:
                defricher_err("Received %d from mosquitto_loop, corresponding to %s", ret, mosquitto_strerror(ret));
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
pthread_t netreader_init(const char *m_host, char *m_telemetry)
{
    pthread_t netread_thread;

    gethostname(client_id, HOST_NAME_MAX);
    strncpy(remote_host, m_host, HOST_NAME_MAX);

    mosquitto_lib_init();
    mosq = mosquitto_new(client_id, true, NULL);
    if (!mosq) {
        defricher_strerr("mosquitto_new() failed");
        return 0;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);

    mosquitto_connect_callback_set(mosq, frame_connect_callback);
    mosquitto_message_callback_set(mosq, frame_message_callback);
    mosquitto_user_data_set(mosq, (void *) m_telemetry);
//    mosquitto_subscribe_callback_set(mosq, frame_subscribe_callback);

    if (mosquitto_connect(mosq, remote_host, port, keepalive)) {
        defricher_strerr("Unable to connect.\n");
        return 0;
    }
            
    char framename[32] = {0};

    if (rc.linklist_file) { // linklist mode
        int i;
        for (i = strlen(rc.linklist_file)-1; i >=0 ; i--) {
          if (rc.linklist_file[i] == '/') break;
        }
        sprintf(framename, "linklists/%s", rc.linklist_file+i+1); 
    } else { // normal full channel mode
        if (strcmp(m_telemetry, "lab") == 0) {
            sprintf(framename, "frames/#");
        } else {
            sprintf(framename, "frames/%s/#", m_telemetry);
        }
    }

		mosquitto_subscribe(mosq, NULL, framename, 2);
    mosquitto_subscribe(mosq, NULL, "channels/#", 2);
    mosquitto_subscribe(mosq, NULL, "derived/#", 2);

		printf("Subscribed to \"%s\"\n", framename);

    if (!pthread_create(&netread_thread, NULL, &netreader_routine, NULL))
            return netread_thread;
    return 0;
}

