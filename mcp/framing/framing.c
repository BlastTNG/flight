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
static struct mosquitto *mosq = NULL;
extern short int SouthIAm;

static void frame_handle_data(const char *m_fc, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;
    SRC_LOOKUP_T *src;

    if (!m_fc || !m_rate) {
        blast_err("Err in pointers\n");
        return;
    }
    if (!m_len) {
        blast_warn("Zero-length string for frame\n");
        return;
    }

    for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
        if (strcmp(rate->text, m_rate) == 0) break;
    }
    if (rate->position == RATE_END) {
        blast_warn("Did not recognize rate %s!\n", m_rate);
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

    blast_info("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        blast_info("\t %d", granted_qos[i]);
    }
}

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        blast_info("%s\n", str);
}

void uei_publish_1hz(void)
{
    static uint32_t uei_of_1hz_framenum = 0;
    static channel_t *uei_of_1hz_framenum_addr = NULL;
    static char frame_name[32];
    if (uei_of_1hz_framenum_addr == NULL) {
        uei_of_1hz_framenum_addr = channels_find_by_name("uei_of_1hz_framecount");
        snprintf(frame_name, 32, "frames/of_uei/dummy/1Hz");
    }

    if (frame_stop) return;

    uei_of_1hz_framenum++;
    SET_UINT32(uei_of_1hz_framenum_addr, uei_of_1hz_framenum);
    if (frame_size[SRC_OF_UEI][RATE_1HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[SRC_OF_UEI][RATE_1HZ], channel_data[SRC_OF_UEI][RATE_1HZ], 0, false);
    }

}

void framing_publish_1hz(void)
{
    static uint32_t mcp_1hz_framenum = 0;
    static channel_t *mcp_1hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_1hz_framenum_addr == NULL) {
        mcp_1hz_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
        snprintf(frame_name, 32, "frames/fc/%d/1Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_1hz_framenum++;
    SET_UINT32(mcp_1hz_framenum_addr, mcp_1hz_framenum);
    if (frame_size[SRC_FC][RATE_1HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[SRC_FC][RATE_1HZ], channel_data[SRC_FC][RATE_1HZ], 0, false);
    }

}

void framing_publish_5hz(void)
{
    static uint32_t mcp_5hz_framenum = 0;
    static channel_t *mcp_5hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_5hz_framenum_addr == NULL) {
        mcp_5hz_framenum_addr = channels_find_by_name("mcp_5hz_framecount");
        snprintf(frame_name, 32, "frames/fc/%d/5Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_5hz_framenum++;
    SET_UINT32(mcp_5hz_framenum_addr, mcp_5hz_framenum);
    if (frame_size[SRC_FC][RATE_5HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[SRC_FC][RATE_5HZ], channel_data[SRC_FC][RATE_5HZ], 0, false);
    }
}

void framing_publish_100hz(void)
{
    static uint32_t mcp_100hz_framenum = 0;
    static channel_t *mcp_100hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_100hz_framenum_addr == NULL) {
        mcp_100hz_framenum_addr = channels_find_by_name("mcp_100hz_framecount");
        snprintf(frame_name, 32, "frames/fc/%d/100Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_100hz_framenum++;
    SET_UINT32(mcp_100hz_framenum_addr, mcp_100hz_framenum);
    if (frame_size[SRC_FC][RATE_100HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[SRC_FC][RATE_100HZ], channel_data[SRC_FC][RATE_100HZ], 0, false);
    }
}

void framing_publish_200hz(void)
{
    static uint32_t mcp_200hz_framenum = 0;
    static channel_t *mcp_200hz_framenum_addr = NULL;
    static char frame_name[32];

    if (mcp_200hz_framenum_addr == NULL) {
        mcp_200hz_framenum_addr = channels_find_by_name("mcp_200hz_framecount");
        snprintf(frame_name, 32, "frames/fc/%d/200Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_200hz_framenum++;
    SET_UINT32(mcp_200hz_framenum_addr, mcp_200hz_framenum);
    if (frame_size[SRC_FC][RATE_200HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[SRC_FC][RATE_200HZ], channel_data[SRC_FC][RATE_200HZ], 0, false);
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

    char id[4] = "fcX";
    char host[4] = "fcX";
    char topic[64];

    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    snprintf(id, 4, "fc%d", SouthIAm + 1);
    snprintf(host, 4, "fc%d", SouthIAm + 1);
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

    if (mosquitto_connect_async(mosq, host, port, keepalive)) {
        fprintf(stderr, "Unable to connect.\n");
        return -1;
    }

    /**
     * Set up the channels and derived packages for subscribers
     */
    if (!(channels_pkg = channels_create_map(channel_list))) {
        blast_err("Exiting framing routine because we cannot get the channel list");
        return -1;
    }

    mosquitto_reconnect_delay_set(mosq, 1, 10, 1);
    mosquitto_loop_start(mosq);

    mosquitto_subscribe(mosq, NULL, "frames/#", 2);

    snprintf(topic, 64, "channels/fc/%d", SouthIAm + 1);
    mosquitto_publish(mosq, NULL, topic,
            sizeof(channel_header_t) + channels_pkg->length * sizeof(struct channel_packed), channels_pkg, 1, true);
    bfree(err, channels_pkg);

    if (!(derived_pkg = channels_create_derived_map(m_derived))) blast_warn("Failed sending derived packages");
    else {
        snprintf(topic, 64, "derived/fc/%d", SouthIAm + 1);
        mosquitto_publish(mosq, NULL, topic,
                sizeof(derived_header_t) + derived_pkg->length * sizeof(derived_tng_t), derived_pkg, 1, true);
        bfree(err, derived_pkg);
    }

    return 0;
}

void framing_shutdown(void)
{
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, true);
    frame_stop = 1;
}

