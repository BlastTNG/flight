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
#include <command_struct.h>
#include <crc.h>
#include <derived.h>
#include <mputs.h>

static int frame_stop;
static struct mosquitto *mosq = NULL;
static struct mosquitto *mosq_other = NULL;
extern int16_t SouthIAm;
extern int16_t InCharge;

static int32_t mcp_244hz_framenum = -1;
static int32_t mcp_200hz_framenum = -1;
static int32_t mcp_100hz_framenum = -1;
static int32_t mcp_5hz_framenum = -1;
static int32_t mcp_1hz_framenum = -1;

/**
 * Returns the current MCP framenumber of the 200Hz Frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_200hz_framenum(void)
{
    return mcp_200hz_framenum;
}
/**
 * Returns the current MCP framenumber of the 100Hz frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_100hz_framenum(void)
{
    return mcp_100hz_framenum;
}
/**
 * Returns the current MCP framenumber of the 5Hz frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_5hz_framenum(void)
{
    return mcp_5hz_framenum;
}
/**
 * Returns the current MCP framenumber of the 1Hz frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_1hz_framenum(void)
{
    return mcp_1hz_framenum;
}

static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        blast_info("%s\n", str);
}

void framing_publish_1hz(void)
{
    static channel_t *mcp_1hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_1hz_framenum_addr == NULL) {
        mcp_1hz_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/1Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_1hz_framenum++;
    SET_INT32(mcp_1hz_framenum_addr, mcp_1hz_framenum);
    if (frame_size[RATE_1HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_1HZ], channel_data[RATE_1HZ], 0, false);
    }
}

void framing_publish_5hz(void)
{
    static channel_t *mcp_5hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_5hz_framenum_addr == NULL) {
        mcp_5hz_framenum_addr = channels_find_by_name("mcp_5hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/5Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_5hz_framenum++;
    SET_INT32(mcp_5hz_framenum_addr, mcp_5hz_framenum);
    if (frame_size[RATE_5HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_5HZ], channel_data[RATE_5HZ], 0, false);
    }
}

void framing_publish_100hz(void)
{
    static channel_t *mcp_100hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_100hz_framenum_addr == NULL) {
        mcp_100hz_framenum_addr = channels_find_by_name("mcp_100hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/100Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_100hz_framenum++;
    SET_INT32(mcp_100hz_framenum_addr, mcp_100hz_framenum);
    if (frame_size[RATE_100HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_100HZ], channel_data[RATE_100HZ], 0, false);
    }
}

void framing_publish_200hz(void)
{
    static channel_t *mcp_200hz_framenum_addr = NULL;
    static char frame_name[32];

    if (mcp_200hz_framenum_addr == NULL) {
        mcp_200hz_framenum_addr = channels_find_by_name("mcp_200hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/200Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_200hz_framenum++;
    SET_INT32(mcp_200hz_framenum_addr, mcp_200hz_framenum);
    if (frame_size[RATE_200HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_200HZ], channel_data[RATE_200HZ], 0, false);
    }
}

void framing_publish_244hz(void)
{
    static channel_t *mcp_244hz_framenum_addr = NULL;
    static char frame_name[32];

    if (mcp_244hz_framenum_addr == NULL) {
        mcp_244hz_framenum_addr = channels_find_by_name("mcp_244hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/244Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_244hz_framenum++;
    SET_INT32(mcp_244hz_framenum_addr, mcp_244hz_framenum);
    if (frame_size[RATE_244HZ]) {
        mosquitto_publish(mosq, NULL, frame_name,
                frame_size[RATE_244HZ], channel_data[RATE_244HZ], 0, false);
    }
}

/**
 * Publish updated CommandData
 */
void framing_publish_command_data(struct CommandDataStruct *m_commanddata)
{
    mosquitto_publish(mosq, NULL, "commanddata", sizeof(struct CommandDataStruct), m_commanddata, 1, 1);
}

static void framing_handle_data(const char *m_src, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;

    if (!m_src || !m_rate) {
        blast_err("Err in pointers");
        return;
    }
    if (!m_len) {
        blast_warn("Zero-length string for frame");
        return;
    }

    for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
        if (strncasecmp(rate->text, m_rate, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
    }
    if (rate->position == RATE_END) {
        blast_err("Did not recognize rate %s!", m_rate);
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
 * Received data from "other" flight computer including CommandData struct,
 * EtherCAT state, Flight computer temps, etc.
 * @param mosq Pointer to mosq connection to other flight computer
 * @param userdata
 * @param message
 */
static void framing_shared_data_callback(struct mosquitto *mosq, void *userdata,
                                         const struct mosquitto_message *message)
{
    char **topics;
    int count;
    static int has_warned = 0;

    if (message->payloadlen > 0) {
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {
            if (topics[0] && strcmp(topics[0], "commanddata") == 0) {
                if (!InCharge && (sizeof(CommandData) == message->payloadlen)) {
                    struct CommandDataStruct temp_command_data = {0};
                    uint32_t prev_crc;

                    memcpy(&temp_command_data, message->payload, message->payloadlen);
                    prev_crc = temp_command_data.checksum;
                    temp_command_data.checksum = 0;
                    if (prev_crc == crc32_le(0, (uint8_t*)&temp_command_data, sizeof(temp_command_data))) {
                        has_warned = 0;
                        memcpy(&CommandData, &temp_command_data, sizeof(CommandData));
                    } else if (!has_warned) {
                        has_warned = 1;
                        blast_err("Received invalid CommandData from in charge flight computer!"
                                " Please check that both FCs are running the same mcp");
                    }
                }
            }
            mosquitto_sub_topic_tokens_free(&topics, count);
        }
    }
}

static void framing_shared_connect_callback(struct mosquitto *m_mosq, void *obj, int rc)
{
    if (rc == MOSQ_ERR_SUCCESS) {
        mosquitto_subscribe(m_mosq, NULL, "commanddata", 1);
    }
}

int framing_shared_data_init(void)
{
    char id[4] = "fcX";
    char host[4] = "fcX";

    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    snprintf(id, sizeof(id), "fc%d", SouthIAm + 1);
    snprintf(host, sizeof(host), "fc%d", (SouthIAm + 2) % 2 + 1);

    mosq_other = mosquitto_new(id, clean_session, NULL);
    if (!mosq_other) {
        blast_err("mosquitto_new() failed creating other");
        return -1;
    }
    mosquitto_log_callback_set(mosq_other, frame_log_callback);
    mosquitto_message_callback_set(mosq_other, framing_shared_data_callback);
    mosquitto_connect_callback_set(mosq_other, framing_shared_connect_callback);

    if (mosquitto_connect_async(mosq_other, host, port, keepalive)) {
        fprintf(stderr, "Unable to connect.\n");
        return -1;
    }

    mosquitto_reconnect_delay_set(mosq_other, 1, 10, 1);
    mosquitto_loop_start(mosq_other);
    return 0;
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

    snprintf(id, sizeof(id), "fc%d", SouthIAm + 1);
    snprintf(host, sizeof(host), "fc%d", SouthIAm + 1);
    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if (!mosq) {
        perror("mosquitto_new() failed");
        return -1;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);
    mosquitto_message_callback_set(mosq, framing_message_callback);

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

    snprintf(topic, sizeof(topic), "channels/fc/%d", SouthIAm + 1);
    mosquitto_publish(mosq, NULL, topic,
            sizeof(channel_header_t) + channels_pkg->length * sizeof(struct channel_packed), channels_pkg, 1, true);
    bfree(err, channels_pkg);

    if (!(derived_pkg = channels_create_derived_map(m_derived))) {
        blast_warn("Failed sending derived packages");
    } else {
        snprintf(topic, sizeof(topic), "derived/fc/%d", SouthIAm + 1);
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

