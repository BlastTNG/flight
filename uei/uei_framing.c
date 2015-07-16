/* 
 * uei_framing.c: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
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
#include <error.h>
#include <ctype.h>

#include <native/task.h>
#include <native/event.h>
#include <native/timer.h>
#include <mosquitto.h>

#include <channels_tng.h>
#include <lookup.h>

#include "uei_framing.h"

extern int stop;

static E_SRC uei_which = 0;

static char server[2][4] = {
        "fc1",
        "fc2"
};

static struct mosquitto *mosq[2] = {
        NULL,
        NULL
};

void message_handle_frame(const char *m_fc, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;
	SRC_LOOKUP_T *src;

	if (!m_fc || !m_rate) {
		printf("Err in pointers\n");
		return;
	}
	if (!m_len) {
		printf("Zero-length string for frame\n");
		return;
	}

	for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
		if (strcmp(rate->text, m_rate) == 0) break;
	}
	if (rate->position == RATE_END) {
		printf("Did not recognize rate %s!\n", m_rate);
		return;
	}

	for (src = SRC_LOOKUP_TABLE; src->position < SRC_END; src++) {
		if (strncmp(src->text, m_fc, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
	}
	if (src->position == SRC_END) {
		printf("Did not recognize source %s\n", m_fc);
		return;
	}
	channels_store_data(src->position, rate->position, m_data, m_len);

}

void uei_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	char **topics;
	int count;

    if(message->payloadlen){
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {

        	if ( count == 3 && strcmp(topics[1], "frames") == 0) {
        		message_handle_frame(topics[0], topics[2], message->payload, message->payloadlen);
			}

        	mosquitto_sub_topic_tokens_free(&topics, count);
        }

    }
    fflush(stdout);
}

void frame_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
    if(!result){
        /* Subscribe to broker information topics on successful connect. */
        mosquitto_subscribe(mosq, NULL, "$SYS/#", 2);
    }else{
        fprintf(stderr, "Connect failed\n");
    }
}

void frame_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;

    printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        printf(", %d", granted_qos[i]);
    }
    printf("\n");
}

void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        printf("%s\n", str);
}

int uei_framing_init(int m_which)
{
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;
    uei_which = m_which;

    mosquitto_lib_init();
    for (int hostnum = 0; hostnum < 2; hostnum++) {
        mosq[hostnum] = mosquitto_new(SRC_LOOKUP_TABLE[uei_which].text, clean_session, server[hostnum]);
        if (!mosq[hostnum]) {
            error(0, errno, "mosquitto_new() failed for %s", server[hostnum]);
            continue;
        }

        mosquitto_log_callback_set(mosq[hostnum], frame_log_callback);
        mosquitto_connect_callback_set(mosq[hostnum], frame_connect_callback);
        mosquitto_message_callback_set(mosq[hostnum], uei_message_callback);
        mosquitto_subscribe_callback_set(mosq[hostnum], frame_subscribe_callback);

        if (mosquitto_connect_async(mosq[hostnum], server[hostnum], port, keepalive)) {
            error(0, errno, "Unable to connect to %s", server[hostnum]);
            continue;
        }

        mosquitto_loop_start(mosq[hostnum]);
    }

    return 0;
}


void uei_framing_routine(void *m_arg)
{
    int ret;
    int counter_100hz = 1;
    int counter_5hz=39;
    int counter_1hz=199;

    char *topic_1hz;
    char *topic_5hz;
    char *topic_100hz;
    char *topic_200hz;

    SRC_LOOKUP_T source = *(SRC_LOOKUP_T*)m_arg;
    char src_name[BLAST_LOOKUP_TABLE_TEXT_SIZE] = {'\0'};

    RT_TIMER_INFO timer_info;
    long long task_period;

    printf("Starting Framing task\n");

    for (int i = 0; i <= strlen(source.text); i++) src_name[i] = tolower(source.text[i]);

    asprintf(&topic_1hz, "%s/frames/1HZ", src_name);
    asprintf(&topic_5hz, "%s/frames/5HZ", src_name);
    asprintf(&topic_100hz, "%s/frames/100HZ", src_name);
    asprintf(&topic_200hz, "%s/frames/200HZ", src_name);

    rt_timer_inquire(&timer_info);
    if (timer_info.period == TM_ONESHOT)
    {
        // When using an aperiodic timer, task period is specified in ns
        task_period = rt_timer_ns2ticks(1000000000ll / 200);
    }
    else
    {
        // When using a periodic timer, task period is specified in number of timer periods
        task_period = (1000000000ll / 200) / timer_info.period;
    }

    ret = rt_task_set_periodic(NULL, TM_NOW, task_period);
    if (ret)
    {
        printf("error while set periodic, code %d\n", ret);
        return;
    }


    // Make sure we are in primary mode before entering the timer loop
    rt_task_set_mode(0, T_PRIMARY, NULL);
    while (!stop)
    {
        unsigned long ov;

        // Wait for next time period
        ret = rt_task_wait_period(&ov);
        if (ret && ret != -ETIMEDOUT)
        {
            printf("error while rt_task_wait_period, code %d (%s)\n", ret,
                    strerror(-ret));
            break;
        }
        if (frame_size[source.position][RATE_200HZ]) {
        	mosquitto_publish(mosq[0], NULL, topic_200hz,
        			frame_size[source.position][RATE_200HZ], channel_data[source.position][RATE_200HZ],0, false);
            mosquitto_publish(mosq[1], NULL, topic_200hz,
                    frame_size[source.position][RATE_200HZ], channel_data[source.position][RATE_200HZ],0, false);
        }

        if (!counter_100hz--) {
        	counter_100hz = 1;
            if (frame_size[source.position][RATE_100HZ]) {
            	mosquitto_publish(mosq[0], NULL, topic_100hz,
            			frame_size[source.position][RATE_100HZ], channel_data[source.position][RATE_100HZ],0, false);
                mosquitto_publish(mosq[1], NULL, topic_100hz,
                        frame_size[source.position][RATE_100HZ], channel_data[source.position][RATE_100HZ],0, false);
            }
        }
        if (!counter_5hz--) {
        	counter_5hz = 39;
        	if (frame_size[source.position][RATE_5HZ]) {
        		mosquitto_publish(mosq[0], NULL, topic_5hz,
        				frame_size[source.position][RATE_5HZ], channel_data[source.position][RATE_5HZ], 0, false);
                mosquitto_publish(mosq[1], NULL, topic_5hz,
                        frame_size[source.position][RATE_5HZ], channel_data[source.position][RATE_5HZ], 0, false);
        	}
        }
        if (!counter_1hz--) {
        	counter_1hz = 199;
        	if (frame_size[source.position][RATE_5HZ]) {
        		mosquitto_publish(mosq[0], NULL, topic_1hz,
        				frame_size[source.position][RATE_1HZ], channel_data[source.position][RATE_1HZ], 0, false);
                mosquitto_publish(mosq[1], NULL, topic_1hz,
                        frame_size[source.position][RATE_1HZ], channel_data[source.position][RATE_1HZ], 0, false);
        	}
        }
    }
}

void uei_store_analog32_data(uei_channel_map_t *m_map, uint32_t *m_data) {

    for (int i = 0; m_map[i].channel_num > -1; i++) {
        if (m_map[i].channel) SET_UINT32(m_map[i].channel, m_data[i]);
    }
}

void uei_store_analog16_data(uei_channel_map_t *m_map, uint16_t *m_data) {

    for (int i = 0; m_map[i].channel_num > -1; i++) {
        if (m_map[i].channel) SET_UINT16(m_map[i].channel, m_data[i]);
    }
}


void uei_framing_deinit(void) {

    mosquitto_disconnect(mosq[0]);
    mosquitto_loop_stop(mosq[0], false);
    mosquitto_disconnect(mosq[1]);
    mosquitto_loop_stop(mosq[1], false);

    mosquitto_destroy(mosq[0]);
    mosquitto_destroy(mosq[1]);

    mosquitto_lib_cleanup();
}
