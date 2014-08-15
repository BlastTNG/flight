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

#include <native/task.h>
#include <native/event.h>
#include <native/timer.h>

#include <mosquitto.h>

extern int stop;

static struct mosquitto *mosq = NULL;

void uei_message_handle_frame(const char *m_fc, const char *m_rate, const void *m_data, const int m_len)
{

}

void uei_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	char **topics;
	int count;

    if(message->payloadlen){
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {

        	if (strcmp(topics[0], "$SYS") == 0) {

        	} else if ( count > 1 && strcmp(topics[1], "frames") == 0) {
        		uei_message_handle_frame(topics[0], topics[2], message->payload, message->payloadlen);
			}

        	mosquitto_sub_topic_tokens_free(&topics, count);
        }

    }
    fflush(stdout);
}

void uei_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
    if(!result){
        /* Subscribe to broker information topics on successful connect. */
        mosquitto_subscribe(mosq, NULL, "$SYS/#", 2);
    }else{
        fprintf(stderr, "Connect failed\n");
    }
}

void uei_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;

    printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        printf(", %d", granted_qos[i]);
    }
    printf("\n");
}

void uei_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & ( MOSQ_LOG_ERR | MOSQ_LOG_WARNING ))
        printf("%s\n", str);
}

int uei_framing_init(void)
{
    char id[30];
    char *host = "fc1";
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if (!mosq) {
        fprintf(stderr, "Error: Out of memory.\n");
        return -1;
    }
    mosquitto_log_callback_set(mosq, uei_log_callback);

    mosquitto_connect_callback_set(mosq, uei_connect_callback);
    mosquitto_message_callback_set(mosq, uei_message_callback);
    mosquitto_subscribe_callback_set(mosq, uei_subscribe_callback);

    if (mosquitto_connect(mosq, host, port, keepalive)) {
        fprintf(stderr, "Unable to connect.\n");
        return -1;
    }

    mosquitto_subscribe(mosq, NULL, "fc1/frames/#", 2);
    mosquitto_subscribe(mosq, NULL, "fc2/frames/#", 2);

    return 0;
}


void uei_framing_routine(void *m_arg)
{
    int ret;

    RT_TIMER_INFO timer_info;
    long long task_period;

    printf("Starting Framing task\n");

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

        if ((ret = mosquitto_loop(mosq, 0, 100)) != MOSQ_ERR_SUCCESS) {
            printf("Received %d from mosquitto_loop\n", ret);
        }
    }
    //switch to secondary mode
    ret = rt_task_set_mode(T_PRIMARY, 0, NULL);

    if (ret)
    {
        printf("error while rt_task_set_mode, code %d\n", ret);
    }
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

}
