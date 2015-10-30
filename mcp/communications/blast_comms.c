/**
 * @file blast_comms.c
 *
 * @date 2011-02-08
 * @author Seth Hillbrand
 *
 * @brief Implements asynchronous serial line communications
 *
 * This software is copyright (C) 2011-2015 Seth Hillbrand
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * @details
 * This file provides generalized asynchronous communications
 * structure for all uplink/downlink serial ports.
 *
 * Rather than maintaining multiple threads for each serial line, we
 * keep all communications lines in the same thread, decreasing the
 * overhead from context swapping between many threads that are all
 * performing the same task.
 *
 * A client will open and set the options on their comm port, initializing
 * an comms_serial_t structure.  Once completed, the client sets the appropriate
 * callback functions for dealing with incoming data, write signals and errors.
 * This structure is then registered with the asynchronous handler which takes
 * care of the rest.
 */
#include <pthread.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <time.h>

#include <blast.h>
#include <blast_time.h>
#include <crc.h>
#include <comms_netasync.h>
#include <comms_serial.h>
#include <mputs.h>

#include <blast_comms.h>
#include "blast_comms_internal.h"


static void blast_comms_cleanup(void *m_arg)
{
	blast_comms_list_t *ports = NULL;
	blast_comms_list_t *next_ptr = NULL;

	if (comms_ctx)
	{
		comms_net_async_ctx_free(comms_ctx);
		comms_ctx = NULL;
	}

	for (ports = (blast_comms_list_t*)m_arg; ports; ports=next_ptr)
	{
		next_ptr = ports->next;
		comms_serial_free(ports->tty);
		bfree(err, ports);
		ports = NULL;
	}

}

bool initialize_blast_comms(void)
{
	blast_startup("Initializing BLAST Communication system");
	if (comms_thread != (pthread_t)-1) pthread_cancel(comms_thread);

	while (comms_ports) pthread_yield();

	if (!(comms_ctx = comms_net_async_ctx_new(4)))
	{
		blast_tfatal("Could not get async context for comm port monitoring!");
		return NULL;
	}

	if ((pthread_create(&comms_thread, NULL, blast_comms_monitor, NULL) == 0) &&
			(pthread_detach(comms_thread) == 0))
		return true;

	blast_fatal("Could not initialize comms thread");
	return false;
}

static void *blast_comms_monitor(void *m_arg __attribute__((unused)))
{
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 1000000}; /// 1000HZ interval

    nameThread("Comms");
    pthread_cleanup_push(blast_comms_cleanup, comms_ports);
    clock_gettime(CLOCK_REALTIME, &ts);

    do {
        int ret;
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (ret && ret != -EINTR) {
            blast_err("error while sleeping, code %d (%s)\n", ret, strerror(-ret));
            break;
        }
        if (comms_net_async_poll(comms_ctx, 0) < 0) {
            blast_err("Received error while polling COMM ports context");
        }

    } while (1);

    pthread_cleanup_pop(1);
    return NULL;
}

/**
 * Adds an allocated and connected socket to the central communication
 * monitor thread
 * @param m_sock Pointer to the socket
 * @return true on success, false on failure
 */
bool blast_comms_add_socket(comms_socket_t *m_sock)
{
	if (comms_net_async_ctx_add_sock(comms_ctx, m_sock) != NETSOCK_OK)
	{
		blast_err("Could not add %s to async comms monitor", m_sock->host);
		return false;
	}

	blast_dbg("Added '%s' to async comms monitor", m_sock->host);
	return true;
}

/**
 * Adds an allocated an connected character device (serial port, FIFO, etc...) to the central communication
 * monitor thread
 * @param m_tty Pointer to the character device structure
 * @return true on success, false on failure
 */
bool blast_comms_add_port(comms_serial_t *m_tty)
{
	blast_comms_list_t *port = NULL;
	blast_comms_list_t *iter = NULL;

	if (!comms_ctx)
	{
		blast_err("BLAST Communication thread not initialized");
		return false;
	}
	if (!m_tty)
	{
		blast_err("Passed NULL port to add");
		return false;
	}
	if (m_tty->sock->fd == INVALID_SOCK)
	{
		blast_err("Passed closed socket to monitor");
		return false;
	}

	if (!(port = (blast_comms_list_t*) balloc(err, sizeof(blast_comms_list_t)))) return false;

	port->tty = m_tty;
	port->next = NULL;

	if (!comms_ports)
	{
		blast_startup("Initializing new communication ports list");
		comms_ports = port;
	}
	else
	{
		for(iter = comms_ports; iter->next; iter=iter->next)
			;/**Do Nothing */
		iter->next = port;
	}
	if (comms_net_async_ctx_add_sock(comms_ctx, m_tty->sock) == NETSOCK_OK)
	{
		blast_dbg("Added '%s' to async comms monitor", port->tty->sock->host);
		return true;
	}

	blast_err("Could not add '%s' to async comms monitor", port->tty->sock->host);
	bfree(err, port);
	if (comms_ports == port) comms_ports = NULL;
	else if (iter && iter->next == port) iter->next = NULL;

	port=NULL;
	return false;
}

static void blast_comms_monitor_cleanup(void *m_arg)
{
	comms_socket_t *socket = (comms_socket_t*)m_arg;

	network_monitor_thread = (pthread_t) -1;
	comms_sock_free(socket);
}

/**
 * Provides the asynchronous listener for commanding connections on a TCP port
 * @return true on success, false on failure
 */
bool blast_comms_init_cmd_server(void)
{
	if (network_monitor_thread != (pthread_t)-1)
	{
		pthread_cancel(network_monitor_thread);
	}

	while(network_monitor_thread != (pthread_t)-1) usleep(10);

	pthread_create(&network_monitor_thread, NULL, blast_comms_network_monitor, NULL);
	pthread_detach(network_monitor_thread);
	return true;

}

/**
 * This loop listens on BLAST_CMD_SERVER_PORT for a new connection from a client.  When
 * a new connection is received, it sets up the socket handlers and adds them to the
 * general communication context that is polled for activity
 * @param m_arg
 */
static void *blast_comms_network_monitor(void *m_arg __attribute__((unused)))
{
	comms_socket_t *cmd_sock = NULL;
	comms_socket_t *new_cmd = NULL;
	int fd = 0;

	pthread_cleanup_push(blast_comms_monitor_cleanup, cmd_sock);
	if (!(cmd_sock = comms_sock_new()))
	{
		blast_startup("Error allocating space for a new socket!");
		return NULL;
	}

	if (comms_sock_listen(cmd_sock, NULL, BLAST_CMD_SERVER_PORT) != NETSOCK_OK)
	{
		blast_startup("Could not begin listening on command socket");
		comms_sock_free(cmd_sock);
		return NULL;
	}

	blast_startup("Initialized Network Command Monitor");
	while(fd >= 0)
	{
		fd = comms_sock_accept(cmd_sock);
		switch(fd)
		{
			case NETSOCK_ERR:
				break;
			case NETSOCK_AGAIN:
				usleep(100);
				fd = 0;
				break;
			case NETSOCK_OK:
				break;
			default:
				new_cmd = comms_sock_new();
				new_cmd->host = bstrdup(err, cmd_sock->host);

				new_cmd->callbacks.priv = new_cmd;
				new_cmd->callbacks.connected = blast_comms_net_new_connection;
				new_cmd->callbacks.data = blast_comms_net_process_data;
				new_cmd->callbacks.finished = blast_comms_net_cleanup;
				new_cmd->callbacks.error = blast_comms_net_error;

				new_cmd->fd = fd;
				new_cmd->state = NETSOCK_STATE_CONNECTING;
				if (comms_net_async_ctx_add_sock(comms_ctx, new_cmd) == NETSOCK_OK)
				{
					blast_dbg("Added '%s' to async comms monitor", new_cmd->host);
				}
				comms_net_async_set_events(comms_sock_get_poll_handle(new_cmd), POLLOUT);
				fd = 0;
				break;
		}
	}

	pthread_cleanup_pop(1);

	blast_err("Exiting BLAST Commanding network monitor");
	return NULL;
}

/**
 * Handles the connection to a remote command client.  Currently only provides debug output for the user
 * @param m_status Corresponds to SOCK_OK if success or SOCK_ERR otherwise
 * @param m_errorcode The error code given in the case of SOCK_ERR
 * @param m_userdata Pointer to the dfmux_boardlist_t structure
 */
static void blast_comms_net_new_connection(int m_status, int m_errorcode, void *m_userdata)
{
	comms_socket_t *socket = (comms_socket_t*)m_userdata;

	if (m_status == NETSOCK_OK)
	{
		blast_info("New connection from %s OK", socket->host);
	}
	else
	{
		blast_err("Could not connect to client %s: %s", socket->host, strerror(m_errorcode));
	}

}

/**
 * Handles the network command callback.
 * @param m_data Pointer to the data queue
 * @param m_len length in bytes of the data queue
 * @param m_userdata unused
 * @return Number of bytes processed (that should be removed from the queue)
 */
static int blast_comms_net_process_data(const void *m_data, size_t m_len, void *m_userdata )
{
    return 0;
}

/**
 * Handles the network command hangup.
 * @param m_data Pointer to the data queue
 * @param m_len length in bytes of the data queue
 * @param m_userdata socket pointer
 * @return Number of bytes processed (that should be removed from the queue)
 */
static int blast_comms_net_cleanup(const void *m_data, size_t m_len, void *m_userdata )
{
	comms_socket_t *socket = (comms_socket_t*)m_userdata;
	size_t consumed = 0;

	if (m_len)
		consumed = blast_comms_net_process_data(m_data, m_len, m_userdata);

	if (consumed < m_len) blast_err("Did not receive full packet from %s", socket->host);

	comms_sock_free(socket);
	socket = NULL;

	return consumed;
}

/**
 * Currently unused function to process errors received on the Network socket
 * @param m_code
 * @param m_userdata
 */
static void blast_comms_net_error(int m_code, void *m_userdata)
{
    comms_socket_t *socket = (comms_socket_t*)m_userdata;
	blast_err("Got error %d on %s", m_code, socket->host);
	return;
}


