/*
 * remote_serial.c
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
 *  Created on: Mar 15, 2016
 *      Author: seth
 */

#include <remote_serial.h>

#include <unistd.h>

#include <blast.h>
#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

static const char addresses[4][16] = {"192.168.40.61", "192.168.40.62", "192.168.40.63", "192.168.40.64"};
static const uint16_t port = 10001;
static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;
extern int16_t InCharge;

typedef struct remote_serial {
    int which;
    int port;
    bool connected;
    bool have_warned_version;
    uint32_t backoff_sec;
    struct timeval timeout;
    ph_job_t connect_job;
    ph_sock_t *sock;
    ph_bufq_t *input_buffer;
} remote_serial_t;

/**
 * Process an incoming packet or event.  If we have an error, we'll disable
 * the socket and schedule a reconnection attempt.  Otherwise, read and store the serial string.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void remote_serial_process_packet(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    ph_buf_t *buf;
    remote_serial_t *state = (remote_serial_t*) m_data;
    size_t buflen;

    /**
     * If we have an error, or do not receive data from the star camera in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & PH_IOMASK_ERR) {
      blast_err("disconnecting RemoteSerial %d:%d due to connection issue", state->which, state->port);
      ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
      ph_sock_enable(m_sock, 0);
      state->connected = false;
      ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 100);
      return;
    }

    /**
     * If we timeout, send a newline to keep the socket alive
     */
    if (m_why & PH_IOMASK_TIME) ph_stm_printf(m_sock->stream, "\n");

    buflen = ph_bufq_len(m_sock->rbuf);
    if (buflen) {
        buf = ph_sock_read_bytes_exact(m_sock, buflen);
        ph_bufq_append(state->input_buffer, ph_buf_mem(buf), ph_buf_len(buf), NULL);
        ph_buf_delref(buf);
    }
}

/**
 * Write data into the remote serial sock buffer and flush to network
 *
 */
int remote_serial_write_data(remote_serial_t *m_serial, uint8_t *m_data, size_t m_len)
{
    uint64_t written;
    if (!InCharge) return -2;
    if (!m_serial->connected) {
        m_serial->timeout.tv_sec = 5;
        ph_job_dispatch_now(&m_serial->connect_job);
    }

    if (!m_serial->connected) {
        blast_err("Could not connect to %s:%d", addresses[m_serial->which], m_serial->port);
        return -1;
    }

    ph_stm_write(m_serial->sock->stream, m_data, m_len, &written);
    ph_stm_flush(m_serial->sock->stream);
    return (int)written;
}

/**
 * Read data from the remote serial input buffer
 *
 */
int remote_serial_read_data(remote_serial_t *m_serial, uint8_t *m_buffer, size_t m_size)
{
    ph_buf_t *buf;
    int retval = -1;
    if (!InCharge) return -2;
    if (!m_serial->connected) return -1;

    while (m_serial->connected) {
        buf = ph_bufq_consume_bytes(m_serial->input_buffer, m_size);
        if (buf) {
            memcpy(m_buffer, ph_buf_mem(buf), m_size);
            ph_buf_delref(buf);
            retval = m_size;
            break;
        }
        usleep(1000);
    }
    return retval;
}

int remote_serial_flush(remote_serial_t *m_serial)
{
    ph_buf_t *buf;

    if (!m_serial->connected) return -1;

    buf = ph_bufq_consume_bytes(m_serial->input_buffer, ph_bufq_len(m_serial->input_buffer));
    ph_buf_delref(buf);

    buf = ph_bufq_consume_bytes(m_serial->sock->rbuf, ph_bufq_len(m_serial->input_buffer));
    ph_buf_delref(buf);

    return 0;
}
/**
 * Handle a connection callback.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our Remote Serial State variable
 */
static void connected(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    remote_serial_t *state = (remote_serial_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", addresses[state->which], port, gai_strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;

        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                      addresses[state->which], port, m_errcode, strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;
    }

    blast_info("Connected to ROACH%d at %s", state->which, addresses[state->which]);

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    state->connected = true;
    state->backoff_sec = min_backoff_sec;
    state->timeout.tv_sec = 1;
    state->timeout.tv_usec = 0;
    m_sock->callback = remote_serial_process_packet;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);
}

/**
 * Handles the connection job.  Formatted this way to allow us to schedule
 * a future timeout in the PH_JOB infrastructure
 *
 * @param m_job Unused
 * @param m_why Unused
 * @param m_data Pointer to the Remote Serial State variable
 */
static void connect_remote_serial(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    remote_serial_t *state = (remote_serial_t*)m_data;

    blast_info("Connecting to %s", addresses[state->which]);
    ph_sock_resolve_and_connect(addresses[state->which], port, 0,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM,
        connected, m_data);
}

/**
 * Initialize the remote serial I/O routine.  The state variable tracks each
 * camera connection and is passed to the connect job.
 *
 * @param m_which 0,1,2,3 for ROACH0, ROACH1, ROACH2 or ROACH3
 */
remote_serial_t *remote_serial_init(int m_which, int m_port)
{
    remote_serial_t *new_port = calloc(1, sizeof(remote_serial_t));

    blast_dbg("Remote Serial Init for %s", addresses[m_which]);

    new_port->connected = false;
    new_port->have_warned_version = false;
    new_port->which = m_which;
    new_port->backoff_sec = min_backoff_sec;
    new_port->timeout.tv_sec = 5;
    new_port->timeout.tv_usec = 0;
    ph_job_init(&(new_port->connect_job));
    new_port->connect_job.callback = connect_remote_serial;
    new_port->connect_job.data = new_port;

    ph_job_dispatch_now(&(new_port->connect_job));

    return new_port;
}
