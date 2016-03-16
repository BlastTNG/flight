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




#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

int xsc_server_index[2] = {0, 0};

static const char addresses[4][16] = {"192.168.40.1", "192.168.40.2", "192.168.40.3", "192.168.40.4"};
static const uint16_t port = 30001;
static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;
extern int16_t InCharge;

typedef struct {
  int port;
  bool connected;
  bool have_warned_version;
  uint32_t backoff_sec;
  struct timeval timeout;
  ph_job_t connect_job;
  ph_sock_t *sock;
} remote_serial_t;


/**
 * Process an incoming XSC packet or event.  If we have an error, we'll disable
 * the socket and schedule a reconnection attempt.  Otherwise, read and store the
 * camera data.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void xsc_process_packet(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    ph_buf_t *buf;
    XSCServerData *data;
    remote_serial_t *state = (remote_serial_t*) m_data;


    /**
     * If we have an error, or do not receive data from the star camera in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & (PH_IOMASK_ERR|PH_IOMASK_TIME)) {
      blast_err("disconnecting XSC%d due to connection issue", state->which);
      ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
      ph_sock_enable(m_sock, 0);
      state->connected = false;
      ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 100);
      return;
    }

    buf = ph_sock_read_bytes_exact(m_sock, sizeof(XSCServerData));
    if (!buf) return; /// We do not have enough data


    ph_buf_delref(buf);
}

/**
 * Write data into the XSC sock buffer, if we are connected to the camera.
 *
 * @param which 0/1 for XSC0/XSC1
 */
void xsc_write_data(remote_serial_t *m_serial)
{
    XSCClientData xsc_client_data;
    int pointing_read_index = GETREADINDEX(point_index);

    if (!m_serial->connected) return;


    ph_stm_write(m_serial->sock->stream, &xsc_client_data, sizeof(xsc_client_data), NULL);
}

/**
 * Handle a connection callback from @connect_xsc.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our XSC State variable
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

    blast_info("Connected to XSC%d at %s", state->which, addresses[state->which]);

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    state->connected = true;
    state->backoff_sec = min_backoff_sec;
    m_sock->callback = xsc_process_packet;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);
}

/**
 * Handles the connection job.  Formatted this way to allow us to schedule
 * a future timeout in the PH_JOB infrastructure
 *
 * @param m_job Unused
 * @param m_why Unused
 * @param m_data Pointer to the XSC State variable
 */
static void connect_remote_serial(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    remote_serial_t *state = (remote_serial_t*)m_data;

    blast_info("Connecting to %s", addresses[state->which]);
    ph_sock_resolve_and_connect(addresses[state->which], port,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM,
        connected, m_data);
}

/**
 * Initialize the star camera I/O routine.  The state variable tracks each
 * camera connection and is passed to the connect job.
 *
 * @param m_which 0,1,2,3 for ROACH0, ROACH1, ROACH2 or ROACH3
 */
remote_serial_t *remote_serial_init(const char *m_address, int m_port)
{
    remote_serial_t *new_port = calloc(1, sizeof(remote_serial_t));

    blast_dbg("Remote Serial Init for %s", m_address);

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
}
