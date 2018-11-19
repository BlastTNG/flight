/**
 * @file xsc_network.c
 *
 * @date Sep 23, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "xsc_network.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "conversions.h"
#include "blast.h"
#include "framing.h"
#include "mcp.h"
#include "angles.h"
#include "command_struct.h"
#include "pointing_struct.h"

#define XSC_PROTOCOL_VERSION 3

XSCServerData xsc_mserver_data[2][3];

int xsc_server_index[2] = {0, 0};

static const char addresses[2][16] = {"192.168.1.7", "192.168.1.8"};
static const uint16_t port = 2017;
static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;
extern int16_t InCharge;

typedef struct {
  int which;
  bool connected;
  bool have_warned_version;
  uint8_t have_warned_connect;
  uint32_t backoff_sec;
  struct timeval timeout;
  ph_job_t connect_job;
  ph_sock_t *sock;
} xsc_state_t;

static xsc_state_t state[2];

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
    xsc_state_t *state = (xsc_state_t*) m_data;
    int array_index = xsc_server_index[state->which];

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

    data = (XSCServerData*)ph_buf_mem(buf);

    if (data->xsc_protocol_version == XSC_PROTOCOL_VERSION) {
        memcpy(&xsc_mserver_data[state->which][array_index], data, sizeof(XSCServerData));
        xsc_server_index[state->which] = INC_INDEX(xsc_server_index[state->which]);
        xsc_pointing_state[state->which].stars_response_counter++;
        state->have_warned_version = false;
    } else {
        if (!state->have_warned_version) {
            blast_info("received packet with incorrect xsc_protocol_version: %u\n",
                    data->xsc_protocol_version);
            state->have_warned_version = true;
        }
    }

    ph_buf_delref(buf);
}

/**
 * Write data into the XSC sock buffer, if we are connected to the camera.
 *
 * @param which 0/1 for XSC0/XSC1
 */
void xsc_write_data(int which)
{
    XSCClientData xsc_client_data;
    int pointing_read_index = GETREADINDEX(point_index);

    if (!state[which].connected) return;

    memcpy(&xsc_client_data, &CommandData.XSC[which].net, sizeof(XSCClientData));

    xsc_client_data.horizontal.valid = true;
    xsc_client_data.horizontal.lat = from_degrees(PointingData[pointing_read_index].lat);
    xsc_client_data.horizontal.lst = from_seconds(PointingData[pointing_read_index].lst);
    xsc_client_data.solver.filters.hor_location_limit_az =
            from_degrees(PointingData[pointing_read_index].estimated_xsc_az_deg[which]);
    xsc_client_data.solver.filters.hor_location_limit_el =
            from_degrees(PointingData[pointing_read_index].estimated_xsc_el_deg[which]);
    xsc_client_data.solver.filters.eq_location_limit_ra =
            from_hours(PointingData[pointing_read_index].estimated_xsc_ra_hours[which]);
    xsc_client_data.solver.filters.eq_location_limit_dec =
            from_degrees(PointingData[pointing_read_index].estimated_xsc_dec_deg[which]);
    // TODO(seth): Adjust fcp<->stars in charge flag
    xsc_client_data.in_charge = 1;

    /**
     * Pause here to allow STARS to get the new image from the camera.  MCP will increment
     * its counter immediately after lowering the trigger.  We give STARS the
     * exposure_time (cs) + a commandable delay) to allow the camera to retrieve and stamp the new image.
     */

    // int post_trigger_counter_mcp_share_delay_cs =
    //          CommandData.XSC[which].trigger.post_trigger_counter_mcp_share_delay_cs;
    // if (get_100hz_framenum() > (xsc_pointing_state[which].exposure_time_cs +
    //         xsc_pointing_state[which].last_trigger_time + post_trigger_counter_mcp_share_delay_cs)) {
        xsc_client_data.counter_mcp = xsc_pointing_state[which].counter_mcp;
    // } else {
    //     xsc_client_data.counter_mcp = xsc_pointing_state[which].last_counter_mcp;
    // }
    /**
     * note: everything that must be valid, and doesn't come from CommandData.XSC.net,
     * must be set here or it will be overwritten by CommandData.XSC.net
     */

    xsc_client_data.xsc_protocol_version = XSC_PROTOCOL_VERSION;

    ph_stm_write(state[which].sock->stream, &xsc_client_data, sizeof(xsc_client_data), NULL);
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
    xsc_state_t *state = (xsc_state_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            if (!state->have_warned_connect) blast_err("resolve %s:%d failed %s",
                     addresses[state->which], port, gai_strerror(m_errcode));
            state->have_warned_connect = 1;

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;

        case PH_SOCK_CONNECT_ERRNO:
            if (!state->have_warned_connect) blast_err("connect %s:%d failed: `Error %d: %s`",
                      addresses[state->which], port, m_errcode, strerror(m_errcode));
            state->have_warned_connect = 1;

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;
    }

    blast_info("Connected to XSC%d at %s", state->which, addresses[state->which]);
    state->have_warned_connect = 0;

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
static void connect_xsc(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    xsc_state_t *state = (xsc_state_t*)m_data;

    if (!state->have_warned_connect) blast_info("Connecting to %s", addresses[state->which]);
    state->have_warned_connect = 1;

    ph_sock_resolve_and_connect(addresses[state->which], port, 0,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM,
        connected, m_data);
}

/**
 * Initialize the star camera I/O routine.  The state variable tracks each
 * camera connection and is passed to the connect job.
 *
 * @param m_which 0,1 for XSC0 or XSC1
 */
void xsc_networking_init(int m_which)
{
    // blast_dbg("XSC Init for %d", m_which);
    for (unsigned int i = 0; i < 3; i++) {
        xsc_clear_server_data(&xsc_mserver_data[m_which][i]);
        xsc_init_server_data(&xsc_mserver_data[m_which][i]);
    }

    state[m_which].connected = false;
    state[m_which].have_warned_version = false;
    state[m_which].which = m_which;
    state[m_which].backoff_sec = min_backoff_sec;
    state[m_which].timeout.tv_sec = 5;
    state[m_which].timeout.tv_usec = 0;
    ph_job_init(&(state[m_which].connect_job));
    state[m_which].connect_job.callback = connect_xsc;
    state[m_which].connect_job.data = &state[m_which];

    ph_job_dispatch_now(&(state[m_which].connect_job));
}

