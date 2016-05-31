/*
 * labjack.c:
 *
 * This software is copyright
 *  (C) 2014-2016 University of Pennsylvania
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
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: May 5, 2016 by seth
 */


#include <stdint.h>
#include <glib.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "blast.h"

// Target types for stream configuration
#define STREAM_TARGET_ETHERNET 0x01  // Ethernet
#define STREAM_TARGET_USB 0x02  // USB
#define STREAM_TARGET_CR 0x10  // Command/Response

// Max samples per packet
#define STREAM_MAX_SAMPLES_PER_PACKET_TCP 512
#define STREAM_TYPE 16

// Stream response statuses
#define STREAM_STATUS_AUTO_RECOVER_ACTIVE 2940
#define STREAM_STATUS_AUTO_RECOVER_END 2941  // Additional Info. = # scans skipped
#define STREAM_STATUS_SCAN_OVERLAP 2942
#define STREAM_STATUS_AUTO_RECOVER_END_OVERFLOW 2943
#define STREAM_STATUS_BURST_COMPLETE 2944

#define LJ_CMD_PORT 502
#define LJ_DATA_PORT 702

static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;

typedef struct {
    uint16_t trans_id;
    uint16_t proto_id;
    uint16_t length;
    uint8_t  unit_id; // This should be 1
    uint8_t  fn_id;   // This should be 76
    uint8_t  type;    // This should be 16 (STREAM_TYPE)
} __attribute__((packed)) labjack_resp_header_t;

typedef struct {
    labjack_resp_header_t resp;
    uint8_t  reserved;
    uint16_t backlog;
    uint16_t status;
    uint16_t addl_status;
} __attribute__((packed)) labjack_data_header_t;

typedef struct {
    labjack_data_header_t header;
    uint16_t data[];
} __attribute__((packed)) labjack_data_pkt_t;

typedef struct {
    char address[16];
    uint16_t port;
    bool connected;
    bool have_warned_version;
    uint32_t backoff_sec;
    struct timeval timeout;
    ph_job_t connect_job;
    ph_sock_t *sock;
    void *conn_data;
} labjack_state_t;

typedef struct {
    int trans_id;
    uint16_t num_channels;
    uint16_t data[];
} labjack_data_t;

typedef struct {
    int trans_id;
    int complete;
    int has_error;
    uint16_t expected_size;
    void (*handle_success)(labjack_state_t*, ph_buf_t*);
} labjack_trans_t;

#define LABJACK_CMD 0
#define LABJACK_STREAM 1

/** First index is for the labjack, second for cmd/data ports */
static labjack_state_t state[2][2] = {
        { [LABJACK_CMD] =
            {
                    .address = "labjack1",
                    .port = LJ_CMD_PORT
            },
          [LABJACK_STREAM] =
            {
                  .address = "labjack1",
                  .port = LJ_DATA_PORT
            }
        }
};

uint16_t labjack_get_value(int m_labjack, int m_channel)
{
    labjack_data_t *state_data = (labjack_data_t*)state[m_labjack][LABJACK_STREAM].conn_data;
    if (m_channel > state_data->num_channels) {
        blast_err("Invalid channel %d requested from '%s'", m_channel, state[m_labjack][LABJACK_STREAM].address);
        return 0;
    }
    return state_data->data[m_channel];
}

static gint labjack_compare_trans(gconstpointer m_p1, gconstpointer m_p2, gpointer m_data)
{
    gint16 a, b;
    a = GPOINTER_TO_INT(m_p1);
    b = GPOINTER_TO_INT(m_p2);
    return (a > b ? 1 : a == b ? 0 : 1);
}

static void init_labjack_commands(labjack_state_t *m_state)
{
    // Configure stream
    enum {NUM_ADDRESSES = 2};
    float scanRate = 1000.0f; // Scans per second. Samples per second = scanRate * numAddresses
    unsigned int numAddresses = NUM_ADDRESSES;
    unsigned int samplesPerPacket = STREAM_MAX_SAMPLES_PER_PACKET_TCP;
    float settling = 10.0; // 10 microseconds
    unsigned int resolutionIndex = 0;
    unsigned int bufferSizeBytes = 0;
    unsigned int autoTarget = STREAM_TARGET_ETHERNET;
    unsigned int numScans = 0; // 0 = Run continuously.
    unsigned int scanListAddresses[NUM_ADDRESSES] = {0};
    uint16_t nChanList[NUM_ADDRESSES] = {0};
    float rangeList[NUM_ADDRESSES] = {0.0};
    unsigned int gainList[NUM_ADDRESSES]; // Based off rangeList

    // Using a loop to add Modbus addresses for AIN0 - AIN(NUM_ADDRESSES-1) to the
    // stream scan and configure the analog input settings.
    for (int i = 0; i < numAddresses; i++) {
        scanListAddresses[i] = i * 2; // AIN(i) (Modbus address i*2)
        nChanList[i] = 199; // Negative channel is 199 (single ended)
        rangeList[i] = 10.0; // 0.0 = +/-10V, 10.0 = +/-10V, 1.0 = +/-1V, 0.1 = +/-0.1V, or 0.01 = +/-0.01V.
        gainList[i] = 0; // gain index 0 = +/-10V
    }
}


/**
 * Process an incoming LabJack packet.  If we have an error, we'll disable
 * the socket and schedule a reconnection attempt.  Otherwise, read and store the
 * camera data.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void labjack_process_cmd_resp(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    ph_buf_t *buf;
    ph_buf_t *data_buf;
    labjack_state_t *state = (labjack_state_t*) m_data;
    labjack_resp_header_t *data;
    uint16_t len;
    labjack_trans_t *transaction = NULL;

    /**
     * If we have an error, or do not receive data from the star camera in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & (PH_IOMASK_ERR|PH_IOMASK_TIME)) {
      blast_err("disconnecting LabJack command port at %s due to connection issue", state->address);
      ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
      ph_sock_enable(m_sock, 0);
      state->connected = false;
      ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 100);
      return;
    }

    buf = ph_sock_read_bytes_exact(m_sock, sizeof(labjack_resp_header_t));
    if (!buf) return; /// We do not have enough data

    data = (labjack_resp_header_t*)ph_buf_mem(buf);
    data->trans_id = be16toh(data->trans_id);
    data->length = be16toh(data->length);

    data_buf = ph_sock_read_bytes_exact(m_sock, len - 3);
    if (!data_buf) {
        blast_err("Could not read expected %d bytes from LabJack %s", len, state->address);
        data_buf = ph_sock_read_bytes_exact(m_sock, ph_bufq_len(m_sock->rbuf));
        ph_buf_delref(data_buf);
        ph_buf_delref(buf);
        return;
    }
    transaction = g_sequence_lookup(state->conn_data, &data->trans_id, labjack_compare_trans, NULL);

    transaction->handle_success(state, data_buf);

    ph_buf_delref(buf);
}

/**
 * Process an incoming LabJack packet.  If we have an error, we'll disable
 * the socket and schedule a reconnection attempt.  Otherwise, read and store the
 * camera data.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void labjack_process_stream(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    ph_buf_t *buf;
    labjack_state_t *state = (labjack_state_t*) m_data;
    labjack_data_pkt_t *data_pkt;
    labjack_data_t *state_data = (labjack_data_t*)state->conn_data;

    /**
     * If we have an error, or do not receive data from the LabJack in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & (PH_IOMASK_ERR|PH_IOMASK_TIME)) {
      blast_err("disconnecting LabJack at %s due to connection issue", state->address);
      ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
      ph_sock_enable(m_sock, 0);
      state->connected = false;
      ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 100);
      return;
    }

    buf = ph_sock_read_bytes_exact(m_sock, sizeof(labjack_data_header_t) + state_data->num_channels * 2);
    if (!buf) return; /// We do not have enough data

    data_pkt = (labjack_data_pkt_t*)ph_buf_mem(buf);

    if (data_pkt->header.resp.trans_id != ++(state_data->trans_id)) {
        blast_warn("Expected transaction ID %d but received %d from LabJack at %s",
                   state_data->trans_id, state_data->trans_id, state->address);
    }
    state_data->trans_id = data_pkt->header.resp.trans_id;

    if (data_pkt->header.resp.type != STREAM_TYPE) {
        blast_warn("Unknown packet type %d received from LabJack at %s", data_pkt->header.resp.type, state->address);
        ph_buf_delref(buf);
        return;
    }

    switch (data_pkt->header.status) {
    case STREAM_STATUS_AUTO_RECOVER_ACTIVE:
    case STREAM_STATUS_AUTO_RECOVER_END:
    case STREAM_STATUS_AUTO_RECOVER_END_OVERFLOW:
    case STREAM_STATUS_BURST_COMPLETE:
    case STREAM_STATUS_SCAN_OVERLAP:
        break;
    }

    memcpy(state_data->data, data_pkt->data, state_data->num_channels * sizeof(uint16_t));

    ph_buf_delref(buf);
}

/**
 * Handle a connection callback from @connect_lj.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our LabJack State variable
 */
static void connected(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    labjack_state_t *state = (labjack_state_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", state->address, state->address, gai_strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;

        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->address, state->address, m_errcode, strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;
    }

    blast_info("Connected to LabJack at %s", state->address);

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    state->connected = true;
    state->backoff_sec = min_backoff_sec;
    m_sock->callback = (state->port == LJ_CMD_PORT)?labjack_process_cmd_resp:labjack_process_stream;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);
}

/**
 * Handles the connection job.  Formatted this way to allow us to schedule
 * a future timeout in the PH_JOB infrastructure
 *
 * @param m_job Unused
 * @param m_why Unused
 * @param m_data Pointer to the labjack State variable
 */
static void connect_lj(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    labjack_state_t *state = (labjack_state_t*)m_data;

    blast_info("Connecting to %s", state->address);
    ph_sock_resolve_and_connect(state->address, state->port, 0,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, connected, m_data);
}

/**
 * Initialize the labjack I/O routine.  The state variable tracks each
 * labjack connection and is passed to the connect job.
 *
 * @param m_which
 */
void labjack_networking_init(int m_which, size_t m_numchannels)
{
    blast_dbg("Labjack Init for %d", m_which);

    for (int i = 0; i < 2; i++) {
        state[m_which][i].connected = false;
        state[m_which][i].have_warned_version = false;
        state[m_which][i].backoff_sec = min_backoff_sec;
        state[m_which][i].timeout.tv_sec = 5;
        state[m_which][i].timeout.tv_usec = 0;
        ph_job_init(&(state[m_which][i].connect_job));
        state[m_which][i].connect_job.callback = connect_lj;
        state[m_which][i].connect_job.data = &state[m_which][i];
        if (i == LABJACK_CMD) {
            state[m_which][i].conn_data = g_sequence_new(free);
        } else if (i == LABJACK_STREAM) {
            labjack_data_t *data_state = calloc(1, sizeof(labjack_data_t) + m_numchannels * sizeof(uint16_t));
            data_state->num_channels = m_numchannels;
            state[m_which][i].conn_data = data_state;
        }

        ph_job_dispatch_now(&(state[m_which][i].connect_job));
    }
}
