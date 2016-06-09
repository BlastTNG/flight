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
#include <modbus/modbus.h>
#include <errno.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "blast.h"
#include "mcp.h"

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

// Stream addresses/starting addresses
#define STREAM_SCANRATE_HZ_ADDR 4002
#define STREAM_NUM_ADDRESSES_ADDR 4004
#define STREAM_SAMPLES_PER_PACKET_ADDR 4006
#define STREAM_SETTLING_US_ADDR 4008
#define STREAM_RESOLUTION_INDEX_ADDR 4010
#define STREAM_BUFFER_SIZE_BYTES_ADDR 4012
#define STREAM_CLOCK_SOURCE_ADDR 4014
#define STREAM_AUTO_TARGET_ADDR 4016
#define STREAM_NUM_SCANS_ADDR 4020
#define STREAM_EXTERNAL_CLOCK_DIVISOR_ADDR 4022
#define STREAM_ENABLE_ADDR 4990
#define STREAM_SCANLIST_ADDRESS_ADDR 4100 // #(0:127)
#define STREAM_TRIGGER_INDEX_ADDR 4024

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

// Status of the labjack commanding thread.
typedef enum {
    LJ_STATE_DISCONNECT = 0,
    LJ_STATE_READY,
    LJ_STATE_RESET,
    LJ_STATE_SHUTDOWN
} e_ljc_state_t;


typedef struct {
    char address[16];
    char ip[16];
    int which;

    ph_thread_t cmd_thread;
    modbus_t *cmd_mb;

    uint16_t port;
    bool connected;
    bool have_warned_version;
    bool shutdown;

// Used for setting up the streaming in the command thread
    uint16_t comm_stream_state;
    uint16_t req_comm_stream_state;
    uint16_t has_comm_stream_error;

    float DAC[2];

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


static labjack_state_t state[2] = {
    {
          .address = "labjack1",
          .port = LJ_DATA_PORT,
          .DAC = {0, 0}
    }
};

uint16_t labjack_get_value(int m_labjack, int m_channel)
{
    labjack_data_t *state_data = (labjack_data_t*)state[m_labjack].conn_data;
    if (m_channel > state_data->num_channels) {
        blast_err("Invalid channel %d requested from '%s'", m_channel, state[m_labjack].address);
        return 0;
    }
    return state_data->data[m_channel];
}

int labjack_analog_in_config(labjack_state_t *m_state, uint32_t m_numaddresses,
                             const uint32_t *m_scan_addresses, const uint16_t *m_chan_list,
                             const float *m_range_list)
{
    uint16_t data[2] = {0};
    unsigned int i = 0;
    int ret = 0;

    for (i = 0; i < m_numaddresses; i++) {
        if (m_scan_addresses[i] % 2 != 0 || m_scan_addresses[i] > 508) {
            blast_err("Invalid AIN address %d\n", m_scan_addresses[i]);
            return -1;
        }
    }

    for (i = 0; i < m_numaddresses; i++) {
        // Setting AIN range.
        // Starting address is 40000 (AIN0_RANGE).
        modbus_set_float(m_range_list[i], data);
        if ((ret = modbus_write_registers(m_state->cmd_mb, 40000 + m_scan_addresses[i], 2, data)) < 0) {
            blast_err("Could not set AIN registers!");
            return ret;
        }

        // Setting AIN negative channel.
        // Starting address is 41000 (AIN0_NEGATIVE_CH).
        if ((ret = modbus_write_register(m_state->cmd_mb, 41000 + m_scan_addresses[i] / 2,
                                         htons(m_chan_list[i]))) < 0) {
            blast_err("Could not set negative registers");
            return ret;
        }
    }
    return 0;
}

static void init_labjack_stream_commands(labjack_state_t *m_state)
{
    static int have_warned_write_reg = 0;
    int ret = 0;
    uint16_t data[2] = {0}; // Used to write floats.

    // Configure stream
    enum {NUM_ADDRESSES = 7};
    float scanRate = 1000.0f; // Scans per second. Samples per second = scanRate * numAddresses
    unsigned int numAddresses = NUM_ADDRESSES;
    unsigned int samplesPerPacket = 70;
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
    for (int i = 0; i < 2 * numAddresses; i+=2) {
        scanListAddresses[i] = i * 2; // AIN(i) (Modbus address i*2)
        nChanList[i] = 199; // Negative channel is 199 (single ended)
        rangeList[i] = 10.0; // 0.0 = +/-10V, 10.0 = +/-10V, 1.0 = +/-1V, 0.1 = +/-0.1V, or 0.01 = +/-0.01V.
        gainList[i] = 0; // gain index 0 = +/-10V
    }

    // Write to appropriate Modbus registers to setup and start labjack streaming.
    modbus_set_float(scanRate, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, 4002, 2, data)) < 0) {
        if (!have_warned_write_reg) {
           blast_err("Could not set stream scan rate at address: %s",
                modbus_strerror(errno));
        }
        m_state->has_comm_stream_error = 1;
        have_warned_write_reg = 1;
        return;
    }
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_NUM_ADDRESSES_ADDR, 2, &numAddresses)) < 0) {
        if (!have_warned_write_reg) {
           blast_err("Could not set stream number of addresses: %s",
                modbus_strerror(errno));
        }
        m_state->has_comm_stream_error = 1;
        have_warned_write_reg = 1;
        return;
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
            blast_err("resolve %s:%d failed %s", state->address, state->port, gai_strerror(m_errcode));

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;

        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->address, state->port, m_errcode, strerror(m_errcode));

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
    m_sock->callback = labjack_process_stream;
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

void *labjack_cmd_thread(void *m_lj) {
    static int have_warned_connect = 0;
    static int have_warned_write_reg = 0;
    labjack_state_t *state = (labjack_state_t*)m_lj;

    char tname[10];
    snprintf(tname, sizeof(tname), "LJCMD%1d", state->which);
    ph_thread_set_name(tname);
    nameThread(tname);

    blast_info("Starting Labjack%02d Commanding at IP %s", state->which, state->address);

	state->req_comm_stream_state = 1;
	state->comm_stream_state = 0;
	state->has_comm_stream_error = 0;

    {
        struct hostent *lj_ent = gethostbyname(state->address);
        uint32_t hostaddr;
        if (!lj_ent) {
            blast_err("Could not resolve %s!", state->address);
            return NULL;
        }
        hostaddr = *(uint32_t*)(lj_ent->h_addr_list[0]);

        snprintf(state->ip, sizeof(state->ip), "%d.%d.%d.%d",
                 (hostaddr & 0xff), ((hostaddr >> 8) & 0xff),
                 ((hostaddr >> 16) & 0xff), ((hostaddr >> 24) & 0xff));
        blast_info("Labjack%02d address %s corresponds to IP %s", state->which, state->address, state->ip);
    }
    while (!state->shutdown) {
        uint16_t dac_buffer[4];
        usleep(10000);
        if (!state->cmd_mb) {
            state->cmd_mb = modbus_new_tcp(state->ip, 502);

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            modbus_set_slave(state->cmd_mb, 1);
            modbus_set_response_timeout(state->cmd_mb, &tv);
            modbus_set_error_recovery(state->cmd_mb,
                                      MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL);

            if (modbus_connect(state->cmd_mb)) {
                if (!have_warned_connect) {
                    blast_err("Could not connect to ModBUS charge controller at %s: %s", state->address,
                            modbus_strerror(errno));
                }
                modbus_free(state->cmd_mb);
                state->cmd_mb = NULL;
                have_warned_connect = 1;
                continue;
            }
            have_warned_connect = 0;
        }

    /*  Start streaming */
    if (state->req_comm_stream_state && !state->comm_stream_state) {
        init_labjack_stream_commands(state);
    }
	/*  Set DAC level */
        modbus_set_float(state->DAC[0], &dac_buffer[0]);
        modbus_set_float(state->DAC[1], &dac_buffer[2]);
        if (modbus_write_registers(state->cmd_mb, 1000, 4, dac_buffer) < 0) {
            if (!have_warned_write_reg) {
                blast_err("Could not write DAC Modbus registers: %s", modbus_strerror(errno));
            }
            have_warned_write_reg = 0;
            continue;
        }
    }
    return NULL;
}

/** Create labjack commanding thread.
  * Called by mcp during startup.
  */

void initialize_labjack_commands(int m_which)
{
    ph_thread_t *ljcomm_thread = NULL;

    blast_info("start_labjack_command: creating labjack %d ModBus thread", m_which);

    ljcomm_thread = ph_thread_spawn(labjack_cmd_thread, (void*) &state[m_which]);
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


    state[m_which].connected = false;
    state[m_which].have_warned_version = false;
    state[m_which].backoff_sec = min_backoff_sec;
    state[m_which].timeout.tv_sec = 5;
    state[m_which].timeout.tv_usec = 0;
    ph_job_init(&(state[m_which].connect_job));
    state[m_which].connect_job.callback = connect_lj;
    state[m_which].connect_job.data = &state[m_which];
    labjack_data_t *data_state = calloc(1, sizeof(labjack_data_t) + m_numchannels * sizeof(uint16_t));
    data_state->num_channels = m_numchannels;
    state[m_which].conn_data = data_state;

    ph_job_dispatch_now(&(state[m_which].connect_job));
}
