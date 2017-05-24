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
#include "phenom/queue.h"

#include "command_struct.h"
#include "blast.h"
#include "mcp.h"
#include "mputs.h"
#include "tx.h"
#include "labjack_functions.h"

#define NUM_LABJACK_AIN 14
extern labjack_state_t state[NUM_LABJACKS];
extern int16_t InCharge;

static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;

typedef struct labjack_command {
    PH_STAILQ_ENTRY(labjack_command) q;
    int labjack;
    int address;
    int command;
} labjack_command_t;

PH_STAILQ_HEAD(labjack_command_q, labjack_command)
    s_labjack_command = PH_STAILQ_HEAD_INITIALIZER(s_labjack_command);

static void labjack_execute_command_queue(void) {
    labjack_command_t *cmd, *tcmd;
    PH_STAILQ_FOREACH_SAFE(cmd, &s_labjack_command, q, tcmd) {
        if (InCharge) {
            heater_write(cmd->labjack, cmd->address, cmd->command);
        }
        PH_STAILQ_REMOVE_HEAD(&s_labjack_command, q);
        free(cmd);
    }
}

void labjack_queue_command(int m_labjack, int m_address, int m_command) {
    labjack_command_t *cmd;

    cmd = balloc(err, sizeof(labjack_command_t));
    cmd->labjack = m_labjack;
    cmd->address = m_address;
    cmd->command = m_command;
    PH_STAILQ_INSERT_TAIL(&s_labjack_command, cmd, q);
}


// This isn't working now.  TODO(laura): Fix read from internal FLASH memory.
int labjack_get_cal(labjack_state_t *m_state, labjack_device_cal_t *devCal)
{
	const unsigned int EFAdd_CalValues = 0x3C4000;
	const int FLASH_PTR_ADDRESS	= 61810;

	// 3 frames	of 13 values, one frame	of 2 values
	const int FLASH_READ_ADDRESS = 61812;
	const int FLASH_READ_NUM_REGS[4] = {26, 26, 26, 4};
    uint16_t err_data[2] = {0}; // Used to read labjack specific error codes.
	static uint16_t have_warned_write_mem = 0;
	static uint16_t have_warned_read_cal = 0;
    int ret = 0;

	float calValue = 0.0;
	int calIndex = 0;
	uint16_t data[26];

	int i = 0;
	int j = 0;

	for(i = 0; i < 4; i++) {
		// Set the pointer. This indicates which part of the memory we want to read
		blast_info("i = %i, Flash Memory Address = %i", i, EFAdd_CalValues + i * 13 * 4);
        labjack_set_short(EFAdd_CalValues + i * 13 * 4, data);
        if ((ret = modbus_write_registers(m_state->cmd_mb, FLASH_PTR_ADDRESS, 2, data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!have_warned_write_mem) {
                blast_err("Could not set memory to read cal info (index = %i): %s. Data sent [0]=%d, [1]=%d",
                    i, modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            have_warned_write_mem = 1;
            m_state->calibration_read = 0;
            return -1;
        }
// uint32ToBytes(EFAdd_CalValues + i * 13 * 4, data);
// if(writeMultipleRegistersTCP(sock, FLASH_PTR_ADDRESS, 2, data) < 0)
// return -1;

		// Read the calibration constants
        if ((ret = modbus_read_registers(m_state->cmd_mb, FLASH_READ_ADDRESS, FLASH_READ_NUM_REGS[i], data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!have_warned_read_cal) {
                blast_err("Could not read cal info (index = %i): %s. Data sent [0]=%d, [1]=%d",
                    i, modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            have_warned_read_cal = 1;
            m_state->calibration_read = 0;
            return -1;
        }
// if(readMultipleRegistersTCP(sock, FLASH_READ_ADDRESS, FLASH_READ_NUM_REGS[i], data) < 0)
//     return -1;

		for(j = 0; j < FLASH_READ_NUM_REGS[i]*2; j+=4) {
			calValue = labjack_get_float(&data[j]);
			((float *)devCal)[calIndex]	= calValue;
			blast_info("Dev Cal i=%i, j=%i, data[j] = %u, val=%f", i, j, data[j], calValue);
			calIndex++;
		}
	}
	blast_info("Successfully read labjack calibration info.");
    have_warned_read_cal = 0;
    have_warned_write_mem = 0;
    m_state->calibration_read = 1;
	return 0;
}



static void init_labjack_stream_commands(labjack_state_t *m_state)
{
    int ret = 0;
    int m_state_number;
    uint16_t data[2] = {0}; // Used to write floats.
    uint16_t err_data[2] = {0}; // Used to read labjack specific error codes.
    labjack_data_t *state_data = (labjack_data_t*)m_state->conn_data;

    // Configure stream
    float scanRate = LJ_STREAM_RATE; // Scans per second. Samples per second = scanRate * numAddresses
    unsigned int numAddresses = state_data->num_channels;
    unsigned int samplesPerPacket = numAddresses*state_data->scans_per_packet;
    float settling = 10.0; // 10 microseconds
    unsigned int resolutionIndex = 0;
    unsigned int bufferSizeBytes = 0;
    unsigned int autoTarget = STREAM_TARGET_ETHERNET;
    unsigned int numScans = 0; // 0 = Run continuously.
    unsigned int scanListAddresses[MAX_NUM_ADDRESSES] = {0};
    uint16_t nChanList[MAX_NUM_ADDRESSES] = {0};
    float rangeList[MAX_NUM_ADDRESSES];

	blast_info("Attempting to set registers for labjack%02d streaming.", m_state->which);
// Disable streaming (otherwise we can't set the other streaming registers.
    labjack_set_short(0, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_ENABLE_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not disable streaming (could be streaming is off already): %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
    }
    // Write to appropriate Modbus registers to setup and start labjack streaming.
    labjack_set_float(scanRate, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SCANRATE_HZ_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream scan rate at address: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(numAddresses, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_NUM_ADDRESSES_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream number of addresses: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(samplesPerPacket, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SAMPLES_PER_PACKET_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set samples per packet: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_float(settling, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SETTLING_US_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream settling time: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(resolutionIndex, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_RESOLUTION_INDEX_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream resolution: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(bufferSizeBytes, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_BUFFER_SIZE_BYTES_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream buffer size: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(autoTarget, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_AUTO_TARGET_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream auto target: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(numScans, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_NUM_SCANS_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set continuous scanning: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }

	blast_info("Setting Modbus register addresses for labjack%02d streaming.", m_state->which);

    // Using a loop to add Modbus addresses for AIN0 - AIN(NUM_ADDRESSES-1) to the
    // stream scan and configure the analog input settings.
    for (int i = 0; i < numAddresses; i++) {
        scanListAddresses[i] = i*2; // AIN(i) (Modbus address i*2)
        nChanList[i] = 199; // Negative channel is 199 (single ended)
        // rangeList[i] = 10.0; // 0.0 = +/-10V, 10.0 = +/-10V, 1.0 = +/-1V, 0.1 = +/-0.1V, or 0.01 = +/-0.01V.
	    labjack_set_short(scanListAddresses[i], data);
        m_state_number = m_state->which;
        if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SCANLIST_ADDRESS_ADDR + i*2, 2, data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!m_state->have_warned_write_reg) {
                blast_err("Could not set scan address %d: %s. Data sent [0]=%d, [1]=%d",
                    scanListAddresses[i], modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            m_state->has_comm_stream_error = 1;
            m_state->have_warned_write_reg = 1;
            return;
        }
        if (m_state_number == 1) {
            rangeList[0] = 0.0;
            rangeList[1] = 0.0;
            rangeList[2] = 1.0;
            rangeList[3] = 1.0;
            rangeList[4] = 1.0;
            rangeList[5] = 1.0;
            rangeList[6] = 1.0;
            rangeList[7] = 1.0;
            rangeList[8] = 1.0;
            rangeList[9] = 1.0;
            rangeList[10] = 1.0;
            rangeList[11] = 0.0;
            rangeList[12] = 0.0;
            rangeList[13] = 0.0;
            labjack_set_float(rangeList[i], data);
        }
        if (!(m_state_number == 1)) {
            rangeList[i] = 0.0;
            labjack_set_float(rangeList[i], data);
        }
        if ((ret = modbus_write_registers(m_state->cmd_mb, AIN0_RANGE_ADDR + i*2, 2, data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            int max_tries = 10;
            usleep(100);
            for (int tries = 1; tries < max_tries; tries++) {
                if ((ret = modbus_write_registers(m_state->cmd_mb, AIN0_RANGE_ADDR + i*2, 2, data)) < 0) {
                    ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
                    usleep(100);
                } else {
                    break;
                }
            }
            if (!m_state->have_warned_write_reg) {
                blast_err("Could not set %d-th AIN range: %s. Data sent [0]=%d, [1]=%d",
                    i, modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            m_state->has_comm_stream_error = 1;
            m_state->have_warned_write_reg = 1;
            return;
        } else {
        }
        if ((ret = modbus_write_registers(m_state->cmd_mb, AIN0_NEGATIVE_CH_ADDR + i, 1, nChanList+i)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!m_state->have_warned_write_reg) {
                blast_err("Could not set %d-th AIN negative channel: %s. Data sent %d",
                    i, modbus_strerror(errno), nChanList[i]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            m_state->has_comm_stream_error = 1;
            m_state->have_warned_write_reg = 1;
            return;
        }
    }
	blast_info("Attempting to enable streaming for labjack%02d.", m_state->which);

	// Last step: enable streaming
    labjack_set_short(1, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_ENABLE_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not enable streaming (could be streaming is off already): %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
	blast_info("Stream configuration commanding completed.");
	m_state->has_comm_stream_error = 0;
	m_state->have_warned_write_reg = 0;
	m_state->comm_stream_state = 1;
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
    CommandData.Relays.labjack[state->which] = 1;
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
    labjack_state_t *m_state = (labjack_state_t*)m_lj;

    char tname[10];
    snprintf(tname, sizeof(tname), "LJCMD%1d", m_state->which);
    ph_thread_set_name(tname);
    nameThread(tname);

    blast_info("Starting Labjack%02d Commanding at IP %s", m_state->which, m_state->address);
    /* while (!InCharge) {
        if (first_time) {
            blast_info("not in charge... waiting");
            first_time = 0;
        }
        usleep(1000);
    } */
	m_state->req_comm_stream_state = 1;
	m_state->comm_stream_state = 0;
	m_state->has_comm_stream_error = 0;

    {
        struct hostent *lj_ent = gethostbyname(m_state->address);
        uint32_t hostaddr;
        if (!lj_ent) {
            blast_err("Could not resolve %s!", m_state->address);
            return NULL;
        }
        hostaddr = *(uint32_t*)(lj_ent->h_addr_list[0]);

        snprintf(m_state->ip, sizeof(m_state->ip), "%d.%d.%d.%d",
                 (hostaddr & 0xff), ((hostaddr >> 8) & 0xff),
                 ((hostaddr >> 16) & 0xff), ((hostaddr >> 24) & 0xff));
        blast_info("Labjack%02d address %s corresponds to IP %s", m_state->which, m_state->address, m_state->ip);
    }
    while (!m_state->shutdown) {
        usleep(10000);
        if (!m_state->cmd_mb) {
            m_state->cmd_mb = modbus_new_tcp(m_state->ip, 502);

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            modbus_set_slave(m_state->cmd_mb, 1);
            modbus_set_response_timeout(m_state->cmd_mb, &tv);
            modbus_set_error_recovery(m_state->cmd_mb,
                                      MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL);

            if (modbus_connect(m_state->cmd_mb)) {
                if (!have_warned_connect) {
                    blast_err("Could not connect to ModBUS charge controller at %s: %s", m_state->address,
                            modbus_strerror(errno));
                }
                modbus_free(m_state->cmd_mb);
                m_state->cmd_mb = NULL;
                have_warned_connect = 1;
                continue;
            }
            have_warned_connect = 0;
        }

        /*  Start streaming */
        if (m_state->req_comm_stream_state && !m_state->comm_stream_state) {
            init_labjack_stream_commands(m_state);
        }

        labjack_execute_command_queue();

        /*
          // Set DAC level
            modbus_set_float(m_state->DAC[0], &dac_buffer[0]);
            modbus_set_float(m_state->DAC[1], &dac_buffer[2]);
            if (modbus_write_registers(m_state->cmd_mb, 1000, 4, dac_buffer) < 0) {
                if (!m_state->have_warned_write_reg) {
                    blast_err("Could not write DAC Modbus registers: %s", modbus_strerror(errno));
                }
                m_state->have_warned_write_reg = 1;
                continue;
            } */
    }
    return NULL;
}

/** Create labjack commanding thread.
  * Called by mcp during startup.
  */

void initialize_labjack_commands(int m_which)
{
    blast_info("start_labjack_command: creating labjack %d ModBus thread", m_which);

    ph_thread_spawn(labjack_cmd_thread, (void*) &state[m_which]);
}

/**
 * Initialize the labjack I/O routine.  The state variable tracks each
 * labjack connection and is passed to the connect job.
 *
 * @param m_which
 */
void labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet)
{
    blast_dbg("Labjack Init for %d", m_which);


    state[m_which].connected = false;
    CommandData.Relays.labjack[m_which] = 0;
    state[m_which].have_warned_version = false;
    state[m_which].backoff_sec = min_backoff_sec;
    state[m_which].timeout.tv_sec = 5;
    state[m_which].timeout.tv_usec = 0;
    ph_job_init(&(state[m_which].connect_job));
    state[m_which].connect_job.callback = connect_lj;
    state[m_which].connect_job.data = &state[m_which];
    labjack_data_t *data_state = calloc(1, sizeof(labjack_data_t) +
        m_numchannels * m_scans_per_packet * sizeof(uint16_t));
    data_state->num_channels = m_numchannels;
    data_state->scans_per_packet = m_scans_per_packet;
    state[m_which].conn_data = data_state;
    state[m_which].initialized = true;
    ph_job_dispatch_now(&(state[m_which].connect_job));
}

void store_labjack_data(void)
{
    static channel_t *LabjackCryoAINAddr[NUM_LABJACKS][NUM_LABJACK_AIN];
	char channel_name[128] = {0};
	int i, j;
    static int firsttime = 1;

    if (firsttime) {
        firsttime = 0;
        for (i = 0; i < NUM_LABJACKS; i++) {
            for (j = 0; j < NUM_LABJACK_AIN; j++) {
                snprintf(channel_name, sizeof(channel_name), "ain%02d%s", j, state[i].channel_postfix);
                LabjackCryoAINAddr[i][j] = channels_find_by_name(channel_name);
            }
        }
    }

    for (i = 0; i < NUM_LABJACKS; i++) {
        for (j = 0; j < NUM_LABJACK_AIN; j++) {
            SET_SCALED_VALUE(LabjackCryoAINAddr[i][j], state[i].AIN[j]);
//            blast_info("ain%02d%s = %f", j, state[i].channel_postfix, state[i].AIN[j]);
        }
    }
}
