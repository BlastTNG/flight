/**
 * @file labjack_functions.h
 *
 * @date may 23, 2017
 * @author ian
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2017 University of Pennsylvania
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

#ifndef INCLUDE_LABJACK_FUNCTIONS_H_
#define INCLUDE_LABJACK_FUNCTIONS_H_

#include <stdint.h>
#include <glib.h>
#include <modbus/modbus.h>
#include <errno.h>
#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "command_struct.h"
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
#define REBOOT_ADDR 61998


// Modbus addresses to set the ranges and gains of the Analog Inputs
#define AIN0_RANGE_ADDR 40000 // Setting AIN range for each AIN channel
// 0.0 = +/-10V, 10.0 = +/-10V, 1.0 = +/-1V, 0.1 = +/-0.1V, or 0.01 = +/-0.01
#define AIN0_NEGATIVE_CH_ADDR 41000 // Negative channel is 199 (single ended)

#define LJ_MODBUS_ERROR_INFO_ADDR 55000 // Gives a specific error code

#define LJ_CMD_PORT 502
#define LJ_DATA_PORT 702

// TODO(laura): make this commandable from the call in mcp.c
#define LJ_STREAM_RATE 200.0 // Streaming Rate (Hz)


// Maximum number of addresses that can be targeted in stream mode.
#define MAX_NUM_ADDRESSES 4096

// For now we only have one cyro readout Labjack
// TODO(laura): Integrate PSS and OF Labjacks
#define NUM_LABJACKS 6

typedef struct { // temp names
    channel_t* status_charcoal_heater_Addr;
    channel_t* status_250_LNA_Addr;
    channel_t* status_1K_heater_Addr;
    channel_t* status_charcoal_hs_Addr;
    channel_t* status_500_LNA_Addr;
    channel_t* status_350_LNA_Addr;
} labjack_digital_in_t;

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

// Calibration structures to convert from AIN digital units to voltages.
typedef struct
{
    float PSlope;
    float NSlope;
    float Center;
    float Offset;
} labjack_calset_t;

// Stores T7 calibration constants
typedef struct
{
    labjack_calset_t HS[4];
    labjack_calset_t HR[4];
    struct
    {
        float Slope;
        float Offset;
    } DAC[2];
    float Temp_Slope;
    float Temp_Offset;
    float ISource_10u;
    float ISource_200u;
    float I_Bias;
} labjack_device_cal_t;

typedef struct {
    int trans_id;
    uint16_t num_channels;
    uint16_t scans_per_packet;
    uint16_t data[];
} labjack_data_t;


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
    bool initialized;
    // Used for setting up the streaming in the command thread
    uint16_t comm_stream_state;
    uint16_t req_comm_stream_state;
    uint16_t has_comm_stream_error;
    uint16_t have_warned_write_reg;
    uint16_t calibration_read;

    float DAC[2];
    float AIN[84]; // Analog input channels read from Labjack

    uint32_t backoff_sec;
    struct timeval timeout;
    ph_job_t connect_job;
    ph_sock_t *sock;
    void *conn_data; // points to a labjack_data_t structure.
    char channel_postfix[16];
    int num_LJ;
} labjack_state_t;

typedef struct {
    int trans_id;
    int complete;
    int has_error;
    uint16_t expected_size;
    void (*handle_success)(labjack_state_t*, ph_buf_t*);
} labjack_trans_t;

void labjack_set_float(float float_in, uint16_t* data);
float labjack_get_float(uint16_t* data_in);
void labjack_set_short(uint32_t short_in, uint16_t* data);
float labjack_get_value(int m_labjack, int m_channel);
int labjack_analog_in_config(labjack_state_t *m_state, uint32_t m_numaddresses,
                             const uint32_t *m_scan_addresses, const uint16_t *m_chan_list,
                             const float *m_range_list);
int labjack_get_volts(const labjack_device_cal_t *devCal, const uint16_t data_raw,
                      unsigned int gainIndex, float *volts);
void labjack_get_nominal_cal(labjack_state_t *m_state, labjack_device_cal_t *devCal);
void labjack_convert_stream_data(labjack_state_t *m_state, labjack_device_cal_t *m_labjack_cal,
                                 uint32_t *m_gainlist, uint16_t n_data);
void labjack_reboot(int m_labjack);
void labjack_test_dac(float v_value, int m_labjack);
void query_time(int m_labjack);
int labjack_dio(int m_labjack, int address, int command);
uint16_t labjack_read_dio(int m_labjack, int address);
void heater_write(int m_labjack, int address, int command);
int labjack_data_word_swap(labjack_data_pkt_t* m_data_pkt, size_t n_bytes);
void labjack_process_stream(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data);




#endif /* LABJACK_FUNCTIONS_H_ */
