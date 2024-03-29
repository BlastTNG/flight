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
 * Created on: May 23, 2017 by ian
 */


#include <stdint.h>
#include <glib.h>
#include <modbus/modbus.h>
#include <errno.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "command_struct.h"
#include "blast.h"
#include "mcp.h"
#include "tx.h"
#include "labjack_functions.h"

#include "channels_tng.h"
#include "lut.h"



static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;
extern uint16_t InCharge;

labjack_state_t state[NUM_LABJACKS] = {
    {
        .which = 0,
        .address = "labjack1",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_cryo_labjack1",
        .have_warned_write_reg = 0,
        .initialized = 0,
        .connected = 0,
    },
    {
        .which = 1,
        .address = "labjack2",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_cryo_labjack2",
        .have_warned_write_reg = 0,
        .initialized = 0,
        .connected = 0,
    },
    {
        .which = 2,
        .address = "labjack3",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_of_labjack1",
        .have_warned_write_reg = 0,
        .initialized = 0,
        .connected = 0,
    },
    {
        .which = 3,
        .address = "labjack4",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_of_labjack2",
        .have_warned_write_reg = 0,
        .initialized = 0,
        .connected = 0,
    },
    {
        .which = 4,
        .address = "labjack5",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_of_labjack3",
        .have_warned_write_reg = 0,
        .initialized = 0,
        .connected = 0,
    },
    {
        .which = 5, // multiplexed pss
        .address = "labjack6",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_mult_labjack1",
        .initialized = 0,
        .have_warned_write_reg = 0,
    },
    {
        .which = 6, // multiplexed of
        .address = "labjack7",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_mult_labjack2",
        .initialized = 0,
        .have_warned_write_reg = 0,
    },
    {
        .which = 7,
        .address = "labjack8",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_highbay_labjack",
        .initialized = 0,
        .have_warned_write_reg = 0,
    },
    {
        .which = 8,
        .address = "labjack9",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_mapping_labjack",
        .initialized = 0,
        .have_warned_write_reg = 0,
    },
    {
        .which = 9,
        .address = "labjack10",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_microscroll_labjack",
        .initialized = 0,
        .have_warned_write_reg = 0,
    },
    {
        .which = 10,
        .address = "labjack11",
        .port = LJ_DATA_PORT,
        .DAC = {0, 0},
        .channel_postfix = "_hawkeye_labjack",
        .initialized = 0,
        .have_warned_write_reg = 0,
    }
};

labjack_digital_in_t labjack_digital;

// Used to correct for word swap between the mcp convention and the Labjack.
// float_in: floating point value to be converted to two 16-bit words.
// data: two element uint16_t array to store the modbus formated data.
void labjack_set_float(float float_in, uint16_t* data)
{
    uint16_t data_swapped[2] = {0};
    modbus_set_float(float_in, data_swapped);
    data[1] = data_swapped[0];
    data[0] = data_swapped[1];
}

// Used to correct for word swap between the mcp convention and the Labjack.
// float_in: floating point value to be converted to two 16-bit words.
// data: two element uint16_t array to store the modbus formated data.
float labjack_get_float(uint16_t* data_in)
{
    uint16_t data_swapped[2] = {0};
    data_swapped[0] = data_in[1];
    data_swapped[1] = data_in[0];
    float float_out = modbus_get_float(data_swapped);
    return float_out;
}

// Used to package a 32 bit integer into a two element array in modbus format.
void labjack_set_short(uint32_t short_in, uint16_t* data)
{
    data[1] = short_in & 0xffff;
    data[0] = (short_in & 0xffff0000) >> 16;
}

float labjack_get_value(int m_labjack, int m_channel)
{
    static int first_time = 1;
    // Make sure the labjack number is sensible and the labjack in question has had its
    // state structure initialized.
    if ((m_labjack < 0) || (m_labjack >= NUM_LABJACKS)) {
        if (first_time) {
            blast_err("No such labjack %i!", m_labjack);
            first_time = 0;
        }
        return 0;
    }
    if (!(state[m_labjack].initialized)) return 0; // We haven't yet started the command thread
        // so conn_data is not allocated.

    labjack_data_t *state_data = (labjack_data_t*)state[m_labjack].conn_data;
    if (m_channel > state_data->num_channels) {
        blast_err("Invalid channel %d requested from '%s'", m_channel, state[m_labjack].address);
        return 0;
    }
    return state[m_labjack].AIN[m_channel];
    // add incharge BS here
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

int labjack_get_volts(const labjack_device_cal_t *devCal, const uint16_t data_raw,
                      unsigned int gainIndex, float *volts)
{
    static uint16_t have_warned = 0;
    if (gainIndex > 3) {
        if (!have_warned) blast_err("Invalid gainIndex %u\n", gainIndex);
        have_warned = 1;
        return -1;
    }
    // if(*volts < devCal->HS[gainIndex].Center) {
    if (devCal->HS[gainIndex].Center > 0) {
        *volts = (devCal->HS[gainIndex].Center - data_raw) * devCal->HS[gainIndex].NSlope;
    } else {
        *volts = (data_raw - devCal->HS[gainIndex].Center) * devCal->HS[gainIndex].PSlope;
    }
    have_warned =0;
    return 0;
}

// Copied from the T7 example streaming code.
// Use for now until labjack_get_cal is working.
void labjack_get_nominal_cal(labjack_state_t *m_state, labjack_device_cal_t *devCal)
{
    int i = 0;
    devCal->HS[0].PSlope = 0.000315805780f;
    devCal->HS[0].NSlope = -0.000315805800f;
    devCal->HS[0].Center = 33523.0f;
    devCal->HS[0].Offset = -10.58695652200f;
    devCal->HS[1].PSlope = 0.0000315805780f;
    devCal->HS[1].NSlope = -0.0000315805800f;
    devCal->HS[1].Center = 33523.0f;
    devCal->HS[1].Offset = -1.058695652200f;
    devCal->HS[2].PSlope = 0.00000315805780f;
    devCal->HS[2].NSlope = -0.00000315805800f;
    devCal->HS[2].Center = 33523.0f;
    devCal->HS[2].Offset = -0.1058695652200f;
    devCal->HS[3].PSlope = 0.000000315805780f;
    devCal->HS[3].NSlope = -0.000000315805800f;
    devCal->HS[3].Center = 33523.0f;
    devCal->HS[3].Offset = -0.01058695652200f;

    for ( i = 0; i < 4; i++)
        devCal->HR[i] = devCal->HS[i];

    devCal->DAC[0].Slope = 13200.0f;
    devCal->DAC[0].Offset = 0.0f;
    devCal->DAC[1].Slope = 13200.0f;
    devCal->DAC[1].Offset = 0.0f;

    devCal->Temp_Slope = -92.379f;
    devCal->Temp_Offset = 465.129f;

    devCal->ISource_10u = 0.000010f;
    devCal->ISource_200u = 0.000200f;

    devCal->I_Bias = 0;
    m_state->calibration_read = 1;
    blast_info("Labjack calibration data read.");
}


/**
 * Convert data from labjack format to voltages for use by mcp.
 *
 * @param m_state: state structure (which contains both the digital data and AIN array)
 * @param data_raw: Raw digital data
 */
void labjack_convert_stream_data(labjack_state_t *m_state, labjack_device_cal_t *m_labjack_cal,
                                 uint32_t *m_gainlist, uint16_t n_data)
{
    labjack_data_t *raw_data = (labjack_data_t*) m_state->conn_data;
    int ret;
    for (int i = 0; i < n_data; i++) {
        if (raw_data->data[i] == 0xffff) {
            // blast_err("Labjack channel AIN%d received a dummy sample indicating we received an incomplete scan!", i);
        } else {
            ret = labjack_get_volts(m_labjack_cal, raw_data->data[i], m_gainlist[i], &(m_state->AIN[i]));
        }
    }
    /*
    if (m_state->which == 6) {
        static int counter = 1;
        if (counter == 1) {
            for (int i = 0; i < 85; i++) {
                blast_info("data for %d is value %f", i, m_state->AIN[i]);
            }
        }
            counter++;
            if (counter > 200) {
                counter = 1;
            }
        }
     */
    }
// Correct for word swaps between mcp and the labjack
int labjack_data_word_swap(labjack_data_pkt_t* m_data_pkt, size_t n_bytes)
{
    uint16_t data_swapped;
    static int have_warned = 0;

    int n_data = (n_bytes - 16) / 2;

    // TODO(laura): add proper error handling.
    if (n_bytes < 16) {
        blast_err("Read only %u bytes!  Aborting", (unsigned int) n_bytes);
        have_warned = 1;
        return -1;
    }
    if (n_bytes % 2) { // We should have an even number of bytes
        blast_err("Odd number of bytes read!");
        have_warned = 1;
        return -1;
    }
    data_swapped = ntohs(m_data_pkt->header.resp.trans_id);
    m_data_pkt->header.resp.trans_id = data_swapped;
    data_swapped = ntohs(m_data_pkt->header.resp.proto_id);
    m_data_pkt->header.resp.proto_id = data_swapped;
    data_swapped = ntohs(m_data_pkt->header.resp.length);
    m_data_pkt->header.resp.length = data_swapped;
    data_swapped = ntohs(m_data_pkt->header.backlog);
    m_data_pkt->header.backlog = data_swapped;
    data_swapped = ntohs(m_data_pkt->header.status);
    m_data_pkt->header.status = data_swapped;
    data_swapped = ntohs(m_data_pkt->header.addl_status);
    m_data_pkt->header.addl_status = data_swapped;

    // Correct the streamed data
    for (int i = 0; i < n_data; i++) {
        data_swapped = ntohs(m_data_pkt->data[i]);
        m_data_pkt->data[i] = data_swapped;
    }

    have_warned = 0;
    return 1;
}


// labjack functions from Ian are below

void labjack_reboot(int m_labjack) {
    uint16_t data[2] = {0};
    data[1] = 0x0000;
    data[0] = 0x4c4a;
    int ret;
    static int max_tries = 10;
    ret = modbus_write_registers(state[m_labjack].cmd_mb, REBOOT_ADDR, 2, data);
    if (ret < 0) {
        int tries = 1;
        while (tries < max_tries) {
            tries++;
            usleep(100);
            ret = modbus_write_registers(state[m_labjack].cmd_mb, REBOOT_ADDR, 2, data);
            if (ret > 0) {
                break;
            }
        }
    }
}

void labjack_test_dac(float v_value, int m_labjack)
{
    uint16_t data[2];
    int ret;
    static int max_tries = 10;
    labjack_set_float(v_value, data);
    // data[1] = 0xffff;
    // data[0] = 0x0000;
    ret = modbus_write_registers(state[m_labjack].cmd_mb, 1000, 2, data);
    if (ret < 0) {
        int tries = 1;
        while (tries < max_tries) {
            tries++;
            usleep(100);
            ret = modbus_write_registers(state[m_labjack].cmd_mb, 1000, 2, data);
            if (ret > 0) {
                break;
            }
        }
    }
}

void query_time(int m_labjack)
{
    uint16_t data[2] = {0};
    uint32_t time_up;
    int ret;
    static int max_tries = 10;
    ret = modbus_read_registers(state[m_labjack].cmd_mb, 61522, 2, data);
    if (ret < 0) {
        int tries = 1;
        while (tries < max_tries) {
            tries++;
            usleep(100);
            ret = modbus_read_registers(state[m_labjack].cmd_mb, 61522, 2, data);
            if (ret > 0) {
                blast_warn("the system has been up for %u", data[1]);
            }
        }
    } else {
        blast_warn("the system has been up for %u", data[1]);
    }
}

int labjack_dio(int m_labjack, int address, int command) {
    int ret;
    static int max_tries = 10;
    uint16_t err_data[2] = {0}; // Used to read labjack specific error codes.
    ret = modbus_write_register(state[m_labjack].cmd_mb, address, command);
    if (ret < 0) {
        int tries = 1;
        while (tries < max_tries) {
            tries++;
            usleep(100);
            ret = modbus_write_register(state[m_labjack].cmd_mb, address, command);
            if (ret > 0) {
                break;
            }
        }
        return command;
    } else {
        return command;
    }
}


uint16_t labjack_read_dio(int m_labjack, int address) {
    uint16_t ret[1];
    int works;
    uint16_t value;
    static int max_tries = 10;
    works = modbus_read_registers(state[m_labjack].cmd_mb, address, 1, ret);
    value = ret[0];
    if (works < 0) {
        int tries = 1;
        while (tries < max_tries) {
            tries++;
            usleep(100);
            works = modbus_read_registers(state[m_labjack].cmd_mb, address, 1, ret);
            value = ret[0];
            if (works > 0) {
                break;
            }
        }
        return value;
    } else {
        return value;
    }
}

void heater_write(int m_labjack, int address, float command) {
    int ret;
    uint16_t retprime[1];
    int works;
    uint16_t value;
    static int max_tries = 10;
    uint16_t data[2];
    labjack_set_float(command, data);
    if (m_labjack != 1) {
        if (address != 1000 && address != 1002 &&
			   	address != 30004 && address != 30006 && address != 30008 && address != 30010) {
            ret = modbus_write_register(state[m_labjack].cmd_mb, address, command);
            if (ret < 0) {
                int tries = 1;
                while (tries < max_tries) {
                    tries++;
                    usleep(100);
                    ret = modbus_write_register(state[m_labjack].cmd_mb, address, command);
                    if (ret > 0) {
                        break;
                    }
                }
            }
        } else {
            // blast_info("writing to a DAC");
			// blast_info("Writing to labjack %d address %d command %f", m_labjack, address, command);
			// blast_info("data[0]=%u, data[1]=%u", data[0], data[1]);
            ret = modbus_write_registers(state[m_labjack].cmd_mb, address, 2, data);
            if (ret < 0) {
                int tries = 1;
                while (tries < max_tries) {
                    tries++;
                    usleep(100);
                    ret = modbus_write_registers(state[m_labjack].cmd_mb, address, 2, data);
                    if (ret > 0) {
                        break;
                    }
                }
            }
        }
    }
    if (m_labjack == 1 && address > 2008) {
        works = modbus_read_registers(state[m_labjack].cmd_mb, address, 1, retprime);
        value = retprime[0];
        if (works < 0) {
            int tries = 1;
            while (tries < max_tries) {
                tries++;
                usleep(100);
                works = modbus_read_registers(state[m_labjack].cmd_mb, address, 1, retprime);
                value = retprime[0];
                if (works > 0) {
                    break;
                }
            }
            switch (address) {
                case 2009:
                    SET_SCALED_VALUE(labjack_digital.status_charcoal_heater_Addr, value);
                    break;
                case 2010:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_250_LNA_Addr, value);
                    break;
                case 2011:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_1K_heater_Addr, value);
                    break;
                case 2013:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_charcoal_hs_Addr, value);
                    break;
                case 2015:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_350_LNA_Addr, value);
                    break;
                case 2016:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_500_LNA_Addr, value);
                    break;
            }
        } else {
            switch (address) {
                case 2009:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_charcoal_heater_Addr, value);
                    break;
                case 2010:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_250_LNA_Addr, value);
                    break;
                case 2011:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_1K_heater_Addr, value);
                    break;
                case 2013:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_charcoal_hs_Addr, value);
                    break;
                case 2015:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_350_LNA_Addr, value);
                    break;
                case 2016:
                    // blast_info("writing to %d, value %d", address, value);
                    SET_SCALED_VALUE(labjack_digital.status_500_LNA_Addr, value);
                    break;
            }
        }
    }
    if (m_labjack == 1 && address <= 2008) {
        ret = modbus_write_register(state[m_labjack].cmd_mb, address, command);
        if (ret < 0) {
            int tries = 1;
            while (tries < max_tries) {
                tries++;
                usleep(100);
                ret = modbus_write_register(state[m_labjack].cmd_mb, address, command);
                if (ret > 0) {
                    break;
                }
            }
        }
    }
}

void init_labjack_10hz_filter(labjack_10hz_filter_t *m_lj_filter) {
    for (int i = 0; i < LJ_10HZ_BUFFER_LEN; i++) m_lj_filter->val_buf[i] = 0.0;
    m_lj_filter->ind_last = 0;
    m_lj_filter->sum = 0.0;
}

void filter_labjack_channel_10hz(float new_val, labjack_10hz_filter_t *m_lj_filter) {
    m_lj_filter->sum = m_lj_filter->sum + new_val - m_lj_filter->val_buf[m_lj_filter->ind_last];
    m_lj_filter->val_buf[m_lj_filter->ind_last] = new_val;
    m_lj_filter->ind_last = (++(m_lj_filter->ind_last)) % LJ_10HZ_BUFFER_LEN;
    m_lj_filter->filt_val = m_lj_filter->sum / LJ_10HZ_BUFFER_LEN;
}
