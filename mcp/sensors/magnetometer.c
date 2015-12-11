/**
 * @file magnetometer.c
 *
 * @date Nov 23, 2015
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

#include <stdint.h>
#include <endian.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/serial.h"
#include "phenom/memory.h"

#include "blast.h"
#include "channels_tng.h"
#include "pointing_struct.h"

extern struct ACSDataStruct ACSData;

#define MAGCOM "/dev/ttyUSB0"

ph_serial_t	*mag_comm = NULL;

typedef enum {
	MAG_WE_BIN = 0,
	MAG_BIN,
	MAG_WE_RATE,
	MAG_RATE,
	MAG_CONT,
	MAG_READ,
	MAG_END
} e_mag_state;

typedef struct {
	char cmd[32];
	char resp[16];
} mag_state_cmd_t;

static mag_state_cmd_t state_cmd[MAG_END] = {
		[MAG_WE_BIN] = { "*99WE", "OK" },
		[MAG_BIN] = { "*99B", "BINARY ON" },
		[MAG_WE_RATE] = { "*99WE", "OK" },
		[MAG_RATE] = { "*99R=100", "OK" },
		[MAG_CONT] = { "*99C" },
};
static int cur_state = 0;

static void mag_set_framedata(int16_t m_magx, int16_t m_magy, int16_t m_magz)
{
    static channel_t *mag_x_channel = NULL;
    static channel_t *mag_y_channel = NULL;
    static channel_t *mag_z_channel = NULL;

    if (!mag_x_channel) {
        mag_x_channel = channels_find_by_name("x_mag");
        mag_y_channel = channels_find_by_name("y_mag");
        mag_z_channel = channels_find_by_name("z_mag");
    }

    ACSData.mag_x = ((double)(int16_t)be16toh(m_magx))/15000.0;
    SET_SCALED_VALUE(mag_x_channel, ACSData.mag_x);
    ACSData.mag_y = ((double)(int16_t)be16toh(m_magy))/15000.0;
    SET_SCALED_VALUE(mag_y_channel, ACSData.mag_y);
    ACSData.mag_z = ((double)(int16_t)be16toh(m_magz))/15000.0;
    SET_SCALED_VALUE(mag_z_channel, ACSData.mag_z);
}

/**
 * Magnetometer callback function handling events from the serial device.
 * @param serial
 * @param why
 * @param m_data
 */
static void mag_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);

    typedef struct {
    	int16_t mag_x;
    	int16_t mag_y;
    	int16_t mag_z;
    } mag_data_t;
    mag_data_t *mag_reading;

    ph_buf_t *buf;

    /**
     * If we timeout, then the assumption is that we need to re-initialize the
     * magnetometer stream
     */
    if (why & PH_IOMASK_TIME) {
        cur_state = 0;
        blast_info("Sending CMD '%s' to the MAG", state_cmd[cur_state].cmd);
        ph_stm_printf(serial->stream, "%s\r", state_cmd[cur_state].cmd);
        ph_stm_flush(serial->stream);
        return;
    }

    if (why & PH_IOMASK_READ) {
        buf = ph_serial_read_record(serial, "\r", 1);
        if (!buf) return;

        /**
         * Handle the initial handshaking and setup with the magnetometer
         */
        if (cur_state < MAG_CONT) {
            if ((ph_buf_len(buf) - 1) == strlen(state_cmd[cur_state].resp)) {
                if (!memcmp(ph_buf_mem(buf), state_cmd[cur_state].resp, strlen(state_cmd[cur_state].resp))) {
                    cur_state++;
                    ph_stm_printf(serial->stream, "%s\r", state_cmd[cur_state].cmd);
                    ph_stm_flush(serial->stream);

                    ph_buf_delref(buf);
                }
            } else {
                /**
                 * If we don't receive the length response that we expect, try to interrupt whatever
                 * is happening and reset our initialization to the beginning.
                 */
                cur_state = 0;
                ph_stm_printf(serial->stream, "\e\r");
                ph_stm_flush(serial->stream);
                do {
                    ph_buf_delref(buf);
                    buf = ph_serial_read_record(serial, "\r", 1);
                } while (buf);
            }
            return;
        }

        /**
         * We expect 2 bytes per reading plus 1 byte for the <CR>
         */
        if (ph_buf_len(buf) != 7) {
            ph_buf_delref(buf);
            return;
        }

        mag_reading = (mag_data_t*)ph_buf_mem(buf);

        mag_set_framedata(mag_reading->mag_x, mag_reading->mag_y, mag_reading->mag_z);
        ph_buf_delref(buf);
    }
}
/**
 * This initialization function can be called at anytime to close, re-open and initialize the magnetometer.
 */
void initialize_magnetometer(void)
{
    if (mag_comm) ph_serial_free(mag_comm);
    mag_set_framedata(0, 0, 0);

    mag_comm = ph_serial_open(MAGCOM, NULL, state_cmd);
    if (!mag_comm) {
    	blast_err("Could not open Magnetometer port %s", MAGCOM);
    	return;
    }
    mag_comm->callback = mag_process_data;
    mag_comm->timeout_duration.tv_sec = 1;

    ph_serial_setspeed(mag_comm, B9600);
    ph_serial_enable(mag_comm, true);
    ph_stm_printf(mag_comm->stream, "%s\r", state_cmd[cur_state].cmd);
    ph_stm_flush(mag_comm->stream);


    blast_startup("Initialized Magnetometer");
}
