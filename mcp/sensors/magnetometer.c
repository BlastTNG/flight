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
#include <errno.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/serial.h"
#include "phenom/memory.h"

#include "blast.h"
#include "channels_tng.h"
#include "magnetometer.h"
#include "mcp.h"
#include "pointing_struct.h"

#define MAGCOM "/dev/ttyMAG"
#define MAG_ERR_THRESHOLD 400

extern int16_t SouthIAm; // defined in mcp.c

ph_serial_t *mag_comm = NULL;

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

typedef enum {
    MAG_BOOT = 0,
    MAG_INIT,
    MAG_READING,
    MAG_ERROR,
    MAG_RESET
} e_mag_status_t;

typedef struct {
	e_mag_state cmd_state;
	e_mag_status_t status;
	uint16_t err_count;
	uint16_t error_warned;
} mag_state_t;

mag_state_t mag_state = {0, 0, 0};

static mag_state_cmd_t state_cmd[MAG_END] = {
		[MAG_WE_BIN] = { "*99WE", "OK" },
		[MAG_BIN] = { "*99B", "BINARY ON" },
		[MAG_WE_RATE] = { "*99WE", "OK" },
		[MAG_RATE] = { "*99R=100", "OK" },
		[MAG_CONT] = { "*99C" },
};

static void mag_set_framedata(int16_t m_magx, int16_t m_magy, int16_t m_magz)
{
    static channel_t *mag_x_channel = NULL;
    static channel_t *mag_y_channel = NULL;
    static channel_t *mag_z_channel = NULL;

// Since each flight computer has its own magnetometer we only want to write to the channels
// corresponding to that computer's magnetometer.
    static uint8_t mag_index = 0;
    static int firsttime = 1;

    if (firsttime) {
        mag_index = SouthIAm;
    }
    if (!mag_x_channel) {
        if (mag_index == 0) { // We are North (fc1)
            mag_x_channel = channels_find_by_name("x_mag_n");
            mag_y_channel = channels_find_by_name("y_mag_n");
            mag_z_channel = channels_find_by_name("z_mag_n");
        } else { // We are South (fc2)
            mag_x_channel = channels_find_by_name("x_mag_s");
            mag_y_channel = channels_find_by_name("y_mag_s");
            mag_z_channel = channels_find_by_name("z_mag_s");
        }
    }

    // TODO(seth): Mag data should should be filtered (à la gyroscopes) and read by ACS.c
//    ACSData.mag_x = ((double)(int16_t)be16toh(m_magx))/15000.0;
    SET_SCALED_VALUE(mag_x_channel, (double)(int16_t)be16toh(m_magx)/15000.0);
//    ACSData.mag_y = ((double)(int16_t)be16toh(m_magy))/15000.0;
    SET_SCALED_VALUE(mag_y_channel, (double)(int16_t)be16toh(m_magy)/15000.0);
//    ACSData.mag_z = ((double)(int16_t)be16toh(m_magz))/15000.0;
    SET_SCALED_VALUE(mag_z_channel, (double)(int16_t)be16toh(m_magz)/15000.0);
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

#ifdef DEBUG_MAGNETOMETER
    blast_info("Magnetometer callback for reason %u, mag_state.cmd_state = %u, status = %u",
                 (uint8_t) why, (uint8_t) mag_state.cmd_state, (uint8_t) mag_state.status);
#endif

    /**
     * If we timeout, then the assumption is that we need to re-initialize the
     * magnetometer stream
     */
    if ((why & PH_IOMASK_TIME) || (mag_state.status == MAG_RESET)) {
        mag_state.cmd_state = 0;
        blast_info("Sending CMD '%s' to the MAG", state_cmd[mag_state.cmd_state].cmd);
        ph_stm_printf(serial->stream, "%s\r", state_cmd[mag_state.cmd_state].cmd);
        ph_stm_flush(serial->stream);
        return;
    }

    if (why & PH_IOMASK_READ) {
#ifdef DEBUG_MAGNETOMETER
    	blast_info("Reading mag data!");
#endif
        buf = ph_serial_read_record(serial, "\r", 1);
        if (!buf) return;
		mag_state.status = MAG_READING;
        /**
         * Handle the initial handshaking and setup with the magnetometer
         */
        if (mag_state.cmd_state < MAG_CONT) {
            if ((ph_buf_len(buf) - 1) == strlen(state_cmd[mag_state.cmd_state].resp)) {
                if (!memcmp(ph_buf_mem(buf), state_cmd[mag_state.cmd_state].resp,
                                      strlen(state_cmd[mag_state.cmd_state].resp))) {
                    mag_state.cmd_state++;
                    blast_info("writing %s", state_cmd[mag_state.cmd_state].cmd);
                    ph_stm_printf(serial->stream, "%s\r", state_cmd[mag_state.cmd_state].cmd);
                    ph_stm_flush(serial->stream);

                    ph_buf_delref(buf);
                }
            } else {
                /**
                 * If we don't receive the length response that we expect, try to interrupt whatever
                 * is happening and reset our initialization to the beginning.
                 */
                blast_info("We didn't receive the appropriate length.  Resetting...");
                mag_state.cmd_state = 0;
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
        mag_state.error_warned = 0;
    }

    if (why & PH_IOMASK_ERR) {
    	if (mag_state.status != MAG_RESET) {
    	    mag_state.status = MAG_ERROR;
    	    mag_state.err_count++;
    	    // Try to restart the sequence.
    	    ph_stm_printf(serial->stream, "\e\r");
            ph_stm_flush(serial->stream);
        }
    	if (!(mag_state.error_warned)) {
    		blast_err("Error reading from the magnetometer! %s", strerror(errno));
    		mag_state.error_warned = 1;
    	}
    	if (mag_state.err_count > MAG_ERR_THRESHOLD) {
    		blast_err("Too many errors reading the magnetometer...attempting to reset.");
    		mag_state.status = MAG_RESET;
    		mag_state.err_count = 0;
    		mag_state.error_warned = 0;
    		mag_state.cmd_state = 0;
    	}
    }
}

/**
 * This initialization function can be called at anytime to close, re-open and initialize the magnetometer.
 */
void initialize_magnetometer()
{
    if (mag_comm) ph_serial_free(mag_comm);
    mag_set_framedata(0, 0, 0);

    mag_comm = ph_serial_open(MAGCOM, NULL, state_cmd);
    if (!mag_comm) {
    	blast_err("Could not open Magnetometer port %s", MAGCOM);
    	return;
    } else {
    	blast_info("Successfully opened Magnetometer port %s", MAGCOM);
    }

    mag_comm->callback = mag_process_data;
    mag_comm->timeout_duration.tv_sec = 1;

    ph_serial_setspeed(mag_comm, B9600);
    ph_stm_printf(mag_comm->stream, "\e");
    ph_stm_flush(mag_comm->stream);
    ph_stm_printf(mag_comm->stream, "%s\r", state_cmd[mag_state.cmd_state].cmd);
    ph_stm_flush(mag_comm->stream);
    ph_serial_enable(mag_comm, true);

    mag_state.status = MAG_INIT;
    blast_startup("Initialized Magnetometer");
}

void *monitor_magnetometer(void *m_arg)
{
  while (!shutdown_mcp) {
    if (mag_state.status == MAG_RESET) {
      blast_info("Received a request to reset the magnetometer communications.");
      ph_stm_printf(mag_comm->stream, "\e");
      ph_stm_flush(mag_comm->stream);
      usleep(1000);
      initialize_magnetometer();
      blast_info("Magnetometer reset complete.");
    }
    usleep(1000);
  }
  return NULL;
}
