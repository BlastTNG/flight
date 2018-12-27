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
#include "command_struct.h"

#define MAGCOM "/dev/ttyMAG"
#define MAG_ERR_THRESHOLD 1000
#define MAG_TIMEOUT_THRESHOLD 10
#define MAG_RESET_THRESHOLD 50

extern int16_t SouthIAm; // defined in mcp.c

int verbose_level = 0;
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
    MAG_RESET,
    MAG_POWERCYCLE
} e_mag_status_t;

typedef struct {
	e_mag_state cmd_state;
	e_mag_status_t status;
	uint16_t err_count;
	uint16_t error_warned;
	uint16_t timeout_count;
	uint16_t reset_count;
} mag_state_t;

mag_state_t mag_state = {0};

static mag_state_cmd_t state_cmd[MAG_END] = {
		[MAG_WE_BIN] = { "*99WE", "OK" },
		[MAG_BIN] = { "*99A", "ASCII ON" },
		[MAG_WE_RATE] = { "*99WE", "OK" },
		[MAG_RATE] = { "*99R=20", "OK" },
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
        if (mag_index == 0) { // We are North (fc1)
            mag_x_channel = channels_find_by_name("x_mag1_n");
            mag_y_channel = channels_find_by_name("y_mag1_n");
            mag_z_channel = channels_find_by_name("z_mag1_n");
        } else { // We are South (fc2)
            mag_x_channel = channels_find_by_name("x_mag2_s");
            mag_y_channel = channels_find_by_name("y_mag2_s");
            mag_z_channel = channels_find_by_name("z_mag2_s");
        }
        firsttime = 0;
    }

    // TODO(seth): Mag data should should be filtered (à la gyroscopes) and read by ACS.c
//    ACSData.mag_x = ((double)(int16_t)be16toh(m_magx))/15000.0;
    SET_SCALED_VALUE(mag_x_channel, ((double)m_magx)/15000.0);
//    ACSData.mag_y = ((double)(int16_t)be16toh(m_magy))/15000.0;
    SET_SCALED_VALUE(mag_y_channel, ((double)m_magy)/15000.0);
//    ACSData.mag_z = ((double)(int16_t)be16toh(m_magz))/15000.0;
    SET_SCALED_VALUE(mag_z_channel, ((double)m_magz)/15000.0);
}

static void mag_get_data(char *mag_buf, size_t len_mag_buf)
{
    static int have_warned = 0;
    static int firsttime = 1;
    char x2[2], x3[2], x4[2], x5[2];
    char y2[2], y3[2], y4[2], y5[2];
    char z2[2], z3[2], z4[2], z5[2];
    char xsn, ysn, zsn;
    x2[1] = x3[1] = x4[1] = x5[1] =
    y2[1] = y3[1] = y4[1] = y5[1] =
    z2[1] = z3[1] = z4[1] = z5[1] = 0;

    if (len_mag_buf != 28) {
        if (!have_warned) {
            blast_warn("We were only passed %d bytes of data instead of 28.", (uint16_t)len_mag_buf);
            have_warned = 1;
        }
        return;
    }
    xsn = mag_buf[0]; // sign/space
    x2[0]=mag_buf[2]; // number/space
    x3[0]=mag_buf[4]; // number
    x4[0]=mag_buf[5]; // number
    x5[0]=mag_buf[6]; // number
    ysn = mag_buf[9]; // sign/space
    y2[0]=mag_buf[11]; // number/space
    y3[0]=mag_buf[13]; // number
    y4[0]=mag_buf[14]; // number
    y5[0]=mag_buf[15]; // number
    zsn = mag_buf[18]; // sign/space
    z2[0]=mag_buf[20]; // number/space
    z3[0]=mag_buf[22]; // number
    z4[0]=mag_buf[23]; // number
    z5[0]=mag_buf[24]; // number
    mag_buf[25]='\0';
    int x = 1000*(atoi(x2))+100*(atoi(x3))+10*(atoi(x4))+atoi(x5);
    int y = 1000*(atoi(y2))+100*(atoi(y3))+10*(atoi(y4))+atoi(y5);
    int z = 1000*(atoi(z2))+100*(atoi(z3))+10*(atoi(z4))+atoi(z5);
    if (xsn == '-') x *= -1;
    if (ysn == '-') y *= -1;
    if (zsn == '-') z *= -1;
    mag_set_framedata(x, y, z);
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

    static int has_warned = 0;

    ph_buf_t *buf;

#ifdef DEBUG_MAGNETOMETER
    if (verbose_level) blast_info("Magnetometer callback for reason %u, mag_state.cmd_state = %u, status = %u",
                 (uint8_t) why, (uint8_t) mag_state.cmd_state, (uint8_t) mag_state.status);
#endif

    // First check to see whether we have been asked to reset the magnetometer.
    if (CommandData.mag_reset) {
        mag_state.status = MAG_RESET;
        CommandData.mag_reset = 0;
        return;
    }
    /**
     * If we timeout, then the assumption is that we need to re-initialize the
     * magnetometer stream
     */
    if ((why & PH_IOMASK_TIME)) {
        mag_state.cmd_state = 0;
        mag_state.timeout_count++;
        if (verbose_level) blast_info("We timed out, count = %d , status = %d, Sending CMD '%s' to the MAG",
                               mag_state.timeout_count, mag_state.status, state_cmd[mag_state.cmd_state].cmd);
        // Try again!
        mag_state.status = MAG_ERROR;
        ph_stm_printf(serial->stream, "%s\r", state_cmd[mag_state.cmd_state].cmd);
        ph_stm_flush(serial->stream);
        if (mag_state.timeout_count > MAG_TIMEOUT_THRESHOLD) {
            mag_state.status = MAG_RESET;
        }
        return;
    }

    if (why & PH_IOMASK_READ) {
        if (verbose_level) blast_info("Reading mag data!");
        // Read until we get a carriage return (indicating the end of the response)
        buf = ph_serial_read_record(serial, "\r", 1);
        if (!buf) return; // we didn't get anything
        mag_state.status = MAG_READING;
        /**
         * Handle the initial handshaking and setup with the magnetometer
         */
        if (mag_state.cmd_state < MAG_CONT) {
            if ((ph_buf_len(buf) - 1) == strlen(state_cmd[mag_state.cmd_state].resp)) {
                if (!memcmp(ph_buf_mem(buf), state_cmd[mag_state.cmd_state].resp,
                                      strlen(state_cmd[mag_state.cmd_state].resp))) {
                    mag_state.cmd_state++;
                    // blast_info("writing %s", state_cmd[mag_state.cmd_state].cmd);
                    ph_stm_printf(serial->stream, "%s\r", state_cmd[mag_state.cmd_state].cmd);
                    ph_stm_flush(serial->stream);

                    ph_buf_delref(buf);
                }
            } else {
                /**
                 * If we don't receive the length response that we expect, try to interrupt whatever
                 * is happening and reset our initialization to the beginning.
                 */
                if (!has_warned) blast_info("We didn't receive the appropriate response.  Resetting...");
                has_warned = 1;
                mag_state.status = MAG_RESET;
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
         * We expect 28 bytes total per reading including the <CR>
         */
        if (ph_buf_len(buf) != 28) {
            ph_buf_delref(buf);
            return;
        }

        mag_get_data((char*)ph_buf_mem(buf), ph_buf_len(buf));
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
    		// blast_err("Too many errors reading the magnetometer...attempting to reset.");
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
    static int has_warned = 0;
    static int firsttime = 1;
    if (mag_comm) ph_serial_free(mag_comm);
    mag_set_framedata(0, 0, 0);

    mag_comm = ph_serial_open(MAGCOM, NULL, state_cmd);
    if (!mag_comm) {
    	if (!has_warned) blast_err("Could not open Magnetometer port %s", MAGCOM);
      has_warned = 1;
    	return;
    } else {
    	// blast_info("Successfully opened Magnetometer port %s", MAGCOM);
      has_warned = 0;
    }

    mag_comm->callback = mag_process_data;
    mag_comm->timeout_duration.tv_sec = 1;

    ph_serial_setspeed(mag_comm, B9600);
    ph_stm_printf(mag_comm->stream, "*99\e\r");
    ph_stm_flush(mag_comm->stream);
    ph_stm_printf(mag_comm->stream, "%s\r", state_cmd[mag_state.cmd_state].cmd);
    ph_stm_flush(mag_comm->stream);
    ph_serial_enable(mag_comm, true);

    mag_state.err_count = 0;
    mag_state.timeout_count = 0;
    mag_state.status = MAG_INIT;
    if (firsttime) {
        blast_startup("Initialized Magnetometer");
        firsttime = 0;
    }
}

void reset_mag()
{
    ph_stm_printf(mag_comm->stream, "\e");
    ph_stm_flush(mag_comm->stream);
    usleep(1000);
    initialize_magnetometer();
}

void *monitor_magnetometer(void *m_arg)
{
  static int has_warned = 0;
  while (!shutdown_mcp) {
    if (mag_state.status == MAG_RESET) {
      if (mag_state.reset_count >= MAG_RESET_THRESHOLD) {
          if (!has_warned) {
              blast_info("Still not able to connect to the magnetometers. reset_count = %d", mag_state.reset_count);
          }
          has_warned = 1;
          mag_state.reset_count = 0;
      }
      if (verbose_level) blast_info("Received a request to reset the magnetometer communications.");
      reset_mag();
      mag_state.reset_count++;
      if (verbose_level) blast_info("Magnetometer reset complete. reset counter = %d", mag_state.reset_count);
    }
    usleep(100000);
  }
  return NULL;
}

// Called in store_1hz_acs of acs.c
void store_1hz_mag(void)
{
    static int firsttime = 1;
    static channel_t *StatusMagAddr;
    static channel_t *ErrCountMagAddr;
    static channel_t *TimeoutCountMagAddr;
    static channel_t *ResetCountMagAddr;

    if (firsttime) {
        if (SouthIAm) {
            StatusMagAddr = channels_find_by_name("status_mag2_s");
            ErrCountMagAddr = channels_find_by_name("err_count_mag2_s");
            TimeoutCountMagAddr = channels_find_by_name("timeout_count_mag2_s");
            ResetCountMagAddr = channels_find_by_name("reset_count_mag2_s");
        } else {
            StatusMagAddr = channels_find_by_name("status_mag1_n");
            ErrCountMagAddr = channels_find_by_name("err_count_mag1_n");
            TimeoutCountMagAddr = channels_find_by_name("timeout_count_mag1_n");
            ResetCountMagAddr = channels_find_by_name("reset_count_mag1_n");
        }
        firsttime = 0;
    }
    SET_UINT8(StatusMagAddr, mag_state.status);
    SET_UINT16(ErrCountMagAddr, mag_state.err_count);
    SET_UINT16(TimeoutCountMagAddr, mag_state.timeout_count);
    SET_UINT16(ResetCountMagAddr, mag_state.reset_count);
}

