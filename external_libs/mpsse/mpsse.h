/**************************************************************************
 *   Copyright (C) 2012 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef MPSSE_H_
#define MPSSE_H_

#include <stdint.h>
#include <stdbool.h>
#include "binarybuffer.h"

/* general failures
 * error codes < 100
 */
#define ERROR_OK                        (0)
#define ERROR_NO_CONFIG_FILE            (-2)
#define ERROR_BUF_TOO_SMALL             (-3)
/* see "Error:" log entry for meaningful message to the user. The caller should
 * make no assumptions about what went wrong and try to handle the problem.
 */
#define ERROR_FAIL                      (-4)
#define ERROR_WAIT                      (-5)

/* Mode flags */
#define POS_EDGE_OUT 0x00
#define NEG_EDGE_OUT 0x01
#define POS_EDGE_IN 0x00
#define NEG_EDGE_IN 0x04
#define MSB_FIRST 0x00
#define LSB_FIRST 0x08

/* Shifting commands IN MPSSE Mode*/
#define MPSSE_WRITE_NEG 0x01   /* Write TDI/DO on negative TCK/SK edge*/
#define MPSSE_BITMODE   0x02   /* Write bits, not bytes */
#define MPSSE_READ_NEG  0x04   /* Sample TDO/DI on negative TCK/SK edge */
#define MPSSE_LSB       0x08   /* LSB first */
#define MPSSE_DO_WRITE  0x10   /* Write TDI/DO */
#define MPSSE_DO_READ   0x20   /* Read TDO/DI */
#define MPSSE_WRITE_TMS 0x40   /* Write TMS/CS */

/*DEBUG FLAG*/
#define _DEBUG_JTAG_IO_ 1

enum ftdi_chip_type {
	TYPE_FT2232C,
	TYPE_FT2232H,
	TYPE_FT4232H,
	TYPE_FT232H,
};

struct mpsse_ctx {
	struct libusb_context *usb_ctx;
	struct libusb_device_handle *usb_dev;
	unsigned int usb_write_timeout;
	unsigned int usb_read_timeout;
	uint8_t in_ep;
	uint8_t out_ep;
	uint16_t max_packet_size;
	uint16_t index;
	uint8_t interface;
	enum ftdi_chip_type type;
	uint8_t *write_buffer;
	unsigned write_size;  // size in bytes
	unsigned write_count; // size in bytes
	uint8_t *read_buffer;
	unsigned read_size;
	unsigned read_count;
	uint8_t *read_chunk;
	unsigned read_chunk_size;
	struct bit_copy_queue read_queue;
	int retval;
};

/* Device handling */
struct mpsse_ctx *mpsse_open(const uint16_t *vid, const uint16_t *pid, const char *description,
	const char *serial, int channel);
void mpsse_close(struct mpsse_ctx *ctx);
bool mpsse_is_high_speed(struct mpsse_ctx *ctx);

void mpsse_reset_purge_close(struct mpsse_ctx *ctx);

/* Command queuing. These correspond to the MPSSE commands with the same names, but no need to care
 * about bit/byte transfer or data length limitation. Read data is guaranteed to be available only
 * after the following mpsse_flush(). */
void mpsse_clock_data_out(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
			 unsigned length, uint8_t mode);
void mpsse_clock_data_in(struct mpsse_ctx *ctx, uint8_t *in, unsigned in_offset, unsigned length,
			uint8_t mode);
void mpsse_clock_data(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
		     unsigned in_offset, unsigned length, uint8_t mode);

void mpsse_set_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir);
void mpsse_set_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir);
void mpsse_read_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t *data);
void mpsse_read_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t *data);
void mpsse_loopback_config(struct mpsse_ctx *ctx, bool enable);
void mpsse_set_divisor(struct mpsse_ctx *ctx, uint16_t divisor);
int mpsse_divide_by_5_config(struct mpsse_ctx *ctx, bool enable);
int mpsse_rtck_config(struct mpsse_ctx *ctx, bool enable);

/* Helper to set frequency in Hertz. Returns actual realizable frequency or negative error.
 * Frequency 0 means RTCK. */
int mpsse_set_frequency(struct mpsse_ctx *ctx, int frequency);

/* Queue handling */
int mpsse_flush(struct mpsse_ctx *ctx);
void mpsse_purge(struct mpsse_ctx *ctx);


/* Biphase specific routines */
void mpsse_biphase_write_data(struct mpsse_ctx *ctx, const uint16_t *out, uint32_t length);
void mpsse_watchdog_ping_low(struct mpsse_ctx *ctx);
void mpsse_watchdog_ping_high(struct mpsse_ctx *ctx);
int mpsse_watchdog_get_incharge(struct mpsse_ctx *ctx);

#endif /* MPSSE_H_ */
